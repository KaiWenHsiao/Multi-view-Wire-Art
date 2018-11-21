#include "ImageObject.h"


ImageObject::ImageObject()
{
	m_imageHeightConstraint = 512;
	// Default is perspective
	m_projectionMode = 0; 
}


ImageObject::~ImageObject()
{
	delete m_graph;

	int numRay = m_rays.size();
	for (int i = 0; i < numRay; i++){
		delete m_rays[i];
	}
}

void ImageObject::clearComputationMaps(){
	m_thinContour.copyTo(m_remainMap);	
	m_pathFindingMap = cv::Mat(m_imageHeight, m_imageWidth, CV_8U, cv::Scalar(0));
	m_distanceTransformVerPathFinding = cv::Mat(m_imageHeight, m_imageWidth, CV_32F);	
}
bool ImageObject::renewImage(const cv::Mat &img, int radius){
	
	// Let 2d image as SINGLE-CONNECTED-COMPONENT
	cv::Mat gray; 
	cv::cvtColor(img, gray, CV_BGR2GRAY);
	cv::threshold(gray, gray, 200, 255, cv::THRESH_BINARY);

	img.copyTo(m_originImage);

	
	PixelConnectedComponent *pcc = new PixelConnectedComponent();
	pcc->labeling(gray);
	pcc->setUpGraph(pcc->m_groups);
	pcc->connect();

	ProgressTestSender::Instance()->addLog("Num 2D Component: " + std::to_string(pcc->m_groups.size()) + "\nNum Edge: " + std::to_string(pcc->m_result.size()));

	for (int i = 0; i < pcc->m_result.size(); i++){
		const PixelConnectedComponent::Node::Link &l = pcc->m_result[i];
		int pixel0[2], pixel1[2];
		pcc->getIndex(pixel0, l.selfPixelIndex);
		pcc->getIndex(pixel1, l.linkedPixelIndex);

		cv::line(m_originImage, cv::Point(pixel0[0], pixel0[1]), cv::Point(pixel1[0], pixel1[1]), cv::Scalar(0, 0, 0), 4);
	}

	delete pcc;
	
	// Thinning
	bool thinningFlag = this->setUpThinningContour();
	if (!thinningFlag){
		return false;
	}

	// Origin Image = Thin Contour + Dilate
	// 32F Origin Image
	cv::cvtColor(m_thinContour, m_originImage, CV_GRAY2BGR);
	cv::erode(m_originImage, m_originImage, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

	// 8U Origin Image
	m_thinContour.copyTo(m_originImage8U);
	cv::erode(m_originImage8U, m_originImage8U, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

	m_originDistanceTransformMap = cv::Mat(m_imageHeight, m_imageWidth, CV_32F, cv::Scalar(0.0));
	cv::distanceTransform(m_originImage8U, m_originDistanceTransformMap, CV_DIST_L2, 3);
	cv::normalize(m_originDistanceTransformMap, m_distanceTransformVerThinForDisplay, 0, 1, cv::NORM_MINMAX);

	m_imageWidth = m_originImage.cols;
	m_imageHeight = m_originImage.rows;

	m_graph = new ImageGraph(m_thinContour, radius);

	
	return true;
}
bool ImageObject::loadImage(const std::string& file, const std::string& name, bool requireGraph, int radius){
	//////////////////////////////////////////////////////
	// Get Optimized Image
	bool load = this->getOptimizedImage(file);
	// Image Loading is failed
	if (!load){
		return false;
	}

	if (requireGraph){
		// Create graph
		m_graph = new ImageGraph(m_thinContour, radius);
	}
	else{
		m_graph = nullptr;
	}
	

	m_imageWidth = m_originImage.cols;
	m_imageHeight = m_originImage.rows;

	// Create Pixel Weight Map
	m_pixelWeightMap = cv::Mat(m_imageHeight, m_imageWidth, CV_32F);
	int total = m_imageWidth * m_imageHeight;
	// No spedity weight map -> whole weights are 1.0f
	float *data = (float*)(m_pixelWeightMap.data);
	for (int i = 0; i < total; i++){
		data[i] = 1.0f;
	}

	return true;
}
bool ImageObject::loadImage(const std::string& file, const std::string &pixelWeight, const std::string& name, bool requireGraph, int radius){
	//////////////////////////////////////////////////////
	// Get Optimized Image
	bool load = this->getOptimizedImage(file, pixelWeight);
	// Image Loading is failed
	if (!load){
		return false;
	}

	if (requireGraph){
		// Create graph
		m_graph = new ImageGraph(m_thinContour, radius);
	}
	else{
		m_graph = nullptr;
	}

	m_imageWidth = m_originImage.cols;
	m_imageHeight = m_originImage.rows;

	// Create Pixel Weight Map
	m_pixelWeightMap = cv::Mat(m_imageHeight, m_imageWidth, CV_32F);
	//cv::Mat weightDisplay = cv::Mat(m_imageHeight, m_imageWidth, CV_8U);
	for (int i = 0; i < m_imageHeight; i++){
		for (int j = 0; j < m_imageWidth; j++){
			const cv::Vec3b &color = m_originPixelWeightMap.at<cv::Vec3b>(i, j);
			float &w = m_pixelWeightMap.at<float>(i, j);
			//uchar &c = weightDisplay.at<uchar>(i, j);
			// Red pixel should have higher weight
			if (color[0] < 200 && color[1] < 100 && color[2] > 100){
				w = 5.0f;				
			}
			else{
				w = 1.0f;				
			}
		}
	}	

	return true;
}
bool ImageObject::getOptimizedImage(const std::string& file){
	cv::Mat src = cv::imread(file);
	if (!src.data){
		return false;
	}

	cv::Mat gray;
	cv::cvtColor(src, gray, CV_BGR2GRAY);
	cv::threshold(gray, gray, 200, 255, cv::THRESH_BINARY);

	// Get non-background pixel as Contours
	std::vector<cv::Point> contourPixels;
	for (int i = 0; i < src.rows; i++){
		for (int j = 0; j < src.cols; j++){
			uchar pixel = gray.at<uchar>(i, j);
			if (pixel < ImageObject::CONTOUR_PIXEL_THRESHOLD){
				contourPixels.push_back(cv::Point(j, i));
			}
		}
	}

	// Approximate Contours to polygons 
	std::vector<std::vector<cv::Point>> contoursPoly(1);
	cv::Rect boundRect;
	cv::approxPolyDP(contourPixels, contoursPoly[0], 3, true);
	boundRect = cv::boundingRect(contoursPoly[0]);

	// Scale Bounding Rect
	boundRect.x -= 20;
	boundRect.y -= 20;
	boundRect.width += 40;
	boundRect.height += 40;

	// Becarefun out of boundary ...
	if (boundRect.x < 0 || boundRect.y < 0 || boundRect.x + boundRect.width >= src.cols || boundRect.y + boundRect.height >= src.rows){
		m_log.append("The Remain White Background is not enough !");
		return false;
	}

	m_originImage = src(boundRect);

	// Apply Uniform Scale
	// The height is locked to 256
	const int TARGET_HEIGHT = m_imageHeightConstraint;
	float scaleFactor = TARGET_HEIGHT * 1.0 / boundRect.height;
	int targetWidth = (int)ceil(boundRect.width * scaleFactor);
	// The too-large aspect(W/H) is not allowed
	if (targetWidth >= TARGET_HEIGHT * 1.2){
		m_log.append("The aspect is too large to find satisfy voxel structure\n");
		targetWidth = TARGET_HEIGHT * 1.2;
	}
	
	// resize with uniform scaling
	cv::resize(m_originImage, m_originImage, cv::Size(targetWidth, TARGET_HEIGHT));

	// Thinning
	bool thinningFlag = this->setUpThinningContour();
	if (!thinningFlag){		
		return false;
	}

	// Origin Image = Thin Contour + Dilate
	// 32F Origin Image
	cv::cvtColor(m_thinContour, m_originImage, CV_GRAY2BGR);
	cv::erode(m_originImage, m_originImage, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
	
	// 8U Origin Image
	m_thinContour.copyTo(m_originImage8U);
	cv::erode(m_originImage8U, m_originImage8U, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

	return true;
}
bool ImageObject::getOptimizedImage(const std::string& file, const std::string &pixelWeight){
	cv::Mat src = cv::imread(file);
	if (!src.data){
		return false;
	}
	cv::Mat weightSrc = cv::imread(pixelWeight);
	if (!weightSrc.data){
		return false;
	}

	cv::Mat gray;
	cv::cvtColor(src, gray, CV_BGR2GRAY);
	cv::threshold(gray, gray, 200, 255, cv::THRESH_BINARY);

	// Get non-background pixel as Contours
	std::vector<cv::Point> contourPixels;
	for (int i = 0; i < src.rows; i++){
		for (int j = 0; j < src.cols; j++){
			uchar pixel = gray.at<uchar>(i, j);
			if (pixel < ImageObject::CONTOUR_PIXEL_THRESHOLD){
				contourPixels.push_back(cv::Point(j, i));
			}
		}
	}

	// Approximate Contours to polygons 
	std::vector<std::vector<cv::Point>> contoursPoly(1);
	cv::Rect boundRect;
	cv::approxPolyDP(contourPixels, contoursPoly[0], 3, true);
	boundRect = cv::boundingRect(contoursPoly[0]);

	// Scale Bounding Rect
	boundRect.x -= 20;
	boundRect.y -= 20;
	boundRect.width += 40;
	boundRect.height += 40;

	// Becarefun out of boundary ...
	if (boundRect.x < 0 || boundRect.y < 0 || boundRect.x + boundRect.width >= src.cols || boundRect.y + boundRect.height >= src.rows){
		m_log.append("The Remain White Background is not enough !");
		return false;
	}

	m_originImage = src(boundRect);
	m_originPixelWeightMap = weightSrc(boundRect);

	// Apply Uniform Scale
	// The height is locked to 256
	const int TARGET_HEIGHT = m_imageHeightConstraint;
	float scaleFactor = TARGET_HEIGHT * 1.0 / boundRect.height;
	int targetWidth = (int)ceil(boundRect.width * scaleFactor);
	// The too-large aspect(W/H) is not allowed
	if (targetWidth >= TARGET_HEIGHT * 1.2){
		m_log.append("The aspect is too large to find satisfy voxel structure\n");
		targetWidth = TARGET_HEIGHT * 1.2;
	}

	// resize with uniform scaling
	cv::resize(m_originImage, m_originImage, cv::Size(targetWidth, TARGET_HEIGHT));
	cv::resize(m_originPixelWeightMap, m_originPixelWeightMap, cv::Size(targetWidth, TARGET_HEIGHT));

	// Thinning
	bool thinningFlag = this->setUpThinningContour();
	if (!thinningFlag){
		return false;
	}

	// Origin Image = Thin Contour + Dilate
	// 32F Origin Image
	cv::cvtColor(m_thinContour, m_originImage, CV_GRAY2BGR);
	cv::erode(m_originImage, m_originImage, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
	
	// 8U Origin Image
	m_thinContour.copyTo(m_originImage8U);
	cv::erode(m_originImage8U, m_originImage8U, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

	return true;
}
bool ImageObject::setUpThinningContour(){
	cv::Mat grayImage;

	cv::cvtColor(m_originImage, grayImage, CV_BGR2GRAY);
	cv::threshold(grayImage, grayImage, 200, 255, cv::THRESH_BINARY_INV);

	if (!HKCV::Thinning(grayImage)){
		m_log.append("Error Occured When Thinning !");
		return false;
	}
	cv::threshold(grayImage, grayImage, 200, 255, cv::THRESH_BINARY_INV);

	grayImage.copyTo(m_thinContour);

	// Create Distance Transform Map
	m_distanceTransformVerThin = cv::Mat(m_originImage.rows, m_originImage.cols, CV_32F);
	cv::distanceTransform(grayImage, m_distanceTransformVerThin, CV_DIST_L2, 3);
	// Create Distance Transfomr Map for display
	cv::normalize(m_distanceTransformVerThin, m_distanceTransformVerThinForDisplay, 0, 1, cv::NORM_MINMAX);
	
	return true;
}
void ImageObject::setNearPlaneWH(float planeWidth, float planeHeight){
	// Set near plane attribute
	m_nearPlaneWidth = planeWidth;
	m_nearPlaneHeight = planeHeight;

	// Compute mapping factor
	m_xmf = m_nearPlaneWidth / (m_imageWidth - 1);
	m_ymf = m_nearPlaneHeight / (m_imageHeight - 1);
}
void ImageObject::setUpViewFrustrum(glm::vec3 viewPos, glm::vec3 viewLookat, const glm::vec3 &upVector, float fovy, float near, float far){
	m_viewPos = viewPos;

	// Compute z axis
	m_z = viewLookat - viewPos;
	m_z = glm::normalize(m_z);

	// y axis is constant
	m_y = upVector;

	// Compute x axis
	m_x = glm::cross(m_y, m_z);
	m_x = glm::normalize(m_x);

	// Set up transform matrix
	glm::mat4x4 projMat = glm::frustum<float>(-m_nearPlaneWidth*0.5, m_nearPlaneWidth*0.5, -m_nearPlaneHeight*0.5, m_nearPlaneHeight*0.5, near, far);
	m_viewMat = glm::lookAt(viewPos, viewLookat, m_y);
	m_viewProjMat = projMat * m_viewMat;

	// Compute near plane center 
	m_nearPlaneCenter = viewPos + near * m_z;

	// Record
	m_near = near;
	m_far = far;
}
void ImageObject::createRay(){
	int rayIter = 0;
	m_rays.clear();

	for (int i = 0; i < m_imageHeight; i++){
		for (int j = 0; j < m_imageWidth; j++){

			if (this->backgroundPixel(j, i)){
				continue;
			}

			m_rays.push_back(new Ray());			

			// Create ray
			Ray *r = m_rays[rayIter];
			float xf = (m_imageWidth - 1 - j) * m_xmf - m_nearPlaneWidth * 0.5;
			float yf = (m_imageHeight - 1 - i) * m_ymf - m_nearPlaneHeight * 0.5;

			r->m_orig = m_nearPlaneCenter + xf * m_x + yf * m_y;
			r->m_dir = r->m_orig - m_viewPos;
			r->m_dir = glm::normalize(r->m_dir);

			rayIter++;
		}
	}	
}
bool ImageObject::backgroundPixel(int w, int h){
	uchar &pixel = m_thinContour.at<uchar>(h, w);
	//uchar &pixel = m_originImage8U.at<uchar>(h, w);

	return pixel > CONTOUR_PIXEL_THRESHOLD;
}

void ImageObject::proj(float *vertex, int *pixel){
	// Multi-view wire art
	
	// Create vector
	glm::vec4 v = glm::vec4(vertex[0], vertex[1], vertex[2], 1);

	// Compute mapping pixel 
	glm::vec4 p = m_viewProjMat * v;

	// [-1 1] -> [0 1]
	const static glm::mat4x4 bias = glm::mat4x4(
		0.5, 0, 0, 0,
		0, 0.5, 0, 0,
		0, 0, 0.5, 0,
		0.5, 0.5, 0.5, 1.0);
	p = bias * (p / p.w);

	// To image space
	pixel[0] = (int)(p[0] * m_imageWidth);
	pixel[1] = (int)(p[1] * m_imageHeight);

	pixel[1] = m_imageHeight - pixel[1];
	
	
}
bool ImageObject::getToleranceProj(float *v, int *p, int tolerance){
	// Projection		
	int pixel[2];
	this->proj(v, pixel);

	// Adjust when the erro is not so serious
	if (pixel[0] < 0 && abs(pixel[0]) < tolerance){
		pixel[0] = 0;
	}
	else if (pixel[0] >= m_imageWidth && (pixel[0] - m_imageWidth + 1) < tolerance){
		pixel[0] = m_imageWidth - 1;
	}

	if (pixel[1] < 0 && abs(pixel[1]) < tolerance){
		pixel[1] = 0;
	}
	else if (pixel[1] >= m_imageHeight && (pixel[1] - m_imageHeight + 1) < tolerance){
		pixel[1] = m_imageHeight - 1;
	}


	// Be Culled, Do nothing ;
	if (pixel[0] < 0 || pixel[0] > m_imageWidth - 1 || pixel[1] < 0 || pixel[1] > m_imageHeight - 1){
		return false;
	}

	p[0] = pixel[0];
	p[1] = pixel[1];

	return true;
}
void ImageObject::voxelCover(float *v, int radius, int color){
	int pixel[2];
	if (this->getToleranceProj(v, pixel, 2)){
		cv::circle(m_remainMap, cv::Point(pixel[0], pixel[1]), radius, cv::Scalar(255), -1);		
	}
}
float ImageObject::getDistanceTransformVerThin(float *v){
	int pixel[2];
	if (!this->getToleranceProj(v, pixel, 2)){
		return -1;
	}

	return m_distanceTransformVerThin.at<float>(pixel[1], pixel[0]);
}
float ImageObject::getOriginDistanceTransform(float *v){
	int pixel[2];
	if (!this->getToleranceProj(v, pixel, 2)){
		return -1;
	}

	return m_originDistanceTransformMap.at<float>(pixel[1], pixel[0]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ImageObject::getNearestPixelRay(Ray *r, float *vertex){
	// Projection		
	int pixel[2];
	this->getToleranceProj(vertex, pixel, 3);

	ImageGraph::PixelNode *pn = m_graph->getNearNeighbor(pixel[0], pixel[1]);

	// Get Ray
	float xf = (m_imageWidth - 1 - pn->pixel.x) * m_xmf - m_nearPlaneWidth * 0.5;
	float yf = (m_imageHeight - 1 - pn->pixel.y) * m_ymf - m_nearPlaneHeight * 0.5;

	r->m_orig = m_nearPlaneCenter + xf * m_x + yf * m_y;
	r->m_dir = r->m_orig - m_viewPos;
	r->m_dir = glm::normalize(r->m_dir);
	
}
void ImageObject::getNearestThinContourPixel(float *v, int *res){
	// Projection	
	int pixel[2];
	if (!this->getToleranceProj(v, pixel, 3)){
		res[0] = -1;
		res[1] = -1;
		return;
	}

	ImageGraph::PixelNode *pn = m_graph->getNearNeighbor(pixel[0], pixel[1]);
	res[0] = pn->pixel.x;
	res[1] = pn->pixel.y;
}
glm::vec3 ImageObject::getWorldSpaceVertex(int *pixel){
	float xf = (m_imageWidth - 1 - pixel[0]) * m_xmf - m_nearPlaneWidth * 0.5;
	float yf = (m_imageHeight - 1 - pixel[1]) * m_ymf - m_nearPlaneHeight * 0.5;

	return m_nearPlaneCenter + xf * m_x + yf * m_y;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ortho Projection
void ImageObject::setUpOrthoViewFrustrum(glm::vec3 viewPos, glm::vec3 viewLookat, const glm::vec3 &upVector, float left, float right, float bottom, float up, float near, float far){
	m_viewPos = viewPos;

	// Compute z axis
	m_z = viewLookat - viewPos;
	m_z = glm::normalize(m_z);

	// y axis is constant
	m_y = upVector;

	// Compute x axis
	m_x = glm::cross(m_y, m_z);
	m_x = glm::normalize(m_x);

	// Set up transform matrix
	glm::mat4 projMat = glm::ortho(left, right, bottom, up, near, far);
	m_viewMat = glm::lookAt(viewPos, viewLookat, m_y);
	m_viewProjMat = projMat * m_viewMat;

	// Compute near plane center 
	m_nearPlaneCenter = viewPos + near * m_z;

	// Record
	m_near = near;
	m_far = far;

	// Compute mapping factor
	m_nearPlaneWidth = right - left;
	m_nearPlaneHeight = up - bottom;
	m_xmf = m_nearPlaneWidth / (m_imageWidth - 1);
	m_ymf = m_nearPlaneHeight / (m_imageHeight - 1);
}
void ImageObject::createOrthoRay(){
	int rayIter = 0;
	m_rays.clear();

	for (int i = 0; i < m_imageHeight; i++){
		for (int j = 0; j < m_imageWidth; j++){

			if (this->backgroundPixel(j, i)){
				continue;
			}

			m_rays.push_back(new Ray());

			// Create ray
			Ray *r = m_rays[rayIter];
			float xf = (m_imageWidth - 1 - j) * m_xmf - m_nearPlaneWidth * 0.5;
			float yf = (m_imageHeight - 1 - i) * m_ymf - m_nearPlaneHeight * 0.5;			

			r->m_orig = m_nearPlaneCenter + xf * m_x + yf * m_y;
			r->m_dir = m_z;
			r->m_dir = glm::normalize(r->m_dir);

			rayIter++;
		}
	}

	// Set mode as ortho
	m_projectionMode = 1;
}
////////////////////////////////////////////////////////////////////////////////////////
void ImageObject::getRay(Ray *r, int x, int y){
	
	if (m_projectionMode == 0){
		// perspective
		float xf = (m_imageWidth - 1 - x) * m_xmf - m_nearPlaneWidth * 0.5;
		float yf = (m_imageHeight - 1 - y) * m_ymf - m_nearPlaneHeight * 0.5;

		r->m_orig = m_nearPlaneCenter + xf * m_x + yf * m_y;
		r->m_dir = r->m_orig - m_viewPos;
		r->m_dir = glm::normalize(r->m_dir);
	}
	else if (m_projectionMode == 1){
		// ortho
		float xf = (m_imageWidth - 1 - x) * m_xmf - m_nearPlaneWidth * 0.5;
		float yf = (m_imageHeight - 1 - y) * m_ymf - m_nearPlaneHeight * 0.5;

		r->m_orig = m_nearPlaneCenter + xf * m_x + yf * m_y;
		r->m_dir = m_z;
		r->m_dir = glm::normalize(r->m_dir);
	}
}
///////////////////////////////////////////////////////////////////////////////////////
float ImageObject::getLineDistanceTransform(float *v0, float *v1, const int w){
	int p0[2], p1[2];

	if (!this->getToleranceProj(v0, p0, 3))
		return -1;
	if (!this->getToleranceProj(v1, p1, 3))
		return -1;

	// Prepare mask
	if (m_lineDTMask.data == nullptr){
		m_lineDTMask = cv::Mat(m_imageHeight, m_imageWidth, CV_8U, cv::Scalar(0));
	}
	else{
		m_lineDTMask = cv::Scalar(0);
	}

	// Draw Line	
	cv::line(
		m_lineDTMask,
		cv::Point(p0[0], p0[1]),
		cv::Point(p1[0], p1[1]),
		cv::Scalar(255),
		w,
		8,
		0
		);
	

	// Check the pixel and increasing
	float sumDT = 0.0;
	int totalPixel = 0;
	for (int h = 0; h < m_imageHeight; h++){
		unsigned char *ptr = m_lineDTMask.ptr<uchar>(h);
		for (int w = 0; w < m_imageWidth; w++){
			// In the range
			if (ptr[w] > 200){
				sumDT += m_distanceTransformVerThin.at<float>(h, w);
				totalPixel++;
			}
		}
	}
	if (totalPixel <= 0)
		return -1;

	return sumDT / totalPixel;
}
float ImageObject::getDistanceTransformNorm(float *v){
	int pixel[2];
	if (!this->getToleranceProj(v, pixel, 3)){
		return -1;
	}

	return m_distanceTransformVerThinForDisplay.at<float>(pixel[1], pixel[0]);
}
float ImageObject::getVoxelPathDistanceTransform(const std::vector<glm::vec3> &voxels, int radius){
	// Prepare mask
	if (m_lineDTMask.data == nullptr){
		m_lineDTMask = cv::Mat(m_imageHeight, m_imageWidth, CV_8U, cv::Scalar(0));
	}
	else{
		m_lineDTMask = cv::Scalar(0);
	}

	int numVoxel = voxels.size();
	for (int i = 0; i < numVoxel; i++){
		int pixel[2];
		float v[] = { voxels[i].x, voxels[i].y, voxels[i].z };
		if (this->getToleranceProj(v, pixel, 3)){
			cv::circle(m_lineDTMask, cv::Point(pixel[0], pixel[1]), radius, cv::Scalar(255));
		}
	}

	// Check the pixel and increasing
	float sumDT = 0.0;
	int totalPixel = 0;
	for (int h = 0; h < m_imageHeight; h++){
		unsigned char *ptr = m_lineDTMask.ptr<uchar>(h);
		for (int w = 0; w < m_imageWidth; w++){
			// In the range
			if (ptr[w] > 200){
				sumDT += m_distanceTransformVerPathFinding.at<float>(h, w);
				totalPixel++;
			}
		}
	}
	if (totalPixel <= 0)
		return -1;

	return sumDT / totalPixel;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ImageObject::getClipPlaneGeometry(float *vertices, float z){
	// Calculate offset x & offset y
	float offsetX = z * m_nearPlaneWidth * 0.5 / m_near;
	float offsetY = z * m_nearPlaneHeight * 0.5 / m_near;

	glm::mat4 viewTranspose = glm::transpose(m_viewMat);
	glm::vec4 planeVertices[4];
	planeVertices[0] = glm::vec4(m_viewPos, 1) + viewTranspose * glm::vec4(-offsetX, offsetY, -z, 1);
	planeVertices[1] = glm::vec4(m_viewPos, 1) + viewTranspose * glm::vec4(-offsetX, -offsetY, -z, 1);
	planeVertices[2] = glm::vec4(m_viewPos, 1) + viewTranspose * glm::vec4(offsetX, -offsetY, -z, 1);
	planeVertices[3] = glm::vec4(m_viewPos, 1) + viewTranspose * glm::vec4(offsetX, offsetY, -z, 1);

	int twoTriangleIndexes[] = { 0, 1, 2, 2, 3, 0 };
	// Extend to two triangle
	for (int i = 0; i < 6; i++){
		vertices[i * 3 + 0] = planeVertices[twoTriangleIndexes[i]][0];
		vertices[i * 3 + 1] = planeVertices[twoTriangleIndexes[i]][1];
		vertices[i * 3 + 2] = planeVertices[twoTriangleIndexes[i]][2];
	}
}
void ImageObject::getImage(float *content, float alpha){
	if (content == nullptr)
		return;

	for (int i = 0; i < m_imageHeight; i++){
		for (int j = 0; j < m_imageWidth; j++){
			int offset = (i * m_imageWidth + j) * 4;
			if (this->backgroundPixel(j, i)){
				content[offset + 0] = 0;
				content[offset + 1] = 0;
				content[offset + 2] = 0;
				content[offset + 3] = 0;
			}
			else{
				content[offset + 0] = 0.1;
				content[offset + 1] = 0.1;
				content[offset + 2] = 0.1;
				content[offset + 3] = alpha;
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
void ImageObject::coverToPathFindingImage(float *v, int radius){
	int pixel[2];
	if (this->getToleranceProj(v, pixel, 3)){
		cv::circle(m_pathFindingMap, cv::Point(pixel[0], pixel[1]), radius, cv::Scalar(255), -1);	
	}
}
void ImageObject::setUpPathFindingDistanceTransformMap(){
	HKCV::Thinning(m_pathFindingMap);
	/*
	static int OUTPUT_INDEX = 0;
	cv::imwrite(std::to_string(OUTPUT_INDEX) + ".jpg", m_pathFindingMap);
	OUTPUT_INDEX++;
	*/
	cv::threshold(m_pathFindingMap, m_pathFindingMap, 200, 255, cv::THRESH_BINARY_INV);
	
	// Create Distance Transform Map
	cv::distanceTransform(m_pathFindingMap, m_distanceTransformVerPathFinding, CV_DIST_L2, 3);	
}
float ImageObject::getDistanceTransformVerPathFinding(float *v){
	int pixel[2];
	if (!this->getToleranceProj(v, pixel, 3)){
		return -1;
	}

	return m_distanceTransformVerPathFinding.at<float>(pixel[1], pixel[0]);
}
///////////////////////////////////////////////////////////////////////////////
void ImageObject::getPatchPixels(float *vertices, int numVertex, int radius, PixelSet *ps){
	cv::Mat projMask = cv::Mat(m_imageHeight, m_imageWidth, CV_8U, cv::Scalar(255));

	for (int i = 0; i < numVertex; i++){
		int pixel[2];
		if (this->getToleranceProj(vertices + i * 3, pixel, 3)){
			cv::circle(projMask, cv::Point(pixel[0], pixel[1]), radius, cv::Scalar(0), -1);
		}
	}

	int numPatchPixel = 0;
	for (int y = 0; y < m_imageHeight; y++){
		for (int x = 0; x < m_imageWidth; x++){
			const unsigned char p = projMask.at<uchar>(y, x);
			if (p < 100){
				numPatchPixel++;
			}
		}
	}
	ps->numPixel = numPatchPixel;
	ps->pixels = new int[numPatchPixel * 2];

	int offset = 0;
	for (int y = 0; y < m_imageHeight; y++){
		for (int x = 0; x < m_imageWidth; x++){
			const unsigned char p = projMask.at<uchar>(y, x);
			if (p < 100){
				int *pixel = ps->pixels + offset * 2;
				pixel[0] = x;
				pixel[1] = y;
				offset++;
			}
		}
	}
}
bool ImageObject::isProjectedIn2DConstraintRegion(float *v){
	int pixel[2];
	if (!this->getToleranceProj(v, pixel, 3))
		return false;

	uchar j = m_graph->m_2dConstraintRegionMap.at<uchar>(pixel[1], pixel[0]);

	if (j < 100){
		return true;
	}
	return false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
