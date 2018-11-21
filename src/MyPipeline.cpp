#include "MyPipeline.h"


MyPipeline::MyPipeline() 
{
	m_source = new Source();
}


MyPipeline::~MyPipeline()
{
}
bool MyPipeline::ready(const std::vector<std::string> &images){
	// Two view case
	if (images.size() == 2){
			
		float phis[] = { 0, 90 };
		// Set up ImageObjects	
		int numImage = images.size();
		for (int i = 0; i < numImage; i++){
			const std::string str = images[i];
			if (!this->createImageObject(str, 200, phis[i])){
				return false;
			}
		}
		return true;		
	}

	///////////////////////////////////////////////
	// Three view case
	if (images.size() >= 3){
		/*
		if (this->createThreeViewOrthoSet(images)){
			return true;
		}
		*/
		
		
		if (this->createThreeViewPerspectiveSet(images)){
			return true;
		}				
	}	

	return false;
}
bool MyPipeline::createImageObject(const std::string &file, float radius, float phi){
	ImageObject *io = new ImageObject();
	if (!io->loadImage(file, "image", m_requireImageGraph, 2)){
		
		delete io;
		return false;
	}

	const float TARGET_NEAR_HEIGHT = 0.5;
	float targetNearWidth = io->m_imageWidth * TARGET_NEAR_HEIGHT / io->m_imageHeight;
	io->setNearPlaneWH(targetNearWidth, TARGET_NEAR_HEIGHT);

	float phiRad = glm::radians(phi);
	float thetaRad = glm::radians(90.0);
	glm::vec3 center = glm::vec3(0, 0, 0);
	glm::vec3 upVector = glm::vec3(0, 1, 0);
	io->setUpViewFrustrum(
		glm::vec3(
		radius * glm::cos(phiRad) * glm::sin(thetaRad),
		radius * glm::cos(thetaRad),
		radius * glm::sin(phiRad) * glm::sin(thetaRad)) + center,
		center,
		upVector,
		glm::radians(45.0),
		2,
		500);
	io->createRay();

	m_source->m_ioList.push_back(io);
	
	return true;
}
bool MyPipeline::createImageObject(const std::string &file, const std::string &weightMap, float radius, float phi){
	ImageObject *io = new ImageObject();
	if (!io->loadImage(file, weightMap, "image", m_requireImageGraph, 2)){

		delete io;
		return false;
	}

	const float TARGET_NEAR_HEIGHT = 0.5;
	float targetNearWidth = io->m_imageWidth * TARGET_NEAR_HEIGHT / io->m_imageHeight;
	io->setNearPlaneWH(targetNearWidth, TARGET_NEAR_HEIGHT);

	float phiRad = glm::radians(phi);
	float thetaRad = glm::radians(90.0);
	glm::vec3 center = glm::vec3(0, 0, 0);
	glm::vec3 upVector = glm::vec3(0, 1, 0);
	io->setUpViewFrustrum(
		glm::vec3(
		radius * glm::cos(phiRad) * glm::sin(thetaRad),
		radius * glm::cos(thetaRad),
		radius * glm::sin(phiRad) * glm::sin(thetaRad)) + center,
		center,
		upVector,
		glm::radians(45.0),
		2,
		500);
	io->createRay();

	m_source->m_ioList.push_back(io);

	return true;
}
bool MyPipeline::createImageObject(const std::string &file, float radius, float phi, float theta, const glm::vec3 &center, const glm::vec3 &upVector){
	
	ImageObject *io = new ImageObject();
	if (!io->loadImage(file, "image", m_requireImageGraph, 2)){
		
		delete io;
		return false;
	}

	const float TARGET_NEAR_HEIGHT = 0.5;
	float targetNearWidth = io->m_imageWidth * TARGET_NEAR_HEIGHT / io->m_imageHeight;
	io->setNearPlaneWH(targetNearWidth, TARGET_NEAR_HEIGHT);

	float phiRad = glm::radians(phi);
	float thetaRad = glm::radians(theta);
	io->setUpViewFrustrum(
		glm::vec3(
		radius * glm::cos(phiRad) * glm::sin(thetaRad),
		radius * glm::cos(thetaRad),
		radius * glm::sin(phiRad) * glm::sin(thetaRad)) + center,
		center,
		upVector,
		glm::radians(45.0),
		2,
		500);
	io->createRay();

	m_source->m_ioList.push_back(io);

	return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MyPipeline::sentLog(const std::string &log){
	std::cout << log << "\n";
}
////////////////////////////////////////////////////////////////////////////
// Special case of three view
bool MyPipeline::createThreeViewOrthoSet(const std::vector<std::string> &images){
	// First, 
	ImageObject *io0 = new ImageObject();
	if (!io0->loadImage(images[0], "image", m_requireImageGraph, 2)){
		return false;
	}
	// Calculate Height & Width
	const float TARGET_NEAR_HEIGHT0 = 50;
	float targetNearWidth0 = io0->m_imageWidth * TARGET_NEAR_HEIGHT0 / io0->m_imageHeight;
	io0->setNearPlaneWH(targetNearWidth0, TARGET_NEAR_HEIGHT0);
	// Set up projection
	this->setUpOrthoProjection(io0, 200, TARGET_NEAR_HEIGHT0, targetNearWidth0, 0, 90, glm::vec3(0, 1, 0));
	io0->createOrthoRay(); 
	
	//////////////////////////////////////////////////////////////////////
	
	// Second
	ImageObject *io1 = new ImageObject();
	if (!io1->loadImage(images[1], "image", m_requireImageGraph, 2)){
		return false;
	}
	// Calculate Height & Width
	const float TARGET_NEAR_HEIGHT1 = 50;
	float targetNearWidth1 = io1->m_imageWidth * TARGET_NEAR_HEIGHT1 / io1->m_imageHeight;
	io1->setNearPlaneWH(targetNearWidth1, TARGET_NEAR_HEIGHT1);
	// Set up projection
	this->setUpOrthoProjection(io1, 200, TARGET_NEAR_HEIGHT1, targetNearWidth1, 90, 90, glm::vec3(0, 1, 0));
	io1->createOrthoRay();
	
	m_source->m_ioList.push_back(io0);
	m_source->m_ioList.push_back(io1);
	return true;
	//////////////////////////////////////////////////////////////////////
	// Third, Top view, should be constraint
	ImageObject *io2 = new ImageObject();
	if (!io2->loadImage(images[2], "image", m_requireImageGraph, 2)){
		return false;
	}
	// Calculate Height & Width
	float targetNearHeight2 = targetNearWidth0;
	float targetNearWidth2 = targetNearWidth1;
	io2->setNearPlaneWH(targetNearWidth2, targetNearHeight2);
	// Set up projection
	this->setUpOrthoProjection(io2, 200, targetNearHeight2, targetNearWidth2, 0, 0, glm::vec3(0, 0, -1));
	io2->createOrthoRay();

	m_source->m_ioList.push_back(io0);
	m_source->m_ioList.push_back(io1);
	m_source->m_ioList.push_back(io2);

	return true;
}
void MyPipeline::setUpOrthoProjection(ImageObject *io, float radius, float height, float width, float phi, float theta, const glm::vec3 &upVector){
	float phiRad = glm::radians(phi);
	float thetaRad = glm::radians(theta);
	io->setUpOrthoViewFrustrum(
		glm::vec3(
		radius * glm::cos(phiRad) * glm::sin(thetaRad),
		radius * glm::cos(thetaRad),
		radius * glm::sin(phiRad) * glm::sin(thetaRad)
		),
		glm::vec3(0, 0, 0),
		upVector,
		-1 * width * 0.5,
		width * 0.5,
		-1 * height * 0.5,
		height * 0.5,
		2,
		500);	
}
bool MyPipeline::createThreeViewPerspectiveSet(const std::vector<std::string> &images){
	bool weightMapFlag = false;
	if (images.size() > 3){
		weightMapFlag = true; 
	}

	// First, 
	ImageObject *io0 = new ImageObject();

	if (weightMapFlag){
		if (!io0->loadImage(images[0], images[3], "image", m_requireImageGraph)){
			return false;			
		}
	}
	else{
		if (!io0->loadImage(images[0], "image", m_requireImageGraph, 2)){
			return false;
		}
	}

	
	// Calculate Height & Width
	const float TARGET_NEAR_HEIGHT0 = 0.5;
	float targetNearWidth0 = io0->m_imageWidth * TARGET_NEAR_HEIGHT0 / io0->m_imageHeight;
	io0->setNearPlaneWH(targetNearWidth0, TARGET_NEAR_HEIGHT0);
	// Set up projection
	this->setUpPerspectiveProjection(io0, 200, 0, 90, glm::vec3(0, 1, 0));
	//io0->createRayVerDilated();
	io0->createRay();

	int TH = 1024;
	int TW0 = (int)(io0->m_imageWidth * TH * 1.0 / io0->m_imageHeight);

	//////////////////////////////////////////////////////////////////////

	// Second
	ImageObject *io1 = new ImageObject();
	if (weightMapFlag){
		if (!io1->loadImage(images[1], images[4], "image", m_requireImageGraph)){
			return false;
		}
	}
	else{
		if (!io1->loadImage(images[1], "image", m_requireImageGraph, 2)){
			return false;
		}
	}
	// Calculate Height & Width
	const float TARGET_NEAR_HEIGHT1 = 0.5;
	float targetNearWidth1 = io1->m_imageWidth * TARGET_NEAR_HEIGHT1 / io1->m_imageHeight;
	io1->setNearPlaneWH(targetNearWidth1, TARGET_NEAR_HEIGHT1);
	// Set up projection
	this->setUpPerspectiveProjection(io1, 200, 90, 90, glm::vec3(0, 1, 0));
	//io1->createRayVerDilated();
	io1->createRay();

	TH = 1024;
	int TW1 = (int)(io1->m_imageWidth * TH * 1.0 / io1->m_imageHeight);

	//////////////////////////////////////////////////////////////////////
	// Third, Top view, should be constraint
	ImageObject *io2 = new ImageObject();
	if (weightMapFlag){
		if (!io2->loadImage(images[2], images[5], "image", m_requireImageGraph)){
			return false;
		}
	}
	else{
		if (!io2->loadImage(images[2], "image", m_requireImageGraph, 2)){
			return false;
		}
	}
	// Calculate Height & Width
	float targetNearHeight2 = targetNearWidth0;
	float targetNearWidth2 = targetNearWidth1;
	io2->setNearPlaneWH(targetNearWidth2, targetNearHeight2);
	// Set up projection
	this->setUpPerspectiveProjection(io2, 200, 0, 0, glm::vec3(0, 0, -1));
	//io2->createRayVerDilated();
	io2->createRay();

	m_source->m_ioList.push_back(io0);
	m_source->m_ioList.push_back(io1);
	m_source->m_ioList.push_back(io2);

	for (int i = 0; i < 3; i++){
		this->sentLog(std::to_string(m_source->m_ioList[i]->m_rays.size()));
	}

	TH = TW0;
	int TW2 = TW1;

	return true;
}
void MyPipeline::setUpPerspectiveProjection(ImageObject *io, float radius,  float phi, float theta, const glm::vec3 &upVector){
	float phiRad = glm::radians(phi);
	float thetaRad = glm::radians(theta);
	io->setUpViewFrustrum(
		glm::vec3(
		radius * glm::cos(phiRad) * glm::sin(thetaRad),
		radius * glm::cos(thetaRad),
		radius * glm::sin(phiRad) * glm::sin(thetaRad)),
		glm::vec3(0, 0, 0),
		upVector,
		glm::radians(45.0),
		2,
		500);	
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MyPipeline::outputVoxelLabel(const std::string &fileName){
	std::ofstream output(fileName);

	if (!output.is_open()){
		return;
	}

	// Prepare the buffer
	unsigned char *labelBuffer = new unsigned char[m_source->m_total];
	for (int i = 0; i < m_source->m_total; i++){
		labelBuffer[i] = m_source->m_sculpture[i].selected;
	}

	// Write 
	output.write((char*)labelBuffer, m_source->m_total);

	output.close();

	delete[] labelBuffer;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MyPipeline::sequentiallyMultiResolutionSculpture(const std::vector<std::string> &images, const std::vector<std::string> &fileNames, int resolution){
	// Prepare ContourArt of multi resolution
	const int RES_SELECTION[] = { 512, 256, 128, 64 };
	const float VOXEL_SIZE[] = { 0.125f, 0.25f, 0.5f, 1.0f };

	// Ckeck requirement
	if (images.size() > 3){
		std::cout << "Doesn't support the number of input that more than 3\n";
		return false;
	}	
	if (resolution < 0 || resolution > 3){
		std::cout << "The resolution required is not supported";
		return false;
	}
	else{
		std::cout << "Process " << images.size() << " case with" <<  RES_SELECTION[resolution] << "resolution setting \n";
	}
		
	// Prepare voxel buffer
	int total = 512 * 512 * 512;
	m_source->m_sculpture = new Source::Voxel[total];		

	// Initialize
	int i = resolution;
	m_source->m_width = RES_SELECTION[i];
	m_source->m_height = RES_SELECTION[i];
	m_source->m_length = RES_SELECTION[i];
	m_source->m_total = m_source->m_width * m_source->m_height * m_source->m_length;
	m_source->m_voxelSize = VOXEL_SIZE[i];
	ContourArt *ca = new ContourArt(m_source);
	ca->buildOctree(7 - i);
	m_multiRes[i] = ca;
	
	/////////////////////////////////////////////////////////////////////
	if (!this->ready(images)){
		this->sentLog("=============\nCannot process " + fileNames[0]);
		return false;
	}
	else{
		this->sentLog("=============\nNow process " + fileNames[0]);
	}

	// Create sculpture with multiple volume resolution
	this->createVariousResolutoinSculpture(fileNames[0], resolution);		

	for (int i = 0; i < m_source->m_ioList.size(); i++){
		delete m_source->m_ioList[i];
	}
	m_source->m_ioList.clear();		
	

	this->sentLog("Done !");

	return true;
}
void MyPipeline::createVariousResolutoinSculpture(const std::string &fileName, int res){
	const int RES_SELECTION[] = { 512, 256, 128, 64 };
	const float VOXEL_SIZE[] = { 0.125f, 0.25f, 0.5f, 1.0f };

	int r = res;
	const int RADIUS = pow(2, (r + 1));
	this->sentLog("Now process resolution " + std::to_string(RES_SELECTION[r]));

	// Initialize Source setting
	// The sculpture buffer is created with max resolution (512)
	m_source->m_width = RES_SELECTION[r];
	m_source->m_height = RES_SELECTION[r];
	m_source->m_length = RES_SELECTION[r];
	m_source->m_total = m_source->m_width * m_source->m_height * m_source->m_length;
	m_source->m_voxelSize = VOXEL_SIZE[r];

	// Initialize the label
	for (int i = 0; i < m_source->m_total; i++){
		m_source->m_sculpture[i].selected = 0;
		m_source->m_sculpture[i].satisfied = 0;
	}

	// Clear computation maps
	int numImage = m_source->m_ioList.size();
	for (int i = 0; i < numImage; i++){
		m_source->m_ioList[i]->clearComputationMaps();
	}

	// Create Sculpture
	m_multiRes[r]->reset();
	for (int i = 0; i < numImage; i++){
		m_multiRes[r]->addNewImage(m_source->m_ioList[i]);
	}

	// Set up Voxel Selector
	VoxelSelector *voxelSelector = new VoxelSelector(m_source);
	voxelSelector->selectBestFitVoxels(0);

	// Calculate the num voxel of initial visual hull
	int numVoxelOfInitialVisualHull = 0;
	for (int i = 0; i < m_source->m_total; i++){
		if (m_source->getLabel(i, 0)){
			numVoxelOfInitialVisualHull++;
		}
	}

	// Repair
	voxelSelector->updateAfterFirstSelect(RADIUS);
	voxelSelector->repairWithRay_180101(m_multiRes[r]);

	// Calculate the num voxel of expanded visual hull
	int numVoxelOfExpandedVisualHull = 0;
	for (int i = 0; i < m_source->m_total; i++){
		if (m_source->getLabel(i, 0)){
			numVoxelOfExpandedVisualHull++;
		}
	}

	// Calculate the ratio
	std::cout << "Initial: " << numVoxelOfInitialVisualHull <<
		"\nExpanded: " << numVoxelOfExpandedVisualHull <<
		"\nRatio: " << numVoxelOfInitialVisualHull * 1.0 / numVoxelOfExpandedVisualHull << "\n";

	// Dilate
	voxelSelector->dilateStructure();

	unsigned char clearMask = 1;
	for (int i = 0; i < m_source->m_total; i++){
		unsigned char label = m_source->m_sculpture[i].selected;
		label = label & clearMask;
		m_source->m_sculpture[i].selected = label;
	}
		
	this->outputVoxelLabel("Results\\VoxelLabel\\" + fileName + "_cv1_" + std::to_string(RES_SELECTION[r]) + "_expanded.vl");
		
	/////////////////////////////////////////////////////////////////////
	// Renew Image
	this->renewImageObject(RADIUS);		
		
	/////////////////////////////////////////////////////////////////////
	// Remove Inner
	voxelSelector->fillInner();

	// Calculate Connected Component
	VoxelConnectedComponent *voxelConnectedComponent = new VoxelConnectedComponent(m_source);
	voxelConnectedComponent->labeling(VoxelConnectedComponent::NEIGHBORS_6, 0);

	if (voxelConnectedComponent->m_groups.size() > 1){
		this->sentLog("Linking Start");
		VoxelConnector *voxelConnector = new VoxelConnector(m_source, voxelConnectedComponent, m_multiRes[r], RADIUS);

		int numTotalVoxel = m_source->m_total;
		int numImage = m_source->m_ioList.size();
		for (int i = 0; i < numTotalVoxel; i++){
			if (m_source->getLabel(i, 0)){
				float center[3];
				m_source->getCenter(center, i);
				for (int j = 0; j < numImage; j++){
					// The radius of cover cirvle should be determined by resolution
					m_source->m_ioList[j]->coverToPathFindingImage(center, 2 * (r + 1));
				}
			}
		}
		for (int i = 0; i < numImage; i++){
			m_source->m_ioList[i]->setUpPathFindingDistanceTransformMap();
		}


		voxelConnector->setupGraph_171201(voxelConnectedComponent->m_groups, VoxelConnector::ASTAR_WITH_CONSTRAINT_VOLUME, fileName);

		int numNode = voxelConnector->m_nodes.size();
		for (int x = 0; x < numNode; x++){
			int numLink = voxelConnector->m_nodes[x]->links.size();
			for (int y = 0; y < numLink; y++){
				if (voxelConnector->m_nodes[x]->links[y].linkType != 0)
					continue;
				std::vector<int> *path = voxelConnector->m_nodes[x]->links[y].pathVoxels;
				int numVoxel = path->size();
				for (int z = 0; z < numVoxel; z++){
					m_source->setLabel(path->at(z), 1, true);
				}
			}
		}
		this->outputVoxelLabel("Results\\VoxelLabel\\" + fileName + "_" + std::to_string(RES_SELECTION[r]) + "_graph.vl");
		// Recover
		const unsigned char RECOVER_MASK = 1;
		for (int i = 0; i < m_source->m_total; i++){
			unsigned char label = m_source->m_sculpture[i].selected;
			label = label & RECOVER_MASK;
			m_source->m_sculpture[i].selected = label;
		}

		voxelConnector->connect();
		this->sentLog("Num Components: " + std::to_string(voxelConnectedComponent->m_groups.size()) + "\nNum Links: " + std::to_string(voxelConnector->m_result.size()) + "\nLinking Finish");
				
		// Select MST 
		const std::vector<VoxelConnector::Node::Link> &links = voxelConnector->m_result;
		int numLink = links.size();
		std::vector<int> *pathVoxels = nullptr;
		for (int i = 0; i < numLink; i++){
			const VoxelConnector::Node::Link &l = links[i];
			pathVoxels = l.pathVoxels;
			// Mark these voxels
			int numPathVoxel = pathVoxels->size();
			for (int j = 0; j < numPathVoxel; j++){
				int voxel = pathVoxels->at(j);
				m_source->setLabel(voxel, 1, true);
			}
		}

		delete voxelConnector;	
	}
		
	// Release
	delete voxelSelector;
	delete voxelConnectedComponent;

		
	// Write Voxel Lable
	this->outputVoxelLabel("Results\\VoxelLabel\\" + fileName + "_" + std::to_string(RES_SELECTION[r]) + ".vl");
		
	// Write Volume
	std::string volumeFileName = "Results\\Volume\\" + fileName + ".vol";
	this->setUpVolume(volumeFileName);	
}
ImageObject *MyPipeline::renewImageObject(int radius){
	int numVoxel = m_source->m_total;
	
	int numImg = m_source->m_ioList.size();
	cv::Mat newImg[Source::MAX_ACCEPTED_IMAGE]; 
	for (int i = 0; i < numImg; i++){
		int imgWidth = m_source->m_ioList[i]->m_imageWidth;
		int imgHeight = m_source->m_ioList[i]->m_imageHeight;
		newImg[i] = cv::Mat(imgHeight, imgWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	}
	
	for (int i = 0; i < numVoxel; i++){
		if (!m_source->getLabel(i, 0))
			continue;

		float v[3];
		m_source->getCenter(v, i);

		for (int j = 0; j < numImg; j++){
			int pixel[3];
			if (m_source->m_ioList[j]->getToleranceProj(v, pixel, 3)){
				cv::circle(newImg[j], cv::Point(pixel[0], pixel[1]), radius, cv::Scalar(0, 0, 0), -1);
			}
		}
	}

	for (int i = 0; i < numImg; i++){
		m_source->m_ioList[i]->renewImage(newImg[i], radius);
		//m_source->m_ioList[i]->createRay();
		//this->sentLog(QString::number(m_source->m_ioList[i]->m_rays.size()));
	}

	return nullptr;
}

bool MyPipeline::sequentiallyProcessMultiSS(const std::vector<std::string> &images, const std::vector<std::string> &sss, const std::vector<std::string> &fileNames, int function){
	std::vector<float*> segs;
	std::vector<int> ptNums;

	if (!this->ready(images)){
		this->sentLog("=============\nCannot process " + fileNames[0]);
		return false;
	}	
	if (!LineWidget::readSimpleSkeletonFile(sss[0], segs, ptNums)){
		this->sentLog("=============\nCannot process SS " + fileNames[0]);
		return false;
	}
	
	if (function == 0){
		this->sentLog("Simplify " + fileNames[0]);
		this->simplify(segs, ptNums, fileNames[0]);
	}	
	else if (function == 1){
		this->optimize(segs, ptNums, fileNames[0]);
	}
			
	// Release
	int numSeg = segs.size();
	for (int j = 0; j < numSeg; j++){
		delete[] segs[j];
	}
	segs.clear();
	ptNums.clear();
		

	int numImage = m_source->m_ioList.size();
	for (int j = 0; j < numImage; j++){
		delete m_source->m_ioList[j];
	}
	m_source->m_ioList.clear();
	
	this->sentLog("Done !");
	
	return true;
}
void MyPipeline::simplify(std::vector<float*> &segs, const std::vector<int> &ptNums, const std::string &fileName){
	this->sentLog("Num of branch of origin: " + std::to_string(segs.size()));
	
	std::vector<float*> correctSegs;
	std::vector<int> correctPtNums;

	LineImprover *lineImprover = new LineImprover();
	lineImprover->correctStructure(segs, ptNums, correctSegs, correctPtNums);
	lineImprover->retopology(correctSegs, correctPtNums, 10000, true);

	// Remove small cycle before doing anything, the Trouble maker
	this->sentLog("Start remove small cycle before doing anything");
	LineImprover4 *lineImprover4 = new LineImprover4(m_source);
	std::vector<float*> correctSegs3;
	std::vector<int> correctPtNums3;
	lineImprover4->removeSmallCycle(correctSegs3, correctPtNums3, correctSegs, correctPtNums);

	// Remove cycle, the Trouble Maker
	LineImprover::removeCycle(correctSegs3, correctPtNums3);

	std::vector<float*> ssSegs = std::vector<float*>(correctSegs3.size());
	std::vector<int> ssPtNums = std::vector<int>(correctPtNums3.size());

	LineImprover::copyLine(ssSegs, ssPtNums, correctSegs3, correctPtNums3);

	lineImprover->retopology(ssSegs, ssPtNums, 10000, true);
	LineWidget::writeSimpleSkeletonFile("Results\\Simplify\\" + fileName + "_Correct2.ss", ssSegs, ssPtNums);
	
	lineImprover4->selectBest_1222(ssSegs, ssPtNums, "", false);
	LineWidget::writeSimpleSkeletonFile("Results\\Simplify\\" + fileName + "_Res.ss", ssSegs, ssPtNums);

	// Progress End, Record
	this->sentLog("Num of branch after optimize: " + std::to_string(ssSegs.size()));

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second Split
	std::vector<float*> correctSegs2;
	std::vector<int> correctPtNums2;
	lineImprover->correctStructure(ssSegs, ssPtNums, correctSegs2, correctPtNums2);

	std::vector<float*> rscSegs;
	std::vector<int> rscPtNums;
	lineImprover4->removeSmallCycle(rscSegs, rscPtNums, correctSegs2, correctPtNums2);

	// Correct topology
	lineImprover4->retopo(rscSegs, rscPtNums);
	LineWidget::writeSimpleSkeletonFile("Results\\Simplify\\" + fileName + "_FinalSimp.ss", rscSegs, rscPtNums);	

	delete lineImprover;
	delete lineImprover4;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void MyPipeline::optimize(std::vector<float*> &segs, const std::vector<int> &ptNums, const std::string &fileName){
	
	std::vector<float*> correctSegs;
	std::vector<int> correctPtNums;
	LineImprover *lineImprover = new LineImprover();
	lineImprover->correctStructure(segs, ptNums, correctSegs, correctPtNums);

	// Curve Fitting
	LineDeformer *lineDeformer = new LineDeformer(m_source);
	std::vector<float*> curveSegments;
	std::vector<int> curvePtNum;
	std::vector<Spline3D*> splines;
	std::vector<float> firstLastLock;

	lineDeformer->initCurve(correctSegs, correctPtNums, splines, firstLastLock);
	LineDeformer::getCurveSamples(curveSegments, curvePtNum, splines, firstLastLock);
	LineWidget::writeSimpleSkeletonFile("Results\\Optimize\\" + fileName + "_curve.ss", curveSegments, curvePtNum);

	lineDeformer->initDeformProcess(splines, firstLastLock);
	lineDeformer->m_sigma = 1.0;
	lineDeformer->m_dtWeight = 0.3;
	lineDeformer->deform_180125(splines, firstLastLock, "", false);

	std::vector<float*> optSegs;
	std::vector<int> optPtNums;
	LineDeformer::getCurveSamples(optSegs, optPtNums, splines, firstLastLock);
	LineWidget::writeSimpleSkeletonFile("Results\\Optimize\\" + fileName + "_optimize.ss", optSegs, optPtNums);

	// Release
	int numSpline = splines.size();
	for (int i = 0; i < numSpline; i++){
		delete splines[i];
	}

	delete lineImprover;
	delete lineDeformer;
	
}

void MyPipeline::solidification(const std::vector<float*> &segs, const std::vector<int> &ptNums, const std::vector<float> &firstLastLock, bool splitJointAndTube, const std::string &outputFileName){
	float m_tubeRadius = 0.30;
	int m_numMaxVertices = 3000000;
	float *m_meshBuffer = new float[m_numMaxVertices * 3 * 2];
	float *m_meshVBuffer = m_meshBuffer;
	float *m_meshNBuffer = m_meshBuffer + m_numMaxVertices * 3;
	unsigned int *indexDataBuffer = new unsigned int[m_numMaxVertices * 2];
	int numVertex;
	int numIndex;

	// Create tube mesh
	Solidificator::solidification(segs, ptNums, m_meshVBuffer, m_meshNBuffer, indexDataBuffer, &numVertex, &numIndex, m_tubeRadius);
	// Output tube mesh
	Solidificator::createOBJ("Results\\objModel\\" + outputFileName + "_tube.obj", m_meshVBuffer, m_meshNBuffer, indexDataBuffer, numVertex, numIndex);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Create joint mesh
	int jointNumVertex;
	int jointNumIndex;
	Solidificator::addJointGeometry(firstLastLock, m_meshVBuffer, m_meshNBuffer, indexDataBuffer, &jointNumVertex, &jointNumIndex, 0, m_tubeRadius * 1.0);
	// Output joint mesh
	Solidificator::createOBJ("Results\\objModel\\" + outputFileName + "_joint.obj", m_meshVBuffer, m_meshNBuffer, indexDataBuffer, jointNumVertex, jointNumIndex);

	delete[] indexDataBuffer;	
}
void MyPipeline::solidficate(const std::string &file, const std::string &outputFileName){
	std::vector<float*> segs;
	std::vector<int> ptNums;

	if (!LineWidget::readSimpleSkeletonFile(file, segs, ptNums)){
		return;
	}

	// Collect first & last
	int numSeg = segs.size();
	std::vector<float> firstLast(numSeg * 2 * 3);
	for (int i = 0; i < numSeg; i++){
		float *f = segs[i];
		float *l = segs[i] + (ptNums[i] - 1) * 3;

		firstLast[i * 6 + 0] = f[0];
		firstLast[i * 6 + 1] = f[1];
		firstLast[i * 6 + 2] = f[2];

		firstLast[i * 6 + 3] = l[0];
		firstLast[i * 6 + 4] = l[1];
		firstLast[i * 6 + 5] = l[2];
	}

	this->solidification(segs, ptNums, firstLast, true, outputFileName);
}
void MyPipeline::setUpVolume(const std::string &outputVolumeFile){

	m_volumeFormat.data = new int[m_source->m_total];

	int volumeIter = 0;
	for (int i = 0; i < m_source->m_width; i++){
		for (int j = 0; j < m_source->m_height; j++){
			for (int k = 0; k < m_source->m_length; k++){
				int index = m_source->getIndex(i, j, k);

				if (m_source->getLabel(index, Source::SELECTED_BIT) || m_source->getLabel(index, 1))
					m_volumeFormat.data[volumeIter] = 1;
				else
					m_volumeFormat.data[volumeIter] = 0;

				volumeIter++;
			}
		}
	}

	m_volumeFormat.sizexyz[0] = m_source->m_width;
	m_volumeFormat.sizexyz[1] = m_source->m_height;
	m_volumeFormat.sizexyz[2] = m_source->m_length;

	m_volumeFormat.spacexyz[0] = (double)(m_source->m_voxelSize);
	m_volumeFormat.spacexyz[1] = (double)(m_source->m_voxelSize);
	m_volumeFormat.spacexyz[2] = (double)(m_source->m_voxelSize);

	m_volumeFormat.cornerPos[0] = (0 - m_source->m_width * 0.5) * m_source->m_voxelSize + m_source->m_structureCenter[0];
	m_volumeFormat.cornerPos[1] = (0 - m_source->m_height * 0.5) * m_source->m_voxelSize + m_source->m_structureCenter[1];
	m_volumeFormat.cornerPos[2] = (0 - m_source->m_length * 0.5) * m_source->m_voxelSize + m_source->m_structureCenter[2];

	m_volumeFormat.threshold = 0;

	// Output .vol File, if reauired
	if (outputVolumeFile != ""){
		std::ofstream output(outputVolumeFile, std::ios::binary);
		if (!output.is_open()){
			return;
		}

		// Write Total voxels
		int totalVoxel = m_source->m_total;
		output.write((char*)(&totalVoxel), 4);

		// Write Data
		output.write((char*)(m_volumeFormat.data), 4 * totalVoxel);

		// Write Size
		output.write((char*)(m_volumeFormat.sizexyz), 4 * 3);

		// Write Space (double)
		output.write((char*)(m_volumeFormat.spacexyz), 8 * 3);

		// Write Corner (double)
		output.write((char*)(m_volumeFormat.cornerPos), 8 * 3);

		// Write Threshold
		int threshold = m_volumeFormat.threshold;
		output.write((char*)(&threshold), 4);

		// Output finish
		output.close();
	}

	delete[] m_volumeFormat.data;
}
void MyPipeline::outputVoxelStructureOBJ(const std::string &vlFile, const std::string &outputFileName){
	ContourArt *ca = new ContourArt(512, 512, 512, 0.125, m_source);
	//ContourArt *ca = new ContourArt(256, 256, 256, 0.25, m_source);
	// Load Voxel Label File
	ifstream input(vlFile);
	if (!input.is_open()){
		return;
	}
	unsigned char *labelBuffer = new unsigned char[m_source->m_total];
	input.read((char*)labelBuffer, m_source->m_total);
	for (int i = 0; i < m_source->m_total; i++){
		m_source->m_sculpture[i].selected = labelBuffer[i];
	}
	for (int i = 0; i < m_source->m_total; i++){
		if (m_source->getLabel(i, 0)){
			m_source->m_sculpture[i].selected = 1;
		}
	}

	input.close();
	///////////////////////////////////////////////////////////////
	// Create Mesh Buffer
	int numMaxVertices = 10 * 1000000;
	float *meshBuffer = new float[numMaxVertices * 3 * 2];
	float *meshVBuffer = meshBuffer;
	float *meshNBuffer = meshBuffer + numMaxVertices * 3;
	//////////////////////////////////////////////////////////////
	VoxelRenderer *voxelRenderer = new VoxelRenderer(m_source);
	int numV = voxelRenderer->voxelGeometry(meshVBuffer, meshNBuffer, 0);
	Solidificator::createVoxelStructureOBJ(outputFileName + "_main.obj", meshVBuffer, meshNBuffer, numV);

	numV = voxelRenderer->voxelGeometry(meshVBuffer, meshNBuffer, 1);
	Solidificator::createVoxelStructureOBJ(outputFileName + "_link.obj", meshVBuffer, meshNBuffer, numV);

	delete[] meshBuffer;	
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MyPipeline::scriptForSkeletonExtraction(const std::string &filename){
	std::ofstream s("Results\\skeleonX64.txt");
	if (!s.is_open()){
		return;
	}

	s << "Results\\Volume\\\n1\n" << filename;
	s.close();
}
void MyPipeline::scriptForCurveProcess(const std::vector<std::string> &imgs, const std::string &filename){
	std::ofstream s("Results\\script1.txt");
	if (!s.is_open()){
		return;
	}
	s << imgs.size() << "\n";
	for (int i = 0; i < imgs.size(); i++){
		s << imgs[i] << "\n";
	}
	s << "1\n" << "Results\\OriginSS\\" + filename + ".ss\n1\n" << filename << "\n" ;
	s.close();	
}