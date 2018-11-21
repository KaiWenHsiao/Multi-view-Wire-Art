#include "EditorImageObject.h"


EditorImageObject::EditorImageObject(ImageObject *io)
{
	m_imageObject = io;

	int h = io->m_imageHeight;
	int w = io->m_imageWidth;
	// Set up various map
	m_lineCoverCounterMap = cv::Mat(h, w, CV_32F, cv::Scalar(0.0f));
	m_structureProjectionMap = cv::Mat(h, w, CV_8U, cv::Scalar(255));
	m_currentDistanceTransformMap = cv::Mat(h, w, CV_32F, cv::Scalar(0.0f));

	m_tmpSPM = cv::Mat(h, w, CV_8U, cv::Scalar(255));
	m_tmpLCCM = cv::Mat(h, w, CV_32F, cv::Scalar(0.0f));

	m_totalPixels = h * w;

	// Collect contour pixel of origin line drawing
	int totalPixel = h * w;
	uchar *pixelData = io->m_originImage8U.data;
	for (int i = 0; i < totalPixel; i++){
		if (pixelData[i] < 100){
			m_originContourPixel.push_back(i);
		}
	}
}


EditorImageObject::~EditorImageObject()
{
}

void EditorImageObject::getSingleIndexCoverPixel(float *seg, const int ptNum, std::vector<int> &singleIndexPixels, int width){
	// Vertices after proj buffer
	std::vector<int> verticesProj(ptNum * 2);
	verticesProj.clear();

	int w = m_imageObject->m_imageWidth;
	int h = m_imageObject->m_imageHeight;
	cv::Mat projMask(h, w, CV_8U, cv::Scalar(0));

	// Proj All Vertices
	for (int j = 0; j < ptNum; j++){
		float *v = seg + 3 * j;
		int pixel[2];
		if (!m_imageObject->getToleranceProj(v, pixel, 3))
			continue;

		verticesProj.push_back(pixel[0]);
		verticesProj.push_back(pixel[1]);
	}

	// Draw Line
	int verticesSize = verticesProj.size() / 2;
	for (int k = 0; k < verticesSize - 1; k++){
		cv::line(
			projMask,
			cv::Point(verticesProj[k * 2 + 0], verticesProj[k * 2 + 1]),
			cv::Point(verticesProj[(k + 1) * 2 + 0], verticesProj[(k + 1) * 2 + 1]),
			cv::Scalar(255),
			width,
			8,
			0
			);
	}

	// Check the pixel and increasing
	int pixelCounter = 0;
	uchar *pixels = projMask.data;
	int total = w * h;

	for (int i = 0; i < total; i++){
		if (pixels[i] > 200){
			singleIndexPixels.push_back(i);
		}
	}	
}
void EditorImageObject::renderToLineCoverCounterMap(const std::vector<EditorLineData*> &lines, int pixelSetIndex){
	int numLines = lines.size();
	float *lccmPtr = (float*)(m_lineCoverCounterMap.data);
	uchar *spmPtr = m_structureProjectionMap.data;
	for (int i = 0; i < numLines; i++){
		// Just render enabled
		if (lines[i]->m_enabledFlag == 1){
			PixelSet *ps = lines[i]->m_originImageProjectedPixels[pixelSetIndex];

			for (int j = 0; j < ps->numPixel; j++){
				int pixelIndex = ps->pixels[j];
				lccmPtr[pixelIndex] = lccmPtr[pixelIndex] + 1.0f;
				spmPtr[pixelIndex] = 0;
			}
		}		
	}

	// Collect contour pixel of structure projection
	int totalPixel = m_imageObject->m_imageWidth * m_imageObject->m_imageHeight;
	for (int i = 0; i < totalPixel; i++){
		if (spmPtr[i] < 100){
			m_origStructureContourPixel.push_back(i);
		}
	}



	/*
	static int outputIndex = 0;
	cv::normalize(m_lineCoverCounterMap, m_lineCoverCounterMap, 0, 1, cv::NORM_MINMAX);
	for (int i = 0; i < m_totalPixels; i++){
		lccmPtr[i] = lccmPtr[i] * 255;
	}
	imwrite("test00000_" + std::to_string(outputIndex) + ".jpg", m_lineCoverCounterMap);
	outputIndex++;
	*/
}

void EditorImageObject::removeLineFromLCCM(PixelSet *ps){
	int numSingleIndexPixel = ps->numPixel;
	float *lccmPtr = (float*)(m_lineCoverCounterMap.data);
	uchar *spmPtr = m_structureProjectionMap.data;

	for (int i = 0; i < numSingleIndexPixel; i++){
		int pixelIndex = ps->pixels[i];
		lccmPtr[pixelIndex] = lccmPtr[pixelIndex] - 1.0f;
		// Check if there is any other contribute to same pixel
		if (lccmPtr[pixelIndex] < 0.5){
			spmPtr[pixelIndex] = 255;
		}
	}
}
void EditorImageObject::getSimilarityVerSimplify(PixelSet *exceptPixels, float *similarity, float similarityTol, float *complexity, float complexityTol){
	float *lccm = (float*)(m_lineCoverCounterMap.data);
	uchar *spm = m_structureProjectionMap.data;
	uchar *orig = m_imageObject->m_originImage8U.data;
	float *currentDT = (float*)(m_currentDistanceTransformMap.data);
	float *originDT = (float*)(m_imageObject->m_distanceTransformVerThin.data);

	float es = 0;
	float ec = 0;

	// Remove the except lines, and record the edited pixel for quick recover
	std::vector<int> shouleBeRecovered(exceptPixels->numPixel);
	shouleBeRecovered.clear();
	for (int i = 0; i < exceptPixels->numPixel; i++){
		int pixelIndex = exceptPixels->pixels[i];
		if (lccm[pixelIndex] - 1 < 0.5){
			// Disable the pixel
			spm[pixelIndex] = 255;
			// Note this pixel should be recover after calculating
			shouleBeRecovered.push_back(pixelIndex);
		}
	}

	// Calculate distance transform of current map
	cv::distanceTransform(m_structureProjectionMap, m_currentDistanceTransformMap, CV_DIST_L2, 3);	

	float bdt = 0;
	int totalCalculatePixels = 0;

	for (int i = 0; i < m_totalPixels; i++){
		if (orig[i] < 200 && spm[i] > 200){
			// Lost, Take DT from Current
			bdt = bdt + 2 * currentDT[i] * 2;
			totalCalculatePixels++;
		}
		else if (orig[i] > 200 && spm[i] < 200){
			// Redundant, Take DT from Origin
			bdt = bdt + originDT[i];
			totalCalculatePixels++;
		}
	}
	if (totalCalculatePixels <= 0){
		es = -1;
	}
	else{
		es = bdt / (similarityTol * totalCalculatePixels);
	}
	

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// Complexity
	int numCoveredPixel = 0;
	int c = 0;
	int imageWidth = m_imageObject->m_imageWidth;
	for (int i = 0; i < m_totalPixels; i++){
		if (orig[i] > 200)
			continue;

		if (lccm[i] > 1.5){
			c += lccm[i];
			numCoveredPixel++;
		}
		if (lccm[i] > 0.5){
			//numCoveredPixel++;
		}
	}
	if (numCoveredPixel > 0){
		ec = c * 1.0 / (complexityTol * numCoveredPixel);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////

	*complexity = ec;
	*similarity = es;

	// Recover Projected Map
	int numShouldBeRecover = shouleBeRecovered.size();
	for (int i = 0; i < numShouldBeRecover; i++)
		spm[shouleBeRecovered[i]] = 0;

	
}
/////////////////////////////////////////////////////////////////
void EditorImageObject::addLineToLCCM(PixelSet *ps){
	int numSingleIndexPixel = ps->numPixel;
	float *lccmPtr = (float*)(m_lineCoverCounterMap.data);
	uchar *spmPtr = m_structureProjectionMap.data;

	for (int i = 0; i < numSingleIndexPixel; i++){
		int pixelIndex = ps->pixels[i];
		lccmPtr[pixelIndex] = lccmPtr[pixelIndex] + 1.0f;
		
		if (lccmPtr[pixelIndex] > 0.5){
			spmPtr[pixelIndex] = 0;
		}
	}
}
float EditorImageObject::getSimilarityVerRepair(PixelSet *addedpixels, float tol){
	float *lccm = (float*)(m_lineCoverCounterMap.data);
	uchar *spm = m_structureProjectionMap.data;
	uchar *orig = m_imageObject->m_originImage8U.data;
	float *currentDT = (float*)(m_currentDistanceTransformMap.data);
	float *originDT = (float*)(m_imageObject->m_distanceTransformVerThin.data);

	// Add the specify lines, and record the edited pixel for quick recover
	std::vector<int> shouleBeRecovered(addedpixels->numPixel);
	shouleBeRecovered.clear();
	for (int i = 0; i < addedpixels->numPixel; i++){
		int pixelIndex = addedpixels->pixels[i];
		if (lccm[pixelIndex] + 1 > 0.5){
			// Enable the pixel, tempory
			spm[pixelIndex] = 0;
			// Note this pixel should be recover after calculating
			shouleBeRecovered.push_back(pixelIndex);
		}
	}

	// Calculate distance transform of current map
	cv::distanceTransform(m_structureProjectionMap, m_currentDistanceTransformMap, CV_DIST_L2, 3);

	float bdt = 0;
	int totalCalculatePixels = 0;

	for (int i = 0; i < m_totalPixels; i++){
		if (orig[i] < 200 && spm[i] > 200){
			// Lost, Take DT from Current
			bdt = bdt + 2 * currentDT[i];
			totalCalculatePixels++;
		}
		else if (orig[i] > 200 && spm[i] < 200){
			// Redundant, Take DT from Origin
			bdt = bdt + originDT[i];
			totalCalculatePixels++;
		}
	}

	// Recover Projected Map
	int numShouldBeRecover = shouleBeRecovered.size();
	for (int i = 0; i < numShouldBeRecover; i++)
		spm[shouleBeRecovered[i]] = 255;

	// Return calculating result
	if (totalCalculatePixels <= 0)
		return -1;
	return bdt / (tol * totalCalculatePixels);
}
int EditorImageObject::getNumNotBeCoveredPixel(PixelSet *ps){
	int numNNCP = 0;
	uchar *spm = m_structureProjectionMap.data;
	
	for (int i = 0; i < ps->numPixel; i++){
		if (spm[ps->pixels[i]] > 200)
			numNNCP++;
	}

	return numNNCP;
}
/////////////////////////////////////////////////////////////////////
void EditorImageObject::getSimilarityAndComplexity(float *s, float *c, std::vector<EditorLineData*> &removedLines, int pixelSetIndex, float similarityTol, float complexityTol){
	// Create TMP map
	m_lineCoverCounterMap.copyTo(m_tmpLCCM);
	m_structureProjectionMap.copyTo(m_tmpSPM);	
	
	// Get Map's data pointer
	float *lccm = (float*)(m_tmpLCCM.data);
	uchar *spm = m_tmpSPM.data;
	uchar *orig = m_imageObject->m_originImage8U.data;
	float *currentDT = (float*)(m_currentDistanceTransformMap.data);
	float *originDT = (float*)(m_imageObject->m_distanceTransformVerThin.data);

	int numRemovedLine = removedLines.size();
	for (int i = 0; i < numRemovedLine; i++){
		PixelSet *ps = removedLines[i]->m_originImageProjectedPixels[pixelSetIndex];
		for (int j = 0; j < ps->numPixel; j++){
			int pixelIndex = ps->pixels[j];
			lccm[pixelIndex] = lccm[pixelIndex] - 1;
			if (lccm[pixelIndex] < 0.5){
				spm[pixelIndex] = 255;
			}
		}
	}

	float eSimilarity = -1;
	float eComplexity = -1;
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Similarity, (Bi-directional distance transform)
	// Calculate distance transform of current map
	cv::distanceTransform(m_tmpSPM, m_currentDistanceTransformMap, CV_DIST_L2, 3);

	float *pixelWeight = (float*)(m_imageObject->m_pixelWeightMap.data);

	float bdt = 0;
	int totalCalculatePixels = 0;

	/*
	// Deviation
	int numTotalStructureProjectionPixel = m_origStructureContourPixel.size();
	for (int i = 0; i < numTotalStructureProjectionPixel; i++){
		int pixelIndex = m_origStructureContourPixel[i];

		if (spm[pixelIndex] > 200)
			continue; 
				
		if (orig[pixelIndex] > 200){
			bdt = bdt + 1.0 * originDT[pixelIndex];
			totalCalculatePixels++;
		}
	}

	// Incompleteness
	int numOriginLineDrawingPixel = m_originContourPixel.size();
	for (int i = 0; i < numOriginLineDrawingPixel; i++){
		int pixelIndex = m_originContourPixel[i];
		if (spm[pixelIndex] > 200){
			bdt = bdt + 2.0 * currentDT[pixelIndex];
			totalCalculatePixels++;
		}
	}
	*/


	
	for (int i = 0; i < m_totalPixels; i++){		
		if (orig[i] < 200 && spm[i] > 200){
			// Lost, Take DT from Current
			bdt = bdt + 2.0 * currentDT[i] * pixelWeight[i];
			totalCalculatePixels++;
		}
		else if (orig[i] > 200 && spm[i] < 200){
			// Redundant, Take DT from Origin
			bdt = bdt + originDT[i];
			totalCalculatePixels++;
		}
		//////////////////////////////////////////////
	}
	
	if (totalCalculatePixels > 0){
		eSimilarity = bdt / (similarityTol * totalCalculatePixels);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////
	// Complexity
	int numCoveredPixel = 0;
	int complexity = 0;	
	/*
	for (int i = 0; i < numTotalStructureProjectionPixel; i++){
		int pixelIndex = m_origStructureContourPixel[i];
		if (orig[pixelIndex] > 200){
			continue;
		}
		if (lccm[i] > 1.5){
			complexity += lccm[i];
			numCoveredPixel++;
		}		
	}
	*/
	
	for (int i = 0; i < m_totalPixels; i++){
		if (orig[i] > 200)
			continue;

		if (lccm[i] > 1.5){
			complexity += lccm[i];
			numCoveredPixel++;
		}
		if (lccm[i] > 0.5){
			//numCoveredPixel++;
		}
	}	
	
	if (numCoveredPixel > 0){
		eComplexity = complexity * 1.0 / (complexityTol * numCoveredPixel);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////
	
	// Return result
	*s = eSimilarity;
	*c = eComplexity;
}
void EditorImageObject::outputLineCoverCounterMap(){
	
	static int outputIndex = 0;

	cv::distanceTransform(m_structureProjectionMap, m_currentDistanceTransformMap, CV_DIST_L2, 3);

	cv::Mat om;
	m_lineCoverCounterMap.copyTo(om);
	cv::normalize(m_currentDistanceTransformMap, m_currentDistanceTransformMap, 0, 1, cv::NORM_MINMAX);
	float *lccmPtr = (float*)(m_currentDistanceTransformMap.data);
	for (int i = 0; i < m_totalPixels; i++){
		lccmPtr[i] = lccmPtr[i] * 255;
	}
	cv::imwrite("test00000_" + std::to_string(outputIndex) + ".jpg", m_currentDistanceTransformMap);
	outputIndex++;
	cv::imwrite("test00000_" + std::to_string(outputIndex) + ".jpg", m_structureProjectionMap);
	outputIndex++;
}
/////////////////////////////////////////////////////////////////////
float EditorImageObject::getAndOutputSimilarityMap(const std::vector<float*> &segs, const std::vector<int> &ptNums, const std::string &fileName, int width, int height){
	cv::Mat structureProjection(m_imageObject->m_imageHeight, m_imageObject->m_imageWidth, CV_8U, cv::Scalar(255));
	// Project lines
	int numLine = segs.size();
	std::vector<int> verticesProj;
	for (int i = 0; i < numLine; i++){
		float *seg = segs[i];
		int numPt = ptNums[i];
		verticesProj.clear();

		for (int j = 0; j < numPt; j++){
			float *v = seg + j * 3;
			int pixel[2];
			if (m_imageObject->getToleranceProj(v, pixel, 3)){
				verticesProj.push_back(pixel[0]);
				verticesProj.push_back(pixel[1]);
			}
		}

		// Draw line
		int verticesSize = verticesProj.size() / 2;
		for (int k = 0; k < verticesSize - 1; k++){
			cv::line(
				structureProjection,
				cv::Point(verticesProj[k * 2 + 0], verticesProj[k * 2 + 1]),
				cv::Point(verticesProj[(k + 1) * 2 + 0], verticesProj[(k + 1) * 2 + 1]),
				cv::Scalar(0),
				4,
				8,
				0
			);
		}
	}

	// Calculate distance transform map
	cv::Mat currDTMap;
	cv::distanceTransform(structureProjection, currDTMap, CV_DIST_L2, 3);
			
	///////////////////////////////////////////////////////////////////////////////////////
	// Prepare Similarity Map
	cv::Mat similarityMap(m_imageObject->m_imageHeight, m_imageObject->m_imageWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	// Calculate Similarity
	uchar *spm = structureProjection.data;
	uchar *orig = m_imageObject->m_originImage8U.data;
	float *currentDT = (float*)(currDTMap.data);
	float *originDT = (float*)(m_imageObject->m_distanceTransformVerThin.data);

	float bdt = 0;
	int totalCalculatePixels = 0;

	for (int i = 0; i < m_totalPixels; i++){
		uchar *mapPixel = similarityMap.data + i * 3;

		if (orig[i] < 200 && spm[i] > 200){
			// Lost, Take DT from Current
			//bdt = bdt + currentDT[i] * 2;
			bdt = bdt + currentDT[i];
			totalCalculatePixels++;

			mapPixel[0] = 0;
			mapPixel[1] = 0;
			mapPixel[2] = 255;
		}
		else if (orig[i] > 200 && spm[i] < 200){
			// Redundant, Take DT from Origin
			bdt = bdt + originDT[i];
			totalCalculatePixels++;

			mapPixel[0] = 0;
			mapPixel[1] = 200;
			mapPixel[2] = 200;
		}
		else if(orig[i] < 200 && spm[i] < 200){
			mapPixel[0] = 0;
			mapPixel[1] = 0;
			mapPixel[2] = 0;
		}
	}
	
	// Resize the image
	cv::resize(similarityMap, similarityMap, cv::Size(width, height));
	//cv::imwrite(fileName, similarityMap);

	float similarityTol = 1.0;
	return bdt / (similarityTol * totalCalculatePixels);	
}
void EditorImageObject::precisionRecall(const std::vector<float*> &segs, const std::vector<int> &ptNums, int *data, float threshold){
	cv::Mat structureProjection(m_imageObject->m_imageHeight, m_imageObject->m_imageWidth, CV_8U, cv::Scalar(255));
	this->getMap(structureProjection, segs, ptNums);

	// Calculate distance transform of current projection
	cv::Mat currDTMap;
	cv::distanceTransform(structureProjection, currDTMap, CV_DIST_L2, 3);

	int ra = 0;
	int relevent = 0;
	int retrieved = 0;

	// Prepare Similarity Map
	//cv::Mat similarityMap(m_imageObject->m_imageHeight, m_imageObject->m_imageWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	// Calculate Similarity
	uchar *spm = structureProjection.data;
	uchar *orig = m_imageObject->m_originImage8U.data;
	float *currentDT = (float*)(currDTMap.data);
	float *originDT = (float*)(m_imageObject->m_distanceTransformVerThin.data);

	float bdt = 0;
	int totalCalculatePixels = 0;

	for (int i = 0; i < m_totalPixels; i++){
		//uchar *mapPixel = similarityMap.data + i * 3;
		float similarity = 100;

		if (orig[i] < 200 && spm[i] > 200){
			// Lost, Take DT from Current
			similarity = currentDT[i];
			relevent++;
			if (similarity <= threshold){
				ra++;
				retrieved++;
			}
		}
		else if (orig[i] > 200 && spm[i] < 200){
			// Redundant, Take DT from Origin
			similarity = originDT[i];
			retrieved++; 
			if (similarity <= threshold){
				ra++;
				relevent++;
			}
		}
		else if (orig[i] < 200 && spm[i] < 200){
			similarity = 0;	
			ra++;
			relevent++;
			retrieved++;
		}

		// Justify relevent & retrieved
		
	}

	data[0] = relevent;
	data[1] = retrieved;
	data[2] = ra;
}
void EditorImageObject::getSimilarities(const std::vector<float*> &segs, const std::vector<int> &ptNums, std::vector<float> &similarities){
	cv::Mat structureProjection(m_imageObject->m_imageHeight, m_imageObject->m_imageWidth, CV_8U, cv::Scalar(255));
	this->getMap(structureProjection, segs, ptNums);

	// Calculate distance transform of current projection
	cv::Mat currDTMap;
	cv::distanceTransform(structureProjection, currDTMap, CV_DIST_L2, 3);

	int ra = 0;
	int relevent = 0;
	int retrieved = 0;

	// Prepare Similarity Map
	//cv::Mat similarityMap(m_imageObject->m_imageHeight, m_imageObject->m_imageWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	// Calculate Similarity
	uchar *spm = structureProjection.data;
	uchar *orig = m_imageObject->m_originImage8U.data;
	float *currentDT = (float*)(currDTMap.data);
	float *originDT = (float*)(m_imageObject->m_distanceTransformVerThin.data);

	float bdt = 0;
	int totalCalculatePixels = 0;

	// Collecting the data
	similarities = std::vector<float>(m_totalPixels);

	for (int i = 0; i < m_totalPixels; i++){
		//uchar *mapPixel = similarityMap.data + i * 3;
		float similarity = 100;

		if (orig[i] < 200 && spm[i] > 200){
			// Lost, Take DT from Current
			similarity = currentDT[i];
			relevent++;
		}
		else if (orig[i] > 200 && spm[i] < 200){
			// Redundant, Take DT from Origin
			similarity = originDT[i];
			retrieved++;
		}
		else if (orig[i] < 200 && spm[i] < 200){
			similarity = 0;
		}

		similarities[i] = similarity;		
	}
}
void EditorImageObject::getMap(cv::Mat &map, const std::vector<float*> &segs, const std::vector<int> &ptNums){
	// Project lines
	int numLine = segs.size();
	std::vector<int> verticesProj;
	for (int i = 0; i < numLine; i++){
		float *seg = segs[i];
		int numPt = ptNums[i];
		verticesProj.clear();

		for (int j = 0; j < numPt; j++){
			float *v = seg + j * 3;
			int pixel[2];
			if (m_imageObject->getToleranceProj(v, pixel, 3)){
				verticesProj.push_back(pixel[0]);
				verticesProj.push_back(pixel[1]);
			}
		}

		// Draw line
		int verticesSize = verticesProj.size() / 2;
		for (int k = 0; k < verticesSize - 1; k++){
			cv::line(
				map,
				cv::Point(verticesProj[k * 2 + 0], verticesProj[k * 2 + 1]),
				cv::Point(verticesProj[(k + 1) * 2 + 0], verticesProj[(k + 1) * 2 + 1]),
				cv::Scalar(0),
				4,
				8,
				0
				);
		}
	}
}
float EditorImageObject::getSimilarities(float *voxelVertices, int numVoxel, int radius){
	cv::Mat structureProjection(m_imageObject->m_imageHeight, m_imageObject->m_imageWidth, CV_8U, cv::Scalar(255));
	// Project lines
	for (int i = 0; i < numVoxel; i++){
		int pixel[2];
		if (this->m_imageObject->getToleranceProj(voxelVertices + i * 3, pixel, 3)){
			cv::circle(structureProjection, cv::Point(pixel[0], pixel[1]), radius, cv::Scalar(0), -1);
		}
	}

	// Calculate distance transform map
	cv::Mat currDTMap;
	cv::distanceTransform(structureProjection, currDTMap, CV_DIST_L2, 3);

	///////////////////////////////////////////////////////////////////////////////////////
	// Calculate Similarity
	uchar *spm = structureProjection.data;
	uchar *orig = m_imageObject->m_originImage8U.data;
	float *currentDT = (float*)(currDTMap.data);
	float *originDT = (float*)(m_imageObject->m_distanceTransformVerThin.data);

	float bdt = 0;
	int totalCalculatePixels = 0;

	for (int i = 0; i < m_totalPixels; i++){
		if (orig[i] < 200 && spm[i] > 200){
			// Lost, Take DT from Current
			bdt = bdt + currentDT[i];
			totalCalculatePixels++;
		}
		else if (orig[i] > 200 && spm[i] < 200){
			// Redundant, Take DT from Origin
			bdt = bdt + originDT[i];
			totalCalculatePixels++;
		}
		else if (orig[i] < 200 && spm[i] < 200){			
		}
	}

	float similarityTol = 1;
	return bdt / (similarityTol * totalCalculatePixels);
}