#include "EditorManager.h"


EditorManager::EditorManager()
{
	m_currentProjectionIndex = 0;
}


EditorManager::~EditorManager()
{
	for (int i = 0; i < m_screenWidth; i++)
		delete[] m_pixelDataFlags[i];
	delete[] m_pixelDataFlags;
}

EditorManager *EditorManager::Instance(){
	static EditorManager *m_instance = nullptr;

	if (m_instance == nullptr){
		m_instance = new EditorManager();
	}
	return m_instance;
}

void EditorManager::markEnabledTargetLine(std::vector<EditorLineData*> &targets){
	// Remove all targeted
	int numEnabledPixel = m_currentSelectedPixels.size();

	// First disable all checked flag of whole lines
	int numLine = m_lineDatas.size();
	for (int i = 0; i < numLine; i++){
		m_lineDatas[i]->m_checkedFlag = 0;
		m_lineDatas[i]->m_targetFlag = 0;
	}

	// Get lines who should be checked with enabled pixel set
	for (int i = 0; i < numEnabledPixel; i++){
		const glm::ivec2 &p = m_currentSelectedPixels[i];
		PixelCorresponding ***currentMap = m_correspondingMaps[m_currentProjectionIndex];
		const PixelCorresponding *pc = currentMap[p.x][p.y];

		if (pc == nullptr)
			continue;
		
		int numCorrespondingLine = pc->correspondLines.size();
		for (int j = 0; j < numCorrespondingLine; j++){
			EditorLineData *eld = pc->correspondLines[j];
			if (eld->m_checkedFlag == 0 && eld->m_enabledFlag != 0){
				if (eld->isTarget(m_currentProjectionIndex, this->m_pixelDataFlags)){
					// Mark target
					eld->m_targetFlag = 1;
					targets.push_back(eld);
				}
				// Mark Checked
				eld->m_checkedFlag = 1;
			}
		}
	}
}
void EditorManager::markDisabledTargetLine(std::vector<EditorLineData*> &targets){
	// Remove all targeted
	int numEnabledPixel = m_currentSelectedPixels.size();

	// First disable all checked flag of whole lines
	int numLine = m_lineDatas.size();
	for (int i = 0; i < numLine; i++){
		m_lineDatas[i]->m_checkedFlag = 0;
		m_lineDatas[i]->m_targetFlag = 0;
	}

	// Get lines who should be checked with enabled pixel set
	for (int i = 0; i < numEnabledPixel; i++){
		const glm::ivec2 &p = m_currentSelectedPixels[i];
		PixelCorresponding ***currentMap = m_correspondingMaps[m_currentProjectionIndex];
		const PixelCorresponding *pc = currentMap[p.x][p.y];

		if (pc == nullptr)
			continue;

		int numCorrespondingLine = pc->correspondLines.size();
		for (int j = 0; j < numCorrespondingLine; j++){
			EditorLineData *eld = pc->correspondLines[j];
			if (eld->m_checkedFlag == 0 && eld->m_enabledFlag == 0){
				if (eld->isTarget(m_currentProjectionIndex, this->m_pixelDataFlags)){
					// Mark target
					eld->m_targetFlag = 1;
					targets.push_back(eld);
				}
				// Mark Checked
				eld->m_checkedFlag = 1;
			}
		}
	}
}
void EditorManager::setupLineDatas(const std::vector<float*> &segs, const std::vector<int> &ptNums, const glm::mat4 &proj, std::vector<glm::mat4> &viewMats, int *sideViewProperties){
	int numSideView = viewMats.size();

	// Set up corresponding maps
	m_correspondingMaps = std::vector<PixelCorresponding***>(numSideView);
	for (int i = 0; i < numSideView; i++){

		PixelCorresponding ***map = new PixelCorresponding **[m_screenWidth];
		for (int w = 0; w < m_screenWidth; w++){
			map[w] = new PixelCorresponding *[m_screenHeight];
			for (int h = 0; h < m_screenHeight; h++){
				map[w][h] = nullptr;
			}
		}

		m_correspondingMaps[i] = map;
	}

	// Prepare Mask, In "Side View Space"
	cv::Mat mask = cv::Mat(sideViewProperties[3], sideViewProperties[2], CV_8U, cv::Scalar(0));

	// Create Corresponding Line datas
	int numLine = segs.size();
	for (int i = 0; i < numLine; i++){
		EditorLineData *eld = new EditorLineData();
		eld->m_imageProjectedPixels = std::vector<ProjectedPixels*>(numSideView);
		eld->priority = -1;		
		eld->m_enabledFlag = 1;
		eld->m_correspondingMaterialIndex = i;

		eld->geometry = segs[i];
		eld->numPt = ptNums[i];
		
		// Collect projected pixels
		for (int j = 0; j < numSideView; j++){
			// Get current map
			PixelCorresponding ***currentMap = m_correspondingMaps[j];
			// Get Current Projected Pixels
			ProjectedPixels *currentProjectedPixels = new ProjectedPixels;
			// Calculate Current View-Projection Matrix
			glm::mat4 cvp = proj * viewMats[j];

			// Draw Lines
			mask = cv::Scalar(0);
			LineWidget::projectLine(segs[i], ptNums[i], cvp, mask);

			// Get Pixels
			for (int h = 0; h < sideViewProperties[3]; h++){
				for (int w = 0; w < sideViewProperties[2]; w++){
					const unsigned char &p = mask.at<uchar>(h, w);
					if (p > 200){
						// Change Pixel from "Side View Space" to "Whole Screen Space"
						glm::ivec2 screenSapcePixel = glm::ivec2(w, h) + glm::ivec2(sideViewProperties[0], sideViewProperties[1]);						
						///////////////////////////////////////////////////////
						// Corresponding Map Side
						if (currentMap[screenSapcePixel.x][screenSapcePixel.y] == nullptr){
							currentMap[screenSapcePixel.x][screenSapcePixel.y] = new PixelCorresponding;
							currentMap[screenSapcePixel.x][screenSapcePixel.y]->correspondLines.push_back(eld);
							
						}
						else{
							currentMap[screenSapcePixel.x][screenSapcePixel.y]->correspondLines.push_back(eld);
						}
						///////////////////////////////////////////////////////
						// Line Data Side
						currentProjectedPixels->push_back(screenSapcePixel);
					}
				}
			}
			eld->m_imageProjectedPixels[j] = currentProjectedPixels;
			
		}
		
		m_lineDatas.push_back(eld);
	}

	this->setupOriginImageCoverPixelSet(segs, ptNums);	
}
void EditorManager::clearDataFlags(){
	int numSelected = m_currentSelectedPixels.size();
	for (int i = 0; i < numSelected; i++){
		const glm::ivec2 &pixel = m_currentSelectedPixels[i];
		m_pixelDataFlags[pixel.x][pixel.y] = 0;
	}
	m_currentSelectedPixels.clear();	
}
void EditorManager::setupEditorImageObjects(const std::vector<ImageObject*> &ios){
	int numImage = ios.size();
	m_eios = std::vector<EditorImageObject*>(numImage);
	for (int i = 0; i < numImage; i++){
		m_eios[i] = new EditorImageObject(ios[i]);
	}
}
void EditorManager::setupOriginImageCoverPixelSet(const std::vector<float*> &segs, const std::vector<int> &ptNums){
	std::vector<int> pixelBuffer;
	int numLine = m_lineDatas.size();
	int numImage = m_eios.size();

	for (int i = 0; i < numLine; i++){
		EditorLineData *eld = m_lineDatas[i];
		eld->m_originImageProjectedPixels = std::vector<PixelSet*>(numImage);

		for (int j = 0; j < numImage; j++){
			pixelBuffer.clear();
			m_eios[j]->getSingleIndexCoverPixel(segs[i], ptNums[i], pixelBuffer, 2);
			int numSingleIndexPixel = pixelBuffer.size();
			int *singleIndexPixelBuffer = new int[numSingleIndexPixel];
			for (int k = 0; k < numSingleIndexPixel; k++){
				singleIndexPixelBuffer[k] = pixelBuffer[k];
			}
			PixelSet *ps = new PixelSet;
			ps->pixels = singleIndexPixelBuffer;
			ps->numPixel = numSingleIndexPixel;

			eld->m_originImageProjectedPixels[j] = ps;			
		}
	}	
}
bool EditorManager::setup(const std::string &originSS, const std::string &simpSS, const glm::mat4 &proj, std::vector<glm::mat4> &viewMats, int *sideViewProperties){
	// Record side view properties
	for (int i = 0; i < 4; i++){
		m_sideViewProperties[i] = sideViewProperties[i];
	}
	
	// Get Line from Orig
	if (!LineWidget::readSimpleSkeletonFile(originSS, m_segs, m_ptNums)){
		return false;
	}
	// Get Line from Simp
	std::vector<float*> simpSegs;
	std::vector<int> simpPtNums;
	if (!LineWidget::readSimpleSkeletonFile(simpSS, simpSegs, simpPtNums)){
		return false;
	}

	// Set up functions, the adjacency matrix will be setup at its constructor
	m_functions = new EditorFunctions(m_segs, m_ptNums);

	// Prepare buffer
	const int VERTEX_CAPACITY = 500000;
	m_gBuffer = new float[VERTEX_CAPACITY * 3 * 2];
	m_vBuffer = m_gBuffer;
	m_nBuffer = m_gBuffer + VERTEX_CAPACITY * 3;

	//m_structureGeometry = LineWidget::createLineSegmentSceneObject(m_segs, m_ptNums, m_functions->m_adjacencyMatrix, m_vBuffer, m_nBuffer);
	m_structureGeometry = LineWidget::createLineSegmentSceneObject(m_segs, m_ptNums, m_functions->m_adjacencyMatrix, m_vBuffer, m_nBuffer, glm::vec3(0.1, 0.1, 0.9));

	this->setupLineDatas(m_segs, m_ptNums, proj, viewMats, sideViewProperties);

	// Just Enable the line who exist in Simp
	int numOrigLine = m_segs.size();
	int numSimpLine = simpSegs.size(); 
	// Note the mapping is 1-1
	std::vector<unsigned char> mapped(numSimpLine);
	for (int i = 0; i < numSimpLine; i++)
		mapped[i] = 0;

	for (int i = 0; i < numOrigLine; i++){
		bool isOneOfSimp = false;
		for (int j = 0; j < numSimpLine; j++){
			if (mapped[j] == 1){
				continue;
			}
			if (LineWidget::isSameLine(m_segs[i], m_ptNums[i], simpSegs[j], simpPtNums[j])){
				// Mark the mapped of Simp
				mapped[j] = 1;
				isOneOfSimp = true;
				break;
			}
		}
		if (!isOneOfSimp){
			// Disable
			m_structureGeometry->m_diffuseMaterials[i]->m_vertexCount = 0;
			m_lineDatas[i]->m_enabledFlag = 0;
		}
	}

	// Debug, Gurantee all simp lines are mapped
	for (int i = 0; i < numSimpLine; i++){
		if (mapped[i] == 0){
			int a = 1; 
		}
	}

	m_eios[0]->renderToLineCoverCounterMap(m_lineDatas, 0);
	m_eios[1]->renderToLineCoverCounterMap(m_lineDatas, 1);
	m_eios[2]->renderToLineCoverCounterMap(m_lineDatas, 2);	

	delete[] m_gBuffer;

	return true;
}
void EditorManager::saveRes(const std::string &fileName){
	// Collect Valid
	std::vector<float*> segs;
	std::vector<int> ptNums;

	int numLine = m_lineDatas.size();
	for (int i = 0; i < numLine; i++){
		if (m_lineDatas[i]->m_enabledFlag == 1){
			segs.push_back(m_lineDatas[i]->geometry);
			ptNums.push_back(m_lineDatas[i]->numPt);
		}
	}

	LineWidget::writeSimpleSkeletonFile(fileName, segs, ptNums);
}
void EditorManager::imagePixelToWholeScreenPixel(int *imagePixel, int imageWidth, int imageHeight, int *wholeScreenPixel){
	// Map to side view
	glm::ivec2 sideViewPixel;
	sideViewPixel[0] = (int)floor(imagePixel[0] * (m_sideViewProperties[2] * 1.0 / imageWidth));
	sideViewPixel[1] = (int)floor(imagePixel[1] * (m_sideViewProperties[3] * 1.0 / imageHeight));

	// Map side t0 whole screen space
	wholeScreenPixel[0] = m_sideViewProperties[0] + sideViewPixel[0];
	wholeScreenPixel[1] = m_sideViewProperties[1] + sideViewPixel[1];

}
bool EditorManager::isSelectedPixel(int *wholeScreenPixel){
	if (wholeScreenPixel[0] < 0 || wholeScreenPixel[0] >= m_screenWidth || wholeScreenPixel[1] < 0 || wholeScreenPixel[1] >= m_screenHeight){
		return false;
	}

	if (m_pixelDataFlags[wholeScreenPixel[0]][wholeScreenPixel[1]] == 1){
		return true;
	}

	return false;
}
void EditorManager::setUpForDeform(int *sideViewProperties){
	// Record side view properties
	for (int i = 0; i < 4; i++){
		m_sideViewProperties[i] = sideViewProperties[i];
	}
}