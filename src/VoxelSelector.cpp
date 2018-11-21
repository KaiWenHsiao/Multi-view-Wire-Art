#include "VoxelSelector.h"


VoxelSelector::VoxelSelector(Source *s) : m_source(s)
{	
}


VoxelSelector::~VoxelSelector()
{
}

void VoxelSelector::initVoxelLabel(){
	int total = m_source->m_width * m_source->m_height * m_source->m_length;

	for (int i = 0; i < total; i++){
		m_source->m_sculpture[i].selected = 0;
	}
}
void VoxelSelector::selectBestFitVoxels(int markedLabel){
	// Calculate best fit condition
	int numImage = m_source->m_ioList.size();
	int bestFitCondition = pow(2, numImage) - 1;

	int total = m_source->m_width * m_source->m_height * m_source->m_length;
	for (int i = 0; i < total; i++){
		
		if (m_source->m_sculpture[i].satisfied == bestFitCondition){
		//if (accept){
			m_source->setLabel(i, markedLabel, true);			
		}
		else{
			m_source->setLabel(i, markedLabel, false);
		}
	}
}
void VoxelSelector::selectPairwiseConsistentVoxels(int markedLabel, bool exactly){
	// Calculate best fit condition
	int numImage = m_source->m_ioList.size();

	int acceptance[] = {
		7, // three intersections
		3, 5, 6 // two intersections
	};

	int startCheckingAcceptance = 0;
	if (exactly){
		startCheckingAcceptance = 1;
	}

	int total = m_source->m_width * m_source->m_height * m_source->m_length;
	for (int i = 0; i < total; i++){
		
		if (m_source->getLabel(i, 0)){
			continue;
		}

		for (int j = startCheckingAcceptance; j < 4; j++){
			if (m_source->m_sculpture[i].satisfied == acceptance[j]){
				m_source->setLabel(i, markedLabel, true);
				break;
			}			
		}		
	}
}
void VoxelSelector::removeNoise(VoxelConnectedComponent::NodeParent **nodes, const std::vector<VoxelConnectedComponent::Group*> &groups){
	if (groups.size() == 1){		
		return;
	}

	// Just Get Max Voxel Connected Component
	// Who has max voxels
	int maxOnes = 0;
	int parentId = -1;
	for (int i = 0; i < groups.size(); i++){
		if (groups[i]->numNode > maxOnes){
			maxOnes = groups[i]->numNode;
			parentId = groups[i]->relativeParentId;
		}
	}

	// Disselect others
	int total = m_source->m_width * m_source->m_height * m_source->m_length;
	int numRemove = 0;
	for (int i = 0; i < total; i++){
		if (nodes[i] != nullptr){
			if (nodes[i]->parentId != parentId){
				m_source->m_sculpture[i].selected = 0;
				numRemove++;
			}
		}
	}

	ProgressTestSender::Instance()->addLog("Num Removed voxel: " + std::to_string(numRemove));
}
void VoxelSelector::removeNoise(VoxelConnectedComponent::NodeParent **nodes, const std::vector<VoxelConnectedComponent::Group*> &groups, int threshold, int label){
	if (groups.size() == 1){
		return;
	}

	int numGroup = groups.size();
	ProgressTestSender::Instance()->addLog("Number of Connect Component: " + std::to_string(numGroup) + "\n");

	std::vector<int> remainedParentId;
	// Remove those too small connected component
	for (int i = 0; i < numGroup; i++){
		if (groups[i]->numNode >= threshold){
			remainedParentId.push_back(groups[i]->relativeParentId);
		}
	}

	int remainedSize = remainedParentId.size();
	ProgressTestSender::Instance()->addLog("Number of Remained Connect Component: " + std::to_string(remainedSize));

	// Disselect others
	int total = m_source->m_width * m_source->m_height * m_source->m_length;
	int numRemove = 0;
	for (int i = 0; i < total; i++){
		if (nodes[i] != nullptr){
			int nodeParentId = nodes[i]->parentId;
			// Is its parent is the remained one or removed one
			bool remainedFlag = false;
			
			for (int j = 0; j < remainedSize; j++){
				if (nodeParentId == remainedParentId[j]){
					remainedFlag = true;
					break;
				}
			}


			if (!remainedFlag){
				m_source->setLabel(i, label, false);
				numRemove++;
			}
		}
	}

	ProgressTestSender::Instance()->addLog("Number of removed Voxel: " + std::to_string(numRemove));
}
void VoxelSelector::updateAfterFirstSelect(int radius){
	int total = m_source->m_width * m_source->m_height * m_source->m_length;
	int imageCount = m_source->m_ioList.size();

	for (int i = 0; i < total; i++){

		if (m_source->getLabel(i, Source::SELECTED_BIT)){			
			// Record the pixel is covered for each image
			float vertex[3];
			int pixel[2];
			m_source->getCenter(vertex, i);
			for (int j = 0; j < imageCount; j++){
				m_source->m_ioList[j]->voxelCover(vertex, radius, 0);
			}			
		}
	}	
}
void VoxelSelector::getNeighbors(int index, int *numNeighbor, int *neighbors){
	int ic[3];
	int count = 0;
	m_source->getIndexCoord(ic, index);

	// 23 neighbor
	for (int x = ic[0] - 1; x <= ic[0] + 1; x++){
		if (x < 0 || x >= m_source->m_width)
			continue;

		for (int y = ic[1] - 1; y <= ic[1] + 1; y++){
			if (y < 0 || y >= m_source->m_height)
				continue;

			for (int z = ic[2] - 1; z <= ic[2] + 1; z++){
				if (z < 0 || z >= m_source->m_length)
					continue;

				neighbors[count] = m_source->getIndex(x, y, z);
				count++;
			}
		}
	}


	*numNeighbor = count;
}
void VoxelSelector::dilateStructure(){
	// Enable 26 neighbors of all-selected voxels
	int numEnhance = 0;

	// Get its 26 neighbors
	int neighbors[27];
	int imageCount = m_source->m_ioList.size();

	int total = m_source->m_width * m_source->m_height * m_source->m_length;

	for (int i = 0; i < total; i++){
		if (m_source->getLabel(i, Source::SELECTED_BIT)){
			int neighborCount;
			//this->getNeighbors(i, &neighborCount, neighbors);
			this->getSixNeighbors(i, neighbors, &neighborCount);

			// Set ENHANCE_BIT
			for (int j = 0; j < neighborCount; j++){
				if (!m_source->getLabel(neighbors[j], Source::SELECTED_BIT)){
					m_source->setLabel(neighbors[j], Source::ENHANCE_BIT, true);					
				}
			}
		}
	}

	// Set enhance as selected
	for (int i = 0; i < total; i++){
		if (m_source->getLabel(i, Source::ENHANCE_BIT)){
			m_source->setLabel(i, Source::SELECTED_BIT, true);
			numEnhance++;
		}
	}

	ProgressTestSender::Instance()->addLog("Number of enhance: " + std::to_string(numEnhance));
}
void VoxelSelector::dilateStructure(int label, int aware){
	// Enable 26 neighbors of all-selected voxels
	int numEnhance = 0;

	// Get its 26 neighbors
	int neighbors[27];
	int imageCount = m_source->m_ioList.size();

	int total = m_source->m_width * m_source->m_height * m_source->m_length;

	for (int i = 0; i < total; i++){


		if (m_source->getLabel(i, label)){
			int neighborCount;
			//this->getNeighbors(i, &neighborCount, neighbors);
			this->getSixNeighbors(i, neighbors, &neighborCount);

			// Set ENHANCE_BIT
			for (int j = 0; j < neighborCount; j++){
				if (!m_source->getLabel(neighbors[j], label)){
					if (m_source->getLabel(neighbors[j], aware)){
						continue;
					}
					m_source->setLabel(neighbors[j], 7, true);
				}
			}
		}
	}

	// Set enhance as selected
	for (int i = 0; i < total; i++){
		if (m_source->getLabel(i, 7)){
			m_source->setLabel(i, label, true);
			m_source->setLabel(i, 7, false);
			numEnhance++;
		}
	}

	ProgressTestSender::Instance()->addLog("Number of enhance: " + std::to_string(numEnhance));
}
void VoxelSelector::getSixNeighbors(int index, int *neighbors, int *neighborCount){
	int ic[3];
	int count = 0;
	m_source->getIndexCoord(ic, index);

	int neighborCoord[] = {
		ic[0] - 1, ic[1], ic[2],
		ic[0] + 1, ic[1], ic[2],
		ic[0], ic[1] - 1, ic[2],
		ic[0], ic[1] + 1, ic[2],
		ic[0], ic[1], ic[2] - 1,
		ic[0], ic[1], ic[2] + 1
	};


	// 6 Neighbors
	for (int i = 0; i < 6; i++){
		int *nc = neighborCoord + i * 3;
		if (nc[0] < 0 || nc[0] >= m_source->m_width ||
			nc[1] < 0 || nc[1] >= m_source->m_height ||
			nc[2] < 0 || nc[2] >= m_source->m_length){
			continue;
		}

		neighbors[count] = m_source->getIndex(nc[0], nc[1], nc[2]);
		count++;
	}

	*neighborCount = count;
}
/////////////////////////////////////////////////////////////////////////////////////
void VoxelSelector::fillInner(){
	int numFill = 0;
	const int CHECK_RANGE = 2;

	for (int y = 0; y < m_source->m_height; y++){
		for (int x = 0; x < m_source->m_width; x++){
			for (int z = 0; z < m_source->m_length; z++){

				int index = m_source->getIndex(x, y, z);
				if ((!m_source->getLabel(index, Source::SELECTED_BIT)) && (!m_source->getLabel(index, 1)) && (!m_source->getLabel(index, 2))){
					// Search Six Direction
					bool innerFlag = false;
					int boundary = -1;

					///////////////////////////////////////////////////////////////
					// +x
					innerFlag = false;
					boundary = glm::clamp<int>(x + CHECK_RANGE, 0, m_source->m_width - 1);
					for (int sx = x + 1; sx <= boundary; sx++){
						int idx = m_source->getIndex(sx, y, z);
						if (m_source->getLabel(idx, Source::SELECTED_BIT) || m_source->getLabel(idx, 1) || m_source->getLabel(idx, 2)){
							innerFlag = true;
							break;
						}
					}
					if (!innerFlag){
						continue;
					}
					///////////////////////////////////////////////////////////////
					// -x
					innerFlag = false;
					boundary = glm::clamp<int>(x - CHECK_RANGE, 0, m_source->m_width - 1);
					for (int sx = x - 1; sx >= boundary; sx--){
						int idx = m_source->getIndex(sx, y, z);
						if (m_source->getLabel(idx, Source::SELECTED_BIT) || m_source->getLabel(idx, 1) || m_source->getLabel(idx, 2)){
							innerFlag = true;
							break;
						}
					}
					if (!innerFlag){
						continue;
					}
					///////////////////////////////////////////////////////////////
					///////////////////////////////////////////////////////////////
					// +y
					innerFlag = false;
					boundary = glm::clamp<int>(y + CHECK_RANGE, 0, m_source->m_height - 1);
					for (int sy = y + 1; sy <= boundary; sy++){
						int idx = m_source->getIndex(x, sy, z);
						if (m_source->getLabel(idx, Source::SELECTED_BIT) || m_source->getLabel(idx, 1) || m_source->getLabel(idx, 2)){
							innerFlag = true;
							break;
						}
					}
					if (!innerFlag){
						continue;
					}
					///////////////////////////////////////////////////////////////
					// -y
					innerFlag = false;
					boundary = glm::clamp<int>(y - CHECK_RANGE, 0, m_source->m_height - 1);
					for (int sy = y - 1; sy >= boundary; sy--){
						int idx = m_source->getIndex(x, sy, z);
						if (m_source->getLabel(idx, Source::SELECTED_BIT) || m_source->getLabel(idx, 1) || m_source->getLabel(idx, 2)){
							innerFlag = true;
							break;
						}
					}
					if (!innerFlag){
						continue;
					}
					///////////////////////////////////////////////////////////////
					///////////////////////////////////////////////////////////////
					// +z
					innerFlag = false;
					boundary = glm::clamp<int>(z + CHECK_RANGE, 0, m_source->m_length - 1);
					for (int sz = z + 1; sz <= boundary; sz++){
						int idx = m_source->getIndex(x, y, sz);
						if (m_source->getLabel(idx, Source::SELECTED_BIT) || m_source->getLabel(idx, 1) || m_source->getLabel(idx, 2)){
							innerFlag = true;
							break;
						}
					}
					if (!innerFlag){
						continue;
					}
					///////////////////////////////////////////////////////////////
					// -z
					innerFlag = false;
					boundary = glm::clamp<int>(z - CHECK_RANGE, 0, m_source->m_length - 1);
					for (int sz = z - 1; sz >= boundary; sz--){
						int idx = m_source->getIndex(x, y, sz);
						if (m_source->getLabel(idx, Source::SELECTED_BIT) || m_source->getLabel(idx, 1) || m_source->getLabel(idx, 2)){
							innerFlag = true;
							break;
						}
					}
					if (!innerFlag){
						continue;
					}

					// All six direction can find selected voxel -> it is not-be-selected inner
					m_source->setLabel(index, Source::SELECTED_BIT, true);
					numFill++;
				}
			}
		}	
	}

	ProgressTestSender::Instance()->addLog("Number of not-be-selected-inner: " + std::to_string(numFill));	
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VoxelSelector::repairWithRay_180101(ContourArt *ca){
	// Setup Density Table
	int numTotalVoxel = m_source->m_width * m_source->m_height * m_source->m_length;
	m_density = new float[numTotalVoxel];
	for (int i = 0; i < numTotalVoxel; i++){
		m_density[i] = 0;
	}
	// Setup Gaussian Kernel
	this->getGaussianKernel();

	// Distribute all-satisfied
	for (int i = 0; i < numTotalVoxel; i++){
		if (m_source->getLabel(i, 0)){
			this->distribute(i);
		}
	}

	/////////////////////////////////////////////////////////////////////////////////
	// A ray of remain pixel
	Ray *r = new Ray();
	// Intersected voxels, will selec best amoung these
	std::vector<int> candidates;

	int numImage = m_source->m_ioList.size();
	int numSelect = 0;

	for (int k = 0; k < numImage; k++){
		const cv::Mat &remainMap = m_source->m_ioList[k]->m_remainMap;

		for (int x = 0; x < remainMap.cols; x++){
			for (int y = 0; y < remainMap.rows; y++){
				if (remainMap.at<uchar>(y, x) > 200){
					continue;
				}

				// Get ray of remain pixel
				m_source->m_ioList[k]->getRay(r, x, y);
				// Get voxels the ray intersected
				candidates.clear();
				ca->getIntersectedVoxels(candidates, r);

				// Select best
				int numCandidate = candidates.size();
				float minPriority = 100000.0;
				int minVoxelIndex = -1;
				for (int j = 0; j < numCandidate; j++){
					// Get priority of the voxel
					int currVoxel = candidates[j];
					float distanceTransform = 0;
					for (int oth = 0; oth < numImage; oth++){
						// Get voxel center
						float center[3];
						m_source->getCenter(center, currVoxel);
						// Get priority = Distance transform
						float p = m_source->m_ioList[oth]->getDistanceTransformVerThin(center);
						if (p < 0){
							// Be culled, it can not be selected
							distanceTransform = -1;
							break;
						}
						else{
							distanceTransform = distanceTransform + p;
						}

					}
					if (distanceTransform >= 0){
						// Calculate Priority
						float eDistanceTransform = distanceTransform / (10 * numImage);
						float eContribution = 1 - m_density[currVoxel];
						float priority = 0.5 * eContribution + 0.5 * eDistanceTransform;
						if (priority < minPriority){
							minPriority = priority;
							minVoxelIndex = currVoxel;
						}
					}
				}

				if (minVoxelIndex >= 0){
					// Enable the voxel
					m_source->setLabel(minVoxelIndex, 0, true);
					// Distribute
					this->distribute(minVoxelIndex);
					numSelect++;
				}

			}
		}
	}	

	delete[] m_density;
}
void VoxelSelector::getGaussianKernel(){
	const float SIGMA = 2.0f;
	const int RANGE = 6;

	float tmp = pow(2 * glm::pi<float>(), 0.5);
	float did = pow(SIGMA * tmp, 3.0);

	for (int x = -RANGE; x <= RANGE; x++){
		for (int y = -RANGE; y <= RANGE; y++){
			for (int z = -RANGE; z <= RANGE; z++){
				float e = exp(-1 * (x*x + y*y + z*z) / (2 * SIGMA * SIGMA));
				m_3dGuassianKernel[z + RANGE][y + RANGE][x + RANGE] = e / did;
			}
		}
	}
}
void VoxelSelector::distribute(int centerVoxel){
	const int RANGE = 6;
	int coord[3];
	m_source->getIndexCoord(coord, centerVoxel);

	int tmp[3];

	for (int x = -RANGE; x <= RANGE; x++){
		tmp[0] = coord[0] + x;
		if (tmp[0] < 0 || tmp[0] >= m_source->m_width)
			continue;

		for (int y = -RANGE; y <= RANGE; y++){
			tmp[1] = coord[1] + y;
			if (tmp[1] < 0 || tmp[1] >= m_source->m_height)
				continue;


			for (int z = -RANGE; z <= RANGE; z++){
				tmp[2] = coord[2] + z;
				if (tmp[2] < 0 || tmp[2] >= m_source->m_length)
					continue;

				int idx = m_source->getIndex(tmp[0], tmp[1], tmp[2]);
				m_density[idx] = m_density[idx] + m_3dGuassianKernel[x + RANGE][y + RANGE][z + RANGE] * 10;
			}
		}
	}
}
	