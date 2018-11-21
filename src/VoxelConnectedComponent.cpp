#include "VoxelConnectedComponent.h"


VoxelConnectedComponent::VoxelConnectedComponent(Source *source)
{
	m_source = source;
}


VoxelConnectedComponent::~VoxelConnectedComponent()
{
	int totalParent = m_parents.size();
	for (int i = 0; i < totalParent; i++)
		delete m_parents[i];
	
	int totalGroup = m_groups.size();
	for (int i = 0; i < totalGroup; i++){
		delete m_groups[i];
	}

	delete[] m_nodes;
}

void VoxelConnectedComponent::labeling(int neighborType, int label){
	int totalNode = m_source->m_width * m_source->m_height * m_source->m_length;

	m_nodes = new NodeParent*[totalNode];
	// Initialize
	for (int i = 0; i < totalNode; i++)
		m_nodes[i] = nullptr;
	m_parents.clear();

	// For checking neighbor
	std::vector<NodeParent*> neighborsLabels(26);
	
	for (int i = 0; i < m_source->m_length; i++){
		for (int j = 0; j < m_source->m_height; j++){
			for (int k = 0; k < m_source->m_width; k++){
				int voxelIndex = m_source->getIndex(k, j, i);

				// Not Selected Voxel
				if (!m_source->getLabel(voxelIndex, label)){
					continue;
				}

				// Clear Neighbors
				neighborsLabels.clear();

				if (neighborType == VoxelConnectedComponent::NEIGHBORS_26){
					// Check Neighbors' label
					this->getNeighborLabels(k, j, i, neighborsLabels);
				}
				else if (neighborType == VoxelConnectedComponent::NEIGHBORS_6){
					// Check Neighbors' label
					this->getSixNeighborLabels(k, j, i, neighborsLabels);
				}
				

				// New Set
				if (neighborsLabels.size() <= 0){
					int index = m_parents.size();
					NodeParent *p = new NodeParent;
					p->parentId = index;
					m_parents.push_back(p);
					// Set parent 
					m_nodes[voxelIndex] = p;
				}
				else if (neighborsLabels.size() == 1){
					m_nodes[voxelIndex] = neighborsLabels[0];
				}
				// Union
				else{
					// Find Minimum Label
					int minLabel = neighborsLabels[0]->parentId;
					NodeParent* minNodeParent = neighborsLabels[0];
					for (int l = 1; l < neighborsLabels.size(); l++){
						if (minLabel > neighborsLabels[l]->parentId){
							minLabel = neighborsLabels[l]->parentId;
							minNodeParent = neighborsLabels[l];
						}
					}
					// Merge
					int numShouldBeMerge = neighborsLabels.size();
					for (int n = 0; n < numShouldBeMerge; n++){
						int target = neighborsLabels[n]->parentId;

						// Change All who are target set
						for (int s = 0; s < m_parents.size(); s++){
							if (m_parents[s]->parentId == target){
								m_parents[s]->parentId = minLabel;
							}
						}
					}
					// Set parent 
					m_nodes[voxelIndex] = minNodeParent;
				}
			}			
		}
	}

	this->setUpGroups();
	
}
void VoxelConnectedComponent::getNeighborLabels(int cx, int cy, int cz, std::vector<NodeParent*> &neighborLabels){
	
	std::vector<int> alreadyExist(26);
	alreadyExist.clear();


	for (int z = cz - 1; z <= cz + 1; z++){
		if (z < 0 || z >= m_source->m_length){
			continue;
		}

		for (int y = cy - 1; y <= cy + 1; y++){
			if (y < 0 || y >= m_source->m_height){
				continue;
			}

			for (int x = cx - 1; x <= cx + 1; x++){
				if (x < 0 || x >= m_source->m_width){
					continue;
				}

				int voxelId = m_source->getIndex(x, y, z);
				
				// Not be labeled
				if (m_nodes[voxelId] == nullptr){
					continue;
				}

				int parentId = m_nodes[voxelId]->parentId;

				// Does it already exist in record
				int r;
				for (r = 0; r < alreadyExist.size(); r++){
					if (alreadyExist[r] == parentId)
						break;
				}
				// The new one
				if (r >= alreadyExist.size()){
					neighborLabels.push_back(m_nodes[voxelId]);
					alreadyExist.push_back(parentId);
				}
			}
		}
	}	
}
void VoxelConnectedComponent::getSixNeighborLabels(int cx, int cy, int cz, std::vector<NodeParent*> &neighborLabels){

	int offset = 1;
	int boundary[6];
	int ic[] = { cx, cy, cz };
	boundary[0] = glm::clamp<int>(ic[0] - offset, 0, m_source->m_width - 1);
	boundary[1] = glm::clamp<int>(ic[0] + offset, 0, m_source->m_width - 1);
	boundary[2] = glm::clamp<int>(ic[1] - offset, 0, m_source->m_height - 1);
	boundary[3] = glm::clamp<int>(ic[1] + offset, 0, m_source->m_height - 1);
	boundary[4] = glm::clamp<int>(ic[2] - offset, 0, m_source->m_length - 1);
	boundary[5] = glm::clamp<int>(ic[2] + offset, 0, m_source->m_length - 1);


	int neighborCoord[] = {
		boundary[0], ic[1], ic[2],
		boundary[1], ic[1], ic[2],
		ic[0], boundary[2], ic[2],
		ic[0], boundary[3], ic[2],
		ic[0], ic[1], boundary[4],
		ic[0], ic[1], boundary[5]
	};

	std::vector<int> alreadyExist(26);
	alreadyExist.clear();

	// 6 Neighbors
	for (int i = 0; i < 6; i++){
		int voxelId = m_source->getIndex(neighborCoord[i * 3 + 0], neighborCoord[i * 3 + 1], neighborCoord[i * 3 + 2]);

		// Not be labeled
		if (m_nodes[voxelId] == nullptr){
			continue;
		}

		int parentId = m_nodes[voxelId]->parentId;

		// Does it already exist in record
		int r;
		for (r = 0; r < alreadyExist.size(); r++){
			if (alreadyExist[r] == parentId)
				break;
		}
		// The new one
		if (r >= alreadyExist.size()){
			neighborLabels.push_back(m_nodes[voxelId]);
			alreadyExist.push_back(parentId);
		}
	}	
}

void VoxelConnectedComponent::setUpGroups(){
	// Check How many connected component
	std::vector<int> parentId;

	// Collect Disjoint
	int ccCount = 0;
	for (int i = 0; i < m_parents.size(); i++){
		int idx;
		for (idx = 0; idx < parentId.size(); idx++){
			if (parentId[idx] == m_parents[i]->parentId)
				break;
		}
		// New One
		if (idx >= parentId.size()){
			Group *g = new Group;
			g->numNode = 0;
			g->relativeParentId = m_parents[i]->parentId;
			m_parents[i]->groupPtr = g;

			parentId.push_back(m_parents[i]->parentId);
			m_groups.push_back(g);
		}
		else{
			m_parents[i]->groupPtr = m_groups[idx];
		}
	}

	int total = m_source->m_width * m_source->m_height * m_source->m_length;
	for (int i = 0; i < total; i++){
		if (m_nodes[i] == nullptr)
			continue;

		m_nodes[i]->groupPtr->numNode++;
		m_nodes[i]->groupPtr->m_nodes.push_back(i);
	}
	int numGroup = m_groups.size();
	for (int i = 0; i < numGroup; i++){
		//m_log += "Parent " + std::to_string(m_groups[i]->relativeParentId) + ": " + std::to_string(m_groups[i]->numNode) + "\n";

		// Record each index in array
		m_groups[i]->indexInArray = i;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
