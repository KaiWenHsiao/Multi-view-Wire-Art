#include "VoxelConnector.h"


VoxelConnector::VoxelConnector(Source *s, VoxelConnectedComponent *vcc, ContourArt *ca, int radius) : RADIUS(radius)
{
	m_source = s;
	m_vcc = vcc;
	m_ca = ca;	
}
VoxelConnector::VoxelConnector(Source *s) : RADIUS(0){
	m_source = s;
}
	


VoxelConnector::~VoxelConnector()
{
	/*if (m_disAdjMat != nullptr){
		int numNode = m_nodes.size();
		for (int i = 0; i < numNode; i++){
			delete[] m_disAdjMat[i];
		}		
		delete[] m_disAdjMat;
	}
	*/

	if (m_resAdjMat != nullptr){
		int numNode = m_nodes.size();
		for (int i = 0; i < numNode; i++){
			delete[] m_resAdjMat[i];			
		}
		delete[] m_resAdjMat;		
	}	

	int numPath = m_pathVoxelLibrarys.size();
	for (int i = 0; i < numPath; i++){
		delete m_pathVoxelLibrarys[i];
	}
}

void VoxelConnector::connect(){
	if (m_nodes.size() <= 0){
		return;
	}

	int numNode = m_nodes.size();
	// Create the Adjacency Matrix of Result
	m_resAdjMat = new unsigned char *[numNode];
	for (int i = 0; i < numNode; i++){
		m_resAdjMat[i] = new unsigned char[numNode];

		for (int j = 0; j < numNode; j++){
			m_resAdjMat[i][j] = 0;
		}
	}

	std::priority_queue<Node::Link, std::vector<Node::Link>, std::greater<Node::Link>> pqueue;

	// Reset Node connect flag	
	for (int i = 0; i < numNode; i++){
		m_nodes[i]->haveBeenConnected = 0;
	}

	// Determine Start Vertex
	// Push the links to pqueue
	Node *startNode = m_nodes[0];
	startNode->haveBeenConnected = 1;
	// Start Node is root
	startNode->treeNodeType = 0;
	int numLink = startNode->links.size();
	for (int i = 0; i < numLink; i++){
		// When the link is fixed, it can be a candidate
		if (startNode->links[i].linkType == 0){
			pqueue.push(startNode->links[i]);
		}		
	}
	
	// Create notBeLinkedNode set, push except start node
	std::list<Node*> notBeLinkedNodes;
	for (int i = 1; i < numNode; i++)
		notBeLinkedNodes.push_back(m_nodes[i]);

	while (true){
		// There is no any candidate link
		if (pqueue.empty()){
			// Continue select if there are independent components
			if (!notBeLinkedNodes.empty()){
				// Re start
				std::list<Node*>::iterator iter;
				bool hasCandidateFlag = false;
				for (iter = notBeLinkedNodes.begin(); iter != notBeLinkedNodes.end(); iter++){
					if ((*iter)->links.size() > 0){
						startNode = *iter;
						int nl = startNode->links.size();
						for (int i = 0; i < nl; i++){
							// When the link is fixed, it can be a candidate
							if (startNode->links[i].linkType == 0){
								pqueue.push(startNode->links[i]);
							}
						}
						// Remove
						startNode->haveBeenConnected = 1;
						notBeLinkedNodes.erase(notBeLinkedNodes.begin());

						hasCandidateFlag = true;
						break;
					}	
					else{
						// This node doesn't have any links
						int a = 1;
					}
				}
				if (!hasCandidateFlag){
					break;
				}
			}
			else{
				break;
			}					
		}			

		// Get Smallest Weight
		Node::Link l = pqueue.top();
		pqueue.pop();

		if (l.linked->haveBeenConnected == 1)
			continue;

		// Start Side become internal
		l.self->treeNodeType = 1;
		// End Side become leaf
		l.linked->treeNodeType = 2;

		// Mark adjacency matrix
		m_resAdjMat[l.self->groupId][l.linked->groupId] = 1;
		m_resAdjMat[l.linked->groupId][l.self->groupId] = 1;
		
		// Push to result
		m_result.push_back(l); 
		// Mark the linked node
		l.linked->haveBeenConnected = 1;
		// Push new candidate link that link to node that are not linked
		int numNewLink = l.linked->links.size();
		for (int i = 0; i < numNewLink; i++){
			// When the link is fixed, it can be a candidate
			if (l.linked->links[i].linkType != 0){
				continue;
			}

			if (l.linked->links[i].linked->haveBeenConnected == 0){
				pqueue.push(l.linked->links[i]);
			}			
		}
		// Check warehouse
		bool warehouseFlag = this->checkNotBeLinkeds(notBeLinkedNodes);
		if (warehouseFlag){
			// All nodes have been connected
			break;
		}
	}	
}

bool VoxelConnector::checkNotBeLinkeds(std::list<Node*> &nodeList){
	// Remove that have been connected
	std::list<Node*>::iterator iter;
	for (iter = nodeList.begin(); iter != nodeList.end(); iter++){
		if ((*iter)->haveBeenConnected == 1){
			// Remove
			nodeList.erase(iter);
			break;
		}
	}

	if (nodeList.empty()){
		return true;
	}
	// Linking should be continued
	return false;
}
glm::vec3 VoxelConnector::calculateGroupCenter(const VoxelConnectedComponent::Group *g){
	int numVoxel = g->m_nodes.size();
	float sum[3] = { 0, 0, 0 };
	for (int i = 0; i < numVoxel; i++){
		float center[3];
		m_source->getCenter(center, g->m_nodes[i]);		

		sum[0] = sum[0] + center[0];
		sum[1] = sum[1] + center[1];
		sum[2] = sum[2] + center[2];
	}

	return glm::vec3(
		sum[0] / numVoxel,
		sum[1] / numVoxel,
		sum[2] / numVoxel);
}
void VoxelConnector::collectContourPixel(Node *n){
	int numVoxel = n->relativeGroup->m_nodes.size();
	std::vector<int> neighbors(27);

	for (int i = 0; i < numVoxel; i++){
		int currentIndex = n->relativeGroup->m_nodes[i];
		// Get neighbors
		neighbors.clear();
		this->get26Neighbors(currentIndex, neighbors);
		// If whole neighbors are enabled, it is inner , not contour pixel
		int innerFlag = true;
		int numNeighbor = neighbors.size();
		for (int j = 0; j < numNeighbor; j++){
			if (!m_source->getLabel(neighbors[j], Source::SELECTED_BIT)){
				innerFlag = false;
				break;
			}
		}
		if (!innerFlag){
			n->contourVoxels.push_back(currentIndex);
		}
	}
}
void VoxelConnector::get26Neighbors(int index, std::vector<int> &neighbors){
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

				neighbors.push_back(m_source->getIndex(x, y, z));				
			}
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////
void VoxelConnector::getProjectedComponentCenter(std::vector<Node*> &nodes, ImageObject *io){
	int numNode = nodes.size();
	for (int i = 0; i < numNode; i++){
		Node *n = nodes[i];

		int numVoxel = n->relativeGroup->m_nodes.size();
		int numValidVoxel = 0;
		float sum[3] = { 0.0f, 0.0f, 0.0f };
		for (int j = 0; j < numVoxel; j++){
			// Get voxel center
			float center[3];
			int pixel[2];
			m_source->getCenter(center, n->relativeGroup->m_nodes[j]);
			if (io->getToleranceProj(center, pixel, 3)){
				sum[0] = sum[0] + pixel[0] * 1.0;
				sum[1] = sum[1] + pixel[1] * 1.0;
				sum[2] = 0.0f;
				numValidVoxel = numValidVoxel + 1;
			}
		}

		n->center = glm::vec3(
			sum[0] / numValidVoxel,
			sum[1] / numValidVoxel,
			sum[2] / numValidVoxel);
	}
}
glm::vec3 VoxelConnector::getProjectedComponentCenter(const VoxelConnectedComponent::Group *g, ImageObject *io){
	int numVoxel = g->m_nodes.size();
	int numValidVoxel = 0;
	float sum[3] = { 0.0f, 0.0f, 0.0f };
	for (int j = 0; j < numVoxel; j++){
		// Get voxel center
		float center[3];
		int pixel[2];
		m_source->getCenter(center, g->m_nodes[j]);
		if (io->getToleranceProj(center, pixel, 3)){
			sum[0] = sum[0] + pixel[0] * 1.0;
			sum[1] = sum[1] + pixel[1] * 1.0;
			sum[2] = 0.0f;
			numValidVoxel = numValidVoxel + 1;
		}
	}

	return glm::vec3(
		sum[0] / numValidVoxel,
		sum[1] / numValidVoxel,
		sum[2] / numValidVoxel);
}
float VoxelConnector::getPathNodePriority(const PathNode &pn, const glm::vec3 &terminalComponentCenter, const float length, bool *secondPriority){
	const float SECOND_PRIORITY_THRESHOLD = 0.5f;
	
	float center[3];
	m_source->getCenter(center, pn.voxelIndex);

	float l = glm::length(
		glm::vec3(center[0], center[1], center[2]) - terminalComponentCenter);
	l = l / length;

	float dt = 0;
	int numImage = m_source->m_ioList.size();
	const float DT_THRESHOLD = 10.0f;
	bool sp = false;
	for (int i = 0; i < numImage; i++){
		float d = m_source->m_ioList[i]->getDistanceTransformVerPathFinding(center);

		// Be culled
		if (d < 0){
			return -1;
		}

		d = d / DT_THRESHOLD;
		dt = dt + d;

		if (d > SECOND_PRIORITY_THRESHOLD){
			sp = true;
		}
	}
	
	dt = dt / numImage;

	*secondPriority = sp;

	return 0.6 * dt + 0.4 * l;
}
void VoxelConnector::getSixNeighbors(int voxel, std::vector<int> &neighbors){
	int ic[3];
	int count = 0;
	m_source->getIndexCoord(ic, voxel);

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
		if (
			neighborCoord[i * 3 + 0] < 0 || neighborCoord[i * 3 + 0] >= m_source->m_width ||
			neighborCoord[i * 3 + 1] < 0 || neighborCoord[i * 3 + 1] >= m_source->m_height ||
			neighborCoord[i * 3 + 2] < 0 || neighborCoord[i * 3 + 2] >= m_source->m_length )
		{}
		else{
			neighbors.push_back(m_source->getIndex(neighborCoord[i * 3 + 0], neighborCoord[i * 3 + 1], neighborCoord[i * 3 + 2]));
		}		
	}	
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float VoxelConnector::calculatePathWeight(std::vector<int> *pathVoxels){
	
	// Get centers
	int numVoxel = pathVoxels->size();
	std::vector<glm::vec3> voxelCenters(numVoxel);

	for (int i = 0; i < numVoxel; i++){
		float center[3];
		m_source->getCenter(center, pathVoxels->at(i));
		voxelCenters[i] = glm::vec3(center[0], center[1], center[2]);
	}

	int numImage = m_source->m_ioList.size(); 
	float maxDT = 0;
	for (int i = 0; i < numImage; i++){
		maxDT += m_source->m_ioList[i]->getVoxelPathDistanceTransform(voxelCenters, RADIUS); 		
	}

	return maxDT;
}
//////////////////////////////////////////////////////////////////////////////////////////////
void VoxelConnector::astarWithConstraintVolume(Node *n0, Node *n1, int startVoxelIndex, int goalVoxelIndex){
	// Clear all voxels
	int numTotalVoxel = m_source->m_total;
	unsigned char clearMask = 1;
	for (int i = 0; i < numTotalVoxel; i++){
		unsigned char label = m_source->m_sculpture[i].selected;
		label = label & clearMask;
		m_source->m_sculpture[i].selected = label;
	}

	// Use 2d constraint region
	int numImg = m_source->m_ioList.size();
	for (int i = 0; i < numImg; i++){
		glm::vec3 c0 = n0->projectedCenter[i];
		glm::vec3 c1 = n1->projectedCenter[i];		

		m_source->m_ioList[i]->m_graph->singleVoxel2DShortestPath(
			cv::Point((int)c0.x, (int)c0.y),
			cv::Point((int)c1.x, (int)c1.y),
			n0->patchPixels[i],
			n1->patchPixels[i]);
	}
	
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
	// A* on Constraint volume
	float VOXEL_CENTER_BUFFER[3];
	m_source->getCenter(VOXEL_CENTER_BUFFER, startVoxelIndex);
	const glm::vec3 START_VERTEX = glm::vec3(VOXEL_CENTER_BUFFER[0], VOXEL_CENTER_BUFFER[1], VOXEL_CENTER_BUFFER[2]);

	m_source->getCenter(VOXEL_CENTER_BUFFER, goalVoxelIndex);
	const glm::vec3 GOAL_VERTEX = glm::vec3(VOXEL_CENTER_BUFFER[0], VOXEL_CENTER_BUFFER[1], VOXEL_CENTER_BUFFER[2]);

	const glm::vec3 FORWARD_DIR = GOAL_VERTEX - START_VERTEX;
	const float STRAIGHT_LENGTH = glm::length(FORWARD_DIR);
	// -x, +x, -y, +y, -z, +z
	int QUADRANT_MASK[6];
	for (int i = 0; i < 3; i++){
		if (abs(FORWARD_DIR[i]) < 0.001){
			QUADRANT_MASK[i * 2 + 0] = 1;
			QUADRANT_MASK[i * 2 + 1] = 1;
			continue;
		}
		if (FORWARD_DIR[i] > 0){
			QUADRANT_MASK[i * 2 + 0] = 1;
			QUADRANT_MASK[i * 2 + 1] = 1;
		}
		else{
			QUADRANT_MASK[i * 2 + 0] = 1;
			QUADRANT_MASK[i * 2 + 1] = 1;
		}
	}
	
	const int VISITED_LABEL = 4;
	const int INQUEUE_LABEL = 5;
		
	m_pathNodes.clear();

	std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> pqueue;	

	// Initialize HaveBeenMeet flag
	int numNode = m_nodes.size();
	for (int i = 0; i < numNode; i++){
		m_nodes[i]->haveBeenMeet = 0;
	}

	// Prepare neighbor buffer
	std::vector<int> neighbors(7);

	// Start voxel is the "explicit" path node
	PathNode startPN;
	startPN.parent = -1;
	startPN.indexInArray = 0;
	startPN.voxelIndex = startVoxelIndex;
	startPN.f = 0;
	m_source->setLabel(startVoxelIndex, VISITED_LABEL, true);
	m_pathNodes.push_back(startPN);

	// Push neighbors of candidate
	neighbors.clear();
	this->getSixNeighbors(startVoxelIndex, neighbors);
	int numNeighbor = neighbors.size();
	for (int i = 0; i < numNeighbor; i++){
		bool visit = m_source->getLabel(neighbors[i], VISITED_LABEL);
		bool inqueue = m_source->getLabel(neighbors[i], INQUEUE_LABEL);
		if ((!visit) && (!inqueue) ){
			PathNode pn;
			pn.parent = startPN.indexInArray;
			pn.indexInArray = m_pathNodes.size();
			pn.voxelIndex = neighbors[i];
			pn.g = this->getProjectionError(pn.voxelIndex);
			if (pn.g < 0)
				continue;
			pn.h = this->getHeuristic(pn, GOAL_VERTEX, STRAIGHT_LENGTH);
			pn.f = 0.5 * pn.g + 0.5 * pn.h;

			m_pathNodes.push_back(pn);
			pqueue.push(pn);
			m_source->setLabel(pn.voxelIndex, INQUEUE_LABEL, true);
		}
	}

	

	// First
	PathNode currentPn;
	int step = 0;
	int numMeetNode = 0;
	n0->haveBeenMeet = 1;

	////////////////////////////////////////
	// Prepare for record
	int numCreatedEdge = 0;
	int edgeNumberIndex = -1; 	
	////////////////////////////////////////

	while (true){
		// Get new node
		if (pqueue.empty()){
			break;
		}
		else{
			currentPn = pqueue.top();
			pqueue.pop();
		}

		if (m_source->getLabel(currentPn.voxelIndex, VISITED_LABEL)){
			continue;
		}

		// If meet component b, Jump to b's closet voxel to GOAL
		bool pushNeighborFlag = true;

		// This voxel is relative to the group
		if (m_vcc->m_nodes[currentPn.voxelIndex] != nullptr){
			// Meet other group (Note as g2, g2 != Have been traced group)
			// 1. Create Link between start component & g2
			// 2. if g2 == terminal group, tracing is finished, else continue tracing
			VoxelConnectedComponent::Group *g2 = m_vcc->m_nodes[currentPn.voxelIndex]->groupPtr;

			// Disable the second or more meeting
			//if (m_nodes[g2->indexInArray]->haveBeenMeet == 1)
				//continue;

			// Avoid self-connect
			if (m_nodes[g2->indexInArray]->haveBeenMeet == 0){
				////////////////////////////////////////////////////////////////////////////////////
				// Find the linked group
				// Collect path voxel
				std::vector<int> *pathVoxels = new std::vector<int>();
				VoxelConnectedComponent::Group *linkedGroup = nullptr;
				PathNode traceBackPN = currentPn;
				while (true){
					pathVoxels->push_back(traceBackPN.voxelIndex);
					if (m_vcc->m_nodes[traceBackPN.voxelIndex] != nullptr){
						VoxelConnectedComponent::Group *lg = m_vcc->m_nodes[traceBackPN.voxelIndex]->groupPtr;
						if (m_nodes[lg->indexInArray]->haveBeenMeet == 1){
							// this is the linked group with current
							linkedGroup = lg;
							break;
						}
						else{
							traceBackPN = m_pathNodes[traceBackPN.parent];
						}
					}
					else{
						traceBackPN = m_pathNodes[traceBackPN.parent];
					}
				}
				////////////////////////////////////////////////////////////////////////////////////
				int groupIndex0 = linkedGroup->indexInArray;
				int groupIndex1 = g2->indexInArray;

				// Calculate path weight & compare to existing, if comparison is required
				float pathWeight = this->calculatePathWeight(pathVoxels);
				
				if (m_linkAdjMat[groupIndex0][groupIndex1] == 0){
					// It is the newest edge					

					Node::Link l01;
					l01.self = m_nodes[groupIndex0];
					l01.linked = m_nodes[groupIndex1];
					l01.weight = pathWeight;
					l01.pathVoxels = pathVoxels;
					l01.linkType = 0;
					m_nodes[groupIndex0]->links.push_back(l01);

					Node::Link l10;
					l10.self = m_nodes[groupIndex1];
					l10.linked = m_nodes[groupIndex0];
					l10.weight = pathWeight;
					l10.pathVoxels = pathVoxels;
					l10.linkType = 0;
					m_nodes[groupIndex1]->links.push_back(l10);

					// Record the link
					//m_pathVoxelLibrarys.push_back(pathVoxels);
					m_linkAdjMat[groupIndex0][groupIndex1] = 1;
					m_linkAdjMat[groupIndex1][groupIndex0] = 1;

					////////////////////////////////////////					
				}
				else{
					
					// Compare to existing
					// Find existing one
					int existingEdgeInGroup0 = -1;
					int numEdgeOfG0 = m_nodes[groupIndex0]->links.size();
					for (int i = 0; i < numEdgeOfG0; i++){
						if (m_nodes[groupIndex0]->links[i].linkType == 0 &&m_nodes[groupIndex0]->links[i].linked == m_nodes[groupIndex1]){
							existingEdgeInGroup0 = i;
							break;
						}
					}
					int existingEdgeInGroup1 = -1;
					int numEdgeOfG1 = m_nodes[groupIndex1]->links.size();
					for (int i = 0; i < numEdgeOfG1; i++){
						if (m_nodes[groupIndex1]->links[i].linkType == 0 && m_nodes[groupIndex1]->links[i].linked == m_nodes[groupIndex0]){
							existingEdgeInGroup1 = i;
							break;
						}
					}
					if (existingEdgeInGroup0 < 0 || existingEdgeInGroup1 < 0){
						int a = 1;
					}
					// Compare the weight
					if (m_nodes[groupIndex0]->links[existingEdgeInGroup0].weight > pathWeight){
						// The new one is better
						Node::Link l01;
						l01.self = m_nodes[groupIndex0];
						l01.linked = m_nodes[groupIndex1];
						l01.weight = pathWeight;
						l01.pathVoxels = pathVoxels;
						l01.linkType = 0;
						
						Node::Link l10;
						l10.self = m_nodes[groupIndex1];
						l10.linked = m_nodes[groupIndex0];
						l10.weight = pathWeight;
						l10.pathVoxels = pathVoxels;
						l10.linkType = 0;
						
						// Push new
						m_nodes[groupIndex0]->links.push_back(l01);
						m_nodes[groupIndex1]->links.push_back(l10);
						// Disable elder
						m_nodes[groupIndex0]->links[existingEdgeInGroup0].linkType = 1;
						m_nodes[groupIndex1]->links[existingEdgeInGroup1].linkType = 1;						
					}
					
				}

				numMeetNode++;
				m_nodes[groupIndex1]->haveBeenMeet = 1;

				/////////////////////////////////////////////////////////////////////				
				if (numMeetNode >= m_processedNodeThreshold){
					m_numCannotLink++;
					break;
				}
			}
		}

		// Meet terminal
		if (n1->haveBeenMeet == 1){
			break;
		}

		// Mark VISITED
		m_source->setLabel(currentPn.voxelIndex, VISITED_LABEL, true);
		step++;
		
		if (pushNeighborFlag){
			// Push neighbors
			neighbors.clear();
			//this->getSixNeighbors(currentPn.voxelIndex, neighbors);
			this->getValuableSixCandidate(currentPn, QUADRANT_MASK, neighbors);
			int numNeighbor = neighbors.size();
			for (int i = 0; i < numNeighbor; i++){
				bool visit = m_source->getLabel(neighbors[i], VISITED_LABEL);
				bool inqueue = m_source->getLabel(neighbors[i], INQUEUE_LABEL);

				if (visit || inqueue){
					continue;
				}
				/////////////////////////////////////////////////////////////////
				// Checking with volume
				// Calulate center 
				float wv[3];
				m_source->getCenter(wv, neighbors[i]);

				bool pass = true;
				if (m_source->getLabel(neighbors[i], 0)){
					// Directly pass
				}
				else{					
					//////////////////////////////////////////////////////////////////////
					// CV1
					pass = false;
					for (int j = 0; j < numImg; j++){
						if (m_source->m_ioList[j]->isProjectedIn2DConstraintRegion(wv)){
							pass = true;
							break;
						}						
					}
					//////////////////////////////////////////////////////////////////////					
				}

				if (pass){
					PathNode pn;
					pn.parent = currentPn.indexInArray;
					pn.indexInArray = m_pathNodes.size();
					pn.voxelIndex = neighbors[i];
					pn.g = this->getProjectionError(pn.voxelIndex);

					if (pn.g < 0)
						continue;

					pn.h = this->getHeuristic(pn, GOAL_VERTEX, STRAIGHT_LENGTH);
					pn.f = 0.5 * pn.g + 0.5 * pn.h;

					m_pathNodes.push_back(pn);
					pqueue.push(pn);
					m_source->setLabel(pn.voxelIndex, INQUEUE_LABEL, true);
				}
			}
		}		
	}	
}
void VoxelConnector::efficientAStar(Node *n0, Node *n1, int startVoxelIndex, int goalVoxelIndex){
	// Clear all voxels
	int numTotalVoxel = m_source->m_total;
	unsigned char clearMask = 1;
	for (int i = 0; i < numTotalVoxel; i++){
		unsigned char label = m_source->m_sculpture[i].selected;
		label = label & clearMask;
		m_source->m_sculpture[i].selected = label;
	}

	/////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////
	float VOXEL_CENTER_BUFFER[3];
	m_source->getCenter(VOXEL_CENTER_BUFFER, startVoxelIndex);
	const glm::vec3 START_VERTEX = glm::vec3(VOXEL_CENTER_BUFFER[0], VOXEL_CENTER_BUFFER[1], VOXEL_CENTER_BUFFER[2]);

	m_source->getCenter(VOXEL_CENTER_BUFFER, goalVoxelIndex);
	const glm::vec3 GOAL_VERTEX = glm::vec3(VOXEL_CENTER_BUFFER[0], VOXEL_CENTER_BUFFER[1], VOXEL_CENTER_BUFFER[2]);

	const glm::vec3 FORWARD_DIR = GOAL_VERTEX - START_VERTEX;
	const float STRAIGHT_LENGTH = glm::length(FORWARD_DIR);
	// -x, +x, -y, +y, -z, +z
	int QUADRANT_MASK[6];
	for (int i = 0; i < 3; i++){
		if (abs(FORWARD_DIR[i]) < 0.001){
			QUADRANT_MASK[i * 2 + 0] = 1;
			QUADRANT_MASK[i * 2 + 1] = 1;
			continue;
		}
		if (FORWARD_DIR[i] > 0){
			QUADRANT_MASK[i * 2 + 0] = 1;
			QUADRANT_MASK[i * 2 + 1] = 1;
		}
		else{
			QUADRANT_MASK[i * 2 + 0] = 1;
			QUADRANT_MASK[i * 2 + 1] = 1;
		}
	}
	

	const int VISITED_LABEL = 4;
	const int INQUEUE_LABEL = 5;

	m_pathNodes.clear();

	std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> pqueue;

	// Initialize HaveBeenMeet flag
	int numNode = m_nodes.size();
	for (int i = 0; i < numNode; i++){
		m_nodes[i]->haveBeenMeet = 0;
	}

	// Prepare neighbor buffer
	std::vector<int> neighbors(7);

	// Start voxel is the "explicit" path node
	PathNode startPN;
	startPN.parent = -1;
	startPN.indexInArray = 0;
	startPN.voxelIndex = startVoxelIndex;
	startPN.f = 0;
	m_source->setLabel(startVoxelIndex, VISITED_LABEL, true);
	m_pathNodes.push_back(startPN);

	// Push neighbors of candidate
	neighbors.clear();
	this->getSixNeighbors(startVoxelIndex, neighbors);
	int numNeighbor = neighbors.size();
	for (int i = 0; i < numNeighbor; i++){
		bool visit = m_source->getLabel(neighbors[i], VISITED_LABEL);
		bool inqueue = m_source->getLabel(neighbors[i], INQUEUE_LABEL);
		if ((!visit) && (!inqueue)){
			PathNode pn;
			pn.parent = startPN.indexInArray;
			pn.indexInArray = m_pathNodes.size();
			pn.voxelIndex = neighbors[i];
			pn.g = this->getProjectionError(pn.voxelIndex);
			if (pn.g < 0)
				continue;
			pn.h = this->getHeuristic(pn, GOAL_VERTEX, STRAIGHT_LENGTH);
			pn.f = 0.5 * pn.g + 0.5 * pn.h;

			m_pathNodes.push_back(pn);
			pqueue.push(pn);
			m_source->setLabel(pn.voxelIndex, INQUEUE_LABEL, true);
		}
	}

	// First
	PathNode currentPn;
	int step = 0;
	int numMeetNode = 0;
	n0->haveBeenMeet = 1;

	while (true){
		// Get new node
		if (pqueue.empty()){
			break;
		}
		else{
			currentPn = pqueue.top();
			pqueue.pop();
		}

		if (m_source->getLabel(currentPn.voxelIndex, VISITED_LABEL)){
			continue;
		}

		// This voxel is relative to the group
		if (m_vcc->m_nodes[currentPn.voxelIndex] != nullptr){
			// Meet other group (Note as g2, g2 != Have been traced group)
			// 1. Create Link between start component & g2
			// 2. if g2 == terminal group, tracing is finished, else continue tracing
			VoxelConnectedComponent::Group *g2 = m_vcc->m_nodes[currentPn.voxelIndex]->groupPtr;

			// Disable the second or more meeting
			//if (m_nodes[g2->indexInArray]->haveBeenMeet == 1)
				//continue;

			// Avoid self-connect
			if (m_nodes[g2->indexInArray]->haveBeenMeet == 0){
				////////////////////////////////////////////////////////////////////////////////////
				// Find the linked group
				// Collect path voxel
				std::vector<int> *pathVoxels = new std::vector<int>();
				VoxelConnectedComponent::Group *linkedGroup = nullptr;
				PathNode traceBackPN = currentPn;
				while (true){
					pathVoxels->push_back(traceBackPN.voxelIndex);
					if (m_vcc->m_nodes[traceBackPN.voxelIndex] != nullptr){
						VoxelConnectedComponent::Group *lg = m_vcc->m_nodes[traceBackPN.voxelIndex]->groupPtr;
						if (m_nodes[lg->indexInArray]->haveBeenMeet == 1){
							// this is the linked group with current
							linkedGroup = lg;
							break;
						}
						else{
							traceBackPN = m_pathNodes[traceBackPN.parent];
						}
					}
					else{
						traceBackPN = m_pathNodes[traceBackPN.parent];
					}
				}
				////////////////////////////////////////////////////////////////////////////////////
				int groupIndex0 = linkedGroup->indexInArray;
				int groupIndex1 = g2->indexInArray;

				// Calculate path weight & compare to existing, if comparison is required
				float pathWeight = this->calculatePathWeight(pathVoxels);

				if (m_linkAdjMat[groupIndex0][groupIndex1] == 0){
					// It is the newest edge					

					Node::Link l01;
					l01.self = m_nodes[groupIndex0];
					l01.linked = m_nodes[groupIndex1];
					l01.weight = pathWeight;
					l01.pathVoxels = pathVoxels;
					l01.linkType = 0;
					m_nodes[groupIndex0]->links.push_back(l01);

					Node::Link l10;
					l10.self = m_nodes[groupIndex1];
					l10.linked = m_nodes[groupIndex0];
					l10.weight = pathWeight;
					l10.pathVoxels = pathVoxels;
					l10.linkType = 0;
					m_nodes[groupIndex1]->links.push_back(l10);

					// Record the link
					//m_pathVoxelLibrarys.push_back(pathVoxels);
					m_linkAdjMat[groupIndex0][groupIndex1] = 1;
					m_linkAdjMat[groupIndex1][groupIndex0] = 1;

					////////////////////////////////////////					
				}
				else{
					// Compare to existing
					// Find existing one
					int existingEdgeInGroup0 = -1;
					int numEdgeOfG0 = m_nodes[groupIndex0]->links.size();
					for (int i = 0; i < numEdgeOfG0; i++){
						if (m_nodes[groupIndex0]->links[i].linked == m_nodes[groupIndex1]){
							existingEdgeInGroup0 = i;
						}
					}
					int existingEdgeInGroup1 = -1;
					int numEdgeOfG1 = m_nodes[groupIndex1]->links.size();
					for (int i = 0; i < numEdgeOfG1; i++){
						if (m_nodes[groupIndex1]->links[i].linked == m_nodes[groupIndex0]){
							existingEdgeInGroup1 = i;
						}
					}					
					// Compare the weight
					if (m_nodes[groupIndex0]->links[existingEdgeInGroup0].weight > pathWeight){
						// The new one is better
						Node::Link l01;
						l01.self = m_nodes[groupIndex0];
						l01.linked = m_nodes[groupIndex1];
						l01.weight = pathWeight;
						l01.pathVoxels = pathVoxels;
						l01.linkType = 0;

						Node::Link l10;
						l10.self = m_nodes[groupIndex1];
						l10.linked = m_nodes[groupIndex0];
						l10.weight = pathWeight;
						l10.pathVoxels = pathVoxels;
						l10.linkType = 0;

						// Push new
						m_nodes[groupIndex0]->links.push_back(l01);
						m_nodes[groupIndex1]->links.push_back(l10);
						// Disable elder
						m_nodes[groupIndex0]->links[existingEdgeInGroup0].linkType = 1;
						m_nodes[groupIndex1]->links[existingEdgeInGroup1].linkType = 1;
					}
				}

				numMeetNode++;
				m_nodes[groupIndex1]->haveBeenMeet = 1;
								
			}
		}

		// Meet terminal
		if (n1->haveBeenMeet == 1){
			break;
		}

		// Mark VISITED
		m_source->setLabel(currentPn.voxelIndex, VISITED_LABEL, true);
		step++;

		// Push neighbors
		neighbors.clear();
		//this->getSixNeighbors(currentPn.voxelIndex, neighbors);
		this->getValuableSixCandidate(currentPn, QUADRANT_MASK, neighbors);
		int numNeighbor = neighbors.size();
		for (int i = 0; i < numNeighbor; i++){
			bool visit = m_source->getLabel(neighbors[i], VISITED_LABEL);
			bool inqueue = m_source->getLabel(neighbors[i], INQUEUE_LABEL);

			if ((!visit) && (!inqueue)){
				PathNode pn;
				pn.parent = currentPn.indexInArray;
				pn.indexInArray = m_pathNodes.size();
				pn.voxelIndex = neighbors[i];
				pn.g = this->getProjectionError(pn.voxelIndex);

				if (pn.g < 0)
					continue;

				pn.h = this->getHeuristic(pn, n1->center, STRAIGHT_LENGTH);
				pn.f = 0.5 * pn.g + 0.5 * pn.h;

				// The score trend cannot be ascent
				m_pathNodes.push_back(pn);
				pqueue.push(pn);
				m_source->setLabel(pn.voxelIndex, INQUEUE_LABEL, true);						
			}
		}
	}	
}
bool VoxelConnector::markConstraintVolume(Node *n0, Node *n1, int *markingLabel){
	// Get limited volume from 2d shortest path
	std::vector<cv::Point> path;
	std::vector<int> intersectionVoxels;

	int numImage = m_source->m_ioList.size();
	
	for (int i = 0; i < numImage; i++){
		ImageObject *io = m_source->m_ioList[i];
		// Get Component Center
		glm::vec3 c0 = n0->projectedCenter[i];
		glm::vec3 c1 = n1->projectedCenter[i];

		// Get shortest on 2D
		path.clear();
		io->m_graph->getPath(cv::Point((int)c0.x, (int)c0.y), cv::Point((int)c1.x, (int)c1.y), n0->patchPixels[i], n1->patchPixels[i], path);

		if (path.size() <= 0){
			// These two node cannot link on 2D
			return false;
		}

		// Get intersection voxels
		int numPathPixel = path.size();
		for (int j = 0; j < numPathPixel; j++){
			const cv::Point &p = path[j];
			io->getRay(m_ray, p.x, p.y);
			intersectionVoxels.clear();
			m_ca->getIntersectedVoxels(intersectionVoxels, m_ray);

			// Enable the voxels
			int numVoxel = intersectionVoxels.size();
			for (int k = 0; k < numVoxel; k++){
				m_source->setLabel(intersectionVoxels[k], markingLabel[i], true);				
			}
		}
	}

	// Don't forget component volume
	PixelSet *vs = n0->componentVolume;
	for (int i = 0; i < vs->numPixel; i++){
		m_source->setLabel(vs->pixels[i], markingLabel[0], true);
	}
	vs = n1->componentVolume;
	for (int i = 0; i < vs->numPixel; i++){
		m_source->setLabel(vs->pixels[i], markingLabel[0], true);
	}

	return true;
}
void VoxelConnector::setupProjectedComponentCenter(){
	int numNode = m_nodes.size();
	int numImage = m_source->m_ioList.size();

	for (int i = 0; i < numNode; i++){
		m_nodes[i]->projectedCenter = std::vector<glm::vec3>(numImage);

		for (int j = 0; j < numImage; j++){
			m_nodes[i]->projectedCenter[j] = this->getProjectedComponentCenter(m_nodes[i]->relativeGroup, m_source->m_ioList[j]);
		}
	}
}
void VoxelConnector::setupGraph_171201(const std::vector<VoxelConnectedComponent::Group*> &groups, int astarFunc, const std::string &logFileName){
	int numGroup = groups.size();
	m_nodes = std::vector<Node*>(numGroup);

	// Initialize the ray
	m_ray = new Ray();

	// Create nodes
	for (int i = 0; i < numGroup; i++){
		Node *n = new Node;
		n->groupId = i;
		n->haveBeenConnected = 0;
		n->relativeGroup = groups[i];
		n->center = this->calculateGroupCenter(n->relativeGroup);		
		this->collectContourPixel(n);

		this->collectProjectedPatchPixel(n);

		//this->calculateComponentVolume(n);

		m_nodes[i] = n;
	}

	// Calculate the shortest distance & nearest voxels
	m_shortestDistanceMatrix = new float*[numGroup];
	m_nodeClosetIndexMatrix = new int*[numGroup];
	for (int i = 0; i < numGroup; i++){
		m_shortestDistanceMatrix[i] = new float[numGroup];
		m_nodeClosetIndexMatrix[i] = new int[numGroup];
	}
	for (int i = 0; i < numGroup; i++){
		m_shortestDistanceMatrix[i][i] = 0;
		m_nodeClosetIndexMatrix[i][i] = -1;

		for (int j = i + 1; j < numGroup; j++){
			int v0 = -1, v1 = -1;
			m_shortestDistanceMatrix[i][j] = this->calculateShortestDistanceWithContourVoxel(m_nodes[i], m_nodes[j], &v0, &v1);
			m_shortestDistanceMatrix[j][i] = m_shortestDistanceMatrix[i][j];

			m_nodeClosetIndexMatrix[i][j] = v0;
			m_nodeClosetIndexMatrix[j][i] = v1;
		}		
	}

	// Set up voxel dt table
	m_voxelDTTable = new float[m_source->m_total];
	for (int i = 0; i < m_source->m_total; i++){
		m_voxelDTTable[i] = -1;
	}

	// Set up adjacency matrix
	m_linkAdjMat = new unsigned char *[numGroup];
	for (int i = 0; i < numGroup; i++){
		m_linkAdjMat[i] = new unsigned char[numGroup];

		for (int j = 0; j < numGroup; j++){
			m_linkAdjMat[i][j] = 0;
		}
	}	

	int numCreatedLink = 0;
	std::string totalText = std::to_string(numGroup);
	ProgressTestSender *sender = ProgressTestSender::Instance();

	if (astarFunc == VoxelConnector::ASTAR_WITH_CONSTRAINT_VOLUME){
		sender->addLog("Create Edge With ASTAR ver. Constraint Volume");
		m_createEdgeThreshold = 10.0f;
		m_processedNodeThreshold = 200;
	}
	else{
		sender->addLog("Create Edge With ASTAR ver. Efficient");
		m_createEdgeThreshold = 5.0f;
		m_processedNodeThreshold = 200;		
	}

	/////////////////////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////////////////////
		
	// Create Graph
	sender->updateProgressText("Total step: " + totalText);
	for (int self = 0; self < numGroup; self++){
		sender->updateProgressText(std::to_string(self) + " / " + totalText);

		for (int linked = self + 1; linked < numGroup; linked++){

			if (self == linked){
				continue;
			}

			// Does this pair already have link ?
			if (m_linkAdjMat[self][linked] == 1){
				//continue;
				// Should compare to existing
			}

			float l = m_shortestDistanceMatrix[self][linked];
			
			if (l <= m_createEdgeThreshold){
				
				
				// For those distance smaller than 1, constraint volume is no benifit
				if (astarFunc == VoxelConnector::ASTAR_WITH_CONSTRAINT_VOLUME && l > 1.0){
					this->astarWithConstraintVolume(m_nodes[self], m_nodes[linked], m_nodeClosetIndexMatrix[self][linked], m_nodeClosetIndexMatrix[linked][self]);
				}
				else{
					this->efficientAStar(m_nodes[self], m_nodes[linked], m_nodeClosetIndexMatrix[self][linked], m_nodeClosetIndexMatrix[linked][self]);
				}
			}
		}
	}

	sender->addLog("Num cannot link: " + std::to_string(m_numCannotLink));
	
	// Clear Node's flag
	int numNode = m_nodes.size();
	for (int i = 0; i < numNode; i++){
		m_nodes[i]->haveBeenConnected = 0;
	}

	
	// Clear all voxels
	int numTotalVoxel = m_source->m_total;
	unsigned char clearMask = 1;
	for (int i = 0; i < numTotalVoxel; i++){
		unsigned char label = m_source->m_sculpture[i].selected;
		label = label & clearMask;
		m_source->m_sculpture[i].selected = label;
	}
	

	// Release memories
	for (int i = 0; i < numGroup; i++){
		delete[] m_linkAdjMat[i];
		delete[] m_shortestDistanceMatrix[i];
		delete[] m_nodeClosetIndexMatrix[i];
	}
	delete[] m_linkAdjMat;
	delete[] m_shortestDistanceMatrix;
	delete[] m_nodeClosetIndexMatrix;

	delete[] m_voxelDTTable;	
}
float VoxelConnector::calculateShortestDistanceWithContourVoxel(Node *n0, Node *n1, int *v0, int *v1){
	// Find shortest path
	int n0Voxel = 0;
	int n1Voxel = 0;
	// Get Voxel Center
	float n0Center[3], n1Center[3];
	m_source->getCenter(n0Center, n0->contourVoxels[0]);
	m_source->getCenter(n1Center, n1->contourVoxels[0]);
	// Calculate Length
	float shortestLength = glm::length(
		glm::vec3(n0Center[0], n0Center[1], n0Center[2]) - glm::vec3(n1Center[0], n1Center[1], n1Center[2])
		);

	int numN0ContourVoxel = n0->contourVoxels.size();
	int numN1CountourVoxel = n1->contourVoxels.size();
	for (int i = 0; i < numN0ContourVoxel; i++){
		int n0V = n0->contourVoxels[i];
		float n0VCenter[3];
		m_source->getCenter(n0VCenter, n0V);

		for (int j = 0; j < numN1CountourVoxel; j++){
			int n1V = n1->contourVoxels[j];
			float n1VCenter[3];
			m_source->getCenter(n1VCenter, n1V);

			float l = glm::length(
				glm::vec3(n0VCenter[0], n0VCenter[1], n0VCenter[2]) - glm::vec3(n1VCenter[0], n1VCenter[1], n1VCenter[2])
				);

			if (l < shortestLength){
				shortestLength = l;
				n0Voxel = n0V;
				n1Voxel = n1V;
			}
		}
	}

	*v0 = n0Voxel;
	*v1 = n1Voxel;

	return shortestLength;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
void VoxelConnector::collectProjectedPatchPixel(Node *n){
	int numImage = m_source->m_ioList.size();
	n->patchPixels = std::vector<PixelSet*>(numImage);
	n->projectedCenter = std::vector<glm::vec3>(numImage);

	// Collect voxel center 
	int numVoxel = n->relativeGroup->m_nodes.size();
	float *vertices = new float[numVoxel * 3];
	for (int i = 0; i < numVoxel; i++){
		m_source->getCenter(vertices + i * 3, n->relativeGroup->m_nodes[i]);
	}

	
	for (int i = 0; i < numImage; i++){
		PixelSet *ps = new PixelSet;
		m_source->m_ioList[i]->getPatchPixels(vertices, numVoxel, RADIUS, ps);
		
		n->patchPixels[i] = ps;

		// Calculate patch center
		float sum[2] = { 0.0f, 0.0f };
		for (int j = 0; j < ps->numPixel; j++){
			sum[0] = sum[0] + ps->pixels[j * 2 + 0] * 1.0;
			sum[1] = sum[1] + ps->pixels[j * 2 + 1] * 1.0;
		}

		n->projectedCenter[i] = glm::vec3(
			sum[0] / ps->numPixel,
			sum[1] / ps->numPixel,
			0
		);
	}	
}
float VoxelConnector::getProjectionError(int voxelIndex){

	// Be culled
	if (m_voxelDTTable[voxelIndex] < -3){
		return -1;
	}

	// Has been calculated
	if (m_voxelDTTable[voxelIndex] > 0){
		return m_voxelDTTable[voxelIndex];
	}

	// Not be calculated
	float center[3];
	m_source->getCenter(center, voxelIndex);

	int numImage = m_source->m_ioList.size();
	const float DT_THRESHOLD = 1.0f;
	// Get maximum as priority
	float maxDT = 0;
	for (int i = 0; i < numImage; i++){
		float d = m_source->m_ioList[i]->getDistanceTransformVerPathFinding(center);

		// Be culled
		if (d < 0){
			m_voxelDTTable[voxelIndex] = -5;
			return -1;
		}

		if (d > maxDT){
			maxDT = d;
		}
	}

	m_voxelDTTable[voxelIndex] = maxDT / DT_THRESHOLD;

	return m_voxelDTTable[voxelIndex];
}
float VoxelConnector::getHeuristic(const PathNode &pn, const glm::vec3 &terminalComponentCenter, const float length){
	float center[3];
	m_source->getCenter(center, pn.voxelIndex);

	float l = glm::length(
		glm::vec3(center[0], center[1], center[2]) - terminalComponentCenter);
	l = l * 4;
	
	return l;
}
void VoxelConnector::getValuableSixCandidate(const PathNode &current, int *quadrantMask, std::vector<int> &neighbors){
	int ic[3];
	int count = 0;
	m_source->getIndexCoord(ic, current.voxelIndex);

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
		if (quadrantMask[i] == 0)
			continue;

		if (
			neighborCoord[i * 3 + 0] < 0 || neighborCoord[i * 3 + 0] >= m_source->m_width ||
			neighborCoord[i * 3 + 1] < 0 || neighborCoord[i * 3 + 1] >= m_source->m_height ||
			neighborCoord[i * 3 + 2] < 0 || neighborCoord[i * 3 + 2] >= m_source->m_length)
		{
		}
		else{
			neighbors.push_back(m_source->getIndex(neighborCoord[i * 3 + 0], neighborCoord[i * 3 + 1], neighborCoord[i * 3 + 2]));
		}
	}
}
