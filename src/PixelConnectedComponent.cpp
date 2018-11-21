#include "PixelConnectedComponent.h"


PixelConnectedComponent::PixelConnectedComponent()
{
}


PixelConnectedComponent::~PixelConnectedComponent()
{
	
	delete[] m_nodes;

	for (int i = 0; i < m_parents.size(); i++)
		delete m_parents[i];

	for (int i = 0; i < m_groupNodes.size(); i++){
		delete m_groupNodes[i]->relativeGroup;
		delete m_groupNodes[i];
	}
}

void PixelConnectedComponent::labeling(const cv::Mat &image){
	int totalNode = image.rows * image.cols;

	m_imgWidth = image.cols;
	m_imgHeight = image.rows;

	m_nodes = new NodeParent*[totalNode];
	// Initialize
	for (int i = 0; i < totalNode; i++)
		m_nodes[i] = nullptr;
	m_parents.clear();

	// For checking neighbor
	std::vector<NodeParent*> neighborsLabels(9);

	for (int i = 0; i < m_imgHeight; i++){
		for (int j = 0; j < m_imgWidth; j++){
			// Clear Neighbors
			neighborsLabels.clear();

			// Background
			if (image.at<uchar>(i, j) > 200)
				continue;

			// Check Neighbors' label
			this->getEightNeighbors(j, i, neighborsLabels);

			// New Set
			if (neighborsLabels.size() <= 0){
				int index = m_parents.size();
				NodeParent *p = new NodeParent;
				p->parentId = index;
				m_parents.push_back(p);
				// Set parent 
				m_nodes[i * m_imgWidth + j] = p;
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
				m_nodes[i * m_imgWidth + j] = minNodeParent;
			}
		}
	}

	this->setUpGroups();

}
void PixelConnectedComponent::getEightNeighbors(int cx, int cy, std::vector<NodeParent*> &neighborLabels){

	std::vector<int> alreadyExist(8);
	alreadyExist.clear();


	for (int y = cy - 1; y <= cy + 1; y++){
		if (y < 0 || y >= m_imgHeight){
			continue;
		}

		for (int x = cx - 1; x <= cx + 1; x++){
			if (x < 0 || x >= m_imgWidth){
				continue;
			}

			if (m_nodes[y * m_imgWidth + x] != nullptr){
				int parentId = m_nodes[y * m_imgWidth + x]->parentId;

				// Does it already exist in record
				int r;
				for (r = 0; r < alreadyExist.size(); r++){
					if (alreadyExist[r] == parentId)
						break;
				}
				if (r >= alreadyExist.size()){
					neighborLabels.push_back(m_nodes[y * m_imgWidth + x]);
					alreadyExist.push_back(parentId);
				}
			}
		}
	}
}
void PixelConnectedComponent::setUpGroups(){

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

	int total = m_imgHeight * m_imgWidth;
	for (int i = 0; i < total; i++){
		if (m_nodes[i] == nullptr)
			continue;

		m_nodes[i]->groupPtr->numNode++;
		m_nodes[i]->groupPtr->m_nodes.push_back(i);
	}
	int numGroup = m_groups.size();
	for (int i = 0; i < numGroup; i++){
		m_groups[i]->indexInArray = i;
	}
}
////////////////////////////////////////////////////////////////////////////
void PixelConnectedComponent::connect(){
	if (m_groupNodes.size() <= 0){
		return;
	}

	int numNode = m_groupNodes.size();
	std::priority_queue<Node::Link, std::vector<Node::Link>, std::greater<Node::Link>> pqueue;

	// Reset Node connect flag	
	for (int i = 0; i < numNode; i++){
		m_groupNodes[i]->haveBeenConnected = 0;
	}

	// Determine Start Vertex
	// Push the links to pqueue
	Node *startNode = m_groupNodes[0];
	startNode->haveBeenConnected = 1;
	int numLink = startNode->links.size();
	for (int i = 0; i < numLink; i++){
		pqueue.push(startNode->links[i]);
		
	}

	// Create notBeLinkedNode set, push except start node
	std::list<Node*> notBeLinkedNodes;
	for (int i = 1; i < numNode; i++)
		notBeLinkedNodes.push_back(m_groupNodes[i]);

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
							pqueue.push(startNode->links[i]);							
						}
						// Remove
						startNode->haveBeenConnected = 1;
						notBeLinkedNodes.erase(notBeLinkedNodes.begin());

						hasCandidateFlag = true;
						break;
					}
					else{						
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

		// Push to result
		m_result.push_back(l);
		// Mark the linked node
		l.linked->haveBeenConnected = 1;
		// Push new candidate link that link to node that are not linked
		int numNewLink = l.linked->links.size();
		for (int i = 0; i < numNewLink; i++){
			if (l.linked->links[i].linked->haveBeenConnected == 0){
				pqueue.push(l.linked->links[i]);
			}
		}
		// Check warehouse
		bool warehouseFlag = this->checkNotBeLinked(notBeLinkedNodes);
		if (warehouseFlag){
			// All nodes have been connected
			break;
		}
	}
}
void PixelConnectedComponent::setUpGraph(std::vector<Group*> groups){
	int numGroup = groups.size();
	for (int i = 0; i < numGroup; i++){
		Node *n = new Node;
		n->relativeGroup = groups[i]; 
		m_groupNodes.push_back(n);
	}

	for (int i = 0; i < numGroup; i++){
		for (int j = 1; j < numGroup; j++){
			this->setUpEdge(m_groupNodes[i], m_groupNodes[j]);
		}
	}
}
void PixelConnectedComponent::setUpEdge(Node *n0, Node *n1){
	// Find shortest path
	int n0Pixel = n0->relativeGroup->m_nodes[0];
	int n1Pixel = n1->relativeGroup->m_nodes[1];

	// Get Voxel Center
	int n0PixelIndex[2], n1PixelIndex[2];
	this->getIndex(n0PixelIndex, n0Pixel);
	this->getIndex(n1PixelIndex, n1Pixel);
	// Calculate Length
	float shortestLength = this->getSquaredLength(n0PixelIndex, n1PixelIndex);

	int numN0ContourVoxel = n0->relativeGroup->m_nodes.size();
	int numN1CountourVoxel = n1->relativeGroup->m_nodes.size();
	for (int i = 0; i < numN0ContourVoxel; i++){
		int n0V = n0->relativeGroup->m_nodes[i];
		this->getIndex(n0PixelIndex, n0V);

		for (int j = 0; j < numN1CountourVoxel; j++){
			int n1V = n1->relativeGroup->m_nodes[j];
			this->getIndex(n1PixelIndex, n1V);
			
			float l = this->getSquaredLength(n0PixelIndex, n1PixelIndex);

			if (l < shortestLength){
				shortestLength = l;
				n0Pixel = n0V;
				n1Pixel = n1V;
			}
		}
	}

	// Create Edge
	Node::Link l01;
	l01.self = n0;
	l01.linked = n1;
	l01.selfPixelIndex = n0Pixel;
	l01.linkedPixelIndex = n1Pixel;
	l01.weight = shortestLength;
	n0->links.push_back(l01);

	Node::Link l10;
	l10.self = n1;
	l10.linked = n0;
	l10.selfPixelIndex = n1Pixel;
	l10.linkedPixelIndex = n0Pixel;
	l10.weight = shortestLength;
	n1->links.push_back(l10);
}
bool PixelConnectedComponent::checkNotBeLinked(std::list<Node*> &nodeList){
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
int PixelConnectedComponent::getSquaredLength(int *i0, int *i1){
	float delta[] = { i0[0] - i1[0], i0[1] - i1[1] };
	return delta[0] * delta[0] + delta[1] * delta[1];
}
void PixelConnectedComponent::getIndex(int *index, int id){
	index[0] = id % m_imgWidth;
	index[1] = (id - index[0]) / m_imgWidth;
}