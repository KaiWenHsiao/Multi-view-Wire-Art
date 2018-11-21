#include "ImageGraph.h"


ImageGraph::ImageGraph(const cv::Mat &thinImg, int radius) : RADIUS(radius)
{
	m_projectedMask = cv::Mat(thinImg.rows, thinImg.cols, CV_8U);
	m_2dConstraintRegionMap = cv::Mat(thinImg.rows, thinImg.cols, CV_8U);

	this->setUpGraph(thinImg);
	this->setUpKDTree();

	// For query use
	m_nearNeighborIndex = new ANNidx[this->NUM_NEAR_NEIGHBOR];
	m_nearNeighborDist = new ANNdist[this->NUM_NEAR_NEIGHBOR];
	m_query = annAllocPt(this->DIM);

	m_pathMask = cv::Mat(thinImg.rows, thinImg.cols, CV_8U);	
	
}


ImageGraph::~ImageGraph()
{
	int numNode = m_nodes.size();
	for (int i = 0; i < numNode; i++)
		delete m_nodes[i];

	//delete m_kdTree;
	annDeallocPts(m_dataSet);
	annDeallocPt(m_query);

	annClose();
}

void ImageGraph::setUpGraph(const cv::Mat &thinImg){
	int imgHeight = thinImg.rows;
	int imgWidth = thinImg.cols;
	int numPixel = imgHeight * imgWidth;
	int numThinContourPixel = 0;

	// Wholw nodes for convenient search
	m_nodeTable = new PixelNode*[numPixel];
	
	// Collect thin contour pixel
	std::vector<cv::Point> thinContourPixels;
	for (int i = 0; i < imgHeight; i++){
		for (int j = 0; j < imgWidth; j++){
			const uchar &pixel = thinImg.at<uchar>(i, j);
			if (pixel < 100){
				// create nodes
				PixelNode *pn = new PixelNode;
				pn->pixel.x = j;
				pn->pixel.y = i;
				
				m_nodeTable[i * imgWidth + j] = pn;	

				numThinContourPixel++;
			}
			else{
				m_nodeTable[i * imgWidth + j] = nullptr;
			}
		}
	}

	// Collect neighbors and create links
	std::vector<int> neighbors(8);
	for (int i = 0; i < imgHeight; i++){
		for (int j = 0; j < imgWidth; j++){
			int nodeIndex = i * imgWidth + j;
			if (m_nodeTable[nodeIndex] != nullptr){
				neighbors.clear();
				this->getEightNeighbors(j, i, neighbors, imgWidth, imgHeight);
				// Record neighbors
				int numNeighbor = neighbors.size();
				for (int k = 0; k < numNeighbor; k++){
					if (m_nodeTable[neighbors[k]] != nullptr){
						m_nodeTable[nodeIndex]->neighbors.push_back(m_nodeTable[neighbors[k]]);
					}
				}
			}
		}
	}

	// Collect thin contour pixels ;
	m_nodes = std::vector<PixelNode*>(numThinContourPixel);
	m_nodes.clear(); 
	for (int i = 0; i < numPixel; i++){
		if (m_nodeTable[i] != nullptr){
			m_nodes.push_back(m_nodeTable[i]);
		}
	}

	this->collectProjectedPixels(RADIUS);
}
void ImageGraph::setUpKDTree(){
	int numNode = m_nodes.size();

	m_dataSet = annAllocPts(numNode, this->DIM);
	for (int i = 0; i < numNode; i++){
		ANNpoint p = m_dataSet[i];
		PixelNode *pn = m_nodes[i]; 
		p[0] = pn->pixel.x;
		p[1] = pn->pixel.y;	
	}

	m_kdTree = new ANNkd_tree(m_dataSet, numNode, this->DIM);
}
void ImageGraph::getEightNeighbors(int x, int y, std::vector<int> &neighbors, const int width, const int height){
	for (int i = -1; i <= 1; i++){

		int ix = x + i;
		if (ix < 0 || ix >= width)
			continue; 

		for (int j = -1; j <= 1; j++){

			int iy = y + j;
			if (iy < 0 || iy >= height)
				continue;

			if (i == 0 && j == 0)
				continue;

			neighbors.push_back(iy * width + ix);
		}

	}
}
ImageGraph::PixelNode *ImageGraph::getNearNeighbor(int x, int y){
	m_query[0] = x; 
	m_query[1] = y;

	m_kdTree->annkSearch(m_query, this->NUM_NEAR_NEIGHBOR, m_nearNeighborIndex, m_nearNeighborDist, this->EPS);
	return m_nodes[m_nearNeighborIndex[0]];
}
void ImageGraph::getPath(const cv::Point &start, const cv::Point &end, PixelSet *ps0, PixelSet *ps1, std::vector<cv::Point> &pathPixels){
	const int RADIUS = 2;
	
	// Prepare path mask
	m_pathMask = cv::Scalar(255);	
	
	// Get nearest pixel of start & end
	PixelNode *nearStartNode = this->getNearNeighbor(start.x, start.y);
	PixelNode *nearEndNode = this->getNearNeighbor(end.x, end.y);

	// Get shortest path
	std::vector<PixelNode*> thinPixelNodePath;
	this->aStar(nearStartNode, nearEndNode, ps0, ps1, thinPixelNodePath);
	int numThinPixelPathNode = thinPixelNodePath.size();

	if (numThinPixelPathNode <= 0){
		pathPixels.clear();
		return;
	}

	// Draw link line
	cv::line(m_pathMask, start, nearStartNode->pixel, cv::Scalar(0), RADIUS);
	cv::line(m_pathMask, end, nearEndNode->pixel, cv::Scalar(0), RADIUS);

	uchar *pm = m_pathMask.data;
	for (int i = 0; i < numThinPixelPathNode; i++){
		int numProjectedPixel = thinPixelNodePath[i]->projectedPixels.size();
		for (int j = 0; j < numProjectedPixel; j++){
			pm[thinPixelNodePath[i]->projectedPixels[j]] = 0;
		}
	}

	// Mark component's projected patch
	int imgWidth = m_pathMask.cols;
	int imgHeight = m_pathMask.rows;
	
	/*
	for (int j = 0; j < ps0->numPixel; j++){
		int *p = ps0->pixels + j * 2;
		pm[p[1] * imgWidth + p[0]] = 0;		
	}
	for (int j = 0; j < ps1->numPixel; j++){
		int *p = ps1->pixels + j * 2;
		pm[p[1] * imgWidth + p[0]] = 0;
	}
	*/
	
	
	// Collect path pixel	
	for (int i = 0; i < imgHeight; i++){
		for (int j = 0; j < imgWidth; j++){
			if (pm[i * imgWidth + j] < 100){
				pathPixels.push_back(cv::Point(j, i));
			}
		}
	}	
}
bool ImageGraph::render2DConstraintRegion(const cv::Point &start, const cv::Point &end, PixelSet *ps0, PixelSet *ps1){
	m_2dConstraintRegionMap = cv::Scalar(255);

	// Get nearest pixel of start & end
	PixelNode *nearStartNode = this->getNearNeighbor(start.x, start.y);
	PixelNode *nearEndNode = this->getNearNeighbor(end.x, end.y);

	// Get shortest path
	std::vector<PixelNode*> thinPixelNodePath;
	this->aStar(nearStartNode, nearEndNode, ps0, ps1, thinPixelNodePath);
	int numThinPixelPathNode = thinPixelNodePath.size();

	uchar *pm = m_2dConstraintRegionMap.data;

	if (numThinPixelPathNode <= 0){
		// Two component may overlap
	}
	else{
		// Draw link line
		cv::line(m_pathMask, start, nearStartNode->pixel, cv::Scalar(0), RADIUS * 2);
		cv::line(m_pathMask, end, nearEndNode->pixel, cv::Scalar(0), RADIUS * 2);

		for (int i = 0; i < numThinPixelPathNode; i++){
			int numProjectedPixel = thinPixelNodePath[i]->projectedPixels.size();
			for (int j = 0; j < numProjectedPixel; j++){
				pm[thinPixelNodePath[i]->projectedPixels[j]] = 0;
			}
		}
	}	

	// Mark component's projected patch
	int imgWidth = m_pathMask.cols;
	int imgHeight = m_pathMask.rows;
		
	for (int j = 0; j < ps0->numPixel; j++){
		int *p = ps0->pixels + j * 2;
		pm[p[1] * imgWidth + p[0]] = 0;
	}
	for (int j = 0; j < ps1->numPixel; j++){
		int *p = ps1->pixels + j * 2;
		pm[p[1] * imgWidth + p[0]] = 0;
	}
	/*
	static int crm = 0;
	cv::imwrite(std::to_string(crm) + "_constraintRegionMap.png", m_2dConstraintRegionMap);
	crm++;
	*/
	return true;	
}
bool ImageGraph::singleVoxel2DShortestPath(const cv::Point &start, const cv::Point &end, PixelSet *ps0, PixelSet *ps1){
	m_2dConstraintRegionMap = cv::Scalar(255);

	// Get nearest pixel of start & end
	PixelNode *nearStartNode = this->getNearNeighbor(start.x, start.y);
	PixelNode *nearEndNode = this->getNearNeighbor(end.x, end.y);

	// Get shortest path
	std::vector<PixelNode*> thinPixelNodePath;
	this->aStar(nearStartNode, nearEndNode, thinPixelNodePath);
	int numThinPixelPathNode = thinPixelNodePath.size();

	uchar *pm = m_2dConstraintRegionMap.data;

	if (numThinPixelPathNode <= 0){
		// Two component may overlap
	}
	else{
		// Draw link line
		cv::line(m_pathMask, start, nearStartNode->pixel, cv::Scalar(0), 8);
		cv::line(m_pathMask, end, nearEndNode->pixel, cv::Scalar(0), 8);

		for (int i = 0; i < numThinPixelPathNode; i++){
			int numProjectedPixel = thinPixelNodePath[i]->projectedPixels.size();
			for (int j = 0; j < numProjectedPixel; j++){
				pm[thinPixelNodePath[i]->projectedPixels[j]] = 0;
			}
		}
	}

	// Mark component's projected patch
	int imgWidth = m_pathMask.cols;
	int imgHeight = m_pathMask.rows;

	for (int j = 0; j < ps0->numPixel; j++){
		int *p = ps0->pixels + j * 2;
		pm[p[1] * imgWidth + p[0]] = 0;
	}
	for (int j = 0; j < ps1->numPixel; j++){
		int *p = ps1->pixels + j * 2;
		pm[p[1] * imgWidth + p[0]] = 0;
	}
	/*
	static int crm = 3;
	cv::imwrite(std::to_string(crm) + "_constraintRegionMap.png", m_2dConstraintRegionMap);
	crm++;
	*/
	return true;
}
float ImageGraph::getLengthSquare(PixelNode *n0, PixelNode *n1){
	float delta[] = {
		n0->pixel.x - n1->pixel.x,
		n0->pixel.y - n1->pixel.y
	};
	
	return delta[0] * delta[0] + delta[1] * delta[1];
}
void ImageGraph::aStar(PixelNode *start, PixelNode *end, PixelSet *startPatch, PixelSet *endPatch, std::vector<PixelNode*> &pathPixels){
	// Initialize data structure
	m_pathNodes.clear();
	std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> pqueue;

	// Initialize visit flag & end flag & start flag
	int numNode = m_nodes.size();
	for (int i = 0; i < numNode; i++){
		m_nodes[i]->visited = 0;
		m_nodes[i]->startPixel = 0;
		m_nodes[i]->endPixel = 0;
	}

	// Mark pixel of end patch
	int imgWidth = m_pathMask.cols;
	int imgHeight = m_pathMask.rows;
	int numPixelOfEndPatch = endPatch->numPixel;
	for (int i = 0; i < numPixelOfEndPatch; i++){
		int *p = endPatch->pixels + i * 2;
		PixelNode *n = m_nodeTable[p[1] * imgWidth + p[0]];
		if (n != nullptr){
			// Set as end pixel
			n->endPixel = 1;
		}
	}
	end->endPixel = 1;

	// Collect start node
	int numPixelOfStartPatch = startPatch->numPixel;
	for (int i = 0; i < numPixelOfStartPatch; i++){
		int *p = startPatch->pixels + i * 2;
		PixelNode *n = m_nodeTable[p[1] * imgWidth + p[0]];
		if (n != nullptr){
			// Create start node
			PathNode pn;
			pn.parent = -1;
			pn.p = n;
			pn.g = 0;
			pn.h = sqrt(this->getLengthSquare(n, end));;
			pn.f = pn.h + pn.g;
			pn.indexInArray = m_pathNodes.size();
			// Record the node
			m_pathNodes.push_back(pn);
			// Push to pqueue
			pqueue.push(pn);
			// Mark as start pixel
			n->startPixel = 1;
		}
	}
	if (start->startPixel == 0){
		// This means start patch doesn't contain any node
		PathNode pn;
		pn.parent = -1;
		pn.p = start;
		pn.g = 0;
		pn.h = sqrt(this->getLengthSquare(start, end));;
		pn.f = pn.h + pn.g;
		pn.indexInArray = m_pathNodes.size();
		// Record the node
		m_pathNodes.push_back(pn);
		// Push to pqueue
		pqueue.push(pn);
	}
	

	PixelNode *currentPixel = nullptr;
	PathNode currentPathNode;
	std::vector<int> neighbors(8);
	while (true){
		if (pqueue.empty()){
			return;
		}			

		// Get lowest cost node
		currentPathNode = pqueue.top();
		currentPixel = currentPathNode.p;
		currentPixel->visited = 1;
		pqueue.pop();

		if (currentPixel->endPixel == 1)
			break;
		
		// Push its neighbors
		int numNeighbor = currentPixel->neighbors.size();
		for (int i = 0; i < numNeighbor; i++){
			if (currentPixel->neighbors[i]->visited == 0){
				// Create new path node
				PathNode spn;
				spn.parent = currentPathNode.indexInArray;
				spn.p = currentPixel->neighbors[i];
				spn.g = currentPathNode.g + 1;
				spn.h = sqrt(this->getLengthSquare(currentPixel->neighbors[i], end));
				spn.f = spn.g + spn.h;
				// Record
				spn.indexInArray = m_pathNodes.size();
				m_pathNodes.push_back(spn);
				// Push to pqueue
				pqueue.push(spn);
			}
		}
	}

	// Trace from end to start
	while (true){
		pathPixels.push_back(currentPathNode.p);
		if (currentPathNode.parent == -1)
			break;
		else
			currentPathNode = m_pathNodes[currentPathNode.parent];
	}
}
void ImageGraph::aStar(PixelNode *start, PixelNode *end, std::vector<PixelNode*> &pathPixels){
	// Initialize data structure
	m_pathNodes.clear();
	std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> pqueue;

	// Initialize visit flag & end flag & start flag
	int numNode = m_nodes.size();
	for (int i = 0; i < numNode; i++){
		m_nodes[i]->visited = 0;		
	}
	
	// This means start patch doesn't contain any node
	PathNode pn;
	pn.parent = -1;
	pn.p = start;
	pn.g = 0;
	pn.h = sqrt(this->getLengthSquare(start, end));;
	pn.f = pn.h + pn.g;
	pn.indexInArray = m_pathNodes.size();
	// Record the node
	m_pathNodes.push_back(pn);
	// Push to pqueue
	pqueue.push(pn);	


	PixelNode *currentPixel = nullptr;
	PathNode currentPathNode;
	std::vector<int> neighbors(8);
	while (true){
		if (pqueue.empty()){
			return;
		}

		// Get lowest cost node
		currentPathNode = pqueue.top();
		currentPixel = currentPathNode.p;
		currentPixel->visited = 1;
		pqueue.pop();

		if (currentPixel == end)
			break;

		// Push its neighbors
		int numNeighbor = currentPixel->neighbors.size();
		for (int i = 0; i < numNeighbor; i++){
			if (currentPixel->neighbors[i]->visited == 0){
				// Create new path node
				PathNode spn;
				spn.parent = currentPathNode.indexInArray;
				spn.p = currentPixel->neighbors[i];
				spn.g = currentPathNode.g + 1;
				spn.h = sqrt(this->getLengthSquare(currentPixel->neighbors[i], end));
				spn.f = spn.g + spn.h;
				// Record
				spn.indexInArray = m_pathNodes.size();
				m_pathNodes.push_back(spn);
				// Push to pqueue
				pqueue.push(spn);
			}
		}
	}

	// Trace from end to start
	while (true){
		pathPixels.push_back(currentPathNode.p);
		if (currentPathNode.parent == -1)
			break;
		else
			currentPathNode = m_pathNodes[currentPathNode.parent];
	}
}
void ImageGraph::collectProjectedPixels(int radius){
	int imgHeight = m_projectedMask.rows;
	int imgWidth = m_projectedMask.cols;
	int numPixel = imgHeight * imgWidth;
	uchar *pm = m_projectedMask.data;

	int numNode = m_nodes.size();
	
	for (int i = 0; i < numNode; i++){
		
		m_nodes[i]->projectedPixels.clear();
		m_projectedMask = cv::Scalar(255);
		// Draw Circle
		cv::circle(m_projectedMask, m_nodes[i]->pixel, radius, cv::Scalar(0), -1);
		// Collect pixels
		for (int j = 0; j < numPixel; j++){
			if (pm[j] < 100){
				m_nodes[i]->projectedPixels.push_back(j);
			}
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
