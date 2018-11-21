#pragma once

#include <vector>
#include <queue>
#include <functional>
#include <stack>

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "../MWA/ANN/include/ANN.h"

#include "MyDataStructure.h"

class ImageGraph
{
public:
	struct PixelNode{
		cv::Point pixel;
		std::vector<PixelNode*> neighbors;
		std::vector<int> projectedPixels;
				
		unsigned char visited;
		unsigned char endPixel;
		unsigned char startPixel;
	};

	// A* algorithm
private:
	struct PathNode{
		int parent;
		PixelNode *p;
		float f;
		float g;
		float h;
		int indexInArray;
	};
	friend bool operator>(const PathNode &lhs, const PathNode &rhs){
		return lhs.f > rhs.f;
	}
	friend bool operator<(const PathNode &lhs, const PathNode &rhs){
		return lhs.f < rhs.f;
	}
	std::vector<PathNode> m_pathNodes;	

private:
	cv::Mat m_projectedMask;
	void collectProjectedPixels(int radius);

public:
	std::vector<PixelNode*> m_nodes;
	PixelNode **m_nodeTable;
private:
	

	ANNkd_tree *m_kdTree;
	ANNpointArray m_dataSet;
	ANNidxArray m_nearNeighborIndex;
	ANNdistArray m_nearNeighborDist;
	ANNpoint m_query;
	
	const int DIM = 2;
	const double EPS = 0.0;

	const int NUM_NEAR_NEIGHBOR = 1;

	void getEightNeighbors(int x, int y, std::vector<int> &neighbors, const int width, const int height);
	float getLengthSquare(PixelNode *n0, PixelNode *n1);

	cv::Mat m_pathMask;
	

	void aStar(PixelNode *start, PixelNode *end, PixelSet *startPatch, PixelSet *endPatch, std::vector<PixelNode*> &pathPixels);
	void aStar(PixelNode *start, PixelNode *end, std::vector<PixelNode*> &pathPixels);

	const int RADIUS;
public:
	ImageGraph(const cv::Mat &thinImg, int radius);
	virtual ~ImageGraph();

	void setUpGraph(const cv::Mat &thinImg);
	void setUpKDTree();

	PixelNode *getNearNeighbor(int x, int y);

	void getPath(const cv::Point &start, const cv::Point &end, PixelSet *ps0, PixelSet *ps1, std::vector<cv::Point> &pathPixels);

/////////////////////////////////////////////////////////////////////////////////////////////////////

	

public:
	bool render2DConstraintRegion(const cv::Point &start, const cv::Point &end, PixelSet *ps0, PixelSet *ps1);
	cv::Mat m_2dConstraintRegionMap;

	bool singleVoxel2DShortestPath(const cv::Point &start, const cv::Point &end, PixelSet *ps0, PixelSet *ps1);


};

