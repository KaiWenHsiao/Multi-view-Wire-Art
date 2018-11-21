#pragma once

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include <list>
#include <queue>
#include <vector>
#include <functional>

class PixelConnectedComponent
{
private:
	struct Group{
		int relativeParentId;
		int numNode;
		std::vector<int> m_nodes;
		int indexInArray;
	};

	struct NodeParent{
		int parentId;
		Group *groupPtr;
	};
	int m_imgWidth;
	int m_imgHeight;
	NodeParent **m_nodes;
	std::vector<NodeParent*> m_parents;
	

	void getEightNeighbors(int cx, int cy, std::vector<NodeParent*> &neighborLabels);

public:
	PixelConnectedComponent();
	virtual ~PixelConnectedComponent();

	void labeling(const cv::Mat &image);

	std::vector<Group*> m_groups;

private:
	void setUpGroups();

////////////////////////////////////////////
public:
	struct Node{
		struct Link{
			Node *self;
			Node *linked;
			int selfPixelIndex;
			int linkedPixelIndex;
			float weight;
		};
		unsigned char haveBeenConnected;
		std::vector<Node::Link> links;
		Group *relativeGroup;
	};
	friend bool operator>(const Node::Link &lhs, const Node::Link &rhs){
		return lhs.weight > rhs.weight;
	}
	friend bool operator<(const Node::Link &lhs, const Node::Link &rhs){
		return lhs.weight < rhs.weight;
	}
private:
	void setUpEdge(Node *n0, Node *n1);
	bool checkNotBeLinked(std::list<Node*> &nodeList);
	int getSquaredLength(int *i0, int *i1);

public:
	std::vector<Node*> m_groupNodes;
	std::vector<Node::Link> m_result;
	void setUpGraph(std::vector<Group*> groups);
	void connect();

	void getIndex(int *index, int id);
};

