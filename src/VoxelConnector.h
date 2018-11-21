#pragma once

#include <vector>
#include <list>
#include <queue>
#include <functional>

#include <iostream>


#include "VoxelConnectedComponent.h"
#include "ContourArt.h"

#include "ProgressTestSender.h"

#include "MyDataStructure.h"

class VoxelConnector 
{
private:
	Source *m_source;
	VoxelConnectedComponent *m_vcc;
	ContourArt *m_ca;
	

public:
	struct Node{
		struct Link{
			Node *self;
			int selfLinkedVoxelIndex;
			Node *linked;
			int linkedLinkedVoxelIndex;
			float weight;
			std::vector<int> *pathVoxels;
			// 0: Fixed, 1: Disabled
			unsigned char linkType;
		};
		std::vector<Link> links;
		VoxelConnectedComponent::Group *relativeGroup;
		int groupId;
		glm::vec3 center;
		std::vector<int> contourVoxels;
		unsigned char haveBeenConnected;
		unsigned char haveBeenMeet;
		unsigned char subsetFlag;
		std::vector<glm::vec3> projectedCenter;
		// 0: Root, 1: Internal, 2: Leaf
		unsigned char treeNodeType;		
		std::vector<PixelSet*> patchPixels;
		PixelSet *componentVolume;
	};
	
	std::vector<Node::Link> m_result;	

private:
	friend bool operator>(const Node::Link &lhs, const Node::Link &rhs){
		return lhs.weight > rhs.weight;
	}
	friend bool operator<(const Node::Link &lhs, const Node::Link &rhs){
		return lhs.weight < rhs.weight;
	}
	//std::priority_queue<Node::Link, std::vector<Node::Link>, std::greater<Node::Link>> m_pqueue;

	bool checkNotBeLinkeds(std::list<Node*> &nodeList);

	void get26Neighbors(int index, std::vector<int> &neighbors);
	void collectContourPixel(Node *n);
	glm::vec3 calculateGroupCenter(const VoxelConnectedComponent::Group *g);
	

public:
	std::vector<Node*> m_nodes;

public:
	VoxelConnector(Source *s, VoxelConnectedComponent *vcc, ContourArt *ca, int radius);
	VoxelConnector(Source *s);
	virtual ~VoxelConnector();

	void connect();

	


private:
	void getProjectedComponentCenter(std::vector<Node*> &nodes, ImageObject *io);
	void collectProjectedPatchPixel(Node *n);
public:
	glm::vec3 getProjectedComponentCenter(const VoxelConnectedComponent::Group *g, ImageObject *io);

///////////////////////////////////////////////////////////////
public:
	void getSixNeighbors(int voxel, std::vector<int> &neighbors);

private:
	
	struct PathNode{
		int parent;
		int indexInArray;
		int voxelIndex;
		float f;
		float g;
		float h;			
	};
	friend bool operator>(const PathNode &lhs, const PathNode &rhs){
		return lhs.f > rhs.f;
		//return lhs.d > rhs.d;
	}
	friend bool operator<(const PathNode &lhs, const PathNode &rhs){
		return lhs.f < rhs.f;
		//return lhs.d < rhs.d;
	}
	std::vector<PathNode> m_pathNodes;
	float getPathNodePriority(const PathNode &pn, const glm::vec3 &terminalComponentCenter, const float length, bool *secondPriority);
	
	float calculatePathWeight(std::vector<int> *pathVoxels);

///////////////////////////////////////////////////////////////
public:
	std::vector< std::vector<int>* > m_pathVoxelLibrarys;	
	int m_numCannotLink = 0;
private:
	unsigned char **m_linkAdjMat;

///////////////////////////////////////////////////////////////

	
private:
	Ray *m_ray;

///////////////////////////////////////////////////////////////
public:
	static const int ASTAR_WITH_CONSTRAINT_VOLUME = 0;
	static const int EFFICIENT_ASTAR = 1;
	void setupGraph_171201(const std::vector<VoxelConnectedComponent::Group*> &groupps, int astarFunc, const std::string &logFileName);



private:
	void astarWithConstraintVolume(Node *n0, Node *n1, int startVoxelIndex, int goalVoxelIndex);
	bool markConstraintVolume(Node *n0, Node *n1, int *markingLabel);
	void efficientAStar(Node *n0, Node *n1, int startVoxelIndex, int goalVoxelIndex);

	void setupProjectedComponentCenter();
	float calculateShortestDistanceWithContourVoxel(Node *n0, Node *n1, int *v0, int *v1);
	
	unsigned char **m_resAdjMat;	

public:
	float m_createEdgeThreshold;
	int m_processedNodeThreshold;

private:
	void getValuableSixCandidate(const PathNode &current, int *quadrantMask, std::vector<int> &neighbors);
	float getProjectionError(int voxelIndex);
	float getHeuristic(const PathNode &pn, const glm::vec3 &terminalComponentCenter, const float length);
	

	// -1, not be calculated, -5, be culled 
	float *m_voxelDTTable;
	// [a][b] a's closet voxel to b
	// [b][a] b's closet voxel to a
	int **m_nodeClosetIndexMatrix;
	float **m_shortestDistanceMatrix;



	const int RADIUS;



};

