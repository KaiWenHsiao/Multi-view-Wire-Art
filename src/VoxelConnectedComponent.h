#pragma once

// With Disjoint-Set, Union-Find Algorithm

#include <iostream>
#include <vector>
#include "Source.h"

class VoxelConnectedComponent
{
public:
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
	NodeParent **m_nodes;
	std::vector<NodeParent*> m_parents;
	std::vector<Group*> m_groups;

	std::string m_log;

private:
	Source *m_source;
		
	void setUpGroups();
public:
	VoxelConnectedComponent(Source *source);
	virtual ~VoxelConnectedComponent();

	static const int NEIGHBORS_26 = 0;
	static const int NEIGHBORS_6 = 1;
	void labeling(int neighborType, int label);
	
	void getNeighborLabels(int cx, int cy, int cz, std::vector<NodeParent*> &neighborLabels);
	void getSixNeighborLabels(int cx, int cy, int cz, std::vector<NodeParent*> &neighborLabels);

	
	
};

