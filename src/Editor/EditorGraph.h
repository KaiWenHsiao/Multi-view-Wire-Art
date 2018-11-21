#pragma once

#include <iostream>
#include <list>
#include <algorithm>
#include <vector>

#include "EditorLineData.h"

using namespace std;

class EditorGraph
{
private:
	
private:
	void APUtil(int v, unsigned char **adjMat, const std::vector<EditorLineData*> &lineDatas);

	int m_v;
	bool *m_visited;
	int *m_disc;
	int *m_low;
	int *m_parent;
	bool *m_ap;

	int m_time;

public:
	EditorGraph(int numVertex);   // Constructor
	~EditorGraph();

	
	void AP(unsigned char **adjMat, const std::vector<EditorLineData*> &lineDatas);    // prints articulation points

	
};

