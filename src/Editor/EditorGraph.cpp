#include "EditorGraph.h"



EditorGraph::EditorGraph(int numVertex)
{
	m_v = numVertex;
	m_visited = new bool[numVertex];
	m_disc = new int[numVertex];
	m_low = new int[numVertex];
	m_parent = new int[numVertex]; 
	m_ap = new bool[numVertex];	
}


EditorGraph::~EditorGraph()
{
	delete[] m_visited;
	delete[] m_disc;
	delete[] m_low;
	delete[] m_parent;
	delete[] m_ap;
}

// A recursive function that find articulation points using DFS traversal
// u --> The vertex to be visited next
// visited[] --> keeps tract of visited vertices
// disc[] --> Stores discovery times of visited vertices
// parent[] --> Stores parent vertices in DFS tree
// ap[] --> Store articulation points
void EditorGraph::APUtil(int v, unsigned char **adjMat, const std::vector<EditorLineData*> &lineDatas)
{
	// Count of children in DFS Tree
	int children = 0;

	// Mark the current node as visited
	m_visited[v] = true;

	// Initialize discovery time and low value
	m_disc[v] = m_low[v] = ++m_time;

	// Go through all vertices aadjacent to this
	for (int i = 0; i < m_v; i++){
		if (adjMat[v][i] != 0 && lineDatas[i]->m_enabledFlag == 1){

			if (!m_visited[i]){
				children++;
				m_parent[i] = v;
				this->APUtil(i, adjMat, lineDatas);

				m_low[v] = min(m_low[v], m_low[i]);
				// (1) u is root of DFS tree and has two or more chilren.
				if (m_parent[v] == -1 && children > 1)
					lineDatas[v]->m_articulationFlag = 1;

				// (2) If u is not root and low value of one of its child is more
				// than discovery value of u.
				if (m_parent[v] != -1 && m_low[i] >= m_disc[v])
					lineDatas[v]->m_articulationFlag = 1;
			}

			// Update low value of u for parent function calls.
			else if (i != m_parent[v])
				m_low[v] = min(m_low[v], m_disc[i]);
		}
	}	
}

// The function to do DFS traversal. It uses recursive function APUtil()
void EditorGraph::AP(unsigned char **adjMat, const std::vector<EditorLineData*> &lineDatas)
{
	m_time = 0;

	// Initialize parent and visited, and ap(articulation point) arrays
	for (int i = 0; i < m_v; i++)
	{
		m_parent[i] = -1;
		m_visited[i] = false;
		m_ap[i] = false;

		m_disc[i] = 0;
		m_low[i] = 0;
	}

	// Call the recursive helper function to find articulation points
	// in DFS tree rooted with vertex 'i'
	for (int i = 0; i < m_v; i++){
		if (lineDatas[i]->m_enabledFlag == 1){
			if (m_visited[i] == false){
				this->APUtil(i, adjMat, lineDatas);
			}
		}
	}	
}