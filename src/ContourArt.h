#pragma once

#include <fstream>
#include <string>

#include <glm\gtc\type_ptr.hpp>

#include "Source.h"

class ContourArt 
{
private:
	Source *m_source;
	
	unsigned char m_satisfiedTarget;
	unsigned char m_newBitLabel;
	int m_newBitPos;

	/////////////////////////////////////////////////////////////////////
	// Octree
	struct Node{
		Node *child[8];
		int *elementArray;
		int elementCount;
		float minCoord[3];
		float maxCoord[3];
	};
	Node *m_root;
	int m_maxLayer;
	Node *buildOctree_recursive(int *minIndexCoord, int layer);
	void rayVoxelTest_recursive(float *orig, float *dir, Ray *cr, Node *item);
	bool slabMethod(float *orig, float *dir, float *maxCoord, float *minCoord);
	/////////////////////////////////////////////////////////////////////

public:
	//ContourArt(int width,int height, int length);
	ContourArt(int width, int height, int length, float voxelSize, Source *s);
	ContourArt(Source *s);
	ContourArt();
	~ContourArt();
	void destructorRecursive(Node *item);

	
	// New image
	int addNewImage(ImageObject *IO);

	// Build octree if required
	void buildOctree(int maxLayer);
	
	
	void renderWhole();
	void renderSpecifyCC(int idx);

	void information();	

	void reset();
	////////////////////////////////////////////////////////////////////

public:
	void getIntersectedVoxels(std::vector<int> &intersectedVoxels, Ray *r);
private:
	void rayVoxelTest_recursive(float *orig, float *dir, Ray *cr, Node *item, std::vector<int> &intersectedVoxels);
};

