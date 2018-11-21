#include "ContourArt.h"
#include <cmath>

#include <iostream>
ContourArt::ContourArt(int width, int height, int length, float voxelSize, Source *s){
	// Record 
	m_source = s;

	// init 
	m_source->m_width = width;
	m_source->m_height = height;
	m_source->m_length = length; 
	m_source->m_total = width * length * height;
	m_source->m_voxelSize = voxelSize;

	m_source->m_sculpture = new Source::Voxel[m_source->m_total];

	for (int i = 0; i < m_source->m_total; i++)
		m_source->m_sculpture[i].satisfied = 0;

	m_newBitLabel = 1;
	m_newBitPos = 0;
}
ContourArt::ContourArt(){
	// Set up class with .ca file
}
ContourArt::ContourArt(Source *s){
	m_source = s;
}
void ContourArt::reset(){
	m_newBitLabel = 1;
	m_newBitPos = 0;
}
ContourArt::~ContourArt()
{
	for (int i = 0; i < 8; i++){
		destructorRecursive(m_root->child[i]);
		delete m_root->child[i];
	}
	delete m_root;
}
void ContourArt::destructorRecursive(Node *item){
	if (item->child[0] == nullptr){
		delete[] item->elementArray;
		return;
	}

	for (int i = 0; i < 8; i++){
		destructorRecursive(item->child[i]);
		delete item->child[i];
	}
}
int ContourArt::addNewImage(ImageObject *io){
	const std::vector<Ray*> &rays = io->m_rays;
	int numRay = rays.size();
	
	for (int i = 0; i < numRay; i++){
		// Intersection test
		for (int k = 0; k < 8; k++){
			this->rayVoxelTest_recursive(
				glm::value_ptr(rays[i]->m_orig),
				glm::value_ptr(rays[i]->m_dir),
				rays[i],
				m_root->child[k]);
		}
	}


	int currentBitPos = m_newBitPos;
	// Update bit label
	m_newBitLabel = m_newBitLabel << 1;
	// Update bit position
	m_newBitPos = m_newBitPos++;
	// return bit position
	return currentBitPos;
}
void ContourArt::rayVoxelTest_recursive(float *orig, float *dir, Ray *cr, Node *item){
	if (this->slabMethod(orig, dir, item->maxCoord, item->minCoord)){
		// Leaf
		if (item->child[0] == nullptr){
			for (int i = 0; i < item->elementCount; i++){
				// Get center
				float center[3];
				m_source->getCenter(center, item->elementArray[i]);
				// Compute maxCoord & minCoord
				float maxCoord[3], minCoord[3];
				maxCoord[0] = center[0] + m_source->m_voxelSize * 0.5;
				maxCoord[1] = center[1] + m_source->m_voxelSize * 0.5;
				maxCoord[2] = center[2] + m_source->m_voxelSize * 0.5;
				minCoord[0] = center[0] - m_source->m_voxelSize * 0.5;
				minCoord[1] = center[1] - m_source->m_voxelSize * 0.5;
				minCoord[2] = center[2] - m_source->m_voxelSize * 0.5;
				if (this->slabMethod(orig, dir, maxCoord, minCoord)){
					m_source->m_sculpture[item->elementArray[i]].satisfied = 
						m_source->m_sculpture[item->elementArray[i]].satisfied | m_newBitLabel;	
				}
			}
			return;
		}

		// Has child
		for (int i = 0; i < 8; i++){
			this->rayVoxelTest_recursive(orig, dir, cr, item->child[i]);
		}
	}
}
bool ContourArt::slabMethod(float *orig, float *dir, float *maxCoord, float *minCoord){
	// intersection test with "Slab method"
	const float MAX = 1000000.0;
	float tMax = MAX, tMin = -MAX;

	// o + td = x
	// => t = (x - o) / d

	for (int i = 0; i<3; i++){
		// should not be divided by zero
		float t0 = -1000000.0, t1 = 1000000.0;

		if (abs(dir[i]) < 0.001){
			t0 = (minCoord[i] - orig[i]) * MAX;
			t1 = (maxCoord[i] - orig[i]) * MAX;
		}
		else if (dir[i] > 0){
			float inv = 1.0 / dir[i];
			t0 = (minCoord[i] - orig[i]) * inv;
			t1 = (maxCoord[i] - orig[i]) * inv;
		}
		else{
			float inv = 1.0 / dir[i];
			t0 = (maxCoord[i] - orig[i]) * inv;
			t1 = (minCoord[i] - orig[i]) * inv;
		}

		// Find bigger between t0, record in tMin
		if (t0 > tMin){
			tMin = t0;
		}
		// Find smaller between t1, record in tMax
		if (t1 < tMax){
			tMax = t1;
		}
	}

	return tMax >= tMin;
}

////////////////////////////////////////////////////////////////
// Octree
void ContourArt::buildOctree(int maxLayer){
	m_maxLayer = maxLayer;

	m_root = new Node;

	int wCount = m_source->m_width / 2;
	int lCount = m_source->m_length / 2;
	int hCount = m_source->m_height / 2;

	int offset[] = {
		0, 0, 0,
		wCount, 0, 0,
		0, 0, lCount,
		wCount, 0, lCount,

		0, hCount, 0,
		wCount, hCount, 0,
		0, hCount, lCount,
		wCount, hCount, lCount
	};
	int childMin[3];
	for (int i = 0; i < 8; i++){
		childMin[0] = offset[i * 3 + 0];
		childMin[1] = offset[i * 3 + 1];
		childMin[2] = offset[i * 3 + 2];
		m_root->child[i] = buildOctree_recursive(childMin, 1);
	}
}
ContourArt::Node *ContourArt::buildOctree_recursive(int *minIndexCoord, int layer){
	Node *item = new Node;

	// current w, l, h
	int d = (int)pow(2, layer);
	int wCount = m_source->m_width / d;
	int hCount = m_source->m_height / d;
	int lCount = m_source->m_length / d;

	// Set min world coord & max world coord
	float center[3];
	m_source->getCenter(center, minIndexCoord[0], minIndexCoord[1], minIndexCoord[2]);
	item->minCoord[0] = center[0] - 0.5 * m_source->m_voxelSize;
	item->minCoord[1] = center[1] - 0.5 * m_source->m_voxelSize;
	item->minCoord[2] = center[2] - 0.5 * m_source->m_voxelSize;

	m_source->getCenter(center, minIndexCoord[0] + (wCount - 1), minIndexCoord[1] + (hCount - 1), minIndexCoord[2] + (lCount - 1));
	item->maxCoord[0] = center[0] + 0.5 * m_source->m_voxelSize;
	item->maxCoord[1] = center[1] + 0.5 * m_source->m_voxelSize;
	item->maxCoord[2] = center[2] + 0.5 * m_source->m_voxelSize;

	// Leaf
	if (layer == m_maxLayer){
		// Leaf does not have child
		for (int i = 0; i < 8; i++)
			item->child[i] = nullptr;

		// Calculate element count
		item->elementCount = wCount * hCount * lCount;

		// Prepare data buffer
		item->elementArray = new int[item->elementCount];

		// Put data into buffer
		int iter = 0;
		for (int i = 0; i < hCount; i++){
			for (int j = 0; j < lCount; j++){
				for (int k = 0; k < wCount; k++){
					item->elementArray[iter] = m_source->getIndex(minIndexCoord[0] + k, minIndexCoord[1] + i, minIndexCoord[2] + j);
					iter++;
				}
			}
		}
		return item;
	}


	// Has child
	// w, h, l of child
	int cwCount = wCount / 2;
	int chCount = hCount / 2;
	int clCount = lCount / 2;

	int offset[] = {
		0, 0, 0,
		cwCount, 0, 0,
		0, 0, clCount,
		cwCount, 0, clCount,

		0, chCount, 0,
		cwCount, chCount, 0,
		0, chCount, clCount,
		cwCount, chCount, clCount
	};
	int childMin[3];
	for (int i = 0; i < 8; i++){
		childMin[0] = minIndexCoord[0] + offset[i * 3 + 0];
		childMin[1] = minIndexCoord[1] + offset[i * 3 + 1];
		childMin[2] = minIndexCoord[2] + offset[i * 3 + 2];
		item->child[i] = this->buildOctree_recursive(childMin, layer + 1);
	}
	return item;
}
//////////////////////////////////////////////////////////////////
void ContourArt::rayVoxelTest_recursive(float *orig, float *dir, Ray *cr, Node *item, std::vector<int> &intersectedVoxels){
	if (this->slabMethod(orig, dir, item->maxCoord, item->minCoord)){
		// Leaf
		if (item->child[0] == nullptr){
			for (int i = 0; i < item->elementCount; i++){
				// Get center
				float center[3];
				m_source->getCenter(center, item->elementArray[i]);
				// Compute maxCoord & minCoord
				float maxCoord[3], minCoord[3];
				maxCoord[0] = center[0] + m_source->m_voxelSize * 0.5;
				maxCoord[1] = center[1] + m_source->m_voxelSize * 0.5;
				maxCoord[2] = center[2] + m_source->m_voxelSize * 0.5;
				minCoord[0] = center[0] - m_source->m_voxelSize * 0.5;
				minCoord[1] = center[1] - m_source->m_voxelSize * 0.5;
				minCoord[2] = center[2] - m_source->m_voxelSize * 0.5;
				if (this->slabMethod(orig, dir, maxCoord, minCoord)){
					// Record the intersected voxel
					intersectedVoxels.push_back(item->elementArray[i]);					
				}
			}
			return;
		}

		// Has child
		for (int i = 0; i < 8; i++){
			this->rayVoxelTest_recursive(orig, dir, cr, item->child[i], intersectedVoxels);
		}
	}
}
void ContourArt::getIntersectedVoxels(std::vector<int> &intersectedVoxels, Ray *r){
	// Intersection test
	for (int k = 0; k < 8; k++){
		this->rayVoxelTest_recursive(
			glm::value_ptr(r->m_orig),
			glm::value_ptr(r->m_dir),
			r,
			m_root->child[k],
			intersectedVoxels);
	}
}


