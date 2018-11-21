#pragma once

#include <list>
#include <queue>
#include <functional>

#include "Source.h"
#include "VoxelConnectedComponent.h"
#include "VoxelConnector.h"

#include "ContourArt.h"

#include "ProgressTestSender.h"

class VoxelSelector
{
private:
	Source *m_source;

private:
	void getNeighbors(int index, int *numNeighbor, int *neighbors);
	void getSixNeighbors(int index, int *neighbors, int *neighborCount);

public:
	VoxelSelector(Source *s);
	virtual ~VoxelSelector();	

	void initVoxelLabel();
	void selectBestFitVoxels(int markedLabel);
	void selectPairwiseConsistentVoxels(int markedLabel, bool exactly);
	// Remain who have most voxels
	void removeNoise(VoxelConnectedComponent::NodeParent **nodes, const std::vector<VoxelConnectedComponent::Group*> &groups);
	// Remove whose voxel is less than threshold
	void removeNoise(VoxelConnectedComponent::NodeParent **nodes, const std::vector<VoxelConnectedComponent::Group*> &groups, int threshold, int label);
	void updateAfterFirstSelect(int radius);
	void dilateStructure();
	void dilateStructure(int label, int aware);

// 171024
public:
	void fillInner();	

private:
	const int m_voxelCoverRadius = 2;

private:
	float m_3dGuassianKernel[13][13][13];
	float *m_density;
	void getGaussianKernel();
	void distribute(int centerVoxel);

public:
	void repairWithRay_180101(ContourArt *ca);
	


};

