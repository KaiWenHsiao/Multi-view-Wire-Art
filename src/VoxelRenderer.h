#pragma once

#include "Source.h"

class VoxelRenderer
{
private:
	Source *m_source;

	void zFace(int source, float *vertexData, float *normalData, int sign);
	void xFace(int source, float *vertexData, float *normalData, int sign);
	void yFace(int source, float *vertexData, float *normalData, int sign);

	void createMesh(float *center, float *vertexData, float *normalData, int *index);
public:
	VoxelRenderer(Source *s);
	virtual ~VoxelRenderer();

	int voxelGeometry(float *vertexData, float *normalData, int labelType);

private:
	float m_vertexTable[36];
};

