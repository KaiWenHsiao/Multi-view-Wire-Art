#include "VoxelRenderer.h"


VoxelRenderer::VoxelRenderer(Source *s) : m_source(s)
{
	float table[] = {
		0, 0, 0,
		1, -1, -1,
		1, -1, 1,
		-1, -1, 1,
		-1, -1, -1,
		1, 1, -1,
		1, 1, 1,
		-1, 1, 1,
		-1, 1, -1
	};

	for (int i = 0; i < 36; i++){
		m_vertexTable[i] = table[i];
	}
}


VoxelRenderer::~VoxelRenderer()
{
}
int VoxelRenderer::voxelGeometry(float *vertexData, float *normalData, int labelType){
	int iterator = 0;

	for (int i = 0; i < m_source->m_height; i++){
		for (int j = 0; j < m_source->m_length; j++){
			for (int k = 0; k < m_source->m_width; k++){
				int currentVoxel = m_source->getIndex(k, i, j);
				if (m_source->getLabel(currentVoxel, labelType)){
					// -z face
					if (j <= 0 || (!m_source->getLabel(currentVoxel - m_source->m_width, labelType))){
						this->zFace(currentVoxel, vertexData + iterator, normalData + iterator, -1);
						iterator += 18;
					}
					// +z face
					if (j >= m_source->m_length - 1 || (!m_source->getLabel(currentVoxel + m_source->m_width, labelType))){
						this->zFace(currentVoxel, vertexData + iterator, normalData + iterator, 1);
						iterator += 18;
					}
					// -x face
					if (k <= 0 || (!m_source->getLabel(currentVoxel - 1, labelType))){
						this->xFace(currentVoxel, vertexData + iterator, normalData + iterator, -1);
						iterator += 18;
					}
					// +x face
					if (k >= m_source->m_width - 1 || (!m_source->getLabel(currentVoxel + 1, labelType))){
						this->xFace(currentVoxel, vertexData + iterator, normalData + iterator, 1);
						iterator += 18;
					}
					// -y face
					if (i <= 0 || (!m_source->getLabel(currentVoxel - m_source->m_width * m_source->m_length, labelType))){
						this->yFace(currentVoxel, vertexData + iterator, normalData + iterator, -1);
						iterator += 18;
					}
					// +y face
					if (i >= m_source->m_height - 1 || (!m_source->getLabel(currentVoxel + m_source->m_width * m_source->m_length, labelType))){
						this->yFace(currentVoxel, vertexData + iterator, normalData + iterator, 1);
						iterator += 18;
					}
				}				
			}
		}
	}

	// Return Number of Vertices ;
	return iterator / 3;
}
void VoxelRenderer::zFace(int source, float *vertexData, float *normalData, int sign){
	float center[3];
	m_source->getCenter(center, source);
	float voxelSize = m_source->m_voxelSize;

	int index[] = { 
		// Sign
		6, 7, 3, 2, 6, 3,
		// Unsign
		1, 4, 8, 5, 1, 8};
	int *indexPtr = index;

	if (sign < 0){
		this->createMesh(center, vertexData, normalData, index + 6);
	}
	else{
		this->createMesh(center, vertexData, normalData, index + 0);
	}	

	// set normal
	for (int n = 0; n < 6; n++){
		normalData[n * 3 + 0] = 0;
		normalData[n * 3 + 1] = 0;
		normalData[n * 3 + 2] = 1;
	}
}
void VoxelRenderer::xFace(int source, float *vertexData, float *normalData, int sign){
	float center[3];
	m_source->getCenter(center, source);
	float voxelSize = m_source->m_voxelSize;

	
	int index[] = {
		// Sign
		5, 6, 2, 1, 5, 2,
		// Unsign
		3, 7, 8, 4, 3, 8 };
	int *indexPtr = index;

	if (sign < 0){
		this->createMesh(center, vertexData, normalData, index + 6);
	}
	else{
		this->createMesh(center, vertexData, normalData, index + 0);
	}
	

	// set normal
	for (int n = 0; n < 6; n++){
		normalData[n * 3 + 0] =	1;
		normalData[n * 3 + 1] = 0;
		normalData[n * 3 + 2] = 0;
	}
}
void VoxelRenderer::yFace(int source, float *vertexData, float *normalData, int sign){
	float center[3];
	m_source->getCenter(center, source);
	float voxelSize = m_source->m_voxelSize;

	int index[] = {
		// Sign
		8, 7, 6, 5, 8, 6,
		// Unsign
		2, 3, 4, 1, 2, 4 };
	int *indexPtr = index;

	if (sign < 0){
		this->createMesh(center, vertexData, normalData, index + 6);
	}
	else{
		this->createMesh(center, vertexData, normalData, index + 0);
	}

	// set normal
	for (int n = 0; n < 6; n++){
		normalData[n * 3 + 0] = 0;
		normalData[n * 3 + 1] = 1;
		normalData[n * 3 + 2] = 0;
	}
}
void VoxelRenderer::createMesh(float *center, float *vertexData, float *normalData, int *index){
	float voxelSize = m_source->m_voxelSize;

	vertexData[0] = center[0] + m_vertexTable[index[0] * 3 + 0] * voxelSize * 0.5;
	vertexData[1] = center[1] + m_vertexTable[index[0] * 3 + 1] * voxelSize * 0.5;
	vertexData[2] = center[2] + m_vertexTable[index[0] * 3 + 2] * voxelSize * 0.5;

	vertexData[3] = center[0] + m_vertexTable[index[1] * 3 + 0] * voxelSize * 0.5;
	vertexData[4] = center[1] + m_vertexTable[index[1] * 3 + 1] * voxelSize * 0.5;
	vertexData[5] = center[2] + m_vertexTable[index[1] * 3 + 2] * voxelSize * 0.5;

	vertexData[6] = center[0] + m_vertexTable[index[2] * 3 + 0] * voxelSize * 0.5;
	vertexData[7] = center[1] + m_vertexTable[index[2] * 3 + 1] * voxelSize * 0.5;
	vertexData[8] = center[2] + m_vertexTable[index[2] * 3 + 2] * voxelSize * 0.5;

	vertexData[9] = center[0] + m_vertexTable[index[3] * 3 + 0] * voxelSize * 0.5;
	vertexData[10] = center[1] + m_vertexTable[index[3] * 3 + 1] * voxelSize * 0.5;
	vertexData[11] = center[2] + m_vertexTable[index[3] * 3 + 2] * voxelSize * 0.5;

	vertexData[12] = center[0] + m_vertexTable[index[4] * 3 + 0] * voxelSize * 0.5;
	vertexData[13] = center[1] + m_vertexTable[index[4] * 3 + 1] * voxelSize * 0.5;
	vertexData[14] = center[2] + m_vertexTable[index[4] * 3 + 2] * voxelSize * 0.5;

	vertexData[15] = center[0] + m_vertexTable[index[5] * 3 + 0] * voxelSize * 0.5;
	vertexData[16] = center[1] + m_vertexTable[index[5] * 3 + 1] * voxelSize * 0.5; 
	vertexData[17] = center[2] + m_vertexTable[index[5] * 3 + 2] * voxelSize * 0.5;
}
