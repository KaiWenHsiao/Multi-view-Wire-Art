#pragma once

#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm\gtx\transform.hpp>
#include <glm\gtc\quaternion.hpp>
#include <glm\vec4.hpp>
#include <glm\mat4x4.hpp>
#include <glm\gtc\type_ptr.hpp>

#include <fstream>

class Solidificator
{
public:
	Solidificator();
	virtual ~Solidificator();

	static void solidification(const std::vector<float*> &segments, const std::vector<int> &ptNum, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, float radius);
	static bool tubing(float *v, float *b, float *n, std::vector<float> &cVertex, std::vector<float> &cTangent);
	static unsigned int createCylinder(float *meshVertex, float *meshNormal, unsigned int *meshIndex, unsigned int indexBase, float *vertex, std::vector<float> &tangent, float *normal, int vertexCount, int section, float radius);
	static void addJointToSphere(float *joint, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, int indexOffset, float radius);
	static void addJointGeometry(const std::vector<float> &firstLastLock, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, const int indexOffset, float radius);
	static bool createOBJ(std::string fileName, float *vertex, float *normal, unsigned int *index, int vertexCount, int indexCount);

	static void getSphere(float *joint, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, int indexOffset, float radius);
	static void getCube(float *joint, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, int indexOffset, float size);

	static void createVoxelStructureOBJ(const std::string &fileName, float *meshVertex, float *meshNormal, int numVertex);
	
};

