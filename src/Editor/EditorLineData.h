#pragma once

#include <vector>
#include <glm\vec2.hpp>

#include "../MyDataStructure.h"

typedef std::vector<glm::ivec2> ProjectedPixels;

class EditorLineData
{
public:
	EditorLineData();
	virtual ~EditorLineData();

public:
	float *geometry;
	int numPt;

	float priority;

	// Index Pointer
	int ptStart;
	int ptEnd;

	std::vector<ProjectedPixels*> m_imageProjectedPixels;

	bool isTarget(int dataSetIdx, unsigned char **flagTable);

	unsigned char m_checkedFlag;
	unsigned char m_targetFlag;
	unsigned char m_enabledFlag;
	unsigned char m_articulationFlag;

	int m_correspondingMaterialIndex;

	std::vector<PixelSet*> m_originImageProjectedPixels;

};

