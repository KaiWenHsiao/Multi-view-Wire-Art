#pragma once

#include "ImageObject.h"

class Source
{
public:
	Source(){}
	virtual ~Source(){}

public:

	static const int MAX_ACCEPTED_IMAGE = 8;

	int m_width;
	int m_height;
	int m_length;
	int m_total;
	float m_voxelSize;

	// 170811, Voxel structure's center
	glm::vec3 m_structureCenter;

	struct Voxel{
		// Record intersection testing rsult with bit for each images
		unsigned char satisfied;
		
		// Various Label
		unsigned char selected;		
	};

	// Label with "selected"
	static const int SELECTED_BIT = 0;
	static const int INQUEUE_BIT = 1;
	static const int BECULLED_BIT = 2;
	static const int ENHANCE_BIT = 3;
	static const int LINKED_BIT = 4;
	void setLabel(int target, int type, bool value){
		if (value){
			m_sculpture[target].selected |= 1 << type;
		}
		else{
			m_sculpture[target].selected &= ~(1 << type);
		}
	}
	bool getLabel(int target, int type){
		unsigned char judge = (m_sculpture[target].selected >> type) & 1;
		if (judge == 1)
			return true;
		return false;
	}

	Voxel *m_sculpture;

	void getCenter(float *center, int voxelIndex){
		int y = voxelIndex / (m_length * m_width);
		int z = (voxelIndex - y * m_length * m_width) / m_width;
		int x = voxelIndex - y * m_length * m_width - z * m_width;

		center[0] = (x - m_width * 0.5) * m_voxelSize + m_structureCenter.x;
		center[1] = (y - m_height * 0.5) * m_voxelSize + m_structureCenter.y;
		center[2] = (z - m_length * 0.5) * m_voxelSize + m_structureCenter.z;
	}
	void getCenter(float *center, int x, int y, int z){
		center[0] = (x - m_width * 0.5) * m_voxelSize + m_structureCenter.x;
		center[1] = (y - m_height * 0.5) * m_voxelSize + m_structureCenter.y;
		center[2] = (z - m_length * 0.5) * m_voxelSize + m_structureCenter.z;
	}
	int getIndex(int x, int y, int z){
		return y*m_length * m_width + z*m_width + x;
	}
	void getIndexCoord(int *ic, int voxelIndex){
		int y = voxelIndex / (m_length * m_width);
		int z = (voxelIndex - y * m_length * m_width) / m_width;
		int x = voxelIndex - y * m_length * m_width - z * m_width;

		ic[0] = x;
		ic[1] = y;
		ic[2] = z;
	}

	// Tell the "Voxel Initialize" the target to deal with
	std::vector<ImageObject*> m_ioList;
	ImageObject *m_currentIO;
};

