#pragma once

#include <string>

#include <time.h>

#include "Source.h"
#include "ContourArt.h"
#include "VoxelSelector.h"
#include "VoxelConnectedComponent.h"
#include "VoxelConnector.h"
#include "VoxelRenderer.h"

#include "LineWidget.h"
#include "LineImprover.h"
#include "LineImprover4.h"
#include "LineDeformer.h"

#include "CVImageObject.h"

#include "Solidificator.h"

class MyPipeline 
{
public:
	MyPipeline();
	virtual ~MyPipeline();

	Source *m_source;	

	bool ready(const std::vector<std::string> &images);
	
	bool createImageObject(const std::string &file, float radius, float phi);
	bool createImageObject(const std::string &file, const std::string &weightMap, float radius, float phi);
	bool createImageObject(const std::string &file, float radius, float phi, float theta, const glm::vec3 &center, const glm::vec3 &upVector);

private:
	void sentLog(const std::string &log);
	void outputVoxelLabel(const std::string &fileName);


	
// Special case of three view
private:
	bool createThreeViewOrthoSet(const std::vector<std::string> &images);
	void setUpOrthoProjection(ImageObject *io, float radius, float height, float width, float phi, float theta, const glm::vec3 &upVector);

	bool createThreeViewPerspectiveSet(const std::vector<std::string> &images);
	void setUpPerspectiveProjection(ImageObject *io, float radius, float phi, float theta, const glm::vec3 &upVector);

// Buffer Size determine by user
public:
	int m_selectedMaxNumVertex;

// Link various 3D Connected Component


/////////////////////////////////////////////////////////////////////////////////////////////////
public:
	bool m_requireImageGraph;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sequentially
private:
	ContourArt *m_multiRes[4];	

public:
	bool sequentiallyMultiResolutionSculpture(const std::vector<std::string> &images, const std::vector<std::string> &fileNames, int resolution);
	void createVariousResolutoinSculpture(const std::string &fileNames, int res);

	bool sequentiallyProcessMultiSS(const std::vector<std::string> &images, const std::vector<std::string> &sss, const std::vector<std::string> &fileNames, int function);
	void simplify(std::vector<float*> &segs, const std::vector<int> &ptNums, const std::string &fileName);
private:
	ImageObject *renewImageObject(int radius);



	

/////////////////////////////////////////////////////////
public:
	void optimize(std::vector<float*> &segs, const std::vector<int> &ptNums, const std::string &fileName);
	void setUpVolume(const std::string &outputVolumeFile);
	void solidficate(const std::string &file, const std::string &outputFileName);
private:
	void solidification(const std::vector<float*> &segs, const std::vector<int> &ptNums, const std::vector<float> &firstLastLock, bool splitJointAndTube, const std::string &outputFileName);


	// Volume
private:
	struct Volume{
		int* data;
		int sizexyz[3];
		double spacexyz[3];
		double cornerPos[3];
		int threshold;
	};
	Volume m_volumeFormat;

public:
	void outputVoxelStructureOBJ(const std::string &vlFile, const std::string &outputFileName);

	static void scriptForSkeletonExtraction(const std::string &filename);
	static void scriptForCurveProcess(const std::vector<std::string> &imgs, const std::string &filename);
};

