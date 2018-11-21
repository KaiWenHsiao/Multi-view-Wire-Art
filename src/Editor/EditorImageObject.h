#pragma once

#include "../ImageObject.h"
#include "EditorLineData.h"

class EditorImageObject
{
private:
	ImageObject *m_imageObject;
public:
	EditorImageObject(ImageObject *io);
	virtual ~EditorImageObject();

private:
	cv::Mat m_lineCoverCounterMap;
	cv::Mat m_structureProjectionMap;
	cv::Mat m_currentDistanceTransformMap;

	int m_totalPixels;

public:
	void getSingleIndexCoverPixel(float *seg, const int ptNum, std::vector<int> &singleIndexPixels, int width);

	void renderToLineCoverCounterMap(const std::vector<EditorLineData*> &lines, int pixelSetIndex);
	void removeLineFromLCCM(PixelSet *ips);
	void getSimilarityVerSimplify(PixelSet *exceptLine, float *similarity, float similarityTol, float *complexity, float complexityTol);

	void addLineToLCCM(PixelSet *ips);
	float getSimilarityVerRepair(PixelSet *addedpixels, float tol);
	int getNumNotBeCoveredPixel(PixelSet *ps);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
	void getSimilarityAndComplexity(float *s, float *c, std::vector<EditorLineData*> &removedLines, int pixelSetIndex, float similarityTol, float complexityTol);
	void outputLineCoverCounterMap();

private:
	cv::Mat m_tmpLCCM;
	cv::Mat m_tmpSPM;
	std::vector<int> m_originContourPixel;
	std::vector<int> m_origStructureContourPixel;

public:
	float getAndOutputSimilarityMap(const std::vector<float*> &segs, const std::vector<int> &ptNums, const std::string &fileName, int width, int height);
	void precisionRecall(const std::vector<float*> &segs, const std::vector<int> &ptNums, int *datas, float threshold);
	void getSimilarities(const std::vector<float*> &segs, const std::vector<int> &ptNums, std::vector<float> &similarities);

	float getSimilarities(float *voxelVertices, int numVoxel, int radius);

private:
	void getMap(cv::Mat &map, const std::vector<float*> &segs, const std::vector<int> &ptNums);
};

