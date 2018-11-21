#pragma once

#include <fstream>

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include <glm\glm.hpp>

class LineWidget
{
public:
	LineWidget();
	virtual ~LineWidget();

	static unsigned char isNeighbor(const float *seg0, const int ptNum0, const float *seg1, const int ptNum1);	
	static unsigned char linkRelation(const float *seg0, const int ptNum0, const float *seg1, const int ptNum1);
	static void setupAdjacencyMatrix(const std::vector<float*> &segments, const std::vector<int> &ptNums, unsigned char **adjacencyMatrix);

	static void projectLine(float *seg, const int ptNum, const glm::mat4 &viewProj, cv::Mat &mask);
	static void project(glm::ivec2 &pixel, float *vertex, const glm::mat4 &viewProj, int w, int h);

	static bool writeSimpleSkeletonFile(std::string file, const std::vector<float*> &segments, const std::vector<int> &ptNum);
	static bool readSimpleSkeletonFile(std::string file, std::vector<float*> &segments, std::vector<int> &ptNum);	

	static bool isSameLine(float *seg0, int ptNum0, float *seg1, int ptNum1);
	
};

