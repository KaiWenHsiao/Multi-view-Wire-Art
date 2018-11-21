#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <stack>
#include <fstream>
#include <string>

#include "ProgressTestSender.h"


class LineImprover
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
	LineImprover();
	virtual ~LineImprover();	

	static void copyLine(std::vector<float*> &dstSegs, std::vector<int> &dstPtNum, const std::vector<float*> &srcSegs, const std::vector<int> &srcPtNum);
	static void connect(std::vector<float*> &segs, std::vector<int> &ptNums);

	void correctStructure(const std::vector<float*> &origSegs, const std::vector<int> &origPtNums, std::vector<float*> &dstSegs, std::vector<int> &dstPtNums);
	
private:
	std::vector<std::vector<int>*> m_sids;
	
	bool isJoint(float *v, const std::vector<float*> &lineSegments, const std::vector<int> &linePtNums);
	void splitLines(const std::vector<float*> &lineSegments, const std::vector<int> &linePtNums);
	void createNewStructure(const std::vector<float*> &origLineSegments, const std::vector<int> &origLinePtNum, std::vector<float*> &newLineSegments, std::vector<int> &newLinePtNum);

public:
	void retopology(std::vector<float*> &lineSegments, std::vector<int> &linePtNum, int lengthThreshold, bool carefulSelfCycle);
private:
	bool merge(int line0, int line1, std::vector<float*> &lineSegments, std::vector<int> &linePtNum, bool carefulSelfCycle);
	int isOneConnectTerminalVertex(const std::vector<float*> &lineSegments, const std::vector<int> &linePtNum, float *v, int self);
	
	void setToDataBuffer(float *first, int firstVertexCount, bool firstInverse, float *second, int secondVertexCount, bool secondInverse, float *dataBuffer, int fixedOffset);

public:
	static bool equal(float *v0, float *v1);
	static void removeCycle(const std::vector<float*> &segments, std::vector<int> &ptNums);

	static void smoothLine(const std::vector<float*> &segments, const std::vector<int> &ptNum, std::vector<float*> &smoothSegments, std::vector<int> &smoothPtNum, int iteration);
};

