#pragma once

#include <deque>
#include <ctime>

#include "Source.h"
#include "Editor/EditorLineData.h"
#include "Editor/EditorImageObject.h"
#include "Editor/EditorGraph.h"
#include "LineWidget.h"
#include "ProgressTestSender.h"


class LineImprover4
{
private:
	std::vector<EditorLineData*> m_lineDatas;
	std::vector<EditorImageObject*> m_eios;
	EditorGraph *m_graph;

	struct StructureContent{
		unsigned char *lineFlags;
		int numLine;
		int numLineOfCorrectTopology;

		std::vector<EditorLineData*> removedLines;
		int* parents;

		float eNumOfLine;
		float eComplexity;
		float eSimilarity;
		float energy;

		unsigned char enabled;
	};


public:
	LineImprover4(Source *s);
	virtual ~LineImprover4();

public:
	void selectBest_1222(std::vector<float*> &newLineSegments, std::vector<int> &newLinePtNum, const std::string &processedSSFileName, bool outputProcessedSS);
	
private:
	std::vector<StructureContent*> m_subStructureLibrary;

	unsigned char **m_adjacencyMatrix;
	int m_adjacencyMatrixSize;
	void setUpLines(const std::vector<float*> &segments, const std::vector<int> &ptNums);
	void setupAdjacencyMatrix(const std::vector<float*> &segments, const std::vector<int> &ptNums);

	int directlyRemoveTooShortLeaf(StructureContent *s, int threshold);
	int isLeaf(int index, unsigned char *lineFlags);
	void changeDataStructure(StructureContent *s, std::vector<float*> &segs, std::vector<int> &ptNums);

	StructureContent *getBestSubStructure(StructureContent *current);
	StructureContent *getBestSubStructure_parallel(StructureContent *current);
	StructureContent *getCopy(StructureContent *src);

	void calculateStructureEnergy(StructureContent *sc, int exceptUnit);

	int E_MAX_BRANCH = 0;

	void markEnabledLine(StructureContent *sc);
	void markArticulations();

	std::vector<float> m_energyLog;

	void outputProcessedSS(StructureContent *s, const std::string &fileName);

	struct Neighbors{
		std::vector<int> neighborIdx;
	};
	struct UnitLine{
		std::vector<int> lineIndices;
	};
	
	std::vector<UnitLine*> m_unitLines;
	int* m_parentCounter;
	int getCorrectStructureNumSeg(StructureContent *s, int exceptUnit);

	void collectUnitLines(StructureContent *s);

	void updateParent(StructureContent *s);
	
	void initParent(StructureContent *s);

	void mergeUnitLine(StructureContent *s, std::vector<float*> &segs, std::vector<int> &ptNums);
	bool mergeOne(float **srcSeg, int *stcPtNum, std::vector<EditorLineData*> &elds);
	void setToDataBuffer(float *first, int firstVertexCount, bool firstInverse, float *second, int secondVertexCount, bool secondInverse, float *dataBuffer);
	
	
	bool isSmallCycle(const std::vector<float*> &srcSegs, const std::vector<int> &srcPtNums, int s0, int s1);

public:
	void removeSmallCycle(std::vector<float*> &resSegs, std::vector<int> &resPtNums, const std::vector<float*> &srcSegs, const std::vector<int> &srcPtNums);
	void retopo(std::vector<float*> &srcSegs, std::vector<int> &srcPtNums);

private:
	unsigned char **m_unitLineAdjMat;
	int m_unitLineAdjMatSize;
	void setUpUnitLineAdjacencyMatrix();
	std::vector<EditorLineData*> m_unitELDs;
	
	void markUnitLineArticulations(StructureContent *sc);
	bool isLeafUnitLine(UnitLine *ul, StructureContent *s);

public:
	void directlyRemoveUnLinked(StructureContent *sc);

private:
	void updateParent(StructureContent *s, int removedUnitLine);



};

