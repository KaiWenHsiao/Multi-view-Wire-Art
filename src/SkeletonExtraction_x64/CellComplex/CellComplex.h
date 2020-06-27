#pragma once
#include"config.h"
#include"Element.h"
#include "score2surface.h"
#include <climits>

#include <ctime>
#include <stack>
#include <map>

#define HASGEOMETRY		//if has geometry, read in data and save it

class scoreReflector;

struct Line
{
	float pts[6];
	int iso;
};

class CellComplex
{
public:
	CellComplex();
	~CellComplex();
	void read(const char*);
	void readCC_skl_significance(const char* fname);
	void construct();
	void construct(const vector<float>&,
		const vector<int>&,
		const vector<int>&, const vector<int>&,
		const vector<int>&, const vector<int>&,
		const vector<int>&);
	void setParents(int);
	void getEdgePts(vector<float*>&);
	void extractPointStructure(vector<float*>&, vector<float*>&, vector<float*>&, vector<vector<float*> >&);
	void extractPointStructure_iso(vector<Line>&);
	void extractLine(vector<float*>&, vector<int>&);
	void test();
	void setL(int resol) { L = resol;  L /= 128; }
	float* getPointFromIndex(int);
	void clear();
	void thin2Pass_Cmp(float, float);
	void initSign_setExist_clearOthers();
	void sigMeasureFP();
	void restoreExist_HighScore();
	void componentAnalysis(const int);
	void setLockSign_RestoreExist();
	int removalFP_PreserveLock();
	void copy_sign_2_bexist();
	void setMaxDdS1();
	void initSign();
	void computeFaceNorm();
	void measureVoxelLen();
	void setDistDiffThresh(float, float);
	void setComponentThresh(int, int);
	void writeCellComplexV0(const char*);
	void writeCellComplexV0_Exist(const char*);
	void writeCellComplexV0MM(const char*);


	void setTypes();
	void setFacePatchInd();
	void setEdgeTypes();
	void setPtsTypes();
	void setEdgeCurveInd();

	void rescaling(float);

	scoreReflector *m_scoreRefPtr;

	int patchNum;
	int curveNum;

	int *elementSize;
	Element ***elements;
	float *ptPos;
	float L;
	int* facePtInds;
	float* faceNorm;

	int componentThresh[3];
	int distDiffThresh[3];
	float speedThreshThin[3];
	
	int maxIterNum;
	int maxDist[3];
	int maxD[3];
	int maxd[3];
	float maxS1[3];
	int m_nMaxDim;
	float m_fUnitlen;
	float m_aCenter[3];

	float minMaxCorner[3][2];
	int m_aVoxelLen[3];
	int m_nMaxDir = 0;

	int isoThresh;
	float speedThresh;
	int distThresh;
	int scaleThresh;
	void setIsoThresh(int iso){ isoThresh = iso; }
	void setSpeedThresh(float speed) { speedThresh = speed; }
	void setDistThresh(int thresh){ distThresh = thresh; }
	void setScaleThresh(int thresh){ scaleThresh = thresh; }

	int* parentsNum[3];

	enum EdgeType { NAEdge = 0, CurveEdge, BoundaryEdge, InteriorEdge, NMEdge };
	enum PointType {
		NAPt = 0, JunctionPt, NMPoint, PatchBoundaryPoint, PatchInteriorPoint, CurveInteriorPoint
	};

private:

};

