#pragma once
#include "config.h"
#include "CellComplex.h"

class CellComplex;

struct structExteriorFace
{
	int faceInd;
	float IterNum_ArrivalTime3D;
	float IterNum_ArrivalTime2D;

	//temporary
	int isoFaceInd;
	int medialEdgeInd;

	struct
	{
		unsigned char hasSkeletonFace : 1;	//faces have high scores 
		unsigned char hasSkeletonFace_MedialEdge : 1;
		unsigned char hasSkeletonFace_MedialPt : 1;
		unsigned char hasSkeletonFace2 : 1;	//trace from intermediate edges among the strong faces
		unsigned char hasSkeletonEdge : 1;	//isolated edges with high scores
		unsigned char hasSkeletonEdge2 : 1;	//trace from intermediate points of the strong edges		
		unsigned char hasSkeletonEdge_MedialPt : 1;
		unsigned char others : 1;
	} m_bSign;
};

class scoreReflector
{
	vector<int> m_CCFaceInd2ExteriorFaceInd;
	vector<structExteriorFace> m_vecExteriorFaces;

	vector<int> m_CCFaceInd2IsoFace;
	vector<int> m_vecIsoFace2ExtFace; //2 integers for one isolated face; it's the ind in m_vecExteriorFacess

	CellComplex* m_cc;

	//these are for rending
	int m_nMax3DTime;
	int m_nMax2DTime;
public:
	bool m_bHighScoreOnly;	//only measure iso for medial faces and medial edges on skeleton; false: measure for all iso elements
	bool m_bMixed;

	scoreReflector();
	~scoreReflector() { m_cc = NULL; }
	void clearData();

	void setCellComplexPtr(CellComplex* cc) { m_cc = cc; }

	/* step1.find all the exterior faces, and set the map

	save:
	(1) m_CCFaceInd2ExteriorFaceInd, index of the exterior face in the original cell complex to index in the gathered vector
	(2) m_vecExteriorFaces: save the face index(when going through the arrival time, it's easier to fetch the original index)
	initialize the arrival time to 0./
	*/
	void setExteriorFaceInd();	//find all the faces that are outermost(only has one parent)	

								//this works for both step2 and step 3
	void findIsoHighScoreInterLD(const int hD, vector<int>& ldInd, vector<float>&ldScore);
	void findLowerD(const int hD, const vector<int>& hdInd, const vector<float>& hdScore,
		vector<int>&ldInd, vector<float>&ldScore);

	/*
	step2. go through all the isolated faces in the 1st burning process,
	find the outmost pair of faces that the 3d fire starts burning and end here
	save:
	(1) refresh the IterNum_ArrivalTime[ 0 ]  in the exteriorface with the highest one
	(2) m_vec1isIsoFace2ExteriorFace save the ending faces for the isolated face
	*/
	int countIsoFace();
	void initializeIsoFace2ExtFace();
	void traceIsoFaceToExteriorFace();
	inline void traceWeakIsoFace(const int edgeInd, const float score, int type, int interPtInd);
	void findMedialEdge_FaceIsoTime(vector<int>& edgeInd, vector<float>& edgeScore);
	void traceWeakIsoEdge_ExcludeTracedEdges(const int ptInd, const float score, vector<int>& edgeMarks);
	void traceIsoFaceInterEdgeToExteriorFace();

	/*
	step3. go through all the medial edges(edges with high score, i don't care about isolated edges!!)
	on each traced chain, for each face, find its two exterior faces, refresh IterNum_ArrivalTime[1] with highest possible value
	*/
	void traceMedialEdgeToExteriorFace();
	inline void traceWeakIsoEdge(const int ptInd, const float score);
	void traceIsoEdgeInterPtToExteriorFace();

	/*
	reflect arrival time of the 3d fire and 2d fire on the isoface and medial edge
	to the boundary faces
	*/
	void setMaxArrivalTime();
	//////////////////////////////////////////////////////////////////////////
	void setFaceCenters(); //temporary
						   //////////////////////////////////////////////////////////////////////////
	void reflectScore();


	/************************************************************************/
	/*
	rendering functions
	*/
	/************************************************************************/
	bool m_bShow3DTime;	//show 3d score
	bool m_bShow2DTime;	//show 2d score
	bool m_bShowExt2IsoFace;	//show the connection between exterior face and the isolated face
	bool m_bShowExt2MedEdge;	//show the connection between exterior face to the medial edge that gives it maximal 2d arrival time
	bool m_bShowSkeletonExt;
	bool m_bShowSecondary;	//show those faces corresponding to interior face and curves.
};
