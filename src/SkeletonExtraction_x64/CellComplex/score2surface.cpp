#include "score2surface.h"
#include "config.h"
#include "myalgorithm.h"
#include "mymath.h"

/************************************************************************/
/*
Trace isolated faces:
trace isolated faces with high score to set high face scores on the exterior faces
trace intermediate edges through the isolated faces with small score to set weak face scores on the exterior faces
trace isolated edges:
trace isolated edges with high score to set high edge scores on the exterior faces
trace intermediate points through the isolated edges with small score to set weak edge scores on the exterior faces
*/
/************************************************************************/
scoreReflector::scoreReflector()
{
	m_cc = NULL;

	m_bShow3DTime = false;
	m_bShow2DTime = true;
	m_bShowExt2IsoFace = false;
	m_bShowExt2MedEdge = false;
	m_bShowSkeletonExt = true;

	m_bHighScoreOnly = false;	//later on, useless, since i save exist info in the data structure
	m_bMixed = true;
	m_bShowSecondary = true;
	//m_bHighScoreOnly = true;
}

/* step1.find all the exterior faces, and set the map
save:
(1) m_CCFaceInd2ExteriorFaceInd, index of the exterior face in the original cell complex to index in the gathered vector
(2) m_vecExteriorFaces: save the face index(when going through the arrival time, it's easier to fetch the original index)
initialize the arrival time to 0./
*/
void scoreReflector::setExteriorFaceInd()
{
	//////////////////////////////////////////////////////////////////////////
	cout << "setting exterior faces.." << endl;
	//////////////////////////////////////////////////////////////////////////

	//if( m_cc->m_aElemsSize[ 2 ] == 0 )return;

	int faceNum = m_cc->elementSize[2];
	swap(m_CCFaceInd2ExteriorFaceInd, vector<int>(faceNum, -1));	//helps removing the reserve too much space problem

																	//////////////////////////////////////////////////////////////////////////
	cout << "m_CCFaceInd2ExteriorFaceIndsize:" << m_CCFaceInd2ExteriorFaceInd.size() << endl;
	cout << "m_aElemsSize[2]" << m_cc->elementSize[2] << endl;
	//////////////////////////////////////////////////////////////////////////


	Element**& faceElems = m_cc->elements[2];
	int parNum = faceElems[0]->getParentsNum();

	int count = 0;
	vector<int> exterioInd;
	for (int i = 0; i < faceNum; i++)
	{
		Element*& curElem = faceElems[i];
		int j = 0;
		while (j < parNum && curElem->getParents()[j] != -1)j++;
		if (j == 1)
		{
			exterioInd.push_back(i);
			m_CCFaceInd2ExteriorFaceInd[i] = count;
			count++;
		}
	}

	//////////////////////////////////////////////////////////////////////////	
	cout << "allocating size for m_vecExteriorFaces.." << endl;
	//////////////////////////////////////////////////////////////////////////
	swap(m_vecExteriorFaces, vector<structExteriorFace>(count));

	//////////////////////////////////////////////////////////////////////////	
	cout << "m_vecExteriorFaces.size" << m_vecExteriorFaces.size() << endl;
	//////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < count; i++)
	{
		m_vecExteriorFaces[i].faceInd = exterioInd[i];

		m_vecExteriorFaces[i].IterNum_ArrivalTime3D =
			m_vecExteriorFaces[i].IterNum_ArrivalTime2D = -1;

		m_vecExteriorFaces[i].m_bSign.hasSkeletonFace = 0;
		m_vecExteriorFaces[i].m_bSign.hasSkeletonFace2 = 0;
		m_vecExteriorFaces[i].m_bSign.hasSkeletonEdge = 0;
		m_vecExteriorFaces[i].m_bSign.hasSkeletonEdge2 = 0;

		m_vecExteriorFaces[i].m_bSign.hasSkeletonFace_MedialEdge = 0;
		m_vecExteriorFaces[i].m_bSign.hasSkeletonFace_MedialPt = 0;
		m_vecExteriorFaces[i].m_bSign.hasSkeletonEdge_MedialPt = 0;

	}

	//////////////////////////////////////////////////////////////////////////
	cout << "setting exterior faces done!" << endl;
	//////////////////////////////////////////////////////////////////////////
}


int scoreReflector::countIsoFace()
{
	//if( m_cc->m_aElemsSize[ 2 ] == 0 )return 0;

	int count = 0;
	int faceNum = m_cc->elementSize[2];
	Element**& faceElems = m_cc->elements[2];
	for (int i = 0; i < faceNum; i++)
	{
		Element*& curElem = faceElems[i];
		if (ISISOSET(curElem->signs))
			count++;
	}
	return count;
}

void scoreReflector::initializeIsoFace2ExtFace()
{
	//if( m_cc->m_aElemsSize[ 2 ] == 0 )return;

	int count = 0;
	int faceNum = m_cc->elementSize[2];
	swap(m_CCFaceInd2IsoFace, vector<int>(faceNum, -1));

	Element**& faceElems = m_cc->elements[2];
	for (int i = 0; i < faceNum; i++)
	{
		Element*& curElem = faceElems[i];
		if (ISISOSET(curElem->signs))
		{
			m_CCFaceInd2IsoFace[i] = count;
			count++;
		}
	}

	swap(m_vecIsoFace2ExtFace, vector<int>(countIsoFace() * 2, -1));
}

/*
step2. go through all the isolated faces in the 1st burning process,
find the outmost pair of faces that the 3d fire starts burning and end here
save:
(1) refresh the IterNum_ArrivalTime[ 0 ]  in the exteriorface with the highest one
(2) m_vec1isIsoFace2ExteriorFace save the ending faces for the isolated face
*/
vector<int> db_traceFace;
bool db_first = false;
int dbcount = 0;
int dbcountMedialFace = 0;
void scoreReflector::traceIsoFaceToExteriorFace()
{
	//////////////////////////////////////////////////////////////////////////
	cout << "tracing isoface to exterior faces...." << endl;
	//////////////////////////////////////////////////////////////////////////
	//if( m_cc->m_aElemsSize[ 2 ] == 0 )return;

	initializeIsoFace2ExtFace();
	//////////////////////////////////////////////////////////////////////////
	cout << "after initializing isoface2extface" << endl;
	//////////////////////////////////////////////////////////////////////////


	int faceNum = m_cc->elementSize[2];
	Element**& faceElems = m_cc->elements[2];
	Element**& cellElems = m_cc->elements[3];
	int curIsoInd = 0;
	for (int i = 0; i < faceNum; i++)
	{
		Element*& curElem = faceElems[i];
		int* cellInd = curElem->getParents();
		//		if( !ISISOSET( curElem-> signs ) || cellInd[ 0 ] == -1 || cellInd[ 1 ] == -1 ) 
		if (!ISISOSET(curElem->signs) || (cellInd[0] == -1 && cellInd[1] == -1))
			continue;

		bool bIsExist = ISEXIST(curElem->signs);
		if (ISEXIST(curElem->signs))
			dbcountMedialFace++;
		//////////////////////////////////////////////////////////////////////////
		db_first = false;
		//if( ISEXIST( curElem->signs ))
		//	db_first = true;
		//////////////////////////////////////////////////////////////////////////
		//go through the two chains to find the crspding two exterior faces
		int iso2extInd = m_CCFaceInd2IsoFace[i] * 2;
		int isoNum = curElem->isoNum;
		for (int j = 0; j < 2; j++)
		{
			int curCellInd = cellInd[j];
			int lastFaceInd = i;

			//////////////////////////////////////////////////////////////////////////
			if (db_first)
				db_traceFace.push_back(lastFaceInd);
			//////////////////////////////////////////////////////////////////////////
			while (curCellInd != -1)
			{

				Element* curCell = cellElems[curCellInd];
				int childInd = GETPARENT(curCell->signs);	//it is actually fetcing child
				lastFaceInd = curCell->getChildren()[childInd];
				int* facePars = faceElems[lastFaceInd]->getParents();

				//////////////////////////////////////////////////////////////////////////
				if (db_first)
					db_traceFace.push_back(lastFaceInd);
				//////////////////////////////////////////////////////////////////////////

				if (facePars[0] == curCellInd)	//if the first one is the old one
					curCellInd = facePars[1];
				else
					curCellInd = facePars[0];
			}


			if (lastFaceInd != -1)	//lastfaceind is not -1, must be an exterior face
			{
				int ind = m_CCFaceInd2ExteriorFaceInd[lastFaceInd];

				//////////////////////////////////////////////////////////////////////////
				if (ind == -1)cout << "warning";
				//////////////////////////////////////////////////////////////////////////

				//only those exterior faces that have real medial geometry get updated if highscoreonly = true
				if (!m_bHighScoreOnly || (m_bHighScoreOnly && ISEXIST(curElem->signs)))
				{
					int oldTime = m_vecExteriorFaces[ind].IterNum_ArrivalTime3D;
					if (oldTime < isoNum || bIsExist)
					{
						m_vecExteriorFaces[ind].IterNum_ArrivalTime3D = isoNum;
						m_vecExteriorFaces[ind].isoFaceInd = i;

						//////////////////////////////////////////////////////////////////////////
						//cout<<(int)(curElem ->isoNum)<<" ";
						//////////////////////////////////////////////////////////////////////////
					}
					if (bIsExist)
					{
						m_vecExteriorFaces[ind].m_bSign.hasSkeletonFace = 1;

						////////////////////////////////////////////////////////////////////////////
						//dbcount ++;
						//if( !m_vecExteriorFaces[ ind ].m_bSign.hasSkeletonFace )
						//	cout<<"error! ";
						//else
						//	cout<<"good! ";
						////////////////////////////////////////////////////////////////////////////
					}
				}

				//////////////////////////////////////////////////////////////////////////
				/*if( db_first && db_traceFace.size() < 20)
				cout<<"lastind:"<<lastFaceInd<<" ind in exter:"<<ind<<" oldtiem:"<<oldTime<<" isonum:"<<isoNum<<" ";*/
				//////////////////////////////////////////////////////////////////////////

				m_vecIsoFace2ExtFace[iso2extInd + j] = ind;
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	cout << "tracing isoface to exterior faces done!" << endl;
	//////////////////////////////////////////////////////////////////////////
}

//find the intermediate edges of all the faces
void scoreReflector::findIsoHighScoreInterLD(const int hD, vector<int>& ldInd, vector<float>&ldScore)
{
	const int elemsNum = m_cc->elementSize[hD];
	if (elemsNum == 0)
		return;

	//step1. get all the intermediate edges
	const int lD = hD - 1;
	const int lDParNum = m_cc->elements[lD][0]->getParentsNum();
	Element**& elems = m_cc->elements[hD];
	const int hDChildNum = elems[0]->getChildrenNum();
	Element**& childElems = m_cc->elements[lD];

	//////////////////////////////////////////////////////////////////////////
	int countExistFace = 0;
	//////////////////////////////////////////////////////////////////////////

	ldInd.reserve(elemsNum / 10);
	for (int i = 0; i < elemsNum; i++)
	{
		Element* hDElem = elems[i];
		if (!ISEXIST(hDElem->signs))
			continue;

		//////////////////////////////////////////////////////////////////////////
		countExistFace++;
		//////////////////////////////////////////////////////////////////////////

		int* childInds = hDElem->getChildren();
		int childInd;
		int count = -1;
		while ((++count < hDChildNum) && ((childInd = *childInds++) != -1))
			ldInd.push_back(childInd);
	}

	//////////////////////////////////////////////////////////////////////////
	//cout<<"existface number is:"<<countExistFace<<endl;
	//cout<<"the number of interior edges:"<<ldInd.size()<<endl;
	/*cout<<"interior edge number before soring and removing repeated items:"<<endl;
	for( int i = 0; i< ldInd.size(); i++ )
	cout<<ldInd[ i ]<<" ";
	cout<<endl;*/
	//////////////////////////////////////////////////////////////////////////

	//remove repeated ones
	sort(ldInd.begin(), ldInd.end());
	vector<int>::iterator it = unique(ldInd.begin(), ldInd.end());
	ldInd.resize(it - ldInd.begin());
	swap(ldInd, vector<int>(ldInd.begin(), ldInd.end()));

	//////////////////////////////////////////////////////////////////////////
	//cout<<"after removing repeated ones:"<<endl;
	//for( int i = 0; i < ldInd.size(); i ++ )
	//	cout<<ldInd[ i ]<<" ";
	//cout<<endl;
	//////////////////////////////////////////////////////////////////////////

	//step2. get the scores of the low dimensional elements based on their parents' score
	vector<int> parScore(lDParNum, -1);
	int* ldIndPtr = &(ldInd[0]);

	swap(ldScore, vector<float>(ldInd.size(), 0));
	float* scorePtr = &(ldScore[0]);
	for (int i = 0; i < ldInd.size(); i++)
	{
		Element* childElem = childElems[*ldIndPtr++];
		int* parIndsPtr = childElem->getParents();
		int parInd;
		int count2 = -1;
		int count = 0;
		while ((++count2 < lDParNum) && (parInd = *parIndsPtr++) != -1)
		{
			if (ISEXIST(elems[parInd]->signs))
			{
				parScore[count] = elems[parInd]->isoNum;
				count++;
			}
		}
		if (count == 0)	//error!! should not happen!!!
		{
			cout << "ERROR! no parent found for the intermediate edge\n";
			scorePtr++;
			continue;
		}
		MyAlgorithmNSpace::mean<vector<int>::iterator, float>(parScore.begin(), parScore.begin() + count, *scorePtr);
		scorePtr++;
	}
}

#include <utility>
bool isLess_Pair(pair<int, float> pair1, pair<int, float> pair2)
{
	return pair1.first <= pair2.first;
}

bool isEqual_Pair(pair<int, float> pair1, pair<int, float> pair2)
{
	return pair1.first == pair2.first;
}

//proof read
void scoreReflector::findLowerD(const int hD, const vector<int>& hdInd, const vector<float>& hdScore,
	vector<int>&ldInd, vector<float>&ldScore)
{
	//step1. find all the intermediate points of the scores.
	const int lD = hD - 1;
	int hDNum = hdInd.size();
	Element**& hdElems = m_cc->elements[hD];

	const int hdChildNum = hdElems[0]->getChildrenNum();
	const int ldParNum = m_cc->elements[lD][0]->getParentsNum();

	const int* hdIndPtr = &(hdInd[0]);
	const float* hdScorePtr = &(hdScore[0]);
	vector<pair<int, float> > vecldIndScore;
	for (int i = 0; i< hDNum; i++)
	{
		Element* hdElem = hdElems[*hdIndPtr++];
		int* childIndPtr = hdElem->getChildren();
		int count = -1;
		int childInd;
		while (++count < hdChildNum && (childInd = *childIndPtr++) != -1)
			vecldIndScore.push_back(pair<int, float>(childInd, *hdScorePtr));
		//ldInd.push_back( childInd );		
		hdScorePtr++;
	}
	sort(vecldIndScore.begin(), vecldIndScore.end(), isLess_Pair);
	vector<pair<int, float> >::iterator it = vecldIndScore.begin();
	vector<pair<int, float> >::iterator firstUniqIt, lastIt;
	lastIt = vecldIndScore.end();
	while (it != lastIt)
	{
		firstUniqIt = it;
		int val = (*firstUniqIt).first;
		float score = (*firstUniqIt).second;
		while ((++it) != lastIt && (*it).first == val) score += (*it).second;
		(*firstUniqIt).second = score / (it - firstUniqIt);
	}

	lastIt = unique(vecldIndScore.begin(), vecldIndScore.end(), isEqual_Pair);
	it = vecldIndScore.begin();

	ldInd.resize(lastIt - it);
	ldScore.resize(lastIt - it);
	int pos = 0;
	while (it != lastIt)
	{
		ldInd[pos] = (*it).first;
		ldScore[pos++] = (*it).second;
		it++;
	}
	swap(ldInd, vector<int>(ldInd.begin(), ldInd.end()));
	swap(ldScore, vector<float>(ldScore.begin(), ldScore.end()));
}

/*trace edge-face pair, only through those faces with low scores(not marked as exist )

type
0 - start with intermediate edges of medial faces and medial edges, through isolated faces with low scores(s1, s2), set exterior face score
1-  start with intermediate points of medial edges, through isolated faces with low scores(s1, s2), set exterior edge score
2 - start with intermediate points of edges in type 0, and set exterior face score

*/
void scoreReflector::traceWeakIsoFace(const int edgeInd, const float score, int type = 0, int interPtInd = -1)
{
	//step1. initialize the starting stack	
	Element**& faceElems = m_cc->elements[2];
	Element**& edgeElems = m_cc->elements[1];

	Element* edgeElem = edgeElems[edgeInd];
	int parNum = edgeElem->getParentsNum();
	stack<int> faceStack;
	int* parInds = edgeElem->getParents();
	int parInd;
	int count = -1;
	while ((++count < parNum) && ((parInd = *parInds++) != -1))
	{
		unsigned char sign = faceElems[parInd]->signs;
		if (ISEXIST(sign) || !ISISOSET(sign))
			continue;

		faceStack.push(parInd);
	}

	//step2. trace edge-face pair, for each face, set its exterior face pair score if it is not set
	while (!faceStack.empty())
	{
		int curFaceInd = faceStack.top();
		faceStack.pop();
		///while( curFaceInd != -1 )
		{
			//step 2.1. set the exterior faces of the isolated face with the passed in score
			int *extPos = &m_vecIsoFace2ExtFace[m_CCFaceInd2IsoFace[curFaceInd] * 2];
			for (int i = 0; i < 2; i++)
			{
				int extInd;

				switch (type)
				{
				case 0:	//set face score through intermediate edges in high score faces, or medial edges, only for those exterior faces(not set by high score isolated faces)
					if (((extInd = *extPos++) != -1) && !(m_vecExteriorFaces[extInd].m_bSign.hasSkeletonFace))	 //extind is set first, before the second condition
					{
						float oldTime = m_vecExteriorFaces[extInd].IterNum_ArrivalTime3D;
						if (oldTime < score || !m_vecExteriorFaces[extInd].m_bSign.hasSkeletonFace2)
						{
							m_vecExteriorFaces[extInd].IterNum_ArrivalTime3D = score;
							m_vecExteriorFaces[extInd].isoFaceInd = edgeInd;
						}
						m_vecExteriorFaces[extInd].m_bSign.hasSkeletonFace2 = 1;
						m_vecExteriorFaces[extInd].m_bSign.hasSkeletonFace_MedialEdge = 1;	//need change*****
					}
					break;
				case 2:	//set face score through the points bouding edges in type 0 (only for those faces not set in high socre face stage, and intermeidate edge stage)
					if (((extInd = *extPos++) != -1) && !(m_vecExteriorFaces[extInd].m_bSign.hasSkeletonFace) &&
						!(m_vecExteriorFaces[extInd].m_bSign.hasSkeletonFace_MedialEdge))
					{

						float oldTime = m_vecExteriorFaces[extInd].IterNum_ArrivalTime3D;
						if (oldTime < score || !m_vecExteriorFaces[extInd].m_bSign.hasSkeletonFace_MedialPt)
						{
							m_vecExteriorFaces[extInd].IterNum_ArrivalTime3D = score;
							m_vecExteriorFaces[extInd].isoFaceInd = interPtInd;
						}
						m_vecExteriorFaces[extInd].m_bSign.hasSkeletonFace_MedialPt = 1;

					}
					break;
				case 1:	//set edge score for those exterior faces that haven't been set with strong edge scores
					if (((extInd = *extPos++) != -1) && !(m_vecExteriorFaces[extInd].m_bSign.hasSkeletonEdge))
					{

						float oldTime = m_vecExteriorFaces[extInd].IterNum_ArrivalTime2D;
						if (oldTime < score || !m_vecExteriorFaces[extInd].m_bSign.hasSkeletonEdge2)
						{
							m_vecExteriorFaces[extInd].IterNum_ArrivalTime2D = score;
							m_vecExteriorFaces[extInd].medialEdgeInd = interPtInd;
						}
						m_vecExteriorFaces[extInd].m_bSign.hasSkeletonEdge2 = 1;
						m_vecExteriorFaces[extInd].m_bSign.hasSkeletonEdge_MedialPt = 1;		//need change*****

					}
					break;
				default:
					break;
				}
			}

			//step 2.2. find the edge that is removed with this face, find all other faces that are incident
			//			with this edge, they are isolated and have weak scores
			int nextEdgeInd = faceElems[curFaceInd]->getChildren()[GETPARENT(faceElems[curFaceInd]->signs)];	//getparent fetches child actually
			parInds = edgeElems[nextEdgeInd]->getParents();
			int count = -1;
			while ((++count < parNum) && ((parInd = *parInds++) != -1))
			{
				if (parInd == curFaceInd)	//cann't be the one that has already been processed!!
					continue;

				unsigned char sign = faceElems[parInd]->signs;
				if (ISEXIST(sign) || !ISISOSET(sign))	//it has to be an isolated weak face, "weak" = "isolated"&"low score"; "low score" must not exist
					continue;

				faceStack.push(parInd);
			}
		}
	}
}

//proof read
void scoreReflector::findMedialEdge_FaceIsoTime(vector<int>& edgeInd, vector<float>& edgeScore)
{
	int edgeNum = m_cc->elementSize[1];
	Element**& edgeElems = m_cc->elements[1];
	Element**& faceElems = m_cc->elements[2];

	edgeInd.reserve(edgeInd.size() + edgeNum / 10);
	edgeScore.reserve(edgeScore.size() + edgeNum / 10);

	const int edgeParNum = edgeElems[0]->getParentsNum();
	for (int i = 0; i < edgeNum; i++)
	{
		Element* curEdgeElem = edgeElems[i];
		unsigned char sign = edgeElems[i]->signs;
		if (!ISEXIST(sign))
			continue;

		//it is high score edge, find its nbring isolated face, and compute average isolation score
		int count = -1;
		int parInd;
		int* parInds = curEdgeElem->getParents();
		int countIsoNum = 0;
		float score = 0;
		while (++count < edgeParNum && (parInd = *parInds++) != -1)
		{
			Element* faceElem = faceElems[parInd];
			if (ISEXIST(faceElem->signs))	//edge belongs to high score face, already in edgeInd, no need to add!!
			{
				countIsoNum = 0;
				break;
			}
			if (ISISOSET(faceElem->signs))
			{
				countIsoNum++;
				score += (int)(faceElem->isoNum);
			}
		}
		if (countIsoNum != 0)
		{
			score /= countIsoNum;
			edgeInd.push_back(i);
			edgeScore.push_back(score);
		}
	}
	swap(edgeInd, vector<int>(edgeInd.begin(), edgeInd.end()));
	swap(edgeScore, vector<float>(edgeScore.begin(), edgeScore.end()));
}

void scoreReflector::traceWeakIsoEdge_ExcludeTracedEdges(const int ptInd, const float score,
	vector<int>& edgeMarks)
{
	//go through all the weak edges
	stack<int> edgeStack;
	Element**& pointElems = m_cc->elements[0];
	Element**& edgeElems = m_cc->elements[1];
	const int ptParNum = pointElems[0]->getParentsNum();

	Element* ptElem = pointElems[ptInd];
	int* edgeParInds = ptElem->getParents();
	int edgeParInd;
	int count = -1;
	while ((++count < ptParNum) && (edgeParInd = *edgeParInds++) != -1)
	{
		unsigned char sign = edgeElems[edgeParInd]->signs;
		if ((ISEXIST(sign) || !ISISOSET(sign)) || (edgeMarks[edgeParInd])) //either strong medial edge,or traced already
			continue;
		edgeStack.push(edgeParInd);
	}

	while (!edgeStack.empty())
	{
		int curEdgeInd = edgeStack.top();
		edgeStack.pop();

		//step1. trace the edge through weak faces., set the scores at the faces	
		traceWeakIsoFace(curEdgeInd, score, 2, ptInd);	//type 2!!! set face score of exterior faces that don't have high score medial faces or edges

														//step2. find the point that makes a simple pair with current edge, trace the point's incident edges for more weak isolated edges
		Element* edgeElem = edgeElems[curEdgeInd];
		Element* nextPtElem = pointElems[edgeElem->getChildren()[GETPARENT(edgeElem->signs)]];
		edgeParInds = nextPtElem->getParents();
		int count = -1;
		while ((++count < ptParNum) && (edgeParInd = *edgeParInds++) != -1)
		{
			if (edgeParInd == curEdgeInd)
				continue;

			unsigned char sign = edgeElems[edgeParInd]->signs;
			if ((ISEXIST(sign) || !ISISOSET(sign)) || (edgeMarks[edgeParInd]))
				continue;

			edgeStack.push(edgeParInd);
		}
	}
}

void scoreReflector::traceIsoFaceInterEdgeToExteriorFace()
{
	//step1. find all the intermediate edges for faces with high scores
	vector<int> edgeInds;
	vector<float> edgeScores;
	findIsoHighScoreInterLD(2, edgeInds, edgeScores);

	//step2. go through the edges with high scores(medial edges), they are on medial surface, except that their nbring faces' 
	//3d arrival time = 2d arrival time(in continuous field, not discrete field), but still, those faces' 3d arrival time
	//means the 3d arrival time of the generating set(exterior faces)
	findMedialEdge_FaceIsoTime(edgeInds, edgeScores);

	//step3. trace all the edges, only through those weak isolated faces, find all the boundary exterior faces, set their face scores
	int edgeNum = edgeInds.size();
	for (int i = 0; i < edgeNum; i++)
	{
		traceWeakIsoFace(edgeInds[i], edgeScores[i]);
	}

	//////////////////////////////////////////////////////////////////////////
	//return;
	//////////////////////////////////////////////////////////////////////////
	//step4. find all the intermediate points of the high score edges
	vector<int> ptInds;
	vector<float> ptScores;
	findLowerD(1, edgeInds, edgeScores, ptInds, ptScores);

	swap(edgeScores, vector<float>(0));	//clear edgescore's memory, even the reserved memory
	vector<int> edgeMarks;
	swap(edgeMarks, vector<int>(m_cc->elementSize[1], 0));
	for (int i = 0; i < edgeInds.size(); i++)
	{
		edgeMarks[edgeInds[i]] = 1;
	}
	swap(edgeInds, vector<int>(0));	//clear edgeind's memory, even the reserved memory

									//step5. go through all the intermediate points, and set the point scores;
	for (int i = 0; i < ptInds.size(); i++)
	{
		traceWeakIsoEdge_ExcludeTracedEdges(ptInds[i], ptScores[i], edgeMarks);
	}
}

/*
step3. go through all the medial edges(edges with high score, i don't care about isolated edges!!)
on each traced chain, for each face, find its two exterior faces, refresh IterNum_ArrivalTime[1] with highest possible value
*/
void scoreReflector::traceMedialEdgeToExteriorFace()
{
	if (m_cc->elementSize[1] == 0)return;

	//////////////////////////////////////////////////////////////////////////
	cout << "tracing medialedge to exterior faces...." << endl;
	//////////////////////////////////////////////////////////////////////////

	//go through all the medial edges, those edges with exist = 1(after component analysis in the 2 pass thinning)
	int edgeNum = m_cc->elementSize[1];
	Element**& edgeElems = m_cc->elements[1];
	Element**& faceElems = m_cc->elements[2];
	int edgeParNum = edgeElems[0]->getParentsNum();

	for (int i = 0; i < edgeNum; i++)
	{
		Element* curEdge = edgeElems[i];

		if ((m_bHighScoreOnly && (!ISEXIST(curEdge->signs))) || (!m_bHighScoreOnly && (!ISISOSET(curEdge->signs))))
			continue;

		const bool bIsExist = ISEXIST(curEdge->signs);

		int isoNum = curEdge->isoNum;

		int* parFaceInd = curEdge->getParents();
		stack<int> facesStack;
		for (int j = 0; j < edgeParNum; j++)
		{
			int faceInd = *parFaceInd++;
			if (faceInd == -1)
				break;

			if (ISISOSET(faceElems[faceInd]->signs))
				facesStack.push(faceInd);
		}

		while (!facesStack.empty())
		{
			int curFaceInd = facesStack.top();
			facesStack.pop();

			//set the two exterior faces of the isolated face 
			int isoInd = m_CCFaceInd2IsoFace[curFaceInd] * 2;
			for (int j = 0; j < 2; j++)
			{
				int extInd = m_vecIsoFace2ExtFace[isoInd + j];
				if (extInd == -1)
					continue;

				int oldtime = m_vecExteriorFaces[extInd].IterNum_ArrivalTime2D;
				if (oldtime < isoNum || bIsExist)
				{
					m_vecExteriorFaces[extInd].IterNum_ArrivalTime2D = isoNum;
					m_vecExteriorFaces[extInd].medialEdgeInd = i;
				}
				if (bIsExist)
					m_vecExteriorFaces[extInd].m_bSign.hasSkeletonEdge = 1;
			}

			//by tracing the edge, find the other isolated incident with the edge that is different from the coming face
			Element* curFace = faceElems[curFaceInd];
			int edgeInd = GETPARENT(curFace->signs);	//actually fetch child
			edgeInd = curFace->getChildren()[edgeInd];

			//all possible isofaces incident with the edge that is different from the coming face
			Element* tEdge = edgeElems[edgeInd];
			int* faceParInds = tEdge->getParents();
			for (int j = 0; j < edgeParNum; j++)
			{
				int tfaceInd = *faceParInds++;
				if (tfaceInd == -1)
					break;
				if (tfaceInd == curFaceInd)
					continue;
				if (ISISOSET(faceElems[tfaceInd]->signs))
				{
					facesStack.push(tfaceInd);
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	cout << "tracing medialedge to exterior faces done!" << endl;
	//////////////////////////////////////////////////////////////////////////
}

void scoreReflector::traceWeakIsoEdge(const int ptInd, const float score)
{
	//go through all the weak edges
	stack<int> edgeStack;
	Element**& pointElems = m_cc->elements[0];
	Element**& edgeElems = m_cc->elements[1];
	const int ptParNum = pointElems[0]->getParentsNum();

	Element* ptElem = pointElems[ptInd];
	int* edgeParInds = ptElem->getParents();
	int edgeParInd;
	int count = -1;
	while ((++count < ptParNum) && (edgeParInd = *edgeParInds++) != -1)
	{
		unsigned char sign = edgeElems[edgeParInd]->signs;
		if (ISEXIST(sign) || !ISISOSET(sign))
			continue;

		edgeStack.push(edgeParInd);
	}

	while (!edgeStack.empty())
	{
		int curEdgeInd = edgeStack.top();
		edgeStack.pop();

		//step1. trace the edge through weak faces., set the scores at the faces	
		traceWeakIsoFace(curEdgeInd, score, 1, ptInd);

		//step2. find the point that makes a simple pair with current edge, trace the point's incident edges for more weak isolated edges
		Element* edgeElem = edgeElems[curEdgeInd];
		Element* nextPtElem = pointElems[edgeElem->getChildren()[GETPARENT(edgeElem->signs)]];
		edgeParInds = nextPtElem->getParents();
		int count = -1;
		while ((++count < ptParNum) && (edgeParInd = *edgeParInds++) != -1)
		{
			if (edgeParInd == curEdgeInd)
				continue;

			unsigned char sign = edgeElems[edgeParInd]->signs;
			if (ISEXIST(sign) || !ISISOSET(sign))
				continue;

			edgeStack.push(edgeParInd);
		}
	}
}

//trace from the point, find all the weak incident isolated edges, trace from those edges, set scores.
void scoreReflector::traceIsoEdgeInterPtToExteriorFace()
{
	//step1. find all the intermediate points of high score isolated edges
	vector<int> ptInds;
	vector<float> ptScores;
	findIsoHighScoreInterLD(1, ptInds, ptScores);

	//step2. trace all the points, only through those weak isolated edges, find all the boundary exterior faces, set their face scores
	int ptNum = ptInds.size();
	for (int i = 0; i < ptNum; i++)
	{
		traceWeakIsoEdge(ptInds[i], ptScores[i]);
	}
}

void scoreReflector::setMaxArrivalTime()
{
	//m_nMax3DTime = m_nMax2DTime = -1;
	//int faceSize = m_vecExteriorFaces.size();
	//for( int i = 0; i < faceSize; i ++ )
	//{
	//	m_nMax2DTime = max(	m_nMax2DTime , m_vecExteriorFaces[ i ].IterNum_ArrivalTime2D );
	//	m_nMax3DTime = max(	m_nMax3DTime , m_vecExteriorFaces[ i ].IterNum_ArrivalTime3D );
	//}

	////go through them, and set them as the ratio
	//for( int i = 0; i < faceSize; i ++ )
	//{
	//	m_vecExteriorFaces[ i ].IterNum_ArrivalTime2D /= m_nMax2DTime;
	//	m_vecExteriorFaces[ i ].IterNum_ArrivalTime3D /= m_nMax3DTime;
	//}

	//go through them, and set them as the ratio
	int faceSize = m_vecExteriorFaces.size();
	float maxSize = m_cc->L * 35;
	for (int i = 0; i < faceSize; i++)
	{
		m_vecExteriorFaces[i].IterNum_ArrivalTime2D /= maxSize;
		m_vecExteriorFaces[i].IterNum_ArrivalTime3D /= maxSize;
	}
}

void scoreReflector::clearData()
{
	m_CCFaceInd2ExteriorFaceInd.clear();
	m_vecExteriorFaces.clear();
	m_CCFaceInd2IsoFace.clear();
	m_vecIsoFace2ExtFace.clear();
}

void scoreReflector::reflectScore()
{
	if (!m_cc || m_cc->elementSize[2] == 0)return;

	clearData();

	setExteriorFaceInd();
	traceIsoFaceToExteriorFace();
	traceIsoFaceInterEdgeToExteriorFace();

	traceMedialEdgeToExteriorFace();
	traceIsoEdgeInterPtToExteriorFace();

	setMaxArrivalTime();

	//////////////////////////////////////////////////////////////////////////
	setFaceCenters();
	//////////////////////////////////////////////////////////////////////////

	//will be called later, since they are temporary
	//m_CCFaceInd2IsoFace.clear();
	//m_vecIsoFace2ExtFace.clear();
}

vector<float> db_faceCenters;

void scoreReflector::setFaceCenters()
{
	swap(db_faceCenters, vector<float>(3 * m_cc->elementSize[2], 0));

	int faceNum = m_cc->elementSize[2];
	float* ptPos = m_cc->ptPos;
	int* facePtInds = m_cc->facePtInds;

	float* faceCenter = &(db_faceCenters[0]);
	for (int i = 0; i < faceNum; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			int ptIndex = *facePtInds++;
			MyMath::addVec(faceCenter, ptPos + 3 * ptIndex);
		}
		MyMath::stretchVec(faceCenter, 0.25);
		faceCenter += 3;
	}
}

