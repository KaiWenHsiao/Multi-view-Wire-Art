#include "CellComplex.h"
#include "mymath.h"
#include <fstream>

#define _CRT_SECURE_NO_WARNINGS

int maxFaceIterNum_STEP;
int maxEdgeIterNum_STEP;

CellComplex::CellComplex()
{
	//elementSize = new int[4];
	//elements = new Element**[4];

#ifdef HASGEOMETRY
	ptPos = NULL;
#endif

	m_scoreRefPtr = NULL;
	m_nMaxDim = 0;
	elements = NULL;
	elementSize = NULL;
	ptPos = NULL;

	faceNorm = NULL;
	facePtInds = NULL;

	//skl
	maxIterNum = 0;
	isoThresh = 0;
	speedThresh = 0;

	parentsNum[0] = parentsNum[1] = parentsNum[2] = NULL;
	distDiffThresh[0] = distDiffThresh[1] = distDiffThresh[2] = 5;
	speedThreshThin[0] = speedThreshThin[1] = speedThreshThin[2] = .5;

	componentThresh[1] = componentThresh[2] = 10;

	m_fUnitlen = 0;
}

CellComplex::~CellComplex()
{
	/*
	for (int i = 0; i < 4; i++)
	{
		if (elementSize[i] != 0)
			delete[]elements[i];
	}
	delete[] elements;
	delete[] elementSize;
	delete[] ptPos;
	*/
	clear();
}

void CellComplex::clear()
{
	if (m_nMaxDim != 0)
	{
		for (int i = 0; i < m_nMaxDim; i++)
		{
			if (elementSize[i] != 0)
				delete[]elements[i];
		}
		delete[]elements;

		delete[]elementSize;

#ifdef HASGEOMETRY
		delete[]ptPos;
#endif

		delete[]faceNorm;
		delete[]facePtInds;
	}
	maxIterNum = 0;

	/*
	for (int i = 0; i < 4; i++)
	{
		if (elementSize[i] != 0)
			delete[] elements[i];
	}
	delete[] elements;

	delete[] elementSize;

	delete[] ptPos;
	*/
	//delete[] faceNorm;
	//delete[] facePtInds;
}

void CellComplex::read(const char *fname)
{
	clear();
	//m_scoreRefPtr->clearData();

	FILE* fp;
	errno_t err;
	err = fopen_s(&fp, fname, "rb");

	if (fp == NULL)
	{
		cout << "Unable to open file!" << endl;
	}

	m_nMaxDim = 4;
	elements = new Element**[m_nMaxDim];
	elementSize = new int[m_nMaxDim];

	fseek(fp, sizeof(int), SEEK_CUR);
	fread(elementSize, sizeof(int), 1, fp);
	cout << "#vertex:" << elementSize[0] << endl;

	elements[0] = new Element*[elementSize[0]];

	for (int i = 0; i < elementSize[0]; i++)
	{
		elements[0][i] = new Point();
	}

	float pos[3];
#ifdef HASGEOMETRY
	ptPos = new float[3 * elementSize[0]];
	float minxyz[3], maxxyz[3];
	bool isFirst = true;
#endif

	for (int i = 0; i < elementSize[0]; i++)
	{
#ifdef HASGEOMETRY
		fread(ptPos + 3 * i, sizeof(float), 3, fp);

		if (isFirst)
		{
			isFirst = false;
			for (int j = 0; j < 3; j++)
			{
				minxyz[j] = maxxyz[j] = ptPos[j];
			}
		}
		else
		{
			for (int j = 0; j < 3; j++)
			{
				if (minxyz[j] > ptPos[3 * i + j])
				{
					minxyz[j] = ptPos[3 * i + j];
				}
				else if (maxxyz[j] < ptPos[3 * i + j])
				{
					maxxyz[j] = ptPos[3 * i + j];
				}
			}
		}
#else
		fread(pos, sizeof(float), 3, fp);
#endif
	}

#ifdef HASGEOMETRY

	//if( m_fUnitlen == 0 )	//for those i want them to resize based on the same center and length.
	/*
	{
		m_fUnitlen = max(max(maxxyz[0] - minxyz[0], maxxyz[1] - minxyz[1]),
			maxxyz[2] - minxyz[2]);

		m_fUnitlen /= 3;
		MyMath::center(minxyz, maxxyz, m_aCenter);
	}
	*/

#ifdef _DEFINE_USING_GLOBAL_RESIZE_
	memcpy(m_aCenter, _global_center, sizeof(float)* 3);
	m_fUnitlen = _global_unitlen;
#endif
	//uniform the positions
	/*
	for (int i = 0; i < elementSize[0]; i++)
	{
		MyMath::getrelativepos(ptPos + 3 * i, m_aCenter, m_fUnitlen);
	}
	*/
#endif

	fread(elementSize + 1, sizeof(int), 1, fp);
	cout << "#edge:" << elementSize[1] << endl;

	if (elementSize[1] != 0)
	{
		elements[1] = new Element*[elementSize[1]];
		for (int i = 0; i < elementSize[1]; i++)
		{
			elements[1][i] = new Edge();
			fread(elements[1][i]->getChildren(), sizeof(int), 2, fp);
		}
	}

	fread(elementSize + 2, sizeof(int), 1, fp);
	cout << "#face:" << elementSize[2] << endl;

	if (elementSize[2] != 0)
	{
		elements[2] = new Element*[elementSize[2]];
		for (int i = 0; i < elementSize[2]; i++)
		{
			elements[2][i] = new Face();
			fread(elements[2][i]->getChildren(), sizeof(int), 4, fp);
		}
	}

	fread(elementSize + 3, sizeof(int), 1, fp);
	cout << "#cellcomplex:" << elementSize[3] << endl;

	if (elementSize[3] < 0)
	{
		elementSize[3] = 0;
		cout << "Error!" << endl;
	}
	if (elementSize[3] != 0)
	{
		elements[3] = new Element*[elementSize[3]];
		for (int i = 0; i < elementSize[3]; i++)
		{
			elements[3][i] = new Cell();
			fread(elements[3][i]->getChildren(), sizeof(int), 6, fp);
		}
	}

	fclose(fp);

	for(int dim = 0 ; dim < 3 ; dim++)
		setParents(dim);

	cout << "reading done!" << endl;

	initSign();
	computeFaceNorm();
	measureVoxelLen();

	for (int i = 0; i < elementSize[1]; i++)
	{
		elements[1][i]->distance = 0;
	}
}

void CellComplex::readCC_skl_significance(const char* fname)
{
	FILE* fin;
	errno_t err;
	err = fopen_s(&fin, fname, "rb");

	if (fin == NULL)
	{
		cout << "unable to open file " << fname << " to read!" << endl;
		return;
	}

	cout << "Start reading ccsklsig file... \n";
	m_nMaxDim = 4;

	//4 size numbers
	elementSize = new int[4];
	for (int i = 0; i< 4; i++)
	{
		fread(&elementSize[i], 1, sizeof(int), fin);
	}
	cout << "#vertex : " << elementSize[0] << endl;
	cout << "#edge : " << elementSize[1] << endl;
	cout << "#face : " << elementSize[2] << endl;
	cout << "#cell : " << elementSize[3] << endl;
	//vertex all the positions

	ptPos = new float[3 * elementSize[0]];
	fread(ptPos, 1, sizeof(float) * elementSize[0] * 3, fin);

	float minMax[3][2];
	minMax[0][0] = minMax[0][1] = ptPos[0];
	minMax[1][0] = minMax[1][1] = ptPos[1];
	minMax[2][0] = minMax[2][1] = ptPos[2];
	float* ptPtr = ptPos;
	for (int i = 0; i < elementSize[0]; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (ptPtr[j] < minMax[j][0])
				minMax[j][0] = ptPtr[j];
			else if (ptPtr[j] > minMax[j][1])
				minMax[j][1] = ptPtr[j];
		}
		ptPtr += 3;
	}

	//resize
	m_fUnitlen = 0;
	for (int i = 0; i < 3; i++)
	{
		m_aCenter[i] = (minMax[i][0] + minMax[i][1]) / 2;
		if (minMax[i][1] - minMax[i][0] > m_fUnitlen)
			m_fUnitlen = minMax[i][1] - minMax[i][0];
	}
	m_fUnitlen /= 3;

	/*
	for (int i = 0; i < elementSize[0]; i++)
	{
		MyMath::getrelativepos(ptPos + 3 * i, m_aCenter, m_fUnitlen);
	}
	*/

	elements = new Element**[4];
	for (int i = 0; i < 4; i++)
		elements[i] = new Element*[elementSize[i]];
	for (int i = 0; i < elementSize[0]; i++)
		elements[0][i] = new Point();
	for (int i = 0; i < elementSize[1]; i++)
		elements[1][i] = new Edge();
	for (int i = 0; i < elementSize[2]; i++)
		elements[2][i] = new Face();
	for (int i = 0; i < elementSize[3]; i++)
		elements[3][i] = new Cell();

	for (int i = 0; i < 4; i++)
	{
		if (elementSize[i] == 0)
		{
			continue;
		}

		int parNum, childNum;
		fread(&parNum, 1, sizeof(int), fin);
		fread(&childNum, 1, sizeof(int), fin);

		//write out all of them
		for (int j = 0; j < elementSize[i]; j++)
		{
			//parent
			fread(elements[i][j]->getParents(), 1, sizeof(int) * parNum, fin);
			fread(elements[i][j]->getChildren(), 1, sizeof(int) * childNum, fin);
			fread(&(elements[i][j]->signs), 1, 1, fin);
			fread(&(elements[i][j]->speed), 1, sizeof(float), fin);
			fread(&(elements[i][j]->distance), 1, sizeof(float), fin);
			fread(&(elements[i][j]->isoNum), 1, 1, fin);
			elements[i][j]->isExist = ISEXIST(elements[i][j]->signs);
		}
	}

	fread(&maxIterNum, 1, sizeof(int), fin);
	fread(maxDist, 1, sizeof(int) * 3, fin);

	measureVoxelLen();
	computeFaceNorm();
	cout << "done!\n";
	fclose(fin);
}

void CellComplex::construct()
{
	for (int dim_i = 0; dim_i < 3; dim_i++)
	{
		setParents(dim_i);
	}
}

void CellComplex::setParents(int dim)
{
	if (elementSize[dim + 1] == 0)
	{
		return;
	}

	int *childPtr;
	int *parentPtr;
	int parentNum = elements[dim][0]->getParentsNum();
	int childNum = elements[dim + 1][0]->getChildrenNum();

	for (int i = 0; i < elementSize[dim + 1]; i++)
	{
		childPtr = elements[dim + 1][i]->getChildren();

		for (int j = 0; j < childNum; j++)
		{
			int elemi = childPtr[j];

			parentPtr = elements[dim][elemi]->getParents();

			int k = -1;
			while (parentPtr[++k] != -1);
			
			parentPtr[k] = i;
		}
	}
}

void CellComplex::test()
{
	for (int i = 0; i < elementSize[1]; i++)
	{
		int edgeInd1 = elements[1][i]->getChildren()[0];
		int edgeInd2 = elements[1][i]->getChildren()[1];
	}
}

void CellComplex::thin2Pass_Cmp(float faceThresh, float edgeThresh)
{
	//componentThresh[ 1 ] = componentThresh[ 2 ] = 10;

	speedThreshThin[2] = faceThresh;
	speedThreshThin[1] = edgeThresh;

	clock_t startTime = clock();

	//measurement
	initSign_setExist_clearOthers();
	sigMeasureFP();

	//component analysis on the elements that have high score
	restoreExist_HighScore();

#ifdef COMPUTE_COMPONENT
	for (int i = 1; i <= 2; i++)
	for (int j = 0; j < elementSize[i]; j++)
	{
		elements[i][j]->isoNum = 0;
	}

	for (int i = 1; i <= 2; i++)
	{
		componentAnalysis(i);
	}
#endif

#ifdef _GLOBAL_DRAW_COMPONENT //for showing clustered elements. after thinning, only restore exist signs of these elements in clusters.
	vector<int> faceExistCMPN, edgeExistCMPN;
	swap(faceExistCMPN, vector<int>(elementSize[2], 0));
	swap(edgeExistCMPN, vector<int>(elementSize[1], 0));
	int* existPtr[2];
	existPtr[0] = &(edgeExistCMPN[0]);
	existPtr[1] = &(faceExistCMPN[0]);

	for (int i = 0; i < 2; i++)
	{
		int* ptrExist = existPtr[i];
		int dim = i + 1;
		for (int j = 0; j < m_aElemsSize[dim]; j++)
		{
			if (ISEXIST(m_daElems[dim][j]->signs))
			{
				*ptrExist = 1;
			}
			ptrExist++;
		}
	}
#endif	

	//set lock sign for all those that still remain after this
	setLockSign_RestoreExist();

	int iterNum = removalFP_PreserveLock();

	//get intial skeleton, based on the scores
	//initSign_setExist_preserveOthers();
	//removalFP();	

	maxIterNum = iterNum;
	maxDist[0] = maxDist[1] = maxDist[2] = iterNum;

	clock_t endTime = clock();
	printf("\n\t2 pass thinning, time consuming(s): ");
	cout << (endTime - startTime) / (float)CLOCKS_PER_SEC;

	copy_sign_2_bexist();
	setMaxDdS1();

#ifdef _GLOBAL_DRAW_COMPONENT
	existPtr[0] = &(edgeExistCMPN[0]);
	existPtr[1] = &(faceExistCMPN[0]);
	for (int i = 0; i < 2; i++)
	{
		int* ptrExist = existPtr[i];
		int dim = i + 1;
		for (int j = 0; j < m_aElemsSize[dim]; j++)
		{
			if (*ptrExist++)
			{
				m_daElems[dim][j]->m_bIsExist = true;
			}
			else
				m_daElems[dim][j]->m_bIsExist = false;
		}
	}
#endif
}

void CellComplex::setMaxDdS1()
{
	for (int i = 1; i < 3; i++)
	{
		maxD[i] = maxd[i] = maxS1[i] = 0;

		for (int j = 0; j < elementSize[i]; j++)
		{
			if (ISISOSET(elements[i][j]->signs))
			{
				float s1 = elements[i][j]->distance;
				int D = elements[i][j]->isoNum;
				int d = (s1 * L) + D;

				if (d > maxd[i])
					maxd[i] = d;
				if (D > maxD[i])
					maxD[i] = D;
				if (s1 > maxS1[i])
					maxS1[i] = s1;
			}
		}
	}

	for (int i = 1; i < 3; i++)
	{
		cout << "i: D" << maxD[i] << "  d:" << maxd[i] << " s1:" << maxS1[i] << endl;
	}
}

void CellComplex::copy_sign_2_bexist()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < elementSize[i]; j++)
		{
			elements[i][j]->isExist = (ISEXIST(elements[i][j]->signs)) ? true : false;
		}
	}
}

int CellComplex::removalFP_PreserveLock()
{
	int iterNum = 0;
	bool shouldStop = false;
	int lastTag, curTag;

	lastTag = 1;
	curTag = 2;

	//////////////////////////////////////////////////////////////////////////
	int countMark = 0;
	int countRemoval = 0;
	int count = 0;
	//////////////////////////////////////////////////////////////////////////
	int parentNumA[3] = { 6, 4, 2 };
	int childNumA[3] = { 2, 4, 6 };
	for (int i = 0; i <= 2; i++)
	{
		if (elementSize[i] != 0)
			parentNumA[i] = elements[i][0]->getParentsNum();
		if (elementSize[i + 1] != 0)
			childNumA[i] = elements[i + 1][0]->getChildrenNum();
	}

	cout << "iternum: ";
	//printf("%5d", 0);
	while (!shouldStop)
	{
		iterNum++;
		//printf("\b\b\b\b\b%5d", iterNum);

		count++;
		shouldStop = true;
		for (int lD = 2; lD >= 0; lD--)
		{
			int hD = lD + 1;

			if ((elementSize[lD] == 0) || (elementSize[hD] == 0))
				continue;

			int childNum, parentNum;
			childNum = childNumA[lD];
			parentNum = parentNumA[lD];

			for (int i = 0; i < elementSize[lD]; i++)
			{
				if (!ISEXIST(elements[lD][i]->signs) || ISLOCKED(elements[lD][i]->signs))
					continue;

				//to remove?
				int tag = GETTAGGED2(elements[lD][i]->signs);
				if (tag == curTag)
					continue;

				if (tag == lastTag)
				{

					//ld - hD pair or (ld-1)-ld pair					
					int parInd = GETPARENT(elements[lD][i]->signs);
					if (parInd >= parentNum)
						continue;

					parInd = elements[lD][i]->getParents()[parInd];
					if (parInd == -1)
						continue;

					int tag2 = GETTAGGED2(elements[hD][parInd]->signs);
					if (tag2 != lastTag)
					{
						continue;

					}

					//////////////////////////////////////////////////////////////////////////
					countRemoval++;
					//////////////////////////////////////////////////////////////////////////
					//remove it with its parent!
					//exist sign, and tag sign
					CLEAREXIST(elements[lD][i]->signs);
					CLEARTAGGED2(elements[lD][i]->signs);

					CLEAREXIST(elements[hD][parInd]->signs);
					CLEARTAGGED2(elements[hD][parInd]->signs);
					continue;
				}


				//count its parents
				int uniqParInd;
				int simpleTest = 0;
				int* parPtr = elements[lD][i]->getParents();

				for (int j = 0; j < parentNum; j++)
				{
					int parInd = parPtr[j];
					if (parInd == -1)
					{
						break;
					}

					if (ISEXIST(elements[hD][parInd]->signs))
					{
						//parent locked?
						if (ISLOCKED(elements[hD][parInd]->signs))
						{
							simpleTest = 100;
							SETLOCKED(elements[lD][i]->signs);
							break;
						}

						//marked by someone else?
						int tag2 = GETTAGGED2(elements[hD][parInd]->signs);
						if (tag2 == lastTag)
							continue;

						if (tag2 == curTag)
						{
							simpleTest = 100;
							break;
						}

						simpleTest++;
						if (simpleTest == 1)
						{
							uniqParInd = j;
						}
						else
							break;
					}
				}

				if (simpleTest == 1)	//tag the simple pair
				{
					//priority. is this child the first child of the parent?
					int parInd = parPtr[uniqParInd];
					int* childPtr = elements[hD][parInd]->getChildren();
					for (int j = 0; j < childNum; j++)
					{
						int lDChildInd = childPtr[j];
						if (ISLOCKED(elements[lD][lDChildInd]->signs))	//no need to check the locked child!!
							continue;

						//count parents of this child
						int* parPtr2 = elements[lD][lDChildInd]->getParents();
						int parCount2 = 0;
						for (int k = 0; k < parentNum; k++)
						{
							if (parPtr2[k] == -1)
								break;

							if (ISEXIST(elements[hD][parPtr2[k]]->signs))
							{
								int tag = GETTAGGED2(elements[hD][parPtr2[k]]->signs);
								if (tag == lastTag)
									continue;
								parCount2++;
								if (parCount2 == 1)
									uniqParInd = k;
								else
									break;
							}
						}
						if (parCount2 == 1)	//must point to this face, check if this child is the same as the one we are checking
						{
							SETTAGGED2(elements[lD][lDChildInd]->signs, curTag);
							SETPARENT(elements[lD][lDChildInd]->signs, uniqParInd);
							SETTAGGED2(elements[hD][parInd]->signs, curTag);
							SETPARENT(elements[hD][parInd]->signs, j);
							shouldStop = false;
							//////////////////////////////////////////////////////////////////////////
							countMark++;
							//////////////////////////////////////////////////////////////////////////
							break;
						}
					}
				}
			}
		}
		lastTag = curTag;
		curTag = 3 - lastTag;
	}
	cout << endl;

	return iterNum;
}

void CellComplex::setLockSign_RestoreExist()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < elementSize[i]; j++)
		{
			if (ISEXIST(elements[i][j]->signs))
			{
				elements[i][j]->signs = 1;
				SETLOCKED(elements[i][j]->signs);
			}
			else
			{
				elements[i][j]->signs = 1;
			}
		}
	}
}

void CellComplex::sigMeasureFP()
{
	bool shouldStop = false;
	int iterNum = 1;
	int lastTag, curTag;
	lastTag = 1;
	curTag = 2;

	cout << "iternum: ";
	//printf("%5d", 1);


	int parentNumA[3] = { 6, 4, 2 };
	int childNumA[3] = { 2, 4, 6 };
	for (int i = 0; i <= 2; i++)
	{
		if (elementSize[i] != 0)
			parentNumA[i] = elements[i][0]->getParentsNum();
		if (elementSize[i + 1] != 0)
			childNumA[i] = elements[i + 1][0]->getChildrenNum();
	}
	while (!shouldStop)
	{
		{
			//printf("\b\b\b\b\b%5d", iterNum);
		}

		shouldStop = true;

		for (int i = 2; i >= 0; i--)
		{
			if (elementSize[i] == 0 || elementSize[i + 1] == 0)
				continue;

			int parentNum, hD, lD, childNum;
			hD = i + 1;
			lD = i;
			parentNum = parentNumA[i];				//m_daElems[ lD ][ 0 ]->getParentsNum();
			childNum = childNumA[i];					//m_daElems[ hD ][ 0 ]->getChildrenNum();

			for (int j = 0; j < elementSize[i]; j++)
			{
				if (!ISEXIST(elements[lD][j]->signs))
				{
					continue;
				}

				//should this one to be removed with the cell it's adjacent to?
				int tag = GETTAGGED2(elements[lD][j]->signs);

				if (tag == curTag)
					continue;

				if (tag == lastTag)
				{
					int parInd = GETPARENT(elements[lD][j]->signs);
					if (parInd >= parentNum)	//not ld-hD pair, but ld-1 - ld pair
						continue;

					parInd = elements[lD][j]->getParents()[parInd];
					if (parInd == -1)
						continue;
					int tagP = GETTAGGED2(elements[hD][parInd]->signs);
					if (tagP != lastTag)	//not ld-hD pair, but ld-1 - ld pair
						continue;

					//get its parent, mark exist for both as 0
					//clear the tag in this element
					CLEARTAGGED2(elements[lD][j]->signs);
					CLEARTAGGED2(elements[hD][parInd]->signs);

					//clear exist for both
					CLEAREXIST(elements[lD][j]->signs);
					CLEAREXIST(elements[hD][parInd]->signs);

					//set for the step by step removal, so that i can have the true colors.
					if (hD == 2)
						maxFaceIterNum_STEP = iterNum;
					if (hD == 1)
						maxEdgeIterNum_STEP = iterNum;

					continue;
				}

				//count neighboring cells
				int* parPtr = elements[lD][j]->getParents();

				int simpleTest = 0;
				int uniqParentI = 0;	//index in the parent array			
				for (int k = 0; k < parentNum; k++)
				{
					if (parPtr[k] == -1)
						break;

					if (ISEXIST(elements[hD][parPtr[k]]->signs))
					{
						//check if the parent has already been tagged by someone else
						int tag = GETTAGGED2(elements[hD][parPtr[k]]->signs);
						if (tag == lastTag)
							continue;

						if (tag == curTag) //tagged by someone else already, or to be removed soon
						{
							simpleTest = 100;
							break;
						}

						simpleTest++;
						if (simpleTest == 1)
						{
							uniqParentI = k;
						}
						else
							break;
					}
				}

				if (simpleTest == 0)	//isolated face
				{
					if (!ISISOSET(elements[lD][j]->signs))
					{
						SETISOSET(elements[lD][j]->signs);
						elements[lD][j]->isoNum = iterNum - 1;
					}
				}
				else if (simpleTest == 1)
				{
					//check priority, first child is removed with the face
					const int parInd = parPtr[uniqParentI];
					int* childPtr = elements[hD][parInd]->getChildren();

					for (int ii = 0; ii < childNum; ii++)
					{
						//find the first child that has single nbring cell
						int ldInd = childPtr[ii];
						int* parPtr2 = elements[lD][ldInd]->getParents();
						int parCount = 0;
						for (int jj = 0; jj < parentNum; jj++)
						{
							if (parPtr2[jj] == -1)
								break;
							if (ISEXIST(elements[hD][parPtr2[jj]]->signs))
							{
								int tag = GETTAGGED2(elements[hD][parPtr2[jj]]->signs);
								if (tag == lastTag)
									continue;

								parCount++;
								if (parCount == 1)	//refresh the uniqiparent with the new face
									uniqParentI = jj;
								else
									break;
							}
						}
						if (parCount == 1)	//find one child that has single parent
						{
							//same as the one we are checking?
							//if( ldInd == j )//yes
							{
								//1. set d of the higher dimensional element, speed and distance
								//2. set removal tag for the low dimensional element with parInd
								if (lD != 2)
								{
									float d = iterNum;
									int D = elements[hD][parInd]->isoNum;
									if (d == 0 || d <= 2)
									{
										elements[hD][parInd]->distance = 0;
										elements[hD][parInd]->speed = 0;
									}
									else
									{
										d--;
										elements[hD][parInd]->distance = d - D;
										elements[hD][parInd]->speed = 1 - D / d;
									}
								}

								SETTAGGED2(elements[lD][ldInd]->signs, curTag);
								SETPARENT(elements[lD][ldInd]->signs, uniqParentI);
								SETTAGGED2(elements[hD][parInd]->signs, curTag);
								SETPARENT(elements[hD][parInd]->signs, ii);

								shouldStop = false;
							}
							break;
						}
					}
				}
			}
		}
		iterNum++;
		lastTag = curTag;
		curTag = 3 - lastTag;
	}
	cout << endl;
}

void CellComplex::initSign_setExist_clearOthers()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < elementSize[i]; j++)
		{
			elements[i][j]->signs = 0x1;
			elements[i][j]->speed = 0;
			elements[i][j]->distance = 0;
		}
	}
}

void CellComplex::restoreExist_HighScore()
{
	for (int i = 1; i <= 2; i++)
	{
		for (int j = 0; j < elementSize[i]; j++)
		{
			if (ISEXIST(elements[i][j]->signs))
			{
				elements[i][j]->speed = 1;
				elements[i][j]->distance = 100;
				continue;
			}
			if ((elements[i][j]->distance > distDiffThresh[i])
				&& (elements[i][j]->speed > speedThreshThin[i]))
			{
				SETEXIST(elements[i][j]->signs);
			}
		}
	}
}

void CellComplex::componentAnalysis(const int dim)
{

	//go through all the faces, and find connected pieces
	vector<int> faceMark;	//not necessarily face, anyway, the element at dimension dim.
	faceMark.resize(elementSize[dim], 0);
	int startInd = 0;
	bool isFirst = true;
	for (int i = 0; i< elementSize[dim]; i++)
	{
		if (ISEXIST(elements[dim][i]->signs))
		{
			faceMark[i] = 1;
			if (isFirst)
			{
				startInd = i;
				isFirst = false;
			}
		}
	}

	stack<int> cmp;
	vector<int> curCmp;
	const int CHILDNUM = (dim == 2) ? 4 : 2;
	const int PARENTSNUM = (dim == 2) ? 4 : 6;

	while (startInd < elementSize[dim])
	{
		//find the starting face
		while ((startInd < elementSize[dim]) && (faceMark[startInd] == 0))startInd++;
		if (startInd >= elementSize[dim])	break;

		//go through all the faces, and find one component
		cmp.push(startInd);
		curCmp.push_back(startInd);
		faceMark[startInd] = 0;
		startInd++;

		//find component
		while (!cmp.empty())
		{
			//find all neighbors and push into the stack
			int curFace = cmp.top();
			cmp.pop();

			int* childPtr = elements[dim][curFace]->getChildren();
			for (int i = 0; i < CHILDNUM; i++)
			{
				int childInd = childPtr[i];
				//go through parents of this child
				int* parPtr = elements[dim - 1][childInd]->getParents();
				for (int j = 0; j< PARENTSNUM; j++)
				{
					int parInd = parPtr[j];
					if (parInd == -1)
						break;

					if (faceMark[parInd] == 0)
						continue;
					cmp.push(parInd);
					curCmp.push_back(parInd);
					faceMark[parInd] = 0;
				}
			}
		}

		//check is the component larger than threshold?
		if (curCmp.size() <= componentThresh[dim])
		{
			//clear exist for all the faces that are in the current component
			for (int i = 0; i < curCmp.size(); i++)
			{
				int faceInd = curCmp[i];
				CLEAREXIST(elements[dim][faceInd]->signs);
			}
		}
		curCmp.clear();
	}
}

void CellComplex::getEdgePts(vector<float*> &edgePts)
{
	for (int i = 0; i < elementSize[1]; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			int index = elements[1][i]->getChildren()[j];
			float *pos = new float[3];

			for (int k = 0; k < 3; k++)
			{
				pos[k] = ptPos[3*index+k];
			}
			edgePts.push_back(pos);
		}
	}
	cout << "Getting Edge Points is done!\n" << endl;
}

void CellComplex::extractPointStructure(vector<float*> &_articulationPts, vector<float*> &_internalPts, vector<float*> &_rootPts, vector<vector<float*> > &_curve)
{
	enum Type
	{
		Articulation,
		Internal,
		Root
	};

	Type *nodeType = new Type[elementSize[0]];
	int *unVisited = new int[elementSize[0]];
	vector<int> *node = new vector<int>[elementSize[0]];
	vector<int> articulationPts, internalPts, rootPts;
	vector<int> newArticulationPts;
	vector< vector<int> > curveList;
	map<pair<int,int>, int> edgeMap;

	for (int i = 0; i < elementSize[1] ; i++)
	{
		int edgeInd1 = elements[1][i]->getChildren()[0];
		int edgeInd2 = elements[1][i]->getChildren()[1];
		node[edgeInd1].push_back(edgeInd2);
		node[edgeInd2].push_back(edgeInd1);
	}

	for (int i = 0; i < elementSize[0]; i++)
	{
		if (node[i].size() == 1)
		{
			rootPts.push_back(i);
			nodeType[i] = Root;
			unVisited[i] = 1;
		}
		else if (node[i].size() == 2)
		{
			internalPts.push_back(i);
			nodeType[i] = Internal;
			unVisited[i] = 1;
		}
		else if(node[i].size() > 2)
		{
			articulationPts.push_back(i);
			nodeType[i] = Articulation;
			unVisited[i] = node[i].size();
		}
	}

	for (int i = 0; i < rootPts.size(); i++)
	{
		_rootPts.push_back(getPointFromIndex(rootPts[i]));
	}
	for (int i = 0; i < internalPts.size(); i++)
	{
		_internalPts.push_back(getPointFromIndex(internalPts[i]));
	}
	for (int i = 0; i < articulationPts.size(); i++)
	{
		_articulationPts.push_back(getPointFromIndex(articulationPts[i]));
	}
	
	for (int i = 0; i < articulationPts.size(); i++)
	{
		int nowPt = articulationPts[i];

		for (int j = 0; j < unVisited[nowPt]; j++)
		{
			newArticulationPts.push_back(nowPt);
		}
	}

	for (int i = 0; i < rootPts.size(); i++)
	{
		vector<int> curve;
		int nowPt = rootPts[i];
		int lastPt = rootPts[i];
		map<pair<int, int>, int>::iterator iter;

		curve.push_back(nowPt);

		nowPt = node[nowPt][0];

		while (true)
		{
			if (nodeType[nowPt] == Internal)
			{
				curve.push_back(nowPt);
				edgeMap[pair<int, int>(nowPt, lastPt)] = 1;
				edgeMap[pair<int, int>(lastPt, nowPt)] = 1;
				for (int j = 0; j < 2; j++)
				{
					iter = edgeMap.find(pair<int, int>(nowPt, node[nowPt][j]));
					if (iter != edgeMap.end())
					{
						continue;
					}
					else
					{
						lastPt = nowPt;
						nowPt = node[nowPt][j];
						break;
					}
				}
			}
			else
			{
				curve.push_back(nowPt);
				edgeMap[pair<int, int>(nowPt, lastPt)] = 1;
				edgeMap[pair<int, int>(lastPt, nowPt)] = 1;
				break;
			}
		}

		if (curve.size() > 1)
		{
			curveList.push_back(curve);
		}
	}

	for (int i = 0; i < newArticulationPts.size(); i++)
	{
		vector<int> curve;
		int nowPt = newArticulationPts[i];
		int lastPt = newArticulationPts[i];
		map<pair<int, int>, int>::iterator iter;

		curve.push_back(nowPt);

		for (int j = 0; j < node[nowPt].size(); j++)
		{
			iter = edgeMap.find(pair<int, int>(nowPt, node[nowPt][j]));
			if (iter != edgeMap.end())
			{
				continue;
			}
			else
			{
				lastPt = nowPt;
				nowPt = node[nowPt][j];
				break;
			}
		}
		while (true)
		{
			if (nodeType[nowPt] == Internal)
			{
				curve.push_back(nowPt);
				edgeMap[pair<int, int>(nowPt, lastPt)] = 1;
				edgeMap[pair<int, int>(lastPt, nowPt)] = 1;
				for (int j = 0; j < 2; j++)
				{
					iter = edgeMap.find(pair<int, int>(nowPt, node[nowPt][j]));
					if (iter != edgeMap.end())
					{
						continue;
					}
					else
					{
						lastPt = nowPt;
						nowPt = node[nowPt][j];
						break;
					}
				}
			}
			else
			{
				curve.push_back(nowPt);
				edgeMap[pair<int, int>(nowPt, lastPt)] = 1;
				edgeMap[pair<int, int>(lastPt, nowPt)] = 1;
				break;
			}
		}

		if (curve.size() > 1)
		{
			curveList.push_back(curve);
		}
	}

	cout << "#curve : " << curveList.size() << endl;

	for (int i = 0; i < curveList.size(); i++)
	{
		vector<float*> curve;
		for (int j = 0; j < curveList[i].size(); j++)
		{
			float *pos = new float[3];
			pos = getPointFromIndex(curveList[i][j]);
			curve.push_back(pos);
		}
		_curve.push_back(curve);
	}
}

void CellComplex::extractPointStructure_iso(vector<Line>  &_line)
{
	for (int i = 0; i < elementSize[1]; i++)
	{
		Line line;
		line.iso = elements[1][i]->isoNum;
		float *pt1, *pt2;
		pt1 = getPointFromIndex(elements[1][i]->getChildren()[0]);
		pt2 = getPointFromIndex(elements[1][i]->getChildren()[1]);
		for (int j = 0; j < 3; j++)
		{
			line.pts[j] = pt1[j];
			line.pts[j + 3] = pt2[j];
		}
		_line.push_back(line);
	}
}

float* CellComplex::getPointFromIndex(int index)
{
	float* pos = new float[3];
	memcpy(pos, (ptPos + 3*index), sizeof(float)*3);
	return pos;
}

void CellComplex::initSign()
{

	for( int i = 0;i < 4; i ++ )
	{	
		for ( int j = 0; j < elementSize[ i ]; j ++ )
		{
			//exist = 1, locked = 0, isoSet = distSet = 0
			if( elements[ i ][ j ] -> isExist  )
			{
				elements[ i ][ j ] -> signs = 0x1;
			}
			else
			{
				elements[ i ][ j ] -> signs = 0;
			}
		}	
	}
}

void CellComplex::computeFaceNorm()
{
	int faceNum = elementSize[2];

	faceNorm = new float[faceNum * 3];
	facePtInds = new int[faceNum * 4];


	int edges[4][2];
	int pts[4];
	int* childPtr;
	int* childPtr2;

	for (int i = 0; i < faceNum; i++)
	{
		childPtr = elements[2][i]->getChildren(); //4 edges belong to the face i
		for (int j = 0; j < 4; j++)	//go through all the child edges
		{
			int edgei = childPtr[j];	//edge index
			childPtr2 = elements[1][edgei]->getChildren();
			edges[j][0] = childPtr2[0];
			edges[j][1] = childPtr2[1];
		}

		//the first and second edges are opposite
		//switch the second edge with third one.
		if (edges[0][0] != edges[1][0] &&
			edges[0][0] != edges[1][1] &&
			edges[0][1] != edges[1][0] &&
			edges[0][1] != edges[1][1])
		{
			int tEdgei = childPtr[1];
			childPtr[1] = childPtr[2];
			childPtr[2] = tEdgei;

			int tPt = edges[1][0];
			edges[1][0] = edges[2][0];
			edges[2][0] = tPt;
			tPt = edges[1][1];
			edges[1][1] = edges[2][1];
			edges[2][1] = tPt;
		}
		//the second and third ones are opposite
		else if
			(
				edges[1][0] != edges[2][0] &&
				edges[1][0] != edges[2][1] &&
				edges[1][1] != edges[2][0] &&
				edges[1][1] != edges[2][1])

		{
			//switch the 3rd and 4th one
			int tEdgei = childPtr[2];
			childPtr[2] = childPtr[3];
			childPtr[3] = tEdgei;

			int tPt = edges[2][0];
			edges[2][0] = edges[3][0];
			edges[3][0] = tPt;

			tPt = edges[2][1];
			edges[2][1] = edges[3][1];
			edges[3][1] = tPt;
		}

		//four points.
		if ((edges[0][0] == edges[1][0]) || (edges[0][0] == edges[1][1]))
		{
			pts[0] = edges[0][0];
		}
		else
			pts[0] = edges[0][1];

		for (int ii = 1; ii < 4; ii++)
		{
			if ((edges[ii][0] == edges[ii - 1][0]) || (edges[ii][0] == edges[ii - 1][1]))
			{
				pts[ii] = edges[ii][1];
			}
			else
				pts[ii] = edges[ii][0];
		}

		//copy index of the 4 points of the face
		memcpy(facePtInds + 4 * i, pts, sizeof(int) * 4);

		//compute normal			
		float vec1[3], vec2[3];
		for (int j = 0; j < 3; j++)
		{
			vec1[j] = ptPos[pts[2] * 3 + j] - ptPos[pts[0] * 3 + j];
			vec2[j] = ptPos[pts[3] * 3 + j] - ptPos[pts[1] * 3 + j];
		}
		MyMath::crossProduct(vec1, vec2, faceNorm + 3 * i);
	}
}

void CellComplex::measureVoxelLen()
{
	//get one edge, and get the until length

	int* childPtr = elements[1][0]->getChildren();

	float unitlLen = 0;
	for (int i = 0; i < 3; i++)
	{
		float temp = abs(ptPos[3 * childPtr[0] + i] - ptPos[3 * childPtr[1] + i]);
		if (temp > unitlLen)
			unitlLen = temp;
	}

	//find the minmax corner
	//float minMaxCorner[ 3 ][ 2 ];
	minMaxCorner[0][0] = minMaxCorner[0][1] = ptPos[0];
	minMaxCorner[1][0] = minMaxCorner[1][1] = ptPos[1];
	minMaxCorner[2][0] = minMaxCorner[2][1] = ptPos[2];

	int pos = 3;
	for (int i = 1; i < elementSize[0]; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (ptPos[pos] < minMaxCorner[j][0])
				minMaxCorner[j][0] = ptPos[pos];
			else if (ptPos[pos] > minMaxCorner[j][1])
				minMaxCorner[j][1] = ptPos[pos];
			pos++;
		}
	}

	//get voxel len along the 3 directions
	for (int i = 0; i < 3; i++)
	{
		m_aVoxelLen[i] = (int)((minMaxCorner[i][1] - minMaxCorner[i][0]) / unitlLen + 0.5);
	}

	m_nMaxDir = (m_aVoxelLen[1] > m_aVoxelLen[0]) ? 1 : 0;
	m_nMaxDir = (m_aVoxelLen[2] > m_aVoxelLen[m_nMaxDir]) ? 2 : m_nMaxDir;

	cout << "voxel length along 3 direction:"
		<< m_aVoxelLen[0] << ","
		<< m_aVoxelLen[1] << ","
		<< m_aVoxelLen[2] << " "
		<< " maxDir:" << m_nMaxDir << endl;

	//set approximately for the tetrahedron voxellen after turning
	//_global_MaxVoxelLen = m_aVoxelLen[m_nMaxDir] * 2;
}

void CellComplex::rescaling(float param)
{
	for (int i = 0; i < elementSize[0] * 3; i++)
	{
		ptPos[i] *= param;
	}
}

void CellComplex::setDistDiffThresh(float faceThresh, float edgeThresh)
{
	distDiffThresh[1] = (int)(m_aVoxelLen[m_nMaxDir] * edgeThresh / 100 + 0.5);
	distDiffThresh[2] = (int)(m_aVoxelLen[m_nMaxDir] * faceThresh / 100 + 0.5);
}

void CellComplex::setComponentThresh(int surfaceThresh, int curveThresh)
{
	componentThresh[2] = surfaceThresh;
	componentThresh[1] = curveThresh;
}

void CellComplex::setTypes()
{
	//set patch number for face
	setFacePatchInd();

	//set types for edges
	setEdgeTypes();

	//set types for points
	setPtsTypes();
}

void CellComplex::setFacePatchInd()
{
	if (elementSize[2] == 0)
		return;

	int childnum = elements[2][0]->getChildrenNum();
	int parNum = elements[1][0]->getParentsNum();

	//find a seed, and broadcast the patch index to faces
	stack<int> seedsStack;

	//find the first seed 
	int searchStart = 0;
	for (int i = searchStart; i < elementSize[2]; i++)
	{
		if (ISEXIST(elements[2][i]->signs))
		{
			seedsStack.push(i);
			searchStart = i + 1;
			break;
		}
	}

	int patchi = 1;

	while (!seedsStack.empty())
	{
		while (!seedsStack.empty())
		{
			int facei = seedsStack.top();
			seedsStack.pop();

			elements[2][facei]->setType(1, patchi);
			int* childPtr = elements[2][facei]->getChildren();
			for (int i = 0; i < childnum; i++)
			{
				int edgei = childPtr[i];

				//check all the parents it has
				int * parPtr = elements[1][edgei]->getParents();
				for (int j = 0; j < parNum; j++)
				{
					int nbrFacei = parPtr[j];
					if (nbrFacei == -1)
						break;

					if (!ISEXIST(elements[2][nbrFacei]->signs))
						continue;

					int nbrfaceType, nbrfaceTypeVal;
					elements[2][nbrFacei]->getType(nbrfaceType, nbrfaceTypeVal);

					if (nbrfaceTypeVal != 0)
					{
						continue;
					}

					//the parent is not marked, push it into the stack
					seedsStack.push(nbrFacei);
				}
			}
		}

		//find the seed for next patch.
		patchi++;
		for (int i = searchStart; i < elementSize[2]; i++)
		{
			if (!ISEXIST(elements[2][i]->signs))
				continue;

			int faceType, faceTypeVal;
			elements[2][i]->getType(faceType, faceTypeVal);

			if (faceTypeVal == 0)
			{
				searchStart = i + 1;
				seedsStack.push(i);
				break;
			}
		}
	}

	patchNum = patchi;
	//////////////////////////////////////////////////////////////////////////
	cout << "FACE PATCHES:" << patchNum - 1 << endl;
	//////////////////////////////////////////////////////////////////////////
}

void CellComplex::setEdgeTypes()
{
	if (elementSize[1] == 0)
		return;

	int parNum = elements[1][0]->getParentsNum();
	int childNum = elements[1][0]->getChildrenNum();

	int* parTypeVal = new int[parNum];
	int parCount;
	for (int i = 0; i < elementSize[1]; i++)
	{
		if (!ISEXIST(elements[1][i]->signs))
			continue;

		parCount = 0;
		int* parPtr = elements[1][i]->getParents();
		for (int j = 0; j < parNum; j++)
		{
			if (parPtr[j] == -1)
				break;

			//find a parent
			int faceInd = parPtr[j];
			if (!ISEXIST(elements[2][faceInd]->signs))
				continue;

			int faceType, faceTypeVal;
			elements[2][faceInd]->getType(faceType, faceTypeVal);

			//only faceTypeVal matter
			parTypeVal[parCount] = faceTypeVal;
			parCount++;
		}

		//4 faces or other case
		int edgeType, edgeVal;
		switch (parCount)
		{
		case 2://patch interior edge
			//assert(parTypeVal[0] == parTypeVal[1]);
			edgeType = 2 | (EdgeType::InteriorEdge << 2);
			edgeVal = parTypeVal[0];
			break;
		case 1:	//patch boundary edge
			edgeType = 1 | (EdgeType::BoundaryEdge << 2);
			edgeVal = parTypeVal[0];
			break;
		case 0:	//curve edge, no need to set val yet, will be set later
			edgeType = 1 | (EdgeType::CurveEdge << 2);
			edgeVal = 0;
			break;
		default:	//more than 2, NMEdge
			edgeType = 1 | (EdgeType::NMEdge << 2);
			edgeVal = 0;
			break;
		}
		elements[1][i]->setType(edgeType, edgeVal);
	}
	delete[]parTypeVal;

	//for all the curve edges, set the edgetypeVal
	setEdgeCurveInd();
}

void CellComplex::setPtsTypes()
{
	int ptParNum = elements[0][0]->getParentsNum();
	int edgeChildNum;
	if (elementSize[1] == 0)
	{
		edgeChildNum = 0;
	}
	else
		edgeChildNum = elements[1][0]->getChildrenNum();

	int* parEdgeType = new int[ptParNum];
	int* parEdgeVal = new int[ptParNum];
	int parCount = 0;
	int ptType, ptVal;
	for (int i = 0; i < elementSize[0]; i++)
	{
		if (!ISEXIST(elements[0][i]->signs))
			continue;

		//exist, how many nbring edges, and what types they are
		parCount = 0;
		int* parPtr = elements[0][i]->getParents();
		for (int j = 0; j < ptParNum; j++)
		{
			int edgeInd = parPtr[j];
			if (edgeInd == -1)
				break;
			if (!ISEXIST(elements[1][edgeInd]->signs))
				continue;

			//what type and what val of the edge?
			elements[1][edgeInd]->getType(parEdgeType[parCount], parEdgeVal[parCount]);
			parCount++;
		}

		//either patch boundary or patch interior, patch index are all of the same,
		//if having patch boundary, it is boundary point, otherwise, patch interior points
		bool havingBoundary = false;
		bool allSame = true;
		for (int j = 0; j < parCount; j++)
		{
			if ((parEdgeType[j] >> 2) == EdgeType::BoundaryEdge)
			{
				havingBoundary = true;
			}
			else if ((parEdgeType[j] & 0x3) != 2)	//not interior edge
			{
				allSame = false;
				break;
			}

			if (j != parCount - 1)
			{
				if (parEdgeVal[j] != parEdgeVal[j + 1])
				{
					allSame = false;
					break;
				}
			}
		}

		if (allSame)
		{
			if (havingBoundary)
			{
				ptType = (2 | (PointType::PatchBoundaryPoint << 2));
				ptVal = parEdgeVal[0];	//patch index
			}
			else
			{
				ptType = (3 | (PointType::PatchInteriorPoint << 2)); //( 2 | (PointType::PatchInteriorPoint << 2));
				ptVal = parEdgeVal[0];	//patch index
			}
		}
		else	//curve point, junction points, or non-manifold points
		{
			//curve points
			//only 2 parents, and the two having the same curve index
			if ((parCount == 2) && (parEdgeVal[0] == parEdgeVal[1]))
			{
				ptType = (2 | (PointType::CurveInteriorPoint << 2));
				ptVal = parEdgeVal[0];
			}
			//non-mainfold points, exist 2 and only 2 non-mainfold edge parents
			else
			{
				int nmEdgeCount = 0;
				for (int j = 0; j < parCount; j++)
				{
					if ((parEdgeType[j] >> 2) == EdgeType::NMEdge)
					{
						nmEdgeCount++;
					}
				}
				if (nmEdgeCount == 2)
				{
					ptType = (2 | (PointType::NMPoint << 2));
					ptVal = 0;	//doesn't matter
				}
				else
				{
					ptType = 1 | (PointType::JunctionPt << 2);
					ptVal = 0;	//deosn't matter
				}
			}
		}

		elements[0][i]->setType(ptType, ptVal);
	}
}

void CellComplex::setEdgeCurveInd()
{
	if (elementSize[1] == 0)
		return;

	//find the first seed
	int seekStart = 0;
	stack<int> seeds;

	int edgeType, edgeTypeVal;

	for (int i = 0; i < elementSize[1]; i++)
	{
		//exist and the type is 1, curveedge
		if (!ISEXIST(elements[1][i]->signs))
		{
			continue;
		}

		elements[1][i]->getType(edgeType, edgeTypeVal);

		//curve edge?
		if ((edgeType >> 2) != EdgeType::CurveEdge)
			continue;

		seeds.push(i);
		seekStart = i + 1;
		break;
	}

	int curveInd = 1;
	edgeType = 1 | (EdgeType::CurveEdge << 2);
	int parNum = elements[0][0]->getParentsNum();
	int childNum = elements[1][0]->getChildrenNum();

	while (!seeds.empty())
	{
		while (!seeds.empty())
		{
			int edgei = seeds.top();
			seeds.pop();

			//set edgeTypeVal
			edgeTypeVal = curveInd;
			elements[1][edgei]->setType(edgeType, edgeTypeVal);

			//push all the nbring edges in by searching its points' parents
			int* childPtr = elements[1][edgei]->getChildren();
			for (int i = 0; i < childNum; i++)
			{
				int pointInd = childPtr[i];

				//child must exist, since parent exists!
				int* parPtr = elements[0][pointInd]->getParents();
				//	if( parPtr[ 1 ] == -1 || parPtr[ 2 ] != -1)
				//		continue;				

				int uniqEdge = -1;
				for (int j = 0; j < parNum; j++)
				{
					int nbrEdgeInd = parPtr[j];
					if (nbrEdgeInd == -1)
						break;

					//does the edge still exist
					if (!ISEXIST(elements[1][nbrEdgeInd]->signs))
						continue;

					//exist, type = curveedge and val = 0?
					int nbrEdgeType, nbrEdgeVal;
					elements[1][nbrEdgeInd]->getType(nbrEdgeType, nbrEdgeVal);
					if (nbrEdgeType == edgeType)
					{
						if (nbrEdgeInd == edgei)	//it is the edge itself
							continue;

						if (nbrEdgeVal != 0)	//already set
						{
							uniqEdge = -1;
							break;
						}

						//not set, wait and see.
						if (nbrEdgeVal == 0)
						{
							if (uniqEdge != -1)	//has been marked by some other edge, that edge is not connected to current curve either.
							{
								uniqEdge = -1;
								break;
							}
							uniqEdge = nbrEdgeInd;	//remember it, possible candidate in the current curve.
						}

					}
				}
				if (uniqEdge != -1)
					seeds.push(uniqEdge);

			}
		}

		//find the next seed
		for (int i = seekStart; i < elementSize[1]; i++)
		{
			//exist and the type is 1, curveedge
			if (!ISEXIST(elements[1][i]->signs))
			{
				continue;
			}

			int tEdgeType, tEdgeTypeVal;
			elements[1][i]->getType(tEdgeType, tEdgeTypeVal);

			//curve edge?
			if (((tEdgeType >> 2) != EdgeType::CurveEdge) ||
				(
				(tEdgeType >> 2) == EdgeType::CurveEdge &&
					tEdgeTypeVal != 0)
				)
				continue;

			seeds.push(i);
			seekStart = i + 1;
			break;
		}
		curveInd++;
	}

	curveNum = curveInd;
}

void CellComplex::extractLine(vector<float*> &lineSegment, vector<int> &linePtNum)
{
	vector<vector<int>> lines;
	vector<int> leftPts;
	int ptNum = elementSize[0];
	int* degree = new int[ptNum];
	bool* mark = new bool[ptNum];
	vector<int>* node = new vector<int>[ptNum];
	

	cout << ptNum << endl;

	for (int i = 0; i < ptNum; i++)
	{
		degree[i] = 0;
		mark[i] = false;
	}

	for (int i = 0; i < elementSize[1]; i++)
	{
		int edgeNode1 = elements[1][i]->getChildren()[0];
		int edgeNode2 = elements[1][i]->getChildren()[1];
		node[edgeNode1].push_back(edgeNode2);
		node[edgeNode2].push_back(edgeNode1);
		degree[edgeNode1]++;
		degree[edgeNode2]++;
	}

	for (int i = 0; i < ptNum; i++)
	{
		if (degree[i] == 1 && !mark[i])
		{
			vector<int> line;
			int currentNode = i;
			line.push_back(currentNode);
			mark[currentNode] = true;
			int neighbor = node[i][0];
			while (true)
			{
				if (degree[neighbor] == 1)
				{
					line.push_back(neighbor);
					mark[neighbor] = true;
					lines.push_back(line);
					break;
				}
				else if (degree[neighbor] == 2)
				{
					line.push_back(neighbor);
					mark[neighbor] = true;
					if (node[neighbor][0] == currentNode)
					{
						currentNode = neighbor;
						neighbor = node[neighbor][1];
					}
					else
					{
						currentNode = neighbor;
						neighbor = node[neighbor][0];
					}
				}
				else 
				{
					line.push_back(neighbor);
					lines.push_back(line);
					break;
				}
			}
		}
	}

	int *mappingTable = new int[ptNum];
	int *inverseMappingTable = new int[ptNum];
	int index = 0;

	for (int i = 0; i < ptNum; i++)
	{
		if (!mark[i])
		{
			leftPts.push_back(i);
			mappingTable[i] = index;
			inverseMappingTable[index++] = i;
		}
		else
		{
			mappingTable[i] = -1;
		}
	}

	int leftPtsNum = leftPts.size();

	vector<int>* leftNode = new vector<int>[leftPtsNum];
	int** leftNodeMap = new int*[leftPtsNum];
	for (int i = 0; i < leftPtsNum; i++)
	{
		leftNodeMap[i] = new int[leftPtsNum];
	}

	for (int i = 0; i < leftPtsNum; i++)
	{
		for (int j = 0; j < leftPtsNum; j++)
		{
			leftNodeMap[i][j] = INT_MAX;
		}
	}

	for (int i = 0; i < elementSize[1]; i++)
	{
		int edgeNode1 = elements[1][i]->getChildren()[0];
		int edgeNode2 = elements[1][i]->getChildren()[1];
		if (!mark[edgeNode1] && !mark[edgeNode2])
		{
			leftNodeMap[mappingTable[edgeNode1]][mappingTable[edgeNode2]] = 1;
			leftNodeMap[mappingTable[edgeNode2]][mappingTable[edgeNode1]] = 1;
		}
	}

	for (int i = 0; i < lines.size(); i++)
	{
		float* line = new float[3*lines[i].size()];

		for (int j = 0; j < lines[i].size(); j++)
		{
			index = lines[i][j];
			float* pt = getPointFromIndex(index);
			memcpy(&line[3*j], pt, sizeof(float) * 3);
		}

		lineSegment.push_back(line);
		linePtNum.push_back(lines[i].size());
	}

	int* parent = new int[leftPtsNum];
	bool* visited = new bool[leftPtsNum];
	int* dist = new int[leftPtsNum];
	for (int i = 0; i < leftPtsNum; i++)
	{
		parent[i] = i;
		visited[i] = false;
		dist[i] = INT_MAX;
	}
	
	dist[0] = 0;
	parent[0] = 0;

	for (int i = 0; i < leftPtsNum; i++)
	{
		int a = -1, b = -1, min = INT_MAX;
		for (int j = 0; j < leftPtsNum; j++)
		{
			if (!visited[j] && dist[j] < min)
			{
				a = j;
				min = dist[j];
			}
		}
		if (a == -1)	break;
		visited[a] = true;
		
		for (b = 0; b < leftPtsNum ; b++)
		{
			if (!visited[b] && dist[b] > leftNodeMap[a][b])
			{
				dist[b] = leftNodeMap[a][b];
				parent[b] = a;
			}
		}
	}

	cout << leftPtsNum << endl;
	for (int i = 0; i < leftPtsNum; i++)
	{
		visited[i] = false;
		//cout << "(" << i << "," << parent[i] << ")" << endl;
	}

	int* newDegree = new int[leftPtsNum];
	for (int i = 0; i < leftPtsNum; i++)
	{
		newDegree[i] = 0;
	}

	for (int i = 0; i < leftPtsNum; i++)
	{
		newDegree[parent[i]]++;
	}

	/*
	for (int i = 0; i < leftPtsNum; i++)
	{
		cout << "[" << i << "]" << "=" << newDegree[i] << endl;
	}
	*/
	int cnt = 0;
	vector<int> candidate;

	for (int i = 0; i < leftPtsNum; i++)
	{
		if (newDegree[i] == 0)
		{
			cnt++;
			candidate.push_back(i);
		}
	}

	//cout << "#cnt = " << cnt << endl;

	vector<vector<int>> newLines;

	for (int i = 0; i < cnt; i++)
	{
		vector<int> line;
		int current = candidate[i];
		while (true)
		{
			int next = parent[current];
			line.push_back(current);
			visited[current] = true;
			if (next == current)
			{
				break;
			}
			else if (newDegree[next] > 1)
			{
				newDegree[next]--;
				line.push_back(next);
				visited[next] = true;
				break;
			}
			current = next;
		}
		newLines.push_back(line);
	}

	for (int i = 0; i < newLines.size(); i++)
	{
		vector<int> line;
		line.push_back(inverseMappingTable[newLines[i][0]]);
		//cout << newLines[i][0];
		for (int j = 1; j < newLines[i].size(); j++)
		{
			line.push_back(inverseMappingTable[newLines[i][j]]);
			//cout << "->" << newLines[i][j];
		}
		//cout << endl;

		float* output = new float[3 * line.size()];
		for (int j = 0; j < line.size(); j++)
		{
			index = line[j];
			float* pt = getPointFromIndex(index);
			memcpy(&output[3 * j], pt, sizeof(float) * 3);
		}
		linePtNum.push_back(line.size());
		lineSegment.push_back(output);
	}

	for (int i = 0; i < leftPtsNum; i++)
	{
		if (!visited[i])
		{
			cout << i << " is unvisited!" << endl;
		}
	}

}

void CellComplex::construct(const vector<float>& pts, 
			   const vector<int>& ptParents,
			   const vector<int>& edgeParents, const vector<int>& edgeChildren,
			   const vector<int>& faceParents, const vector<int>& faceChildren,
			   const vector<int>& cellChildren	)
{
	int verNum = pts.size() / 3;
	int edgeNum = edgeParents.size() / 4;
	int faceNum = faceParents.size() / 2;
	int cellNum = cellChildren.size() / 6;

	ptPos = new float[ 3  * verNum ];
	memcpy( ptPos, &pts[ 0 ], sizeof( float ) * verNum * 3 );

	m_nMaxDim = 4;
	elementSize = new int[ 4 ];
	elementSize[ 0 ] =  verNum;
	elementSize[ 1 ] =  edgeNum;
	elementSize[ 2 ] =  faceNum;
	elementSize[ 3 ] =  cellNum;

	elements = new Element** [ 4 ];
	for( int i = 0; i < 4; i++ )
	{
		elements[ i ] = new Element*[ elementSize[ i ]];		
	}

	//ver
	cout<<"copying  vertices....\t";
	int parNum[ 4 ] = {6, 4, 2, 0};
	int childNum[ 4 ] = {0, 2, 4, 6};
	int pos = 0;

	for( int i = 0; i < verNum; i ++ )
	{
		elements[ 0 ][ i ] = new Point();

		//set parents
		int* ptPtr = elements[ 0 ][ i ] -> getParents();
		int pos2 = 0;
		for( int j = 0; j < parNum[ 0 ]; j ++ )
		{
			if( ptParents[ pos ] != -1 ) 
			{
				ptPtr[ pos2 ] = ptParents[ pos ];
				pos2 ++;
			}

			pos ++;
		}
	}
	cout<<"done!\n";

	pos = 0;
	int childPos = 0;
	cout<<"copying edges...\t";
	for( int i = 0; i< edgeNum; i ++ )
	{
		elements[ 1 ][ i ] = new Edge();

		//set parents
		int* pPtr = elements[ 1 ][ i ] -> getParents();
		int pos2 = 0;
		for( int j = 0; j < parNum[ 1 ]; j++ )
		{
			if( edgeParents[ pos ] != -1 )
			{
				pPtr[ pos2 ] =edgeParents[ pos ];
				pos2 ++;
			}

			pos ++;
		}		

		//set children
		int* childPtr = elements[ 1 ][ i ] ->getChildren();		
		for( int j = 0; j< childNum[ 1 ]; j ++ )
		{
			childPtr[ j ] = edgeChildren[ childPos ];
			childPos ++;
		}
	}
	cout<<"done!\n";

	//face
	cout<<"copying faces...\t";
	pos = 0;
	childPos = 0;
	for( int i = 0; i < faceNum; i ++ )
	{
		elements[ 2 ][ i ] = new Face();

		int* pPtr = elements[ 2 ][ i ] -> getParents();
		int pos2 = 0;
		for( int j = 0; j < parNum[ 2 ]; j ++ )
		{
			if( faceParents[ pos ] != -1 )
			{
                pPtr[ pos2 ] = faceParents[ pos ];
                pos2 ++;
			}
			pos ++;
		}

		int* childPtr = elements[ 2 ][ i ] -> getChildren();
		for( int j = 0; j < childNum[ 2 ]; j ++ )
		{
			childPtr[ j ] = faceChildren[ childPos ];
			childPos ++;
		}
	}
	cout<<"done!\n";

	cout<<"copying cells...\t";
    childPos = 0;
	for(int i = 0; i <cellNum; i ++ )
	{
		elements[ 3 ][ i ] = new Cell();
		int* childPtr = elements[ 3 ][ i] -> getChildren();
		for( int j = 0; j < childNum[ 3 ]; j ++ )
		{
			childPtr[ j ] = cellChildren[ childPos ];
			childPos ++;
		}
	}
	cout<<"done!\n";

	computeFaceNorm();

	//maxIterNum = 10;	//fake it

	maxIterNum = 0;	//fake it

}

void CellComplex::writeCellComplexV0(const char* fname)
{
	FILE* fout;
	errno_t err = fopen_s(&fout, fname, "wb");
	if (err != 0)
	{
		cout << "unable to open file " << fname << " to write!\n";
		return;
	}

	//<veresion No>integer: which verseion of cell complex file. 
	int temp = 0;
	fwrite(&temp, 1, sizeof(int), fout);

	//	<#vertex>integer: number of vertices in total, type integer
	//	<vertices>float: vertex one by one, each one has 3 floating number
	fwrite(elementSize, 1, sizeof(int), fout);
	fwrite(ptPos, 1, sizeof(float)* 3 * elementSize[0], fout);

	int childNum[4] = { 0, 2, 4, 6 };

	//	<#Edges>integer: number of edges in total
	//	<edges>integer: each edge has two integers, one for index of the vertex in this edge
	fwrite(elementSize + 1, 1, sizeof(int), fout);
	int cNum = childNum[1] * sizeof(int);
	for (int i = 0; i < elementSize[1]; i++)
	{
		int* childPtr = elements[1][i]->getChildren();
		fwrite(childPtr, 1, cNum, fout);
	}

	//	<#faces>intger: number of faces in total
	//	<faces>integer: each face has 4 integers, one for index of the edge the face has
	fwrite(elementSize + 2, 1, sizeof(int), fout);
	cNum = childNum[2] * sizeof(int);
	for (int i = 0; i < elementSize[2]; i++)
	{
		int* childPtr = elements[2][i]->getChildren();
		fwrite(childPtr, 1, cNum, fout);
	}

	//	<#cells>integer: number of cells in total
	//	<cells>integer: each cell has 6 integers, each for the index of the face in the cell
	fwrite(elementSize + 3, 1, sizeof(int), fout);
	cNum = childNum[3] * sizeof(int);
	for (int i = 0; i < elementSize[3]; i++)
	{
		int* childPtr = elements[3][i]->getChildren();
		fwrite(childPtr, 1, cNum, fout);
	}

	fclose(fout);
}

void CellComplex::writeCellComplexV0_Exist(const char* fname)
{
	FILE* fout;
	errno_t err = fopen_s(&fout, fname, "wb");
	if (err != 0)
	{
		cout << "unable to open file " << fname << " to write!\n";
		return;
	}

	cout << "Start writing...\t";
	//<veresion No>integer: which verseion of cell complex file. 
	int temp = 0;
	fwrite(&temp, 1, sizeof(int), fout);

	//	<#vertex>integer: number of vertices in total, type integer
	//	<vertices>float: vertex one by one, each one has 3 floating number
	//make up the position for # veres
	int pos = ftell(fout);
	int verNum = 0;
	fwrite(&verNum, 1, sizeof(int), fout);	 //make up the room for #ver.
	vector<int> oInd2nInd(elementSize[0], -1);
	int cNum = sizeof(float)* 3;
	for (int i = 0; i < elementSize[0]; i++)
	{
		if (elements[0][i]->isExist)
		{
			float tempPos[3];
			memcpy(tempPos, ptPos + 3 * i, sizeof(float)* 3);
			//MyMath::getOriginPos(tempPos, m_aCenter, m_fUnitlen);
			//cout << tempPos[0] << "," << tempPos[1] << "," << tempPos[2] << endl;
			fwrite(tempPos, 1, cNum, fout);
			oInd2nInd[i] = verNum;
			verNum++;
		}
	}
	fseek(fout, pos, SEEK_SET);
	fwrite(&verNum, 1, sizeof(int), fout);
	fseek(fout, 0, SEEK_END);

	const int childNum[4] = { 0, 2, 4, 6 };

	//	<#Edges>integer: number of edges in total
	//	<edges>integer: each edge has two integers, one for index of the vertex in this edge
	pos = ftell(fout);
	int edgeNum = 0;
	fwrite(&edgeNum, 1, sizeof(int), fout);
	cNum = childNum[1] * sizeof(int);
	vector<int> nChildPtr(childNum[1], -1);
	vector<int> oInd2nInd2(elementSize[1], -1);
	for (int i = 0; i < elementSize[1]; i++)
	{
		if (elements[1][i]->isExist)
		{
			int* childPtr = elements[1][i]->getChildren();
			for (int j = 0; j < childNum[1]; j++)
			{
				nChildPtr[j] = oInd2nInd[childPtr[j]];
			}
			fwrite(&nChildPtr[0], 1, cNum, fout);
			oInd2nInd2[i] = edgeNum;
			edgeNum++;
		}
	}
	fseek(fout, pos, SEEK_SET);
	fwrite(&edgeNum, 1, sizeof(int), fout);
	fseek(fout, 0, SEEK_END);

	//	<#faces>intger: number of faces in total
	//	<faces>integer: each face has 4 integers, one for index of the edge the face has	
	int faceNum = 0;
	pos = ftell(fout);
	fwrite(&faceNum, 1, sizeof(int), fout);
	cNum = childNum[2] * sizeof(int);
	nChildPtr.resize(childNum[2]);
	oInd2nInd.clear();
	oInd2nInd.resize(elementSize[2], -1);
	for (int i = 0; i < elementSize[2]; i++)
	{
		if (elements[2][i]->isExist)
		{
			int* childPtr = elements[2][i]->getChildren();
			for (int j = 0; j < childNum[2]; j++)
			{
				nChildPtr[j] = oInd2nInd2[childPtr[j]];
			}
			fwrite(&nChildPtr[0], 1, cNum, fout);
			oInd2nInd[i] = faceNum;
			faceNum++;
		}
	}
	fseek(fout, pos, SEEK_SET);
	fwrite(&faceNum, 1, sizeof(int), fout);
	fseek(fout, 0, SEEK_END);

	//	<#cells>integer: number of cells in total
	//	<cells>integer: each cell has 6 integers, each for the index of the face in the cell
	int cellNum = 0;
	pos = ftell(fout);
	fwrite(&cellNum, 1, sizeof(int), fout);
	cNum = childNum[3] * sizeof(int);
	nChildPtr.resize(childNum[3]);
	for (int i = 0; i < elementSize[3]; i++)
	{
		if (elements[3][i]->isExist)
		{
			int* childPtr = elements[3][i]->getChildren();
			for (int j = 0; j < childNum[3]; j++)
			{
				nChildPtr[j] = oInd2nInd[childPtr[j]];
			}
			fwrite(&nChildPtr[0], 1, cNum, fout);
			cellNum++;
		}
	}
	fseek(fout, pos, SEEK_SET);
	fwrite(&cellNum, 1, sizeof(int), fout);
	fseek(fout, 0, SEEK_END);

	fclose(fout);
	cout << verNum << " "
		<< edgeNum << " "
		<< faceNum << " "
		<< cellNum << " ";
	cout << "Done\n";
	cout << "original:" << elementSize[0] << " "
		<< elementSize[1] << " "
		<< elementSize[2] << " "
		<< elementSize[3] << "\n";
}

void CellComplex::writeCellComplexV0MM(const char* fname)
{
	ofstream fout(fname);

	if (!fout.good())
	{
		cout << "unable to read in " << fname << " to write!" << endl;
		return;
	}

	fout << "{";

	//write the positions of points
	fout << "{";

	for (int i = 0; i < elementSize[0]; i++)
	{
		fout << "{"
			<< ptPos[3 * i] << ","
			<< ptPos[3 * i + 1] << ","
			<< ptPos[3 * i + 2] << "}";

		if (i != elementSize[0] - 1)
			fout << ",";
	}
	fout << "},";

	//write tall the edges
	fout << "{";

	for (int i = 0; i < elementSize[1]; i++)
	{
		fout << "{" << (elements[1][i]->getChildren())[0] << "," << (elements[1][i]->getChildren())[1] << "}";
		if (i != elementSize[1] - 1)
			fout << ",";
	}
	fout << "},";

	//write all the face
	fout << "{";
	for (int i = 0; i <elementSize[2]; i++)
	{
		fout << "{" << (elements[2][i]->getChildren())[0] << ","
			<< (elements[2][i]->getChildren())[1] << ","
			<< (elements[2][i]->getChildren())[2] << ","
			<< (elements[2][i]->getChildren())[3] << "}";
		if (i != elementSize[2] - 1)
			fout << ",";
	}
	fout << "},";

	//write all cells
	fout << "{";
	for (int i = 0; i < elementSize[3]; i++)
	{
		fout << "{" << (elements[3][i]->getChildren())[0] << ","
			<< (elements[3][i]->getChildren())[1] << ","
			<< (elements[3][i]->getChildren())[2] << ","
			<< (elements[3][i]->getChildren())[3] << ","
			<< (elements[3][i]->getChildren())[4] << ","
			<< (elements[3][i]->getChildren())[5] << "}";

		if (i != elementSize[3] - 1)
			fout << ",";
	}

	fout << "}}";
	fout.close();
}


