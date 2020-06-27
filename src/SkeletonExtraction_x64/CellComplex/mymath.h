#ifndef _MYMATH_H_
#define _MYMATH_H_

#include "config.h"

class MyMath {
public:
	static int getSign(float val);
	static bool isEqualInToler(float num1, float num2, float toler);
	//represented in x, y, z
	static float vectorLen(float x, float y, float z);
	//represented in array
	static float dotProduct(const float vec1[3], const float vec2[3]);
	//compute the cos of angle pt1 pt0 pt2. angle between vector pt0pt1 and pt0pt2
	static float getCosOfAngle(float pt1[3], float pt2[3], float pt0[3]);
	//get vector between two poitns, and save the vector into vec
	static void getVec(const float pt1[3], const float pt2[3], float vec[3]);
	static void getNeg(float vec[3]);
	static void getPtOnSeg(float pt1[3], float pt2[2], float ratio, float pt[3]);
	static void getEndPtOfSeg(const float ept[3], const float vec[3], float resultpt[3]);
	static void getPtOnRay(float pt[3], float dir[3], float t, float endpt[3]);
	//compute the length of the vector between the two points
	static float vectorlen(float pt1[3], float pt2[3]);
	//compute the vector's length
	static float vectorlen(float vec[3]);
	//normalize the svec, and keep it unchanged, save the normalized value into dvec
	static void normalize(float svec[3], float dvec[3]);
	//normalize vec, and save the normalized value into vec
	static void normalize(float vec[3]);
	//compute center of two points p1, and p2, save it into center
	static void center(float p1[3], float p2[3], float center[3]);
	//compute the relative position to center, and unit lenght is len.
	static void getrelativepos(float pt[3], const float center[3], const float len);
	//compute the original position
	static void getOriginPos(float pt[3], const float center[3], const float unitlen);
	/*
	* return normalized crossproduct of vec1 and vec2  and save it into cp
	*/
	static void crossProduct(float vec1[3], float vec2[3], float cp[3]);
	/**
	* return non-normalized cross product of vec1, vec2 and save it in cp
	*/
	static void crossProductNotNorm(float vec1[3], float vec2[3], float cp[3]);
	//a naive algorithm to compute the difference for vertex sver to all the vertices in dver.
	static float computeDiff(float* sver, float* dver, int dvernum);

	static void stretchVec(float vec[3], const float ratio[3]);
	static void stretchVec(float vec[3], const float ratio);

	//add the second vector to the first one.
	static void addVec(float dvec[3], const float svec[3]);

	//another naive algorithm to compute the difference for vertex sver to another mesh, dvers, dtrians

	//dist: save the resulting distance from the point to the line segment lies on
	//dist = the distance from the point to the line the segment lies on , if the perpendicular line from sver crosses the seg
	//dist = the minimal distance from the point to one of the two endpoints of the segment, otherwise
	static void computeDistPt2Seg(float* sver, float* dver, int ind[2], float& dist);
	//compute distance from point to a triangle 
	//dist = distance from the point to the plane the triangle is on, if the perpendicular line crosses trian
	//dist = nearest distance from the point to the three segments
	static void computeDistPt2Trian(float* sver, float* dver, int ind[3], float& dist);

	static float computeDiff(float* sver, float* dver, int* dtrians, int triannum);

	//generate a random vector that is perpendicular to the given vector
	static void perpendicularVec(const float vec[3], float perVec[3]);

	/************************************************************************/
	/*
	1 dimensional array of arbitray length.
	*/
	/************************************************************************/
	//add the source vector to the destination vector, they are of length m.
	static void addVec_X(float dvec[], const int m, const float sdvec[]);

	/************************************************************************/
	/*
	similar to matrix operation
	*/
	/************************************************************************/
	//covmat = colvec * rowvec, covmat: m*m, colvec: m*1, rowvec: 1 * m, matrix is saved row by row.
	static void covMat(const float colVec[], const float rowVec[], float covMat[], const  int m);

	/************************************************************************/
	/*
	linear interpolation
	*/
	/************************************************************************/
	static int getInd(const int size[3], int x, int y, int z);
	//triple linear interpolation, in a x*y*z grid, with a given position, do trilinear interpolation inside the voxel.
	static float linearInterData(const float*& data, const int size[3], const double x, const double y, const double z);

	//compute new coordinate in the new coordinate system, old coord is in the canonical form
	static void getNewCoord(const float cen[3], const float vec[3][3], const float oldCoord[3],
		float newCoord[3]);
	static void getOldCoord(const float cen[3], const float vec[3][3], const float newCoord[3], float oldCoord[3]);

	/************************************************************************/
	/*
	spherical coordinates representation operations
	*/
	/************************************************************************/
	//	static void sphericalCoord(float center[ 3 ],float axis[ 9 ] )

	/************************************************************************/
	/*
	print functions
	*/
	/************************************************************************/
	static void printCOORD(const float pt[3]);

	//screen right and up vectors in the coord system after appying currentMat matrix.
	static void NewRotXY(float nx[3], float ny[3], const float currentMat[16]);

	/************************************************************************/
	/*
	memory functions
	*/
	/************************************************************************/
	//swap the array from [posI, posI + len-1] with the array [posJ, posJ + len -1]. Warning: no length check, no validity check!!

	template<typename T>
	static void swapArrayMemory(T* ptr, const int posI, const int posJ, const int len)
	{
		vector<T> tempV;
		tempV.resize(len, 0);
		memcpy(&(tempV[0]), ptr + posI, sizeof(T) * len);
		memcpy(ptr + posI, ptr + posJ, sizeof(T) * len);
		memcpy(ptr + posJ, &(tempV[0]), sizeof(T) * len);
	}

	/************************************************************************/
	/*
	template functions
	*/
	/************************************************************************/
	template<typename T>
	static void addVecTemplate(T* vecSrc, const T* vecDst, const int dim)
	{
		for (int i = 0; i < dim; i++)
		{
			vecSrc[i] = vecSrc[i] + vecDst[i];
		}
	}

	template<typename T>
	static void getVecTemplate(const T* ptStart, const T* ptEnd, T* resultVec, const int dim)
	{
		for (int i = 0; i < dim; i++)
		{
			resultVec[i] = ptEnd[i] - ptStart[i];
		}
	}

	//cross product of two vectors 
	template<typename T>
	static void crossProductNormalizeTemplate(const T vec1[3], const T vec2[3], T cp[3])
	{
		cp[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
		cp[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
		cp[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
		normalizeTemplate<T>(cp);
	}

	// return non-normalized cross product of vec1, vec2 and save it in cp	
	template<typename T>
	static void crossProductNotNormalizeTemplate(const T vec1[3], const T vec2[3], T cp[3])
	{
		cp[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
		cp[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
		cp[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
	}

	template<typename T>
	static void normalizeTemplate(T vec[3])
	{
		T len = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
		len = sqrt((T)len);
		if (len < 0.00001)
			return;
		for (int i = 0; i < 3; i++)
			vec[i] /= len;
	}

	template<typename T>
	static float dotProductTemplate(const T vec1[3], const T  vec2[3])
	{
		return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
	}

	template<typename T>
	static void stretchVecTemplate(T* vec, const T& ratio, const int dim)
	{
		for (int i = 0; i < dim; i++)
			vec[i] *= ratio;
	}

	template<typename T>
	static void stretchVecTemplate(T* vec, const T ratio[3], const int dim)
	{
		for (int i = 0; i < dim; i++)
			vec[i] *= ratio[i];
	}

	template<typename T>
	static void stretchVecTemplate(T* destVec, const T* srcVec, const T ratio[3], const int dim)
	{
		for (int i = 0; i < dim; i++)
			destVec[i] = srcVec[i] * ratio[i];
	}

	template<typename T>
	static void stretchVecTemplate(T* destVec, const T* srcVec, const T& ratio, const int dim)
	{
		for (int i = 0; i < dim; i++)
			destVec[i] = srcVec[i] * ratio;
	}


	/*template<typename T>
	static void addVecTemplate( T* sVec, const T* dVec, int dim )
	{
	for( int i = 0; i < dim; i++ )
	sVec[ i ] += dVec[ i ];
	}*/

	template<typename T>
	static void applyTransformationTemplate(T* pts, const int PtNum, const T rotMat[9], const T scaling[3], const T trans[3])
	{
		T pos[3], pos2[3];
		T* dataPTR = pts;
		for (int i = 0; i < PtNum; i++)
		{
			memcpy(pos, dataPTR, sizeof(T) * 3);
			for (int j = 0; j < 3; j++)
			{
				pos2[j] = dotProductTemplate<T>(rotMat + 3 * j, pos);
			}
			stretchVecTemplate<T>(pos2, scaling, 3);
			addVecTemplate<T>(pos2, trans, 3);
			memcpy(dataPTR, pos2, sizeof(T) * 3);
			dataPTR += 3;
		}
	}

	template<typename T>
	static void getOldCoord_With_NewCoordInNewCoordSystem(T* pts, const int PtNum, const T* nCenter, const T* nXDir, const T* nYDir, const T* nZDir)
	{
		T pos[3], vec[3];
		T* dataPTR = pts;
		const T* dirs[3] = { nXDir, nYDir, nZDir };
		for (int i = 0; i < PtNum; i++)
		{
			memcpy(pos, nCenter, sizeof(T) * 3);
			for (int j = 0; j < 3; j++)
			{
				stretchVecTemplate<T>(vec, dirs[j], dataPTR[j], 3);
				addVecTemplate<T>(pos, vec, 3);
			}
			memcpy(dataPTR, pos, sizeof(T) * 3);
			dataPTR += 3;
		}
	}

	template<typename T>
	static void getNewCoord_With_coordInOldCoordSystem(T* pts, const int PtNum, const T* nCenter, const T* nXDir, const T* nYDir, const T* nZDir)
	{
		T vec[3];
		T* dataPTR = pts;
		for (int i = 0; i < PtNum; i++)
		{
			getVecTemplate<T>(nCenter, dataPTR, vec, 3);
			dataPTR[0] = dotProductTemplate<float>(vec, nXDir);
			dataPTR[1] = dotProductTemplate<float>(vec, nYDir);
			dataPTR[2] = dotProductTemplate<float>(vec, nZDir);

			dataPTR += 3;
		}
	}
};

#endif