#include "mymath.h"

int MyMath::getSign(float val)
{
	if (val > 0)
		return 1;
	if (val == 0)
		return 0;
	return -1;
}
void MyMath::getPtOnRay(float pt[3], float dir[3], float t, float endpt[3])
{
	for (int i = 0; i < 3; i++)
	{
		endpt[i] = pt[i] + dir[i] * t;
	}

}
bool MyMath::isEqualInToler(float num1, float num2, float toler)
{
	if (abs(num1 - num2) < toler)
		return true;
	return false;
}

float MyMath::vectorLen(float x, float y, float z)
{
	return sqrt(x*x + y*y + z* z);
}
//represented in array
float MyMath::dotProduct(const float vec1[3], const float vec2[3])
{
	float dotp = 0;
	for (int i = 0; i < 3; i++)
		dotp += (vec1[i] * vec2[i]);
	return dotp;
}
//compute the cos of angle pt1 pt0 pt2. angle between vector pt0pt1 and pt0pt2
float MyMath::getCosOfAngle(float pt1[3], float pt2[3], float pt0[3])
{
	float vec1[3], vec2[3];
	getVec(pt0, pt1, vec1);
	getVec(pt0, pt2, vec2);
	//////////////////////////////////////////////////////////////////////////
	//	cout<<"length:"<<vectorlen(vec1)<<"  "<<vectorlen(vec2)<<endl;
	return (dotProduct(vec1, vec2) / (vectorlen(vec1)*vectorlen(vec2)));
}
//get vector between two poitns, and save the vector into vec
void MyMath::getVec(const float pt1[3], const float pt2[3], float vec[3])
{
	for (int i = 0; i < 3; i++)
		vec[i] = pt2[i] - pt1[i];
}
void MyMath::getNeg(float vec[3])
{
	for (int i = 0; i < 3; i++)
		vec[i] = -vec[i];
}
void MyMath::getPtOnSeg(float pt1[3], float pt2[2], float ratio, float pt[3])
{
	for (int i = 0; i < 3; i++)
	{
		pt[i] = (1 - ratio)*pt1[i] + ratio*pt2[i];
	}
}
void MyMath::getEndPtOfSeg(const float ept[3], const float vec[3], float resultpt[3])
{
	for (int i = 0; i< 3; i++)
		resultpt[i] = ept[i] + vec[i];
}
//compute the length of the vector between the two points
float MyMath::vectorlen(float pt1[3], float pt2[3])
{
	float len = 0;
	for (int i = 0; i < 3; i++)
		len += pow((pt2[i] - pt1[i]), 2);
	return sqrt(len);
}
//compute the vector's length
float MyMath::vectorlen(float vec[3])
{
	float len = 0;
	for (int i = 0; i < 3; i++)
		len = len + vec[i] * vec[i];
	len = sqrt(len);
	return len;
}
//normalize the svec, and keep it unchanged, save the normalized value into dvec
void MyMath::normalize(float svec[3], float dvec[3])
{
	float len = vectorlen(svec);
	for (int i = 0; i < 3; i++)
		dvec[i] = svec[i] / len;
}
//normalize vec, and save the normalized value into vec
void MyMath::normalize(float vec[3])
{
	float len = vectorlen(vec);
	for (int i = 0; i < 3; i++)
		vec[i] = vec[i] / len;
}
//compute center of two points p1, and p2, save it into center
void MyMath::center(float p1[3], float p2[3], float center[3])
{
	for (int i = 0; i < 3; i++)
		center[i] = (p1[i] + p2[i]) / 2;
}
//compute the relative position to center, and unit lenght is len.
void MyMath::getrelativepos(float pt[3], const float center[3], const float len)
{
	for (int i = 0; i < 3; i++)
	{
		pt[i] = (pt[i] - center[i]) / len;
	}
}

void MyMath::getOriginPos(float pt[3], const float center[3], const float unitlen)
{
	for (int i = 0; i < 3; i++)
	{
		pt[i] = (pt[i] * unitlen + center[i]);
	}
}

/**
* return normalized crossproduct of vec1 and vec2  and save it into cp
*/
void MyMath::crossProduct(float vec1[3], float vec2[3], float cp[3])
{
	cp[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	cp[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
	cp[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
	normalize(cp);
}
/**
* return non-normalized cross product of vec1, vec2 and save it in cp
*/
void MyMath::crossProductNotNorm(float vec1[3], float vec2[3], float cp[3])
{
	cp[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	cp[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
	cp[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

//a naive algorithm to compute the difference for vertex sver to all the vertices in dver.
float MyMath::computeDiff(float* sver, float* dver, int dvernum)
{
	float minval = vectorlen(sver, dver);
	float val;
	for (int i = 1; i < dvernum; i++)
	{
		val = vectorlen(sver, dver + 3 * i);
		if (val < minval)
			minval = val;
	}
	return minval;
}

void MyMath::stretchVec(float vec[3], const float ratio[3])
{
	for (int i = 0; i < 3; i++)
		vec[i] *= ratio[i];
}

void MyMath::stretchVec(float vec[3], const float ratio)
{
	for (int i = 0; i < 3; i++)
		vec[i] *= ratio;
}

void MyMath::addVec(float dvec[3], const float svec[3])
{
	for (int i = 0; i < 3; i++)
		dvec[i] += svec[i];
}

//another naive algorithm to compute the difference for vertex sver to another mesh, dvers, dtrians

//dist: save the resulting distance from the point to the line segment lies on
//dist = the distance from the point to the line the segment lies on , if the perpendicular line from sver crosses the seg
//dist = the minimal distance from the point to one of the two endpoints of the segment, otherwise
void MyMath::computeDistPt2Seg(float* sver, float* dver, int ind[2], float& dist)
{
	float vec[3];
	getVec(&dver[3 * ind[0]], &dver[3 * ind[1]], vec);
	float veclen = vectorlen(vec);

	//degenerate case
	if (veclen < 0.001)
	{
		getVec(sver, &dver[3 * ind[0]], vec);
		dist = vectorlen(vec);
		return;
	}

	stretchVec(vec, 1 / veclen);
	float vec2[3];
	getVec(&dver[3 * ind[0]], sver, vec2);
	float projlen = dotProduct(vec, vec2);
	bool result = true;
	if (projlen < 0 || projlen > veclen)
		result = false;

	if (result)
	{
		float vec2len = vectorlen(vec2);
		dist = sqrt(vec2len * vec2len - projlen*projlen);
		return;
	}

	dist = vectorlen(sver, &dver[ind[0] * 3]);
	float tdist = vectorlen(sver, &dver[ind[1] * 3]);
	if (tdist < dist)
		dist = tdist;
}

//compute distance from point to a triangle 
//dist = distance from the point to the plane the triangle is on, if the perpendicular line crosses trian
//dist = nearest distance from the point to the three segments
void MyMath::computeDistPt2Trian(float* sver, float* dver, int ind[3], float& dist)
{
	bool result = true;

	//decide if perpendicular line crosses triangle or not
	float tvec1[3], tvec2[3];
	float vecnorm[3][3];
	for (int i = 0; i < 3; i++)
	{
		getVec(sver, &dver[3 * ind[i]], tvec1);
		getVec(sver, &dver[3 * ind[(i + 1) % 3]], tvec2);
		crossProductNotNorm(tvec1, tvec2, vecnorm[i]);
	}
	float dotp;
	for (int i = 0; i < 3; i++)
	{
		dotp = dotProduct(vecnorm[i], vecnorm[(i + 1) % 3]);
		if (dotp < 0)
		{
			result = false;
			break;
		}
	}

	//compute the distance from the point to the plane which triangle lies in.
	float norm[3];
	getVec(&dver[3 * ind[0]], &dver[3 * ind[1]], tvec1);
	getVec(&dver[3 * ind[0]], &dver[3 * ind[2]], tvec2);
	crossProductNotNorm(tvec1, tvec2, norm);
	float len = vectorlen(norm);
	//degenerate case - two segments lie on the same line
	if (len < 0.000001)
	{
		result = false;
	}
	stretchVec(norm, 1 / len);

	if (result)	//compute the distance from the point to the intersection point and return
	{
		getVec(sver, &dver[3 * ind[0]], tvec2);
		dist = dotProduct(tvec2, norm);
		if (dist < 0) dist = -dist;
		return;
	}

	//compute the distance from the point to the three segments and select the minimal
	float tdist;
	int tind[2];
	for (int i = 0; i < 3; i++)
	{
		tind[0] = ind[i];
		tind[1] = ind[(i + 1) % 3];
		computeDistPt2Seg(sver, dver, tind, tdist);
		if (i == 0)
			dist = tdist;
		else
		{
			if (tdist < dist)
				dist = tdist;
		}
	}
}


float MyMath::computeDiff(float* sver, float* dver, int* dtrians, int triannum)
{
	//go through all the triangles
	bool succ = true;
	float dist, mdist;
	for (int i = 0; i < triannum; i++)
	{
		computeDistPt2Trian(sver, dver, &dtrians[3 * i], dist);
		if (i == 0)
		{
			mdist = dist;
		}
		else
		{
			if (dist < mdist)
				mdist = dist;
		}
	}
	return mdist;
}

void MyMath::perpendicularVec(const float vec[3], float perVec[3])
{
	int maxInd = (abs(vec[0]) > abs(vec[1])) ? 0 : 1;
	maxInd = (abs(vec[maxInd])  > abs(vec[2])) ? maxInd : 2;

	if (maxInd < 2)
	{
		perVec[0] = -vec[1];
		perVec[1] = vec[0];
		perVec[2] = 0;
	}
	else
	{
		perVec[0] = 0;
		perVec[1] = -vec[2];
		perVec[2] = vec[1];
	}
}

int MyMath::getInd(const int size[3], int x, int y, int z)
{
	return x * size[1] * size[2] + y * size[2] + z;
}

void MyMath::addVec_X(float dvec[], const int m, const float sdvec[])
{
	for (int i = 0; i < m; i++)
	{
		dvec[i] += sdvec[i];
	}
}

void MyMath::covMat(const float colVec[], const float rowVec[], float covMat[], const int m)
{
	int pos = 0;
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < m; j++)
		{
			covMat[pos] = colVec[i] * rowVec[j];
			pos++;
		}
	}
}

float MyMath::linearInterData(const float*& data, const int size[3], const double x, const double y, const double z)
{
	double rvalue;
	int hx = (int)ceil(x);
	int lx = (int)floor(x);
	int hy = (int)ceil(y);
	int ly = (int)floor(y);
	int hz = (int)ceil(z);
	int lz = (int)floor(z);

	double x1 = x - lx, x2 = 1 - x1;
	double r1 = x2 * data[getInd(size, lx, ly, lz)] + x1 * data[getInd(size, hx, ly, lz)];
	double r2 = x2 * data[getInd(size, lx, ly, hz)] + x1 * data[getInd(size, hx, ly, hz)];
	double r3 = x2 * data[getInd(size, lx, hy, lz)] + x1 * data[getInd(size, hx, hy, lz)];
	double r4 = x2 * data[getInd(size, lx, hy, hz)] + x1 * data[getInd(size, hx, hy, hz)];

	double y1 = y - ly, y2 = 1 - y1;
	double r5 = y2 * r1 + y1 * r3;
	double r6 = y2 * r2 + y1 * r4;

	double z1 = z - lz, z2 = 1 - z1;
	rvalue = z2 * r5 + z1 * r6;

	return rvalue;
}

void MyMath::getNewCoord(const float cen[3], const float vec[3][3], const float oldCoord[3],
	float newCoord[3])
{
	float nvec[3];
	getVec(cen, oldCoord, nvec);
	for (int i = 0; i < 3; i++)
	{
		//compute the dotproduct
		newCoord[i] = dotProduct(nvec, vec[i]);
	}
}

void MyMath::getOldCoord(const float cen[3], const float vec[3][3], const float newCoord[3], float oldCoord[3])
{
	memcpy(oldCoord, cen, sizeof(float) * 3);
	for (int i = 0; i < 3; i++)	//3 direction
	{

		for (int j = 0; j < 3; j++)	//xyz
		{
			oldCoord[j] += newCoord[i] * vec[i][j];
		}
	}
}

void MyMath::printCOORD(const float pt[3])
{
	cout << " (" << pt[0] << ", "
		<< pt[1] << ", "
		<< pt[2] << ") ";
}


//for a coordinate system, after applying the rotation matrix, what are the
//right vector and up vector on the screen in the coord system.
void MyMath::NewRotXY(float nx[3], float ny[3], const float currentMat[16])
{
	float vx[3] = { 1,0,0 }, vy[3] = { 0,1,0 };
	for (int i = 0; i < 3; i++)
	{
		nx[i] = 0;
		ny[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			nx[i] += vx[j] * currentMat[4 * i + j];
			ny[i] += vy[j] * currentMat[4 * i + j];
		}
	}
	// normalize
	float magx = sqrt(nx[0] * nx[0] + nx[1] * nx[1] + nx[2] * nx[2]);
	float magy = sqrt(ny[0] * ny[0] + ny[1] * ny[1] + ny[2] * ny[2]);
	nx[0] /= magx; nx[1] /= magx; nx[2] /= magx;
	ny[0] /= magy; ny[1] /= magy; ny[2] /= magy;
}



