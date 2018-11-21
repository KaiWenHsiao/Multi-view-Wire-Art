#pragma once

#include <string>
#include <fstream>

#include <glm\gtc\type_ptr.hpp>
#include <Eigen\Dense>

#include "Source.h"
#include "Spline\Spline3D.h"

#include "LineWidget.h"
#include "ProgressTestSender.h"



class LineDeformer
{
private:
	Source *m_source;
public:
	LineDeformer(Source *s);
	virtual ~LineDeformer();

	

	void initCurve(const std::vector<float*> &segs, const std::vector<int> &ptNums, std::vector<Spline3D*> &curves, std::vector<float> &firstLastLock);
	void initDeformProcess(const std::vector<Spline3D*> &curves, const std::vector<float> &firstLastLock);
	void deform_180125(std::vector<Spline3D*> &curves, std::vector<float> &firstLastLock, const std::string &processedFileName, bool outputProcessedSS);

private:
	
	void getParametricCurves(const vector<float*> &segments, const vector<int> &ptNum, vector<Spline3D*> &curves, std::vector<float> &firstLastLock, float fitRatio);

public:
	struct CurveStructure{
		float similarity;
		std::vector<float*> curveCtrls;
		std::vector<int> numCurveCtrl;
		std::vector<float> firstLastLock;
	};


private:
	unsigned char **m_adjacencyMat;
	int m_adjacencyMatSize;
	std::vector<Spline3D*> m_curves;

	glm::vec3 getDeformVector_180124(float *src);

private:
	std::vector<float> minimumDeformMagnitudes;
	std::vector<float> maximumDeformMagnitudes;

	float getDeformVectorWeight(float x);

	CurveStructure *m_buffer0;
	CurveStructure *m_buffer1;
	bool getDeformedStructure_180125(const std::vector<Spline3D*> &curves, const std::vector<float> &firstLastLock, CurveStructure *res, float translateRatio, bool requireReFit);
	void setUpInitialCurveStructure(const std::vector<Spline3D*> &curves, const std::vector<float> &firstLastLock);

	std::vector<float> brokePartSimilarity;
	std::vector<float> redundantPartSimilarity;

public:
	float m_sigma = 1.0f;
	float m_dtWeight = 1.0f;

private:
	float *m_sampleBuffer;
	void outputCurveSamples(const std::string &fileName, const std::vector<Spline3D*> &curves);

public:
	static void getCurveSamples(vector<float*> &segments, vector<int> &ptNum, const std::vector<Spline3D*> &curves, const std::vector<float> &firstLastLock);

};

