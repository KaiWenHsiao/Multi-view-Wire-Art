#pragma once

#include "SplineSample.h"
#include "HSplineCore.h"

#include <iostream>


class Spline3D : public HSSSpline::HSplineCore<3>
{
protected:
	// Original path point
	HSSSpline::PathPoints<3> m_piecewisePoints;
	HSSSpline::PathPoints<3> m_ctrlPoints;

public:
	Spline3D();
	~Spline3D();
	double m_length;

	void assignPoints(HSSSpline::PathPoints<3> &points){
		m_piecewisePoints.val.assign(points.val.begin(), points.val.end());
	}
	void fittingCurve(double fit_ratio, bool autoAdjust);
	void UniformSampling(HSSSpline::Samples &samples, double perLen);
	void getUniformSamplePoints(HSSSpline::PathPoints<3> &points, double perLen);


	double length();

	// Get Information
	HSSSpline::PathPoint<3> getPosition(int _seg, double _t);
	double getX(int seg, double t){
		return HSSSpline::HSplineCore<3>::get_value(0, seg, t);
	}
	double getY(int seg, double t){
		return HSSSpline::HSplineCore<3>::get_value(1, seg, t);
	}
	double getZ(int seg, double t){
		return HSSSpline::HSplineCore<3>::get_value(2, seg, t);
	}

	// Render content	
	void getLineVertex(float *data, int *count, double step);
	void getLineGeometry(float *data, int *numberCount, double step);
	void getLineGeometry(std::vector<float>& data, double step);
	void getLineGeometry(std::vector<float>& vertexBuffer, HSSSpline::Sample &from, HSSSpline::Sample &to, double step);
	
	void getLineGeometry(float **data, int *count);
	void getLineVertex(std::vector<float>& vertexBuffer, HSSSpline::Sample &from, HSSSpline::Sample &to, double step);

	void getCurveSegmentData(std::vector<float> &vertex, std::vector<float> &derivative, std::vector<float> &secondDerivative, double step);
	void getCurveSegmentData(std::vector<float> &vertex, std::vector<float> &derivative, std::vector<float> &secondDerivative, HSSSpline::Sample &from, HSSSpline::Sample &to, double step);
	void getDerivative(float *d, int seg, double t);
	void getSecondDerivative(float *sd, int seg, double t);

	// 170119, providing ctrl point data, return number of vertices
	int getCtrlPoints(float *data);
	
	// 170119, Edit curve with changing ctrl point
	HSSSpline::PathPoints<3> & getCtrlPoints();
	void reFitCurve();

protected:
	double segLength(int segIdx);

public:
	void fittingCurve(double fixedFitRatio);
};

