#include "Spline3D.h"


Spline3D::Spline3D()
{
}


Spline3D::~Spline3D()
{
}

void Spline3D::fittingCurve(double fit_ratio, bool autoAdjust){
	if (m_piecewisePoints().empty())return;

	HSSSpline::HSplineCore<3>::BuildingSpline(m_piecewisePoints);
	m_ctrlPoints().clear();
	m_length = this->length();

	if (autoAdjust)
	{
		if (m_length<50)
		{
			fit_ratio = 0.5;
		}
		else if (m_length<150)
		{
			fit_ratio = 0.2;
		}
		else
		{
			fit_ratio /= (int)(1 + m_length / 500);
		}
	}

	getUniformSamplePoints(m_ctrlPoints, fit_ratio*m_length);
	HSSSpline::HSplineCore<3>::BuildingSpline(m_ctrlPoints);
}
void Spline3D::fittingCurve(double fixedFitRatio){
	if (m_piecewisePoints().empty())return;

	HSSSpline::HSplineCore<3>::BuildingSpline(m_piecewisePoints);
	m_ctrlPoints().clear();
	m_length = this->length();

	getUniformSamplePoints(m_ctrlPoints, fixedFitRatio);
	HSSSpline::HSplineCore<3>::BuildingSpline(m_ctrlPoints);
}
void Spline3D::getUniformSamplePoints(HSSSpline::PathPoints<3> &points, double perLen){
	double temp_len = 0;

	points().push_back(HSSSpline::HSplineCore<3>::get_point(0, 0));
	for (int i = 0; i<m_LineSeg_List.size(); i++)
	{
		double step = 0.01;
		for (double k = 0; k<1; k += step)
		{
			HSSSpline::PathPoint<3> p0 = HSSSpline::HSplineCore<3>::get_point(i, k);
			HSSSpline::PathPoint<3> p1;
			if (k + step <= 1){ p1 = HSSSpline::HSplineCore<3>::get_point(i, k + step); }
			else { p1 = HSSSpline::HSplineCore<3>::get_point(i, 1); }
			double delta[3] = { 
				p1[0] - p0[0], 
				p1[1] - p0[1], 
				p1[2] - p0[2] };
			double len = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2]*delta[2]);
			temp_len += len;

			if (abs(temp_len - perLen)<0.01)
			{
				points().push_back(p1);
				temp_len = 0;
			}
			else{
				if (temp_len > perLen)
				{
					temp_len -= len;
					step /= 10;
				}
			}
		}
	}

	if (points().size() <= 1){
		points().push_back(get_point(m_LineSeg_List.size() - 1, 1));
	}

	if (temp_len>perLen*0.5)
	{
		points().push_back(get_point(m_LineSeg_List.size() - 1, 1));
	}
}
void Spline3D::UniformSampling(HSSSpline::Samples &samples, double perLen){
	
	HSSSpline::Sample sample_0;
	sample_0.seg_idx = 0;
	sample_0._t = 0;
	samples.push_back(sample_0);

	double temp_len = 0;
	for (int i = 0; i<m_LineSeg_List.size(); i++)
	{
		double step = 1 / this->segLength(i);
		for (double k = 0; k<1; k += step)
		{
			HSSSpline::PathPoint<3> p0 = this->getPosition(i, k);
			HSSSpline::PathPoint<3> p1;
			if (k + step <= 1){ p1 = this->getPosition(i, k + step); }
			else { p1 = this->getPosition(i, 1); }

			double delta[3] = { 
				p1[0] - p0[0], 
				p1[1] - p0[1], 
				p1[2] - p0[2] };
			temp_len += sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);

			if (temp_len >= perLen)
			{
				HSSSpline::Sample sample;
				sample.seg_idx = i;
				sample._t = k + step;
				samples.push_back(sample);
				temp_len = 0;
			}
		}
	}	
}
double Spline3D::length(){
	double len = 0;
	int size = HSSSpline::HSplineCore<3>::m_LineSeg_List.size();
	for (int i = 0; i < size; i++){
		//std::cout << i << std::endl;
		len += this->segLength(i);
	}
	return len;
}
double Spline3D::segLength(int segIdx){
	double len = 0;
	double step = 0.01;
	for (double k = 0; k<1; k += step)
	{
		HSSSpline::PathPoint<3> p0 = this->getPosition(segIdx, k);
		HSSSpline::PathPoint<3> p1 = this->getPosition(segIdx, k + step);
		
		double delta[3] = { 
			p1[0] - p0[0], 
			p1[1] - p0[1], 
			p1[2] - p0[2] };
		len += sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
	}
	return len;
}
HSSSpline::PathPoint<3> Spline3D::getPosition(int _seg, double _t){
	HSSSpline::PathPoint<3> position;
	position[0] = this->getX(_seg, _t);
	position[1] = this->getY(_seg, _t);
	position[2] = this->getZ(_seg, _t);
	return position;
}
void Spline3D::getLineGeometry(std::vector<float>& data, double step){
	HSSSpline::Sample begin(0, 0);
	HSSSpline::Sample end(HSSSpline::HSplineCore<3>::m_LineSeg_List.size() - 1, 1);

	this->getLineGeometry(data, begin, end, 0.1);
}
void Spline3D::getLineGeometry(float **data, int *count){
	HSSSpline::Sample begin(0, 0);
	HSSSpline::Sample end(HSSSpline::HSplineCore<3>::m_LineSeg_List.size() - 1, 1);
	
	// Create vertex buffer
	std::vector<float> vertexBuffer;
	this->getLineGeometry(vertexBuffer, begin, end, 0.1);

	// Create float array
	float *buffer = new float[vertexBuffer.size()];
	for (int i = 0; i < vertexBuffer.size(); i++)
		buffer[i] = vertexBuffer.at(i);

	*data = buffer;
	*count = vertexBuffer.size();
}
void Spline3D::getLineGeometry(float *data, int *count, double step){
	HSSSpline::Sample begin(0, 0);
	HSSSpline::Sample end(HSSSpline::HSplineCore<3>::m_LineSeg_List.size() - 1, 1);

	// Create vertex buffer
	std::vector<float> vertexBuffer;
	this->getLineGeometry(vertexBuffer, begin, end, step);

	// Create float array
	for (int i = 0; i < vertexBuffer.size(); i++)
		data[i] = vertexBuffer.at(i);

	*count = vertexBuffer.size();
}
void Spline3D::getLineGeometry(std::vector<float>& vertexBuffer, HSSSpline::Sample &from, HSSSpline::Sample &to, double step){
	for (int i = from.seg_idx; i <= to.seg_idx; i++)
	{
		double k = 0;
		if (i == from.seg_idx)
			k = from._t;
		double end = 1;
		if (i == to.seg_idx)
			end = to._t;

		while (k<end)
		{
			HSSSpline::PathPoint<3> p0 = this->getPosition(i, k);
			k += step;
			if (k>end)k = end;
			HSSSpline::PathPoint<3> p1 = getPosition(i, k);

			vertexBuffer.push_back(p0[0]);
			vertexBuffer.push_back(p0[1]);
			vertexBuffer.push_back(p0[2]);

			vertexBuffer.push_back(p1[0]);
			vertexBuffer.push_back(p1[1]);
			vertexBuffer.push_back(p1[2]);
		}
	}	
}
void Spline3D::getLineVertex(float *data, int *count, double step){
	HSSSpline::Sample begin(0, 0);
	HSSSpline::Sample end(HSSSpline::HSplineCore<3>::m_LineSeg_List.size() - 1, 1);

	// Create vertex buffer
	std::vector<float> vertexBuffer;
	this->getLineVertex(vertexBuffer, begin, end, step);

	// Create float array
	for (int i = 0; i < vertexBuffer.size(); i++)
		data[i] = vertexBuffer.at(i);

	*count = vertexBuffer.size();
}
void Spline3D::getLineVertex(std::vector<float>& vertexBuffer, HSSSpline::Sample &from, HSSSpline::Sample &to, double step){
	for (int i = from.seg_idx; i <= to.seg_idx; i++)
	{
		double k = 0;
		if (i == from.seg_idx)k = from._t;
		double end = 1;
		if (i == to.seg_idx)end = to._t;

		while (k<end)
		{
			HSSSpline::PathPoint<3> p0 = this->getPosition(i, k);
			k += step;
			if (k>end)k = end;
			
			vertexBuffer.push_back(p0[0]);
			vertexBuffer.push_back(p0[1]);
			vertexBuffer.push_back(p0[2]);			
		}	
	}
}

void Spline3D::getCurveSegmentData(std::vector<float> &vertex, std::vector<float> &derivative, std::vector<float> &secondDerivative, double step){
	HSSSpline::Sample begin(0, 0);
	HSSSpline::Sample end(HSSSpline::HSplineCore<3>::m_LineSeg_List.size() - 1, 1);

	// Create vertex buffer
	this->getCurveSegmentData(vertex, derivative, secondDerivative, begin, end, step);
}
void Spline3D::getCurveSegmentData(std::vector<float> &vertex, std::vector<float> &derivative, std::vector<float> &secondDerivative, HSSSpline::Sample &from, HSSSpline::Sample &to, double step){
	for (int i = from.seg_idx; i <= to.seg_idx; i++)
	{
		double k = 0;
		if (i == from.seg_idx)k = from._t;
		double end = 1;
		if (i == to.seg_idx)end = to._t;

		
		while (k<end)
		{
			float d0[3];
			float sd0[3];
			HSSSpline::PathPoint<3> p0 = this->getPosition(i, k);
			this->getDerivative(d0, i, k);
			this->getSecondDerivative(sd0, i, k);

			vertex.push_back(p0[0]);
			vertex.push_back(p0[1]);
			vertex.push_back(p0[2]);

			derivative.push_back(d0[0]);
			derivative.push_back(d0[1]);
			derivative.push_back(d0[2]);

			secondDerivative.push_back(sd0[0]);
			secondDerivative.push_back(sd0[1]);
			secondDerivative.push_back(sd0[2]);
			
			k += step;
			if (k>end)k = end;

			float d1[3];
			float sd1[3];
			HSSSpline::PathPoint<3> p1 = this->getPosition(i, k);
			this->getDerivative(d1, i, k);
			this->getSecondDerivative(sd1, i, k);

			vertex.push_back(p1[0]);
			vertex.push_back(p1[1]);
			vertex.push_back(p1[2]);

			derivative.push_back(d1[0]);
			derivative.push_back(d1[1]);
			derivative.push_back(d1[2]);

			secondDerivative.push_back(sd1[0]);
			secondDerivative.push_back(sd1[1]);
			secondDerivative.push_back(sd1[2]);			
		}
		
		/*
		if (k >= end){
			// Add terminal
			float stangent[3];
			float sbinormal[3];
			HSSSpline::PathPoint<3> p0 = this->getPosition(i, end);
			this->getTangent(stangent, i, end);
			this->getBinormal(sbinormal, i, end);
						
			vertex.push_back(p0[0]);
			vertex.push_back(p0[1]);
			vertex.push_back(p0[2]);

			tangent.push_back(stangent[0]);
			tangent.push_back(stangent[1]);
			tangent.push_back(stangent[2]);

			binormal.push_back(sbinormal[0]);
			binormal.push_back(sbinormal[1]);
			binormal.push_back(sbinormal[2]);
		}
		*/
	}
}

void Spline3D::getDerivative(float *d, int seg, double t){
	d[0] = HSSSpline::HSplineCore<3>::get_D1_value(0, seg, t);
	d[1] = HSSSpline::HSplineCore<3>::get_D1_value(1, seg, t);
	d[2] = HSSSpline::HSplineCore<3>::get_D1_value(2, seg, t);	
}
void Spline3D::getSecondDerivative(float *sd, int seg, double t){
	// Binormal = Second Derivative ??
	sd[0] = HSSSpline::HSplineCore<3>::get_D2_value(0, seg, t);
	sd[1] = HSSSpline::HSplineCore<3>::get_D2_value(1, seg, t);
	sd[2] = HSSSpline::HSplineCore<3>::get_D2_value(2, seg, t);	
}
// 170119, providing ctrl point data
int Spline3D::getCtrlPoints(float *data){
	int size = m_ctrlPoints.val.size();
	for (int i = 0; i < size; i++){
		int offset = i * 3;

		data[offset + 0] = m_ctrlPoints[i][0];
		data[offset + 1] = m_ctrlPoints[i][1];
		data[offset + 2] = m_ctrlPoints[i][2];
	}
	
	return size;
}
// 170119, Edit curve with changing ctrl point
HSSSpline::PathPoints<3> & Spline3D::getCtrlPoints(){
	return m_ctrlPoints;
}
void Spline3D::reFitCurve(){
	HSSSpline::HSplineCore<3>::BuildingSpline(m_ctrlPoints);
}