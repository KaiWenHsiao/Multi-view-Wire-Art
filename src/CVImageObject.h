#pragma once

#include <glm\mat4x4.hpp>
#include <glm\gtc\matrix_transform.hpp>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

class CVImageObject
{
public:
	CVImageObject(const std::string &img);
	virtual ~CVImageObject();

	cv::Mat m_originImage;
	cv::Mat m_originImage8U;
	cv::Mat m_distanceTransform;

	glm::ivec2 m_imgCenter;
	glm::mat4x3 m_RT;
	glm::mat3x3 m_K;
	glm::mat4x3 m_KRT;

	int m_imgWidth;
	int m_imgHeight;

	void proj(float *v, int *pixel);
	void addPoint(float *v);	
	void addLine(float *v0, float *v1, const cv::Scalar &color);
	bool isMapToForeground(float *v);
};

