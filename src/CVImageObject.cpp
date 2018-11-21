#include "CVImageObject.h"


CVImageObject::CVImageObject(const std::string &imgFile)
{
	m_originImage = cv::imread(imgFile);
	m_imgWidth = m_originImage.cols;
	m_imgHeight = m_originImage.rows;

	cv::cvtColor(m_originImage, m_originImage8U, CV_RGB2GRAY);
	cv::distanceTransform(m_originImage8U, m_distanceTransform, CV_DIST_L2, 3);

}


CVImageObject::~CVImageObject()
{
}

void CVImageObject::proj(float *v, int *pixel){
	glm::vec4 wv = { v[0], v[1], v[2], 1 };
	glm::vec3 p = m_KRT * wv;
	
	if (p.z < 0){
		pixel[0] = -1;
		pixel[1] = -1;
		return;
	}

	if (glm::abs(p.z) < 0.0001){
		
	}
	else{
		p = p / p.z;
	}
	

	// Change to Pixel-Index-Coordinate-System
	//p = p + glm::vec3(m_imgCenter[1], m_imgCenter[0], 0);
	
	pixel[0] = (int)floor(p.x);
	pixel[1] = (int)floor(p.y);	
}
void CVImageObject::addPoint(float *v){
	int pixel[2];
	this->proj(v, pixel);

	cv::circle(m_originImage, cv::Point(pixel[0], pixel[1]), 5, cv::Scalar(255, 0, 0), -1);
}
void CVImageObject::addLine(float *v0, float *v1, const cv::Scalar &color){
	int p0[2], p1[2];
	this->proj(v0, p0);
	this->proj(v1, p1);

	cv::line(m_originImage, cv::Point(p0[0], p0[1]), cv::Point(p1[0], p1[1]), color, 5);
}
bool CVImageObject::isMapToForeground(float *v){
	int pixel[2];
	this->proj(v, pixel);

	if (pixel[0] < 0 || pixel[0] >= m_imgWidth || pixel[1] < 0 || pixel[1] >= m_imgHeight){
		return false;
	}

	const cv::Vec3b &color = m_originImage.at<cv::Vec3b>(pixel[1], pixel[0]);
	if (m_distanceTransform.at<float>(pixel[1], pixel[0]) <= 5){
		return true;
	}
	return false;
}
