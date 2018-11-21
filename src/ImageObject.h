#pragma once

#include <glm\mat4x4.hpp>
#include <glm\gtc\matrix_transform.hpp>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

#include <vector>

#include <fstream>

#include "Ray.h"
#include "HKImgThinning.h"
#include "MyDataStructure.h"

#include "ImageGraph.h"
#include "PixelConnectedComponent.h"

#include "ProgressTestSender.h"



class ImageObject
{
public:
	int m_imageHeightConstraint;
	void clearComputationMaps();

// Definition
private:
	const static int CONTOUR_PIXEL_THRESHOLD = 100;
	bool backgroundPixel(int w, int h);
	
	// 0: perspective, 1: ortho
	int m_projectionMode;

// Public Information
public:
	std::string m_log;
	int m_imageWidth;
	int m_imageHeight;

// Map
public:
	cv::Mat m_originImage;
	cv::Mat m_thinContour;
	cv::Mat m_distanceTransformVerThin;
	cv::Mat m_distanceTransformVerThinForDisplay;
	cv::Mat m_remainMap;
	

	void renderCoverMap(float *seg, int ptNum, float *color, const std::string fileName);

// Pre processing
private:
	bool getOptimizedImage(const std::string& file);
	bool setUpThinningContour();

// View Frustum
public:
	float m_nearPlaneWidth;
	float m_nearPlaneHeight;
	float m_near;
	float m_far;
	float m_xmf;
	float m_ymf;
	glm::vec3 m_viewPos;
	glm::vec3 m_z;
	glm::vec3 m_y;
	glm::vec3 m_x;	
	glm::vec3 m_nearPlaneCenter;

public:
	std::vector<Ray*> m_rays;
	glm::mat4 m_viewMat;
	glm::mat4 m_viewProjMat;


// Projection
private:
	void proj(float *vertex, int *pixel);


public:
	ImageObject();
	virtual ~ImageObject();

	bool loadImage(const std::string& file, const std::string& name, bool requireGraph, int radius);
	void setNearPlaneWH(float planeWidth, float planeHeight);
	void setUpViewFrustrum(glm::vec3 viewPos, glm::vec3 viewLookat, const glm::vec3 &upVector, float fovy, float near, float far);
	void createRay();	

	// When Selecting Voxel
	bool getToleranceProj(float *v, int *pixel, int tolerance);
	void voxelCover(float *v, int sampleRadius, int color);
	
	float getDistanceTransformVerThin(float *v);

	void ImageObject::getNearestPixelRay(Ray *r, float *vertex);

// Ortho Projection
public:
	void setUpOrthoViewFrustrum(glm::vec3 viewPos, glm::vec3 viewLookat, const glm::vec3 &upVector, float left, float right, float bottom, float up, float near, float far);
	void createOrthoRay();

// With new structure fixing
public:
	void getRay(Ray *r, int x, int y);

// With new connecting
private:
	cv::Mat m_lineDTMask;
public:
	float getLineDistanceTransform(float *v0, float *v1, const int w);
public:
	ImageGraph *m_graph;
	float getDistanceTransformNorm(float *v);
	float getVoxelPathDistanceTransform(const std::vector<glm::vec3> &voxels, int radius);

public:
	void getClipPlaneGeometry(float *vertices, float z);
	void getImage(float *content, float alpha);

private:
	cv::Mat m_distanceTransformVerPathFinding;
	cv::Mat m_pathFindingMap;

public:
	void coverToPathFindingImage(float *v, int radius);
	void setUpPathFindingDistanceTransformMap();
	float getDistanceTransformVerPathFinding(float *v);

//////////////////////////////////////////////////////////
public:
	cv::Mat m_originImage8U;

public:

	void getNearestThinContourPixel(float *v, int *pixel);
	glm::vec3 getWorldSpaceVertex(int *pixel);
	
//////////////////////////////////////////////////////////
public:
	bool loadImage(const std::string& file, const std::string &pixelWeight, const std::string& name, bool requireGraph, int radius);
public:
	cv::Mat m_pixelWeightMap;
private:
	cv::Mat m_originPixelWeightMap;
	bool getOptimizedImage(const std::string& file, const std::string &pixelWeight);

public:
	void getPatchPixels(float *vertices, int numVertex, int radius, PixelSet *ps);

public:
	bool isProjectedIn2DConstraintRegion(float *v);

public:
	bool renewImage(const cv::Mat &img, int radius);
	cv::Mat m_originDistanceTransformMap;
	float getOriginDistanceTransform(float *v);

/////////////////////////////////////////////////////////
};

