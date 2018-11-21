#pragma once

#include <vector>
#include <glm\vec2.hpp>
#include <glm\mat4x4.hpp>

#include "EditorLineData.h"
#include "EditorImageObject.h"
#include "EditorFunctions.h"
#include "../LineWidget.h"

class EditorManager
{
public:
	EditorManager();
	virtual ~EditorManager();

	static EditorManager *Instance();

	// Whole Screen, Image Coordinate
	unsigned char **m_pixelDataFlags;
	int m_screenWidth;
	int m_screenHeight;
	ProjectedPixels m_currentSelectedPixels;

	std::vector<EditorLineData*> m_lineDatas;

	struct PixelCorresponding{
		std::vector<EditorLineData*> correspondLines;
	};
	PixelCorresponding ***m_correspondingMap;
	std::vector<PixelCorresponding***> m_correspondingMaps;

	int m_currentProjectionIndex;

	

	std::vector<EditorImageObject*> m_eios;

	EditorFunctions *m_functions;

	SceneGeometry *m_structureGeometry;
	

public:
	void clearDataFlags(); 
	void markEnabledTargetLine(std::vector<EditorLineData*> &targets);
	void markDisabledTargetLine(std::vector<EditorLineData*> &targets);
	void setupEditorImageObjects(const std::vector<ImageObject*> &ios);
	void setupLineDatas(const std::vector<float*> &segs, const std::vector<int> &ptNums, const glm::mat4 &proj, std::vector<glm::mat4> &viewMats, int *sideViewProperties);
	void setupOriginImageCoverPixelSet(const std::vector<float*> &segs, const std::vector<int> &ptNums);

	bool setup(const std::string &originSS, const std::string &simpSS, const glm::mat4 &proj, std::vector<glm::mat4> &viewMats, int *sideViewProperties);
	void setUpForDeform(int *sideViewProperties);

	void saveRes(const std::string &fileName);

private:
	std::vector<float*> m_segs;
	std::vector<int> m_ptNums;
	float *m_gBuffer;
	float *m_vBuffer;
	float *m_nBuffer;


private:
	int m_sideViewProperties[4];

public:
	void imagePixelToWholeScreenPixel(int *imagePixel, int imageWidth, int imageHeight, int *wholeScreenPixel);
	bool isSelectedPixel(int *wholeScreenPixel);
};

