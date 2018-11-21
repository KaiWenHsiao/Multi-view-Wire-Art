#include "EditorLineData.h"


EditorLineData::EditorLineData()
{
}


EditorLineData::~EditorLineData()
{
	// Does not make copy, geometry cannot be delete
	//delete[] geometry;

	// Release PixelSet
	int nps = m_originImageProjectedPixels.size();
	for (int i = 0; i < nps; i++){
		delete[] m_originImageProjectedPixels[i]->pixels;
		delete m_originImageProjectedPixels[i];
	}
}

bool EditorLineData::isTarget(int dataSetIdx, unsigned char **flagTable){
	ProjectedPixels *usingPP = m_imageProjectedPixels[dataSetIdx];
	int total = usingPP->size();
	int threshold = ceil(total * 0.02);
	int numEnabled = 0;

	// If its number of pixel that is enabled over 80% => it is selected line
	for (int i = 0; i < total; i++){
		const glm::ivec2 &pixel = usingPP->at(i);
		if (flagTable[pixel.x][pixel.y] == 1){
			numEnabled++;
		}
		if (numEnabled >= threshold)
			return true;
	}
	return false;
}
