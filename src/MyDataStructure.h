#pragma once

#include <vector>

struct PixelSet{
	int *pixels;
	int numPixel;
};

struct ImagePixelSet{
	std::vector<PixelSet*> pixelSets;
};

