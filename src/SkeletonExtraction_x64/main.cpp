#include <iostream>
#include <fstream>
#include <vector>

#include "volume2cellcomplex.h"
#include "CellComplex\CellComplex.h"

void extractSkeleton(const std::string &inputFile, const std::string &outputFile);
bool writeSimpleSkeletonFile(std::string file, const vector<float*> &segments, const vector<int> &ptNum);

int main(){

	// Process script
	std::ifstream input("sex64.txt");
	if (!input.is_open()){
		std::cout << "Cannot open the script\n";
		system("pause");
		input.close();
		return 0;
	}

	// Prepare string buffer
	char strBuffer[500];
	// Get volume file direction
	input.getline(strBuffer, 500);
	const std::string dir = strBuffer;
	// Get number of filesImages
	input.getline(strBuffer, 500);
	int numFile = std::atoi(strBuffer);
	// Get file	
	std::vector<std::string> volFiles(numFile);
	for (int i = 0; i < numFile; i++){
		input.getline(strBuffer, 500);
		volFiles[i] = strBuffer;
	}

	input.close();

	const std::string &outputDir = "C:\\Users\\Kevin\\Desktop\\OpenGLRenderer\\OpenGLRenderer\\Motivation\\OriginSS\\";

	for (int i = 0; i < volFiles.size(); i++){
		extractSkeleton(dir + volFiles[i] + ".vol", outputDir + volFiles[i] + ".ss");
	}	

	system("pause");
	return 0;
}

void extractSkeleton(const std::string &inputFile, const std::string &outputFile){
	int *data;
	int sizexyz[3];
	double spacexyz[3];
	double cornerPos[3];
	int threshold;	
	
	// Load .vol File
	std::ifstream input(inputFile, std::ios::binary);
	if (!input.is_open()){
		std::cout << "File: " << inputFile.c_str() << " Loaded Failed\n";		
		return;
	}
	// Read Total
	int total;
	input.read((char*)(&total), 4);
	// Read data
	data = new int[total];
	input.read((char*)data, 4 * total);
	// Read Size
	input.read((char*)sizexyz, 4 * 3);
	// Read Space (double)
	input.read((char*)spacexyz, 8 * 3);
	// Read Corner (double)
	input.read((char*)cornerPos, 8 * 3);
	// Read Threshold
	input.read((char*)(&threshold), 4);

	// Volume To CC
	std::vector<float> ccPts;
	std::vector<int> ptParents[6];
	volume2CC(data,
		sizexyz,
		spacexyz,
		cornerPos,
		threshold,

		ccPts, ptParents[0], ptParents[1], ptParents[2], ptParents[3], ptParents[4], ptParents[5]);

	std::cout << "Now Extract Skeleton\n";
	// Extract Skeleton
	CellComplex *cellComplex = new CellComplex();

	cellComplex->construct(
		ccPts, ptParents[0], ptParents[1], ptParents[2], ptParents[3], ptParents[4], ptParents[5]);

	cellComplex->setDistDiffThresh(4.5f, 4.5f);
	cellComplex->setComponentThresh(20, 6);
	cellComplex->thin2Pass_Cmp(1.0f, 0.5f);
	cellComplex->setTypes();

	cellComplex->writeCellComplexV0_Exist("test.cc");

	CellComplex *newCC = new CellComplex();
	std::vector<float*> m_originSegs;
	std::vector<int> m_originPtNums;
	newCC->read("test.cc");
	newCC->extractLine(m_originSegs, m_originPtNums);

	std::cout << "Now Output File\n";
	writeSimpleSkeletonFile(outputFile, m_originSegs, m_originPtNums);

	delete cellComplex;
	delete newCC;
}

bool writeSimpleSkeletonFile(std::string file, const vector<float*> &segments, const vector<int> &ptNum){

	std::ofstream output(file, std::ios::binary);
	if (!output.is_open()){
		return false;
	}

	// Write Number of segments
	int segmentCount = segments.size();
	output.write((char*)(&segmentCount), 4);

	for (int i = 0; i < segmentCount; i++){
		// Write Number of Vertices
		int numVertices = ptNum[i];
		output.write((char*)(&numVertices), 4);

		// Write Vertices
		float *ptr = segments[i];
		output.write((char*)(ptr), numVertices * 12);
	}

	// Finish
	output.close();

	return true;
}