#include "LineImprover4.h"


LineImprover4::LineImprover4(Source *s)
{
	if (s == nullptr)
		return;

	// Set up editor image objects
	int numImage = s->m_ioList.size(); 
	m_eios = std::vector<EditorImageObject*>(numImage);
	for (int i = 0; i < numImage; i++){
		m_eios[i] = new EditorImageObject(s->m_ioList[i]);
	}

	m_unitLineAdjMat = nullptr;
}


LineImprover4::~LineImprover4()
{
	// Release Editor Graph
	delete m_graph;

	// Release Structure Contents
	int nsc = m_subStructureLibrary.size();
	for (int i = 0; i < nsc; i++){
		StructureContent *sc = m_subStructureLibrary[i];
		delete[] sc->lineFlags;
		delete[] sc->parents;
		delete sc;
	}	

	// Release Unit Lines
	int numUnitLine = m_unitLines.size();
	for (int i = 0; i < numUnitLine; i++){
		delete m_unitLines[i];		
	}
	int numUnit = m_unitELDs.size();
	for (int i = 0; i < numUnit; i++){
		delete m_unitELDs[i];
	}

	// Release Parent Counter
	delete[] m_parentCounter;	

	// Release Editor Image Object
	int numImg = m_eios.size();
	for (int i = 0; i < numImg; i++){
		delete m_eios[i];
	}

	// Release LineData
	int numSeg = m_lineDatas.size();
	for (int i = 0; i < numSeg; i++){
		delete m_lineDatas[i];
	}

	// Release Adjacency Matrix
	for (int i = 0; i < numSeg; i++){
		delete[] m_adjacencyMatrix[i];
	}
	delete[] m_adjacencyMatrix;

	// Release Unit Adjacency Matrix
	for (int i = 0; i < numSeg; i++){
		delete[] m_unitLineAdjMat[i];
	}
	delete[] m_unitLineAdjMat;
}

void LineImprover4::selectBest_1222(std::vector<float*> &newLineSegments, std::vector<int> &newLinePtNum, const std::string &processedSSFileName, bool outputProcessedSS){
	int numLine = newLineSegments.size();
	int numImage = m_eios.size();

	StructureContent *current = new StructureContent;
	current->lineFlags = new unsigned char[numLine];
	current->parents = new int[numLine];
	for (int i = 0; i < numLine; i++){
		current->lineFlags[i] = 1;
	}
	current->numLine = numLine;	
	current->removedLines.clear();
	
	
	m_subStructureLibrary = std::vector<StructureContent*>(numLine);
	for (int i = 0; i < numLine; i++){
		StructureContent *sc = new StructureContent;
		sc->lineFlags = new unsigned char[numLine];
		for (int j = 0; j < numLine; j++){
			sc->lineFlags[j] = 0;
		}
		sc->parents = new int[numLine];
		m_subStructureLibrary[i] = sc;
	}

	m_unitLines = std::vector<UnitLine*>(numLine);
	m_unitELDs = std::vector<EditorLineData*>(numLine);
	for (int i = 0; i < numLine; i++){
		m_unitLines[i] = new UnitLine;
		m_unitELDs[i] = new EditorLineData;
	}

	// Setup parent counter
	m_parentCounter = new int[numLine];

	// Setup Graph
	m_graph = new EditorGraph(numLine);

	// Setup Adjacency Matrix
	this->setupAdjacencyMatrix(newLineSegments, newLinePtNum);

	// Set up Lines
	this->setUpLines(newLineSegments, newLinePtNum);
	this->markEnabledLine(current);
	
	// Initialize Parent
	this->initParent(current);
	
	// Directly Remove too-short leaf
	int numTSL = 5;
	int numIter0 = 0;
	while (numTSL > 0){
		numTSL = this->directlyRemoveTooShortLeaf(current, 5);
		this->updateParent(current);
		numIter0++;
	}
	this->collectUnitLines(current);
	current->numLineOfCorrectTopology = this->getCorrectStructureNumSeg(current, -1);
		
	// Create Maps
	for (int i = 0; i < m_eios.size(); i++){
		m_eios[i]->renderToLineCoverCounterMap(m_lineDatas, i);
	}

	// Record Max
	E_MAX_BRANCH = current->numLineOfCorrectTopology;
	// Set Fake Energy
	current->energy = 9;

	// Record the structure of every steps
	std::vector<StructureContent*> steps(numLine);
	steps.clear();
	steps.push_back(current);

	std::string totalNumSegString = std::to_string(current->numLineOfCorrectTopology);
	ProgressTestSender *sender = ProgressTestSender::Instance();
	sender->addLog("Total step: " + totalNumSegString);
	
	int numIter = 0;
	
	std::clock_t startTime, endTime;	

	startTime = std::clock();
	while (true){

		m_energyLog.push_back(current->eComplexity);
		m_energyLog.push_back(current->eNumOfLine);
		m_energyLog.push_back(current->eSimilarity);
		m_energyLog.push_back(current->numLineOfCorrectTopology);
		m_energyLog.push_back(current->energy);

		if (outputProcessedSS){
			//this->outputProcessedSS(current, processedSSFileName + "_" + std::to_string(numIter) + ".ss");
		}

		if (current->numLineOfCorrectTopology <= 1){
			break;
		}

		if (current->energy >= 20){
			break;
		}


		// Update Progress
		std::string currNumSegString = std::to_string(current->numLineOfCorrectTopology);
		sender->updateProgressText(currNumSegString + " / " + totalNumSegString);

		StructureContent *minSibling = this->getBestSubStructure(current);
		//StructureContent *minSibling = this->getBestSubStructure_parallel(current);
		steps.push_back(minSibling); 
		
		current = minSibling;

		// Removed Line from Images
		int numRemoved = minSibling->removedLines.size();
		for (int j = 0; j < numRemoved; j++){
			for (int i = 0; i < numImage; i++){				
				m_eios[i]->removeLineFromLCCM(minSibling->removedLines[j]->m_originImageProjectedPixels[i]);
			}			
		}

		numIter++;		
	}
	endTime = std::clock();

	std::cout << "Simplify done: " << (endTime - startTime) / (double)(CLOCKS_PER_SEC) << "\n";
	
	// Select Global Minimum
	int numStep = steps.size();
	int globalMinimumIndex = 0;
	float globalMinimumE = steps[0]->energy;
	for (int i = 0; i < numStep; i++){
		if (steps[i]->energy < globalMinimumE){
			globalMinimumIndex = i;
			globalMinimumE = steps[i]->energy;
		}
	}

	this->outputProcessedSS(steps[globalMinimumIndex], processedSSFileName + "_selected.ss");
	//this->changeDataStructure(steps[globalMinimumIndex], newLineSegments, newLinePtNum); 
	this->mergeUnitLine(steps[globalMinimumIndex], newLineSegments, newLinePtNum);
}
void LineImprover4::setupAdjacencyMatrix(const std::vector<float*> &segments, const std::vector<int> &ptNums){
	int numSeg = segments.size();
	m_adjacencyMatrixSize = numSeg;

	m_adjacencyMatrix = new unsigned char*[numSeg];
	for (int i = 0; i < numSeg; i++){
		m_adjacencyMatrix[i] = new unsigned char[numSeg];
		for (int j = 0; j < numSeg; j++){
			m_adjacencyMatrix[i][j] = 0;
		}
	}

	LineWidget::setupAdjacencyMatrix(segments, ptNums, m_adjacencyMatrix);
}
LineImprover4::StructureContent *LineImprover4::getBestSubStructure(LineImprover4::StructureContent *current){
	
	// First reset Enabled Flag of Current
	//this->markEnabledLine(current);
	// Mark articulations
	//this->markArticulations();
	
	// Collect Unit Lines
	this->collectUnitLines(current);
	this->markUnitLineArticulations(current);

	int numTotalLine = m_lineDatas.size();
	int subStructureIter = 1;
	StructureContent *minSibling = nullptr;
	float minimumEnergy = 10000;

	//QTime timer;
	
	for (int i = 0; i < numTotalLine; i++){
		//timer.restart();

		UnitLine *ul = m_unitLines[i];
		// Not a enabled unit line
		if (ul->lineIndices.size() <= 0)
			continue;

		//if (m_unitELDs[i]->m_articulationFlag == 1)
			//continue;

		int numMemberLine = ul->lineIndices.size();
		
		if (m_unitELDs[i]->m_articulationFlag == 1)
			continue;
				

		// Remove specify unit line
		StructureContent *sub = m_subStructureLibrary[subStructureIter];
		sub->removedLines.clear();
		sub->numLine = current->numLine - numMemberLine;
		
		for (int j = 0; j < numTotalLine; j++){
			sub->lineFlags[j] = current->lineFlags[j];	
			sub->parents[j] = current->parents[j];
		}
		for (int j = 0; j < numMemberLine; j++){
			int idx = ul->lineIndices[j];
			sub->lineFlags[idx] = 0;
			sub->parents[idx] = -1;
			sub->removedLines.push_back(m_lineDatas[idx]);
		}

		this->updateParent(sub, i);	
		
		// Calculate Energy		
		this->calculateStructureEnergy(sub, i);
		
		// Find minimum sibling
		if (sub->energy < minimumEnergy){
			minSibling = sub;
			minimumEnergy = sub->energy;
		}

		subStructureIter++;		
	}

	return this->getCopy(minSibling);
}
LineImprover4::StructureContent *LineImprover4::getBestSubStructure_parallel(StructureContent *current){
	return nullptr;
}
LineImprover4::StructureContent *LineImprover4::getCopy(StructureContent *src){
	int numLine = m_lineDatas.size();

	StructureContent *sc = new StructureContent;
	sc->lineFlags = new unsigned char[numLine];
	sc->parents = new int[numLine];

	for (int i = 0; i < numLine; i++){
		sc->lineFlags[i] = src->lineFlags[i];
		sc->parents[i] = src->parents[i];
	}

	sc->eComplexity = src->eComplexity;
	sc->energy = src->energy;
	sc->eNumOfLine = src->eNumOfLine;
	sc->eSimilarity = src->eSimilarity;
	sc->numLine = src->numLine;
	sc->numLineOfCorrectTopology = src->numLineOfCorrectTopology;
	sc->removedLines.clear();
	
	int numRemoved = src->removedLines.size();
	sc->removedLines = std::vector<EditorLineData*>(numRemoved);
	for (int i = 0; i < numRemoved; i++){
		sc->removedLines[i] = src->removedLines[i];
	}

	return sc;
}
/////////////////////////////////////////////////////////////////////////
void LineImprover4::markEnabledLine(StructureContent *sc){
	int numTotalLine = m_lineDatas.size();
	for (int i = 0; i < numTotalLine; i++){
		m_lineDatas[i]->m_enabledFlag = sc->lineFlags[i];
	}
}
void LineImprover4::markArticulations(){
	// Clear articulation flag of all
	int numLine = m_lineDatas.size();
	for (int i = 0; i < numLine; i++){
		m_lineDatas[i]->m_articulationFlag = 0;
	}
	m_graph->AP(m_adjacencyMatrix, this->m_lineDatas);
}
/////////////////////////////////////////////////////////////////////////
void LineImprover4::calculateStructureEnergy(StructureContent *sc, int exceptUnit){
	int numImage = m_eios.size(); 

	float eComplexity = 0;
	float eSimilarity = 0;
	
	// Calculate Complexity of Whole-System
	for (int i = 0; i < numImage; i++){
		float s, c;
		m_eios[i]->getSimilarityAndComplexity(&s, &c, sc->removedLines, i, 10, 5);

		eSimilarity = eSimilarity + s;
		eComplexity = eComplexity + c;
	}
	

	sc->numLineOfCorrectTopology = this->getCorrectStructureNumSeg(sc, exceptUnit);

	sc->eComplexity = eComplexity / numImage;
	sc->eSimilarity = eSimilarity / numImage;
	sc->eNumOfLine = sc->numLineOfCorrectTopology * 1.0 / E_MAX_BRANCH;

	sc->energy = 0.25 *sc->eComplexity + 0.15 * sc->eNumOfLine + 0.6 * sc->eSimilarity;
}
/////////////////////////////////////////////////////////////////////////
void LineImprover4::setUpLines(const std::vector<float*> &segments, const std::vector<int> &ptNums){
	std::vector<int> pixelBuffer;
	int numLine = segments.size();
	int numImage = m_eios.size();

	m_lineDatas = std::vector<EditorLineData*>(numLine);

	for (int i = 0; i < numLine; i++){
		EditorLineData *eld = new EditorLineData();

		eld->priority = -1;
		eld->m_enabledFlag = 1;
		eld->m_correspondingMaterialIndex = i;
		eld->m_originImageProjectedPixels = std::vector<PixelSet*>(numImage);

		eld->geometry = segments[i];
		eld->numPt = ptNums[i];

		for (int j = 0; j < numImage; j++){
			pixelBuffer.clear();
			m_eios[j]->getSingleIndexCoverPixel(segments[i], ptNums[i], pixelBuffer, 1);
			int numSingleIndexPixel = pixelBuffer.size();
			int *singleIndexPixelBuffer = new int[numSingleIndexPixel];
			for (int k = 0; k < numSingleIndexPixel; k++){
				singleIndexPixelBuffer[k] = pixelBuffer[k];
			}
			PixelSet *ps = new PixelSet;
			ps->pixels = singleIndexPixelBuffer;
			ps->numPixel = numSingleIndexPixel;

			eld->m_originImageProjectedPixels[j] = ps;
		}

		m_lineDatas[i] = eld;
	}
}
///////////////////////////////////////////////////////////////////////////
int LineImprover4::directlyRemoveTooShortLeaf(StructureContent *s, int threshold){
	// Return Number of removed noise (too-short leaf)
	int numLine = m_lineDatas.size();
	int numRemoved = 0;

	for (int i = 0; i<numLine; i++){
		if (s->lineFlags[i] == 1){
			// 0: leaf, 1: not leaf, 2: error
			int label = this->isLeaf(i, s->lineFlags); 
			if (label == 2){
				// Error occur, directly remove				
				s->lineFlags[i] = 0;
				s->parents[i] = -1;
				m_lineDatas[i]->m_enabledFlag = 0;
				numRemoved++;
				s->numLine = s->numLine - 1;
				s->removedLines.push_back(m_lineDatas[i]);				
			}
			else if (label == 0){
				if (m_lineDatas[i]->numPt <= threshold){
					// Remove
					s->lineFlags[i] = 0;
					s->parents[i] = -1;
					m_lineDatas[i]->m_enabledFlag = 0; 
					numRemoved++;
					s->numLine = s->numLine - 1;
					s->removedLines.push_back(m_lineDatas[i]);					
				}
			}
		}		
	}

	return numRemoved;
}
int LineImprover4::isLeaf(int index, unsigned char *lineFlags){
	// 1 = Num of head neighbors
	// 2 = Num of tail neighbors 

	//0: leaf, 1: not leaf, 2:error
	int numNeighbor[3] = { 0 };

	int numLine = m_lineDatas.size();
	for (int i = 0; i < numLine; i++){

		// I is not the line of current structure
		if (lineFlags[i] == 0)
			continue;

		numNeighbor[m_adjacencyMatrix[index][i]]++;
	}

	if (numNeighbor[1] == 0 && numNeighbor[2] == 0){
		return 2;
	}

	if (numNeighbor[1] == 0 && numNeighbor[2] != 0){
		// A leaf
		return 0;
	}
	if (numNeighbor[1] != 0 && numNeighbor[2] == 0){
		// A leaf
		return 0;
	}

	return 1;
}
void LineImprover4::changeDataStructure(StructureContent *s, std::vector<float*> &segs, std::vector<int> &ptNums){
	int numTotalLine = m_lineDatas.size();

	segs = std::vector<float*>(s->numLine);
	ptNums = std::vector<int>(s->numLine);
	int segIter = 0;

	for (int i = 0; i < numTotalLine; i++){
		
		if (s->lineFlags[i] == 0)
			continue;

		EditorLineData *eld = m_lineDatas[i];
		float *buffer = new float[eld->numPt * 3];

		for (int j = 0; j < eld->numPt; j++){
			float *v = eld->geometry + j * 3;
			buffer[j * 3 + 0] = v[0];
			buffer[j * 3 + 1] = v[1];
			buffer[j * 3 + 2] = v[2];
		}

		segs[segIter] = buffer;
		ptNums[segIter] = eld->numPt;
		segIter++;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////}
void LineImprover4::outputProcessedSS(StructureContent *s, const std::string &fileName){
	std::ofstream output(fileName, std::ios::binary);
	if (!output.is_open()){
		return;
	}

	// Write Number of segments
	int segmentCount = s->numLine;
	output.write((char*)(&segmentCount), 4);

	int numLine = m_lineDatas.size();
	for (int i = 0; i < numLine; i++){
		if (s->lineFlags[i] == 1){
			// Write Number of Vertices
			int numVertices = m_lineDatas[i]->numPt;
			output.write((char*)(&numVertices), 4);

			// Write Vertices
			float *ptr = m_lineDatas[i]->geometry;
			output.write((char*)(ptr), numVertices * 12);
		}		
	}

	// Finish
	output.close();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int LineImprover4::getCorrectStructureNumSeg(StructureContent *s, int exceptUnit){
	
	int numTotalLine = m_lineDatas.size();
	
	for (int i = 0; i < numTotalLine; i++)
		m_parentCounter[i] = 0;

	for (int i = 0; i < numTotalLine; i++){
		if (s->lineFlags[i] == 1){
			m_parentCounter[s->parents[i]]++;
		}
	}

	int numLineOfCorrectTopology = 0;
	for (int i = 0; i < numTotalLine; i++){
		if (m_parentCounter[i] > 0){
			numLineOfCorrectTopology++;
		}
	}

	return numLineOfCorrectTopology;
	
	/*
	int numLineOfCorrectTopology = 0;
	for (int i = 0; i < numTotalLine; i++){
		if (i != exceptUnit && m_unitLines[i]->lineIndices.size() > 0){
			numLineOfCorrectTopology = numLineOfCorrectTopology + 1;
		}
	}
	return numLineOfCorrectTopology;
	*/
}
void LineImprover4::initParent(StructureContent *s){
	// The structure is retopology-correct, so every line have its independent parent
	int numLine = m_lineDatas.size();

	for (int i = 0; i < numLine; i++){
		if (s->lineFlags[i] == 1){
			s->parents[i] = i;
		}
		else{
			s->parents[i] = -1;
		}
	}
}
void LineImprover4::collectUnitLines(StructureContent *s){
	int numTotalLine = m_lineDatas.size();

	for (int i = 0; i < numTotalLine; i++){
		m_unitLines[i]->lineIndices.clear();
	}

	for (int i = 0; i < numTotalLine; i++){
		int parentId = s->parents[i];
		if (parentId >= 0){
			m_unitLines[parentId]->lineIndices.push_back(i);
		}		
	}
}

void LineImprover4::updateParent(StructureContent *s){
	int numTotalLine = m_lineDatas.size();

	Neighbors neighbors[3];

	for (int m = 0; m < numTotalLine; m++){
		if (s->lineFlags[m] == 0)
			continue;

		// Check Neighbor
		for (int n = 0; n < 3; n++){
			neighbors[n].neighborIdx.clear();
		}
		for (int o = 0; o < numTotalLine; o++){
			if (s->lineFlags[o] == 1){
				neighbors[m_adjacencyMatrix[m][o]].neighborIdx.push_back(o);
			}
		}

		int numNeighbor1 = neighbors[1].neighborIdx.size(); 
		int numNeighbor2 = neighbors[2].neighborIdx.size();

		// Check Statement
		if (numNeighbor1 == 1 && numNeighbor2 != 1){
			int resParentId = s->parents[m];
			int src1ParentId = s->parents[neighbors[1].neighborIdx[0]];
			
			if (resParentId != src1ParentId){				
				for (int p = 0; p < numTotalLine; p++){
					if (s->parents[p] == src1ParentId){
						s->parents[p] = resParentId;
					}
				}
			}			
		}
		else if (numNeighbor2 == 1 && numNeighbor1 != 1){
			int resParentId = s->parents[m];
			int src2ParentId = s->parents[neighbors[2].neighborIdx[0]];
			if (resParentId != src2ParentId){				
				for (int q = 0; q < numTotalLine; q++){
					if (s->parents[q] == src2ParentId){
						s->parents[q] = resParentId;
					}
				}
			}			
		}
		else if (numNeighbor1 == 1 && numNeighbor2 == 1){
			int resParentId = s->parents[m];
			int src1ParentId = s->parents[neighbors[1].neighborIdx[0]];
			int src2ParentId = s->parents[neighbors[2].neighborIdx[0]];
						
			if (resParentId != src1ParentId){				
				for (int r = 0; r < numTotalLine; r++){
					if (s->parents[r] == src1ParentId){
						s->parents[r] = resParentId;
					}
				}
			}

			if (resParentId != src2ParentId){				
				for (int t = 0; t < numTotalLine; t++){
					if (s->parents[t] == src2ParentId){
						s->parents[t] = resParentId;
					}
				}
			}
		}		
	}
}
void LineImprover4::mergeUnitLine(StructureContent *s, std::vector<float*> &segs, std::vector<int> &ptNums){
	// Clear all used flag
	int numLine = m_lineDatas.size();

	this->collectUnitLines(s);

	std::vector<EditorLineData*> unitELDs;

	segs.clear();
	ptNums.clear();

	for (int i = 0; i < numLine; i++){
		UnitLine *ul = m_unitLines[i];
		int numMemberLine = ul->lineIndices.size();
		if (numMemberLine <= 0)
			continue;

		unitELDs.clear();
		for (int j = 1; j < numMemberLine; j++){
			unitELDs.push_back(m_lineDatas[ul->lineIndices[j]]);
		}

		// Get first as start
		EditorLineData *start = m_lineDatas[ul->lineIndices[0]];
		float *unitLineSeg = new float[start->numPt * 3];
		int unitLineNumPt = start->numPt;
		for (int i = 0; i < start->numPt; i++){
			unitLineSeg[i * 3 + 0] = start->geometry[i * 3 + 0];
			unitLineSeg[i * 3 + 1] = start->geometry[i * 3 + 1];
			unitLineSeg[i * 3 + 2] = start->geometry[i * 3 + 2];
		}

		while (this->mergeOne(&unitLineSeg, &unitLineNumPt, unitELDs));

		if (unitELDs.size() > 0){
			int a = 1;
		}

		segs.push_back(unitLineSeg);
		ptNums.push_back(unitLineNumPt);		
	}
}
bool LineImprover4::mergeOne(float **srcSeg, int *srcPtNum, std::vector<EditorLineData*> &elds){
	int numELD = elds.size();	
	
	for (int i = 0; i < numELD; i++){
		unsigned char statement = LineWidget::linkRelation(*srcSeg, *srcPtNum, elds[i]->geometry, elds[i]->numPt);

		if (statement == 0)
			continue;		

		// Reallocate memories of line0
		int totalPt = *srcPtNum + elds[i]->numPt - 1;
		float *newLineData = new float[totalPt * 3];

		// tail -> head
		if (statement == 3 ){
			this->setToDataBuffer(*srcSeg, *srcPtNum, false, elds[i]->geometry, elds[i]->numPt, false, newLineData);
		}
		// tail -> tail
		else if (statement == 4 ){
			this->setToDataBuffer(*srcSeg, *srcPtNum, false, elds[i]->geometry, elds[i]->numPt, true, newLineData);
		}
		// head -> head
		else if (statement == 1 ){
			this->setToDataBuffer(*srcSeg, *srcPtNum, true, elds[i]->geometry, elds[i]->numPt, false, newLineData);
		}
		// head -> tail
		else if (statement == 2 ){
			this->setToDataBuffer(*srcSeg, *srcPtNum, true, elds[i]->geometry, elds[i]->numPt, true, newLineData);
		}

		elds.erase(elds.begin() + i);
		*srcSeg = newLineData;
		*srcPtNum = totalPt;

		return true;
	}

	return false;
}
void LineImprover4::setToDataBuffer(float *first, int firstVertexCount, bool firstInverse, float *second, int secondVertexCount, bool secondInverse, float *dataBuffer){
	int offset = 0;

	// First Part
	if (firstInverse){
		for (int i = firstVertexCount - 1; i >= 0; i--){
			float *v = first + i * 3;
			dataBuffer[offset + 0] = v[0];
			dataBuffer[offset + 1] = v[1];
			dataBuffer[offset + 2] = v[2];
			offset += 3;
		}
	}
	else{
		for (int i = 0; i < firstVertexCount; i++){
			float *v = first + i * 3;
			dataBuffer[offset + 0] = v[0];
			dataBuffer[offset + 1] = v[1];
			dataBuffer[offset + 2] = v[2];
			offset += 3;
		}
	}


	// Second Part, remove first vertex
	if (secondInverse){
		for (int i = secondVertexCount - 2; i >= 0; i--){
			float *v = second + i * 3;
			dataBuffer[offset + 0] = v[0];
			dataBuffer[offset + 1] = v[1];
			dataBuffer[offset + 2] = v[2];
			offset += 3;
		}
	}
	else{
		for (int i = 1; i < secondVertexCount; i++){
			float *v = second + i * 3;
			dataBuffer[offset + 0] = v[0];
			dataBuffer[offset + 1] = v[1];
			dataBuffer[offset + 2] = v[2];
			offset += 3;
		}
	}
}
//////////////////////////////////////////////////////////////////////////////
void LineImprover4::removeSmallCycle(std::vector<float*> &resSegs, std::vector<int> &resPtNums, const std::vector<float*> &srcSegs, const std::vector<int> &srcPtNums){
	const int SMALL_CYCLE_LENGTH_THRESHOLD = 20;
	// Create Table
	int numSeg = srcSegs.size();
	int numSmallCycle = 0;

	std::vector<unsigned char> mergeFlagTable(numSeg);
	for (int i = 0; i < numSeg; i++){
		mergeFlagTable[i] = 0;
	}


	for (int i = 0; i < numSeg; i++){
		if (mergeFlagTable[i] == 1)
			continue;
		if (srcPtNums[i] > SMALL_CYCLE_LENGTH_THRESHOLD){
			// Directly Remain
			resSegs.push_back(srcSegs[i]);
			resPtNums.push_back(srcPtNums[i]);
			continue;
		}
			

		for (int j = i+1; j < numSeg; j++){
			if (mergeFlagTable[i] == 1)
				continue;
			if (srcPtNums[j] > SMALL_CYCLE_LENGTH_THRESHOLD)
				continue;

			// Check statement
			if (this->isSmallCycle(srcSegs, srcPtNums, i, j)){
				mergeFlagTable[i] = 1;
				mergeFlagTable[j] = 1;
				numSmallCycle++;
				break;
			}
		}

		if (mergeFlagTable[i] == 0){
			// Does not be merge
			resSegs.push_back(srcSegs[i]);
			resPtNums.push_back(srcPtNums[i]);
		}
		else{
			// Remain One 
			resSegs.push_back(srcSegs[i]);
			resPtNums.push_back(srcPtNums[i]);
		}
	}

	ProgressTestSender::Instance()->addLog("Num Small Cycle: " + std::to_string(numSmallCycle));
}
bool LineImprover4::isSmallCycle(const std::vector<float*> &srcSegs, const std::vector<int> &srcPtNums, int s0, int s1){
	// 0, S1 is not a neighbor of S0
	// 1, S1 is the head neighbor of S0
	// 2, S1 is the tail neighbor of S0

	const float *head = srcSegs[s0];
	const float *tail = srcSegs[s0] + (srcPtNums[s0] - 1) * 3;
	const float *othHead = srcSegs[s1];
	const float *othTail = srcSegs[s1] + (srcPtNums[s1] - 1) * 3;
	const float THRESHOLD = 0.000000001;
	//////////////////////////////////////////////////////////////////
	// Seg0 Head -> Seg1 Head
	if (fabs(othHead[0] - head[0]) < THRESHOLD && fabs(othHead[1] - head[1]) < THRESHOLD && fabs(othHead[2] - head[2]) < THRESHOLD){
		if (fabs(othTail[0] - tail[0]) < THRESHOLD && fabs(othTail[1] - tail[1]) < THRESHOLD && fabs(othTail[2] - tail[2]) < THRESHOLD){
			return true;
		}
	}
	// Seg0 Head -> Seg1 Tail
	if (fabs(othTail[0] - head[0]) < THRESHOLD && fabs(othTail[1] - head[1]) < THRESHOLD && fabs(othTail[2] - head[2]) < THRESHOLD){
		if (fabs(othHead[0] - tail[0]) < THRESHOLD && fabs(othHead[1] - tail[1]) < THRESHOLD && fabs(othHead[2] - tail[2]) < THRESHOLD){
			return true;
		}
	}

	return false;
}
void LineImprover4::retopo(std::vector<float*> &srcSegs, std::vector<int> &srcPtNums){
	// Clear line datas and unit lines
	if (m_lineDatas.size() > 0){
		int numLine = m_lineDatas.size();
		for (int i = 0; i < numLine; i++){
			delete m_lineDatas[i];
		}
	}
	if (m_unitLines.size() > 0){
		int numUnit = m_unitLines.size();
		for (int i = 0; i < numUnit; i++){
			delete m_unitLines[i];
		}
	}

	int numLine = srcSegs.size();
	// Create Line Datas
	m_lineDatas = std::vector<EditorLineData*>(numLine);
	m_unitLines = std::vector<UnitLine*>(numLine);
	for (int i = 0; i < numLine; i++){
		EditorLineData *eld = new EditorLineData;
		eld->geometry = srcSegs[i];
		eld->numPt = srcPtNums[i];
		m_lineDatas[i] = eld;

		m_unitLines[i] = new UnitLine;
	} 
	// Create Structure
	StructureContent *sc = new StructureContent;
	sc->lineFlags = new unsigned char[numLine];
	sc->parents = new int[numLine];
	for (int i = 0; i < numLine; i++){
		sc->lineFlags[i] = 1;
	}

	this->setupAdjacencyMatrix(srcSegs, srcPtNums);

	this->initParent(sc);
	this->updateParent(sc);

	this->mergeUnitLine(sc, srcSegs, srcPtNums); 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LineImprover4::markUnitLineArticulations(StructureContent *sc){
	int numLine = m_lineDatas.size();

	if (m_unitLineAdjMat == nullptr){
		m_unitLineAdjMat = new unsigned char*[numLine];
		for (int i = 0; i < numLine; i++){
			m_unitLineAdjMat[i] = new unsigned char[numLine];
		}
	}
	
	// Clear unit line adjacency matrix
	for (int i = 0; i < numLine; i++){
		for (int j = 0; j < numLine; j++){
			m_unitLineAdjMat[i][j] = 0;
		}		
	}	
	// Clear Articulation of unit line edl
	for (int i = 0; i < numLine; i++){
		m_unitELDs[i]->m_articulationFlag = 0;
	}
	
	for (int i = 0; i < numLine; i++){
		
		// Mark Enabled
		if (m_unitLines[i]->lineIndices.size() <= 0){
			m_unitELDs[i]->m_enabledFlag = 0;
			continue;
		}
		else{
			m_unitELDs[i]->m_enabledFlag = 1;
		}

		// Find its neighbor
		int numMemberLine = m_unitLines[i]->lineIndices.size();
		for (int j = 0; j < numMemberLine; j++){
			int lineIdx = m_unitLines[i]->lineIndices[j];
			
			// Check neighbor line's parent
			for (int k = 0; k < numLine; k++){
				if (sc->lineFlags[k] == 1 && m_adjacencyMatrix[lineIdx][k] != 0){
					int neighborParent = sc->parents[k];
					
					// Record the link
					m_unitLineAdjMat[i][neighborParent] = 1;
					m_unitLineAdjMat[neighborParent][i] = 1;
				}
			}
		}
	}

	// Clear self link
	for (int i = 0; i < numLine; i++){
		m_unitLineAdjMat[i][i] = 0;
	}

	m_graph->AP(m_unitLineAdjMat, m_unitELDs);
}
bool LineImprover4::isLeafUnitLine(UnitLine *ul, StructureContent *s){
	// If one of the member line is leaf -> it is leaf segment
	int numMember = ul->lineIndices.size();

	for (int i = 0; i < numMember; i++){
		if (this->isLeaf(ul->lineIndices[i], s->lineFlags))
			return true;
	}

	return false;
}
/////////////////////////////////////////////////////////////////////////////
void LineImprover4::updateParent(StructureContent *s, int removedUnitLine){
	std::vector<int> relationLines;
	int numTotalLine = m_lineDatas.size();


	int numLineOfUnit = m_unitLines[removedUnitLine]->lineIndices.size();
	int removedParent = removedUnitLine;
	for (int i = 0; i < numLineOfUnit; i++){
		int lineIdx = m_unitLines[removedUnitLine]->lineIndices.at(i);
		for (int j = 0; j < numTotalLine; j++){
			if (s->lineFlags[j] == 1 && m_adjacencyMatrix[lineIdx][j] != 0 && s->parents[j] != removedParent){
				relationLines.push_back(j);
			}
		}
	}

	////////////////////////////////////////
	Neighbors neighbors[3];
	int numTotalRelationLines = relationLines.size();
	for (int m = 0; m < numTotalRelationLines; m++){
		int targetLine = relationLines[m];
		// Check Neighbor
		for (int n = 0; n < 3; n++){
			neighbors[n].neighborIdx.clear();
		}
		for (int o = 0; o < numTotalLine; o++){
			if (s->lineFlags[o] == 1){
				neighbors[m_adjacencyMatrix[targetLine][o]].neighborIdx.push_back(o);
			}
		}

		int numNeighbor1 = neighbors[1].neighborIdx.size();
		int numNeighbor2 = neighbors[2].neighborIdx.size();

		// Check Statement
		if (numNeighbor1 == 1){
			int resParentId = s->parents[targetLine];
			int src1ParentId = s->parents[neighbors[1].neighborIdx[0]];

			if (resParentId != src1ParentId){
				int numSrc1ParentId = m_unitLines[src1ParentId]->lineIndices.size();
				for (int p = 0; p < numSrc1ParentId; p++){
					int srcLine = m_unitLines[src1ParentId]->lineIndices.at(p);
					s->parents[srcLine] = resParentId;
				}
			}
		}		
		if (numNeighbor2 == 1 && numNeighbor2 == 1){
			int resParentId = s->parents[targetLine];
			int src2ParentId = s->parents[neighbors[2].neighborIdx[0]];

			if (resParentId != src2ParentId){
				int numSrc2ParentId = m_unitLines[src2ParentId]->lineIndices.size();
				for (int p = 0; p < numSrc2ParentId; p++){
					int srcLine = m_unitLines[src2ParentId]->lineIndices.at(p);
					s->parents[srcLine] = resParentId;
				}
			}
		}
	}
}
