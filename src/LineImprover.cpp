#include "LineImprover.h"


LineImprover::LineImprover()
{
	
}


LineImprover::~LineImprover()
{
	int numSid = m_sids.size();
	for (int i = 0; i < numSid; i++){
		delete m_sids[i];
	}
}

void LineImprover::correctStructure(const std::vector<float*> &origSegs, const std::vector<int> &origPtNums, std::vector<float*> &dstSegs, std::vector<int> &dstPtNums){
	m_sids.clear();

	// Connect
	std::vector<float*> reConnectSegments(origSegs);
	std::vector<int> reConnectPtNum(origPtNums);
	LineImprover::copyLine(reConnectSegments, reConnectPtNum, origSegs, origPtNums);
	
	// Loop is a very big trouble maker !!!!!!!
	LineImprover::connect(reConnectSegments, reConnectPtNum);
	

	// Split
	this->splitLines(reConnectSegments, reConnectPtNum);

	// Create New
	this->createNewStructure(reConnectSegments, reConnectPtNum, dstSegs, dstPtNums);
}

void LineImprover::copyLine(std::vector<float*> &dstSegs, std::vector<int> &dstPtNums, const std::vector<float*> &srcSegs, const std::vector<int> &srcPtNums){
	int numSeg = srcSegs.size();

	for (int i = 0; i < numSeg; i++){
		dstPtNums[i] = srcPtNums[i];
		float *data = new float[srcPtNums[i] * 3];

		for (int j = 0; j < srcPtNums[i]; j++){
			float *v = srcSegs[i] + j * 3;

			data[j * 3 + 0] = v[0];
			data[j * 3 + 1] = v[1];
			data[j * 3 + 2] = v[2];
		}

		dstSegs[i] = data;
	}
}
void LineImprover::connect(std::vector<float*> &lineSegments, std::vector<int> &linePtNums){
	// Calculate Voxel Edge Length as threshold
	float *v0 = lineSegments[0];
	float *v1 = lineSegments[0] + 3;
	float delta[3] = {
		v0[0] - v1[0],
		v0[1] - v1[1],
		v0[2] - v1[2]
	};
	float VOXEL_EDGE_LENGTH = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);

	float THRESHOLD = VOXEL_EDGE_LENGTH * 1.2f;

	//////////////////////////////////////////////////////////////////////

	// Find the Disappear Segment Terminal
	int lineCount = lineSegments.size();

	for (int i = 0; i < lineCount; i++){
		bool targetFlag = true;
		float *first = lineSegments[i];

		for (int j = 0; j < lineCount; j++){
			if (j == i)
				continue;
			for (int k = 0; k < linePtNums[j]; k++){
				float *v = lineSegments[j] + k * 3;
				// Calculate Distance
				float delta[3] = {
					first[0] - v[0],
					first[1] - v[1],
					first[2] - v[2]
				};
				float dis = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
				if (dis < 0.001){
					// Not the vertices of disappear segment					
					targetFlag = false;
					break;
				}
			}
			if (!targetFlag){
				break;
			}
		}

		if (targetFlag){
			// Repair
			bool repairEndFlag = false;

			for (int j = 0; j < lineCount; j++){
				if (j == i)
					continue;
				for (int k = 0; k < linePtNums[j]; k++){
					float *v = lineSegments[j] + k * 3;
					// Calculate Distance
					float delta[3] = {
						first[0] - v[0],
						first[1] - v[1],
						first[2] - v[2]
					};
					float dis = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
					if (dis < THRESHOLD){
						// Connect
						float *dataBuffer = new float[(linePtNums[i] + 1) * 3];
						// Put the new vertex in head
						dataBuffer[0] = v[0];
						dataBuffer[1] = v[1];
						dataBuffer[2] = v[2];
						for (int l = 0; l < linePtNums[i]; l++){
							float *ov = lineSegments[i] + l * 3;
							dataBuffer[3 + l * 3 + 0] = ov[0];
							dataBuffer[3 + l * 3 + 1] = ov[1];
							dataBuffer[3 + l * 3 + 2] = ov[2];
						}
						lineSegments[i] = dataBuffer;
						linePtNums[i] += 1;
						repairEndFlag = true;
						break;

					}
				}
				if (repairEndFlag){
					break;
				}
			}
			// The First is target, the last must not a target
			continue;
		}
		////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////		

		// Do the same thing for last
		float *last = lineSegments[i] + (linePtNums[i] - 1) * 3;
		targetFlag = false;

		for (int j = 0; j < lineCount; j++){
			if (j == i)
				continue;
			for (int k = 0; k < linePtNums[j]; k++){
				float *v = lineSegments[j] + k * 3;
				// Calculate Distance
				float delta[3] = {
					last[0] - v[0],
					last[1] - v[1],
					last[2] - v[2]
				};
				float dis = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
				if (dis < 0.001){
					// Not the vertices of disappear segment					
					targetFlag = false;
					break;
				}
			}
			if (!targetFlag){
				break;
			}
		}

		if (targetFlag){
			// Repair
			bool repairEndFlag = false;

			for (int j = 0; j < lineCount; j++){
				if (j == i)
					continue;
				for (int k = 0; k < linePtNums[j]; k++){
					float *v = lineSegments[j] + k * 3;
					// Calculate Distance
					float delta[3] = {
						last[0] - v[0],
						last[1] - v[1],
						last[2] - v[2]
					};
					float dis = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
					if (dis < THRESHOLD){
						// Connect
						float *dataBuffer = new float[(linePtNums[i] + 1) * 3];
						// Put the new vertex in tail
						int offset = linePtNums[i] * 3;
						dataBuffer[offset + 0] = v[0];
						dataBuffer[offset + 1] = v[1];
						dataBuffer[offset + 2] = v[2];
						for (int l = 0; l < linePtNums[i]; l++){
							float *ov = lineSegments[i] + l * 3;
							dataBuffer[l * 3 + 0] = ov[0];
							dataBuffer[l * 3 + 1] = ov[1];
							dataBuffer[l * 3 + 2] = ov[2];
						}
						lineSegments[i] = dataBuffer;
						linePtNums[i] += 1;
						repairEndFlag = true;
						break;

					}
				}
				if (repairEndFlag){
					break;
				}
			}
		}
	}
}
bool LineImprover::isJoint(float *v, const std::vector<float*> &lineSegments, const std::vector<int> &linePtNums){
	const float threshold = 0.000000001;

	int size = lineSegments.size();
	for (int i = 0; i < size; i++){
		float *head = lineSegments[i] + 0;
		float *tail = lineSegments[i] + (linePtNums[i] - 1) * 3;

		if (fabs(head[0] - v[0]) < threshold && fabs(head[1] - v[1]) < threshold && fabs(head[2] - v[2]) < threshold){
			return true;

		}
		if (fabs(tail[0] - v[0]) < threshold && fabs(tail[1] - v[1]) < threshold && fabs(tail[2] - v[2]) < threshold){
			return true;
		}
	}
	return false;
}
void LineImprover::splitLines(const std::vector<float*> &lineSegments, const std::vector<int> &linePtNum){
	int size = lineSegments.size();
	for (int i = 0; i < size; i++){
		// create New sid
		std::vector<int>* sid = new std::vector<int>();
		// Push first to sid
		sid->push_back(0);
		// Check if there is joint in 1~(n-1)
		for (int j = 1; j < linePtNum[i] - 1; j++){
			float *v = lineSegments[i] + j * 3;
			if (isJoint(v, lineSegments, linePtNum)){
				// Push 2 times
				sid->push_back(j);
				sid->push_back(j);
			}
		}
		// Push Last to sid
		sid->push_back(linePtNum[i] - 1);
		// Push to sid set
		m_sids.push_back(sid);
	}
}
void LineImprover::createNewStructure(const std::vector<float*> &origLineSegments, const std::vector<int> &origLinePtNum, std::vector<float*> &newLineSegments, std::vector<int> &newLinePtNums){
	// Compute how many lines after split
	int total = 0;
	for (int i = 0; i < m_sids.size(); i++){
		total += (m_sids.at(i)->size()) / 2;
	}

	ProgressTestSender::Instance()->addLog("Orig Line Count: " + std::to_string(origLineSegments.size()) + "\nAfter Split Line Count: " + std::to_string(total));

	int size = m_sids.size();
	for (int i = 0; i < size; i++){
		std::vector<int> *sid = m_sids.at(i);

		for (int j = 0; j < sid->size(); j += 2){
			// Create New One !!!!!!!!!
			int ptCount = sid->at(j + 1) - sid->at(j) + 1;

			float *dataBuffer = new float[ptCount * 3];

			int offset = 0;
			for (int k = sid->at(j); k <= sid->at(j + 1); k++){
				float *v = origLineSegments[i] + k * 3;
				dataBuffer[offset + 0] = v[0];
				dataBuffer[offset + 1] = v[1];
				dataBuffer[offset + 2] = v[2];
				offset += 3;
			}
			// Set new line segments
			newLineSegments.push_back(dataBuffer);
			// Set linePtNum			
			newLinePtNums.push_back(ptCount);
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LineImprover::retopology(std::vector<float*> &lineSegments, std::vector<int> &linePtNum, int lengthThreshold, bool carefulSelfCycle){
	int currentLineIdx = 0;
	while (true){
		int size = lineSegments.size();
		// Improvment is complete
		if (currentLineIdx >= size){
			break;
		}

		float *headVertex = lineSegments[currentLineIdx];
		int headConnectLineIdx = this->isOneConnectTerminalVertex(lineSegments, linePtNum, headVertex, currentLineIdx);
		if (headConnectLineIdx != -1){
			if (linePtNum[currentLineIdx] < lengthThreshold || linePtNum[headConnectLineIdx] < lengthThreshold){
				// Merge
				if (this->merge(currentLineIdx, headConnectLineIdx, lineSegments, linePtNum, carefulSelfCycle)){
					// Restart the cycle
					//currentLineIdx = 0;
					currentLineIdx++;
					continue;
				}
				else{
					// Something Happened make merge failed
				}
			}
		}

		float *tailVertex = lineSegments[currentLineIdx] + (linePtNum[currentLineIdx] - 1) * 3;
		int tailConnectLineIdx = this->isOneConnectTerminalVertex(lineSegments, linePtNum, tailVertex, currentLineIdx);
		if (tailConnectLineIdx != -1){
			if (linePtNum[currentLineIdx] < lengthThreshold || linePtNum[tailConnectLineIdx] < lengthThreshold){
				// Merge
				if (this->merge(currentLineIdx, tailConnectLineIdx, lineSegments, linePtNum, carefulSelfCycle)){
					// Restart the cycle
					//currentLineIdx = 0;
					currentLineIdx++;
					continue;
				}
				else{
					// Something Happened make merge failed
				}
			}
		}

		// The Current line is independent
		currentLineIdx++;
	}
}
bool LineImprover::equal(float *v0, float *v1){
	const float THRESHOLD = 0.000000001;
	return
		fabs(v0[0] - v1[0]) < THRESHOLD &&
		fabs(v0[1] - v1[1]) < THRESHOLD &&
		fabs(v0[2] - v1[2]) < THRESHOLD;
}
void LineImprover::setToDataBuffer(float *first, int firstVertexCount, bool firstInverse, float *second, int secondVertexCount, bool secondInverse, float *dataBuffer, int fixedOffset){
	int offset = fixedOffset;;

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
bool LineImprover::merge(int line0, int line1, std::vector<float*> &lineSegments, std::vector<int> &linePtNum, bool carefulSelfCycle){
	float *line0Head = lineSegments[line0];
	float *line0Tail = lineSegments[line0] + (linePtNum[line0] - 1) * 3;
	float *line1Head = lineSegments[line1];
	float *line1Tail = lineSegments[line1] + (linePtNum[line1] - 1) * 3;

	// Reallocate memories of line0
	float *newLineData = new float[(linePtNum[line0] + linePtNum[line1] - 1) * 3];

	// tail -> head
	if (this->equal(line0Tail, line1Head)){
		// A Self Cycle
		if (carefulSelfCycle && this->equal(line0Head, line1Tail)){
			// Trouble Maker
			return false;
		}
		this->setToDataBuffer(lineSegments[line0], linePtNum[line0], false, lineSegments[line1], linePtNum[line1], false, newLineData, 0);
	}
	// tail -> tail
	else if (this->equal(line0Tail, line1Tail)){
		// A Self Cycle
		if (carefulSelfCycle && this->equal(line0Head, line1Head)){
			// Trouble Maker
			return false;
		}
		this->setToDataBuffer(lineSegments[line0], linePtNum[line0], false, lineSegments[line1], linePtNum[line1], true, newLineData, 0);
	}
	// head -> head
	else if (this->equal(line0Head, line1Head)){
		// A Self Cycle
		if (carefulSelfCycle && this->equal(line0Tail, line1Tail)){
			// Trouble Maker
			return false;
		}
		this->setToDataBuffer(lineSegments[line0], linePtNum[line0], true, lineSegments[line1], linePtNum[line1], false, newLineData, 0);
	}
	// head -> tail
	else if (this->equal(line0Head, line1Tail)){
		// A Self Cycle
		if (carefulSelfCycle && this->equal(line0Tail, line1Head)){
			// Trouble Maker
			return false;
		}
		this->setToDataBuffer(lineSegments[line0], linePtNum[line0], true, lineSegments[line1], linePtNum[line1], true, newLineData, 0);
	}

	// Update LineSegments & LintPtNum
	float *old = lineSegments[line0];
	lineSegments[line0] = newLineData;
	linePtNum[line0] = linePtNum[line0] + linePtNum[line1] - 1;

	delete[] old;

	// Remove Line1
	lineSegments.erase(lineSegments.begin() + line1);
	linePtNum.erase(linePtNum.begin() + line1);

	return true;
}
int LineImprover::isOneConnectTerminalVertex(const std::vector<float*> &lineSegments, const std::vector<int> &linePtNum, float *v, int self){
	int connectCount = 0;
	int connectLineIdx;

	int lineCount = lineSegments.size();
	for (int i = 0; i < lineCount; i++){
		if (i == self)
			continue;

		float *head = lineSegments[i];
		float *tail = lineSegments[i] + (linePtNum[i] - 1) * 3;
		if (this->equal(v, head)){
			connectCount++;
			connectLineIdx = i;
		}
		else if (this->equal(v, tail)){
			connectCount++;
			connectLineIdx = i;
		}
	}

	if (connectCount == 1)
		return connectLineIdx;
	else
		return -1;
}
void LineImprover::removeCycle(const std::vector<float*> &segments, std::vector<int> &ptNums){
	int numSeg = segments.size();
	int numCycle = 0;

	for (int i = 0; i < numSeg; i++){
		float *line0Head = segments[i];
		float *line0Tail = segments[i] + (ptNums[i] - 1) * 3;

		// tail -> head
		if (LineImprover::equal(line0Tail, line0Head)){
			// Remove last vertex
			ptNums[i] = ptNums[i] - 1;
			numCycle++;
		}
	}

	ProgressTestSender::Instance()->addLog("Removed cycle: " + std::to_string(numCycle));
}
void LineImprover::smoothLine(const std::vector<float*> &segments, const std::vector<int> &ptNum, std::vector<float*> &smoothSegments, std::vector<int> &smoothPtNum, int iteration){
	int segmentCount = segments.size();

	for (int i = 0; i < segmentCount; i++){
		float *buffer0 = new float[ptNum[i] * 3];
		float *buffer1 = new float[ptNum[i] * 3];

		float *v = segments[i];
		for (int j = 0; j < ptNum[i] * 3; j++){
			buffer0[j] = v[j];
			buffer1[j] = v[j];
		}

		float *src = buffer0;
		float *res = buffer1;

		for (int iter = 0; iter < iteration; iter++){
			for (int j = 1; j < ptNum[i] - 1; j++){
				float *v0 = src + (j - 1) * 3;
				float *v1 = src + j * 3;
				float *v2 = src + (j + 1) * 3;

				for (int k = 0; k < 3; k++){
					float v01 = (v0[k] + v1[k]) * 0.5;
					float v12 = (v1[k] + v2[k]) * 0.5;

					res[j * 3 + k] = (v01 + v12) * 0.5;
				}
			}

			// switcn Src & Res
			float *hold = src;
			src = res;
			res = hold;
		}

		smoothSegments.push_back(src);
		smoothPtNum.push_back(ptNum[i]);

		delete[] res;
	}
}

