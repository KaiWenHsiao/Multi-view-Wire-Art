#include "LineWidget.h"


LineWidget::LineWidget()
{
}


LineWidget::~LineWidget()
{
}

unsigned char LineWidget::isNeighbor(const float *seg0, const int ptNum0, const float *seg1, const int ptNum1){
	// 0, S1 is not a neighbor of S0
	// 1, S1 is the head neighbor of S0
	// 2, S1 is the tail neighbor of S0

	const float *head = seg0;
	const float *tail = seg0 + (ptNum0 - 1) * 3;
	const float *othHead = seg1;
	const float *othTail = seg1 + (ptNum1 - 1) * 3;
	const float THRESHOLD = 0.000000001;
	//////////////////////////////////////////////////////////////////
	// Seg0 Head -> Seg1 Head & Tail
	if (fabs(othHead[0] - head[0]) < THRESHOLD && fabs(othHead[1] - head[1]) < THRESHOLD && fabs(othHead[2] - head[2]) < THRESHOLD){
		return 1;
	}
	if (fabs(othTail[0] - head[0]) < THRESHOLD && fabs(othTail[1] - head[1]) < THRESHOLD && fabs(othTail[2] - head[2]) < THRESHOLD){
		return 1;
	}

	// Seg0 Tail -> Seg1 Head & Tail
	if (fabs(othHead[0] - tail[0]) < THRESHOLD && fabs(othHead[1] - tail[1]) < THRESHOLD && fabs(othHead[2] - tail[2]) < THRESHOLD){
		return 2;
	}
	if (fabs(othTail[0] - tail[0]) < THRESHOLD && fabs(othTail[1] - tail[1]) < THRESHOLD && fabs(othTail[2] - tail[2]) < THRESHOLD){
		return 2;
	}

	return 0;
}
unsigned char LineWidget::linkRelation(const float *seg0, const int ptNum0, const float *seg1, const int ptNum1){
	const float *head = seg0;
	const float *tail = seg0 + (ptNum0 - 1) * 3;
	const float *othHead = seg1;
	const float *othTail = seg1 + (ptNum1 - 1) * 3;
	const float THRESHOLD = 0.000000001;
	//////////////////////////////////////////////////////////////////
	// Seg0 Head -> Seg1 Head & Tail
	if (fabs(othHead[0] - head[0]) < THRESHOLD && fabs(othHead[1] - head[1]) < THRESHOLD && fabs(othHead[2] - head[2]) < THRESHOLD){
		return 1;
	}
	if (fabs(othTail[0] - head[0]) < THRESHOLD && fabs(othTail[1] - head[1]) < THRESHOLD && fabs(othTail[2] - head[2]) < THRESHOLD){
		return 2;
	}

	// Seg0 Tail -> Seg1 Head & Tail
	if (fabs(othHead[0] - tail[0]) < THRESHOLD && fabs(othHead[1] - tail[1]) < THRESHOLD && fabs(othHead[2] - tail[2]) < THRESHOLD){
		return 3;
	}
	if (fabs(othTail[0] - tail[0]) < THRESHOLD && fabs(othTail[1] - tail[1]) < THRESHOLD && fabs(othTail[2] - tail[2]) < THRESHOLD){
		return 4;
	}

	return 0;
}
void LineWidget::setupAdjacencyMatrix(const std::vector<float*> &segments, const std::vector<int> &ptNums, unsigned char **adjacencyMatrix){
	int numSeg = segments.size();
	
	for (int i = 0; i < numSeg; i++){
		adjacencyMatrix[i][i] = 0;
		for (int j = i + 1; j < numSeg; j++){
			int stament = LineWidget::linkRelation(segments[i], ptNums[i], segments[j], ptNums[j]);
			if (stament == 0){
				adjacencyMatrix[i][j] = 0;
				adjacencyMatrix[j][i] = 0;
			}
			else if (stament == 1){
				// head -> head
				adjacencyMatrix[i][j] = 1;
				adjacencyMatrix[j][i] = 1;
			}
			else if (stament == 2){
				// head -> tail
				adjacencyMatrix[i][j] = 1;
				adjacencyMatrix[j][i] = 2;
			}
			else if (stament == 3){
				// tail -> head
				adjacencyMatrix[i][j] = 2;
				adjacencyMatrix[j][i] = 1;
			}
			else if (stament == 4){
				// tail -> tail
				adjacencyMatrix[i][j] = 2;
				adjacencyMatrix[j][i] = 2;
			}			
		}
	}
}
void LineWidget::projectLine(float *seg, const int ptNum, const glm::mat4 &viewProj, cv::Mat &mask){
	// Vertices after proj buffer
	std::vector<int> verticesProj(ptNum * 2);
	verticesProj.clear();

	int imageHeight = mask.rows;
	int imageWidth = mask.cols;

	// Proj All Vertices
	for (int j = 0; j < ptNum; j++){
		float *v = seg + 3 * j;
		glm::ivec2 pixel;
		LineWidget::project(pixel, v, viewProj, imageWidth, imageHeight);		

		verticesProj.push_back(pixel[0]);
		verticesProj.push_back(pixel[1]);
	}

	// Draw Line
	int verticesSize = verticesProj.size() / 2;
	for (int k = 0; k < verticesSize - 1; k++){
		cv::line(
			mask,
			cv::Point(verticesProj[k * 2 + 0], verticesProj[k * 2 + 1]),
			cv::Point(verticesProj[(k + 1) * 2 + 0], verticesProj[(k + 1) * 2 + 1]),
			cv::Scalar(255),
			4,
			8,
			0
			);
	}	
}
void LineWidget::project(glm::ivec2 &pixel, float *vertex, const glm::mat4 &viewProj, int w, int h){
	// Create vector
	glm::vec4 v = glm::vec4(vertex[0], vertex[1], vertex[2], 1);

	// Compute mapping pixel 
	glm::vec4 p = viewProj * v;

	// [-1 1] -> [0 1]
	const static glm::mat4x4 bias = glm::mat4x4(
		0.5, 0, 0, 0,
		0, 0.5, 0, 0,
		0, 0, 0.5, 0,
		0.5, 0.5, 0.5, 1.0);
	p = bias * (p / p.w);

	// To image space
	pixel[0] = (int)(p[0] * w);
	pixel[1] = (int)(p[1] * h);

	pixel[1] = h - pixel[1];
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool LineWidget::writeSimpleSkeletonFile(std::string file, const std::vector<float*> &segments, const std::vector<int> &ptNum){

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
bool LineWidget::readSimpleSkeletonFile(std::string file, std::vector<float*> &segments, std::vector<int> &ptNum){
	std::ifstream input(file, std::ios::binary);
	if (!input.is_open()){
		return false;
	}

	// Read Number of segments
	int segmentCount;
	input.read((char*)(&segmentCount), 4);

	for (int i = 0; i < segmentCount; i++){
		// Read Number of Vertices
		int numVertices;
		input.read((char*)(&numVertices), 4);
		ptNum.push_back(numVertices);

		// Write Vertices
		float *ptr = new float[numVertices * 3];
		input.read((char*)(ptr), numVertices * 12);
		segments.push_back(ptr);
	}

	// Finish
	input.close();

	return true;
}////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool LineWidget::isSameLine(float *seg0, int ptNum0, float *seg1, int ptNum1){
	if (ptNum0 != ptNum1)
		return false;

	for (int i = 0; i < ptNum0; i++){
		float *v0 = seg0 + i * 3;
		float *v1 = seg1 + i * 3;
		if (fabs(v0[0] - v1[0]) > 0.001 || fabs(v0[1] - v1[1]) > 0.001 || fabs(v0[2] - v1[2]) > 0.001){
			return false;
		}
	}

	return true;
}
