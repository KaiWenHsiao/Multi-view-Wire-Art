#include "Solidificator.h"


Solidificator::Solidificator()
{
}


Solidificator::~Solidificator()
{
}

void Solidificator::solidification(const std::vector<float*> &segments, const std::vector<int> &ptNum, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, float radius){
	// Return the number of numbers

	// For Solidification
	float *vBuffer = nullptr;
	float *bBuffer = nullptr;
	float *nBuffer = nullptr;
	int currentNumberCapacity = 0;

	int segmentCount = segments.size();
	std::vector<float> newVersionVertex;
	std::vector<float> newVersionTangent;

	int meshVertexOffset = 0;
	unsigned int meshIndexOffset = 0;


	for (int i = 0; i < segmentCount; i++){
		// Change Data Structure
		newVersionVertex.clear();
		
		// Set First
		float *v = segments[i];
		newVersionVertex.push_back(v[0]);
		newVersionVertex.push_back(v[1]);
		newVersionVertex.push_back(v[2]);
		float *preV = v;
		// 1 ~ n
		for (int j = 1 ; j < ptNum[i] ; j++){
			float *v = segments[i] + j * 3;
			// There are some continuous same value, because of optimization(?)
			if (fabs(v[0] - preV[0]) < 0.00001 &&
				fabs(v[1] - preV[1]) < 0.00001 &&
				fabs(v[2] - preV[2]) < 0.00001){
				continue;
			}
			newVersionVertex.push_back(v[0]);
			newVersionVertex.push_back(v[1]);
			newVersionVertex.push_back(v[2]);

			preV = v;
		}
		// Reset count
		int newVersionPtCount = newVersionVertex.size() / 3;

		if (newVersionPtCount <= 6){
			continue; 
		}

		///////////////////////////////////////////////////////
		// Calcualte tangent with Finite Difference
		glm::vec3 tangent;
		newVersionTangent.clear();

		// First use Backward Difference
		tangent[0] = (newVersionVertex[3] - newVersionVertex[0]);
		tangent[1] = (newVersionVertex[4] - newVersionVertex[1]);
		tangent[2] = (newVersionVertex[5] - newVersionVertex[2]);
		tangent = glm::normalize(tangent);
		newVersionTangent.push_back(tangent[0]);
		newVersionTangent.push_back(tangent[1]);
		newVersionTangent.push_back(tangent[2]);

		// Central use Central Difference
		for (int j = 1; j < newVersionPtCount - 1; j++){
			int pre = j - 1;
			int next = j + 1;
			tangent[0] = (newVersionVertex[next * 3 + 0] - newVersionVertex[pre * 3 + 0]) / 2;
			tangent[1] = (newVersionVertex[next * 3 + 1] - newVersionVertex[pre * 3 + 1]) / 2;
			tangent[2] = (newVersionVertex[next * 3 + 2] - newVersionVertex[pre * 3 + 2]) / 2;
			tangent = glm::normalize(tangent);
			newVersionTangent.push_back(tangent[0]);
			newVersionTangent.push_back(tangent[1]);
			newVersionTangent.push_back(tangent[2]);
		}

		// Last use Forward Difference
		int last = newVersionPtCount - 1;
		int pre = newVersionPtCount - 2;
		tangent[0] = (newVersionVertex[last * 3 + 0] - newVersionVertex[pre * 3 + 0]);
		tangent[1] = (newVersionVertex[last * 3 + 1] - newVersionVertex[pre * 3 + 1]);
		tangent[2] = (newVersionVertex[last * 3 + 2] - newVersionVertex[pre * 3 + 2]);
		tangent = glm::normalize(tangent);
		newVersionTangent.push_back(tangent[0]);
		newVersionTangent.push_back(tangent[1]);
		newVersionTangent.push_back(tangent[2]);
		////////////////////////////////////////////////////////////
		// Create Cylinder// Tubing
		if (vBuffer == nullptr){
			//m_log.append("Create Buffer\n");
			vBuffer = new float[newVersionPtCount * 3];
			bBuffer = new float[newVersionPtCount * 3];
			nBuffer = new float[newVersionPtCount * 3];
			currentNumberCapacity = newVersionPtCount;
		}
		else{
			if (newVersionPtCount > currentNumberCapacity){
				//m_log.append("Reset Buffer\n");
				delete[] vBuffer;
				delete[] bBuffer;
				delete[] nBuffer;
				vBuffer = new float[newVersionPtCount * 3];
				bBuffer = new float[newVersionPtCount * 3];
				nBuffer = new float[newVersionPtCount * 3];
				currentNumberCapacity = newVersionPtCount;
			}
			else{
				//m_log.append("Reuse Buffer\n");
			}
		}
		if (!tubing(vBuffer, bBuffer, nBuffer, newVersionVertex, newVersionTangent)){
			//m_log.append("Tubing Failed\n");
		}
		else{
			// Return number of index
			unsigned int indexCount = createCylinder(meshVertex + meshVertexOffset * 3, meshNormal + meshVertexOffset * 3, meshIndex + meshIndexOffset, meshVertexOffset, vBuffer, newVersionTangent, nBuffer, newVersionPtCount, 12, radius);
			meshVertexOffset = meshVertexOffset + newVersionPtCount * 18;
			meshIndexOffset = meshIndexOffset + indexCount;
		}		
	}

	////////////////////////////////////////////////////
	*vertexCount = meshVertexOffset;
	*indexCount = meshIndexOffset;
}
bool Solidificator::tubing(float *v, float *b, float *n, std::vector<float> &cVertex, std::vector<float> &cTangent){
	glm::vec3 preTangent = { 0, 0, 1 };
	glm::vec3 preNormal = { 0, 1, 0 };
	glm::vec3 preBitangent = { 1, 0, 0 };

	int total = cVertex.size() / 3;
	for (int i = 0; i < total; i++){

		int offset = i * 3;

		// Tangent is zero vector
		if (abs(cTangent[offset + 0]) < 0.00001 && abs(cTangent[offset + 1]) < 0.00001 &&abs(cTangent[offset + 2]) < 0.00001)
			return false;

		v[offset + 0] = cVertex[offset + 0];
		v[offset + 1] = cVertex[offset + 1];
		v[offset + 2] = cVertex[offset + 2];

		glm::vec3 tangent, bitangent, normal, binormal;
		tangent[0] = cTangent[offset + 0];
		tangent[1] = cTangent[offset + 1];
		tangent[2] = cTangent[offset + 2];
		tangent = glm::normalize(tangent);

		if (true){
			// Second Derivative is zero vector
			// Use rotation		

			// Find rotation axis
			glm::vec3 tangentRotAxis = glm::cross(preTangent, tangent);

			if (glm::length(tangentRotAxis) < 0.001){
				// Same tangent, no rotation
				tangent = preTangent;
				normal = preNormal;
				bitangent = glm::cross(normal, tangent);
				bitangent = glm::normalize(bitangent);
			}
			else{
				tangentRotAxis = glm::normalize(tangentRotAxis);
				// Find rotaiont matrix
				float dot = glm::dot(tangent, preTangent);
				if (dot > 1.0)
					dot = 1.0;
				float rad = glm::acos(dot);
				// Rotation matrix
				glm::mat4x4 rotMat = glm::rotate(rad, tangentRotAxis);
				// Rotate normal		
				glm::vec4 temp = rotMat * glm::vec4(preNormal[0], preNormal[1], preNormal[2], 0);
				normal[0] = temp[0];
				normal[1] = temp[1];
				normal[2] = temp[2];
				// Compute bitangent
				bitangent = glm::cross(normal, tangent);
				bitangent = glm::normalize(bitangent);
			}
		}


		// Assign TBN to buffer
		n[offset + 0] = normal[0];
		n[offset + 1] = normal[1];
		n[offset + 2] = normal[2];

		b[offset + 0] = bitangent[0];
		b[offset + 1] = bitangent[1];
		b[offset + 2] = bitangent[2];

		// Origin tangent may be zero vector
		//cTangent[offset + 0] = tangent[0];
		//cTangent[offset + 1] = tangent[1];
		//cTangent[offset + 2] = tangent[2];

		// Assign to Pre-data
		preTangent = tangent;
		preNormal = normal;
	}

	return true;
}
unsigned int Solidificator::createCylinder(float *meshVertex, float *meshNormal, unsigned int *meshIndex, unsigned int indexBase, float *vertex, std::vector<float> &tangent, float *normal, int vertexCount, int section, float r){
	// Return index Count

	float perPhi = glm::radians(360.0 / section);
	float radius = r;

	// Last = start
	glm::vec3 *selfCircle = new glm::vec3[section + 1];
	glm::vec3 *nextCircle = new glm::vec3[section + 1];

	glm::vec3 *currVertex = new glm::vec3[section * 6];
	glm::vec3 *currNormal = new glm::vec3[section * 6];

	int vertexBufferOffset = 0;

	// Create Vertices
	for (int i = 0; i < vertexCount; i++){
		// Get vertex, tangent, normal
		glm::vec3 selfVertex = { vertex[i * 3 + 0], vertex[i * 3 + 1], vertex[i * 3 + 2] };
		glm::vec3 selfTangent = { tangent[i * 3 + 0], tangent[i * 3 + 1], tangent[i * 3 + 2] };
		glm::vec3 selfNormal = { normal[i * 3 + 0], normal[i * 3 + 1], normal[i * 3 + 2] };

		// Compute circle vertex
		for (int j = 0; j < section; j++){
			// Compute rotation matrix
			glm::mat4x4 rot = glm::rotate(perPhi * j, selfTangent);
			// Rotate the normal
			glm::vec4 currVec = rot * glm::vec4(selfNormal, 0);
			// Normalize
			glm::vec3 curr = { currVec[0], currVec[1], currVec[2] };
			curr = glm::normalize(curr);
			// Compute vertex
			selfCircle[j] = selfVertex + radius * curr;
		}

		// To the buffer
		for (int j = 0; j < section; j++){
			int offset = i * section * 3;
			meshVertex[offset + j * 3 + 0] = selfCircle[j].x;
			meshVertex[offset + j * 3 + 1] = selfCircle[j].y;
			meshVertex[offset + j * 3 + 2] = selfCircle[j].z;

			glm::vec3 normal = selfCircle[j] - selfVertex;
			normal = glm::normalize(normal);
			meshNormal[offset + j * 3 + 0] = normal[0];
			meshNormal[offset + j * 3 + 1] = normal[1];
			meshNormal[offset + j * 3 + 2] = normal[2];
		}
	}

	// Create Triangle
	unsigned int indexOffset = 0;
	unsigned int uiSection = (unsigned int)section;
	for (unsigned int i = 0; i < vertexCount - 1; i++){
		unsigned int offset = indexBase + i * uiSection;
		unsigned int j = 0;
		for (j = 0; j < section - 1; j++){
			meshIndex[indexOffset + 0] = offset + j;
			meshIndex[indexOffset + 1] = offset + j + 1;
			meshIndex[indexOffset + 2] = offset + j + uiSection + 1;
			meshIndex[indexOffset + 3] = offset + j + uiSection + 1;
			meshIndex[indexOffset + 4] = offset + j + uiSection;
			meshIndex[indexOffset + 5] = offset + j;
			indexOffset += 6;
		}
		meshIndex[indexOffset + 0] = offset + j;
		meshIndex[indexOffset + 1] = offset;
		meshIndex[indexOffset + 2] = offset + uiSection;
		meshIndex[indexOffset + 3] = offset + uiSection;
		meshIndex[indexOffset + 4] = offset + j + uiSection;
		meshIndex[indexOffset + 5] = offset + j;
		indexOffset += 6;
	}

	return indexOffset;
}
void Solidificator::addJointToSphere(float *joint, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, int indexOffset, float radius){
	// Num vertices = 1 + (parallels - 1) * meridians + 1
	const int parallels = 12;
	const int meridians = 12;
	const int NUM_VERTICES = 1 + (parallels - 1) * meridians + 1;

	// Add north pole vertex
	meshVertex[0] = 0.0 + joint[0];
	meshVertex[1] = radius + joint[1];
	meshVertex[2] = 0.0 + joint[2];

	meshNormal[0] = 0.0;
	meshNormal[1] = 1.0;
	meshNormal[2] = 0.0;

	int vertexOffset = 3;
	// Quads
	for (unsigned int j = 0; j < parallels - 1; j++){
		const double polar = glm::pi<double>() * double(j + 1) / double(parallels);
		const double sp = glm::sin(polar);
		const double cp = glm::cos(polar);

		for (unsigned int i = 0; i < meridians; i++){
			const double azimuth = 2.0 * glm::pi<double>() * double(i) / double(meridians);
			const double sa = glm::sin(azimuth);
			const double ca = glm::cos(azimuth);
			const double x = sp * ca;
			const double y = cp;
			const double z = sp * sa;

			meshVertex[vertexOffset + 0] = radius * x + joint[0];
			meshVertex[vertexOffset + 1] = radius * y + joint[1];
			meshVertex[vertexOffset + 2] = radius * z + joint[2];

			meshNormal[vertexOffset + 0] = x;
			meshNormal[vertexOffset + 1] = y;
			meshNormal[vertexOffset + 2] = z;

			vertexOffset += 3;
		}
	}

	// South pole
	meshVertex[vertexOffset + 0] = 0.0 + joint[0];
	meshVertex[vertexOffset + 1] = -1 * radius + joint[1];
	meshVertex[vertexOffset + 2] = 0.0 + joint[2];

	meshNormal[vertexOffset + 0] = 0.0;
	meshNormal[vertexOffset + 1] = -1;
	meshNormal[vertexOffset + 2] = 0.0;

	//////////////////////////////////////////////////////
	// Create triangles
	int numIndex = 0;
	for (unsigned int i = 0; i < meridians; i++){
		const unsigned int a = i + 1;
		const unsigned int b = (i + 1) % meridians + 1;

		meshIndex[numIndex + 0] = 0 + indexOffset;
		meshIndex[numIndex + 1] = b + indexOffset;
		meshIndex[numIndex + 2] = a + indexOffset;
		numIndex += 3;
	}
	for (unsigned int j = 0; j < parallels - 2; j++){
		unsigned int aStart = j * meridians + 1;
		unsigned int bStart = (j + 1) * meridians + 1;

		for (unsigned int i = 0; i < meridians; i++){
			const unsigned int a = aStart + i;
			const unsigned int a1 = aStart + (i + 1) % meridians;
			const unsigned int b = bStart + i;
			const unsigned int b1 = bStart + (i + 1) % meridians;

			meshIndex[numIndex + 0] = b1 + indexOffset;
			meshIndex[numIndex + 1] = b + indexOffset;
			meshIndex[numIndex + 2] = a + indexOffset;
			meshIndex[numIndex + 3] = a + indexOffset;
			meshIndex[numIndex + 4] = a1 + indexOffset;
			meshIndex[numIndex + 5] = b1 + indexOffset;
			numIndex += 6;
		}
	}
	for (unsigned int i = 0; i < meridians; i++){
		const unsigned int a = i + meridians * (parallels - 2) + 1;
		const unsigned int b = (i + 1) % meridians + meridians * (parallels - 2) + 1;

		meshIndex[numIndex + 0] = NUM_VERTICES - 1 + indexOffset;
		meshIndex[numIndex + 1] = b + +indexOffset;
		meshIndex[numIndex + 2] = a + indexOffset;
		numIndex += 3;
	}
	
	*vertexCount = NUM_VERTICES;
	*indexCount = numIndex;	
}
void Solidificator::getSphere(float *joint, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, int indexOffset, float radius){
	
}

void Solidificator::addJointGeometry(const std::vector<float> &firstLastLock, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, const int indexOffset, float radius){
	std::vector<float> readyBuffer;
	int numVertex = 0;
	int numIndex = 0;
	int io = indexOffset;

	int firstLastCount = firstLastLock.size() / 3;
	for (int i = 0; i < firstLastCount; i++){
		glm::vec3 src = {
			firstLastLock[i * 3 + 0],
			firstLastLock[i * 3 + 1],
			firstLastLock[i * 3 + 2]
		};
		// Check if there is a sphere on the vertex
		bool readyFlag = false;
		int readyVertexCount = readyBuffer.size() / 3;
		for (int j = 0; j < readyVertexCount; j++){
			glm::vec3 ready = {
				readyBuffer[j * 3 + 0],
				readyBuffer[j * 3 + 1],
				readyBuffer[j * 3 + 2],
			};

			if (glm::length(src - ready) < 0.0001){
				readyFlag = true;
				break;
			}
		}
		if (readyFlag){
			continue;
		}
		/////////////////////////////////////////
		// Record
		readyBuffer.push_back(src[0]);
		readyBuffer.push_back(src[1]);
		readyBuffer.push_back(src[2]);

		// Create Geometry
		int v, i;
		addJointToSphere(glm::value_ptr(src), meshVertex + numVertex * 3, meshNormal + numVertex * 3, meshIndex + numIndex, &v, &i, io, radius);
		numVertex = numVertex + v;
		numIndex = numIndex + i;
		io = io + v;
	}

	*vertexCount = numVertex;
	*indexCount = numIndex;
}
bool Solidificator::createOBJ(std::string fileName, float *vertex, float *normal, unsigned int *index, int vertexCount, int indexCount){
	std::ofstream output(fileName);
	if (!output.is_open()){
		return false;
	}


	// Write v
	for (int i = 0; i < vertexCount; i++){
		output << "v " << vertex[i * 3 + 0] << " " << vertex[i * 3 + 1] << " " << vertex[i * 3 + 2] << std::endl;
	}
	// Write vn
	for (int i = 0; i < vertexCount; i++){
		output << "vn " << normal[i * 3 + 0] << " " << normal[i * 3 + 1] << " " << normal[i * 3 + 2] << std::endl;
	}
	// Write f
	int faceCount = indexCount / 3;
	for (int i = 0; i < faceCount; i++){
		output << "f " << index[i * 3 + 0] + 1 << "//" << index[i * 3 + 0] + 1 << " " <<
			index[i * 3 + 1] + 1 << "//" << index[i * 3 + 1] + 1 << " " <<
			index[i * 3 + 2] + 1 << "//" << index[i * 3 + 2] + 1 << std::endl;
	}
	output.close();

	return true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Solidificator::getCube(float *joint, float *meshVertex, float *meshNormal, unsigned int *meshIndex, int *vertexCount, int *indexCount, int indexOffset, float size){
	static float vData[] = {
		- 1.000000, - 1.000000, 1.000000,
		- 1.000000, 1.000000, 1.000000,
		- 1.000000, - 1.000000, - 1.000000,
		- 1.000000, 1.000000, - 1.000000,
		1.000000, - 1.000000, 1.000000,
		1.000000, 1.000000, 1.000000,
		1.000000, - 1.000000, - 1.000000,
		1.000000, 1.000000, - 1.000000
	};
	static float nData[] = {
		-1.000000, 0.000000, 0.000000,
		0.000000, 0.000000, -1.000000,
		1.000000, 0.000000, 0.000000,
		0.000000, 0.000000, 1.000000,
		0.000000, -1.000000, 0.000000,
		0.000000, 1.000000, 0.000000
	};
	static int iData[] = {
		4, 1, 3, 1, 1, 1,
		8, 2, 7, 2, 3, 2,
		6, 3, 5, 3, 7, 3,
		2, 4, 1, 4, 5, 4,
		3, 5, 7, 5, 5, 5,
		8, 6, 4, 6, 2, 6,
		2, 1, 4, 1, 1, 1,
		4, 2, 8, 2, 3, 2,
		8, 3, 6, 3, 7, 3,
		6, 4, 2, 4, 5, 4,
		1, 5, 3, 5, 5, 5,
		6, 6, 8, 6, 2, 6
	};

	for (int i = 0; i < 8; i++){

	}

	for (int i = 0; i < 36; i++){
		int offset = i * 3;

		float *v = vData + iData[i * 2 + 0];
		meshVertex[offset + 0] = v[0] * size + joint[0];
		meshVertex[offset + 1] = v[1] * size + joint[1];
		meshVertex[offset + 2] = v[2] * size + joint[2];

		float *n = vData + iData[i * 2 + 1];
		meshNormal[offset + 0] = n[0];
		meshNormal[offset + 1] = n[1];
		meshNormal[offset + 2] = n[2];
	}
}
void Solidificator::createVoxelStructureOBJ(const std::string &fileName, float *meshVertex, float *meshNormal, int numVertex){
	std::ofstream output(fileName);
	if (!output.is_open()){
		return;
	}

	// Write V
	for (int i = 0; i < numVertex; i++){
		output << "v " << meshVertex[i * 3 + 0] << " " << meshVertex[i * 3 + 1] << " " << meshVertex[i * 3 + 2] << "\n";
	}
	output << std::endl;
	// Write VN
	for (int i = 0; i < numVertex; i++){
		output << "vn " << meshNormal[i * 3 + 0] << " " << meshNormal[i * 3 + 1] << " " << meshNormal[i * 3 + 2] << "\n";
	}
	output << std::endl;
	// Write f
	int numFace = numVertex / 3;
	for (int i = 0; i < numFace; i++){
		output << "f " << i * 3 + 1 << "//" << i * 3 + 1 << " " <<
			i * 3 + 2 << "//" << i * 3 + 2 << " " <<
			i * 3 + 3 << "//" << i * 3 + 3 << "\n";
	}
	output << std::endl;

	output.close();	
}