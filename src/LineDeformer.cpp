#include "LineDeformer.h"


LineDeformer::LineDeformer(Source *s)
{
	m_source = s;
}


LineDeformer::~LineDeformer()
{
}



void LineDeformer::getParametricCurves(const vector<float*> &segments, const vector<int> &ptNum, vector<Spline3D*> &curves, std::vector<float> &firstLastLock, float fr){
	// Prepare Spline
	HSSSpline::PathPoints<3> points;
	HSSSpline::Samples samples;

	// For preserving First & Last
	float *first, *last;

	int numSegment = segments.size();
	for (int i = 0; i < numSegment; i++){
		points.val.clear();

		// Create Spline
		Spline3D *spline = new Spline3D();
		for (int j = 0; j < ptNum[i]; j++){
			float *v = segments[i] + j * 3;
			HSSSpline::PathPoint<3> p;
			p[0] = v[0];
			p[1] = v[1];
			p[2] = v[2];

			points.val.push_back(p);
		}
		first = segments[i];
		last = segments[i] + (ptNum[i] - 1) * 3;

		spline->assignPoints(points);

		float fitRatio = 0;
		float length = points.val.size();

		// Auto Fit Ratio definition
		if (fr <= 0){
			if (length >= 300){
				fitRatio = 0.005;
			}
			else if (length >= 200){
				fitRatio = 0.01;
			}
			else if (length >= 100){
				fitRatio = 0.05;
			}
			else{
				fitRatio = 0.1;
			}
		}
		else{
			fitRatio = fr;
		}


		// Fit Curve
		spline->fittingCurve(2.0);
		//spline->UniformSampling(samples, 10);

		// Record First and Last
		firstLastLock.push_back(first[0]);
		firstLastLock.push_back(first[1]);
		firstLastLock.push_back(first[2]);
		firstLastLock.push_back(last[0]);
		firstLastLock.push_back(last[1]);
		firstLastLock.push_back(last[2]);

		curves.push_back(spline);

		m_curves.push_back(spline);

		// Fix first & last
		HSSSpline::PathPoints<3> &ctrls = spline->getCtrlPoints();
		int numCtrl = ctrls.val.size();
		ctrls[0][0] = first[0];
		ctrls[0][1] = first[1];
		ctrls[0][2] = first[2];
		ctrls[numCtrl - 1][0] = last[0];
		ctrls[numCtrl - 1][1] = last[1];
		ctrls[numCtrl - 1][2] = last[2];		

		spline->reFitCurve();
	}
}
void LineDeformer::initCurve(const std::vector<float*> &segs, const std::vector<int> &ptNums, std::vector<Spline3D*> &curves, std::vector<float> &firstLastLock){
	// First setup need parametric curves
	this->getParametricCurves(segs, ptNums, curves, firstLastLock, -1);

	// Set up adjacency matrix
	int numSeg = segs.size();
	m_adjacencyMat = new unsigned char*[numSeg];
	for (int i = 0; i < numSeg; i++){
		m_adjacencyMat[i] = new unsigned char[numSeg];
	}
	LineWidget::setupAdjacencyMatrix(segs, ptNums, m_adjacencyMat);
	m_adjacencyMatSize = numSeg; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
glm::vec3 LineDeformer::getDeformVector_180124(float *src){
	glm::vec3 resDV = { 0, 0, 0 };
	int numImage = m_source->m_ioList.size();	

	for (int i = 0; i < numImage; i++){
		int nearestPixel[2];
		m_source->m_ioList[i]->getNearestThinContourPixel(src, nearestPixel);

		// Get projected pixel
		int srcPixel[2];
		m_source->m_ioList[i]->getToleranceProj(src, srcPixel, 3);

		// Get vertex in world space
		glm::vec3 worldNP = m_source->m_ioList[i]->getWorldSpaceVertex(nearestPixel);
		glm::vec3 worldSRC = m_source->m_ioList[i]->getWorldSpaceVertex(srcPixel);

		glm::vec3 dv = worldNP - worldSRC;
		float f = glm::length(glm::vec3(src[0], src[1], src[2]) - m_source->m_ioList[i]->m_viewPos);
		float n = glm::length(worldSRC - m_source->m_ioList[i]->m_viewPos);

		dv = dv * f / n;	
		float srcDT = m_source->m_ioList[i]->getDistanceTransformVerThin(src);		
		
		resDV = resDV + dv * this->getDeformVectorWeight(srcDT) * 1.0f ;		
	}

	return resDV;
}
float LineDeformer::getDeformVectorWeight(float x){
	x = x * m_dtWeight;

	const float MU = 0.0;
	
	float tmp = glm::exp(-1 * x * x / (2 * m_sigma * m_sigma));
	float f = tmp / sqrt(2 * glm::pi<float>() * m_sigma * m_sigma);

	return f;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LineDeformer::initDeformProcess(const std::vector<Spline3D*> &curves, const std::vector<float> &firstLastLock){
	int numCurve = curves.size();
	
	////////////////////////////////////////////////////////////////////
	// Create Two Buffers
	m_buffer0 = new CurveStructure;
	m_buffer0->curveCtrls = std::vector<float*>(numCurve);
	m_buffer0->numCurveCtrl = std::vector<int>(numCurve);

	m_buffer1 = new CurveStructure;
	m_buffer1->curveCtrls = std::vector<float*>(numCurve);
	m_buffer1->numCurveCtrl = std::vector<int>(numCurve);

	// Create initial case and set to buffer0, then prepare buffer1
	for (int i = 0; i < numCurve; i++){
		HSSSpline::PathPoints<3> &ctrls = curves[i]->getCtrlPoints();
		int numCtrl = ctrls.val.size();
		
		float *ctrlBuffer0 = new float[numCtrl * 3];
		m_buffer0->curveCtrls[i] = ctrlBuffer0;
		m_buffer0->numCurveCtrl[i] = numCtrl;

		float *ctrlBuffer1 = new float[numCtrl * 3];
		m_buffer1->curveCtrls[i] = ctrlBuffer1;
		m_buffer1->numCurveCtrl[i] = numCtrl;
	}

	int numNum = firstLastLock.size();
	m_buffer0->firstLastLock = std::vector<float>(numNum);
	m_buffer1->firstLastLock = std::vector<float>(numNum);

	this->setUpInitialCurveStructure(curves, firstLastLock);	

	m_sampleBuffer = new float[10000 * 3];
}
void LineDeformer::setUpInitialCurveStructure(const std::vector<Spline3D*> &curves, const std::vector<float> &firstLastLock){
	int numCurve = curves.size();
	// Create initial case and set to buffer0, then prepare buffer1
	for (int i = 0; i < numCurve; i++){
		HSSSpline::PathPoints<3> &ctrls = curves[i]->getCtrlPoints();
		int numCtrl = ctrls.val.size();
		float *ctrlBuffer0 = m_buffer0->curveCtrls[i];

		for (int j = 0; j < numCtrl; j++){
			ctrlBuffer0[j * 3 + 0] = ctrls[j][0];
			ctrlBuffer0[j * 3 + 1] = ctrls[j][1];
			ctrlBuffer0[j * 3 + 2] = ctrls[j][2];
		}	
	}

	int numNum = firstLastLock.size();
	for (int i = 0; i < numNum; i++){
		m_buffer0->firstLastLock[i] = firstLastLock[i];
	}	

	
}
void LineDeformer::deform_180125(std::vector<Spline3D*> &curves, std::vector<float> &firstLastLock, const std::string &processedFileName, bool outputProcessedSS){
	this->setUpInitialCurveStructure(curves, firstLastLock);

	int numCurve = curves.size();

	ProgressTestSender::Instance()->addLog("Start Deform");
	int numDeformStep = 0;
	while (true){

		if (numDeformStep < 300 && outputProcessedSS){
			this->outputCurveSamples(processedFileName + "_" + std::to_string(numDeformStep) + ".ss", curves);
		}

		if (!this->getDeformedStructure_180125(curves, m_buffer0->firstLastLock, m_buffer1, 0.1, outputProcessedSS)){			
			break;
		}

		numDeformStep++;

		if (numDeformStep >= 1000){
			break;
		}

		// Exchange
		CurveStructure *hold = m_buffer0;
		m_buffer0 = m_buffer1;
		m_buffer1 = hold;
	}

	ProgressTestSender::Instance()->addLog("Num Deform Step: " + std::to_string(numDeformStep));

	///////////////////////////////////////////////////////////////////
	// Set to best
	for (int i = 0; i < numCurve; i++){
		HSSSpline::PathPoints<3> &ctrls = curves[i]->getCtrlPoints();
		int numCtrl = ctrls.val.size();
		float *ctrlBuffer = m_buffer0->curveCtrls[i];

		for (int j = 0; j < numCtrl; j++){
			ctrls[j][0] = ctrlBuffer[j * 3 + 0];
			ctrls[j][1] = ctrlBuffer[j * 3 + 1];
			ctrls[j][2] = ctrlBuffer[j * 3 + 2];
		}

		ctrls[0][0] = m_buffer0->firstLastLock[i * 6 + 0];
		ctrls[0][1] = m_buffer0->firstLastLock[i * 6 + 1];
		ctrls[0][2] = m_buffer0->firstLastLock[i * 6 + 2];

		ctrls[numCtrl - 1][0] = m_buffer0->firstLastLock[i * 6 + 3];
		ctrls[numCtrl - 1][1] = m_buffer0->firstLastLock[i * 6 + 4];
		ctrls[numCtrl - 1][2] = m_buffer0->firstLastLock[i * 6 + 5];

		curves[i]->reFitCurve();
	}

	int numNum = firstLastLock.size();
	for (int i = 0; i < numNum; i++){
		firstLastLock[i] = m_buffer0->firstLastLock[i];
	}		
}
bool LineDeformer::getDeformedStructure_180125(const std::vector<Spline3D*> &curves, const std::vector<float> &firstLastLock, CurveStructure *res, float translateRatio, bool requireReFit){
	std::vector<glm::vec3> deformVector;
	//std::vector<glm::vec3> gaussianDeformVector;
	std::vector<glm::vec3> firstLastLockDeformVector;

	// Deform target is Ctrl Points
	int numCurve = curves.size();
	
	// Get minimum deform vector & maximum deform vector
	float minDeformVectorMagnitude = 100;
	float maxDeformVectorMagnitude = -1;

	for (int i = 0; i < numCurve; i++){
		// Get Ctrl Points
		HSSSpline::PathPoints<3> &ctrls = curves[i]->getCtrlPoints();
		int numCtrl = ctrls.val.size();

		for (int j = 0; j < numCtrl; j++){
			HSSSpline::PathPoint<3> &p = ctrls.val.at(j);
			float src[3] = { p[0], p[1], p[2] };
			float res[3] = { 0 };

			glm::vec3 dv = this->getDeformVector_180124(src);
			float mag = glm::length(dv);
			deformVector.push_back(dv);

			if (mag < minDeformVectorMagnitude){
				minDeformVectorMagnitude = mag;
			}
			else if (mag > maxDeformVectorMagnitude){
				maxDeformVectorMagnitude = mag;
			}
		}

		// Deform first
		float firstLock[] = { firstLastLock[i * 6 + 0], firstLastLock[i * 6 + 1], firstLastLock[i * 6 + 2] };
		float firstLockRes[3];
		glm::vec3 firstLockDeformVector = this->getDeformVector_180124(firstLock);
		float mag = glm::length(firstLockDeformVector);
		firstLastLockDeformVector.push_back(firstLockDeformVector);
		if (mag < minDeformVectorMagnitude){
			minDeformVectorMagnitude = mag;
		}
		else if (mag > maxDeformVectorMagnitude){
			maxDeformVectorMagnitude = mag;
		}

		// Deform Last
		float lastLock[] = { firstLastLock[i * 6 + 3], firstLastLock[i * 6 + 4], firstLastLock[i * 6 + 5] };
		float lastLockRes[3];		
		glm::vec3 lastLockDeformVector = this->getDeformVector_180124(lastLock);
		firstLastLockDeformVector.push_back(lastLockDeformVector);
		mag = glm::length(lastLockDeformVector);
		if (mag < minDeformVectorMagnitude){
			minDeformVectorMagnitude = mag;
		}
		else if (mag > maxDeformVectorMagnitude){
			maxDeformVectorMagnitude = mag;
		}			
	}
	// Record
	minimumDeformMagnitudes.push_back(minDeformVectorMagnitude);
	maximumDeformMagnitudes.push_back(maxDeformVectorMagnitude);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// If magnitude < minimum deform vector, the deform process is end
	if (maxDeformVectorMagnitude < 0.1){
		return false;
	}

	// Calculate the deformed structure
	int dvOffset = 0;
	for (int i = 0; i < numCurve; i++){
		// Get Ctrl Points
		HSSSpline::PathPoints<3> &ctrls = curves[i]->getCtrlPoints();
		int numCtrl = ctrls.val.size();

		// Apply & Saving the result
	
		if (numCtrl != res->numCurveCtrl[i]){
			// Rebuild a buffer
			if (numCtrl > res->numCurveCtrl[i]){
				delete[] res->curveCtrls[i];
				res->curveCtrls[i] = new float[numCtrl * 3];
			}			
			res->numCurveCtrl[i] = numCtrl;
		}
		else{
			// Use existed
		}
		float *buffer = res->curveCtrls[i];
		for (int j = 0; j < numCtrl; j++){		
			HSSSpline::PathPoint<3> &p = ctrls.val.at(j);
			const glm::vec3 &r = deformVector[dvOffset];

			p[0] = p[0] + translateRatio * r[0];
			p[1] = p[1] + translateRatio * r[1];
			p[2] = p[2] + translateRatio * r[2];

			buffer[j * 3 + 0] = p[0];
			buffer[j * 3 + 1] = p[1];
			buffer[j * 3 + 2] = p[2];		

			dvOffset++;
		}		

		float firstLock[] = { firstLastLock[i * 6 + 0], firstLastLock[i * 6 + 1], firstLastLock[i * 6 + 2] };
		glm::vec3 dv = firstLastLockDeformVector[i * 2 + 0];
		ctrls[0][0] = firstLock[0] + translateRatio * dv.x;
		ctrls[0][1] = firstLock[1] + translateRatio * dv.y;
		ctrls[0][2] = firstLock[2] + translateRatio * dv.z;

		float lastLock[] = { firstLastLock[i * 6 + 3], firstLastLock[i * 6 + 4], firstLastLock[i * 6 + 5] };
		dv = firstLastLockDeformVector[i * 2 + 1];
		ctrls[numCtrl - 1][0] = lastLock[0] + translateRatio * dv.x;
		ctrls[numCtrl - 1][1] = lastLock[1] + translateRatio * dv.y;
		ctrls[numCtrl - 1][2] = lastLock[2] + translateRatio * dv.z;		

		res->firstLastLock[i * 6 + 0] = ctrls[0][0];
		res->firstLastLock[i * 6 + 1] = ctrls[0][1];
		res->firstLastLock[i * 6 + 2] = ctrls[0][2];

		res->firstLastLock[i * 6 + 3] = ctrls[numCtrl - 1][0];
		res->firstLastLock[i * 6 + 4] = ctrls[numCtrl - 1][1];
		res->firstLastLock[i * 6 + 5] = ctrls[numCtrl - 1][2];
		//////////////////////////////////////////////////////////////////////////////////////////////////////
		// Apply & Saving the result
		if (requireReFit){
			curves[i]->reFitCurve();
		}			
	}		

	return true;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
void LineDeformer::outputCurveSamples(const std::string &fileName, const std::vector<Spline3D*> &curves){
	// Open file
	std::ofstream output(fileName, ios::binary);
	if (!output.is_open()){
		return;
	}

	int numCurve = curves.size();
	output.write((char*)(&numCurve), 4);

	for (int i = 0; i < numCurve; i++){
		int numNum = 0;
		curves[i]->getLineVertex(m_sampleBuffer, &numNum, 0.1);

		int numVertex = numNum / 3;
		output.write((char*)(&numVertex), 4);

		output.write((char*)m_sampleBuffer, numNum * 4);
	}

	output.close();
}
void LineDeformer::getCurveSamples(vector<float*> &segments, vector<int> &ptNum, const std::vector<Spline3D*> &curves, const std::vector<float> &firstLastLock){
	float *buffer = new float[10000];

	for (int i = 0; i < curves.size(); i++){
		Spline3D *spline = curves[i];

		/////////////////////////////////////////////////////////////////
		int count = 0;
		spline->getLineVertex(buffer, &count, 0.1);
		
		// Save the data
		float *v = new float[count];
		int ptCount = count / 3;
		for (int i = 0; i < count; i++){
			v[i] = buffer[i];
		}
		segments.push_back(v);
		ptNum.push_back(ptCount);
	}
}