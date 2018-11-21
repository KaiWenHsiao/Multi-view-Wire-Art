#include "src\MyPipeline.h"

bool processScript(const std::string &SCRIPT_FILE, std::vector<std::string> &imgs, std::vector<std::string> &files, std::vector<std::string> &fileNames);

int main(){
	
	std::vector<std::string> imgs, files, fileNames;
	

	MyPipeline *pipeline = new MyPipeline();
	pipeline->m_requireImageGraph = false;
	
	int order = -1;
	std::cout << "Please input your requirement: ";
	std::cin >> order;

	switch (order){
	case 0:
		if (!processScript("script.txt", imgs, files, fileNames)){
			system("pause");
			break;
		};

		std::cout << "Voxel process\n";
		pipeline->sequentiallyMultiResolutionSculpture(imgs, fileNames, 1);

		pipeline->scriptForSkeletonExtraction(fileNames[0]);
		pipeline->scriptForCurveProcess(imgs, fileNames[0]);

		break;

	case 1:
		if (!processScript("Results\\script1.txt", imgs, files, fileNames)){
			system("pause");
			break;
		};

		std::cout << "curve process (simplify)\n";
		pipeline->sequentiallyProcessMultiSS(imgs, files, fileNames, 0);

		std::cout << "curve process (optimiz)\n";
		pipeline->m_requireImageGraph = true;
		files[0] = "Results\\Simplify\\" + fileNames[0] + "_FinalSimp.ss";
		pipeline->sequentiallyProcessMultiSS(imgs, files, fileNames, 1);

		std::cout << "Solidificate\n";
		files[0] = "Results\\Optimize\\" + fileNames[0] + "_optimize.ss";
		pipeline->solidficate(files[0], fileNames[0]);

		break;

	/*
	case 2:
		std::cout << "curve process (simplify)\n";
		pipeline->sequentiallyProcessMultiSS(imgs, files, fileNames, 0);
		break;
	case 3:
		std::cout << "curve process (optimize)\n";
		pipeline->m_requireImageGraph = true;
		pipeline->sequentiallyProcessMultiSS(imgs, files, fileNames, 1);
		break;
	case 4:
		std::cout << "Solidificate\n";
		pipeline->solidficate(files[0], fileNames[0]);
		break;	
	*/

	default:
		std::cout << "The specify order is not defined\n";
		break;

	}

	system("pause");

	return 0;
}
bool processScript(const std::string &SCRIPT_FILE, std::vector<std::string> &imgs, std::vector<std::string> &files, std::vector<std::string> &fileNames){

	std::ifstream input(SCRIPT_FILE);
	if (!input.is_open()){
		std::cout << "Processing script failed\n";
		input.close();
		return false;
	}
	// Prepare string buffer
	char strBuffer[500];

	// Get Images
	input.getline(strBuffer, 500);
	int numImage = std::atoi(strBuffer);
	imgs = std::vector<std::string>(numImage);
	for (int i = 0; i < numImage; i++){
		input.getline(strBuffer, 500);
		imgs[i] = std::string(strBuffer);
	}
	// Get SS
	input.getline(strBuffer, 500);
	int numSS = std::atoi(strBuffer);
	files = std::vector<std::string>(numSS);
	for (int i = 0; i < numSS; i++){
		input.getline(strBuffer, 500);
		files[i] = std::string(strBuffer);
	}
	// Get File Names
	input.getline(strBuffer, 500);
	int numFiles = std::atoi(strBuffer);
	fileNames = std::vector<std::string>(numFiles);
	for (int i = 0; i < numFiles; i++){
		input.getline(strBuffer, 500);
		fileNames[i] = std::string(strBuffer);
	}
	// Display the content
	std::cout << "Num Image: " + std::to_string(numImage) << "\n";
	for (int i = 0; i < numImage; i++){
		std::cout << imgs[i] << "\n";
	}
	std::cout << "Num SS: " + std::to_string(numSS) << "\n";
	for (int i = 0; i < numSS; i++){
		std::cout << files[i] << "\n";
	}
	std::cout << "Num File: " + std::to_string(numFiles) << "\n";
	for (int i = 0; i < numFiles; i++){
		std::cout << fileNames[i] << "\n";
	}

	input.close();

	return true;
}
