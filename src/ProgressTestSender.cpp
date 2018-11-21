#include "ProgressTestSender.h"


ProgressTestSender::ProgressTestSender()
{
}


ProgressTestSender::~ProgressTestSender()
{
}

ProgressTestSender *ProgressTestSender::Instance(){
	static ProgressTestSender *m_instance = nullptr;

	if (m_instance == nullptr){
		m_instance = new ProgressTestSender();
	}
	return m_instance;
}
void ProgressTestSender::updateProgressText(const std::string &str){
	std::cout << str.c_str() << "\n";
}
void ProgressTestSender::addLog(const std::string &str){
	std::cout << str.c_str() << "\n";
}
