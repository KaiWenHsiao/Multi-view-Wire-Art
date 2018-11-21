#pragma once

#include <iostream>

class ProgressTestSender 
{
	


public:
	ProgressTestSender();
	virtual ~ProgressTestSender();

public:
	void updateProgressText(const std::string &str);
	void addLog(const std::string &str);

	static ProgressTestSender *Instance();
};

