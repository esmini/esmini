#pragma once
#include <iostream>
#include <string>
#include "CommonMini.hpp"

class OSCFile
{
public:
	std::string filepath;

	void printOSCFile()
	{
		LOG(" - OSCFile");
		LOG("\tfilepath = %s", filepath.c_str());
		LOG("\n");
	};
};


