#pragma once
#include <iostream>
#include <string>
#include "CommonMini.hpp"

class OSCFile
{
public:
	std::string filepath;

	void Print()
	{
		LOG("file: %s", filepath.c_str());
	};
};


