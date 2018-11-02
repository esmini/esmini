#pragma once
#include <iostream>
#include <string>

class OSCFile
{
public:
	std::string filepath;

	void printOSCFile()
	{
		std::cout << " - OSCFile" << std::endl;
		std::cout << "\t" << "filepath = " << filepath << std::endl;
		std::cout << "\n" << std::endl;
	};
};


