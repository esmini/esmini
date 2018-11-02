#pragma once
#include <iostream>
#include <string>

#include "OSCPersonDescription.hpp"
#include "OSCParameterDeclaration.hpp"


class OSCDirectory
{
public:
	std::string path;

	void printOSCDirectory()
	{
		std::cout << " - OSCDirectory" << std::endl;
		std::cout << "\t" << "name = " << path << std::endl;
		std::cout << "\n" << std::endl;
	}
};
