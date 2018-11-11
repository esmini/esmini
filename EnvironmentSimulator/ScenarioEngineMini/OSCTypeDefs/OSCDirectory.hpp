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
		LOG(" - OSCDirectory");
		LOG("\tname = %.2f", path);
		LOG("\n");
	}
};
