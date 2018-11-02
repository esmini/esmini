#pragma once
#include "OSCParameterAssignment.hpp"

#include <iostream>
#include <string>

class OSCCatalogReference
{
public:
	std::string catalogName;
	std::string entryName;

	OSCParameterAssignment ParameterAssignment;

	void printOSCCatalogReference()
	{
		std::cout << " - CatalogReference" << std::endl;
		std::cout << std::endl;
	};
};