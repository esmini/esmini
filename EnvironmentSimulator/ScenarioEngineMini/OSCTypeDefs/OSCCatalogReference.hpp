#pragma once
#include "OSCParameterAssignment.hpp"

#include <iostream>
#include <string>

struct OSCCatalogReference
{
	std::string catalogName;
	std::string entryName;

	OSCParameterAssignment ParameterAssignment;

	void printOSCCatalogReference() 
	{
		std::cout << " - CatalogReference" << std::endl;
		std::cout << std::endl;
	};
};