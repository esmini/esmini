#pragma once
#include "OSCParameterAssignment.hpp"

#include <iostream>
#include <string>
#include "CommonMini.hpp"


class OSCCatalogReference
{
public:
	std::string catalogName;
	std::string entryName;

	OSCParameterAssignment ParameterAssignment;

	void printOSCCatalogReference()
	{
		LOG(" - CatalogReference");
		LOG("\n");
	};
};