#pragma once
#include <iostream>
#include <string>

#include "OSCPersonDescription.hpp"
#include "OSCParameterDeclaration.hpp"


class OSCDriver
{
public:
	OSCParameterDeclaration ParameterDeclaration;
	OSCPersonDescription Description;

	std::string name;

	void printOSCDriver()
	{
		LOG(" - Driver");
		LOG("\tname = %s", name.c_str());

		LOG(" - Driver - ParameterDeclaration ");
		ParameterDeclaration.printOSCParameterDeclaration();

		LOG(" - Driver - Description ");
		Description.printOSCPersonDescription();
	}
};
