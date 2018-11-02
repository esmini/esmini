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
		std::cout << " - Driver" << std::endl;
		std::cout << "\t" << "name = " << name << std::endl;
		std::cout << "\n" << std::endl;

		std::cout << " - Driver - ParameterDeclaration " << std::endl;
		ParameterDeclaration.printOSCParameterDeclaration();
		std::cout << "\n" << std::endl;

		std::cout << " - Driver - Description " << std::endl;
		Description.printOSCPersonDescription();
		std::cout << "\n" << std::endl;
	}
};
