#pragma once
#include "OSCProperties.hpp"
#include "OSCProperties.hpp"

#include <iostream>
#include <string>
#include <math.h>

struct OSCPersonDescription
{
	OSCProperties Properties;

	double weight = NAN;
	double height = NAN;
	double eyeDistance = NAN;
	double age = NAN;
	std::string sex = ""; //Wrong type

	void printOSCPersonDescription()
	{
		std::cout << "\t" << "weight = " << weight << std::endl;
		std::cout << "\t" << "height = " << height << std::endl;
		std::cout << "\t" << "eyeDistance = " << eyeDistance << std::endl;
		std::cout << "\t" << "age = " << age << std::endl;
		std::cout << "\t" << "sex = " << sex << std::endl;
		std::cout << "\n" << std::endl;

		std::cout << "Controller - Driver - PersonDescription -  Properties" << std::endl;
		Properties.printOSCProperties();
		std::cout << "\n" << std::endl;

	}
};
