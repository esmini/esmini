#pragma once
#include "OSCProperties.hpp"
#include "OSCProperties.hpp"
#include "CommonMini.hpp"

#include <iostream>
#include <string>
#include <math.h>

class OSCPersonDescription
{
public:
	OSCProperties Properties;

	double weight;
	double height;
	double eyeDistance;
	double age;
	std::string sex; //Wrong type

	void printOSCPersonDescription()
	{
		LOG("\tweight = %.2f", weight);
		LOG("\theight = %.2f", height);
		LOG("\teyeDistance = %.2f", eyeDistance);
		LOG("\tage = %d", age);
		LOG("\tsex = %s", sex.c_str());

		LOG("Controller - Driver - PersonDescription -  Properties");
		Properties.printOSCProperties();

	}
};
