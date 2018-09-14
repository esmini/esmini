#pragma once
#include <iostream>
#include <string>
#include <math.h>

struct OSCAxle
{
	double maxSteering = NAN;
	double wheelDiameter = NAN;
	double trackWidth = NAN;
	double positionX = NAN;
	double positionZ = NAN;

	void printOSCAxle() 
	{
		std::cout << "\t" << "maxSteering = " << maxSteering << std::endl;
		std::cout << "\t" << "wheelDiameter = " << wheelDiameter << std::endl;
		std::cout << "\t" << "trackWidth = " << trackWidth << std::endl;
		std::cout << "\t" << "positionX = " << positionX << std::endl;
		std::cout << "\t" << "positionZ = " << positionZ << std::endl;
		std::cout << std::endl;

	}
};