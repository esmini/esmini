#pragma once
#include <iostream>
#include <string>
#include <math.h>

class OSCAxle
{
public:

	double maxSteering;
	double wheelDiameter;
	double trackWidth;
	double positionX;
	double positionZ;

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