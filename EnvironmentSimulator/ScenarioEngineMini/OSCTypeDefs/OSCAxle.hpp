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
		LOG("\t maxSteering: %.2f", maxSteering);
		LOG("\t wheelDiameter: %.2f",wheelDiameter);
		LOG("\t trackWidth: %.2f", trackWidth);
		LOG("\t %.2f", positionX);
		LOG("\t %.2f", positionZ);
		LOG("\n");

	}
};