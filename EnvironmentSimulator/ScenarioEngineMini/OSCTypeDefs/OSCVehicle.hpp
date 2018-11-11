#pragma once
#include "OSCParameterDeclaration.hpp"
#include "OSCBoundingBox.hpp"
#include "OSCAxle.hpp"
#include "OSCProperties.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>


class OSCVehicle
{
public:
	OSCParameterDeclaration ParameterDeclaration;
	OSCBoundingBox BoundingBox;

	struct 
	{
		double maxSpeed;
		double maxDeceleration;
		double mass;
	} Performance;

	struct
	{
		OSCAxle Front;
		OSCAxle Rear;
	} Axles;

	OSCProperties Properties;

	std::string name;
	std::string category; // Wrong type

	void printOSCVehicle() 
	{
		LOG("\tname = %.2f", name);
		LOG("\tcategory = %.2f", category);
		LOG("\n");

		LOG(" - ParameterDeclaration");
		ParameterDeclaration.printOSCParameterDeclaration();

		LOG(" - BoundingBox");
		BoundingBox.printOSCBoundingBox();

		LOG(" - Performance");
		LOG("\tmaxSpeed = %.2f", Performance.maxSpeed);
		LOG("\tmaxDeceleration = %.2f", Performance.maxDeceleration);
		LOG("\tmass = %.2f", Performance.mass);
		LOG("\n");

		LOG(" - Axles - Front");
		Axles.Front.printOSCAxle();

		LOG(" - Axles - Rear");
		Axles.Rear.printOSCAxle();

		LOG(" - Properties");
		Properties.printOSCProperties();
	};

};
