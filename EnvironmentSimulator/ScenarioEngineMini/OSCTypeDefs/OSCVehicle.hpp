#pragma once
#include "OSCParameterDeclaration.hpp"
#include "OSCBoundingBox.hpp"
#include "OSCAxle.hpp"
#include "OSCProperties.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>


struct OSCVehicle
{
	OSCParameterDeclaration ParameterDeclaration;
	OSCBoundingBox BoundingBox;

	struct 
	{
		double maxSpeed = NAN;
		double maxDeceleration = NAN;
		double mass = NAN;
	} Performance;

	struct
	{
		OSCAxle Front;
		OSCAxle Rear;
	} Axles;

	OSCProperties Properties;

	std::string name = "";
	std::string category = ""; // Wrong type

	void printOSCVehicle() 
	{
		std::cout << "\t" << "name = " << name << std::endl;
		std::cout << "\t" << "category = " << category << std::endl;
		std::cout << std::endl;

		std::cout << " - ParameterDeclaration" << std::endl;
		ParameterDeclaration.printOSCParameterDeclaration();

		std::cout << " - BoundingBox" << "\n" << std::endl;
		BoundingBox.printOSCBoundingBox();

		std::cout << " - Performance" << std::endl;
		std::cout << "\t" << "maxSpeed = " << Performance.maxSpeed << std::endl;
		std::cout << "\t" << "maxDeceleration = " << Performance.maxDeceleration << std::endl;
		std::cout << "\t" << "mass = " << Performance.mass << std::endl;
		std::cout << std::endl;

		std::cout << " - Axles - Front" << std::endl;
		Axles.Front.printOSCAxle();

		std::cout << " - Axles - Rear" << std::endl;
		Axles.Rear.printOSCAxle();

		std::cout << " - Properties" << std::endl;
		Properties.printOSCProperties();
	};

};
