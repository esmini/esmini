#pragma once
#include "OSCOrientation.hpp"

#include <iostream>
#include <string>
#include <math.h>

struct OSCPosition
{
	bool exists = false;

	struct {
		bool exists = false;
		double x = NAN;
		double y = NAN;
		double z = NAN;
		double h = NAN;
		double p = NAN;
		double r = NAN;

	} World;

	struct {} RelativeWorld;
	struct {} RelativeObject;
	struct {} Road;
	struct {} RelativeRoad;

	struct {
		bool exists = false;
		std::string roadId = "";
		int laneId = NAN;
		double offset = NAN;
		double s = NAN;

		OSCOrientation Orientation;
	
	} Lane;

	struct {} RelativeLane;
	struct {} Route;

	void printOSCPosition()
	{
		std::cout << "\t" << " - Lane" << std::endl;
		std::cout << "\t" << "roadId = " << Lane.roadId << std::endl;
		std::cout << "\t" << "laneId = " << Lane.laneId << std::endl;
		std::cout << "\t" << "offset = " << Lane.offset << std::endl;
		std::cout << "\t" << "s = " << Lane.s << std::endl;
		std::cout << std::endl;

		std::cout << "\t" << " - Lane - Orientation" << std::endl;
		Lane.Orientation.printOSCOrientation();

	};
};