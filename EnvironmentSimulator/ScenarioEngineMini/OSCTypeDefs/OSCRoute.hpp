#pragma once
#include <OSCParameterDeclaration.hpp>

#include <iostream>
#include <string>

// Forward declaration to overcome the circular dependency
struct OSCPosition;

struct OSCRoute
{
	OSCParameterDeclaration ParameterDeclaration;

	struct WaypointStruct
	{
		OSCPosition *Position;	// Note that this is a pointer which is neccessary for the forward declaration.
		std::string strategy;
	};

	std::vector<WaypointStruct> Waypoint;

	std::string name;
	std::string closed; // Wrong type

	void printOSCRoute()
	{
		std::cout << "OSCRoute: printOSCRoute not defined" << std::endl;

	};
};