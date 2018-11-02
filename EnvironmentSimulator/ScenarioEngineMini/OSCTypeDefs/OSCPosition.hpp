#pragma once
#include "OSCOrientation.hpp"
#include "OSCCatalogReference.hpp"
#include "OSCRoute.hpp"

#include <iostream>
#include <string>
#include <math.h>

#define INT_UNDEF 0xFFFFFFFF

// Forward declaration to overcome the circular dependency
class OSCRoute;

class OSCPosition
{
public:

	OSCPosition()
	{
		world_ = 0;
		lane_ = 0;
		route_ = 0;
	}

	typedef struct {
		bool exists;
		double x;
		double y;
		double z;
		double h;
		double p;
		double r;

	} World;

	struct {} RelativeWorld;
	struct {} RelativeObject;
	struct {} Road;
	struct {} RelativeRoad;

	typedef struct {
		bool exists;
		int roadId;
		int laneId;
		double offset;
		double s;

		OSCOrientation Orientation;
	
	} Lane;

	struct {} RelativeLane;

	typedef struct 
	{
		struct
		{
			OSCRoute Route;
			OSCCatalogReference CatalogReference;

		} RouteRef;

		struct
		{
			OSCOrientation Orientation;
		} Orientation;

		struct
		{
			struct
			{
				std::string object;
			} Current;

			struct
			{
				double pathS;
				double t;
			} RoadCoord;

			struct
			{
				double pathS;
				int laneId;
				double laneOffset;
			} LaneCoord;

		} Position;
	
	} Route;

	World *world_;
	Lane *lane_;
	Route *route_;

	void printOSCPosition()
	{
		//std::cout << "\t" << " - Lane" << std::endl;
		//std::cout << "\t" << "roadId = " << Lane.roadId << std::endl;
		//std::cout << "\t" << "laneId = " << Lane.laneId << std::endl;
		//std::cout << "\t" << "offset = " << Lane.offset << std::endl;
		//std::cout << "\t" << "s = " << Lane.s << std::endl;
		//std::cout << std::endl;

		//std::cout << "\t" << " - Lane - Orientation" << std::endl;
		//Lane.Orientation.printOSCOrientation();

	};
};