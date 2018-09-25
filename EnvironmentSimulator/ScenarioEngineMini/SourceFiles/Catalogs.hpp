#pragma once
#include <OSCDirectory.hpp>
#include <OSCRoute.hpp>

#include <iostream>
#include <string>
#include <vector>

class Catalogs
{
public:
	Catalogs();
	void printCatalogs();

	struct { OSCDirectory Directory; }VehicleCatalog;
	struct { OSCDirectory Directory; }DriverCatalog;
	struct 
	{ 
		std::vector<OSCRoute> Route; 
	} RouteCatalog;


};

