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
	void Print();

	struct { OSCDirectory Directory; }VehicleCatalog;
	struct { OSCDirectory Directory; }DriverCatalog;
	std::vector<roadmanager::Route*> RouteCatalog; 

};

