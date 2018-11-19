#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <OSCDirectory.hpp>
#include "roadmanager.hpp"


class Catalogs
{
public:
	Catalogs();
	void Print();

	struct { OSCDirectory Directory; }VehicleCatalog;
	struct { OSCDirectory Directory; }DriverCatalog;
	std::vector<roadmanager::Route*> RouteCatalog; 

};

