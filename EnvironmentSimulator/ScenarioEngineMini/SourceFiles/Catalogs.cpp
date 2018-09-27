#pragma once
#include <Catalogs.hpp>

#include <iostream>
#include <string>
#include <vector>


Catalogs::Catalogs()
{
	std::cout << "RoadNetwork: New RoadNetwork created" << std::endl;
}

void Catalogs::printCatalogs()
{
	VehicleCatalog.Directory.printOSCDirectory();
	DriverCatalog.Directory.printOSCDirectory();
	for (auto &route : RouteCatalog.Route)
	{
		route.printOSCRoute();
	}
}
