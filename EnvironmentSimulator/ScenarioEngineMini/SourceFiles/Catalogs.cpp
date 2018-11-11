#pragma once
#include <Catalogs.hpp>
#include "CommonMini.hpp"

#include <iostream>
#include <string>
#include <vector>


Catalogs::Catalogs()
{
}

void Catalogs::printCatalogs()
{
	VehicleCatalog.Directory.printOSCDirectory();
	DriverCatalog.Directory.printOSCDirectory();
	for (size_t i=0; i<RouteCatalog.Route.size(); i++)
	{
		RouteCatalog.Route[i].printOSCRoute();
	}
}
