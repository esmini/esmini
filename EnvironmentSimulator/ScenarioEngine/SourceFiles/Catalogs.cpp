/* 
 * esmini - Environment Simulator Minimalistic 
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "Catalogs.hpp"
#include "pugixml.hpp"

using namespace scenarioengine;



int Catalogs::RegisterCatalogDirectory(std::string type, std::string directory)
{
	CatalogDirEntry entry;
	entry.dir_name_ = directory;

	if (type == "VehicleCatalog")
	{
		entry.type_ = CatalogType::CATALOG_VEHICLE;
	}
	else if (type == "ManeuverCatalog")
	{
		entry.type_ = CatalogType::CATALOG_MANEUVER;
	}
	else if (type == "RouteCatalog")
	{
		entry.type_ = CatalogType::CATALOG_ROUTE;
	}
	else
	{
		LOG("Warning: %s not yet supported", type.c_str());
		return -1;
	}

	catalog_dirs_.push_back(entry);

	return 0;
}

std::string Entry::GetTypeAsStr_(CatalogType type)
{
	if (type == CATALOG_VEHICLE) return "VEHICLE";
		else if (type == CATALOG_DRIVER) return "DRIVER";
		else if (type == CATALOG_PEDESTRIAN) return "PEDESTRIAN";
		else if (type == CATALOG_PEDESTRIAN_CONTROLLER) return "PEDESTRIAN_CONTROLLER";
		else if (type == CATALOG_MISC_OBJECT) return "MISC_OBJECT";
		else if (type == CATALOG_ENVIRONMENT) return "ENVIRONMENT";
		else if (type == CATALOG_MANEUVER) return "MANEUVER";
		else if (type == CATALOG_TRAJECTORY) return "TRAJECTORY";
		else if (type == CATALOG_ROUTE) return "ROUTE";
		else if (type == CATALOG_UNDEFINED) return "UNDEFINED";
		else LOG("Type %d not recognized", type);

	return "";
}