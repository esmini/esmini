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

CatalogType Entry::GetTypeByNodeName(pugi::xml_node node)
{
    if (!strcmp(node.name(), "Route"))
    {
        return CatalogType::CATALOG_ROUTE;
    }
    else if (!strcmp(node.name(), "Maneuver"))
    {
        return CatalogType::CATALOG_MANEUVER;
    }
    else if (!strcmp(node.name(), "Vehicle"))
    {
        return CatalogType::CATALOG_VEHICLE;
    }
    else if (!strcmp(node.name(), "Pedestrian"))
    {
        return CatalogType::CATALOG_PEDESTRIAN;
    }
    else if (!strcmp(node.name(), "MiscObject"))
    {
        return CatalogType::CATALOG_MISC_OBJECT;
    }
    else if (!strcmp(node.name(), "Controller"))
    {
        return CatalogType::CATALOG_CONTROLLER;
    }
    else if (!strcmp(node.name(), "Trajectory"))
    {
        return CatalogType::CATALOG_TRAJECTORY;
    }
    else
    {
        LOG("Unsupported catalog entry type: %s", node.name());
    }

    return CatalogType::CATALOG_UNDEFINED;
}

Entry::Entry(std::string name, pugi::xml_document root)
{
    name_ = name;
    root_ = std::move(root);
    type_ = GetTypeByNodeName(GetNode());
}

int Catalogs::RegisterCatalogDirectory(std::string type, std::string directory)
{
    CatalogDirEntry entry;
    entry.dir_name_ = directory;

    if (type == "VehicleCatalog")
    {
        entry.type_ = CatalogType::CATALOG_VEHICLE;
    }
    else if (type == "PedestrianCatalog")
    {
        entry.type_ = CatalogType::CATALOG_PEDESTRIAN;
    }
    else if (type == "MiscObjectCatalog")
    {
        entry.type_ = CatalogType::CATALOG_MISC_OBJECT;
    }
    else if (type == "ManeuverCatalog")
    {
        entry.type_ = CatalogType::CATALOG_MANEUVER;
    }
    else if (type == "RouteCatalog")
    {
        entry.type_ = CatalogType::CATALOG_ROUTE;
    }
    else if (type == "ControllerCatalog")
    {
        entry.type_ = CatalogType::CATALOG_CONTROLLER;
    }
    else if (type == "TrajectoryCatalog")
    {
        entry.type_ = CatalogType::CATALOG_TRAJECTORY;
    }
    else
    {
        entry.type_ = CatalogType::CATALOG_UNDEFINED;
    }

    catalog_dirs_.push_back(entry);

    return 0;
}

std::string Entry::GetTypeAsStr_(CatalogType type)
{
    if (type == CATALOG_VEHICLE)
        return "VEHICLE";
    else if (type == CATALOG_DRIVER)
        return "DRIVER";
    else if (type == CATALOG_PEDESTRIAN)
        return "PEDESTRIAN";
    else if (type == CATALOG_PEDESTRIAN_CONTROLLER)
        return "PEDESTRIAN_CONTROLLER";
    else if (type == CATALOG_MISC_OBJECT)
        return "MISC_OBJECT";
    else if (type == CATALOG_ENVIRONMENT)
        return "ENVIRONMENT";
    else if (type == CATALOG_MANEUVER)
        return "MANEUVER";
    else if (type == CATALOG_TRAJECTORY)
        return "TRAJECTORY";
    else if (type == CATALOG_ROUTE)
        return "ROUTE";
    else if (type == CATALOG_CONTROLLER)
        return "CONTROLLER";
    else if (type == CATALOG_UNDEFINED)
        return "UNDEFINED";
    else
        LOG("Type %d not recognized", type);

    return "";
}
