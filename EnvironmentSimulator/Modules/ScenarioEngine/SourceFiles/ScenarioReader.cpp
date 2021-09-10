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

#include "ScenarioReader.hpp"
#include "CommonMini.hpp"
#include "ControllerSloppyDriver.hpp"
#include "ControllerInteractive.hpp"
#include "ControllerFollowGhost.hpp"
#ifdef _USE_SUMO
#include "ControllerSumo.hpp"
#endif  // _USE_SUMO
#include "ControllerExternal.hpp"
#include "ControllerRel2Abs.hpp"
#include "ControllerACC.hpp"

#include <cstdlib>

using namespace scenarioengine;

namespace scenarioengine
{
	static ControllerPool controllerPoolStatic;
	ControllerPool ScenarioReader::controllerPool_ = controllerPoolStatic;
}

ScenarioReader::~ScenarioReader()
{
	for (size_t i = 0; i < controller_.size(); i++)
	{
		delete controller_[i];
	}
	controller_.clear();
}

void ScenarioReader::LoadControllers()
{
	// Register all internal controllers. The user may register custom ones as well before reading the scenario.
	RegisterController(ControllerSloppyDriver::GetTypeNameStatic(), InstantiateControllerSloppyDriver);
	RegisterController(ControllerInteractive::GetTypeNameStatic(), InstantiateControllerInteractive);
	RegisterController(ControllerFollowGhost::GetTypeNameStatic(), InstantiateControllerFollowGhost);
#ifdef _USE_SUMO
	RegisterController(ControllerSumo::GetTypeNameStatic(), InstantiateControllerSumo);
#endif
	RegisterController(ControllerExternal::GetTypeNameStatic(), InstantiateControllerExternal);
	RegisterController(ControllerRel2Abs::GetTypeNameStatic(), InstantiateControllerRel2Abs);
	RegisterController(ControllerACC::GetTypeNameStatic(), InstantiateControllerACC);
}

void ScenarioReader::UnloadControllers()
{
	ScenarioReader::controllerPool_.Clear();
}

int ScenarioReader::loadOSCFile(const char *path)
{
	pugi::xml_parse_result result = doc_.load_file(path);
	if (!result)
	{
		LOG("%s at offset (character position): %d", result.description(), result.offset);
		return -1;
	}

	osc_root_ = doc_.child("OpenSCENARIO");
	if (!osc_root_)
	{
		// Another try
		osc_root_ = doc_.child("OpenScenario");
	}

	if (!osc_root_)
	{
		throw std::runtime_error("Couldn't find OpenSCENARIO or OpenScenario element - check XML!");
	}

	oscFilename_ = path;

	return 0;
}

int ScenarioReader::loadOSCMem(const pugi::xml_document &xml_doc)
{
	LOG("Loading XML document from memory");

	doc_.reset(xml_doc);

	osc_root_ = doc_.child("OpenSCENARIO");
	if (!osc_root_)
	{
		// Another try
		osc_root_ = doc_.child("OpenScenario");
	}

	if (!osc_root_)
	{
		throw std::runtime_error("Couldn't find OpenSCENARIO or OpenScenario element - check XML!");
	}

	oscFilename_ = "inline";

	return 0;
}

int ScenarioReader::RegisterCatalogDirectory(pugi::xml_node catalogDirChild)
{
	if (strcmp(catalogDirChild.name(), "Directory"))
	{
		// Additional subdirectory level. Expect Directory next.
		if (catalogDirChild.child("Directory") == NULL)
		{
			LOG("Catalog %s sub element Directory not found - skipping", catalogDirChild.name());
			return -1;
		}

		catalogDirChild = catalogDirChild.child("Directory");
	}

	std::string dirname = parameters.ReadAttribute(catalogDirChild, "path", true);

	if (dirname == "")
	{
		LOG("Catalog %s missing filename - ignoring", catalogDirChild.name());
		return -1;
	}

	if (FileNameExtOf(dirname) == ".xosc")
	{
		// In case the reference is an actual catalog file, load it now
		// Separate directory and name
		Catalogs::CatalogDirEntry entry = {CatalogType::CATALOG_UNDEFINED, DirNameOf(dirname)};
		catalogs_->catalog_dirs_.push_back(entry);
		if (LoadCatalog(FileNameWithoutExtOf(dirname)) == 0)
		{
			LOG("Catalog %s loaded", dirname.c_str());
		}
	}
	else
	{
		catalogs_->RegisterCatalogDirectory(catalogDirChild.parent().name(), dirname);
	}

	return 0;
}

int ScenarioReader::parseOSCHeader()
{
	pugi::xml_node hdr_node = osc_root_.child("FileHeader");

	versionMajor_ = strtoi(hdr_node.attribute("revMajor").value());
	versionMinor_ = strtoi(hdr_node.attribute("revMinor").value());
	description_ = hdr_node.attribute("description").value();

	return 0;
}

Object *ScenarioReader::ResolveObjectReference(std::string name)
{
	Object *object = entities_->GetObjectByName(name);

	if (object == 0)
	{
		throw std::runtime_error(std::string("Failed to find object: ") + name);
	}

	return object;
}

Catalog *ScenarioReader::LoadCatalog(std::string name)
{
	Catalog *catalog;

	// Check if already loaded
	if ((catalog = catalogs_->FindCatalogByName(name)) != 0)
	{
		// Catalog already loaded
		return catalog;
	}

	// Not found, try to locate it in one the registered catalog directories
	pugi::xml_document catalog_doc;
	pugi::xml_parse_result result;
	std::vector<std::string> file_name_candidates;
	for (size_t i = 0; i < catalogs_->catalog_dirs_.size() && !result; i++)
	{
		file_name_candidates.clear();
		// absolute path or relative to current directory
		file_name_candidates.push_back(catalogs_->catalog_dirs_[i].dir_name_ + "/" + name + ".xosc");
		// Then assume relative path to scenario directory - which perhaps should be the expected location
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(oscFilename_), catalogs_->catalog_dirs_[i].dir_name_) + "/" + name + ".xosc");
		// Check registered paths
		for (size_t j = 0; j < SE_Env::Inst().GetPaths().size(); j++)
		{
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[j], catalogs_->catalog_dirs_[i].dir_name_ + "/" + name + ".xosc"));
		}
		for (size_t j = 0; j < file_name_candidates.size() && !result; j++)
		{
			if (FileExists(file_name_candidates[j].c_str()))
			{
				// Load it
				result = catalog_doc.load_file(file_name_candidates[j].c_str());
			}
		}
	}
	if (!result)
	{
		throw std::runtime_error(std::string("Couldn't locate catalog file: " + name + ". " + result.description()));
	}

	pugi::xml_node osc_node_ = catalog_doc.child("OpenSCENARIO");
	if (!osc_node_)
	{
		osc_node_ = catalog_doc.child("OpenScenario");
		if (!osc_node_)
		{
			throw std::runtime_error("Couldn't find Catalog OpenSCENARIO or OpenScenario element - check XML!");
		}
	}
	pugi::xml_node catalog_node = osc_node_.child("Catalog");

	catalog = new Catalog();
	catalog->name_ = name;

	for (pugi::xml_node entry_n = catalog_node.first_child(); entry_n; entry_n = entry_n.next_sibling())
	{
		std::string entry_name = parameters.ReadAttribute(entry_n, "name");

		// To copy a XML node it needs to be put into a XML doc
		pugi::xml_document *xml_doc = new pugi::xml_document;
		xml_doc->append_copy(entry_n);
		catalog->AddEntry(new Entry(entry_name, xml_doc->first_child()));
	}

	// Get type by inspecting first entry
	if (catalog->entry_.size() > 0)
	{
		catalog->type_ = catalog->entry_[0]->type_;
	}
	else
	{
		LOG("Warning: Catalog %s seems to be empty!", catalog->name_.c_str());
	}

	catalogs_->AddCatalog(catalog);

	return catalog;
}

void ScenarioReader::parseRoadNetwork(RoadNetwork &roadNetwork)
{
	pugi::xml_node roadNetworkNode = osc_root_.child("RoadNetwork");

	for (pugi::xml_node roadNetworkChild = roadNetworkNode.first_child(); roadNetworkChild; roadNetworkChild = roadNetworkChild.next_sibling())
	{
		std::string roadNetworkChildName(roadNetworkChild.name());

		if (roadNetworkChildName == "LogicFile")
		{
			parseOSCFile(roadNetwork.logicFile, roadNetworkChild);
		}
		else if (roadNetworkChildName == "SceneGraphFile")
		{
			parseOSCFile(roadNetwork.sceneGraphFile, roadNetworkChild);
		}
	}

	LOG("OpenDRIVE: %s", roadNetwork.logicFile.filepath.empty() ? "None" : roadNetwork.logicFile.filepath.c_str());
	LOG("Scenegraph: %s", roadNetwork.sceneGraphFile.filepath.empty() ? "None" : roadNetwork.sceneGraphFile.filepath.c_str());
}

void ScenarioReader::ParseOSCProperties(OSCProperties &properties, pugi::xml_node &xml_node)
{
	pugi::xml_node properties_node = xml_node.child("Properties");
	if (properties_node != NULL)
	{
		for (pugi::xml_node propertiesChild = properties_node.first_child(); propertiesChild; propertiesChild = propertiesChild.next_sibling())
		{
			if (!strcmp(propertiesChild.name(), "File"))
			{
				properties.file_.filepath_ = parameters.ReadAttribute(propertiesChild, "filepath");
			}
			else if (!strcmp(propertiesChild.name(), "Property"))
			{
				OSCProperties::Property property;
				property.name_ = parameters.ReadAttribute(propertiesChild, "name");
				property.value_ = parameters.ReadAttribute(propertiesChild, "value");
				properties.property_.push_back(property);
			}
			else
			{
				LOG("Unexpected property element: %s", propertiesChild.name());
			}
		}
	}
}

Vehicle *ScenarioReader::createRandomOSCVehicle(std::string name)
{
	Vehicle *vehicle = new Vehicle();

	vehicle->name_ = name;
	vehicle->category_ = Vehicle::Category::CAR;
	vehicle->model_id_ = -1;
	vehicle->model_filepath_ = "";

	// Set some default bounding box just to avoid division-by-zero-problems
	vehicle->boundingbox_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	vehicle->boundingbox_.dimensions_.length_ = 4.0f;
	vehicle->boundingbox_.dimensions_.width_ = 2.0f;
	vehicle->boundingbox_.dimensions_.height_ = 1.2f;

	return vehicle;
}

roadmanager::CoordinateSystem ScenarioReader::ParseCoordinateSystem(pugi::xml_node node, roadmanager::CoordinateSystem defaultValue)
{
	roadmanager::CoordinateSystem cs = defaultValue;

	std::string str = parameters.ReadAttribute(node, "coordinateSystem");
	if (!str.empty())
	{
		if (GetVersionMajor() == 1 && GetVersionMinor() == 0)
		{
			LOG("coordinateSystem introduced in v1.1. Reading it anyway.");
		}

		if (str == "entity")
		{
			cs = roadmanager::CoordinateSystem::CS_ENTITY;
		}
		else if (str == "lane")
		{
			cs = roadmanager::CoordinateSystem::CS_LANE;
		}
		else if (str == "road")
		{
			cs = roadmanager::CoordinateSystem::CS_ROAD;
		}
		else if (str == "trajectory")
		{
			cs = roadmanager::CoordinateSystem::CS_ROAD;
		}
		else
		{
			LOG_AND_QUIT("Unexcpected coordinateSytem: %s", str.c_str());
		}
	}

	return cs;
}

roadmanager::RelativeDistanceType ScenarioReader::ParseRelativeDistanceType(pugi::xml_node node, roadmanager::RelativeDistanceType defaultValue)
{
	roadmanager::RelativeDistanceType rdt = defaultValue;

	std::string str = parameters.ReadAttribute(node, "relativeDistanceType");
	if (!str.empty())
	{
		if (str == "lateral")
		{
			rdt = roadmanager::RelativeDistanceType::REL_DIST_LATERAL;
		}
		else if (str == "longitudinal")
		{
			rdt = roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL;
		}
		else if (str == "cartesianDistance")
		{
			if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
			{
				LOG("relativeDistanceType::cartesianDistance depricated in v1.1. Reading it anyway.");
			}

			rdt = roadmanager::RelativeDistanceType::REL_DIST_CARTESIAN;
		}
		else if (str == "euclidianDistance")
		{
			if (GetVersionMajor() == 1 && GetVersionMinor() == 0)
			{
				LOG("relativeDistanceType::euclidianDistance introduced in v1.1. Reading it anyway.");
			}

			rdt = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
		}
		else
		{
			LOG_AND_QUIT("Unexcpected relativeDistanceType: %s", str.c_str());
		}
	}

	return rdt;
}

void ScenarioReader::ParseOSCBoundingBox(OSCBoundingBox &boundingbox, pugi::xml_node &xml_node)
{
	pugi::xml_node boundingbox_node = xml_node.child("BoundingBox");
	if (boundingbox_node != NULL)
	{
		for (pugi::xml_node boundingboxChild = boundingbox_node.first_child(); boundingboxChild; boundingboxChild = boundingboxChild.next_sibling())
		{
			std::string boundingboxChildName(boundingboxChild.name());
			if (boundingboxChildName == "Center")
			{
				boundingbox.center_.x_ = std::stof(parameters.ReadAttribute(boundingboxChild, "x"));
				boundingbox.center_.y_ = std::stof(parameters.ReadAttribute(boundingboxChild, "y"));
				boundingbox.center_.z_ = std::stof(parameters.ReadAttribute(boundingboxChild, "z"));
			}
			else if (boundingboxChildName == "Dimensions")
			{
				boundingbox.dimensions_.width_ = std::stof(parameters.ReadAttribute(boundingboxChild, "width"));
				boundingbox.dimensions_.length_ = std::stof(parameters.ReadAttribute(boundingboxChild, "length"));
				boundingbox.dimensions_.height_ = std::stof(parameters.ReadAttribute(boundingboxChild, "height"));
			}
			else
			{
				LOG("Not valid boudingbox attribute name:%s", boundingboxChildName.c_str());
			}
		}
	}
	else
	{
		// Fill empy values to indicate missing bounding box
		boundingbox.center_.x_ = 0;
		boundingbox.center_.y_ = 0;
		boundingbox.center_.z_ = 0;
		boundingbox.dimensions_.length_ = 0;
		boundingbox.dimensions_.width_ = 0;
		boundingbox.dimensions_.height_ = 0;
	}
}

Vehicle *ScenarioReader::parseOSCVehicle(pugi::xml_node vehicleNode)
{
	Vehicle *vehicle = new Vehicle();

	if (vehicleNode == 0)
	{
		return 0;
	}

	// First check for parameter declaration
	pugi::xml_node paramDecl = vehicleNode.child("ParameterDeclarations");

	parameters.CreateRestorePoint();
	parameters.addParameterDeclarations(paramDecl);

	vehicle->name_ = parameters.ReadAttribute(vehicleNode, "name");
	vehicle->SetCategory(parameters.ReadAttribute(vehicleNode, "vehicleCategory"));

	// get File based on Category, and set default 3D model id
	if (vehicle->category_ == Vehicle::Category::BICYCLE ||
		vehicle->category_ == Vehicle::Category::MOTORBIKE)
	{
		vehicle->model_id_ = 9; // magic number for cyclist, set as default
		vehicle->model_filepath_ = "cyclist.osgb";
	}
	else
	{
		// magic numbers: If first vehicle make it white, else red
		vehicle->model_id_ = entities_->object_.size() == 0 ? 0 : 2;
		vehicle->model_filepath_ = entities_->object_.size() == 0 ? "car_white.osgb" : "car_red.osgb";
	}

	ParseOSCProperties(vehicle->properties_, vehicleNode);

	// Overwrite default values if properties set
	if (vehicle->properties_.file_.filepath_ != "")
	{
		vehicle->model_filepath_ = vehicle->properties_.file_.filepath_;
	}
	std::string modelIdStr = vehicle->properties_.GetValueStr("model_id");
	if (!modelIdStr.empty())
	{
		vehicle->model_id_ = strtoi(modelIdStr);
	}

	OSCBoundingBox boundingbox = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	ParseOSCBoundingBox(boundingbox, vehicleNode);
	vehicle->boundingbox_ = boundingbox;

	parameters.RestoreParameterDeclarations();

	return vehicle;
}

Pedestrian *ScenarioReader::parseOSCPedestrian(pugi::xml_node pedestrianNode)
{
	Pedestrian *pedestrian = new Pedestrian();

	if (pedestrianNode == 0)
	{
		return 0;
	}

	pedestrian->name_ = parameters.ReadAttribute(pedestrianNode, "name");
	pedestrian->SetCategory(parameters.ReadAttribute(pedestrianNode, "pedestrianCategory"));
	pedestrian->model_ = parameters.ReadAttribute(pedestrianNode, "pedestrianCategory");
	pedestrian->mass_ = strtod(parameters.ReadAttribute(pedestrianNode, "mass"));

	// Parse BoundingBox
	OSCBoundingBox boundingbox = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	ParseOSCBoundingBox(boundingbox, pedestrianNode);
	pedestrian->boundingbox_ = boundingbox;

	// Set default model_id, will be overwritten if that property is defined
	if (pedestrian->category_ == Pedestrian::Category::ANIMAL)
	{
		pedestrian->model_id_ = 8; // magic number for moose, set as default
		pedestrian->model_filepath_ = "moose_cc0.osgb";
	}
	else
	{
		pedestrian->model_id_ = 7; // magic number for pedestrian, set as default
		pedestrian->model_filepath_ = "walkman.osgb";
	}

	ParseOSCProperties(pedestrian->properties_, pedestrianNode);

	// Overwrite default values if properties set
	if (pedestrian->properties_.file_.filepath_ != "")
	{
		pedestrian->model_filepath_ = pedestrian->properties_.file_.filepath_;
	}
	std::string modelIdStr = pedestrian->properties_.GetValueStr("model_id");
	if (!modelIdStr.empty())
	{
		pedestrian->model_id_ = strtoi(modelIdStr);
	}

	return pedestrian;
}

MiscObject *ScenarioReader::parseOSCMiscObject(pugi::xml_node miscObjectNode)
{
	MiscObject *miscObject = new MiscObject();

	if (miscObjectNode == 0)
	{
		return 0;
	}

	miscObject->name_ = parameters.ReadAttribute(miscObjectNode, "name");
	miscObject->SetCategory(parameters.ReadAttribute(miscObjectNode, "MiscObjectCategory"));
	miscObject->model_ = parameters.ReadAttribute(miscObjectNode, "MiscObjectCategory");
	miscObject->mass_ = strtod(parameters.ReadAttribute(miscObjectNode, "mass"));

	// Parse BoundingBox
	OSCBoundingBox boundingbox = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	ParseOSCBoundingBox(boundingbox, miscObjectNode);
	miscObject->boundingbox_ = boundingbox;

	ParseOSCProperties(miscObject->properties_, miscObjectNode);

	// Overwrite default values if properties set
	if (miscObject->properties_.file_.filepath_ != "")
	{
		miscObject->model_filepath_ = miscObject->properties_.file_.filepath_;
	}
	std::string modelIdStr = miscObject->properties_.GetValueStr("model_id");
	if (!modelIdStr.empty())
	{
		miscObject->model_id_ = strtoi(modelIdStr);
	}

	return miscObject;
}

Controller *ScenarioReader::parseOSCObjectController(pugi::xml_node controllerNode)
{
	std::string name = parameters.ReadAttribute(controllerNode, "name");
	Controller *controller = 0;
	OSCProperties properties;

	// First check for parameter declaration
	pugi::xml_node paramDecl = controllerNode.child("ParameterDeclarations");

	parameters.CreateRestorePoint();
	parameters.addParameterDeclarations(paramDecl);

	// Then read any properties
	ParseOSCProperties(properties, controllerNode);
	std::string filename = properties.file_.filepath_;

	if (controllerNode == 0)
	{
		LOG("Warning: Empty controller node");
	}

	if (!properties.file_.filepath_.empty())
	{
		// Localize file
		if (!FileExists(filename.c_str()))
		{
			// Then assume relative path to scenario directory - which perhaps should be the expected location
			std::string filename2 = CombineDirectoryPathAndFilepath(DirNameOf(oscFilename_), filename);

			if (!FileExists(filename2.c_str()))
			{
				// Give up
				LOG("Failed to localize controller file %s, also tried %s", filename.c_str(), filename2.c_str());
			}
			else
			{
				// Update file path
				properties.file_.filepath_ = filename2;
			}
		}
	}

	// Find controller type among registered ones
	std::string ctrlType = properties.GetValueStr("esminiController");
	if (ctrlType.empty())
	{
		LOG("Missing esminiController property, using controller name: %s", name.c_str());
		ctrlType = name;
	}

	ControllerPool::ControllerEntry *ctrl_entry = ScenarioReader::controllerPool_.GetControllerByType(ctrlType);
	if (ctrl_entry)
	{
		Controller::InitArgs args;
		args.name = name;
		args.type = ctrlType;
		args.entities = entities_;
		args.gateway = gateway_;
		args.parameters = &parameters;
		args.properties = &properties;
		controller = (Controller *)ctrl_entry->instantiateFunction(&args);
	}
	else
	{
		LOG("Unsupported controller type: %s. Falling back to default controller", ctrlType.c_str());
		controller = 0;
	}

	parameters.RestoreParameterDeclarations();

	return controller;
}

roadmanager::Route *ScenarioReader::parseOSCRoute(pugi::xml_node routeNode)
{
	roadmanager::Route *route = new roadmanager::Route;

	route->setName(parameters.ReadAttribute(routeNode, "name"));

	parameters.CreateRestorePoint();

	// Closed attribute not supported by roadmanager yet
	std::string closed_str = parameters.ReadAttribute(routeNode, "closed");
	bool closed = false;
	(void)closed;
	if (closed_str == "true" || closed_str == "1")
	{
		closed = true;
	}

	for (pugi::xml_node routeChild = routeNode.first_child(); routeChild; routeChild = routeChild.next_sibling())
	{
		std::string routeChildName(routeChild.name());

		if (routeChildName == "ParameterDeclaration")
		{
			LOG("%s is not implemented", routeChildName.c_str());
		}
		else if (routeChildName == "Waypoint")
		{
			OSCPosition *pos = parseOSCPosition(routeChild.first_child());
			if (pos)
			{
				route->AddWaypoint(pos->GetRMPos());
			}
			else
			{
				LOG("Failed to parse waypoint position");
			}
		}
	}

	parameters.RestoreParameterDeclarations();

	return route;
}

roadmanager::RMTrajectory *ScenarioReader::parseTrajectoryRef(pugi::xml_node trajNode)
{
	roadmanager::RMTrajectory *traj = 0;

	pugi::xml_node refChild = trajNode.first_child();

	if (!strcmp(refChild.name(), "Trajectory"))
	{
		// Parse inline trajectory
		traj = parseTrajectory(refChild);
	}
	else if (!strcmp(refChild.name(), "CatalogReference"))
	{
		// Find route in catalog
		Entry *entry = ResolveCatalogReference(refChild);

		if (entry == 0)
		{
			throw std::runtime_error("Failed to resolve catalog reference");
		}

		if (entry->type_ == CatalogType::CATALOG_TRAJECTORY)
		{
			// Make a new instance from catalog entry
			traj = parseTrajectory(entry->GetNode());
		}
		else
		{
			LOG("Catalog entry of type %s expected - found %s", Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE).c_str(), entry->GetTypeAsStr().c_str());
			throw std::runtime_error("Failed to resolve catalog reference");
		}
	}
	else
	{
		LOG("Missing TrajectoryRef child element (Trajectory or CatalogReference expected)");
		throw std::runtime_error("Missing TrajectoryRef child element (Trajectory or CatalogReference expected)");
	}

	return traj;
}

void ScenarioReader::parseCatalogs()
{
	pugi::xml_node catalogsNode = osc_root_.child("CatalogLocations");

	for (pugi::xml_node catalogsChild = catalogsNode.first_child(); catalogsChild; catalogsChild = catalogsChild.next_sibling())
	{
		RegisterCatalogDirectory(catalogsChild);
	}
}

void ScenarioReader::parseOSCFile(OSCFile &file, pugi::xml_node fileNode)
{
	file.filepath = parameters.ReadAttribute(fileNode, "filepath");
}

roadmanager::RMTrajectory *ScenarioReader::parseTrajectory(pugi::xml_node node)
{
	roadmanager::RMTrajectory *traj = new roadmanager::RMTrajectory;
	roadmanager::Shape *shape = 0;

	parameters.CreateRestorePoint();

	traj->name_ = parameters.ReadAttribute(node, "name");
	traj->closed_ = parameters.ReadAttribute(node, "closed") == "true" ? true : false;

	for (pugi::xml_node childNode = node.first_child(); childNode; childNode = childNode.next_sibling())
	{
		std::string childNodeName(childNode.name());

		if (childNodeName == "ParameterDeclarations")
		{
			parameters.addParameterDeclarations(childNode);
		}
		else if (childNodeName == "Shape")
		{
			pugi::xml_node shapeNode = childNode.first_child();
			if (!shapeNode)
			{
				throw std::runtime_error("Missing Trajectory Shape");
			}

			std::string shapeType = shapeNode.name();
			if (shapeType == "Polyline")
			{
				roadmanager::PolyLineShape *pline = new roadmanager::PolyLineShape();
				for (pugi::xml_node vertexNode = shapeNode.first_child(); vertexNode; vertexNode = vertexNode.next_sibling())
				{
					pugi::xml_node posNode = vertexNode.child("Position");
					if (!posNode)
					{
						throw std::runtime_error("Missing Trajectory/Polyline/Vertex/Position node");
					}
					OSCPosition *pos = parseOSCPosition(posNode);
					double time = strtod(parameters.ReadAttribute(vertexNode, "time"));
					pline->AddVertex(*pos->GetRMPos(), time, posNode.first_child().child("Orientation") ? false : true);
				}
				shape = pline;
			}
			else if (shapeType == "Clothoid")
			{
				pugi::xml_node posNode = shapeNode.child("Position");
				OSCPosition *pos = parseOSCPosition(posNode);

				double curvature = strtod(parameters.ReadAttribute(shapeNode, "curvature"));
				double curvatureDot = strtod(parameters.ReadAttribute(shapeNode, "curvatureDot"));
				double length = strtod(parameters.ReadAttribute(shapeNode, "length"));
				double startTime = strtod(parameters.ReadAttribute(shapeNode, "startTime"));
				double stopTime = strtod(parameters.ReadAttribute(shapeNode, "stopTime"));

				LOG("Adding clothoid(x=%.2f y=%.2f h=%.2f curv=%.2f curvDot=%.2f len=%.2f startTime=%.2f stopTime=%.2f",
					pos->GetRMPos()->GetX(), pos->GetRMPos()->GetY(), pos->GetRMPos()->GetH(), curvature, curvatureDot, length, startTime, stopTime);

				roadmanager::ClothoidShape *clothoid = new roadmanager::ClothoidShape(*pos->GetRMPos(), curvature, curvatureDot, length, startTime, stopTime);

				shape = clothoid;
			}
			else if (shapeType == "Nurbs")
			{
				int order = strtoi(parameters.ReadAttribute(shapeNode, "order"));

				roadmanager::NurbsShape *nurbs = new roadmanager::NurbsShape(order);
				std::vector<double> knots;

				// Parse control points and knots
				for (pugi::xml_node nurbsChild = shapeNode.first_child(); nurbsChild; nurbsChild = nurbsChild.next_sibling())
				{
					std::string nurbsChildName(nurbsChild.name());

					if (nurbsChildName == "ControlPoint")
					{
						pugi::xml_node posNode = nurbsChild.child("Position");
						OSCPosition *pos = parseOSCPosition(posNode);
						double time = strtod(parameters.ReadAttribute(nurbsChild, "time"));
						double weight = 1.0;
						if (!nurbsChild.attribute("weight").empty())
						{
							weight = strtod(parameters.ReadAttribute(nurbsChild, "weight"));
						}
						bool calcHeading = posNode.first_child().child("Orientation") ? false : true;
						nurbs->AddControlPoint(*pos->GetRMPos(), time, weight, calcHeading);
					}
					else if (nurbsChildName == "Knot")
					{
						double value = strtod(parameters.ReadAttribute(nurbsChild, "value"));
						knots.push_back(value);
					}
					else
					{
						throw std::runtime_error(std::string("Unsupported Nurbs child element: ") + nurbsChildName);
					}
				}
				nurbs->AddKnots(knots);
				shape = nurbs;
			}
			else
			{
				throw std::runtime_error("Unexpected/unsupported Trajectory type: " + shapeType);
			}
			if (!shape)
			{
				throw std::runtime_error("Missing Trajectory/Shape element");
			}
			traj->shape_ = shape;
		}
	}

	parameters.RestoreParameterDeclarations();

	return traj;
}

Entry *ScenarioReader::ResolveCatalogReference(pugi::xml_node node)
{
	std::string catalog_name;
	std::string entry_name;

	pugi::xml_node parameterAssignmentsNode = node.child("ParameterAssignments");

	// Read any parameter assignments
	for (pugi::xml_node param_n = parameterAssignmentsNode.child("ParameterAssignment"); param_n; param_n = param_n.next_sibling("ParameterAssignment"))
	{
		OSCParameterDeclarations::ParameterStruct param;
		if (param_n.attribute("parameterRef").value()[0] == PARAMETER_PREFIX)
		{
			param.name = &(param_n.attribute("parameterRef").value()[1]); // Skip prefix character byte
		}
		else
		{
			param.name = param_n.attribute("parameterRef").value();
		}
		param.value._string = parameters.ReadAttribute(param_n, "value");
		parameters.catalog_param_assignments.push_back(param);
	}

	catalog_name = parameters.ReadAttribute(node, "catalogName");
	entry_name = parameters.ReadAttribute(node, "entryName");

	Catalog *catalog;

	// Make sure the catalog item is loaded
	if ((catalog = LoadCatalog(catalog_name)) == 0)
	{
		LOG("Failed to load catalog %s", catalog_name.c_str());
		return 0;
	}

	Entry *entry = catalog->FindEntryByName(entry_name);
	if (entry == 0)
	{
		LOG("Failed to look up entry %s in catalog %s", entry_name.c_str(), catalog_name.c_str());

		return 0;
	}

	return entry;
}

int ScenarioReader::parseEntities()
{
	pugi::xml_node enitiesNode = osc_root_.child("Entities");

	for (pugi::xml_node entitiesChild = enitiesNode.first_child(); entitiesChild; entitiesChild = entitiesChild.next_sibling())
	{
		std::string entitiesChildName(entitiesChild.name());
		if (entitiesChildName == "ScenarioObject")
		{
			Object *obj = 0;
			Controller *ctrl = 0;

			for (pugi::xml_node objectChild = entitiesChild.first_child(); objectChild; objectChild = objectChild.next_sibling())
			{
				std::string objectChildName(objectChild.name());

				if (objectChildName == "CatalogReference")
				{
					parameters.CreateRestorePoint();
					Entry *entry = ResolveCatalogReference(objectChild);

					if (entry == 0)
					{
						// Invalid catalog reference - create random vehicle as fall-back
						LOG("Could not find catalog vehicle, creating a random car as fall-back");
						std::string entry_name = parameters.ReadAttribute(objectChild, "entryName");
						Vehicle *vehicle = createRandomOSCVehicle(entry_name);
						obj = vehicle;
					}
					else
					{
						if (entry->type_ == CatalogType::CATALOG_VEHICLE)
						{
							// Make a new instance from catalog entry
							Vehicle *vehicle = parseOSCVehicle(entry->GetNode());
							obj = vehicle;
						}
						else if (entry->type_ == CatalogType::CATALOG_PEDESTRIAN)
						{
							// Make a new instance from catalog entry
							Pedestrian *pedestrian = parseOSCPedestrian(entry->GetNode());
							obj = pedestrian;
						}
						else if (entry->type_ == CatalogType::CATALOG_MISC_OBJECT)
						{
							// Make a new instance from catalog entry
							MiscObject* miscobj = parseOSCMiscObject(entry->GetNode());
							obj = miscobj;
						}
						else
						{
							LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
						}
					}

					parameters.RestoreParameterDeclarations();
				}
				else if (objectChildName == "Vehicle")
				{
					Vehicle *vehicle = parseOSCVehicle(objectChild);
					obj = vehicle;
				}
				else if (objectChildName == "Pedestrian")
				{
					Pedestrian *pedestrian = parseOSCPedestrian(objectChild);
					obj = pedestrian;
				}
				else if (objectChildName == "MiscObject")
				{
					MiscObject *miscObject = parseOSCMiscObject(objectChild);
					obj = miscObject;
				}
				else if (objectChildName == "ObjectController")
				{
					// get the sub child under ObjectController (should only be one)
					pugi::xml_node objectSubChild = objectChild.first_child();
					std::string objectSubChildName(objectSubChild.name());
					if (objectSubChildName == "CatalogReference")
					{
						Entry *entry = ResolveCatalogReference(objectSubChild);

						if (entry == 0)
						{
							LOG("No entry found");
						}
						else
						{
							if (entry->type_ == CatalogType::CATALOG_CONTROLLER)
							{
								ctrl = parseOSCObjectController(entry->GetNode());
							}
							else
							{
								LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
							}
						}
					}
					else
					{
						ctrl = parseOSCObjectController(objectSubChild);
					}
					if (ctrl)
					{
						// ObjectControllers are assigned automatically
						ctrl->Assign(obj);
					}
				}
				else
				{
					LOG("Unexpected scenario object element: %s", objectChildName.c_str());
				}
			}

			if (obj != 0 && !(ctrl && ctrl->GetType() == Controller::Type::CONTROLLER_TYPE_SUMO))
			{
				// Add all vehicles to the entity collection, except SUMO template vehicles
				obj->name_ = parameters.ReadAttribute(entitiesChild, "name");
				entities_->addObject(obj);
				objectCnt_++;
			}

			if (ctrl)
			{
#ifdef _USE_SUMO
				if (ctrl->GetType() == Controller::Type::CONTROLLER_TYPE_SUMO)
				{
					// Set template vehicle to be used for all vehicles spawned from SUMO
					((ControllerSumo *)ctrl)->SetSumoVehicle(obj);
					obj->id_ = -1;

					// SUMO controller is special in the sense that it is always active
					ctrl->Activate(ControlDomains::DOMAIN_BOTH);

					// SUMO controller is not assigned to any scenario vehicle
				}
#endif // USE_SUMO
				controller_.push_back(ctrl);
			}
		}
		else if (entitiesChildName == "EntitySelection")
		{
			LOG("INFO: %s is not implemented yet", entitiesChildName.c_str());
		}
		else
		{
			// DO NOTHING
		}
	}
	return 0;
}

void ScenarioReader::parseOSCOrientation(OSCOrientation &orientation, pugi::xml_node orientationNode)
{
	orientation.h_ = strtod(parameters.ReadAttribute(orientationNode, "h"));
	orientation.p_ = strtod(parameters.ReadAttribute(orientationNode, "p"));
	orientation.r_ = strtod(parameters.ReadAttribute(orientationNode, "r"));

	std::string type_str = parameters.ReadAttribute(orientationNode, "type");

	if (type_str == "relative")
	{
		orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
	}
	else if (type_str == "absolute")
	{
		orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE;
	}
	else if (type_str == "")
	{
		LOG("No orientation type specified - using absolute");
		orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE;
	}
	else
	{
		LOG("Invalid orientation type: %d - using absolute", type_str.c_str());
		orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE;
	}
}

OSCPosition *ScenarioReader::parseOSCPosition(pugi::xml_node positionNode)
{
	OSCPosition *pos_return = 0;

	pugi::xml_node positionChild = positionNode.first_child();

	std::string positionChildName(positionChild.name());

	if (positionChildName == "WorldPosition")
	{
		double x = std::nan("");
		double y = std::nan("");
		double z = std::nan("");
		double h = std::nan("");
		double p = std::nan("");
		double r = std::nan("");

		if (positionChild.attribute("x"))
		{
			x = strtod(parameters.ReadAttribute(positionChild, "x", true));
		}
		if (positionChild.attribute("y"))
		{
			y = strtod(parameters.ReadAttribute(positionChild, "y", true));
		}
		if (!positionChild.attribute("z").empty())
		{
			z = strtod(parameters.ReadAttribute(positionChild, "z", true));
		}
		if (!positionChild.attribute("h").empty())
		{
			h = strtod(parameters.ReadAttribute(positionChild, "h", true));
		}
		if (!positionChild.attribute("p").empty())
		{
			p = strtod(parameters.ReadAttribute(positionChild, "p", true));
		}
		if (!positionChild.attribute("r").empty())
		{
			r = strtod(parameters.ReadAttribute(positionChild, "r", true));
		}

		if (std::isnan(x) || std::isnan(y))
		{
			LOG_AND_QUIT("Missing x or y attributes!\n");
		}

		OSCPositionWorld *pos = new OSCPositionWorld(x, y, z, h, p, r);

		pos_return = (OSCPosition *)pos;
	}
	else if (positionChildName == "RelativeWorldPosition")
	{
		double dx, dy, dz;

		dx = strtod(parameters.ReadAttribute(positionChild, "dx"));
		dy = strtod(parameters.ReadAttribute(positionChild, "dy"));
		dz = strtod(parameters.ReadAttribute(positionChild, "dz"));

		Object *object = ResolveObjectReference(parameters.ReadAttribute(positionChild, "entityRef"));

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}

		OSCPositionRelativeWorld *pos = new OSCPositionRelativeWorld(object, dx, dy, dz, orientation);

		pos_return = (OSCPosition *)pos;
	}
	else if (positionChildName == "RelativeObjectPosition")
	{
		double dx, dy, dz;

		dx = strtod(parameters.ReadAttribute(positionChild, "dx"));
		dy = strtod(parameters.ReadAttribute(positionChild, "dy"));
		dz = strtod(parameters.ReadAttribute(positionChild, "dz"));
		Object *object = ResolveObjectReference(parameters.ReadAttribute(positionChild, "entityRef"));

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}

		OSCPositionRelativeObject *pos = new OSCPositionRelativeObject(object, dx, dy, dz, orientation);

		pos_return = (OSCPosition *)pos;
	}
	else if (positionChildName == "RelativeLanePosition")
	{
		int dLane;
		double ds, offset;

		dLane = strtoi(parameters.ReadAttribute(positionChild, "dLane"));
		ds = strtod(parameters.ReadAttribute(positionChild, "ds"));
		offset = strtod(parameters.ReadAttribute(positionChild, "offset"));
		Object *object = ResolveObjectReference(parameters.ReadAttribute(positionChild, "entityRef"));

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}
		else
		{
			// If no orientation specified, assume Relative is preferred
			orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
		}

		OSCPositionRelativeLane *pos = new OSCPositionRelativeLane(object, dLane, ds, offset, orientation);

		pos_return = (OSCPosition *)pos;
	}
	else if (positionChildName == "RelativeRoadPosition")
	{
		double ds, dt;

		ds = strtod(parameters.ReadAttribute(positionChild, "ds"));
		dt = strtod(parameters.ReadAttribute(positionChild, "dt"));
		Object *object = ResolveObjectReference(parameters.ReadAttribute(positionChild, "entityRef"));

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}
		else
		{
			// If no orientation specified, assume Relative is preferred
			orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
		}

		OSCPositionRelativeRoad *pos = new OSCPositionRelativeRoad(object, ds, dt, orientation);

		pos_return = (OSCPosition *)pos;
	}
	else if (positionChildName == "RoadPosition")
	{
		int road_id = strtoi(parameters.ReadAttribute(positionChild, "roadId"));
		double s = strtod(parameters.ReadAttribute(positionChild, "s"));
		double t = strtod(parameters.ReadAttribute(positionChild, "t"));
		;

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}
		else
		{
			// If no orientation specified, assume Relative is preferred
			orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
		}

		OSCPositionRoad *pos = new OSCPositionRoad(road_id, s, t, orientation);

		pos_return = (OSCPosition *)pos;
	}
	else if (positionChildName == "LanePosition")
	{
		int road_id = strtoi(parameters.ReadAttribute(positionChild, "roadId"));
		int lane_id = strtoi(parameters.ReadAttribute(positionChild, "laneId"));
		double s = strtod(parameters.ReadAttribute(positionChild, "s"));

		double offset = 0; // Default value of optional parameter
		if (positionChild.attribute("offset"))
		{
			offset = strtod(parameters.ReadAttribute(positionChild, "offset"));
		}

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}
		else
		{
			// If no orientation specified, assume Relative is preferred
			orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
		}

		OSCPositionLane *pos = new OSCPositionLane(road_id, lane_id, s, offset, orientation);

		pos_return = (OSCPosition *)pos;
	}
	else if (positionChildName == "RoutePosition")
	{
		roadmanager::Route *route = 0;
		OSCPositionRoute *pos = new OSCPositionRoute();
		OSCOrientation *orientation = 0;

		for (pugi::xml_node routeChild = positionChild.first_child(); routeChild; routeChild = routeChild.next_sibling())
		{
			if (routeChild.name() == std::string("RouteRef"))
			{
				for (pugi::xml_node routeRefChild = routeChild.first_child(); routeRefChild; routeRefChild = routeRefChild.next_sibling())
				{
					std::string routeRefChildName(routeRefChild.name());

					if (routeRefChildName == "Route")
					{
						// Parse inline route
						route = parseOSCRoute(routeRefChild);
					}
					else if (routeRefChildName == "CatalogReference")
					{
						// Find route in catalog
						parameters.CreateRestorePoint();
						Entry *entry = ResolveCatalogReference(routeRefChild);

						if (entry == 0)
						{
							throw std::runtime_error("Failed to resolve catalog reference");
						}

						if (entry->type_ == CatalogType::CATALOG_ROUTE)
						{
							// Make a new instance from catalog entry
							route = parseOSCRoute(entry->GetNode());
						}
						else
						{
							throw std::runtime_error(std::string("Found catalog entry ") + entry->name_ + ". But wrong type: " + entry->GetTypeAsStr() + ". Expected: " +
													 Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE) + ".");
						}

						parameters.RestoreParameterDeclarations();
					}
				}
			}
			else if (routeChild.name() == std::string("Orientation"))
			{
				orientation = new OSCOrientation;
				parseOSCOrientation(*orientation, routeChild);
			}
			else if (routeChild.name() == std::string("InRoutePosition"))
			{
				for (pugi::xml_node rPositionChild = routeChild.first_child(); rPositionChild; rPositionChild = rPositionChild.next_sibling())
				{
					std::string rPositionChildName(rPositionChild.name());

					if (rPositionChildName == "FromCurrentEntity")
					{
						LOG("%s is not implemented", rPositionChildName.c_str());
					}
					else if (rPositionChildName == "FromRoadCoordinates")
					{
						LOG("%s is not implemented", rPositionChildName.c_str());
					}
					else if (rPositionChildName == "FromLaneCoordinates")
					{
						double s = strtod(parameters.ReadAttribute(rPositionChild, "pathS"));
						int lane_id = strtoi(parameters.ReadAttribute(rPositionChild, "laneId"));
						double lane_offset = 0;

						pugi::xml_attribute laneOffsetAttribute = rPositionChild.attribute("laneOffset");
						if (laneOffsetAttribute != NULL)
						{
							lane_offset = strtod(parameters.ReadAttribute(rPositionChild, "laneOffset"));
						}
						if (orientation)
						{
							pos->SetRouteRefLaneCoord(route, s, lane_id, lane_offset, orientation);
						}
						else
						{
							pos->SetRouteRefLaneCoord(route, s, lane_id, lane_offset);
							if (lane_id > 0)
							{
								pos->SetRouteRelativeHeading(M_PI);
							}
						}
					}
				}
			}
		}

		pos_return = (OSCPosition *)pos;
	}
	else if (positionChildName == "TrajectoryPosition")
	{
		pugi::xml_node trajectoryRef = positionChild.child("TrajectoryRef");

		if (trajectoryRef.empty())
		{
			LOG("Missing expected TrajectoryRef element");
			throw std::runtime_error("Missing expected TrajectoryRef element");
		}

		roadmanager::RMTrajectory *traj = parseTrajectoryRef(trajectoryRef);

		double s = strtod(parameters.ReadAttribute(positionChild, "s"));
		if (!positionChild.attribute("t").empty())
		{
			LOG("TrajectoryPosition -> t not supported yet, set to zero");
		}
		double t = 0;

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}

		OSCPositionTrajectory *pos = new OSCPositionTrajectory(traj, s, t, orientation);

		pos_return = (OSCPosition *)pos;
	}

	if (pos_return == 0)
	{
		throw std::runtime_error(std::string("Failed parse position in node ") + positionNode.name());
	}

	return pos_return;
}

OSCPrivateAction::DynamicsShape ParseDynamicsShape(std::string shape)
{
	if (shape == "linear")
	{
		return OSCPrivateAction::DynamicsShape::LINEAR;
	}
	else if (shape == "sinusoidal")
	{
		return OSCPrivateAction::DynamicsShape::SINUSOIDAL;
	}
	else if (shape == "step")
	{
		return OSCPrivateAction::DynamicsShape::STEP;
	}
	else if (shape == "cubic")
	{
		return OSCPrivateAction::DynamicsShape::CUBIC;
	}
	else
	{
		std::string msg = "Dynamics shape " + shape + " not supported yet";
		throw std::runtime_error(msg);
	}

	return OSCPrivateAction::DynamicsShape::SHAPE_UNDEFINED;
}

OSCPrivateAction::DynamicsDimension ParseDynamicsDimension(std::string dimension)
{
	if (dimension.empty())
	{
		LOG("Dynamics dimension missing - fall back to TIME");
		return OSCPrivateAction::DynamicsDimension::TIME;
	}
	else if (dimension == "rate")
	{
		return OSCPrivateAction::DynamicsDimension::RATE;
	}
	else if (dimension == "time")
	{
		return OSCPrivateAction::DynamicsDimension::TIME;
	}
	else if (dimension == "distance")
	{
		return OSCPrivateAction::DynamicsDimension::DISTANCE;
	}
	else
	{
		LOG("Dynamics dimension %s not supported - fall back to TIME", dimension.c_str());
		return OSCPrivateAction::DynamicsDimension::TIME;
	}
}

int ScenarioReader::ParseTransitionDynamics(pugi::xml_node node, OSCPrivateAction::TransitionDynamics &td)
{
	td.shape_ = ParseDynamicsShape(parameters.ReadAttribute(node, "dynamicsShape", true));
	td.dimension_ = ParseDynamicsDimension(parameters.ReadAttribute(node, "dynamicsDimension", true));
	td.target_value_ = strtod(parameters.ReadAttribute(node, "value", true));

	return 0;
}

OSCGlobalAction *ScenarioReader::parseOSCGlobalAction(pugi::xml_node actionNode)
{
	OSCGlobalAction *action = 0;

	if (actionNode.first_child() == 0)
	{
		throw std::runtime_error("Missing action child node");
	}

	for (pugi::xml_node actionChild = actionNode.first_child(); actionChild; actionChild = actionChild.next_sibling())
	{
		if (actionChild.name() == std::string("ParameterAction"))
		{
			for (pugi::xml_node paramChild = actionChild.first_child(); paramChild; paramChild = paramChild.next_sibling())
			{
				if (paramChild.name() == std::string("SetAction"))
				{
					ParameterSetAction *paramSetAction = new ParameterSetAction();

					paramSetAction->name_ = parameters.ReadAttribute(actionChild, "parameterRef");
					paramSetAction->value_ = parameters.ReadAttribute(paramChild, "value");
					paramSetAction->parameters_ = &parameters;

					action = paramSetAction;
				}
				else
				{
					LOG("ParameterAction %s not supported yet", paramChild.name());
				}
			}
		}
		else
		{
			LOG("Unsupported global action: %s", actionChild.name());
		}
	}

	if (action != 0)
	{
		if (actionNode.parent().attribute("name"))
		{
			action->name_ = parameters.ReadAttribute(actionNode.parent(), "name");
		}
		else
		{
			action->name_ = "no name";
		}
	}

	return action;
}

ActivateControllerAction *ScenarioReader::parseActivateControllerAction(pugi::xml_node node)
{
	bool domain_longitudinal = parameters.ReadAttribute(node, "longitudinal") == "true";
	bool domain_lateral = parameters.ReadAttribute(node, "lateral") == "true";

	int domainMask = 0;
	if (domain_longitudinal)
	{
		domainMask |= static_cast<int>(ControlDomains::DOMAIN_LONG);
	}
	if (domain_lateral)
	{
		domainMask |= static_cast<int>(ControlDomains::DOMAIN_LAT);
	}

	ActivateControllerAction* activateControllerAction = new ActivateControllerAction(static_cast<ControlDomains>(domainMask));

	return activateControllerAction;
}

// ------------------------------------------------------
OSCPrivateAction *ScenarioReader::parseOSCPrivateAction(pugi::xml_node actionNode, Object *object)
{
	OSCPrivateAction *action = 0;

	for (pugi::xml_node actionChild = actionNode.first_child(); actionChild; actionChild = actionChild.next_sibling())
	{
		if (actionChild.name() == std::string("LongitudinalAction"))
		{
			for (pugi::xml_node longitudinalChild = actionChild.first_child(); longitudinalChild; longitudinalChild = longitudinalChild.next_sibling())
			{
				if (longitudinalChild.name() == std::string("SpeedAction"))
				{
					LongSpeedAction *action_speed = new LongSpeedAction();

					for (pugi::xml_node speedChild = longitudinalChild.first_child(); speedChild; speedChild = speedChild.next_sibling())
					{
						if (speedChild.name() == std::string("SpeedActionDynamics"))
						{
							ParseTransitionDynamics(speedChild, action_speed->transition_dynamics_);
						}
						else if (speedChild.name() == std::string("SpeedActionTarget"))
						{
							for (pugi::xml_node targetChild = speedChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("RelativeTargetSpeed"))
								{
									LongSpeedAction::TargetRelative *target_rel = new LongSpeedAction::TargetRelative;

									target_rel->value_ = strtod(parameters.ReadAttribute(targetChild, "value"));

									target_rel->continuous_ = (parameters.ReadAttribute(targetChild, "continuous") == "true" ||
															   parameters.ReadAttribute(targetChild, "continuous") == "1");

									target_rel->object_ = ResolveObjectReference(parameters.ReadAttribute(targetChild, "entityRef"));

									std::string value_type = parameters.ReadAttribute(targetChild, "speedTargetValueType");
									if (value_type == "delta")
									{
										target_rel->value_type_ = LongSpeedAction::TargetRelative::ValueType::DELTA;
									}
									else if (value_type == "factor")
									{
										target_rel->value_type_ = LongSpeedAction::TargetRelative::ValueType::FACTOR;
									}
									else if (value_type == "")
									{
										LOG("Value type missing - falling back to delta");
										target_rel->value_type_ = LongSpeedAction::TargetRelative::DELTA;
									}
									else
									{
										LOG("Value type %s not valid", value_type.c_str());
									}
									action_speed->target_ = target_rel;
								}
								else if (targetChild.name() == std::string("AbsoluteTargetSpeed"))
								{
									LongSpeedAction::TargetAbsolute *target_abs = new LongSpeedAction::TargetAbsolute;

									target_abs->value_ = strtod(parameters.ReadAttribute(targetChild, "value"));
									action_speed->target_ = target_abs;
								}
								else
								{
									LOG("Unsupported Target type: %s", targetChild.name());
								}
							}
						}
					}
					action = action_speed;
				}
				else if (longitudinalChild.name() == std::string("LongitudinalDistanceAction"))
				{
					LongDistanceAction *action_dist = new LongDistanceAction();

					pugi::xml_node dynamics_node = longitudinalChild.child("DynamicConstraints");
					if (dynamics_node != NULL)
					{
						action_dist->dynamics_.max_acceleration_ = strtod(parameters.ReadAttribute(dynamics_node, "maxAcceleration"));
						if (action_dist->dynamics_.max_acceleration_ < SMALL_NUMBER)
						{
							LOG("Unexpected small maxAcceleration value: %.2, replacing with %.2f", action_dist->dynamics_.max_acceleration_, 10);
							action_dist->dynamics_.max_acceleration_ = 10.0;
						}

						action_dist->dynamics_.max_deceleration_ = strtod(parameters.ReadAttribute(dynamics_node, "maxDeceleration"));
						if (action_dist->dynamics_.max_deceleration_ < SMALL_NUMBER)
						{
							LOG("Unexpected small maxDeceleration value: %.2, replacing with %.2f", action_dist->dynamics_.max_deceleration_, 10);
							action_dist->dynamics_.max_deceleration_ = 10.0;
						}

						if (!dynamics_node.attribute("maxSpeed"))
						{
							action_dist->dynamics_.max_speed_ = 250 / 3.6; // Use default 250 kph if attribute is missing
						}
						else
						{
							action_dist->dynamics_.max_speed_ = strtod(parameters.ReadAttribute(dynamics_node, "maxSpeed"));
						}
						action_dist->dynamics_.none_ = false;
					}
					else
					{
						action_dist->dynamics_.none_ = true;
					}

					action_dist->target_object_ = ResolveObjectReference(parameters.ReadAttribute(longitudinalChild, "entityRef"));
					if (longitudinalChild.attribute("distance"))
					{
						action_dist->dist_type_ = LongDistanceAction::DistType::DISTANCE;
						action_dist->distance_ = strtod(parameters.ReadAttribute(longitudinalChild, "distance"));
					}
					else if (longitudinalChild.attribute("timeGap"))
					{
						action_dist->dist_type_ = LongDistanceAction::DistType::TIME_GAP;
						action_dist->distance_ = strtod(parameters.ReadAttribute(longitudinalChild, "timeGap"));
					}
					else
					{
						LOG("Need distance or timeGap");
					}

					std::string continuous = parameters.ReadAttribute(longitudinalChild, "continuous");
					action_dist->continuous_ = continuous == "true" ? true : false;

					std::string freespace = parameters.ReadAttribute(longitudinalChild, "freespace");
					if (freespace == "true" || freespace == "1")
						action_dist->freespace_ = true;
					else
						action_dist->freespace_ = false;

					std::string displacement = parameters.ReadAttribute(longitudinalChild, "displacement");
					if (GetVersionMajor() <= 1 && GetVersionMinor() >= 1)
					{
						if (displacement.empty())
						{
							LOG("displacement attribute missing, setting default trailingReferencedEntity");
							action_dist->displacement_ = LongDistanceAction::DisplacementType::TRAILING;
						}
						else if (displacement == "any")
						{
							action_dist->displacement_ = LongDistanceAction::DisplacementType::ANY;
						}
						else if (displacement == "trailingReferencedEntity")
						{
							action_dist->displacement_ = LongDistanceAction::DisplacementType::TRAILING;
						}
						else if (displacement == "leadingReferencedEntity")
						{
							action_dist->displacement_ = LongDistanceAction::DisplacementType::LEADING;
						}
						else
						{
							LOG_AND_QUIT("Unsupported displacement type: %s", displacement.c_str());
						}

						if (action_dist->distance_ < 0.0)
						{
							// action_dist->displacement_ != LongDistanceAction::DisplacementType::NONE &&
							LOG("Negative distance or timeGap not supported in OSC version >= 1.1. Using absolute value. Use displacement to specify leading or trailing behavior.");
							action_dist->distance_ = abs(action_dist->distance_);
						}
					}
					else if (!displacement.empty())
					{
						LOG("LongitudinalDistanceAction displacement not supported in OSC version %d.%d", GetVersionMajor(), GetVersionMinor());
					}

					action = action_dist;
				}
			}
		}
		else if (actionChild.name() == std::string("LateralAction"))
		{
			for (pugi::xml_node lateralChild = actionChild.first_child(); lateralChild; lateralChild = lateralChild.next_sibling())
			{
				if (lateralChild.name() == std::string("LaneChangeAction"))
				{
					LatLaneChangeAction *action_lane = new LatLaneChangeAction();

					if (parameters.ReadAttribute(lateralChild, "targetLaneOffset") != "")
					{
						action_lane->target_lane_offset_ = strtod(parameters.ReadAttribute(lateralChild, "targetLaneOffset"));
					}
					else
					{
						action_lane->target_lane_offset_ = 0;
					}

					for (pugi::xml_node laneChangeChild = lateralChild.first_child(); laneChangeChild; laneChangeChild = laneChangeChild.next_sibling())
					{
						if (laneChangeChild.name() == std::string("LaneChangeActionDynamics"))
						{
							ParseTransitionDynamics(laneChangeChild, action_lane->transition_dynamics_);
						}
						else if (laneChangeChild.name() == std::string("LaneChangeTarget"))
						{
							LatLaneChangeAction::Target *target = 0;

							for (pugi::xml_node targetChild = laneChangeChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("RelativeTargetLane"))
								{
									LatLaneChangeAction::TargetRelative *target_rel = new LatLaneChangeAction::TargetRelative;

									if ((target_rel->object_ = ResolveObjectReference(parameters.ReadAttribute(targetChild, "entityRef"))) == 0)
									{
										LOG("Failed to find object %s", parameters.ReadAttribute(targetChild, "entityRef").c_str());
										return 0;
									}
									target_rel->value_ = strtoi(parameters.ReadAttribute(targetChild, "value"));
									target = target_rel;
								}
								else if (targetChild.name() == std::string("AbsoluteTargetLane"))
								{
									LatLaneChangeAction::TargetAbsolute *target_abs = new LatLaneChangeAction::TargetAbsolute;

									target_abs->value_ = strtoi(parameters.ReadAttribute(targetChild, "value"));
									target = target_abs;
								}
								else
								{
									throw std::runtime_error(std::string("Unsupported LaneChangeTarget: ") + targetChild.name());
								}
							}
							if (target)
							{
								action_lane->target_ = target;
							}
						}
					}
					action = action_lane;
				}
				else if (lateralChild.name() == std::string("LaneOffsetAction"))
				{
					LatLaneOffsetAction *action_lane = new LatLaneOffsetAction();
					LatLaneOffsetAction::Target *target = nullptr;
					for (pugi::xml_node laneOffsetChild = lateralChild.first_child(); laneOffsetChild; laneOffsetChild = laneOffsetChild.next_sibling())
					{
						if (laneOffsetChild.name() == std::string("LaneOffsetActionDynamics"))
						{
							if (parameters.ReadAttribute(laneOffsetChild, "maxLateralAcc") != "")
							{
								action_lane->dynamics_.max_lateral_acc_ = strtod(parameters.ReadAttribute(laneOffsetChild, "maxLateralAcc"));
								if (action_lane->dynamics_.max_lateral_acc_ < SMALL_NUMBER)
								{
									action_lane->dynamics_.max_lateral_acc_ = SMALL_NUMBER;
								}
							}
							else
							{
								action_lane->dynamics_.max_lateral_acc_ = 0.5; // Just set some reasonable default value
								LOG("Missing optional LaneOffsetAction maxLateralAcc attribute. Using default: %.2f", action_lane->dynamics_.max_lateral_acc_);
							}

							action_lane->dynamics_.transition_.shape_ = ParseDynamicsShape(parameters.ReadAttribute(laneOffsetChild, "dynamicsShape"));
						}
						else if (laneOffsetChild.name() == std::string("LaneOffsetTarget"))
						{
							for (pugi::xml_node targetChild = laneOffsetChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("RelativeTargetLaneOffset"))
								{
									LatLaneOffsetAction::TargetRelative *target_rel = new LatLaneOffsetAction::TargetRelative;

									target_rel->object_ = ResolveObjectReference(parameters.ReadAttribute(targetChild, "entityRef"));
									target_rel->value_ = strtod(parameters.ReadAttribute(targetChild, "value"));
									target = target_rel;
								}
								else if (targetChild.name() == std::string("AbsoluteTargetLaneOffset"))
								{
									LatLaneOffsetAction::TargetAbsolute *target_abs = new LatLaneOffsetAction::TargetAbsolute;

									target_abs->value_ = strtod(parameters.ReadAttribute(targetChild, "value"));
									target = target_abs;
								}
							}
							action_lane->target_ = target;
						}
					}
					if (target == nullptr)
					{
						throw std::runtime_error("Missing LaneOffset Target");
					}
					action = action_lane;
				}
				else
				{
					LOG("Unsupported element type: %s", lateralChild.name());
				}
			}
		}
		else if (actionChild.name() == std::string("SynchronizeAction"))
		{
			SynchronizeAction *action_synch = new SynchronizeAction;

			std::string master_object_str = parameters.ReadAttribute(actionChild, "masterEntityRef");
			action_synch->master_object_ = ResolveObjectReference(master_object_str);

			pugi::xml_node target_position_master_node = actionChild.child("TargetPositionMaster");
			if (!target_position_master_node)
			{
				LOG("Missing required element \"TargetPositionMaster\"");
				return 0;
			}
			action_synch->target_position_master_ = parseOSCPosition(target_position_master_node)->GetRMPos();
			if (parameters.ReadAttribute(actionChild, "targetToleranceMaster") != "")
			{
				action_synch->tolerance_master_ = strtod(parameters.ReadAttribute(actionChild, "targetToleranceMaster"));
			}

			pugi::xml_node target_position_node = actionChild.child("TargetPosition");
			if (!target_position_node)
			{
				LOG("Missing required element \"TargetPosition\"");
				return 0;
			}
			action_synch->target_position_ = parseOSCPosition(target_position_node)->GetRMPos();
			if (parameters.ReadAttribute(actionChild, "targetTolerance") != "")
			{
				action_synch->tolerance_ = strtod(parameters.ReadAttribute(actionChild, "targetTolerance"));
			}

			pugi::xml_node target_speed_node = actionChild.child("FinalSpeed");
			if (target_speed_node)
			{
				pugi::xml_node final_speed_element = target_speed_node.first_child();
				if (final_speed_element.empty())
				{
					throw std::runtime_error("Unexpected missing FinalSpeed child element");
				}

				if (!strcmp(final_speed_element.name(), "AbsoluteSpeed"))
				{
					LongSpeedAction::TargetAbsolute *targetSpeedAbs = new LongSpeedAction::TargetAbsolute;
					targetSpeedAbs->value_ = strtod(parameters.ReadAttribute(final_speed_element, "value"));
					action_synch->final_speed_ = targetSpeedAbs;
				}
				else if (!strcmp(final_speed_element.name(), "RelativeSpeedToMaster"))
				{
					LongSpeedAction::TargetRelative *targetSpeedRel = new LongSpeedAction::TargetRelative;

					targetSpeedRel->value_ = strtod(parameters.ReadAttribute(final_speed_element, "value"));

					targetSpeedRel->continuous_ = true; // Continuous adaption needed

					targetSpeedRel->object_ = action_synch->master_object_; // Master object is the pivot vehicle

					std::string value_type = parameters.ReadAttribute(final_speed_element, "speedTargetValueType");
					if (value_type == "delta")
					{
						targetSpeedRel->value_type_ = LongSpeedAction::TargetRelative::ValueType::DELTA;
					}
					else if (value_type == "factor")
					{
						targetSpeedRel->value_type_ = LongSpeedAction::TargetRelative::ValueType::FACTOR;
					}
					else if (value_type == "")
					{
						LOG("Value type missing - falling back to delta");
						targetSpeedRel->value_type_ = LongSpeedAction::TargetRelative::DELTA;
					}
					else
					{
						LOG("Value type %s not valid", value_type.c_str());
					}
					action_synch->final_speed_ = targetSpeedRel;
				}
				else
				{
					LOG("Unexpected FinalSpeed element: %s", final_speed_element.name());
					throw std::runtime_error("Unexpected FinalSpeed element");
				}

				// Check for optional steady state element
				pugi::xml_node steady_state_node = final_speed_element.first_child();
				if (!steady_state_node.empty())
				{
					if (GetVersionMajor() <= 1 && GetVersionMinor() < 1)
					{
						LOG("SynchronizeAction::SteadyState introduced in v1.1. Reading anyway.");
					}
					if (action_synch->final_speed_->type_ == LongSpeedAction::Target::TargetType::ABSOLUTE &&
						action_synch->final_speed_->GetValue() < SMALL_NUMBER)
					{
						LOG("SynchronizeAction steady state with 0 or negative final speed (%.2f) is not supported", action_synch->final_speed_->GetValue());
						throw std::runtime_error("SynchronizeAction steady state with 0 or negative final speed is not supported");
					}
					if (!strcmp(steady_state_node.name(), "TargetDistanceSteadyState"))
					{
						action_synch->steadyState_.type_ = SynchronizeAction::SteadyStateType::STEADY_STATE_DIST;
						action_synch->steadyState_.dist_ = strtod(parameters.ReadAttribute(steady_state_node, "distance"));
					}
					else if (!strcmp(steady_state_node.name(), "TargetTimeSteadyState"))
					{
						action_synch->steadyState_.type_ = SynchronizeAction::SteadyStateType::STEADY_STATE_TIME;
						action_synch->steadyState_.time_ = strtod(parameters.ReadAttribute(steady_state_node, "time"));
					}
					else if (!strcmp(steady_state_node.name(), "TargetPositionSteadyState"))
					{
						OSCPosition *pos = parseOSCPosition(steady_state_node);
						action_synch->steadyState_.type_ = SynchronizeAction::SteadyStateType::STEADY_STATE_POS;
						action_synch->steadyState_.pos_ = pos->GetRMPos();
					}
					else
					{
						LOG("Unexpected child element: %s", steady_state_node.name());
						throw std::runtime_error("Unexpected child element: " + std::string(steady_state_node.name()));
					}
				}
			}
			action = action_synch;
		}
		else if (actionChild.name() == std::string("TeleportAction"))
		{
			TeleportAction *action_pos = new TeleportAction;
			OSCPosition *pos = parseOSCPosition(actionChild.first_child());
			action_pos->position_ = pos->GetRMPos();
			action = action_pos;
		}
		else if (actionChild.name() == std::string("RoutingAction"))
		{
			for (pugi::xml_node routingChild = actionChild.first_child(); routingChild; routingChild = routingChild.next_sibling())
			{
				if (routingChild.name() == std::string("AssignRouteAction"))
				{
					for (pugi::xml_node assignRouteChild = routingChild.first_child(); assignRouteChild; assignRouteChild = assignRouteChild.next_sibling())
					{
						if (assignRouteChild.name() == std::string("Route"))
						{
							AssignRouteAction *action_follow_route = new AssignRouteAction;
							action_follow_route->route_ = parseOSCRoute(assignRouteChild);
							action = action_follow_route;
						}
						else if (assignRouteChild.name() == std::string("CatalogReference"))
						{
							AssignRouteAction *action_assign_route = new AssignRouteAction;

							// Find route in catalog
							Entry *entry = ResolveCatalogReference(assignRouteChild);

							if (entry == 0 || entry->node_ == 0)
							{
								throw std::runtime_error("Failed to resolve catalog reference");
							}

							if (entry->type_ == CatalogType::CATALOG_ROUTE)
							{
								// Make a new instance from catalog entry
								action_assign_route->route_ = parseOSCRoute(entry->GetNode());
								action = action_assign_route;
								break;
							}
							else
							{
								throw std::runtime_error(std::string("Found catalog entry ") + entry->name_ + ". But wrong type: " + entry->GetTypeAsStr() + ". Expected: " +
														 Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE) + ".");
							}
						}
					}
				}
				else if (routingChild.name() == std::string("FollowTrajectoryAction"))
				{
					FollowTrajectoryAction *action_follow_trajectory = new FollowTrajectoryAction;

					if (!routingChild.attribute("initialDistanceOffset").empty())
					{
						action_follow_trajectory->initialDistanceOffset_ = strtod(parameters.ReadAttribute(routingChild, "initialDistanceOffset"));
					}
					else
					{
						action_follow_trajectory->initialDistanceOffset_ = 0.0;
					}

					for (pugi::xml_node followTrajectoryChild = routingChild.first_child(); followTrajectoryChild; followTrajectoryChild = followTrajectoryChild.next_sibling())
					{
						if (followTrajectoryChild.name() == std::string("TrajectoryRef"))
						{
							action_follow_trajectory->traj_ = parseTrajectoryRef(followTrajectoryChild);
						}
						else if (followTrajectoryChild.name() == std::string("Trajectory"))
						{
							action_follow_trajectory->traj_ = parseTrajectory(followTrajectoryChild);
						}
						else if (followTrajectoryChild.name() == std::string("CatalogReference"))
						{
							// Find trajectory in catalog
							Entry *entry = ResolveCatalogReference(followTrajectoryChild);

							if (entry == 0 || entry->node_ == 0)
							{
								throw std::runtime_error("Failed to resolve catalog reference");
							}

							if (entry->type_ == CatalogType::CATALOG_TRAJECTORY)
							{
								// Make a new instance from catalog entry
								action_follow_trajectory->traj_ = parseTrajectory(entry->GetNode());
							}
							else
							{
								throw std::runtime_error(std::string("Found catalog entry ") + entry->name_ + ". But wrong type: " + entry->GetTypeAsStr() + ". Expected: " +
														 Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE) + ".");
							}
						}
						else if (followTrajectoryChild.name() == std::string("TimeReference"))
						{
							pugi::xml_node timingNode = followTrajectoryChild.first_child();
							if (timingNode && std::string(timingNode.name()) == "Timing")
							{
								std::string timeDomain = parameters.ReadAttribute(timingNode, "domainAbsoluteRelative");
								if (timeDomain == "relative")
								{
									action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::TIMING_RELATIVE;
								}
								else if (timeDomain == "absolute")
								{
									action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::TIMING_ABSOLUTE;
								}
								else
								{
									throw std::runtime_error("Unexpected TimeDomain: " + timeDomain);
								}

								action_follow_trajectory->timing_scale_ = strtod(parameters.ReadAttribute(timingNode, "scale"));
								action_follow_trajectory->timing_offset_ = strtod(parameters.ReadAttribute(timingNode, "offset"));
							}
							else if (timingNode && std::string(timingNode.name()) == "None")
							{
								action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::NONE;
							}
							else
							{
								LOG("Unexpected TimeReference element: %s, fallback to NONE", timingNode.name());
								action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::NONE;
							}
						}
						else if (followTrajectoryChild.name() == std::string("TrajectoryFollowingMode"))
						{
							std::string followingMode = parameters.ReadAttribute(followTrajectoryChild, "followingMode");
							if (followingMode.empty())
							{
								LOG("trajectoryFollowingMode missing, applying \"position\"");
							}
							else if (followingMode != "position")
							{
								LOG("trajectoryFollowingMode %s not supported yet, applying \"position\"", followingMode.c_str());
							}
						}
						else
						{
							throw std::runtime_error("Unexpected element: " + std::string(followTrajectoryChild.name()));
						}
					}
					// Check that trajectory has time duration
					if (action_follow_trajectory->timing_domain_ != FollowTrajectoryAction::TimingDomain::NONE &&
						action_follow_trajectory->traj_->GetDuration() < SMALL_NUMBER)
					{
						LOG("Warning: FollowTrajectoryAction timeref is != NONE but trajectory duration is 0. Applying timeref=NONE.",
							action_follow_trajectory->name_.c_str());
						action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::NONE;
					}
					action = action_follow_trajectory;
				}
				else if (routingChild.name() == std::string("AcquirePositionAction"))
				{
					AcquirePositionAction* acqPosAction = new AcquirePositionAction;
					OSCPosition* pos = parseOSCPosition(routingChild.first_child());
					acqPosAction->target_position_ = pos->GetRMPos();
					action = acqPosAction;
				}
				else
				{
					throw std::runtime_error("Action is not supported: " + std::string(routingChild.name()));
				}
			}
		}
		else if (actionChild.name() == std::string("ActivateControllerAction"))
		{
			if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
			{
				LOG("In OSC 1.1 ActivateControllerAction should be placed under ControllerAction. Accepting anyway.");
			}

			ActivateControllerAction* activateControllerAction = parseActivateControllerAction(actionChild);

			action = activateControllerAction;
		}
		else if (actionChild.name() == std::string("ControllerAction"))
		{
			for (pugi::xml_node controllerChild = actionChild.first_child(); controllerChild; controllerChild = controllerChild.next_sibling())
			{
				if (controllerChild.name() == std::string("AssignControllerAction"))
				{
					for (pugi::xml_node controllerDefNode = controllerChild.first_child(); controllerDefNode; controllerDefNode = controllerDefNode.next_sibling())
					{
						Controller *controller = 0;
						if (controllerDefNode.name() == std::string("Controller"))
						{
							controller = parseOSCObjectController(controllerDefNode);
						}
						else if (controllerDefNode.name() == std::string("CatalogReference"))
						{
							Entry *entry = ResolveCatalogReference(controllerDefNode);

							if (entry == 0)
							{
								LOG("No entry found");
							}
							else
							{
								if (entry->type_ == CatalogType::CATALOG_CONTROLLER)
								{
									controller = parseOSCObjectController(entry->GetNode());
								}
								else
								{
									LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
								}
							}
						}
						else
						{
							LOG("Unexpected AssignControllerAction subelement: %s", controllerDefNode.name());
							return 0;
						}

						if (controller)
						{
							controller_.push_back(controller);
						}
						AssignControllerAction *assignControllerAction = new AssignControllerAction(controller);
						action = assignControllerAction;
					}
				}
				else if (controllerChild.name() == std::string("OverrideControllerValueAction"))
				{
					OverrideControlAction *override_action = new OverrideControlAction();
					Object::OverrideActionStatus overrideStatus;

					for (pugi::xml_node controllerDefNode = controllerChild.first_child(); controllerDefNode; controllerDefNode = controllerDefNode.next_sibling())
					{
						double value = strtod(parameters.ReadAttribute(controllerDefNode, "value"));
						// read active flag
						overrideStatus.active = parameters.ReadAttribute(controllerDefNode, "active") == "true" ? true : false;

						if (controllerDefNode.name() == std::string("Throttle"))
						{
							overrideStatus.type = Object::OverrideType::OVERRIDE_THROTTLE;
							overrideStatus.value = override_action->RangeCheckAndErrorLog(overrideStatus.type, value);
						}
						else if (controllerDefNode.name() == std::string("Brake"))
						{
							overrideStatus.type = Object::OverrideType::OVERRIDE_BRAKE;
							overrideStatus.value = override_action->RangeCheckAndErrorLog(overrideStatus.type, value);
						}
						else if (controllerDefNode.name() == std::string("Clutch"))
						{
							overrideStatus.type = Object::OverrideType::OVERRIDE_CLUTCH;
							overrideStatus.value = override_action->RangeCheckAndErrorLog(overrideStatus.type, value);
						}
						else if (controllerDefNode.name() == std::string("ParkingBrake"))
						{
							overrideStatus.type = Object::OverrideType::OVERRIDE_PARKING_BRAKE;
							overrideStatus.value = override_action->RangeCheckAndErrorLog(overrideStatus.type, value);
						}
						else if (controllerDefNode.name() == std::string("SteeringWheel"))
						{
							overrideStatus.type = Object::OverrideType::OVERRIDE_STEERING_WHEEL;
							overrideStatus.value = override_action->RangeCheckAndErrorLog(overrideStatus.type, value, -2 * M_PI, 2 * M_PI);
						}
						else if (controllerDefNode.name() == std::string("Gear"))
						{
							overrideStatus.type = Object::OverrideType::OVERRIDE_GEAR;
							overrideStatus.value = override_action->RangeCheckAndErrorLog(overrideStatus.type, value, -1, 8, true);
						}
						else
						{
							LOG("Unexpected OverrideControllerValueAction subelement: %s", controllerDefNode.name());
							return 0;
						}

						override_action->overrideActionList.push_back(overrideStatus);
					}

					action = override_action;
				}
				else if (controllerChild.name() == std::string("ActivateControllerAction"))
				{
					if (GetVersionMajor() == 1 && GetVersionMinor() == 0)
					{
						LOG("In OSC 1.0 ActivateControllerAction should be placed under PrivateAction. Accepting anyway.");
					}
					ActivateControllerAction* activateControllerAction = parseActivateControllerAction(controllerChild);

					action = activateControllerAction;
				}
				else
				{
					LOG("Unexpected ControllerAction subelement: %s", controllerChild.name());
				}
			}
		}
		else if (actionChild.name() == std::string("VisibilityAction"))
		{
			bool graphics = true;
			bool traffic = true;
			bool sensors = true;

			// Find attributes
			std::string attributeString = parameters.ReadAttribute(actionChild, "graphics");
			if (attributeString == "false" || attributeString == "False")
			{
				graphics = false;
			}

			attributeString = parameters.ReadAttribute(actionChild, "traffic");
			if (attributeString == "false" || attributeString == "False")
			{
				traffic = false;
			}

			attributeString = parameters.ReadAttribute(actionChild, "sensors");
			if (attributeString == "false" || attributeString == "False")
			{
				sensors = false;
			}

			VisibilityAction *visAction = new VisibilityAction();
			visAction->graphics_ = graphics;
			visAction->traffic_ = traffic;
			visAction->sensors_ = sensors;

			action = visAction;
		}
		else
		{
			throw std::runtime_error("Action is not supported: " + std::string(actionChild.name()));
		}
	}

	if (action != 0)
	{
		if (actionNode.parent().attribute("name"))
		{
			action->name_ = parameters.ReadAttribute(actionNode.parent(), "name");
		}
		else
		{
			action->name_ = "no name";
		}
		action->object_ = object;
	}

	return action;
}

void ScenarioReader::parseInit(Init &init)
{
	pugi::xml_node actionsNode = osc_root_.child("Storyboard").child("Init").child("Actions");

	for (pugi::xml_node actionsChild = actionsNode.first_child(); actionsChild; actionsChild = actionsChild.next_sibling())
	{
		std::string actionsChildName(actionsChild.name());

		if (actionsChildName == "GlobalAction")
		{
			LOG("Parsing global action %s", parameters.ReadAttribute(actionsChild, "name").c_str());
			OSCGlobalAction *action = parseOSCGlobalAction(actionsChild);
			if (action != 0)
			{
				action->name_ = "Init " + parameters.ReadAttribute(actionsChild, "name");
			}
		}
		else if (actionsChildName == "UserDefined")
		{
			LOG("Init %s is not implemented", actionsChildName.c_str());
		}
		else if (actionsChildName == "Private")
		{
			Object *entityRef;

			entityRef = ResolveObjectReference(parameters.ReadAttribute(actionsChild, "entityRef"));
			if (entityRef != NULL)
			{
				for (pugi::xml_node privateChild = actionsChild.first_child(); privateChild; privateChild = privateChild.next_sibling())
				{
					// Assume children are PrivateActions
					OSCPrivateAction *action = parseOSCPrivateAction(privateChild, entityRef);
					if (action != 0)
					{
						action->name_ = "Init " + entityRef->name_ + " " + privateChild.first_child().name();
						init.private_action_.push_back(action);
					}
				}
			}
		}
	}

	// Now, potentially the list of init actions needs to be sorted based on dependencies
	for (size_t i = 1; i < init.private_action_.size(); i++)
	{
		// If relative actions depends on earlier action, switch position
		for (size_t j = 0; j < i; j++)
		{
			roadmanager::Position *pos = 0;
			if (init.private_action_[j]->type_ == OSCPrivateAction::ActionType::TELEPORT)
			{
				TeleportAction *action = (TeleportAction *)init.private_action_[j];
				if (action->position_->GetType() == roadmanager::Position::PositionType::RELATIVE_LANE)
				{
					pos = ((roadmanager::Position *)action->position_)->GetRelativePosition();
				}
				else if (action->position_->GetType() == roadmanager::Position::PositionType::RELATIVE_OBJECT)
				{
					pos = ((roadmanager::Position *)action->position_)->GetRelativePosition();
				}
			}
			if (pos == &init.private_action_[i]->object_->pos_)
			{
				// swap places
				OSCPrivateAction *tmp_action = init.private_action_[i];
				init.private_action_[i] = init.private_action_[j];
				init.private_action_[j] = tmp_action;
			}
		}
	}
}

static OSCCondition::ConditionEdge ParseConditionEdge(std::string edge)
{
	if (edge == "rising")
	{
		return OSCCondition::ConditionEdge::RISING;
	}
	else if (edge == "falling")
	{
		return OSCCondition::ConditionEdge::FALLING;
	}
	else if (edge == "risingOrFalling")
	{
		return OSCCondition::ConditionEdge::RISING_OR_FALLING;
	}
	else if (edge == "none")
	{
		return OSCCondition::ConditionEdge::NONE;
	}
	else
	{
		LOG("Unsupported edge: %s", edge.c_str());
	}

	return OSCCondition::ConditionEdge::UNDEFINED;
}

static Rule ParseRule(std::string rule)
{
	if (rule == "greaterThan")
	{
		return Rule::GREATER_THAN;
	}
	else if (rule == "greaterOrEqual")
	{
		return Rule::GREATER_OR_EQUAL;
	}
	else if (rule == "lessThan")
	{
		return Rule::LESS_THAN;
	}
	else if (rule == "lessOrEqual")
	{
		return Rule::LESS_OR_EQUAL;
	}
	else if (rule == "equalTo")
	{
		return Rule::EQUAL_TO;
	}
	else if (rule == "notEqualTo")
	{
		return Rule::NOT_EQUAL_TO;
	}
	else
	{
		LOG("Invalid or missing rule %s", rule.c_str());
	}

	return Rule::UNDEFINED;
}

static TrigByState::CondElementState ParseState(std::string state)
{
	if (state == "startTransition")
	{
		return TrigByState::CondElementState::START_TRANSITION;
	}
	else if (state == "endTransition")
	{
		return TrigByState::CondElementState::END_TRANSITION;
	}
	else if (state == "stopTransition")
	{
		return TrigByState::CondElementState::STOP_TRANSITION;
	}
	else if (state == "skipTransition")
	{
		return TrigByState::CondElementState::SKIP_TRANSITION;
	}
	else if (state == "completeState")
	{
		return TrigByState::CondElementState::COMPLETE;
	}
	else if (state == "runningState")
	{
		return TrigByState::CondElementState::RUNNING;
	}
	else if (state == "standbyState")
	{
		return TrigByState::CondElementState::STANDBY;
	}
	else
	{
		LOG("Invalid state %s", state.c_str());
	}

	return TrigByState::CondElementState::UNDEFINED_ELEMENT_STATE;
}

static StoryBoardElement::ElementType ParseElementType(std::string element_type)
{
	if (element_type == "story")
	{
		return StoryBoardElement::ElementType::STORY;
	}
	else if (element_type == "act")
	{
		return StoryBoardElement::ElementType::ACT;
	}
	else if (element_type == "maneuver")
	{
		return StoryBoardElement::ElementType::MANEUVER;
	}
	else if (element_type == "event")
	{
		return StoryBoardElement::ElementType::EVENT;
	}
	else if (element_type == "action")
	{
		return StoryBoardElement::ElementType::ACTION;
	}
	else
	{
		LOG("Invalid or unsupported element type %s", element_type.c_str());
	}

	return StoryBoardElement::ElementType::UNDEFINED_ELEMENT_TYPE;
}
// ------------------------------------------
OSCCondition *ScenarioReader::parseOSCCondition(pugi::xml_node conditionNode)
{
	std::string condition_type;

	OSCCondition *condition = 0;

	for (pugi::xml_node conditionChild = conditionNode.first_child(); conditionChild; conditionChild = conditionChild.next_sibling())
	{
		std::string conditionChildName(conditionChild.name());
		if (conditionChildName == "ByEntityCondition")
		{
			pugi::xml_node entity_condition = conditionChild.child("EntityCondition");
			if (entity_condition == NULL)
			{
				throw std::runtime_error("Missing EntityCondition");
			}
			else
			{
				for (pugi::xml_node condition_node = entity_condition.first_child(); condition_node; condition_node = condition_node.next_sibling())
				{
					condition_type = condition_node.name();
					if (condition_type == "TimeHeadwayCondition")
					{
						TrigByTimeHeadway *trigger = new TrigByTimeHeadway;
						trigger->object_ = ResolveObjectReference(parameters.ReadAttribute(condition_node, "entityRef"));

						std::string freespace_str = parameters.ReadAttribute(condition_node, "freespace");
						if ((freespace_str == "true") || (freespace_str == "1"))
						{
							trigger->freespace_ = true;
						}
						else
						{
							trigger->freespace_ = false;
						}

						trigger->cs_ = ParseCoordinateSystem(condition_node, roadmanager::CoordinateSystem::CS_UNDEFINED);
						trigger->relDistType_ = ParseRelativeDistanceType(condition_node, roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED);

						if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED && trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
						{
							// look for v1.0 attribute alongroute
							std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
							if (!along_route_str.empty())
							{
								if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
								{
									LOG("alongRoute attribute is depricated from v1.1. Reading it anyway.");
								}
								if ((along_route_str == "true") || (along_route_str == "1"))
								{
									trigger->cs_ = roadmanager::CoordinateSystem::CS_ROAD;
								}
							}
						}

						if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED)
						{
							// Set default value
							trigger->cs_ = roadmanager::CoordinateSystem::CS_ENTITY;
						}

						if (trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
						{
							// Set default value
							trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
						}

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "TimeToCollisionCondition")
					{
						TrigByTimeToCollision *trigger = new TrigByTimeToCollision;

						pugi::xml_node target = condition_node.child("TimeToCollisionConditionTarget");
						pugi::xml_node targetChild = target.first_child();
						std::string targetChildName(targetChild.name());
						if (targetChildName == "Position")
						{
							trigger->position_ = parseOSCPosition(targetChild);
						}
						else if (targetChildName == "EntityRef")
						{
							trigger->object_ = ResolveObjectReference(parameters.ReadAttribute(targetChild, "entityRef"));
						}
						else
						{
							LOG("Unexpected target type: %s", targetChildName.c_str());
							return 0;
						}

						std::string freespace_str = parameters.ReadAttribute(condition_node, "freespace");
						if ((freespace_str == "true") || (freespace_str == "1"))
						{
							trigger->freespace_ = true;
						}
						else
						{
							trigger->freespace_ = false;
						}

						trigger->cs_ = ParseCoordinateSystem(condition_node, roadmanager::CoordinateSystem::CS_UNDEFINED);
						trigger->relDistType_ = ParseRelativeDistanceType(condition_node, roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED);

						if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED && trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
						{
							// look for v1.0 attribute alongroute
							std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
							if (!along_route_str.empty())
							{
								if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
								{
									LOG("alongRoute attribute is depricated from v1.1. Reading it anyway.");
								}
								if ((along_route_str == "true") || (along_route_str == "1"))
								{
									trigger->cs_ = roadmanager::CoordinateSystem::CS_ROAD;
								}
							}
						}

						if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED)
						{
							// Set default value
							trigger->cs_ = roadmanager::CoordinateSystem::CS_ENTITY;
						}

						if (trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
						{
							// Set default value
							trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
						}

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "ReachPositionCondition")
					{
						TrigByReachPosition *trigger = new TrigByReachPosition;

						if (!condition_node.attribute("tolerance"))
						{
							LOG("tolerance is required");
						}
						else
						{
							trigger->tolerance_ = strtod(parameters.ReadAttribute(condition_node, "tolerance"));
						}

						// Read position
						pugi::xml_node pos_node = condition_node.child("Position");
						trigger->position_ = parseOSCPosition(pos_node);

						condition = trigger;
					}
					else if (condition_type == "RelativeDistanceCondition")
					{
						TrigByRelativeDistance *trigger = new TrigByRelativeDistance;
						trigger->object_ = ResolveObjectReference(parameters.ReadAttribute(condition_node, "entityRef"));

						std::string freespace_str = parameters.ReadAttribute(condition_node, "freespace");
						if ((freespace_str == "true") || (freespace_str == "1"))
						{
							trigger->freespace_ = true;
						}
						else
						{
							trigger->freespace_ = false;
						}

						trigger->cs_ = ParseCoordinateSystem(condition_node, roadmanager::CoordinateSystem::CS_ENTITY);
						trigger->relDistType_ = ParseRelativeDistanceType(condition_node, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN);

						if (!condition_node.attribute("relativeDistanceType").empty())
						{
							std::string type = parameters.ReadAttribute(condition_node, "relativeDistanceType");
							if ((type == "longitudinal") || (type == "Longitudinal"))
							{
								trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL;
							}
							else if ((type == "lateral") || (type == "Lateral"))
							{
								trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_LATERAL;
							}
							else if ((type == "cartesianDistance") || (type == "CartesianDistance"))
							{
								trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
							}
							else
							{
								std::string msg = "Unexpected relativeDistanceType: " + type;
								LOG(msg.c_str());
								throw std::runtime_error(msg);
							}
						}

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "CollisionCondition")
					{
						TrigByCollision *trigger = new TrigByCollision;

						pugi::xml_node by_type = condition_node.child("ByType");
						if (by_type)
						{
							std::string type_str = parameters.ReadAttribute(by_type, "type");
							if (type_str == "pedestrian")
							{
								trigger->type_ = Object::Type::PEDESTRIAN;
							}
							else if (type_str == "vehicle")
							{
								trigger->type_ = Object::Type::VEHICLE;
							}
							else if (type_str == "miscellaneous")
							{
								trigger->type_ = Object::Type::MISC_OBJECT;
							}
							else
							{
								throw std::runtime_error(std::string("Unexpected ObjectType in CollisionCondition: ") + type_str);
							}
						}
						else  // Assume EntityRef
						{
							pugi::xml_node target = condition_node.child("EntityRef");
							trigger->object_ = ResolveObjectReference(parameters.ReadAttribute(target, "entityRef"));
							trigger->type_ = Object::Type::TYPE_NONE;
						}

						condition = trigger;
					}
					else if (condition_type == "DistanceCondition")
					{
						TrigByDistance *trigger = new TrigByDistance;

						// Read position
						pugi::xml_node pos_node = condition_node.child("Position");

						trigger->position_ = parseOSCPosition(pos_node);

						std::string freespace_str = parameters.ReadAttribute(condition_node, "freespace");
						if ((freespace_str == "true") || (freespace_str == "1"))
						{
							trigger->freespace_ = true;
						}
						else
						{
							trigger->freespace_ = false;
						}

						trigger->cs_ = ParseCoordinateSystem(condition_node, roadmanager::CoordinateSystem::CS_UNDEFINED);
						trigger->relDistType_ = ParseRelativeDistanceType(condition_node, roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED);

						if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED && trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
						{
							// look for v1.0 attribute alongroute
							std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
							if (!along_route_str.empty())
							{
								if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
								{
									LOG("alongRoute attribute is depricated from v1.1. Reading it anyway.");
								}
								if ((along_route_str == "true") || (along_route_str == "1"))
								{
									trigger->cs_ = roadmanager::CoordinateSystem::CS_ROAD;
								}
							}
						}

						if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED)
						{
							// Set default value
							trigger->cs_ = roadmanager::CoordinateSystem::CS_ENTITY;
						}

						if (trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
						{
							// Set default value
							trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
						}

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "TraveledDistanceCondition")
					{
						TrigByTraveledDistance *trigger = new TrigByTraveledDistance;

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));

						condition = trigger;
					}
					else if (condition_type == "EndOfRoadCondition")
					{
						TrigByEndOfRoad *trigger = new TrigByEndOfRoad;

						trigger->duration_ = strtod(parameters.ReadAttribute(condition_node, "duration"));

						condition = trigger;
					}
					else if (condition_type == "OffroadCondition")
					{
						TrigByOffRoad *trigger = new TrigByOffRoad;

						trigger->duration_ = strtod(parameters.ReadAttribute(condition_node, "duration"));

						condition = trigger;
					}
					else if (condition_type == "StandStillCondition")
					{
						TrigByStandStill *trigger = new TrigByStandStill;

						trigger->duration_ = strtod(parameters.ReadAttribute(condition_node, "duration"));

						condition = trigger;
					}
					else if (condition_type == "AccelerationCondition")
					{
						TrigByAcceleration *trigger = new TrigByAcceleration;

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "SpeedCondition")
					{
						TrigBySpeed *trigger = new TrigBySpeed;

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "RelativeSpeedCondition")
					{
						TrigByRelativeSpeed *trigger = new TrigByRelativeSpeed;

						trigger->object_ = ResolveObjectReference(parameters.ReadAttribute(condition_node, "entityRef"));
						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else
					{
						LOG("Entity condition %s not supported", condition_type.c_str());
					}
				}
			}

			pugi::xml_node triggering_entities = conditionChild.child("TriggeringEntities");
			if (triggering_entities != NULL)
			{
				TrigByEntity *trigger = (TrigByEntity *)condition;

				std::string trig_ent_rule = parameters.ReadAttribute(triggering_entities, "triggeringEntitiesRule");
				if (trig_ent_rule == "any")
				{
					trigger->triggering_entity_rule_ = TrigByEntity::TriggeringEntitiesRule::ANY;
				}
				else if (trig_ent_rule == "all")
				{
					trigger->triggering_entity_rule_ = TrigByEntity::TriggeringEntitiesRule::ALL;
				}
				else
				{
					LOG("Invalid triggering entity type: %s", trig_ent_rule.c_str());
				}

				for (pugi::xml_node triggeringEntitiesChild = triggering_entities.first_child(); triggeringEntitiesChild; triggeringEntitiesChild = triggeringEntitiesChild.next_sibling())
				{
					std::string triggeringEntitiesChildName(triggeringEntitiesChild.name());

					if (triggeringEntitiesChildName == "EntityRef")
					{
						TrigByEntity::Entity entity;
						entity.object_ = ResolveObjectReference(parameters.ReadAttribute(triggeringEntitiesChild, "entityRef"));
						if (entity.object_ == 0)
						{
							throw std::runtime_error("Failed to find referenced entity - see log");
						}
						trigger->triggering_entities_.entity_.push_back(entity);
					}
				}
			}
			else
			{
				throw std::runtime_error("Missing TriggeringEntities");
			}
		}
		else if (conditionChildName == "ByValueCondition")
		{
			for (pugi::xml_node byValueChild = conditionChild.first_child(); byValueChild; byValueChild = byValueChild.next_sibling())
			{
				condition_type = byValueChild.name();
				if (condition_type == "SimulationTimeCondition")
				{
					TrigBySimulationTime *trigger = new TrigBySimulationTime;
					trigger->value_ = strtod(parameters.ReadAttribute(byValueChild, "value"));
					trigger->rule_ = ParseRule(parameters.ReadAttribute(byValueChild, "rule"));
					condition = trigger;
				}
				else if (condition_type == "ParameterCondition")
				{
					TrigByParameter *trigger = new TrigByParameter;
					trigger->name_ = parameters.ReadAttribute(byValueChild, "parameterRef");
					trigger->value_ = parameters.ReadAttribute(byValueChild, "value");
					trigger->rule_ = ParseRule(parameters.ReadAttribute(byValueChild, "rule"));
					trigger->parameters_ = &parameters;
					condition = trigger;
				}
				else if (condition_type == "StoryboardElementStateCondition")
				{
					StoryBoardElement::ElementType element_type = ParseElementType(parameters.ReadAttribute(byValueChild, "storyboardElementType"));
					TrigByState::CondElementState state = ParseState(parameters.ReadAttribute(byValueChild, "state"));
					std::string element_name = parameters.ReadAttribute(byValueChild, "storyboardElementRef");

					TrigByState *trigger = new TrigByState(state, element_type, element_name);

					condition = trigger;
				}
				else
				{
					LOG("TrigByValue %s not implemented", condition_type.c_str());
				}
			}
		}
		else
		{
			throw std::runtime_error("Unsupported condition: " + conditionChildName);
		}
	}

	if (condition == 0)
	{
		return 0;
	}
	condition->name_ = parameters.ReadAttribute(conditionNode, "name");

	if (condition->name_.empty())
	{
		// No name, set a dummy name
		condition->name_ = "no name " + condition_type;
	}

	if (conditionNode.attribute("delay") != NULL)
	{
		condition->delay_ = strtod(parameters.ReadAttribute(conditionNode, "delay"));
	}
	else
	{
		LOG("Attribute \"delay\" missing");
	}

	std::string edge_str = parameters.ReadAttribute(conditionNode, "conditionEdge");
	if (edge_str != "")
	{
		condition->edge_ = ParseConditionEdge(edge_str);
	}
	else
	{
		condition->edge_ = OSCCondition::ConditionEdge::NONE;
	}

	return condition;
}

Trigger *ScenarioReader::parseTrigger(pugi::xml_node triggerNode, bool defaultValue)
{
	Trigger *trigger = new Trigger(defaultValue);

	for (pugi::xml_node cgNode = triggerNode.first_child(); cgNode; cgNode = cgNode.next_sibling())
	{
		ConditionGroup *condition_group = new ConditionGroup;

		for (pugi::xml_node cNode = cgNode.first_child(); cNode; cNode = cNode.next_sibling())
		{
			OSCCondition *condition = parseOSCCondition(cNode);
			condition_group->condition_.push_back(condition);
		}

		trigger->conditionGroup_.push_back(condition_group);
	}

	return trigger;
}

void ScenarioReader::parseOSCManeuver(OSCManeuver *maneuver, pugi::xml_node maneuverNode, ManeuverGroup *mGroup)
{
	maneuver->name_ = parameters.ReadAttribute(maneuverNode, "name");

	for (pugi::xml_node maneuverChild = maneuverNode.first_child(); maneuverChild; maneuverChild = maneuverChild.next_sibling())
	{
		std::string maneuverChildName(maneuverChild.name());

		if (maneuverChildName == "ParameterDeclarations")
		{
			parameters.addParameterDeclarations(maneuverChild);
		}
		else if (maneuverChildName == "Event")
		{
			Event *event = new Event;

			event->name_ = parameters.ReadAttribute(maneuverChild, "name");

			std::string prio = parameters.ReadAttribute(maneuverChild, "priority");
			if (prio == "overwrite")
			{
				event->priority_ = Event::Priority::OVERWRITE;
			}
			else if (prio == "skip")
			{
				event->priority_ = Event::Priority::SKIP;
			}
			else if (prio == "parallel")
			{
				event->priority_ = Event::Priority::PARALLEL;
			}
			else
			{
				LOG("Invalid priority: %s", prio.c_str());
			}

			if (parameters.ReadAttribute(maneuverChild, "maximumExecutionCount") != "")
			{
				event->max_num_executions_ = strtoi(parameters.ReadAttribute(maneuverChild, "maximumExecutionCount"));
			}
			else
			{
				event->max_num_executions_ = 1; // 1 is default
			}

			for (pugi::xml_node eventChild = maneuverChild.first_child(); eventChild; eventChild = eventChild.next_sibling())
			{

				std::string childName(eventChild.name());

				if (childName == "Action")
				{
					for (pugi::xml_node actionChild = eventChild.first_child(); actionChild; actionChild = actionChild.next_sibling())
					{
						std::string actionChildName(actionChild.name());

						if (actionChildName == "GlobalAction")
						{
							OSCGlobalAction *action = parseOSCGlobalAction(actionChild);
							if (action != 0)
							{
								event->action_.push_back((OSCAction *)action);
							}
						}
						else if (actionChildName == "UserDefinedAction")
						{
							LOG("%s is not implemented", childName.c_str());
						}
						else if (actionChildName == "PrivateAction")
						{
							for (size_t i = 0; i < mGroup->actor_.size(); i++)
							{
								OSCPrivateAction *action = parseOSCPrivateAction(actionChild, mGroup->actor_[i]->object_);
								if (action != 0)
								{
									event->action_.push_back((OSCAction *)action);
								}
								else
								{
									LOG("Failed to parse Init action - continue regardless");
								}
							}
						}
					}
				}
				else if (childName == "StartTrigger")
				{
					event->start_trigger_ = parseTrigger(eventChild, true);

					// Check and warn for start trigger with rising edge in combination with simulationTime == 0
					for (size_t i = 0; i < event->start_trigger_->conditionGroup_.size(); i++)
					{
						for (size_t j = 0; j < event->start_trigger_->conditionGroup_[i]->condition_.size(); j++)
						{
							OSCCondition* cond = event->start_trigger_->conditionGroup_[i]->condition_[j];
							if (cond->base_type_ == OSCCondition::ConditionType::BY_VALUE)
							{
								TrigByValue* trig = (TrigByValue*)cond;
								if (trig->type_ == TrigByValue::Type::SIMULATION_TIME &&
									trig->edge_ != OSCCondition::NONE &&
									fabs(((TrigBySimulationTime*)(trig))->value_) < SMALL_NUMBER)
								{
									LOG("Warning: simulationTime = 0 condition used with edge \"%s\" which could be missed. Edge \"none\" is recommended.", trig->Edge2Str().c_str());
								}
							}
						}
					}
				}
				else
				{
					LOG("%s not supported", childName.c_str());
				}
			}
			if (event->start_trigger_ == 0)
			{
				// Add a default (empty) trigger
				event->start_trigger_ = new Trigger(true);
			}
			maneuver->event_.push_back(event);
		}
	}
}

ConditionGroup *ScenarioReader::ParseConditionGroup(pugi::xml_node node)
{
	ConditionGroup *cg = new ConditionGroup();

	for (pugi::xml_node conditionNode = node.first_child(); conditionNode; conditionNode = conditionNode.next_sibling())
	{
		OSCCondition *condition = parseOSCCondition(conditionNode);
		cg->condition_.push_back(condition);
	}

	return cg;
}

int ScenarioReader::parseStoryBoard(StoryBoard &storyBoard)
{
	pugi::xml_node storyNode = osc_root_.child("Storyboard").child("Story");

	for (; storyNode; storyNode = storyNode.next_sibling())
	{
		std::string storyNodeName(storyNode.name());

		if (storyNodeName == "Story")
		{
			std::string name = parameters.ReadAttribute(storyNode, "name", true);
			Story *story = new Story(name);

			parameters.CreateRestorePoint();

			if (!strcmp(storyNode.first_child().name(), "ParameterDeclarations"))
			{
				parameters.addParameterDeclarations(storyNode.first_child());
			}

			for (pugi::xml_node storyChild = storyNode.child("Act"); storyChild; storyChild = storyChild.next_sibling("Act"))
			{
				std::string childName(storyChild.name());

				if (childName == "Act")
				{
					Act *act = new Act;

					act->name_ = parameters.ReadAttribute(storyChild, "name");

					for (pugi::xml_node actChild = storyChild.first_child(); actChild; actChild = actChild.next_sibling())
					{

						std::string actChildName(actChild.name());

						if (actChildName == "ManeuverGroup")
						{
							ManeuverGroup *mGroup = new ManeuverGroup;

							if (parameters.ReadAttribute(actChild, "maximumExecutionCount") != "")
							{
								mGroup->max_num_executions_ = strtoi(parameters.ReadAttribute(actChild, "maximumExecutionCount"));
							}
							else
							{
								mGroup->max_num_executions_ = 1; // 1 is Default
							}

							mGroup->name_ = parameters.ReadAttribute(actChild, "name");

							pugi::xml_node actors_node = actChild.child("Actors");
							if (actors_node != NULL)
							{
								for (pugi::xml_node actorsChild = actors_node.first_child(); actorsChild; actorsChild = actorsChild.next_sibling())
								{
									ManeuverGroup::Actor *actor = new ManeuverGroup::Actor;

									std::string actorsChildName(actorsChild.name());
									if (actorsChildName == "EntityRef")
									{
										if ((actor->object_ = ResolveObjectReference(parameters.ReadAttribute(actorsChild, "entityRef"))) == 0)
										{
											throw std::runtime_error(std::string("Failed to resolve entityRef ") + parameters.ReadAttribute(actorsChild, "entityRef"));
										}
									}
									else if (actorsChildName == "ByCondition")
									{
										LOG("Actor by condition - not implemented");
									}
									mGroup->actor_.push_back(actor);
								}
							}

							for (pugi::xml_node catalog_n = actChild.child("CatalogReference"); catalog_n; catalog_n = catalog_n.next_sibling("CatalogReference"))
							{
								// Maneuver catalog reference. The catalog entry is simply the maneuver XML node
								parameters.CreateRestorePoint();
								Entry *entry = ResolveCatalogReference(catalog_n);

								if (entry == 0 || entry->node_ == 0)
								{
									throw std::runtime_error("Failed to resolve catalog reference");
								}

								if (entry->type_ == CatalogType::CATALOG_MANEUVER)
								{
									OSCManeuver *maneuver = new OSCManeuver;

									// Make a new instance from catalog entry
									parseOSCManeuver(maneuver, entry->GetNode(), mGroup);
									mGroup->maneuver_.push_back(maneuver);
								}
								else
								{
									throw std::runtime_error(std::string("Unexpected catalog type: ") + entry->GetTypeAsStr());
								}

								// Remove temporary parameters used for catalog reference
								parameters.RestoreParameterDeclarations();
							}

							for (pugi::xml_node maneuver_n = actChild.child("Maneuver"); maneuver_n; maneuver_n = maneuver_n.next_sibling("Maneuver"))
								if (maneuver_n != NULL)
								{
									OSCManeuver *maneuver = new OSCManeuver;

									parseOSCManeuver(maneuver, maneuver_n, mGroup);
									mGroup->maneuver_.push_back(maneuver);
								}

							act->maneuverGroup_.push_back(mGroup);
						}
						else if (actChildName == "StartTrigger")
						{
							act->start_trigger_ = parseTrigger(actChild, true);
						}
						else if (actChildName == "StopTrigger")
						{
							act->stop_trigger_ = parseTrigger(actChild, false);
						}
					}
					story->act_.push_back(act);
				}
			}
			storyBoard.story_.push_back(story);
			parameters.RestoreParameterDeclarations();
		}
		else if (storyNodeName == "StopTrigger")
		{
			storyBoard.stop_trigger_ = parseTrigger(storyNode, false);
		}
	}

	return 0;
}
