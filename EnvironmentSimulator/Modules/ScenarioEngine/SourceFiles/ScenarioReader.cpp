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
#include "ControllerSumo.hpp"
#include "ControllerExternal.hpp"

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
	RegisterController(ControllerSumo::GetTypeNameStatic(), InstantiateControllerSumo);
	RegisterController(ControllerExternal::GetTypeNameStatic(), InstantiateControllerExternal);
}

int ScenarioReader::loadOSCFile(const char * path)
{
	LOG("Loading %s", path);

	pugi::xml_parse_result result = doc_.load_file(path);
	if (!result)
	{
		LOG("%s at offset (character position): %d", result.description(), result.offset);
		return -1;
	}

	oscFilename_ = path;

	return 0;
}

void ScenarioReader::loadOSCMem(const pugi::xml_document &xml_doc)
{
	LOG("Loading XML document from memory");

	doc_.reset(xml_doc);
	oscFilename_ = "inline";
}

int ScenarioReader::RegisterCatalogDirectory(pugi::xml_node catalogDirChild)
{
	if (catalogDirChild.child("Directory") == NULL)
	{
		LOG("Catalog %s sub element Directory not found - skipping", catalogDirChild.name());
		return -1;
	}

	std::string dirname = parameters.ReadAttribute(catalogDirChild.child("Directory"), "path", true);

	if (dirname == "")
	{
		LOG("Catalog %s missing filename - ignoring", catalogDirChild.name());
		return -1;
	}

	std::string catalog_dir = dirname;

	catalogs_->RegisterCatalogDirectory(catalogDirChild.name(), catalog_dir);

	return 0;
}

Catalog* ScenarioReader::LoadCatalog(std::string name)
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
	size_t i;
	for (i = 0; i < catalogs_->catalog_dirs_.size(); i++)
	{
		// First assume absolute path or relative current directory
		std::string file_path = catalogs_->catalog_dirs_[i].dir_name_ + "/" + name + ".xosc";

		// Load it
		pugi::xml_parse_result result;

		if (!FileExists(file_path.c_str()) || !(result = catalog_doc.load_file(file_path.c_str())))
		{
			// Then assume relative path to scenario directory - which perhaps should be the expected location
			std::string file_path = CombineDirectoryPathAndFilepath(DirNameOf(oscFilename_), catalogs_->catalog_dirs_[i].dir_name_) + "/" + name + ".xosc";

			// Load it
			result = catalog_doc.load_file(file_path.c_str());
		}

		if (result)
		{
			break;
		}
	}

	if (i == catalogs_->catalog_dirs_.size())
	{
		LOG("Couldn't locate catalog file %s make sure it is located in one of the catalog directories listed in the scenario file", name.c_str());
		return 0;
	}

	LOG("Loading catalog %s", name.c_str());
	pugi::xml_node catalog_node = catalog_doc.child("OpenSCENARIO").child("Catalog");

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
	LOG("Parsing RoadNetwork");

	pugi::xml_node roadNetworkNode = doc_.child("OpenSCENARIO").child("RoadNetwork");

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

	if (roadNetwork.logicFile.filepath == "")
	{
		LOG("Warning: No road network ODR file loaded!");
	}

	if (roadNetwork.sceneGraphFile.filepath == "")
	{
		LOG("Warning: No road network 3D model file loaded! Setting default path.");

		// Since the scene graph file path is used to locate other 3D files, like vehicles, create a default path
		roadNetwork.sceneGraphFile.filepath = DirNameOf(oscFilename_);
	}

	LOG("Roadnetwork ODR: %s", roadNetwork.logicFile.filepath.c_str());
	LOG("Scenegraph: %s", roadNetwork.sceneGraphFile.filepath.c_str());
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
				if (properties.file_.filepath_ != "")
				{
					LOG("Properties/File = %s registered", properties.file_.filepath_.c_str());
				}
			}
			else if (!strcmp(propertiesChild.name(), "Property"))
			{
				OSCProperties::Property property;
				property.name_ = parameters.ReadAttribute(propertiesChild, "name");
				property.value_ = parameters.ReadAttribute(propertiesChild, "value");
				properties.property_.push_back(property);
				LOG("Property %s = %s registered", property.name_.c_str(), property.value_.c_str());
			}
			else
			{
				LOG("Unexpected property element: %s", propertiesChild.name());
			}
		}
	}
}

Vehicle* ScenarioReader::createRandomOSCVehicle(std::string name)
{
	Vehicle *vehicle = new Vehicle();

	vehicle->name_ = name;
	vehicle->category_ = Vehicle::Category::CAR;
	vehicle->model_id_ = -1;
	vehicle->model_filepath_ = "";

	// Set some default bounding box just to avoid division-by-zero-problems
	vehicle->boundingbox_ = OSCBoundingBox();  
	vehicle->boundingbox_.dimensions_.length_ = 4.0f;
	vehicle->boundingbox_.dimensions_.width_ = 2.0f;
	vehicle->boundingbox_.dimensions_.height_ = 1.2f;

	return vehicle;
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
			} else if (boundingboxChildName == "Dimensions")
			{
				boundingbox.dimensions_.width_ = std::stof(parameters.ReadAttribute(boundingboxChild, "width"));
				boundingbox.dimensions_.length_ = std::stof(parameters.ReadAttribute(boundingboxChild, "length"));
				boundingbox.dimensions_.height_ = std::stof(parameters.ReadAttribute(boundingboxChild, "height"));
			} else {
				LOG("Not valid boudingbox attribute name:%s",boundingboxChildName.c_str());
			}
		}
		//LOG("Parsing boundingbox for vehicle:%s,center_x:%f, dimensions_width: %f.",ReadAttribute(xml_node, "name").c_str(),boundingbox.center_.x_,boundingbox.dimensions_.width_);
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

Vehicle* ScenarioReader::parseOSCVehicle(pugi::xml_node vehicleNode)
{
	Vehicle *vehicle = new Vehicle();

	if (vehicleNode == 0)
	{
		return 0;
	}
	vehicle->name_ = parameters.ReadAttribute(vehicleNode, "name");
	LOG("Parsing Vehicle %s", vehicle->name_.c_str());
	vehicle->SetCategory(parameters.ReadAttribute(vehicleNode, "vehicleCategory"));

	OSCProperties properties;
	ParseOSCProperties(properties, vehicleNode);

	for(size_t i=0; i<properties.property_.size(); i++)
	{
		if (properties.property_[i].name_ == "model_id")
		{
			vehicle->model_id_ = strtoi(properties.property_[i].value_);
		}
		else
		{
			LOG("Unsupported property: %s", properties.property_[i].name_.c_str());
		}
	}

	if (properties.file_.filepath_ != "")
	{
		vehicle->model_filepath_ = properties.file_.filepath_;
	}

	OSCBoundingBox boundingbox;
	ParseOSCBoundingBox(boundingbox, vehicleNode);
	vehicle->boundingbox_=boundingbox;

	return vehicle;
}

Pedestrian* ScenarioReader::parseOSCPedestrian(pugi::xml_node pedestrianNode)
{
	Pedestrian* pedestrian = new Pedestrian();

	if (pedestrianNode == 0)
	{
		return 0;
	}

	pedestrian->name_ = parameters.ReadAttribute(pedestrianNode, "name");
	LOG("Parsing Pedestrian %s", pedestrian->name_.c_str());
	pedestrian->SetCategory(parameters.ReadAttribute(pedestrianNode, "pedestrianCategory"));
	pedestrian->model_ = parameters.ReadAttribute(pedestrianNode, "pedestrianCategory");
	pedestrian->mass_ = strtod(parameters.ReadAttribute(pedestrianNode, "mass"));

	// Parse BoundingBox
	OSCBoundingBox boundingbox;
	ParseOSCBoundingBox(boundingbox,pedestrianNode);
	pedestrian->boundingbox_=boundingbox;

	// Parse Properties
	OSCProperties properties;
	ParseOSCProperties(properties, pedestrianNode);

	for(size_t i=0; i<properties.property_.size(); i++)
	{
		if (properties.property_[i].name_ == "model_id")
		{
			pedestrian->model_id_ = strtoi(properties.property_[i].value_);
		}
		else
		{
			LOG("Unsupported property: %s", properties.property_[i].name_.c_str());
		}
	}

	if (properties.file_.filepath_ != "")
	{
		pedestrian->model_filepath_ = properties.file_.filepath_;
	}

	return pedestrian;
}

MiscObject* ScenarioReader::parseOSCMiscObject(pugi::xml_node miscObjectNode)
{
	MiscObject* miscObject = new MiscObject();

	if (miscObjectNode == 0)
	{
		return 0;
	}

	miscObject->name_ = parameters.ReadAttribute(miscObjectNode, "name");
	LOG("Parsing MiscObject %s", miscObject->name_.c_str());
	miscObject->SetCategory(parameters.ReadAttribute(miscObjectNode, "MiscObjectCategory"));
	miscObject->model_ = parameters.ReadAttribute(miscObjectNode, "MiscObjectCategory");
	miscObject->mass_ = strtod(parameters.ReadAttribute(miscObjectNode, "mass"));

	// Parse BoundingBox
	OSCBoundingBox boundingbox;
	ParseOSCBoundingBox(boundingbox,miscObjectNode);
	miscObject->boundingbox_=boundingbox;

	// Parse Properties
	OSCProperties properties;
	ParseOSCProperties(properties, miscObjectNode);

	for(size_t i=0; i<properties.property_.size(); i++)
	{
		if (properties.property_[i].name_ == "model_id")
		{
			miscObject->model_id_ = strtoi(properties.property_[i].value_);
		}
		else
		{
			LOG("Unsupported property: %s", properties.property_[i].name_.c_str());
		}
	}

	if (properties.file_.filepath_ != "")
	{
		miscObject->model_filepath_ = properties.file_.filepath_;
	}

	return miscObject;
}

Controller* ScenarioReader::parseOSCObjectController(pugi::xml_node controllerNode)
{
	std::string name = parameters.ReadAttribute(controllerNode, "name");
	Controller* controller = 0;
	OSCProperties properties;

	// First check for parameter declaration
	pugi::xml_node paramDecl = controllerNode.child("ParameterDeclarations");
	if (paramDecl)
	{
		parameters.addParameterDeclarations(paramDecl);
	}

	// Then read any properties
	ParseOSCProperties(properties, controllerNode);
	std::string filename = properties.file_.filepath_;

	if (controllerNode == 0)
	{
		LOG("Warning: Empty controller node");
	}

	LOG("Parsing Controller %s", name.c_str());

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
		LOG("esminiController property missing!");
	}

	if (ctrlType == "DefaultController")
	{
		// Fall back to esmini default operation - no controller involved
		controller = 0;
	}
	else
	{
		ControllerPool::ControllerEntry* ctrl_entry = ScenarioReader::controllerPool_.GetControllerByType(ctrlType);
		if (ctrl_entry)
		{
			Controller::InitArgs args;
			args.name = name;
			args.type = ctrlType;
			args.entities = entities_;
			args.gateway = gateway_;
			args.parameters = &parameters;
			args.properties = &properties;
			controller = (Controller*)ctrl_entry->instantiateFunction(&args);
		}
		else
		{
			LOG("Unsupported controller type: %s", ctrlType.c_str());
		}
	}

	if (paramDecl)
	{
		parameters.RestoreParameterDeclarations();
	}

	return controller;
}

roadmanager::Route* ScenarioReader::parseOSCRoute(pugi::xml_node routeNode)
{
	roadmanager::Route *route = new roadmanager::Route;

	route->setName(parameters.ReadAttribute(routeNode, "name"));

	LOG("Parsing OSCRoute %s", route->getName().c_str());

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

	return route;
}

void ScenarioReader::parseCatalogs()
{
	LOG("Parsing Catalogs");

	pugi::xml_node catalogsNode = doc_.child("OpenSCENARIO").child("CatalogLocations");

	for (pugi::xml_node catalogsChild = catalogsNode.first_child(); catalogsChild; catalogsChild = catalogsChild.next_sibling())
	{
		RegisterCatalogDirectory(catalogsChild);
	}
}

void ScenarioReader::parseOSCFile(OSCFile &file, pugi::xml_node fileNode)
{
	file.filepath = parameters.ReadAttribute(fileNode, "filepath");
}

roadmanager::Trajectory* ScenarioReader::parseTrajectory(pugi::xml_node node)
{
	roadmanager::Trajectory* traj = new roadmanager::Trajectory;
	roadmanager::Shape* shape = 0;
	bool params = false;

	traj->name_ = parameters.ReadAttribute(node, "name");
	traj->closed_ = parameters.ReadAttribute(node, "closed") == "true" ? true : false;

	LOG("Parsing Trajectory %s", traj->name_.c_str());

	for (pugi::xml_node childNode = node.first_child(); childNode; childNode = childNode.next_sibling())
	{
		std::string childNodeName(childNode.name());

		if (childNodeName == "ParameterDeclarations")
		{
			params = true;
			parameters.addParameterDeclarations(childNode);
		}
		else if (childNodeName == "Shape")
		{
			LOG("Parsing trajectory shape");

			pugi::xml_node shapeNode = childNode.first_child();
			if (!shapeNode)
			{
				throw std::runtime_error("Missing Trajectory Shape");
			}

			std::string shapeType = shapeNode.name();
			if (shapeType == "Polyline")
			{
				LOG("Parsing Polyline");
				roadmanager::PolyLine* pline = new roadmanager::PolyLine();
				for (pugi::xml_node vertexNode = shapeNode.first_child(); vertexNode; vertexNode = vertexNode.next_sibling())
				{
					pugi::xml_node posNode = vertexNode.child("Position");
					if (!posNode)
					{
						throw std::runtime_error("Missing Trajectory/Polyline/Vertex/Position node");
					}
					OSCPosition* pos = parseOSCPosition(posNode);
					double time = strtod(parameters.ReadAttribute(vertexNode, "time"));
					pline->AddVertex(*pos->GetRMPos(), time);
				}
				shape = pline;
			}
			else if (shapeType == "Clothoid")
			{
				LOG("Parsing Clothoid");
				pugi::xml_node posNode = shapeNode.child("Position");
				OSCPosition* pos = parseOSCPosition(posNode);
				
				double curvature = strtod(parameters.ReadAttribute(shapeNode, "curvature"));
				double curvatureDot = strtod(parameters.ReadAttribute(shapeNode, "curvatureDot"));
				double length = strtod(parameters.ReadAttribute(shapeNode, "length"));
				double startTime = strtod(parameters.ReadAttribute(shapeNode, "startTime"));
				double stopTime = strtod(parameters.ReadAttribute(shapeNode, "stopTime"));

				LOG("Adding clothoid(x=%.2f y=%.2f h=%.2f curv=%.2f curvDot=%.2f len=%.2f startTime=%.2f stopTime=%.2f",
					pos->GetRMPos()->GetX(), pos->GetRMPos()->GetY(), pos->GetRMPos()->GetH(), curvature, curvatureDot, length, startTime, stopTime);

				roadmanager::Clothoid* clothoid = new roadmanager::Clothoid(*pos->GetRMPos(), curvature, curvatureDot, length, startTime, stopTime);
				clothoid->spiral_->Print();
				
				shape = clothoid;
			}
			else if (shapeType == "Nurbs")
			{
				LOG("Trajectory type Nurbs not supported yet");
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

	if (params) parameters.RestoreParameterDeclarations();

	return traj;
}

Entry* ScenarioReader::ResolveCatalogReference(pugi::xml_node node)
{
	std::string catalog_name;
	std::string entry_name;

	pugi::xml_node parameterAssignmentsNode = node.child("ParameterAssignments");

	// Read any parameter assignments
	for (pugi::xml_node param_n = parameterAssignmentsNode.child("ParameterAssignment"); param_n; param_n = param_n.next_sibling("ParameterAssignment"))
	{
		ParameterStruct param;
		param.name = &(param_n.attribute("parameterRef").value()[1]);  // Skip prefix character byte
		param.value = parameters.ReadAttribute(param_n, "value");
		parameters.catalog_param_assignments.push_back(param);
	}

	catalog_name = parameters.ReadAttribute(node, "catalogName");
	entry_name = parameters.ReadAttribute(node, "entryName");

	if (parameters.catalog_param_assignments.size() > 0)
	{
		LOG("Assigned %d parameters for catalog %s entry %s", parameters.catalog_param_assignments.size(), catalog_name.c_str(), entry_name.c_str());
	}

	Catalog *catalog;

	// Make sure the catalog item is loaded
	if ((catalog = LoadCatalog(catalog_name)) == 0)
	{
		LOG("Failed to load catalog %s",  catalog_name.c_str());
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
	LOG("Parsing Entities");

	pugi::xml_node enitiesNode = doc_.child("OpenSCENARIO").child("Entities");

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
							Pedestrian* pedestrian = parseOSCPedestrian(entry->GetNode());
							obj = pedestrian;
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
								LOG("Parsing controller from reference: %s", entry->GetNode().name());
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
						LOG("Parsing controller: %s", objectSubChild.name());
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
				if (ctrl->GetType() == Controller::Type::CONTROLLER_TYPE_SUMO)
				{
					// Set template vehicle to be used for all vehicles spawned from SUMO
					((ControllerSumo*)ctrl)->SetSumoVehicle(obj);
					obj->id_ = -1;

					// SUMO controller is special in the sense that it is always active
					ctrl->Activate(Controller::Domain::CTRL_BOTH);

					// SUMO controller is not assigned to any scenario vehicle
				}
				else if (ctrl->GetType() == Controller::Type::CONTROLLER_TYPE_FOLLOW_GHOST)
				{
					// Ghost controllers are assigned and activated from start, to get correct headstart 
					ctrl->Assign(obj);
				}
				controller_.push_back(ctrl);
			}

		}
		else if (entitiesChildName == "EntitySelection")
		{
			LOG("Parsing %s: is not implemented yet", entitiesChildName.c_str());
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

		double x = strtod(parameters.ReadAttribute(positionChild, "x", true));
		double y = strtod(parameters.ReadAttribute(positionChild, "y", true));
		double z = strtod(parameters.ReadAttribute(positionChild, "z", false));
		double h = strtod(parameters.ReadAttribute(positionChild, "h", false));
		double p = strtod(parameters.ReadAttribute(positionChild, "p", false));
		double r = strtod(parameters.ReadAttribute(positionChild, "r", false));

		OSCPositionWorld *pos = new OSCPositionWorld(x, y, z, h, p, r);

		pos_return = (OSCPosition*)pos;
	}
	else if (positionChildName == "RelativeWorldPosition")
	{
		double dx, dy, dz;

		dx = strtod(parameters.ReadAttribute(positionChild, "dx"));
		dy = strtod(parameters.ReadAttribute(positionChild, "dy"));
		dz = strtod(parameters.ReadAttribute(positionChild, "dz"));
		Object* object = entities_->GetObjectByName(parameters.ReadAttribute(positionChild, "entityRef"));

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}

		OSCPositionRelativeWorld* pos = new OSCPositionRelativeWorld(object, dx, dy, dz, orientation);

		pos_return = (OSCPosition*)pos;
	}
	else if (positionChildName == "RelativeObjectPosition")
	{
		double dx, dy, dz;

		dx = strtod(parameters.ReadAttribute(positionChild, "dx"));
		dy = strtod(parameters.ReadAttribute(positionChild, "dy"));
		dz = strtod(parameters.ReadAttribute(positionChild, "dz"));
		Object *object = entities_->GetObjectByName(parameters.ReadAttribute(positionChild, "entityRef"));

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}

		OSCPositionRelativeObject *pos = new OSCPositionRelativeObject(object, dx, dy, dz, orientation);

		pos_return = (OSCPosition*)pos;
	}
	else if (positionChildName == "RelativeLanePosition")
	{
		int dLane;
		double ds, offset;

		dLane = strtoi(parameters.ReadAttribute(positionChild, "dLane"));
		ds = strtod(parameters.ReadAttribute(positionChild, "ds"));
		offset = strtod(parameters.ReadAttribute(positionChild, "offset"));
		Object *object = entities_->GetObjectByName(parameters.ReadAttribute(positionChild, "entityRef"));

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

		pos_return = (OSCPosition*)pos;
	}
	else if (positionChildName == "RoadPosition")
	{
		LOG("%s is not implemented ", positionChildName.c_str());
	}
	else if (positionChildName == "RelativeRoadPosition")
	{
		LOG("%s is not implemented ", positionChildName.c_str());
	}
	else if (positionChildName == "LanePosition")
	{
		int road_id = strtoi(parameters.ReadAttribute(positionChild, "roadId"));
		int lane_id = strtoi(parameters.ReadAttribute(positionChild, "laneId"));
		double s = strtod(parameters.ReadAttribute(positionChild, "s"));

		double offset = 0;  // Default value of optional parameter
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

		pos_return = (OSCPosition*)pos;
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
						Entry *entry = ResolveCatalogReference(routeRefChild);

						if (entry == 0)
						{
							return 0;
						}

						if (entry->type_ == CatalogType::CATALOG_ROUTE)
						{
							// Make a new instance from catalog entry
							route = parseOSCRoute(entry->GetNode());
						}
						else
						{
							LOG("Catalog entry of type %s expected - found %s", Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE).c_str(), entry->GetTypeAsStr().c_str());
							return 0;
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
				for (pugi::xml_node positionChild = routeChild.first_child(); positionChild; positionChild = positionChild.next_sibling())
				{
					std::string positionChildName(positionChild.name());

					if (positionChildName == "FromCurrentEntity")
					{
						LOG("%s is not implemented", positionChildName.c_str());
					}
					else if (positionChildName == "FromRoadCoordinates")
					{
						LOG("%s is not implemented", positionChildName.c_str());
					}
					else if (positionChildName == "FromLaneCoordinates")
					{
						double s = strtod(parameters.ReadAttribute(positionChild, "pathS"));
						int lane_id = strtoi(parameters.ReadAttribute(positionChild, "laneId"));
						double lane_offset = 0;

						pugi::xml_attribute laneOffsetAttribute = positionChild.attribute("laneOffset");
						if (laneOffsetAttribute != NULL)
						{
							lane_offset = strtod(parameters.ReadAttribute(positionChild, "laneOffset"));
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

		pos_return = (OSCPosition*)pos;
	}

	if (pos_return == 0)
	{
		LOG("Failed parse position (node %s)", positionNode.name());
		throw std::runtime_error("Failed parse position");
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
	else
	{
		LOG("Dynamics shape %s not implemented", shape.c_str());
	}

	return OSCPrivateAction::DynamicsShape::SHAPE_UNDEFINED;
}

OSCPrivateAction::DynamicsDimension ParseDynamicsDimension(std::string dimension)
{
	if (dimension == "rate")
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
		LOG("Dynamics dimension %s not supported", dimension.c_str());
	}

	return OSCPrivateAction::DynamicsDimension::DIMENSION_UNDEFINED;
}

int ScenarioReader::ParseTransitionDynamics(pugi::xml_node node, OSCPrivateAction::TransitionDynamics& td)
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
		LOG("Unsupported global action: %s", actionChild.name());
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

									target_rel->continuous_ = (
										parameters.ReadAttribute(targetChild, "continuous") == "true" ||
										parameters.ReadAttribute(targetChild, "continuous") == "1");

									target_rel->object_ = entities_->GetObjectByName(parameters.ReadAttribute(targetChild, "entityRef"));

									std::string value_type = parameters.ReadAttribute(targetChild, "speedTargetValueType");
									if (value_type == "delta")
									{
										target_rel->value_type_ = LongSpeedAction::TargetRelative::ValueType::DELTA;
									}
									else if(value_type == "factor")
									{
										target_rel->value_type_ = LongSpeedAction::TargetRelative::ValueType::FACTOR;
									}
									else if(value_type == "")
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

						action_dist->dynamics_.max_speed_ = strtod(parameters.ReadAttribute(dynamics_node, "maxSpeed"));
                        action_dist->dynamics_.none_ = false;
					}
					else
					{
						action_dist->dynamics_.none_ = true;
					}

					action_dist->target_object_ = entities_->GetObjectByName(parameters.ReadAttribute(longitudinalChild, "entityRef"));
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

					if (longitudinalChild.attribute("continuous"))
					{
						LOG("continuous flag assumed and always on by default");
					}

					std::string freespace = parameters.ReadAttribute(longitudinalChild, "freespace");
					if (freespace == "true" || freespace == "1") action_dist->freespace_ = true;
					else action_dist->freespace_ = false;

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
							LatLaneChangeAction::Target *target;

							for (pugi::xml_node targetChild = laneChangeChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("RelativeTargetLane"))
								{
									LatLaneChangeAction::TargetRelative *target_rel = new LatLaneChangeAction::TargetRelative;

									if ((target_rel->object_ = entities_->GetObjectByName(parameters.ReadAttribute(targetChild, "entityRef"))) == 0)
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
							}
							action_lane->target_ = target;
						}
					}
					action = action_lane;
				}
				else if(lateralChild.name() == std::string("LaneOffsetAction"))
				{
					LatLaneOffsetAction *action_lane = new LatLaneOffsetAction();
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

							action_lane->dynamics_.transition_.shape_ = ParseDynamicsShape(parameters.ReadAttribute(laneOffsetChild, "dynamicsShape"));
						}
						else if (laneOffsetChild.name() == std::string("LaneOffsetTarget"))
						{
							LatLaneOffsetAction::Target *target;

							for (pugi::xml_node targetChild = laneOffsetChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("RelativeTargetLaneOffset"))
								{
									LatLaneOffsetAction::TargetRelative *target_rel = new LatLaneOffsetAction::TargetRelative;

									target_rel->object_ = entities_->GetObjectByName(parameters.ReadAttribute(targetChild, "entityRef"));
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
					// Approximate duration (not conforming to OSC 1.0), assume one lane of 5 meter
					action_lane->dynamics_.duration_ = sqrt(5.0 / action_lane->dynamics_.max_lateral_acc_);
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
			action_synch->master_object_ = entities_->GetObjectByName(master_object_str);

			pugi::xml_node target_position_master_node = actionChild.child("TargetPositionMaster");
			if (!target_position_master_node)
			{
				LOG("Missing required element \"TargetPositionMaster\"");
				return 0;
			}
			action_synch->target_position_master_ = parseOSCPosition(target_position_master_node)->GetRMPos();

			pugi::xml_node target_position_node = actionChild.child("TargetPosition");
			if (!target_position_node)
			{
				LOG("Missing required element \"TargetPosition\"");
				return 0;
			}
			action_synch->target_position_ = parseOSCPosition(target_position_node)->GetRMPos();

			pugi::xml_node target_speed_node = actionChild.child("FinalSpeed");
			if (target_speed_node)
			{
				LOG("Parsing optional element \"FinalSpeed\"");

				pugi::xml_node target_speed_element = target_speed_node.child("AbsoluteSpeed");
				if (target_speed_element != NULL)
				{
					LongSpeedAction::TargetAbsolute *targetSpeedAbs = new LongSpeedAction::TargetAbsolute;
					targetSpeedAbs->value_ = strtod(parameters.ReadAttribute(target_speed_element, "value"));
					action_synch->final_speed_ = targetSpeedAbs;
				}
				else
				{
					pugi::xml_node target_speed_element = target_speed_node.child("RelativeSpeedToMaster");

					if (target_speed_element != NULL)
					{
						LongSpeedAction::TargetRelative *targetSpeedRel = new LongSpeedAction::TargetRelative;

						targetSpeedRel->value_ = strtod(parameters.ReadAttribute(target_speed_element, "value"));

						targetSpeedRel->continuous_ = true;  // Continuous adaption needed

						targetSpeedRel->object_ = action_synch->master_object_;  // Master object is the pivot vehicle

						std::string value_type = parameters.ReadAttribute(target_speed_element, "speedTargetValueType");
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
						LOG("Missing speed TargetSpeed sub element \"Absolute\" or \"RelativeMaster\"");
						return 0;
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
							AssignRouteAction* action_follow_route = new AssignRouteAction;
							action_follow_route->route_ = parseOSCRoute(assignRouteChild);
							action = action_follow_route;
						}
						else if (assignRouteChild.name() == std::string("CatalogReference"))
						{
							AssignRouteAction* action_assign_route = new AssignRouteAction;

							// Find route in catalog
							Entry *entry = ResolveCatalogReference(assignRouteChild);

							if (entry == 0 || entry->node_ == 0)
							{
								return 0;
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
								LOG("Catalog entry of type %s expected - found %s", Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE).c_str(), entry->GetTypeAsStr().c_str());
								return 0;
							}

							parameters.RestoreParameterDeclarations();
						}
					}
				}
				else if (routingChild.name() == std::string("FollowTrajectoryAction"))
				{
					FollowTrajectoryAction* action_follow_trajectory = new FollowTrajectoryAction;

					for (pugi::xml_node followTrajetoryChild = routingChild.first_child(); followTrajetoryChild; followTrajetoryChild = followTrajetoryChild.next_sibling())
					{
						if (followTrajetoryChild.name() == std::string("Trajectory"))
						{
							action_follow_trajectory->traj_ = parseTrajectory(routingChild.child("Trajectory"));
						}
						else if (followTrajetoryChild.name() == std::string("CatalogReference"))
						{
							// Find trajectory in catalog
							Entry* entry = ResolveCatalogReference(followTrajetoryChild);

							if (entry == 0 || entry->node_ == 0)
							{
								return 0;
							}

							if (entry->type_ == CatalogType::CATALOG_TRAJECTORY)
							{
								// Make a new instance from catalog entry
								action_follow_trajectory->traj_ = parseTrajectory(entry->GetNode());
								break;
							}
							else
							{
								LOG("Catalog entry of type %s expected - found %s", Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE).c_str(), entry->GetTypeAsStr().c_str());
								return 0;
							}

							parameters.RestoreParameterDeclarations();
						}
						else if (followTrajetoryChild.name() == std::string("TimeReference"))
						{
							pugi::xml_node timingNode = followTrajetoryChild.first_child();
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
								LOG("Unexpected TimeReference element: %s", timingNode.name());
							}

						}
						else if (followTrajetoryChild.name() == std::string("TrajectoryFollowingMode"))
						{
							LOG("trajectoryFollowingMode not implemented yet - applying \"position\"");
						}
						else
						{
							throw std::runtime_error("Unexpected element: " + std::string(followTrajetoryChild.name()));
						}

					}
					action = action_follow_trajectory;
				}
				else
				{
					throw std::runtime_error("Action is not supported: " + std::string(routingChild.name()));
				}
			}
		}
		else if (actionChild.name() == std::string("ActivateControllerAction"))
		{
			bool domain_longitudinal = parameters.ReadAttribute(actionChild, "longitudinal") == "true";
			bool domain_lateral = parameters.ReadAttribute(actionChild, "lateral") == "true";

			int domainMask = 0;
			if (domain_longitudinal)
			{
				domainMask |= Controller::Domain::CTRL_LONGITUDINAL;
			}
			if (domain_lateral)
			{
				domainMask |= Controller::Domain::CTRL_LATERAL;
			}

			ActivateControllerAction * activateControllerAction = new ActivateControllerAction(domainMask);

			LOG("ActivateControllerAction: Longitudinal: %s Lateral: %s",
				domain_longitudinal ? "true" : "false",
				domain_lateral ? "true" : "false"
			);

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
						Controller* controller = 0;
						if (controllerDefNode.name() == std::string("Controller"))
						{
							controller = parseOSCObjectController(controllerDefNode);
						}
						else if (controllerDefNode.name() == std::string("CatalogReference"))
						{
							Entry* entry = ResolveCatalogReference(controllerDefNode);

							if (entry == 0)
							{
								LOG("No entry found");
							}
							else
							{
								if (entry->type_ == CatalogType::CATALOG_CONTROLLER)
								{
									LOG("Parsing controller from reference: %s", entry->GetNode().name());
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
						AssignControllerAction* assignControllerAction = new AssignControllerAction(controller);
						action = assignControllerAction;
					}
				}
				else if (controllerChild.name() == std::string("OverrideControllerValueAction"))
				{
					LOG("OverrideControllerValueAction not supported yet");
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

			VisibilityAction* visAction = new VisibilityAction();
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
	LOG("Parsing init");

	pugi::xml_node actionsNode = doc_.child("OpenSCENARIO").child("Storyboard").child("Init").child("Actions");

	for (pugi::xml_node actionsChild = actionsNode.first_child(); actionsChild; actionsChild = actionsChild.next_sibling())
	{
		std::string actionsChildName(actionsChild.name());

		if (actionsChildName == "GlobalAction")
		{
			LOG("Init %s is not implemented", actionsChildName.c_str());
		}
		else if (actionsChildName == "UserDefined")
		{
			LOG("Init %s is not implemented", actionsChildName.c_str());
		}
		else if (actionsChildName == "Private")
		{
			Object *entityRef;

			entityRef = entities_->GetObjectByName(parameters.ReadAttribute(actionsChild, "entityRef"));
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
			roadmanager::Position* pos = 0;
			if (init.private_action_[j]->type_ == OSCPrivateAction::ActionType::TELEPORT)
			{
				TeleportAction* action = (TeleportAction*)init.private_action_[j];
				if (action->position_->GetType() == roadmanager::Position::PositionType::RELATIVE_LANE)
				{
					pos = ((roadmanager::Position*)action->position_)->GetRelativePosition();
				}
				else if (action->position_->GetType() == roadmanager::Position::PositionType::RELATIVE_OBJECT)
				{
					pos = ((roadmanager::Position*)action->position_)->GetRelativePosition();
				}
			}
			if (pos == &init.private_action_[i]->object_->pos_)
			{
				// swap places
				OSCPrivateAction* tmp_action = init.private_action_[i];
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
	else if(edge == "falling")
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
	else if (rule == "lessThan")
	{
		return Rule::LESS_THAN;
	}
	else if (rule == "equalTo")
	{
		return Rule::EQUAL_TO;
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
	LOG("Parsing OSCCondition %s", parameters.ReadAttribute(conditionNode, "name").c_str());

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
					std::string condition_type(condition_node.name());
					if (condition_type == "TimeHeadwayCondition")
					{
						TrigByTimeHeadway *trigger = new TrigByTimeHeadway;
						trigger->object_ = entities_->GetObjectByName(parameters.ReadAttribute(condition_node, "entityRef"));

						std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
						if ((along_route_str == "true") || (along_route_str == "1"))
						{
							trigger->along_route_ = true;
						}
						else
						{
							trigger->along_route_ = false;
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
						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "TimeToCollisionCondition")
					{
						TrigByTimeToCollision* trigger = new TrigByTimeToCollision;

						pugi::xml_node target = condition_node.child("TimeToCollisionConditionTarget");
						pugi::xml_node targetChild = target.first_child();
						std::string targetChildName(targetChild.name());
						if (targetChildName == "Position")
						{
							trigger->position_ = parseOSCPosition(targetChild);
						}
						else if (targetChildName == "EntityRef")
						{
							trigger->object_ = entities_->GetObjectByName(parameters.ReadAttribute(targetChild, "entityRef"));
						}
						else
						{
							LOG("Unexpected target type: %s", targetChildName.c_str());
							return 0;
						}
						
						std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
						if ((along_route_str == "true") || (along_route_str == "1"))
						{
							trigger->along_route_ = true;
						}
						else
						{
							trigger->along_route_ = false;
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
						trigger->object_ = entities_->GetObjectByName(parameters.ReadAttribute(condition_node, "entityRef"));

						std::string type = parameters.ReadAttribute(condition_node, "relativeDistanceType");
						if ((type == "longitudinal") || (type == "Longitudinal"))
						{
							trigger->type_ = TrigByRelativeDistance::RelativeDistanceType::LONGITUDINAL;
						}
						else if ((type == "lateral") || (type == "Lateral"))
						{
							trigger->type_ = TrigByRelativeDistance::RelativeDistanceType::LATERAL;
						}
						else if ((type == "cartesianDistance") || (type == "CartesianDistance"))
						{
							trigger->type_ = TrigByRelativeDistance::RelativeDistanceType::INTERIAL;
						}
						else
						{
							LOG("Unknown RelativeDistance condition type: %s", type.c_str());
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
						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "CollisionCondition")
					{
						TrigByCollision* trigger = new TrigByCollision;

						pugi::xml_node target = condition_node.child("EntityRef");

						trigger->object_ = entities_->GetObjectByName(parameters.ReadAttribute(target, "entityRef"));

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

						std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
						if ((along_route_str == "true") || (along_route_str == "1"))
						{
							LOG("Condition Distance along route not supported yet - falling back to alongeRoute = false");
							trigger->along_route_ = false;
						}
						else
						{
							trigger->along_route_ = false;
						}

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "TraveledDistanceCondition")
					{
						TrigByTraveledDistance* trigger = new TrigByTraveledDistance;

						trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));

						condition = trigger;
					}
					else if (condition_type == "EndOfRoadCondition")
					{
						TrigByEndOfRoad* trigger = new TrigByEndOfRoad;

						trigger->duration_ = strtod(parameters.ReadAttribute(condition_node, "duration"));

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
				TrigByEntity *trigger = (TrigByEntity*)condition;

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
						entity.object_ = entities_->GetObjectByName(parameters.ReadAttribute(triggeringEntitiesChild, "entityRef"));
						trigger->triggering_entities_.entity_.push_back(entity);
					}
				}
			}
		}
		else if (conditionChildName == "ByValueCondition")
		{
			for (pugi::xml_node byValueChild = conditionChild.first_child(); byValueChild; byValueChild = byValueChild.next_sibling())
			{
				std::string byValueChildName(byValueChild.name());
				if (byValueChildName == "SimulationTimeCondition")
				{
					TrigBySimulationTime *trigger = new TrigBySimulationTime;
					trigger->value_ = strtod(parameters.ReadAttribute(byValueChild, "value"));
					trigger->rule_ = ParseRule(parameters.ReadAttribute(byValueChild, "rule"));
					condition = trigger;
				}
				else if (byValueChildName == "StoryboardElementStateCondition")
				{
					StoryBoardElement::ElementType element_type = ParseElementType(parameters.ReadAttribute(byValueChild, "storyboardElementType"));
					TrigByState::CondElementState state = ParseState(parameters.ReadAttribute(byValueChild, "state"));
					std::string element_name = parameters.ReadAttribute(byValueChild, "storyboardElementRef");

					TrigByState *trigger = new TrigByState(state, element_type, element_name);

					condition = trigger;
				}
				else
				{
					LOG("TrigByValue %s not implemented", byValueChildName.c_str());
				}
			}
		}
		else
		{
			throw std::runtime_error("Unsupported condition %s\n" + conditionChildName);
		}
	}

	if (condition == 0)
	{
		return 0;
	}

	condition->name_ = parameters.ReadAttribute(conditionNode, "name");
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

Trigger* ScenarioReader::parseTrigger(pugi::xml_node triggerNode)
{
	LOG("Parsing Trigger");
	Trigger *trigger = new Trigger;

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
	LOG("Parsing OSCManeuver %s", maneuver->name_.c_str());

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
			LOG("Parsing Event %s", event->name_.c_str());

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
			event->max_num_executions_ = strtoi(parameters.ReadAttribute(maneuverChild, "maximumExecutionCount"));

			for (pugi::xml_node eventChild = maneuverChild.first_child(); eventChild; eventChild = eventChild.next_sibling())
			{

				std::string childName(eventChild.name());

				if (childName == "Action")
				{
					for (pugi::xml_node actionChild = eventChild.first_child(); actionChild; actionChild = actionChild.next_sibling())
					{
						std::string childName(actionChild.name());

						if (childName == "GlobalAction")
						{
							LOG("Parsing global action %s", parameters.ReadAttribute(eventChild, "name").c_str());
							OSCGlobalAction *action = parseOSCGlobalAction(actionChild);
							event->action_.push_back((OSCAction*)action);
						}
						else if (childName == "UserDefinedAction")
						{
							LOG("%s is not implemented", childName.c_str());
						}
						else if (childName == "PrivateAction")
						{
							for (size_t i = 0; i < mGroup->actor_.size(); i++)
							{
								LOG("Parsing private action %s", parameters.ReadAttribute(eventChild, "name").c_str());
								OSCPrivateAction *action = parseOSCPrivateAction(actionChild, mGroup->actor_[i]->object_);
								if (action != 0)
								{
									event->action_.push_back((OSCAction*)action);
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
					event->start_trigger_ = parseTrigger(eventChild);
				}
				else
				{
					LOG("%s not supported", childName.c_str());
				}
			}
			maneuver->event_.push_back(event);
		}
	}
}

ConditionGroup* ScenarioReader::ParseConditionGroup(pugi::xml_node node)
{
	ConditionGroup* cg = new ConditionGroup();

	for (pugi::xml_node conditionNode = node.first_child(); conditionNode; conditionNode = conditionNode.next_sibling())
	{
		OSCCondition* condition = parseOSCCondition(conditionNode);
		cg->condition_.push_back(condition);
	}

	return cg;
}

int ScenarioReader::parseStoryBoard(StoryBoard &storyBoard)
{
	LOG("Parsing Story");

	pugi::xml_node storyNode = doc_.child("OpenSCENARIO").child("Storyboard").child("Story");

	for (; storyNode; storyNode = storyNode.next_sibling())
	{
		std::string storyNodeName(storyNode.name());

		if (storyNodeName == "Story")
		{
			std::string name = parameters.ReadAttribute(storyNode, "name", true);
			Story *story = new Story(name);

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

						std::string childName(actChild.name());

						if (childName == "ManeuverGroup")
						{
							ManeuverGroup *mGroup = new ManeuverGroup;

							mGroup->max_num_executions_ = strtoi(parameters.ReadAttribute(actChild, "maximumExecutionCount"));
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
										actor->object_ = entities_->GetObjectByName(parameters.ReadAttribute(actorsChild, "entityRef"));
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
								Entry *entry = ResolveCatalogReference(catalog_n);

								if (entry == 0 || entry->node_ == 0)
								{
									return -1;
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
									LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
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
						else if (childName == "StartTrigger")
						{
							act->start_trigger_ = parseTrigger(actChild);
						}
						else if (childName == "StopTrigger")
						{
							act->stop_trigger_ = parseTrigger(actChild);
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
			storyBoard.stop_trigger_ = parseTrigger(storyNode);
		}
	}

	return 0;
}
