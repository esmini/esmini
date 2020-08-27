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

#include <cstdlib>

namespace {
	int strtoi(std::string s) {
		return atoi(s.c_str());
	}

	double strtod(std::string s) {
		return atof(s.c_str());
	}
}

using namespace scenarioengine;

void ScenarioReader::addParameterDeclarations(pugi::xml_node xml_node)
{
	parseParameterDeclarations(xml_node, &parameterDeclarations_);
}

void ScenarioReader::parseGlobalParameterDeclarations()
{
	parseParameterDeclarations(doc_.child("OpenSCENARIO").child("ParameterDeclarations"), &parameterDeclarations_);
	paramDeclarationsSize_ = (int)parameterDeclarations_.Parameter.size();
}

void ScenarioReader::RestoreParameterDeclarations()
{
	parameterDeclarations_.Parameter.erase(
		parameterDeclarations_.Parameter.begin(),
		parameterDeclarations_.Parameter.begin() + parameterDeclarations_.Parameter.size() - paramDeclarationsSize_);
	catalog_param_assignments.clear();
}

void ScenarioReader::addParameter(std::string name, std::string value)
{
	ParameterStruct param;

	LOG("adding %s = %s", name.c_str(), value.c_str());

	param.name = name;
	param.type = "string";
	param.value = value;

	parameterDeclarations_.Parameter.insert(parameterDeclarations_.Parameter.begin(), param);
}

std::string ScenarioReader::getParameter(OSCParameterDeclarations &parameterDeclaration, std::string name)
{
	LOG("Resolve parameter %s", name.c_str());

	// If string already present in parameterDeclaration
	for (size_t i = 0; i < parameterDeclaration.Parameter.size(); i++)
	{
		if (parameterDeclaration.Parameter[i].name == name)
		{
			LOG("%s replaced with %s", name.c_str(), parameterDeclaration.Parameter[i].value.c_str());
			return parameterDeclaration.Parameter[i].value;
		}
	}
	LOG("Failed to resolve parameter %s", name.c_str());
	throw std::runtime_error("Failed to resolve parameter");
	return 0;
}

std::string ScenarioReader::ReadAttribute(pugi::xml_node node, std::string attribute_name, bool required)
{
	if (!strcmp(attribute_name.c_str(), ""))
	{
		if (required)
		{
			LOG("Warning: Empty attribute");
		}
		return "";
	}

	pugi::xml_attribute attr;

	if ((attr = node.attribute(attribute_name.c_str())))
	{
		if (attr.value()[0] == '$')
		{
			// Resolve variable
			return getParameter(parameterDeclarations_, attr.value());
		}
		else
		{
			return attr.value();
		}
	}
	else
	{
		if (required)
		{
			LOG("Error: missing required attribute: %s", attribute_name.c_str());
		}
		else
		{
			LOG("Warning: missing attribute: %s", attribute_name.c_str());
		}
	}

	return "";
}

int ScenarioReader::loadOSCFile(const char * path)
{
	LOG("Loading %s", path);

	pugi::xml_parse_result result = doc_.load_file(path);
	if (!result)
	{
		LOG("Error: %s", result.description());
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

	std::string dirname = ReadAttribute(catalogDirChild.child("Directory"), "path", true);

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
		std::string entry_name = ReadAttribute(entry_n, "name");

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

void ScenarioReader::parseParameterDeclarations(pugi::xml_node parameterDeclarationsNode, OSCParameterDeclarations *pd)
{
	LOG("Parsing ParameterDeclarations");

	for (pugi::xml_node pdChild = parameterDeclarationsNode.first_child(); pdChild; pdChild = pdChild.next_sibling())
	{
		ParameterStruct param;

		param.name = pdChild.attribute("name").value();

		// Check for catalog parameter assignements, overriding default value
		param.value = pdChild.attribute("value").value();
		for (size_t i = 0; i < catalog_param_assignments.size(); i++)
		{
			if (param.name == catalog_param_assignments[i].name)
			{
				param.value = catalog_param_assignments[i].value;
				break;
			}
		}
		param.type = pdChild.attribute("parameterType").value();
		pd->Parameter.insert(pd->Parameter.begin(), param);
	}
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
				properties.file_.filepath_ = ReadAttribute(propertiesChild, "filepath");
				if (properties.file_.filepath_ != "")
				{
					LOG("Properties/File = %s registered", properties.file_.filepath_.c_str());
				}
			}
			else if (!strcmp(propertiesChild.name(), "Property"))
			{
				OSCProperties::Property property;
				property.name_ = ReadAttribute(propertiesChild, "name");
				property.value_ = ReadAttribute(propertiesChild, "value");
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
	vehicle->control_ = Object::Control::INTERNAL;
	vehicle->model_id_ = -1;
	vehicle->model_filepath_ = "";

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
				boundingbox.center_.x_ = std::stof(ReadAttribute(boundingboxChild, "x"));
				boundingbox.center_.y_ = std::stof(ReadAttribute(boundingboxChild, "y"));
				boundingbox.center_.z_ = std::stof(ReadAttribute(boundingboxChild, "z"));
			} else if (boundingboxChildName == "Dimensions")
			{
				boundingbox.dimensions_.width_ = std::stof(ReadAttribute(boundingboxChild, "width"));
				boundingbox.dimensions_.length_ = std::stof(ReadAttribute(boundingboxChild, "length"));
				boundingbox.dimensions_.height_ = std::stof(ReadAttribute(boundingboxChild, "height"));
			} else {
				LOG("Not valid boudingbox attribute name:%s",boundingboxChildName.c_str());
			}
		}
		//LOG("Parsing boundingbox for vehicle:%s,center_x:%f, dimensions_width: %f.",ReadAttribute(xml_node, "name").c_str(),boundingbox.center_.x_,boundingbox.dimensions_.width_);
	}
}

Vehicle* ScenarioReader::parseOSCVehicle(pugi::xml_node vehicleNode)
{
	Vehicle *vehicle = new Vehicle();

	if (vehicleNode == 0)
	{
		return 0;
	}
	vehicle->name_ = ReadAttribute(vehicleNode, "name");
	LOG("Parsing Vehicle %s", vehicle->name_.c_str());
	vehicle->SetCategory(ReadAttribute(vehicleNode, "vehicleCategory"));

	OSCProperties properties;
	ParseOSCProperties(properties, vehicleNode);

	for(size_t i=0; i<properties.property_.size(); i++)
	{
		// Check if the property is something supported
		if (properties.property_[i].name_ == "control")
		{
			if (properties.property_[i].value_ == "internal")
			{
				vehicle->control_ = Object::Control::INTERNAL;
			}
			else if (properties.property_[i].value_ == "external")
			{
				vehicle->control_ = Object::Control::EXTERNAL;
			}
			else if (properties.property_[i].value_ == "hybrid")
			{
				// This will be the ghost vehicle, controlled by scenario engine,
				// which the externally controlled vehicle will follow
				vehicle->control_ = Object::Control::HYBRID_GHOST;
			}
			else
			{
				vehicle->control_ = Object::Control::UNDEFINED;
			}
		}
		else if (properties.property_[i].name_ == "model_id")
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
	ParseOSCBoundingBox(boundingbox,vehicleNode);
	vehicle->boundingbox_=boundingbox;

	return vehicle;
}

Controller* ScenarioReader::parseOSCObjectController(pugi::xml_node controllerNode)
{
	Controller *controller = new Controller();

	if (controllerNode == 0)
	{
		return 0;
	}
	controller->name_ = ReadAttribute(controllerNode, "name");
	LOG("Parsing Controller %s", controller->name_.c_str());

	OSCProperties properties;
	ParseOSCProperties(properties, controllerNode);

	if (properties.file_.filepath_ != "")
	{
		controller->config_filepath_ = properties.file_.filepath_;
	}

	return controller;
}

roadmanager::Route* ScenarioReader::parseOSCRoute(pugi::xml_node routeNode)
{
	roadmanager::Route *route = new roadmanager::Route;

	route->setName(ReadAttribute(routeNode, "name"));

	LOG("Parsing OSCRoute %s", route->getName().c_str());

	// Closed attribute not supported by roadmanager yet
	std::string closed_str = ReadAttribute(routeNode, "closed");
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
	file.filepath = ReadAttribute(fileNode, "filepath");
}

roadmanager::Trajectory* ScenarioReader::parseTrajectory(pugi::xml_node node)
{
	roadmanager::Trajectory* traj = new roadmanager::Trajectory;
	roadmanager::Shape* shape = 0;
	bool params = false;

	traj->name_ = ReadAttribute(node, "name");
	traj->closed_ = ReadAttribute(node, "closed") == "true" ? true : false;

	LOG("Parsing Trajectory %s", traj->name_.c_str());

	for (pugi::xml_node childNode = node.first_child(); childNode; childNode = childNode.next_sibling())
	{
		std::string childNodeName(childNode.name());

		if (childNodeName == "ParameterDeclarations")
		{
			params = true;
			addParameterDeclarations(childNode);
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
					double time = strtod(ReadAttribute(vertexNode, "time"));
					pline->AddVertex(*pos->GetRMPos(), time);
				}
				shape = pline;
			}
			else if (shapeType == "Clothoid")
			{
				LOG("Parsing Clothoid");
				pugi::xml_node posNode = shapeNode.child("Position");
				OSCPosition* pos = parseOSCPosition(posNode);
				
				double curvature = strtod(ReadAttribute(shapeNode, "curvature"));
				double curvatureDot = strtod(ReadAttribute(shapeNode, "curvatureDot"));
				double length = strtod(ReadAttribute(shapeNode, "length"));
				double startTime = strtod(ReadAttribute(shapeNode, "startTime"));
				double stopTime = strtod(ReadAttribute(shapeNode, "stopTime"));

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

	if (params) RestoreParameterDeclarations();

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
		param.name = param_n.attribute("parameterRef").value();
		param.value = ReadAttribute(param_n, "value");
		catalog_param_assignments.push_back(param);
	}

	catalog_name = ReadAttribute(node, "catalogName");
	entry_name = ReadAttribute(node, "entryName");

	if (catalog_param_assignments.size() > 0)
	{
		LOG("Assigned %d parameters for catalog %s entry %s", catalog_param_assignments.size(), catalog_name.c_str(), entry_name.c_str());
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
					std::string entry_name = ReadAttribute(objectChild, "entryName");
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
					else
					{
						LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
					}
				}

				RestoreParameterDeclarations();
			}
			else if (objectChildName == "Vehicle")
			{
				Vehicle *vehicle = parseOSCVehicle(objectChild);
				obj = vehicle;
			}
			else if (objectChildName == "ObjectController")
			{
				//get the sub child under ObjectController (should only be one)
				pugi::xml_node objectSubChild = objectChild.first_child();
				std::string objectSubChildName(objectSubChild.name());
				if (objectSubChildName == "CatalogReference")
				{
					Entry *entry = ResolveCatalogReference(objectSubChild);

					if (entry == 0){
						LOG("No entry found");
					} else {
						if (entry->type_ == CatalogType::CATALOG_CONTROLLER)
						{
							LOG("START!!! Parsing traffic controller from reference");
							Controller *controller = parseOSCObjectController(entry->GetNode());
							ctrl = controller;
							LOG("Parsing traffic controller from reference: %s",ctrl->name_.c_str());
						} else {
							LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
						}
					}
				}
				else {
					Controller *controller = parseOSCObjectController(objectSubChild);
					ctrl = controller;
					LOG("Parsing traffic controller: %s",ctrl->name_.c_str());
				}
			}
			else
			{
				LOG("%s not supported yet", objectChildName.c_str());
			}
		}

		if (obj != 0 && ctrl == 0)
		{
			obj->name_ = ReadAttribute(entitiesChild, "name");
			entities_->addObject(obj);
			objectCnt_++;
		} 
		else if (obj != 0 && ctrl != 0)
		{
			// if sumo controlled vehicle, the object will be passed to sumo template, and
			std::string configfile_path = ctrl->config_filepath_;
			
			if (!FileExists(configfile_path.c_str()) || (docsumo_.load_file(configfile_path.c_str()).status == pugi::status_file_not_found))
			{
				// Then assume relative path to scenario directory - which perhaps should be the expected location
				std::string configfile_path2 = CombineDirectoryPathAndFilepath(DirNameOf(oscFilename_), configfile_path);

				if (!FileExists(configfile_path2.c_str()) || (docsumo_.load_file(configfile_path2.c_str()).status == pugi::status_file_not_found))
				{
					// Give up
					LOG("Failed to load SUMO config file %s, also tried %s", configfile_path.c_str(), configfile_path2.c_str());
					return -1;
				}
				else
				{
					// Update file path
					configfile_path = configfile_path2;
					ctrl->config_filepath_ = configfile_path;
				}
			}

			std::vector<std::string> file_name_candidates;
			file_name_candidates.push_back(ReadAttribute(docsumo_.child("configuration").child("input").child("net-file"), "value"));
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(configfile_path), file_name_candidates[0]));
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(oscFilename_), file_name_candidates[0]));
			pugi::xml_parse_result sumonet;
			size_t i;
			for (i = 0; i < file_name_candidates.size(); i++)
			{
				sumonet = docsumo_.load_file(file_name_candidates[i].c_str());
				if (sumonet.status == pugi::status_ok)
				{
					break;
				}
			}
			if (sumonet.status != pugi::status_ok)
			{
				// Give up
				LOG("Failed to load SUMO net file %s", file_name_candidates[0].c_str());
				return -1;
			}

			pugi::xml_node location = docsumo_.child("net").child("location");
			std::string netoffset = ReadAttribute(location, "netOffset");
			std::size_t delim = netoffset.find(',');
			entities_->sumo_x_offset = std::stof(netoffset.substr(0,delim));
			entities_->sumo_y_offset = std::stof(netoffset.substr(delim+1,netoffset.npos));			
			
			entities_->sumo_vehicle = obj;
			entities_->sumo_config_path = ctrl->config_filepath_;
		}
	}

	return 0;
}

void ScenarioReader::parseOSCOrientation(OSCOrientation &orientation, pugi::xml_node orientationNode)
{
	orientation.h_ = strtod(ReadAttribute(orientationNode, "h"));
	orientation.p_ = strtod(ReadAttribute(orientationNode, "p"));
	orientation.r_ = strtod(ReadAttribute(orientationNode, "r"));

	std::string type_str = ReadAttribute(orientationNode, "type");

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

	LOG("Parsing %s: ", positionChild.name());

	std::string positionChildName(positionChild.name());

	if (positionChildName == "WorldPosition")
	{

		double x = strtod(ReadAttribute(positionChild, "x"));
		double y = strtod(ReadAttribute(positionChild, "y"));
		double z = strtod(ReadAttribute(positionChild, "z"));
		double h = strtod(ReadAttribute(positionChild, "h"));
		double p = strtod(ReadAttribute(positionChild, "p"));
		double r = strtod(ReadAttribute(positionChild, "r"));

		OSCPositionWorld *pos = new OSCPositionWorld(x, y, z, h, p, r);

		pos_return = (OSCPosition*)pos;
	}
	else if (positionChildName == "RelativeWorldPosition")
	{
		double dx, dy, dz;

		dx = strtod(ReadAttribute(positionChild, "dx"));
		dy = strtod(ReadAttribute(positionChild, "dy"));
		dz = strtod(ReadAttribute(positionChild, "dz"));
		Object* object = FindObjectByName(ReadAttribute(positionChild, "entityRef"));

		// Check for optional Orientation element
		pugi::xml_node orientation_node = positionChild.child("Orientation");
		OSCOrientation orientation;
		if (orientation_node)
		{
			parseOSCOrientation(orientation, orientation_node);
		}
		else
		{
			// Calculate orientation
			orientation.h_ = GetAngleOfVector(dx, dy);
		}

		OSCPositionRelativeWorld* pos = new OSCPositionRelativeWorld(object, dx, dy, dz, orientation);

		pos_return = (OSCPosition*)pos;
	}
	else if (positionChildName == "RelativeObjectPosition")
	{
		double dx, dy, dz;

		dx = strtod(ReadAttribute(positionChild, "dx"));
		dy = strtod(ReadAttribute(positionChild, "dy"));
		dz = strtod(ReadAttribute(positionChild, "dz"));
		Object *object = FindObjectByName(ReadAttribute(positionChild, "entityRef"));

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

		dLane = strtoi(ReadAttribute(positionChild, "dLane"));
		ds = strtod(ReadAttribute(positionChild, "ds"));
		offset = strtod(ReadAttribute(positionChild, "offset"));
		Object *object = FindObjectByName(ReadAttribute(positionChild, "entityRef"));

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
		int road_id = strtoi(ReadAttribute(positionChild, "roadId"));
		int lane_id = strtoi(ReadAttribute(positionChild, "laneId"));
		double s = strtod(ReadAttribute(positionChild, "s"));

		double offset = 0;  // Default value of optional parameter
		if (positionChild.attribute("offset"))
		{
			offset = strtod(ReadAttribute(positionChild, "offset"));
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
						// Add inline route to route catalog
						LOG("Inline route reference not supported yet - put the route into a catalog");
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

						RestoreParameterDeclarations();
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
						double s = strtod(ReadAttribute(positionChild, "pathS"));
						int lane_id = strtoi(ReadAttribute(positionChild, "laneId"));
						double lane_offset = 0;

						pugi::xml_attribute laneOffsetAttribute = positionChild.attribute("laneOffset");
						if (laneOffsetAttribute != NULL)
						{
							lane_offset = strtod(ReadAttribute(positionChild, "laneOffset"));
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

	if (pos_return)
	{
		pos_return->Print();
	}
	else
	{
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
	td.shape_ = ParseDynamicsShape(ReadAttribute(node, "dynamicsShape"));
	td.dimension_ = ParseDynamicsDimension(ReadAttribute(node, "dynamicsDimension"));
	td.target_value_ = strtod(ReadAttribute(node, "value"));

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
			action->name_ = ReadAttribute(actionNode.parent(), "name");
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

									target_rel->value_ = strtod(ReadAttribute(targetChild, "value"));

									target_rel->continuous_ = (
										ReadAttribute(targetChild, "continuous") == "true" ||
										ReadAttribute(targetChild, "continuous") == "1");

									target_rel->object_ = FindObjectByName(ReadAttribute(targetChild, "entityRef"));

									std::string value_type = ReadAttribute(targetChild, "speedTargetValueType");
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

									target_abs->value_ = strtod(ReadAttribute(targetChild, "value"));
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
						action_dist->dynamics_.max_acceleration_ = strtod(ReadAttribute(dynamics_node, "maxAcceleration"));
						if (action_dist->dynamics_.max_acceleration_ < SMALL_NUMBER)
						{
							LOG("Unexpected small maxAcceleration value: %.2, replacing with %.2f", action_dist->dynamics_.max_acceleration_, 10);
							action_dist->dynamics_.max_acceleration_ = 10.0;
						}

						action_dist->dynamics_.max_deceleration_ = strtod(ReadAttribute(dynamics_node, "maxDeceleration"));
						if (action_dist->dynamics_.max_deceleration_ < SMALL_NUMBER)
						{
							LOG("Unexpected small maxDeceleration value: %.2, replacing with %.2f", action_dist->dynamics_.max_deceleration_, 10);
							action_dist->dynamics_.max_deceleration_ = 10.0;
						}

						action_dist->dynamics_.max_speed_ = strtod(ReadAttribute(dynamics_node, "maxSpeed"));
                        action_dist->dynamics_.none_ = false;
					}
					else
					{
						action_dist->dynamics_.none_ = true;
					}

					action_dist->target_object_ = FindObjectByName(ReadAttribute(longitudinalChild, "entityRef"));
					if (longitudinalChild.attribute("distance"))
					{
						action_dist->dist_type_ = LongDistanceAction::DistType::DISTANCE;
						action_dist->distance_ = strtod(ReadAttribute(longitudinalChild, "distance"));
					}
					else if (longitudinalChild.attribute("timeGap"))
					{
						action_dist->dist_type_ = LongDistanceAction::DistType::TIME_GAP;
						action_dist->distance_ = strtod(ReadAttribute(longitudinalChild, "timeGap"));
					}
					else
					{
						LOG("Need distance or timeGap");
					}

					if (longitudinalChild.attribute("continuous"))
					{
						LOG("continuous flag assumed and always on by default");
					}

					std::string freespace = ReadAttribute(longitudinalChild, "freespace");
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

					if (ReadAttribute(lateralChild, "targetLaneOffset") != "")
					{
						action_lane->target_lane_offset_ = strtod(ReadAttribute(lateralChild, "targetLaneOffset"));
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

									if ((target_rel->object_ = FindObjectByName(ReadAttribute(targetChild, "entityRef"))) == 0)
									{
										LOG("Failed to find object %s", ReadAttribute(targetChild, "entityRef").c_str());
										return 0;
									}
									target_rel->value_ = strtoi(ReadAttribute(targetChild, "value"));
									target = target_rel;
								}
								else if (targetChild.name() == std::string("AbsoluteTargetLane"))
								{
									LatLaneChangeAction::TargetAbsolute *target_abs = new LatLaneChangeAction::TargetAbsolute;

									target_abs->value_ = strtoi(ReadAttribute(targetChild, "value"));
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
							if (ReadAttribute(laneOffsetChild, "maxLateralAcc") != "")
							{
								action_lane->dynamics_.max_lateral_acc_ = strtod(ReadAttribute(laneOffsetChild, "maxLateralAcc"));
								if (action_lane->dynamics_.max_lateral_acc_ < SMALL_NUMBER)
								{
									action_lane->dynamics_.max_lateral_acc_ = SMALL_NUMBER;
								}
							}

							action_lane->dynamics_.transition_.shape_ = ParseDynamicsShape(ReadAttribute(laneOffsetChild, "dynamicsShape"));
						}
						else if (laneOffsetChild.name() == std::string("LaneOffsetTarget"))
						{
							LatLaneOffsetAction::Target *target;

							for (pugi::xml_node targetChild = laneOffsetChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("RelativeTargetLaneOffset"))
								{
									LatLaneOffsetAction::TargetRelative *target_rel = new LatLaneOffsetAction::TargetRelative;

									target_rel->object_ = FindObjectByName(ReadAttribute(targetChild, "entityRef"));
									target_rel->value_ = strtod(ReadAttribute(targetChild, "value"));
									target = target_rel;
								}
								else if (targetChild.name() == std::string("AbsoluteTargetLaneOffset"))
								{
									LatLaneOffsetAction::TargetAbsolute *target_abs = new LatLaneOffsetAction::TargetAbsolute;

									target_abs->value_ = strtod(ReadAttribute(targetChild, "value"));
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

			std::string master_object_str = ReadAttribute(actionChild, "masterEntityRef");
			action_synch->master_object_ = FindObjectByName(master_object_str);

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
					targetSpeedAbs->value_ = strtod(ReadAttribute(target_speed_element, "value"));
					action_synch->final_speed_ = targetSpeedAbs;
				}
				else
				{
					pugi::xml_node target_speed_element = target_speed_node.child("RelativeSpeedToMaster");

					if (target_speed_element != NULL)
					{
						LongSpeedAction::TargetRelative *targetSpeedRel = new LongSpeedAction::TargetRelative;

						targetSpeedRel->value_ = strtod(ReadAttribute(target_speed_element, "value"));

						targetSpeedRel->continuous_ = true;  // Continuous adaption needed

						targetSpeedRel->object_ = action_synch->master_object_;  // Master object is the pivot vehicle

						std::string value_type = ReadAttribute(target_speed_element, "speedTargetValueType");
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
			PositionAction *action_pos = new PositionAction;
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
					for (pugi::xml_node followRouteChild = routingChild.first_child(); followRouteChild; followRouteChild = followRouteChild.next_sibling())
					{
						if (followRouteChild.name() == std::string("Route"))
						{
							FollowRouteAction* action_follow_route = new FollowRouteAction;
							action_follow_route->route_ = parseOSCRoute(followRouteChild);
							action = action_follow_route;
						}
						else if (followRouteChild.name() == std::string("CatalogReference"))
						{
							FollowRouteAction *action_follow_route = new FollowRouteAction;

							// Find route in catalog
							Entry *entry = ResolveCatalogReference(followRouteChild);

							if (entry == 0 || entry->node_ == 0)
							{
								return 0;
							}

							if (entry->type_ == CatalogType::CATALOG_ROUTE)
							{
								// Make a new instance from catalog entry
								action_follow_route->route_ = parseOSCRoute(entry->GetNode());
								action = action_follow_route;
								break;
							}
							else
							{
								LOG("Catalog entry of type %s expected - found %s", Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE).c_str(), entry->GetTypeAsStr().c_str());
								return 0;
							}

							RestoreParameterDeclarations();
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

							RestoreParameterDeclarations();
						}
						else if (followTrajetoryChild.name() == std::string("TimeReference"))
						{
							pugi::xml_node timingNode = followTrajetoryChild.first_child();
							if (timingNode && std::string(timingNode.name()) == "Timing")
							{
								std::string timeDomain = ReadAttribute(timingNode, "domainAbsoluteRelative");
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

								action_follow_trajectory->timing_scale_ = strtod(ReadAttribute(timingNode, "scale"));
								action_follow_trajectory->timing_offset_ = strtod(ReadAttribute(timingNode, "offset"));
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
			ActivateControllerAction * activateControllerAction = new ActivateControllerAction;

			activateControllerAction->longitudinal_ = ReadAttribute(actionChild, "longitudinal") == "true";
			activateControllerAction->lateral_ = ReadAttribute(actionChild, "lateral") == "true";

			LOG("ActivateControllerAction: Longitudinal: %s Lateral: %s",
				activateControllerAction->longitudinal_ ? "true" : "false",
				activateControllerAction->lateral_ ? "true" : "false"
			);

			action = activateControllerAction;
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
			action->name_ = ReadAttribute(actionNode.parent(), "name");
		}
		else
		{
			action->name_ = "no name";
		}
		action->object_ = object;
	}

	return action;
}

Object* ScenarioReader::FindObjectByName(std::string name)
{
	for (size_t i = 0; i < entities_->object_.size(); i++)
	{
		if (name == entities_->object_[i]->name_)
		{
			return entities_->object_[i];
		}
	}

	LOG("Failed to find object %s", name.c_str());
	throw std::runtime_error(std::string("Failed to find object " + name).c_str());
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

			entityRef = FindObjectByName(ReadAttribute(actionsChild, "entityRef"));
			if (entityRef != NULL)
			{
				for (pugi::xml_node privateChild = actionsChild.first_child(); privateChild; privateChild = privateChild.next_sibling())
				{
					// Assume children are PrivateActions
					OSCPrivateAction *action = parseOSCPrivateAction(privateChild, entityRef);
					if (action == 0)
					{
						LOG("Failed to parse private action");
						return;
					}
					action->name_ = "Init " + entityRef->name_ + " " + privateChild.first_child().name();
					init.private_action_.push_back(action);
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
			if (init.private_action_[j]->type_ == OSCPrivateAction::ActionType::POSITION)
			{
				PositionAction* action = (PositionAction*)init.private_action_[j];
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
	LOG("Parsing OSCCondition %s", ReadAttribute(conditionNode, "name").c_str());

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
						trigger->object_ = FindObjectByName(ReadAttribute(condition_node, "entityRef"));

						std::string along_route_str = ReadAttribute(condition_node, "alongRoute");
						if ((along_route_str == "true") || (along_route_str == "1"))
						{
							trigger->along_route_ = true;
						}
						else
						{
							trigger->along_route_ = false;
						}

						std::string freespace_str = ReadAttribute(condition_node, "freespace");
						if ((freespace_str == "true") || (freespace_str == "1"))
						{
							trigger->freespace_ = true;
						}
						else
						{
							trigger->freespace_ = false;
						}
						trigger->value_ = strtod(ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(ReadAttribute(condition_node, "rule"));

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
							trigger->tolerance_ = strtod(ReadAttribute(condition_node, "tolerance"));
						}

						// Read position
						pugi::xml_node pos_node = condition_node.child("Position");
						trigger->position_ = parseOSCPosition(pos_node);

						condition = trigger;
					}
					else if (condition_type == "RelativeDistanceCondition")
					{
						TrigByRelativeDistance *trigger = new TrigByRelativeDistance;
						trigger->object_ = FindObjectByName(ReadAttribute(condition_node, "entityRef"));

						std::string type = ReadAttribute(condition_node, "relativeDistanceType");
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

						std::string freespace_str = ReadAttribute(condition_node, "freespace");
						if ((freespace_str == "true") || (freespace_str == "1"))
						{
							trigger->freespace_ = true;
						}
						else
						{
							trigger->freespace_ = false;
						}
						trigger->value_ = strtod(ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "DistanceCondition")
					{
						TrigByDistance *trigger = new TrigByDistance;

						// Read position
						pugi::xml_node pos_node = condition_node.child("Position");

						trigger->position_ = parseOSCPosition(pos_node);

						std::string freespace_str = ReadAttribute(condition_node, "freespace");
						if ((freespace_str == "true") || (freespace_str == "1"))
						{
							trigger->freespace_ = true;
						}
						else
						{
							trigger->freespace_ = false;
						}

						std::string along_route_str = ReadAttribute(condition_node, "alongRoute");
						if ((along_route_str == "true") || (along_route_str == "1"))
						{
							LOG("Condition Distance along route not supported yet - falling back to alongeRoute = false");
							trigger->along_route_ = false;
						}
						else
						{
							trigger->along_route_ = false;
						}

						trigger->value_ = strtod(ReadAttribute(condition_node, "value"));
						trigger->rule_ = ParseRule(ReadAttribute(condition_node, "rule"));

						condition = trigger;
					}
					else if (condition_type == "TraveledDistanceCondition")
					{
						TrigByTraveledDistance* trigger = new TrigByTraveledDistance;

						trigger->value_ = strtod(ReadAttribute(condition_node, "value"));

						condition = trigger;
					}
					else if (condition_type == "EndOfRoadCondition")
					{
						TrigByEndOfRoad* trigger = new TrigByEndOfRoad;

						trigger->duration_ = strtod(ReadAttribute(condition_node, "duration"));

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

				std::string trig_ent_rule = ReadAttribute(triggering_entities, "triggeringEntitiesRule");
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
						entity.object_ = FindObjectByName(ReadAttribute(triggeringEntitiesChild, "entityRef"));
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
					trigger->value_ = strtod(ReadAttribute(byValueChild, "value"));
					trigger->rule_ = ParseRule(ReadAttribute(byValueChild, "rule"));
					condition = trigger;
				}
				else if (byValueChildName == "StoryboardElementStateCondition")
				{
					StoryBoardElement::ElementType element_type = ParseElementType(ReadAttribute(byValueChild, "storyboardElementType"));
					TrigByState::CondElementState state = ParseState(ReadAttribute(byValueChild, "state"));
					std::string element_name = ReadAttribute(byValueChild, "storyboardElementRef");

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

	condition->name_ = ReadAttribute(conditionNode, "name");
	if (conditionNode.attribute("delay") != NULL)
	{
		condition->delay_ = strtod(ReadAttribute(conditionNode, "delay"));
	}
	else
	{
		LOG("Attribute \"delay\" missing");
	}

	std::string edge_str = ReadAttribute(conditionNode, "conditionEdge");
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
	maneuver->name_ = ReadAttribute(maneuverNode, "name");
	LOG("Parsing OSCManeuver %s", maneuver->name_.c_str());

	for (pugi::xml_node maneuverChild = maneuverNode.first_child(); maneuverChild; maneuverChild = maneuverChild.next_sibling())
	{
		std::string maneuverChildName(maneuverChild.name());

		if (maneuverChildName == "ParameterDeclarations")
		{
			addParameterDeclarations(maneuverChild);
		}
		else if (maneuverChildName == "Event")
		{
			Event *event = new Event;

			event->name_ = ReadAttribute(maneuverChild, "name");
			LOG("Parsing Event %s", event->name_.c_str());

			std::string prio = ReadAttribute(maneuverChild, "priority");
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
			event->max_num_executions_ = strtoi(ReadAttribute(maneuverChild, "maximumExecutionCount"));

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
							LOG("Parsing global action %s", ReadAttribute(eventChild, "name").c_str());
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
								LOG("Parsing private action %s", ReadAttribute(eventChild, "name").c_str());
								OSCPrivateAction *action = parseOSCPrivateAction(actionChild, mGroup->actor_[i]->object_);
								event->action_.push_back((OSCAction*)action);
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
			std::string name = ReadAttribute(storyNode, "name", true);
			Story *story = new Story(name);

			if (!strcmp(storyNode.first_child().name(), "ParameterDeclarations"))
			{
				addParameterDeclarations(storyNode.first_child());
			}

			for (pugi::xml_node storyChild = storyNode.child("Act"); storyChild; storyChild = storyChild.next_sibling("Act"))
			{
				std::string childName(storyChild.name());

				if (childName == "Act")
				{
					Act *act = new Act;

					act->name_ = ReadAttribute(storyChild, "name");

					for (pugi::xml_node actChild = storyChild.first_child(); actChild; actChild = actChild.next_sibling())
					{

						std::string childName(actChild.name());

						if (childName == "ManeuverGroup")
						{
							ManeuverGroup *mGroup = new ManeuverGroup;

							mGroup->max_num_executions_ = strtoi(ReadAttribute(actChild, "maximumExecutionCount"));
							mGroup->name_ = ReadAttribute(actChild, "name");

							pugi::xml_node actors_node = actChild.child("Actors");
							if (actors_node != NULL)
							{
								for (pugi::xml_node actorsChild = actors_node.first_child(); actorsChild; actorsChild = actorsChild.next_sibling())
								{
									ManeuverGroup::Actor *actor = new ManeuverGroup::Actor;

									std::string actorsChildName(actorsChild.name());
									if (actorsChildName == "EntityRef")
									{
										actor->object_ = FindObjectByName(ReadAttribute(actorsChild, "entityRef"));
									}
									else if (actorsChildName == "ByCondition")
									{
										LOG("Actor by condition - not implemented");
									}
									mGroup->actor_.push_back(actor);
									if (actor->object_->ghost_)
									{
										// Add ghost as well
										ManeuverGroup::Actor *actor = new ManeuverGroup::Actor;
										actor->object_ = FindObjectByName(ReadAttribute(actorsChild, "entityRef").append("_ghost"));
										mGroup->actor_.push_back(actor);
									}
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
								RestoreParameterDeclarations();
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
			RestoreParameterDeclarations();
		}
		else if (storyNodeName == "StopTrigger")
		{
			storyBoard.stop_trigger_ = parseTrigger(storyNode);
		}
	}

	return 0;
}
