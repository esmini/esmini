#include "ScenarioReader.hpp"
#include "CommonMini.hpp"


ScenarioReader::ScenarioReader()
{
	objectCnt = 0;
}

void ScenarioReader::addParameter(std::string name, std::string value)
{
	LOG("adding %s = %s", name.c_str(), value.c_str());

	parameterDeclaration.Parameter.resize(parameterDeclaration.Parameter.size() + 1);

	parameterDeclaration.Parameter.back().name = name;
	parameterDeclaration.Parameter.back().type = "string";
	parameterDeclaration.Parameter.back().value = value;
}

std::string ScenarioReader::getParameter(std::string name)
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
	return 0;
}

std::string ScenarioReader::ReadAttribute(pugi::xml_attribute attribute)
{
	if (attribute == 0)
	{
		return "";
	}

	if (attribute.value()[0] == '$')
	{
		// Resolve variable
		return getParameter(attribute.value());
	}
	else
	{
		return attribute.value();
	}
}

int ScenarioReader::loadOSCFile(const char * path)
{
	LOG("Loading %s", path);
	
	pugi::xml_parse_result result = doc.load_file(path);
	if (!result)
	{
		return -1;
	}

	oscFilename = path;

	return 0;
}

void ScenarioReader::parseParameterDeclaration()
{
	LOG("Parsing parameters");

	pugi::xml_node parameterDeclarationNode = doc.child("OpenSCENARIO").child("ParameterDeclaration");
	
	for (pugi::xml_node parameterDeclarationChild = parameterDeclarationNode.first_child(); parameterDeclarationChild; parameterDeclarationChild = parameterDeclarationChild.next_sibling())
	{
		parameterDeclaration.Parameter.resize(parameterDeclaration.Parameter.size() + 1);

		parameterDeclaration.Parameter.back().name = parameterDeclarationChild.attribute("name").value();
		parameterDeclaration.Parameter.back().type = parameterDeclarationChild.attribute("type").value();
		parameterDeclaration.Parameter.back().value = parameterDeclarationChild.attribute("value").value();
	}
}

void ScenarioReader::parseRoadNetwork(RoadNetwork &roadNetwork)
{
	LOG("Parsing RoadNetwork");

	pugi::xml_node roadNetworkNode = doc.child("OpenSCENARIO").child("RoadNetwork");

	for (pugi::xml_node roadNetworkChild = roadNetworkNode.first_child(); roadNetworkChild; roadNetworkChild = roadNetworkChild.next_sibling())
	{
		std::string roadNetworkChildName(roadNetworkChild.name());

		if (roadNetworkChildName == "Logics")
		{
			parseOSCFile(roadNetwork.Logics, roadNetworkChild);
		}
		else if (roadNetworkChildName == "SceneGraph")
		{
			parseOSCFile(roadNetwork.SceneGraph, roadNetworkChild);
		}
	}
}

void ScenarioReader::parseOSCRoute(OSCRoute &route, pugi::xml_node routeNode)
{
	LOG("Parsing OSCRoute");

	route.name = ReadAttribute(routeNode.attribute("name"));
	route.closed = ReadAttribute(routeNode.attribute("closed"));

	for (pugi::xml_node routeChild = routeNode.first_child(); routeChild; routeChild = routeChild.next_sibling())
	{
		std::string routeChildName(routeChild.name());

		if (routeChildName == "ParameterDeclaration")
		{
			LOG("%s is not implemented", routeChildName.c_str());

		}
		else if (routeChildName == "Waypoint")
		{
			route.Waypoint.resize(route.Waypoint.size() + 1);

			route.Waypoint.back().strategy = ReadAttribute(routeChild.attribute("strategy"));

			route.Waypoint.back().Position = new roadmanager::Position;

			parseOSCPosition(*route.Waypoint.back().Position, routeChild.first_child());
		}
	}
	LOG("parseOSCRoute finished");
}

void ScenarioReader::parseCatalogs(Catalogs &catalogs) 
{
	LOG("Parsing Catalogs");

	pugi::xml_node catalogsNode = doc.child("OpenSCENARIO").child("Catalogs");

	for (pugi::xml_node catalogsChild = catalogsNode.first_child(); catalogsChild; catalogsChild = catalogsChild.next_sibling())
	{
		std::string catalogsChildName(catalogsChild.name());

		if (catalogsChildName == "RouteCatalog")
		{
			for (pugi::xml_node route = catalogsChild.first_child(); route; route = route.next_sibling())
			{
				OSCRoute route_item;
				parseOSCRoute(route_item, route);
				catalogs.RouteCatalog.Route.push_back(route_item);
			}
		}
		else if (catalogsChildName == "VehicleCatalog")
		{
			catalogs.VehicleCatalog.Directory.path = ReadAttribute(catalogsChild.child("Directory").attribute("path"));
		}
		else if (catalogsChildName == "DriverCatalog")
		{
			catalogs.DriverCatalog.Directory.path = ReadAttribute(catalogsChild.child("Directory").attribute("path"));
		}
	}
}

static std::string dirnameOf(const std::string& fname)
{
	size_t pos = fname.find_last_of("\\/");
	return (std::string::npos == pos) ? "" : fname.substr(0, pos);
}

void ScenarioReader::parseOSCFile(OSCFile &file, pugi::xml_node fileNode)
{
	LOG("Parsing OSCFile %s", file.filepath.c_str());

	file.filepath = ReadAttribute(fileNode.attribute("filepath"));

	// If relative path (starting with "."), then assume it is relative to the scenario .xosc file
	if (file.filepath[0] == '.')
	{
		file.filepath.insert(0, dirnameOf(oscFilename) + "/");
	}
}

void ScenarioReader::parseOSCBoundingBox(OSCBoundingBox &boundingBox, pugi::xml_node boundingBoxNode)
{
	LOG("Parsing OSCBoundingBox");

	for (pugi::xml_node boundingBoxChild = boundingBoxNode.first_child(); boundingBoxChild; boundingBoxChild = boundingBoxChild.next_sibling())
	{
		std::string boundingBoxChildName(boundingBoxChild.name());

		if (boundingBoxChildName == "Center")
		{
			boundingBox.center.x = std::stod(ReadAttribute(boundingBoxChild.attribute("x")));
			boundingBox.center.y = std::stod(ReadAttribute(boundingBoxChild.attribute("y")));
			boundingBox.center.z = std::stod(ReadAttribute(boundingBoxChild.attribute("z")));
		}
		if (boundingBoxChildName == "Dimension")
		{
			boundingBox.dimension.width = std::stod(ReadAttribute(boundingBoxChild.attribute("width")));
			boundingBox.dimension.length = std::stod(ReadAttribute(boundingBoxChild.attribute("length")));
			boundingBox.dimension.height = std::stod(ReadAttribute(boundingBoxChild.attribute("height")));

		}
		boundingBox.printOSCBoundingBox();
	}
}


void ScenarioReader::parseOSCPersonDescription(OSCPersonDescription &personDescription, pugi::xml_node descriptionNode)
{
	LOG("Parsing OSCPersonDescription");

	if (descriptionNode.child("Properties"))
	{
		LOG("Properties is not implemented ");
	}

	personDescription.weight = std::stod(ReadAttribute(descriptionNode.attribute("weight")));
	personDescription.height = std::stod(ReadAttribute(descriptionNode.attribute("height")));
	personDescription.eyeDistance = std::stod(ReadAttribute(descriptionNode.attribute("eyeDistance")));
	personDescription.age = std::stod(ReadAttribute(descriptionNode.attribute("age")));
	personDescription.sex = ReadAttribute(descriptionNode.attribute("sex"));

	personDescription.printOSCPersonDescription();
}



void ScenarioReader::parseOSCAxle(OSCAxle &axle, pugi::xml_node axleNode)
{
	LOG("Parsing OSCAxle");
	axle.maxSteering = std::stod(ReadAttribute(axleNode.attribute("maxSteering")));
	axle.wheelDiameter = std::stod(ReadAttribute(axleNode.attribute("wheelDiameter")));
	axle.trackWidth = std::stod(ReadAttribute(axleNode.attribute("trackWidth")));
	axle.positionX = std::stod(ReadAttribute(axleNode.attribute("positionX")));
	axle.positionZ = std::stod(ReadAttribute(axleNode.attribute("positionZ")));

	axle.printOSCAxle();
}


void ScenarioReader::parseOSCDriver(OSCDriver &driver, pugi::xml_node driverNode)
{
	LOG("Parsing OSCDriver");

	driver.name = ReadAttribute(driverNode.attribute("name"));

	for (pugi::xml_node driverChild = driverNode.first_child(); driverChild; driverChild = driverChild.next_sibling())
	{
		std::string driverChildName(driverChild.name());

		if (driverChildName == "ParameterDeclaration")
		{
			LOG("%s is not implemented", driverChildName.c_str());
		}
		if (driverChildName == "Description")
		{
			parseOSCPersonDescription(driver.Description, driverChild);
		}
	}

	driver.printOSCDriver();
}


void ScenarioReader::parseOSCVehicle(OSCVehicle &vehicle, pugi::xml_node vehicleNode)
{
	LOG("Parsing OSCVehicle");

	vehicle.name = ReadAttribute(vehicleNode.attribute("name"));
	vehicle.category = ReadAttribute(vehicleNode.attribute("category"));

	for (pugi::xml_node vehicleChild = vehicleNode.first_child(); vehicleChild; vehicleChild = vehicleChild.next_sibling())
	{
		std::string vehicleChildName(vehicleChild.name());

		if (vehicleChildName == "ParameterDeclaration")
		{
			LOG("%s is not implemented", vehicleChildName.c_str());
		}
		else if (vehicleChildName == "BoundingBox")
		{
			parseOSCBoundingBox(vehicle.BoundingBox, vehicleChild);
		}
		else if (vehicleChildName == "Performance")
		{
			vehicle.Performance.maxSpeed = std::stod(ReadAttribute(vehicleChild.attribute("maxSpeed")));
			vehicle.Performance.maxDeceleration = std::stod(ReadAttribute(vehicleChild.attribute("maxDeceleration")));
			vehicle.Performance.mass = std::stod(ReadAttribute(vehicleChild.attribute("mass")));
		}
		else if (vehicleChildName == "Axles")
		{
			for (pugi::xml_node axlesChild = vehicleChild.first_child(); axlesChild; axlesChild = axlesChild.next_sibling())
			{
				std::string axlesChildName(axlesChild.name());

				if (axlesChildName == "Front")
				{
					parseOSCAxle(vehicle.Axles.Front, axlesChild);
				}
				else if (axlesChildName == "Rear")
				{
					parseOSCAxle(vehicle.Axles.Rear, axlesChild);
				}
				else if (axlesChildName == "Additional")
				{
					LOG("%s is not implemented", axlesChildName.c_str());
				}

			}
		}
		else if (vehicleChildName == "Properties")
		{
			LOG("%s is not implemented", vehicleChildName.c_str());
		}

	}

	vehicle.printOSCVehicle();
}

void ScenarioReader::parseOSCCatalogReference(OSCCatalogReference &catalogReference, pugi::xml_node catalogReferenceNode)
{

	catalogReference.catalogName = ReadAttribute(catalogReferenceNode.attribute("catalogName"));
	catalogReference.entryName = ReadAttribute(catalogReferenceNode.attribute("entryName"));

	pugi::xml_node catalogReferenceChild = catalogReferenceNode.first_child();
	std::string catalogReferenceChildName(catalogReferenceChild.name());

	if (catalogReferenceChildName == "ParameterAssignment")
	{
		LOG("%s is not implemented", catalogReferenceChildName.c_str());
	}
}

void ScenarioReader::parseEntities(Entities &entities)
{
	LOG("Parsing Entities");

	pugi::xml_node enitiesNode = doc.child("OpenSCENARIO").child("Entities");

	for (pugi::xml_node entitiesChild = enitiesNode.first_child(); entitiesChild; entitiesChild = entitiesChild.next_sibling())
	{
		Object *obj = new Object;
		obj->id_ = (int)entities.object_.size();
		entities.object_.push_back(obj);

		obj->name_ = ReadAttribute(entitiesChild.attribute("name"));

		for (pugi::xml_node objectChild = entitiesChild.first_child(); objectChild; objectChild = objectChild.next_sibling())
		{
			std::string objectChildName(objectChild.name());

			if (objectChildName == "CatalogReference")
			{
				parseOSCCatalogReference(obj->CatalogReference, objectChild);
			}
			else if (objectChildName == "Vehicle")
			{
				parseOSCVehicle(obj->Vehicle, objectChild);

			}
			else if (objectChildName == "Pedestrian")
			{
				LOG("%s is not implemented", objectChildName.c_str());

			}
			else if (objectChildName == "MiscObject")
			{
				LOG("%s is not implemented", objectChildName.c_str());

			}
			else if (objectChildName == "Controller")
			{
				pugi::xml_node controllerChild = objectChild.first_child();
				std::string controllerChildName(controllerChild.name());

				if (controllerChildName == "CatalogReference")
				{
					LOG("%s is not implemented", objectChildName.c_str());

				}
				else if (controllerChildName == "Driver")
				{
					parseOSCDriver(obj->Controller.Driver, controllerChild);
				}
				else if (controllerChildName == "PedestrianController")
				{
					LOG("%s is not implemented", objectChildName.c_str());

				}
			}
			else if (objectChildName == "Properties")
			{
				for (pugi::xml_node propertiesChild = objectChild.first_child(); propertiesChild; propertiesChild = propertiesChild.next_sibling())
				{
					Object::Property *property = new Object::Property;

					obj->properties_.push_back(property);

					property->name_ = ReadAttribute(propertiesChild.attribute("name"));
					property->value_ = ReadAttribute(propertiesChild.attribute("value"));
				}
			}
		}
		objectCnt++;
	}
}


void ScenarioReader::parseOSCPosition(roadmanager::Position &position, pugi::xml_node positionNode)
{
	LOG("Parsing OSCPosition");

	for (pugi::xml_node positionChild = positionNode.first_child(); positionChild; positionChild = positionChild.next_sibling())
	{
		std::string positionChildName(positionChild.name());

		if (positionChildName == "World")
		{
			LOG("%s is not implemented ", positionChildName.c_str());
		}
		else if (positionChildName == "RelativeWorld")
		{
			LOG("%s is not implemented ", positionChildName.c_str());
		}
		else if (positionChildName == "RelativeObject")
		{
			LOG("%s is not implemented ", positionChildName.c_str());
		}
		else if (positionChildName == "Road")
		{
			LOG("%s is not implemented ", positionChildName.c_str());
		}
		else if (positionChildName == "RelativeRoad")
		{
			LOG("%s is not implemented ", positionChildName.c_str());
		}
		else if (positionChildName == "Lane")
		{
			int road_id = std::stoi(ReadAttribute(positionChild.attribute("roadId")));
			int lane_id = std::stoi(ReadAttribute(positionChild.attribute("laneId")));
			double s = std::stod(ReadAttribute(positionChild.attribute("s")));
			double offset = std::stod(ReadAttribute(positionChild.attribute("offset")));

			position.SetLanePos(road_id, lane_id, s, offset);
				//std::stoi(ReadAttribute(positionChild.attribute("roadId"))),
				//std::stoi(ReadAttribute(positionChild.attribute("laneId"))),
				//std::stod(ReadAttribute(positionChild.attribute("s"))),
				//std::stod(ReadAttribute(positionChild.attribute("offset"))));
			position.Print();
		}
		else if (positionChildName == "RelativeLane")
		{
			LOG("%s is not implemented ", positionChildName.c_str());
		}
#if 0
		else if (positionChildName == "Route")
		{
			position.route_ = new OSCPosition::Route();
			for (pugi::xml_node routeChild = positionChild.first_child(); routeChild; routeChild = routeChild.next_sibling())
			{

				if (routeChild.name() == std::string("RouteRef"))
				{
					for (pugi::xml_node routeRefChild = routeChild.first_child(); routeRefChild; routeRefChild = routeRefChild.next_sibling())
					{
						std::string routeRefChildName(routeRefChild.name());

						if (routeRefChildName == "Route")
						{
							parseOSCRoute(position.route_->RouteRef.Route, routeRefChild);
						}
						else if (routeRefChildName == "CatalogReference")
						{
							parseOSCCatalogReference(position.route_->RouteRef.CatalogReference, routeRefChild);
						}
					}
				}
				if (routeChild.name() == std::string("Orientation"))
				{
					LOG("%s is not implemented", routeChild.name());
				}
				else if (routeChild.name() == std::string("Position"))
				{
					for (pugi::xml_node positionChild = routeChild.first_child(); positionChild; positionChild = positionChild.next_sibling())
					{
						std::string positionChildName(positionChild.name());

						if (positionChildName == "Current")
						{
							LOG("%s is not implemented", positionChildName.c_str());
						}
						else if (positionChildName == "RoadCoord")
						{
							LOG("%s is not implemented", positionChildName.c_str());
						}
						else if (positionChildName == "LaneCoord")
						{
							position.route_->Position.LaneCoord.pathS = std::stod(ReadAttribute(positionChild.attribute("pathS")));
							position.route_->Position.LaneCoord.laneId = std::stoi(ReadAttribute(positionChild.attribute("laneId")));

							pugi::xml_attribute laneOffsetAttribute = positionChild.attribute("laneOffset");
							if (laneOffsetAttribute != NULL)
							{
								position.route_->Position.LaneCoord.laneOffset = std::stod(ReadAttribute(positionChild.attribute("laneOffset")));
							}
						}
					}
				}
			}
		}
#endif
	}
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
		LOG("Dynamics shape %s not implemented", shape);
	}

	return OSCPrivateAction::DynamicsShape::UNDEFINED;
}

// ------------------------------------------------------
OSCPrivateAction *ScenarioReader::parseOSCPrivateAction(pugi::xml_node actionNode, Entities *entities, Object *object)
{
	LOG("Parsing OSCPrivateAction %s", actionNode.name());
	OSCPrivateAction *action = 0;

	for (pugi::xml_node actionChild = actionNode.first_child(); actionChild; actionChild = actionChild.next_sibling())
	{
		if (actionChild.name() == std::string("Longitudinal"))
		{
			for (pugi::xml_node longitudinalChild = actionChild.first_child(); longitudinalChild; longitudinalChild = longitudinalChild.next_sibling())
			{
				if (longitudinalChild.name() == std::string("Speed"))
				{
					LongSpeedAction *action_speed = new LongSpeedAction();

					for (pugi::xml_node speedChild = longitudinalChild.first_child(); speedChild; speedChild = speedChild.next_sibling())
					{
						if (speedChild.name() == std::string("Dynamics"))
						{
							action_speed->dynamics_.transition_. shape_ = ParseDynamicsShape(ReadAttribute(speedChild.attribute("shape")));
							
							if (ReadAttribute(speedChild.attribute("rate")) != "")
							{
								action_speed->dynamics_.timing_type_ = LongSpeedAction::Timing::RATE;
								action_speed->dynamics_.timing_target_value_ = std::stod(ReadAttribute(speedChild.attribute("rate")));
							}

							if (ReadAttribute(speedChild.attribute("time")) != "")
							{
								action_speed->dynamics_.timing_type_ = LongSpeedAction::Timing::TIME;
								action_speed->dynamics_.timing_target_value_ = std::stod(ReadAttribute(speedChild.attribute("time")));
							}

							if (ReadAttribute(speedChild.attribute("distance")) != "")
							{
								action_speed->dynamics_.timing_type_ = LongSpeedAction::Timing::DISTANCE;
								action_speed->dynamics_.timing_target_value_ = std::stod(ReadAttribute(speedChild.attribute("distance")));
							}
						}
						else if (speedChild.name() == std::string("Target"))
						{
							for (pugi::xml_node targetChild = speedChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("Relative"))
								{
									LongSpeedAction::TargetRelative *target_rel = new LongSpeedAction::TargetRelative;

									target_rel->value_ = stod(ReadAttribute(targetChild.attribute("value")));

									target_rel->continuous_ = (
										ReadAttribute(targetChild.attribute("continuous")) == "true" ||
										ReadAttribute(targetChild.attribute("continuous")) == "1");
									
									target_rel->object_ = FindObjectByName(ReadAttribute(targetChild.attribute("object")), entities);

									std::string value_type = ReadAttribute(targetChild.attribute("valueType"));
									if (value_type == "delta")
									{
										target_rel->valueType_ = LongSpeedAction::TargetRelative::ValueType::DELTA;
									}
									else if(value_type == "factor")
									{
										target_rel->valueType_ = LongSpeedAction::TargetRelative::ValueType::FACTOR;
									}
									else
									{
										LOG("Value type %s not valid", value_type);
									}
									action_speed->target_ = target_rel;
								}
								else if (targetChild.name() == std::string("Absolute"))
								{
									LongSpeedAction::TargetAbsolute *target_abs = new LongSpeedAction::TargetAbsolute;

									target_abs->value_ = stod(ReadAttribute(targetChild.attribute("value")));
									action_speed->target_ = target_abs;
								}
							}
						}
					}
					action = action_speed;
				}
				else if (longitudinalChild.name() == std::string("Distance"))
				{
					LOG("%s is not implemented", longitudinalChild.name());
				}
			}
		}
		else if (actionChild.name() == std::string("Lateral"))
		{
			for (pugi::xml_node lateralChild = actionChild.first_child(); lateralChild; lateralChild = lateralChild.next_sibling())
			{
				if (lateralChild.name() == std::string("LaneChange"))
				{
					LatLaneChangeAction *action_lane = new LatLaneChangeAction();

					if (ReadAttribute(lateralChild.attribute("targetLaneOffset")) != "")
					{
						action_lane->target_lane_offset_ = std::stod(ReadAttribute(lateralChild.attribute("targetLaneOffset")));
					}
					else
					{
						action_lane->target_lane_offset_ = 0;
					}

					for (pugi::xml_node laneChangeChild = lateralChild.first_child(); laneChangeChild; laneChangeChild = laneChangeChild.next_sibling())
					{
						if (laneChangeChild.name() == std::string("Dynamics"))
						{
							if (ReadAttribute(laneChangeChild.attribute("time")) != "")
							{
								action_lane->dynamics_.timing_type_ = LatLaneChangeAction::Timing::TIME;
								action_lane->dynamics_.timing_target_value_ = std::stod(ReadAttribute(laneChangeChild.attribute("time")));
							}
							else if (ReadAttribute(laneChangeChild.attribute("distance")) != "")
							{
								action_lane->dynamics_.timing_type_ = LatLaneChangeAction::Timing::DISTANCE;
								action_lane->dynamics_.timing_target_value_ = std::stod(ReadAttribute(laneChangeChild.attribute("distance")));
							}

							action_lane->dynamics_.transition_.shape_ = ParseDynamicsShape(ReadAttribute(laneChangeChild.attribute("shape")));
						}
						else if (laneChangeChild.name() == std::string("Target"))
						{
							LatLaneChangeAction::Target *target;

							for (pugi::xml_node targetChild = laneChangeChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("Relative"))
								{
									LatLaneChangeAction::TargetRelative *target_rel = new LatLaneChangeAction::TargetRelative;

									target_rel->object_ = FindObjectByName(ReadAttribute(targetChild.attribute("object")), entities);
									target_rel->value_ = std::stoi(ReadAttribute(targetChild.attribute("value")));
									target = target_rel;
								}
								else if (targetChild.name() == std::string("Absolute"))
								{
									LatLaneChangeAction::TargetAbsolute *target_abs = new LatLaneChangeAction::TargetAbsolute;

									target_abs->value_ = std::stoi(ReadAttribute(targetChild.attribute("value")));
									target = target_abs;
								}
							}
							action_lane->target_ = target;
						}
					}
					action = action_lane;
				}
				else if(lateralChild.name() == std::string("LaneOffset"))
				{
					LatLaneOffsetAction *action_lane = new LatLaneOffsetAction();
					for (pugi::xml_node laneOffsetChild = lateralChild.first_child(); laneOffsetChild; laneOffsetChild = laneOffsetChild.next_sibling())
					{
						if (laneOffsetChild.name() == std::string("Dynamics"))
						{
							if (ReadAttribute(laneOffsetChild.attribute("maxLateralAcc")) != "")
							{
								action_lane->dynamics_.max_lateral_acc_= std::stod(ReadAttribute(laneOffsetChild.attribute("maxLateralAcc")));
							}

							if (ReadAttribute(laneOffsetChild.attribute("duration")) != "")
							{
								action_lane->dynamics_.duration_ = std::stod(ReadAttribute(laneOffsetChild.attribute("duration")));
							}

							action_lane->dynamics_.transition_.shape_ = ParseDynamicsShape(ReadAttribute(laneOffsetChild.attribute("shape")));
						}
						else if (laneOffsetChild.name() == std::string("Target"))
						{
							LatLaneOffsetAction::Target *target;

							for (pugi::xml_node targetChild = laneOffsetChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								if (targetChild.name() == std::string("Relative"))
								{
									LatLaneOffsetAction::TargetRelative *target_rel = new LatLaneOffsetAction::TargetRelative;

									target_rel->object_ = FindObjectByName(ReadAttribute(targetChild.attribute("object")), entities);
									target_rel->value_ = std::stod(ReadAttribute(targetChild.attribute("value")));
									target = target_rel;
								}
								else if (targetChild.name() == std::string("Absolute"))
								{
									LatLaneOffsetAction::TargetAbsolute *target_abs = new LatLaneOffsetAction::TargetAbsolute;

									target_abs->value_ = std::stod(ReadAttribute(targetChild.attribute("value")));
									target = target_abs;
								}
							}
							action_lane->target_ = target;
						}
					}
					action = action_lane;
				}
				else
				{
					LOG("Unsupported element type: %s", lateralChild.name());
				}
			}
		}
		else if (actionChild.name() == std::string("Meeting"))
		{
			for (pugi::xml_node meetingChild = actionChild.first_child(); meetingChild; meetingChild = meetingChild.next_sibling())
			{
				roadmanager::Position *pos = new roadmanager::Position;
				if (meetingChild.name() == std::string("Position"))
				{
					parseOSCPosition(*pos, meetingChild);
				}
				else if (meetingChild.name() == std::string("Relative"))
				{
					MeetingRelativeAction *meeting_rel = new MeetingRelativeAction;
					meeting_rel->target_position_ = pos;

					std::string mode = ReadAttribute(meetingChild.attribute("mode"));
					if (mode == "straight")
					{
						meeting_rel->mode_ = MeetingRelativeAction::MeetingPositionMode::STRAIGHT;
					}
					else if (mode == "route")
					{
						meeting_rel->mode_ = MeetingRelativeAction::MeetingPositionMode::ROUTE;
					}
					else
					{
						LOG("mode %s invalid", mode);
					}

					meeting_rel->object_ = FindObjectByName(ReadAttribute(meetingChild.attribute("object")), entities);
					meeting_rel->continuous_ = (
						ReadAttribute(meetingChild.attribute("continuous")) == "true" ||
						ReadAttribute(meetingChild.attribute("continuous")) == "1");
					meeting_rel->offsetTime_ = std::stod(ReadAttribute(meetingChild.attribute("offsetTime")));

					roadmanager::Position *pos_object = new roadmanager::Position;
					pugi::xml_node pos_node = meetingChild.child("Position");
					if (pos_node != NULL)
					{
						parseOSCPosition(*pos_object, pos_node);
					}
					meeting_rel->object_target_position_ = pos_object;

					action = meeting_rel;
				}
				else if (meetingChild.name() == std::string("Absolute"))
				{
					MeetingAbsoluteAction *meeting_abs = new MeetingAbsoluteAction;
					meeting_abs->target_position_ = pos;
					meeting_abs->time_to_destination_ = std::stod(ReadAttribute(meetingChild.attribute("TimeToDestination")));

					action = meeting_abs;
				}
			}
		}
		else if (actionChild.name() == std::string("Position"))
		{
			PositionAction *action_pos = new PositionAction;
			roadmanager::Position pos;
			parseOSCPosition(pos, actionChild);
			action_pos->position_ = pos;
			action = action_pos;
		}
#if 0
		else if (actionChild.name() == std::string("Routing"))
		{
			for (pugi::xml_node routingChild = actionChild.first_child(); routingChild; routingChild = routingChild.next_sibling())
			{
				if (routingChild.name() == std::string("FollowRoute"))
				{
					for (pugi::xml_node followRouteChild = routingChild.first_child(); followRouteChild; followRouteChild = followRouteChild.next_sibling())
					{
						if (followRouteChild.name() == std::string("Route"))
						{
							LOG("%s is not implemented", followRouteChild.name());
						}
						else if (followRouteChild.name() == std::string("CatalogReference"))
						{
							action.routing_ = new OSCPrivateAction::Routing();
							parseOSCCatalogReference(action.routing_->FollowRoute.CatalogReference, followRouteChild);
						}
					}
				}
			}
		}
#endif
		else
		{
			LOG("%s is not supported", actionChild.name());

		}
	}
	action->object_ = object;

	return action;
}

Object* ScenarioReader::FindObjectByName(std::string name, Entities *entities)
{
	for (size_t i = 0; i < entities->object_.size(); i++)
	{
		if (name == entities->object_[i]->name_)
		{
			return entities->object_[i];
		}
	}

	return 0;
}


void ScenarioReader::parseInit(Init &init, Entities *entities)
{
	LOG("Parsing init");

	pugi::xml_node actionsNode = doc.child("OpenSCENARIO").child("Storyboard").child("Init").child("Actions");

	for (pugi::xml_node actionsChild = actionsNode.first_child(); actionsChild; actionsChild = actionsChild.next_sibling())
	{

		std::string actionsChildName(actionsChild.name());

		if (actionsChildName == "Global")
		{
			LOG("%s is not implemented", actionsChildName.c_str());

		}
		else if (actionsChildName == "UserDefined")
		{
			LOG("%s is not implemented", actionsChildName.c_str());

		}
		else if (actionsChildName == "Private")
		{
			Object *object;

			object = FindObjectByName(ReadAttribute(actionsChild.attribute("object")), entities);

			for (pugi::xml_node privateChild = actionsChild.first_child(); privateChild; privateChild = privateChild.next_sibling())
			{
				init.private_action_.push_back(parseOSCPrivateAction(privateChild, entities, object));
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
	else if (edge == "any")
	{
		return OSCCondition::ConditionEdge::ANY;
	}
	else
	{
		LOG("Unsupported edge: %s", edge);
	}

	return OSCCondition::ConditionEdge::UNDEFINED;
}

static Rule ParseRule(std::string rule)
{
	if (rule == "greater_than")
	{
		return Rule::GREATER_THAN;
	}
	else if (rule == "less_than")
	{
		return Rule::LESS_THAN;
	}
	else if (rule == "equal_to")
	{
		return Rule::EQUAL_TO;
	}
	else
	{
		LOG("Invalid rule %s", rule);
	}

	return Rule::UNDEFINED;
}

static TrigByState::StoryElementType ParseElementType(std::string element_type)
{
	if (element_type == "act")
	{
		return TrigByState::StoryElementType::ACT;
	}
	else if (element_type == "action")
	{
		return TrigByState::StoryElementType::ACTION;
	}
	else if (element_type == "scene")
	{
		return TrigByState::StoryElementType::SCENE;
	}
	else if (element_type == "maneuver")
	{
		return TrigByState::StoryElementType::MANEUVER;
	}
	else if (element_type == "event")
	{
		return TrigByState::StoryElementType::EVENT;
	}
	else if (element_type == "action")
	{
		return TrigByState::StoryElementType::ACTION;
	}
	else
	{
		LOG("Invalid element type %s", element_type);
	}

	return TrigByState::StoryElementType::UNDEFINED;
}
// ------------------------------------------
OSCCondition *ScenarioReader::parseOSCCondition(pugi::xml_node conditionNode, Entities *entities)
{
	LOG("Parsing OSCCondition");

	OSCCondition *condition;

	for (pugi::xml_node conditionChild = conditionNode.first_child(); conditionChild; conditionChild = conditionChild.next_sibling())
	{
		std::string conditionChildName(conditionChild.name());
		if (conditionChildName == "ByEntity")
		{
			pugi::xml_node entity_condition = conditionChild.child("EntityCondition");
			if (entity_condition != NULL)
			{
				for (pugi::xml_node condition_node = entity_condition.first_child(); condition_node; condition_node = condition_node.next_sibling())
				{
					std::string condition_type(condition_node.name());
					if (condition_type == "TimeHeadway")
					{
						TrigByTimeHeadway *trigger = new TrigByTimeHeadway;
						trigger->object_ = FindObjectByName(ReadAttribute(condition_node.attribute("entity")), entities);

						std::string along_route_str = ReadAttribute(condition_node.attribute("alongRoute"));
						if ((along_route_str == "true") || (along_route_str == "1"))
						{
							trigger->along_route_ = true;
						}
						else
						{
							trigger->along_route_ = false;
						}

						std::string freespace_str = ReadAttribute(condition_node.attribute("freespace"));
						if ((freespace_str == "true") || (freespace_str == "1"))
						{
							trigger->freespace_ = true;
						}
						else
						{
							trigger->freespace_ = false;
						}
						trigger->value_ = std::stod(ReadAttribute(condition_node.attribute("value")));
						trigger->rule_ = ParseRule(ReadAttribute(condition_node.attribute("rule")));

						condition = trigger;
					}
					else
					{
						LOG("Entity condition %s not supported", condition_type);
					}
				}
			}

			pugi::xml_node triggering_entities = conditionChild.child("TriggeringEntities");
			if (triggering_entities != NULL)
			{
				TrigByEntity *trigger = (TrigByEntity*)condition;				
				
				std::string trig_ent_rule = ReadAttribute(triggering_entities.attribute("rule"));
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
					LOG("Invalid triggering entity type: %s", trig_ent_rule);
				}

				for (pugi::xml_node triggeringEntitiesChild = triggering_entities.first_child(); triggeringEntitiesChild; triggeringEntitiesChild = triggeringEntitiesChild.next_sibling())
				{
					std::string triggeringEntitiesChildName(triggeringEntitiesChild.name());

					if (triggeringEntitiesChildName == "Entity")
					{
						TrigByEntity::Entity entity;
						entity.object_ = FindObjectByName(ReadAttribute(triggeringEntitiesChild.attribute("name")), entities);
						trigger->triggering_entities_.entity_.push_back(entity);
					}
				}
			}
		}
		else if (conditionChildName == "ByState")
		{
			for (pugi::xml_node byStateChild = conditionChild.first_child(); byStateChild; byStateChild = byStateChild.next_sibling())
			{
				std::string byStateChildName(byStateChild.name());

				if (byStateChildName == "AtStart")
				{
					TrigAtStart *trigger = new TrigAtStart;
					trigger->element_name_ = ReadAttribute(byStateChild.attribute("name"));
					trigger->element_type_ = ParseElementType(ReadAttribute(byStateChild.attribute("type")));
					condition = trigger;
				}
				else if (byStateChildName == "AfterTermination")
				{
					TrigAfterTermination *trigger = new TrigAfterTermination;
					trigger->element_name_ = ReadAttribute(byStateChild.attribute("name"));
					trigger->element_type_ = ParseElementType(ReadAttribute(byStateChild.attribute("type")));
					std::string term_rule = ReadAttribute(byStateChild.attribute("rule"));
					if (term_rule == "end")
					{
						trigger->rule_ = TrigAfterTermination::AfterTerminationRule::END;
					} 
					else if (term_rule == "cancel")
					{
						trigger->rule_ = TrigAfterTermination::AfterTerminationRule::CANCEL;
					}
					else if (term_rule == "any")
					{
						trigger->rule_ = TrigAfterTermination::AfterTerminationRule::ANY;
					}
					else
					{
						LOG("Invalid AfterTerminationRule %s", term_rule);
					}					
					condition = trigger;
				}
				else 
				{
					LOG("%s is not implemented", byStateChildName.c_str());
				}
			}
		}
		else if (conditionChildName == "ByValue")
		{
			for (pugi::xml_node byValueChild = conditionChild.first_child(); byValueChild; byValueChild = byValueChild.next_sibling())
			{
				std::string byValueChildName(byValueChild.name());
				if (byValueChildName == "SimulationTime")
				{
					TrigBySimulationTime *trigger = new TrigBySimulationTime;
					trigger->value_ = std::stod(ReadAttribute(byValueChild.attribute("value")));
					trigger->rule_ = ParseRule(ReadAttribute(byValueChild.attribute("rule")));
					condition = trigger;
				}
				else
				{
					LOG("TrigByValue %s not implemented", byValueChildName);
				}
			}
		}
	}
	condition->name_ = ReadAttribute(conditionNode.attribute("name"));
	condition->delay_ = std::stod(ReadAttribute(conditionNode.attribute("delay")));
	condition->edge_ = ParseConditionEdge(ReadAttribute(conditionNode.attribute("edge")));

	return condition;
}


void ScenarioReader::parseOSCManeuver(OSCManeuver *maneuver, pugi::xml_node maneuverNode, Entities *entities, ActSequence *sequence)
{
	LOG("Parsing OSCManeuver");

	maneuver->name_ = ReadAttribute(maneuverNode.attribute("name"));

	for (pugi::xml_node maneuverChild = maneuverNode.first_child(); maneuverChild; maneuverChild = maneuverChild.next_sibling())
	{
		std::string maneuverChildName(maneuverChild.name());

		if (maneuverChildName == "ParameterDeclaration")
		{
			LOG("%s is not implemented", maneuverChildName.c_str());
		}
		else if (maneuverChildName == "Event")
		{
			Event *event = new Event;

			event->name_ = ReadAttribute(maneuverChild.attribute("name"));
			std::string prio = ReadAttribute(maneuverChild.attribute("priority"));
			if (prio == "overwrite")
			{
				event->priority_ = Event::Priority::OVERWRITE;
			}
			else if (prio == "following")
			{
				event->priority_ = Event::Priority::FOLLOWING;
			}
			else if (prio == "skip")
			{
				event->priority_ = Event::Priority::SKIP;
			}
			else
			{
				LOG("Invalid priority: %s", prio.c_str());
			}

			for (pugi::xml_node eventChild = maneuverChild.first_child(); eventChild; eventChild = eventChild.next_sibling())
			{

				std::string childName(eventChild.name());

				if (childName == "Action")
				{
					for (pugi::xml_node actionChild = eventChild.first_child(); actionChild; actionChild = actionChild.next_sibling())
					{
						std::string childName(actionChild.name());

						if (childName == "Global")
						{
							LOG("%s is not implemented", childName.c_str());
						}
						else if (childName == "UserDefined")
						{
							LOG("%s is not implemented", childName.c_str());
						}
						else if (childName == "Private")
						{
							for (size_t i = 0; i < sequence->actor_.size(); i++)
							{
								OSCPrivateAction *action = parseOSCPrivateAction(actionChild, entities, sequence->actor_[i]->object_);
								event->action_.push_back((OSCAction*)action);
							}
						}
					}
				}
				else if (childName == "StartConditions")
				{
					for (pugi::xml_node startConditionsChild = eventChild.first_child(); startConditionsChild; startConditionsChild = startConditionsChild.next_sibling())
					{
						OSCConditionGroup *condition_group = new OSCConditionGroup;

						for (pugi::xml_node conditionGroupChild = startConditionsChild.first_child(); conditionGroupChild; conditionGroupChild = conditionGroupChild.next_sibling())
						{
							OSCCondition *condition = parseOSCCondition(conditionGroupChild, entities);
							condition_group->condition_.push_back(condition);
						}

						event->start_condition_group_.push_back(condition_group);
					}
				}

			}
			maneuver->event_.push_back(event);
		}
	}
}

void ScenarioReader::parseStory(std::vector<Story*> &storyVector, Entities *entities)
{
	LOG("Parsing Story");

	pugi::xml_node storyNode = doc.child("OpenSCENARIO").child("Storyboard").child("Story");

	for (; storyNode; storyNode = storyNode.next_sibling())
	{
		std::string storyNodeName(storyNode.name());

		if (storyNodeName == "Story")
		{
			Story *story = new Story;

			story->owner_ = ReadAttribute(storyNode.attribute("owner"));
			story->name_ = ReadAttribute(storyNode.attribute("name"));;

			addParameter("$owner", story->owner_);

			for (pugi::xml_node storyChild = storyNode.child("Act"); storyChild; storyChild = storyChild.next_sibling("Act"))
			{
				Act *act = new Act;

				act->name_ = ReadAttribute(storyChild.attribute("name"));

				for (pugi::xml_node actChild = storyChild.first_child(); actChild; actChild = actChild.next_sibling())
				{

					std::string childName(actChild.name());

					if (childName == "Sequence")
					{
						ActSequence *sequence = new ActSequence;

						sequence->number_of_executions_ = std::stoi(ReadAttribute(actChild.attribute("numberOfExecutions")));
						sequence->name_ = ReadAttribute(actChild.attribute("name"));

						pugi::xml_node actors_node = actChild.child("Actors");
						if (actors_node != NULL)
						{
							for (pugi::xml_node actorsChild = actors_node.first_child(); actorsChild; actorsChild = actorsChild.next_sibling())
							{
								ActSequence::Actor *actor = new ActSequence::Actor;

								std::string actorsChildName(actorsChild.name());
								if (actorsChildName == "Entity")
								{
									actor->object_ = FindObjectByName(ReadAttribute(actorsChild.attribute("name")), entities);
								}
								else if (actorsChildName == "ByCondition")
								{
									LOG("Actor by condition - not implemented");
								}
								sequence->actor_.push_back(actor);
							}
						}

						for (pugi::xml_node catalog_n = actChild.child("CatalogReference"); catalog_n; catalog_n = catalog_n.next_sibling("CatalogReference"))
						{
							LOG("Catalog reference not implemented yet (%s)", catalog_n.name());
						}

						for (pugi::xml_node maneuver_n = actChild.child("Maneuver"); maneuver_n; maneuver_n = maneuver_n.next_sibling("Maneuver"))
						if (maneuver_n != NULL)
						{
							OSCManeuver *maneuver = new OSCManeuver;

							parseOSCManeuver(maneuver, maneuver_n, entities, sequence);
							sequence->maneuver_.push_back(maneuver);
						}

						act->sequence_.push_back(sequence);
					}
					else if (childName == "Conditions")
					{
						for (pugi::xml_node conditionsChild = actChild.first_child(); conditionsChild; conditionsChild = conditionsChild.next_sibling())
						{
							std::string conditionsChildName(conditionsChild.name());
							if (conditionsChildName == "Start")
							{
								for (pugi::xml_node startChild = conditionsChild.first_child(); startChild; startChild = conditionsChild.next_sibling())
								{
									OSCConditionGroup *condition_group = new OSCConditionGroup;

									for (pugi::xml_node conditionGroupChild = startChild.first_child(); conditionGroupChild; conditionGroupChild = conditionGroupChild.next_sibling())
									{
										OSCCondition *condition = parseOSCCondition(conditionGroupChild, entities);
										condition_group->condition_.push_back(condition);
									}

									act->start_condition_group_.push_back(condition_group);
								}
							}
							else if (conditionsChildName == "End")
							{
								LOG("%s is not implemented", conditionsChildName.c_str());
							}
							else if (conditionsChildName == "Cancel")
							{
								LOG("%s is not implemented", conditionsChildName.c_str());
							}
						}
					}
				}
				story->act_.push_back(act);
			}
			storyVector.push_back(story);
		}
	}
}


