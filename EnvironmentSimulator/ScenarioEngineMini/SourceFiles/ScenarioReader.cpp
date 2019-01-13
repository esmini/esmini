#include "ScenarioReader.hpp"
#include "CommonMini.hpp"

using namespace scenarioengine;


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
	LOG("Failed to resolve parameter %s", name.c_str());
	return 0;
}

std::string ScenarioReader::ReadAttribute(pugi::xml_attribute attribute)
{
	if (attribute == 0)
	{
		LOG("Empty attribute");
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

int ScenarioReader::loadOSCFile(const char * path, ExternalControlMode ext_control)
{
	LOG("Loading %s", path);

	req_ext_control_ = ext_control;

	pugi::xml_parse_result result = doc.load_file(path);
	if (!result)
	{
		LOG("%s", result.description());
		return -1;
	}

	oscFilename = path;

	return 0;
}

void ScenarioReader::LoadCatalog(pugi::xml_node catalogChild, Catalogs *catalogs)
{
	pugi::xml_document catalog_doc;

	if (catalogChild.child("Directory") == NULL)
	{
		LOG("Catalog %s sub element Directory not found - skipping", catalogChild.name());
		return;
	}

	std::string filename = ReadAttribute(catalogChild.child("Directory").attribute("path"));

	// Filename should be relative the XOSC file
	std::string path = DirNameOf(oscFilename) + "/" + filename;
	pugi::xml_parse_result result = catalog_doc.load_file(path.c_str());

	if (!result)
	{
		LOG("Couldn't locate catalog file %s (%s + %s) - make sure path is relative xosc file", path.c_str(), DirNameOf(oscFilename).c_str(), filename.c_str());
		return;
	}
	
	pugi::xml_node catalog_node = catalog_doc.child("OpenSCENARIO").child("Catalog");
	
	std::string catalogsChildName(catalogChild.name());

	if (catalogsChildName == "RouteCatalog")
	{
		Catalog *catalog = new Catalog();
		catalog->name_ = ReadAttribute(catalog_node.attribute("name"));
		if (catalog->name_ == "")
		{
			LOG("Warning: Route catalog lacks name");
		}

		for (pugi::xml_node route_n = catalog_node.first_child(); route_n; route_n = route_n.next_sibling())
		{
			roadmanager::Route *route_item = parseOSCRoute(route_n, catalogs);
			catalog->AddEntry(new Entry(Entry::Type::ROUTE, route_item->getName(), (void*)route_item));
		}
		catalogs->AddCatalog(catalog);
	}
	else if (catalogsChildName == "VehicleCatalog")
	{
		Catalog *catalog = new Catalog();
		catalog->name_ = ReadAttribute(catalog_node.attribute("name"));
		if (catalog->name_ == "")
		{
			LOG("Warning: Vehicle catalog lacks name");
		}

		for (pugi::xml_node vehicle_n = catalog_node.first_child(); vehicle_n; vehicle_n = vehicle_n.next_sibling())
		{
			Vehicle *vehicle_item = parseOSCVehicle(vehicle_n, catalogs);
			catalog->AddEntry(new Entry(Entry::Type::VEHICLE, vehicle_item->name_, (void*)vehicle_item));
		}
		catalogs->AddCatalog(catalog);
	}
	else
	{
		LOG("Catalog type %s not supported yet", catalogsChildName.c_str());
	}

	return;
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

Vehicle* ScenarioReader::parseOSCVehicle(pugi::xml_node vehicleNode, Catalogs *catalogs)
{
	Vehicle *vehicle = new Vehicle();

	vehicle->name_ = ReadAttribute(vehicleNode.attribute("name"));
	LOG("Parsing Vehicle %s", vehicle->name_.c_str());
	vehicle->SetCategory(ReadAttribute(vehicleNode.attribute("category")));

	pugi::xml_node properties_node = vehicleNode.child("Properties");
	if (properties_node != NULL)
	{
		for (pugi::xml_node propertiesChild = properties_node.first_child(); propertiesChild; propertiesChild = propertiesChild.next_sibling())
		{
			std::string prop_name = ReadAttribute(propertiesChild.attribute("name"));
			std::string prop_value = ReadAttribute(propertiesChild.attribute("value"));

			// Check if the property is something supported
			if (prop_name == "control")
			{
				// check that external control has not been overridden
				if (req_ext_control_ == ExternalControlMode::EXT_CONTROL_BY_OSC)
				{
					if (prop_value == "external")
					{
						vehicle->extern_control_ = true;
					}
					else
					{
						vehicle->extern_control_ = false;
					}
				}
			}
			else if (prop_name == "model_id")
			{
				vehicle->model_id_ = stoi(prop_value);
			}
			else
			{
				LOG("Unsupported property: %s", prop_name.c_str());
			}
		}
	}


	return vehicle;
}

roadmanager::Route* ScenarioReader::parseOSCRoute(pugi::xml_node routeNode, Catalogs *catalogs)
{
	roadmanager::Route *route = new roadmanager::Route;

	route->setName(ReadAttribute(routeNode.attribute("name")));

	LOG("Parsing OSCRoute %s", route->getName().c_str());

	// Closed attribute not supported by roadmanager yet
	std::string closed_str = ReadAttribute(routeNode.attribute("closed"));
	bool closed = false;
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
			roadmanager::Position *pos = new roadmanager::Position;
			parseOSCPosition(*pos, routeChild.first_child(), catalogs);
			route->AddWaypoint(pos);
		}
	}
	LOG("parseOSCRoute finished");

	return route;
}

void ScenarioReader::parseCatalogs(Catalogs &catalogs) 
{
	LOG("Parsing Catalogs");

	pugi::xml_node catalogsNode = doc.child("OpenSCENARIO").child("Catalogs");

	for (pugi::xml_node catalogsChild = catalogsNode.first_child(); catalogsChild; catalogsChild = catalogsChild.next_sibling())
	{
		LoadCatalog(catalogsChild, &catalogs);
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

void ScenarioReader::parseEntities(Entities &entities, Catalogs *catalogs)
{
	LOG("Parsing Entities");

	pugi::xml_node enitiesNode = doc.child("OpenSCENARIO").child("Entities");

	for (pugi::xml_node entitiesChild = enitiesNode.first_child(); entitiesChild; entitiesChild = entitiesChild.next_sibling())
	{
		Object *obj = 0;

		for (pugi::xml_node objectChild = entitiesChild.first_child(); objectChild; objectChild = objectChild.next_sibling())
		{
			std::string objectChildName(objectChild.name());

			if (objectChildName == "CatalogReference")
			{
				Entry *entry = catalogs->FindCatalogEntry(ReadAttribute(objectChild.attribute("catalogName")), ReadAttribute(objectChild.attribute("entryName")));
				if (entry == 0)
				{
					LOG("Failed to look up catalog entry %s, %s", 
						ReadAttribute(objectChild.attribute("catalogName")).c_str(), ReadAttribute(objectChild.attribute("entryName")).c_str());
				}
				else if (entry->type_ == Entry::Type::VEHICLE)
				{
					// Make a new instance from catalog entry 
					obj = new Vehicle(*(Vehicle*)entry->GetElement());
				}
				else
				{
					LOG("Entity of type %s not supported yet", entry->Type2Str(entry->type_).c_str());
				}
			}
			else if (objectChildName == "Vehicle")
			{				
				Vehicle *vehicle = parseOSCVehicle(objectChild, catalogs);
				obj = vehicle;
			}
			else
			{
				LOG("%s not supported yet", objectChildName.c_str());
			}
		}
		if (obj != 0)
		{
			obj->name_ = ReadAttribute(entitiesChild.attribute("name"));
			obj->id_ = (int)entities.object_.size();
			entities.object_.push_back(obj);
			objectCnt++;
		}
	}
}


void ScenarioReader::parseOSCPosition(roadmanager::Position &position, pugi::xml_node positionNode, Catalogs *catalogs)
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
			double offset = 0;  // Default value of optional parameter
			if (positionChild.attribute("offset"))
			{
				offset = std::stod(ReadAttribute(positionChild.attribute("offset")));
			}

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
		else if (positionChildName == "Route")
		{
			pugi::xml_node route_ref = positionChild.child("RouteRef");
			if (route_ref != NULL)
			{

			}
			for (pugi::xml_node routeChild = positionChild.first_child(); routeChild; routeChild = routeChild.next_sibling())
			{

				if (routeChild.name() == std::string("RouteRef"))
				{
					for (pugi::xml_node routeRefChild = routeChild.first_child(); routeRefChild; routeRefChild = routeRefChild.next_sibling())
					{
						std::string routeRefChildName(routeRefChild.name());
						roadmanager::Route *route = 0;

						if (routeRefChildName == "Route")
						{
							// Add inline route to route catalog
							LOG("Inline route reference not supported yet - put the route into a catalog");
						}
						else if (routeRefChildName == "CatalogReference")
						{
							// Find route in catalog
							route = (roadmanager::Route*)catalogs->FindCatalogElement(ReadAttribute(routeRefChild.attribute("catalogName")), ReadAttribute(routeRefChild.attribute("entryName")));

							if(route == 0)
							{
								LOG("Couldn't find route %s", ReadAttribute(routeRefChild.attribute("entryName")).c_str());
								return;
							}
						}
						
						position.SetRoute(route);
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
							double s = std::stod(ReadAttribute(positionChild.attribute("pathS")));
							int lane_id = std::stoi(ReadAttribute(positionChild.attribute("laneId")));
							double lane_offset = 0;

							pugi::xml_attribute laneOffsetAttribute = positionChild.attribute("laneOffset");
							if (laneOffsetAttribute != NULL)
							{
								lane_offset = std::stod(ReadAttribute(positionChild.attribute("laneOffset")));
							}

							position.SetRouteLanePosition(s, lane_id, lane_offset);
						}
					}
				}
			}
		}
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
		LOG("Dynamics shape %s not implemented", shape.c_str());
	}

	return OSCPrivateAction::DynamicsShape::UNDEFINED;
}

// ------------------------------------------------------
OSCPrivateAction *ScenarioReader::parseOSCPrivateAction(pugi::xml_node actionNode, Entities *entities, Object *object, Catalogs *catalogs)
{
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
										target_rel->value_type_ = LongSpeedAction::TargetRelative::ValueType::DELTA;
									}
									else if(value_type == "factor")
									{
										target_rel->value_type_ = LongSpeedAction::TargetRelative::ValueType::FACTOR;
									}
									else
									{
										LOG("Value type %s not valid", value_type.c_str());
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
					LongDistanceAction *action_dist = new LongDistanceAction();

					pugi::xml_node dynamics_node = longitudinalChild.child("Dynamics");
					if (dynamics_node != NULL)
					{
						if (dynamics_node.child("None"))
						{
							action_dist->dynamics_.none_ = true;
						}
						else
						{
							pugi::xml_node limits_node = dynamics_node.child("Limited");
							if (limits_node != NULL)
							{
								action_dist->dynamics_.max_acceleration_ = stod(ReadAttribute(limits_node.attribute("maxAcceleration")));
								action_dist->dynamics_.max_deceleration_ = stod(ReadAttribute(limits_node.attribute("maxDeceleration")));
								action_dist->dynamics_.max_speed_ = stod(ReadAttribute(limits_node.attribute("maxSpeed")));
							}
							else
							{
								LOG("Limited element missing");
							}
						}
					}
					else
					{
						LOG("Missing child \"Dynamics\"");
					}
					
					action_dist->target_object_ = FindObjectByName(ReadAttribute(longitudinalChild.attribute("object")), entities);
					if (longitudinalChild.attribute("distance"))
					{
						action_dist->dist_type_ = LongDistanceAction::DistType::DISTANCE;
						action_dist->distance_ = stod(ReadAttribute(longitudinalChild.attribute("distance")));
					}
					else if (longitudinalChild.attribute("timeGap"))
					{
						action_dist->dist_type_ = LongDistanceAction::DistType::TIME_GAP;
						action_dist->distance_ = stod(ReadAttribute(longitudinalChild.attribute("timeGap")));
					}
					else
					{
						LOG("Need distance or timeGap");
					}
					std::string freespace = ReadAttribute(longitudinalChild.attribute("freespace"));
					if (freespace == "true" || freespace == "1") action_dist->freespace_ = true;
					else action_dist->freespace_ = false;

					action = action_dist;
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
			roadmanager::Position *pos = new roadmanager::Position;

			pugi::xml_node pos_child = actionChild.child("Position");
			if (pos_child)
			{
				parseOSCPosition(*pos, pos_child, catalogs);
			}

			pugi::xml_node rel_child = actionChild.child("Relative");
			if (rel_child)
			{
				MeetingRelativeAction *meeting_rel = new MeetingRelativeAction;

				meeting_rel->own_target_position_ = pos;

				std::string mode = ReadAttribute(rel_child.attribute("mode"));
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
					LOG("mode %s invalid", mode.c_str());
				}

				meeting_rel->relative_object_ = FindObjectByName(ReadAttribute(rel_child.attribute("object")), entities);
				meeting_rel->continuous_ = (
					ReadAttribute(rel_child.attribute("continuous")) == "true" ||
					ReadAttribute(rel_child.attribute("continuous")) == "1");
				meeting_rel->offsetTime_ = std::stod(ReadAttribute(rel_child.attribute("offsetTime")));

				roadmanager::Position *pos_relative_object = new roadmanager::Position;
				pugi::xml_node pos_node = rel_child.child("Position");
				if (pos_node != NULL)
				{
					parseOSCPosition(*pos_relative_object, pos_node, catalogs);
				}
				meeting_rel->relative_target_position_ = pos_relative_object;

				action = meeting_rel;
			} 
			else
			{
				pugi::xml_node abs_child = actionChild.child("Absolute");
				if (abs_child)
				{
					MeetingAbsoluteAction *meeting_abs = new MeetingAbsoluteAction;
					meeting_abs->target_position_ = pos;
					meeting_abs->time_to_destination_ = std::stod(ReadAttribute(abs_child.attribute("TimeToDestination")));

					action = meeting_abs;
				}
			}
		}
		else if (actionChild.name() == std::string("Position"))
		{
			PositionAction *action_pos = new PositionAction;
			roadmanager::Position pos;
			parseOSCPosition(pos, actionChild, catalogs);
			action_pos->position_ = pos;
			action = action_pos;
		}
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
							FollowRouteAction *action_follow_route = new FollowRouteAction;
							
							// Find route in catalog
							roadmanager::Route *route = (roadmanager::Route*)catalogs->FindCatalogEntry(ReadAttribute(followRouteChild.attribute("catalogName")), ReadAttribute(followRouteChild.attribute("entryName")));

							if(route != 0)
							{
								action_follow_route->route_ = route;
									
								action = action_follow_route;
								break;
							}
							else
							{
								LOG("Route %s, %s not found", ReadAttribute(followRouteChild.attribute("catalogName")).c_str(), ReadAttribute(followRouteChild.attribute("entryName")).c_str());
							}
						}
					}
				}
			}
		}
		else
		{
			LOG("%s is not supported", actionChild.name());

		}
	}

	if (action != 0)
	{
		action->name_ = ReadAttribute(actionNode.parent().attribute("name"));
		action->object_ = object;
	}

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

	LOG("Failed to find object %s", name.c_str());
	return 0;
}


void ScenarioReader::parseInit(Init &init, Entities *entities, Catalogs *catalogs)
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
			if (object != NULL)
			{
				for (pugi::xml_node privateChild = actionsChild.first_child(); privateChild; privateChild = privateChild.next_sibling())
				{
					OSCPrivateAction *action = parseOSCPrivateAction(privateChild, entities, object, catalogs);
					action->name_ = "Init " + object->name_ + " " + privateChild.first_child().name();
					init.private_action_.push_back(action);
				}
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
		LOG("Unsupported edge: %s", edge.c_str());
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
		LOG("Invalid rule %s", rule.c_str());
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
		LOG("Invalid element type %s", element_type.c_str());
	}

	return TrigByState::StoryElementType::UNDEFINED;
}
// ------------------------------------------
OSCCondition *ScenarioReader::parseOSCCondition(pugi::xml_node conditionNode, Entities *entities, Catalogs *catalogs)
{
	LOG("Parsing OSCCondition %s", ReadAttribute(conditionNode.attribute("name")).c_str());

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
					else if (condition_type == "ReachPosition")
					{
						TrigByReachPosition *trigger = new TrigByReachPosition;

						if (!condition_node.attribute("tolerance"))
						{
							LOG("tolerance is required");
						}
						else
						{
							trigger->tolerance_ = stod(ReadAttribute(condition_node.attribute("tolerance")));
						}

						// Read position
						pugi::xml_node pos_node = condition_node.child("Position");
						parseOSCPosition(trigger->position_, pos_node, catalogs);

						condition = trigger;
					}
					else if (condition_type == "RelativeDistance")
					{
						TrigByRelativeDistance *trigger = new TrigByRelativeDistance;
						trigger->object_ = FindObjectByName(ReadAttribute(condition_node.attribute("entity")), entities);

						std::string type = ReadAttribute(condition_node.attribute("type"));
						if ((type == "longitudinal") || (type == "Longitudinal"))
						{
							trigger->type_ = TrigByRelativeDistance::RelativeDistanceType::LONGITUDINAL;
						}
						else if ((type == "lateral") || (type == "Lateral"))
						{
							trigger->type_ = TrigByRelativeDistance::RelativeDistanceType::LATERAL;
						}
						else if ((type == "interial") || (type == "Intertial"))
						{
							trigger->type_ = TrigByRelativeDistance::RelativeDistanceType::INTERIAL;
						}
						else
						{
							LOG("Unknown RelativeDistance condition type: %s", type.c_str());
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
						LOG("Entity condition %s not supported", condition_type.c_str());
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
					LOG("Invalid triggering entity type: %s", trig_ent_rule.c_str());
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
					trigger->element_type_ = ParseElementType(ReadAttribute(byStateChild.attribute("type")));
					trigger->element_name_ = ReadAttribute(byStateChild.attribute("name"));
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
						LOG("Invalid AfterTerminationRule %s", term_rule.c_str());
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
					LOG("TrigByValue %s not implemented", byValueChildName.c_str());
				}
			}
		}
		else
		{
			LOG("Condition %s not supported\n", conditionChildName.c_str());
		}
	}
	condition->name_ = ReadAttribute(conditionNode.attribute("name"));
	if (conditionNode.attribute("delay") != NULL)
	{
		condition->delay_ = std::stod(ReadAttribute(conditionNode.attribute("delay")));
	}
	else
	{
		LOG("Attribute \"delay\" missing");
	}

	std::string edge_str = ReadAttribute(conditionNode.attribute("edge"));
	if (edge_str != "")
	{
		condition->edge_ = ParseConditionEdge(edge_str);
	}

	return condition;
}


void ScenarioReader::parseOSCManeuver(OSCManeuver *maneuver, pugi::xml_node maneuverNode, Entities *entities, ActSequence *act_sequence, Catalogs *catalogs)
{
	maneuver->name_ = ReadAttribute(maneuverNode.attribute("name"));
	LOG("Parsing OSCManeuver %s", maneuver->name_.c_str());

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
			LOG("Parsing Event %s", event->name_.c_str());

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
							for (size_t i = 0; i < act_sequence->actor_.size(); i++)
							{
								LOG("Parsing private action %s", ReadAttribute(eventChild.attribute("name")).c_str());
								OSCPrivateAction *action = parseOSCPrivateAction(actionChild, entities, act_sequence->actor_[i]->object_, catalogs);
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
							OSCCondition *condition = parseOSCCondition(conditionGroupChild, entities, catalogs);
							condition_group->condition_.push_back(condition);
						}

						event->start_condition_group_.push_back(condition_group);
					}
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

void ScenarioReader::parseStory(std::vector<Story*> &storyVector, Entities *entities, Catalogs *catalogs)
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

							parseOSCManeuver(maneuver, maneuver_n, entities, sequence, catalogs);
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
								for (pugi::xml_node startChild = conditionsChild.first_child(); startChild; startChild = startChild.next_sibling())
								{
									OSCConditionGroup *condition_group = new OSCConditionGroup;

									for (pugi::xml_node conditionGroupChild = startChild.first_child(); conditionGroupChild; conditionGroupChild = conditionGroupChild.next_sibling())
									{
										OSCCondition *condition = parseOSCCondition(conditionGroupChild, entities, catalogs);
										condition_group->condition_.push_back(condition);
									}

									act->start_condition_group_.push_back(condition_group);
								}
							}
							else if (conditionsChildName == "End")
							{
								for (pugi::xml_node endChild = conditionsChild.first_child(); endChild; endChild = conditionsChild.next_sibling())
								{
									OSCConditionGroup *condition_group = new OSCConditionGroup;

									for (pugi::xml_node conditionGroupChild = endChild.first_child(); conditionGroupChild; conditionGroupChild = conditionGroupChild.next_sibling())
									{
										OSCCondition *condition = parseOSCCondition(conditionGroupChild, entities, catalogs);
										condition_group->condition_.push_back(condition);
									}

									act->end_condition_group_.push_back(condition_group);
								}
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


