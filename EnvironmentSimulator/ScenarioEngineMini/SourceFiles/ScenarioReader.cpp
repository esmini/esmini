#include "ScenarioReader.hpp"


ScenarioReader::ScenarioReader()
{
	std::cout << "ScenarioReader: ScenarioReader started" << std::endl;
	doc;
	objectCnt = 0;
	std::cout << "ScenarioReader: ScenarioReader finished" << std::endl;
}

void ScenarioReader::addParameter(std::string name, std::string value)
{
	std::cout << "ScenarioReader: addParameter started" << std::endl;

	parameterDeclaration.Parameter.resize(parameterDeclaration.Parameter.size() + 1);

	parameterDeclaration.Parameter.back().name = name;
	parameterDeclaration.Parameter.back().type = "string";
	parameterDeclaration.Parameter.back().value = value;

	std::cout << "ScenarioReader: " << name << " = " << value << " addeed to parameterDeclaration" << std::endl;

	std::cout << "ScenarioReader: addParameter finished" << std::endl;
}

std::string ScenarioReader::getParameter(std::string name)
{
	std::cout << "ScenarioReader: getParameter started" << std::endl;

	// If string already present in parameterDeclaration
	for (size_t i = 0; i < parameterDeclaration.Parameter.size(); i++)
	{
		if (parameterDeclaration.Parameter[i].name == name)
		{
			std::cout << "ScenarioReader: " << name << " replaced with " << parameterDeclaration.Parameter[i].value << std::endl;
			return parameterDeclaration.Parameter[i].value;
		}
	}

	std::cout << "ScenarioReader: getParameter finished" << std::endl;
	return 0;
}

std::string ScenarioReader::ReadAttribute(pugi::xml_attribute attribute)
{
	if (attribute == 0)
	{
		printf("ReadAttribute: NULL\n");
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
	std::cout << "ScenarioReader: loadOSCFile started. Loading " << path << std::endl;

	pugi::xml_parse_result result = doc.load_file(path);
	if (!result)
	{
		return -1;
	}

	std::cout << "ScenarioReader: loadOSCFile finished" << std::endl;

	return 0;
}

void ScenarioReader::parseParameterDeclaration()
{
	std::cout << "ScenarioReader: parseParameterDeclaration started" << std::endl;

	pugi::xml_node parameterDeclarationNode = doc.child("OpenSCENARIO").child("ParameterDeclaration");
	
	for (pugi::xml_node parameterDeclarationChild = parameterDeclarationNode.first_child(); parameterDeclarationChild; parameterDeclarationChild = parameterDeclarationChild.next_sibling())
	{
		parameterDeclaration.Parameter.resize(parameterDeclaration.Parameter.size() + 1);

		parameterDeclaration.Parameter.back().name = parameterDeclarationChild.attribute("name").value();
		parameterDeclaration.Parameter.back().type = parameterDeclarationChild.attribute("type").value();
		parameterDeclaration.Parameter.back().value = parameterDeclarationChild.attribute("value").value();
	}
	std::cout << "ScenarioReader: parseParameterDeclaration finished" << std::endl;
}

void ScenarioReader::parseRoadNetwork(RoadNetwork &roadNetwork)
{
	std::cout << "ScenarioReader: parseRoadNetwork started" << std::endl;

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

	std::cout << "ScenarioReader: parseRoadNetwork finished" << std::endl;
}

void ScenarioReader::parseOSCRoute(OSCRoute &route, pugi::xml_node routeNode)
{
	std::cout << "ScenarioReader: parseOSCRoute started" << std::endl;

	route.name = ReadAttribute(routeNode.attribute("name"));
	route.closed = ReadAttribute(routeNode.attribute("closed"));

	for (pugi::xml_node routeChild = routeNode.first_child(); routeChild; routeChild = routeChild.next_sibling())
	{
		std::string routeChildName(routeChild.name());

		if (routeChildName == "ParameterDeclaration")
		{
			std::cout << routeChildName << " is not implemented " << std::endl;

		}
		else if (routeChildName == "Waypoint")
		{
			route.Waypoint.resize(route.Waypoint.size() + 1);

			route.Waypoint.back().strategy = ReadAttribute(routeChild.attribute("strategy"));

			route.Waypoint.back().Position = new OSCPosition;

			parseOSCPosition(*(route.Waypoint.back().Position), routeChild.first_child());

		}
	}
	std::cout << "ScenarioReader: parseOSCRoute finished" << std::endl;
}

void ScenarioReader::parseCatalogs(Catalogs &catalogs) 
{
	std::cout << "ScenarioReader: parseCatalogs started" << std::endl;

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

	std::cout << "ScenarioReader: parseRoadNetwork finished" << std::endl;
}

void ScenarioReader::parseOSCFile(OSCFile &file, pugi::xml_node fileNode)
{
	std::cout << "ScenarioReader: parseOSCFile started" << std::endl;

	file.filepath = ReadAttribute(fileNode.attribute("filepath"));

	std::cout << "ScenarioReader: parseOSCFile finished" << std::endl;
}

void ScenarioReader::parseOSCBoundingBox(OSCBoundingBox &boundingBox, pugi::xml_node boundingBoxNode)
{
	std::cout << "ScenarioReader: parseOSCBoundingBox started" << std::endl;

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
	}
	std::cout << "ScenarioReader: parseOSCBoundingBox finshed" << std::endl;
}


void ScenarioReader::parseOSCPersonDescription(OSCPersonDescription &personDescription, pugi::xml_node descriptionNode)
{
	std::cout << "ScenarioReader: parseOSCPersonDescription started" << std::endl;

	if (descriptionNode.child("Properties"))
	{
		std::cout << "Properties" << " is not implemented " << std::endl;
	}

	personDescription.weight = std::stod(ReadAttribute(descriptionNode.attribute("weight")));
	personDescription.height = std::stod(ReadAttribute(descriptionNode.attribute("height")));
	personDescription.eyeDistance = std::stod(ReadAttribute(descriptionNode.attribute("eyeDistance")));
	personDescription.age = std::stod(ReadAttribute(descriptionNode.attribute("age")));
	personDescription.sex = ReadAttribute(descriptionNode.attribute("sex"));

	std::cout << "ScenarioReader: parseOSCPersonDescription finshed" << std::endl;
}



void ScenarioReader::parseOSCAxle(OSCAxle &axle, pugi::xml_node axleNode)
{
	std::cout << "ScenarioReader: parseOSCAxle started" << std::endl;
	axle.maxSteering = std::stod(ReadAttribute(axleNode.attribute("maxSteering")));
	axle.wheelDiameter = std::stod(ReadAttribute(axleNode.attribute("wheelDiameter")));
	axle.trackWidth = std::stod(ReadAttribute(axleNode.attribute("trackWidth")));
	axle.positionX = std::stod(ReadAttribute(axleNode.attribute("positionX")));
	axle.positionZ = std::stod(ReadAttribute(axleNode.attribute("positionZ")));

	std::cout << "ScenarioReader: parseOSCAxle finshed" << std::endl;
}


void ScenarioReader::parseOSCDriver(OSCDriver &driver, pugi::xml_node driverNode)
{
	std::cout << "ScenarioReader: parseOSCDriver started" << std::endl;

	driver.name = ReadAttribute(driverNode.attribute("name"));

	for (pugi::xml_node driverChild = driverNode.first_child(); driverChild; driverChild = driverChild.next_sibling())
	{
		std::string driverChildName(driverChild.name());

		if (driverChildName == "ParameterDeclaration")
		{
			std::cout << driverChildName << " is not implemented " << std::endl;
		}
		if (driverChildName == "Description")
		{
			parseOSCPersonDescription(driver.Description, driverChild);
		}
	}

	std::cout << "ScenarioReader: parseOSCDriver finshed" << std::endl;
}


void ScenarioReader::parseOSCVehicle(OSCVehicle &vehicle, pugi::xml_node vehicleNode)
{
	std::cout << "ScenarioReader: parseOSCVehicle started" << std::endl;

	vehicle.name = ReadAttribute(vehicleNode.attribute("name"));
	vehicle.category = ReadAttribute(vehicleNode.attribute("category"));

	for (pugi::xml_node vehicleChild = vehicleNode.first_child(); vehicleChild; vehicleChild = vehicleChild.next_sibling())
	{
		std::string vehicleChildName(vehicleChild.name());

		if (vehicleChildName == "ParameterDeclaration")
		{
			std::cout << vehicleChildName << " is not implemented " << std::endl;
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
					std::cout << axlesChildName << " is not implemented " << std::endl;
				}

			}
		}
		else if (vehicleChildName == "Properties")
		{
			std::cout << vehicleChildName << " is not implemented " << std::endl;
		}

	}

	std::cout << "ScenarioReader: parseOSCVehicle finished" << std::endl;
}

void ScenarioReader::parseOSCCatalogReference(OSCCatalogReference &catalogReference, pugi::xml_node catalogReferenceNode)
{

	catalogReference.catalogName = ReadAttribute(catalogReferenceNode.attribute("catalogName"));
	catalogReference.entryName = ReadAttribute(catalogReferenceNode.attribute("entryName"));

	pugi::xml_node catalogReferenceChild = catalogReferenceNode.first_child();
	std::string catalogReferenceChildName(catalogReferenceChild.name());

	if (catalogReferenceChildName == "ParameterAssignment")
	{
		std::cout << catalogReferenceChildName << " is not implemented " << std::endl;
	}
}

void ScenarioReader::parseEntities(Entities &entities)
{
	std::cout << "ScenarioReader: parseEntities started" << std::endl;

	pugi::xml_node enitiesNode = doc.child("OpenSCENARIO").child("Entities");

	for (pugi::xml_node entitiesChild = enitiesNode.first_child(); entitiesChild; entitiesChild = entitiesChild.next_sibling())
	{
		entities.Object.resize(entities.Object.size() + 1);

		entities.Object.back().name = ReadAttribute(entitiesChild.attribute("name"));

		for (pugi::xml_node objectChild = entitiesChild.first_child(); objectChild; objectChild = objectChild.next_sibling())
		{
			std::string objectChildName(objectChild.name());

			if (objectChildName == "CatalogReference")
			{
				parseOSCCatalogReference(entities.Object.back().CatalogReference, objectChild);
			}
			else if (objectChildName == "Vehicle")
			{
				parseOSCVehicle(entities.Object.back().Vehicle, objectChild);

			}
			else if (objectChildName == "Pedestrian")
			{
				std::cout << objectChildName << " is not implemented " << std::endl;

			}
			else if (objectChildName == "MiscObject")
			{
				std::cout << objectChildName << " is not implemented " << std::endl;

			}
			else if (objectChildName == "Controller")
			{
				pugi::xml_node controllerChild = objectChild.first_child();
				std::string controllerChildName(controllerChild.name());

				if (controllerChildName == "CatalogReference")
				{
					std::cout << objectChildName << " is not implemented " << std::endl;

				}
				else if (controllerChildName == "Driver")
				{
					parseOSCDriver(entities.Object.back().Controller.Driver, controllerChild);
				}
				else if (controllerChildName == "PedestrianController")
				{
					std::cout << objectChildName << " is not implemented " << std::endl;

				}
			}
			else if (objectChildName == "Properties")
			{
				for (pugi::xml_node propertiesChild = objectChild.first_child(); propertiesChild; propertiesChild = propertiesChild.next_sibling())
				{
					std::string propertiesChildName(propertiesChild.name());

					entities.Object.back().Properties.resize(entities.Object.back().Properties.size() + 1);
					entities.Object.back().Properties.back().Property.name = ReadAttribute(propertiesChild.attribute("name"));
					entities.Object.back().Properties.back().Property.value = ReadAttribute(propertiesChild.attribute("value"));
				}
			}
		}
		objectCnt++;
	}

	std::cout << "ScenarioReader: parseEntities finshed" << std::endl;
}


void ScenarioReader::parseOSCPosition(OSCPosition &position, pugi::xml_node positionNode)
{
	std::cout << "ScenarioReader: parseOSCPosition started" << std::endl;

	for (pugi::xml_node positionChild = positionNode.first_child(); positionChild; positionChild = positionChild.next_sibling())
	{
		std::string positionChildName(positionChild.name());

		if (positionChildName == "World")
		{
			std::cout << positionChildName << " is not implemented " << std::endl;
		}
		else if (positionChildName == "RelativeWorld")
		{
			std::cout << positionChildName << " is not implemented " << std::endl;
		}
		else if (positionChildName == "RelativeObject")
		{
			std::cout << positionChildName << " is not implemented " << std::endl;
		}
		else if (positionChildName == "Road")
		{
			std::cout << positionChildName << " is not implemented " << std::endl;
		}
		else if (positionChildName == "RelativeRoad")
		{
			std::cout << positionChildName << " is not implemented " << std::endl;
		}
		else if (positionChildName == "Lane")
		{
			position.Lane.exists = true;

			if (positionChild.child("Orientation"))
			{
				std::cout << "Orientation" << " is not implemented " << std::endl;
			}

			position.Lane.roadId = ReadAttribute(positionChild.attribute("roadId"));
			position.Lane.laneId = std::stoi(ReadAttribute(positionChild.attribute("laneId")));
			position.Lane.offset = std::stod(ReadAttribute(positionChild.attribute("offset")));
			position.Lane.s = std::stod(ReadAttribute(positionChild.attribute("s")));
		}
		else if (positionChildName == "RelativeLane")
		{
			std::cout << positionChildName << " is not implemented " << std::endl;
		}
		else if (positionChildName == "Route")
		{

			for (pugi::xml_node routeChild = positionChild.first_child(); routeChild; routeChild = routeChild.next_sibling())
			{
				std::string routeChildName(routeChild.name());

				if (routeChildName == "RouteRef")
				{
					for (pugi::xml_node routeRefChild = routeChild.first_child(); routeRefChild; routeRefChild = routeRefChild.next_sibling())
					{
						std::string routeRefChildName(routeRefChild.name());

						if (routeRefChildName == "Route")
						{
							parseOSCRoute(position.Route.RouteRef.Route, routeRefChild);
						}
						else if (routeRefChildName == "CatalogReference")
						{
							parseOSCCatalogReference(position.Route.RouteRef.CatalogReference, routeRefChild);
						}
					}
				}
				if (routeChildName == "Orientation")
				{
					std::cout << routeChildName << " is not implemented " << std::endl;
				}
				else if (routeChildName == "Position")
				{
					for (pugi::xml_node positionChild = routeChild.first_child(); positionChild; positionChild = positionChild.next_sibling())
					{
						std::string positionChildName(positionChild.name());

						if (positionChildName == "Current")
						{
							std::cout << positionChildName << " is not implemented " << std::endl;
						}
						else if (positionChildName == "RoadCoord")
						{
							std::cout << positionChildName << " is not implemented " << std::endl;
						}
						else if (positionChildName == "LaneCoord")
						{
							position.Route.Position.LaneCoord.pathS = std::stod(ReadAttribute(positionChild.attribute("pathS")));

							position.Route.Position.LaneCoord.laneId = std::stoi(ReadAttribute(positionChild.attribute("laneId")));

							pugi::xml_attribute laneOffsetAttribute = positionChild.attribute("laneOffset");
							if (laneOffsetAttribute != NULL)
							{
								position.Route.Position.LaneCoord.laneOffset = std::stod(ReadAttribute(positionChild.attribute("laneOffset")));
							}
						}
					}
				}
			}
		}
	}
	std::cout << "ScenarioReader: parseOSCPosition finshed" << std::endl;
}


// ------------------------------------------------------
void ScenarioReader::parseOSCPrivateAction(OSCPrivateAction &action, pugi::xml_node actionNode)
{
	std::cout << "ScenarioReader: parseOSCPrivateAction started" << std::endl;

	for (pugi::xml_node actionChild = actionNode.first_child(); actionChild; actionChild = actionChild.next_sibling())
	{
		std::string actionChildName(actionChild.name());

		if (actionChildName == "Longitudinal")
		{
			for (pugi::xml_node longitudinalChild = actionChild.first_child(); longitudinalChild; longitudinalChild = longitudinalChild.next_sibling())
			{
				action.Longitudinal.exists = true;
				std::string longitudinalChildName(longitudinalChild.name());

				if (longitudinalChildName == "Speed")
				{
					action.Longitudinal.Speed.exists = true;

					for (pugi::xml_node speedChild = longitudinalChild.first_child(); speedChild; speedChild = speedChild.next_sibling())
					{
						std::string speedChildName(speedChild.name());

						if (speedChildName == "Dynamics")
						{
							action.Longitudinal.Speed.Dynamics.exists = true;

							action.Longitudinal.Speed.Dynamics.shape = ReadAttribute(speedChild.attribute("shape"));

							if (ReadAttribute(speedChild.attribute("rate")) != "")
							{
								action.Longitudinal.Speed.Dynamics.rate = std::stod(ReadAttribute(speedChild.attribute("rate")));
							}

							if (ReadAttribute(speedChild.attribute("time")) != "")
							{
								action.Longitudinal.Speed.Dynamics.time = std::stod(ReadAttribute(speedChild.attribute("time")));
							}

							if (ReadAttribute(speedChild.attribute("distance")) != "")
							{
								action.Longitudinal.Speed.Dynamics.distance = std::stod(ReadAttribute(speedChild.attribute("distance")));
							}
						}
						else if (speedChildName == "Target")
						{
							action.Longitudinal.Speed.Target.exists = true;

							for (pugi::xml_node targetChild = speedChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{
								std::string targetChildName(targetChild.name());

								if (targetChildName == "Relative")
								{
									action.Longitudinal.Speed.Target.Relative.exists = true;

									action.Longitudinal.Speed.Target.Relative.object = ReadAttribute(targetChild.attribute("object"));
									action.Longitudinal.Speed.Target.Relative.value = std::stod(ReadAttribute(targetChild.attribute("value")));
									action.Longitudinal.Speed.Target.Relative.valueType = ReadAttribute(targetChild.attribute("valueType"));
									action.Longitudinal.Speed.Target.Relative.continuous = std::stod(ReadAttribute(targetChild.attribute("continuous")));
								}
								else if (targetChildName == "Absolute")
								{
									action.Longitudinal.Speed.Target.Absolute.exists = true;

									action.Longitudinal.Speed.Target.Absolute.value = std::stod(ReadAttribute(targetChild.attribute("value")));
								}
							}
						}
					}
				}
				else if (longitudinalChildName == "Distance")
				{
					std::cout << longitudinalChildName << " is not implemented " << std::endl;
				}

			}
		}
		else if (actionChildName == "Lateral")
		{
			for (pugi::xml_node lateralChild = actionChild.first_child(); lateralChild; lateralChild = lateralChild.next_sibling())
			{

				std::string lateralChildName(lateralChild.name());

				if (lateralChildName == "LaneChange")
				{
					if (ReadAttribute(lateralChild.attribute("targetLaneOffset")) != "")
					{
						action.Lateral.LaneChange.targetLaneOffset = std::stod(ReadAttribute(lateralChild.attribute("targetLaneOffset")));
					}

					for (pugi::xml_node laneChangeChild = lateralChild.first_child(); laneChangeChild; laneChangeChild = laneChangeChild.next_sibling())
					{

						std::string laneChangeChildName(laneChangeChild.name());

						if (laneChangeChildName == "Dynamics")
						{
							action.Lateral.LaneChange.Dynamics.time = std::stod(ReadAttribute(laneChangeChild.attribute("time")));

							if (ReadAttribute(laneChangeChild.attribute("distance")) != "")
							{
								action.Lateral.LaneChange.Dynamics.distance = std::stod(ReadAttribute(laneChangeChild.attribute("distance")));
							}

							action.Lateral.LaneChange.Dynamics.shape = ReadAttribute(laneChangeChild.attribute("shape"));
						}
						else if (laneChangeChildName == "Target")
						{
							for (pugi::xml_node targetChild = laneChangeChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{

								std::string targetChildName(targetChild.name());

								if (targetChildName == "Relative")
								{
									action.Lateral.LaneChange.Target.Relative.object = ReadAttribute(targetChild.attribute("object"));
									action.Lateral.LaneChange.Target.Relative.value = std::stod(ReadAttribute(targetChild.attribute("value")));
								}
								else if (targetChildName == "Absolute")
								{
									action.Lateral.LaneChange.Target.Absolute.value = std::stod(ReadAttribute(targetChild.attribute("value")));
								}
							}
						}
					}
				}
				else if(lateralChildName == "LaneOffset")
				{
					for (pugi::xml_node laneOffsetChild = lateralChild.first_child(); laneOffsetChild; laneOffsetChild = laneOffsetChild.next_sibling())
					{

						std::string laneOffsetChildName(laneOffsetChild.name());

						if (laneOffsetChildName == "Dynamics")
						{
							if (ReadAttribute(laneOffsetChild.attribute("maxLateralAcc")) != "")
							{
								action.Lateral.LaneOffset.Dynamics.maxLateralAcc = std::stod(ReadAttribute(laneOffsetChild.attribute("maxLateralAcc")));
							}

							if (ReadAttribute(laneOffsetChild.attribute("duration")) != "")
							{
								action.Lateral.LaneOffset.Dynamics.duration = std::stod(ReadAttribute(laneOffsetChild.attribute("duration")));
							}

							action.Lateral.LaneOffset.Dynamics.shape = ReadAttribute(laneOffsetChild.attribute("shape"));
						}
						else if (laneOffsetChildName == "Target")
						{
							for (pugi::xml_node targetChild = laneOffsetChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{

								std::string targetChildName(targetChild.name());

								if (targetChildName == "Relative")
								{
									action.Lateral.LaneOffset.Target.Relative.object = ReadAttribute(targetChild.attribute("object"));
									action.Lateral.LaneOffset.Target.Relative.value = std::stod(ReadAttribute(targetChild.attribute("value")));
								}
								else if (targetChildName == "Absolute")
								{
									action.Lateral.LaneOffset.Target.Absolute.value = std::stod(ReadAttribute(targetChild.attribute("value")));
								}
							}
						}
					}
				}
				else
				{
					std::cout << "Unsupported element type: " << lateralChildName << std::endl;
				}
			}
		}
		else if (actionChildName == "Visibility")
		{
			std::cout << actionChildName << " is not implemented " << std::endl;
		}
		else if (actionChildName == "Meeting")
		{

			action.Meeting.mode = ReadAttribute(actionChild.attribute("mode"));

			for (pugi::xml_node meetingChild = actionChild.first_child(); meetingChild; meetingChild = meetingChild.next_sibling())
			{
				std::string meetingChildName(meetingChild.name());

				if (meetingChildName == "Position")
				{
					parseOSCPosition(action.Meeting.Position, meetingChild);
				}
				else if (meetingChildName == "Relative")
				{

					action.Meeting.Relative.mode = ReadAttribute(meetingChild.attribute("mode"));
					action.Meeting.Relative.object = ReadAttribute(meetingChild.attribute("object"));
					action.Meeting.Relative.offsetTime = std::stod(ReadAttribute(meetingChild.attribute("offsetTime")));
					action.Meeting.Relative.continuous = ReadAttribute(meetingChild.attribute("continuous"));

					parseOSCPosition(action.Meeting.Relative.Position, meetingChild.child("Position"));

				}
			}

		}
		else if (actionChildName == "Autonomous")
		{
			std::cout << actionChildName << " is not implemented " << std::endl;
		}
		else if (actionChildName == "Controller")
		{
			std::cout << actionChildName << " is not implemented " << std::endl;
		}
		else if (actionChildName == "Position")
		{
			action.Position.exists = true;
			parseOSCPosition(action.Position, actionChild);

		}
		else if (actionChildName == "Routing")
		{
			for (pugi::xml_node routingChild = actionChild.first_child(); routingChild; routingChild = routingChild.next_sibling())
			{
				std::string routingChildName(routingChild.name());

				if (routingChildName == "FollowRoute")
				{
					for (pugi::xml_node followRouteChild = routingChild.first_child(); followRouteChild; followRouteChild = followRouteChild.next_sibling())
					{
						std::string followRouteChildName(followRouteChild.name());

						if (followRouteChildName == "Route")
						{
							std::cout << followRouteChildName << " is not implemented " << std::endl;
						}
						else if (followRouteChildName == "CatalogReference")
						{
							parseOSCCatalogReference(action.Routing.FollowRoute.CatalogReference, followRouteChild);
						}
					}
				}
			}
		}
	}

	std::cout << "ScenarioReader: parseOSCPrivateAction finshed" << std::endl;
}


void ScenarioReader::parseInit(Init &init)
{
	std::cout << "ScenarioReader: parseInit started" << std::endl;

	pugi::xml_node actionsNode = doc.child("OpenSCENARIO").child("Storyboard").child("Init").child("Actions");

	for (pugi::xml_node actionsChild = actionsNode.first_child(); actionsChild; actionsChild = actionsChild.next_sibling())
	{

		std::string actionsChildName(actionsChild.name());

		if (actionsChildName == "Global")
		{
			std::cout << actionsChildName << " is not implemented " << std::endl;

		}
		else if (actionsChildName == "UserDefined")
		{
			std::cout << actionsChildName << " is not implemented " << std::endl;

		}
		else if (actionsChildName == "Private")
		{
			init.Actions.Private.resize(init.Actions.Private.size() + 1);
			init.Actions.Private.back().exists = true;

			init.Actions.Private.back().object = ReadAttribute(actionsChild.attribute("object"));

			for (pugi::xml_node privateChild = actionsChild.first_child(); privateChild; privateChild = privateChild.next_sibling())
			{
				init.Actions.Private.back().Action.resize(init.Actions.Private.back().Action.size() + 1);
				init.Actions.Private.back().Action.back().exists = true;

				parseOSCPrivateAction(init.Actions.Private.back().Action.back(), privateChild);
			}

		}

	}
	std::cout << "ScenarioReader: parseInit finshed" << std::endl;

}




// ------------------------------------------
void ScenarioReader::parseOSCCondition(OSCCondition &condition, pugi::xml_node conditionNode)
{
	std::cout << "ScenarioReader: parseOSCCondition started" << std::endl;

	condition.name = ReadAttribute(conditionNode.attribute("name"));

	condition.delay = std::stod(ReadAttribute(conditionNode.attribute("delay")));

	condition.edge = ReadAttribute(conditionNode.attribute("edge"));

	for (pugi::xml_node conditionChild = conditionNode.first_child(); conditionChild; conditionChild = conditionChild.next_sibling())
	{
		std::string conditionChildName(conditionChild.name());
		if (conditionChildName == "ByEntity")
		{
			for (pugi::xml_node byEntityChild = conditionChild.first_child(); byEntityChild; byEntityChild = byEntityChild.next_sibling())
			{
				std::string byEntityChildName(byEntityChild.name());

				if (byEntityChildName == "TriggeringEntities")
				{
					pugi::xml_attribute triggeringEntitiesRuleAttribute = byEntityChild.attribute("rule");
					//std::cout << triggeringEntitiesRuleAttribute.name() << " = " << triggeringEntitiesRuleAttribute.value() << std::endl;
					condition.ByEntity.TriggeringEntities.rule = triggeringEntitiesRuleAttribute.value();

					for (pugi::xml_node triggeringEntitiesChild = byEntityChild.first_child(); triggeringEntitiesChild; triggeringEntitiesChild = triggeringEntitiesChild.next_sibling())
					{
						std::string triggeringEntitiesChildName(triggeringEntitiesChild.name());

						if (triggeringEntitiesChildName == "Entity")
						{
							condition.ByEntity.TriggeringEntities.Entity.resize(condition.ByEntity.TriggeringEntities.Entity.size() + 1);

							pugi::xml_attribute entityNameAttribute = triggeringEntitiesChild.attribute("name");
							//std::cout << entityNameAttribute.name() << " = " << entityNameAttribute.value() << std::endl;
							condition.ByEntity.TriggeringEntities.Entity.back().name = entityNameAttribute.value();
						}

					}
				}
				else if (byEntityChildName == "EntityCondition")
				{
					for (pugi::xml_node entityConditionChild = byEntityChild.first_child(); entityConditionChild; entityConditionChild = entityConditionChild.next_sibling())
					{
						std::string entityConditionChildName(entityConditionChild.name());

						if (entityConditionChildName == "EndOfRoad")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "Collision")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "Offroad")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "TimeHeadway")
						{

							pugi::xml_attribute timeHeadwayEntityAttribute = entityConditionChild.attribute("entity");
							//std::cout << timeHeadwayEntityAttribute.name() << " = " << timeHeadwayEntityAttribute.value() << std::endl;
							if (timeHeadwayEntityAttribute.value()[0] == '$')
							{
								condition.ByEntity.EntityCondition.TimeHeadway.entity = getParameter(timeHeadwayEntityAttribute.value());
							}
							else
							{
								condition.ByEntity.EntityCondition.TimeHeadway.entity = timeHeadwayEntityAttribute.value();
							}

							pugi::xml_attribute timeHeadwayValueAttribute = entityConditionChild.attribute("value");
							//std::cout << timeHeadwayValueAttribute.name() << " = " << timeHeadwayValueAttribute.value() << std::endl;
							if (timeHeadwayValueAttribute.value()[0] == '$')
							{
								condition.ByEntity.EntityCondition.TimeHeadway.value = getParameter(timeHeadwayValueAttribute.value());
							}
							else
							{
								condition.ByEntity.EntityCondition.TimeHeadway.value = timeHeadwayValueAttribute.value();
							}

							pugi::xml_attribute timeHeadwayFreespaceAttribute = entityConditionChild.attribute("freespace");
							//std::cout << timeHeadwayFreespaceAttribute.name() << " = " << timeHeadwayFreespaceAttribute.value() << std::endl;
							condition.ByEntity.EntityCondition.TimeHeadway.freespace = timeHeadwayFreespaceAttribute.value();

							pugi::xml_attribute timeHeadwayAlongRouteAttribute = entityConditionChild.attribute("alongRoute");
							//std::cout << timeHeadwayAlongRouteAttribute.name() << " = " << timeHeadwayAlongRouteAttribute.value() << std::endl;
							condition.ByEntity.EntityCondition.TimeHeadway.alongRoute = timeHeadwayAlongRouteAttribute.value();

							pugi::xml_attribute timeHeadwayRuleAttribute = entityConditionChild.attribute("rule");
							//std::cout << timeHeadwayRuleAttribute.name() << " = " << timeHeadwayRuleAttribute.value() << std::endl;
							condition.ByEntity.EntityCondition.TimeHeadway.rule = timeHeadwayRuleAttribute.value();

						}
						else if (entityConditionChildName == "TimeToCollision")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "Acceleration")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "StandStill")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "Speed")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "RelativeSpeed")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "TraveledDistance")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "ReachPosition")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "Distance")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
						else if (entityConditionChildName == "RelativeDistance")
						{
							std::cout << entityConditionChildName << " is not implemented " << std::endl;
						}
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
					std::cout << byStateChildName << " is not implemented " << std::endl;
				}
				else if (byStateChildName == "AfterTermination")
				{
					pugi::xml_attribute afterTerminationTypeAttribute = byStateChild.attribute("type");
					condition.ByState.AfterTermination.type = afterTerminationTypeAttribute.value();

					pugi::xml_attribute afterTerminationNameAttribute = byStateChild.attribute("name");
					condition.ByState.AfterTermination.name = afterTerminationNameAttribute.value();
				}
				else if (byStateChildName == "Command")
				{
					std::cout << byStateChildName << " is not implemented " << std::endl;
				}
				else if (byStateChildName == "Signal")
				{
					std::cout << byStateChildName << " is not implemented " << std::endl;
				}
				else if (byStateChildName == "Controller")
				{
					std::cout << byStateChildName << " is not implemented " << std::endl;
				}
			}
		}
		else if (conditionChildName == "ByValue")
		{
			for (pugi::xml_node byValueChild = conditionChild.first_child(); byValueChild; byValueChild = byValueChild.next_sibling())
			{
				std::string byValueChildName(byValueChild.name());
				if (byValueChildName == "Parameter")
				{
					std::cout << byValueChildName << " is not implemented " << std::endl;
				}
				else if (byValueChildName == "TimeOfDay")
				{
					std::cout << byValueChildName << " is not implemented " << std::endl;
				}
				else if (byValueChildName == "SimulationTime")
				{
					pugi::xml_attribute simulationTimeValueAttribute = byValueChild.attribute("value");
					condition.ByValue.SimulationTime.value = std::stod(simulationTimeValueAttribute.value());

					pugi::xml_attribute simulationTimeRuleAttribute = byValueChild.attribute("rule");
					condition.ByValue.SimulationTime.rule = simulationTimeRuleAttribute.value();

				}
			}
		}
	}

	std::cout << "ScenarioReader: parseOSCCondition finished" << std::endl;
}

void ScenarioReader::parseOSCConditionGroup(OSCConditionGroup &conditionGroup, pugi::xml_node conditionGroupNode)
{
	std::cout << "ScenarioReader: parseOSCConditionGroup started" << std::endl;

	for (pugi::xml_node conditionGroupChild = conditionGroupNode.first_child(); conditionGroupChild; conditionGroupChild = conditionGroupChild.next_sibling())
	{
		conditionGroup.Condition.resize(conditionGroup.Condition.size() + 1);
		parseOSCCondition(conditionGroup.Condition.back(), conditionGroupChild);
	}

	std::cout << "ScenarioReader: parseOSCConditionGroup finished" << std::endl;

}


void ScenarioReader::parseOSCManeuver(OSCManeuver &maneuver, pugi::xml_node maneuverNode)
{
	std::cout << "ScenarioReader: parseOSCManeuver started" << std::endl;

	pugi::xml_attribute maneuverNodeNameAttribute = maneuverNode.attribute("name");
	//std::cout << maneuverNodeNameAttribute.name() << " = " << maneuverNodeNameAttribute.value() << std::endl;
	maneuver.name = maneuverNodeNameAttribute.value();

	for (pugi::xml_node maneuverChild = maneuverNode.first_child(); maneuverChild; maneuverChild = maneuverChild.next_sibling())
	{
		std::string maneuverChildName(maneuverChild.name());

		if (maneuverChildName == "ParameterDeclaration")
		{
			std::cout << maneuverChildName << " is not implemented " << std::endl;
		}
		else if (maneuverChildName == "Event")
		{
			maneuver.Event.resize(maneuver.Event.size() + 1);

			pugi::xml_attribute eventNameAttribute = maneuverChild.attribute("name");
			//std::cout << eventNameAttribute.name() << " = " << eventNameAttribute.value() << std::endl;
			maneuver.Event.back().name = eventNameAttribute.value();

			pugi::xml_attribute eventPriorityAttribute = maneuverChild.attribute("priority");
			//std::cout << eventPriorityAttribute.name() << " = " << eventPriorityAttribute.value() << std::endl;
			maneuver.Event.back().priority = eventPriorityAttribute.value();

			for (pugi::xml_node eventChild = maneuverChild.first_child(); eventChild; eventChild = eventChild.next_sibling())
			{
				std::string eventChildName(eventChild.name());

				if (eventChildName == "Action")
				{
					maneuver.Event.back().Action.resize(maneuver.Event.back().Action.size() + 1);

					pugi::xml_attribute actionNameAttribute = eventChild.attribute("name");
					//std::cout << actionNameAttribute.name() << " = " << actionNameAttribute.value() << std::endl;
					maneuver.Event.back().Action.back().name = actionNameAttribute.value();

					for (pugi::xml_node actionChild = eventChild.first_child(); actionChild; actionChild = actionChild.next_sibling())
					{
						std::string actionChildName(actionChild.name());

						if (actionChildName == "Global")
						{
							std::cout << maneuverChildName << " is not implemented " << std::endl;
						}
						else if (actionChildName == "UserDefined")
						{
							std::cout << maneuverChildName << " is not implemented " << std::endl;
						}
						else if (actionChildName == "Private")
						{
							parseOSCPrivateAction(maneuver.Event.back().Action.back().Private, actionChild);
						}
					}
				}
				else if (eventChildName == "StartConditions")
				{
					for (pugi::xml_node startConditionsChild = eventChild.first_child(); startConditionsChild; startConditionsChild = startConditionsChild.next_sibling())
					{
						maneuver.Event.back().StartConditions.ConditionGroup.resize(maneuver.Event.back().StartConditions.ConditionGroup.size() + 1);
						parseOSCConditionGroup(maneuver.Event.back().StartConditions.ConditionGroup.back(), startConditionsChild);
					}
				}
			}
		}
	}

	std::cout << "ScenarioReader: parseOSCManeuver finished" << std::endl;
}



void ScenarioReader::parseStory(std::vector<Story> &storyVector)
{
	std::cout << "ScenarioReader: parseStory started" << std::endl;

	pugi::xml_node storyNode = doc.child("OpenSCENARIO").child("Storyboard").child("Story");

	for (storyNode; storyNode; storyNode = storyNode.next_sibling())
	{
		std::string storyNodeName(storyNode.name());

		if (storyNodeName == "Story")
		{
			storyVector.resize(storyVector.size() + 1);

			pugi::xml_attribute storyNodeOwnerAttribute = storyNode.attribute("owner");
			//std::cout << storyNodeOwnerAttribute.name() << " = " << storyNodeOwnerAttribute.value() << std::endl;
			addParameter("$owner", storyNodeOwnerAttribute.value());
			storyVector.back().owner = storyNodeOwnerAttribute.value();

			pugi::xml_attribute storyNodeNameAttribute = storyNode.attribute("name");
			//std::cout << storyNodeNameAttribute.name() << " = " << storyNodeNameAttribute.value() << std::endl;
			storyVector.back().name = storyNodeNameAttribute.value();

			for (pugi::xml_node storyChild = storyNode.first_child(); storyChild; storyChild = storyChild.next_sibling())
			{
				storyVector.back().Act.resize(storyVector.back().Act.size() + 1);

				pugi::xml_attribute actNameAttribute = storyChild.attribute("name");
				//std::cout << actNameAttribute.name() << " = " << actNameAttribute.value() << std::endl;
				storyVector.back().Act.back().name = actNameAttribute.value();

				for (pugi::xml_node actChild = storyChild.first_child(); actChild; actChild = actChild.next_sibling())
				{

					std::string actChildName(actChild.name());

					if (actChildName == "Sequence")
					{
						storyVector.back().Act.back().Sequence.resize(storyVector.back().Act.back().Sequence.size() + 1);

						pugi::xml_attribute sequenceNumberAttribute = actChild.attribute("numberOfExecutions");
						//std::cout << sequenceNumberAttribute.name() << " = " << sequenceNumberAttribute.value() << std::endl;
						storyVector.back().Act.back().Sequence.back().numberOfExecutions = std::stoi(sequenceNumberAttribute.value());

						pugi::xml_attribute sequenceNameAttribute = actChild.attribute("name");
						//std::cout << sequenceNameAttribute.name() << " = " << sequenceNameAttribute.value() << std::endl;
						storyVector.back().Act.back().Sequence.back().name = sequenceNameAttribute.value();

						for (pugi::xml_node sequenceChild = actChild.first_child(); sequenceChild; sequenceChild = sequenceChild.next_sibling())
						{
							std::string sequenceChildName(sequenceChild.name());
							if (sequenceChildName == "Actors")
							{
								for (pugi::xml_node actorsChild = sequenceChild.first_child(); actorsChild; actorsChild = actorsChild.next_sibling())
								{
									std::string actorsChildName(actorsChild.name());
									if (actorsChildName == "Entity")
									{
										storyVector.back().Act.back().Sequence.back().Actors.Entity.resize(storyVector.back().Act.back().Sequence.back().Actors.Entity.size() + 1);

										storyVector.back().Act.back().Sequence.back().Actors.Entity.back().name = ReadAttribute(actorsChild.attribute("name"));

									}
									else if (actorsChildName == "ByCondition")
									{
										storyVector.back().Act.back().Sequence.back().Actors.ByCondition.actor = ReadAttribute(actorsChild.attribute("object"));

									}
								}
							}
							else if (sequenceChildName == "CatalogReference")
							{
								std::cout << sequenceChildName << " is not implemented " << std::endl;
							}
							else if (sequenceChildName == "Maneuver")
							{
								storyVector.back().Act.back().Sequence.back().Maneuver.resize(storyVector.back().Act.back().Sequence.back().Maneuver.size() + 1);
								parseOSCManeuver(storyVector.back().Act.back().Sequence.back().Maneuver.back(), sequenceChild);
							}
						}
					}

					else if (actChildName == "Conditions")
					{
						for (pugi::xml_node conditionsChild = actChild.first_child(); conditionsChild; conditionsChild = conditionsChild.next_sibling())
						{
							std::string conditionsChildName(conditionsChild.name());
							if (conditionsChildName == "Start")
							{
								for (pugi::xml_node startChild = conditionsChild.first_child(); startChild; startChild = startChild.next_sibling())
								{
									storyVector.back().Act.back().Conditions.start.ConditionGroup.resize(storyVector.back().Act.back().Conditions.start.ConditionGroup.size() + 1);
									parseOSCConditionGroup(storyVector.back().Act.back().Conditions.start.ConditionGroup.back(), startChild);
								}
							}
							else if (conditionsChildName == "End")
							{
								std::cout << conditionsChildName << " is not implemented " << std::endl;
							}
							else if (conditionsChildName == "Cancel")
							{
								std::cout << conditionsChildName << " is not implemented " << std::endl;
							}
						}
					}
				}
			}
		}
	}

	std::cout << "ScenarioReader: parseStory finshed" << std::endl;

}


