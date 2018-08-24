#include "ScenarioReader.hpp"


ScenarioReader::ScenarioReader()
{
	std::cout << "ScenarioReader: ScenarioReader started" << std::endl;
	doc;
	objectCnt = 0;
	std::cout << "ScenarioReader: ScenarioReader finished" << std::endl;
}


void ScenarioReader::loadXmlFile(const char * path)
{
	std::cout << "ScenarioReader: loadXmlFile started. Loading " << path << std::endl;


	pugi::xml_parse_result result = doc.load_file(path);
	std::cout << "ScenarioReader: loadXmlFile finished" << std::endl;
}

void ScenarioReader::parseParameterDeclaration()
{
	std::cout << "ScenarioReader: parseParameterDeclaration started" << std::endl;

	pugi::xml_node parameterDeclarationNode = doc.child("OpenSCENARIO").child("ParameterDeclaration");
	
	for (pugi::xml_node parameterDeclarationChild = parameterDeclarationNode.first_child(); parameterDeclarationChild; parameterDeclarationChild = parameterDeclarationChild.next_sibling())
	{
		parameterDeclaration.Parameter.resize(parameterDeclaration.Parameter.size() + 1);

		pugi::xml_attribute parameterNameAttribute = parameterDeclarationChild.attribute("name");
		//std::cout << parameterNameAttribute.name() << " = " << parameterNameAttribute.value() << std::endl;
		parameterDeclaration.Parameter.back().name = parameterNameAttribute.value();

		pugi::xml_attribute parameterTypeAttribute = parameterDeclarationChild.attribute("type");
		//std::cout << parameterTypeAttribute.name() << " = " << parameterTypeAttribute.value() << std::endl;
		parameterDeclaration.Parameter.back().type = parameterTypeAttribute.value();

		pugi::xml_attribute parameterValueAttribute = parameterDeclarationChild.attribute("value");
		//std::cout << parameterValueAttribute.name() << " = " << parameterValueAttribute.value() << std::endl;
		parameterDeclaration.Parameter.back().value = parameterValueAttribute.value();
	}
	std::cout << "ScenarioReader: parseParameterDeclaration finished" << std::endl;
}

void ScenarioReader::addParameter(std::string name, std::string value)
{
	std::cout << "ScenarioReader: addParameter started" << std::endl;

	std::cout << "ScenarioReader: " << name << " addeed to parameterDeclaration" << std::endl;

	parameterDeclaration.Parameter.resize(parameterDeclaration.Parameter.size() + 1);

	parameterDeclaration.Parameter.back().name = name;
	parameterDeclaration.Parameter.back().type = "string";
	parameterDeclaration.Parameter.back().value = value;

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
}

void ScenarioReader::parseOSCBoundingBox(OSCBoundingBox &boundingBox, pugi::xml_node boundingBoxNode)
{
	std::cout << "ScenarioReader: parseOSCBoundingBox started" << std::endl;

	for (pugi::xml_node boundingBoxChild = boundingBoxNode.first_child(); boundingBoxChild; boundingBoxChild = boundingBoxChild.next_sibling())
	{
		std::string boundingBoxChildName(boundingBoxChild.name());

		if (boundingBoxChildName == "Center")
		{
			pugi::xml_attribute boundingBoxXAttribute = boundingBoxChild.attribute("x");
			//std::cout << boundingBoxXAttribute.name() << " = " << boundingBoxXAttribute.value() << std::endl;
			boundingBox.center.x = std::stod(boundingBoxXAttribute.value());

			pugi::xml_attribute boundingBoxYAttribute = boundingBoxChild.attribute("y");
			//std::cout << boundingBoxYAttribute.name() << " = " << boundingBoxYAttribute.value() << std::endl;
			boundingBox.center.y = std::stod(boundingBoxYAttribute.value());

			pugi::xml_attribute boundingBoxZAttribute = boundingBoxChild.attribute("z");
			//std::cout << boundingBoxZAttribute.name() << " = " << boundingBoxZAttribute.value() << std::endl;
			boundingBox.center.z = std::stod(boundingBoxZAttribute.value());
		}
		if (boundingBoxChildName == "Dimension")
		{
			pugi::xml_attribute boundingBoxWidthAttribute = boundingBoxChild.attribute("width");
			//std::cout << boundingBoxWidthAttribute.name() << " = " << boundingBoxWidthAttribute.value() << std::endl;
			boundingBox.dimension.width = std::stod(boundingBoxWidthAttribute.value());

			pugi::xml_attribute boundingBoxLengthAttribute = boundingBoxChild.attribute("length");
			//std::cout << boundingBoxLengthAttribute.name() << " = " << boundingBoxLengthAttribute.value() << std::endl;
			boundingBox.dimension.length = std::stod(boundingBoxLengthAttribute.value());

			pugi::xml_attribute boundingBoxHeightAttribute = boundingBoxChild.attribute("height");
			//std::cout << boundingBoxHeightAttribute.name() << " = " << boundingBoxHeightAttribute.value() << std::endl;
			boundingBox.dimension.height = std::stod(boundingBoxHeightAttribute.value());

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

	pugi::xml_attribute descriptionWeightAttribute = descriptionNode.attribute("weight");
	//std::cout << descriptionWeightAttribute.name() << " = " << descriptionWeightAttribute.value() << std::endl;
	personDescription.weight = std::stod(descriptionWeightAttribute.value());

	pugi::xml_attribute descriptionHeightAttribute = descriptionNode.attribute("height");
	//std::cout << descriptionHeightAttribute.name() << " = " << descriptionHeightAttribute.value() << std::endl;
	personDescription.height = std::stod(descriptionHeightAttribute.value());

	pugi::xml_attribute descriptionEyeDistanceAttribute = descriptionNode.attribute("eyeDistance");
	//std::cout << descriptionEyeDistanceAttribute.name() << " = " << descriptionEyeDistanceAttribute.value() << std::endl;
	personDescription.eyeDistance = std::stod(descriptionEyeDistanceAttribute.value());

	pugi::xml_attribute descriptionAgeAttribute = descriptionNode.attribute("age");
	//std::cout << descriptionAgeAttribute.name() << " = " << descriptionAgeAttribute.value() << std::endl;
	personDescription.age = std::stod(descriptionAgeAttribute.value());

	pugi::xml_attribute descriptionSexAttribute = descriptionNode.attribute("sex");
	//std::cout << descriptionSexAttribute.name() << " = " << descriptionSexAttribute.value() << std::endl;
	personDescription.sex = descriptionSexAttribute.value();

	std::cout << "ScenarioReader: parseOSCPersonDescription finshed" << std::endl;
}



void ScenarioReader::parseOSCAxle(OSCAxle &axle, pugi::xml_node axleNode)
{
	std::cout << "ScenarioReader: parseOSCAxle started" << std::endl;

	pugi::xml_attribute axleMaxSteeringAttribute = axleNode.attribute("maxSteering");
	//std::cout << axleMaxSteeringAttribute.name() << " = " << axleMaxSteeringAttribute.value() << std::endl;
	axle.maxSteering = std::stod(axleMaxSteeringAttribute.value());

	pugi::xml_attribute axleWheelDiameterAttribute = axleNode.attribute("wheelDiameter");
	//std::cout << axleWheelDiameterAttribute.name() << " = " << axleWheelDiameterAttribute.value() << std::endl;
	axle.wheelDiameter = std::stod(axleWheelDiameterAttribute.value());

	pugi::xml_attribute axleTrackWidthAttribute = axleNode.attribute("trackWidth");
	//std::cout << axleTrackWidthAttribute.name() << " = " << axleTrackWidthAttribute.value() << std::endl;
	axle.trackWidth = std::stod(axleTrackWidthAttribute.value());

	pugi::xml_attribute axlePositionXAttribute = axleNode.attribute("positionX");
	//std::cout << axlePositionXAttribute.name() << " = " << axlePositionXAttribute.value() << std::endl;
	axle.positionX = std::stod(axlePositionXAttribute.value());

	pugi::xml_attribute axleMPositionZAttribute = axleNode.attribute("positionZ");
	//std::cout << axleMPositionZAttribute.name() << " = " << axleMPositionZAttribute.value() << std::endl;
	axle.positionZ = std::stod(axleMPositionZAttribute.value());

	std::cout << "ScenarioReader: parseOSCAxle finshed" << std::endl;
}


void ScenarioReader::parseOSCDriver(OSCDriver &driver, pugi::xml_node driverNode)
{
	std::cout << "ScenarioReader: parseOSCDriver started" << std::endl;

	pugi::xml_attribute driverNameAttribute = driverNode.attribute("name");
	//std::cout << driverNameAttribute.name() << " = " << driverNameAttribute.value() << std::endl;
	driver.name = driverNameAttribute.value();

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

	pugi::xml_attribute vehicleNameAttribute = vehicleNode.attribute("name");
	//std::cout << vehicleNameAttribute.name() << " = " << vehicleNameAttribute.value() << std::endl;
	vehicle.name = vehicleNameAttribute.value();

	pugi::xml_attribute vehicleCategoryAttribute = vehicleNode.attribute("category");
	//std::cout << vehicleCategoryAttribute.name() << " = " << vehicleCategoryAttribute.value() << std::endl;
	vehicle.category = vehicleCategoryAttribute.value();


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
			pugi::xml_attribute performanceMaxSpeedAttribute = vehicleChild.attribute("maxSpeed");
			//std::cout << performanceMaxSpeedAttribute.name() << " = " << performanceMaxSpeedAttribute.value() << std::endl;
			vehicle.Performance.maxSpeed = std::stod(performanceMaxSpeedAttribute.value());

			pugi::xml_attribute performanceMaxDecelerationAttribute = vehicleChild.attribute("maxDeceleration");
			//std::cout << performanceMaxDecelerationAttribute.name() << " = " << performanceMaxDecelerationAttribute.value() << std::endl;
			vehicle.Performance.maxDeceleration = std::stod(performanceMaxDecelerationAttribute.value());

			pugi::xml_attribute performanceMassAttribute = vehicleChild.attribute("mass");
			//std::cout << performanceMassAttribute.name() << " = " << performanceMassAttribute.value() << std::endl;
			vehicle.Performance.mass = std::stod(performanceMassAttribute.value());
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

	std::cout << "ScenarioReader: parseOSCVehicle finshed" << std::endl;
}


void ScenarioReader::parseEntities(Entities &entities)
{
	std::cout << "ScenarioReader: parseEntities started" << std::endl;

	pugi::xml_node enitiesNode = doc.child("OpenSCENARIO").child("Entities");

	for (pugi::xml_node entitiesChild = enitiesNode.first_child(); entitiesChild; entitiesChild = entitiesChild.next_sibling())
	{
		entities.Object.resize(entities.Object.size() + 1);

		pugi::xml_attribute objectNameAttribute = entitiesChild.attribute("name");
		//std::cout << objectNameAttribute.name() << " = " << objectNameAttribute.value() << std::endl;
		entities.Object.back().name = objectNameAttribute.value();

		for (pugi::xml_node objectChild = entitiesChild.first_child(); objectChild; objectChild = objectChild.next_sibling())
		{
			std::string objectChildName(objectChild.name());

			if (objectChildName == "CatalogReference")
			{
				std::cout << objectChildName << " is not implemented " << std::endl;

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

			pugi::xml_attribute laneRoadIdAttribute = positionChild.attribute("roadId");
			//std::cout << laneRoadIdAttribute.name() << " = " << laneRoadIdAttribute.value() << std::endl;
			position.Lane.roadId = laneRoadIdAttribute.value();

			pugi::xml_attribute laneLaneIdAttribute = positionChild.attribute("laneId");
			//std::cout << laneLaneIdAttribute.name() << " = " << laneLaneIdAttribute.value() << std::endl;
			position.Lane.laneId = std::stoi(laneLaneIdAttribute.value());

			pugi::xml_attribute laneOffsetAttribute = positionChild.attribute("offset");
			if (laneOffsetAttribute != NULL)
			{
				//std::cout << laneOffsetAttribute.name() << " = " << laneOffsetAttribute.value() << std::endl;
				position.Lane.offset = std::stod(laneOffsetAttribute.value());
			}

			pugi::xml_attribute laneSAttribute = positionChild.attribute("s");
			//std::cout << laneSAttribute.name() << " = " << laneSAttribute.value() << std::endl;
			position.Lane.s = std::stod(laneSAttribute.value());
		}
		else if (positionChildName == "RelativeLane")
		{
			std::cout << positionChildName << " is not implemented " << std::endl;
		}
		else if (positionChildName == "Route")
		{
			std::cout << positionChildName << " is not implemented " << std::endl;
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

							pugi::xml_attribute dynamicsShapeAttribute = speedChild.attribute("shape");
							if (dynamicsShapeAttribute != NULL)
							{
								//std::cout << dynamicsShapeAttribute.name() << " = " << dynamicsShapeAttribute.value() << std::endl;
								action.Longitudinal.Speed.Dynamics.shape = dynamicsShapeAttribute.value();
							}

							pugi::xml_attribute dynamicsRateAttribute = speedChild.attribute("rate");
							if (dynamicsRateAttribute != NULL)
							{
								//std::cout << dynamicsRateAttribute.name() << " = " << dynamicsRateAttribute.value() << std::endl;
								action.Longitudinal.Speed.Dynamics.rate = std::stod(dynamicsRateAttribute.value());
							}

							pugi::xml_attribute dynamicsTimeAttribute = speedChild.attribute("time");
							if (dynamicsTimeAttribute != NULL)
							{
								//std::cout << dynamicsTimeAttribute.name() << " = " << dynamicsTimeAttribute.value() << std::endl;
								action.Longitudinal.Speed.Dynamics.time = std::stod(dynamicsTimeAttribute.value());
							}

							pugi::xml_attribute dynamicsDistanceAttribute = speedChild.attribute("distance");
							if (dynamicsDistanceAttribute != NULL)
							{
								//std::cout << dynamicsDistanceAttribute.name() << " = " << dynamicsDistanceAttribute.value() << std::endl;
								action.Longitudinal.Speed.Dynamics.distance = std::stod(dynamicsDistanceAttribute.value());
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

									pugi::xml_attribute relativeObjectAttribute = targetChild.attribute("object");
									//if (isDollar(relativeObjectAttribute.value()))
									//{
									//	//action.Longitudinal.Speed.Target.Relative.object = stor;

									//}
									//else
									//{
										action.Longitudinal.Speed.Target.Relative.object = relativeObjectAttribute.value();
									//}
									//std::cout << relativeObjectAttribute.name() << " = " << relativeObjectAttribute.value() << std::endl;

									pugi::xml_attribute relativeValueAttribute = targetChild.attribute("value");
									//std::cout << relativeValueAttribute.name() << " = " << relativeValueAttribute.value() << std::endl;
									action.Longitudinal.Speed.Target.Relative.value = std::stod(relativeValueAttribute.value());

									pugi::xml_attribute relativeValueTypeAttribute = targetChild.attribute("valueType");
									//std::cout << relativeValueTypeAttribute.name() << " = " << relativeValueTypeAttribute.value() << std::endl;
									action.Longitudinal.Speed.Target.Relative.valueType = relativeValueTypeAttribute.value();

									pugi::xml_attribute relativeContinuousAttribute = targetChild.attribute("continuous");
									//std::cout << relativeContinuousAttribute.name() << " = " << relativeContinuousAttribute.value() << std::endl;
									action.Longitudinal.Speed.Target.Relative.continuous = std::stod(relativeContinuousAttribute.value());
								}
								else if (targetChildName == "Absolute")
								{
									action.Longitudinal.Speed.Target.Absolute.exists = true;

									pugi::xml_attribute absoluteValueAttribute = targetChild.attribute("value");
									//std::cout << absoluteValueAttribute.name() << " = " << absoluteValueAttribute.value() << std::endl;
									action.Longitudinal.Speed.Target.Absolute.value = std::stod(absoluteValueAttribute.value());

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
					pugi::xml_attribute laneChangeTargetLaneOffsetAttribute = lateralChild.attribute("targetLaneOffset");
					if (laneChangeTargetLaneOffsetAttribute != NULL)
					{
						action.Lateral.LaneChange.targetLaneOffset = std::stod(laneChangeTargetLaneOffsetAttribute.value());
					}

					for (pugi::xml_node laneChangeChild = lateralChild.first_child(); laneChangeChild; laneChangeChild = laneChangeChild.next_sibling())
					{

						std::string laneChangeChildName(laneChangeChild.name());

						if (laneChangeChildName == "Dynamics")
						{
							pugi::xml_attribute dynamicsTimeAttribute = laneChangeChild.attribute("time");
							action.Lateral.LaneChange.Dynamics.time = std::stod(dynamicsTimeAttribute.value());

							pugi::xml_attribute dynamicsDistanceAttribute = laneChangeChild.attribute("distance");
							if (dynamicsDistanceAttribute != NULL)
							{
								action.Lateral.LaneChange.Dynamics.distance = std::stod(dynamicsDistanceAttribute.value());
							}

							pugi::xml_attribute dynamicsShapeAttribute = laneChangeChild.attribute("shape");
							action.Lateral.LaneChange.Dynamics.shape = dynamicsShapeAttribute.value();
						}
						else if (laneChangeChildName == "Target")
						{
							for (pugi::xml_node targetChild = laneChangeChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
							{

								std::string targetChildName(targetChild.name());

								if (lateralChildName == "Relative")
								{
									pugi::xml_attribute relativeObjectAttribute = lateralChild.attribute("object");
									action.Lateral.LaneChange.Target.Relative.object = getParameter(relativeObjectAttribute.value());

									pugi::xml_attribute relativeValueAttribute = lateralChild.attribute("value");
									action.Lateral.LaneChange.Target.Relative.value = std::stod(relativeValueAttribute.value());
								}
								else if (lateralChildName == "Absolute")
								{
									pugi::xml_attribute absoluteValueAttribute = lateralChild.attribute("value");
									action.Lateral.LaneChange.Target.Absolute.value = std::stod(absoluteValueAttribute.value());
								}
							}
						}
					}
				}
				else if (lateralChildName == "LaneOffset")
				{
					std::cout << lateralChildName << " is not implemented " << std::endl;
				}
				else if (lateralChildName == "Distance")
				{
					std::cout << lateralChildName << " is not implemented " << std::endl;
				}
			}
		}
		else if (actionChildName == "Visibility")
		{
			std::cout << actionChildName << " is not implemented " << std::endl;
		}
		else if (actionChildName == "Meeting")
		{
			std::cout << actionChildName << " is not implemented " << std::endl;
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
			std::cout << actionChildName << " is not implemented " << std::endl;
		}
	}

	std::cout << "ScenarioReader: parseOSCPrivateAction finshed" << std::endl;
}


void ScenarioReader::parseInit(Init &init)
{
	std::cout << "ScenarioReader: parseInit started" << std::endl;

	//init.Actions.Private.resize(1);
	//init.Actions.Private[0].object = "Anton";

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

			pugi::xml_attribute actionsChildAttribute = actionsChild.attribute("object");
			//std::cout << actionsChildAttribute.name() << " = " << actionsChildAttribute.value() << std::endl;
			init.Actions.Private.back().object = actionsChildAttribute.value();

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

	pugi::xml_attribute conditionNameAttribute = conditionNode.attribute("name");
	//std::cout << conditionNameAttribute.name() << " = " << conditionNameAttribute.value() << std::endl;
	condition.name = conditionNameAttribute.value();

	pugi::xml_attribute conditionDelayAttribute = conditionNode.attribute("delay");
	//std::cout << conditionDelayAttribute.name() << " = " << conditionDelayAttribute.value() << std::endl;
	condition.delay = std::stod(conditionDelayAttribute.value());

	pugi::xml_attribute conditionEdgeAttribute = conditionNode.attribute("edge");
	//std::cout << conditionEdgeAttribute.name() << " = " << conditionEdgeAttribute.value() << std::endl;
	condition.edge = conditionEdgeAttribute.value();

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
							condition.ByEntity.EntityCondition.TimeHeadway.entity = getParameter(timeHeadwayEntityAttribute.value());

							pugi::xml_attribute timeHeadwayValueAttribute = entityConditionChild.attribute("value");
							//std::cout << timeHeadwayValueAttribute.name() << " = " << timeHeadwayValueAttribute.value() << std::endl;
							condition.ByEntity.EntityCondition.TimeHeadway.value = getParameter(timeHeadwayValueAttribute.value());

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
					std::cout << byStateChildName << " is not implemented " << std::endl;
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
					//std::cout << simulationTimeValueAttribute.name() << " = " << simulationTimeValueAttribute.value() << std::endl;
					condition.ByEntity.EntityCondition.TimeHeadway.rule = std::stod(simulationTimeValueAttribute.value());

					pugi::xml_attribute simulationTimeRuleAttribute = byValueChild.attribute("rule");
					//std::cout << simulationTimeRuleAttribute.name() << " = " << simulationTimeRuleAttribute.value() << std::endl;
					condition.ByEntity.EntityCondition.TimeHeadway.rule = simulationTimeRuleAttribute.value();

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

	double count = (std::distance(storyNode.begin(), storyNode.end()));

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

										pugi::xml_attribute entityNameAttribute = actorsChild.attribute("name");
										//std::cout << entityNameAttribute.name() << " = " << entityNameAttribute.value() << std::endl;
										storyVector.back().Act.back().Sequence.back().Actors.Entity.back().name = getParameter(entityNameAttribute.value());

									}
									else if (actorsChildName == "ByCondition")
									{
										pugi::xml_attribute byConditionobjectAttribute = actorsChild.attribute("object");
										//std::cout << byConditionobjectAttribute.name() << " = " << byConditionobjectAttribute.value() << std::endl;
										storyVector.back().Act.back().Sequence.back().Actors.ByCondition.actor = byConditionobjectAttribute.value();

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


