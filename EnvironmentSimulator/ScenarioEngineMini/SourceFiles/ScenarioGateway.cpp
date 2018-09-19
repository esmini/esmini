#include "ScenarioGateway.hpp"



ScenarioGateway::ScenarioGateway()
{
}

void ScenarioGateway::addExternalCar(Car externalCar)
{
	externalCars.addCar(externalCar);
}

void ScenarioGateway::setExternalCarPosition(int objectId, roadmanager::Position position)
{
	externalCars.setPosition(objectId, position);
}

void ScenarioGateway::setExternalCarPosition(std::string objectName, roadmanager::Position position)
{
	externalCars.setPosition(objectName, position);
}

void ScenarioGateway::setExternalObjectId(int objectId, int newObjectId)
{
	externalCars.setObjectId(objectId, newObjectId);
}

void ScenarioGateway::setExternalObjectId(std::string objectName, int newObjectId)
{
	externalCars.setObjectId(objectName, newObjectId);
}

roadmanager::Position ScenarioGateway::getExternalCarPosition(int objectId)
{
	return externalCars.getPosition(objectId);
}
roadmanager::Position ScenarioGateway::getExternalCarPosition(std::string objectName)
{
	return externalCars.getPosition(objectName);
}