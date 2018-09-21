#include "ScenarioGateway.hpp"


ExternalCar::ExternalCar()
{
}

void ExternalCar::setPosition(roadmanager::Position position)
{
	this->position = position;
}

void ExternalCar::setSpeed(double speed)
{
	this->speed = speed;
}

void ExternalCar::setId(int objectId)
{
	this->objectId = objectId;
}

void ExternalCar::setName(std::string objectName)
{
	this->objectName = objectName;
}

roadmanager::Position ExternalCar::getPosition()
{
	return position;
}

double ExternalCar::getSpeed()
{
	return speed;
}

int ExternalCar::getId()
{
	return objectId;
}

std::string ExternalCar::getName()
{
	return objectName;
}



// ScenarioGateway
ScenarioGateway::ScenarioGateway()
{
}

void ScenarioGateway::addExternalCar(int objectId, std::string objectName)
{
	ExternalCar externalCar;
	externalCar.setId(objectId);
	externalCar.setName(objectName);
	externalCars.push_back(externalCar);
}

int ScenarioGateway::getIdx(std::string objectName)
{
	for (size_t i = 0; i < externalCars.size(); i++)
	{
		if (externalCars[i].getName() == objectName)
		{
			return i;
		}
	}

	return -1;
}

int ScenarioGateway::getIdx(int objectId)
{
	for (size_t i = 0; i < externalCars.size(); i++)
	{
		if (externalCars[i].getId() == objectId)
		{
			return externalCars[i].getId();
		}
	}

	return -1;
}

void ScenarioGateway::setExternalCarPosition(int objectId, roadmanager::Position position)
{
	int idx = getIdx(objectId);
	externalCars[idx].setPosition(position);
}

void ScenarioGateway::setExternalCarPosition(std::string objectName, roadmanager::Position position)
{
	int idx = getIdx(objectName);
	externalCars[idx].setPosition(position);
}

void ScenarioGateway::setExternalCarSpeed(int objectId, double speed)
{
	int idx = getIdx(objectId);
	externalCars[idx].setSpeed(speed);
}

void ScenarioGateway::setExternalCarSpeed(std::string objectName, double speed)
{
	int idx = getIdx(objectName);
	externalCars[idx].setSpeed(speed);
}

void ScenarioGateway::setExternalObjectId(int objectId, int newObjectId)
{
	int idx = getIdx(objectId);
	externalCars[idx].setId(newObjectId);
}

void ScenarioGateway::setExternalObjectId(std::string objectName, int newObjectId)
{
	int idx = getIdx(objectName);
	externalCars[idx].setId(newObjectId);
}

roadmanager::Position ScenarioGateway::getExternalCarPosition(int objectId)
{
	int idx = getIdx(objectId);
	return externalCars[idx].getPosition();
}

roadmanager::Position ScenarioGateway::getExternalCarPosition(std::string objectName)
{
	int idx = getIdx(objectName);
	return externalCars[idx].getPosition();
}

double ScenarioGateway::getExternalCarSpeed(int objectId)
{
	int idx = getIdx(objectId);
	return externalCars[idx].getSpeed();
}

double ScenarioGateway::getExternalCarSpeed(std::string objectName)
{
	int idx = getIdx(objectName);
	return externalCars[idx].getSpeed();
}