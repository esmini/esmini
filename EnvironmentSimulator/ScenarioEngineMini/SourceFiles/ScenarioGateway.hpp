#pragma once
#include "Cars.hpp"

class ScenarioGateway
{
public:
	
	ScenarioGateway();

	void addExternalCar(Car externalCar);

	void setExternalCarPosition(int objectId, roadmanager::Position position);
	void setExternalCarPosition(std::string objectName, roadmanager::Position position);

	void setExternalObjectId(int objectId, int newObjectId);
	void setExternalObjectId(std::string objectName, int newObjectId);

	roadmanager::Position getExternalCarPosition(int objectId);
	roadmanager::Position getExternalCarPosition(std::string objectName);

private:
	Cars externalCars;
};

