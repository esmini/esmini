#pragma once
#include "../../RoadManager/roadmanager.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class ExternalCar
{
public:
	ExternalCar();
	void setPosition(roadmanager::Position position);
	void setSpeed(double speed);
	void setId(int objectId);
	void setName(std::string objectName);

	roadmanager::Position getPosition();
	double getSpeed();
	int getId();
	std::string getName();

private:
	roadmanager::Position position;
	double speed;
	int objectId;
	std::string objectName;
};


class ScenarioGateway
{
public:
	
	ScenarioGateway();
	void addExternalCar(int objectId, std::string objectName);

	void setExternalCarPosition(int objectId, roadmanager::Position position);
	void setExternalCarPosition(std::string objectName, roadmanager::Position position);

	void setExternalCarSpeed(int objectId, double speed);
	void setExternalCarSpeed(std::string objectName, double speed);

	void setExternalObjectId(int objectId, int newObjectId);
	void setExternalObjectId(std::string objectName, int newObjectId);

	roadmanager::Position getExternalCarPosition(int objectId);
	roadmanager::Position getExternalCarPosition(std::string objectName);

	double getExternalCarSpeed(int objectId);
	double getExternalCarSpeed(std::string objectName);

private:
	int getIdx(std::string objectName);
	int getIdx(int objectId);

private:
	std::vector<ExternalCar> externalCars;
};

