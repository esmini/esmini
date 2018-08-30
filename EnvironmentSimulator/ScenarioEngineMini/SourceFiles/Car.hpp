#pragma once
#include "OSCPrivateAction.hpp"
#include "../../RoadManager/roadmanager.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class Car
{
public:
	Car();

	void setObjectId(int objectId);
	void setName(std::string objectName);
	void setPosition(int roadId, int laneId, double s, double offset);
	void setSpeed(double speed);
	void setOffset(double offset);

	void step(double dt);
	roadmanager::Position getPosition();
	double getSpeed();
	double getObjectId();
	std::string getObjectName();
	void printState();

	//private:
	int objectId;
	std::string objectName;
	double speed;
	roadmanager::Position position;

};


