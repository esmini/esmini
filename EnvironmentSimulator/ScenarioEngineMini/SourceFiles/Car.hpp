#pragma once
#include "OSCPrivateAction.hpp"
#include "Entities.hpp"
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
	void setPosition(roadmanager::Position position);
	void setSpeed(double speed);
	void setObjectStruct(Entities::ObjectStruct objectStruct);
	void setOffset(double offset);
	void setExtControlled(bool Boolean);
	bool getExtControlled();

	void step(double dt);
	roadmanager::Position getPosition();
	double getSpeed();
	double getObjectId();
	std::string getObjectName();
	void printState();

	//private:
	int objectId;
	std::string objectName;
	Entities::ObjectStruct objectStruct;
	double speed;
	roadmanager::Position position;
	bool extControlled;
};


