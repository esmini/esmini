#pragma once
#include "OSCPrivateAction.hpp"

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
	void setPosition(OSCPosition position, std::string posType);
	void setSpeed(double speed);
	void setOffset(double offset);

	void step(double dt);
	double getSpeed();
	OSCPosition getPosition();
	std::string getPositionType();
	double getOffset();
	double getObjectId();
	std::string getObjectName();
	void printState();

	//private:
	int objectId;
	std::string objectName;
	OSCPosition position;
	std::string	posType;
	double speed;
};


