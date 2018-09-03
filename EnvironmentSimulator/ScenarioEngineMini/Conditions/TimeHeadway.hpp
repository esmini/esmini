#pragma once
#include "OSCCondition.hpp"
#include "OSCPosition.hpp"
#include "Car.hpp"
#include "Cars.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>


class TimeHeadway
{
public:
	TimeHeadway(OSCCondition &condition, Cars &cars, std::vector<int> storyId,std::vector<std::string> &actionEntities);

	bool checkTimeHeadway();

	//~TimeHeadway();

	OSCCondition condition;
	Cars * carsPtr;
	std::vector<int> storyId;
	std::vector<std::string> actionEntities;

	// Number of TriggeringEntities
	unsigned N;

	// Triggering entities
	std::vector<std::string> triggeringEntities;
	std::vector<roadmanager::Position> triggeringEntityPos;

	// Entity
	std::string entity;
	roadmanager::Position entityPos;
	double entitySpeed;

	// Headwaytime
	std::vector<double> headwayTimeOld;
	std::vector<double> headwayTimeNew;
	std::vector<bool> triggs;

};

