#pragma once
#include "OSCCondition.hpp"
#include "OSCPosition.hpp"
#include "Car.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>


class TimeHeadway
{
public:
	TimeHeadway(OSCCondition &condition, std::vector<Car> &carVector, std::vector<int> storyId,std::vector<std::string> &actionEntities);

	bool checkTimeHeadway();
	int getObjectId(std::string objectName);

	//~TimeHeadway();

	OSCCondition condition;
	std::vector<Car> * carVectorPtr;
	std::vector<int> storyId;
	std::vector<std::string> actionEntities;

	// Number of TriggeringEntities
	unsigned N;

	// Triggering entities
	std::vector<int> triggeringEntityIds;
	std::vector<roadmanager::Position> triggeringEntityPos;

	// Entity
	int entityId;
	roadmanager::Position entityPos;
	double entitySpeed;

	// Headwaytime
	std::vector<double> headwayTimeOld;
	std::vector<double> headwayTimeNew;
	std::vector<bool> triggs;

};

