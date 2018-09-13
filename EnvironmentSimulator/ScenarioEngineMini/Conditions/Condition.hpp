#pragma once
#include "OSCCondition.hpp"
#include "OSCPosition.hpp"
#include "Cars.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>


class Condition
{
public:
	Condition(OSCCondition &condition, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities);

	bool checkCondition();
	std::vector<int> getStoryId();

private:

	// Private methods
	void identifyConditionType(OSCCondition &condition);
	bool checkTimeHeadway();

	// Constructor variables
	OSCCondition condition;
	Cars * carsPtr;
	std::vector<int> storyId;
	std::vector<std::string> actionEntities;

	// General class variables
	std::string conditionType;

	// Entity
	std::string entity;
	roadmanager::Position entityPos;
	double entitySpeed;

	// Headwaytime
	std::vector<double> headwayTimeOld;
	std::vector<double> headwayTimeNew;
	std::vector<bool> triggs;

	// Triggering entities
	std::vector<std::string> triggeringEntities;
	std::vector<roadmanager::Position> triggeringEntityPos;
	unsigned N; // Number of TriggeringEntities
};

