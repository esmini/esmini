#pragma once

#include "OSCPrivateAction.hpp"
#include "Car.hpp"
#include "Cars.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class PrivateAction
{
public:
	PrivateAction(OSCPrivateAction &privateAction, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities);
	void setStartTime(double simulationTime);
	void ExecuteAction(double simulationTime, double timeStep);
	void executeSinusoidal(double simulationTime);
	void executeSpeed(double simulationTime, double timeStep);
	bool getFirstRun();

	OSCPrivateAction privateAction;
	Cars * carsPtr;
	std::vector<int> storyId;
	std::vector<std::string> actionEntities;

	std::vector<int> actionEntitiesIds;

	std::string actionType;
	bool ActionCompleted;
	bool startAction;

	double startTime;
	double endTime;
	bool firstRun;

	double newOffset;

	// Sinusoidal
	double n;
	double f;
};

