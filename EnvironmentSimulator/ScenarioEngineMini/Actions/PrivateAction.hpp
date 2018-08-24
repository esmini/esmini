#pragma once

#include "OSCPrivateAction.hpp"
#include "Car.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class PrivateAction
{
public:
	PrivateAction(OSCPrivateAction &privateAction, std::vector<Car> &carVector, std::vector<int> storyId, std::vector<std::string> &actionEntities);
	//~PrivateAction();
	void setStartTime(double simulationTime);
	void ExecuteAction(double simulationTime, double timeStep);
	void executeSinusoidal(double simulationTime);
	void executeSpeed(double simulationTime, double timeStep);
	int getObjectId(std::string objectName);
	bool getFirstRun();

	OSCPrivateAction privateAction;
	std::vector<Car> * carVectorPtr;
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

