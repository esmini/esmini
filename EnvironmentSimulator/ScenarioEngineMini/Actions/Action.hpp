#pragma once

#include "OSCPrivateAction.hpp"
#include "Cars.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class Action
{
public:
	Action(OSCPrivateAction &privateAction, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities);

	// Public methods
	void ExecuteAction(double simulationTime, double timeStep);
	void setStartTime(double simulationTime);
	void setStartAction();
	bool getStartAction();
	bool getActionCompleted();
	std::vector<int> getStoryId();

private:

	// Private methods
	void identifyActionType(OSCPrivateAction privateAction);
	void executeSinusoidal(double simulationTime);
	void executeSpeed(double simulationTime, double timeStep);

	// Constructor variables
	OSCPrivateAction privateAction;
	Cars * carsPtr;
	std::vector<int> storyId;
	std::vector<std::string> actionEntities;

	// General class variables
	std::string actionType;
	bool startAction;
	bool actionCompleted;
	double startTime;
	bool firstRun;

	// Sinusoidal
	std::string targetObject;
	double targetValue;
	double time;
	double f;
	std::vector<int> initialOffsets;


	// Speed
	double speedRate;

};

