#pragma once
#include "Catalogs.hpp"
#include "Entities.hpp"
#include "Init.hpp"
#include "Story.hpp"
#include "Conditions.hpp"
#include "Condition.hpp"
#include "Actions.hpp"
#include "Action.hpp"
#include "Cars.hpp"
#include "ScenarioGateway.hpp"


#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class ScenarioEngine
{
public:

	ScenarioEngine(Catalogs &catalogs, Entities &entities, Init &init, std::vector<Story> &story, double startTime);

	void initCars();
	void initInit();
	void printCars();

	void setSimulationTime(double simulationTime);
	void setTimeStep(double timeStep);
	void printSimulationTime();
	void stepObjects(double dt);


	ScenarioGateway & getScenarioGateway();

	void initRoute();
	void initConditions();
	void checkConditions();
	void executeActions();
	
	//private:
	// OpenSCENARIO parameters
	Catalogs catalogs;
	Entities entities;
	Init init;
	std::vector<Story> story;

	// Simulation parameters
	double startTime;
	double simulationTime;
	double timeStep;

	// 
	Conditions conditions;
	Actions actions;
	Cars cars;
	ScenarioGateway scenarioGateway;

	// Route
	roadmanager::Route route;

};

