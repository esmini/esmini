#pragma once

#include "Entities.hpp"
#include "Init.hpp"
#include "Story.hpp"
#include "Car.hpp"
#include "TimeHeadway.hpp"
#include "PrivateAction.hpp"
#include "Cars.hpp"


#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class ScenarioEngine
{
public:

	ScenarioEngine(Entities &entities, Init &init, std::vector<Story> &story, double startTime);

	void initCars();
	void printCars();

	void setSimulationTime(double simulationTime);
	void setTimeStep(double timeStep);
	void printSimulationTime();
	void stepObjects(double dt);

	void initConditions();
	void checkConditions();
	void executeActions();
	
	//private:
	Entities entities;
	Init init;
	std::vector<Story> story;
	double startTime;
	double simulationTime;
	double timeStep;

	std::vector<Car> carVector;
	std::vector<TimeHeadway> timeHeadwayVector;
	std::vector<PrivateAction> actionVector;
	Cars cars;

};

