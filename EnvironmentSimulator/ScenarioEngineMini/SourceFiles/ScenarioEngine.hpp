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
#include "ScenarioReader.hpp"
#include "RoadNetwork.hpp"


#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class ScenarioEngine
{
public:

	ScenarioEngine(std::string oscFilename, double startTime);
		
	void step(double deltaSimTime);
	void initCars();
	void initInit();
	void printCars();

	void setSimulationTime(double simulationTime);
	void setTimeStep(double timeStep);
	void printSimulationTime();
	void stepObjects(double dt);

	std::string getSceneGraphFilename() { return roadNetwork.SceneGraph.filepath; }
	std::string getOdrFilename() { return roadNetwork.Logics.filepath; }
	roadmanager::OpenDrive *getRoadManager() { return odrManager; }

	ScenarioGateway *getScenarioGateway();

	void initRoutes();
	void initConditions();
	void checkConditions();
	void executeActions();
	
	//private:
	// OpenSCENARIO parameters
	Catalogs catalogs;
	Entities entities;
	Init init;
	std::vector<Story> story;
	ScenarioReader scenarioReader;
	RoadNetwork roadNetwork;
	roadmanager::OpenDrive *odrManager;


	// Simulation parameters
	double startTime;
	double simulationTime;
	double timeStep;

	// 
	Conditions conditions;
	Actions actions;
	Cars cars;
	ScenarioGateway scenarioGateway;


};

