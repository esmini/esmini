#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include "Catalogs.hpp"
#include "Entities.hpp"
#include "Init.hpp"
#include "Story.hpp"
#include "ScenarioGateway.hpp"
#include "ScenarioReader.hpp"
#include "RoadNetwork.hpp"



class ScenarioEngine
{
public:

	Entities entities;

//	Cars cars;

	ScenarioEngine(std::string oscFilename, double startTime);
	ScenarioEngine() {};
	~ScenarioEngine();

	void InitScenario(std::string oscFilename, double startTime); // use with default constructor
		
	void step(double deltaSimTime, bool initial = false);
	void setSimulationTime(double simulationTime);
	void setTimeStep(double timeStep);
	void printSimulationTime();
	void stepObjects(double dt);


	std::string getSceneGraphFilename() { return roadNetwork.SceneGraph.filepath; }
	std::string getOdrFilename() { return roadNetwork.Logics.filepath; }
	roadmanager::OpenDrive *getRoadManager() { return odrManager; }

	ScenarioGateway *getScenarioGateway();
	
private:
	// OpenSCENARIO parameters
	Catalogs catalogs;
	Init init;
	std::vector<Story*> story;
	ScenarioReader scenarioReader;
	RoadNetwork roadNetwork;
	roadmanager::OpenDrive *odrManager;


	// Simulation parameters
	double startTime;
	double simulationTime;
	double timeStep;

	// 
	
	//Conditions conditions;
	//Actions actions;
	ScenarioGateway scenarioGateway;
};

