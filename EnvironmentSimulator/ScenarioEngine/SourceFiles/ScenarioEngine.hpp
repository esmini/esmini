/* 
 * esmini - Environment Simulator Minimalistic 
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

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



namespace scenarioengine
{
	#define DEFAULT_HEADSTART_TIME 1.0

	class ScenarioEngine
	{
	public:

		typedef enum
		{
			CONTROL_BY_OSC,
			CONTROL_INTERNAL,
			CONTROL_EXTERNAL,
			CONTROL_HYBRID
		} RequestControlMode;

		Entities entities;

		//	Cars cars;

		ScenarioEngine(std::string oscFilename, double headstart_time = DEFAULT_HEADSTART_TIME, RequestControlMode control_mode_first_vehicle = CONTROL_BY_OSC);
		ScenarioEngine(const pugi::xml_document &xml_doc, double headstart_time = DEFAULT_HEADSTART_TIME, RequestControlMode control_mode_first_vehicle = CONTROL_BY_OSC);
		ScenarioEngine() {};
		~ScenarioEngine();

		void InitScenario(std::string oscFilename, double headstart_time, RequestControlMode control_mode_first_vehicle = CONTROL_BY_OSC);
		void InitScenario(const pugi::xml_document &xml_doc, double headstart_time, RequestControlMode control_mode_first_vehicle = CONTROL_BY_OSC);

		void step(double deltaSimTime, bool initial = false);
		void printSimulationTime();
		void stepObjects(double dt);
		void exit();

		std::string getScenarioFilename() { return scenarioReader->getScenarioFilename(); }
		std::string getSceneGraphFilename() { return roadNetwork.SceneGraph.filepath; }
		std::string getOdrFilename() { return roadNetwork.Logics.filepath; }
		roadmanager::OpenDrive *getRoadManager() { return odrManager; }

		ScenarioGateway *getScenarioGateway();
		Object::Control RequestControl2ObjectControl(RequestControlMode control);
		double getSimulationTime() { return simulationTime; }
		bool GetQuitFlag() { return quit_flag; }

	private:
		// OpenSCENARIO parameters
		Catalogs catalogs;
		Init init;
		ScenarioReader *scenarioReader;
		StoryBoard storyBoard;
		RoadNetwork roadNetwork;
		roadmanager::OpenDrive *odrManager;

		// Simulation parameters
		double simulationTime;
		double headstart_time_;

		ScenarioGateway scenarioGateway;

		// execution control flags
		bool quit_flag;

		void parseScenario(RequestControlMode control_mode_first_vehicle = CONTROL_BY_OSC);
		void ResolveHybridVehicles();
	};

}
