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
	typedef void (*ParamDeclCallbackFunc)(void*);

	static struct
	{
		ParamDeclCallbackFunc func;
		void* data;
	} paramDeclCallback = {0, 0};

	void RegisterParameterDeclarationCallback(ParamDeclCallbackFunc func, void* data);

	class ScenarioEngine
	{
	public:
		Entities entities;

		ScenarioEngine(std::string oscFilename, bool disable_controllers = false);
		ScenarioEngine(const pugi::xml_document &xml_doc, bool disable_controllers = false);
		~ScenarioEngine();

		void InitScenario(std::string oscFilename, bool disable_controllers = false);
		void InitScenario(const pugi::xml_document &xml_doc, bool disable_controllers = false);

		int step(double deltaSimTime);
		void printSimulationTime();
		void prepareGroundTruth(double dt);
		int defaultController(Object *obj, double dt);

		void ReplaceObjectInTrigger(Trigger *trigger, Object *obj1, Object *obj2, double timeOffset, Event* event = 0);
		void SetupGhost(Object *object);
		void ResetEvents();

		std::string getScenarioFilename() { return scenarioReader->getScenarioFilename(); }
		std::string getSceneGraphFilename() { return roadNetwork.sceneGraphFile.filepath; }
		std::string getOdrFilename() { return roadNetwork.logicFile.filepath; }
		roadmanager::OpenDrive *getRoadManager() { return odrManager; }

		ScenarioGateway *getScenarioGateway();
		double getSimulationTime() { return simulationTime_; }
		bool GetQuitFlag() { return quit_flag; }
		ScenarioReader *scenarioReader;
		ScenarioReader *GetScenarioReader() { return scenarioReader; }
		void SetHeadstartTime(double headstartTime) { headstart_time_ = headstartTime; }
		double GetHeadstartTime() { return headstart_time_; }
		void SetSimulationTime(double time) { simulationTime_ = time; }
		double *GetSimulationTimePtr() { return &simulationTime_; }

		void SetTrueTime(double time) { trueTime_ = time; }
		double GetTrueTime() { return trueTime_; }
		double* GetTrueTimePtr() { return &trueTime_;  }

		//static void TimeSetBack();

		double trueTime_;
		bool doOnce = true;

	private:
		// OpenSCENARIO parameters
		Catalogs catalogs;
		Init init;
		StoryBoard storyBoard;
		RoadNetwork roadNetwork;
		roadmanager::OpenDrive *odrManager;
		bool disable_controllers_;

		// Simulation parameters
		double simulationTime_;
		double headstart_time_;
		Vehicle sumotemplate;
		ScenarioGateway scenarioGateway;

		// execution control flags
		bool quit_flag;
		bool initialized_;

		void parseScenario();
	};

}
