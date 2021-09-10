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

#include <string>
#include "Controller.hpp"
#include "pugixml.hpp"
#include "Parameters.hpp"
#include "Entities.hpp"

#define CONTROLLER_REL2ABS_TYPE_NAME "ControllerRel2Abs"

namespace scenarioengine
{
	class ScenarioEngine;

	// base class for controllers
	class ControllerRel2Abs: public Controller
	{
	public:
		struct PreSimData
		{
			std::vector<double> time;
			std::vector<double> speeds;
			std::vector<double> posX;
			std::vector<double> posY;
		};

		struct position_copy
		{
			Object* object;
			roadmanager::Position* pos;
			double speed;
			int dirtyBits;
		};

		enum ctrl_mode {
			RELATIVE,
			ABSOLUTE
		};

		std::vector<PreSimData> data_vector;
		PreSimData actualData;
		PreSimData data;
		double timestamp;

		// Presim parameters
		double pred_horizon;
		double pred_timestep;
		double pred_nbr_timesteps;
		int action_whitelist[7] = { 0, 1, 2, 3, 4, 11, 12 };
		double switching_threshold_dist;
		double switching_threshold_speed;

		//CSV loggning
		std::ofstream logData;
		int csv_iter;

		ControllerRel2Abs(InitArgs* args);

		void Init();
		void Step(double timeStep);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);
		void SetScenarioEngine(ScenarioEngine* scenarioEngine) { scenarioEngine_ = scenarioEngine; };
		void CopyPosition(Object* object, position_copy* obj_copy);

		static const char* GetTypeNameStatic() { return CONTROLLER_REL2ABS_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return Controller::Type::CONTROLLER_TYPE_REL2ABS; }
		virtual int GetType() { return GetTypeStatic(); }

	private:
		int ego_obj;
		double prev_ego_speed;
		double prev_target_speed;
		bool switchNow;
		//vector with pairs of timestep and speed before that timestep
		std::vector<std::pair<double, double>> speeds;

		//alters ego_obj to correct entities index for ego
		void findEgo();

		ScenarioEngine* scenarioEngine_;
	};

	Controller* InstantiateControllerRel2Abs(void* args);
}