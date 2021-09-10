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
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_ACC_TYPE_NAME "ACCController"

namespace scenarioengine
{
	class ScenarioPlayer;

	// base class for controllers
	class ControllerACC: public Controller
	{
	public:
		ControllerACC(InitArgs *args);

		static const char* GetTypeNameStatic() { return CONTROLLER_ACC_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_TYPE_ACC; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double timeStep);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);

	private:
		vehicle::Vehicle vehicle_;
		double timeGap_;  // target headway time
		bool active_;
		double setSpeed_;
		double currentSpeed_;
		DampedSpring regulator_;
	};

	Controller* InstantiateControllerACC(void* args);
}