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
#include "vehicle.hpp"


#define CONTROLLER_INTERACTIVE_TYPE_NAME "InteractiveController"

namespace scenarioengine
{
	// base class for controllers
	class ControllerInteractive: public Controller
	{
	public:

		ControllerInteractive(InitArgs* args) : Controller(args) {}

		void Init();
		void Step(double timeStep);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);

		static const char* GetTypeNameStatic() { return CONTROLLER_INTERACTIVE_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return Controller::Type::CONTROLLER_TYPE_INTERACTIVE; }
		virtual int GetType() { return GetTypeStatic(); }

	private:
		vehicle::Vehicle vehicle_;
		vehicle::THROTTLE accelerate = vehicle::THROTTLE_NONE;
		vehicle::STEERING steer = vehicle::STEERING_NONE;

	};

	Controller* InstantiateControllerInteractive(void* args);
}