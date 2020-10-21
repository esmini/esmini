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

#define CONTROLLER_SLOPPY_DRIVER_TYPE_NAME "SloppyDriverController"

namespace scenarioengine
{
	// base class for controllers
	class ControllerSloppyDriver: public Controller
	{
	public:

		ControllerSloppyDriver(InitArgs* args);

		static const char* GetTypeNameStatic() { return CONTROLLER_SLOPPY_DRIVER_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return Controller::Type::CONTROLLER_TYPE_SLOPPY_DRIVER; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double timeStep);
		void Activate(int domainMask);
		void ReportKeyEvent(int key, bool down);


	private:
		double sloppiness_;  // range [0-1], default = 0.5
		OSCProperties properties_;
		double initSpeed_;
		double speedTimer_;
		double speedTimerDuration_;
		DampedSpring speedFilter_;
		double relativeH_;
		double initT_;
		double lateralTimer_;
		double lateralTimerDuration_;
		DampedSpring lateralFilter_;
		const char* type_name_ = "SloppyDriver";
	};

	Controller* InstantiateControllerSloppyDriver(void* args);
}