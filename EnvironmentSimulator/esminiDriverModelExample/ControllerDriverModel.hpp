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
#include "vehicle.hpp"

#define CONTROLLER_DRIVER_MODEL_NAME "DriverModelController"
#define CONTROLLER_TYPE_DRIVER_MODEL 1

namespace scenarioengine
{
	// base class for controllers
	class ControllerDriverModel: public Controller
	{
	public:
		ControllerDriverModel(std::string name, Entities* entities, ScenarioGateway* gateway,
			Parameters* parameters, OSCProperties* properties);

		static const char* GetTypeNameStatic() { return CONTROLLER_DRIVER_MODEL_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		
		// Make sure to define a unique ID for your controller
		static const int GetTypeStatic() { return Controller::Type::USER_CONTROLLER_TYPE_BASE + CONTROLLER_TYPE_DRIVER_MODEL; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double timeStep);
		void PostFrame();
		void Activate(int domainMask);
		void ReportKeyEvent(int key, bool down);

	private:
		vehicle::Vehicle vehicle_;
	};

	Controller* InstantiateControllerDriverModel(std::string name, Entities* entities, ScenarioGateway* gateway,
		Parameters* parameters, OSCProperties* properties);
}