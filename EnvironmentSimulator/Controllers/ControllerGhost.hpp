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
#include "Parameters.hpp"

#define CONTROLLER_GHOST_NAME "GhostController"

namespace scenarioengine
{
	// base class for controllers
	class ControllerGhost: public Controller
	{
	public:
		ControllerGhost(std::string name, Entities* entities, ScenarioGateway* gateway,
			Parameters* parameters, OSCProperties* properties);

		static const char* GetTypeNameStatic() { return CONTROLLER_GHOST_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_TYPE_GHOST; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double timeStep);
		void PostFrame();
		void Activate(int domainMask);
		void ReportKeyEvent(int key, bool down);
	};

	Controller* InstantiateControllerGhost(std::string name, Entities* entities, ScenarioGateway* gateway,
		Parameters* parameters, OSCProperties* properties);
}