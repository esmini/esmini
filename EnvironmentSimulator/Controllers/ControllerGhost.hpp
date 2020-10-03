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


namespace scenarioengine
{
	// base class for controllers
	class ControllerGhost: public Controller
	{
	public:

		ControllerGhost(Controller::Type type, std::string name, Entities *entities, ScenarioGateway* gateway, OSCProperties properties);

		void Init();
		void Step(double timeStep);
		void PostFrame();
		void Activate(int domainMask);
		void ReportKeyEvent(int key, bool down);
	};
}