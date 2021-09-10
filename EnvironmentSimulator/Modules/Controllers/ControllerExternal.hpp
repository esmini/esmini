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

#define CONTROLLER_EXTERNAL_TYPE_NAME "ExternalController"

namespace scenarioengine
{
	// base class for controllers
	class ControllerExternal: public Controller
	{
	public:
		ControllerExternal(InitArgs* args);

		static const char* GetTypeNameStatic() { return CONTROLLER_EXTERNAL_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_TYPE_EXTERNAL; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double timeStep);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);
		bool UseGhost() { return useGhost_; }

	private:
		bool useGhost_;
		double headstart_time_;
	};

	Controller* InstantiateControllerExternal(void* args);

}