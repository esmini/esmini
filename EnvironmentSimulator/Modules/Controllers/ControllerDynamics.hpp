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

 /*
  * This controller let the user control the vehicle interactively by the arrow keys
  */


#pragma once

#include <string>
#include "Controller.hpp"
#include "Parameters.hpp"
#include "DynamicVehicle.hpp"


#define CONTROLLER_DYNAMICS_TYPE_NAME "DynamicsController"

namespace scenarioengine
{

	// base class for controllers
	class ControllerDynamics: public Controller
	{
	public:

		ControllerDynamics(InitArgs* args);
		~ControllerDynamics();

		void Init();
		void Step(double timeStep);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);

		static const char* GetTypeNameStatic() { return CONTROLLER_DYNAMICS_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static int GetTypeStatic() { return Controller::Type::CONTROLLER_TYPE_DYNAMICS; }
		virtual int GetType() { return GetTypeStatic(); }

	private:
		dynamicvehicle::DynamicVehicle vehicle_;
		double length_;
		double width_;
		double height_;
		double mass_;
		double suspension_stiffness_;
		double friction_slip_;
		double roll_influence_;
	};

	Controller* InstantiateControllerDynamics(void* args);
}