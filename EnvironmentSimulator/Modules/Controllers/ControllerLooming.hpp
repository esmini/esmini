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

#define CONTROLLER_LOOMING_TYPE_NAME "LoomingController"

namespace scenarioengine
{
	class ControllerLooming: public Controller
	{
	public:
		ControllerLooming(InitArgs *args);
		// ControllerLooming();
		static const char* GetTypeNameStatic() { return CONTROLLER_LOOMING_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static int GetTypeStatic() { return CONTROLLER_TYPE_LOOMING; }
		virtual int GetType() { return GetTypeStatic(); }


		void Init();
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);
		void SetSetSpeed(double setSpeed) { setSpeed_ = setSpeed; }
		void Step(double timeStep);
		double checkAngle(double angle);

	private:
		vehicle::Vehicle vehicle_;
		bool active_;
		double timeGap_;  // target headway time
		double setSpeed_;
		double currentSpeed_;
		bool setSpeedSet_;
		double nearAngle = 0.0;
		double farAngle = 0.0;
		double prevNearAngle = 0.0;
		double prevFarAngle = 0.0;
		double steering = 0.0;
		double prevTime = 0.0;
		double acc = 0.0;
		double steering_rate_; 
	};
	
	Controller* InstantiateControllerLooming(void* args);
}