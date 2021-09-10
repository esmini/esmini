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

#define CONTROLLER_SUMO_TYPE_NAME "SumoController"

namespace scenarioengine
{
	// base class for controllers
	class ControllerSumo: public Controller
	{
	public:
		ControllerSumo(InitArgs* args);

		static const char* GetTypeNameStatic() { return CONTROLLER_SUMO_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_TYPE_SUMO; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double time);
		void Activate(ControlDomains domainMask);

		void SetSumoVehicle(Object* object);

	private:
		float sumo_x_offset_;
		float sumo_y_offset_;
		double time_;
		pugi::xml_document docsumo_;
		std::string model_filepath_;
		Object* template_vehicle_;
	};

	Controller* InstantiateControllerSumo(void* args);
}