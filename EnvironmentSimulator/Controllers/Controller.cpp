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

#include "Controller.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"


using namespace scenarioengine;

Controller* scenarioengine::InstantiateController(std::string name, Entities* entities, 
	ScenarioGateway* gateway, Parameters* parameters, OSCProperties* properties)
{
	LOG("The base class should not be instantiated");
	return 0;
}

void Controller::Step(double timeStep)
{
	if (object_)
	{
		object_->dirty_lat_ = domain_ & ControllerDomain::CTRL_LATERAL;
		object_->dirty_long_ = domain_ & ControllerDomain::CTRL_LONGITUDINAL;
	}
}

void Controller::Assign(Object* object)
{
	if (object == 0)
	{
		if (object_)
		{
			// Detach any existing object from controller
			object_->SetAssignedController(0);
		}
		object_ = 0;
	}
	else
	{
		object_ = object;
		object_->SetAssignedController(this);
	}
}

bool Controller::Active()
{
	return (object_ != 0 && domain_ != 0);
}

void Controller::ReportKeyEvent(int key, bool down)
{
	LOG("Key %c %s", key, down ? "down" : "up");
}
