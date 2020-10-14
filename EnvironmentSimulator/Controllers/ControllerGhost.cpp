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
 * This controller simulates a bad or dizzy driver by manipulating 
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

#include "ControllerGhost.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerGhost(std::string name, Entities* entities, ScenarioGateway* gateway,
	Parameters* parameters, OSCProperties* properties)
{
	return new ControllerGhost(name, entities, gateway, parameters, properties);
}

ControllerGhost::ControllerGhost(std::string name, Entities* entities, ScenarioGateway* gateway,
	Parameters* parameters, OSCProperties* properties) :
	Controller(name, entities, gateway, parameters, properties)
{
	if (properties->ValueExists("headstartTime"))
	{
		headstart_time_ = strtod(properties->GetValueStr("headstartTime"));
	}
}

void ControllerGhost::Init()
{
	Controller::Init();
}

void ControllerGhost::PostFrame()
{
	Controller::PostFrame();
}

void ControllerGhost::Step(double timeStep)
{

	object_->pos_.SetY(100);
	Controller::Step(timeStep);
}

void ControllerGhost::Activate(int domainMask)
{

	Controller::Activate(domainMask);
}

void ControllerGhost::ReportKeyEvent(int key, bool down)
{

}