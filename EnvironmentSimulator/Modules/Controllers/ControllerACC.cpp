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

#include "ControllerACC.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerACC(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerACC(initArgs);
}

ControllerACC::ControllerACC(InitArgs* args) : Controller(args)
{
	if (args->properties->ValueExists("timeGap"))
	{
		timeGap_ = strtod(args->properties->GetValueStr("timeGap"));
	}
	mode_ = Mode::MODE_ADDITIVE;
}

void ControllerACC::Init()
{
	Controller::Init();
}

void ControllerACC::Step(double timeStep)
{
	// Find any vehicle within 2 x timeGap seconds headway
	for (size_t i = 0; i < entities_->object_.size(); i++)
	{
		// Measure distance along road
		roadmanager::PositionDiff posDiff;
//		entities_->object_[i]->pos_.( Delta(&object_->pos_, posDiff);
		entities_->object_[i]->pos_.Delta(&object_->pos_, posDiff);
	}

	gateway_->reportObjectSpeed(object_->GetId(), 10);

	Controller::Step(timeStep);
}

void ControllerACC::Activate(int domainMask)
{
	Controller::Activate(domainMask);
}

void ControllerACC::ReportKeyEvent(int key, bool down)
{
}