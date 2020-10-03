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

#include "ControllerExternal.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

ControllerExternal::ControllerExternal(Controller::Type type, std::string name, Entities* entities,
	ScenarioGateway* gateway, Parameters* parameters, OSCProperties properties) :
	Controller(type, name, entities, gateway)
{
	if (properties.ValueExists("ghost"))
	{
		std::string ghost_name = properties.GetValueStr("ghost");
		ghost_ = entities->GetObjectByName(ghost_name);
		if (ghost_ <= 0)
		{
			LOG("Error: Failed to find ghost %s", ghost_name);
		}
	}
	LOG("");
}

void ControllerExternal::Init()
{
	Controller::Init();
}

void ControllerExternal::PostFrame()
{
	Controller::PostFrame();
}

void ControllerExternal::Step(double timeStep)
{
	if (ghost_)
	{
		if (ghost_->trail_.FindClosestPoint(object_->pos_.GetX(), object_->pos_.GetY(),
			object_->trail_closest_pos_[0], object_->trail_closest_pos_[1],
			object_->trail_follow_s_, object_->trail_follow_index_, object_->trail_follow_index_) == 0)
		{
			object_->trail_closest_pos_[2] = object_->pos_.GetZ();
		}
		else
		{
			// Failed find point along trail, copy entity position
			object_->trail_closest_pos_[0] = object_->pos_.GetX();
			object_->trail_closest_pos_[1] = object_->pos_.GetY();
			object_->trail_closest_pos_[2] = object_->pos_.GetZ();
		}
	}

	Controller::Step(timeStep);
}

void ControllerExternal::Activate(int domainMask)
{
	Controller::Activate(domainMask);
}

void ControllerExternal::ReportKeyEvent(int key, bool down)
{
}