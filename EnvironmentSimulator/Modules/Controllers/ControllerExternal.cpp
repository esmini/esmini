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


Controller* scenarioengine::InstantiateControllerExternal(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerExternal(initArgs);
}

ControllerExternal::ControllerExternal(InitArgs* args) : useGhost_(false), Controller(args)
{
	if (args->properties->ValueExists("useGhost"))
	{
		useGhost_ = (args->properties->GetValueStr("useGhost") == "true" ||
			args->properties->GetValueStr("useGhost") == "True");
	}

	if (args->properties->ValueExists("headstartTime"))
	{
		headstart_time_ = strtod(args->properties->GetValueStr("headstartTime"));
	}

	// External controller forced into override mode - will not perform any scenario actions
	if (mode_ != Mode::MODE_OVERRIDE)
	{
		LOG("External controller mode \"%s\" not applicable. Using override mode instead.", Mode2Str(mode_).c_str());
		mode_ = Controller::Mode::MODE_OVERRIDE;
	}
}

void ControllerExternal::Init()
{
	if (object_)
	{
		object_->SetHeadstartTime(headstart_time_);
	}
	else if (useGhost_)
	{
		LOG("External controller with ghost needs to be assigned by ObjectController");
		LOG("  in the Init section, in order for the headstart time to be correctly registered.");
		LOG("  -> ignoring the headstartTime attribute.");
	}

	Controller::Init();
}

void ControllerExternal::Step(double timeStep)
{
	if (object_->ghost_)
	{
		if (object_->ghost_->trail_.FindClosestPoint(object_->pos_.GetX(), object_->pos_.GetY(),
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