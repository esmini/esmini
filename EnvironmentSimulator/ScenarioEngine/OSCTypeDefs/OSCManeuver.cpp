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

#include "OSCManeuver.hpp"

using namespace scenarioengine;

void Event::Trig()
{
	for (size_t i = 0; i < action_.size(); i++)
	{
		action_[i]->Trig();
	}
	state_ = Event::State::ACTIVATED;
	LOG("Event %s trigged", name_.c_str());
}

void Event::Stop()
{
	state_ = Event::State::DEACTIVATED;
	LOG("Event %s stopped", name_.c_str());
}

bool scenarioengine::Event::Triggable()
{
	if (state_ == Event::State::DEACTIVATED ||
		state_ == Event::State::INACTIVE)
	{
		return true;
	}
	return false;
}

int scenarioengine::OSCManeuver::GetWaitingEventIdx()
{
	int waiting_event = -1;

	for (size_t i = 0; i < event_.size(); i++)
	{
		if (event_[i]->IsActive())
		{
			if (waiting_event != -1)
			{
				LOG("Warning: More than 1 waiting event (%s + %s)- should not happen!", event_[waiting_event]->name_.c_str(), event_[i]->name_.c_str());
			}
			waiting_event = (int)i;
		}
	}
	return waiting_event;
}

int scenarioengine::OSCManeuver::GetActiveEventIdx()
{
	int active_event = -1;

	for (size_t i = 0; i < event_.size(); i++)
	{
		if (event_[i]->IsActive())
		{
			if (active_event != -1)
			{
				LOG("Warning: More than 1 active event (%s + %s)- should not happen!", event_[active_event]->name_.c_str(), event_[i]->name_.c_str());
			}
			active_event = (int)i;
		}
	}
	return active_event;
}
