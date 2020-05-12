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

void Event::Start()
{
	for (size_t i = 0; i < action_.size(); i++)
	{
		action_[i]->Start();
	}
	StoryBoardElement::Start();
}

void Event::End()
{
	for (size_t i = 0; i < action_.size(); i++)
	{
		action_[i]->End();
	}
	StoryBoardElement::End();
}

void Event::Stop()
{
	for (size_t i = 0; i < action_.size(); i++)
	{
		action_[i]->Stop();
	}
	StoryBoardElement::Stop();
}

bool scenarioengine::OSCManeuver::IsAnyEventActive()
{
	for (size_t i = 0; i < event_.size(); i++)
	{
		if (event_[i]->IsActive())
		{
			return true;
		}
	}
	return false;
}
