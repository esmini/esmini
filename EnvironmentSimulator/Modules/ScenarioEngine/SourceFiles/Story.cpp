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

#include "Story.hpp"
#include "CommonMini.hpp"

using namespace scenarioengine;


Act* Story::FindActByName(std::string name)
{
	for (size_t i = 0; i < act_.size(); i++)
	{
		if (name == act_[i]->name_)
		{
			return act_[i];
		}
	}

	return nullptr;
}

Event* Story::FindEventByName(std::string name)
{
	for (size_t i = 0; i < act_.size(); i++)
	{
		for (size_t j = 0; j < act_[i]->maneuverGroup_.size(); j++)
		{
			for (size_t k = 0; k < act_[i]->maneuverGroup_[j]->maneuver_.size(); k++)
			{
				for (size_t l = 0; l < act_[i]->maneuverGroup_[j]->maneuver_[k]->event_.size(); l++)
				{
					Event *event = act_[i]->maneuverGroup_[j]->maneuver_[k]->event_[l];
					if (name == event->name_)
					{
						return event;
					}
				}
			}
		}
	}

	return nullptr;
}

OSCAction * Story::FindActionByName(std::string name)
{
	for (size_t i = 0; i < act_.size(); i++)
	{
		for (size_t j = 0; j < act_[i]->maneuverGroup_.size(); j++)
		{
			for (size_t k = 0; k < act_[i]->maneuverGroup_[j]->maneuver_.size(); k++)
			{
				for (size_t l = 0; l < act_[i]->maneuverGroup_[j]->maneuver_[k]->event_.size(); l++)
				{
					for (size_t m = 0; m < act_[i]->maneuverGroup_[j]->maneuver_[k]->event_[l]->action_.size(); m++)
					{
						OSCAction *action = act_[i]->maneuverGroup_[j]->maneuver_[k]->event_[l]->action_[m];
						if (name == action->name_)
						{
							return action;
						}
					}
				}
			}
		}
	}

	return nullptr;
}

void Story::Print()
{
	LOG("Story: %s", name_.c_str());
}

Act* StoryBoard::FindActByName(std::string name)
{
	Act *act = 0;
	for (size_t i = 0; i < story_.size(); i++)
	{
		if ((act = story_[i]->FindActByName(name)) != 0)
		{
			return act;
		}
	}

	return 0;
}

Event* StoryBoard::FindEventByName(std::string name)
{
	Event *event = 0;
	for (size_t i = 0; i < story_.size(); i++)
	{
		if ((event = story_[i]->FindEventByName(name)) != 0)
		{
			return event;
		}
	}

	return 0;
}

OSCAction* StoryBoard::FindActionByName(std::string name)
{
	OSCAction *action = 0;
	for (size_t i = 0; i < story_.size(); i++)
	{
		if ((action = story_[i]->FindActionByName(name)) != 0)
		{
			return action;
		}
	}

	return 0;
}

void StoryBoard::Print()
{
	LOG("Storyboard:");
	for (size_t i = 0; i < story_.size(); i++)
	{
		story_[i]->Print();
	}
}

