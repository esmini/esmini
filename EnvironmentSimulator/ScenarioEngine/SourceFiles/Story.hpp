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

#include "OSCManeuver.hpp"
#include "OSCConditionGroup.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace scenarioengine
{

	struct ActSequence  // Act Sequence
	{
		typedef struct
		{
			Object *object_;
			// By condition not supported yet
		} Actor;

		std::vector<Actor*> actor_;

		// std::vector<OSCCatalogReference> catalog_reference_;  // not supported yet
		std::vector<OSCManeuver*> maneuver_;

		int number_of_executions_;
		std::string name_;
	};

	class Act
	{
	public:
		typedef enum
		{
			INACTIVE,
			ACTIVATED,      // Just activated - this state last for one step
			ACTIVE,
			DEACTIVATED,    // Just done/deactivated - this state last for one step
		} State;

		State state_;

		std::vector<ActSequence*> sequence_;
		std::vector<OSCConditionGroup*> start_condition_group_;
		std::vector<OSCConditionGroup*> end_condition_group_;
		std::vector<OSCConditionGroup*> cancel_condition_group_;

		std::string name_;

		Act() : state_(State::INACTIVE) {}

		bool IsActive()
		{
			return state_ == State::ACTIVATED || state_ == State::ACTIVE;
		}

		void Trig()
		{
			state_ = State::ACTIVATED;
			LOG("Act %s trigged", name_.c_str());
		}

		void Stop()
		{
			if (IsActive())
			{
				LOG("Act %s stopped", name_.c_str());
				state_ = State::DEACTIVATED;
			}
		}

	};

	class Story
	{
	public:
		Story(std::string name, std::string owner);

		Act* FindActByName(std::string name);
		Event* FindEventByName(std::string name);
		OSCAction* FindActionByName(std::string name);
		void Print();

		std::vector<Act*> act_;

		std::string owner_;
		std::string name_;
	};

	class StoryBoard
	{
	public:
		Act* FindActByName(std::string name);
		Event* FindEventByName(std::string name);
		OSCAction* FindActionByName(std::string name); 
		void Print();

		std::vector<Story*> story_;
	};
}