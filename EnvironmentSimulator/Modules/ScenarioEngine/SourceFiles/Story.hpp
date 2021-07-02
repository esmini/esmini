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
#include "OSCCondition.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace scenarioengine
{

	class ManeuverGroup : public StoryBoardElement
	{
	public:
		typedef struct
		{
			Object *object_;
		} Actor;

		ManeuverGroup() : StoryBoardElement(StoryBoardElement::ElementType::MANEUVER_GROUP) {}
		Object* FindActorByName(std::string name)
		{
			for (size_t i = 0; i < actor_.size(); i++)
			{
				if (actor_[i]->object_->name_ == name)
				{
					return actor_[i]->object_;
				}
			}
			return 0;
		}

		bool IsObjectActor(Object* object)
		{
			for (size_t i = 0; i < actor_.size(); i++)
			{
				if (actor_[i]->object_ == object)
				{
					return true;
				}
			}
			return false;
		}

		std::vector<Actor*> actor_;
		std::vector<OSCManeuver*> maneuver_;

		std::string name_;
	};

	class Act: public StoryBoardElement
	{
	public:

		std::vector<ManeuverGroup*> maneuverGroup_;
		Trigger *start_trigger_;
		Trigger *stop_trigger_;

		Act() : start_trigger_(0), stop_trigger_(0), StoryBoardElement(StoryBoardElement::ElementType::ACT) {}
	};

	class Story
	{
	public:
		Story(std::string name) {}

		OSCParameterDeclarations parameter_declarations_;
		Act* FindActByName(std::string name);
		Event* FindEventByName(std::string name);
		OSCAction* FindActionByName(std::string name);
		void Print();

		std::vector<Act*> act_;
		std::string name_;
	};

	class StoryBoard
	{
	public:
		StoryBoard() : stop_trigger_(0) {}
		Act* FindActByName(std::string name);
		Event* FindEventByName(std::string name);
		OSCAction* FindActionByName(std::string name);
		Entities* entities_;
		void Print();

		std::vector<Story*> story_;
		Trigger *stop_trigger_;
	};
}