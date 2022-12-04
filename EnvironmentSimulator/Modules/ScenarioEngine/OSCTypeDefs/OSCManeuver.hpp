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

#include "OSCGlobalAction.hpp"
#include "OSCPrivateAction.hpp"
#include "OSCParameterDeclarations.hpp"
#include "OSCCondition.hpp"
#include "CommonMini.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace scenarioengine
{
	class Event: public StoryBoardElement
	{
	public:
		typedef enum
		{
			OVERWRITE,
			SKIP,
			PARALLEL,
			UNDEFINED_PRIORITY
		} Priority;

		Priority priority_;

		std::vector<OSCAction*> action_;

		Trigger *start_trigger_;

		Event() : StoryBoardElement(StoryBoardElement::ElementType::EVENT), start_trigger_(0) {}
		~Event()
		{
			for (auto* entry : action_)
			{
				delete entry;
			}

			if (start_trigger_)
			{
				delete start_trigger_;
			}
		}

		void Start(double simTime, double dt);
		void End(double simTime);
		void Stop();

		void UpdateState();
	};

	class Maneuver: public StoryBoardElement
	{
	public:
		OSCParameterDeclarations parameter_declarations_;
		std::vector<Event*> event_;

		Maneuver() : StoryBoardElement(StoryBoardElement::ElementType::MANEUVER) {}
		~Maneuver()
		{
			for (auto* entry : event_)
			{
				delete entry;
			}
		}

		bool IsAnyEventActive();
		bool AreAllEventsComplete();
		void UpdateState();
		void Reset();

		void Print()
		{
			LOG("\tname = %s", name_.c_str());
		};
	};
}