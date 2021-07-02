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

		Event() : start_trigger_(0), StoryBoardElement(StoryBoardElement::ElementType::EVENT) {}

		void Start(double simTime, double dt);
		void End();
		void Stop();

	};

	class OSCManeuver
	{
	public:
		OSCParameterDeclarations parameter_declarations_;
		std::vector<Event*> event_;
		std::string name_;

		bool IsAnyEventActive();

		void Print()
		{
			LOG("\tname = %s", name_.c_str());
		};
	};

}