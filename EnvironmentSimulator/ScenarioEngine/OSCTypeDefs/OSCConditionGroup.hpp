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
#include "OSCCondition.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace scenarioengine
{

	// Forward declaration
	class OSCCondition;

	class OSCConditionGroup
	{
	public:
		std::vector<OSCCondition*> condition_;

		void Print()
		{
			LOG("\t - ConditionGroup");
		}
	};

}