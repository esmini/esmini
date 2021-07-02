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

#include <vector>

namespace scenarioengine
{

	class Init
	{

	public:

		std::vector<OSCPrivateAction*> private_action_;
		std::vector<OSCGlobalAction*> global_action_;
	};

}