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

#include "OSCAction.hpp"

using namespace scenarioengine;

std::string OSCAction::basetype2str(BaseType type)
{
	if (type == BaseType::GLOBAL)
	{
		return "Global";
	}
	else if(type == BaseType::PRIVATE)
	{
		return "Private";
	}
	else if (type == BaseType::USER_DEFINED)
	{
		return "User defined";
	}
	else
	{
		LOG("Undefined Base Type: %d", type);
	}

	return std::string();
}
