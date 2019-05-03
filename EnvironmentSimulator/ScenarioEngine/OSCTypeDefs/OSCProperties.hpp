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
#include <iostream>
#include <string>
#include "CommonMini.hpp"

namespace scenarioengine
{

	class OSCProperties
	{
	public:
		class Property
		{
		public:
			std::string name_;
			std::string value_;
		};

		class File
		{
		public:
			std::string filepath_;
		};

		std::vector<Property> property_;
		File file_;  // Should be no more than one?
	};

}