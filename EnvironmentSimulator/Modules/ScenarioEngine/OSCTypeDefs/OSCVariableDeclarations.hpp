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

#include <iostream>
#include <string>
#include <vector>

#include "CommonMini.hpp"


namespace scenarioengine
{
	class OSCVariableDeclarations
	{
	public:
		enum class VariableType {
			VAR_TYPE_INTEGER,
			VAR_TYPE_DOUBLE,
			VAR_TYPE_STRING,
			VAR_TYPE_BOOL
		};

		struct VariableStruct
		{
			std::string name;
			VariableType type;
			struct value
			{
				int _int = 0;
				double _double = 0;
				std::string _string;
				bool _bool = false;
			} value;
		};

		std::vector<VariableStruct> Variable;

		void* getValueFromStruct(VariableStruct *p)
		{
			return (void*)&p->value._int;
		}

		void* setValueInStruct(VariableStruct* p)
		{
			return (void*)&p->value._int;
		}
	};

}