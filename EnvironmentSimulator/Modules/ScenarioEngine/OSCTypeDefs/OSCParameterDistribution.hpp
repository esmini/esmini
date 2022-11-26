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
#include <vector>
#include "pugixml.hpp"


namespace scenarioengine
{
	class OSCParameterDistribution
	{
		struct ParameterValueList
		{
			std::string name;
			std::vector<std::string> value;
		};

		std::vector<ParameterValueList> param_list_;
		std::string filename_;
		std::string scenario_filename_;
		int index_;
		int requested_index_;
		pugi::xml_document doc_;

	public:
		OSCParameterDistribution() { Reset(); }
		~OSCParameterDistribution();
		static OSCParameterDistribution& Inst();

		int Load(std::string filename);
		int GetNumPermutations();
		int GetNumParameters();
		void Reset();
		std::string GetFilename() { return filename_; }

		// Returns current permutation index
		int GetIndex() { return index_; }
		int SetIndex(int index);
		int SetRequestedIndex(int index);
		int GetRequestedIndex() { return requested_index_; }
		int IncrementIndex();
		std::string GetParamName(int index);
		std::string GetParamValue(int param_index);
		std::string AddInfoToFilename(std::string filename);
	};
}
