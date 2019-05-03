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

#include <string>
#include <fstream>
#include "CommonMini.hpp"
#include "ScenarioGateway.hpp"

namespace scenarioengine
{

#define REPLAY_FILENAME_SIZE 128

	typedef struct
	{
		char odr_filename[REPLAY_FILENAME_SIZE];
		char model_filename[REPLAY_FILENAME_SIZE];
	} ReplayHeader;



	class Replay
	{
	public:
		Replay(std::string filename);
		~Replay();
		void Step(double dt);
		ObjectStateStruct * GetState(int id);

		ReplayHeader header_;
		std::vector<ObjectStateStruct> data_;
		std::ifstream file_;
		double time_;
		unsigned int index_;
	};

}