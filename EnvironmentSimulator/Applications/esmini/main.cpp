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


#include "playerbase.hpp"
#include "CommonMini.cpp"

#define MIN_TIME_STEP 0.01
#define MAX_TIME_STEP 0.1


int main(int argc, char *argv[])
{
	ScenarioPlayer *player;
	__int64 time_stamp = 0;

	try
	{
		player = new ScenarioPlayer(argc, argv);
	}
	catch (const std::exception& e)
	{
		LOG(std::string("Exception: ").append(e.what()).c_str());
		return -1;
	}

	while (!player->IsQuitRequested())
	{
		double dt;
		if (player->GetFixedTimestep() > 0.0)
		{
			dt = player->GetFixedTimestep();
		}
		else
		{
			dt = SE_getSimTimeStep(time_stamp, player->minStepSize, player->maxStepSize);
		}

		player->Frame(dt);
	}

	delete player;

	return 0;
}
