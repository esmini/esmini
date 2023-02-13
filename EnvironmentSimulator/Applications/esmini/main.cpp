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
#include "OSCParameterDistribution.hpp"
#include <signal.h>

#define MIN_TIME_STEP 0.01
#define MAX_TIME_STEP 0.1


static bool quit = false;

static void signal_handler(int s)
{
	if (s == SIGINT)
	{
		LOG("Quit request from user");
		quit = true;
	}
}

static int execute_scenario(int argc, char* argv[])
{
	__int64 time_stamp = 0;

	std::unique_ptr<ScenarioPlayer> player;

	// Setup signal handler to catch Ctrl-C
	signal(SIGINT, signal_handler);

	try
	{
		player = std::make_unique<ScenarioPlayer>(argc, argv);
		if (player->Init() != 0)
		{
			return -1;
		}

		if (player->opt.GetOptionSet("return_nr_permutations"))
		{
			// Skip scenario, return immediately
			return static_cast<int>(OSCParameterDistribution::Inst().GetNumPermutations());
		}
	}
	catch (const std::exception& e)
	{
		LOG(std::string("Exception: ").append(e.what()).c_str());
		return -1;
	}


	while (!player->IsQuitRequested() && !quit)
	{
		double dt;
		if (player->GetFixedTimestep() > SMALL_NUMBER)
		{
			dt = player->GetFixedTimestep();
		}
		else
		{
			dt = SE_getSimTimeStep(time_stamp, player->minStepSize, player->maxStepSize);
		}

		player->Frame(dt);
	}

	if (player->opt.IsOptionArgumentSet("param_permutation"))
	{
		// Single permutation requested and executed, quit now
		quit = true;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	OSCParameterDistribution& dist = OSCParameterDistribution::Inst();
	int retval = 0;

	do
	{
		 retval = execute_scenario(argc, argv);

	} while (retval == 0 && dist.GetIndex() >= 0 && dist.GetIndex() < static_cast<int>(dist.GetNumPermutations() - 1) && !quit);

	return retval;
}
