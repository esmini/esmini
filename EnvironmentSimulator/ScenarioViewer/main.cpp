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

 /*
  * This application is a minimalistic example of how to make use of the scenarioengine DLL to simply play and view OpenSCENARIO files
  * In addition to Init and Step, it shows how to retrieve the state of scenario objects.
  */

#include "stdio.h"
#include "scenarioenginedll.hpp"
#include "CommonMini.hpp"


#define MAX_N_OBJECTS 10
#define TIME_STEP 0.017f

static SE_ScenarioObjectState states[MAX_N_OBJECTS];

void log_callback(const char *str)
{
	printf("%s\n", str);
}

int main(int argc, char *argv[])
{
	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	if (argc < 2)
	{
		LOG("Usage: %s <osc filename>\n", argv[0]);
		return -1;
	}

	if (SE_Init(argv[1], 1, 1, 0) != 0)
	{
		LOG("Failed to load %s", argv[1]);
		return -1;
	}

	for (int i = 0; i < 1000; i++)
	{
		if (SE_Step(TIME_STEP) != 0)
		{
			return 0;
		}

		int nObjects = MAX_N_OBJECTS;
		SE_GetObjectStates(&nObjects, states);

		SE_sleep((unsigned int)(TIME_STEP * 1000));
	}

	SE_Close();


	return 0;
}
