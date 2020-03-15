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

	for (int a = 0; a < 1; a++)
	{

		if (SE_Init(argv[1], 3, 1, 0, 0.2f) != 0)
		{
			LOG("Failed to load %s", argv[1]);
			return -1;
		}

		SE_AddObjectSensor(0, 4.0f, 0.0f, 0.5f, 5.0f, 50.0f, (float)(50.0 * M_PI / 180.0), 10);

		int objList[2];
	
		for (int i = 0; i < 400; i++)
		{
			if (SE_StepDT(TIME_STEP) != 0)
			{
				return 0;
			}

			int nHits = SE_FetchSensorObjectList(0, objList);
			for (int j = 0; j < nHits; j++)
			{
				LOG("sensor hit obj_id %d", j, objList[j]);
			}

			//SE_ScenarioObjectState state;
			//SE_GetObjectState(0, &state);
			
			//if (state.control == 3)
			//{
			//	SE_RoadInfo info;
			//	float speed;
			//	SE_GetRoadInfoAlongGhostTrail(0, 20, &info, &speed);
			//	//LOG("y %.2f trail h %.2f speed %.2f", info.global_pos_y, info.trail_heading, speed);
			//}

			SE_sleep((unsigned int)(TIME_STEP * 1000));
		}
		SE_Close();
	}


	return 0;
}
