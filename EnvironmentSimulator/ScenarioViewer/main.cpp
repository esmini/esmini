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


#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_version.pb.h"

#include "stdio.h"
#include "scenarioenginedll.hpp"
#include "CommonMini.hpp"

#define DEMONSTRATE_SENSORS 1
#define DEMONSTRATE_OSI 0
#define DEMONSTRATE_ROADINFO 0
#define DEMONSTRATE_THREAD 0
#define DEMONSTRATE_CALLBACK 0

#define MAX_N_OBJECTS 10
#define TIME_STEP 0.017f
#define DURATION 25
#define MAX_DETECTIONS 8

static SE_ScenarioObjectState states[MAX_N_OBJECTS];

typedef struct
{
	int counter;
} Stuff;

void log_callback(const char *str)
{
	printf("%s\n", str);
}

void objectCallback(SE_ScenarioObjectState* state, void *my_data)
{
	const double startTrigTime = 3.0;
	const double latDist = 3.5;
	const double duration = 5.0;
	static bool firstTime = true;
	static double latOffset0;

	Stuff* stuff = (Stuff*)my_data;
	
	printf("mydata.counter: %d\n", stuff->counter);

	if (SE_GetSimulationTime() > startTrigTime && SE_GetSimulationTime() < startTrigTime + duration)
	{
		if (firstTime)
		{
			latOffset0 = state->laneOffset;
			firstTime = false;
		}
		else 
		{
			float latOffset = (float)(latOffset0 + latDist * (SE_GetSimulationTime() - startTrigTime)/duration);
			SE_ReportObjectRoadPos(state->id, state->timestamp, state->roadId, state->laneId, latOffset, state->s, state->speed);
		}
	}
}

int main(int argc, char *argv[])
{
	Stuff stuff;
	
	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	if (argc < 2)
	{
		LOG("Usage: %s <osc filename>\n", argv[0]);
		return -1;
	}

	for (int a = 0; a < 1; a++)
	{
		stuff.counter = 0;

		if (SE_Init(argv[1], 0, 1, 1, 0, 2.0f) != 0)
		{
			LOG("Failed to load %s", argv[1]);
			return -1;
		}

#if DEMONSTRATE_CALLBACK
		SE_RegisterObjectCallback(0, objectCallback, (void*)&stuff);
#endif

#if DEMONSTRATE_SENSORS
		// Add four sensors around the vehicle
		SE_AddObjectSensor(0, 4.0f, 0.0f, 0.5f, 0.0f, 6.0f, 50.0f, (float)(50.0 * M_PI / 180.0), MAX_DETECTIONS);
		SE_AddObjectSensor(0, 2.0f, 1.0f, 0.5f, 1.5f, 1.0f, 20.0f, (float)(120.0 * M_PI / 180.0), MAX_DETECTIONS);
		SE_AddObjectSensor(0, 2.0f, -1.0f, 0.5f, -1.5f, 1.0f, 20.0f, (float)(120.0 * M_PI / 180.0), MAX_DETECTIONS);
		SE_AddObjectSensor(0, -1.0f, 0.0f, 0.5f, 3.14f, 5.0f, 30.0f, (float)(50.0 * M_PI / 180.0), MAX_DETECTIONS);
#endif

#if DEMONSTRATE_OSI
		osi3::GroundTruth* gt;
		SE_OpenOSISocket("127.0.0.1");
#endif

		for (int i = 0; i*TIME_STEP < DURATION; i++)
		{
			if (SE_StepDT(TIME_STEP) != 0)
			{
				return 0;
			}

			stuff.counter++;
#if DEMONSTRATE_OSI  // set to 1 to demonstrate example of how to query OSI Ground Truth

			int svSize = 0;

			SE_UpdateOSIGroundTruth();
			// Fetch and parse OSI message
			gt = (osi3::GroundTruth*)SE_GetOSIGroundTruthRaw();
			
			// Print timestamp
			printf("timestamp: %.2f\n", gt->mutable_timestamp()->seconds() +
				1E-9 * gt->mutable_timestamp()->nanos());

			// Print object id, position, orientation and velocity
			for (int i = 0; i < gt->mutable_moving_object()->size(); i++)
			{
				printf(" obj id %lld pos (%.2f, %.2f, %.2f) orientation (%.2f, %.2f, %.2f) velocity (%.2f, %.2f, %.2f) \n",
					gt->mutable_moving_object(i)->mutable_id()->value(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_position()->x(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_position()->y(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_position()->z(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_orientation()->yaw(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_orientation()->pitch(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_orientation()->roll(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_velocity()->x(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_velocity()->y(),
					gt->mutable_moving_object(i)->mutable_base()->mutable_velocity()->z()
				);
			}
#endif		

#if DEMONSTRATE_ROADINFO  // set to 1 to demonstrate example of how to query road information
			SE_RoadInfo data;
			double look_ahead_distance = 10;
			int id = 0;

			SE_GetRoadInfoAtDistance(id, look_ahead_distance, &data, 0);

			LOG("Road info at %.2f meter from Vehicle %d: pos (%.2f, %.2f, %.2f) curvature %.5f (r %.2f) heading %.2f pitch %.2f lane width %.2f",
				look_ahead_distance, id, data.global_pos_x, data.global_pos_y, data.global_pos_z, data.curvature, 1.0 / data.curvature,  data.road_heading, data.road_pitch);
#endif

#if DEMONSTRATE_SENSORS  

			printf("Detections [sensor ID, Object ids]:");
			int objList[MAX_DETECTIONS];  // make room for max nr vehicles, as specified when added sensor
			for (int j = 0; j < 4; j++)
			{
				int nHits = SE_FetchSensorObjectList(j, objList);
				for (int k = 0; k < nHits; k++)
				{
					printf(" [%d, %d]", j, objList[k]);
				}
			}
			printf("\n");
#endif

			if (DEMONSTRATE_THREAD && i == (int)(0.5 * DURATION / TIME_STEP))
			{
				// Halfway through, pause the simulation for a few seconds
				// to demonstrate how camera can still move independently
				// when running viewer in a separate thread
				// Only Linux and Win supported (due to OSG and MacOS issue)
				LOG("Taking a 4 sec nap - if running with threads (Win/Linux) you can move camera around meanwhile");
				SE_sleep(4000);
			}
			else
			{
				// Normal case, sleep until its time for next simulation step
				SE_sleep((unsigned int)(TIME_STEP * 1000));
			}
		}

		SE_Close();
	}

	return 0;
}
