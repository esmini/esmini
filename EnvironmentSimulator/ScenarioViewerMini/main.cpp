
#include "stdio.h"
#include "scenarioenginedll.h"
#include "CommonMini.hpp"

#define MAX_N_OBJECTS 10

static ScenarioObjectState states[MAX_N_OBJECTS];

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		LOG("Usage: %s <osc filename>\n", argv[0]);
		return -1;
	}

	SE_Init(argv[1], true);

	for (int i = 0; i < 1000; i++)
	{
		SE_Step(0.016f);

		int nObjects = MAX_N_OBJECTS;
		SE_GetObjectStates(&nObjects, states);

		float angle;
		SE_GetSteeringTargetAngle(0, states[0].speed * 3.0f, &angle);
		LOG("Steering target angle: %.2f", angle);
	}

	SE_Close();

	return 0;
}