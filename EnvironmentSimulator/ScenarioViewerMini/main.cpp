
#include "stdio.h"
#include "scenarioenginedll.h"
#include "CommonMini.hpp"
#include <Windows.h>

#define MAX_N_OBJECTS 10
#define TIME_STEP 1.0f

static ScenarioObjectState states[MAX_N_OBJECTS];

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

	if (SE_Init(argv[1], 0, 1, 0) != 0)
	{
		LOG("Failed to load %s", argv[1]);
		return -1;
	}

	for (int i = 0; i < 1000; i++)
	{
		if(i == 500)
		{
			Sleep(5000);
		}

		if (SE_Step(TIME_STEP) != 0)
		{
			return 0;
		}

		int nObjects = MAX_N_OBJECTS;
		SE_GetObjectStates(&nObjects, states);
		for (int i = 0; i < nObjects; i++)
		{
			printf("%d: %d\n", i, states[i].ext_control);
		}

		float angle;
		SE_GetSteeringTargetAngle(0, states[0].speed * 3.0f, &angle);

		Sleep(DWORD(TIME_STEP * 1000));
	}

	SE_Close();

	return 0;
}