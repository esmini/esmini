
#include "stdio.h"
#include "scenarioenginedll.h"
#include <thread>
#include <chrono>

// #define RESTART_TEST

using namespace std::chrono;

#define MAX_N_OBJECTS 10

static ScenarioObjectState states[MAX_N_OBJECTS];

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		printf("Usage: %s <osc filename>\n", argv[0]);
		return -1;
	}

#ifdef RESTART_TEST
	char *fname[2] =
	{
		"C:\\eknabe1\\GIT\\environment-simulator\\resources\\xosc\\ltap-od-variant_C_two_targets_internal.xosc",
		"C:\\eknabe1\\GIT\\environment-simulator\\resources\\xosc\\cut-in.xosc"
	};

	for (int j = 0; j < 2; j++)
	{
		SE_Init(fname[j], true);
#else
	{
	SE_Init(argv[1], true);
#endif

		for (int i = 0; i < 1000; i++)
		{
			SE_Step(0.016f);

			int nObjects = MAX_N_OBJECTS;
			SE_GetObjectStates(&nObjects, states);
		}

#ifdef RESTART_TEST
		std::this_thread::sleep_for(milliseconds(2000));
#endif
	}
	SE_Close();

	return 0;
}