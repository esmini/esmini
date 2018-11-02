
#include "stdio.h"
#include "scenarioenginedll.h"

#define MAX_N_OBJECTS 10

static ScenarioObjectState states[MAX_N_OBJECTS];

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		printf("Usage: %s <osc filename>\n", argv[0]);
		return -1;
	}

	SE_Init(argv[1], true);

	for (int i = 0; i < 1000; i++)
	{
		SE_Step(0.016f);

		int nObjects = MAX_N_OBJECTS;
		SE_GetObjectStates(&nObjects, states);
	}


	SE_Close();

	return 0;
}