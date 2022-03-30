#include "stdio.h"
#include "esminiLib.hpp"

#define MAX_HITS 10

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

	SE_AddObjectSensor(0, 2.0, 1.0, 0.5, 1.57, 1.0, 50.0, 1.57, MAX_HITS);
	SE_AddObjectSensor(0, -1.0, 0.0, 0.5, 3.14, 0.5, 20.0, 1.57, MAX_HITS);

	// Turn on visualization of object sensors, toggle key 'r'
	SE_ViewerShowFeature(1, true);

	for (int i = 0; i < 2000 && !(SE_GetQuitFlag() == 1); i++)
	{
		SE_Step();

		int objList[MAX_HITS];
		for (int j = 0; j < 2; j++)  // iterate over added sensors
		{
			int nHits = SE_FetchSensorObjectList(j, objList);
			for (int k = 0; k < nHits; k++)
			{
				printf("Sensor[%d] detected obj: %d\n", j, objList[k]);

				// Get some info of that detected object
				SE_ScenarioObjectState state;

				SE_GetObjectState(objList[k], &state);
				printf("object[%d] pos: (%.2f, %.2f) heading: %.2f\n", objList[k], state.x, state.y, state.h);
			}
		}
	}

	return 0;
}
