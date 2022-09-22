#include "stdio.h"
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/lane_change_simple.xosc", 0, 1, 0, 0);

	for (int i = 0; i < 500 && SE_GetQuitFlag() != 1; i++)
	{
		SE_Step();

		for (int j = 0; j < SE_GetNumberOfObjects(); j++)
		{
			SE_ScenarioObjectState state;

			SE_GetObjectState(SE_GetId(j), &state);
			printf("time [%.2f] object[%d] id %d pos[%.2f, %.2f] %.2f %.2f \n", state.timestamp, j, SE_GetId(j), state.x, state.y, state.wheel_angle, state.wheel_rot);
		}
	}

	SE_Close();

	return 0;
}
