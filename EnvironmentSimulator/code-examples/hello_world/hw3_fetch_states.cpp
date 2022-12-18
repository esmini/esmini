#include "stdio.h"
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

	SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();

		for (int j = 0; j < SE_GetNumberOfObjects(); j++)
		{
			SE_ScenarioObjectState state;

			SE_GetObjectState(SE_GetId(j), &state);
			printf("time [%.2f] object[%d] id %d pos[%.2f, %.2f] %.2f %.2f \n", state.timestamp, j, SE_GetId(j), static_cast<double>(state.x), static_cast<double>(state.y), static_cast<double>(state.wheel_angle), static_cast<double>(state.wheel_rot)); // TODO: @Emil
		}
	}

	SE_Close();

	return 0;
}
