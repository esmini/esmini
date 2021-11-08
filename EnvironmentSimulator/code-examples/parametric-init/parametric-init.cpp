#include "stdio.h"
#include "esminiLib.hpp"

void paramDeclCB(void*)
{
	static int counter = 0;
	double value[2] = { 1.1, 1.5 };

	if (counter < 2)
	{
		printf("paramDeclCB call nr %d. Set TargetSpeedFactor = %.2f\n", counter, value[counter]);
		SE_SetParameterDouble("TargetSpeedFactor", value[counter]);
	}
	else
	{
		double defaultValue;
		SE_GetParameterDouble("TargetSpeedFactor", &defaultValue);
		printf("paramDeclCB call nr %d. Using TargetSpeedFactor default value (%.2f)\n", counter, defaultValue);
	}

	counter++;
}

int main(int argc, char* argv[])
{
	SE_RegisterParameterDeclarationCallback(paramDeclCB, 0);

	for (int i = 0; i < 3 && SE_GetQuitFlag() != 1; i++)
	{
		if (i == 0)
		{
			SE_SetParameterDouble("TargetSpeedFactor", 1.1);
		}
		else
		{
			SE_SetParameterDouble("TargetSpeedFactor", 2.1);
		}

		SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

		while (SE_GetSimulationTime() < 20.0 && SE_GetQuitFlag() != 1)
		{
			SE_Step();
		}

		SE_Close();
	}
	return 0;
}