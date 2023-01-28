#include "esminiLib.hpp"
#include "stdio.h"

int main(void)
{
	SE_Init("../EnvironmentSimulator/Unittest/xosc/test-collision-detection.xosc", 1, 1, 0, 0);
	SE_CollisionDetection(true);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();

		for (int j = 0; j < SE_GetNumberOfObjects(); j++)
		{
			for (int k = 0; k < SE_GetObjectNumberOfCollisions(j); k++)
			{
				printf("Collision[%d]: %d\n", j, SE_GetObjectCollision(j, k));
			}
		}
	}

	return 0;
}
