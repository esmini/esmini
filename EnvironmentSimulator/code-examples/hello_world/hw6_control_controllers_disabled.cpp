#include "esminiLib.hpp"

int main(int, char**)
{
	SE_Init("../resources/xosc/cut-in_interactive.xosc", 1, 1, 0, 0);

	for (int i = 0; i < 2000 && SE_GetQuitFlag() != 1; i++)
	{
		SE_Step();
	}

	SE_Close();

	return 0;
}
