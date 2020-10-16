
#include "scenarioenginedll.hpp"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in.xosc", 1, 1, 0, 0, 0.0f);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();
	}

	return 0;
}
