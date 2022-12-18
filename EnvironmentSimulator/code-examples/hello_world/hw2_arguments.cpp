#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	if (argc > 1)
	{
		SE_InitWithArgs(argc, const_cast<const char**>(argv));
	}
	else
	{
		SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);
	}

	for (int i = 0; i < 500; i++)
	{
		SE_Step();
	}

	SE_Close();

	return 0;
}
