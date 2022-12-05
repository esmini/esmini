#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	const char* filename = argc > 1 ? argv[1] : "../resources/xosc/cut-in_external_example.xosc";

	SE_Init(filename, 0, 1, 0, 0);

	for (int i = 0; i < 500 && SE_GetQuitFlag() != 1; i++)
	{
		SE_ReportObjectPos(SE_GetId(0), 0.0f, 8.0f, (float)i, 0.0f, float(1.57 + 0.01 * i), 0.0f, 0.0f);
		SE_Step();
	}

	SE_Close();

	return 0;
}
