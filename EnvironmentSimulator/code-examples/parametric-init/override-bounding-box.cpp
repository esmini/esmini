#include "stdio.h"
#include "esminiLib.hpp"

void paramDeclCB(void*)
{
    printf("parameter declaration callback\n");
    SE_SetParameterDouble("CarDimX", 2.0);
    SE_SetParameterDouble("CarDimY", 2.5);
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    SE_RegisterParameterDeclarationCallback(paramDeclCB, 0);

    SE_Init("../resources/xosc/override_bb.xosc", 0, 1, 0, 0);

    while (SE_GetSimulationTime() < 20.0f && SE_GetQuitFlag() == 0)
    {
        SE_Step();
    }

    SE_Close();

    return 0;
}