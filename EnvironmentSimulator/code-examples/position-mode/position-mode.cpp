#include "esminiLib.hpp"
#include "stdio.h"
#include <cmath>

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    SE_AddPath("../resources/xodr");

    if (SE_Init("../EnvironmentSimulator/code-examples/position-mode/skater.xosc", 0, 1, 0, 0) != 0)
    {
        return -1;
    }

    SE_SetCameraObjectFocus(2);
    SE_AddCustomAimingCamera(-50.0, 20.0, 20.0);

    int state = 0;
    while (SE_GetQuitFlag() == 0)
    {
        if (state == 0 && SE_GetSimulationTime() > 5.0f)
        {
            SE_ReportObjectPosMode(
                0,
                0.0f,
                std::nanf(""),
                std::nanf(""),
                0.0f,
                1.57f,
                0.0f,
                0.0f,
                SE_PositionMode::SE_H_REL | SE_PositionMode::SE_P_ABS | SE_PositionMode::SE_R_ABS);  // report only (relative) heading
            SE_SetObjectPositionMode(0,
                                     SE_PositionModeType::SE_UPDATE,
                                     SE_PositionMode::SE_H_REL | SE_PositionMode::SE_P_ABS | SE_PositionMode::SE_R_ABS);
            SE_ReportObjectSpeed(0, 20.0f);

            state++;
        }
        else if (state == 1 && SE_GetSimulationTime() > 7.0f)
        {
            SE_SetCameraObjectFocus(0);
            SE_AddCustomAimingCamera(-23.0, 14.0, 6.0);
            state++;
        }
        else if (state == 2 && SE_GetSimulationTime() > 15.0f)
        {
            SE_ReportObjectPosMode(0,
                                   0.0f,
                                   std::nanf(""),
                                   std::nanf(""),
                                   1.0f,
                                   0.0f,
                                   0.0f,
                                   0.0f,
                                   SE_PositionMode::SE_Z_REL | SE_PositionMode::SE_H_REL | SE_PositionMode::SE_P_ABS |
                                       SE_PositionMode::SE_R_ABS);  // report only (relative) heading

            SE_SetObjectPositionMode(0,
                                     SE_PositionModeType::SE_UPDATE,
                                     SE_PositionMode::SE_Z_REL | SE_PositionMode::SE_H_REL | SE_PositionMode::SE_P_ABS | SE_PositionMode::SE_R_ABS);

            SE_ReportObjectSpeed(0, 30.0f);

            state++;
        }
        else if (state == 3 && SE_GetSimulationTime() > 23.0f)
        {
            SE_SetCameraObjectFocus(2);
            SE_AddCustomAimingCamera(-35.3, -59.0, 26.5);
            state++;
        }

        SE_Step();
    }

    SE_Close();

    return 0;
}
