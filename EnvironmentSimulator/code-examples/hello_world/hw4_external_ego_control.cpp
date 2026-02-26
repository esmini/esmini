#include "esminiLib.hpp"
#include <stdio.h>

int main(int argc, char* argv[])
{
    const char*            filename = argc > 1 ? argv[1] : "../resources/xosc/cut-in_external.xosc";
    SE_ScenarioObjectState state;
    double                 z = 0.0f;

    if (SE_Init(filename, 0, 1, 0, 0) != 0)
    {
        return -1;
    }

    for (int i = 0; i < 500 && SE_GetQuitFlag() != 1; i++)
    {
        SE_ReportObjectPos(SE_GetId(0), 8.0f, static_cast<float>(i), static_cast<float>(z), float(1.57 + 0.01 * i), 0.0f, 0.0f);

        SE_Step();

        // fetch position in terms of road coordinates
        SE_GetObjectState(SE_GetId(0), &state);

        printf("road_id: %d s: %.3f lane_id %d lane_offset: %.3f z: %.2f\n",
               state.roadId,
               static_cast<double>(state.s),
               state.laneId,
               static_cast<double>(state.laneOffset),
               static_cast<double>(state.z));

        if (i == 100)
        {
            z = 10.0;
            printf("Release relative road alignment, set absolute z = %.2f\n", z);
            SE_SetObjectPositionMode(SE_GetId(0),
                                     SE_PositionModeType::SE_SET,
                                     SE_PositionMode::SE_Z_ABS);  // release relative alignment to road surface
        }

        if (i == 200)
        {
            z = 0.0;
            printf("Restore relative road alignment, set relative z = %.2f\n", z);
            SE_SetObjectPositionMode(SE_GetId(0),
                                     SE_PositionModeType::SE_SET,
                                     SE_PositionMode::SE_Z_REL);  // enforce relative alignment to road surface
        }
    }

    SE_Close();

    return 0;
}
