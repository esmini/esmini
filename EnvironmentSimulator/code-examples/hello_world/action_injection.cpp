
#include "esminiLib.hpp"
#include <string.h>
#include <iostream>

int main(int argc, char **argv)
{
    float dt = -1.0f;  // default to variable timestep

    if (argc > 2)
    {
        if (SE_InitWithArgs(argc, const_cast<const char **>(argv)) != 0)
        {
            printf("Error: Could not initialize scenario %s\n", argv[1]);
            return -1;
        }
        for (int i = 2; i < argc; i++)
        {
            if (strcmp(argv[i], "--fixed_timestep") == 0 && i < argc - 1)
            {
                dt = static_cast<float>(atof(argv[i + 1]));  // use any provided fixed timestep
            }
        }
    }
    else if (argc > 1)
    {
        if (SE_Init(argv[1], 0, 1, 0, 0) == 0)
        {
            printf("Error: Could not initialize scenario %s\n", argv[1]);
            return -1;
        }
    }
    else
    {
        printf("Usage: %s <path to xosc file>\n", argv[0]);
        return -1;
    }

    int state = 0;
    while (SE_GetQuitFlag() == 0 && SE_GetSimulationTime() < 22.0f)
    {
        if (state == 0 && SE_GetSimulationTime() > 2.0f)
        {
            printf("Injecting lane offset action\n");
            SE_LaneOffsetActionStruct lane_offset;
            lane_offset.id               = 0;
            lane_offset.offset           = -0.45f;
            lane_offset.maxLateralAcc    = 0.5f;
            lane_offset.transition_shape = 0;
            SE_InjectLaneOffsetAction(&lane_offset);
            state++;
        }
        else if (state == 1 && SE_GetSimulationTime() > 7.0f)
        {
            printf("Injecting lane change action\n");
            SE_LaneChangeActionStruct lane_change;
            lane_change.id               = 0;
            lane_change.mode             = 1;
            lane_change.target           = 1;
            lane_change.transition_shape = 2;
            lane_change.transition_dim   = 2;
            lane_change.transition_value = 3.0f;
            SE_InjectLaneChangeAction(&lane_change);
            state++;
        }
        else if (state == 2 && SE_GetSimulationTime() > 14.0f)
        {
            printf("Injecting speed action\n");
            SE_SpeedActionStruct speed;
            speed.id               = 0;
            speed.speed            = 0.0f;
            speed.transition_shape = 0;
            speed.transition_dim   = 1;
            speed.transition_value = 7.0f;
            SE_InjectSpeedAction(&speed);
            state++;
        }

        if (dt < 0.0f)
        {
            SE_Step();
        }
        else
        {
            SE_StepDT(dt);
        }
    }

    SE_Close();

    return 0;
}
