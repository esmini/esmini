
#include "esminiLib.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    int retval = 0;

    if (argc > 1)
    {
        // use specific scenario file, but allow for custom command line arguments
        SE_SetOptionValue("osc", "../EnvironmentSimulator/code-examples/hello_world/acc_with_external_controller.xosc");
        retval = SE_InitWithArgs(argc, const_cast<const char**>(argv));
    }
    else
    {
        retval = SE_Init("../EnvironmentSimulator/code-examples/hello_world/acc_with_external_controller.xosc", 0, 1, 0, 0);
    }

    if (retval != 0)
    {
        printf("Failed to initialize the scenario\n");
        return -1;
    }

    SE_ScenarioObjectState objectState;
    SE_GetObjectState(0, &objectState);

    void* vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);
    SE_SimpleVehicleSetThrottleDisabled(vehicleHandle, true);
    SE_SimpleVehicleState vehicleState = {0, 0, 0, 0, 0, 0, 0, 0};

    double dt = 0.01;
    while (SE_GetQuitFlag() == 0 && SE_GetSimulationTime() < 60.0f)
    {
        // find out and apply speed set by acc controller
        SE_GetObjectState(0, &objectState);
        SE_SimpleVehicleSetSpeed(vehicleHandle, objectState.speed);

        // Step vehicle model with driver input
        int steering = 0;
        if (SE_GetSimulationTime() > 1.0f && SE_GetSimulationTime() < 1.1f)
        {
            steering = 1;
        }
        else if (SE_GetSimulationTime() > 3.0f && SE_GetSimulationTime() < 3.25f)
        {
            steering = -1;
        }
        else if (SE_GetSimulationTime() > 4.0f && SE_GetSimulationTime() < 4.18f)
        {
            steering = 1;
        }

        SE_SimpleVehicleControlBinary(vehicleHandle, dt, 0, steering);

        // Fetch updated state and report to scenario engine
        SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
        SE_ReportObjectPosXYH(0, vehicleState.x, vehicleState.y, vehicleState.h);

        // wheel status (revolution and steering angles)
        SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation, vehicleState.wheel_angle);

        SE_StepDT(static_cast<float>(dt));
    }

    SE_SimpleVehicleDelete(vehicleHandle);
    SE_Close();

    return 0;
}
