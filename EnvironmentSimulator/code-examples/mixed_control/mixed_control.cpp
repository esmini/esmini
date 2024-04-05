/*
    This example demonstrates an experimental use case: Combining esmini controller and external functionality

    The application (this code) represents a driver model looking along the lane for steering and
    adapting the speed to road curvature OR respecting ACC controller output, when activated.

    The scenario plays out on a somewhat curvy road. Ego (white) car drives along. When distance to the standstill
    Target (red) car becomes less than 70 m, Target starts driving according to a specified speed profile making
    it accelerate and brake kind of randomly. When distance becomes less than 40 m the Ego assigned ACC controller
    is activated. Now is the tricky part. The controller will run in "virtual" operation mode, to only perform
    the calculations, not applying them. Instead the external application (this code) will 1. detect that the
    controller is active and then 2. read the calculated target/desired acceleration and use it as input to its
    own speed calculations.

    This way a controller can hand over result to an external application for further processing.

    It should be stated that this scheme is not standardized, neither in the OpenSCENARIO standard or esmini.
    For example, only ACCController currently have the "virtual" flag. But nonetheless it's an interesting case.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const double defaultTargetSpeed = 110.0 / 3.6;
    const double curveWeight        = 5.0;
    const double throttleWeight     = 0.1;

    void*                  vehicleHandle = 0;
    SE_SimpleVehicleState  vehicleState  = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;
    SE_RoadInfo            roadInfo;

    float dt = 0.0f;

    // look for specified timestep, else run in realtime mode
    bool fixed_timestep = false;
    bool headless       = false;
    for (int i = 1; argc > 1 && i < argc; i++)
    {
        if (strcmp(argv[i], "--fixed_timestep") == 0 && i < argc - 1)
        {
            fixed_timestep = true;
            dt             = static_cast<float>(atof(argv[i + 1]));  // use any provided fixed timestep
        }
        else if (strcmp(argv[i], "--headless") == 0)
        {
            headless = true;
        }
    }

    if (SE_Init("../EnvironmentSimulator/code-examples/mixed_control/mixed_control.xosc", 0, headless ? 0 : 1, 0, 1) != 0)
    {
        SE_LogMessage("Failed to initialize the scenario, quit\n");
        return -1;
    }

    // Lock object to the original lane
    // If setting to false, the object road position will snap to closest lane
    SE_SetLockOnLane(0, true);

    // Initialize the vehicle model, fetch initial state from the scenario
    SE_GetObjectState(0, &objectState);
    vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);
    SE_SimpleVehicleSteeringRate(vehicleHandle, 6.0f);

    // show some road features, including road sensor
    SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

    // Run for specified duration or until 'Esc' button is pressed
    while (SE_GetQuitFlag() == 0)
    {
        if (!SE_GetPauseFlag())
        {
            // Get road information at a point some speed dependent distance ahead
            double targetSpeed;
            // Look ahead along the road, to establish target info for the driver model
            SE_GetRoadInfoAtDistance(0, 5 + 0.75f * vehicleState.speed, &roadInfo, 0, true);

            // Slow down when curve ahead - CURVE_WEIGHT is the tuning parameter
            targetSpeed = defaultTargetSpeed / (1 + curveWeight * static_cast<double>(fabs(roadInfo.angle)));

            // Steer towards where the point
            double steerAngle = roadInfo.angle;

            // Get simulation delta time since last call (first will be minimum timestep)
            if (!fixed_timestep)
            {
                dt = SE_GetSimTimeStep();
            }

            double throttle = 0.0;

            if (objectState.ctrl_type == 8)  // ACCController
            {
                throttle = throttleWeight * static_cast<double>(SE_GetObjectAcceleration(SE_GetId(0)));
            }
            else
            {
                // Accelerate or decelerate towards target speed - THROTTLE_WEIGHT tunes magnitude
                throttle = throttleWeight * (targetSpeed - static_cast<double>(vehicleState.speed));
            }

            // Step vehicle model with driver input, but wait until time > 0
            if (SE_GetSimulationTime() > 0 && !SE_GetPauseFlag())
            {
                SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);
            }

            // Fetch updated state and report to scenario engine
            SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

            // Report updated vehicle position and heading. z, pitch and roll will be aligned to the road
            SE_ReportObjectPosXYH(0, 0, vehicleState.x, vehicleState.y, vehicleState.h);

            // The following values are not necessary to report.
            // If not reported, esmini will calculate based on motion over time
            // but for accuracy it's recommendeded to report if available.

            // wheel status (revolution and steering angles)
            SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation, vehicleState.wheel_angle);

            // speed (along vehicle longitudinal (x) axis)
            SE_ReportObjectSpeed(0, vehicleState.speed);
        }

        // Finally, update scenario using same time step as for vehicle model
        SE_StepDT(dt);
        SE_GetObjectState(0, &objectState);
    }

    SE_Close();

    return 0;
}