#include "stdio.h"
#include "math.h"
#include <string>
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    std::string scenario_file = "lookahead_route.xosc";

    const double defaultTargetSpeed = 30.0 / 3.6;
    const double curveWeight        = 5.0;
    const double throttleWeight     = 0.1;
    const double steeringRate       = 8.0;

    SE_SimpleVehicleState  vehicleState = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_RoadInfo            roadInfo;
    SE_ScenarioObjectState objectState;

    SE_SetOptionValue("path", "../EnvironmentSimulator/code-examples/lookahead_route");           // for running from code-examples-bin folder
    SE_SetOptionValue("path", "../../../../EnvironmentSimulator/code-examples/lookahead_route");  // for VS debug sessions
    SE_SetOption("bounding_boxes");                                                               // see through bounding box view mode

    if (SE_Init(scenario_file.c_str(), 0, 1, 0, 0) != 0)
    {
        printf("Failed to initialize scenario/n");
        return -1;
    }

    // Fetch initial state from the scenario and initialize the simple vehicle model
    SE_GetObjectState(0, &objectState);
    void* vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);
    SE_SimpleVehicleSteeringRate(vehicleHandle, steeringRate);

    // show some road features, including road sensor in case viewer is enabled
    SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

    // Run for specified duration or until 'Esc' button is pressed
    while (SE_GetQuitFlag() == 0)
    {
        if (!SE_GetPauseFlag())
        {
            double dt = SE_GetSimTimeStep();  // get elapsed time since last step

            // Look ahead along the route, to establish target info for the driver model
            double lookahead_distance = 2 + 0.75 * vehicleState.speed;

            if (SE_GetObjectRouteStatus(0) == 2)
            {
                // Look along route as long as vehicle is on route
                SE_GetRoadInfoAlongRoute(0, lookahead_distance, &roadInfo, 0, false);

                // Enable snap to route for succeeding SE_ReportObjectPosXYH() calls
                SE_SetObjectPositionMode(0, SE_PositionModeType::SE_SET, SE_SNAP_TO_ROUTE_ON);
            }
            else
            {
                // If off route, use standard function to look ahead
                SE_GetRoadInfoAtDistance(0, lookahead_distance, &roadInfo, 0, false);

                // Snap to any closest lane (default) in succeeding SE_ReportObjectPosXYH() calls
                SE_SetObjectPositionMode(0, SE_PositionModeType::SE_SET, SE_SNAP_TO_ROUTE_OFF);
            }

            // Slow down when curve ahead - curveWeight is the tuning parameter
            double targetSpeed = defaultTargetSpeed / (1 + curveWeight * fabs(roadInfo.angle));

            // Accelerate or decelerate towards target speed - throttleWeight tunes magnitude
            double throttle = throttleWeight * (targetSpeed - vehicleState.speed);

            // Steer towards where the point
            double steerAngle = roadInfo.angle;

            SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);

            // Fetch updated state and report to scenario engine
            SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

            // Report updated vehicle position and heading. z, pitch and roll will be aligned to the road
            SE_ReportObjectPosXYH(0, vehicleState.x, vehicleState.y, vehicleState.h);

            // wheel status (revolution and steering angles)
            SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation, vehicleState.wheel_angle);

            // speed (along vehicle longitudinal (x) axis)
            SE_ReportObjectSpeed(0, vehicleState.speed);

            // Finally, update scenario using same time step as for vehicle model
            SE_StepDT(dt);
        }
        else
        {
            // Step with zero delta time, just to catch event disabling pause mode
            SE_StepDT(0.0);
        }
    }

    SE_Close();
}