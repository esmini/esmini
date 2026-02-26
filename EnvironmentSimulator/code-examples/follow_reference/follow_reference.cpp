/*
 * This code example demonstrates a simple vehicle following a reference vehicle in realtime
 * using a Stanley controller for the steering angle and a simple critically damped spring model
 * for the longitudinal acceleration.
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

#include "esminiLib.hpp"

#define SIGN(x) ((x) < 0 ? -1 : 1)

double distance_point_to_line(double px, double py, double lx, double ly, double angle)
{
    return (px - lx) * std::sin(angle) - (py - ly) * std::cos(angle);
}

void rotate_vec(double x, double y, double angle, double& xr, double& yr)
{
    xr = x * cos(angle) - y * sin(angle);
    yr = x * sin(angle) + y * cos(angle);
}

double normalize_angle(double angle)
{
    // Normalize to the range [-2*pi, 2*pi] first.
    angle = fmod(angle, 2.0 * M_PI);

    // Shift the range from [-2*pi, 2*pi] to [-pi, pi]
    if (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    else if (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}

class StanleyController
{
public:
    // Create a Stanley controller with given gains
    // @param cross_track_error_gain: Gain for the cross-track error term, recommended range [0.5:3.0]
    // @param speed_stabilizer: Speed gain to avoid singularities at low speeds, recommended range [0.5:2.0]
    StanleyController(double cross_track_error_gain, double speed_stabilizer)
        : cross_track_error_gain_(cross_track_error_gain),
          speed_stabilizer_(speed_stabilizer)
    {
    }

    double computeSteering(double dy, double dh, double speed) const
    {
        return dh + std::atan2(cross_track_error_gain_ * dy, speed_stabilizer_ + speed);
    }

private:
    double cross_track_error_gain_;  // Cross-track error gain
    double speed_stabilizer_;        // Speed gain
};

class CriticallyDampedSpring
{
public:
    CriticallyDampedSpring(double stiffness) : k_(stiffness)
    {
        d_ = 2.0 * std::sqrt(stiffness);
    }

    double computeAcceleration(double pos, double vel) const
    {
        return -k_ * pos - d_ * vel;
    }

private:
    double d_;  // damping coefficient
    double k_;  // stiffness coefficient
};

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const double x_offset     = 2.8;  // Distance from the reference point at the rear axle to the front axle which is used for steering control
    const float  fixed_dt     = 1.0f / 60.0f;
    const bool   use_fixed_dt = false;
    const bool   visualize    = true;

    // create controllers for longitudinal and lateral operations
    CriticallyDampedSpring speed_ctrl(1.0);
    StanleyController      steer_ctrl(10.0, 1.0);

    // use fixed IDs for ego and the reference vehicle
    int ego_id = 0;
    int ref_id = 1;

    // initialize esmini, establish initial states for given scenario
    if (SE_Init("../../../../EnvironmentSimulator/code-examples/follow_reference/follow_reference.xosc", 0, visualize ? 1 : 0, 0, 1) != 0)
    {
        printf("Failed to initialize the scenario, quit\n");
        SE_LogMessage("Failed to initialize the scenario, quit\n");
        return -1;
    }

    // fetch initial state of reference vehicle and create simple vehicle, based on bicycle kinematic model, to represent ego
    SE_ScenarioObjectState ref_state;
    SE_GetObjectState(ref_id, &ref_state);
    SE_SimpleVehicleState ego_state;
    void*                 ego_handle = SE_SimpleVehicleCreate(ref_state.x, ref_state.y, ref_state.h, 4.0, ref_state.speed);
    SE_SimpleVehicleGetState(ego_handle, &ego_state);

    // show some road features, including road sensor
    SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

    // Run for specified duration or until 'Esc' button is pressed
    while (SE_GetQuitFlag() == 0)
    {
        float dt = use_fixed_dt ? fixed_dt : SE_GetSimTimeStep();
        if (!SE_GetPauseFlag())
        {
            // find lateral distance between Ego and reference vehicle x-axis
            double front_axle[2] = {0.0, 0.0};
            if (ego_state.speed >= 0.0f)
            {
                // measure from Ego front axle when driving forward
                rotate_vec(x_offset, 0.0, ego_state.h, front_axle[0], front_axle[1]);
            }
            double dy = distance_point_to_line(static_cast<double>(ego_state.x) + front_axle[0],
                                               static_cast<double>(ego_state.y) + front_axle[1],
                                               ref_state.x,
                                               ref_state.y,
                                               ref_state.h);

            // find longitudinal distance and heading difference between Ego and reference vehicle
            double dx = distance_point_to_line(ego_state.x, ego_state.y, ref_state.x, ref_state.y, static_cast<double>(ref_state.h) + M_PI_2);
            double dh = normalize_angle(ref_state.h - ego_state.h);

            SE_SimpleVehicleControlAccAndSteer(ego_handle,
                                               dt,
                                               speed_ctrl.computeAcceleration(dx, ego_state.speed - ref_state.speed),
                                               steer_ctrl.computeSteering(dy, dh * SIGN(ego_state.speed), ego_state.speed));

            // Fetch updated Ego state and report to scenario engine
            SE_SimpleVehicleGetState(ego_handle, &ego_state);

            SE_ReportObjectPosXYH(ego_id, ego_state.x, ego_state.y, ego_state.h);
            SE_ReportObjectWheelStatus(ego_id, ego_state.wheel_rotation, ego_state.wheel_angle);
            SE_ReportObjectSpeed(ego_id, ego_state.speed);
        }

        SE_StepDT(dt);
        SE_GetObjectState(ref_id, &ref_state);  // fetch reference state for next iteration
    }

    SE_Close();

    return 0;
}