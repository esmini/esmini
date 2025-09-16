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
    StanleyController(double k_e, double k_psi) : k_e_(k_e), k_psi_(k_psi)
    {
    }

    double computeSteering(double dy, double dh, double speed) const
    {
        double psi_e          = normalize_angle(dh);
        double e_t            = dy;
        double steering_angle = psi_e + std::atan2(k_e_ * e_t, k_psi_ + speed);

        return steering_angle;
    }

private:
    double k_e_;    // Cross-track error gain
    double k_psi_;  // Speed gain
};

class CriticallyDampedSpring
{
public:
    CriticallyDampedSpring(double stiffness, double pos_init, double vel_init) : k_(stiffness), pos_(pos_init), vel_(vel_init), acc_(0.0)
    {
        d_ = 2.0 * std::sqrt(stiffness);
    }

    void setPosition(double pos)
    {
        pos_ = pos;
    }

    void setVelocity(double velocity)
    {
        vel_ = velocity;
    }

    void update(double dt)
    {
        acc_ = -k_ * pos_ - d_ * vel_;
        vel_ += acc_ * dt;
        pos_ += vel_ * dt;
    }

    double getAcceleration() const
    {
        return acc_;
    }

private:
    double d_;    // damping coefficient
    double k_;    // stiffness coefficient
    double pos_;  // position
    double vel_;  // velocity
    double acc_;  // acceleration
};

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    // create controllers for longitudinal and lateral operations
    CriticallyDampedSpring speed_ctrl(1, 0.0, 0.0);
    StanleyController      steer_ctrl(1.0, 1.0);

    // use fixed IDs for ego and the reference vehicle
    int ego_id = 0;
    int ref_id = 1;

    // initialize esmini, establish initial states for given scenario
    if (SE_Init("../EnvironmentSimulator/code-examples/follow_reference/follow_reference.xosc", 0, 1, 0, 1) != 0)
    {
        SE_LogMessage("Failed to initialize the scenario, quit\n");
        return -1;
    }

    // fetch initial state of reference vehicle and create simple vehicle, based on bicycle kinematic model, to represent ego
    SE_ScenarioObjectState ref_state;
    SE_GetObjectState(ref_id, &ref_state);
    SE_SimpleVehicleState ego_state;
    void*                 ego_handle = SE_SimpleVehicleCreate(ref_state.x, ref_state.y, ref_state.h, 4.0, 0.0);
    SE_SimpleVehicleGetState(ego_handle, &ego_state);

    // show some road features, including road sensor
    SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

    // Run for specified duration or until 'Esc' button is pressed
    while (SE_GetQuitFlag() == 0)
    {
        // float dt = SE_GetSimTimeStep();  // Get simulation delta time since last call
        static float dt = 1.0f / 60.0f;  // 60 fps

        if (!SE_GetPauseFlag())
        {
            // find lateral diff
            double dy = distance_point_to_line(ego_state.x, ego_state.y, ref_state.x, ref_state.y, ref_state.h);
            double dx = distance_point_to_line(ego_state.x, ego_state.y, ref_state.x, ref_state.y, static_cast<double>(ref_state.h) + M_PI_2);
            double dh = normalize_angle(ref_state.h - ego_state.h);

            speed_ctrl.setPosition(dx);
            speed_ctrl.setVelocity(ego_state.speed - ref_state.speed);
            speed_ctrl.update(dt);

            SE_SimpleVehicleControlAccAndSteer(ego_handle,
                                               dt,
                                               speed_ctrl.getAcceleration(),
                                               steer_ctrl.computeSteering(dy, dh * SIGN(ego_state.speed), ego_state.speed));

            // Fetch updated Ego state and report to scenario engine
            SE_SimpleVehicleGetState(ego_handle, &ego_state);

            SE_ReportObjectPosXYH(ego_id, 0.0f, ego_state.x, ego_state.y, ego_state.h);
            SE_ReportObjectWheelStatus(ego_id, ego_state.wheel_rotation, ego_state.wheel_angle);
            SE_ReportObjectSpeed(ego_id, ego_state.speed);
        }

        SE_StepDT(dt);
        SE_GetObjectState(ref_id, &ref_state);  // fetch reference state for next iteration
    }

    SE_Close();

    return 0;
}