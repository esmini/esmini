/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once

#include <string>
#include "Controller.hpp"
#include "pugixml.hpp"
#include "Parameters.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_FOLLOW_REFERENCE_TYPE_NAME "FollowReferenceController"

namespace scenarioengine
{
    class ScenarioPlayer;
    class ScenarioEngine;

    class StanleyController
    {
    public:
        StanleyController() = default;

        // Set gain for the cross-track error term, recommended range [0.5:3.0]
        void SetCrossTrackError(double gain)
        {
            cross_track_error_gain_ = gain;
        }

        // Set speed stabilizer gain to avoid singularities at low speeds, recommended range [0.5:2.0]
        void SetSpeedStabilizer(double gain)
        {
            speed_stabilizer_ = gain;
        }

        // Set heading diff term gain, recommended value 1.0
        void SetHeadingGain(double gain)
        {
            heading_gain_ = gain;
        }

        // Set curvature diff term gain, recommended value 1.0
        void SetCurvatureGain(double gain)
        {
            curvature_gain_ = gain;
        }

        // Compute steering angle
        // @param dy: lateral distance between ego front axle and reference vehicle x-axis
        // @param dh: heading difference between ego and reference vehicle
        // @param speed: ego speed
        // @param curvature: reference path/trajectory curvature
        // @param wheelbase: ego vehicle wheelbase
        // @return steering angle
        double computeSteering(double dy, double dh, double speed, double curvature, double wheelbase) const
        {
            return heading_gain_ * dh + std::atan2(cross_track_error_gain_ * dy, speed_stabilizer_ + speed) +
                   curvature_gain_ * std::atan(curvature * wheelbase);
        }

    private:
        double cross_track_error_gain_ = 1.0;  // Cross-track error gain
        double speed_stabilizer_       = 1.0;  // Speed stabilizer gain
        double heading_gain_           = 1.0;  // Heading gain
        double curvature_gain_         = 1.0;  // Curvature gain
    };

    class CriticallyDampedSpring
    {
    public:
        CriticallyDampedSpring() = default;

        // Set stiffness coefficient, damping coefficient is set to critical damping accordingly
        void SetStiffness(double stiffness)
        {
            k_ = stiffness;
            d_ = 2.0 * std::sqrt(stiffness);
        }

        double computeAcceleration(double pos, double vel) const
        {
            return -k_ * pos - d_ * vel;
        }

    private:
        double d_ = 1.0;  // damping coefficient
        double k_ = 1.0;  // stiffness coefficient
    };

    // base class for controllers
    class ControllerFollowReference : public Controller
    {
    public:
        enum class FollowMode
        {
            FOLLOW_MODE_NONE,
            FOLLOW_MODE_TIME,
            FOLLOW_MODE_POSITION,
        };

        ControllerFollowReference(InitArgs* args);

        virtual const char* GetTypeName() const
        {
            return CONTROLLER_FOLLOW_REFERENCE_TYPE_NAME;
        }

        virtual Type GetType() const
        {
            return CONTROLLER_TYPE_FOLLOW_REFERENCE;
        }

        void Init();
        void Step(double timeStep);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);
        void ReportKeyEvent(int key, bool down);

        std::string GetReferenceName() const
        {
            return reference_entity_name_;
        }

    private:
        vehicle::Vehicle       vehicle_;
        double                 stanley_cross_track_error_gain_ = 1.0;  // Stanley cross-track error gain
        double                 stanley_speed_stabilizer_       = 1.0;  // Stanley speed stabilizer term
        double                 stanley_heading_gain_           = 1.0;  // Stanley heading gain
        double                 stanley_curvature_gain_         = 1.0;  // Stanley curvature gain
        double                 speed_control_stiffness_        = 1.0;  // Speed control stiffness
        StanleyController      steer_ctrl_;
        CriticallyDampedSpring speed_ctrl_;
        double                 wheelbase_  = 0.0;
        bool                   hide_ghost_ = false;
        std::string            reference_entity_name_;
        Object*                reference_entity_ = nullptr;
    };

    Controller* InstantiateControllerFollowReference(void* args);
}  // namespace scenarioengine