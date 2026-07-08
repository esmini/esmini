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
#include "Parameters.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_FOLLOW_ROAD_TYPE_NAME "FollowRoadController"

namespace scenarioengine
{
    class ScenarioPlayer;
    class ScenarioEngine;

    class ControllerFollowRoad : public Controller
    {
    public:
        enum class FollowMode
        {
            FOLLOW_MODE_NONE,
            FOLLOW_MODE_TIME,
            FOLLOW_MODE_POSITION,
        };

        ControllerFollowRoad(InitArgs* args);

        virtual const char* GetTypeName() const
        {
            return CONTROLLER_FOLLOW_ROAD_TYPE_NAME;
        }

        virtual Type GetType() const
        {
            return CONTROLLER_TYPE_FOLLOW_ROAD;
        }

        void Init();
        void Step(double timeStep);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);
        void ReportKeyEvent(int key, bool down);
        void SetSetSpeed(double setSpeed)
        {
            set_speed_ = setSpeed;
        }

    private:
        vehicle::Vehicle vehicle_;
        double           set_speed_                   = 0.0;
        double           current_speed_               = 0.0;
        double           speed_change_factor_         = 1.0;
        double           steer_factor_                = 1.0;
        double           lookahead_speed_dist_factor_ = 1.0;
        double           lookahead_steer_dist_factor_ = 1.0;
        idx_t            sensor_idx_                  = IDX_UNDEFINED;
        double           acc_                         = 2.0;  // acceleration/deceleration for speed control
    };

    Controller* InstantiateControllerFollowRoad(void* args);
}  // namespace scenarioengine