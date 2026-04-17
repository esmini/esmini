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

#define CONTROLLER_FOLLOW_GHOST_TYPE_NAME "FollowGhostController"

namespace scenarioengine
{
    class ScenarioPlayer;
    class ScenarioEngine;

    // base class for controllers
    class ControllerFollowGhost : public Controller
    {
    public:
        enum class FollowMode
        {
            FOLLOW_MODE_NONE,
            FOLLOW_MODE_TIME,
            FOLLOW_MODE_POSITION,
        };

        ControllerFollowGhost(InitArgs* args);

        virtual const char* GetTypeName() const
        {
            return CONTROLLER_FOLLOW_GHOST_TYPE_NAME;
        }

        virtual Type GetType() const
        {
            return CONTROLLER_TYPE_FOLLOW_GHOST;
        }

        void Init();
        void Step(double timeStep);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);
        void ReportKeyEvent(int key, bool down);

    private:
        vehicle::Vehicle vehicle_;
        double           headstart_time_;
        FollowMode       follow_mode_;
        double           lookahead_speed_;
        double           min_lookahead_speed_;
        double           lookahead_steering_;
        double           min_lookahead_steering_;
        double           steering_speed_inertia_;
    };

    Controller* InstantiateControllerFollowGhost(void* args);
}  // namespace scenarioengine