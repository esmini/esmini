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

/*
 * This controller will follow another object disregarding any road network
 */

#pragma once

#include <string>
#include "Controller.hpp"
#include "pugixml.hpp"
#include "Parameters.hpp"
#include "vehicle.hpp"

#define CONTROLLER_OFFROAD_FOLLOWER_TYPE_NAME "OffroadFollower"

namespace scenarioengine
{
    // base class for controllers
    class ControllerOffroadFollower : public Controller
    {
    public:
        ControllerOffroadFollower(InitArgs* args);

        void Init();
        void Step(double timeStep);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);
        void ReportKeyEvent(int key, bool down);

        virtual const char* GetTypeName() const
        {
            return CONTROLLER_OFFROAD_FOLLOWER_TYPE_NAME;
        }

        virtual int GetType() const
        {
            return Controller::Type::CONTROLLER_TYPE_INTERACTIVE;
        }

    private:
        vehicle::Vehicle vehicle_;
        Object*          follow_entity_;
        double           target_distance_;
        double           steering_rate_;
        double           speed_factor_;
    };

    Controller* InstantiateControllerOffroadFollower(void* args);
}  // namespace scenarioengine