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
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);
        void ReportKeyEvent(int key, bool down);

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_OFFROAD_FOLLOWER_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return Controller::Type::CONTROLLER_TYPE_INTERACTIVE;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
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