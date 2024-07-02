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
 * This controller let the user control the vehicle interactively by the arrow keys
 */

#pragma once

#include <string>
#include "EmbeddedController.hpp"
#include "pugixml.hpp"
#include "Parameters.hpp"
#include "vehicle.hpp"

#define CONTROLLER_INTERACTIVE_TYPE_NAME "InteractiveController"

namespace scenarioengine::controller
{
    // base class for controllers
    class ControllerInteractive : public controller::EmbeddedController
    {
    public:
        ControllerInteractive(InitArgs* args);

        // std::string GetName() const override;

        controller::Type GetType() const override;

        void         Init();
        void         Step(double timeStep);
        int          Activate(ControlActivationMode lat_activation_mode,
                              ControlActivationMode long_activation_mode,
                              ControlActivationMode light_activation_mode,
                              ControlActivationMode anim_activation_mode);
        virtual void ReportKeyEvent(int key, bool down);
        // Riz
        /*
        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_INTERACTIVE_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return controller::Type::CONTROLLER_TYPE_INTERACTIVE;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
        }
        */
    private:
        vehicle::Vehicle  vehicle_;
        vehicle::THROTTLE accelerate = vehicle::THROTTLE_NONE;
        vehicle::STEERING steer      = vehicle::STEERING_NONE;
        double            steering_rate_;
        double            speed_factor_;
    };

    ControllerBase* InstantiateControllerInteractive(void* args);
}  // namespace scenarioengine::controller