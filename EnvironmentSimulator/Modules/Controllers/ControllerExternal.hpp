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
 * This controller simply deactivates the default controller for the specific object
 * instead esmini expects the state to be reported from an external application.
 * Optionally the ghost feature can be enabled to create a "blue print" run, e.g. to
 * be used by an external driver model.
 */

#pragma once

#include <string>
#include "Controller.hpp"
#include "Parameters.hpp"

#define CONTROLLER_EXTERNAL_TYPE_NAME "ExternalController"

namespace scenarioengine
{
    // base class for controllers
    class ControllerExternal : public Controller
    {
    public:
        ControllerExternal(InitArgs* args);

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_EXTERNAL_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return CONTROLLER_TYPE_EXTERNAL;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
        }

        void Init();
        void Step(double timeStep);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);
        void ReportKeyEvent(int key, bool down);
        bool UseGhost()
        {
            return useGhost_;
        }

    private:
        bool   useGhost_;
        double headstart_time_;
    };

    Controller* InstantiateControllerExternal(void* args);

}  // namespace scenarioengine