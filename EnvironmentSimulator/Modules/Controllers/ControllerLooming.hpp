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
 * This is the first step implementation of driver model including
   steering target based on whatever closet within 80m of below points
  - lane center ahead
  - lane boundary tangent point
  - lead vehicle
 * For longitudinal control the plan is to implement looming perception model.
 * Meanwhile using a simple ACC model.
 *
 * This controller is inspired by work done by Ola Benderius:
 * https://research.chalmers.se/en/person/benderiu
 */

#pragma once

#include <string>
#include "Controller.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_LOOMING_TYPE_NAME "LoomingController"

namespace scenarioengine
{
    class ControllerLooming : public Controller
    {
    public:
        ControllerLooming(InitArgs* args);

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_LOOMING_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return CONTROLLER_TYPE_LOOMING;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
        }

        void Init();
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);
        void ReportKeyEvent(int key, bool down);
        void SetSetSpeed(double setSpeed)
        {
            setSpeed_ = setSpeed;
        }
        void Step(double timeStep);
        bool hasFarTan;
        bool getHasFarTan()
        {
            return hasFarTan;
        }

    private:
        vehicle::Vehicle vehicle_;
        bool             active_        = false;
        double           timeGap_       = 1.5;  // target headway time
        double           setSpeed_      = 0.0;
        double           currentSpeed_  = 0.0;
        bool             setSpeedSet_   = false;
        double           prevNearAngle  = 0.0;
        double           prevFarAngle   = 0.0;
        double           steering       = 0.0;
        double           acc            = 0.0;
        double           steering_rate_ = 4.0;
        double           angleDiff      = 0.0;
    };

    Controller* InstantiateControllerLooming(void* args);
}  // namespace scenarioengine