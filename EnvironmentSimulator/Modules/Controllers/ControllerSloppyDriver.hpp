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
 * This controller simulates a bad or dizzy driver by manipulating
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

#pragma once

#include <string>
#include "Controller.hpp"
#include "pugixml.hpp"
#include "Parameters.hpp"

#define CONTROLLER_SLOPPY_DRIVER_TYPE_NAME "SloppyDriverController"

namespace scenarioengine
{
    class SinusoidalTransition
    {
    public:
        SinusoidalTransition() : amplitude_(0), startAngle_(0), start_(0)
        {
        }

        void Init(double start, double targetValueRelative)
        {
            amplitude_  = fabs(0.5 * targetValueRelative);
            startAngle_ = SIGN(targetValueRelative) > 0 ? M_PI : 0;
            offset_     = SIGN(targetValueRelative) > 0 ? +1 : -1;
            start_      = start;
            factor_     = 0.0;
        }

        double amplitude_;
        double startAngle_;
        double start_;
        double offset_;
        double factor_;

        double GetValue();
        double GetFactor()
        {
            return factor_;
        }
        double GetHeading();
        void   SetFactor(double factor)
        {
            factor_ = factor;
        }
    };

    // base class for controllers
    class ControllerSloppyDriver : public Controller
    {
    public:
        ControllerSloppyDriver(InitArgs* args);

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_SLOPPY_DRIVER_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return Controller::Type::CONTROLLER_TYPE_SLOPPY_DRIVER;
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

    private:
        double        sloppiness_;  // range [0-1], default = 0.5
        OSCProperties properties_;
        double        time_;

        SE_SimulationTimer speedTimer_;
        double             speedTimerAverage_;
        double             referenceSpeed_;  // set by default driver
        double             initSpeed_;       // start speed for each timer period
        double             currentSpeed_;
        double             targetFactor_;  // factor to multiply reference speed

        SE_SimulationTimer lateralTimer_;
        double             lateralTimerAverage_;
        double             currentT_;
        double             tFuzz0;
        double             tFuzzTarget;
        double             currentH_;

        const char* type_name_ = "SloppyDriver";
    };

    Controller* InstantiateControllerSloppyDriver(void* args);
}  // namespace scenarioengine