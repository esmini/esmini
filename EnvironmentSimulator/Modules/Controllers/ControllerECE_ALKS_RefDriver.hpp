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
#include "EmbeddedController.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_ECE_ALKS_REF_DRIVER_TYPE_NAME "ECE_ALKS_RefDriverController"

namespace scenarioengine
{
    class ScenarioPlayer;

    namespace controller
    {
        // base class for controllers
        class ControllerECE_ALKS_REF_DRIVER : public controller::EmbeddedController
        {
        public:
            ControllerECE_ALKS_REF_DRIVER(InitArgs* args);

            // std::string GetName() const override;

            controller::Type GetType() const override;

            // Riz
            /*
            static const char* GetTypeNameStatic()
            {
                return CONTROLLER_ECE_ALKS_REF_DRIVER_TYPE_NAME;
            }
            virtual const char* GetTypeName()
            {
                return GetTypeNameStatic();
            }
            static int GetTypeStatic()
            {
                return CONTROLLER_TYPE_ECE_ALKS_REF_DRIVER;
            }
            virtual int GetType()
            {
                return GetTypeStatic();
            }
            */
            void         Init();
            void         Step(double timeStep);
            int          Activate(controller::ControlActivationMode lat_activation_mode,
                                  controller::ControlActivationMode long_activation_mode,
                                  controller::ControlActivationMode light_activation_mode,
                                  controller::ControlActivationMode anim_activation_mode);
            void         Reset();
            virtual void ReportKeyEvent(int key, bool down);

        private:
            vehicle::Vehicle vehicle_;
            bool             active_;
            double           setSpeed_;
            double           currentSpeed_;
            bool             logging_;

            double dtFreeCutOut_;
            bool   cutInDetected_;
            double waitTime_;
            bool   driverBraking_;
            bool   aebBraking_;
            double timeSinceBraking_;
        };

        ControllerBase* InstantiateControllerECE_ALKS_REF_DRIVER(void* args);

    }  // namespace controller

}  // namespace scenarioengine
