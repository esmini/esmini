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

#define CONTROLLER_SUMO_TYPE_NAME "SumoController"

namespace scenarioengine
{
    // base class for controllers
    class ControllerSumo : public Controller
    {
    public:
        ControllerSumo(InitArgs* args);

        virtual const char* GetTypeName()
        {
            return CONTROLLER_SUMO_TYPE_NAME;
        }
        virtual int GetType()
        {
            return CONTROLLER_TYPE_SUMO;
        }

        void Init();
        void Step(double timeStep);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);

        void SetSumoVehicle(Object* object);

    private:
        float              sumo_x_offset_;
        float              sumo_y_offset_;
        double             time_;
        pugi::xml_document docsumo_;
        std::string        model_filepath_;
        Object*            template_vehicle_;
    };

    Controller* InstantiateControllerSumo(void* args);
}  // namespace scenarioengine