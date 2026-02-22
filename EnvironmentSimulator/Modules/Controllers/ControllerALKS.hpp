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
#include "ControllerACC.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_ALKS_TYPE_NAME "ALKSController"

// As a starting point the ALKS controller inherit all functionality of the ACC controller

namespace scenarioengine
{
    class ControllerALKS : public ControllerACC
    {
    public:
        ControllerALKS(InitArgs* args) : ControllerACC(args)
        {
        }

        virtual const char* GetTypeName() const
        {
            return CONTROLLER_ALKS_TYPE_NAME;
        }
        virtual Type GetType() const
        {
            return CONTROLLER_TYPE_ALKS;
        }
    };

    Controller* InstantiateControllerALKS(void* args);
}  // namespace scenarioengine