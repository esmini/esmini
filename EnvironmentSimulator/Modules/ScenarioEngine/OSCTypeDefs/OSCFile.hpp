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
#include <iostream>
#include <string>
#include "CommonMini.hpp"

namespace scenarioengine
{

    class OSCFile
    {
    public:
        std::string filepath;

        void Print()
        {
            LOG("file: %s", filepath.c_str());
        };
    };

}  // namespace scenarioengine