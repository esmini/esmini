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

#include "Action.hpp"
#include "CommonMini.hpp"

using namespace scenarioengine;

unsigned int OSCAction::n_actions_ = 0;

std::string OSCAction::BaseType2Str()
{
    BaseType base_type = GetBaseType();
    if (base_type == BaseType::GLOBAL)
    {
        return "Global";
    }
    else if (base_type == BaseType::PRIVATE)
    {
        return "Private";
    }
    else if (base_type == BaseType::USER_DEFINED)
    {
        return "User defined";
    }
    else
    {
        LOG("Undefined Base Type: %d", base_type);
    }

    return "Undefined";
}