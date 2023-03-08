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

namespace scenarioengine
{

    typedef enum
    {
        GREATER_THAN,
        GREATER_OR_EQUAL,
        LESS_THAN,
        LESS_OR_EQUAL,
        EQUAL_TO,
        NOT_EQUAL_TO,
        UNDEFINED_RULE
    } Rule;

    typedef enum
    {
        LONGITUDINAL,
        LATERAL,
        VERTICAL,
        UNDEFINED_DIRECTION
    } Direction;

}  // namespace scenarioengine