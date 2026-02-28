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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "ScenarioEngine.hpp"

typedef struct
{
    double x;            // m
    double y;            // m
    double z;            // m
    double h;            // rad
    double p;            // rad
    double r;            // rad
    double speed;        // m/s
    double wheel_angle;  // rad
} EgoStateBuffer_t;

namespace scenarioengine
{
    void StartServer(ScenarioEngine *scenarioEngine);
    void StopServer();
}  // namespace scenarioengine
