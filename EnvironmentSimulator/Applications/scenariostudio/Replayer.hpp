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
#include <fstream>
#include "CommonMini.hpp"
#include "ScenarioGateway.hpp"

namespace viewer
{
    class StudioViewer;
}

namespace scenarioengine
{
    class Replay;
}

namespace replayer
{
    Replay* GetReplay();
    void    ReportKeyEvent(viewer::KeyEvent* keyEvent, void* data, float* time);
    int     Run(int argc, char** argv, viewer::StudioViewer* viewer, float* time, bool* quit);
}  // namespace replayer