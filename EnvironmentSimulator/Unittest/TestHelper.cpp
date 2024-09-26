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

#include "logger.hpp"

int ParseAndSetLoggerOptions(int argc, char** argv)
{
    if (argc > 1)
    {
        if (!strcmp(argv[1], "--disable_stdout"))
        {
            EnableConsoleLogging(false, true);
        }
        else
        {
            printf("Usage: %s [--disable_stout] [google test options...]\n", argv[0]);
            return -1;
        }
    }
    else
    {
        EnableConsoleLogging(true, true);
    }

    return 0;
}