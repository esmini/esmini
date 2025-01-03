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

#include "playerbase.hpp"
#include "CommonMini.cpp"
#include "OSCParameterDistribution.hpp"
#include "Plot.hpp"
#include <osgViewer/ViewerEventHandlers>
#include <signal.h>

#define MIN_TIME_STEP 0.01
#define MAX_TIME_STEP 0.1

static bool quit = false;

static void signal_handler(int s)
{
    if (s == SIGINT)
    {
        LOG_INFO("Quit request from user");
        quit = true;
    }
}

static int execute_scenario(int argc, char* argv[])
{
    __int64     time_stamp = 0;
    int         retval     = 0;
    SE_Options& opt        = SE_Env::Inst().GetOptions();

    std::unique_ptr<ScenarioPlayer> player;

    // Setup signal handler to catch Ctrl-C
    signal(SIGINT, signal_handler);

    try
    {
        player = std::make_unique<ScenarioPlayer>(argc, argv);
        if (player->Init() != 0)
        {
            return -1;
        }

        if (opt.GetOptionSet("return_nr_permutations"))
        {
            // Skip scenario, return immediately
            return static_cast<int>(OSCParameterDistribution::Inst().GetNumPermutations());
        }
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("Exception: {}", e.what());
        return -1;
    }

#ifdef _USE_IMPLOT
    // Initialize ImPlot
    std::unique_ptr<Plot> plot;

    if (opt.GetOptionSet("plot"))
    {
        // Create and run plot in a separate thread as default
        plot = std::make_unique<Plot>(player->scenarioEngine, opt.GetOptionArg("plot") == "synchronous");
    }
#endif  // _USE_IMPLOT

    while (!player->IsQuitRequested() && !quit && retval == 0)
    {
        double dt;
        if (player->GetFixedTimestep() > SMALL_NUMBER)
        {
            dt = player->GetFixedTimestep();
        }
        else
        {
            dt = SE_getSimTimeStep(time_stamp, player->minStepSize, player->maxStepSize);
        }

        retval = player->Frame(dt);

#ifdef _USE_IMPLOT
        if (plot != nullptr && plot->IsModeSynchronuous())
        {
            plot->Frame();
        }
#endif  // _USE_IMPLOT
    }

    if (opt.IsOptionArgumentSet("param_permutation"))
    {
        // Single permutation requested and executed, quit now
        quit = true;
    }

#ifdef _USE_IMPLOT
    if (plot != nullptr)
    {
        plot->Quit();  // ensure plot window is closed correctly
    }
#endif  // _USE_IMPLOT

    return (retval < 0 ? -1 : 0);
}

int main(int argc, char* argv[])
{
    OSCParameterDistribution& dist   = OSCParameterDistribution::Inst();
    int                       retval = 0;

    do
    {
        retval = execute_scenario(argc, argv);

    } while (retval == 0 && dist.GetIndex() >= 0 && dist.GetIndex() < static_cast<int>(dist.GetNumPermutations() - 1) && !quit);

    return retval;
}
