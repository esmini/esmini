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
#include <thread>
#include <future>

#define MIN_TIME_STEP 0.01
#define MAX_TIME_STEP 0.1

static bool quit = false;

static void signal_handler(int s)
{
    if (s == SIGINT)
    {
        LOG("Quit request from user");
        quit = true;
    }
}

static int execute_scenario(int argc, char* argv[])
{
    __int64 time_stamp = 0;
    int     retval     = 0;

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

        if (player->opt.GetOptionSet("return_nr_permutations"))
        {
            // Skip scenario, return immediately
            return static_cast<int>(OSCParameterDistribution::Inst().GetNumPermutations());
        }
    }
    catch (const std::exception& e)
    {
        LOG(std::string("Exception: ").append(e.what()).c_str());
        return -1;
    }

#ifdef _USE_IMPLOT
    // Initialize ImPlot
    std::unique_ptr<Plot> plot;
    std::thread plot_thread;

    if (player->opt.GetOptionSet("plot"))
    {
        plot = std::make_unique<Plot>(player->scenarioEngine->entities_.object_);
        plot_thread = std::thread(&Plot::renderImguiWindow, plot.get());
    }
#endif

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

#ifdef _USE_IMPLOT
        if (plot && !player->IsPaused())
        {
            plot->updateData(player->scenarioEngine->entities_.object_, dt);
        }
#endif

        retval = player->Frame(dt);
    }

    if (player->opt.IsOptionArgumentSet("param_permutation"))
    {
        // Single permutation requested and executed, quit now
        quit = true;
    }


#ifdef _USE_IMPLOT
    plot->set_quit_flag();
    plot_thread.join();
#endif

    return retval;
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
