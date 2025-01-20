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

#include <yaml-cpp/yaml.h>
#include <ryml.hpp>
#include <ryml_std.hpp>  // For std::string and std::istream integration
#include <fstream>
#include <iostream>

#define MIN_TIME_STEP 0.01
#define MAX_TIME_STEP 0.1

static bool quit = false;

static void CreateConfigFile(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file)
    {
        std::cerr << "Failed to create file: " << filename << std::endl;
        return;
    }

    // Write YAML content
    file << "options:\n";
    file << "  window: 60 60 800 400\n";
    file << "  log_filepath: c:/tmp/kalle.txt\n";
    file << "\n";
    file << "parameters:\n";
    file << "  tesselation_factor: 1.1\n";

    file.close();
    std::cout << "Config file created: " << filename << std::endl;
}

static void TesYamlcpp()
{
    std::cout << "---start testing yaml-cpp---\n";
    YAML::Node content = YAML::LoadFile("config.yaml");
    std::cout << "content start\n" << content << "\ncontent end\n";
    std::cout << "Picking them up one by one\n";
    std::cout << "window:" << content["options"]["window"] << std::endl;
    std::cout << "log_filepath:" << content["options"]["log_filepath"] << std::endl;
    std::cout << "tesselation_factor:" << content["parameters"]["tesselation_factor"] << std::endl;
    std::cout << "---end testing yaml-cpp---\n";
}

static void TestRyml()
{
    std::cout << "---start testing ryml---\n";
    std::ifstream file("config.yaml");
    if (!file)
    {
        std::cerr << "Failed to open config.yaml" << std::endl;
        return;
    }

    // Load the file into a string
    std::string yaml_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Parse the YAML content
    ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr(yaml_content));
    std::cout << "tree start\n" << tree << "\ntree end\n";
    std::cout << "Picking them up one by one\n";
    std::cout << "window:" << tree["options"]["window"].val() << std::endl;
    std::cout << "log_filepath:" << tree["options"]["log_filepath"].val() << std::endl;
    std::cout << "tesselation_factor:" << tree["parameters"]["tesselation_factor"].val() << std::endl;
    std::cout << "---end testing ryml---\n";
}

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
    CreateConfigFile("config.yaml");
    TesYamlcpp();
    TestRyml();
    OSCParameterDistribution& dist   = OSCParameterDistribution::Inst();
    int                       retval = 0;

    do
    {
        retval = execute_scenario(argc, argv);

    } while (retval == 0 && dist.GetIndex() >= 0 && dist.GetIndex() < static_cast<int>(dist.GetNumPermutations() - 1) && !quit);

    return retval;
}
