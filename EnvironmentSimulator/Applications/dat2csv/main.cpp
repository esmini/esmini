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

/*
 * This application uses the Dat2csv class to convert to dat to csv format of previously executed scenarios stored in dat format
 */

#include <clocale>

#include <string>
#include <fstream>

#include "CommonMini.hpp"
#include "Dat2csv.hpp"

int main(int argc, char** argv)
{
    SE_Options opt;
    opt.AddOption("file", "Simulation recording data file (.dat)", "filename");
    opt.AddOption("extended", "add road coordinates");
    opt.AddOption("file_refs", "include odr and model file references");
    opt.AddOption("time_mode",
                  "control timestamps in the csv (original, min_step, min_step_mixed, custom_time_step, custom_time_step_mixed)",
                  "mode",
                  "original");
    opt.AddOption("time_step", "use fixed time step (ms) - overrides time_mode", "time_step", "0.05");

    std::setlocale(LC_ALL, "C.UTF-8");

    if (opt.ParseArgs(argc, argv) != 0 || argc < 1)
    {
        opt.PrintUsage();
        return -1;
    }

    if (opt.GetOptionArg("file").empty())
    {
        printf("Missing file argument\n");
        opt.PrintUsage();
        return -1;
    }

    std::unique_ptr<Dat2csv> dat_to_csv;

    // Create dat2csv object for parsing the binary data file
    try
    {
        dat_to_csv = std::make_unique<Dat2csv>(opt.GetOptionArg("file"));
    }
    catch (const std::exception& e)
    {
        printf("%s", e.what());
        return -1;
    }

    if (opt.GetOptionSet("extended"))
    {
        std::cout << "added extended" << std::endl;
        dat_to_csv->SetLogExtended(true);
    }

    if (opt.GetOptionSet("file_refs"))
    {
        std::cout << "Including file references" << std::endl;
        dat_to_csv->SetIncludeRefs(true);
    }

    Dat2csv::log_mode logMode = Dat2csv::log_mode::ORIGINAL;
    if (opt.GetOptionSet("time_mode"))
    {
        std::string time_mode_str = opt.GetOptionArg("time_mode");
        if (!time_mode_str.empty())
        {
            if (time_mode_str == "original")
            {
                logMode = Dat2csv::log_mode::ORIGINAL;
            }
            else if (time_mode_str == "min_step")
            {
                logMode = Dat2csv::log_mode::MIN_STEP;
            }
            else if (time_mode_str == "min_step_mixed")
            {
                logMode = Dat2csv::log_mode::MIN_STEP_MIXED;
            }
            else if (time_mode_str == "custom_time_step")
            {
                logMode = Dat2csv::log_mode::CUSTOM_TIME_STEP;
            }
            else if (time_mode_str == "custom_time_step_mixed")
            {
                logMode = Dat2csv::log_mode::CUSTOM_TIME_STEP_MIXED;
            }
            else
            {
                LOG_WARN("Unsupported time mode: {} - using default (Original)", time_mode_str);
            }
        }
    }

    if (opt.GetOptionSet("time_step"))
    {
        std::string timeStep_str = opt.GetOptionArg("time_step");
        if (!timeStep_str.empty())
        {
            double delta_time_step = strtod(timeStep_str);
            dat_to_csv->SetStepTime(delta_time_step);
        }
        else
        {
            LOG_WARN("Failed to provide fixed time step, Logging with default step time 0.05 ");
        }
    }

    dat_to_csv->SetLogMode(logMode);
    dat_to_csv->CreateCSV();
}