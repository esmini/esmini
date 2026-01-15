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
 * This application uses the Replay class to read and binary recordings and print content in ascii format to stdout
 */

#include <clocale>

#include "Replay.hpp"
#include "CommonMini.hpp"
#include "PacketHandler.hpp"

using namespace scenarioengine;

#define MAX_LINE_LEN 2048

int main(int argc, char** argv)
{
    SE_Options& opt = SE_Env::Inst().GetOptions();
    opt.Reset();

    SE_Env::Inst().AddPath(DirNameOf(argv[0]));

    opt.AddOption("csv", "Optional name of the output file (default is Dat-file-name with .csv extension)", "name");
    opt.AddOption("extended", "Make csv with extended data fields");
    opt.AddOption("file", "Dat-file to convert to csv", "filename", "", false, true, true);
    opt.AddOption("help", "Show this help message (-h works as well)");
    opt.AddOption("file_refs", "include odr model, and git file references");
    opt.AddOption("print_csv", "Print the csv to stdout instead of saving it");
    opt.AddOption("version", "Show version and quit");

    if (int ret = OnRequestShowHelpOrVersion(argc, argv, opt); ret > 0)
    {
        return ret;
    }

    if (opt.ParseArgs(argc, argv) != 0 || argc < 2)
    {
        opt.PrintUsage();
        return -1;
    }

    if (opt.HasUnknownArgs())
    {
        opt.PrintUnknownArgs("Unrecognized arguments:");
        opt.PrintUsage();
        return -1;  // we harmonize all applications to quit on unknown arguments
    }

    Replay*     player;
    static char line[MAX_LINE_LEN];
    bool        extended  = opt.GetOptionSet("extended");
    bool        save_csv  = !opt.GetOptionSet("print_csv");
    bool        file_refs = opt.GetOptionSet("file_refs");

    std::setlocale(LC_ALL, "C.UTF-8");

    std::string dat_file = opt.GetOptionValue("file");

    std::ofstream file;
    if (save_csv)
    {
        std::string filename = opt.GetOptionValue("csv");
        if (filename.empty())
        {
            filename = FilePathWithoutExtOf(dat_file) + ".csv";
        }
        file.open(filename);
        if (!file.is_open())
        {
            printf("Failed to create file %s\n", filename.c_str());
            return -1;
        }
    }

    // Create replayer object for parsing the binary data file
    try
    {
        player = new Replay(dat_file, true);
    }
    catch (const std::exception& e)
    {
        printf("%s", e.what());
        return -1;
    }

    if (file_refs && save_csv)
    {
        snprintf(line,
                 MAX_LINE_LEN,
                 "Version: %d.%d, OpenDRIVE: %s, 3DModel: %s GIT REV: %s\n",
                 player->dat_header_.version_major,
                 player->dat_header_.version_minor,
                 player->dat_header_.odr_filename.string.c_str(),
                 player->dat_header_.model_filename.string.c_str(),
                 player->dat_header_.git_rev.string.c_str());
        file << line;
    }
    else if (file_refs)
    {
        snprintf(line,
                 MAX_LINE_LEN,
                 "Version: %d.%d, OpenDRIVE: %s, 3DModel: %s GIT REV: %s\n",
                 player->dat_header_.version_major,
                 player->dat_header_.version_minor,
                 player->dat_header_.odr_filename.string.c_str(),
                 player->dat_header_.model_filename.string.c_str(),
                 player->dat_header_.git_rev.string.c_str());
        printf("%s", line);
    }

    if (!extended)
    {
        snprintf(line, MAX_LINE_LEN, "time, id, name, x, y, z, h, p, r, speed, wheel_angle, wheel_rot\n");
    }
    else
    {
        snprintf(line, MAX_LINE_LEN, "time, id, name, x, y, z, h, p, r, roadId, laneId, offset, t, s, speed, wheel_angle, wheel_rot\n");
    }

    if (save_csv)
    {
        file << line;
    }
    else
    {
        printf("%s", line);
    }

    // If not fixed timestep in log, we loop over all timestamps_
    for (size_t i = 0; i < player->timestamps_.size(); i++)
    {
        for (const auto& [id, _] : player->objects_timeline_)
        {
            auto                  entry = player->GetReplayEntryAtTimeIncremental(id, player->timestamps_[i]);
            ObjectStateStructDat* state = &entry.state;

            if (!state->info.active)
            {
                continue;
            }

            // Output all entries with comma separated values
            if (!extended)
            {
                snprintf(line,
                         MAX_LINE_LEN,
                         "%.3f, %d, %s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                         static_cast<double>(state->info.timeStamp),
                         state->info.id,
                         state->info.name.c_str(),
                         static_cast<double>(state->pos.x),
                         static_cast<double>(state->pos.y),
                         static_cast<double>(state->pos.z),
                         static_cast<double>(state->pos.h),
                         static_cast<double>(state->pos.p),
                         static_cast<double>(state->pos.r),
                         static_cast<double>(state->info.speed),
                         static_cast<double>(state->info.wheel_angle),
                         static_cast<double>(state->info.wheel_rot));
            }
            else
            {
                snprintf(line,
                         MAX_LINE_LEN,
                         "%.3f, %d, %s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f \n",
                         static_cast<double>(state->info.timeStamp),
                         state->info.id,
                         state->info.name.c_str(),
                         static_cast<double>(state->pos.x),
                         static_cast<double>(state->pos.y),
                         static_cast<double>(state->pos.z),
                         static_cast<double>(state->pos.h),
                         static_cast<double>(state->pos.p),
                         static_cast<double>(state->pos.r),
                         state->pos.roadId,
                         state->pos.laneId,
                         static_cast<double>(state->pos.offset),
                         static_cast<double>(state->pos.t),
                         static_cast<double>(state->pos.s),
                         static_cast<double>(state->info.speed),
                         static_cast<double>(state->info.wheel_angle),
                         static_cast<double>(state->info.wheel_rot));
            }

            if (save_csv)
            {
                file << line;
            }
            else
            {
                printf("%s", line);
            }
        }
    }

    if (file.is_open())
    {
        file.close();
    }

    delete player;
}