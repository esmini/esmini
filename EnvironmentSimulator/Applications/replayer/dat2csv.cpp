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
    Replay*     player;
    static char line[MAX_LINE_LEN];
    bool        extended = false;
    bool        save_csv = false;
    bool        quiet    = false;

    std::setlocale(LC_ALL, "C.UTF-8");

    if (argc < 2)
    {
        printf("Usage: %s <filename> <option>\n\tOption '--extended' increases data output\n", argv[0]);
        return -1;
    }
    else if (argc > 2)
    {
        for (int i = 2; i < argc; i++)
        {
            if (strcmp(argv[i], "--extended") == 0)
            {
                extended = true;
            }
            else if (strcmp(argv[i], "--save_csv") == 0)
            {
                save_csv = true;
            }
            else if (strcmp(argv[i], "--quiet") == 0)
            {
                quiet = true;
            }
            else
            {
                printf("Unrecognized argument %s, ignoring", argv[i]);
            }
        }
    }

    std::string   filename = FileNameWithoutExtOf(argv[1]) + ".csv";
    std::ofstream file;
    if (save_csv)
    {
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
        player = new Replay(argv[1], quiet);
    }
    catch (const std::exception& e)
    {
        printf("%s", e.what());
        return -1;
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