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

    std::setlocale(LC_ALL, "C.UTF-8");

    if (argc < 2)
    {
        printf("Usage: %s <filename>\n", argv[0]);
        return -1;
    }

    std::string   filename = FileNameWithoutExtOf(argv[1]) + ".csv";
    std::ofstream file;
    file.open(filename);
    if (!file.is_open())
    {
        printf("Failed to create file %s\n", filename.c_str());
        return -1;
    }

    // Create replayer object for parsing the binary data file
    try
    {
        player = new Replay(argv[1]);
    }
    catch (const std::exception& e)
    {
        printf("%s", e.what());
        return -1;
    }

    // First output header and CSV labels
    snprintf(line,
             MAX_LINE_LEN,
             "Version: %d.%d, OpenDRIVE: %s, 3DModel: %s\n",
             player->dat_header_.version_major,
             player->dat_header_.version_minor,
             player->dat_header_.odr_filename.string.c_str(),
             player->dat_header_.model_filename.string.c_str());
    file << line;
    snprintf(line, MAX_LINE_LEN, "time, id, name, x, y, z, h, p, r, speed, wheel_angle, wheel_rot\n");
    file << line;

    float fixed_timestep = player->GetFixedTimestep();

    // If not fixed timestep in log, we loop over all timestamps_
    if (fixed_timestep < 0.0f)
    {
        for (size_t i = 0; i < player->timestamps_.size(); i++)
        {
            for (const auto& [id, _] : player->objects_timeline_)
            {
                auto                  entry = player->GetReplayEntryAtTimeIncremental(id, player->timestamps_[i].first);
                ObjectStateStructDat* state = &entry.state;

                if (!state->info.active)
                {
                    continue;
                }

                // Output all entries with comma separated values
                snprintf(line,
                         MAX_LINE_LEN,
                         "%.3f, %d, %s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                         static_cast<double>(state->info.timeStamp),
                         state->info.id,
                         state->info.name,
                         static_cast<double>(state->pos.x),
                         static_cast<double>(state->pos.y),
                         static_cast<double>(state->pos.z),
                         static_cast<double>(state->pos.h),
                         static_cast<double>(state->pos.p),
                         static_cast<double>(state->pos.r),
                         static_cast<double>(state->info.speed),
                         static_cast<double>(state->info.wheel_angle),
                         static_cast<double>(state->info.wheel_rot));

                file << line;
            }
        }
    }
    else  // Fixed timesteps, we increment time with the fixed timestep
    {
        float start_time = player->timestamps_.begin()->first;
        float end_time   = static_cast<float>(player->GetStopTime());

        int steps = static_cast<int>((end_time - start_time) / fixed_timestep);  // Avoid floating point precision issues

        for (int i = 0; i <= steps; i++)
        {
            float time = start_time + i * fixed_timestep;
            for (const auto& [id, _] : player->objects_timeline_)
            {
                auto                  entry = player->GetReplayEntryAtTimeIncremental(id, time);
                ObjectStateStructDat* state = &entry.state;

                if (!state->info.active)
                {
                    continue;
                }

                // Output all entries with comma separated values
                snprintf(line,
                         MAX_LINE_LEN,
                         "%.3f, %d, %s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                         static_cast<double>(state->info.timeStamp),
                         state->info.id,
                         state->info.name,
                         static_cast<double>(state->pos.x),
                         static_cast<double>(state->pos.y),
                         static_cast<double>(state->pos.z),
                         static_cast<double>(state->pos.h),
                         static_cast<double>(state->pos.p),
                         static_cast<double>(state->pos.r),
                         static_cast<double>(state->info.speed),
                         static_cast<double>(state->info.wheel_angle),
                         static_cast<double>(state->info.wheel_rot));

                file << line;
            }

            time += fixed_timestep;
        }
    }

    file.close();

    delete player;
}