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
#include <iomanip>

#include "Dat2csv.hpp"
#include "CommonMini.hpp"
#include "DatLogger.hpp"
#include "Replay.hpp"

#define MAX_LINE_LEN 2048

Dat2csv::Dat2csv(std::string filename) : log_mode_(log_mode::ORIGINAL), step_time_(0.05)
{
    std::string filename_ = FileNameWithoutExtOf(filename) + ".csv";

    file.open(filename_);
    if (!file.is_open())
    {
        LOG_ERROR_AND_QUIT("Failed to create file {}\n", filename_);
    }

    // Create replayer object for parsing the binary data file
    try
    {
        player = std::make_unique<scenarioengine::Replay>(filename);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR_AND_QUIT("{}", e.what());
    }
}

Dat2csv::~Dat2csv()
{
}

void Dat2csv::PrintData(size_t i)
{
    static char line[MAX_LINE_LEN];
    int         obj_id = player->scenarioState.obj_states[i].id;
    std::string name;
    player->GetName(obj_id, name);
    if (!extended)
    {
        snprintf(line,
                 MAX_LINE_LEN,
                 "%.3f, %d, %s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                 player->scenarioState.sim_time,
                 obj_id,
                 name.c_str(),
                 player->GetX(obj_id),
                 player->GetY(obj_id),
                 player->GetZ(obj_id),
                 player->GetH(obj_id),
                 player->GetP(obj_id),
                 player->GetR(obj_id),
                 player->GetSpeed(obj_id),
                 player->GetWheelAngle(obj_id),
                 player->GetWheelRot(obj_id));
        file << line;
    }
    else
    {
        snprintf(line,
                 MAX_LINE_LEN,
                 "%.3f, %d, %s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, ",
                 player->scenarioState.sim_time,
                 obj_id,
                 name.c_str(),
                 player->GetX(obj_id),
                 player->GetY(obj_id),
                 player->GetZ(obj_id),
                 player->GetH(obj_id),
                 player->GetP(obj_id),
                 player->GetR(obj_id),
                 player->GetRoadId(obj_id),
                 player->GetLaneId(obj_id),
                 player->GetPosOffset(obj_id),
                 static_cast<double>(player->GetPosT(obj_id)),
                 static_cast<double>(player->GetPosS(obj_id)),
                 player->GetSpeed(obj_id),
                 player->GetWheelAngle(obj_id),
                 player->GetWheelRot(obj_id));
        file << line;
        datLogger::LightState lightState_;
        player->GetLightStates(obj_id, lightState_);
        const size_t numLights = sizeof(lightState_) / sizeof(datLogger::LightRGB);
        std::string  light_state_string;
        for (size_t k = 0; k < numLights; ++k)
        {
            datLogger::LightRGB* light = reinterpret_cast<datLogger::LightRGB*>(&lightState_) + k;
            // Format each color component using std::stringstream:
            snprintf(line, MAX_LINE_LEN, "#%.02X%.02X%.02X-%.02X, ", light->red, light->green, light->blue, light->intensity);
            file << line;
        }
        snprintf(line, MAX_LINE_LEN, "\n");
        file << line;
    }
}

void Dat2csv::CreateCSV()
{
    static char line[MAX_LINE_LEN];
    if (include_refs)
    {
        // First output header and CSV labels
        snprintf(line,
                 MAX_LINE_LEN,
                 "Version: %d, OpenDRIVE: %s, 3DModel: %s\n",
                 player->header_.version,
                 player->header_.odrFilename.string.data(),
                 player->header_.modelFilename.string.data());
        file << line;
    }
    if (!extended)
    {
        snprintf(line, MAX_LINE_LEN, "time, id, name, x, y, z, h, p, r, speed, wheel_angle, wheel_rot\n");
        file << line;
    }
    else
    {
        snprintf(
            line,
            MAX_LINE_LEN,
            "time, id, name, x, y, z, h, p, r, roadId, laneId, offset, t, s, speed, wheel_angle, wheel_rot, day_light, low_beam, high_beam, fog_light_front, fog_light_rear, brake_light, ind_left, ind_right, reversing_light, license_plate, special_pur_light, fog_light, warning_light\n");
        file << line;
    }
    if (log_mode_ == log_mode::MIN_STEP || log_mode_ == log_mode::MIN_STEP_MIXED || log_mode_ == log_mode::CUSTOM_TIME_STEP ||
        log_mode_ == log_mode::CUSTOM_TIME_STEP_MIXED)
    {
        double requestedTime = SMALL_NUMBER;
        double delta_time    = SMALL_NUMBER;
        if (log_mode_ == log_mode::MIN_STEP || log_mode_ == log_mode::MIN_STEP_MIXED)
        {
            delta_time = player->deltaTime_;
        }
        else
        {
            delta_time = step_time_;
        }
        while (true)
        {
            for (size_t i = 0; i < player->scenarioState.obj_states.size(); i++)
            {
                if (player->scenarioState.obj_states[i].active)
                {
                    PrintData(i);
                }
            }

            if (player->GetTime() > player->GetStopTime() - SMALL_NUMBER)
            {
                break;  // reached end of file
            }
            else if (delta_time < SMALL_NUMBER)
            {
                LOG_WARN("Warning: Unexpected delta time zero found! Can't process remaining part of the file");
                break;
            }
            else
            {
                if (log_mode_ == log_mode::MIN_STEP || log_mode_ == log_mode::CUSTOM_TIME_STEP)
                {
                    player->MoveToTime(player->GetTime() + delta_time);  // continue
                }
                else
                {
                    if (isEqualDouble(player->GetTime(), requestedTime) || isEqualDouble(player->GetTime(), player->GetStartTime()))
                    {  // first time frame or until reach requested time frame reached, dont move to next time frame
                        requestedTime = player->GetTime() + delta_time;
                        player->MoveToTime(player->GetTime() + delta_time, true);  // continue
                    }
                    else
                    {
                        player->MoveToTime(requestedTime, true);  // continue
                    }
                }
            }
        }
    }
    else if (log_mode_ == log_mode::ORIGINAL)
    {  // default setting, write time stamps available only in dat file
        for (size_t j = 0; j < player->pkgs_.size(); j++)
        {
            if (player->pkgs_[j].hdr.id == static_cast<int>(datLogger::PackageId::TIME_SERIES))
            {
                double timeTemp = *reinterpret_cast<double*>(player->pkgs_[j].content.data());

                // next time
                player->SetTime(timeTemp);
                player->SetIndex(static_cast<int>(j));

                player->CheckObjAvailabilityForward();
                player->UpdateCache();
                player->scenarioState.sim_time = timeTemp;
                for (size_t i = 0; i < player->scenarioState.obj_states.size(); i++)
                {
                    if (player->scenarioState.obj_states[i].active)
                    {
                        PrintData(i);
                    }
                }
            }
        }
    }
    file.close();
}