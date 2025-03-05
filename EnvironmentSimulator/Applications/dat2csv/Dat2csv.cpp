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

    file_.open(filename_);
    if (!file_.is_open())
    {
        LOG_ERROR_AND_QUIT("Failed to create file {}\n", filename_);
    }

    // Create replayer object for parsing the binary data file
    player_ = std::make_unique<scenarioengine::Replay>(filename);
}

Dat2csv::~Dat2csv()
{
}

void Dat2csv::PrintData(int obj_id)
{
    char        line[MAX_LINE_LEN];
    std::string name;
    player_->GetName(obj_id, name);
    snprintf(line,
             MAX_LINE_LEN,
             "%.3f, %d, %s, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
             player_->scenarioState_.sim_time,
             obj_id,
             name.c_str(),
             player_->GetX(obj_id),
             player_->GetY(obj_id),
             player_->GetZ(obj_id),
             player_->GetH(obj_id),
             player_->GetP(obj_id),
             player_->GetR(obj_id),
             player_->GetSpeed(obj_id),
             player_->GetWheelAngle(obj_id),
             player_->GetWheelRot(obj_id));
    file_ << line;
    if (extended_)
    {
        snprintf(line,
                 MAX_LINE_LEN,
                 ", %.d, %.d, %.3f, %.3f, %.3f, ",
                 player_->GetRoadId(obj_id),
                 player_->GetLaneId(obj_id),
                 player_->GetPosOffset(obj_id),
                 player_->GetPosT(obj_id),
                 player_->GetPosS(obj_id));

        file_ << line;
        datLogger::LightState lightState_;
        player_->GetLightStates(obj_id, lightState_);
        const size_t numLights = sizeof(lightState_) / sizeof(datLogger::LightRGB);
        std::string  light_state_string;
        for (size_t k = 0; k < numLights; ++k)
        {
            datLogger::LightRGB* light = reinterpret_cast<datLogger::LightRGB*>(&lightState_) + k;
            // Format each color component using std::stringstream:
            snprintf(line, MAX_LINE_LEN, "#%.02X%.02X%.02X-%.02X, ", light->red, light->green, light->blue, light->intensity);
            file_ << line;
        }
    }
    snprintf(line, MAX_LINE_LEN, "\n");
    file_ << line;
}

void Dat2csv::PrintRow()
{
    char line[MAX_LINE_LEN];
    snprintf(line, MAX_LINE_LEN, "time, id, name, x, y, z, h, p, r, speed, wheel_angle, wheel_rot");
    file_ << line;
    if (extended_)
    {
        snprintf(
            line,
            MAX_LINE_LEN,
            ", roadId, laneId, offset, t, s, day_light, low_beam, high_beam, fog_light_front, fog_light_rear, brake_light, ind_left, ind_right, reversing_light, license_plate, special_pur_light, fog_light, warning_light");
        file_ << line;
    }
    snprintf(line, MAX_LINE_LEN, "\n");
    file_ << line;
}

void Dat2csv::PrintHeader()
{
    char line[MAX_LINE_LEN];
    if (include_refs_)
    {
        // First output header and CSV labels
        snprintf(line,
                 MAX_LINE_LEN,
                 "Version: %d, OpenDRIVE: %s, 3DModel: %s\n",
                 player_->GetHeader().version,
                 player_->GetHeader().odrFilename.string.data(),
                 player_->GetHeader().modelFilename.string.data());
        file_ << line;
    }
}

void Dat2csv::CreateCSVOriginal()
{
    unsigned int index = 0;
    for (auto pkg : player_->pkgs_)
    {
        if (pkg.hdr.id == static_cast<int>(datLogger::PackageId::TIME_SERIES))
        {
            double timeTemp = *reinterpret_cast<double*>(pkg.content.data());

            // next time
            player_->SetTime(timeTemp);
            player_->SetIndex(index);

            player_->CheckObjAvailabilityForward();
            player_->UpdateCache();
            player_->scenarioState_.sim_time = timeTemp;
            for (const auto obj : player_->scenarioState_.obj_states)
            {
                if (obj.active)
                {
                    PrintData(obj.id);
                }
            }
        }
        index++;
    }
}

double Dat2csv::GetReplayerStepTime()
{
    if (log_mode_ == log_mode::MIN_STEP || log_mode_ == log_mode::MIN_STEP_MIXED)
    {
        return player_->deltaTime_;
    }
    else
    {
        player_->deltaTime_ = step_time_;  // make sure move to forward function takes correct delta time
        return step_time_;
    }
}

void Dat2csv::SetLogExtended(bool option)
{
    extended_ = option;
}

void Dat2csv::SetIncludeRefs(bool option)
{
    include_refs_ = option;
}

void Dat2csv::SetLogMode(Dat2csv::log_mode mode_)
{
    log_mode_ = mode_;
}

void Dat2csv::SetStepTime(double t)
{
    step_time_ = t;
}

void Dat2csv::CreateCSV()
{
    PrintHeader();
    PrintRow();
    if (log_mode_ == log_mode::ORIGINAL)
    {  // default setting, write time stamps available only in dat file
        CreateCSVOriginal();
    }
    else
    {
        double requestedTime      = SMALL_NUMBER;
        double replayer_step_time = GetReplayerStepTime();

        while (true)
        {
            perviousSimTime_ = player_->GetTime();
            for (const auto obj : player_->scenarioState_.obj_states)
            {
                if (obj.active)
                {
                    PrintData(obj.id);
                }
            }

            if (player_->GetTime() > player_->GetStopTime() - SMALL_NUMBER)
            {
                break;  // reached end of file
            }
            else if (replayer_step_time < SMALL_NUMBER)
            {
                LOG_WARN("Warning: Unexpected delta time zero found! Can't process remaining part of the file");
                break;
            }
            else
            {
                if (log_mode_ == log_mode::MIN_STEP || log_mode_ == log_mode::CUSTOM_TIME_STEP)
                {
                    double dt = player_->GetTime() + replayer_step_time;
                    if (std::fabs(dt) < SMALL_NUMBER)
                    {
                        // If so, return 1E-6
                        dt = SMALL_NUMBER;
                    }
                    player_->GoToTime(dt);  // continue
                }
                else
                {
                    if ((fabs(player_->GetTime() - requestedTime) < SMALL_NUMBER) || IsEqualDouble(player_->GetTime(), player_->GetStartTime()))
                    {  // first time frame or until reach requested time frame reached, dont move to next time frame
                        requestedTime = player_->GetTime() + replayer_step_time;

                        player_->GoToTime(player_->GetTime() + replayer_step_time, true);  // continue
                        if (perviousSimTime_ > player_->GetTime())
                        {
                            requestedTime = player_->GetTime();  // restarted, change the requested time accordingly
                        }
                    }
                    else
                    {
                        player_->GoToTime(requestedTime, true);  // continue
                    }
                }
            }
        }
    }

    file_.close();
}