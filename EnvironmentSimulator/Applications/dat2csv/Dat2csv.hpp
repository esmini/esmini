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

#pragma once

#include <clocale>

#include <string>
#include <fstream>

#include "Replay.hpp"

class Dat2csv
{
public:
    Dat2csv(std::string filename);
    ~Dat2csv();

    enum class log_mode
    {
        ORIGINAL               = 0,  // default
        MIN_STEP               = 1,
        MIN_STEP_MIXED         = 2,
        CUSTOM_TIME_STEP       = 3,  // 0.05 default step time.
        CUSTOM_TIME_STEP_MIXED = 4   // 0.05 default step time

    };

    void CreateCSV();
    void PrintData(size_t index);

    void SetLogExtended(bool option)
    {
        extended = option;
    }

    void SetIncludeRefs(bool option)
    {
        include_refs = option;
    }

    void SetLogMode(Dat2csv::log_mode mode_)
    {
        log_mode_ = mode_;
    }

    void SetStepTime(double t)
    {
        step_time_ = t;
    }

private:
    Dat2csv::log_mode                       log_mode_;
    double                                  step_time_;
    bool                                    extended     = false;
    bool                                    include_refs = false;
    std::ofstream                           file_;
    std::unique_ptr<scenarioengine::Replay> player_;
};