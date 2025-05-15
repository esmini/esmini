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

#include <iostream>
#include <string>
#include <vector>
#include "pugixml.hpp"

namespace scenarioengine
{
    class OSCParameterDistribution
    {
        struct ParameterValueEntry
        {
            std::string name;
            std::string value;
        };

        std::vector<std::vector<std::vector<ParameterValueEntry>>> param_list_;
        std::string                                                filename_;
        std::string                                                scenario_filename_;
        int                                                        index_;
        int                                                        requested_index_;
        pugi::xml_document                                         doc_;

    public:
        OSCParameterDistribution()
        {
            Reset();
        }
        ~OSCParameterDistribution();
        static OSCParameterDistribution& Inst();

        int          Load(std::string filename);
        unsigned int GetNumPermutations();
        unsigned int GetNumParameters();
        void         Reset();
        std::string  GetFilename() const
        {
            return filename_;
        }
        std::string GetScenarioFileName() const;
        // Returns current permutation index
        int GetIndex() const
        {
            return index_;
        }
        int SetIndex(unsigned int index);
        int SetRequestedIndex(unsigned int index);
        int GetRequestedIndex() const
        {
            return requested_index_;
        }
        int                 IncrementIndex();
        ParameterValueEntry GetParameterEntry(unsigned int param_index);
        std::string         GetParamName(unsigned int param_index);
        std::string         GetParamValue(unsigned int param_index);
        std::string         AddInfoToFilename(std::string filename);
        std::string         AddInfoToFilepath(std::string filepath);
        bool                IsParamDist = true;
    };
}  // namespace scenarioengine
