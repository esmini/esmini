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

#include "CommonMini.hpp"
#include "OSCParameterDistribution.hpp"
#include "logger.hpp"
#include <iomanip>
#include <sstream>

using namespace scenarioengine;

OSCParameterDistribution& OSCParameterDistribution::Inst()
{
    static OSCParameterDistribution instance_;
    return instance_;
}

OSCParameterDistribution::~OSCParameterDistribution()
{
    Reset();
}

int OSCParameterDistribution::Load(std::string filename)
{
    std::vector<std::string> file_name_candidates;

    file_name_candidates.push_back(filename);

    // Check registered paths
    for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
    {
        file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], filename));
    }

    pugi::xml_parse_result result;
    size_t                 i;
    for (i = 0; i < file_name_candidates.size(); i++)
    {
        if (FileExists(file_name_candidates[i].c_str()))
        {
            result = doc_.load_file(file_name_candidates[i].c_str());
            if (!result)
            {
                LOG_INFO("{}: {} at offset (character position): {}", file_name_candidates[i], result.description(), result.offset);
                return -1;
            }
            else
            {
                break;
            }
        }
    }

    if (i == file_name_candidates.size())
    {
        LOG_ERROR("Failed to load parameter distribution file {}. Tried:", filename);
        for (unsigned int j = 0; j < file_name_candidates.size(); j++)
        {
            LOG_INFO("   {}\n", file_name_candidates[j]);
        }
        LOG_WARN("continue without road description\n");
    }
    else
    {
        filename_ = file_name_candidates[i];
        LOG_INFO("Loaded {}", filename_);
    }

    // Parse
    pugi::xml_node node = doc_.child("OpenSCENARIO");
    if (!node)
    {
        // Another try
        node = doc_.child("OpenScenario");
    }

    if (!node)
    {
        throw std::runtime_error("Couldn't find OpenSCENARIO or OpenScenario element in - check XML!");
    }

    node = node.child("ParameterValueDistribution");
    if (node.empty())
    {
        LOG_WARN("No or empty ParameterValueDistribution element");
    }

    pugi::xml_node scenario_filename_node = node.child("ScenarioFile");
    if (scenario_filename_node.empty())
    {
        LOG_ERROR("ScenarioFile missing");
    }
    else
    {
        scenario_filename_ = scenario_filename_node.attribute("filepath").value();
        if (scenario_filename_.empty())
        {
            LOG_ERROR("Scenario filepath attribute missing");
        }
    }

    pugi::xml_node deterministic = node.child("Deterministic");
    if (!deterministic.empty())
    {
        for (pugi::xml_node child_node = deterministic.first_child(); child_node; child_node = child_node.next_sibling())
        {
            std::string child_node_name(child_node.name());

            if (child_node_name == "DeterministicMultiParameterDistribution")
            {
                pugi::xml_node value_set_dist = child_node.child("ValueSetDistribution");
                if (value_set_dist.empty())
                {
                    LOG_ERROR("ValueSetDistribution missing");
                    return -1;
                }

                std::vector<std::vector<ParameterValueEntry>> p_list;
                for (pugi::xml_node value_set = value_set_dist.child("ParameterValueSet"); value_set;
                     value_set                = value_set.next_sibling("ParameterValueSet"))
                {
                    std::vector<ParameterValueEntry> p_value_list;
                    for (pugi::xml_node param_assign = value_set.child("ParameterAssignment"); param_assign;
                         param_assign                = param_assign.next_sibling("ParameterAssignment"))
                    {
                        ParameterValueEntry entry = {param_assign.attribute("parameterRef").value(), param_assign.attribute("value").value()};
                        p_value_list.push_back(entry);
                    }
                    p_list.push_back(p_value_list);
                }
                param_list_.push_back(p_list);
            }
            else if (child_node_name == "DeterministicSingleParameterDistribution")
            {
                std::string param_name = child_node.attribute("parameterName").value();
                if (param_name.empty() || param_name == "")
                {
                    LOG_ERROR("Missing single distribution parameter name");
                    return -1;
                }

                pugi::xml_node dist = child_node.first_child();
                if (dist.empty())
                {
                    LOG_ERROR("Missing single distribution definition");
                    return -1;
                }

                if (!strcmp(dist.name(), "DistributionSet"))
                {
                    std::vector<std::vector<ParameterValueEntry>> p_list;
                    ParameterValueEntry                           entry;
                    entry.name = param_name;

                    for (pugi::xml_node elem = dist.child("Element"); elem; elem = elem.next_sibling("Element"))
                    {
                        entry.value                                   = elem.attribute("value").value();
                        std::vector<ParameterValueEntry> p_value_list = {entry};
                        p_list.push_back(p_value_list);
                    }
                    param_list_.push_back(p_list);
                }
                else if (!strcmp(dist.name(), "DistributionRange"))
                {
                    std::vector<std::vector<ParameterValueEntry>> p_list;
                    ParameterValueEntry                           entry;
                    entry.name = param_name;

                    double step_width = std::atof(dist.attribute("stepWidth").value());

                    pugi::xml_node range = dist.child("Range");
                    if (range.empty())
                    {
                        LOG_ERROR("Distribution range missing");
                        return -1;
                    }

                    double lower_limit = std::atof(range.attribute("lowerLimit").value());
                    double upper_limit = std::atof(range.attribute("upperLimit").value());

                    if (upper_limit < lower_limit)
                    {
                        LOG_ERROR("Distribution range invalid range {:.2f}..{:.2f}", lower_limit, upper_limit);
                        return -1;
                    }
                    for (double v = lower_limit; v < upper_limit + SMALL_NUMBER; v += step_width)
                    {
                        entry.value                                   = std::to_string(v);
                        std::vector<ParameterValueEntry> p_value_list = {entry};
                        p_list.push_back(p_value_list);
                    }
                    param_list_.push_back(p_list);
                }
                else if (!strcmp(dist.name(), "UserDefinedDistribution"))
                {
                    LOG_ERROR("UserDefinedDistribution not yet supported");
                    return -1;
                }
                else
                {
                    LOG_ERROR("Unexpected distribution definition");
                    return -1;
                }
            }
            else
            {
                LOG_ERROR("Unexpected deterministic distribution type {}", child_node_name);
            }
        }
    }

    pugi::xml_node stochastic = node.child("Stochastic");
    if (!stochastic.empty())
    {
        if (!deterministic.empty())
        {
            LOG_WARN("Found BOTH Deterministic and Stochastic distribution elements. Only one expected. Using Deterministic.");
        }
        else
        {
            LOG_WARN("Stochastic distributions not supported yet");
        }
    }
    else if (deterministic.empty())
    {
        LOG_ERROR("No distribution defined. Expected Determinstic or Stochastic");
    }

    return 0;
}

unsigned int OSCParameterDistribution::GetNumPermutations()
{
    unsigned int n = 1;

    if (param_list_.size() == 0)
    {
        return 0;
    }

    for (size_t i = 0; i < param_list_.size(); i++)
    {
        n *= static_cast<unsigned int>(param_list_[i].size());
    }

    return n;
}

unsigned int OSCParameterDistribution::GetNumParameters()
{
    if (index_ < 0)
    {
        return 0;
    }

    // Calculate number of parameters in current permutation
    unsigned int num_parameters = 0;

    unsigned int n = static_cast<unsigned int>(index_);

    for (unsigned int i = static_cast<unsigned int>(param_list_.size()); i > 0; i--)
    {
        unsigned int q = n / static_cast<unsigned int>(param_list_[i - 1].size());
        unsigned int r = n % static_cast<unsigned int>(param_list_[i - 1].size());

        // add parameters from this set
        num_parameters += static_cast<unsigned int>(param_list_[i - 1][r].size());

        n = q;
    }

    return num_parameters;
}

OSCParameterDistribution::ParameterValueEntry OSCParameterDistribution::GetParameterEntry(unsigned int param_index)
{
    unsigned int n_parameters = GetNumParameters();

    if (param_index >= n_parameters)
    {
        return {"", ""};
    }
    else if (index_ >= static_cast<int>(GetNumPermutations()))
    {
        return {"", ""};
    }
    else if (GetNumPermutations() == 0 || index_ < 0)
    {
        return {"", ""};
    }

    // mirror internal index starting from end, since permutations are traversed that way
    // varying the latest variable first
    unsigned int p_idx         = n_parameters - param_index - 1;
    unsigned int param_counter = 0;
    unsigned int n             = static_cast<unsigned int>(index_);

    for (unsigned int i = static_cast<unsigned int>(param_list_.size()); i > 0; i--)
    {
        unsigned int q = n / static_cast<unsigned int>(param_list_[i - 1].size());
        unsigned int r = n % static_cast<unsigned int>(param_list_[i - 1].size());

        // add parameters from this set
        param_counter += static_cast<unsigned int>(param_list_[i - 1][r].size());

        if (p_idx < param_counter)
        {
            return param_list_[i - 1][r][param_counter - p_idx - 1];
        }

        n = q;
    }

    return {"", ""};
}

std::string OSCParameterDistribution::GetParamName(unsigned int param_index)
{
    return GetParameterEntry(param_index).name;
}

std::string OSCParameterDistribution::GetParamValue(unsigned int param_index)
{
    return GetParameterEntry(param_index).value;
}

std::string OSCParameterDistribution::AddInfoToFilename(std::string filename)
{
    std::string base_name = FileNameWithoutExtOf(filename);
    std::string ext       = FileNameExtOf(filename);

#if 1  // no leading zeros
    return FileNameWithoutExtOf(base_name) + "_" + std::to_string(GetIndex() + 1) + "_of_" + std::to_string(GetNumPermutations()) + ext;
#else  // leading zeros
    int number     = GetNumPermutations();
    int num_digits = 0;
    while (number != 0)
    {
        number /= 10;
        num_digits++;
    }
    std::ostringstream str;
    str << FileNameWithoutExtOf(base_name) << "_" << std::setw(num_digits) << std::setfill('0') << GetIndex() + 1 << "_of_"
        << std::to_string(GetNumPermutations()) << ext;
    return str.str();
#endif
}

std::string OSCParameterDistribution::AddInfoToFilepath(std::string filepath)
{
    std::string base_path = FilePathWithoutExtOf(filepath);
    std::string ext       = FileNameExtOf(filepath);

#if 1  // no leading zeros
    return base_path + "_" + std::to_string(GetIndex() + 1) + "_of_" + std::to_string(GetNumPermutations()) + ext;
#else  // leading zeros
    int number     = GetNumPermutations();
    int num_digits = 0;
    while (number != 0)
    {
        number /= 10;
        num_digits++;
    }
    std::ostringstream str;
    str << FileNameWithoutExtOf(base_name) << "_" << std::setw(num_digits) << std::setfill('0') << GetIndex() + 1 << "_of_"
        << std::to_string(GetNumPermutations()) << ext;
    return str.str();
#endif
}

void OSCParameterDistribution::Reset()
{
    for (size_t i = 0; i < param_list_.size(); i++)
    {
        for (size_t j = 0; j < param_list_[i].size(); j++)
        {
            param_list_[i][j].clear();
        }
        param_list_[i].clear();
    }
    param_list_.clear();
    filename_.clear();
    scenario_filename_.clear();
    index_           = -1;
    requested_index_ = 0;  // first permutation
}

int OSCParameterDistribution::SetIndex(unsigned int index)
{
    if (index >= GetNumPermutations())
    {
        LOG_ERROR("Permutation index {} out of range (0..{})", index, GetNumPermutations() - 1);
        return -1;
    }

    index_           = static_cast<int>(index);
    requested_index_ = -1;  // set directly, no request

    return 0;
}

int OSCParameterDistribution::IncrementIndex()
{
    if (index_ < -1 || index_ >= static_cast<int>(GetNumPermutations() - 1))
    {
        LOG_ERROR("Can't increment index {}, would end up out of range (0..{})\n", index_, GetNumPermutations() - 1);
        return -1;
    }

    index_++;
    requested_index_ = -1;  // indicate calculated, not requested

    return index_;
}

int OSCParameterDistribution::SetRequestedIndex(unsigned int index)
{
    if (index >= GetNumPermutations())
    {
        LOG_ERROR("requested permutation index {} out of range (0..{})", index, GetNumPermutations() - 1);
        return 0;
    }

    requested_index_ = static_cast<int>(index);

    return 0;
}