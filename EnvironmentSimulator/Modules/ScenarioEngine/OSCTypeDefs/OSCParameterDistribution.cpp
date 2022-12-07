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
	size_t i;
	for (i = 0; i < file_name_candidates.size(); i++)
	{
		if (FileExists(file_name_candidates[i].c_str()))
		{
			result = doc_.load_file(file_name_candidates[i].c_str());
			if (!result)
			{
				LOG("%s: %s at offset (character position): %d", file_name_candidates[i].c_str(), result.description(), result.offset);
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
		LOG("Failed to load parameter distribution file %s. Tried:", filename.c_str());
		for (unsigned int j = 0; j < file_name_candidates.size(); j++)
		{
			LOG("   %s\n", file_name_candidates[j].c_str());
		}
		LOG("continue without road description\n");
	}
	else
	{
		filename_ = file_name_candidates[i];
		LOG("Loaded %s", filename_.c_str());
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
		LOG("No or empty ParameterValueDistribution element");
	}

	pugi::xml_node scenario_filename_node = node.child("ScenarioFile");
	if (scenario_filename_node.empty())
	{
		LOG("ScenarioFile missing");
	}
	else
	{
		scenario_filename_ = scenario_filename_node.attribute("filepath").value();
		if (scenario_filename_.empty())
		{
			LOG("Scenario filepath attribute missing");
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
					LOG("ValueSetDistribution missing");
					return -1;
				}

				for (pugi::xml_node value_set = value_set_dist.child("ParameterValueSet"); value_set; value_set = value_set.next_sibling("ParameterValueSet"))
				{
					for (pugi::xml_node param_assign = value_set.child("ParameterAssignment"); param_assign; param_assign = param_assign.next_sibling("ParameterAssignment"))
					{
						ParameterValueList plist;
						plist.name = param_assign.attribute("parameterRef").value();
						plist.value.push_back(param_assign.attribute("value").value());
						param_list_.push_back(plist);
					}
				}
			}
			else if (child_node_name == "DeterministicSingleParameterDistribution")
			{
				std::string param_name = child_node.attribute("parameterName").value();
				if (param_name.empty() || param_name == "")
				{
					LOG("Missing single distribution parameter name");
					return -1;
				}

				pugi::xml_node dist = child_node.first_child();
				if (dist.empty())
				{
					LOG("Missing single distribution definition");
					return -1;
				}

				if (!strcmp(dist.name(), "DistributionSet"))
				{
					ParameterValueList plist;
					plist.name = param_name;

					for (pugi::xml_node elem = dist.child("Element"); elem; elem = elem.next_sibling("Element"))
					{
						plist.value.push_back(elem.attribute("value").value());
					}
					param_list_.push_back(plist);
				}
				else if (!strcmp(dist.name(), "DistributionRange"))
				{
					ParameterValueList plist;
					plist.name = param_name;

					double step_width = std::atof(dist.attribute("stepWidth").value());

					pugi::xml_node range = dist.child("Range");
					if (range.empty())
					{
						LOG("Distribution range missing");
						return -1;
					}

					double lower_limit = std::atof(range.attribute("lowerLimit").value());
					double upper_limit = std::atof(range.attribute("upperLimit").value());

					if (upper_limit < lower_limit)
					{
						LOG("Distribution range invalid range %.2f..%.2f", lower_limit, upper_limit);
						return -1;
					}
					for (double v = lower_limit; v < upper_limit + SMALL_NUMBER; v += step_width)
					{
						plist.value.push_back(std::to_string(v));
					}
					param_list_.push_back(plist);
				}
				else if (!strcmp(dist.name(), "UserDefinedDistribution"))
				{
					LOG("UserDefinedDistribution not yet supported");
					return -1;
				}
				else
				{
					LOG("Unexpected distribution definition");
					return -1;
				}
			}
			else
			{
				LOG("Unexpected deterministic distribution type %s", child_node_name.c_str());
			}
		}
	}

	pugi::xml_node stochastic = node.child("Stochastic");
	if (!stochastic.empty())
	{
		if (!deterministic.empty())
		{
			LOG("Found BOTH Deterministic and Stochastic distribution elements. Only one expected. Using Deterministic.");
		}
		else
		{
			LOG("Stochastic distributions not supported yet");
		}
	}
	else if (deterministic.empty())
	{
		LOG("No distribution defined. Expected Determinstic or Stochastic");
	}

	return 0;
}

int OSCParameterDistribution::GetNumParameters()
{
	return static_cast<int>(param_list_.size());
}

std::string OSCParameterDistribution::GetParamName(int index)
{
	if (index >= 0 && static_cast<unsigned int>(index) < param_list_.size())
	{
		return param_list_[static_cast<unsigned int>(index)].name;
	}

	return "";
}

int OSCParameterDistribution::GetNumPermutations()
{
	int n = 1;

	if (param_list_.size() == 0)
	{
		return 0;
	}

	for (size_t i = 0; i < param_list_.size(); i++)
	{
		n *= static_cast<int>(param_list_[i].value.size());
	}

	return n;
}

std::string OSCParameterDistribution::GetParamValue(int param_index)
{
	if (param_index < 0 || static_cast<unsigned int>(param_index) >= param_list_.size())
	{
		return "";
	}
	else if (index_ < 0 || index_ >= GetNumPermutations())
	{
		return "";
	}
	else if (GetNumPermutations() == 0)
	{
		return "";
	}

	int n = index_;

	for (int i = static_cast<int>(param_list_.size()) - 1; i >= 0; i--)
	{
		int q = n / static_cast<int>(param_list_[static_cast<unsigned int>(i)].value.size());
		int r = n % static_cast<int>(param_list_[static_cast<unsigned int>(i)].value.size());

		if (i == param_index)
		{
			return param_list_[static_cast<unsigned int>(param_index)].value[static_cast<unsigned int>(r)];
		}
		n = q;
	}

	return "";
}

std::string OSCParameterDistribution::AddInfoToFilename(std::string filename)
{
	std::string base_name = FileNameWithoutExtOf(filename);
	std::string ext = FileNameExtOf(filename);

#if 1  // no leading zeros
	return FileNameWithoutExtOf(base_name) + "_" + std::to_string(GetIndex() + 1) + "_of_" +
		std::to_string(GetNumPermutations()) + ext;
#else  // leading zeros
	int number = GetNumPermutations();
	int num_digits = 0; while (number != 0) { number /= 10; num_digits++; }
	std::ostringstream str;
	str << FileNameWithoutExtOf(base_name) << "_" <<
		std::setw(num_digits) << std::setfill('0') << GetIndex() + 1 << "_of_" <<
		std::to_string(GetNumPermutations()) << ext;
	return str.str();
#endif
}

void OSCParameterDistribution::Reset()
{
	for (size_t i = 0; i < param_list_.size(); i++)
	{
		param_list_[i].value.clear();
	}
	param_list_.clear();
	filename_.clear();
	scenario_filename_.clear();
	index_ = -1;
	requested_index_ = 0;  // first permutation
}

int OSCParameterDistribution::SetIndex(int index)
{
	if (index < 0 || index >= GetNumPermutations())
	{
		LOG("Permutation index %d out of range (0..%d)", index, GetNumPermutations() - 1);
		return -1;
	}

	index_ = index;
	requested_index_ = -1; // set directly, no request

	return 0;
}

int OSCParameterDistribution::IncrementIndex()
{
	if (index_ < -1 || index_ >= GetNumPermutations() - 1)
	{
		LOG("Can't increment index %d, would end up out of range (0..%d)\n", index_, GetNumPermutations() - 1);
		return -1;
	}

	index_++;
	requested_index_ = -1;  // indicate calculated, not requested

	return index_;
}

int OSCParameterDistribution::SetRequestedIndex(int index)
{
	if (index < 0 || index >= GetNumPermutations())
	{
		LOG("requested permutation index %d out of range (0..%d)", index, GetNumPermutations() - 1);
		return 0;
	}

	requested_index_ = index;

	return 0;
}