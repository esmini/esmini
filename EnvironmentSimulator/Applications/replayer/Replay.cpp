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

#include "Replay.hpp"
#include "ScenarioGateway.hpp"
#include "CommonMini.hpp"
#include "dirent.h"

using namespace scenarioengine;


Replay::Replay(std::string filename) : time_(0.0), index_(0), repeat_(false)
{
	file_.open(filename, std::ofstream::binary);
	if (file_.fail())
	{
		LOG("Cannot open file: %s", filename.c_str());
		throw std::invalid_argument(std::string("Cannot open file: ") + filename);
	}

	file_.read((char*)&header_, sizeof(header_));
	LOG("Recording %s opened. dat version: %d odr: %s model: %s", FileNameOf(filename).c_str(), header_.version,
		FileNameOf(header_.odr_filename).c_str(), FileNameOf(header_.model_filename).c_str());

	if (header_.version != DAT_FILE_FORMAT_VERSION)
	{
		LOG_AND_QUIT("Version mismatch. %s is version %d while supported version is %d. Please re-create dat file.",
			filename.c_str(), header_.version, DAT_FILE_FORMAT_VERSION);
	}

	if (header_.version != DAT_FILE_FORMAT_VERSION)
	{
		LOG_AND_QUIT("Version mismatch. %s is version %d while supported version is %d. Please re-create dat file.",
			filename.c_str(), header_.version, DAT_FILE_FORMAT_VERSION);
	}

	while (!file_.eof())
	{
		ReplayEntry data;

		file_.read((char*)&data.state, sizeof(data.state));

		if (!file_.eof())
		{
			data_.push_back(data);
		}
	}

	CleanEntries(data_);

	if (data_.size() > 0)
	{
		// Register first entry timestamp as starting time
		time_ = data_[0].state.info.timeStamp;
		startTime_ = time_;
		startIndex_ = 0;

		// Register last entry timestamp as stop time
		stopTime_ = data_.back().state.info.timeStamp;
		stopIndex_ = FindIndexAtTimestamp(stopTime_);
	}
}

Replay::Replay(const std::string directory, const std::string scenario) : time_(0.0), index_(0), repeat_(false)
{
	GetReplaysFromDirectory(directory, scenario);
	std::vector<std::pair<std::string, std::vector<ReplayEntry>>> scenarioData;

	for (size_t i = 0; i < scenarios_.size(); i++)
	{
		file_.open(scenarios_[i], std::ofstream::binary);
		if (file_.fail())
		{
			LOG("Cannot open file: %s", scenarios_[i].c_str());
			throw std::invalid_argument(std::string("Cannot open file: ") + scenarios_[i]);
		}
		file_.read((char*)&header_, sizeof(header_));
		LOG("Recording %s opened. dat version: %d odr: %s model: %s", FileNameOf(scenarios_[i]).c_str(), header_.version,
			FileNameOf(header_.odr_filename).c_str(), FileNameOf(header_.model_filename).c_str());

		if (header_.version != DAT_FILE_FORMAT_VERSION)
		{
			LOG_AND_QUIT("Version mismatch. %s is version %d while supported version is %d. Please re-create dat file.",
				scenarios_[i].c_str(), header_.version, DAT_FILE_FORMAT_VERSION);
		}
		while (!file_.eof())
		{
			ReplayEntry entry;

			file_.read((char*)&entry.state, sizeof(entry.state));

			if (!file_.eof())
			{
				data_.push_back(entry);
			}
		}
		// pair <scenario name, scenario data>
		scenarioData.push_back(std::make_pair(scenarios_[i], data_));
		data_ = {};
		file_.close();
	}

	if (scenarioData.size() < 2)
	{
		LOG_AND_QUIT("Too few scenarios loaded, use single replay feature instead\n");
	}

	// Scenario with smallest start time first
	std::sort(scenarioData.begin(), scenarioData.end(), [](const auto& sce1, const auto& sce2)
	{
		return sce1.second[0].state.info.timeStamp < sce2.second[0].state.info.timeStamp;
	});

	// Log which scenario belongs to what ID-group (0, 100, 200 etc.)
	for (size_t i = 0; i < scenarioData.size(); i++)
	{
		std::string scenario_tmp = (scenarioData.begin()+i)->first;
		LOG("Scenarios corresponding to IDs (%d:%d): %s", i * 100, (i+1) * 100 - 1, FileNameOf(scenario_tmp.c_str()).c_str());
	}


	// Ensure increasing timestamps. Remove any other entries.
	for (auto& sce : scenarioData)
	{
		CleanEntries(sce.second);
	}

	// Build remaining data in order.
	BuildData(scenarioData);

	if (data_.size() > 0)
	{
		// Register first entry timestamp as starting time
		time_ = data_[0].state.info.timeStamp;
		startTime_ = time_;
		startIndex_ = 0;

		// Register last entry timestamp as stop time
		stopTime_ = data_.back().state.info.timeStamp;
		stopIndex_ = FindIndexAtTimestamp(stopTime_);
	}
}

// Browse through replay-folder and appends strings of absolute path to matching scenario
void Replay::GetReplaysFromDirectory(const std::string dir, const std::string sce)
{
	DIR* directory = opendir(dir.c_str());

	// If no directory found, write error
	if (directory == nullptr)
	{
		LOG_AND_QUIT("No valid directory given, couldn't open %s", dir.c_str());
	}

	// While directory is open, check the filename
	struct dirent* file;
	while ((file = readdir(directory)) != nullptr)
	{
		std::string filename = file->d_name;
		if (file->d_type == DT_DIR && filename.find(sce) != std::string::npos)
		{
			DIR* nested_dir = opendir((dir + filename).c_str());
			if (nested_dir == nullptr)
			{
				LOG("Couldn't open nested directory %s", (dir+filename).c_str());
			}

			struct dirent* nested_file;
			while ((nested_file = readdir(nested_dir)) != nullptr)
			{
				std::string nested_filename = nested_file->d_name;

				if (nested_filename != "." && nested_filename != ".." && nested_filename.find(sce) != std::string::npos && nested_filename.find(".dat") != std::string::npos)
				{
					scenarios_.emplace_back(CombineDirectoryPathAndFilepath(dir+filename, nested_filename));
				}
			}
			closedir(nested_dir);
		}

		if (filename != "." && filename != ".." && filename.find(sce) != std::string::npos && filename.find(".dat") != std::string::npos)
		{
			scenarios_.emplace_back(CombineDirectoryPathAndFilepath(dir, filename));
		}
	}
	closedir(directory);

	if (scenarios_.empty())
	{
		LOG_AND_QUIT("Couldn't read any scenarios named %s in path %s",sce.c_str(), dir.c_str());
	}
}

size_t Replay::GetNumberOfScenarios()
{
	return scenarios_.size();
}

Replay::~Replay()
{
	data_.clear();
}

void Replay::GoToStart()
{
	index_ = startIndex_;
	time_ = startTime_;
}

void Replay::GoToEnd()
{
	if (repeat_)
	{
		index_ = startIndex_;
		time_ = startTime_;
	}
	else
	{
		index_ = stopIndex_;
		time_ = stopTime_;
	}
}

void Replay::GoToTime(double time, bool stop_at_next_frame)
{
	if (!stop_at_next_frame)
	{
		if (time > stopTime_)
		{
			GoToEnd();
		}
		else if (time < GetStartTime())
		{
			GoToStart();
		}
		else
		{
			index_ = FindIndexAtTimestamp(time, index_);
			time_ = time;
		}
	}
	else
	{
		int next_index = index_;

		if (time > time_)
		{
			next_index = FindNextTimestamp();
			if (next_index > (int)index_ && time > data_[next_index].state.info.timeStamp && data_[next_index].state.info.timeStamp <= GetStopTime())
			{
				index_ = next_index;
				time_ = data_[index_].state.info.timeStamp;
			}
			else
			{
				if (time > GetStopTime())
				{
					GoToEnd();
				}
				else
				{
					time_ = time;
				}
			}
		}
		else if (time < time_)
		{
			next_index = FindPreviousTimestamp();
			if (next_index < (int)index_ && time < data_[next_index].state.info.timeStamp)
			{
				index_ = next_index;
				time_ = data_[index_].state.info.timeStamp;
			}
			else
			{
				if (time < GetStartTime())
				{
					GoToStart();
				}
				else
				{
					time_ = time;
				}
			}
		}
	}
}

void Replay::GoToDeltaTime(double dt, bool stop_at_next_frame)
{
	GoToTime(time_ + dt, stop_at_next_frame);
}

void Replay::GoToNextFrame()
{
	double ctime = data_[index_].state.info.timeStamp;
	for (size_t i = index_+1; i < data_.size(); i++)
	{
		if (data_[i].state.info.timeStamp > ctime)
		{
			GoToTime(data_[i].state.info.timeStamp);
			break;
		}
	}
}

void Replay::GoToPreviousFrame()
{
	if (index_ > 0)
	{
		GoToTime(data_[index_ -1].state.info.timeStamp);
	}
}

int Replay::FindIndexAtTimestamp(double timestamp, int startSearchIndex)
{
	int i = 0;

	if (timestamp > stopTime_)
	{
		GoToEnd();
		return index_;
	}
	else if (timestamp < GetStartTime())
	{
		return index_;
	}

	if (timestamp < time_)
	{
		// start search from beginning
		startSearchIndex = 0;
	}

	for (i = startSearchIndex; i < (int)data_.size(); i++)
	{
		if (data_[i].state.info.timeStamp >= timestamp)
		{
			break;
		}
	}

	return MIN(i, (int)data_.size() - 1);
}

int Replay::FindNextTimestamp(bool wrap)
{
	int index = index_ + 1;
	for (; index < data_.size(); index++)
	{
		if (data_[index].state.info.timeStamp > data_[index_].state.info.timeStamp)
		{
			break;
		}
	}

	if (index >= data_.size())
	{
		if (wrap)
		{
			return 0;
		}
		else
		{
			return index_;  // stay on current index
		}
	}

	return index;
}

int Replay::FindPreviousTimestamp(bool wrap)
{
	int index = index_ - 1;

	if (index < 0)
	{
		if (wrap)
		{
			index = (int)(data_.size() - 1);
		}
		else
		{
			return 0;
		}
	}

	for (int i = index - 1; i >= 0; i--)
	{
		// go backwards until we identify the first entry with same timestamp
		if (data_[i].state.info.timeStamp < data_[index].state.info.timeStamp)
		{
			break;
		}
		index = i;
	}

	return index;
}

ReplayEntry* Replay::GetEntry(int id)
{
	// Read all vehicles at current timestamp
	float timestamp = data_[index_].state.info.timeStamp;
	int i = 0;
	while (index_ + i < data_.size() && !(data_[index_ + i].state.info.timeStamp > timestamp))
	{
		if (data_[index_ + i].state.info.id == id)
		{
			return &data_[index_ + i];
		}
		i++;
	}

	return nullptr;
}

ObjectStateStructDat* Replay::GetState(int id)
{
	ReplayEntry* entry = GetEntry(id);
	if (entry != nullptr)
	{
		return &entry->state;
	}
	else
	{
		return nullptr;
	}
}

void Replay::SetStartTime(double time)
{
	startTime_ = time;
	if (time_ < startTime_)
	{
		time_ = startTime_;
	}

	startIndex_ = FindIndexAtTimestamp(startTime_);
}

void Replay::SetStopTime(double time)
{
	stopTime_ = time;
	if (time_ > stopTime_)
	{
		time_ = stopTime_;
	}

	stopIndex_ = FindIndexAtTimestamp(stopTime_);
}

void Replay::CleanEntries(std::vector<ReplayEntry>& entries)
{
	for (size_t i = 0; i < entries.size() - 1; i++)
	{
		if (entries[i + 1].state.info.timeStamp < entries[i].state.info.timeStamp)
		{
			entries.erase(entries.begin() + i + 1);
			i--;
		}

		for (int j = 1; (i + j < entries.size()) && NEAR_NUMBERS(entries[i + j].state.info.timeStamp, entries[i].state.info.timeStamp); j++)
		{
			// Keep the latest instance of entries with same timestamp
			if (entries[i + j].state.info.id == entries[i].state.info.id)
			{
				entries.erase(entries.begin() + i);
				i--;
				break;
			}
		}
	}
}

void Replay::BuildData(std::vector<std::pair<std::string, std::vector<ReplayEntry>>>& scenarios)
{
	// Keep track of current index of each scenario
	std::vector<int> cur_idx;
	std::vector<int> next_idx;

	for (size_t j = 0; j < scenarios.size(); j++)
	{
		cur_idx.push_back(0);
		next_idx.push_back(0);
	}

	// Set scenario ID-group (0, 100, 200 etc.)
	for (size_t j = 0; j < scenarios.size(); j++)
	{
		for (size_t k = 0; k < scenarios[j].second.size(); k++)
		{
			// Set scenario ID-group (0, 100, 200 etc.)
			scenarios[j].second[k].state.info.id += static_cast<int>(j) * 100;
		}
	}

	// Populate data_ based on first (with lowest timestamp) scenario
	double cur_timestamp = scenarios[0].second[0].state.info.timeStamp;
	while (cur_timestamp < LARGE_NUMBER - SMALL_NUMBER)
	{
		// populate entries if all scenarios at current time step
		double min_time_stamp = LARGE_NUMBER;
		for (size_t j = 0; j < scenarios.size() && next_idx[j] != -1; j++)
		{
			int k = cur_idx[j];
			for(; k < scenarios[j].second.size() &&
				scenarios[j].second[k].state.info.timeStamp < cur_timestamp + SMALL_NUMBER; k++)
			{
				// push entry with modified timestamp
				scenarios[j].second[k].state.info.timeStamp = (float)cur_timestamp;
				data_.push_back(scenarios[j].second[k]);
			}

			if (k < scenarios[j].second.size())
			{
				next_idx[j] = k;
				if (scenarios[j].second[k].state.info.timeStamp < min_time_stamp)
				{
					min_time_stamp = scenarios[j].second[k].state.info.timeStamp;
				}
			}
			else
			{
				next_idx[j] = -1;
			}
		}

		if (min_time_stamp < LARGE_NUMBER - SMALL_NUMBER)
		{
			for (size_t j = 0; j < scenarios.size(); j++)
			{
				if (next_idx[j] > 0 && scenarios[j].second[next_idx[j]].state.info.timeStamp < min_time_stamp + SMALL_NUMBER)
				{
					// time has reached next entry, step this scenario
					cur_idx[j] = next_idx[j];
				}
			}
		}

		cur_timestamp = min_time_stamp;
	}
}
