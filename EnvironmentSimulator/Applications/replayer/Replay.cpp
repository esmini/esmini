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
		ObjectStateStructDat data;

		file_.read((char*)&data, sizeof(data));

		if (!file_.eof())
		{
			data_.push_back(data);
		}
	}

	// Ensure increasing timestamps. Remove any other entries.
	for (int i = 0; i < data_.size() - 1; i++)
	{
		if (data_[i + 1].info.timeStamp < data_[i].info.timeStamp)
		{
			data_.erase(data_.begin() + i + 1);
			i--;   // compensate for removed entry
		}
	}

	if (data_.size() > 0)
	{
		// Register first entry timestamp as starting time
		time_ = data_[0].info.timeStamp;
		startTime_ = time_;
		startIndex_ = 0;

		// Register last entry timestamp as stop time
		stopTime_ = data_[data_.size() - 1].info.timeStamp;
		stopIndex_ = FindIndexAtTimestamp(stopTime_);
	}
}

Replay::Replay(const std::string directory, const std::string scenario) : time_(0.0), index_(0), repeat_(false)
{
	GetReplaysFromDirectory(directory, scenario);
	std::vector<std::pair<std::string, std::vector<ObjectStateStructDat>>> scenarioData;

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
			ObjectStateStructDat data;

			file_.read((char*)&data, sizeof(data));

			if (!file_.eof())
			{
				data_.push_back(data);
			}
		}
		// pair <scenario name, scenario data>
		scenarioData.push_back(std::make_pair(scenarios_[i], data_));
		data_ = {};
		file_.close();
	}

	if (scenarioData.size() > 1)
	{
		// Longest scenario first
		std::sort(scenarioData.begin(), scenarioData.end(),
		[](const auto& sce1, const auto& sce2)
		{
			return sce1.second.size() > sce2.second.size();
		}
		);

		LOG("Longest scenario is main scenario: %s", FileNameOf(scenarioData.begin()->first.c_str()).c_str());
	}

	// Log which scenario belongs to what ID-group (0, 100, 200 etc.)
	for (size_t i = 0; i < scenarioData.size(); i++)
	{
		std::string scenario_tmp = (scenarioData.begin()+i)->first;
		LOG("Scenarios corresponding to IDs (%d:%d): %s", i * 100, (i+1) * 100 - 1, FileNameOf(scenario_tmp.c_str()).c_str());
	}

	// Ensure increasing timestamps. Remove any other entries.
	for (auto& sce : scenarioData)
	{
		for (size_t i = 0; i < sce.second.size() - 1; i++)
		{
			if (sce.second[i + 1].info.timeStamp < sce.second[i].info.timeStamp)
			{
				sce.second.erase(sce.second.begin() + i + 1);
				i--;
			}
		}
	}

	const size_t scenario_length = scenarioData.begin()->second.size();

	// Iterate over samples, then scenario, states added to data_ as scenario1+sample1, scenario2+sample1, scenario1+sample2 etc.
	for (size_t i = 0; i < scenario_length; i++)
	{
		int ctr = 0;
		for (auto& sce : scenarioData)
		{
			if (i < sce.second.size())
			{
				sce.second[i].info.id += ctr * 100;
				data_.push_back(sce.second[i]);
			}
			ctr++;
		}
	}

	if (data_.size() > 0)
	{
		// Register first entry timestamp as starting time
		time_ = data_[0].info.timeStamp;
		startTime_ = time_;
		startIndex_ = 0;

		// Register last entry timestamp as stop time
		stopTime_ = data_[data_.size() - 1].info.timeStamp;
		stopIndex_ = FindIndexAtTimestamp(stopTime_);
	}
}

// Browse through replay-folder and appends strings of absolute path to matching scenario
void Replay::GetReplaysFromDirectory(const std::string dir, const std::string sce) {
	DIR* directory = opendir(dir.c_str());

	// If no directory found, write error
	if (directory == nullptr) {
		LOG_AND_QUIT("No valid directory given, couldn't open %s", dir.c_str());
	}
	// While directory is open, check the filename
	struct dirent* file;
	while ((file = readdir(directory)) != nullptr) {
		std::string filename = file->d_name;

		if (filename != "." && filename != ".." && filename.find(sce) != std::string::npos) {
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
			if (next_index > (int)index_ && time > data_[next_index].info.timeStamp)
			{
				index_ = next_index;
				time_ = data_[index_].info.timeStamp;
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
			if (next_index < (int)index_ && time < data_[next_index].info.timeStamp)
			{
				index_ = next_index;
				time_ = data_[index_].info.timeStamp;
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
	double ctime = data_[index_].info.timeStamp;
	for (size_t i = index_+1; i < data_.size(); i++)
	{
		if (data_[i].info.timeStamp > ctime)
		{
			GoToTime(data_[i].info.timeStamp);
			break;
		}
	}
}

void Replay::GoToPreviousFrame()
{
	if (index_ > 0)
	{
		GoToTime(data_[index_ -1].info.timeStamp);
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
		if (data_[i].info.timeStamp >= timestamp)
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
		if (data_[index].info.timeStamp > data_[index_].info.timeStamp)
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
		if (data_[i].info.timeStamp < data_[index].info.timeStamp)
		{
			break;
		}
		index = i;
	}

	return index;
}

ObjectStateStructDat* Replay::GetState(int id)
{
	// Read all vehicles at current timestamp
	float timestamp = data_[index_].info.timeStamp;
	int i = 0;
	while (index_ + i < data_.size() && !(data_[index_ + i].info.timeStamp > timestamp))
	{
		if (data_[index_ + i].info.id == id)
		{
			return &data_[index_ + i];
		}
		i++;
	}

	return nullptr;
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
