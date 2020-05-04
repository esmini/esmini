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

using namespace scenarioengine;


Replay::Replay(std::string filename) : time_(0.0), index_(0)
{
	file_.open(filename, std::ofstream::binary);
	if (file_.fail())
	{
		LOG("Cannot open file: %s", filename.c_str());
		throw std::invalid_argument(std::string("Cannot open file: ") + filename);
	}

	file_.read((char*)&header_, sizeof(header_));
	LOG("Recording %s opened. odr: %s model: %s", filename.c_str(), header_.odr_filename, header_.model_filename);

	while (!file_.eof())
	{
		ObjectStateStruct data;

		file_.read((char*)&data, sizeof(data));

		if (!file_.eof())
		{
			data_.push_back(data);
		}
	}
}

Replay::~Replay()
{
	data_.clear();
}

void Replay::Step(double dt)
{
	ObjectStateStruct data;

	time_ += dt;

	// Find entry according to time 
	while ((index_ < data_.size() - 1 && time_ > data_[index_ + 1].timeStamp) || data_[index_].id != 0)
	{
		index_++;
		if (index_ >= data_.size() - 1)
		{
			index_ = 0;
			time_ = 0;
		}
	}
}

ObjectStateStruct* Replay::GetState(int id)
{
	// Read all vehicles at current timestamp
	if (index_ + id > data_.size() - 1 || data_[index_ + id].id != id)
	{
		return 0;
	}

	return &data_[index_ + id];
}
