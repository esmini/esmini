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

#include <string>
#include <fstream>
#include "CommonMini.hpp"
#include "ScenarioGateway.hpp"

namespace scenarioengine
{
	typedef struct
	{
		ObjectStateStructDat state;
		double odometer;
	} ReplayEntry;

	class Replay
	{
	public:
		DatHeader header_;
		std::vector<ReplayEntry> data_;

		Replay(std::string filename, bool clean);
		// Replay(const std::string directory, const std::string scenario, bool clean);
		Replay(const std::string directory, const std::string scenario, std::string create_datfile);
		~Replay();

		/**
			Go to specific time
			@param time timestamp (0 = beginning, -1 end)
			@param stop_at_next_frame If true move max to next/previous time frame
		*/
		void GoToTime(double time, bool stop_at_next_frame = false);
		void GoToDeltaTime(double dt, bool stop_at_next_frame = false);
		void GetReplaysFromDirectory(const std::string dir, const std::string sce);
		size_t GetNumberOfScenarios();
		void GoToStart();
		void GoToEnd();
		int GoToNextFrame();
		void GoToPreviousFrame();
		int FindNextTimestamp(bool wrap = false);
		int FindPreviousTimestamp(bool wrap = false);
		ReplayEntry* GetEntry(int id);
		ObjectStateStructDat* GetState(int id);
		void SetStartTime(double time);
		void SetStopTime(double time);
		double GetStartTime() { return startTime_; }
		double GetStopTime() { return stopTime_; }
		double GetTime() { return time_; }
		int GetIndex() { return static_cast<int>(index_); }
		void SetRepeat(bool repeat) { repeat_ = repeat; }
		void CleanEntries(std::vector<ReplayEntry>& entries);
		void BuildData(std::vector<std::pair<std::string, std::vector<ReplayEntry>>>& scenarios);
		void CreateMergedDatfile(const std::string filename);

private:
		std::ifstream file_;
		std::vector<std::string> scenarios_;
		double time_;
		double startTime_;
		double stopTime_;
		unsigned int startIndex_;
		unsigned int stopIndex_;
		unsigned int index_;
		bool repeat_;
		bool clean_;
		std::string create_datfile_;

		int FindIndexAtTimestamp(double timestamp, int startSearchIndex = 0);
	};

}