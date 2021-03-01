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

#define REPLAY_FILENAME_SIZE 128

	typedef struct
	{
		char odr_filename[REPLAY_FILENAME_SIZE];
		char model_filename[REPLAY_FILENAME_SIZE];
	} ReplayHeader;

	typedef enum  // copy key enums from OSG GUIEventAdapter
	{
		KEY_Left = 0xFF51,        /* Left arrow */
		KEY_Up = 0xFF52,          /* Up arrow */
		KEY_Right = 0xFF53,       /* Right arrow */
		KEY_Down = 0xFF54,        /* Down arrow */
		KEY_Space = 0x20,         /* Space */

		// Mod key types
		KEY_Shift_L = 0xFFE1,     /* Left shift */
		KEY_Shift_R = 0xFFE2,     /* Right shift */
		KEY_Control_L = 0xFFE3,   /* Left control */
		KEY_Control_R = 0xFFE4,   /* Right control */
	} KeyType;

	class Replay
	{
	public:
		ReplayHeader header_;
		std::vector<ObjectStateStructDat> data_;

		Replay(std::string filename);
		~Replay();
		void GoToTime(double dt);
		void GoToNextFrame();
		void GoToPreviousFrame();
		ObjectStateStructDat * GetState(int id);
		void SetStartTime(double time);
		void SetStopTime(double time);
		double GetStartTime() { return startTime_; }
		double GetTime() { return time_; }
		int GetIndex() { return index_; }
		void SetRepeat(bool repeat) { repeat_ = repeat; }

private:
		std::ifstream file_;
		double time_;
		double startTime_;
		double stopTime_;
		unsigned int startIndex_;
		unsigned int stopIndex_;
		unsigned int index_;
		bool repeat_;

		int FindIndexAtTimestamp(double timestamp, int startSearchIndex = 0);
	};

}