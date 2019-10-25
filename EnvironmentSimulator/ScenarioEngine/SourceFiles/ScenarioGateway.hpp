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
#include "RoadManager.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

namespace scenarioengine
{

#define NAME_LEN 32

	struct ObjectStateStruct
	{
		int id;
		int model_id;
		int control; // 0= undefined, 1=internal, 2=external, 4=hybrid_ghost, 3=hybrid_external
		float timeStamp;
		char name[NAME_LEN];
		roadmanager::Position pos;
		float speed;
		float wheel_angle;
//		int ghost_id;  // id of ghost, if exists
	};

	class ObjectState
	{
	public:
		ObjectState();
#if 0
		ObjectState(int id, std::string name, int model_id, int control, double timestamp, roadmanager::Position *pos, double speed, double wheel_angle, int ghost_id);
		ObjectState(int id, std::string name, int model_id, int control, double timestamp, double x, double y, double z, double h, double p, double r, double speed, double wheel_angle, int ghost_id);
		ObjectState(int id, std::string name, int model_id, int control, double timestamp, int roadId, int laneId, double laneOffset, double s, double speed, double wheel_angle, int ghost_id);
#else
		ObjectState(int id, std::string name, int model_id, int control, double timestamp, roadmanager::Position *pos, double speed, double wheel_angle);
		ObjectState(int id, std::string name, int model_id, int control, double timestamp, double x, double y, double z, double h, double p, double r, double speed, double wheel_angle);
		ObjectState(int id, std::string name, int model_id, int control, double timestamp, int roadId, int laneId, double laneOffset, double s, double speed, double wheel_angle);
#endif

		ObjectStateStruct getStruct() { return state_; }

		void Print();

		ObjectStateStruct state_;

	private:

		friend class ScemarioGateway;
	};


	class ScenarioGateway
	{
	public:

		ScenarioGateway();
		~ScenarioGateway();

		void reportObject(ObjectState objectState, bool update = false);
		int getNumberOfObjects() { return (int)objectState_.size(); }
		ObjectState getObjectStateByIdx(int idx) { return *objectState_[idx]; }
		ObjectState *getObjectStatePtrByIdx(int idx) { return objectState_[idx]; }
		int getObjectStateById(int idx, ObjectState &objState);
		int RecordToFile(std::string filename, std::string odr_filename, std::string model_filename);

	private:
		std::vector<ObjectState*> objectState_;
		std::ofstream data_file_;
	};

}