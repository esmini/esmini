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
#include "OSCBoundingBox.hpp"
#include "Entities.hpp"

namespace scenarioengine
{

#define NAME_LEN 32

	struct ObjectInfoStruct
	{
		int id;
		int model_id;
		int obj_type; // 0=Vehicle, 1=Pedestrian, 2=MiscObj
		int obj_category; // sub type for vehicle, pedestrian and miscobj
		int ctrl_type;  // See Controller::Type enum
		float timeStamp;
		char name[NAME_LEN];
		float speed;
		float wheel_angle; // Only used for vehicle
		float wheel_rot; // Only used for vehicle
		OSCBoundingBox boundingbox;
	};

	struct ObjectStateStruct
	{
		struct ObjectInfoStruct info;
		roadmanager::Position pos;
	};

	struct ObjectPositionStruct
	{
		float x;
		float y;
		float z;
		float h;
		float p;
		float r;
		int roadId;
		int laneId;
		float offset;
		float t;
		float s;
	};

	struct ObjectStateStructDat
	{
		struct ObjectInfoStruct info;
		struct ObjectPositionStruct pos;
	};

	class ObjectState
	{
	public:
		ObjectState();
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position* pos);
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot, double x, double y, double z, double h, double p, double r);
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, int laneId, double laneOffset, double s);
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, double lateralOffset, double s);

		ObjectStateStruct getStruct() { return state_; }

		void Print();

		ObjectStateStruct state_;

	private:

		friend class ScenarioGateway;
	};


	class ScenarioGateway
	{
	public:
		ScenarioGateway();
		~ScenarioGateway();

		void reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position *pos);

		void reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			double x, double y, double z, double h, double p, double r);

		void reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			double x, double y, double h);

		void reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			int roadId, int laneId, double laneOffset, double s);

		void reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			int roadId, double lateralOffset, double s);

		void reportObjectSpeed(int id, double speed);
		void reportObjectVel(int id, double x_vel, double y_vel, double z_vel);
		void reportObjectAcc(int id, double x_acc, double y_acc, double z_acc);
		void reportObjectAngularVel(int id, double h_rate, double p_rate, double r_rate);
		void reportObjectAngularAcc(int id, double h_acc, double p_acc, double r_acc);

		void removeObject(int id);
		void removeObject(std::string name);
		int getNumberOfObjects() { return (int)objectState_.size(); }
		ObjectState getObjectStateByIdx(int idx) { return *objectState_[idx]; }
		ObjectState *getObjectStatePtrByIdx(int idx) { return objectState_[idx]; }
		ObjectState *getObjectStatePtrById(int id);
		int getObjectStateById(int idx, ObjectState &objState);
		void WriteStatesToFile();
		int RecordToFile(std::string filename, std::string odr_filename, std::string model_filename);

		std::vector<ObjectState*> objectState_;

	private:
		void updateObjectInfo(ObjectState* obj_state, double timestamp, double speed, double wheel_angle, double wheel_rot);
		std::ofstream data_file_;
	};

}
