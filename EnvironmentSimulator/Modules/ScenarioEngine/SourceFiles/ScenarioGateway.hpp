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

#define DAT_FILE_FORMAT_VERSION 1

namespace scenarioengine
{

#define NAME_LEN 32

	struct ObjectInfoStruct
	{
		int id;
		int model_id;
		int obj_type; // 0=None, 1=Vehicle, 2=Pedestrian, 3=MiscObj (see Object::Type enum)
		int obj_category; // sub type for vehicle, pedestrian and miscobj
		int ctrl_type;  // See Controller::Type enum
		float timeStamp;
		char name[NAME_LEN];
		float speed;
		float wheel_angle; // Only used for vehicle
		float wheel_rot; // Only used for vehicle
		OSCBoundingBox boundingbox;
		int scaleMode; // 0=None, 1=BoundingBoxToModel, 2=ModelToBoundingBox (see enum EntityScaleMode)
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
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position* pos);
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, double x, double y, double z, double h, double p, double r);
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, int laneId, double laneOffset, double s);
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, double lateralOffset, double s);

		ObjectStateStruct getStruct() { return state_; }

		void Print();
		void clearDirtyBits() { dirty_ = 0; }

		ObjectStateStruct state_;
		unsigned int dirty_;

	private:

		friend class ScenarioGateway;
	};


	class ScenarioGateway
	{
	public:
		ScenarioGateway();
		~ScenarioGateway();

		int reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position *pos);

		int reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
			double x, double y, double z, double h, double p, double r);

		int reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
			double x, double y, double h);

		int reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
			int roadId, int laneId, double laneOffset, double s);

		int reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
			int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
			int roadId, double lateralOffset, double s);

		int updateObjectPos(int id, double timestamp, roadmanager::Position* pos);
		int updateObjectRoadPos(int id, double timestamp, int roadId, double lateralOffset, double s);
		int updateObjectLanePos(int id, double timestamp, int roadId, int laneId, double offset, double s);
		int updateObjectWorldPos(int id, double timestamp, double x, double y, double z, double h, double p, double r);
		int updateObjectWorldPosXYH(int id, double timestamp, double x, double y, double h);
		int updateObjectSpeed(int id, double timestamp, double speed);
		int updateObjectVel(int id, double timestamp, double x_vel, double y_vel, double z_vel);
		int updateObjectAcc(int id, double timestamp, double x_acc, double y_acc, double z_acc);
		int updateObjectAngularVel(int id, double timestamp, double h_rate, double p_rate, double r_rate);
		int updateObjectAngularAcc(int id, double timestamp, double h_acc, double p_acc, double r_acc);
		int updateObjectWheelAngle(int id, double timestamp, double wheelAngle);
		int updateObjectWheelRotation(int id, double timestamp, double wheelRotation);
		bool isObjectReported(int id);
		void clearDirtyBits();

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
		int updateObjectInfo(ObjectState* obj_state, double timestamp, double speed, double wheel_angle, double wheel_rot);
		std::ofstream data_file_;
	};

}
