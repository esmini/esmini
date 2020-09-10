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

	struct ObjectStateStruct
	{
		int id;
		int model_id;
		int obj_type; // 0=Vehicle, 1=Pedestrian, 2=MiscObj
		int obj_category; // sub type for vehicle, pedestrian and miscobj
		int control; // 0= undefined, 1=internal, 2=external, 3=hybrid_external, 4=hybrid_ghost
		float timeStamp;
		char name[NAME_LEN];
		roadmanager::Position pos;
		float speed;
		float wheel_angle ; // Only used for vehicle
		float wheel_rot ; // Only used for vehicle
		OSCBoundingBox boundingbox;
	};

	class ObjectState
	{
	public:
		ObjectState();
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int control, OSCBoundingBox boundingbox,double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position *pos);
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int control, OSCBoundingBox boundingbox,double timestamp, double speed, double wheel_angle, double wheel_rot, double x, double y, double z, double h, double p, double r);
		ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int control, OSCBoundingBox boundingbox,double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, int laneId, double laneOffset, double s);

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

		void reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int control,OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			roadmanager::Position *pos);

		void reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int control,OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			double x, double y, double z, double h, double p, double r);

		void reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int control,OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			int roadId, int laneId, double laneOffset, double s);

		void removeObject(int id);
		void removeObject(std::string name);
		int getNumberOfObjects() { return (int)objectState_.size(); }
		ObjectState getObjectStateByIdx(int idx) { return *objectState_[idx]; }
		ObjectState *getObjectStatePtrByIdx(int idx) { return objectState_[idx]; }
		ObjectState *getObjectStatePtrById(int id);
		int getObjectStateById(int idx, ObjectState &objState);
		int RecordToFile(std::string filename, std::string odr_filename, std::string model_filename);

		std::vector<ObjectState*> objectState_;

	private:
		void updateObjectInfo(ObjectState* obj_state, double timestamp, double speed, double wheel_angle, double wheel_rot);
		std::ofstream data_file_;
	};

	class SumoController
	{
		public:
			SumoController(Entities* entities, ScenarioGateway* scenarioGateway);
			SumoController();
			void InitalizeObjects();
			void step(double time);
			void updatePositions();

		private:
			Entities* entities_;
			ScenarioGateway* scenarioGateway_;
			Vehicle* template_;
			bool sumo_used;
			// std::vector<SumoId> ids;
	};
}
