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

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>

#ifdef _WIN32
	//#include <winsock2.h>
	//#include <Ws2tcpip.h>
#else
	 /* Assume that any non-Windows platform uses POSIX-style sockets instead. */
	#include <sys/socket.h>
	#include <arpa/inet.h>
	#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
	#include <unistd.h> /* Needed for close() */
#endif

namespace scenarioengine
{

#define NAME_LEN 32


	typedef struct
	{
		std::string sensor_view;
		unsigned int size;
	} OSISensorView;

	typedef struct
	{
		std::string osi_lane_info;
		unsigned int size;
	} OSIRoadLane;

	typedef struct
	{
		std::string osi_lane_boundary_info;
		unsigned int size;
	} OSIRoadLaneBoundary;

	struct ObjectStateStruct
	{
		int id;
		int model_id;
		int control; // 0= undefined, 1=internal, 2=external, 3=hybrid_external, 4=hybrid_ghost
		float timeStamp;
		char name[NAME_LEN];
		roadmanager::Position pos;
		float speed;
		float wheel_angle;
		float wheel_rot;
		/*double acceleration;
		double CenterOffsetX;
		double CenterOffsetY;
		double CenterOffsetZ;
		double width;
		double length;
		double height;*/
		OSCBoundingBox boundingbox;
	};

	class ObjectState
	{
	public:
		ObjectState();
		ObjectState(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox,double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position *pos);
		ObjectState(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox,double timestamp, double speed, double wheel_angle, double wheel_rot, double x, double y, double z, double h, double p, double r);
		ObjectState(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox,double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, int laneId, double laneOffset, double s);

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

		void reportObject(int id, std::string name, int model_id, int control,OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			roadmanager::Position *pos);

		void reportObject(int id, std::string name, int model_id, int control,OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			double x, double y, double z, double h, double p, double r);

		void reportObject(int id, std::string name, int model_id, int control,OSCBoundingBox boundingbox,
			double timestamp, double speed, double wheel_angle, double wheel_rot,
			int roadId, int laneId, double laneOffset, double s);

		int getNumberOfObjects() { return (int)objectState_.size(); }
		ObjectState getObjectStateByIdx(int idx) { return *objectState_[idx]; }
		ObjectState *getObjectStatePtrByIdx(int idx) { return objectState_[idx]; }
		ObjectState *getObjectStatePtrById(int id);
		int getObjectStateById(int idx, ObjectState &objState);
		int RecordToFile(std::string filename, std::string odr_filename, std::string model_filename);
		/**
		Creates and opens osi file 
		*/
		bool OpenOSIFile();
		/**
		Writes SensorView in the OSI file 
		*/
		bool WriteOSIFile();
		/**
		Fills up the osi message with SensorView
		*/
		int UpdateOSISensorView();
		/**
		Fills up the osi message with Moving Object
		*/
		int UpdateOSIMovingObject();
		/**
		Fills up the osi message with Lane Boundary
		*/
		int UpdateOSILaneBoundary();
		/**
		Fills up the osi message with Lanes
		*/
		int UpdateOSIRoadLane();
		/**
		This function checks if the angle is in the interval of [-pi/2, +pi/2]
		*/
		bool IsAngleStraight(double teta); 
		const char* GetOSISensorView(int* size);
		const char* GetOSIRoadLane(int* size, int object_id);
		const char* GetOSIRoadLaneBoundary(int* size, int global_id);
		void GetOSILaneBoundaryIds(std::vector<int> &ids, int object_id);
		bool IsCentralOSILane(int lane_idx);
		int GetLaneIdxfromIdOSI(int lane_id);
		int OpenSocket(std::string ipaddr);
		int CloseSocket();
		int GetSocket() { return sendSocket; }

	private:
		void updateObjectInfo(ObjectState* obj_state, double timestamp, double speed, double wheel_angle, double wheel_rot);
		std::vector<ObjectState*> objectState_;
		std::ofstream data_file_;
		int sendSocket;
	};

}
