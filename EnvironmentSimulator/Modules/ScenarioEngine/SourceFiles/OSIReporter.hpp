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

#include "IdealSensor.hpp"
#include "ScenarioGateway.hpp"
#include "osi_sensordata.pb.h"
#include "osi_object.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_version.pb.h"
#include "osi_common.pb.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <math.h>

#define DEFAULT_OSI_TRACE_FILENAME "ground_truth.osi"

using namespace scenarioengine;

class OSIReporter
{
public:

	OSIReporter();
	~OSIReporter();

	/**
	Creates and opens osi file
	@param filename Optional filename, including path. Set to 0 to use default.
	*/
	bool OpenOSIFile(const char* filename);
	/**
	Closes any open osi file
	*/
	void CloseOSIFile();
	/**
	Writes GroundTruth in the OSI file
	*/
	bool WriteOSIFile();
	/**
	Flush (force write) the OSI file
	*/
	void FlushOSIFile();
	/**
	Clears groundtruth osi
	*/
	int ClearOSIGroundTruth();
	/**
	Calls UpdateOSIStaticGroundTruth and UpdateOSIDynamicGroundTruth
	*/
	int UpdateOSIGroundTruth(const std::vector<std::unique_ptr<ObjectState>>& objectState);
	/**
	Fills up the osi message with  static GroundTruth
	*/
	int UpdateOSIStaticGroundTruth(const std::vector<std::unique_ptr<ObjectState>>& objectState);
	/**
	Fills up the osi message with dynamic GroundTruth
	*/
	int UpdateOSIDynamicGroundTruth(const std::vector<std::unique_ptr<ObjectState>>& objectState, bool reportGhost = true);
	/**
	Fills up the osi message with Stationary Object from the OpenDRIVE description
	*/
	int UpdateOSIStationaryObjectODR(int road_id, roadmanager::RMObject* object);
	/**
	Fills up the osi message with Stationary Object
	*/
	int UpdateOSIStationaryObject(ObjectState* objectState);
	/**
	Fills up the osi message with Host Vehicle data
	*/
	int UpdateOSIHostVehicleData(ObjectState* objectState);
	/**
	Fills up the osi message with Moving Object
	*/
	int UpdateOSIMovingObject(ObjectState* objectState);
	/**
	Fills up the osi message with Lane Boundary
	*/
	int UpdateOSILaneBoundary();
	/**
	Fills up the osi message with Lanes
	*/
	int UpdateOSIRoadLane();
	/**
	Fills the intersection type of lanes
	*/
	int UpdateOSIIntersection();
	/**
	Fills the Traffic Signals
	*/
	int UpdateTrafficSignals();

	/**
	 Creates a SensorView from SensorData for plotting
	*/
	int CreateSensorViewFromSensorData(const osi3::SensorData &sd);

	const char* GetOSIGroundTruth(int* size);
	const char* GetOSIGroundTruthRaw();
	const char* GetOSIRoadLane(const std::vector<std::unique_ptr<ObjectState>>& objectState, int* size, int object_id);
	const char* GetOSIRoadLaneBoundary(int* size, int global_id);
	void GetOSILaneBoundaryIds(const std::vector<std::unique_ptr<ObjectState>>& objectState, std::vector<int>& ids, int object_id);
    const char* GetOSISensorDataRaw();
	osi3::SensorView *GetSensorView();
	bool IsCentralOSILane(int lane_idx);
	int GetLaneIdxfromIdOSI(int lane_id);
	int OpenSocket(std::string ipaddr);
	int CloseSocket();
	int GetSocket() { return sendSocket; }
	bool IsFileOpen() { return osi_file.is_open(); }
	void ReportSensors(std::vector<ObjectSensor*> sensor);
	int GetCounter() { return osi_update_counter_; }

	/**
	Set explicit timestap
	@param nanoseconds Nano (1e-9) seconds since 1970-01-01 (epoch time)
	@return 0 if successful, -1 if not
	*/
	int SetOSITimeStampExplicit(unsigned long long int nanoseconds);
	bool IsTimeStampSetExplicit() { return nanosec_ != 0xffffffffffffffff; }

private:
	int sendSocket;
	unsigned long long int nanosec_;
	std::ofstream osi_file;
	int osi_update_counter_;
  	void CreateMovingObjectFromSensorData(const osi3::SensorData &sd, int obj_nr);
  	void CreateLaneBoundaryFromSensordata(const osi3::SensorData &sd, int lane_boundary_nr);
};
