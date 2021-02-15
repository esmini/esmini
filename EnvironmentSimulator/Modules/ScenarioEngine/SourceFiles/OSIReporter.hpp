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
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>


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
	bool OpenOSIFiles(const char* dynamic_filename, const char* static_filename);
	/**
	Closes any open osi file
	*/
	void CloseOSIFiles();
	/**
	Writes Dynamic GroundTruth in the OSI file
	*/
	bool WriteDynamicOSIFile();
	/**
	Writes Static GroundTruth in the OSI file
	*/
	bool WriteStaticOSIFile();
	/**
	Flush (force write) the Dynamic OSI file
	*/
	void FlushDynamicOSIFile();
	/**
	Flush (force write) the Static OSI file
	*/
	void FlushStaticOSIFile();
	/**
	Fills up the dynamic osi message with GroundTruth
	*/
	int UpdateOSIDynamicGroundTruth(std::vector<ObjectState*> objectState);
	/**
	Fills up the static osi message with GroundTruth
	*/
	int UpdateOSIStaticGroundTruth(std::vector<ObjectState*> objectState);
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
	Fills up the osi message with Lanes for static Lanes data
	*/
	int UpdateOSIStaticRoadLane();
	/**
	Fills up the osi message with Lanes for dynamic Lanes data
	*/
	int UpdateOSIDynamicRoadLane(std::vector<ObjectState*> objectState);

	const char* GetOSIDynamicGroundTruth(int* size);
	const char* GetOSIStaticGroundTruth(int* size);
	const char* GetOSIDynamicGroundTruthRaw();
	const char* GetOSIStaticGroundTruthRaw();
	const char* GetOSIRoadLane(std::vector<ObjectState*> objectState, int* size, int object_id);
	const char* GetOSIRoadLaneBoundary(int* size, int global_id);
	void GetOSILaneBoundaryIds(std::vector<ObjectState*> objectState, std::vector<int>& ids, int object_id);
    const char* GetOSISensorDataRaw();
	bool IsCentralOSILane(int lane_idx);
	int GetLaneIdxfromIdOSI(int lane_id);
	int OpenSocket(std::string ipaddr);
	int CloseSocket();
	int GetSocket() { return sendSocket; }
	bool IsDynamicFileOpen() { return osi_file_dynamic.is_open(); }
	bool IsStaticFileOpen() { return osi_file_static.is_open(); }
	void ReportSensors(std::vector<ObjectSensor*> sensor);

private:
	int sendSocket;
	std::ofstream osi_file_dynamic;
	std::ofstream osi_file_static;
};
