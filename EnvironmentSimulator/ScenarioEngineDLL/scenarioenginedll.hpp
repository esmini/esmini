﻿/*
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

#ifdef WIN32
	#define SE_DLL_API __declspec(dllexport)
#else
	#define SE_DLL_API  // Leave empty on Mac
#endif

#define SE_NAME_SIZE 32


typedef struct
{
	int id;					  // Automatically generated unique object id
	int model_id;             // Id to control what 3D model to represent the vehicle - see carModelsFiles_[] in scenarioenginedll.cpp
	int control;		      // 0= undefined, 1=internal, 2=external, 3=hybrid_external, 4=hybrid_ghost
	float timestamp;
	float x;
	float y;
	float z;
	float h;
	float p;
	float r;
	int roadId;
	float t;
	int laneId;
	float laneOffset;
	float s;
	float speed;
	float centerOffsetX;
	float centerOffsetY;
	float centerOffsetZ;
	float width;
	float length;
	float height;
} SE_ScenarioObjectState;


typedef struct
{
	float global_pos_x;     // target position, in global coordinate system
	float global_pos_y;     // target position, in global coordinate system
	float global_pos_z;     // target position, in global coordinate system
	float local_pos_x;      // target position, relative vehicle (pivot position object) coordinate system
	float local_pos_y;      // target position, relative vehicle (pivot position object) coordinate system
	float local_pos_z;      // target position, relative vehicle (pivot position object) coordinate system
	float angle;			// heading angle to target from and relatove to vehicle (pivot position)
	float road_heading;		// road heading at steering target point
	float road_pitch;		// road pitch (inclination) at steering target point
	float road_roll;		// road roll (camber) at target point
	float trail_heading;	// trail heading (only when used for trail lookups, else equals road_heading)
	float curvature;		// road curvature at steering target point
	float speed_limit;		// speed limit given by OpenDRIVE type entry
} SE_RoadInfo;


#ifdef __cplusplus
extern "C"
{
#endif
	/**
	Initialize the scenario engine
	@param oscFilename Path to the OpenSCEANRIO file
	@param control Ego control 0=by OSC 1=Internal 2=External 3=Hybrid
	@param use_viewer 0=no viewer, 1=use viewer
	@param threads 0=single thread, 1=viewer in a separate thread, parallel to scenario engine
	@param record Create recording for later playback 0=no recording 1=recording
	@param headstart_time For hybrid control mode launch ghost vehicle with this headstart time
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_Init(const char *oscFilename, int control, int use_viewer, int threads, int record, float headstart_time);

	/**
	Step the simulation forward with specified timestep
	@param dt time step in seconds
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_StepDT(float dt);

	/**
	Step the simulation forward. Time step will be elapsed system (world) time since last step. Useful for interactive/realtime use cases.
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_Step();

	/**
	Get the bool value of the end of the scenario
	*/
	SE_DLL_API int SE_GetQuitFlag();

	/**
	Stop simulation gracefully. Two purposes: 1. Release memory and 2. Prepare for next simulation, e.g. reset object lists.
	*/
	SE_DLL_API void SE_Close();

	/**
	Send OSI packages over UDP to specified IP address
	*/
	SE_DLL_API int SE_OpenOSISocket(char *ipaddr);

	/**
	Get simulation time in seconds
	*/
	SE_DLL_API float SE_GetSimulationTime();  // Get simulation time in seconds

	SE_DLL_API int SE_ReportObjectPos(int id, float timestamp, float x, float y, float z, float h, float p, float r, float speed);
	SE_DLL_API int SE_ReportObjectRoadPos(int id, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

	SE_DLL_API int SE_GetNumberOfObjects();

	SE_DLL_API int SE_GetObjectState(int index, SE_ScenarioObjectState *state);
	SE_DLL_API int SE_GetObjectGhostState(int index, SE_ScenarioObjectState *state);
	/**
	The SE_GetOSISensorView function returns a char array containing the osi SensorView serialized to a string
	*/
	SE_DLL_API int SE_UpdateOSISensorView();

	/**
	The SE_GetOSISensorView function returns a char array containing the osi SensorView serialized to a string
	*/
	SE_DLL_API const char* SE_GetOSISensorView(int* size);

	/**
	The SE_GetOSIRoadLane function returns a char array containing the osi Lane information/message of the lane where the object with object_id is, serialized to a string
	*/
	SE_DLL_API const char* SE_GetOSIRoadLane(int* size, int object_id);
	/**
	The SE_GetOSIRoadLane function returns a char array containing the osi Lane Boundary information/message with the specified GLOBAL id
	*/
	SE_DLL_API const char* SE_GetOSILaneBoundary(int* size, int global_id);
	/**
	The SE_GetOSILaneBoundaryIds function the global ids for left, far elft, right and far right lane boundaries
	*/
	SE_DLL_API void SE_GetOSILaneBoundaryIds(std::vector<int> &ids, int object_id);
		/**
	Create and open osi file 
	*/
	SE_DLL_API bool SE_OSIFileOpen();
	/**
	Create and open osi file 
	*/
	SE_DLL_API bool SE_OSIFileWrite();

	/**

	/**
	Create an ideal object sensor and attach to specified vehicle
	@param object_id Handle to the object to which the sensor should be attached
	@param x Position x coordinate of the sensor in vehicle local coordinates
	@param y Position y coordinate of the sensor in vehicle local coordinates
	@param z Position z coordinate of the sensor in vehicle local coordinates
	@param h heading of the sensor in vehicle local coordinates
	@param fovH Horizontal field of view, in degrees
	@param rangeNear Near value of the sensor depth range
	@param rangeFar Far value of the sensor depth range
	@param maxObj Maximum number of objects theat the sensor can track
	@return Sensor ID (Global index of sensor), -1 if unsucessful
	*/
	SE_DLL_API int SE_AddObjectSensor(int object_id, float x, float y, float z, float h, float rangeNear, float rangeFar, float fovH, int maxObj);

	/**
	Fetch list of identified objects from a sensor
	@param sensor_id Handle (index) to the sensor
	@param list Array of object indices
	@return Number of identified objects, i.e. length of list. -1 if unsuccesful.
	*/
	SE_DLL_API int SE_FetchSensorObjectList(int sensor_id, int *list);

	/**
	Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
	@param object_id Handle to the position object from which to measure
	@param lookahead_distance The distance, along the road, to the point
	@param data Struct including all result values, see typedef for details
	@param lookAheadMode Measurement strategy: Along 0=lane center, 1=road center (ref line) or 2=current lane offset. See roadmanager::Position::LookAheadMode enum
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo *data, int lookAheadMode);

	/**
	Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle
	@param object_id Handle to the position object from which to measure (the actual externally controlled Ego vehicle, not ghost)
	@param lookahead_distance The distance, along the ghost trail, to the point from the current Ego vehicle location
	@param data Struct including all result values, see typedef for details
	@param speed_ghost reference to a variable returning the speed that the ghost had at this point along trail
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *data, float *speed_ghost);


#ifdef __cplusplus
}
#endif
