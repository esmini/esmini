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

#ifdef WIN32
	#define SE_DLL_API __declspec(dllexport)
#else
	#define SE_DLL_API  // Leave empty on Mac
#endif

#define SE_NAME_SIZE 32

typedef struct
{
	int id;					  // Automatically generated unique object id 
	int model_id;             // Id to control what 3D model to represent the vehicle 
	int ext_control;	      // vehicle controlled by 0 = scenario engine or 1 = external simulator
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
} SE_ScenarioObjectState; 

typedef struct
{
	float global_pos_x;     // steering target position, in global coordinate system
	float global_pos_y;     // steering target position, in global coordinate system
	float global_pos_z;     // steering target position, in global coordinate system
	float local_pos_x;      // steering target position, relative vehicle (pivot position object) coordinate system
	float local_pos_y;      // steering target position, relative vehicle (pivot position object) coordinate system
	float local_pos_z;      // steering target position, relative vehicle (pivot position object) coordinate system
	float angle;			// heading angle to steering target from and relatove to vehicle (pivot position)
	float road_heading;		// road heading at steering target point
	float road_pitch;		// road pitch (inclination) at steering target point
	float road_roll;		// road roll (camber) at steering target point
	float curvature;		// road curvature at steering target point
	float speed_limit;		// speed limit given by OpenDRIVE type entry
} SE_SteeringTargetInfo;


#ifdef __cplusplus
extern "C"
{
#endif
	/**
	Initialize the scenario engine
	@param oscFilename Path to the OpenSCEANRIO file
	@param ext_control Ego control 0=by OSC 1=No 2=Yes
	@param use_viewer 0=no viewer, 1=use viewer
	@param record Create recording for later playback 0=no recording 1=recording
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_Init(const char *oscFilename, int ext_control, int use_viewer, int record);

	SE_DLL_API int SE_Step(float dt);
	SE_DLL_API void SE_Close();

	SE_DLL_API int SE_ReportObjectPos(int id, char *name, int model_id, int ext_control, float timestamp, float x, float y, float z, float h, float p, float r, float speed);
	SE_DLL_API int SE_ReportObjectRoadPos(int id, char *name, int model_id, int ext_control, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

	SE_DLL_API int SE_GetNumberOfObjects();
	SE_DLL_API int SE_GetObjectState(int index, SE_ScenarioObjectState *state);
	SE_DLL_API int SE_GetObjectStates(int *nObjects, SE_ScenarioObjectState* state);

	/**
	Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
	@param handle Handle to the position object from which to measure
	@param lookahead_distance The distance, along the road, to the point
	@param data Struct including all result values, see typedef for details
	@param along_road_center Measure along the reference lane, i.e. at center of the road. Should be false for normal use cases
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetSteeringTargetInfo(int object_id, float lookahead_distance, SE_SteeringTargetInfo *data, int along_road_center);

#ifdef __cplusplus
}
#endif
