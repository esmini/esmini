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
  #define RM_DLL_API __declspec(dllexport)
#else
  #define RM_DLL_API  // Leave empty on Mac
#endif

typedef struct
{
	float x;
	float y;
	float z;
	float h;
	float p;
	float r;
	float hRelative;
	int   roadId;
	int   laneId;
	float laneOffset;
	float s;
} PositionData;

typedef struct
{
	float global_pos[3];    // steering target position, in global coordinate system
	float local_pos[3];     // steering target position, relative vehicle (pivot position object) coordinate system
	float angle;				// heading angle to steering target from and relatove to vehicle (pivot position)
	float curvature;			// road curvature at steering target point
} SteeringTargetData;

#ifdef __cplusplus
extern "C"
{
#endif

	RM_DLL_API int RM_Init(const char *odrFilename);

	RM_DLL_API int RM_Close();

	/**
	Create a position object
	@return Handle to the position object, to use for operations
	*/
	RM_DLL_API int RM_CreatePosition();

	/**
	Get the total number fo roads in the road network of the currently loaded OpenDRIVE file.
	@return Number of roads
	*/
	RM_DLL_API int RM_GetNumberOfRoads();

	/**
	Get the Road ID of the road with specified index. E.g. if there are 4 roads, index 3 means the last one.
	@param index The index of the road
	@return The ID of the road
	*/
	RM_DLL_API int RM_GetIdOfRoadFromIndex(int index);

	/**
	Get the lenght of road with specified ID
	@param id The road ID 
	@return The length of the road if ID exists, else 0.0
	*/
	RM_DLL_API float RM_GetRoadLength(int id);

	/**
	Get the number of drivable lanes of specified road
	@param roadId The road ID
	@param s The distance along the road at what point to check number of lanes (which can vary along the road)
	@return The number of drivable lanes
	*/
	RM_DLL_API int RM_GetRoadNumberOfLanes(int roadId, float s);

	/**
	Get the ID of the lane given by index
	@param roadId The road ID
	@param laneIndex The index of the lane 
	@param s The distance along the road at what point to look up the lane ID
	@return The lane ID
	*/
	RM_DLL_API int RM_GetLaneIdByIndex(int roadId, int laneIndex, float s);

	/**
	Set position from road coordinates, world coordinates being calculated
	@param handle Handle to the position object
	@param roadId Road specifier
	@param laneId Lane specifier
	@param laneOffset Offset from lane center
	@param s Distance along the specified road
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_SetLanePosition(int handle, int roadId, int laneId, int laneOffset, float s);

	/**
	Set s (distance) part of a lane position, world coordinates being calculated
	@param handle Handle to the position object
	@param s Distance along the specified road
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_SetS(int handle, float s);

	/**
	Set position from world coordinates, road coordinates being calculated
	@param handle Handle to the position object
	@param x cartesian coordinate x value
	@param y cartesian coordinate y value
	@param z cartesian coordinate z value
	@param h rotation heading value
	@param p rotation pitch value
	@param r rotation roll value
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_SetWorldPosition(int handle, float x, float y, float z, float h, float p, float r);

	/**
	Set position from world X, Y and heading coordinates; Z, pitch and road coordinates being calculated
	@param handle Handle to the position object
	@param x cartesian coordinate x value
	@param y cartesian coordinate y value
	@param z cartesian coordinate z value
	@param h rotation heading value
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_SetWorldXYZHPosition(int handle, float x, float y, float z, float h);

	/**
	Move position forward along the road. Choose way randomly though any junctions.
	@param handle Handle to the position object
	@param ds distance (meter) to move
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_PositionMoveForward(int handle, float dist);

	/**
	Get the fields of the position of specified index
	@param handle Handle to the position object
	@param data Struct to fill in the values
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_GetPositionData(int handle, PositionData *data);

	// Driver model functions
	/**
	Get the location, in global coordinate system, of the point at a specified distance from starting position along the road ahead
	@param handle Handle to the position object from which to measure
	@param lookahead_distance The distance, along the road, to the point
	@param data Struct including all result values, see SteeringTargetData typedef
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_GetSteeringTarget(int handle, float lookahead_distance, SteeringTargetData *data);

#ifdef __cplusplus
}
#endif
