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
} RM_PositionData;

typedef struct
{
	float pos[3];			// position, in global coordinate system
	float heading;			// road heading 
	float pitch;			// road pitch 
	float roll;				// road roll
	float width;
	float curvature;
	float speed_limit;
} RM_RoadLaneInfo;

typedef struct
{
	RM_RoadLaneInfo road_lane_info; // Road info at probe location
	float relative_pos[3];          // probe position, relative vehicle (pivot position object) coordinate system
	float relative_h;		        // heading angle to steering target from and relatove to vehicle (pivot position)
} RM_RoadProbeInfo;

typedef struct
{
	float ds;				// delta s (longitudinal distance)
	float dt;				// delta t (lateral distance)
	int dLaneId;			// delta laneId (increasing left and decreasing to the right)
} RM_PositionDiff;

#ifdef __cplusplus
extern "C"
{
#endif

	RM_DLL_API int RM_Init(const char *odrFilename);

	RM_DLL_API int RM_Close();

	/**
	Specify logfile name, optionally including directory path
	examples: "../logfile.txt" "c:/tmp/esmini.log" "my.log"
	Set "" to disable logfile
	Note: Needs to be called prior to calling RM_Init()
	@param path Logfile path
	*/
	RM_DLL_API void RM_SetLogFilePath(const char* logFilePath);

	/**
	Create a position object
	@return Handle to the position object, to use for operations
	*/
	RM_DLL_API int RM_CreatePosition();

	/**
	Get the number of created position objects
	@return Number of created position objects
	*/
	RM_DLL_API int RM_GetNrOfPositions();

	/**
	Delete one or all position object(s)
	@param hande Handle to the position object. Set -1 to delete all.
	@return 0 if succesful, -1 if specified position(s) could not be deleted
	*/
	RM_DLL_API int RM_DeletePosition(int handle);

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
	@param align If true the heading will be reset to the lane driving direction (typically only at initialization)
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_SetLanePosition(int handle, int roadId, int laneId, float laneOffset, float s, bool align);

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
	@param h rotation heading value
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_SetWorldXYHPosition(int handle, float x, float y, float h);

	/**
	Set position from world X, Y, Z and heading coordinates; pitch and road coordinates being calculated
	Setting a Z value may have effect in mapping the position to the closest road, e.g. overpass
	@param handle Handle to the position object
	@param x cartesian coordinate x value
	@param y cartesian coordinate y value
	@param h rotation heading value
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_SetWorldXYHPosition(int handle, float x, float y, float h);

	/**
	Move position forward along the road. Choose way randomly though any junctions.
	@param handle Handle to the position object
	@param dist Distance (meter) to move
	@param strategy How to move in a junction where multiple route options appear, see Junction::JunctionStrategyType
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_PositionMoveForward(int handle, float dist, int strategy);

	/**
	Get the fields of the position of specified index
	@param handle Handle to the position object
	@param data Struct to fill in the values
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_GetPositionData(int handle, RM_PositionData *data);

	/**
	Retrieve current speed limit (at current road, s-value and lane) based on ODR type elements or nr of lanes
	@param handle Handle to the position object
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API float RM_GetSpeedLimit(int handle);

	/**
	Retrieve lane information from the position object (at current road, s-value and lane)
	@param handle Handle to the position object
	@param lookahead_distance The distance, along the road, to the point of interest
	@param data Struct including all result values, see RM_RoadLaneInfo typedef
	@param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_GetLaneInfo(int handle, float lookahead_distance, RM_RoadLaneInfo *data, int lookAheadMode);

	/**
	As RM_GetLaneInfo plus relative location of point of interest (probe) from current position
	@param handle Handle to the position object from which to measure
	@param lookahead_distance The distance, along the road to the probe (point of interest)
	@param data Struct including all result values, see RM_RoadProbeInfo typedef
	@param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum
	@return 0 if successful, -1 if not
	*/
	RM_DLL_API int RM_GetProbeInfo(int handle, float lookahead_distance, RM_RoadProbeInfo *data, int lookAheadMode);

	/**
	Find out the difference between two position objects, i.e. delta distance (long and lat) and delta laneId
	@param handleA Handle to the position object from which to measure
	@param handleB Handle to the position object to which the distance is measured
	@param pos_diff Struct including all result values, see PositionDiff typedef
	@return true if a valid path between the road positions was found and calculations could be performed
	*/
	RM_DLL_API bool RM_SubtractAFromB(int handleA, int handleB, RM_PositionDiff *pos_diff);


#ifdef __cplusplus
}
#endif
