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

#include "stdint.h"

#ifdef WIN32
#define RM_DLL_API __declspec(dllexport)
#else
#define RM_DLL_API  // Leave empty on Mac
#endif

typedef uint32_t id_t;

#define RM_ID_UNDEFINED 0xffffffff

typedef struct
{
    float x;
    float y;
    float z;
} RM_PositionXYZ;

typedef struct
{
    float x;
    float y;
    float z;
    float h;
    float p;
    float r;
    float hRelative;
    id_t  roadId;
    id_t  junctionId;  // -1 if not in a junction
    int   laneId;
    float laneOffset;
    float s;
} RM_PositionData;

typedef struct
{
    RM_PositionXYZ pos;      // position, in global coordinate system
    float          heading;  // road heading
    float          pitch;    // road pitch
    float          roll;     // road roll
    float          width;
    float          curvature;
    float          speed_limit;
    id_t           roadId;      // target position, road ID
    id_t           junctionId;  // target position, junction ID. -1 if not in a junction.
    int            laneId;      // target position, lane ID
    float          laneOffset;  // target position, lane offset (lateral distance from lane center)
    float          s;           // target position, s (longitudinal distance along reference line)
    float          t;           // target position, t (lateral distance from reference line)
} RM_RoadLaneInfo;

typedef struct
{
    RM_RoadLaneInfo road_lane_info;  // Road info at probe location
    RM_PositionXYZ  relative_pos;    // probe position, relative vehicle (pivot position object) coordinate system
    float           relative_h;      // heading angle to steering target from and relatove to vehicle (pivot position)
} RM_RoadProbeInfo;

typedef struct
{
    float ds;       // delta s (longitudinal distance)
    float dt;       // delta t (lateral distance)
    int   dLaneId;  // delta laneId (increasing left and decreasing to the right)
} RM_PositionDiff;

typedef struct
{
    int         id;           // just an unique identifier of the sign
    float       x;            // global x coordinate of sign position
    float       y;            // global y coordinate of sign position
    float       z;            // global z coordinate of sign position
    float       z_offset;     // z offset from road level
    float       h;            // global heading of sign orientation
    id_t        roadId;       // road id of sign road position
    float       s;            // longitudinal position along road
    float       t;            // lateral position from road reference line
    const char* name;         // sign name, typically used for 3D model filename
    int         orientation;  // 1=facing traffic in road direction, -1=facing traffic opposite road direction
    float       length;       // length as sepcified in OpenDRIVE
    float       height;       // height as sepcified in OpenDRIVE
    float       width;        // width as sepcified in OpenDRIVE
} RM_RoadSign;

typedef struct
{
    int fromLane;
    int toLane;
} RM_RoadObjValidity;

typedef struct
{
    float       a_;
    const char* axis_;
    float       b_;
    const char* ellps_;
    float       k_;
    float       k_0_;
    float       lat_0_;
    float       lon_0_;
    float       lon_wrap_;
    float       over_;
    const char* pm_;
    const char* proj_;
    const char* units_;
    const char* vunits_;
    float       x_0_;
    float       y_0_;
    const char* datum_;
    const char* geo_id_grids_;
    float       zone_;
    int         towgs84_;
} RM_GeoReference;

// Modes for interpret Z, Head, Pitch, Roll coordinate value as absolute or relative
// grouped as bitmask: 0000 => skip/use current, 0001=DEFAULT, 0011=ABS, 0111=REL
// example: Relative Z, Absolute H, Default R, Current P = RM_Z_REL | RM_H_ABS | RM_R_DEF = 4151 = 0001 0000 0011 0111
// Must match roadmanager::Position::PositionMode
typedef enum
{
    RM_Z_SET = 1,  // 0001
    RM_Z_DEF = 1,  // 0001
    RM_Z_ABS = 3,  // 0011
    RM_Z_REL = 7,  // 0111
    RM_H_SET = RM_Z_SET << 4,
    RM_H_DEF = RM_Z_DEF << 4,
    RM_H_ABS = RM_Z_ABS << 4,
    RM_H_REL = RM_Z_REL << 4,
    RM_P_SET = RM_Z_SET << 8,
    RM_P_DEF = RM_Z_DEF << 8,
    RM_P_ABS = RM_Z_ABS << 8,
    RM_P_REL = RM_Z_REL << 8,
    RM_R_SET = RM_Z_SET << 12,
    RM_R_DEF = RM_Z_DEF << 12,
    RM_R_ABS = RM_Z_ABS << 12,
    RM_R_REL = RM_Z_REL << 12,
} RM_PositionMode;

#ifdef __cplusplus
extern "C"
{
#endif

    RM_DLL_API int RM_Init(const char* odrFilename);

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
    @return Handle >= 0 to the position object to use for operations or -1 on error
    */
    RM_DLL_API int RM_CreatePosition();

    /**
    Get the number of created position objects
    @return Number of created position objects or -1 on error
    */
    RM_DLL_API int RM_GetNrOfPositions();

    /**
    Delete one or all position object(s)
    @param hande Handle to the position object. Set -1 to delete all.
    @return 0 if succesful, -1 if specified position(s) could not be deleted
    */
    RM_DLL_API int RM_DeletePosition(int handle);

    /**
    Copy a position object
    @param handle Handle to the original position object.
    @return handle to new position object. -1 if unsuccessful.
    */
    RM_DLL_API int RM_CopyPosition(int handle);

    /**
    Specify if and how position object will align to the road. The setting is done for individual components:
    Z (elevation), Heading, Pitch, Roll and separately for set- and update operation. Set operations represents
    when position is affected by API calls, e.g. updateObjectWorldPos(). Update operations represents when the
    position is updated implicitly by the scenarioengine, e.g. default controller moving a vehicle along the lane.
    @param hande Handle to the position object
    @param mode Bitmask combining values from roadmanager::PosMode enum
    example: To set relative z and absolute roll: (Z_REL | R_ABS) or (7 | 12288) = (7 + 12288) = 12295
    @param type Type of operations the setting applies to. SET (explicit set-functions) or UPDATE (updates by controllers),
    according to roadmanager::PosModeType
    */
    RM_DLL_API void RM_SetObjectPositionMode(int handle, int type, int mode);

    /**
    Set to default mode how position object will align to the road
    @param hande Handle to the position object
    @param type Type of operations the setting applies to. SET (explicit set-functions) or UPDATE (updates by controllers),
    according to roadmanager::PosModeType
    */
    RM_DLL_API void RM_SetObjectPositionModeDefault(int handle, int type);

    /**
    Specify which lane types the position object snaps to (is aware of)
    @param handle Handle to the position object
    @param laneTypes A combination (bitmask) of lane types according to roadmanager::Lane::LaneType
    examples: ANY_DRIVING = 1966082, ANY_ROAD = 1966214, ANY = -1
    @return 0 if successful, -1 if not
    */
    RM_DLL_API int RM_SetSnapLaneTypes(int handle, int laneTypes);

    /**
    Controls whether to keep lane ID regardless of lateral position or snap to closest lane (default)
    @parameter mode True=keep lane False=Snap to closest (default)
    @return 0 if successful, -1 if not
    */
    RM_DLL_API int RM_SetLockOnLane(int handle, bool mode);

    /**
    Get the total number fo roads in the road network of the currently loaded OpenDRIVE file.
    @return Number of roads, -1 indicates error e.g. no roadnetwork loaded
    */
    RM_DLL_API int RM_GetNumberOfRoads();

    /**
    Get the unit of specified speed (in OpenDRIVE road type element).
    All roads will be looped in search for such an element. First found will be used.
    If speed is specified withouth the optional unit, SI unit m/s is assumed.
    If no speed entries is found, undefined will be returned.
    @return -1=Error, 0=Undefined, 1=km/h 2=m/s, 3=mph
    */
    RM_DLL_API int RM_GetSpeedUnit();

    /**
    Get the Road ID of the road with specified index. E.g. if there are 4 roads, index 3 means the last one.
    @param index The index of the road
    @return The ID of the road, -1 on error
    */
    RM_DLL_API id_t RM_GetIdOfRoadFromIndex(int index);

    /**
    Get the lenght of road with specified ID
    @param id The road ID
    @return The length of the road if ID exists, else 0.0
    */
    RM_DLL_API float RM_GetRoadLength(id_t id);

    /**
    Get original string ID associated with specified road
    @param road_id The integer ID road
    @return string ID, empty string if not found
    */
    RM_DLL_API const char* RM_GetRoadIdString(id_t road_id);

    /**
    Get integer road ID associated with specified road string ID
    @param road_id_str The road string ID
    @return road ID, -1 if not found
    */
    RM_DLL_API id_t RM_GetRoadIdFromString(const char* road_id_str);

    /**
    Get original string ID associated with specified junction
    @param road_id The integer ID junction
    @return string ID, empty string if not found
    */
    RM_DLL_API const char* RM_GetJunctionIdString(id_t junction_id);

    /**
    Get integer junction ID associated with specified junction string ID
    @param road_id_str The junction string ID
    @return junction ID, -1 if not found
    */
    RM_DLL_API id_t RM_GetJunctionIdFromString(const char* junction_id_str);

    /**
    Get the number of drivable lanes of specified road
    @param roadId The road ID
    @param s The distance along the road at what point to check number of lanes (which can vary along the road)
    @return The number of drivable lanes, -1 indicates error e.g. no roadnetwork loaded
    */
    RM_DLL_API int RM_GetRoadNumberOfLanes(id_t roadId, float s);

    /**
    Get the number of roads overlapping the given position
    @param handle Handle to the position object
    @return Number of roads overlapping the given position
    */
    RM_DLL_API int RM_GetNumberOfRoadsOverlapping(int handle);

    /**
    Get the id of an overlapping road according to given position and index
    @param handle Handle to the position object
    @parameter index Index of the total returned by GetNumberOfRoadsOverlapping()
    @return Id of specified overlapping road
    */
    RM_DLL_API id_t RM_GetOverlappingRoadId(int handle, int index);

    /**
    Get the ID of the lane given by index
    @param roadId The road ID
    @param laneIndex The index of the lane
    @param s The distance along the road at what point to look up the lane ID
    @return The lane ID, -1 indicates error e.g. no roadnetwork loaded
    */
    RM_DLL_API int RM_GetLaneIdByIndex(id_t roadId, int laneIndex, float s);

    /**
    Set position from road coordinates, world coordinates being calculated
    @param handle Handle to the position object
    @param roadId Road specifier
    @param laneId Lane specifier
    @param laneOffset Offset from lane center
    @param s Distance along the specified road
    @param align If true the heading will be reset to the lane driving direction (typically only at initialization)
    @return >= 0 on success. For all codes see roadmanager.hpp::Position::enum class ReturnCode
    */
    RM_DLL_API int RM_SetLanePosition(int handle, id_t roadId, int laneId, float laneOffset, float s, bool align);

    /**
    Set s (distance) part of a lane position, world coordinates being calculated
    @param handle Handle to the position object
    @param s Distance along the specified road
    @return >= 0 on success. For all codes see roadmanager.hpp::Position::enum class ReturnCode
    */
    RM_DLL_API int RM_SetS(int handle, float s);

    /**
    Set position from world coordinates, road coordinates being calculated
    Any value set to std::nanf("") will be ignored/no change
    @param handle Handle to the position object
    @param x cartesian coordinate x value
    @param y cartesian coordinate y value
    @param z cartesian coordinate z value
    @param h rotation heading value
    @param p rotation pitch value
    @param r rotation roll value
    @return >= 0 on success. For all codes see roadmanager.hpp::Position::enum class ReturnCode
    */
    RM_DLL_API int RM_SetWorldPosition(int handle, float x, float y, float z, float h, float p, float r);

    /**
    Set position from world X, Y and heading coordinates; Z, pitch and road coordinates being calculated
    Any value set to std::nanf("") will be ignored/no change
    @param handle Handle to the position object
    @param x cartesian coordinate x value
    @param y cartesian coordinate y value
    @param h rotation heading value
    @return >= 0 on success. For all codes see roadmanager.hpp::Position::enum class ReturnCode
    */
    RM_DLL_API int RM_SetWorldXYHPosition(int handle, float x, float y, float h);

    /**
    Set position from world X, Y, Z and heading coordinates; pitch and road coordinates being calculated
    Any value set to std::nanf("") will be ignored/no change
    Setting a Z value may have effect in mapping the position to the closest road, e.g. overpass
    @param handle Handle to the position object
    @param x cartesian coordinate x value
    @param y cartesian coordinate y value
    @param h rotation heading value
    @return >= 0 on success. For all codes see roadmanager.hpp::Position::enum class ReturnCode
    */
    RM_DLL_API int RM_SetWorldXYZHPosition(int handle, float x, float y, float z, float h);

    /**
    Set position from world X, Y, Z and heading coordinates; pitch and road coordinates being calculated
    Any value set to std::nanf("") will be ignored/no change
    Setting a Z value may have effect in mapping the position to the closest road, e.g. overpass
    @param handle Handle to the position object
    @param x cartesian coordinate x value
    @param y cartesian coordinate y value
    @param z cartesian coordinate z value
    @param h rotation heading value
    @param p rotation pitch value
    @param r rotation roll value
    @param mode Bitmask specifying whether z, h, p, and r is absolute or relative road. See RM_PositionMode
    @return >= 0 on success. For all codes see roadmanager.hpp::Position::enum class ReturnCode
    */
    RM_DLL_API int RM_SetWorldPositionMode(int handle, float x, float y, float z, float h, float p, float r, int mode);

    /**
    Change road belonging of position object, keeping actual x,y location, regardless other roads being closer
    @param handle Handle to the position object
    @param roadId Id of the road to belong to
    @return >= 0 on success. For all codes see roadmanager.hpp::Position::enum class ReturnCode
    */
    RM_DLL_API int RM_SetRoadId(int handle, id_t roadId);

    /**
    Move position forward along the road. Choose way randomly though any junctions.
    @param handle Handle to the position object
    @param dist Distance (meter) to move
    @param junctionSelectorAngle Desired direction [0:2pi] from incoming road direction (angle = 0), set -1 to randomize
    @return >= 0 on success. For all codes see roadmanager.hpp::Position::enum class ReturnCode
    */
    RM_DLL_API int RM_PositionMoveForward(int handle, float dist, float junctionSelectorAngle);

    /**
    Get the fields of the position of specified index
    @param handle Handle to the position object
    @param data Struct to fill in the values
    @return 0 if successful, -1 if not
    */
    RM_DLL_API int RM_GetPositionData(int handle, RM_PositionData* data);

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
    @param inRoadDrivingDirection If true always look along primary driving direction. If false, look in most straightforward direction according to
    object heading.
    @return 0 if successful, -1 if not
    */
    RM_DLL_API int RM_GetLaneInfo(int handle, float lookahead_distance, RM_RoadLaneInfo* data, int lookAheadMode, bool inRoadDrivingDirection);

    /**
    As RM_GetLaneInfo plus relative location of point of interest (probe) from current position
    @param handle Handle to the position object from which to measure
    @param lookahead_distance The distance, along the road to the probe (point of interest)
    @param data Struct including all result values, see RM_RoadProbeInfo typedef
    @param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum
    @param inRoadDrivingDirection If true always look along primary driving direction. If false, look in most straightforward direction according to
    object heading.
    @return 0 if successful, 1 if probe reached end of road, 2 if end ouf route, -1 if some error
    */
    RM_DLL_API int RM_GetProbeInfo(int handle, float lookahead_distance, RM_RoadProbeInfo* data, int lookAheadMode, bool inRoadDrivingDirection);

    /**
    Get width of lane with specified lane id, at current longitudinal position
    @param handle Handle to the position object from which to measure
    @param lane_id Id of the lane to measure
    @return Lane width or 0.0 if lane does not exists or any other error
    */
    RM_DLL_API float RM_GetLaneWidth(int handle, int lane_id);

    /**
    Get width of lane with specified lane id, at specified road and longitudinal position
    @param road_id Id of the road
    @param lane_id Id of the lane to measure
    @param s Longitudinal position along the road
    @return Lane width or 0.0 if lane does not exists or any other error
    */
    RM_DLL_API float RM_GetLaneWidthByRoadId(id_t road_id, int lane_id, float s);

    /**
    Get type of lane with specified lane id, at current longitudinal position
    For valid types, see RoadManager.hpp::Lane::LaneType enum
    @param handle Handle to the position object from which to measure
    @param lane_id Id of the lane
    @return Lane type or 0 if lane does not exists or any other error
    */
    RM_DLL_API int RM_GetLaneType(int handle, int lane_id);

    /**
    Find out what lane type object is currently in, reference point projected on road
    Can be used for checking exact lane type or combinations by bitmask.
    Example 1: Check if on border lane: SE_GetObjectLaneType(id) == (1 << 6)
    Example 2: Check if on any drivable lane: SE_GetObjectLaneType(id) & 1966594
    Example 3: Check if on any road lane: SE_GetObjectLaneType(id) & 1966726
    Example 4: Check for no lane (outside defined lanes): SE_GetObjectLaneType(id) == 1
    @param handle Handle to the position object to check
    @return lane type according to enum roadmanager::Lane::LaneType
    */
    RM_DLL_API int RM_GetInLaneType(int handle);

    /**
    Get type of lane with specified lane id, at specified road and longitudinal position
    For valid types, see RoadManager.hpp::Lane::LaneType enum
    @param road_id Id of the road
    @param lane_id Id of the lane
    @param s Longitudinal position along the road
    @return Lane type or 0 if lane does not exists or any other error
    */
    RM_DLL_API int RM_GetLaneTypeByRoadId(id_t road_id, int lane_id, float s);

    /**
    Find out the difference between two position objects, i.e. delta distance (long and lat) and delta laneId
    @param handleA Handle to the position object from which to measure
    @param handleB Handle to the position object to which the distance is measured
    @param pos_diff Struct including all result values, see PositionDiff typedef
    @return 0 if successful, -2 if route between positions can't be found, -1 if some other error
    */
    RM_DLL_API int RM_SubtractAFromB(int handleA, int handleB, RM_PositionDiff* pos_diff);

    /**
    Get the number of road signs along specified road
    @param road_id The road along which to look for signs
    @return Number of road signs, -1 on error
    */
    RM_DLL_API int RM_GetNumberOfRoadSigns(id_t road_id);

    /**
            Get information on specifed road sign
            @param road_id The road of which to look for the sign
            @param index Index of the sign. Note: not ID
            @param road_sign Pointer/reference to a SE_RoadSign struct to be filled in
            @return 0 if successful, -1 if not
    */
    RM_DLL_API int RM_GetRoadSign(id_t road_id, int index, RM_RoadSign* road_sign);

    /**
            Get the number of lane validity records of specified road object/sign
            @param road_id The road of which to look for the sign
            @param index Index of the sign. Note: not ID
            @return Number of validity records of specified road sign, -1 if not
    */
    RM_DLL_API int RM_GetNumberOfRoadSignValidityRecords(id_t road_id, int index);

    /**
            Get specified validity record of specifed road sign
            @param road_id The road of which to look for the sign
            @param signIndex Index of the sign. Note: not ID
            @param validityIndex Index of the validity record
            @param road_sign Pointer/reference to a SE_RoadObjValidity struct to be filled in
            @return 0 if successful, -1 if not
    */
    RM_DLL_API int RM_GetRoadSignValidityRecord(id_t road_id, int signIndex, int validityIndex, RM_RoadObjValidity* validity);

    /**
            Get the xodr road file georeference
    */
    RM_DLL_API int RM_GetOpenDriveGeoReference(RM_GeoReference* rmGeoReference);

#ifdef __cplusplus
}
#endif
