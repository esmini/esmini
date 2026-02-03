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

/*
 * This module provides a generic C# interface/wrapper to the RoadManagerDLL library
 * simply mirroring the interface in terms of datatypes and functions
 */


using System;
using System.Runtime.InteropServices;

namespace OpenDRIVE
{
    [StructLayout(LayoutKind.Sequential), Serializable]
    public struct PositionXYZ
    {
        public float x;
        public float y;
        public float z;
    }

    [StructLayout(LayoutKind.Sequential), Serializable]
    public struct OpenDrivePositionData
    {
        public float x;
        public float y;
        public float z;
        public float h;
        public float p;
        public float r;
        public float hRelative;
        public int roadId;
        /// <summary>-1 if not in a junction</summary>
        public int junctionId;
        public int laneId;
        public float laneOffset;
        public float s;
    }

    [StructLayout(LayoutKind.Sequential), Serializable]
    public struct RoadLaneInfo
    {
        /// <summary>position, in global coordinate system</summary>
        public PositionXYZ pos;
        /// <summary>road heading</summary>
        public float heading;
        /// <summary>road pitch</summary>
        public float pitch;
        /// <summary>road roll</summary>
        public float roll;
        public float width;
        public float curvature;
        /// <summary>m/s</summary>
        public float speedLimit;
        /// <summary>target position, road ID</summary>
        public int roadId;
        /// <summary>target position, junction ID. -1 if not in a junction.</summary>
        public int junctionId;
        /// <summary>target position, lane ID</summary>
        public int laneId;
        /// <summary>target position, lane offset (lateral distance from lane center)</summary>
        public float laneOffset;
        /// <summary>target position, s (longitudinal distance along reference line)</summary>
        public float s;
        /// <summary>target position, t (lateral distance from reference line)</summary>
        public float t;
        /// <summary>road type given by OpenDRIVE type entry, maps to roadmanager::Road::RoadType</summary>
        public int roadType;
        /// <summary>road rule given by OpenDRIVE rule entry, maps to roadmanager::Road::RoadRule</summary>
        public int roadRule;
        /// <summary>lane type given by OpenDRIVE type entry, maps to roadmanager::Road::LaneType</summary>
        public int laneType;
    }

    [StructLayout(LayoutKind.Sequential), Serializable]
    public struct RoadProbeInfo
    {
        /// <summary>Road info at probe location</summary>
        public RoadLaneInfo laneInfo;
        /// <summary>probe position, relative vehicle (pivot position object) coordinate system</summary>
        public PositionXYZ relativePos;
        /// <summary>heading angle to steering target from and relative to vehicle (pivot position)</summary>
        public float relativeHeading;
    }

    [StructLayout(LayoutKind.Sequential), Serializable]
    public struct PositionDiff
    {
        /// <summary>delta s (longitudinal distance)</summary>
        public float ds;
        /// <summary>delta t (lateral distance)</summary>
        public float dt;
        /// <summary>delta laneId (increasing left and decreasing to the right)</summary>
        public int dLaneId;
    }

    [StructLayout(LayoutKind.Sequential), Serializable]
    public struct RoadSign
    {
        /// <summary>just an unique identifier of the sign</summary>
        public int id;
        /// <summary>global x coordinate of sign position</summary>
        public float x;
        /// <summary>global y coordinate of sign position</summary>
        public float y;
        /// <summary>global z coordinate of sign position</summary>
        public float z;
        /// <summary>z offset from road level</summary>
        public float zOffset;
        /// <summary>global heading of sign orientation</summary>
        public float h;
        /// <summary>road id of sign road position</summary>
        public int roadId;
        /// <summary>longitudinal position along road</summary>
        public float s;
        /// <summary>lateral position from road reference line</summary>
        public float t;
        /// <summary>sign name, typically used for 3D model filename</summary>
        public IntPtr name;
        /// <summary>1=facing traffic in road direction, -1=facing traffic opposite road direction</summary>
        public int orientation;
        /// <summary>length as specified in OpenDRIVE</summary>
        public float length;
        /// <summary>height as specified in OpenDRIVE</summary>
        public float height;
        /// <summary>width as specified in OpenDRIVE</summary>
        public float width;
    }

    [StructLayout(LayoutKind.Sequential), Serializable]
    public struct RoadObjValidity
    {
        public int fromLane;
        public int toLane;
    }

    [StructLayout(LayoutKind.Sequential), Serializable]
    public struct GeoReference
    {
        public float a;
        public IntPtr axis;
        public float b;
        public IntPtr ellps;
        public float k;
        public float k0;
        public float lat0;
        public float lon0;
        public float lonWrap;
        public float over;
        public IntPtr pm;
        public IntPtr proj;
        public IntPtr units;
        public IntPtr vunits;
        public float x0;
        public float y0;
        public IntPtr datum;
        public IntPtr geoIdGrids;
        public float zone;
        public int towgs84;
        public IntPtr originalGeorefStr;
    }

    public enum PositionMode
    {
        ZSet = 1,
        ZDefault = 1,
        ZAbs = 3,
        ZRel = 7,
        ZMask = 7,
        HSet = 16,
        HDefault = 16,
        HAbs = 48,
        HRel = 112,
        HMask = 112,
        PSet = 256,
        PDefault = 256,
        PAbs = 768,
        PRel = 1792,
        PMask = 1792,
        RSet = 4096,
        RDefault = 4096,
        RAbs = 12288,
        RRel = 28672,
        RMask = 28672,
    }

    public static class RoadManagerLibraryCS
    {
#if UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
        private const string LIB_NAME = "libesminiRMLib.so";
#else
        private const string LIB_NAME = "esminiRMLib";
#endif

        /// <summary>
        /// * Initialize the RoadManager library with specified OpenDRIVE file /
        /// </summary>
        /// <param name="odrFilename">Path to OpenDRIVE file</param>
        /// <returns>0 on success, -1 on failure</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_Init")]
        public static extern int Init(string odrFilename);

        /// <summary>
        /// * Initialize the RoadManager library with specified OpenDRIVE XML string /
        /// </summary>
        /// <param name="odrAsXMLString">xml content</param>
        /// <returns>0 on success, -1 on failure</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_InitWithString")]
        public static extern int InitWithString(string odrAsXMLString);

        /// <summary>
        /// Close the Road Manager and clean up resources
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_Close")]
        public static extern int Close();

        /// <summary>
        /// * Specify logfile name, optionally including directory path examples: "../logfile.txt" "c:/tmp/esmini.log" "my.log" Set "" to disable logfile Note: Needs to be called prior to calling RM_Init() /
        /// </summary>
        /// <param name="logFilePath">Parameter logFilePath</param>
        /// <returns>No return value</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetLogFilePath")]
        public static extern void SetLogFilePath(string logFilePath);

        /// <summary>
        /// * Create a position object /
        /// </summary>
        /// <returns>Handle >= 0 to the position object to use for operations or -1 on error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_CreatePosition")]
        public static extern int CreatePosition();

        /// <summary>
        /// * Get the number of created position objects /
        /// </summary>
        /// <returns>Number of created position objects or -1 on error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNrOfPositions")]
        public static extern int GetNrOfPositions();

        /// <summary>
        /// * Delete one or all position object(s) /
        /// </summary>
        /// <param name="handle">Parameter handle</param>
        /// <returns>0 if succesful, -1 if specified position(s) could not be deleted</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_DeletePosition")]
        public static extern int DeletePosition(int handle);

        /// <summary>
        /// * Copy a position object /
        /// </summary>
        /// <param name="handle">Handle to the original position object.</param>
        /// <returns>handle to new position object. -1 if unsuccessful.</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_CopyPosition")]
        public static extern int CopyPosition(int handle);

        /// <summary>
        /// * Specify if and how position object will align to the road.
        /// </summary>
        /// <param name="handle">Parameter handle</param>
        /// <param name="type">Type of operations the setting applies to. SET (explicit set-functions) or UPDATE (updates by controllers),</param>
        /// <param name="mode">Bitmask combining values from roadmanager::Position::PosMode enum</param>
        /// <returns>No return value</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetObjectPositionMode")]
        public static extern void SetObjectPositionMode(int handle, int type, int mode);

        /// <summary>
        /// * Set to default mode how position object will align to the road according to roadmanager::Position::PosModeType /
        /// </summary>
        /// <param name="handle">Parameter handle</param>
        /// <param name="type">Type of operations the setting applies to. SET (explicit set-functions) or UPDATE (updates by controllers),</param>
        /// <returns>No return value</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetObjectPositionModeDefault")]
        public static extern void SetObjectPositionModeDefault(int handle, int type);

        /// <summary>
        /// * Specify which lane types the position object snaps to (is aware of) examples: ANY_DRIVING = 1966594, ANY_ROAD = 1966990, ANY = -1 /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="laneTypes">A combination (bitmask) of lane types according to roadmanager::Lane::LaneType</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetSnapLaneTypes")]
        public static extern int SetSnapLaneTypes(int handle, int laneTypes);

        /// <summary>
        /// * Controls whether to keep lane ID regardless of lateral position or snap to closest lane (default) /
        /// </summary>
        /// <param name="handle">Parameter handle</param>
        /// <param name="mode">Parameter mode</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetLockOnLane")]
        public static extern int SetLockOnLane(int handle, bool mode);

        /// <summary>
        /// * Get the total number fo roads in the road network of the currently loaded OpenDRIVE file. /
        /// </summary>
        /// <returns>Number of roads, -1 indicates error e.g. no roadnetwork loaded</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNumberOfRoads")]
        public static extern int GetNumberOfRoads();

        /// <summary>
        /// * Get the unit of specified speed (in OpenDRIVE road type element).
        /// </summary>
        /// <returns>-1=Error, 0=Undefined, 1=km/h 2=m/s, 3=mph</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetSpeedUnit")]
        public static extern int GetSpeedUnit();

        /// <summary>
        /// * Get the Road ID of the road with specified index. E.g. if there are 4 roads, index 3 means the last one. /
        /// </summary>
        /// <param name="index">The index of the road</param>
        /// <returns>The ID of the road, -1 on error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetIdOfRoadFromIndex")]
        public static extern int GetIdOfRoadFromIndex(uint index);

        /// <summary>
        /// * Get the lenght of road with specified ID /
        /// </summary>
        /// <param name="id">The road ID</param>
        /// <returns>The length of the road if ID exists, else 0.0</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadLength")]
        public static extern float GetRoadLength(int id);

        /// <summary>
        /// * Get original string ID associated with specified road /
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <returns>string ID, empty string if not found</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadIdString")]
        public static extern string GetRoadIdString(int roadId);

        /// <summary>
        /// * Get integer road ID associated with specified road string ID /
        /// </summary>
        /// <param name="roadIdStr">Parameter roadIdStr</param>
        /// <returns>road ID, -1 if not found</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadIdFromString")]
        public static extern int GetRoadIdFromString(string roadIdStr);

        /// <summary>
        /// * Get original string ID associated with specified junction /
        /// </summary>
        /// <param name="junctionId">Parameter junctionId</param>
        /// <returns>string ID, empty string if not found</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetJunctionIdString")]
        public static extern string GetJunctionIdString(int junctionId);

        /// <summary>
        /// * Get integer junction ID associated with specified junction string ID /
        /// </summary>
        /// <param name="junctionIdStr">Parameter junctionIdStr</param>
        /// <returns>junction ID, -1 if not found</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetJunctionIdFromString")]
        public static extern int GetJunctionIdFromString(string junctionIdStr);

        /// <summary>
        /// * Get the number of lanes of given type for given road. Reference lane will be included if matching the type. /
        /// </summary>
        /// <param name="roadId">The road ID</param>
        /// <param name="s">The distance along the road at what point to check number of lanes (which can vary along the road)</param>
        /// <param name="typeMask">Parameter typeMask</param>
        /// <returns>The number of matched lanes, -1 indicates error, e.g. no roadnetwork loaded</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadNumberOfLanes")]
        public static extern int GetRoadNumberOfLanes(int roadId, float s, int typeMask);

        /// <summary>
        /// * Get ID of the lane given by index and type. Reference lane will be included if matching the type. /
        /// </summary>
        /// <param name="roadId">The road ID</param>
        /// <param name="laneIndex">The index of the lane</param>
        /// <param name="s">The distance along the road at what point to look up the lane ID</param>
        /// <param name="typeMask">Parameter typeMask</param>
        /// <param name="laneId">Lane specifier</param>
        /// <returns>0 on success, -1 indicates error e.g. no roadnetwork loaded or index out of range</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneIdByIndex")]
        public static extern int GetLaneIdByIndex(int roadId, int laneIndex, float s, int typeMask, out int laneId);

        /// <summary>
        /// * Get the number of drivable lanes of given road (like RM_GetRoadNumberOfLanes with type_mask for any drivable) /
        /// </summary>
        /// <param name="roadId">The road ID</param>
        /// <param name="s">The distance along the road at what point to check number of lanes (which can vary along the road)</param>
        /// <returns>The number of lanes, -1 indicates error, e.g. no roadnetwork loaded</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadNumberOfDrivableLanes")]
        public static extern int GetRoadNumberOfDrivableLanes(int roadId, float s);

        /// <summary>
        /// * Get ID of the drivable lane given by index (like RM_GetLaneIdByIndex with type_mask for any drivable) /
        /// </summary>
        /// <param name="roadId">The road ID</param>
        /// <param name="laneIndex">The index of the lane</param>
        /// <param name="s">The distance along the road at what point to look up the lane ID</param>
        /// <param name="laneId">Lane specifier</param>
        /// <returns>0 on success, -1 indicates error e.g. no roadnetwork loaded or index out of range</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetDrivableLaneIdByIndex")]
        public static extern int GetDrivableLaneIdByIndex(int roadId, int laneIndex, float s, out int laneId);

        /// <summary>
        /// * Get the number of roads overlapping the given position /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <returns>Number of roads overlapping the given position</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNumberOfRoadsOverlapping")]
        public static extern int GetNumberOfRoadsOverlapping(int handle);

        /// <summary>
        /// * Get the id of an overlapping road according to given position and index /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="index">Handle to the position object</param>
        /// <returns>Id of specified overlapping road</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetOverlappingRoadId")]
        public static extern int GetOverlappingRoadId(int handle, uint index);

        /// <summary>
        /// * Set position from road coordinates, world coordinates being calculated /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="roadId">Road specifier</param>
        /// <param name="laneId">Lane specifier</param>
        /// <param name="laneOffset">Offset from lane center</param>
        /// <param name="s">Distance along the specified road</param>
        /// <param name="align">If true the heading will be reset to the lane driving direction (typically only at initialization)</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetLanePosition")]
        public static extern int SetLanePosition(int handle, int roadId, int laneId, float laneOffset, float s, bool align);

        /// <summary>
        /// * Set position from road coordinates, world coordinates being calculated /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="roadId">Road specifier</param>
        /// <param name="s">Distance along the specified road</param>
        /// <param name="t">Lateral position in the road s/t coordinate system</param>
        /// <param name="align">If true the heading will be reset to the lane driving direction (typically only at initialization)</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetRoadPosition")]
        public static extern int SetRoadPosition(int handle, int roadId, float s, float t, bool align);

        /// <summary>
        /// * Set s (distance) part of a lane position, world coordinates being calculated /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="s">Distance along the specified road</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetS")]
        public static extern int SetS(int handle, float s);

        /// <summary>
        /// * Set position from world coordinates, road coordinates being calculated Any value set to std::nanf("") will be ignored/no change /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="z">cartesian coordinate z value</param>
        /// <param name="h">andle Handle to the position object</param>
        /// <param name="p">aram p rotation pitch value</param>
        /// <param name="r">am r rotation roll value</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetWorldPosition")]
        public static extern int SetWorldPosition(int handle, float x, float y, float z, float h, float p, float r);

        /// <summary>
        /// * Set position from world X, Y and heading coordinates; Z, pitch and road coordinates being calculated Any value set to std::nanf("") will be ignored/no change /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="h">andle Handle to the position object</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetWorldXYHPosition")]
        public static extern int SetWorldXYHPosition(int handle, float x, float y, float h);

        /// <summary>
        /// * Set position from world X, Y, Z and heading coordinates; pitch and road coordinates being calculated Any value set to std::nanf("") will be ignored/no change Setting a Z value may have effect in map...
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="z">Parameter z</param>
        /// <param name="h">andle Handle to the position object</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetWorldXYZHPosition")]
        public static extern int SetWorldXYZHPosition(int handle, float x, float y, float z, float h);

        /// <summary>
        /// * Set position from world X, Y, Z and heading coordinates; pitch and road coordinates being calculated Any value set to std::nanf("") will be ignored/no change Setting a Z value may have effect in map...
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="z">cartesian coordinate z value</param>
        /// <param name="h">andle Handle to the position object</param>
        /// <param name="p">aram p rotation pitch value</param>
        /// <param name="r">am r rotation roll value</param>
        /// <param name="mode">Bitmask specifying whether z, h, p, and r is absolute or relative road. See RM_PositionMode</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetWorldPositionMode")]
        public static extern int SetWorldPositionMode(int handle, float x, float y, float z, float h, float p, float r, int mode);

        /// <summary>
        /// * Set heading (yaw), mode (relative/absolute) given by current setting for the object /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="h">andle Handle to the position object</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetH")]
        public static extern int SetH(int handle, float h);

        /// <summary>
        /// * Set heading (yaw), mode (relative/absolute) given by argument /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="h">andle Handle to the position object</param>
        /// <param name="mode">RM_H_ABS or RM_H_REL, see RM_PositionMode</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetHMode")]
        public static extern int SetHMode(int handle, float h, int mode);

        /// <summary>
        /// * Change road belonging of position object, keeping actual x,y location, regardless other roads being closer /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="roadId">Id of the road to belong to</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetRoadId")]
        public static extern int SetRoadId(int handle, int roadId);

        /// <summary>
        /// * Move position forward along the road. Choose way randomly though any junctions. /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="dist">Distance (meter) to move</param>
        /// <param name="junctionSelectorAngle">Target direction from incoming road (angle = 0), e.g. pi/2=right pi=straight 3pi/2=left -1=randomize</param>
        /// <returns>>= 0 on success. For all codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_PositionMoveForward")]
        public static extern int PositionMoveForward(int handle, float dist, float junctionSelectorAngle);

        /// <summary>
        /// * Get the fields of the position of specified index /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="data">Struct to fill in the values</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetPositionData")]
        public static extern int GetPositionData(int handle, ref OpenDrivePositionData data);

        /// <summary>
        /// * Retrieve current speed limit (at current road, s-value and lane) based on ODR type elements or nr of lanes /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetSpeedLimit")]
        public static extern float GetSpeedLimit(int handle);

        /// <summary>
        /// * Retrieve lane information from the position object (at current road, s-value and lane) object heading. /
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="lookaheadDistance">Parameter lookaheadDistance</param>
        /// <param name="data">Struct including all result values, see RM_RoadLaneInfo typedef</param>
        /// <param name="lookAheadMode">Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum</param>
        /// <param name="inRoadDrivingDirection">If true always look along primary driving direction. If false, look in most straightforward direction according to</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneInfo")]
        public static extern int GetLaneInfo(int handle, float lookaheadDistance, ref RoadLaneInfo data, int lookAheadMode, bool inRoadDrivingDirection);

        /// <summary>
        /// * As RM_GetLaneInfo plus relative location of point of interest (probe) from current position object heading. /
        /// </summary>
        /// <param name="handle">Handle to the position object from which to measure</param>
        /// <param name="lookaheadDistance">Parameter lookaheadDistance</param>
        /// <param name="data">Struct including all result values, see RM_RoadProbeInfo typedef</param>
        /// <param name="lookAheadMode">Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum</param>
        /// <param name="inRoadDrivingDirection">If true always look along primary driving direction. If false, look in most straightforward direction according to</param>
        /// <returns>0 if successful, other codes see roadmanager::Position::ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetProbeInfo")]
        public static extern int GetProbeInfo(int handle, float lookaheadDistance, ref RoadProbeInfo data, int lookAheadMode, bool inRoadDrivingDirection);

        /// <summary>
        /// * Get width of lane with specified lane id, at current longitudinal position /
        /// </summary>
        /// <param name="handle">Handle to the position object from which to measure</param>
        /// <param name="laneId">Lane specifier</param>
        /// <param name="width">Parameter width</param>
        /// <returns>0 on success, -1 on any error, e.g. lane with specified id is missing</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneWidth")]
        public static extern int GetLaneWidth(int handle, int laneId, out float width);

        /// <summary>
        /// * Get width of lane with specified lane id, at specified road and longitudinal position /
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="laneId">Lane specifier</param>
        /// <param name="s">Longitudinal position along the road</param>
        /// <param name="width">Pointer to store the lane width</param>
        /// <returns>0 on success, -1 on any error, e.g. specified road is missing</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneWidthByRoadId")]
        public static extern int GetLaneWidthByRoadId(int roadId, int laneId, float s, out float width);

        /// <summary>
        /// * Get type of lane with specified lane id, at current longitudinal position For valid types, see RoadManager.hpp::Lane::LaneType enum /
        /// </summary>
        /// <param name="handle">Handle to the position object from which to measure</param>
        /// <param name="laneId">Lane specifier</param>
        /// <returns>Lane type or 0 if lane does not exists or any other error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneType")]
        public static extern int GetLaneType(int handle, int laneId);

        /// <summary>
        /// * Find out what lane type object is currently in, reference point projected on road Can be used for checking exact lane type or combinations by bitmask.
        /// </summary>
        /// <param name="handle">Handle to the position object to check</param>
        /// <returns>lane type according to enum roadmanager::Lane::LaneType</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetInLaneType")]
        public static extern int GetInLaneType(int handle);

        /// <summary>
        /// * Get type of lane with specified lane id, at specified road and longitudinal position For valid types, see RoadManager::Lane::LaneType enum /
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="laneId">Lane specifier</param>
        /// <param name="s">Longitudinal position along the road</param>
        /// <returns>Lane type or 0 if lane does not exists or any other error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneTypeByRoadId")]
        public static extern int GetLaneTypeByRoadId(int roadId, int laneId, float s);

        /// <summary>
        /// * Find out the difference between two position objects, i.e. delta distance (long and lat) and delta laneId /
        /// </summary>
        /// <param name="handleA">Handle to the position object from which to measure</param>
        /// <param name="handleB">Handle to the position object to which the distance is measured</param>
        /// <param name="posDiff">Parameter posDiff</param>
        /// <returns>0 if successful, -2 if route between positions can't be found, -1 if some other error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SubtractAFromB")]
        public static extern int SubtractAFromB(int handleA, int handleB, ref PositionDiff posDiff);

        /// <summary>
        /// * Get the number of road signs along specified road /
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <returns>Number of road signs, -1 on error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNumberOfRoadSigns")]
        public static extern int GetNumberOfRoadSigns(int roadId);

        /// <summary>
        /// * Get information on specifed road sign /
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="index">Index of the sign. Note: not ID</param>
        /// <param name="roadSign">Parameter roadSign</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadSign")]
        public static extern int GetRoadSign(int roadId, uint index, ref RoadSign roadSign);

        /// <summary>
        /// * Get the number of lane validity records of specified road object/sign /
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="index">Index of the sign. Note: not ID</param>
        /// <returns>Number of validity records of specified road sign, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNumberOfRoadSignValidityRecords")]
        public static extern int GetNumberOfRoadSignValidityRecords(int roadId, uint index);

        /// <summary>
        /// * Get specified validity record of specifed road sign /
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="signIndex">Index of the sign. Note: not ID</param>
        /// <param name="validityIndex">Index of the validity record</param>
        /// <param name="validity">Index Index of the validity record</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadSignValidityRecord")]
        public static extern int GetRoadSignValidityRecord(int roadId, uint signIndex, uint validityIndex, ref RoadObjValidity validity);

        /// <summary>
        /// * Get the xodr road file georeference /
        /// </summary>
        /// <param name="rmGeoReference">Parameter rmGeoReference</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetOpenDriveGeoReference")]
        public static extern int GetOpenDriveGeoReference(ref GeoReference rmGeoReference);

        /// <summary>
        /// * Set option. The option will be unset on next scenario run. If persistence is required check SE_SetOptionPersistent. /
        /// </summary>
        /// <param name="name">the name of the option to be set</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetOption")]
        public static extern int SetOption(string name);

        /// <summary>
        /// * Unset option /
        /// </summary>
        /// <param name="name">the name of the option to be unset</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_UnsetOption")]
        public static extern int UnsetOption(string name);

        /// <summary>
        /// * Set option value. The option's value will be unset on next scenario run. If persistence is required check SE_SetOptionValuePersistent /
        /// </summary>
        /// <param name="name">the name of the option to be set</param>
        /// <param name="value">the value to assign to the option</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetOptionValue")]
        public static extern int SetOptionValue(string name, string value);

        /// <summary>
        /// * Set option persistently. The option will remain over multiple scenario runs, until lib is reloaded. /
        /// </summary>
        /// <param name="name">the name of the option to be set</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetOptionPersistent")]
        public static extern int SetOptionPersistent(string name);

        /// <summary>
        /// * Set option value persistently. The option value will remain over multiple scenario runs, until lib is reloaded. /
        /// </summary>
        /// <param name="name">the name of the option to be set</param>
        /// <param name="value">the value to assign to the option</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetOptionValuePersistent")]
        public static extern int SetOptionValuePersistent(string name, string value);

        /// <summary>
        /// * Get option value /
        /// </summary>
        /// <param name="name">the name of the option whose value to fetch</param>
        /// <returns>value of the option</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetOptionValue")]
        public static extern string GetOptionValue(string name);

        /// <summary>
        /// * Get option set status /
        /// </summary>
        /// <param name="name">is the name of the option whose value is fetch</param>
        /// <returns>Returns true if the option is set otherwise false</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetOptionSet")]
        public static extern bool GetOptionSet(string name);

    }
}
