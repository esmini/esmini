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


using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;


namespace OpenDRIVE
{

    [StructLayout(LayoutKind.Sequential)]
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
        public int junctionId;         // -1 if not in a junction
        public int laneId;
        public float laneOffset;
        public float s;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct PositionXYZ
    {
        public float x;
        public float y;
        public float z;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct RoadLaneInfo
    {
        public PositionXYZ pos;        // position, in global coordinate system
        public float heading;          // road heading
        public float pitch;            // road pitch
        public float roll;			   // road roll
        public float width;            // Lane width
        public float curvature;        // curvature (1/radius), >0 for left curves, <0 for right curves
        public float speed_limit;      // road speed limit
        public int roadId;             // road ID
        public int junctionId;         // junction ID. -1 if not in a junction.
        public int laneId;             // lane ID
        public float laneOffset;       // lane offset (lateral distance from lane center)
        public float s;                // s (longitudinal distance along reference line)
        public float t;                // t (lateral distance from reference line)
    };

    public struct RoadProbeInfo
    {
        public RoadLaneInfo laneInfo;    // Road info at probe location
        public PositionXYZ relativePos;  // probe position, relative vehicle (pivot position object) coordinate system
        public float relativeHeading;    // heading angle to steering target from and relatove to vehicle (pivot position)
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PositionDiff
    {
        public float ds;                    // delta s (longitudinal distance)
        public float dt;                    // delta t (lateral distance)
        public int dLaneId;			        // delta laneId (increasing left and decreasing to the right)
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct RoadSign
    {
        public int id;            // just an unique identifier of the sign
        public float x;           // global x coordinate of sign position
        public float y;           // global y coordinate of sign position
        public float z;           // global z coordinate of sign position
        public float z_offset;    // z offset from road level
        public float h;           // global heading of sign orientation
        public int roadId;        // road id of sign road position
        public float s;           // longitudinal position along road
        public float t;           // lateral position from road reference line
        public IntPtr name;       // sign name, typically used for 3D model filename (Use with: Marshal.PtrToStringAnsi(SE_GetODRFilename())
        public int orientation;   // 1=facing traffic in road direction, -1=facing traffic opposite road direction
        public float length;      // length as sepcified in OpenDRIVE
        public float height;      // height as sepcified in OpenDRIVE
        public float width;       // width as sepcified in OpenDRIVE
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct GeoReference
    {
        // doc reference: https://proj.org/usage/projections.html
        public float a;         // Semimajor radius of the ellipsoid axis
        public float axis;      // Axis orientation
        public float b;         // Semiminor radius of the ellipsoid axis
        public IntPtr ellps;    // Ellipsoid name
        public float k;         // Scaling factor (deprecated)
        public float k_0;       // Scaling factor
        public float lat_0;     // Latitude of origin
        public float lon_0;     // Central meridian
        public float lon_wrap;  // Center longitude to use for wrapping
        public float over;      // Allow longitude output outside -180 to 180 range, disables wrapping (see below)
        public IntPtr pm;       // Alternate prime meridian (typically a city name, see below)
        public IntPtr proj;     // Projection name
        public IntPtr units;    // meters, US survey feet, etc.
        public IntPtr vunits;   // vertical units.
        public float x_0;       // False easting
        public float y_0;       // False northing

        public IntPtr datum;
        public IntPtr geo_id_grids;
        public float zone;
        public int towgs84;

    }


    public static class RoadManagerLibraryCS
    {
#if UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
        private const string LIB_NAME = "libesminiRMLib.so";
#else
        private const string LIB_NAME = "esminiRMLib";
#endif

        /// <summary>Initialize the OpenDRIVE utility manager</summary>
        /// <param name="odrFilename">OpenDRIVE file name</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_Init")]
        public static extern int Init(string odrFilename);

        /// <summary>Close down the OpenDRIVE utility manager respectfully</summary>
        [DllImport(LIB_NAME, EntryPoint = "RM_Close")]
        public static extern void Close();

        /// <summary>Specify logfile name, optionally including directory path
        /// examples: "../logfile.txt" "c:/tmp/esmini.log" "my.log"
        /// Set "" to disable logfile
        /// Note: Needs to be called prior to calling RM_Init() </summary>
        /// <param name="path">Logfile path</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetLogFilePath")]
        public static extern int SetLogFilePath(string path);

        /// <summary>Create a position object</summary>
        /// <returns>Handle >= 0 to the position object to use for operations or -1 on error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_CreatePosition")]
        public static extern int CreatePosition();

        /// <summary>Get the number of created position objects</summary>
        /// <returns>Number of created position objects or -1 on error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNrOfPositions")]
        public static extern int GetNrOfPositions(int index);

        /// <summary>Delete one or all position object(s)</summary>
        /// <param name="handle">Handle to the position object. Set -1 to delete all.</param>
        /// <returns>0 if successful, -1 if specified position(s) could not be deleted</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_DeletePosition")]
        public static extern int DeletePosition(int handle);

        /// <summary>Copy a position object</summary>
        /// <param name="handle">Handle Handle to the original position object.</param>
        /// <returns>Handle to new position object. -1 if unsuccessful.</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_DeletePosition")]
        public static extern int CopyPosition(int handle);

        /// <summary>Specify if and how position object will align to the road. This version
        /// sets same mode for all components: Heading, Pitch, Roll and Z (elevation)</summary>
        /// <param name="mode">@param mode as defined by roadmanager::Position::ALIGN_MODE:
        /// 0 = ALIGN_NONE // No alignment to road
        /// 1 = ALIGN_SOFT // Align to road but add relative orientation
        /// 2 = ALIGN_HARD // Completely align to road, disregard relative orientation </param>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetAlignMode")]
        public static extern void RM_SetAlignMode(int handle, int mode);

        /// <summary>Specify if and how position object will align to the road. This version
        /// sets same mode for only heading component.</summary>
        /// <param name="mode">@param mode as defined by roadmanager::Position::ALIGN_MODE:
        /// 0 = ALIGN_NONE // No alignment to road
        /// 1 = ALIGN_SOFT // Align to road but add relative orientation
        /// 2 = ALIGN_HARD // Completely align to road, disregard relative orientation </param>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetAlignModeH")]
        public static extern void RM_SetAlignModeH(int handle, int mode);

        /// <summary>Specify if and how position object will align to the road. This version
        /// sets same mode for only pitch component.</summary>
        /// <param name="mode">@param mode as defined by roadmanager::Position::ALIGN_MODE:
        /// 0 = ALIGN_NONE // No alignment to road
        /// 1 = ALIGN_SOFT // Align to road but add relative orientation
        /// 2 = ALIGN_HARD // Completely align to road, disregard relative orientation </param>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetAlignModeP")]
        public static extern void RM_SetAlignModeP(int handle, int mode);

        /// <summary>Specify if and how position object will align to the road. This version
        /// sets same mode for only roll component.</summary>
        /// <param name="mode">@param mode as defined by roadmanager::Position::ALIGN_MODE:
        /// 0 = ALIGN_NONE // No alignment to road
        /// 1 = ALIGN_SOFT // Align to road but add relative orientation
        /// 2 = ALIGN_HARD // Completely align to road, disregard relative orientation </param>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetAlignModeR")]
        public static extern void RM_SetAlignModeR(int handle, int mode);

        /// <summary>Specify if and how position object will align to the road. This version
        /// sets same mode for only Z (elevation) component.</summary>
        /// <param name="mode">@param mode as defined by roadmanager::Position::ALIGN_MODE:
        /// 0 = ALIGN_NONE // No alignment to road
        /// 1 = ALIGN_SOFT // Align to road but add relative orientation
        /// 2 = ALIGN_HARD // Completely align to road, disregard relative orientation </param>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetAlignModeZ")]
        public static extern void RM_SetAlignModeZ(int handle, int mode);

        /// <summary>Controls whether to keep lane ID regardless of lateral position or snap to closest lane (default)</summary>
        /// <param name="handle">Handle Handle to the original position object.</param>
        /// <param name="mode">True=keep lane False=Snap to closest (default)</param>
        /// <returns>Handle to new position object. -1 if unsuccessful.</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetLockOnLane")]
        public static extern int SetLockOnLane(int handle, bool mode);

        /// <summaryGet the total number fo roads in the road network of the currently loaded OpenDRIVE file</summary>
        /// <returns>Number of roads, -1 indicates error e.g. no roadnetwork loaded</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNumberOfRoads")]
        public static extern int GetNumberOfRoads();

        /// <summaryGet the unit of specified speed</summary>
        /// All roads will be looped in search for such an element. First found will be used.
        /// If speed is specified withouth the optional unit, SI unit m/s is assumed.
        /// If no speed entries is found, undefined will be returned.
        /// <returns>-1=Error, 0=Undefined, 1=km/h 2=m/s, 3=mph</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetSpeedUnit")]
        public static extern int GetSpeedUnit();

        /// <summary>
        /// Get the road ID, as specified in the OpenDRIVE description, of the road with specified index. E.g. if there are 4 roads, index 3 means the last one.
        /// </summary>
        /// <param name="index">The index of the road</param>
        /// <returns>The OpenDRIVE ID of the road</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetIdOfRoadFromIndex")]
        public static extern int GetIdOfRoadFromIndex(int index);

        /// <summary>
        /// Get the lenght of road with specified ID
        /// </summary>
        /// <param name="id">The OpenDRIVE road ID</param>
        /// <returns> The length of the road if ID exists, else 0.0</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadLength")]
        public static extern float GetRoadLength(int id);

        /// <summary>
        /// Get the number of drivable lanes of specified road
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="s">The distance along the road at what point to check number of lanes (which can vary along the road)</param>
        /// <returns>The number of drivable lanes, -1 indicates error e.g. no roadnetwork loaded</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadNumberOfLanes")]
        public static extern int GetRoadNumberOfLanes(int roadId, float s);

        /// <summary>
        /// Get the number of roads overlapping the given position
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <returns>Number of roads overlapping the given position</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNumberOfRoadsOverlapping")]
        public static extern int GetNumberOfRoadsOverlapping(int handle);

        /// <summary>
        /// Get the id of an overlapping road according to given position and index
        /// </summary>
        /// <param name="handle">Handle to the position object</param>
        /// <param name="index">Index of the total returned by GetNumberOfRoadsOverlapping()</param>
        /// <returns>Id of specified overlapping road</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetOverlappingRoadId")]
        public static extern int GetOverlappingRoadId(int handle, int index);

        /// <summary>
        /// Get the OpenDRIVE ID of the lane given by index
        /// </summary>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="laneIndex">The index of the lane </param>
        /// <param name="s">The distance along the road at what point to look up the lane ID</param>
        /// <returns>The lane ID - as specified in the OpenDRIVE description, -1 on error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneIdByIndex")]
        public static extern int GetLaneIdByIndex(int roadId, int laneIndex, float s);

        /// <summary>
        /// Set position from road coordinates, world coordinates being calculated
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="roadId">The OpenDRIVE road ID</param>
        /// <param name="laneID">Lane specifier</param>
        /// <param name="laneOffset">Offset from lane center</param>
        /// <param name="s">Distance along the specified road</param>
        /// <param name="align">If true the heading will be reset to the lane driving direction (typically only at initialization)</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetLanePosition")]
        public static extern int SetLanePosition(int index, int roadId, int laneId, float laneOffset, float s, bool align);

        /// <summary>
        /// Set s (distance) part of a lane position, world coordinates being calculated
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="s">Distance along the specified road</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetS")]
        public static extern int SetS(int index, float s);

        /// <summary>
        /// Set position from world coordinates in the OpenDRIVE coordinate system.
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="z">cartesian coordinate z value</param>
        /// <param name="h">rotation heading value</param>
        /// <param name="p">rotation pitch value</param>
        /// <param name="r">rotation roll value</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetWorldPosition")]
        public static extern int SetWorldPosition(int index, float x, float y, float z, float h, float p, float r);

        /// <summary>
        /// Set position from world X, Y and heading coordinates; Z, pitch and road coordinates being calculated
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="h">rotation heading value</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetWorldXYHPosition")]
        public static extern int SetWorldXYHPosition(int index, float x, float y, float h);

        /// <summary>
        /// Set position from world X, Y, Z and heading coordinates; pitch and road coordinates being calculated
        /// Setting a Z value may have effect in mapping the position to the closest road, e.g. overpass
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="x">cartesian coordinate x value</param>
        /// <param name="y">cartesian coordinate y value</param>
        /// <param name="z">cartesian coordinate z value</param>
        /// <param name="h">rotation heading value</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetWorldXYZHPosition")]
        public static extern int SetWorldXYZHPosition(int index, float x, float y, float z, float h);

        /// <summary>
        /// Change road belonging of position object, keeping actual x,y location, regardless other roads being closer
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="roadId">Id of the road to belong to</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SetRoadId")]
        public static extern int SetRoadId(int index, int roadId);

        /// <summary>
        /// Move position forward along the road network
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="dist">Distance (in meter) to move</param>
        /// <param name="junctionSelectorAngle">Desired direction [0:2pi] from incoming road direction (angle = 0), set -1 to randomize</param>
        /// <returns>@return >= 0 if successful, < 0 on error. For all codes see esmini roadmanager::Position::enum class ReturnCode</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_PositionMoveForward")]
        public static extern int PositionMoveForward(int index, float dist, float junctionSelectorAngle);

        /// <summary>
        /// Get the fields of the position of specified index
        /// </summary>
        /// <param name="index">Handle to the position object</param>
        /// <param name="data">Struct to fill in the values</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetPositionData")]
        public static extern int GetPositionData(int index, ref OpenDrivePositionData data);

        /// <summary>
        /// Retrieve current speed limit (at current road, s-value and lane) based on ODR type elements or nr of lanes
        /// </summary>
        /// <param name="index">Handle to the position object from which to measure</param>
        /// <returns>SpeedLimit in m/s</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetSpeedLimit")]
        public static extern float GetSpeedLimit(int index);

        /// <summary>
        /// Retrieve lane information from the position object (at current road, s-value and lane)
        /// </summary>
        /// <param name="index">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point of interest</param>
        /// <param name="data">Struct including all result values, see RoadLaneInfo typedef</param>
        /// <param name="lookAheadMode">Measurement strategy: 0=Along lane center, 1=road center, 2=current lane offset. See roadmanager::Position::LookAheadMode enum</param>
        /// <param name="inRoadDrivingDirection">If true always look along primary driving direction. If false, look in most straightforward direction according to object heading.</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneInfo")]
        public static extern int GetLaneInfo(int index, float lookahead_distance, ref RoadLaneInfo data, int lookAheadMode, bool inRoadDrivingDirection);

        /// <summary>
        /// As GetLaneInfo plus relative location of point of interest (probe) from current position
        /// </summary>
        /// <param name="index">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point of interest</param>
        /// <param name="data">Struct including all result values, see RoadProbeInfo typedef</param>
        /// <param name="lookAheadMode">Measurement strategy: 0=Along lane center, 1=road center, 2=current lane offset. See roadmanager::Position::LookAheadMode enum</param>
        /// <param name="inRoadDrivingDirection">If true always look along primary driving direction. If false, look in most straightforward direction according to object heading.</param>
        /// <returns>0 if successful, 1 if probe reached end of road, 2 if end ouf route, -1 if some error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetProbeInfo")]
        public static extern int GetProbeInfo(int index, float lookahead_distance, ref RoadProbeInfo data, int lookAheadMode, bool inRoadDrivingDirection);

        /// <summary>
        /// Get width of lane with specified lane id, at current longitudinal position
        /// </summary>
        /// <param name="handle">Handle to the position object from which to measure</param>
        /// <param name="lane_id">Id of the lane to measure</param>
        /// <returns>Lane width or 0.0 if lane does not exists or any other error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneWidth")]
        public static extern float GetLaneWidth(int handle, int lane_id);

        /// <summary>
        /// Get width of lane with specified lane id, at specified road and longitudinal position
        /// </summary>
        /// <param name="handle">Handle to the position object from which to measure</param>
        /// <param name="road_id">Id of the road</param>
        /// <param name="lane_id">Id of the lane to measure</param>
        /// <param name="s">Longitudinal position along the road</param>
        /// <returns>Lane width or 0.0 if lane does not exists or any other error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneWidthByRoadId")]
        public static extern float GetLaneWidthByRoadId(int road_id, int lane_id, float s);

        /// <summary>
        /// Get type of lane with specified lane id, at current longitudinal position
        /// </summary>
        /// <param name="handle">Handle to the position object from which to measure</param>
        /// <param name="lane_id">Id of the lane</param>
        /// <returns>Lane type or 0 if lane does not exists or any other error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetLaneType")]
        public static extern int GetLaneType(int handle, int lane_id);

        /// <summary>
        /// Find out the difference between two position objects, i.e. delta distance (long and lat) and delta laneId
        /// </summary>
        /// <param name="handleA">Handle to the position object from which to measure</param>
        /// <param name="handleB">Handle to the position object to which the distance is measured</param>
        /// <param name="pos_diff">Struct including all result values, see PositionDiff typedef</param>
        /// <returns>0 if successful, -2 if route between positions can't be found, -1 if some other error </returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_SubtractAFromB")]
        public static extern int SubtractAFromB(int handleA, int handleB, ref PositionDiff pos_diff);

        /// <summary>
        /// Get the number of road signs along specified road
        /// </summary>
        /// <param name="road_id">The road along which to look for signs</param>
        /// <returns>Number of road signs, -1 on error</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetNumberOfRoadSigns")]
        public static extern int GetNumberOfRoadSigns(int road_id);

        /// <summary>
        /// Get information on specifed road sign
        /// </summary>
        /// <param name="road_id">The road of which to look for the sign</param>
        /// <param name="index">Index of the sign. Note: not ID</param>
        /// <param name="road_sign">Pointer/reference to a RoadSign struct to be filled in</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetRoadSign")]
        public static extern int GetRoadSign(int road_id, int index, ref RoadSign road_sign);

        /// <summary>
        /// Get georeference for opendrive file
        /// </summary>
        /// <param name="geo_reference">Pointer/reference to a RoadSign struct to be filled in</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(LIB_NAME, EntryPoint = "RM_GetOpenDriveGeoReference")]
        public static extern int GetOpenDriveGeoReference(ref GeoReference geo_reference);

    }

}