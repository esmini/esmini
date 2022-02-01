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
 * This module provides a generic C# interface/wrapper to the esminiLib shared library
 * simply mirroring the interface in terms of datatypes and functions
 */

using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;
using System.IO;
using UnityEngine.UI;


namespace ESMini
{

    [StructLayout(LayoutKind.Sequential)]
    public struct ScenarioObjectState
    {
        public int id;          // Automatically generated unique object id
        public int model_id;    // Id to control what 3D model to represent the vehicle - see carModelsFiles_[] in scenarioenginedll.cpp
        public int ctrl_type;     // 0: DefaultController 1: External. Further values see esmini/EnvironmentSimulator/Controllers/Controller::Type enum
        public float timestamp;
        public float x;
        public float y;
        public float z;
        public float h;
        public float p;
        public float r;
        public int roadId;
        public int junctionId;
        public float t;
        public int laneId;
        public float laneOffset;
        public float s;
        public float speed;
        public float centerOffsetX;
        public float centerOffsetY;
        public float centerOffsetZ;
        public float width;
        public float length;
        public float height;
        public int   objectType;     // Main type according to entities.hpp / Object / Type (NONE=0, VEHICLE=1, PEDESTRIAN=2, MISC_OBJECT=3)
        public int   objectCategory; // Sub category within type, according to entities.hpp / Vehicle, Pedestrian, MiscObject / Category
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct RoadInfo
    {
        public float global_pos_x;     // steering target position, in global coordinate system
        public float global_pos_y;     // steering target position, in global coordinate system
        public float global_pos_z;     // steering target position, in global coordinate system
        public float local_pos_x;      // steering target position, relative vehicle (pivot position object) coordinate system
        public float local_pos_y;      // steering target position, relative vehicle (pivot position object) coordinate system
        public float local_pos_z;      // steering target position, relative vehicle (pivot position object) coordinate system
        public float angle;            // heading angle to steering target from and relatove to vehicle (pivot position)
        public float road_heading;     // road heading at steering target point
        public float road_pitch;       // road pitch (inclination) at steering target point
        public float road_roll;        // road roll (camber) at steering target point
        public float trail_heading;    // trail heading (only when used for trail lookups, else equals road_heading)
        public float curvature;        // road curvature at steering target point
        public float speed_limit;      // speed limit given by OpenDRIVE type entry
        public int roadId;             // target position, road ID
        public int junctionId;         // target position, junction ID (-1 if not in a junction)
        public int laneId;             // target position, lane ID
        public float laneOffset;       // target position, lane offset (lateral distance from lane center)
        public float s;                // target position, s (longitudinal distance along reference line)
        public float t;                // target position, t (lateral distance from reference line)
    };

public static class ESMiniLib
    {
#if UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
        private const string LIB_NAME = "libesminiLib.so";
#else
        private const string LIB_NAME = "esminiLib";
#endif

        [DllImport(LIB_NAME, EntryPoint = "SE_AddPath")]
        /// <summary>Add a search path for OpenDRIVE and 3D model files </summary>
        /// <param name="path">Path to a directory</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        public static extern int SE_AddPath(string path);

        [DllImport(LIB_NAME, EntryPoint = "SE_ClearPaths")]
        /// <summary>Clear all search paths for OpenDRIVE and 3D model files </summary>
        public static extern void SE_ClearPaths();

        [DllImport(LIB_NAME, EntryPoint = "SE_SetLogFilePath")]
        /// <summary>Specify logfile name, optionally including directory path
        /// examples: "../logfile.txt" "c:/tmp/esmini.log" "my.log"
        /// Set "" to disable logfile
        /// Note: Needs to be called prior to calling SE_Init() </summary>
        /// <param name="path">Logfile path</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        public static extern int SE_SetLogFilePath(string path);

        [DllImport(LIB_NAME, EntryPoint = "SE_SetOSITolerances")]
        /// <summary>Configure tolerances/resolution for OSI road features</summary>
        /// <param name="max_longitudinal_distance">Maximum distance between OSI points, even on straight road. Default=50(m) </param>
        /// <param name="max_lateral_deviation"> Control resolution w.r.t. curvature default=0.05(m)</param>
        /// <return>0 if successful, -1 if not</return>
        public static extern int SE_SetOSITolerances(double maxLongitudinalDistance, double maxLateralDeviation);

        [DllImport(LIB_NAME, EntryPoint = "SE_RegisterParameterDeclarationCallback")]
        /// <summary>
        /// Register a function and optional argument (ref) to be called back from esmini after ParameterDeclarations has been parsed,
        /// but before the scenario is initialized, i.e.before applying the actions in the Init block.One use-case is to
        /// set parameter values for initial entity states, e.g.s value in lane position. So this callback will happen just
        /// after parameters has been parsed, but before they are applied, providing an opportunity to control the initial
        /// states via API.
        /// Registered init callbacks are be cleared between SE_Init calls, i.e.needs to be registered
        /// </summary>
        /// <param name="func">Reference to the callback function to be invoked</param>
        /// <param name="user_data">Optional pointer to a local data object that will be passed as argument in the callback.
        /// Set 0/NULL if not needed.</param>
        public static extern void SE_RegisterParameterDeclarationCallback(Action<IntPtr> callback, IntPtr user_data);

        [DllImport(LIB_NAME, EntryPoint = "SE_Init")]
        /// <summary>Initialize the scenario engine</summary>
        /// <param name="oscFilename">Path to the OpenSCEANRIO file</param>
        /// <param name="disable_ctrls">1=Any controller will be disabled 0=Controllers applied according to OSC file</param>
        /// <param name="use_viewer">0=no viewer, 1=use viewer</param>
        /// <param name="threads">0=single thread, 1=viewer in a separate thread, parallel to scenario engine</param>
        /// <param name="record">Create recording for later playback 0=no recording 1=recording</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        public static extern int SE_Init(string oscFilename, int disable_ctrls = 0, int use_viewer = 0, int threads = 0, int record = 0);

        [DllImport(LIB_NAME, EntryPoint = "SE_InitWithString")]
        /// <summary>Initialize the scenario engine, privding OSC XML string instead of filename</summary>
        /// <param name="oscAsXMLString">OpenSCENARIO XML as string</param>
        /// <param name="disable_ctrls"> 1=Any controller will be disabled 0=Controllers applied according to OSC file</param>
        /// <param name="use_viewer">0=no viewer, 1=use viewer</param>
        /// <param name="threads"> 0=single thread, 1=viewer in a separate thread, parallel to scenario engine</param>
        /// <param name="record"> Create recording for later playback 0=no recording 1=recording</param>
        /// <return>0 if successful, -1 if not</return>
        public static extern int SE_InitWithString(string oscAsXMLString, int disable_ctrls, int use_viewer, int threads, int record);

        [DllImport(LIB_NAME, EntryPoint = "SE_StepDT")]
        public static extern int SE_StepDT(float dt);

        [DllImport(LIB_NAME, EntryPoint = "SE_Step")]
        public static extern int SE_Step();

        [DllImport(LIB_NAME, EntryPoint = "SE_Close")]
        public static extern void SE_Close();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetQuitFlag")]
        /// <summary>Is esmini about to quit?</summary>
        /// <return>0 if not, 1 if yes, -1 if some error e.g. scenario not loaded</return>
        public static extern int SE_GetQuitFlag();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetODRFilename")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get name of currently referred and loaded OpenDRIVE file</summary>
        /// <returns>Filename as string. Use with: Marshal.PtrToStringAnsi(SE_GetODRFilename())</returns>
        public static extern IntPtr SE_GetODRFilename();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetSceneGraphFilename")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get name of currently referred and loaded SceneGraph file</summary>
        /// <returns>Filename as string. Use with: Marshal.PtrToStringAnsi(SE_GetSceneGraphFilename())</returns>
        public static extern IntPtr SE_GetSceneGraphFilename();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetSimulationTime")]
        public static extern float SE_GetSimulationTime();

        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectPos")]
        public static extern int SE_ReportObjectPos(int id, float timestamp, float x, float y, float z, float h, float p, float r, float speed);

        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectRoadPos")]
        public static extern int SE_ReportObjectRoadPos(int id, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

        [DllImport(LIB_NAME, EntryPoint = "SE_SetLockOnLane")]
        /// <summary>Controls whether to keep lane ID regardless of lateral position or snap to closest lane (default)</summary>
        /// <parameter name="mode">True=keep lane False=Snap to closest (default)</parameter>
        /// <return>0 if successful, -1 if not</return>
        public static extern int SE_SetLockOnLane(int id, bool mode);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetNumberOfObjects")]
        /// <summary>Get the number of entities in the current scenario</summary>
        /// <return>Number of entities, -1 on error e.g. scenario not initialized</return>
        public static extern int SE_GetNumberOfObjects();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectState")]
        /// <summary>Get the state of specified object</summary>
        /// <param name="index">Index of the object. Note: not ID</param>
        /// <param name="state">Reference to a ScenarioObjectState struct to be filled in</param>
        /// <return>0 if successful, -1 if not</return>
        public static extern int SE_GetObjectState(int index, ref ScenarioObjectState state);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectTypeName")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get the type name of the specifed vehicle-, pedestrian- or misc object</summary>
        /// <param name="index">Index of the object. Note: not ID</param>
        /// <return>Name</return>
        public static extern IntPtr SE_GetObjectTypeName(int index);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectName")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get the name of specified object</summary>
        /// <param name="index">Index of the object. Note: not ID</param>
        /// <return>Name</return>
        public static extern IntPtr SE_GetObjectName(int index);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectModelFileName")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get the 3D model filename of the specifed object</summary>
        /// <param name="index">Index of the object. Note: not ID</param>
        /// <return>Name</return>
        public static extern IntPtr SE_GetObjectModelFileName(int index);

        [DllImport(LIB_NAME, EntryPoint = "SE_ObjectHasGhost")]
        /// <summary>Check whether an object has a ghost (special purpose lead vehicle)</summary>
        /// <param name="object_id">Handle to the object</param>
        /// <return>1 if ghost, 0 if not, -1 indicates error e.g. scenario not loaded</return>
        public static extern int SE_ObjectHasGhost(int object_id);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectGhostState")]
        public static extern int SE_GetObjectGhostState(int index, ref ScenarioObjectState state);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetSpeedUnit")]
        /// <summaryGet the unit of specified speed</summary>
        /// All roads will be looped in search for such an element. First found will be used.
        /// If speed is specified withouth the optional unit, SI unit m/s is assumed.
        /// If no speed entries is found, undefined will be returned.
        /// <returns>-1=Error, 0=Undefined, 1=km/h 2=m/s, 3=mph</returns>
        public static extern int GetSpeedUnit();

        [DllImport(LIB_NAME, EntryPoint = "SE_AddObjectSensor")]
        /// <summary>Create an ideal object sensor and attach to specified vehicle</summary>
        /// <param name="object_id">Handle to the object to which the sensor should be attached</param>
        /// <param name="x">Position x coordinate of the sensor in vehicle local coordinates</param>
        /// <param name="y">Position y coordinate of the sensor in vehicle local coordinates</param>
        /// <param name="z">Position z coordinate of the sensor in vehicle local coordinates</param>
        /// <param name="h">Position heading of the sensor in vehicle local coordinates</param>
        /// <param name="fovH">Horizontal field of view, in degrees</param>
        /// <param name="rangeNear">Near value of the sensor depth range</param>
        /// <param name="rangeFar">Far value of the sensor depth range</param>
        /// <param name="maxObj"> Maximum number of objects theat the sensor can track</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        public static extern int SE_AddObjectSensor(int object_id, float x, float y, float z, float h, float rangeNear, float rangeFar, float fovH, int maxObj);

        [DllImport(LIB_NAME, EntryPoint = "SE_FetchSensorObjectList")]
        /// <summary>Fetch list of identified objects from a sensor</summary>
        /// <param name="object_id">Handle to the object to which the sensor should is attached</param>
        /// <param name="list">Array of object indices</param>
        /// <returns> Number of identified objects, i.e.length of list. -1 on failure</returns>
        public static extern int SE_FetchSensorObjectList(int object_id, int[] list);

		[DllImport(LIB_NAME, EntryPoint = "SE_GetRoadInfoAtDistance")]
        /// <summary>Get information suitable for driver modeling of a point at a specified distance from object along the road ahead</summary>
        /// <param name="object_id">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="along_road_center">Measure along the reference lane, i.e. at center of the road. Should be false for normal use cases</param>
        /// <param name="lookAheadMode">Measurement strategy: Along 0=lane center, 1=road center(ref line) or 2=current lane offset.See roadmanager::Position::LookAheadMode enum</param>
        /// <param name="inRoadDrivingDirection">If true always look along primary driving direction.If false, look in most straightforward direction according to object heading.</param>
        /// <returns>0 if successful, 1 if probe reached end of road, 2 if end ouf route, -1 if some error</returns>
        public static extern int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, ref RoadInfo data, int along_road_center);

        /// <summary>Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle</summary>
        /// <param name="object_id">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="speed_ghost">Speed of the ghost vehicle at specifed point</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        [DllImport(LIB_NAME, EntryPoint = "SE_GetRoadInfoAlongGhostTrail")]
        public static extern int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, ref RoadInfo data, ref float speed_ghost);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetParameterInt")]
        public static extern int SE_GetParameterInt(string parameterName, out int value);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetParameterDouble")]
        public static extern int SE_GetParameterDouble(string parameterName, out double value);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetParameterString")]
        /// <summary>Get string parameter value </summary>
        /// <param name="value">the string value as output parameter. Use: IntPtr intPtr; string str = Marshal.PtrToStringAnsi(intPtr);</param>
        /// <returns>>0 on success, -1 on failure for any reason</returns>
        public static extern int SE_GetParameterString(string parameterName, out IntPtr value);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetParameterBool")]
        public static extern int SE_GetParameterBool(string parameterName, out bool value);

        [DllImport(LIB_NAME, EntryPoint = "SE_SetParameterInt")]
        public static extern int SE_SetParameterInt(string parameterName, int value);

        [DllImport(LIB_NAME, EntryPoint = "SE_SetParameterDouble")]
        public static extern int SE_SetParameterDouble(string parameterName, double value);

        [DllImport(LIB_NAME, EntryPoint = "SE_SetParameterString")]
        public static extern int SE_SetParameterString(string parameterName, string value);

        [DllImport(LIB_NAME, EntryPoint = "SE_SetParameterBool")]
        public static extern int SE_SetParameterBool(string parameterName, bool value);
    }

}