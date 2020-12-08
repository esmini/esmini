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
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct SE_LaneInfo
    {
        public int far_left_lb_id;
        public int left_lb_id;
        public int right_lb_id;
        public int far_right_lb_id;
    };

    public static class ESMiniLib
    {
#if UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
        private const string LIB_NAME = "libesminiLib.so";
#else
        private const string LIB_NAME = "esminiLib";
#endif

        [DllImport(LIB_NAME, EntryPoint = "SE_AddPath")]
        /// <summary>Add a search path for OpenDRIVE and 3D model files
        /// <param name="path">Path to a directory</param>
        /// <returns>0 on success, -1 on failure for any reason</returns> 
        public static extern int SE_AddPath(string path);

        [DllImport(LIB_NAME, EntryPoint = "SE_ClearPaths")]
        /// <summary>Clear all search paths for OpenDRIVE and 3D model files
        public static extern void SE_ClearPaths();

        [DllImport(LIB_NAME, EntryPoint = "SE_Init")]
        /// <summary>Initialize the scenario engine</summary>
        /// <param name="oscFilename">Path to the OpenSCEANRIO file</param>
        /// <param name="disable_ctrls">1=Any controller will be disabled 0=Controllers applied according to OSC file</param>
        /// <param name="use_viewer">0=no viewer, 1=use viewer</param>
        /// <param name="threads">0=single thread, 1=viewer in a separate thread, parallel to scenario engine</param>
        /// <param name="record">Create recording for later playback 0=no recording 1=recording</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        public static extern int SE_Init(string oscFilename, int disable_ctrls = 0, int use_viewer = 0, int threads = 0, int record = 0);

        [DllImport(LIB_NAME, EntryPoint = "SE_StepDT")]
        public static extern int SE_StepDT(float dt);

        [DllImport(LIB_NAME, EntryPoint = "SE_Step")]
        public static extern int SE_Step();

        [DllImport(LIB_NAME, EntryPoint = "SE_Close")]
        public static extern void SE_Close();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetQuitFlag")]
        public static extern bool SE_GetQuitFlag();

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

        [DllImport(LIB_NAME, EntryPoint = "SE_GetNumberOfObjects")]
        public static extern int SE_GetNumberOfObjects();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectState")]
        public static extern int SE_GetObjectState(int index, ref ScenarioObjectState state);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectGhostState")]
        public static extern int SE_GetObjectGhostState(int index, ref ScenarioObjectState state);

        [DllImport(LIB_NAME, EntryPoint = "SE_ObjectHasGhost")]
        public static extern int SE_ObjectHasGhost(int index);

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
        [DllImport(LIB_NAME, EntryPoint = "SE_AddObjectSensor")]
        public static extern int SE_AddObjectSensor(int object_id, float x, float y, float z, float h, float rangeNear, float rangeFar, float fovH, int maxObj);

        /// <summary>Fetch list of identified objects from a sensor</summary>
        /// <param name="object_id">Handle to the object to which the sensor should is attached</param>
        /// <param name="list">Array of object indices</param>
        /// <returns> Number of identified objects, i.e.length of list. -1 on failure</returns>
        [DllImport(LIB_NAME, EntryPoint = "SE_FetchSensorObjectList")]
        public static extern int SE_FetchSensorObjectList(int object_id, int[] list);


        /// <summary>Get information suitable for driver modeling of a point at a specified distance from object along the road ahead</summary>
        /// <param name="object_id">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="along_road_center">Measure along the reference lane, i.e. at center of the road. Should be false for normal use cases</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        [DllImport(LIB_NAME, EntryPoint = "SE_GetRoadInfoAtDistance")]
        public static extern int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, ref RoadInfo data, int along_road_center);

        /// <summary>Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle</summary>
        /// <param name="object_id">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="speed_ghost">Speed of the ghost vehicle at specifed point</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        [DllImport(LIB_NAME, EntryPoint = "SE_GetRoadInfoAlongGhostTrail")]
        public static extern int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, ref RoadInfo data, ref float speed_ghost);

    }

}