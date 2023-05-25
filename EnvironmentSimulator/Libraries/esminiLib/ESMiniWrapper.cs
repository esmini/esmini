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

using System.Runtime.InteropServices;
using System;


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
        public float wheel_angle;
        public float wheel_rotation;
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

    [StructLayout(LayoutKind.Sequential)]
    public struct SimpleVehicleState
    {
        public float x;
        public float y;
        public float z;
        public float h;
        public float p;
        public float speed;
        public float wheel_rotation;
        public float wheel_angle;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct LaneBoundaryId
    {
        public int far_left_lb_id;
        public int left_lb_id;
        public int right_lb_id;
        public int far_right_lb_id;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct PositionDiff
    {
        public float ds;            // delta s (longitudinal distance)
        public float dt;            // delta t (lateral distance)
        public int d_lane_id;       // delta laneId (increasing left and decreasing to the right)
        public float dx;            // delta x (world coordinate system)
        public float dy;            // delta y (world coordinate system)
        public bool opposite_lanes; // true if the two position objects are in opposite sides of reference lane
    };


public static class ESMiniLib
    {
        private const string LIB_NAME = "esminiLib";

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

        public delegate void ConditionCallback(string name, double timestamp);
        [DllImport(LIB_NAME, EntryPoint = "SE_RegisterConditionCallback")]
        /// <summary>
        /// Registers a function to be called back from esmini every time a condition is triggered.
        /// The name of the respective condition and the current timestamp will be returned.
        /// Registered callbacks will be cleared between SE_Init calls.
        /// </summary>
        /// <param name="cc">The callback function to be invoked</param>
        public static extern void SE_RegisterConditionCallback(ConditionCallback cc);

        public delegate void EventCallback(string name, double timeStamp, bool isStart);
        [DllImport(LIB_NAME, EntryPoint = "SE_RegisterEventCallback")]
        /// <summary>
        /// Registers a function to be called back from esmini every time an event starts or ends.
        /// The name of the respective event, the current timestamp and whether the event
        /// starts(true) or ends(false) will be returned.
        /// In case an event starts and ends within the same simulation step, only the end-transition may occur.
        /// Registered callbacks will be cleared between SE_Init calls.
        /// </summary>
        /// <param name="cc">The callback function to be invoked</param>
        public static extern void SE_RegisterEventCallback(EventCallback ec);

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

        [DllImport(LIB_NAME, EntryPoint = "SE_GetId")]
        /// <summary>Get the Id of an entity present in the current scenario</summary>
        /// <param name="index">Index of the object. Note: not ID</param>
        /// <returns>Id of the object, -1 on error e.g. scenario not initialized</returns>
        public static extern int SE_GetId(int index);

        [DllImport(LIB_NAME, EntryPoint = "SE_AddObject")]
        /// <summary>Add object. Should be followed by one of the SE_Report functions to establish initial state.</summary>
        /// <param name="object_name">Name of the object, preferably be unique</param>
        /// <param name="object_type">Type of the object. See Entities.hpp::Object::Type. Default=1 (VEHICLE).</param>
        /// <param name="object_category">Category of the object. Depends on type, see descendants of Entities.hpp::Object. Set to 0 if not known.</param>
        /// <param name="object_role"> role of the object. Depends on type, See Entities.hpp::Object::Role. Set to 0 if not known.</param>
        /// <param name="model_id">Id of the 3D model to represent the object. See resources/model_ids.txt.</param>
        /// <returns> @return Id [0..inf] of the added object successful, -1 on failure</returns>
        public static extern int SE_AddObject(string object_name, int object_type, int object_category, int object_role, int model_id);

        [DllImport(LIB_NAME, EntryPoint = "SE_DeleteObject")]
        /// <summary>Delete object</summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        public static extern int SE_DeleteObject(int object_id);

        #region ObjectReporter
        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectPos")]
        public static extern int SE_ReportObjectPos(int id, float timestamp, float x, float y, float z, float h, float p, float r);

        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectRoadPos")]
        public static extern int SE_ReportObjectRoadPos(int id, float timestamp, int roadId, int laneId, float laneOffset, float s);

        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectSpeed")]
        public static extern int SE_ReportObjectSpeed(int id, float speed);

        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectVel")]
        public static extern int SE_ReportObjectVel(int id, float timestamp, float x_vel, float y_vel, float z_vel);

        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectAcc")]
        public static extern int SE_ReportObjectAcc(int id, float timestamp, float x_acc, float y_acc, float z_acc);

        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectAngularVel")]
        public static extern int SE_ReportObjectAngularVel(int id, float timestamp, float h_vel, float p_vel, float r_vel);

        [DllImport(LIB_NAME, EntryPoint = "SE_ReportObjectAngularAcc")]
        public static extern int SE_ReportObjectAngularAcc(int id, float timestamp, float h_acc, float p_acc, float r_acc);

        #endregion
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
        /// <param name="object_id">Id of the object (not index, use GetId(index) to find out the id)</param>
        /// <param name="state">Reference to a ScenarioObjectState struct to be filled in</param>
        /// <return>0 if successful, -1 if not</return>
        public static extern int SE_GetObjectState(int object_id, ref ScenarioObjectState state);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectTypeName")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get the type name of the specifed vehicle-, pedestrian- or misc object</summary>
        /// <param name="object_id">Id of the object</param>
        /// <return>Name</return>
        public static extern IntPtr SE_GetObjectTypeName(int object_id);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectName")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get the name of specified object</summary>
        /// <param name="object_id">Id of the object</param>
        /// <return>Name</return>
        public static extern IntPtr SE_GetObjectName(int object_id);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectModelFileName")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get the 3D model filename of the specifed object</summary>
        /// <param name="object_id">Id of the object</param>
        /// <return>Name</return>
        public static extern IntPtr SE_GetObjectModelFileName(int object_id);

        [DllImport(LIB_NAME, EntryPoint = "SE_ObjectHasGhost")]
        /// <summary>Check whether an object has a ghost (special purpose lead vehicle)</summary>
        /// <param name="object_id">Id of the object</param>
        /// <return>1 if ghost, 0 if not, -1 indicates error e.g. scenario not loaded</return>
        public static extern int SE_ObjectHasGhost(int object_id);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectGhostState")]
        /// <summary>Get the state of specified object's ghost (special purpose lead vehicle)</summary>
        /// <param name="object_id">Id of the object to which the ghost is attached</param>
        /// <param name="state">Reference to a ScenarioObjectState struct to be filled in</param>
        /// <return>0 if successful, -1 if not</return>
        public static extern int SE_GetObjectGhostState(int object_id, ref ScenarioObjectState state);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetSpeedUnit")]
        /// <summaryGet the unit of specified speed</summary>
        /// All roads will be looped in search for such an element. First found will be used.
        /// If speed is specified withouth the optional unit, SI unit m/s is assumed.
        /// If no speed entries is found, undefined will be returned.
        /// <returns>-1=Error, 0=Undefined, 1=km/h 2=m/s, 3=mph</returns>
        public static extern int GetSpeedUnit();

        [DllImport(LIB_NAME, EntryPoint = "SE_AddObjectSensor")]
        /// <summary>Create an ideal object sensor and attach to specified vehicle</summary>
        /// <param name="object_id">Id of the object to which the sensor should be attached</param>
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
        /// <param name="object_id">Id of the object to which the sensor should is attached</param>
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
        /// <param name="object_id">Id of the object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="speed_ghost">Speed of the ghost vehicle at specifed point</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
        [DllImport(LIB_NAME, EntryPoint = "SE_GetRoadInfoAlongGhostTrail")]
        public static extern int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, ref RoadInfo data, ref float speed_ghost);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetDistanceToObject")]
        /// <summary>Find out the delta between two objects, e.g. distance (long and lat) and delta laneId. Search range is 1000 meters</summary>
        /// <param name="object_a_id">Id of the object from which to measure</param>
        /// <param name="object_b_id">Id of the object from which to measure</param>
        /// <param name="freespace">Measure distance between bounding boxes (true) or between ref points (false)</param>
        /// <param name="pos_diff">Reference to struct including all result values, see typedef for details</param>
        /// <returns>0 if successful, -1 if not</returns>
        public static extern int SE_GetDistanceToObject(int object_a_id, int object_b_id, bool free_space, ref PositionDiff pos_diff);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetNumberOfParameters")]
        /// <summary>Get the number of parameters in the current scenario</summary>
        /// <return>Number of parameters, -1 on error e.g. scenario not initialized</return>
        public static extern int SE_GetNumberOfParameters();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetParameterName")]
        /// <summary>Get name of parameter with specifed index</summary>
        /// <param name="index">Index of the parameter</param>
        /// <param name="parameterType">Returns the type of the parameter</param>///
        /// <returns>Parameter name as string. Use with: Marshal.PtrToStringAnsi(SE_GetParameterName())</returns>
        //[return: MarshalAs(UnmanagedType.LPStr)]
        public static extern IntPtr SE_GetParameterName(int index, out int parameterType);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetParameterInt")]
        public static extern int SE_GetParameterInt(string parameterName, out int value);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetParameterDouble")]
        public static extern int SE_GetParameterDouble(string parameterName, out double value);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetParameterString")]
        /// <summary>Get string parameter value </summary>
        /// <param name="value">the string value as output parameter. Use: IntPtr intPtr; string str = Marshal.PtrToStringAnsi(intPtr);</param>
        /// <returns>0 on success, -1 on failure for any reason</returns>
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

        [DllImport(LIB_NAME, EntryPoint = "SE_GetObjectPropertyValue")]
        public static extern IntPtr SE_GetObjectPropertyValue(int index, string value);

        // Simple vehicle

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleCreate")]
        /// <summary>Create an instance of a simplistic vehicle based on a 2D bicycle kincematic model</summary>
        /// <param name="x"> Initial position X world coordinate</param>
        /// <param name="y"> Initial position Y world coordinate</param>
        /// <param name="h"> Initial heading</param>
        /// <param name="length"> Length of the vehicle</param>
        /// <param name="speed"> Initial speed</param>
        /// <returns>Handle to the created object</returns>
        public static extern IntPtr SE_SimpleVehicleCreate(float x, float y, float h, float length, float speed);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleDelete")]
        /// <summary>Delete an instance of the simplistic vehicle model</summary>
        /// <param name="handleSimpleVehicle">Handle to the object</param>
        public static extern void SE_SimpleVehicleDelete(IntPtr handleSimpleVehicle);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleControlBinary")]
        /// <summary>Control the speed and steering with discreet [-1, 0, 1] values, suitable for keyboard
        /// control (e.g. up/none/down). The function also steps the vehicle model, updating its position
        /// according to motion state and timestep</summary>
        /// <param name="handleSimpleVehicle">Handle to the object</param>
        /// <param name="dt"> timesStep (s)</param>
        /// <param name="throttle">Longitudinal control, -1: brake, 0: none, +1: accelerate</param>
        /// <param name="steering">Lateral control, -1: left, 0: straight, 1: right</param>
        public static extern void SE_SimpleVehicleControlBinary(IntPtr handleSimpleVehicle,
                                                                double dt,
                                                                int throttle,
                                                                int steering);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleControlAnalog")]
        /// <summary>Control the speed and steering with floating values in the range [-1, 1], suitable for
        /// models. The function also steps the vehicle model, updating its position according to motion
        /// state and timestep.</summary>
        /// <param name="handleSimpleVehicle">Handle to the object</param>
        /// <param name="dt"> timesStep (s)</param>
        /// <param name="throttle">Longitudinal control, -1: maximum brake, 0: no acceleration, +1: maximum acceleration</param>
        /// <param name="steering">Lateral control, -1: max left, 0: straight, 1: max right</param>
        public static extern void SE_SimpleVehicleControlAnalog(IntPtr handleSimpleVehicle,
                                                                double dt,
                                                                double throttle,
                                                                double steering);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleControlTarget")]
        /// <summary>Control the speed and steering by providing steering and speed targets. The function
        /// also steps the vehicle model, updating its position according to motion state and timestep.</summary>
        /// <param name="handleSimpleVehicle">Handle to the object</param>
        /// <param name="dt"> timesStep (s)</param>
        /// <param name="target_speed">Requested speed</param>
        /// <param name="heading_to_target">Heading angle to a target position</param>
        public static extern void SE_SimpleVehicleControlTarget(IntPtr handleSimpleVehicle,
                                                                double dt,
                                                                double target_speed,
                                                                double heading_to_target);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleSetMaxSpeed")]
        /// <summary>Set maximum vehicle speed</summary>
        /// <param name="handleSimpleVehicle">Handle to the object</param>
        /// <param name="speed">speed Maximum speed (km/h)</param>
        public static extern void SE_SimpleVehicleSetMaxSpeed(IntPtr handleSimpleVehicle, float speed);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleSetMaxAcceleration")]
        /// <summary>Set maximum vehicle acceleration</summary>
        /// <param name="handleSimpleVehicle">Set maximum vehicle acceleration.</param>
        /// <param name="maxAcceleration">speed Maximum acceleration (m/s^2)</param>
        public static extern void SE_SimpleVehicleSetMaxAcceleration(IntPtr handleSimpleVehicle, float maxAcceleration);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleSetMaxDeceleration")]
        /// <summary>Set maximum vehicle deceleration</summary>
        /// <param name="handleSimpleVehicle">Set maximum vehicle acceleration.</param>
        /// <param name="maxDeceleration">Maximum deceleration (m/s^2)</param>
        public static extern void SE_SimpleVehicleSetMaxDeceleration(IntPtr handleSimpleVehicle, float maxDeceleration);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleSetEngineBrakeFactor")]
        /// <summary>Set engine brake factor, applied when no throttle is applied</summary>
        /// <param name="handleSimpleVehicle">Set maximum vehicle acceleration.</param>
        /// <param name="engineBrakeFactor">Recommended range = [0.0, 0.01], default = 0.001</param>
        public static extern void SE_SimpleVehicleSetEngineBrakeFactor(IntPtr handleSimpleVehicle, float engineBrakeFactor);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleSteeringScale")]
        /// <summary>Set steering scale factor, which will limit the steering range as speed increases</summary>
        /// <param name="handleSimpleVehicle">Set maximum vehicle acceleration.</param>
        /// <param name="steeringScale">Recommended range = [0.0, 0.1], default = 0.018</param>
        public static extern void SE_SimpleVehicleSteeringScale(IntPtr handleSimpleVehicle, float steeringScale);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleSteeringReturnFactor")]
        /// <summary>Set steering return factor, which will make the steering wheel strive to neutral position (0 angle)</summary>
        /// <param name="handleSimpleVehicle">Set maximum vehicle acceleration.</param>
        /// <param name="steeringScale">Recommended range = [0.0, 10], default = 4.0</param>
        public static extern void SE_SimpleVehicleSteeringReturnFactor(IntPtr handleSimpleVehicle, float steeringReturnFactor);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleSteeringRate")]
        /// <summary>Set steering rate, which will affect the angular speed of which the steering wheel will turn</summary>
        /// <param name="handleSimpleVehicle">Set maximum vehicle acceleration.</param>
        /// <param name="steeringRate">Recommended range = [0.0, 50.0], default = 8.0</param>
        public static extern void SE_SimpleVehicleSteeringRate(IntPtr handleSimpleVehicle, float steeringRate);

        [DllImport(LIB_NAME, EntryPoint = "SE_SimpleVehicleGetState")]
        /// <summary>Get current state of the vehicle. Typically called after Control has been applied</summary>
        /// <param name="handleSimpleVehicle">Set maximum vehicle acceleration.</param>
        /// <param name="engineBrakeFactor">Reference to a SE_SimpleVehicleState struct to be filled in</param>
        public static extern void SE_SimpleVehicleGetState(IntPtr handleSimpleVehicle, ref SimpleVehicleState state);

        // OSI interface (subset)

        [DllImport(LIB_NAME, EntryPoint = "SE_SetOSITolerances")]
        /// <summary>Configure tolerances/resolution for OSI road features</summary>
        /// <param name="max_longitudinal_distance">Maximum distance between OSI points, even on straight road. Default=50(m)</param>
        /// <param name="max_lateral_deviation">Control resolution w.r.t. curvature default=0.05(m)</param>
        /// <returns>0 if successful, -1 if not</returns>
        public static extern int SE_SetOSITolerances(double maxLongitudinalDistance, double maxLateralDeviation);

        [DllImport(LIB_NAME, EntryPoint = "SE_DisableOSIFile")]
        /// <summary>Switch off logging to OSI file(s)</summary>
        /// <returns>0 if successful, -1 if not</returns>
        public static extern void SE_DisableOSIFile();

        [DllImport(LIB_NAME, EntryPoint = "SE_EnableOSIFile")]
        /// <summary>Switch on logging to OSI file(s)</summary>
        /// <param name="filename">Optional filename, including path.Set to 0 or "" to use default.</param>
        /// <returns>0 if successful, -1 if not</returns>
        public static extern void SE_EnableOSIFile(string filename);

        [DllImport(LIB_NAME, EntryPoint = "SE_FlushOSIFile")]
        /// <summary>Enforce flushing OSI file (save all buffered data to file)</summary>
        public static extern void SE_FlushOSIFile();

        [DllImport(LIB_NAME, EntryPoint = "SE_UpdateOSIGroundTruth")]
        /// <summary>Updates OSI Groundtruth</summary>
        public static extern void SE_UpdateOSIGroundTruth();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetOSIGroundTruth")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>char array containing the osi GroundTruth serialized to a string</summary>
        /// <param name="size">Size of the retuned OSI string</param>
        /// <returns>osi3::GroundTruth*</returns>
        public static extern IntPtr SE_GetOSIGroundTruth(ref int size);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetOSIGroundTruthRaw")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>char array containing the OSI GroundTruth information</summary>
        /// <returns>osi3::GroundTruth*</returns>
        public static extern IntPtr SE_GetOSIGroundTruthRaw();

        [DllImport(LIB_NAME, EntryPoint = "SE_GetOSIRoadLane")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get information of the lane where the object with object_id is</summary>
        /// <param name="size">Size of the retuned OSI string</param>
        /// <param name="object_id">Id of the object</param>
        /// <returns>a char array containing the osi Lane information/message, serialized to a string</returns>
        public static extern IntPtr SE_GetOSIRoadLane(ref int size, int object_id);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetOSILaneBoundary")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get information of the lane boundary where the object with object_id is</summary>
        /// <param name="size">Size of the retuned OSI string</param>
        /// <param name="object_id">Id of the object</param>
        /// <returns>a char array containing the osi Lane Boundary information/message, serialized to a string</returns>
        public static extern IntPtr SE_GetOSILaneBoundary(ref int size, int object_id);

        [DllImport(LIB_NAME, EntryPoint = "SE_GetOSILaneBoundaryIds")]
        //[return: MarshalAs(UnmanagedType.LPStr)]
        /// <summary>Get the global ids for left, far left, right and far right lane boundaries, relative an object</summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="ids">Reference to a struct which will be filled with the Ids</param>
        public static extern IntPtr SE_GetOSILaneBoundaryIds(int object_id, ref LaneBoundaryId ids);
    }

}
