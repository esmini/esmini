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

using System;
using System.Runtime.InteropServices;

namespace ESMini
{
    /// <summary>
    /// Represents the state of a scenario object at a given point in time.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ScenarioObjectState
    {
        public int id;
        public int model_id;
        public int ctrl_type;
        public double timestamp;
        public double x;
        public double y;
        public double z;
        public double h;
        public double p;
        public double r;
        public double speed;
        public int roadId;
        public int junctionId;
        public double t;
        public int laneId;
        public double s;
        public double laneOffset;
        public double centerOffsetX;
        public double centerOffsetY;
        public double centerOffsetZ;
        public double width;
        public double length;
        public double height;
        public int objectType;
        public int objectCategory;
        public double wheel_angle;
        public double wheel_rotation;
        public int visibilityMask;
    }

    /// <summary>
    /// Contains information about the road network at a specific location.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RoadInfo
    {
        public double local_pos_x;
        public double local_pos_y;
        public double local_pos_z;
        public double global_pos_x;
        public double global_pos_y;
        public double global_pos_z;
        public double angle;
        public double curvature;
        public double road_heading;
        public double road_pitch;
        public double road_roll;
        public double speed_limit;
        public int junctionId;
        public int roadId;
        public int laneId;
        public double laneOffset;
        public double s;
        public double t;
        public int road_type;
        public int road_rule;
        public int lane_type;
        public double trail_heading;
        public double trail_wheel_angle;
    }

    /// <summary>
    /// State of a simple vehicle model.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct SimpleVehicleState
    {
        public double x;
        public double y;
        public double z;
        public double h;
        public double p;
        public double speed;
        public double wheel_rotation;
        public double wheel_angle;
    }

    /// <summary>
    /// Identifiers for lane boundaries surrounding a lane.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct LaneBoundaryId
    {
        public int far_left_lb_id;
        public int left_lb_id;
        public int right_lb_id;
        public int far_right_lb_id;
    }

    /// <summary>
    /// Represents the difference in position between two objects or locations.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct PositionDiff
    {
        public int dLaneId;
        public double ds;
        public double dt;
        public double dx;
        public double dy;
        [MarshalAs(UnmanagedType.I1)]
        public bool oppositeLanes;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Center
    {
        public double x_;
        public double y_;
        public double z_;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Dimensions
    {
        public double width_;
        public double length_;
        public double height_;
    }

    /// <summary>
    /// Represents an oriented bounding box.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct OSCBoundingBox
    {
        public Center center_;
        public Dimensions dimensions_;
    }

    /// <summary>
    /// Data for a specific wheel of a vehicle.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct WheelData
    {
        public double x;
        public double y;
        public double z;
        public double h;
        public double p;
        public double friction_coefficient;
        public int axle;
        public int index;
    }

    /// <summary>
    /// Information about a road sign.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RoadSign
    {
        public int id;
        public IntPtr name; // const char*
        public double x;
        public double y;
        public double z;
        public double h;
        public double s;
        public double t;
        public int orientation;
        public double z_offset;
        public double length;
        public double height;
        public double width;
    }

    /// <summary>
    /// Validity record for a road object (e.g., sign), defining the lanes it applies to.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RoadObjValidity
    {
        public int fromLane;
        public int toLane;
    }

    /// <summary>
    /// Represents an image buffer.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Image
    {
        public int width;
        public int height;
        public int pixelSize;
        public int pixelFormat;
        public IntPtr data; // unsigned char*
    }

    /// <summary>
    /// Information about a point along a route.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RouteInfo
    {
        public double x;
        public double y;
        public double z;
        public double h;
        public int roadId;
        public int junctionId;
        public int laneId;
        public int osiLaneId;
        public double laneOffset;
        public double s;
        public double t;
    }

    /// <summary>
    /// A generic parameter key-value pair.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Parameter
    {
        public string name;
        public string value;
    }

    /// <summary>
    /// A generic variable key-value pair.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Variable
    {
        public string name;
        public string value;
    }

    // Helper structs for OverrideActionList
    [StructLayout(LayoutKind.Sequential)]
    public struct ActionStatus
    {
        [MarshalAs(UnmanagedType.I1)] public bool active;
        public double maxRate;
        public int type;
        public double value;
        public int value_type;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GearActionStatus
    {
        [MarshalAs(UnmanagedType.I1)] public bool active;
        public double number;
        public int type;
        public int value_type;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ClutchActionStatus
    {
        [MarshalAs(UnmanagedType.I1)] public bool active;
        public double value;
        public double maxRate;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ThrottleActionStatus
    {
        [MarshalAs(UnmanagedType.I1)] public bool active;
        public double value;
        public double maxRate;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SteeringActionStatus
    {
        [MarshalAs(UnmanagedType.I1)] public bool active;
        public double maxRate;
        public double maxTorque;
        public double value;
    }

    /// <summary>
    /// List of override actions currently active on an object.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct OverrideActionList
    {
        public ActionStatus brake;
        public ActionStatus parkingBrake;
        public GearActionStatus gear;
        public ClutchActionStatus clutch;
        public ThrottleActionStatus throttle;
        public SteeringActionStatus steeringWheel;
    }

    public static class ESMiniLib
    {
        private const string LIB_NAME = "esminiLib";
        private const CallingConvention CALL_CONV = CallingConvention.Cdecl;

        /// <summary>
        /// Adds a path to the list of paths searched for resources (e.g., models, catalogs).
        /// </summary>
        /// <param name="path">The directory path to add.</param>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddPath(string path);

        /// <summary>
        /// Clears all added resource search paths.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_ClearPaths();

        /// <summary>
        /// Sets the file path for the log file.
        /// </summary>
        /// <param name="logFilePath">The full path to the log file.</param>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SetLogFilePath(string logFilePath);

        /// <summary>
        /// Sets the file path for the recording (.dat) file.
        /// </summary>
        /// <param name="datFilePath">The full path to the .dat file.</param>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SetDatFilePath(string datFilePath);

        /// <summary>
        /// Gets the current seed used for random number generation.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern uint SE_GetSeed();

        /// <summary>
        /// Sets the seed for random number generation.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SetSeed(uint seed);

        /// <summary>
        /// Sets a boolean option flag (equivalent to setting it to "true").
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetOption(string name);

        /// <summary>
        /// Unsets an option flag.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_UnsetOption(string name);

        /// <summary>
        /// Sets an option with a specific value.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetOptionValue(string name, string value);

        /// <summary>
        /// Sets a persistent option flag that survives scenario resets.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetOptionPersistent(string name);

        /// <summary>
        /// Sets a persistent option value that survives scenario resets.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetOptionValuePersistent(string name, string value);

        /// <summary>
        /// Gets the value of an option.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOptionValue(string name);

        /// <summary>
        /// Gets an option value by its enum identifier.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOptionValueByEnum(uint enum_value);

        /// <summary>
        /// Gets a specific value from a list of values for an option.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOptionValueByIndex(string name, uint index);

        /// <summary>
        /// Gets the number of values associated with an option.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetOptionValuesCount(string name);

        /// <summary>
        /// Checks if an option is set.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool SE_GetOptionSet(string name);

        /// <summary>
        /// Loads a parameter distribution file.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetParameterDistribution(string filename);

        /// <summary>
        /// Resets the parameter distribution.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_ResetParameterDistribution();

        /// <summary>
        /// Gets the total number of permutations in the loaded parameter distribution.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern uint SE_GetNumberOfPermutations();

        /// <summary>
        /// Selects a specific permutation index for the next simulation run.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SelectPermutation(uint index);

        /// <summary>
        /// Gets the index of the currently selected permutation.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetPermutationIndex();

        /// <summary>
        /// Sets the position and size of the viewer window.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SetWindowPosAndSize(int x, int y, int w, int h);

        /// <summary>
        /// Sets tolerances for OSI (Open Simulation Interface) reporting.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetOSITolerances(double maxLongitudinalDistance, double maxLateralDeviation);

        /// <summary>
        /// Initializes the engine with command line arguments.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_InitWithArgs(int argc, string[] argv);

        /// <summary>
        /// Initializes the engine with an OpenSCENARIO XML string.
        /// </summary>
        /// <param name="oscAsXMLString">The OpenSCENARIO content as a string.</param>
        /// <param name="disable_ctrls">1 to disable internal controllers, 0 otherwise.</param>
        /// <param name="use_viewer">Bitmask: 1=enable viewer, 2=off-screen, 4=capture-to-file, 8=disable info-text.</param>
        /// <param name="threads">1 to enable threading, 0 otherwise.</param>
        /// <param name="record">1 to enable recording, 0 otherwise.</param>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_InitWithString(string oscAsXMLString, int disable_ctrls, int use_viewer, int threads, int record);

        public delegate void ParameterDeclarationCallback(IntPtr user_data);
        /// <summary>
        /// Registers a callback to be called during parameter declaration.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_RegisterParameterDeclarationCallback(ParameterDeclarationCallback callback, IntPtr user_data);

        /// <summary>
        /// Initializes the engine with an OpenSCENARIO file.
        /// </summary>
        /// <param name="oscFilename">Path to the OpenSCENARIO file.</param>
        /// <param name="disable_ctrls">1 to disable internal controllers, 0 otherwise.</param>
        /// <param name="use_viewer">Bitmask: 1=enable viewer, 2=off-screen, 4=capture-to-file, 8=disable info-text.</param>
        /// <param name="threads">1 to enable threading, 0 otherwise.</param>
        /// <param name="record">1 to enable recording, 0 otherwise.</param>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_Init(string oscFilename, int disable_ctrls, int use_viewer, int threads, int record);

        /// <summary>
        /// Checks if the quit flag has been set (e.g., by closing the window or end of scenario).
        /// </summary>
        /// <returns>1 if quit is requested, 0 otherwise.</returns>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetQuitFlag();

        /// <summary>
        /// Checks if the simulation is currently paused.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetPauseFlag();

        /// <summary>
        /// Gets the filename of the OpenDRIVE road network being used.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetODRFilename();

        /// <summary>
        /// Gets the filename of the OpenSceneGraph scene graph being used.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetSceneGraphFilename();

        /// <summary>
        /// Gets the number of parameters defined in the scenario.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetNumberOfParameters();

        /// <summary>
        /// Gets the name and type of a parameter by index.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetParameterName(int index, out int type);

        /// <summary>
        /// Gets the number of variables defined in the scenario.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetNumberOfVariables();

        /// <summary>
        /// Gets the name and type of a variable by index.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetVariableName(int index, out int type);

        /// <summary>
        /// Gets the number of properties for a specific object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetNumberOfProperties(int index);

        /// <summary>
        /// Gets the name of a property for a specific object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetObjectPropertyName(int index, int propertyIndex);

        /// <summary>
        /// Gets the value of a property for a specific object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetObjectPropertyValue(int index, string objectPropertyName);

        /// <summary>
        /// Sets a parameter value.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetParameter(Parameter parameter);

        /// <summary>
        /// Gets a parameter value.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetParameter(ref Parameter parameter);

        /// <summary>
        /// Gets a parameter value as an integer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetParameterInt(string parameterName, out int value);

        /// <summary>
        /// Gets a parameter value as a double.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetParameterDouble(string parameterName, out double value);

        /// <summary>
        /// Gets a parameter value as a string.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetParameterString(string parameterName, out IntPtr value);

        /// <summary>
        /// Gets a parameter value as a boolean.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetParameterBool(string parameterName, [MarshalAs(UnmanagedType.I1)] out bool value);

        /// <summary>
        /// Sets a parameter value as an integer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetParameterInt(string parameterName, int value);

        /// <summary>
        /// Sets a parameter value as a double.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetParameterDouble(string parameterName, double value);

        /// <summary>
        /// Sets a parameter value as a string.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetParameterString(string parameterName, string value);

        /// <summary>
        /// Sets a parameter value as a boolean.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetParameterBool(string parameterName, [MarshalAs(UnmanagedType.I1)] bool value);

        /// <summary>
        /// Sets a variable value.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetVariable(Variable variable);

        /// <summary>
        /// Gets a variable value.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetVariable(ref Variable variable);

        /// <summary>
        /// Gets a variable value as an integer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetVariableInt(string variableName, out int value);

        /// <summary>
        /// Gets a variable value as a double.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetVariableDouble(string variableName, out double value);

        /// <summary>
        /// Gets a variable value as a string.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetVariableString(string variableName, out IntPtr value);

        /// <summary>
        /// Gets a variable value as a boolean.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetVariableBool(string variableName, [MarshalAs(UnmanagedType.I1)] out bool value);

        /// <summary>
        /// Sets a variable value as an integer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetVariableInt(string variableName, int value);

        /// <summary>
        /// Sets a variable value as a double.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetVariableDouble(string variableName, double value);

        /// <summary>
        /// Sets a variable value as a string.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetVariableString(string variableName, string value);

        /// <summary>
        /// Sets a variable value as a boolean.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetVariableBool(string variableName, [MarshalAs(UnmanagedType.I1)] bool value);

        /// <summary>
        /// Gets the pointer to the OpenDRIVE manager.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetODRManager();

        /// <summary>
        /// Closes the scenario engine and releases resources.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_Close();

        /// <summary>
        /// Enables or disables logging to the console (stdout).
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_LogToConsole([MarshalAs(UnmanagedType.I1)] bool mode);

        /// <summary>
        /// Enables or disables collision detection.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_CollisionDetection([MarshalAs(UnmanagedType.I1)] bool mode);

        /// <summary>
        /// Steps the simulation by one frame (using the default or previously set timestep).
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_Step();

        /// <summary>
        /// Steps the simulation by a specific time delta.
        /// </summary>
        /// <param name="dt">The time step in seconds.</param>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_StepDT(double dt);

        /// <summary>
        /// Gets the current simulation time.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern double SE_GetSimulationTime();

        /// <summary>
        /// Gets the current simulation time step.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern double SE_GetSimTimeStep();

        /// <summary>
        /// Sets the position mode for an object (e.g., relative to road, global coordinates).
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SetObjectPositionMode(int object_id, int type, int mode);

        /// <summary>
        /// Sets the default position mode for an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SetObjectPositionModeDefault(int object_id, int type);

        /// <summary>
        /// Adds a new object to the scenario.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddObject(string object_name, int object_type, int object_category, int object_role, int model_id, string model_3d);

        /// <summary>
        /// Adds a new object with a specific bounding box.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddObjectWithBoundingBox(string object_name, int object_type, int object_category, int object_role, int model_id, string model_3d, OSCBoundingBox bounding_box, int scale_mode);

        /// <summary>
        /// Deletes an object from the scenario.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_DeleteObject(int object_id);

        /// <summary>
        /// Reports (updates) the position and rotation of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectPos(int object_id, double x, double y, double z, double h, double p, double r);

        /// <summary>
        /// Reports the position of an object with a specific mode.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectPosMode(int object_id, double x, double y, double z, double h, double p, double r, int mode);

        /// <summary>
        /// Reports the position (X, Y) and heading (H) of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectPosXYH(int object_id, double x, double y, double h);

        /// <summary>
        /// Reports the position of an object relative to the road network (RoadId, LaneId, S, Offset).
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectRoadPos(int object_id, int roadId, int laneId, double laneOffset, double s);

        /// <summary>
        /// Reports the speed of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectSpeed(int object_id, double speed);

        /// <summary>
        /// Reports the lateral position (t-coordinate) of an object on the current road.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectLateralPosition(int object_id, double t);

        /// <summary>
        /// Reports the lateral position of an object relative to a specific lane.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectLateralLanePosition(int object_id, int laneId, double laneOffset);

        /// <summary>
        /// Reports the velocity vector of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectVel(int object_id, double x_vel, double y_vel, double z_vel);

        /// <summary>
        /// Reports the angular velocity of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectAngularVel(int object_id, double h_rate, double p_rate, double r_rate);

        /// <summary>
        /// Reports the acceleration vector of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectAcc(int object_id, double x_acc, double y_acc, double z_acc);

        /// <summary>
        /// Reports the angular acceleration of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectAngularAcc(int object_id, double h_acc, double p_acc, double r_acc);

        /// <summary>
        /// Reports the wheel status (rotation and steering angle) of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ReportObjectWheelStatus(int object_id, double rotation, double angle);

        /// <summary>
        /// Sets the lane types that an object should snap to.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetSnapLaneTypes(int object_id, int laneTypes);

        /// <summary>
        /// Enables or disables locking an object to its current lane.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetLockOnLane(int object_id, [MarshalAs(UnmanagedType.I1)] bool mode);

        /// <summary>
        /// Gets the total number of objects in the scenario.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetNumberOfObjects();

        /// <summary>
        /// Gets the ID of an object by its index.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetId(int index);

        /// <summary>
        /// Gets the ID of an object by its name.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetIdByName(string name);

        /// <summary>
        /// Retrieves the full state of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectState(int object_id, ref ScenarioObjectState state);

        /// <summary>
        /// Gets the route status of an object (e.g., on route, off route).
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectRouteStatus(int object_id);

        /// <summary>
        /// Gets the type of lane the object is currently in.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectInLaneType(int object_id);

        /// <summary>
        /// Gets the status of override actions (e.g., throttle, brake override) for an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetOverrideActionStatus(int objectId, ref OverrideActionList list);

        /// <summary>
        /// Gets the type name of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetObjectTypeName(int object_id);

        /// <summary>
        /// Gets the name of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetObjectName(int object_id);

        /// <summary>
        /// Gets the 3D model filename associated with an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetObjectModelFileName(int object_id);

        /// <summary>
        /// Opens a socket for sending OSI data.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_OpenOSISocket(string ipaddr);

        /// <summary>
        /// Sets the mode for OSI static data reporting.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SetOSIStaticReportMode(int mode);

        /// <summary>
        /// Gets the OSI GroundTruth data.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOSIGroundTruth(ref int size);

        /// <summary>
        /// Gets the raw OSI GroundTruth data pointer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOSIGroundTruthRaw();

        /// <summary>
        /// Gets the raw OSI TrafficCommand data pointer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOSITrafficCommandRaw();

        /// <summary>
        /// Sets raw OSI SensorData.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetOSISensorDataRaw(string sensordata);

        /// <summary>
        /// Gets OSI lane data for a specific object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOSIRoadLane(ref int size, int object_id);

        /// <summary>
        /// Gets OSI lane boundary data.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOSILaneBoundary(ref int size, int g_id);

        /// <summary>
        /// Gets the IDs of lane boundaries surrounding an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_GetOSILaneBoundaryIds(int object_id, ref LaneBoundaryId ids);

        /// <summary>
        /// Excludes the ghost object from OSI GroundTruth.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_ExcludeGhostFromGroundTruth();

        /// <summary>
        /// Sets the frequency of OSI updates.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetOSIFrequency(int frequency);

        /// <summary>
        /// Crops the OSI dynamic ground truth to a radius around an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_CropOSIDynamicGroundTruth(int id, double radius);

        /// <summary>
        /// Updates the OSI TrafficCommand.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_UpdateOSITrafficCommand();

        /// <summary>
        /// Gets the raw OSI SensorData pointer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetOSISensorDataRaw();

        /// <summary>
        /// Sets the timestamp for OSI messages.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_OSISetTimeStamp(ulong nanoseconds);

        /// <summary>
        /// Logs a message to the esmini log.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_LogMessage(string message);

        /// <summary>
        /// Closes the log file.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_CloseLogFile();

        /// <summary>
        /// Checks if an object has a ghost object associated with it.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ObjectHasGhost(int object_id);

        /// <summary>
        /// Gets the ID of the ghost object associated with an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectGhostId(int object_id);

        /// <summary>
        /// Gets the state of the ghost object associated with an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectGhostState(int object_id, ref ScenarioObjectState state);

        /// <summary>
        /// Gets the speed unit used in the OpenDRIVE file.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetSpeedUnit();

        /// <summary>
        /// Gets the number of collisions involving an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectNumberOfCollisions(int object_id);

        /// <summary>
        /// Gets the ID of the object involved in a specific collision.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectCollision(int object_id, int index);

        /// <summary>
        /// Gets the odometer value (distance traveled) for an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern double SE_GetObjectOdometer(int object_id);

        /// <summary>
        /// Gets the scalar acceleration of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern double SE_GetObjectAcceleration(int object_id);

        /// <summary>
        /// Gets the global velocity vector (X, Y, Z) of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectVelocityGlobalXYZ(int object_id, out double vel_x, out double vel_y, out double vel_z);

        /// <summary>
        /// Gets the angular velocity (roll, pitch, heading rates) of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectAngularVelocity(int object_id, out double h_rate, out double p_rate, out double r_rate);

        /// <summary>
        /// Gets the angular acceleration of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectAngularAcceleration(int object_id, out double h_acc, out double p_acc, out double r_acc);

        /// <summary>
        /// Gets the global acceleration vector (X, Y, Z) of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectAccelerationGlobalXYZ(int object_id, out double acc_x, out double acc_y, out double acc_z);

        /// <summary>
        /// Gets the local acceleration (lateral, longitudinal) of an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectAccelerationLocalLatLong(int object_id, out double acc_lat, out double acc_long);

        /// <summary>
        /// Gets the number of wheels on a vehicle object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectNumberOfWheels(int object_id);

        /// <summary>
        /// Gets data for a specific wheel of a vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectWheelData(int object_id, int wheel_index, ref WheelData wheeldata);

        /// <summary>
        /// Gets the states of all objects in the scenario.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectStates(out int nObjects, [In, Out] ScenarioObjectState[] state);

        /// <summary>
        /// Adds a sensor to an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddObjectSensor(int object_id, double x, double y, double z, double h, double rangeNear, double rangeFar, double fovH, int maxObj);

        /// <summary>
        /// Gets the total number of object sensors.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetNumberOfObjectSensors();

        /// <summary>
        /// Visualizes sensor data for an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_ViewSensorData(int object_id);

        /// <summary>
        /// Disables writing to the OSI file.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_DisableOSIFile();

        /// <summary>
        /// Enables writing to an OSI file.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_EnableOSIFile(string filename);

        /// <summary>
        /// Flushes the OSI file buffer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_FlushOSIFile();

        /// <summary>
        /// Fetches the list of objects detected by a sensor.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_FetchSensorObjectList(int sensor_id, [In, Out] int[] list);

        /// <summary>
        /// Retrieves information about the road at a specific distance ahead of an object.
        /// </summary>
        /// <param name="object_id">The ID of the object (e.g., vehicle) to query from.</param>
        /// <param name="lookahead_distance">The distance ahead (in meters) to query the road info.</param>
        /// <param name="data">Reference to a RoadInfo struct to be populated with the road information.</param>
        /// <param name="lookAheadMode">Strategy for lookahead: 0 = along road reference line, 1 = along lane center.</param>
        /// <param name="inRoadDrivingDirection">If true, the lookahead is performed in the direction of the lane's traffic flow. If false, it uses the object's heading.</param>
        /// <returns>0 on success, non-zero on failure.</returns>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetRoadInfoAtDistance(int object_id, double lookahead_distance, ref RoadInfo data, int lookAheadMode, [MarshalAs(UnmanagedType.I1)] bool inRoadDrivingDirection);

        /// <summary>
        /// Retrieves information about the road at a specific distance ahead of an object, following the object's assigned route.
        /// </summary>
        /// <param name="object_id">The ID of the object (e.g., vehicle) to query from.</param>
        /// <param name="lookahead_distance">The distance ahead (in meters) to query the road info.</param>
        /// <param name="data">Reference to a RoadInfo struct to be populated with the road information.</param>
        /// <param name="lookAheadMode">Strategy for lookahead: 0 = along road reference line, 1 = along lane center.</param>
        /// <param name="inRoadDrivingDirection">If true, the lookahead is performed in the direction of the lane's traffic flow. If false, it uses the object's heading.</param>
        /// <returns>0 on success, non-zero on failure (e.g., object not found or off-route).</returns>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetRoadInfoAlongRoute(int object_id, double lookahead_distance, ref RoadInfo data, int lookAheadMode, [MarshalAs(UnmanagedType.I1)] bool inRoadDrivingDirection);

        /// <summary>
        /// Gets road information along the ghost object's trail at a lookahead distance.
        /// </summary>
        /// <param name="object_id">The ID of the object.</param>
        /// <param name="lookahead_distance">The distance ahead to query.</param>
        /// <param name="data">Reference to a RoadInfo struct to be populated.</param>
        /// <param name="speed_ghost">Output parameter for the ghost's speed at that point.</param>
        /// <param name="timestamp">Output parameter for the timestamp at that point.</param>
        /// <returns>0 on success, non-zero on failure.</returns>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetRoadInfoAlongGhostTrail(int object_id, double lookahead_distance, ref RoadInfo data, out double speed_ghost, out double timestamp);

        /// <summary>
        /// Gets road information along the ghost object's trail at a specific time.
        /// </summary>
        /// <param name="object_id">The ID of the object.</param>
        /// <param name="time">The simulation time to query.</param>
        /// <param name="data">Reference to a RoadInfo struct to be populated.</param>
        /// <param name="speed_ghost">Output parameter for the ghost's speed at that time.</param>
        /// <returns>0 on success, non-zero on failure.</returns>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetRoadInfoGhostTrailTime(int object_id, double time, ref RoadInfo data, out double speed_ghost);

        /// <summary>
        /// Gets the distance between two objects.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetDistanceToObject(int object_a_id, int object_b_id, [MarshalAs(UnmanagedType.I1)] bool free_space, ref PositionDiff pos_diff);

        /// <summary>
        /// Gets a simplified distance between two objects.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SimpleGetDistanceToObject(int object_a_id, int object_b_id, int dist_type, double tracking_limit, out double distance, out double timestamp);

        public delegate void ObjectCallback(ref ScenarioObjectState state, IntPtr user_data);
        /// <summary>
        /// Registers a callback to be called for a specific object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_RegisterObjectCallback(int object_id, ObjectCallback callback, IntPtr user_data);

        public delegate void ConditionCallback(string name, double timestamp);
        /// <summary>
        /// Registers a callback to be called when a condition is triggered.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_RegisterConditionCallback(ConditionCallback callback);

        public delegate void StateChangeCallback(string name, int type, int state, string full_path);
        /// <summary>
        /// Registers a callback to be called when a storyboard element changes state.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_RegisterStoryBoardElementStateChangeCallback(StateChangeCallback callback);

        /// <summary>
        /// Gets the number of road signs on a specific road.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern uint SE_GetNumberOfRoadSigns(int road_id);

        /// <summary>
        /// Gets information about a specific road sign.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetRoadSign(int road_id, uint index, ref RoadSign road_sign);

        /// <summary>
        /// Gets the number of validity records for a road sign.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern uint SE_GetNumberOfRoadSignValidityRecords(int road_id, uint index);

        /// <summary>
        /// Gets a validity record for a road sign.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetRoadSignValidityRecord(int road_id, uint signIndex, uint validityIndex, ref RoadObjValidity validity);

        /// <summary>
        /// Gets the string ID of a road from its integer ID.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetRoadIdString(int road_id);

        /// <summary>
        /// Gets the integer ID of a road from its string ID.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetRoadIdFromString(string road_id_str);

        /// <summary>
        /// Gets the string ID of a junction from its integer ID.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_GetJunctionIdString(int junction_id);

        /// <summary>
        /// Gets the integer ID of a junction from its string ID.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetJunctionIdFromString(string junction_id_str);

        /// <summary>
        /// Shows or hides a specific feature in the viewer.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_ViewerShowFeature(int featureType, [MarshalAs(UnmanagedType.I1)] bool enable);

        /// <summary>
        /// Creates a simple vehicle model.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern IntPtr SE_SimpleVehicleCreate(double x, double y, double h, double length, double speed);

        /// <summary>
        /// Deletes a simple vehicle model.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleDelete(IntPtr handleSimpleVehicle);

        /// <summary>
        /// Controls a simple vehicle with binary inputs (throttle/brake, steering).
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleControlBinary(IntPtr handleSimpleVehicle, double dt, int throttle, int steering);

        /// <summary>
        /// Controls a simple vehicle with analog inputs.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleControlAnalog(IntPtr handleSimpleVehicle, double dt, double throttle, double steering);

        /// <summary>
        /// Controls a simple vehicle with acceleration and steering angle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleControlAccAndSteer(IntPtr handleSimpleVehicle, double dt, double acceleration, double steering_angle);

        /// <summary>
        /// Sets the speed of a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSetSpeed(IntPtr handleSimpleVehicle, double speed);

        /// <summary>
        /// Disables throttle control for a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSetThrottleDisabled(IntPtr handleSimpleVehicle, [MarshalAs(UnmanagedType.I1)] bool disabled);

        /// <summary>
        /// Disables steering control for a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSetSteeringDisabled(IntPtr handleSimpleVehicle, [MarshalAs(UnmanagedType.I1)] bool disabled);

        /// <summary>
        /// Controls a simple vehicle to reach a target speed and heading.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleControlTarget(IntPtr handleSimpleVehicle, double dt, double target_speed, double heading_to_target);

        /// <summary>
        /// Sets the maximum speed of a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSetMaxSpeed(IntPtr handleSimpleVehicle, double speed);

        /// <summary>
        /// Sets the maximum acceleration of a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSetMaxAcceleration(IntPtr handleSimpleVehicle, double maxAcceleration);

        /// <summary>
        /// Sets the maximum deceleration of a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSetMaxDeceleration(IntPtr handleSimpleVehicle, double maxDeceleration);

        /// <summary>
        /// Sets the engine brake factor for a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSetEngineBrakeFactor(IntPtr handleSimpleVehicle, double engineBrakeFactor);

        /// <summary>
        /// Sets the steering scale for a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSteeringScale(IntPtr handleSimpleVehicle, double steeringScale);

        /// <summary>
        /// Sets the steering return factor for a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSteeringReturnFactor(IntPtr handleSimpleVehicle, double steeringReturnFactor);

        /// <summary>
        /// Sets the steering rate for a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleSteeringRate(IntPtr handleSimpleVehicle, double steeringRate);

        /// <summary>
        /// Gets the state of a simple vehicle.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_SimpleVehicleGetState(IntPtr handleSimpleVehicle, ref SimpleVehicleState state);

        /// <summary>
        /// Enables or disables saving images to RAM.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SaveImagesToRAM([MarshalAs(UnmanagedType.I1)] bool state);

        /// <summary>
        /// Saves a sequence of images to files.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SaveImagesToFile(int nrOfFrames);

        /// <summary>
        /// Fetches the latest captured image.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_FetchImage(ref Image image);

        public delegate void ImageCallback(ref Image image, IntPtr user_data);
        /// <summary>
        /// Registers a callback to receive captured images.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_RegisterImageCallback(ImageCallback callback, IntPtr user_data);

        /// <summary>
        /// Writes an image to a PPM file.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_WritePPMImage(string filename, int width, int height, IntPtr data, int pixelSize, int pixelFormat, [MarshalAs(UnmanagedType.I1)] bool upsidedown);

        /// <summary>
        /// Writes an image to a TGA file.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_WriteTGAImage(string filename, int width, int height, IntPtr data, int pixelSize, int pixelFormat, [MarshalAs(UnmanagedType.I1)] bool upsidedown);

        /// <summary>
        /// Adds a custom camera at a specific position and orientation.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddCustomCamera(double x, double y, double z, double h, double p);

        /// <summary>
        /// Adds a custom fixed camera.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddCustomFixedCamera(double x, double y, double z, double h, double p);

        /// <summary>
        /// Adds a custom camera aiming at a target.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddCustomAimingCamera(double x, double y, double z);

        /// <summary>
        /// Adds a custom fixed camera aiming at a target.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddCustomFixedAimingCamera(double x, double y, double z);

        /// <summary>
        /// Adds a custom fixed top-down camera.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_AddCustomFixedTopCamera(double x, double y, double z, double rot);

        /// <summary>
        /// Sets the camera mode.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetCameraMode(int mode);

        /// <summary>
        /// Sets the camera to focus on a specific object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_SetCameraObjectFocus(int object_id);

        /// <summary>
        /// Gets the ID of the object currently in camera focus.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetObjectInCameraFocus();

        /// <summary>
        /// Gets the current camera position and orientation.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetCameraPos(out double x, out double y, out double z, out double h, out double p, out double r);

        /// <summary>
        /// Gets the number of route points for an object.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetNumberOfRoutePoints(int object_id);

        /// <summary>
        /// Gets information about a specific route point.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern int SE_GetRoutePoint(int object_id, uint route_index, ref RouteInfo routeinfo);

        /// <summary>
        /// Gets the total length of an object's route.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern double SE_GetRouteTotalLength(int object_id);

        /// <summary>
        /// Injects a speed action.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_InjectSpeedAction(IntPtr action);

        /// <summary>
        /// Injects a lane change action.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_InjectLaneChangeAction(IntPtr action);

        /// <summary>
        /// Injects a lane offset action.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        public static extern void SE_InjectLaneOffsetAction(IntPtr action);

        /// <summary>
        /// Checks if an injected action of a specific type is currently ongoing.
        /// </summary>
        [DllImport(LIB_NAME, CallingConvention = CALL_CONV)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool SE_InjectedActionOngoing(int action_type);
    }
}
