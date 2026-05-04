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
 * Created by generate_esminiLib_cs_wrapper.py from esminiLib.hpp
 */

using System;
using System.Runtime.InteropServices;

namespace ESMini
{
    public static class EsminiDefines
    {
        public const string SE_DLL_API = "__declspec(dllexport)";

        public const uint SE_ID_UNDEFINED = 0xffffffff;

        public const uint SE_IDX_UNDEFINED = 0xffffffff;

        public const int SE_PARAM_NAME_SIZE = 32;
    }

    /// <summary>
    /// Modes for interpret Z, Head, Pitch, Roll coordinate value as absolute or relative
    /// grouped as bitmask: 0000 =&gt; skip/use current, 0001=DEFAULT, 0011=ABS, 0111=REL
    /// example: Relative Z, Absolute H, Default R, Current P = SE_Z_REL | SE_H_ABS | SE_R_DEF = 4151 = 0001 0000 0011 0111
    /// Must match roadmanager::Position::PositionMode
    /// </summary>
    public enum SE_PositionMode
    {
        /// <summary>0001</summary>
        SE_Z_SET = 1,
        /// <summary>0001</summary>
        SE_Z_DEFAULT = 1,
        /// <summary>0011</summary>
        SE_Z_ABS = 3,
        /// <summary>0111</summary>
        SE_Z_REL = 7,
        /// <summary>0111</summary>
        SE_Z_MASK = 7,
        SE_H_SET = SE_Z_SET << 4,
        SE_H_DEFAULT = SE_Z_DEFAULT << 4,
        SE_H_ABS = SE_Z_ABS << 4,
        SE_H_REL = SE_Z_REL << 4,
        SE_H_MASK = SE_Z_MASK << 4,
        SE_P_SET = SE_Z_SET << 8,
        SE_P_DEFAULT = SE_Z_DEFAULT << 8,
        SE_P_ABS = SE_Z_ABS << 8,
        SE_P_REL = SE_Z_REL << 8,
        SE_P_MASK = SE_Z_MASK << 8,
        SE_R_SET = SE_Z_SET << 12,
        SE_R_DEFAULT = SE_Z_DEFAULT << 12,
        SE_R_ABS = SE_Z_ABS << 12,
        SE_R_REL = SE_Z_REL << 12,
        SE_R_MASK = SE_Z_MASK << 12,
        /// <summary>Map position to closest lane along route (if 1) or any lane (if 0)</summary>
        SE_SNAP_TO_ROUTE_SET = SE_Z_SET << 16,
        SE_SNAP_TO_ROUTE_DEFAULT = SE_Z_DEFAULT << 16,
        SE_SNAP_TO_ROUTE_OFF = SE_Z_ABS << 16,
        SE_SNAP_TO_ROUTE_ON = SE_Z_REL << 16,
        SE_SNAP_TO_ROUTE_MASK = SE_Z_MASK << 16,
    }

    public enum SE_PositionModeType
    {
        /// <summary>Used by explicit set functions</summary>
        SE_SET = 1,
        /// <summary>Used by controllers updating the position</summary>
        SE_UPDATE = 2,
    }

    public enum SE_GhostTrailReturnCode
    {
        /// <summary>success</summary>
        SE_GHOST_TRAIL_OK = 0,
        /// <summary>generic error</summary>
        SE_GHOST_TRAIL_ERROR = -1,
        /// <summary>ghost trail trajectory has no vertices</summary>
        SE_GHOST_TRAIL_NO_VERTICES = -2,
        /// <summary>given time < first timestamp in trajectory, snapped to start of trajectory</summary>
        SE_GHOST_TRAIL_TIME_PRIOR = -3,
        /// <summary>given time > last timestamp in trajectory, snapped to end of trajectory</summary>
        SE_GHOST_TRAIL_TIME_PAST = -4,
        /// <summary>given distance > last vertex in trajectory, snapped to end of trajectory</summary>
        SE_GHOST_TRAIL_DIST_PAST = -5,
    }

    public enum SE_RelativeDistanceType
    {
        REL_DIST_UNDEFINED = 0,
        REL_DIST_LATERAL = 1,
        REL_DIST_LONGITUDINAL = 2,
        REL_DIST_CARTESIAN = 3,
        REL_DIST_EUCLIDIAN = 4,
    }

    public enum SE_OSIStaticReportMode
    {
        DEFAULT = 0,
        API = 1,
        API_AND_LOG = 2,
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_ScenarioObjectState
    {
        /// <summary>Automatically generated unique object id</summary>
        public int id;
        /// <summary>Id to control what 3D model to represent the vehicle - see carModelsFiles_[] in scenarioenginedll.cpp</summary>
        public int model_id;
        /// <summary>0: DefaultController 1: External. Further values see Controller::Type enum</summary>
        public int ctrl_type;
        /// <summary>Not used yet (idea is to use it to interpolate position for increased sync bewtween simulators)</summary>
        public double timestamp;
        /// <summary>global x coordinate of position</summary>
        public double x;
        /// <summary>global y coordinate of position</summary>
        public double y;
        /// <summary>global z coordinate of position</summary>
        public double z;
        /// <summary>heading/yaw in global coordinate system</summary>
        public double h;
        /// <summary>pitch in global coordinate system</summary>
        public double p;
        /// <summary>roll in global coordinate system</summary>
        public double r;
        /// <summary>road ID</summary>
        public uint roadId;
        /// <summary>Junction ID (-1 if not in a junction)</summary>
        public uint junctionId;
        /// <summary>lateral position in road coordinate system</summary>
        public double t;
        /// <summary>lane ID</summary>
        public int laneId;
        /// <summary>lateral offset from lane center</summary>
        public double laneOffset;
        /// <summary>longitudinal position in road coordinate system</summary>
        public double s;
        /// <summary>speed</summary>
        public double speed;
        /// <summary>x coordinate of bounding box center relative object reference point (local coordinate system)</summary>
        public double centerOffsetX;
        /// <summary>y coordinate of bounding box center relative object reference point (local coordinate system)</summary>
        public double centerOffsetY;
        /// <summary>z coordinate of bounding box center relative object reference point (local coordinate system)</summary>
        public double centerOffsetZ;
        /// <summary>width</summary>
        public double width;
        /// <summary>length</summary>
        public double length;
        /// <summary>height</summary>
        public double height;
        /// <summary>Main type according to entities.hpp / Object / Type</summary>
        public int objectType;
        /// <summary>Sub category within type, according to entities.hpp / Vehicle, Pedestrian, MiscObject / Category</summary>
        public int objectCategory;
        /// <summary>Steering angle of the wheel</summary>
        public double wheel_angle;
        /// <summary>Rotation angle of the wheel</summary>
        public double wheel_rot;
        /// <summary>bitmask according to Object::Visibility (1 = Graphics, 2 = Traffic, 4 = Sensors)</summary>
        public int visibilityMask;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_WheelData
    {
        /// <summary>global x coordinate of position</summary>
        public double x;
        /// <summary>global y coordinate of position</summary>
        public double y;
        /// <summary>global z coordinate of position</summary>
        public double z;
        /// <summary>heading/yaw in global coordinate system</summary>
        public double h;
        /// <summary>pitch in global coordinate system</summary>
        public double p;
        /// <summary>median radius of the wheel measured from the center of the wheel to the outer part of the tire</summary>
        public double wheel_radius;
        /// <summary>the value describes the kinetic friction of the tyre's contact point</summary>
        public double friction_coefficient;
        /// <summary>0=front, 1=next axle from front and so on. -1 indicates wheel is not existing.</summary>
        public int axle;
        /// <summary>The index of the wheel on the axle, counting in the direction of positive-y, that is, right-to-left. -1 indicates wheel</summary>
        public int index;
    }

    /// <summary>
    /// asciidoc tag::SE_RoadInfo_struct[]
    /// </summary>
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_RoadInfo
    {
        /// <summary>target position, in global coordinate system</summary>
        public double global_pos_x;
        /// <summary>target position, in global coordinate system</summary>
        public double global_pos_y;
        /// <summary>target position, in global coordinate system</summary>
        public double global_pos_z;
        /// <summary>target position, relative vehicle (pivot position object) coordinate system</summary>
        public double local_pos_x;
        /// <summary>target position, relative vehicle (pivot position object) coordinate system</summary>
        public double local_pos_y;
        /// <summary>target position, relative vehicle (pivot position object) coordinate system</summary>
        public double local_pos_z;
        /// <summary>heading angle to target from and relative vehicle (pivot position object) coordinate system</summary>
        public double angle;
        /// <summary>road heading at steering target point</summary>
        public double road_heading;
        /// <summary>road pitch (inclination) at steering target point</summary>
        public double road_pitch;
        /// <summary>road roll (camber) at target point</summary>
        public double road_roll;
        /// <summary>trail heading (only when used for trail lookups, else equals road_heading)</summary>
        public double trail_heading;
        /// <summary>road curvature at steering target point</summary>
        public double curvature;
        /// <summary>speed limit given by OpenDRIVE speed max entry in m/s</summary>
        public double speed_limit;
        /// <summary>target position, road ID</summary>
        public uint roadId;
        /// <summary>target position, junction ID (SE_ID_UNDEFINED if not in a junction)</summary>
        public uint junctionId;
        /// <summary>target position, lane ID</summary>
        public int laneId;
        /// <summary>target position, lane offset (lateral distance from lane center)</summary>
        public double laneOffset;
        /// <summary>target position, s (longitudinal distance along reference line)</summary>
        public double s;
        /// <summary>target position, t (lateral distance from reference line)</summary>
        public double t;
        /// <summary>road type given by OpenDRIVE road type, maps to roadmanager::Road::RoadType</summary>
        public int road_type;
        /// <summary>road rule given by OpenDRIVE rule entry, maps to roadmanager::Road::RoadRule</summary>
        public int road_rule;
        /// <summary>lane type given by OpenDRIVE lane type, maps to roadmanager::Road::LaneType</summary>
        public int lane_type;
        /// <summary>trail wheel angle (only when used for trail lookups, e.g. ghost, else 0)</summary>
        public double trail_wheel_angle;
    }

    /// <summary>
    /// asciidoc end::SE_RoadInfo_struct[]
    /// </summary>
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_RouteInfo
    {
        /// <summary>Route point in the global coordinate system</summary>
        public double x;
        /// <summary>Route point in the global coordinate system</summary>
        public double y;
        /// <summary>Route point in the global coordinate system</summary>
        public double z;
        /// <summary>Route point, heading in the global coordinate system</summary>
        public double h;
        /// <summary>Route point, road ID</summary>
        public uint roadId;
        /// <summary>Route point, junction ID (-1 if not in a junction)</summary>
        public uint junctionId;
        /// <summary>Route point, lane ID</summary>
        public int laneId;
        /// <summary>Route point, osi lane ID</summary>
        public int osiLaneId;
        /// <summary>Route point, lane offset (lateral distance from lane center)</summary>
        public double laneOffset;
        /// <summary>Route point, s (longitudinal distance along reference line)</summary>
        public double s;
        /// <summary>Route point, t (lateral distance from reference line)</summary>
        public double t;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_LaneBoundaryId
    {
        public uint far_left_lb_id;
        public uint left_lb_id;
        public uint right_lb_id;
        public uint far_right_lb_id;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_PositionDiff
    {
        /// <summary>delta s (longitudinal distance)</summary>
        public double ds;
        /// <summary>delta t (lateral distance)</summary>
        public double dt;
        /// <summary>delta laneId (increasing left and decreasing to the right)</summary>
        public int dLaneId;
        /// <summary>delta x (world coordinate system)</summary>
        public double dx;
        /// <summary>delta y (world coordinate system)</summary>
        public double dy;
        /// <summary>true if the two position objects are in opposite sides of reference lane</summary>
        [MarshalAs(UnmanagedType.I1)]
        public bool oppositeLanes;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_SimpleVehicleState
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

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_Parameter
    {
        /// <summary>Name of the parameter as defined in the OpenSCENARIO file</summary>
        public IntPtr name;
        /// <summary>Pointer to value which can be an integer, double, bool or string (const char*) as defined in the OpenSCENARIO file</summary>
        public IntPtr value;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_Variable
    {
        /// <summary>Name of the variable as defined in the OpenSCENARIO file</summary>
        public IntPtr name;
        /// <summary>Pointer to value which can be an integer, double, bool or string (const char*) as defined in the OpenSCENARIO file</summary>
        public IntPtr value;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_OverrideActionStatusGear
    {
        /// <summary>True: override; false: stop overriding</summary>
        [MarshalAs(UnmanagedType.I1)]
        public bool active;
        /// <summary>According to Entities::OverrideType</summary>
        public int type;
        /// <summary>Gear number/mode depending on value_type</summary>
        public int number;
        /// <summary>According to Entities::OverrideGearType</summary>
        public int value_type;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_OverrideActionStatusBrake
    {
        /// <summary>True: override; false: stop overriding</summary>
        [MarshalAs(UnmanagedType.I1)]
        public bool active;
        /// <summary>According to Entities::OverrideType</summary>
        public int type;
        /// <summary>BrakeType will define how to interpret the value</summary>
        public double value;
        /// <summary>-1.0 indicates not set</summary>
        public double maxRate;
        /// <summary>According to Entities::OverrideBrakeType</summary>
        public int value_type;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_OverrideActionStatusPedals
    {
        /// <summary>True: override; false: stop overriding</summary>
        [MarshalAs(UnmanagedType.I1)]
        public bool active;
        /// <summary>Depends on action, see SE_OverrideActionList</summary>
        public double value;
        /// <summary>-1.0 indicates not set</summary>
        public double maxRate;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_OverrideActionStatusSteering
    {
        /// <summary>True: override; false: stop overriding</summary>
        [MarshalAs(UnmanagedType.I1)]
        public bool active;
        /// <summary>Depends on action, see SE_OverrideActionList</summary>
        public double value;
        /// <summary>Depends on action, see SE_OverrideActionList, -1.0 indicates not set</summary>
        public double maxRate;
        /// <summary>Depends on action, see SE_OverrideActionList, -1.0 indicates not set</summary>
        public double maxTorque;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_OverrideActionList
    {
        /// <summary>Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the throttle pedal.</summary>
        public SE_OverrideActionStatusPedals throttle;
        /// <summary>Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the brake pedal.</summary>
        public SE_OverrideActionStatusBrake brake;
        /// <summary>Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the clutch pedal.</summary>
        public SE_OverrideActionStatusPedals clutch;
        /// <summary>Value range: [0..1]. 0 represents 0%, The value 1 represent the maximum parking brake state.</summary>
        public SE_OverrideActionStatusBrake parkingBrake;
        /// <summary>Steering wheel angle. Unit: rad. (0: Neutral position, positive: Left, negative: Right)</summary>
        public SE_OverrideActionStatusSteering steeringWheel;
        /// <summary>Gear status</summary>
        public SE_OverrideActionStatusGear gear;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_RoadSign
    {
        /// <summary>just an unique identifier of the sign</summary>
        public int id;
        /// <summary>global x coordinate of sign position</summary>
        public double x;
        /// <summary>global y coordinate of sign position</summary>
        public double y;
        /// <summary>global z coordinate of sign position</summary>
        public double z;
        /// <summary>z offset from road level</summary>
        public double z_offset;
        /// <summary>global heading of sign orientation</summary>
        public double h;
        /// <summary>road id of sign road position</summary>
        public int roadId;
        /// <summary>longitudinal position along road</summary>
        public double s;
        /// <summary>lateral position from road reference line</summary>
        public double t;
        /// <summary>sign name, typically used for 3D model filename</summary>
        public IntPtr name;
        /// <summary>1=facing traffic in road direction, -1=facing traffic opposite road direction</summary>
        public int orientation;
        /// <summary>length as specified in OpenDRIVE</summary>
        public double length;
        /// <summary>height as specified in OpenDRIVE</summary>
        public double height;
        /// <summary>width as specified in OpenDRIVE</summary>
        public double width;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_RoadObjValidity
    {
        public int fromLane;
        public int toLane;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_Image
    {
        public int width;
        public int height;
        /// <summary>3 for RGB/BGR</summary>
        public int pixelSize;
        /// <summary>0x1907=RGB (GL_RGB), 0x80E0=BGR (GL_BGR)</summary>
        public int pixelFormat;
        public IntPtr data;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_Center
    {
        /// <summary>Center offset in x direction.</summary>
        public double x_;
        /// <summary>Center offset in y direction.</summary>
        public double y_;
        /// <summary>Center offset in z direction.</summary>
        public double z_;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_Dimensions
    {
        /// <summary>Width of the entity's bounding box. Unit: m; Range: [0..inf[.</summary>
        public double width_;
        /// <summary>Length of the entity's bounding box. Unit: m; Range: [0..inf[.</summary>
        public double length_;
        /// <summary>Height of the entity's bounding box. Unit: m; Range: [0..inf[.</summary>
        public double height_;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_OSCBoundingBox
    {
        /// <summary>Represents the geometrical center of the bounding box</summary>
        public SE_Center center_;
        /// <summary>Width, length and height of the bounding box.</summary>
        public SE_Dimensions dimensions_;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_SpeedActionStruct
    {
        /// <summary>id of object to perform action</summary>
        public int id;
        public double speed;
        /// <summary>0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step</summary>
        public int transition_shape;
        /// <summary>0 = distance, 1 = rate, 2 = time</summary>
        public int transition_dim;
        public double transition_value;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_LaneChangeActionStruct
    {
        /// <summary>id of object to perform action</summary>
        public int id;
        /// <summary>0 = absolute, 1 = relative (own vehicle)</summary>
        public int mode;
        /// <summary>target lane id (absolute or relative)</summary>
        public int target;
        /// <summary>0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step</summary>
        public int transition_shape;
        /// <summary>0 = distance, 1 = rate, 2 = time</summary>
        public int transition_dim;
        public double transition_value;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SE_LaneOffsetActionStruct
    {
        /// <summary>id of object to perform action</summary>
        public int id;
        public double offset;
        /// <summary>0 = distance, 1 = rate, 2 = time</summary>
        public double maxLateralAcc;
        /// <summary>0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step</summary>
        public int transition_shape;
    }

    public static partial class ESMiniLib
    {
        private const string NativeLibrary = "esminiLib";

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void ParameterDeclarationCallback(IntPtr arg0);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void ObjectCallback(IntPtr arg0, IntPtr arg1);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void ConditionCallback(IntPtr name, double timestamp);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void StoryBoardElementStateChangeCallback(IntPtr name, int type, int state, IntPtr full_path);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void ImageCallback(IntPtr arg0, IntPtr arg1);

        /// <summary>
        /// Basic interface
        /// Add a search path for OpenDRIVE and 3D model files
        /// Needs to be called prior to SE_Init()
        /// </summary>
        /// <param name="path">Path to a directory</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddPath(string path);

        /// <summary>
        /// Clear all search paths for OpenDRIVE and 3D model files
        /// Needs to be called prior to SE_Init()
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_ClearPaths();

        /// <summary>
        /// Specify scenario logfile (.txt) file path,
        /// optionally including directory path and/or filename
        /// Specify only directory (end with "/" or "\") to let esmini set default filename
        /// Specify only filename (no leading "/" or "\") to let esmini set default directory
        /// Set "" to disable logfile
        /// examples:
        /// "../logfile.txt" (relative current directory)
        /// "c:/tmp/esmini.log" (absolute path)
        /// "my.log" (put it in current directory)
        /// "c:/tmp/" (use default filename)
        /// "" (prevent creation of logfile)
        /// Note: Needs to be called prior to calling SE_Init()
        /// </summary>
        /// <param name="path">Logfile path</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SetLogFilePath(string logFilePath);

        /// <summary>
        /// Specify scenario recording (.dat) file path,
        /// optionally including directory path and/or filename
        /// Specify only directory (end with "/" or "\") to let esmini set default filename
        /// Specify only filename (no leading "/" or "\") to let esmini set default directory
        /// Set "" to use default .dat filename
        /// examples:
        /// "../my_sim.dat" (relative current directory)
        /// "c:/tmp/esmini.dat" (absolute path)
        /// "my_sim.dat" (put it in current directory)
        /// "c:/tmp/" (use default filename)
        /// "" (use current directory and default .dat filename)
        /// Note: Needs to be called prior to calling SE_Init()
        /// </summary>
        /// <param name="path">Recording (.dat) file path</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SetDatFilePath(string datFilePath);

        /// <summary>
        /// Get seed that esmini uses for current session. It can then be re-used
        /// in order to achieve repeatable results (for actions that involes some
        /// degree of randomness, e.g. TrafficSwarmAction).
        /// </summary>
        /// <returns>seed number</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern uint SE_GetSeed();

        /// <summary>
        /// Set seed that will be used by esmini random number generator.
        /// Using same seed will ensure same result.
        /// Note: Also timesteps has to be equal. Make sure to use SE_StepDT()
        /// with fixed timestep, or at least same sequence of dt each run.
        /// </summary>
        /// <param name="seed">number</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SetSeed(uint seed);

        /// <summary>
        /// Set option. The option will be unset on next scenario run. If persistence is required check SE_SetOptionPersistent.
        /// </summary>
        /// <param name="name">the name of the option to be set</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetOption(string name);

        /// <summary>
        /// Unset option
        /// </summary>
        /// <param name="name">the name of the option to be unset</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_UnsetOption(string name);

        /// <summary>
        /// Set option value. The option's value will be unset on next scenario run. If persistence is required check SE_SetOptionValuePersistent
        /// </summary>
        /// <param name="name">the name of the option to be set</param>
        /// <param name="value">the value to assign to the option</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetOptionValue(string name, string value);

        /// <summary>
        /// Set option persistently. The option will remain over multiple scenario runs, until lib is reloaded.
        /// </summary>
        /// <param name="name">the name of the option to be set</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetOptionPersistent(string name);

        /// <summary>
        /// Set option value persistently. The option value will remain over multiple scenario runs, until lib is reloaded.
        /// </summary>
        /// <param name="name">the name of the option to be set</param>
        /// <param name="value">the value to assign to the option</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetOptionValuePersistent(string name, string value);

        /// <summary>
        /// Get option value
        /// </summary>
        /// <param name="name">the name of the option whose value to fetch</param>
        /// <returns>value of the option</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOptionValue(string name);

        /// <summary>
        /// Get option value
        /// </summary>
        /// <param name="enum_value">(index) value option whose value to fetch (see CommonMini/EnumConfig.hpp::esmini_options::CONFIG_ENUM)</param>
        /// <returns>value of the option</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOptionValueByEnum(uint enum_value);

        /// <summary>
        /// Get option values count. Some options can have multiple values, this function returns the number of values present for the option.
        /// </summary>
        /// <param name="name">the name of the option whose values count to fetch</param>
        /// <returns>values count of the option</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetOptionValuesCount(string name);

        /// <summary>
        /// Get specified entry of option values, useful when option has multiple values.
        /// </summary>
        /// <param name="name">the name of the option whose value to fetch</param>
        /// <param name="index">index of the value to fetch</param>
        /// <returns>value of the option</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOptionValueByIndex(string name, uint index);

        /// <summary>
        /// Get option set status
        /// </summary>
        /// <param name="name">is the name of the option whose value is fetch</param>
        /// <returns>Returns true if the option is set otherwise false</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool SE_GetOptionSet(string name);

        /// <summary>
        /// Set window position and size. Must be called prior to SE_Init.
        /// </summary>
        /// <param name="x">Screen coordinate in pixels for left side of window</param>
        /// <param name="y">Screen coordinate in pixels for top of window</param>
        /// <param name="w">Width in pixels</param>
        /// <param name="h">Height in pixels</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SetWindowPosAndSize(int x, int y, int w, int h);

        /// <summary>
        /// Register a function and optional argument (ref) to be called back from esmini after ParameterDeclarations has been parsed,
        /// but before the scenario is initialized, i.e. before applying the actions in the Init block. One use-case is to
        /// set parameter values for initial entity states, e.g. s value in lane position. So this callback will happen just
        /// after parameters has been parsed, but before they are applied, providing an opportunity to control the initial
        /// states via API.
        /// Registered init callbacks are be cleared between SE_Init calls, i.e. needs to be registered
        /// </summary>
        /// <param name="fnPtr">A pointer to the function to be invoked</param>
        /// <param name="user_data">Optional pointer to a local data object that will be passed as argument in the callback. Set 0/NULL if not needed.</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_RegisterParameterDeclarationCallback(ParameterDeclarationCallback fnPtr, IntPtr user_data);

        /// <summary>
        /// Configure tolerances/resolution for OSI road features
        /// </summary>
        /// <param name="max_longitudinal_distance">Maximum distance between OSI points, even on straight road. Default=50(m)</param>
        /// <param name="max_lateral_deviation">Control resolution w.r.t. curvature default=0.05(m)</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetOSITolerances(double maxLongitudinalDistance, double maxLateralDeviation);

        /// <summary>
        /// Specify OpenSCENARIO parameter distribution file. Call BEFORE SE_Init.
        /// </summary>
        /// <param name="filename">Name, including any path, of the parameter distribution file</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetParameterDistribution(string filename);

        /// <summary>
        /// Reset and disable parameter distribution.
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_ResetParameterDistribution();

        /// <summary>
        /// Get the number of parameter value permutations. Call AFTER SE_Init.
        /// </summary>
        /// <returns>number of permutations</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern uint SE_GetNumberOfPermutations();

        /// <summary>
        /// Select parameter value permutation. Call BEFORE SE_Init, e.g. during or after preceding run.
        /// </summary>
        /// <returns>-1 on error else number of permutations</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SelectPermutation(uint index);

        /// <summary>
        /// Get current parameter permutation index.
        /// </summary>
        /// <returns>-1 on error or no parameter distribution loaded, else permutation index</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetPermutationIndex();

        /// <summary>
        /// Initialize the scenario engine
        /// </summary>
        /// <param name="oscFilename">Path to the OpenSCENARIO file</param>
        /// <param name="disable_ctrls">1=Any controller will be disabled 0=Controllers applied according to OSC file</param>
        /// <param name="use_viewer">Bitmask: 1=viewer on/off, 2=off-screen only, 4=capture-to-file, 8=disable info-text. Ex1: 0=&gt;No viewer, ex2: 1+2=3=&gt;Off-screen</param>
        /// <param name="threads">0=single thread, 1=viewer in a separate thread, parallel to scenario engine</param>
        /// <param name="record">Create recording for later playback 0=no recording 1=recording</param>
        /// <returns>0 if successful, -1 if not \use_viewer bitmask examples: 0: No viewer instantiated. Improved performance, use when viewer not needed. 1: Viewer instantiated with a window on screen (default viewer usage) 3 (1+2): Off-screen rendering only (no window on screen) 7 (1+2+4): Off-screen + save screenshots to file 11 (1+2+8): Off-screen + disable info-text for better remote/virtual desktop support</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_Init(string oscFilename, int disable_ctrls, int use_viewer, int threads, int record);

        /// <summary>
        /// Initialize the scenario engine
        /// </summary>
        /// <param name="oscAsXMLString">OpenSCENARIO XML as string</param>
        /// <param name="disable_ctrls">1=Any controller will be disabled 0=Controllers applied according to OSC file</param>
        /// <param name="use_viewer">Bitmask: 1=viewer on/off, 2=off-screen only, 4=capture-to-file, 8=disable info-text. Ex1: 0=&gt;No viewer, ex2: 1+2=3=&gt;Off-screen</param>
        /// <param name="threads">0=single thread, 1=viewer in a separate thread, parallel to scenario engine</param>
        /// <param name="record">Create recording for later playback 0=no recording 1=recording</param>
        /// <returns>0 if successful, -1 if not \use_viewer bitmask examples: 0: No viewer instantiated. Improved performance, use when viewer not needed. 1: Viewer instantiated with a window on screen (default viewer usage) 3 (1+2): Off-screen rendering only (no window on screen) 7 (1+2+4): Off-screen + save screenshots to file 11 (1+2+8): Off-screen + disable info-text for better remote/virtual desktop support</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_InitWithString(string oscAsXMLString, int disable_ctrls, int use_viewer, int threads, int record);

        /// <summary>
        /// Initialize the scenario engine
        /// </summary>
        /// <param name="argc">Number of arguments</param>
        /// <param name="argv">Arguments</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_InitWithArgs(int argc, string argv);

        /// <summary>
        /// Step the simulation forward with specified timestep
        /// </summary>
        /// <param name="dt">time step in seconds</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_StepDT(double dt);

        /// <summary>
        /// Step the simulation forward. Time step will be elapsed system (world) time since last step. Useful for interactive/realtime use cases.
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_Step();

        /// <summary>
        /// Stop simulation gracefully. Two purposes: 1. Release memory and 2. Prepare for next simulation, e.g. reset object lists.
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_Close();

        /// <summary>
        /// Enable or disable log to stdout/console
        /// Deprecated, use SE_SetOption() / SE_UnsetOption() with "disable_stdout" instead
        /// which also allows for persistant setting
        /// </summary>
        /// <param name="mode">true=enable, false=disable</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_LogToConsole([MarshalAs(UnmanagedType.I1)] bool mode);

        /// <summary>
        /// Enable or disable global collision detection
        /// </summary>
        /// <param name="mode">true=enable, false=disable</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_CollisionDetection([MarshalAs(UnmanagedType.I1)] bool mode);

        /// <summary>
        /// Get simulation time in seconds - double (64 bit) precision
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern double SE_GetSimulationTime();

        /// <summary>
        /// Get simulation time step in seconds
        /// The time step is calculated as difference since last call to same funtion.
        /// Clamped to some reasonable values. First call returns smallest delta (typically 1 ms).
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern double SE_GetSimTimeStep();

        /// <summary>
        /// Is esmini about to quit?
        /// </summary>
        /// <returns>0 if not, 1 if yes, -1 if some error e.g. scenario not loaded</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetQuitFlag();

        /// <summary>
        /// Is esmini paused (via space button)?
        /// </summary>
        /// <returns>0 if not, 1 if yes, -1 if some error e.g. scenario not loaded</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetPauseFlag();

        /// <summary>
        /// Get name of currently referred and loaded OpenDRIVE file
        /// </summary>
        /// <returns>filename as string (const, since it's allocated and handled by esmini)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetODRFilename();

        /// <summary>
        /// Get name of currently referred and loaded SceneGraph file
        /// </summary>
        /// <returns>filename as string (const, since it's allocated and handled by esmini)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetSceneGraphFilename();

        /// <summary>
        /// Get the number of named parameters within the current scenario
        /// </summary>
        /// <returns>number of parameters, -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetNumberOfParameters();

        /// <summary>
        /// Get the name of a named parameter
        /// </summary>
        /// <param name="index">The index of the parameter, range [0:numberOfParameters-1]</param>
        /// <param name="Output">parameter type 1=int, 2=double, 3=string (const char*), 4=bool, see OSCParameterDeclarations/ParameterType</param>
        /// <returns>name if found, else 0</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetParameterName(int index, out int type);

        /// <summary>
        /// Get the number of named variables within the current scenario
        /// </summary>
        /// <returns>number of variables, -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetNumberOfVariables();

        /// <summary>
        /// Get the name of a named variable
        /// </summary>
        /// <param name="index">The index of the variable, range [0:numberOfVariables-1]</param>
        /// <param name="Output">variable type 1=int, 2=double, 3=string (const char*), 4=bool, see OSCParameterDeclarations/ParameterType</param>
        /// <returns>name if found, else 0</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetVariableName(int index, out int type);

        /// <summary>
        /// Get the number of vehicle properties by index
        /// </summary>
        /// <param name="index">The index of the vehicle</param>
        /// <returns>number of parameters if found, -1 indicating some error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetNumberOfProperties(int index);

        /// <summary>
        /// Get the number of vehicle properties by index
        /// </summary>
        /// <param name="index">The index of the vehicle</param>
        /// <param name="propertyIndex">The index of the property</param>
        /// <returns>the name of the property by index if found, else ""</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetObjectPropertyName(int index, int propertyIndex);

        /// <summary>
        /// Get the value of a vehicle property by name
        /// </summary>
        /// <param name="index">The index of the vehicle</param>
        /// <param name="vehiclePropertyName">the vehicle property name</param>
        /// <returns>the value of a vehicle property by name if found, else ""</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetObjectPropertyValue(int index, string objectPropertyName);

        /// <summary>
        /// Set value of named parameter
        /// </summary>
        /// <param name="parameter">Struct object including name of parameter and pointer to value, see SE_Parameter declaration</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetParameter(SE_Parameter parameter);

        /// <summary>
        /// Get value of parameter. name field is already filled in by caller, value will be filled in.
        /// </summary>
        /// <param name="parameter">Pointer to parameter struct object, see SE_Parameter declaration.</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetParameter(ref SE_Parameter parameter_ref);

        /// <summary>
        /// Get typed value of named parameter
        /// @parameterName Name of the parameter
        /// </summary>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetParameterInt(string parameterName, out int value);

        /// <summary>
        /// Get typed value of named parameter
        /// @parameterName Name of the parameter
        /// </summary>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetParameterDouble(string parameterName, out double value);

        /// <summary>
        /// Get typed value of named parameter
        /// @parameterName Name of the parameter
        /// </summary>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetParameterString(string parameterName, [In] string[] value);

        /// <summary>
        /// Get typed value of named parameter
        /// @parameterName Name of the parameter
        /// </summary>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetParameterBool(string parameterName, out bool value);

        /// <summary>
        /// Set typed value of named parameter
        /// @parameterName Name of the parameter
        /// @value Value
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetParameterInt(string parameterName, int value);

        /// <summary>
        /// Set typed value of named parameter
        /// @parameterName Name of the parameter
        /// @value Value
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetParameterDouble(string parameterName, double value);

        /// <summary>
        /// Set typed value of named parameter
        /// @parameterName Name of the parameter
        /// @value Value
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetParameterString(string parameterName, string value);

        /// <summary>
        /// Set typed value of named parameter
        /// @parameterName Name of the parameter
        /// @value Value
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetParameterBool(string parameterName, [MarshalAs(UnmanagedType.I1)] bool value);

        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetVariable(SE_Variable variable);

        /// <summary>
        /// Get value of named parameter. The value within the parameter struct will be filled in.
        /// </summary>
        /// <param name="parameter">Pointer to parameter struct object, see SE_Parameter declaration.</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetVariable(out SE_Variable variable);

        /// <summary>
        /// Get typed value of named variable
        /// @variableName Name of the variable
        /// </summary>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetVariableInt(string variableName, out int value);

        /// <summary>
        /// Get typed value of named variable
        /// @variableName Name of the variable
        /// </summary>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetVariableDouble(string variableName, out double value);

        /// <summary>
        /// Get typed value of named variable
        /// @variableName Name of the variable
        /// </summary>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetVariableString(string variableName, [In] string[] value);

        /// <summary>
        /// Get typed value of named variable
        /// @variableName Name of the variable
        /// </summary>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetVariableBool(string variableName, out bool value);

        /// <summary>
        /// Set typed value of named variable
        /// @variableName Name of the variable
        /// @value Value
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetVariableInt(string variableName, int value);

        /// <summary>
        /// Set typed value of named variable
        /// @variableName Name of the variable
        /// @value Value
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetVariableDouble(string variableName, double value);

        /// <summary>
        /// Set typed value of named variable
        /// @variableName Name of the parameter
        /// @value Value
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetVariableString(string variableName, string value);

        /// <summary>
        /// Set typed value of named variable
        /// @variableName Name of the variable
        /// @value Value
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetVariableBool(string variableName, [MarshalAs(UnmanagedType.I1)] bool value);

        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetODRManager();

        /// <summary>
        /// Specify if and how position object will align to the road. The setting is done for individual components:
        /// Z (elevation), Heading, Pitch, Roll and separately for set- and update operation. Set operations represents
        /// when position is affected by API calls, e.g. updateObjectWorldPos(). Update operations represents when the
        /// position is updated implicitly by the scenarioengine, e.g. default controller moving a vehicle along the lane.
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="type">Type of operations the setting applies to, according to SE_PositionModeType enum</param>
        /// <param name="mode">Bitmask combining values from SE_PositionMode enum example: To set relative z and absolute roll: (SE_Z_REL | SE_R_ABS) or (7 | 12288) = (7 + 12288) = 12295 according to roadmanager::PosModeType</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SetObjectPositionMode(int object_id, SE_PositionModeType type, int mode);

        /// <summary>
        /// Set default alignment mode for SET or UPDATE operations. See roadmanager::Position::GetModeDefault() to find out
        /// what are the default modes.
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="type">Type of operations the setting applies to, according to SE_PositionModeType enum according to roadmanager::PosModeType</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SetObjectPositionModeDefault(int object_id, SE_PositionModeType type);

        /// <summary>
        /// Add object with bounding box automatically adapted to 3D model (scale mode BB_TO_MODEL)
        /// Should be followed by one of the SE_Report functions to establish initial state.
        /// </summary>
        /// <param name="object_name">Name of the object, preferably be unique</param>
        /// <param name="object_type">Type of the object. See Entities.hpp::Object::Type. Default=1 (VEHICLE).</param>
        /// <param name="object_category">Category of the object. Depends on type, see descendants of Entities.hpp::Object. Set to 0 if not known.</param>
        /// <param name="object_role">role of the object. Depends on type, See Entities.hpp::Object::Role. Set to 0 if not known.</param>
        /// <param name="model_id">Id of the 3D model to represent the object. See resources/model_ids.txt. Set -1 to skip.</param>
        /// <param name="model_3d">Filename of 3D model to represent the object. Overrides model_id. Set NULL to skip.</param>
        /// <returns>Id [0..inf] of the added object successful, -1 on failure</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddObject(string object_name, int object_type, int object_category, int object_role, int model_id, string model_3d);

        /// <summary>
        /// Add object with specified bounding box.
        /// Should be followed by one of the SE_Report functions to establish initial state.
        /// For scale_mode BB_TO_MODEL, set bounding_box to whatever, e.g. {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, or use SE_AddObject()
        /// </summary>
        /// <param name="object_name">Name of the object, preferably be unique</param>
        /// <param name="object_type">Type of the object. See Entities.hpp::Object::Type. Default=1 (VEHICLE).</param>
        /// <param name="object_category">Category of the object. Depends on type, see descendants of Entities.hpp::Object. Set to 0 if not known.</param>
        /// <param name="object_role">role of the object. Depends on type, See Entities.hpp::Object::Role. Set to 0 if not known.</param>
        /// <param name="model_id">Id of the 3D model to represent the object. See resources/model_ids.txt. Set -1 to skip.</param>
        /// <param name="model_3d">Filename of 3D model to represent the object. Overrides model_id. Set NULL to skip.</param>
        /// <param name="bounding_box">sets the internal bounding box of the model and will also be used to scale 3D model accordingly.</param>
        /// <param name="scale_mode">0=NONE, 1=BB_TO_MODEL, 2=MODEL_TO_BB (recommended). See CommonMini::EntityScaleMode enum for details.</param>
        /// <returns>Id [0..inf] of the added object successful, -1 on failure</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddObjectWithBoundingBox(string object_name, int object_type, int object_category, int object_role, int model_id, string model_3d, SE_OSCBoundingBox bounding_box, int scale_mode);

        /// <summary>
        /// Delete object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_DeleteObject(int object_id);

        /// <summary>
        /// Report object position in cartesian coordinates
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="z">Z coordinate, set std::nanf("") to ignore, i.e. re-use current value (#include &lt;cmath&gt;)</param>
        /// <param name="h">Heading / yaw</param>
        /// <param name="p">Pitch, set std::nanf("") to ignore, i.e. re-use current value (#include &lt;cmath&gt;)</param>
        /// <param name="r">Roll, set std::nanf("") to ignore, i.e. re-use current value (#include &lt;cmath&gt;)</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectPos(int object_id, double x, double y, double z, double h, double p, double r);

        /// <summary>
        /// Report object position in cartesian coordinates, with detailed control of absolute or relative coordinates
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="z">Z coordinate, set std::nanf("") to ignore, i.e. re-use current value (#include &lt;cmath&gt;)</param>
        /// <param name="h">Heading / yaw</param>
        /// <param name="p">Pitch, set std::nanf("") to ignore, i.e. re-use current value (#include &lt;cmath&gt;)</param>
        /// <param name="r">Roll, set std::nanf("") to ignore, i.e. re-use current value (#include &lt;cmath&gt;)</param>
        /// <param name="mode">Explicit mode, override current setting. E.g. POS_REL_Z | POS_ABS_H, see SE_PositionMode enum</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectPosMode(int object_id, double x, double y, double z, double h, double p, double r, int mode);

        /// <summary>
        /// Report object position in limited set of cartesian coordinates x, y and heading,
        /// the remaining z, pitch and roll will be aligned to the road surface
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="h">Heading / yaw</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectPosXYH(int object_id, double x, double y, double h);

        /// <summary>
        /// Report object position in road coordinates
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="roadId">Id of the road</param>
        /// <param name="laneId">Id of the lane</param>
        /// <param name="laneOffset">Lateral offset from center of specified lane</param>
        /// <param name="s">Longitudinal distance of the position along the specified road</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectRoadPos(int object_id, uint roadId, int laneId, double laneOffset, double s);

        /// <summary>
        /// Report object longitudinal speed. Useful for an external longitudinal controller.
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="speed">Speed in forward direction of the enitity</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectSpeed(int object_id, double speed);

        /// <summary>
        /// Report object lateral position relative road centerline. Useful for an external lateral controller.
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="t">Lateral position</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectLateralPosition(int object_id, double t);

        /// <summary>
        /// Report object lateral position by lane id and lane offset. Useful for an external lateral controller.
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="laneId">Id of the lane</param>
        /// <param name="laneOffset">Lateral offset from center of specified lane</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectLateralLanePosition(int object_id, int laneId, double laneOffset);

        /// <summary>
        /// Report object position in cartesian coordinates
        /// </summary>
        /// <param name="iobject_idd">Id of the object</param>
        /// <param name="x_vel">X component of linear velocity</param>
        /// <param name="y_vel">Y component of linear velocity</param>
        /// <param name="z_vel">Z component of linear velocity</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectVel(int object_id, double x_vel, double y_vel, double z_vel);

        /// <summary>
        /// Report object position in cartesian coordinates
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="h_vel">Heading component of angular velocity</param>
        /// <param name="p_vel">Pitch component of angular velocity</param>
        /// <param name="r_vel">Roll component of angular velocity</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectAngularVel(int object_id, double h_rate, double p_rate, double r_rate);

        /// <summary>
        /// Report object position in cartesian coordinates
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="x_acc">X component of linear acceleration</param>
        /// <param name="y_acc">Y component of linear acceleration</param>
        /// <param name="z_acc">Z component of linear acceleration</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectAcc(int object_id, double x_acc, double y_acc, double z_acc);

        /// <summary>
        /// Report object position in cartesian coordinates
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="h_acc">Heading component of angular acceleration</param>
        /// <param name="p_acc">Pitch component of angular acceleration</param>
        /// <param name="r_acc">Roll component of angular acceleration</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectAngularAcc(int object_id, double h_acc, double p_acc, double r_acc);

        /// <summary>
        /// Report object wheel status
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="rotation">Wheel rotation</param>
        /// <param name="angle">Wheel steering angle</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ReportObjectWheelStatus(int object_id, double rotation, double angle);

        /// <summary>
        /// Specify which lane types the position object snaps to (is aware of)
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="laneTypes">A combination (bitmask) of lane types according to roadmanager::Lane::LaneType examples: ANY_DRIVING = 6160898, ANY_ROAD = 6161294, ANY = -1</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetSnapLaneTypes(int object_id, int laneTypes);

        /// <summary>
        /// Controls whether to keep lane ID regardless of lateral position or snap to closest lane (default)
        /// </summary>
        /// <param name="object_id">Id of the object @parameter mode True=keep lane False=Snap to closest (default)</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetLockOnLane(int object_id, [MarshalAs(UnmanagedType.I1)] bool mode);

        /// <summary>
        /// Get the number of entities in the current scenario
        /// </summary>
        /// <returns>Number of entities, -1 on error e.g. scenario not initialized</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetNumberOfObjects();

        /// <summary>
        /// Get the Id of an entity present in the current scenario
        /// </summary>
        /// <param name="index">Index of the object. Note: not ID</param>
        /// <returns>Id of the object, -1 on error e.g. scenario not initialized</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetId(int index);

        /// <summary>
        /// Get the Id of an entity present in the current scenario
        /// </summary>
        /// <param name="name">Name of the object.</param>
        /// <returns>Id of the object, -1 on error e.g. scenario not initialized</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetIdByName(string name);

        /// <summary>
        /// Get the state of specified object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="state">Pointer/reference to a SE_ScenarioObjectState struct to be filled in</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectState(int object_id, out SE_ScenarioObjectState state);

        /// <summary>
        /// Get the object route status
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>0 if route not assigned, 1 if outside assigned route, 2 if on assigned route, -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectRouteStatus(int object_id);

        /// <summary>
        /// Find out what lane type object is currently in, reference point projected on road
        /// Can be used for checking exact lane type or combinations by bitmask.
        /// Example 1: Check if on border lane: SE_GetObjectLaneType(id) == (1 &lt;&lt; 6)
        /// Example 2: Check if on any drivable lane: SE_GetObjectLaneType(id) &amp; 1966594
        /// Example 3: Check if on any road lane: SE_GetObjectLaneType(id) &amp; 1966990
        /// Example 4: Check for no lane (outside defined lanes): SE_GetObjectLaneType(id) == 1
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>lane type according to enum roadmanager::Lane::LaneType</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectInLaneType(int object_id);

        /// <summary>
        /// Get the overrideActionStatus of specified object
        /// </summary>
        /// <param name="objectId">Id of the object</param>
        /// <param name="list">Pointer/reference to a SE_OverrideActionList struct to be filled in</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetOverrideActionStatus(int objectId, out SE_OverrideActionList list);

        /// <summary>
        /// Get the type name of the specifed vehicle-, pedestrian- or misc object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>Name</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetObjectTypeName(int object_id);

        /// <summary>
        /// Get the name of specified object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>Name</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetObjectName(int object_id);

        /// <summary>
        /// Get the 3D model filename of the specifed object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>Name</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetObjectModelFileName(int object_id);

        /// <summary>
        /// Check whether an object has a ghost (special purpose lead vehicle)
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>1 if ghost, 0 if not, -1 indicates error e.g. scenario not loaded</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ObjectHasGhost(int object_id);

        /// <summary>
        /// Get ID of the ghost associated with given object
        /// </summary>
        /// <param name="object_id">Id of the ghost object</param>
        /// <returns>ghost object ID, -1 if ghost does not exist for given object</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectGhostId(int object_id);

        /// <summary>
        /// Get the state of specified object's ghost (special purpose lead vehicle)
        /// </summary>
        /// <param name="object_id">Id of the object to which the ghost is attached</param>
        /// <param name="state">Pointer/reference to a SE_ScenarioObjectState struct to be filled in</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectGhostState(int object_id, out SE_ScenarioObjectState state);

        /// <summary>
        /// Get the number of collisions the specified object currently is involved in
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>Number of objects that specified object currently is overlapping/colliding with. -1 on error.</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectNumberOfCollisions(int object_id);

        /// <summary>
        /// Get the object involved in specified collision by object id and collision index
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="index">Index of collision (one object can be involvoed in multiple simultaneous collisions)</param>
        /// <returns>object_id of colliding object. -1 if unsuccessful.</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectCollision(int object_id, int index);

        /// <summary>
        /// Get the traveled distance of an object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>traveled distance if successful, std::nanf if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern double SE_GetObjectOdometer(int object_id);

        /// <summary>
        /// Get the angular velocity of the specified object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="x">reference to a variable returning the velocity along global x-axis</param>
        /// <param name="y">reference to a variable returning the velocity along global y-axis</param>
        /// <param name="z">reference to a variable returning the velocity along global z-axis</param>
        /// <returns>0 if successful.</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectVelocityGlobalXYZ(int object_id, out double vel_x, out double vel_y, out double vel_z);

        /// <summary>
        /// Get the angular velocity of the specified object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="h_rate">The rate of the heading.</param>
        /// <param name="p_rate">The rate of the pitch.</param>
        /// <param name="r_rate">The rate of the roll.</param>
        /// <returns>0 if successful.</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectAngularVelocity(int object_id, out double h_rate, out double p_rate, out double r_rate);

        /// <summary>
        /// Get the angular velocity of the specified object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="h_acc">The rate of the heading.</param>
        /// <param name="p_acc">The rate of the pitch.</param>
        /// <param name="r_acc">The rate of the roll.</param>
        /// <returns>0 if successful.</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectAngularAcceleration(int object_id, out double h_acc, out double p_acc, out double r_acc);

        /// <summary>
        /// Get the acceleration magnitude of specified object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>the acceleration if successful, std::nanf if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern double SE_GetObjectAcceleration(int object_id);

        /// <summary>
        /// Get the acceleration components of specified object in global x, y, z coordinates
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="x">reference to a variable returning the acceleration along global x-axis</param>
        /// <param name="y">reference to a variable returning the acceleration along global y-axis</param>
        /// <param name="z">reference to a variable returning the acceleration along global z-axis</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectAccelerationGlobalXYZ(int object_id, out double acc_x, out double acc_y, out double acc_z);

        /// <summary>
        /// Get the acceleration components of specified object in local x,y coordinates
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="lat">reference to a variable returning the acceleration along local y-axis</param>
        /// <param name="long">reference to a variable returning the acceleration along local x-axis</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectAccelerationLocalLatLong(int object_id, out double acc_lat, out double acc_long);

        /// <summary>
        /// Get the number of wheels of an object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>number of wheels on object if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectNumberOfWheels(int object_id);

        /// <summary>
        /// Get wheel information of specified object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <param name="wheeldata">reference to a struct in which to return the wheeldata</param>
        /// <param name="wheel_index">index of wheeldata to return</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectWheelData(int object_id, int wheel_index, out SE_WheelData wheeldata);

        /// <summary>
        /// Get the unit of specified speed (in OpenDRIVE road type element).
        /// All roads will be looped in search for such an element. First found will be used.
        /// If speed is specified withouth the optional unit, SI unit m/s is assumed.
        /// If no speed entries is found, undefined will be returned.
        /// </summary>
        /// <returns>-1=Error, 0=Undefined, 1=km/h 2=m/s, 3=mph</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetSpeedUnit();

        /// <summary>
        /// Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
        /// </summary>
        /// <param name="object_id">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="lookAheadMode">Measurement strategy: Along 0=lane center, 1=road center (ref line) or 2=current lane offset. See roadmanager::Position::LookAheadMode enum</param>
        /// <param name="inRoadDrivingDirection">If true look along lane driving direction. If false, look in closest direction according to object heading.</param>
        /// <returns>0 = OK, ERROR_OFF_ROAD = -4, ERROR_END_OF_ROUTE = -3, ERROR_END_OF_ROAD = -2, ERROR_GENERIC = -1, ENTERED_NEW_ROAD = 1, MADE_JUNCTION_CHOICE = 2 (see roadmanager.hpp -&gt; Position::ReturnCode)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetRoadInfoAtDistance(int object_id, double lookahead_distance, out SE_RoadInfo data, int lookAheadMode, [MarshalAs(UnmanagedType.I1)] bool inRoadDrivingDirection);

        /// <summary>
        /// Get information suitable for driver modeling of a point at a specified distance from object along the route ahead
        /// </summary>
        /// <param name="object_id">Handle to the position object from which to measure</param>
        /// <param name="lookahead_distance">The distance, along the road, to the point</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="lookAheadMode">Measurement strategy: Along 0=lane center, 1=road center (ref line) or 2=current lane offset. See roadmanager::Position::LookAheadMode enum</param>
        /// <param name="inRoadDrivingDirection">If true look along lane driving direction. If false, look in closest direction according to object heading.</param>
        /// <returns>0 = OK, ERROR_OFF_ROAD = -4, ERROR_END_OF_ROUTE = -3, ERROR_END_OF_ROAD = -2, ERROR_GENERIC = -1, ENTERED_NEW_ROAD = 1, MADE_JUNCTION_CHOICE = 2 (see roadmanager.hpp -&gt; Position::ReturnCode)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetRoadInfoAlongRoute(int object_id, double lookahead_distance, out SE_RoadInfo data, int lookAheadMode, [MarshalAs(UnmanagedType.I1)] bool inRoadDrivingDirection);

        /// <summary>
        /// Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle
        /// </summary>
        /// <param name="object_id">Id of the object from which to measure (the actual externally controlled Ego vehicle, not ghost)</param>
        /// <param name="lookahead_distance">The distance, along the ghost trail, to the point from the current Ego vehicle location</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="speed_ghost">reference to a variable returning the speed that the ghost had at this point along trail</param>
        /// <returns>0 if successful, &lt; 0 see SE_GhostTrailReturnCode enum for error/information codes</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetRoadInfoAlongGhostTrail(int object_id, double lookahead_distance, out SE_RoadInfo data, out double speed_ghost, out double timestamp);

        /// <summary>
        /// Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle
        /// </summary>
        /// <param name="object_id">Id of the object from which to measure (the actual externally controlled Ego vehicle, not ghost)</param>
        /// <param name="time">Simulation time (subtracting headstart time, i.e. time=0 gives the initial state)</param>
        /// <param name="data">Struct including all result values, see typedef for details</param>
        /// <param name="speed_ghost">reference to a variable returning the speed that the ghost had at this point along trail</param>
        /// <returns>0 if successful, &lt; 0 see SE_GhostTrailReturnCode enum for error/information codes</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetRoadInfoGhostTrailTime(int object_id, double time, out SE_RoadInfo data, out double speed_ghost);

        /// <summary>
        /// Find out the delta between two objects, e.g. distance (long and lat) and delta laneId
        /// search range is 1000 meters
        /// </summary>
        /// <param name="object_a_id">Id of the object from which to measure</param>
        /// <param name="object_b_id">Id of the object to which the distance is measured</param>
        /// <param name="free_space">Measure distance between bounding boxes (true) or between ref points (false)</param>
        /// <param name="pos_diff">Struct including all result values, see PositionDiff typedef</param>
        /// <returns>0 if successful, -2 if route between positions can't be found, -1 if some other error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetDistanceToObject(int object_a_id, int object_b_id, [MarshalAs(UnmanagedType.I1)] bool free_space, out SE_PositionDiff pos_diff);

        /// <summary>
        /// Optimized method to find the relative distance between two objects in the entities local coordinate system.
        /// The method discards any object &gt;500m away, will have a reduced tracking frequency (3s) for objects &gt;tracking limit and avoids redundant
        /// calculations.
        /// </summary>
        /// <param name="object_a_id">Id of the object from which to measure</param>
        /// <param name="object_b_id">Id of the object to which the distance is measured</param>
        /// <param name="dist_type">Enum specifying what distance to measure</param>
        /// <param name="distance">reference to a variable returning the distance</param>
        /// <param name="timestamp">reference to a variable returning the timestamp of the distance sample</param>
        /// <returns>0 if successful, -1 if the distance measurement failed and -2 if the objects are out of bounds (&gt;500m) or didn't update the current sample.</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SimpleGetDistanceToObject(int object_a_id, int object_b_id, SE_RelativeDistanceType dist_type, double tracking_limit, out double distance, out double timestamp);

        /// <summary>
        /// Create an ideal object sensor and attach to specified vehicle
        /// </summary>
        /// <param name="object_id">Handle to the object to which the sensor should be attached</param>
        /// <param name="x">Position x coordinate of the sensor in vehicle local coordinates</param>
        /// <param name="y">Position y coordinate of the sensor in vehicle local coordinates</param>
        /// <param name="z">Position z coordinate of the sensor in vehicle local coordinates</param>
        /// <param name="h">heading of the sensor in vehicle local coordinates</param>
        /// <param name="fovH">Horizontal field of view, in degrees</param>
        /// <param name="rangeNear">Near value of the sensor depth range</param>
        /// <param name="rangeFar">Far value of the sensor depth range</param>
        /// <param name="maxObj">Maximum number of objects theat the sensor can track</param>
        /// <returns>Sensor ID (Global index of sensor), -1 if unsucessful</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddObjectSensor(int object_id, double x, double y, double z, double h, double rangeNear, double rangeFar, double fovH, int maxObj);

        /// <summary>
        /// Retrieve the total number of sensors attached to any objects
        /// </summary>
        /// <returns>-1 on failure, else the number of sensors</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetNumberOfObjectSensors();

        /// <summary>
        /// Allow to view detected sensor data.
        /// </summary>
        /// <param name="object_id">Handle to the object to which the sensor should be attached</param>
        /// <returns>Sensor ID (Global index of sensor), -1 if unsucessful</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_ViewSensorData(int object_id);

        /// <summary>
        /// Fetch list of identified objects from a sensor
        /// </summary>
        /// <param name="sensor_id">Handle (index) to the sensor</param>
        /// <param name="list">Array of object indices</param>
        /// <returns>Number of identified objects, i.e. length of list. -1 if unsuccesful.</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_FetchSensorObjectList(int sensor_id, out int list);

        /// <summary>
        /// Register a function and optional parameter (ref) to be called back from esmini after each frame (update of scenario)
        /// The current state of specified entity will be returned.
        /// Complete or part of the state can then be overridden by calling the SE_ReportObjectPos/SE_ReportObjectRoadPos functions.
        /// Registered callbacks will be cleared between SE_Init calls.
        /// </summary>
        /// <param name="object_id">Handle to the position object (entity)</param>
        /// <param name="fnPtr">A pointer to the function to be invoked</param>
        /// <param name="user_data">Optional pointer to a local data object that will be passed as argument in the callback. Set 0/NULL if not needed.</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_RegisterObjectCallback(int object_id, ObjectCallback fnPtr, IntPtr user_data);

        /// <summary>
        /// Registers a function to be called back from esmini every time a condition is triggered.
        /// The name of the respective condition and the current timestamp will be returned.
        /// Registered callbacks will be cleared between SE_Init calls.
        /// </summary>
        /// <param name="fnPtr">A pointer to the function to be invoked</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_RegisterConditionCallback(ConditionCallback fnPtr);

        /// <summary>
        /// Registers a function to be called back from esmini every time a StoryBoardElement changes its state.
        /// The name of the respective StoryBoardElement, the type, state, and full path (parent names delimited by /) will be returned.
        /// See StoryBoardElement.hpp -&gt; StoryBoardElement class ElementType and State enums for type and state values.
        /// Registered callbacks will be cleared between SE_Init calls.
        /// </summary>
        /// <param name="fnPtr">A pointer to the function to be invoked</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_RegisterStoryBoardElementStateChangeCallback(StoryBoardElementStateChangeCallback fnPtr);

        /// <summary>
        /// Get the number of road signs along specified road
        /// </summary>
        /// <param name="road_id">The road along which to look for signs</param>
        /// <returns>Number of road signs</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern uint SE_GetNumberOfRoadSigns(uint road_id);

        /// <summary>
        /// Get information on specifed road sign
        /// </summary>
        /// <param name="road_id">The road of which to look for the signs</param>
        /// <param name="index">Index of the sign. Note: not ID</param>
        /// <param name="road_sign">Pointer/reference to a SE_RoadSign struct to be filled in</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetRoadSign(uint road_id, uint index, out SE_RoadSign road_sign);

        /// <summary>
        /// Get the number of lane validity records of specified road object/sign
        /// </summary>
        /// <param name="road_id">The road of which to look for the sign</param>
        /// <param name="index">Index of the sign. Note: not ID</param>
        /// <returns>Number of validity records of specified road sign</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern uint SE_GetNumberOfRoadSignValidityRecords(uint road_id, uint index);

        /// <summary>
        /// Get specified validity record of specifed road sign
        /// </summary>
        /// <param name="road_id">The road of which to look for the sign</param>
        /// <param name="signIndex">Index of the sign. Note: not ID</param>
        /// <param name="validityIndex">Index of the validity record</param>
        /// <param name="road_sign">Pointer/reference to a SE_RoadObjValidity struct to be filled in</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetRoadSignValidityRecord(uint road_id, uint signIndex, uint validityIndex, out SE_RoadObjValidity validity);

        /// <summary>
        /// Get original string ID asoociated with specified road
        /// </summary>
        /// <param name="road_id">The integer ID road</param>
        /// <returns>string ID, empty string if not found</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetRoadIdString(uint road_id);

        /// <summary>
        /// Get integer road ID associated with specified road string ID
        /// </summary>
        /// <param name="road_id_str">The road string ID</param>
        /// <returns>road ID, -1 if not found</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern uint SE_GetRoadIdFromString(string road_id_str);

        /// <summary>
        /// Get original string ID asoociated with specified junction
        /// </summary>
        /// <param name="road_id">The integer ID junction</param>
        /// <returns>string ID, empty string if not found</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetJunctionIdString(uint junction_id);

        /// <summary>
        /// Get integer junction ID associated with specified junction string ID
        /// </summary>
        /// <param name="road_id_str">The junction string ID</param>
        /// <returns>junction ID, -1 if not found</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern uint SE_GetJunctionIdFromString(string junction_id_str);

        /// <summary>
        /// OSI interface
        /// Send OSI packages over UDP to specified IP address
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_OpenOSISocket(string ipaddr);

        /// <summary>
        /// Switch off logging to OSI file(s)
        /// </summary>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_DisableOSIFile();

        /// <summary>
        /// Switch on logging to OSI file(s)
        /// </summary>
        /// <param name="filename">Optional filename, including path. Set to 0 or "" to use default.</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_EnableOSIFile(string filename);

        /// <summary>
        /// Enforce flushing OSI file (save all buffered data to file)
        /// Normally not necessary, since data is flushed automatically at file closure
        /// </summary>
        /// <returns>0</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_FlushOSIFile();

        /// <summary>
        /// The SE_CropGroundTruth will limit the area of the dynamic groundtruth data to a circle with the specified radius around the given object
        /// id Using the method repeatedly with different object ids will crop the groundtruth data around all objects specified Setting the radius to 0
        /// will remove the cropping
        /// </summary>
        /// <returns>0</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_CropOSIDynamicGroundTruth(int id, double radius);

        /// <summary>
        /// Setting the OSI report mode of the static ground truth data. Default is applied if function not used.
        /// </summary>
        /// <param name="mode">DEFAULT=Static data in API and log first frame only, API=Static data always in API but only logged first frame and API_AND_LOG=Static data always in API and logged.</param>
        /// <returns>0</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SetOSIStaticReportMode(SE_OSIStaticReportMode mode);

        /// <summary>
        /// Excluding ghost vehicle from dynamic ground truth (default is to include)
        /// </summary>
        /// <returns>0</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_ExcludeGhostFromGroundTruth();

        /// <summary>
        /// The SE_SetOSIFrequency function sets the frequency of OSI data updates
        /// </summary>
        /// <param name="frequency">Frequency of OSI data updates</param>
        /// <returns>0</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetOSIFrequency(int frequency);

        /// <summary>
        /// The SE_GetOSIGroundTruth function updates the OSI ground truth and returns a char array containing the osi GroundTruth serialized to a
        /// string
        /// </summary>
        /// <returns>osi3::GroundTruth*</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOSIGroundTruth(out int size);

        /// <summary>
        /// The SE_GetOSIGroundTruthRaw function updates the OSI ground truth and returns a pointer to the internal OSI data structure,
        /// useful for direct access to OSI data in a C/C++ environment
        /// </summary>
        /// <returns>osi3::GroundTruth*</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOSIGroundTruthRaw();

        /// <summary>
        /// Get a pointer to the internal OSI data structure, useful for direct access to OSI data in a C/C++ environment
        /// </summary>
        /// <returns>osi3::TrafficCommand*</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOSITrafficCommandRaw();

        /// <summary>
        /// Populate OSI SensorView from provided pointer to OSI SensorData
        /// </summary>
        /// <returns>0</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetOSISensorDataRaw(string sensordata);

        /// <summary>
        /// Return a pointer to OSI SensorData information
        /// </summary>
        /// <returns>osi3::SensorData*</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOSISensorDataRaw();

        /// <summary>
        /// The SE_GetOSIRoadLane function returns a char array containing the osi Lane information/message of the lane where the object with
        /// object_id is, serialized to a string
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOSIRoadLane(out int size, int object_id);

        /// <summary>
        /// The SE_GetOSIRoadLane function returns a char array containing the osi Lane Boundary information/message with the specified GLOBAL id
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_GetOSILaneBoundary(out int size, int g_id);

        /// <summary>
        /// The SE_GetOSILaneBoundaryIds function the global ids for left, far left, right and far right lane boundaries
        /// </summary>
        /// <param name="object_id">Handle to the object to which the sensor should be attached</param>
        /// <param name="ids">Reference to a struct which will be filled with the Ids</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_GetOSILaneBoundaryIds(int object_id, out SE_LaneBoundaryId ids);

        /// <summary>
        /// Set explicit OSI timestap
        /// Note that this timestamp does NOT affect esmini simulation time
        /// Also note that setting timestamp with this function will move into explicit time mode
        /// and from that point OSI timestamp is exclusively controlled by this function.
        /// </summary>
        /// <param name="nanoseconds">Nano seconds (1e-9 s)</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_OSISetTimeStamp(ulong nanoseconds);

        /// <summary>
        /// End of OSI interface
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_LogMessage(string message);

        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_CloseLogFile();

        /// <summary>
        /// Viewer settings
        /// Switch on/off visualization of specified features
        /// </summary>
        /// <param name="featureType">Type of the features, see roadgeom::NodeMask typedef</param>
        /// <param name="enable">Set true to show features, false to hide</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_ViewerShowFeature(int featureType, [MarshalAs(UnmanagedType.I1)] bool enable);

        /// <summary>
        /// Simple vehicle
        /// Create an instance of a simplistic vehicle based on a 2D bicycle kincematic model
        /// </summary>
        /// <param name="x">Initial position X world coordinate</param>
        /// <param name="y">Initial position Y world coordinate</param>
        /// <param name="h">Initial heading</param>
        /// <param name="length">Length of the vehicle</param>
        /// <param name="speed">Initial speed</param>
        /// <returns>Handle to the created object</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr SE_SimpleVehicleCreate(double x, double y, double h, double length, double speed);

        /// <summary>
        /// Delete an instance of the simplistic vehicle model
        /// </summary>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleDelete(IntPtr handleSimpleVehicle);

        /// <summary>
        /// Update vehicle state in terms of explicit acceleration and steering angle
        /// </summary>
        /// <param name="dt">timesStep (s)</param>
        /// <param name="acceleration">Longitudinal acceleration</param>
        /// <param name="steering_angle">Lateral steering angle</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleControlAccAndSteer(IntPtr handleSimpleVehicle, double dt, double acceleration, double steering_angle);

        /// <summary>
        /// Deactivate or re-activate throttle/brake
        /// </summary>
        /// <param name="disabled">True: throttle disable, False: throttle enable</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSetThrottleDisabled(IntPtr handleSimpleVehicle, [MarshalAs(UnmanagedType.I1)] bool disabled);

        /// <summary>
        /// Deactivate or re-activate steering
        /// </summary>
        /// <param name="disabled">True: steering disable, False: steering enable</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSetSteeringDisabled(IntPtr handleSimpleVehicle, [MarshalAs(UnmanagedType.I1)] bool disabled);

        /// <summary>
        /// Set speed, use together with control binary/analog with throttle set to zero
        /// </summary>
        /// <param name="speed">Speed (m/s)</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSetSpeed(IntPtr handleSimpleVehicle, double speed);

        /// <summary>
        /// Control the speed and steering by providing steering and speed targets
        /// The function also steps the vehicle model, updating its position according to motion state and timestep.
        /// </summary>
        /// <param name="dt">timesStep (s)</param>
        /// <param name="target_speed">Requested speed</param>
        /// <param name="heading_to_target">Heading angle to a target position</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleControlTarget(IntPtr handleSimpleVehicle, double dt, double target_speed, double heading_to_target);

        /// <summary>
        /// Set maximum vehicle speed.
        /// </summary>
        /// <param name="speed">Maximum speed (km/h)</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSetMaxSpeed(IntPtr handleSimpleVehicle, double speed);

        /// <summary>
        /// Set maximum vehicle acceleration.
        /// </summary>
        /// <param name="speed">Maximum acceleration (m/s^2)</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSetMaxAcceleration(IntPtr handleSimpleVehicle, double maxAcceleration);

        /// <summary>
        /// Set maximum vehicle deceleration.
        /// </summary>
        /// <param name="speed">Maximum deceleration (m/s^2)</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSetMaxDeceleration(IntPtr handleSimpleVehicle, double maxDeceleration);

        /// <summary>
        /// Set engine brake factor, applied when no throttle is applied
        /// </summary>
        /// <param name="engineBrakeFactor">recommended range = [0.0, 0.01], default = 0.001</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSetEngineBrakeFactor(IntPtr handleSimpleVehicle, double engineBrakeFactor);

        /// <summary>
        /// Set steering scale factor, which will limit the steering range as speed increases
        /// </summary>
        /// <param name="steeringScale">recommended range = [0.0, 0.1], default = 0.018</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSteeringScale(IntPtr handleSimpleVehicle, double steeringScale);

        /// <summary>
        /// Set steering return factor, which will make the steering wheel strive to neutral position (0 angle)
        /// </summary>
        /// <param name="steeringScale">recommended range = [0.0, 10], default = 4.0</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSteeringReturnFactor(IntPtr handleSimpleVehicle, double steeringReturnFactor);

        /// <summary>
        /// Set steering rate, which will affect the angular speed of which the steering wheel will turn
        /// </summary>
        /// <param name="steeringRate">recommended range = [0.0, 50.0], default = 8.0</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleSteeringRate(IntPtr handleSimpleVehicle, double steeringRate);

        /// <summary>
        /// Get current state of the vehicle. Typically called after Control has been applied.
        /// </summary>
        /// <param name="state">Pointer/reference to a SE_SimpleVehicleState struct to be filled in</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_SimpleVehicleGetState(IntPtr handleSimpleVehicle, out SE_SimpleVehicleState state);

        /// <summary>
        /// Capture rendered image to RAM for possible fetch via API, e.g. SE_FetchImage()
        /// Set true before calling SE_Init() to enable fetching first frame at time = 0
        /// </summary>
        /// <param name="state">true=capture images, false=don't capture (default, might improve performance on some systems)</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SaveImagesToRAM([MarshalAs(UnmanagedType.I1)] bool state);

        /// <summary>
        /// Capture rendered image to file. Call after SE_Init().
        /// </summary>
        /// <param name="nrOfFrames">-1=continuously, 0=stop, &gt;0=number of frames, e.g. 1=next frame only</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SaveImagesToFile(int nrOfFrames);

        /// <summary>
        /// Fetch captured image from RAM (internal memory)
        /// </summary>
        /// <param name="image">Pointer/reference to a SE_Image which will be filled in, even image data pointer</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_FetchImage(out SE_Image image);

        /// <summary>
        /// Register a function and optional parameter (ref) to be called back from esmini after each frame (update of scenario)
        /// The current state of specified entity will be returned.
        /// Complete or part of the state can then be overridden by calling the SE_ReportObjectPos/SE_ReportObjectRoadPos functions.
        /// Registered callbacks will be cleared between SE_Init calls.
        /// </summary>
        /// <param name="fnPtr">A pointer to the function to be invoked</param>
        /// <param name="user_data">Optional pointer to a local data object that will be passed as argument in the callback. Set 0/NULL if not needed.</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_RegisterImageCallback(ImageCallback fnPtr, IntPtr user_data);

        /// <summary>
        /// Store RGB (3*8 bits color values) image data as a PPM image file
        /// PPM info: http://paulbourke.net/dataformats/ppm/
        /// </summary>
        /// <param name="filename">File name including extension which should be ".ppm", e.g. "img0.ppm"</param>
        /// <param name="width">Width</param>
        /// <param name="height">Height</param>
        /// <param name="rgbData">Array of color values</param>
        /// <param name="pixelSize">Should be 3 (RGB/BGR)</param>
        /// <param name="upsidedown">false=lines stored from top to bottom, true=lines stored from bottom to top</param>
        /// <returns>0 if OK, -1 if failed to open file, -2 if unexpected pixelSize</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_WritePPMImage(string filename, int width, int height, in byte data, int pixelSize, int pixelFormat, [MarshalAs(UnmanagedType.I1)] bool upsidedown);

        /// <summary>
        /// Store RGB or BGR (3*8 bits color values) image data as a TGA image file
        /// TGA spec: https://www.dca.fee.unicamp.br/~martino/disciplinas/ea978/tgaffs.pdf
        /// TGA brief: http://paulbourke.net/dataformats/tga/
        /// </summary>
        /// <param name="filename">File name including extension which should be ".ppm", e.g. "img0.ppm"</param>
        /// <param name="width">Width</param>
        /// <param name="height">Height</param>
        /// <param name="rgbData">Array of color values</param>
        /// <param name="pixelSize">Should be 3 (RGB/BGR)</param>
        /// <param name="upsidedown">false=lines stored from top to bottom, true=lines stored from bottom to top</param>
        /// <returns>0 if OK, -1 if failed to open file, -2 if unexpected pixelSize</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_WriteTGAImage(string filename, int width, int height, in byte data, int pixelSize, int pixelFormat, [MarshalAs(UnmanagedType.I1)] bool upsidedown);

        /// <summary>
        /// Add a camera with relative position and orientation (heading and pitch)
        /// </summary>
        /// <param name="x">X coordinate relative vehicle currently in focus</param>
        /// <param name="y">Y coordinate relative vehicle currently in focus</param>
        /// <param name="z">Z coordinate relative vehicle currently in focus</param>
        /// <param name="h">Heading (yaw) (radians) relative vehicle currently in focus</param>
        /// <param name="p">Pitch (radians) relative vehicle currently in focus</param>
        /// <returns>index of the camera, can be used for SE_SetCameraMode(), -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddCustomCamera(double x, double y, double z, double h, double p);

        /// <summary>
        /// Add a fixed camera at custom global position and orientation (heading and pitch)
        /// </summary>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="z">Z coordinate</param>
        /// <param name="h">Heading (yaw) (radians)</param>
        /// <param name="p">P Pitch (radians)</param>
        /// <returns>index of the camera, can be used for SE_SetCameraMode(), -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddCustomFixedCamera(double x, double y, double z, double h, double p);

        /// <summary>
        /// Add a camera with relative position looking at current entity
        /// </summary>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="z">Z coordinate</param>
        /// <returns>index of the camera, can be used for SE_SetCameraMode(), -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddCustomAimingCamera(double x, double y, double z);

        /// <summary>
        /// Add a camera at fixed location but always looking at current entity
        /// </summary>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="z">Z coordinate</param>
        /// <param name="fixed_pos">Position is relative current vehicle (false) or fixed (true)</param>
        /// <returns>index of the camera, can be used for SE_SetCameraMode(), -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddCustomFixedAimingCamera(double x, double y, double z);

        /// <summary>
        /// Add a top view camera with fixed position and rotation
        /// </summary>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="z">Z coordinate</param>
        /// <param name="rot">Rotation (radians)</param>
        /// <returns>index of the camera, can be used for SE_SetCameraMode(), -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_AddCustomFixedTopCamera(double x, double y, double z, double rot);

        /// <summary>
        /// Select camera mode
        /// </summary>
        /// <param name="mode">Camera mode as in RubberbandManipulator::CAMERA_MODE</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetCameraMode(int mode);

        /// <summary>
        /// Sets the camera focus to the specified object
        /// </summary>
        /// <param name="object_id">The object to focus on</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_SetCameraObjectFocus(int object_id);

        /// <summary>
        /// Get the Id of the object the camera is focused on
        /// </summary>
        /// <returns>Id of the object, -1 on error e.g. scenario not initialized or viewer not enabled</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetObjectInCameraFocus();

        /// <summary>
        /// Get the position (x, y, z) and orientation (heading/yaw, pitch, roll) of the viewer camera
        /// </summary>
        /// <param name="x">reference to a variable returning the camera x coordinate</param>
        /// <param name="y">reference to a variable returning the camera y coordinate</param>
        /// <param name="z">reference to a variable returning the camera z coordinate</param>
        /// <param name="h">reference to a variable returning the camera heading/yaw</param>
        /// <param name="p">reference to a variable returning the camera pitch</param>
        /// <param name="r">reference to a variable returning the camera roll</param>
        /// <returns>0 if successful, -1 if not</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetCameraPos(out double x, out double y, out double z, out double h, out double p, out double r);

        /// <summary>
        /// Get the number Route points assigned for a specific vehicle
        /// </summary>
        /// <param name="object_id">The index of the vehicle</param>
        /// <returns>number of Route points (0 means no route assigned), -1 on error</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetNumberOfRoutePoints(int object_id);

        /// <summary>
        /// Get a specific route point for a certain vehicle
        /// </summary>
        /// <param name="object_id">The index of the vehicle</param>
        /// <param name="route_index">The index of Route point</param>
        /// <returns>0 if successful, -1 if not (e.g. wrong type)</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int SE_GetRoutePoint(int object_id, uint route_index, out SE_RouteInfo routeinfo);

        /// <summary>
        /// Get the total length of the route assigned to specified object
        /// </summary>
        /// <param name="object_id">Id of the object</param>
        /// <returns>Length (m) of route, 0.0 if no route is assigned</returns>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern double SE_GetRouteTotalLength(int object_id);

        /// <summary>
        /// Inject a speed action
        /// </summary>
        /// <param name="action">Struct including needed info for the action, see SE_SpeedActionStruct definition</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_InjectSpeedAction(out SE_SpeedActionStruct action);

        /// <summary>
        /// Inject a lane change action
        /// </summary>
        /// <param name="action">Struct including needed info for the action, see SE_LaneChangeActionStruct definition</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_InjectLaneChangeAction(out SE_LaneChangeActionStruct action);

        /// <summary>
        /// Inject a lane offset action
        /// </summary>
        /// <param name="action">Struct including needed info for the action, see SE_LaneOffsetActionStruct definition</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern void SE_InjectLaneOffsetAction(out SE_LaneOffsetActionStruct action);

        /// <summary>
        /// Check whether any injected action is ongoing
        /// </summary>
        /// <param name="action_type">Type of action, see esmini Action.hpp::ActionType enum. Set to -1 to check for any action.</param>
        [DllImport(NativeLibrary, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool SE_InjectedActionOngoing(int action_type);
    }
}