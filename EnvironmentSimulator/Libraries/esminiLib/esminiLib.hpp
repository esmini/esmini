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
#define SE_DLL_API __declspec(dllexport)
#else
#define SE_DLL_API  // Leave empty on Mac
#endif

#include <stdint.h>
#include <stdbool.h>

typedef uint32_t id_t;

#define SE_ID_UNDEFINED    0xffffffff
#define SE_IDX_UNDEFINED   0xffffffff
#define SE_PARAM_NAME_SIZE 32

typedef struct
{
    int   id;              // Automatically generated unique object id
    int   model_id;        // Id to control what 3D model to represent the vehicle - see carModelsFiles_[] in scenarioenginedll.cpp
    int   ctrl_type;       // 0: DefaultController 1: External. Further values see Controller::Type enum
    float timestamp;       // Not used yet (idea is to use it to interpolate position for increased sync bewtween simulators)
    float x;               // global x coordinate of position
    float y;               // global y coordinate of position
    float z;               // global z coordinate of position
    float h;               // heading/yaw in global coordinate system
    float p;               // pitch in global coordinate system
    float r;               // roll in global coordinate system
    id_t  roadId;          // road ID
    id_t  junctionId;      // Junction ID (-1 if not in a junction)
    float t;               // lateral position in road coordinate system
    int   laneId;          // lane ID
    float laneOffset;      // lateral offset from lane center
    float s;               // longitudinal position in road coordinate system
    float speed;           // speed
    float centerOffsetX;   // x coordinate of bounding box center relative object reference point (local coordinate system)
    float centerOffsetY;   // y coordinate of bounding box center relative object reference point (local coordinate system)
    float centerOffsetZ;   // z coordinate of bounding box center relative object reference point (local coordinate system)
    float width;           // width
    float length;          // length
    float height;          // height
    int   objectType;      // Main type according to entities.hpp / Object / Type
    int   objectCategory;  // Sub category within type, according to entities.hpp / Vehicle, Pedestrian, MiscObject / Category
    float wheel_angle;     // Steering angle of the wheel
    float wheel_rot;       // Rotation angle of the wheel
    int   visibilityMask;  // bitmask according to Object::Visibility (1 = Graphics, 2 = Traffic, 4 = Sensors)
} SE_ScenarioObjectState;

typedef struct
{
    float x;  // global x coordinate of position
    float y;  // global y coordinate of position
    float z;  // global z coordinate of position
    float h;  // heading/yaw in global coordinate system
    float p;  // pitch in global coordinate system
    // float r;                     // roll in global coordinate system
    // float width;                 // median width of the tire
    float wheel_radius;          // median radius of the wheel measured from the center of the wheel to the outer part of the tire
    float friction_coefficient;  // the value describes the kinetic friction of the tyre's contact point
    // float rotation_rate;         // rotation rate of the wheel
    // float rim_radius;  // 	median radius of the rim measured from the center to the outer, visible part of the rim
    int axle;   // 0=front, 1=next axle from front and so on. -1 indicates wheel is not existing.
    int index;  // The index of the wheel on the axle, counting in the direction of positive-y, that is, right-to-left. -1 indicates wheel
    // not existing.
} SE_WheelData;

// asciidoc tag::SE_RoadInfo_struct[]
typedef struct
{
    float global_pos_x;       // target position, in global coordinate system
    float global_pos_y;       // target position, in global coordinate system
    float global_pos_z;       // target position, in global coordinate system
    float local_pos_x;        // target position, relative vehicle (pivot position object) coordinate system
    float local_pos_y;        // target position, relative vehicle (pivot position object) coordinate system
    float local_pos_z;        // target position, relative vehicle (pivot position object) coordinate system
    float angle;              // heading angle to target from and relative vehicle (pivot position object) coordinate system
    float road_heading;       // road heading at steering target point
    float road_pitch;         // road pitch (inclination) at steering target point
    float road_roll;          // road roll (camber) at target point
    float trail_heading;      // trail heading (only when used for trail lookups, else equals road_heading)
    float curvature;          // road curvature at steering target point
    float speed_limit;        // speed limit given by OpenDRIVE speed max entry in m/s
    id_t  roadId;             // target position, road ID
    id_t  junctionId;         // target position, junction ID (SE_ID_UNDEFINED if not in a junction)
    int   laneId;             // target position, lane ID
    float laneOffset;         // target position, lane offset (lateral distance from lane center)
    float s;                  // target position, s (longitudinal distance along reference line)
    float t;                  // target position, t (lateral distance from reference line)
    int   road_type;          // road type given by OpenDRIVE road type, maps to roadmanager::Road::RoadType
    int   road_rule;          // road rule given by OpenDRIVE rule entry, maps to roadmanager::Road::RoadRule
    int   lane_type;          // lane type given by OpenDRIVE lane type, maps to roadmanager::Road::LaneType
    float trail_wheel_angle;  // trail wheel angle (only when used for trail lookups, e.g. ghost, else 0)
} SE_RoadInfo;
// asciidoc end::SE_RoadInfo_struct[]

typedef struct
{
    float x;           // Route point in the global coordinate system
    float y;           // Route point in the global coordinate system
    float z;           // Route point in the global coordinate system
    float h;           // Route point, heading in the global coordinate system
    id_t  roadId;      // Route point, road ID
    id_t  junctionId;  // Route point, junction ID (-1 if not in a junction)
    int   laneId;      // Route point, lane ID
    int   osiLaneId;   // Route point, osi lane ID
    float laneOffset;  // Route point, lane offset (lateral distance from lane center)
    float s;           // Route point, s (longitudinal distance along reference line)
    float t;           // Route point, t (lateral distance from reference line)
} SE_RouteInfo;

typedef struct
{
    id_t far_left_lb_id;
    id_t left_lb_id;
    id_t right_lb_id;
    id_t far_right_lb_id;
} SE_LaneBoundaryId;

typedef struct
{
    float ds;             // delta s (longitudinal distance)
    float dt;             // delta t (lateral distance)
    int   dLaneId;        // delta laneId (increasing left and decreasing to the right)
    float dx;             // delta x (world coordinate system)
    float dy;             // delta y (world coordinate system)
    bool  oppositeLanes;  // true if the two position objects are in opposite sides of reference lane
} SE_PositionDiff;

typedef struct
{
    float x;
    float y;
    float z;
    float h;
    float p;
    float speed;
    float wheel_rotation;
    float wheel_angle;
} SE_SimpleVehicleState;

typedef struct
{
    const char *name;   // Name of the parameter as defined in the OpenSCENARIO file
    void       *value;  // Pointer to value which can be an integer, double, bool or string (const char*) as defined in the OpenSCENARIO file
} SE_Parameter;

typedef struct
{
    const char *name;   // Name of the variable as defined in the OpenSCENARIO file
    void       *value;  // Pointer to value which can be an integer, double, bool or string (const char*) as defined in the OpenSCENARIO file
} SE_Variable;

typedef struct
{
    bool active;      // True: override; false: stop overriding
    int  type;        // According to Entities::OverrideType
    int  number;      // Gear number/mode depending on value_type
    int  value_type;  // According to Entities::OverrideGearType
                      // Manual type: Negative number are indicating reverse gears. Zero is neutral gear.
                      // Automatic type: (-1:Reverse, 0:Neutral, 1:Park, 2:Drive)
} SE_OverrideActionStatusGear;

typedef struct
{
    bool   active;      // True: override; false: stop overriding
    int    type;        // According to Entities::OverrideType
    double value;       // BrakeType will define how to interpret the value
    double maxRate;     // -1.0 indicates not set
    int    value_type;  // According to Entities::OverrideBrakeType
} SE_OverrideActionStatusBrake;

typedef struct
{
    bool   active;   // True: override; false: stop overriding
    double value;    // Depends on action, see SE_OverrideActionList
    double maxRate;  // -1.0 indicates not set
} SE_OverrideActionStatusPedals;

typedef struct
{
    bool   active;     // True: override; false: stop overriding
    double value;      // Depends on action, see SE_OverrideActionList
    double maxRate;    // Depends on action, see SE_OverrideActionList, -1.0 indicates not set
    double maxTorque;  // Depends on action, see SE_OverrideActionList, -1.0 indicates not set
} SE_OverrideActionStatusSteering;

typedef struct
{
    SE_OverrideActionStatusPedals   throttle;       // Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the throttle pedal.
    SE_OverrideActionStatusBrake    brake;          // Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the brake pedal.
    SE_OverrideActionStatusPedals   clutch;         // Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the clutch pedal.
    SE_OverrideActionStatusBrake    parkingBrake;   // Value range: [0..1]. 0 represents 0%, The value 1 represent the maximum parking brake state.
    SE_OverrideActionStatusSteering steeringWheel;  // Steering wheel angle. Unit: rad. (0: Neutral position, positive: Left, negative: Right)
    SE_OverrideActionStatusGear     gear;           // Gear status
} SE_OverrideActionList;

typedef struct
{
    int         id;           // just an unique identifier of the sign
    float       x;            // global x coordinate of sign position
    float       y;            // global y coordinate of sign position
    float       z;            // global z coordinate of sign position
    float       z_offset;     // z offset from road level
    float       h;            // global heading of sign orientation
    int         roadId;       // road id of sign road position
    float       s;            // longitudinal position along road
    float       t;            // lateral position from road reference line
    const char *name;         // sign name, typically used for 3D model filename
    int         orientation;  // 1=facing traffic in road direction, -1=facing traffic opposite road direction
    float       length;       // length as specified in OpenDRIVE
    float       height;       // height as specified in OpenDRIVE
    float       width;        // width as specified in OpenDRIVE
} SE_RoadSign;

typedef struct
{
    int fromLane;
    int toLane;
} SE_RoadObjValidity;

typedef struct
{
    int            width;
    int            height;
    int            pixelSize;    // 3 for RGB/BGR
    int            pixelFormat;  // 0x1907=RGB (GL_RGB), 0x80E0=BGR (GL_BGR)
    unsigned char *data;
} SE_Image;  // Should be synked with CommonMini/OffScreenImage

typedef struct
{
    float x_;  // Center offset in x direction.
    float y_;  // Center offset in y direction.
    float z_;  // Center offset in z direction.
} SE_Center;

typedef struct
{
    float width_;   // Width of the entity's bounding box. Unit: m; Range: [0..inf[.
    float length_;  // Length of the entity's bounding box. Unit: m; Range: [0..inf[.
    float height_;  // Height of the entity's bounding box. Unit: m; Range: [0..inf[.
} SE_Dimensions;

typedef struct
{
    SE_Center     center_;      // Represents the geometrical center of the bounding box
    SE_Dimensions dimensions_;  // Width, length and height of the bounding box.
} SE_OSCBoundingBox;

typedef struct
{
    int   id;  // id of object to perform action
    float speed;
    int   transition_shape;  // 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
    int   transition_dim;    // 0 = distance, 1 = rate, 2 = time
    float transition_value;
} SE_SpeedActionStruct;

typedef struct
{
    int   id;                // id of object to perform action
    int   mode;              // 0 = absolute, 1 = relative (own vehicle)
    int   target;            // target lane id (absolute or relative)
    int   transition_shape;  // 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
    int   transition_dim;    // 0 = distance, 1 = rate, 2 = time
    float transition_value;
} SE_LaneChangeActionStruct;

typedef struct
{
    int   id;  // id of object to perform action
    float offset;
    float maxLateralAcc;     // 0 = distance, 1 = rate, 2 = time
    int   transition_shape;  // 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
} SE_LaneOffsetActionStruct;

// Modes for interpret Z, Head, Pitch, Roll coordinate value as absolute or relative
// grouped as bitmask: 0000 => skip/use current, 0001=DEFAULT, 0011=ABS, 0111=REL
// example: Relative Z, Absolute H, Default R, Current P = SE_Z_REL | SE_H_ABS | SE_R_DEF = 4151 = 0001 0000 0011 0111
// Must match roadmanager::Position::PositionMode
typedef enum
{
    SE_Z_SET     = 1,  // 0001
    SE_Z_DEFAULT = 1,  // 0001
    SE_Z_ABS     = 3,  // 0011
    SE_Z_REL     = 7,  // 0111
    SE_Z_MASK    = 7,  // 0111
    SE_H_SET     = SE_Z_SET << 4,
    SE_H_DEFAULT = SE_Z_DEFAULT << 4,
    SE_H_ABS     = SE_Z_ABS << 4,
    SE_H_REL     = SE_Z_REL << 4,
    SE_H_MASK    = SE_Z_MASK << 4,
    SE_P_SET     = SE_Z_SET << 8,
    SE_P_DEFAULT = SE_Z_DEFAULT << 8,
    SE_P_ABS     = SE_Z_ABS << 8,
    SE_P_REL     = SE_Z_REL << 8,
    SE_P_MASK    = SE_Z_MASK << 8,
    SE_R_SET     = SE_Z_SET << 12,
    SE_R_DEFAULT = SE_Z_DEFAULT << 12,
    SE_R_ABS     = SE_Z_ABS << 12,
    SE_R_REL     = SE_Z_REL << 12,
    SE_R_MASK    = SE_Z_MASK << 12
} SE_PositionMode;

typedef enum
{
    SE_SET    = 1,  // Used by explicit set functions
    SE_UPDATE = 2   // Used by controllers updating the position
} SE_PositionModeType;

typedef enum
{
    SE_GHOST_TRAIL_OK          = 0,   // success
    SE_GHOST_TRAIL_ERROR       = -1,  // generic error
    SE_GHOST_TRAIL_NO_VERTICES = -2,  // ghost trail trajectory has no vertices
    SE_GHOST_TRAIL_TIME_PRIOR  = -3,  // given time < first timestamp in trajectory, snapped to start of trajectory
    SE_GHOST_TRAIL_TIME_PAST   = -4,  // given time > last timestamp in trajectory, snapped to end of trajectory
} SE_GhostTrailReturnCode;            // mirror roadmanager::GhostTrailReturnCode

typedef enum
{
    REL_DIST_UNDEFINED    = 0,
    REL_DIST_LATERAL      = 1,
    REL_DIST_LONGITUDINAL = 2,
    REL_DIST_CARTESIAN    = 3,
    REL_DIST_EUCLIDIAN    = 4
} SE_RelativeDistanceType;

typedef enum
{
    DEFAULT     = 0,
    API         = 1,
    API_AND_LOG = 2
} SE_OSIStaticReportMode;  // Must match roadmanager::OSIStaticReportMode

#ifdef __cplusplus
extern "C"
{
#endif

    // Basic interface
    //

    /**
            Add a search path for OpenDRIVE and 3D model files
            Needs to be called prior to SE_Init()
            @param path Path to a directory
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_AddPath(const char *path);

    /**
            Clear all search paths for OpenDRIVE and 3D model files
            Needs to be called prior to SE_Init()
    */
    SE_DLL_API void SE_ClearPaths();

    /**
            Specify scenario logfile (.txt) file path,
            optionally including directory path and/or filename
            Specify only directory (end with "/" or "\") to let esmini set default filename
            Specify only filename (no leading "/" or "\") to let esmini set default directory
            Set "" to disable logfile
            examples:
              "../logfile.txt" (relative current directory)
              "c:/tmp/esmini.log" (absolute path)
              "my.log" (put it in current directory)
              "c:/tmp/" (use default filename)
              "" (prevent creation of logfile)
            Note: Needs to be called prior to calling SE_Init()
            @param path Logfile path
    */
    SE_DLL_API void SE_SetLogFilePath(const char *logFilePath);

    /**
            Specify scenario recording (.dat) file path,
            optionally including directory path and/or filename
            Specify only directory (end with "/" or "\") to let esmini set default filename
            Specify only filename (no leading "/" or "\") to let esmini set default directory
            Set "" to use default .dat filename
            examples:
              "../my_sim.dat" (relative current directory)
              "c:/tmp/esmini.dat" (absolute path)
              "my_sim.dat" (put it in current directory)
              "c:/tmp/" (use default filename)
              "" (use current directory and default .dat filename)
            Note: Needs to be called prior to calling SE_Init()
            @param path Recording (.dat) file path
    */
    SE_DLL_API void SE_SetDatFilePath(const char *datFilePath);

    /**
    Get seed that esmini uses for current session. It can then be re-used
    in order to achieve repeatable results (for actions that involes some
    degree of randomness, e.g. TrafficSwarmAction).
    @return seed number
    */
    SE_DLL_API unsigned int SE_GetSeed();

    /**
            Set seed that will be used by esmini random number generator.
            Using same seed will ensure same result.
            Note: Also timesteps has to be equal. Make sure to use SE_StepDT()
            with fixed timestep, or at least same sequence of dt each run.
            @param seed number
    */
    SE_DLL_API void SE_SetSeed(unsigned int seed);

    /**
    Set option. The option will be unset on next scenario run. If persistence is required check SE_SetOptionPersistent.
    @param name the name of the option to be set
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetOption(const char *name);

    /**
    Unset option
    @param name the name of the option to be unset
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_UnsetOption(const char *name);

    /**
    Set option value. The option's value will be unset on next scenario run. If persistence is required check SE_SetOptionValuePersistent
    @param name the name of the option to be set
    @param value the value to assign to the option
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetOptionValue(const char *name, const char *value);

    /**
    Set option persistently. The option will remain over multiple scenario runs, until lib is reloaded.
    @param name the name of the option to be set
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetOptionPersistent(const char *name);

    /**
    Set option value persistently. The option value will remain over multiple scenario runs, until lib is reloaded.
    @param name the name of the option to be set
    @param value the value to assign to the option
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetOptionValuePersistent(const char *name, const char *value);

    /**
    Get option value
    @param name the name of the option whose value to fetch
    @return value of the option
    */
    SE_DLL_API const char *SE_GetOptionValue(const char *name);

    /**
    Get option value
    @param enum_value (index) value option whose value to fetch (see CommonMini/EnumConfig.hpp::esmini_options::CONFIG_ENUM)
    @return value of the option
    */
    SE_DLL_API const char *SE_GetOptionValueByEnum(unsigned int enum_value);

    /**
    Get option values count. Some options can have multiple values, this function returns the number of values present for the option.
    @param name the name of the option whose values count to fetch
    @return values count of the option
    */
    SE_DLL_API int SE_GetOptionValuesCount(const char *name);

    /**
    Get specified entry of option values, useful when option has multiple values.
    @param name the name of the option whose value to fetch
    @param index index of the value to fetch
    @return value of the option
    */
    SE_DLL_API const char *SE_GetOptionValueByIndex(const char *name, unsigned int index);

    /**
     Get option set status
     @param name is the name of the option whose value is fetch
     @return Returns true if the option is set otherwise false
    */
    SE_DLL_API bool SE_GetOptionSet(const char *name);

    /**
    Set window position and size. Must be called prior to SE_Init.
    @param x Screen coordinate in pixels for left side of window
    @param y Screen coordinate in pixels for top of window
    @param w Width in pixels
    @param h Height in pixels
    */
    SE_DLL_API void SE_SetWindowPosAndSize(int x, int y, int w, int h);

    /**
            Register a function and optional argument (ref) to be called back from esmini after ParameterDeclarations has been parsed,
            but before the scenario is initialized, i.e. before applying the actions in the Init block. One use-case is to
            set parameter values for initial entity states, e.g. s value in lane position. So this callback will happen just
            after parameters has been parsed, but before they are applied, providing an opportunity to control the initial
            states via API.
            Registered init callbacks are be cleared between SE_Init calls, i.e. needs to be registered
            @param fnPtr A pointer to the function to be invoked
            @param user_data Optional pointer to a local data object that will be passed as argument in the callback. Set 0/NULL if not needed.
    */
    SE_DLL_API void SE_RegisterParameterDeclarationCallback(void (*fnPtr)(void *), void *user_data);

    /**
            Configure tolerances/resolution for OSI road features
            @param max_longitudinal_distance Maximum distance between OSI points, even on straight road. Default=50(m)
            @param max_lateral_deviation Control resolution w.r.t. curvature default=0.05(m)
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetOSITolerances(double maxLongitudinalDistance, double maxLateralDeviation);

    /**
            Specify OpenSCENARIO parameter distribution file. Call BEFORE SE_Init.
            @param filename Name, including any path, of the parameter distribution file
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetParameterDistribution(const char *filename);

    /**
            Reset and disable parameter distribution.
    */
    SE_DLL_API void SE_ResetParameterDistribution();

    /**
            Get the number of parameter value permutations. Call AFTER SE_Init.
            @return number of permutations
    */
    SE_DLL_API unsigned int SE_GetNumberOfPermutations();

    /**
            Select parameter value permutation. Call BEFORE SE_Init, e.g. during or after preceding run.
            @return -1 on error else number of permutations
    */
    SE_DLL_API int SE_SelectPermutation(unsigned int index);

    /**
            Get current parameter permutation index.
            @return -1 on error or no parameter distribution loaded, else permutation index
    */
    SE_DLL_API int SE_GetPermutationIndex();

    /**
            Initialize the scenario engine

            @param oscFilename Path to the OpenSCENARIO file
            @param disable_ctrls 1=Any controller will be disabled 0=Controllers applied according to OSC file
            @param use_viewer Bitmask: 1=viewer on/off, 2=off-screen only, 4=capture-to-file, 8=disable info-text. Ex1: 0=>No viewer, ex2:
       1+2=3=>Off-screen
            @param threads 0=single thread, 1=viewer in a separate thread, parallel to scenario engine
            @param record Create recording for later playback 0=no recording 1=recording
            @return 0 if successful, -1 if not

            \use_viewer bitmask examples:
                            0: No viewer instantiated. Improved performance, use when viewer not needed.
                            1: Viewer instantiated with a window on screen (default viewer usage)
                            3 (1+2): Off-screen rendering only (no window on screen)
                            7 (1+2+4): Off-screen + save screenshots to file
                       11 (1+2+8): Off-screen + disable info-text for better remote/virtual desktop support
    */
    SE_DLL_API int SE_Init(const char *oscFilename, int disable_ctrls, int use_viewer, int threads, int record);

    /**
            Initialize the scenario engine

            @param oscAsXMLString OpenSCENARIO XML as string
            @param disable_ctrls 1=Any controller will be disabled 0=Controllers applied according to OSC file
            @param use_viewer Bitmask: 1=viewer on/off, 2=off-screen only, 4=capture-to-file, 8=disable info-text. Ex1: 0=>No viewer, ex2:
       1+2=3=>Off-screen
            @param threads 0=single thread, 1=viewer in a separate thread, parallel to scenario engine
            @param record Create recording for later playback 0=no recording 1=recording
            @return 0 if successful, -1 if not

            \use_viewer bitmask examples:
                            0: No viewer instantiated. Improved performance, use when viewer not needed.
                            1: Viewer instantiated with a window on screen (default viewer usage)
                            3 (1+2): Off-screen rendering only (no window on screen)
                            7 (1+2+4): Off-screen + save screenshots to file
                       11 (1+2+8): Off-screen + disable info-text for better remote/virtual desktop support
    */
    SE_DLL_API int SE_InitWithString(const char *oscAsXMLString, int disable_ctrls, int use_viewer, int threads, int record);

    /**
            Initialize the scenario engine
            @param argc Number of arguments
            @param argv Arguments
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_InitWithArgs(int argc, const char *argv[]);

    /**
            Step the simulation forward with specified timestep
            @param dt time step in seconds
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_StepDT(float dt);

    /**
            Step the simulation forward. Time step will be elapsed system (world) time since last step. Useful for interactive/realtime use cases.
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_Step();

    /**
            Stop simulation gracefully. Two purposes: 1. Release memory and 2. Prepare for next simulation, e.g. reset object lists.
    */
    SE_DLL_API void SE_Close();

    /**
            Enable or disable log to stdout/console
            Deprecated, use SE_SetOption() / SE_UnsetOption() with "disable_stdout" instead
            which also allows for persistant setting
            @param mode true=enable, false=disable
    */
    SE_DLL_API void SE_LogToConsole(bool mode);

    /**
    Enable or disable global collision detection
    @param mode true=enable, false=disable
    */
    SE_DLL_API void SE_CollisionDetection(bool mode);

    /**
            Get simulation time in seconds - float (32 bit) precision
    */
    SE_DLL_API float SE_GetSimulationTime();  // Get simulation time in seconds

    /**
            Get simulation time in seconds - double (64 bit) precision
    */
    SE_DLL_API double SE_GetSimulationTimeDouble();

    /**
            Get simulation time step in seconds
            The time step is calculated as difference since last call to same funtion.
            Clamped to some reasonable values. First call returns smallest delta (typically 1 ms).
    */
    SE_DLL_API float SE_GetSimTimeStep();

    /**
            Is esmini about to quit?
            @return 0 if not, 1 if yes, -1 if some error e.g. scenario not loaded
    */
    SE_DLL_API int SE_GetQuitFlag();

    /**
            Is esmini paused (via space button)?
            @return 0 if not, 1 if yes, -1 if some error e.g. scenario not loaded
    */
    SE_DLL_API int SE_GetPauseFlag();

    /**
            Get name of currently referred and loaded OpenDRIVE file
            @return filename as string (const, since it's allocated and handled by esmini)
    */
    SE_DLL_API const char *SE_GetODRFilename();

    /**
            Get name of currently referred and loaded SceneGraph file
            @return filename as string (const, since it's allocated and handled by esmini)
    */
    SE_DLL_API const char *SE_GetSceneGraphFilename();

    /**
            Get the number of named parameters within the current scenario
            @return number of parameters, -1 on error
    */
    SE_DLL_API int SE_GetNumberOfParameters();

    /**
            Get the name of a named parameter
            @param index The index of the parameter, range [0:numberOfParameters-1]
            @param Output parameter type 1=int, 2=double, 3=string (const char*), 4=bool, see OSCParameterDeclarations/ParameterType
            @return name if found, else 0
    */
    SE_DLL_API const char *SE_GetParameterName(int index, int *type);

    /**
            Get the number of named variables within the current scenario
            @return number of variables, -1 on error
    */
    SE_DLL_API int SE_GetNumberOfVariables();

    /**
            Get the name of a named variable
            @param index The index of the variable, range [0:numberOfVariables-1]
            @param Output variable type 1=int, 2=double, 3=string (const char*), 4=bool, see OSCParameterDeclarations/ParameterType
            @return name if found, else 0
    */
    SE_DLL_API const char *SE_GetVariableName(int index, int *type);

    /**
            Get the number of vehicle properties by index
            @param index The index of the vehicle
            @return number of parameters if found, -1 indicating some error
    */
    SE_DLL_API int SE_GetNumberOfProperties(int index);
    /**
            Get the number of vehicle properties by index
            @param index The index of the vehicle
            @param propertyIndex The index of the property
            @return the name of the property by index if found, else ""
    */
    SE_DLL_API const char *SE_GetObjectPropertyName(int index, int propertyIndex);
    /**
            Get the value of a vehicle property by name
            @param index The index of the vehicle
            @param vehiclePropertyName the vehicle property name
            @return the value of a vehicle property by name if found, else ""
    */
    SE_DLL_API const char *SE_GetObjectPropertyValue(int index, const char *objectPropertyName);

    /**
            Set value of named parameter
            @param parameter Struct object including name of parameter and pointer to value, see SE_Parameter declaration
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetParameter(SE_Parameter parameter);

    /**
            Get value of named parameter. The value within the parameter struct will be filled in.
            @param parameter Pointer to parameter struct object, see SE_Parameter declaration.
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetParameter(SE_Parameter *parameter);

    /**
            Get typed value of named parameter
            @parameterName Name of the parameter
            @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetParameterInt(const char *parameterName, int *value);

    /**
            Get typed value of named parameter
            @parameterName Name of the parameter
            @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetParameterDouble(const char *parameterName, double *value);

    /**
    Get typed value of named parameter
    @parameterName Name of the parameter
    @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetParameterString(const char *parameterName, const char **value);

    /**
    Get typed value of named parameter
    @parameterName Name of the parameter
    @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetParameterBool(const char *parameterName, bool *value);

    /**
    Set typed value of named parameter
    @parameterName Name of the parameter
    @value Value
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetParameterInt(const char *parameterName, int value);

    /**
    Set typed value of named parameter
    @parameterName Name of the parameter
    @value Value
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetParameterDouble(const char *parameterName, double value);

    /**
    Set typed value of named parameter
    @parameterName Name of the parameter
    @value Value
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetParameterString(const char *parameterName, const char *value);

    /**
    Set typed value of named parameter
    @parameterName Name of the parameter
    @value Value
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetParameterBool(const char *parameterName, bool value);

    SE_DLL_API int SE_SetVariable(SE_Variable variable);

    /**
            Get value of named parameter. The value within the parameter struct will be filled in.
            @param parameter Pointer to parameter struct object, see SE_Parameter declaration.
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetVariable(SE_Variable *variable);

    /**
            Get typed value of named variable
            @variableName Name of the variable
            @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetVariableInt(const char *variableName, int *value);

    /**
            Get typed value of named variable
            @variableName Name of the variable
            @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetVariableDouble(const char *variableName, double *value);

    /**
    Get typed value of named variable
    @variableName Name of the variable
    @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetVariableString(const char *variableName, const char **value);

    /**
    Get typed value of named variable
    @variableName Name of the variable
    @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetVariableBool(const char *variableName, bool *value);

    /**
    Set typed value of named variable
    @variableName Name of the variable
    @value Value
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetVariableInt(const char *variableName, int value);

    /**
    Set typed value of named variable
    @variableName Name of the variable
    @value Value
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetVariableDouble(const char *variableName, double value);

    /**
    Set typed value of named variable
    @variableName Name of the parameter
    @value Value
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetVariableString(const char *variableName, const char *value);

    /**
    Set typed value of named variable
    @variableName Name of the variable
    @value Value
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetVariableBool(const char *variableName, bool value);

    SE_DLL_API void *SE_GetODRManager();

    /**
    Specify if and how position object will align to the road. The setting is done for individual components:
    Z (elevation), Heading, Pitch, Roll and separately for set- and update operation. Set operations represents
    when position is affected by API calls, e.g. updateObjectWorldPos(). Update operations represents when the
    position is updated implicitly by the scenarioengine, e.g. default controller moving a vehicle along the lane.
    @param object_id Id of the object
    @param type Type of operations the setting applies to, according to SE_PositionModeType enum
    @param mode Bitmask combining values from SE_PositionMode enum
    example: To set relative z and absolute roll: (SE_Z_REL | SE_R_ABS) or (7 | 12288) = (7 + 12288) = 12295
    according to roadmanager::PosModeType
    */
    SE_DLL_API void SE_SetObjectPositionMode(int object_id, SE_PositionModeType type, int mode);

    /**
    Set default alignment mode for SET or UPDATE operations. See roadmanager::Position::GetModeDefault() to find out
    what are the default modes.
    @param object_id Id of the object
    @param type Type of operations the setting applies to, according to SE_PositionModeType enum
    according to roadmanager::PosModeType
    */
    SE_DLL_API void SE_SetObjectPositionModeDefault(int object_id, SE_PositionModeType type);

    /**
            Add object with bounding box automatically adapted to 3D model (scale mode BB_TO_MODEL)
            Should be followed by one of the SE_Report functions to establish initial state.
            @param object_name Name of the object, preferably be unique
            @param object_type Type of the object. See Entities.hpp::Object::Type. Default=1 (VEHICLE).
            @param object_category Category of the object. Depends on type, see descendants of Entities.hpp::Object. Set to 0 if not known.
            @param object_role role of the object. Depends on type, See Entities.hpp::Object::Role. Set to 0 if not known.
            @param model_id Id of the 3D model to represent the object. See resources/model_ids.txt. Set -1 to skip.
            @param model_3d Filename of 3D model to represent the object. Overrides model_id. Set NULL to skip.
            @return Id [0..inf] of the added object successful, -1 on failure
    */
    SE_DLL_API int SE_AddObject(const char *object_name, int object_type, int object_category, int object_role, int model_id, const char *model_3d);

    /**
            Add object with specified bounding box.
            Should be followed by one of the SE_Report functions to establish initial state.
            For scale_mode BB_TO_MODEL, set bounding_box to whatever, e.g. {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, or use SE_AddObject()
            @param object_name Name of the object, preferably be unique
            @param object_type Type of the object. See Entities.hpp::Object::Type. Default=1 (VEHICLE).
            @param object_category Category of the object. Depends on type, see descendants of Entities.hpp::Object. Set to 0 if not known.
            @param object_role role of the object. Depends on type, See Entities.hpp::Object::Role. Set to 0 if not known.
            @param model_id Id of the 3D model to represent the object. See resources/model_ids.txt. Set -1 to skip.
            @param model_3d Filename of 3D model to represent the object. Overrides model_id. Set NULL to skip.
            @param bounding_box sets the internal bounding box of the model and will also be used to scale 3D model accordingly.
            @param scale_mode 0=NONE, 1=BB_TO_MODEL, 2=MODEL_TO_BB (recommended). See CommonMini::EntityScaleMode enum for details.
            @return Id [0..inf] of the added object successful, -1 on failure
    */
    SE_DLL_API int SE_AddObjectWithBoundingBox(const char       *object_name,
                                               int               object_type,
                                               int               object_category,
                                               int               object_role,
                                               int               model_id,
                                               const char       *model_3d,
                                               SE_OSCBoundingBox bounding_box,
                                               int               scale_mode);

    /**
            Delete object
            @param object_id Id of the object
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_DeleteObject(int object_id);

    /**
            Report object position in cartesian coordinates
            @param object_id Id of the object
            @param timestamp Timestamp (not really used yet, OK to set 0)
            @param x X coordinate
            @param y Y coordinate
            @param z Z coordinate, set std::nanf("") to ignore, i.e. re-use current value (#include <cmath>)
            @param h Heading / yaw
            @param p Pitch, set std::nanf("") to ignore, i.e. re-use current value (#include <cmath>)
            @param r Roll, set std::nanf("") to ignore, i.e. re-use current value (#include <cmath>)
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectPos(int object_id, float timestamp, float x, float y, float z, float h, float p, float r);

    /**
            Report object position in cartesian coordinates, with detailed control of absolute or relative coordinates
            @param object_id Id of the object
            @param timestamp Timestamp (not really used yet, OK to set 0)
            @param x X coordinate
            @param y Y coordinate
            @param z Z coordinate, set std::nanf("") to ignore, i.e. re-use current value (#include <cmath>)
            @param h Heading / yaw
            @param p Pitch, set std::nanf("") to ignore, i.e. re-use current value (#include <cmath>)
            @param r Roll, set std::nanf("") to ignore, i.e. re-use current value (#include <cmath>)
            @param mode Explicit mode, override current setting. E.g. POS_REL_Z | POS_ABS_H, see SE_PositionMode enum
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectPosMode(int object_id, float timestamp, float x, float y, float z, float h, float p, float r, int mode);

    /**
            Report object position in limited set of cartesian coordinates x, y and heading,
            the remaining z, pitch and roll will be aligned to the road surface
            @param object_id Id of the object
            @param timestamp Timestamp (not really used yet, OK to set 0)
            @param x X coordinate
            @param y Y coordinate
            @param h Heading / yaw
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectPosXYH(int object_id, float timestamp, float x, float y, float h);

    /**
            Report object position in road coordinates
            @param object_id Id of the object
            @param timestamp Timestamp (not really used yet, OK to set 0)
            @param roadId Id of the road
            @param laneId Id of the lane
            @param laneOffset Lateral offset from center of specified lane
            @param s Longitudinal distance of the position along the specified road
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectRoadPos(int object_id, float timestamp, id_t roadId, int laneId, float laneOffset, float s);

    /**
            Report object longitudinal speed. Useful for an external longitudinal controller.
            @param object_id Id of the object
            @param speed Speed in forward direction of the enitity
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectSpeed(int object_id, float speed);

    /**
            Report object lateral position relative road centerline. Useful for an external lateral controller.
            @param object_id Id of the object
            @param t Lateral position
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectLateralPosition(int object_id, float t);

    /**
            Report object lateral position by lane id and lane offset. Useful for an external lateral controller.
            @param object_id Id of the object
            @param laneId Id of the lane
            @param laneOffset Lateral offset from center of specified lane
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectLateralLanePosition(int object_id, int laneId, float laneOffset);

    /**
            Report object position in cartesian coordinates
            @param iobject_idd Id of the object
            @param timestamp Timestamp (not really used yet, OK to set 0)
            @param x_vel X component of linear velocity
            @param y_vel Y component of linear velocity
            @param z_vel Z component of linear velocity
            @return 0 if successful, -1 if not

    */
    SE_DLL_API int SE_ReportObjectVel(int object_id, float timestamp, float x_vel, float y_vel, float z_vel);

    /**
            Report object position in cartesian coordinates
            @param object_id Id of the object
            @param timestamp Timestamp (not really used yet, OK to set 0)
            @param h_vel Heading component of angular velocity
            @param p_vel Pitch component of angular velocity
            @param r_vel Roll component of angular velocity
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectAngularVel(int object_id, float timestamp, float h_rate, float p_rate, float r_rate);

    /**
            Report object position in cartesian coordinates
            @param object_id Id of the object
            @param timestamp Timestamp (not really used yet, OK to set 0)
            @param x_acc X component of linear acceleration
            @param y_acc Y component of linear acceleration
            @param z_acc Z component of linear acceleration
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectAcc(int object_id, float timestamp, float x_acc, float y_acc, float z_acc);

    /**
            Report object position in cartesian coordinates
            @param object_id Id of the object
            @param timestamp Timestamp (not really used yet, OK to set 0)
            @param h_acc Heading component of angular acceleration
            @param p_acc Pitch component of angular acceleration
            @param r_acc Roll component of angular acceleration
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectAngularAcc(int object_id, float timestamp, float h_acc, float p_acc, float r_acc);

    /**
            Report object wheel status
            @param object_id Id of the object
            @param rotation Wheel rotation
            @param angle Wheel steering angle
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectWheelStatus(int object_id, float rotation, float angle);

    /**
            Specify which lane types the position object snaps to (is aware of)
            @param object_id Id of the object
            @param laneTypes A combination (bitmask) of lane types according to roadmanager::Lane::LaneType
            examples: ANY_DRIVING = 1966594, ANY_ROAD = 1966990, ANY = -1
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetSnapLaneTypes(int object_id, int laneTypes);

    /**
            Controls whether to keep lane ID regardless of lateral position or snap to closest lane (default)
            @param object_id Id of the object
            @parameter mode True=keep lane False=Snap to closest (default)
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetLockOnLane(int object_id, bool mode);

    /**
            Get the number of entities in the current scenario
            @return Number of entities, -1 on error e.g. scenario not initialized
    */
    SE_DLL_API int SE_GetNumberOfObjects();

    /**
            Get the Id of an entity present in the current scenario
            @param index Index of the object. Note: not ID
            @return Id of the object, -1 on error e.g. scenario not initialized
    */
    SE_DLL_API int SE_GetId(int index);

    /**
            Get the Id of an entity present in the current scenario
            @param name Name of the object.
            @return Id of the object, -1 on error e.g. scenario not initialized
    */
    SE_DLL_API int SE_GetIdByName(const char *name);

    /**
            Get the state of specified object
            @param object_id Id of the object
            @param state Pointer/reference to a SE_ScenarioObjectState struct to be filled in
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetObjectState(int object_id, SE_ScenarioObjectState *state);

    /**
            Get the object route status
            @param object_id Id of the object
            @return 0 if route not assigned, 1 if outside assigned route, 2 if on assigned route, -1 on error
    */
    SE_DLL_API int SE_GetObjectRouteStatus(int object_id);

    /**
        Find out what lane type object is currently in, reference point projected on road
        Can be used for checking exact lane type or combinations by bitmask.
        Example 1: Check if on border lane: SE_GetObjectLaneType(id) == (1 << 6)
        Example 2: Check if on any drivable lane: SE_GetObjectLaneType(id) & 1966594
        Example 3: Check if on any road lane: SE_GetObjectLaneType(id) & 1966990
        Example 4: Check for no lane (outside defined lanes): SE_GetObjectLaneType(id) == 1
        @param object_id Id of the object
        @return lane type according to enum roadmanager::Lane::LaneType
    */
    SE_DLL_API int SE_GetObjectInLaneType(int object_id);

    /**
            Get the overrideActionStatus of specified object
            @param objectId Id of the object
            @param list Pointer/reference to a SE_OverrideActionList struct to be filled in
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetOverrideActionStatus(int objectId, SE_OverrideActionList *list);

    /**
            Get the type name of the specifed vehicle-, pedestrian- or misc object
            @param object_id Id of the object
            @return Name
    */
    SE_DLL_API const char *SE_GetObjectTypeName(int object_id);

    /**
            Get the name of specified object
            @param object_id Id of the object
            @return Name
    */
    SE_DLL_API const char *SE_GetObjectName(int object_id);

    /**
            Get the 3D model filename of the specifed object
            @param object_id Id of the object
            @return Name
    */
    SE_DLL_API const char *SE_GetObjectModelFileName(int object_id);

    /**
            Check whether an object has a ghost (special purpose lead vehicle)
            @param object_id Id of the object
            @return 1 if ghost, 0 if not, -1 indicates error e.g. scenario not loaded
    */
    SE_DLL_API int SE_ObjectHasGhost(int object_id);

    /**
            Get ID of the ghost associated with given object
            @param object_id Id of the ghost object
            @return ghost object ID, -1 if ghost does not exist for given object
    */
    SE_DLL_API int SE_GetObjectGhostId(int object_id);

    /**
            Get the state of specified object's ghost (special purpose lead vehicle)
            @param object_id Id of the object to which the ghost is attached
            @param state Pointer/reference to a SE_ScenarioObjectState struct to be filled in
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetObjectGhostState(int object_id, SE_ScenarioObjectState *state);

    /**
            Get the number of collisions the specified object currently is involved in
            @param object_id Id of the object
            @return Number of objects that specified object currently is overlapping/colliding with. -1 on error.
    */
    SE_DLL_API int SE_GetObjectNumberOfCollisions(int object_id);

    /**
            Get the object involved in specified collision by object id and collision index
            @param object_id Id of the object
            @param index Index of collision (one object can be involvoed in multiple simultaneous collisions)
            @return object_id of colliding object. -1 if unsuccessful.
    */
    SE_DLL_API int SE_GetObjectCollision(int object_id, int index);

    /**
            Get the traveled distance of an object
            @param object_id Id of the object
            @return traveled distance if successful, std::nanf if not
    */
    SE_DLL_API float SE_GetObjectOdometer(int object_id);

    /**
           Get the angular velocity of the specified object
           @param object_id Id of the object
           @param x reference to a variable returning the velocity along global x-axis
           @param y reference to a variable returning the velocity along global y-axis
           @param z reference to a variable returning the velocity along global z-axis
           @return 0 if successful.
    */
    SE_DLL_API int SE_GetObjectVelocityGlobalXYZ(int object_id, float *vel_x, float *vel_y, float *vel_z);

    /**
            Get the angular velocity of the specified object
            @param object_id Id of the object
            @param h_rate The rate of the heading.
            @param p_rate The rate of the pitch.
            @param r_rate The rate of the roll.
            @return 0 if successful.
     */
    SE_DLL_API int SE_GetObjectAngularVelocity(int object_id, float *h_rate, float *p_rate, float *r_rate);

    /**
            Get the angular velocity of the specified object
            @param object_id Id of the object
            @param h_acc The rate of the heading.
            @param p_acc The rate of the pitch.
            @param r_acc The rate of the roll.
            @return 0 if successful.
     */
    SE_DLL_API int SE_GetObjectAngularAcceleration(int object_id, float *h_acc, float *p_acc, float *r_acc);

    /**
            Get the acceleration magnitude of specified object
            @param object_id Id of the object
            @return the acceleration if successful, std::nanf if not
    */
    SE_DLL_API float SE_GetObjectAcceleration(int object_id);

    /**
            Get the acceleration components of specified object in global x, y, z coordinates
            @param object_id Id of the object
            @param x reference to a variable returning the acceleration along global x-axis
            @param y reference to a variable returning the acceleration along global y-axis
            @param z reference to a variable returning the acceleration along global z-axis
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetObjectAccelerationGlobalXYZ(int object_id, float *acc_x, float *acc_y, float *acc_z);

    /**
            Get the acceleration components of specified object in local x,y coordinates
            @param object_id Id of the object
            @param lat reference to a variable returning the acceleration along local y-axis
            @param long reference to a variable returning the acceleration along local x-axis
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetObjectAccelerationLocalLatLong(int object_id, float *acc_lat, float *acc_long);

    /**
            Get the number of wheels of an object
            @param object_id Id of the object
            @return number of wheels on object if successful, -1 if not
    */
    SE_DLL_API int SE_GetObjectNumberOfWheels(int object_id);

    /**
            Get wheel information of specified object
            @param object_id Id of the object
            @param wheeldata reference to a struct in which to return the wheeldata
            @param wheel_index index of wheeldata to return
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetObjectWheelData(int object_id, int wheel_index, SE_WheelData *wheeldata);

    /**
            Get the unit of specified speed (in OpenDRIVE road type element).
            All roads will be looped in search for such an element. First found will be used.
            If speed is specified withouth the optional unit, SI unit m/s is assumed.
            If no speed entries is found, undefined will be returned.
            @return -1=Error, 0=Undefined, 1=km/h 2=m/s, 3=mph
    */
    SE_DLL_API int SE_GetSpeedUnit();

    /**
            Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
            @param object_id Handle to the position object from which to measure
            @param lookahead_distance The distance, along the road, to the point
            @param data Struct including all result values, see typedef for details
            @param lookAheadMode Measurement strategy: Along 0=lane center, 1=road center (ref line) or 2=current lane offset. See
            roadmanager::Position::LookAheadMode enum
            @param inRoadDrivingDirection If true look along lane driving direction. If false, look in closest direction according to object heading.
            @return 0 = OK,
                    ERROR_OFF_ROAD = -4,
                    ERROR_END_OF_ROUTE = -3,
                    ERROR_END_OF_ROAD = -2,
                    ERROR_GENERIC = -1,
                    ENTERED_NEW_ROAD = 1,
                    MADE_JUNCTION_CHOICE = 2
                    (see roadmanager.hpp -> Position::ReturnCode)
    */
    SE_DLL_API int SE_GetRoadInfoAtDistance(int          object_id,
                                            float        lookahead_distance,
                                            SE_RoadInfo *data,
                                            int          lookAheadMode,
                                            bool         inRoadDrivingDirection);

    /**
            Get information suitable for driver modeling of a point at a specified distance from object along the route ahead
            @param object_id Handle to the position object from which to measure
            @param lookahead_distance The distance, along the road, to the point
            @param data Struct including all result values, see typedef for details
            @param lookAheadMode Measurement strategy: Along 0=lane center, 1=road center (ref line) or 2=current lane offset. See
            roadmanager::Position::LookAheadMode enum
            @param inRoadDrivingDirection If true look along lane driving direction. If false, look in closest direction according to object heading.
            @return 0 = OK,
                    ERROR_OFF_ROAD = -4,
                    ERROR_END_OF_ROUTE = -3,
                    ERROR_END_OF_ROAD = -2,
                    ERROR_GENERIC = -1,
                    ENTERED_NEW_ROAD = 1,
                    MADE_JUNCTION_CHOICE = 2
                    (see roadmanager.hpp -> Position::ReturnCode)
    */
    SE_DLL_API int SE_GetRoadInfoAlongRoute(int          object_id,
                                            float        lookahead_distance,
                                            SE_RoadInfo *data,
                                            int          lookAheadMode,
                                            bool         inRoadDrivingDirection);

    /**
            Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle
            @param object_id Id of the object from which to measure (the actual externally controlled Ego vehicle, not ghost)
            @param lookahead_distance The distance, along the ghost trail, to the point from the current Ego vehicle location
            @param data Struct including all result values, see typedef for details
            @param speed_ghost reference to a variable returning the speed that the ghost had at this point along trail
            @param timestamp reference to a variable returning the timestamp of this point along trail
            @return 0 if successful, < 0 see SE_GhostTrailReturnCode enum for error/information codes
    */
    SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *data, float *speed_ghost, float *timestamp);

    /**
            Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle
            @param object_id Id of the object from which to measure (the actual externally controlled Ego vehicle, not ghost)
            @param time Simulation time (subtracting headstart time, i.e. time=0 gives the initial state)
            @param data Struct including all result values, see typedef for details
            @param speed_ghost reference to a variable returning the speed that the ghost had at this point along trail
            @return 0 if successful, < 0 see SE_GhostTrailReturnCode enum for error/information codes
    */
    SE_DLL_API int SE_GetRoadInfoGhostTrailTime(int object_id, float time, SE_RoadInfo *data, float *speed_ghost);

    /**
            Find out the delta between two objects, e.g. distance (long and lat) and delta laneId
            search range is 1000 meters
            @param object_a_id Id of the object from which to measure
            @param object_b_id Id of the object to which the distance is measured
            @param free_space Measure distance between bounding boxes (true) or between ref points (false)
            @param pos_diff Struct including all result values, see PositionDiff typedef
            @return 0 if successful, -2 if route between positions can't be found, -1 if some other error
    */
    SE_DLL_API int SE_GetDistanceToObject(int object_a_id, int object_b_id, bool free_space, SE_PositionDiff *pos_diff);

    /**
            Optimized method to find the relative distance between two objects in the entities local coordinate system.
            The method discards any object >500m away, will have a reduced tracking frequency (3s) for objects >tracking limit and avoids redundant
       calculations.
            @param object_a_id Id of the object from which to measure
            @param object_b_id Id of the object to which the distance is measured
            @param dist_type Enum specifying what distance to measure
            @param distance reference to a variable returning the distance
            @param timestamp reference to a variable returning the timestamp of the distance sample
            @return 0 if successful, -1 if the distance measurement failed and -2 if the objects are out of bounds (>500m) or didn't update the
       current sample.
    */
    SE_DLL_API int SE_SimpleGetDistanceToObject(const int               object_a_id,
                                                const int               object_b_id,
                                                SE_RelativeDistanceType dist_type,
                                                const double            tracking_limit,
                                                double                 *distance,
                                                double                 *timestamp);

    /**
            Create an ideal object sensor and attach to specified vehicle
            @param object_id Handle to the object to which the sensor should be attached
            @param x Position x coordinate of the sensor in vehicle local coordinates
            @param y Position y coordinate of the sensor in vehicle local coordinates
            @param z Position z coordinate of the sensor in vehicle local coordinates
            @param h heading of the sensor in vehicle local coordinates
            @param fovH Horizontal field of view, in degrees
            @param rangeNear Near value of the sensor depth range
            @param rangeFar Far value of the sensor depth range
            @param maxObj Maximum number of objects theat the sensor can track
            @return Sensor ID (Global index of sensor), -1 if unsucessful
    */
    SE_DLL_API int SE_AddObjectSensor(int object_id, float x, float y, float z, float h, float rangeNear, float rangeFar, float fovH, int maxObj);

    /**
            Retrieve the total number of sensors attached to any objects
            @return -1 on failure, else the number of sensors
    */
    SE_DLL_API int SE_GetNumberOfObjectSensors();

    /**
            Allow to view detected sensor data.
            @param object_id Handle to the object to which the sensor should be attached
            @return Sensor ID (Global index of sensor), -1 if unsucessful
    */
    SE_DLL_API int SE_ViewSensorData(int object_id);

    /**
            Fetch list of identified objects from a sensor
            @param sensor_id Handle (index) to the sensor
            @param list Array of object indices
            @return Number of identified objects, i.e. length of list. -1 if unsuccesful.
    */
    SE_DLL_API int SE_FetchSensorObjectList(int sensor_id, int *list);

    /**
            Register a function and optional parameter (ref) to be called back from esmini after each frame (update of scenario)
            The current state of specified entity will be returned.
            Complete or part of the state can then be overridden by calling the SE_ReportObjectPos/SE_ReportObjectRoadPos functions.
            Registered callbacks will be cleared between SE_Init calls.
            @param object_id Handle to the position object (entity)
            @param fnPtr A pointer to the function to be invoked
            @param user_data Optional pointer to a local data object that will be passed as argument in the callback. Set 0/NULL if not needed.
    */
    SE_DLL_API void SE_RegisterObjectCallback(int object_id, void (*fnPtr)(SE_ScenarioObjectState *, void *), void *user_data);

    /**
    Registers a function to be called back from esmini every time a condition is triggered.
    The name of the respective condition and the current timestamp will be returned.
    Registered callbacks will be cleared between SE_Init calls.
    @param fnPtr A pointer to the function to be invoked
    */
    SE_DLL_API void SE_RegisterConditionCallback(void (*fnPtr)(const char *name, double timestamp));

    /**
    Registers a function to be called back from esmini every time a StoryBoardElement changes its state.
    The name of the respective StoryBoardElement, the type, state, and full path (parent names delimited by /) will be returned.
    See StoryBoardElement.hpp -> StoryBoardElement class ElementType and State enums for type and state values.
    Registered callbacks will be cleared between SE_Init calls.
    @param fnPtr A pointer to the function to be invoked
    */
    SE_DLL_API void SE_RegisterStoryBoardElementStateChangeCallback(void (*fnPtr)(const char *name, int type, int state, const char *full_path));

    /**
            Get the number of road signs along specified road
            @param road_id The road along which to look for signs
            @return Number of road signs
    */
    SE_DLL_API unsigned int SE_GetNumberOfRoadSigns(id_t road_id);

    /**
            Get information on specifed road sign
            @param road_id The road of which to look for the signs
            @param index Index of the sign. Note: not ID
            @param road_sign Pointer/reference to a SE_RoadSign struct to be filled in
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetRoadSign(id_t road_id, unsigned int index, SE_RoadSign *road_sign);

    /**
            Get the number of lane validity records of specified road object/sign
            @param road_id The road of which to look for the sign
            @param index Index of the sign. Note: not ID
            @return Number of validity records of specified road sign
    */
    SE_DLL_API unsigned int SE_GetNumberOfRoadSignValidityRecords(id_t road_id, unsigned int index);

    /**
            Get specified validity record of specifed road sign
            @param road_id The road of which to look for the sign
            @param signIndex Index of the sign. Note: not ID
            @param validityIndex Index of the validity record
            @param road_sign Pointer/reference to a SE_RoadObjValidity struct to be filled in
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetRoadSignValidityRecord(id_t road_id, unsigned int signIndex, unsigned int validityIndex, SE_RoadObjValidity *validity);

    /**
            Get original string ID asoociated with specified road
            @param road_id The integer ID road
            @return string ID, empty string if not found
    */
    SE_DLL_API const char *SE_GetRoadIdString(id_t road_id);

    /**
            Get integer road ID associated with specified road string ID
            @param road_id_str The road string ID
            @return road ID, -1 if not found
    */
    SE_DLL_API id_t SE_GetRoadIdFromString(const char *road_id_str);

    /**
            Get original string ID asoociated with specified junction
            @param road_id The integer ID junction
            @return string ID, empty string if not found
    */
    SE_DLL_API const char *SE_GetJunctionIdString(id_t junction_id);

    /**
            Get integer junction ID associated with specified junction string ID
            @param road_id_str The junction string ID
            @return junction ID, -1 if not found
    */
    SE_DLL_API id_t SE_GetJunctionIdFromString(const char *junction_id_str);

    // OSI interface
    //

    /**
            Send OSI packages over UDP to specified IP address
    */
    SE_DLL_API int SE_OpenOSISocket(const char *ipaddr);

    /**
            Switch off logging to OSI file(s)
            @return 0 if successful, -1 if not
    */
    SE_DLL_API void SE_DisableOSIFile();

    /**
            Switch on logging to OSI file(s)
            @param filename Optional filename, including path. Set to 0 or "" to use default.
    */
    SE_DLL_API void SE_EnableOSIFile(const char *filename);

    /**
            Enforce flushing OSI file (save all buffered data to file)
            Normally not necessary, since data is flushed automatically at file closure
            @return 0
    */
    SE_DLL_API void SE_FlushOSIFile();
    /**
     *      The SE_CropGroundTruth will limit the area of the dynamic groundtruth data to a circle with the specified radius around the given object
     * id Using the method repeatedly with different object ids will crop the groundtruth data around all objects specified Setting the radius to 0
     * will remove the cropping
     *      @return 0
     */
    SE_DLL_API void SE_CropOSIDynamicGroundTruth(int id, double radius);

    /**
     *      Setting the OSI report mode of the static ground truth data. Default is applied if function not used.
     *      @param mode DEFAULT=Static data in API and log first frame only, API=Static data always in API but only logged first frame and
     * API_AND_LOG=Static data always in API and logged.
     *     @return 0
     */
    SE_DLL_API void SE_SetOSIStaticReportMode(SE_OSIStaticReportMode mode);

    /**
     *      Excluding ghost vehicle from dynamic ground truth (default is to include)
            @return 0
    */
    SE_DLL_API void SE_ExcludeGhostFromGroundTruth();

    /**
     *      The SE_SetOSIFrequency function sets the frequency of OSI data updates
     *      @param frequency Frequency of OSI data updates
            @return 0
     */
    SE_DLL_API int SE_SetOSIFrequency(int frequency);

    /**
            @return 0
    */
    SE_DLL_API int SE_UpdateOSITrafficCommand();

    /**
            The SE_GetOSIGroundTruth function updates the OSI ground truth and returns a char array containing the osi GroundTruth serialized to a
       string
            @return osi3::GroundTruth*
    */
    SE_DLL_API const char *SE_GetOSIGroundTruth(int *size);

    /**
            The SE_GetOSIGroundTruthRaw function updates the OSI ground truth and returns a pointer to the internal OSI data structure,
            useful for direct access to OSI data in a C/C++ environment
            @return osi3::GroundTruth*
    */
    SE_DLL_API const char *SE_GetOSIGroundTruthRaw();

    /**
            Get a pointer to the internal OSI data structure, useful for direct access to OSI data in a C/C++ environment
            @return osi3::TrafficCommand*
     */
    SE_DLL_API const char *SE_GetOSITrafficCommandRaw();

    /**
            Populate OSI SensorView from provided pointer to OSI SensorData
            @return 0
    */
    SE_DLL_API int SE_SetOSISensorDataRaw(const char *sensordata);

    /**
            Return a pointer to OSI SensorData information
            @return osi3::SensorData*
    */
    SE_DLL_API const char *SE_GetOSISensorDataRaw();

    /**
            The SE_GetOSIRoadLane function returns a char array containing the osi Lane information/message of the lane where the object with
            object_id is, serialized to a string
    */
    SE_DLL_API const char *SE_GetOSIRoadLane(int *size, int object_id);

    /**
            The SE_GetOSIRoadLane function returns a char array containing the osi Lane Boundary information/message with the specified GLOBAL id
    */
    SE_DLL_API const char *SE_GetOSILaneBoundary(int *size, int g_id);

    /**
            The SE_GetOSILaneBoundaryIds function the global ids for left, far left, right and far right lane boundaries
            @param object_id Handle to the object to which the sensor should be attached
            @param ids Reference to a struct which will be filled with the Ids
    */
    SE_DLL_API void SE_GetOSILaneBoundaryIds(int object_id, SE_LaneBoundaryId *ids);

    /**
            The SE_GetOSISensorDataRaw function returns a char array containing the OSI SensorData information
            @return osi3::SensorData*
    */
    SE_DLL_API const char *SE_GetOSISensorDataRaw();

    /**
            Set explicit OSI timestap
            Note that this timestamp does NOT affect esmini simulation time
            Also note that setting timestamp with this function will move into explicit time mode
            and from that point OSI timestamp is exclusively controlled by this function.
            @param nanoseconds Nano seconds (1e-9 s)
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_OSISetTimeStamp(unsigned long long nanoseconds);

    // End of OSI interface

    SE_DLL_API void SE_LogMessage(const char *message);

    SE_DLL_API void SE_CloseLogFile();

    // Viewer settings
    /**
            Switch on/off visualization of specified features
            @param featureType Type of the features, see roadgeom::NodeMask typedef
            @param enable Set true to show features, false to hide
    */
    SE_DLL_API void SE_ViewerShowFeature(int featureType, bool enable);

    // Simple vehicle
    /**
            Create an instance of a simplistic vehicle based on a 2D bicycle kincematic model
            @param x Initial position X world coordinate
            @param y Initial position Y world coordinate
            @param h Initial heading
            @param length Length of the vehicle
            @param speed Initial speed
            @return Handle to the created object
    */
    SE_DLL_API void *SE_SimpleVehicleCreate(float x, float y, float h, float length, float speed);

    /**
            Delete an instance of the simplistic vehicle model
    */
    SE_DLL_API void SE_SimpleVehicleDelete(void *handleSimpleVehicle);

    /**
            Control the speed and steering with discreet [-1, 0, 1] values, suitable for keyboard control (e.g. up/none/down).
            The function also steps the vehicle model, updating its position according to motion state and timestep.
            @param dt timesStep (s)
            @param throttle Longitudinal control, -1: brake, 0: none, +1: accelerate
            @param steering Lateral control, -1: left, 0: straight, 1: right
    */
    SE_DLL_API void SE_SimpleVehicleControlBinary(void  *handleSimpleVehicle,
                                                  double dt,
                                                  int    throttle,
                                                  int    steering);  // throttle and steering [-1, 0 or 1]

    /**
            Control the speed and steering with floating values in the range [-1, 1], suitable for driver models.
            The function also steps the vehicle model, updating its position according to motion state and timestep.
            @param dt timesStep (s)
            @param throttle Longitudinal control, -1: maximum brake, 0: no acceleration, +1: maximum acceleration
            @param steering Lateral control, -1: max left, 0: straight, 1: max right
    */
    SE_DLL_API void SE_SimpleVehicleControlAnalog(void  *handleSimpleVehicle,
                                                  double dt,
                                                  double throttle,
                                                  double steering);  // throttle and steering [-1, 0 or 1]

    /**
            Update vehicle state in terms of explicit acceleration and steering angle
            @param dt timesStep (s)
            @param acceleration Longitudinal acceleration
            @param steering_angle Lateral steering angle
    */
    SE_DLL_API void SE_SimpleVehicleControlAccAndSteer(void *handleSimpleVehicle, double dt, double acceleration, double steering_angle);

    /**
            Set speed, use together with control binary/analog with throttle set to zero
            @param speed Speed (m/s)
    */
    SE_DLL_API void SE_SimpleVehicleSetSpeed(void *handleSimpleVehicle, float speed);

    /**
            Deactivate or re-activate throttle/brake
            @param disabled True: throttle disable, False: throttle enable
    */
    SE_DLL_API void SE_SimpleVehicleSetThrottleDisabled(void *handleSimpleVehicle, bool disabled);

    /**
            Deactivate or re-activate steering
            @param disabled True: steering disable, False: steering enable
    */
    SE_DLL_API void SE_SimpleVehicleSetSteeringDisabled(void *handleSimpleVehicle, bool disabled);

    /**
            Set speed, use together with control binary/analog with throttle set to zero
            @param speed Speed (m/s)
    */
    SE_DLL_API void SE_SimpleVehicleSetSpeed(void *handleSimpleVehicle, float speed);

    /**
            Control the speed and steering by providing steering and speed targets
            The function also steps the vehicle model, updating its position according to motion state and timestep.
            @param dt timesStep (s)
            @param target_speed Requested speed
            @param heading_to_target Heading angle to a target position
    */
    SE_DLL_API void SE_SimpleVehicleControlTarget(void *handleSimpleVehicle, double dt, double target_speed, double heading_to_target);

    /**
            Set maximum vehicle speed.
            @param speed Maximum speed (km/h)
    */
    SE_DLL_API void SE_SimpleVehicleSetMaxSpeed(void *handleSimpleVehicle, float speed);

    /**
            Set maximum vehicle acceleration.
            @param speed Maximum acceleration (m/s^2)
    */
    SE_DLL_API void SE_SimpleVehicleSetMaxAcceleration(void *handleSimpleVehicle, float maxAcceleration);

    /**
            Set maximum vehicle deceleration.
            @param speed Maximum deceleration (m/s^2)
    */
    SE_DLL_API void SE_SimpleVehicleSetMaxDeceleration(void *handleSimpleVehicle, float maxDeceleration);

    /**
            Set engine brake factor, applied when no throttle is applied
            @param engineBrakeFactor recommended range = [0.0, 0.01], default = 0.001
    */
    SE_DLL_API void SE_SimpleVehicleSetEngineBrakeFactor(void *handleSimpleVehicle, float engineBrakeFactor);

    /**
            Set steering scale factor, which will limit the steering range as speed increases
            @param steeringScale recommended range = [0.0, 0.1], default = 0.018
    */
    SE_DLL_API void SE_SimpleVehicleSteeringScale(void *handleSimpleVehicle, float steeringScale);

    /**
            Set steering return factor, which will make the steering wheel strive to neutral position (0 angle)
            @param steeringScale recommended range = [0.0, 10], default = 4.0
    */
    SE_DLL_API void SE_SimpleVehicleSteeringReturnFactor(void *handleSimpleVehicle, float steeringReturnFactor);

    /**
            Set steering rate, which will affect the angular speed of which the steering wheel will turn
            @param steeringRate recommended range = [0.0, 50.0], default = 8.0
    */
    SE_DLL_API void SE_SimpleVehicleSteeringRate(void *handleSimpleVehicle, float steeringRate);

    /**
            Get current state of the vehicle. Typically called after Control has been applied.
            @param state Pointer/reference to a SE_SimpleVehicleState struct to be filled in
    */
    SE_DLL_API void SE_SimpleVehicleGetState(void *handleSimpleVehicle, SE_SimpleVehicleState *state);

    /**
    Capture rendered image to RAM for possible fetch via API, e.g. SE_FetchImage()
    Set true before calling SE_Init() to enable fetching first frame at time = 0
    @param state true=capture images, false=don't capture (default, might improve performance on some systems)
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SaveImagesToRAM(bool state);

    /**
    Capture rendered image to file. Call after SE_Init().
    @param nrOfFrames -1=continuously, 0=stop, >0=number of frames, e.g. 1=next frame only
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SaveImagesToFile(int nrOfFrames);

    /**
    Fetch captured image from RAM (internal memory)
    @param image Pointer/reference to a SE_Image which will be filled in, even image data pointer
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_FetchImage(SE_Image *image);

    /**
    Register a function and optional parameter (ref) to be called back from esmini after each frame (update of scenario)
    The current state of specified entity will be returned.
    Complete or part of the state can then be overridden by calling the SE_ReportObjectPos/SE_ReportObjectRoadPos functions.
    Registered callbacks will be cleared between SE_Init calls.
    @param fnPtr A pointer to the function to be invoked
    @param user_data Optional pointer to a local data object that will be passed as argument in the callback. Set 0/NULL if not needed.
    */
    SE_DLL_API void SE_RegisterImageCallback(void (*fnPtr)(SE_Image *, void *), void *user_data);

    /**
     Store RGB (3*8 bits color values) image data as a PPM image file
     PPM info: http://paulbourke.net/dataformats/ppm/
     @param filename File name including extension which should be ".ppm", e.g. "img0.ppm"
     @param width Width
     @param height Height
     @param rgbData Array of color values
     @param pixelSize Should be 3 (RGB/BGR)
     @param upsidedown false=lines stored from top to bottom, true=lines stored from bottom to top
     @return 0 if OK, -1 if failed to open file, -2 if unexpected pixelSize
    */
    SE_DLL_API int
    SE_WritePPMImage(const char *filename, int width, int height, const unsigned char *data, int pixelSize, int pixelFormat, bool upsidedown);

    /**
    Store RGB or BGR (3*8 bits color values) image data as a TGA image file
    TGA spec: https://www.dca.fee.unicamp.br/~martino/disciplinas/ea978/tgaffs.pdf
    TGA brief: http://paulbourke.net/dataformats/tga/
    @param filename File name including extension which should be ".ppm", e.g. "img0.ppm"
    @param width Width
    @param height Height
    @param rgbData Array of color values
    @param pixelSize Should be 3 (RGB/BGR)
    @param upsidedown false=lines stored from top to bottom, true=lines stored from bottom to top
    @return 0 if OK, -1 if failed to open file, -2 if unexpected pixelSize
    */
    SE_DLL_API int
    SE_WriteTGAImage(const char *filename, int width, int height, const unsigned char *data, int pixelSize, int pixelFormat, bool upsidedown);

    /**
    Add a camera with relative position and orientation (heading and pitch)
    @param x X coordinate relative vehicle currently in focus
    @param y Y coordinate relative vehicle currently in focus
    @param z Z coordinate relative vehicle currently in focus
    @param h Heading (yaw) (radians) relative vehicle currently in focus
    @param p Pitch (radians) relative vehicle currently in focus
    @return index of the camera, can be used for SE_SetCameraMode(), -1 on error
    */
    SE_DLL_API int SE_AddCustomCamera(double x, double y, double z, double h, double p);

    /**
    Add a fixed camera at custom global position and orientation (heading and pitch)
    @param x X coordinate
    @param y Y coordinate
    @param z Z coordinate
    @param h Heading (yaw) (radians)
    @param p P Pitch (radians)
    @return index of the camera, can be used for SE_SetCameraMode(), -1 on error
    */
    SE_DLL_API int SE_AddCustomFixedCamera(double x, double y, double z, double h, double p);

    /**
    Add a camera with relative position looking at current entity
    @param x X coordinate
    @param y Y coordinate
    @param z Z coordinate
    @return index of the camera, can be used for SE_SetCameraMode(), -1 on error
    */
    SE_DLL_API int SE_AddCustomAimingCamera(double x, double y, double z);

    /**
    Add a camera at fixed location but always looking at current entity
    @param x X coordinate
    @param y Y coordinate
    @param z Z coordinate
    @param fixed_pos Position is relative current vehicle (false) or fixed (true)
    @return index of the camera, can be used for SE_SetCameraMode(), -1 on error
    */
    SE_DLL_API int SE_AddCustomFixedAimingCamera(double x, double y, double z);

    /**
    Add a top view camera with fixed position and rotation
    @param x X coordinate
    @param y Y coordinate
    @param z Z coordinate
    @param rot Rotation (radians)
    @return index of the camera, can be used for SE_SetCameraMode(), -1 on error
    */
    SE_DLL_API int SE_AddCustomFixedTopCamera(double x, double y, double z, double rot);

    /**
    Select camera mode
    @param mode Camera mode as in RubberbandManipulator::CAMERA_MODE
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetCameraMode(int mode);

    /**
    Sets the camera focus to the specified object
    @param object_id The object to focus on
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetCameraObjectFocus(int object_id);

    /**
    Get the Id of the object the camera is focused on
    @return Id of the object, -1 on error e.g. scenario not initialized or viewer not enabled
    */
    SE_DLL_API int SE_GetObjectInCameraFocus();

    /**
    Get the position (x, y, z) and orientation (heading/yaw, pitch, roll) of the viewer camera
    @param x reference to a variable returning the camera x coordinate
    @param y reference to a variable returning the camera y coordinate
    @param z reference to a variable returning the camera z coordinate
    @param h reference to a variable returning the camera heading/yaw
    @param p reference to a variable returning the camera pitch
    @param r reference to a variable returning the camera roll
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetCameraPos(float *x, float *y, float *z, float *h, float *p, float *r);

    /**
            Get the number Route points assigned for a specific vehicle
            @param object_id The index of the vehicle
            @return number of Route points (0 means no route assigned), -1 on error
    */
    SE_DLL_API int SE_GetNumberOfRoutePoints(int object_id);

    /**
            Get a specific route point for a certain vehicle
            @param object_id The index of the vehicle
            @param route_index The index of Route point
            @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetRoutePoint(int object_id, unsigned int route_index, SE_RouteInfo *routeinfo);

    /**
        Get the total length of the route assigned to specified object
        @param object_id Id of the object
        @return Length (m) of route, 0.0 if no route is assigned
    */
    SE_DLL_API float SE_GetRouteTotalLength(int object_id);

    /**
            Inject a speed action
            @param action Struct including needed info for the action, see SE_SpeedActionStruct definition
    */
    SE_DLL_API void SE_InjectSpeedAction(SE_SpeedActionStruct *action);

    /**
            Inject a lane change action
            @param action Struct including needed info for the action, see SE_LaneChangeActionStruct definition
    */
    SE_DLL_API void SE_InjectLaneChangeAction(SE_LaneChangeActionStruct *action);

    /**
            Inject a lane offset action
            @param action Struct including needed info for the action, see SE_LaneOffsetActionStruct definition
    */
    SE_DLL_API void SE_InjectLaneOffsetAction(SE_LaneOffsetActionStruct *action);

    /**
            Check whether any injected action is ongoing
            @param action_type Type of action, see esmini Action.hpp::ActionType enum. Set to -1 to check for any action.
    */
    SE_DLL_API bool SE_InjectedActionOngoing(int action_type);

#ifdef __cplusplus
}
#endif
