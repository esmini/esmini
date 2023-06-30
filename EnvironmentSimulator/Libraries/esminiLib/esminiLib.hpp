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
    int   roadId;          // road ID
    int   junctionId;      // Junction ID (-1 if not in a junction)
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
} SE_ScenarioObjectState;

// asciidoc tag::SE_RoadInfo_struct[]
typedef struct
{
    float global_pos_x;   // target position, in global coordinate system
    float global_pos_y;   // target position, in global coordinate system
    float global_pos_z;   // target position, in global coordinate system
    float local_pos_x;    // target position, relative vehicle (pivot position object) coordinate system
    float local_pos_y;    // target position, relative vehicle (pivot position object) coordinate system
    float local_pos_z;    // target position, relative vehicle (pivot position object) coordinate system
    float angle;          // heading angle to target from and relative vehicle (pivot position object) coordinate system
    float road_heading;   // road heading at steering target point
    float road_pitch;     // road pitch (inclination) at steering target point
    float road_roll;      // road roll (camber) at target point
    float trail_heading;  // trail heading (only when used for trail lookups, else equals road_heading)
    float curvature;      // road curvature at steering target point
    float speed_limit;    // speed limit given by OpenDRIVE type entry
    int   roadId;         // target position, road ID
    int   junctionId;     // target position, junction ID (-1 if not in a junction)
    int   laneId;         // target position, lane ID
    float laneOffset;     // target position, lane offset (lateral distance from lane center)
    float s;              // target position, s (longitudinal distance along reference line)
    float t;              // target position, t (lateral distance from reference line)
} SE_RoadInfo;
// asciidoc end::SE_RoadInfo_struct[]

typedef struct
{
    float x;           // Route point in the global coordinate system
    float y;           // Route point in the global coordinate system
    float z;           // Route point in the global coordinate system
    int   roadId;      // Route point, road ID
    int   junctionId;  // Route point, junction ID (-1 if not in a junction)
    int   laneId;      // Route point, lane ID
    int   osiLaneId;   // Route point, osi lane ID
    float laneOffset;  // Route point, lane offset (lateral distance from lane center)
    float s;           // Route point, s (longitudinal distance along reference line)
    float t;           // Route point, t (lateral distance from reference line)
} SE_RouteInfo;

typedef struct
{
    int far_left_lb_id;
    int left_lb_id;
    int right_lb_id;
    int far_right_lb_id;
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
    bool active;  // True: override; false: stop overriding
    int  type;    // According to Entities::OverrideType
    int  number;
    int  value_type;  // According to Entities::OverrideGearType
                      // Manual type: Negative number are indicating reverse gears. Zero is neutral gear.
                      // Automatic type: (-1:Reverse, 0:Neutral, 1:Gear 1, 2:Gear 2, and so on.)
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
            @return -1 on error else number of permutations
    */
    SE_DLL_API int SE_GetNumberOfPermutations();

    /**
            Select parameter value permutation. Call BEFORE SE_Init, e.g. during or after preceding run.
            @return -1 on error else number of permutations
    */
    SE_DLL_API int SE_SelectPermutation(int index);

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
            @return number of parameters
    */
    SE_DLL_API int SE_GetNumberOfParameters();

    /**
            Get the name of a named parameter
            @param index The index of the parameter, range [0:numberOfParameters-1]
            @param Output parameter type 0=int, 1=double, 2=string (const char*), 3=bool, see OSCParameterDeclarations/ParameterType
            @return name if found, else 0
    */
    SE_DLL_API const char *SE_GetParameterName(int index, int *type);

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
            Specify if and how position object will align to the road. This version
            sets same mode for all components: Heading, Pitch, Roll and Z (elevation)
            @object_id Id of the object
            @param mode as defined by roadmanager::Position::ALIGN_MODE:
            0 = ALIGN_NONE // No alignment to road
            1 = ALIGN_SOFT // Align to road but add relative orientation
            2 = ALIGN_HARD // Completely align to road, disregard relative orientation
    */
    SE_DLL_API void SE_SetAlignMode(int object_id, int mode);

    /**
            Specify if and how position object will align to the road. This version
            sets same mode for only heading component.
            @object_id Id of the object
            @param mode as defined by roadmanager::Position::ALIGN_MODE:
            0 = ALIGN_NONE // No alignment to road
            1 = ALIGN_SOFT // Align to road but add relative orientation
            2 = ALIGN_HARD // Completely align to road, disregard relative orientation
    */
    SE_DLL_API void SE_SetAlignModeH(int object_id, int mode);

    /**
            Specify if and how position object will align to the road. This version
            sets same mode for only pitch component.
            @object_id Id of the object
            @param mode as defined by roadmanager::Position::ALIGN_MODE:
            0 = ALIGN_NONE // No alignment to road
            1 = ALIGN_SOFT // Align to road but add relative orientation
            2 = ALIGN_HARD // Completely align to road, disregard relative orientation
    */

    SE_DLL_API void SE_SetAlignModeP(int object_id, int mode);
    /**
            Specify if and how position object will align to the road. This version
            sets same mode for only roll component.
            @object_id Id of the object
            @param mode as defined by roadmanager::Position::ALIGN_MODE:
            0 = ALIGN_NONE // No alignment to road
            1 = ALIGN_SOFT // Align to road but add relative orientation
            2 = ALIGN_HARD // Completely align to road, disregard relative orientation
    */
    SE_DLL_API void SE_SetAlignModeR(int object_id, int mode);

    /**
            Specify if and how position object will align to the road. This version
            sets same mode for only Z (elevation) component.
            @object_id Id of the object
            @param mode as defined by roadmanager::Position::ALIGN_MODE:
            0 = ALIGN_NONE // No alignment to road
            1 = ALIGN_SOFT // Align to road but add relative orientation
            2 = ALIGN_HARD // Completely align to road, disregard relative orientation
    */
    SE_DLL_API void SE_SetAlignModeZ(int object_id, int mode);

    /**
            Add object
            Should be followed by one of the SE_Report functions to establish initial state.
            @param object_name Name of the object, preferably be unique
            @param object_type Type of the object. See Entities.hpp::Object::Type. Default=1 (VEHICLE).
            @param object_category Category of the object. Depends on type, see descendants of Entities.hpp::Object. Set to 0 if not known.
            @param object_role role of the object. Depends on type, See Entities.hpp::Object::Role. Set to 0 if not known.
            @param model_id Id of the 3D model to represent the object. See resources/model_ids.txt.
            @return Id [0..inf] of the added object successful, -1 on failure
    */
    SE_DLL_API int SE_AddObject(const char *object_name, int object_type, int object_category, int object_role, int model_id);

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
            @param z Z coordinate
            @param h Heading / yaw
            @param p Pitch
            @param r Roll
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_ReportObjectPos(int object_id, float timestamp, float x, float y, float z, float h, float p, float r);

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
    SE_DLL_API int SE_ReportObjectRoadPos(int object_id, float timestamp, int roadId, int laneId, float laneOffset, float s);

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
            examples: ANY_DRIVING = 1966082, ANY_ROAD = 1966214, ANY = -1
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
            Get the state of specified object's ghost (special purpose lead vehicle)
            @param object_id Id of the object to which the ghost is attached
            @param state Pointer/reference to a SE_ScenarioObjectState struct to be filled in
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetObjectGhostState(int object_id, SE_ScenarioObjectState *state);

    /**
            Get the number of collisions the specified object currently is involved in
            @param object_id Id of the object
            @return Number of objects that specified object currently is overlapping/colliding with. -1 if unsuccessful.
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
            Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle
            @param object_id Id of the object from which to measure (the actual externally controlled Ego vehicle, not ghost)
            @param lookahead_distance The distance, along the ghost trail, to the point from the current Ego vehicle location
            @param data Struct including all result values, see typedef for details
            @param speed_ghost reference to a variable returning the speed that the ghost had at this point along trail
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *data, float *speed_ghost);

    /**
            Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle
            @param object_id Id of the object from which to measure (the actual externally controlled Ego vehicle, not ghost)
            @param time Simulation time (subtracting headstart time, i.e. time=0 gives the initial state)
            @param data Struct including all result values, see typedef for details
            @param speed_ghost reference to a variable returning the speed that the ghost had at this point along trail
            @return 0 if successful, -1 if not
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
    The name of the respective StoryBoardElement, the type and the state will be returned.

     Values for the StoryBoardElement type
        STORY = 1,
        ACT = 2,
        MANEUVER_GROUP = 3,
        MANEUVER = 4,
        EVENT = 5,
        ACTION = 6,
        UNDEFINED_ELEMENT_TYPE = 0

     Values for the StoryBoardElement state
        STANDBY = 1,
        RUNNING = 2,
        COMPLETE = 3,
        UNDEFINED_ELEMENT_STATE = 0

    Registered callbacks will be cleared between SE_Init calls.
    @param fnPtr A pointer to the function to be invoked
    */
    SE_DLL_API void SE_RegisterStoryBoardElementStateChangeCallback(void (*fnPtr)(const char *name, int type, int state));

    /**
            Get the number of road signs along specified road
            @param road_id The road along which to look for signs
            @return Number of road signs
    */
    SE_DLL_API int SE_GetNumberOfRoadSigns(int road_id);

    /**
            Get information on specifed road sign
            @param road_id The road of which to look for the sign
            @param index Index of the sign. Note: not ID
            @param road_sign Pointer/reference to a SE_RoadSign struct to be filled in
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetRoadSign(int road_id, int index, SE_RoadSign *road_sign);

    /**
            Get the number of lane validity records of specified road object/sign
            @param road_id The road of which to look for the sign
            @param index Index of the sign. Note: not ID
            @return Number of validity records of specified road sign
    */
    SE_DLL_API int SE_GetNumberOfRoadSignValidityRecords(int road_id, int index);

    /**
            Get specified validity record of specifed road sign
            @param road_id The road of which to look for the sign
            @param signIndex Index of the sign. Note: not ID
            @param validityIndex Index of the validity record
            @param road_sign Pointer/reference to a SE_RoadObjValidity struct to be filled in
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_GetRoadSignValidityRecord(int road_id, int signIndex, int validityIndex, SE_RoadObjValidity *validity);

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
            @return 0
    */
    SE_DLL_API void SE_FlushOSIFile();

    /**
            The SE_ClearOSIGroundTruth clears the certain groundtruth data
            This function should only be used together with SE_UpdateOSIStaticGroundTruth and SE_UpdateOSIDynamicGroundTruth
            @return 0
    */
    SE_DLL_API int SE_ClearOSIGroundTruth();

    /**
            The SE_UpdateOSIGroundTruth function calls SE_UpdateOSIStaticGroundTruth and SE_UpdateOSIDynamicGroundTruth and updates OSI Groundtruth
            @return 0
    */
    SE_DLL_API int SE_UpdateOSIGroundTruth();

    /**
            The SE_UpdateOSIStaticGroundTruth function updates OSI static Groundtruth
            @return 0
    */
    SE_DLL_API int SE_UpdateOSIStaticGroundTruth();

    /**
            The SE_UpdateOSIDynamicGroundTruth function updates OSI dynamic Groundtruth
            @param reportGhost Optional flag, if we should include ghost vehicle info in the osi messages
            @return 0
    */
    SE_DLL_API int SE_UpdateOSIDynamicGroundTruth(bool reportGhost = true);

    /**
            The SE_GetOSIGroundTruth function returns a char array containing the osi GroundTruth serialized to a string
            @return osi3::GroundTruth*
    */
    SE_DLL_API const char *SE_GetOSIGroundTruth(int *size);

    /**
            The SE_GetOSIGroundTruthRaw function returns a char array containing the OSI GroundTruth information
            @return osi3::GroundTruth*
    */
    SE_DLL_API const char *SE_GetOSIGroundTruthRaw();

    /**
            The SE_SetOSISensorDataRaw function returns a char array containing the OSI GroundTruth information
            @return 0
    */
    SE_DLL_API int SE_SetOSISensorDataRaw(const char *sensordata);

    /**
            The SE_GetOSISensorDataRaw function returns a char array containing the OSI SensorData information
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
    SE_DLL_API const char *SE_GetOSILaneBoundary(int *size, int global_id);

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
            Create and open osi file
            @param filename Optional filename, including path. Set to 0 to use default.
            @return true=successful false=error
    */
    SE_DLL_API bool SE_OSIFileOpen(const char *filename);

    /**
            Create and open osi file
    */
    SE_DLL_API bool SE_OSIFileWrite(bool flush = false);

    /**
            Set explicit OSI timestap
            Note that this timestamp does NOT affect esmini simulation time
            Also note that setting timestamp with this function will move into explicit time mode
            and from that point OSI timestamp is exclusively controlled by this function.
            @param nanoseconds Nano seconds (1e-9 s)
            @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_OSISetTimeStamp(unsigned long long int nanoseconds);

    // End of OSI interface

    SE_DLL_API void SE_LogMessage(const char *message);

    // Viewer settings
    /**
            Switch on/off visualization of specified features
            @param featureType Type of the features, see viewer::NodeMask typedef
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
    Enable (default) or disable callback that handles framebuffer image capturing. NOTE: Needs to be called before SE_Init()
    @param state true (default) = enable off-screen rendering callback, false = disable off-screen rendering callback
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetOffScreenRendering(bool state);

    /**
    Capture rendered image to RAM for possible fetch via API, e.g. SE_FetchImage()
    @param state true=capture images, false=don't capture (default, might improve performance on some systems)
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SaveImagesToRAM(bool state);

    /**
    Capture rendered image to file
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
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_AddCustomCamera(double x, double y, double z, double h, double p);

    /**
    Add a fixed camera at custom global position and orientation (heading and pitch)
    @param x X coordinate
    @param y Y coordinate
    @param z Z coordinate
    @param h Heading (yaw) (radians)
    @param p P Pitch (radians)
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_AddCustomFixedCamera(double x, double y, double z, double h, double p);

    /**
    Add a camera with relative position looking at current entity
    @param x X coordinate
    @param y Y coordinate
    @param z Z coordinate
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_AddCustomAimingCamera(double x, double y, double z);

    /**
    Add a camera at fixed location but always looking at current entity
    @param x X coordinate
    @param y Y coordinate
    @param z Z coordinate
    @param fixed_pos Position is relative current vehicle (false) or fixed (true)
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_AddCustomFixedAimingCamera(double x, double y, double z);

    /**
    Add a top view camera with fixed position and rotation
    @param x X coordinate
    @param y Y coordinate
    @param z Z coordinate
    @param rot Rotation (radians)
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_AddCustomFixedTopCamera(double x, double y, double z, double rot);

    /**
    Select camera mode
    @param mode Camera mode as in RubberbandManipulator::CAMERA_MODE
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetCameraMode(int mode);

    /**
    Select camera mode
    @param object_id The object to focus on
    @return 0 if successful, -1 if not
    */
    SE_DLL_API int SE_SetCameraObjectFocus(int object_id);

    /**
            Get the number Route points assigned for a specific vehicle
            @param object_id The index of the vehicle
            @return number of Route points (0 means no route assigned)
    */
    SE_DLL_API int SE_GetNumberOfRoutePoints(int object_id);

    /**
            Get a specific route point for a certain vehicle
            @param object_id The index of the vehicle
            @param route_index The index of Route point
            @return 0 if successful, -1 if not (e.g. wrong type)
    */
    SE_DLL_API int SE_GetRoutePoint(int object_id, int route_index, SE_RouteInfo *routeinfo);
#ifdef __cplusplus
}
#endif
