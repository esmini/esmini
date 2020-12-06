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
	int id;					  // Automatically generated unique object id
	int model_id;             // Id to control what 3D model to represent the vehicle - see carModelsFiles_[] in scenarioenginedll.cpp
	int ctrl_type;            // 0: DefaultController 1: External. Further values see Controller::Type enum
	float timestamp;
	float x;
	float y;
	float z;
	float h;
	float p;
	float r;
	int roadId;
	float t;
	int laneId;
	float laneOffset;
	float s;
	float speed;
	float centerOffsetX;
	float centerOffsetY;
	float centerOffsetZ;
	float width;
	float length;
	float height;
} SE_ScenarioObjectState;

typedef struct
{
	float global_pos_x;     // target position, in global coordinate system
	float global_pos_y;     // target position, in global coordinate system
	float global_pos_z;     // target position, in global coordinate system
	float local_pos_x;      // target position, relative vehicle (pivot position object) coordinate system
	float local_pos_y;      // target position, relative vehicle (pivot position object) coordinate system
	float local_pos_z;      // target position, relative vehicle (pivot position object) coordinate system
	float angle;			// heading angle to target from and relatove to vehicle (pivot position)
	float road_heading;		// road heading at steering target point
	float road_pitch;		// road pitch (inclination) at steering target point
	float road_roll;		// road roll (camber) at target point
	float trail_heading;	// trail heading (only when used for trail lookups, else equals road_heading)
	float curvature;		// road curvature at steering target point
	float speed_limit;		// speed limit given by OpenDRIVE type entry
} SE_RoadInfo;

typedef struct
{
	int far_left_lb_id;
	int left_lb_id;
	int right_lb_id;
	int far_right_lb_id;
} SE_LaneBoundaryId;

typedef struct
{
	float x;
	float y;
	float z;
	float h;
	float p;
	float speed;
} SE_SimpleVehicleState;

typedef struct
{
	const char* name;  // Name of the parameter as defined in the OpenSCENARIO file
	void* value;  // Pointer to value which can be an integer, double or string (const char*) as defined in the OpenSCENARIO file
} SE_Parameter;

#ifdef __cplusplus
extern "C"
{
#endif

	// Basic interface
	// 
	
	/**
	Add a search path for OpenDRIVE and 3D model files
	@param path Path to a directory
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_AddPath(const char* path);

	/**
	Clear all search paths for OpenDRIVE and 3D model files
	*/
	SE_DLL_API void SE_ClearPaths();

	/**
	Initialize the scenario engine
	@param oscFilename Path to the OpenSCEANRIO file
	@param disable_ctrls 1=Any controller will be disabled 0=Controllers applied according to OSC file
	@param use_viewer 0=no viewer, 1=use viewer
	@param threads 0=single thread, 1=viewer in a separate thread, parallel to scenario engine
	@param record Create recording for later playback 0=no recording 1=recording
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_Init(const char *oscFilename, int disable_ctrls, int use_viewer, int threads, int record);

	/**
	Initialize the scenario engine
	@param oscFilename Path to the OpenSCEANRIO file
	@param argc Number of arguments
	@param argv Arguments
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_InitWithArgs(int argc, char* argv[]);

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
	Get simulation time in seconds
	*/
	SE_DLL_API float SE_GetSimulationTime();  // Get simulation time in seconds

	/**
	Get the bool value of the end of the scenario
	*/
	SE_DLL_API int SE_GetQuitFlag();

	/**
	Get name of currently referred and loaded OpenDRIVE file 
	@return filename as string (const, since it's allocated and handled by esmini)
	*/
	SE_DLL_API const char* SE_GetODRFilename();

	/**
	Get name of currently referred and loaded SceneGraph file 
	@return filename as string (const, since it's allocated and handled by esmini)
	*/
	SE_DLL_API const char* SE_GetSceneGraphFilename();

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
	SE_DLL_API int SE_GetParameter(SE_Parameter* parameter);

	/**
	Report object position in cartesian coordinates
	@param id Id of the object
	@param timestamp Timestamp (not really used yet, OK to set 0)
	@param x X coordinate
	@param y Y coordinate
	@param z Z coordinate
	@param h Heading / yaw
	@param p Pitch
	@param r Roll
	@param speed Speed in forward direction of the enitity
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_ReportObjectPos(int id, float timestamp, float x, float y, float z, float h, float p, float r, float speed);

	/**
	Report object position in road coordinates
	@param id Id of the object
	@param timestamp Timestamp (not really used yet, OK to set 0)
	@param roadId Id of the road 
	@param laneId Id of the lane 
	@param laneOffset Lateral offset from center of specified lane
	@param s Longitudinal distance of the position along the specified road
	@param speed Speed in forward direction (s axis) of the enitity
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_ReportObjectRoadPos(int id, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

	/**
	Report object longitudinal speed. Useful for an external longitudinal controller.
	@param id Id of the object
	@param speed Speed in forward direction of the enitity
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_ReportObjectSpeed(int id, float speed);

	/**
	Report object lateral position relative road centerline. Useful for an external lateral controller.
	@param id Id of the object
	@param t Lateral position
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_ReportObjectLateralPosition(int id, float t);

	/**
	Report object lateral position by lane id and lane offset. Useful for an external lateral controller.
	@param id Id of the object
	@param laneId Id of the lane
	@param laneOffset Lateral offset from center of specified lane
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_ReportObjectLateralLanePosition(int id, int laneId, float laneOffset);

	/**
	Get the number of entities in the current scenario
	@return Number of entities
	*/
	SE_DLL_API int SE_GetNumberOfObjects();

	/**
	Get the state of specified object
	@param index Index of the object. Note: not ID
	@param state Pointer/reference to a SE_ScenarioObjectState struct to be filled in
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetObjectState(int index, SE_ScenarioObjectState* state);

	/**
	Check whether an object has a ghost (special purpose lead vehicle) 
	@param index Index of the object. Note: not ID
	@return 1 if ghost, 0 if not
	*/
	SE_DLL_API int SE_ObjectHasGhost(int index);

	/**
	Get the state of specified object's ghost (special purpose lead vehicle)
	@param index Index of the object. Note: not ID
	@param state Pointer/reference to a SE_ScenarioObjectState struct to be filled in
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetObjectGhostState(int index, SE_ScenarioObjectState* state);

	/**
	Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
	@param object_id Handle to the position object from which to measure
	@param lookahead_distance The distance, along the road, to the point
	@param data Struct including all result values, see typedef for details
	@param lookAheadMode Measurement strategy: Along 0=lane center, 1=road center (ref line) or 2=current lane offset. See roadmanager::Position::LookAheadMode enum
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo* data, int lookAheadMode);

	/**
	Get information suitable for driver modeling of a ghost vehicle driving ahead of the ego vehicle
	@param object_id Handle to the position object from which to measure (the actual externally controlled Ego vehicle, not ghost)
	@param lookahead_distance The distance, along the ghost trail, to the point from the current Ego vehicle location
	@param data Struct including all result values, see typedef for details
	@param speed_ghost reference to a variable returning the speed that the ghost had at this point along trail
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo* data, float* speed_ghost);

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
	Fetch list of identified objects from a sensor
	@param sensor_id Handle (index) to the sensor
	@param list Array of object indices
	@return Number of identified objects, i.e. length of list. -1 if unsuccesful.
	*/
	SE_DLL_API int SE_FetchSensorObjectList(int sensor_id, int* list);

	/**
	Register a function and optional parameter (ref) to be called back from esmini after each frame (update of scenario)
	The current state of specified entity will be returned.
	Complete or part of the state can then be overridden by calling the SE_ReportObjectPos/SE_ReportObjectRoadPos functions.
	@param object_id Handle to the position object (entity)
	@param SE_ScenarioObjectState A pointer to the function to be invoked
	@param user_data Optional pointer to a local data object that will be passed as argument in the callback. Set 0/NULL if not needed.
	*/
	SE_DLL_API void SE_RegisterObjectCallback(int object_id, void (*fnPtr)(SE_ScenarioObjectState*, void*), void* user_data);


	// OSI interface
	// 

	/**
	Send OSI packages over UDP to specified IP address
	*/
	SE_DLL_API int SE_OpenOSISocket(char *ipaddr);

	/**
	Switch off logging to OSI file(s)
	*/
	SE_DLL_API void SE_DisableOSIFile();

	/**
	Switch on logging to OSI file(s)
	*/
	SE_DLL_API void SE_EnableOSIFile();

	/**
	The SE_UpdateOSIGroundTruth function returns a char array containing the osi GroundTruth serialized to a string
	*/
	SE_DLL_API int SE_UpdateOSIGroundTruth();

	/**
	The SE_GetOSIGroundTruth function returns a char array containing the osi GroundTruth serialized to a string
	*/
	SE_DLL_API const char* SE_GetOSIGroundTruth(int* size);

	/**
	The SE_GetOSIGroundTruthRaw function returns a char array containing the OSI GroundTruth information
	@return osi3::GroundTruth*
	*/
	SE_DLL_API const char* SE_GetOSIGroundTruthRaw();

	/**
	The SE_GetOSIRoadLane function returns a char array containing the osi Lane information/message of the lane where the object with object_id is, serialized to a string
	*/
	SE_DLL_API const char* SE_GetOSIRoadLane(int* size, int object_id);
	
	/**
	The SE_GetOSIRoadLane function returns a char array containing the osi Lane Boundary information/message with the specified GLOBAL id
	*/
	SE_DLL_API const char* SE_GetOSILaneBoundary(int* size, int global_id);
	
	/**
	The SE_GetOSILaneBoundaryIds function the global ids for left, far elft, right and far right lane boundaries
	@param object_id Handle to the object to which the sensor should be attached
	@param ids Reference to a struct which will be filled with the Ids
	*/
	SE_DLL_API void SE_GetOSILaneBoundaryIds(int object_id, SE_LaneBoundaryId* ids);

	/**
	The SE_GetOSISensorDataRaw function returns a char array containing the OSI SensorData information
	@return osi3::SensorData*
	*/
	SE_DLL_API const char* SE_GetOSISensorDataRaw();

	/**
	Create and open osi file
	*/
	SE_DLL_API bool SE_OSIFileOpen();

	/**
	Create and open osi file
	*/
	SE_DLL_API bool SE_OSIFileWrite(bool flush = false);

	SE_DLL_API void SE_LogMessage(char* message);

	// Viewer settings
	/**
	The SE_GetOSILaneBoundaryIds function the global ids for left, far elft, right and far right lane boundaries
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
	@return Handle to the created object
	*/
	SE_DLL_API void* SE_SimpleVehicleCreate(float x, float y, float h, float length);

	/**
	Delete an instance of the simplistic vehicle model
	*/
	SE_DLL_API void SE_SimpleVehicleDelete(void* handleSimpleVehicle);

	/**
	Control the speed and steering with discreet [-1, 0, 1] values, suitable for keyboard control (e.g. up/none/down).
	The function also steps the vehicle model, updating its position according to motion state and timestep.
	@param dt timesStep (s)
	@param throttle Longitudinal control, -1: brake, 0: none, +1: accelerate
	@param steering Lateral control, -1: left, 0: straight, 1: right
	*/
	SE_DLL_API void SE_SimpleVehicleControlBinary(void* handleSimpleVehicle, double dt, int throttle, int steering);  // throttle and steering [-1, 0 or 1]

	/**
	Control the speed and steering with floaing values in the range [-1, 1], suitable for driver models.
	The function also steps the vehicle model, updating its position according to motion state and timestep.
	@param dt timesStep (s)
	@param throttle Longitudinal control, -1: maximum brake, 0: no acceleration, +1: maximum acceleration
	@param steering Lateral control, -1: max left, 0: straight, 1: max right
	*/
	SE_DLL_API void SE_SimpleVehicleControlAnalog(void* handleSimpleVehicle, double dt, double throttle, double steering);  // throttle and steering [-1, 0 or 1]

	/**
	Set maximum vehicle speed.
	@param speed Maximum speed (m/s)
	*/
	SE_DLL_API void SE_SimpleVehicleSetMaxSpeed(void* handleSimpleVehicle, float speed);

	/**
	Get current state of the vehicle. Typically called after Control has been applied.
	@param state Pointer/reference to a SE_SimpleVehicleState struct to be filled in
	*/
	SE_DLL_API void SE_SimpleVehicleGetState(void* handleSimpleVehicle, SE_SimpleVehicleState* state);
	
#ifdef __cplusplus
}
#endif
