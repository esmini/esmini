
#pragma once

#ifdef WIN32
	#define SE_DLL_API __declspec(dllexport)
#else
	#define SE_DLL_API  // Leave empty on Mac
#endif

#define SE_NAME_SIZE 32

typedef struct
{
	int id;  
	int model_id;
	//	char name[SE_NAME_SIZE];
	float timestamp;
	float x;
	float y;
	float z;
	float h;
	float p;
	float r;
	int roadId;
	int laneId;
	float laneOffset;
	float s;
	float speed;
} ScenarioObjectState;


#ifdef __cplusplus
extern "C"
{
#endif
	/**
	Initialize the scenario engine
	@param oscFilename Path to the OpenSCEANRIO file
	@param ext_control Ego control 0=by OSC 1=No 2=Yes
	@param use_viewer 0=no viewer, 1=use viewer
	@param record Create recording for later playback
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_Init(const char *oscFilename, int ext_control = 0, int use_viewer = 0, int record = 0);

	SE_DLL_API int SE_Step(float dt);
	SE_DLL_API void SE_Close();

	SE_DLL_API int SE_ReportObjectPos(int id, char *name, int model_id, float timestamp, float x, float y, float z, float h, float p, float r, float speed);
	SE_DLL_API int SE_ReportObjectRoadPos(int id, char *name, int model_id, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

	SE_DLL_API int SE_GetNumberOfObjects();
//	SE_DLL_API ScenarioObjectState SE_GetObjectState(int index);
	SE_DLL_API int SE_GetObjectState(int index, ScenarioObjectState *state);
	SE_DLL_API int SE_GetObjectStates(int *nObjects, ScenarioObjectState* state);

	// Road related functions
	/**
	Get the location, in global coordinate system, of the point at a specified distance along the road ahead
	@param object_id The ID of the vehicle to measure from
	@param lookahead_distance The distance, along the road, to the point
	@param target_pos Array to fill in calculated X, Y and Z coordinate values
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetSteeringTargetPosGlobal(int object_id, float lookahead_distance, float *target_pos);

	/**
	Get the location, in vehicle local coordinate system, of the point at a specified distance along the road ahead
	@param object_id The ID of the vehicle to measure from
	@param lookahead_distance The distance, along the road, to the point
	@param target_pos Array to fill in calculated X, Y and Z coordinate values
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetSteeringTargetPosLocal(int object_id, float lookahead_distance, float *target_pos);

	/**
	Get the heading angle, in vehicle local coordinate system, to the point at a specified distance along the road ahead
	@param object_id The ID of the vehicle to measure from
	@param lookahead_distance The distance, along the road, to the point
	@param angle Pointer to variable where target angle will be written
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetSteeringTargetAngle(int object_id, float lookahead_distance, float *angle);

	/**
	Get the curvature at the point at a specified distance along the road ahead
	@param object_id The ID of the vehicle to measure from
	@param lookahead_distance The distance, along the road, to the point
	@param curvature Pointer to variable where target curvature will be written
	@return 0 if successful, -1 if not
	*/
	SE_DLL_API int SE_GetSteeringTargetCurvature(int object_id, float lookahead_distance, float *curvature);

#ifdef __cplusplus
}
#endif
