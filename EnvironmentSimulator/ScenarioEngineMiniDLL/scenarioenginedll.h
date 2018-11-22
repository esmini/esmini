
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

#ifdef _SCENARIO_VIEWER
	SE_DLL_API int SE_Init(const char *oscFilename, int useViewer);
#else
	SE_DLL_API int SE_Init(const char *oscFilename);
#endif
	SE_DLL_API void SE_Step(float dt);
	SE_DLL_API void SE_Close();

	SE_DLL_API int SE_ReportObjectPos(int id, char *name, float timestamp, float x, float y, float z, float h, float p, float r, float speed);
	SE_DLL_API int SE_ReportObjectRoadPos(int id, char *name, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

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
