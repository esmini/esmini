
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
	SE_DLL_API int GetSteeringTargetPos(int object_id, float lookahead_distance, double *target_pos);

#ifdef __cplusplus
}
#endif
