
#pragma once

#ifdef WIN32
	#define UNITY_DLL_API __declspec(dllexport)
#else
	#define UNITY_DLL_API  // Leave empty on Mac
#endif

#define SE_NAME_SIZE 32

struct ScenarioObjectState
{
	int id;  
	char name[SE_NAME_SIZE];
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
};

#ifdef __cplusplus
extern "C"
{
#endif

	UNITY_DLL_API int SE_Init(const char *oscFilename);
	UNITY_DLL_API void SE_Step(float dt);
	UNITY_DLL_API void SE_Close();

	UNITY_DLL_API int SE_ReportObjectPos(int id, char *name, float timestamp, float x, float y, float z, float h, float p, float r, float speed);
	UNITY_DLL_API int SE_ReportObjectRoadPos(int id, char *name, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

	UNITY_DLL_API int SE_GetNumberOfObjects();
//	UNITY_DLL_API ScenarioObjectState SE_GetObjectState(int index);
	UNITY_DLL_API int SE_GetObjectState(int index, ScenarioObjectState &state);

#ifdef __cplusplus
}
#endif
