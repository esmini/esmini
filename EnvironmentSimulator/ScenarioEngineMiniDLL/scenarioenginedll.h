
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
	float speed;
};

#ifdef __cplusplus
extern "C"
#endif
{

	UNITY_DLL_API int SE_Init(const char *oscFilename);
	UNITY_DLL_API void SE_Step(float dt);
	UNITY_DLL_API void SE_Close();
	
	//	UNITY_DLL_API const char* SE_GetEnvModelFilename();  // don't know how to return string in a good way...

	UNITY_DLL_API int SE_ReportObjectPos(int id, char *name, float timestamp, float x, float y, float z, float h, float p, float r, float speed);
	UNITY_DLL_API int SE_ReportObjectRoadPos(int id, char *name, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);

	UNITY_DLL_API int SE_GetNumberOfObjects();
	UNITY_DLL_API float SE_GetObjectX(int index);
	UNITY_DLL_API float SE_GetObjectY(int index);
	UNITY_DLL_API float SE_GetObjectZ(int index);
	UNITY_DLL_API float SE_GetObjectH(int index);
	UNITY_DLL_API float SE_GetObjectP(int index);
	UNITY_DLL_API float SE_GetObjectR(int index);
	UNITY_DLL_API int SE_GetObjectRoadId(int index);
	UNITY_DLL_API int SE_GetObjectLaneId(int index);
	UNITY_DLL_API float SE_GetObjectLaneOffset(int index);
	UNITY_DLL_API float SE_GetObjectS(int index);

	// Todo
//	UNITY_DLL_API ScenarioObjectState SE_GetObjectState(int index);

#ifdef __cplusplus
}
#endif
