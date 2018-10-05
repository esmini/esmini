
#pragma once

#ifdef WIN32
	#define UNITY_DLL_API __declspec(dllexport)
#else
	#define UNITY_DLL_API  // Leave empty on Mac
#endif

#define SE_NAME_SIZE 32

extern "C"
{
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

	int UNITY_DLL_API SE_Init(const char *oscFilename);
	void UNITY_DLL_API SE_Close();
	void UNITY_DLL_API SE_Step(float dt);
	int UNITY_DLL_API SE_ReportObjectPos(int id, char *name, float timestamp, float x, float y, float z, float h, float p, float r, float speed);
	int UNITY_DLL_API SE_ReportObjectRoadPos(int id, char *name, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed);
	int UNITY_DLL_API SE_GetNumberOfObjects();
	float UNITY_DLL_API SE_GetObjectX(int index);
	float UNITY_DLL_API SE_GetObjectY(int index);
	float UNITY_DLL_API SE_GetObjectZ(int index);
	float UNITY_DLL_API SE_GetObjectH(int index);
	float UNITY_DLL_API SE_GetObjectP(int index);
	float UNITY_DLL_API SE_GetObjectR(int index);
	ScenarioObjectState UNITY_DLL_API SE_GetObjectState(int index);
}
