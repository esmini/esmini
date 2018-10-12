#include "scenarioenginedll.h"
#include "ScenarioEngine.hpp"


static ScenarioEngine *scenarioEngine = 0;
static ScenarioGateway *scenarioGateway = 0;
static double simTime = 0;

extern "C"
{
	UNITY_DLL_API int SE_Init(const char *oscFilename)
	{
		simTime = 0; // Start time initialized to zero
		scenarioGateway = 0;

		if (scenarioEngine != 0)
		{
			delete scenarioEngine;
			scenarioEngine = 0;
		}
		
		// Create scenario engine
		try
		{
			// Create a scenario engine instance
			scenarioEngine = new ScenarioEngine(std::string(oscFilename), simTime);

			// Fetch ScenarioGateway
			scenarioGateway = scenarioEngine->getScenarioGateway();
		}
		catch (const std::exception& e)
		{
			std::cout << e.what() << std::endl;
			scenarioEngine = 0;
			scenarioGateway = 0;
			return -1;
		}

		// Step scenario engine - zero time - just to reach init state
		// Report all vehicles initially - to communicate initial position for external vehicles as well
		scenarioEngine->step(0.0, true); 

		return 0;
	}

	UNITY_DLL_API void SE_Close()
	{
		delete scenarioEngine;
		scenarioEngine = 0;
	}

	UNITY_DLL_API void SE_Step(float dt)
	{
		if (scenarioEngine != 0)
		{
			// Time operations
			simTime += dt;
			scenarioEngine->setSimulationTime(simTime);
			scenarioEngine->setTimeStep(dt);

			// ScenarioEngine
			scenarioEngine->step((double)dt);
		}
	}

	UNITY_DLL_API int SE_ReportObjectPos(int id, char *name, float timestamp, float x, float y, float z, float h, float p, float r, float speed)
	{
		scenarioGateway->reportObject(ObjectState(id, std::string(name), timestamp, x, y, z, h, p, r, speed));
		
		return 0;
	}

	UNITY_DLL_API int SE_ReportObjectRoadPos(int id, char * name, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed)
	{
		if (scenarioGateway != 0)
		{
			scenarioGateway->reportObject(ObjectState(id, std::string(name), timestamp, roadId, laneId, laneOffset, s, speed));
		}
		
		return 0;
	}

	UNITY_DLL_API int SE_GetNumberOfObjects()
	{
		if (scenarioGateway != 0)
		{
			return scenarioGateway->getNumberOfObjects();
		}
		else
		{
			return 0;
		}
	}

#if 0
	UNITY_DLL_API ScenarioObjectState SE_GetObjectState(int index)
	{
		ScenarioObjectState state;

		if (scenarioGateway)
		{
			state.id = scenarioGateway->getObjectStatePtrByIdx(index)->state_.id;
			strncpy(state.name, scenarioGateway->getObjectStatePtrByIdx(index)->state_.name, NAME_LEN);
			state.timestamp = scenarioGateway->getObjectStatePtrByIdx(index)->state_.timeStamp;
			state.x = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetX();
			state.y = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetY();
			state.z = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetZ();
			state.h = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetH();
			state.p = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetP();
			state.r = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetR();
			state.speed = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.speed;
		}

		return state;
	}
}
#else
	UNITY_DLL_API int SE_GetObjectState(int index, ScenarioObjectState &state)
	{

		if (scenarioGateway)
		{
			state.id = scenarioGateway->getObjectStatePtrByIdx(index)->state_.id;
			strncpy(state.name, scenarioGateway->getObjectStatePtrByIdx(index)->state_.name, NAME_LEN);
			state.timestamp = scenarioGateway->getObjectStatePtrByIdx(index)->state_.timeStamp;
			state.x = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetX();
			state.y = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetY();
			state.z = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetZ();
			state.h = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetH();
			state.p = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetP();
			state.r = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.pos.GetR();
			state.speed = (float)scenarioGateway->getObjectStatePtrByIdx(index)->state_.speed;
		}

		return 0;
	}
}
#endif