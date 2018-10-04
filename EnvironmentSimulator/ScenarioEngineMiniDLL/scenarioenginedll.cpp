#include "scenarioenginedll.h"
#include "ScenarioEngine.hpp"


static ScenarioEngine *scenarioEngine;
static ScenarioGateway *scenarioGateway;
static double simTime;

extern "C"
{
	int UNITY_DLL_API SE_Init(const char *oscFilename)
	{
		simTime = 0; // Start time initialized to zero

		if (scenarioEngine != 0)
		{
			delete scenarioEngine;
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
			return -1;
		}

		return 0;
	}

	void UNITY_DLL_API SE_Close()
	{
		delete scenarioEngine;
	}

	void UNITY_DLL_API SE_Step(float dt)
	{
		// Time operations
		simTime += dt;
		scenarioEngine->setSimulationTime(simTime);
		scenarioEngine->setTimeStep(dt);

		// ScenarioEngine
		scenarioEngine->step((double)dt);
	}

	int UNITY_DLL_API SE_ReportObjectPos(int id, char *name, float timestamp, float x, float y, float z, float h, float p, float r, float speed)
	{
		scenarioGateway->reportObject(ObjectState(id, name, timestamp, x, y, z, h, p, r, speed));
		
		return 0;
	}

	int UNITY_DLL_API SE_ReportObjectRoadPos(int id, char * name, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed)
	{
		scenarioGateway->reportObject(ObjectState(id, name, timestamp, roadId, laneId, laneOffset, s, speed));
		
		return 0;
	}

	int UNITY_DLL_API SE_GetNumberOfObjects()
	{
		return scenarioGateway->getNumberOfObjects();
	}

	float UNITY_DLL_API SE_GetObjectX(int index)
	{
		return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getPosX();
	}

	float UNITY_DLL_API SE_GetObjectY(int index)
	{
		return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getPosY();
	}

	float UNITY_DLL_API SE_GetObjectZ(int index)
	{
		return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getPosZ();
	}

	float UNITY_DLL_API SE_GetObjectH(int index)
	{
		return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getRotH();
	}

	float UNITY_DLL_API SE_GetObjectP(int index)
	{
		return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getRotP();
	}

	float UNITY_DLL_API SE_GetObjectR(int index)
	{
		return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getRotR();
	}

	ScenarioObjectState UNITY_DLL_API SE_GetObjectState(int index)
	{
		struct ScenarioObjectState objStateReturn;

		memset(&objStateReturn, 0, sizeof(ObjectStateStruct));

		ObjectState *o = scenarioGateway->getObjectStatePtrByIdx(index);
		if (o != 0)
		{
			objStateReturn.id = o->getId();
			strncpy_s(objStateReturn.name, o->getName().c_str(), SE_NAME_SIZE);
			objStateReturn.timestamp = (float)o->getTimeStamp();
			objStateReturn.x = (float)o->getPosX();
			objStateReturn.y = (float)o->getPosY();
			objStateReturn.z = (float)o->getPosZ();
			objStateReturn.h = (float)o->getRotH();
			objStateReturn.p = (float)o->getRotP();
			objStateReturn.r = (float)o->getRotR();
			objStateReturn.speed = (float)o->convertVelocityToSpeed(o->getVelX(), o->getVelY(), o->getRotH());
		}
		return objStateReturn;
	}
}