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

	UNITY_DLL_API float SE_GetObjectX(int index)
	{
		if (scenarioGateway)
		{
			return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getPosX();
		}
		else
		{
			return 0.0f;
		}
	}

	UNITY_DLL_API float SE_GetObjectY(int index)
	{
		if (scenarioGateway)
		{
			return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getPosY();
		}
		else
		{
			return 0.0f;
		}
	}

	UNITY_DLL_API float SE_GetObjectZ(int index)
	{
		if (scenarioGateway)
		{
			return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getPosZ();
		}
		else
		{
			return 0.0f;
		}
	}

	UNITY_DLL_API float SE_GetObjectH(int index)
	{
		if (scenarioGateway)
		{
			return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getRotH();
		}
		else
		{
			return 0.0f;
		}
	}

	UNITY_DLL_API float SE_GetObjectP(int index)
	{
		if (scenarioGateway)
		{
			return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getRotP();
		}
		else
		{
			return 0.0f;
		}
	}

	UNITY_DLL_API float SE_GetObjectR(int index)
	{
		if (scenarioGateway)
		{
			return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getRotR();
		}
		else
		{
			return 0.0f;
		}
	}

	UNITY_DLL_API int SE_GetObjectRoadId(int index)
	{
		if (scenarioGateway)
		{
			return (int)scenarioGateway->getObjectStatePtrByIdx(index)->getRoadId();
		}
		else
		{
			return -1;
		}
	}

	UNITY_DLL_API int SE_GetObjectLaneId(int index)
	{
		if (scenarioGateway)
		{
			return (int)scenarioGateway->getObjectStatePtrByIdx(index)->getLaneId();
		}
		else
		{
			return -1;
		}
	}

	UNITY_DLL_API float SE_GetObjectLaneOffset(int index)
	{
		if (scenarioGateway)
		{
			return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getLaneOffset();
		}
		else
		{
			return 0.0f;
		}
	}

	UNITY_DLL_API float SE_GetObjectS(int index)
	{
		if (scenarioGateway)
		{
			return (float)scenarioGateway->getObjectStatePtrByIdx(index)->getS();
		}
		else
		{
			return 0.0f;
		}
	}


	// Not done/tested
	UNITY_DLL_API ScenarioObjectState SE_GetObjectState(int index)
	{
		struct ScenarioObjectState objStateReturn;

		if (scenarioGateway && index < scenarioGateway->getNumberOfObjects())
		{
			memset(&objStateReturn, 0, sizeof(ObjectStateStruct));

			ObjectState *o = scenarioGateway->getObjectStatePtrByIdx(index);
			if (o != 0)
			{
				objStateReturn.id = o->getId();

				// _CRT_SECURE_NO_WARNINGS
				strncpy(objStateReturn.name, o->getName().c_str(), SE_NAME_SIZE);
				objStateReturn.timestamp = (float)o->getTimeStamp();
				objStateReturn.x = (float)o->getPosX();
				objStateReturn.y = (float)o->getPosY();
				objStateReturn.z = (float)o->getPosZ();
				objStateReturn.h = (float)o->getRotH();
				objStateReturn.p = (float)o->getRotP();
				objStateReturn.r = (float)o->getRotR();
				objStateReturn.speed = (float)o->convertVelocityToSpeed(o->getVelX(), o->getVelY(), o->getRotH());
			}

		}

		return objStateReturn;
	}
}