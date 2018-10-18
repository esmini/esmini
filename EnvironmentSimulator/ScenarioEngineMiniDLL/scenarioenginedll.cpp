#include "scenarioenginedll.h"
#include "ScenarioEngine.hpp"

#include "viewer.hpp"
#include "RubberbandManipulator.h"

#define EGO_ID 0	// need to match appearing order in the OpenSCENARIO file

typedef struct
{
	int id;
	viewer::CarModel *carModel;
	roadmanager::Position pos;
} ScenarioCar;

static std::vector<ScenarioCar> scenarioCar;

static ScenarioEngine *scenarioEngine = 0;
static ScenarioGateway *scenarioGateway = 0;
static roadmanager::OpenDrive *roadManager;
static viewer::Viewer *scViewer;

double simTime = 0;
double deltaSimTime = 0;  // external - used by Viewer::RubberBandCamera


ScenarioCar *getScenarioCarById(int id)
{
	for (auto &c : scenarioCar)
	{
		if (c.id == id)
		{
			return &c;
		}
	}

	return 0;
}

extern "C"
{
#ifdef _SCENARIO_VIEWER
	UNITY_DLL_API int SE_Init(const char *oscFilename, int useViewer)
#else
	UNITY_DLL_API int SE_Init(const char *oscFilename)
#endif
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

			// Fetch ScenarioGateway 
			roadManager = scenarioEngine->getRoadManager();

#ifdef _SCENARIO_VIEWER
			if (useViewer)
			{
				// Initialize the viewer
				char *argv[] = { "kalle", "--window", "50", "50", "1000", "500" };
				int argc = 6;
				scViewer = new viewer::Viewer(
					roadManager,
					scenarioEngine->getSceneGraphFilename().c_str(), osg::ArgumentParser(&argc, argv));
			}
#endif
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
			deltaSimTime = dt;
			simTime += dt;

			scenarioEngine->setSimulationTime(simTime);
			scenarioEngine->setTimeStep(dt);

			// ScenarioEngine
			scenarioEngine->step((double)dt);

#ifdef _SCENARIO_VIEWER
			// Fetch states of scenario objects
			for (int i = 0; i < scenarioGateway->getNumberOfObjects(); i++)
			{
				ObjectState *o = scenarioGateway->getObjectStatePtrByIdx(i);

				ScenarioCar *sc = getScenarioCarById(o->state_.id);

				// If not available, create it
				if (sc == 0)
				{
					ScenarioCar new_sc;

					std::cout << "Creating car " << o->state_.id << " - got state from gateway" << std::endl;

					new_sc.id = o->state_.id;

					// Choose model from index - wrap to handle more vehicles than models
					int carModelID = i % scViewer->carModels_.size();
					new_sc.carModel = scViewer->AddCar(carModelID);

					// Add it to the list of scenario cars
					scenarioCar.push_back(new_sc);

					sc = &scenarioCar.back();
				}
				sc->pos = o->state_.pos;
			}

			// Visualize scenario cars
			for (auto &sc : scenarioCar)
			{
				sc.carModel->SetPosition(sc.pos.GetX(), sc.pos.GetY(), sc.pos.GetZ());
				sc.carModel->SetRotation(sc.pos.GetH(), sc.pos.GetR(), sc.pos.GetP());
			}

			// Update graphics
			scViewer->osgViewer_->frame();

#endif
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

#if 0  // return state struct 
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
#else  // fill in state struct provided by reference argument
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

UNITY_DLL_API int SE_GetObjectStates(int &nObjects, ScenarioObjectState* state)
{
	int i;

	if (scenarioGateway)
	{
		for (i = 0; i < nObjects && i < scenarioGateway->getNumberOfObjects(); i++)
		{
			state[i].id = scenarioGateway->getObjectStatePtrByIdx(i)->state_.id;
			strncpy(state[i].name, scenarioGateway->getObjectStatePtrByIdx(i)->state_.name, NAME_LEN);
			state[i].timestamp = scenarioGateway->getObjectStatePtrByIdx(i)->state_.timeStamp;
			state[i].x = (float)scenarioGateway->getObjectStatePtrByIdx(i)->state_.pos.GetX();
			state[i].y = (float)scenarioGateway->getObjectStatePtrByIdx(i)->state_.pos.GetY();
			state[i].z = (float)scenarioGateway->getObjectStatePtrByIdx(i)->state_.pos.GetZ();
			state[i].h = (float)scenarioGateway->getObjectStatePtrByIdx(i)->state_.pos.GetH();
			state[i].p = (float)scenarioGateway->getObjectStatePtrByIdx(i)->state_.pos.GetP();
			state[i].r = (float)scenarioGateway->getObjectStatePtrByIdx(i)->state_.pos.GetR();
			state[i].speed = (float)scenarioGateway->getObjectStatePtrByIdx(i)->state_.speed;
		}
		nObjects = i;
	}
	else
	{
		nObjects = 0;
	}
	return 0;
}