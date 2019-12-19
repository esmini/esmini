/* 
 * esmini - Environment Simulator Minimalistic 
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "scenarioenginedll.hpp"
#include "ScenarioEngine.hpp"
#include "IdealSensor.hpp"

using namespace scenarioengine;

#ifdef _SCENARIO_VIEWER

#include "viewer.hpp"
#include "RubberbandManipulator.hpp"

#define VISUALIZE_DRIVER_MODEL_TARGET 1
#define EGO_ID 0	// need to match appearing order in the OpenSCENARIO file

typedef struct
{
	int id;
	viewer::CarModel *carModel;
	ObjectStateStruct state;

	double se_ghost_pos[3];
	bool flag_received_ghost_pos;
	double se_steering_target_pos[3];
	bool flag_received_steering_target_pos;
} ScenarioCar;


static std::vector<ScenarioCar> scenarioCar;
static viewer::Viewer *scViewer = 0;

static std::vector<viewer::SensorViewFrustum*> sensorFrustum;

static bool closing = false;
static SE_Thread thread;
static SE_Mutex mutex;

static int COLOR_GREEN[3] = { 0x40, 0xA0, 0x50 };
static int COLOR_DARK_GRAY[3] = { 0x80, 0x80, 0x80 };
static int COLOR_GRAY[3] = { 0xBB, 0xBB, 0xBB };
static int COLOR_YELLOW[3] = { 0x90, 0x80, 0x50 };
static int COLOR_RED[3] = { 0x90, 0x30, 0x30 };

typedef enum {
	VIEWER_NOT_RUNNING,
	VIEWER_STARTING,
	VIEWER_RUNNING,
} ViewerState;

static ViewerState viewer_state = ViewerState::VIEWER_NOT_RUNNING;

#endif

#define DEFAULT_RECORDING_FILENAME "scenario.dat"

// Car models used for populating the road network according to scenario object model ID
// path should be relative the OpenDRIVE file
static const char* carModelsFiles_[] =
{
	"../models/car_white.osgb",
	"../models/car_blue.osgb",
	"../models/car_red.osgb",
	"../models/car_yellow.osgb",
	"../models/truck_yellow.osgb",
	"../models/van_red.osgb",
	"../models/bus_blue.osgb",
	"../models/s90.osgb",
	"../models/xc90.osgb",
	"../models/car_white.osgb",
};

static ScenarioEngine *scenarioEngine = 0;
static ScenarioGateway *scenarioGateway = 0;
static roadmanager::OpenDrive *roadManager = 0;
double simTime = 0;
double deltaSimTime = 0;  // external - used by Viewer::RubberBandCamera
static char *args[] = { "kalle", "--window", "50", "50", "1000", "500" };
static std::vector<ObjectSensor*> sensor;


#ifdef _SCENARIO_VIEWER


static void Set_se_steering_target_pos(int id, float x, float y, float z)
{
	if (!(viewer_state == ViewerState::VIEWER_RUNNING))
	{
		return;
	}
	
	scenarioCar[id].se_steering_target_pos[0] = x;
	scenarioCar[id].se_steering_target_pos[1] = y;
	scenarioCar[id].se_steering_target_pos[2] = z;

	scenarioCar[id].flag_received_steering_target_pos = true;
}

static void Set_se_ghost_pos(int id, float x, float y, float z)
{
	if (!(viewer_state == ViewerState::VIEWER_RUNNING))
	{
		return;
	}

	scenarioCar[id].se_ghost_pos[0] = x;
	scenarioCar[id].se_ghost_pos[1] = y;
	scenarioCar[id].se_ghost_pos[2] = z;

	scenarioCar[id].flag_received_ghost_pos = true;
}

ScenarioCar *getScenarioCarById(int id)
{
	for (size_t i = 0; i < scenarioCar.size(); i++)
	{
		if (scenarioCar[i].id == id)
		{
			return &scenarioCar[i];
		}
	}

	return 0;
}


void viewer_thread(void*)
{
	// For some reason can't use args array directly... copy to a true char**
	int argc = sizeof(args) / sizeof(char*);
	char **argv;
	argv = (char**)malloc(argc * sizeof(char*));
	for (int i = 0; i < argc; i++)
	{
		argv[i] = (char*)malloc((strlen(args[i]) + 1) * sizeof(char));
		strcpy(argv[i], args[i]);
	}

	// Initialize the viewer
	scViewer = new viewer::Viewer(roadManager, scenarioEngine->getSceneGraphFilename().c_str(), scenarioEngine->getScenarioFilename().c_str(), osg::ArgumentParser(&argc, argv), VISUALIZE_DRIVER_MODEL_TARGET);

	// Update graphics - until close request or viewer terminated 
	__int64 now, lastTimeStamp = 0;
	double last_dot_time = 0;

	while (!closing)
	{
		now = SE_getSystemTime();
		deltaSimTime = (now - lastTimeStamp) / 1000.0;  // step size in seconds
		lastTimeStamp = now;

		bool add_dot = false;
		if (simTime - last_dot_time > 0.2)
		{
			add_dot = true;
			last_dot_time = simTime;
		}

		// Fetch states of scenario objects
		for (int i = 0; i < scenarioGateway->getNumberOfObjects(); i++)
		{
			ObjectState *o = scenarioGateway->getObjectStatePtrByIdx(i);
			ScenarioCar *sc = getScenarioCarById(o->state_.id);

			// If not available, create it
			if (sc == 0)
			{
				ScenarioCar new_sc;
				new_sc.flag_received_ghost_pos = false;
				new_sc.flag_received_steering_target_pos = false;

				LOG("Creating car %d - got state from gateway", o->state_.id);

				new_sc.id = o->state_.id;

				// Choose model from index - wrap to handle more vehicles than models
				int carModelID = o->state_.model_id;
				bool transparent = scenarioEngine->entities.object_[i]->control_ == Object::Control::HYBRID_GHOST ? true : false;

				// Create trail, choose color
				osg::Vec3 trail_color;
				if (o->state_.control == Object::Control::HYBRID_GHOST)
				{
					transparent = true;
					trail_color[0] = ((double)COLOR_DARK_GRAY[0]) / 0xFF;
					trail_color[1] = ((double)COLOR_DARK_GRAY[1]) / 0xFF;
					trail_color[2] = ((double)COLOR_DARK_GRAY[2]) / 0xFF;
				}
				else if (o->state_.control == Object::Control::HYBRID_EXTERNAL || 
					o->state_.control == Object::Control::EXTERNAL)
				{
					transparent = false;
					trail_color[0] = ((double)COLOR_YELLOW[0]) / 0xFF;
					trail_color[1] = ((double)COLOR_YELLOW[1]) / 0xFF;
					trail_color[2] = ((double)COLOR_YELLOW[2]) / 0xFF;
				}
				else
				{
					transparent = false;
					trail_color[0] = ((double)COLOR_RED[0]) / 0xFF;
					trail_color[1] = ((double)COLOR_RED[1]) / 0xFF;
					trail_color[2] = ((double)COLOR_RED[2]) / 0xFF;
				}

				new_sc.carModel = scViewer->AddCar(carModelsFiles_[carModelID], transparent, trail_color);

				if (scenarioEngine->entities.object_[i]->GetControl() == Object::Control::HYBRID_EXTERNAL)
				{
					new_sc.carModel->speed_sensor_ = scViewer->CreateSensor(COLOR_GRAY, true, true, 0.4, 1.0);
					new_sc.carModel->steering_sensor_ = scViewer->CreateSensor(COLOR_GREEN, false, true, 0.2, 3.0);
				}
				else if (scenarioEngine->entities.object_[i]->GetControl() == Object::Control::EXTERNAL)
				{
					scViewer->CreateRoadSensors(new_sc.carModel);
				}

				// Add it to the list of scenario cars
				scenarioCar.push_back(new_sc);

				sc = &scenarioCar.back();
			}
			sc->state = o->state_;
			
			if (add_dot)
			{
				sc->carModel->trail_->AddDot(simTime, o->state_.pos.GetX(), o->state_.pos.GetY(), o->state_.pos.GetZ(), o->state_.pos.GetH());
			}
		}

		// Visualize scenario cars

		mutex.Lock();

		for (size_t i = 0; i < scenarioCar.size(); i++)
		{
			ScenarioCar *c = &scenarioCar[i];
			if (c->carModel)
			{
				c->carModel->SetPosition(c->state.pos.GetX(), c->state.pos.GetY(), c->state.pos.GetZ());
				c->carModel->SetRotation(c->state.pos.GetH(), c->state.pos.GetP(), c->state.pos.GetR());
				c->carModel->UpdateWheelsDelta(c->state.wheel_angle, fmod(c->state.speed * deltaSimTime / 0.35, 2*M_PI));
				
				if (c->carModel->speed_sensor_ && c->flag_received_steering_target_pos)
				{
					scViewer->UpdateSensor(c->carModel->speed_sensor_, &c->state.pos, c->se_steering_target_pos);
				}
				if (c->carModel->steering_sensor_ && c->flag_received_ghost_pos)
				{
					scViewer->UpdateSensor(c->carModel->steering_sensor_, &c->state.pos, c->se_ghost_pos);
				}
				if (c->carModel->road_sensor_)
				{
					scViewer->UpdateRoadSensors(c->carModel->road_sensor_, c->carModel->lane_sensor_, &c->state.pos);
				}
			}
		}

		// Update info text 
		static char str_buf[128];
		snprintf(str_buf, sizeof(str_buf), "%.2fs %.2fkm/h", scenarioEngine->getSimulationTime(),
			3.6 * scenarioEngine->entities.object_[scViewer->currentCarInFocus_]->speed_);
		scViewer->SetInfoText(str_buf);

		// Update object sensors line visualization
		for (size_t i = 0; i < sensorFrustum.size(); i++)
		{
			sensorFrustum[i]->Update();
		}

		mutex.Unlock();

		scViewer->osgViewer_->frame();
		if (viewer_state == ViewerState::VIEWER_NOT_RUNNING)
		{
			// For some reason also the second frame takes long time
			viewer_state = ViewerState::VIEWER_STARTING;
		}
		else
		{
			viewer_state = ViewerState::VIEWER_RUNNING;
		}
	}
	viewer_state = ViewerState::VIEWER_NOT_RUNNING;
}

#else

static void Set_se_steering_target_pos(int id, float x, float y, float z) {}
static void Set_se_ghost_pos(int id, float x, float y, float z) {}

#endif

static void resetScenario(void)
{
#ifdef _SCENARIO_VIEWER
	if (scViewer != 0)
	{
		printf("Closing viewer\n");

		closing = true;  // Signal to viewer thread
		thread.Wait();
		closing = false;

		sensor.clear();
		sensorFrustum.clear();

		delete scViewer;
		scViewer = 0;
	}

	scenarioCar.clear();
#endif

	simTime = 0; // Start time initialized to zero
	deltaSimTime = 0;
	scenarioGateway = 0;

	if (scenarioEngine != 0)
	{
		delete scenarioEngine;
		scenarioEngine = 0;
		printf("Closing scenario engine\n");
	}
}

static void copyStateFromScenarioGateway(SE_ScenarioObjectState *state, ObjectStateStruct *gw_state)
{
	state->id = gw_state->id;
	state->model_id = gw_state->model_id;
	state->control = gw_state->control;
//	strncpy(state->name, gw_state->name, NAME_LEN);
	state->timestamp = gw_state->timeStamp;
	state->x = (float)gw_state->pos.GetX();
	state->y = (float)gw_state->pos.GetY();
	state->z = (float)gw_state->pos.GetZ();
	state->h = (float)gw_state->pos.GetH();
	state->p = (float)gw_state->pos.GetP();
	state->r = (float)gw_state->pos.GetR();
	state->speed = (float)gw_state->speed;
	state->roadId = (int)gw_state->pos.GetTrackId();
	state->t = (float)gw_state->pos.GetT();
	state->laneId = (int)gw_state->pos.GetLaneId();
	state->s = (float)gw_state->pos.GetS();
	state->laneOffset = (float)gw_state->pos.GetOffset();

}

static int GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo *r_data, int along_reference_lane)
{
	roadmanager::SteeringTargetInfo s_data;

	if (scenarioGateway == 0)
	{
		return -1;
	}

	if (object_id >= scenarioGateway->getNumberOfObjects())
	{
		LOG("Object %d not available, only %d registered", object_id, scenarioGateway->getNumberOfObjects());
		return -1;
	}

	roadmanager::Position *pos = &scenarioGateway->getObjectStatePtrByIdx(object_id)->state_.pos;
	
	if (pos->GetSteeringTargetInfo(lookahead_distance, &s_data, (bool)along_reference_lane) != 0)
	{
		return -1;
	}
	else
	{
		// Copy data
		r_data->local_pos_x = (float)s_data.local_pos[0];
		r_data->local_pos_y = (float)s_data.local_pos[1];
		r_data->local_pos_z = (float)s_data.local_pos[2];
		r_data->global_pos_x = (float)s_data.global_pos[0];
		r_data->global_pos_y = (float)s_data.global_pos[1];
		r_data->global_pos_z = (float)s_data.global_pos[2];
		r_data->angle = (float)s_data.angle;
		r_data->curvature = (float)s_data.curvature;
		r_data->road_heading = (float)s_data.road_heading;
		r_data->road_pitch = (float)s_data.road_pitch;
		r_data->road_roll = (float)s_data.road_roll;
		r_data->trail_heading = r_data->road_heading;
		r_data->speed_limit = (float)s_data.speed_limit;

		return 0;
	}
}

static int GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *r_data, float *speed_ghost)
{
	roadmanager::SteeringTargetInfo s_data;

	if (scenarioGateway == 0)
	{
		return -1;
	}

	if (object_id >= scenarioGateway->getNumberOfObjects())
	{
		LOG("Object %d not available, only %d registered", object_id, scenarioGateway->getNumberOfObjects());
		return -1;
	}

	Object *obj = scenarioEngine->entities.object_[object_id];
	if (obj->ghost_ == 0)
	{
		LOG("Ghost object not available for object id %d", object_id);
		return -1;
	}

	double s_out, x, y, z;
	int index_out;

	if (obj->ghost_->trail_.FindClosestPoint(obj->pos_.GetX(), obj->pos_.GetY(), x, y,
		obj->trail_follow_s_, obj->trail_follow_index_, obj->trail_follow_index_) == 0)
	{
		z = obj->pos_.GetZ();
	}
	else
	{
		// Failed find point along trail, copy entity position
		x = obj->pos_.GetX();
		y = obj->pos_.GetY();
		z = obj->pos_.GetZ();
	}

	ObjectTrailState state;
	state.h_ = obj->pos_.GetH();  // Set default trail heading aligned with road - in case trail is less than two points (no heading)
	obj->ghost_->trail_.FindPointAhead(obj->trail_follow_index_, obj->trail_follow_s_, lookahead_distance, state, index_out, s_out);

	roadmanager::Position pos(state.x_, state.y_, 0, 0, 0, 0);
	obj->pos_.CalcSteeringTarget(&pos, &s_data);

	// Copy data
	r_data->local_pos_x = (float)s_data.local_pos[0];
	r_data->local_pos_y = (float)s_data.local_pos[1];
	r_data->local_pos_z = (float)s_data.local_pos[2];
	r_data->global_pos_x = (float)s_data.global_pos[0];
	r_data->global_pos_y = (float)s_data.global_pos[1];
	r_data->global_pos_z = (float)s_data.global_pos[2];
	r_data->angle = (float)s_data.angle;
	r_data->curvature = (float)s_data.curvature;
	r_data->road_heading = (float)s_data.road_heading;
	r_data->trail_heading = (float)state.h_;
	r_data->road_pitch = (float)s_data.road_pitch;
	r_data->road_roll = (float)s_data.road_roll;
	r_data->speed_limit = (float)s_data.speed_limit;
	LOG("h %.2f", r_data->trail_heading);

	*speed_ghost = (float)obj->ghost_->speed_;

	return 0;
}

extern "C"
{
	SE_DLL_API void log_callback(const char *str)
	{
		printf("%s\n", str);
	}

	SE_DLL_API int SE_Init(const char *oscFilename, int control, int use_viewer, int record, float headstart_time)
	{

		Logger::Inst().SetCallback(log_callback);

#ifdef _SCENARIO_VIEWER		
		closing = false;
#endif
		resetScenario();

#ifndef _SCENARIO_VIEWER
		if (use_viewer)
		{
			LOG("use_viewer flag set, but no viewer available (compiled without -D _SCENARIO_VIEWWER");
		}
#endif	

		// Create scenario engine
		try
		{
			// Create a scenario engine instance
			scenarioEngine = new ScenarioEngine(std::string(oscFilename), headstart_time, (ScenarioEngine::RequestControlMode)control);

			// Fetch ScenarioGateway 
			scenarioGateway = scenarioEngine->getScenarioGateway();

			// Create a data file for later replay?
			if (record)
			{
				LOG("Recording data to file %s", DEFAULT_RECORDING_FILENAME);
				scenarioGateway->RecordToFile(DEFAULT_RECORDING_FILENAME, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
			}

			// Fetch ScenarioGateway 
			roadManager = scenarioEngine->getRoadManager();

			// Step scenario engine - zero time - just to reach init state
			// Report all vehicles initially - to communicate initial position for external vehicles as well
			scenarioEngine->step(0.0, true);

			// Fast forward to time == 0 - launching hybrid ghost vehicles
			while (scenarioEngine->getSimulationTime() < 0)
			{
				scenarioEngine->step(0.05);
			}

#ifdef _SCENARIO_VIEWER
			if (use_viewer)
			{
				// Run viewer in a separate thread
				thread.Start(viewer_thread, 0);

				// Wait for the viewer to launch
				while (viewer_state != ViewerState::VIEWER_RUNNING) SE_sleep(100);
			}
#endif
		}
		catch (const std::exception& e)
		{
			LOG(e.what());
			scenarioEngine = 0;
			scenarioGateway = 0;
#ifdef _SCENARIO_VIEWER
			scViewer = 0;
#endif
			return -1;
		}

		return 0;
	}

	SE_DLL_API void SE_Close()
	{
		resetScenario();
	}

	SE_DLL_API int SE_Step(float dt)
	{
#if _SCENARIO_VIEWER

		// Check for Viewer quit request 
		if (scViewer && scViewer->GetQuitRequest())
		{
			SE_Close();
			return -1;
		}
#endif
		if (scenarioEngine != 0)
		{
			// Time operations
			simTime += dt;

			// ScenarioEngine
#if _SCENARIO_VIEWER
			mutex.Lock();
#endif

			scenarioEngine->step((double)dt);

#if _SCENARIO_VIEWER
			// Update sensors
			for (size_t i = 0; i < sensor.size(); i++)
			{
				sensor[i]->Update();
			}

			mutex.Unlock();
#endif
		}

		return 0;
	}
	
	SE_DLL_API float SE_GetSimulationTime()
	{
		return (float)scenarioEngine->getSimulationTime();
	}

	SE_DLL_API int SE_ReportObjectPos(int id, float timestamp, float x, float y, float z, float h, float p, float r, float speed)
	{
		if (scenarioGateway != 0)
		{
			if (id < scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = scenarioEngine->entities.object_[id];
				int control = obj->control_ == Object::Control::EXTERNAL || obj->control_ == Object::Control::HYBRID_EXTERNAL;
				scenarioGateway->reportObject(ObjectState(id, obj->name_, obj->model_id_, control, timestamp, x, y, z, h, p, r, speed, 0, 0), true);
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectRoadPos(int id, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed)
	{
		if (scenarioGateway != 0)
		{
			if (id < scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = scenarioEngine->entities.object_[id];
				int control = obj->control_ == Object::Control::EXTERNAL || obj->control_ == Object::Control::HYBRID_EXTERNAL;
				scenarioGateway->reportObject(ObjectState(id, obj->name_, obj->model_id_, control, timestamp, roadId, laneId, laneOffset, s, speed, 0, 0), true);
			}
		}

		return 0;
	}

	SE_DLL_API int SE_GetNumberOfObjects()
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

	SE_DLL_API int SE_GetObjectState(int index, SE_ScenarioObjectState *state)
	{
		if (scenarioGateway != 0)
		{
			copyStateFromScenarioGateway(state, &scenarioGateway->getObjectStatePtrByIdx(index)->state_);
		}

		return 0;
	}

	SE_DLL_API int SE_GetObjectGhostState(int index, SE_ScenarioObjectState *state)
	{
		if (scenarioGateway != 0)
		{
			if (index < scenarioEngine->entities.object_.size())
			{
				for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)  // ghost index always higher than external buddy
				{
					if (scenarioEngine->entities.object_[index]->ghost_)
					{
						scenarioengine::ObjectState obj_state;
						scenarioGateway->getObjectStateById(scenarioEngine->entities.object_[index]->ghost_->id_, obj_state);
						copyStateFromScenarioGateway(state, &obj_state.state_);
					}
				}
			}
		}

		return 0;
	}

	SE_DLL_API int SE_GetObjectStates(int *nObjects, SE_ScenarioObjectState* state)
	{
		int i;

		if (scenarioGateway != 0)
		{
			for (i = 0; i < *nObjects && i < scenarioGateway->getNumberOfObjects(); i++)
			{
				copyStateFromScenarioGateway(&state[i], &scenarioGateway->getObjectStatePtrByIdx(i)->state_);
			}
			*nObjects = i;
		}
		else
		{
			*nObjects = 0;
		}
		return 0;
	}

	SE_DLL_API int SE_AddObjectSensor(int object_id, float x, float y, float z, float rangeNear, float rangeFar, float fovH, int maxObj)
	{
		if (scenarioGateway != 0)
		{

			if (object_id < 0 || object_id >= scenarioEngine->entities.object_.size())
			{
				LOG("Invalid object_id (%d/%d)", object_id, scenarioEngine->entities.object_.size());
				return -1;
			}

			ObjectSensor *sensor_ = new ObjectSensor(&scenarioEngine->entities, scenarioEngine->entities.object_[object_id],
				x, y, z, rangeNear, rangeFar, fovH, maxObj);

			sensor.push_back(sensor_);

			// Add visual representation of sensor to viewer
#if _SCENARIO_VIEWER
			if (scViewer)
			{
				mutex.Lock();

				viewer::SensorViewFrustum *sensorFrustum_ = new viewer::SensorViewFrustum(sensor_, scViewer->cars_[object_id]->txNode_);
				sensorFrustum.push_back(sensorFrustum_);

				scViewer->ShowObjectSensors(true);
				mutex.Unlock();
			}

#endif
		}
		return 0;
	}

	SE_DLL_API int SE_FetchSensorObjectList(int object_id, int *list)
	{
		if (scenarioGateway != 0)
		{
			if (object_id < 0 || object_id >= scenarioEngine->entities.object_.size())
			{
				LOG("Invalid object_id (%d/%d)", object_id, scenarioEngine->entities.object_.size());
				return -1;
			}

			for (int i = 0; i < sensor[object_id]->nObj_; i++)
			{
				list[i] = sensor[object_id]->hitList_[i].obj_->id_;
			}

			return sensor[object_id]->nObj_;
		}
		return 0;
	}

	SE_DLL_API int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo *data, int along_road_center)
	{
		if (scenarioGateway == 0 || object_id >= scenarioGateway->getNumberOfObjects())
		{
			return -1;
		}

		if (GetRoadInfoAtDistance(object_id, lookahead_distance, data, along_road_center) != 0)
		{
			return -1;
		}

		Set_se_steering_target_pos(object_id, data->global_pos_x, data->global_pos_y, data->global_pos_z);

		return 0;
	}

	SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *data, float *speed_ghost)
	{
		if (scenarioGateway == 0 || object_id >= scenarioGateway->getNumberOfObjects())
		{
			return -1;
		}

		if (GetRoadInfoAlongGhostTrail(object_id, lookahead_distance, data, speed_ghost) != 0)
		{
			return -1;
		}

		Set_se_ghost_pos(object_id, data->global_pos_x, data->global_pos_y, data->global_pos_z);
		//LOG("id %d dist %.2f x %.2f y %.2f z %.2f", object_id, lookahead_distance, data->global_pos_x, data->global_pos_y, data->global_pos_z);
		return 0;
	}
}
