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

#include "PlayerBase.hpp"
#include "scenarioenginedll.hpp"
#include "IdealSensor.hpp"

using namespace scenarioengine;

#define EGO_ID 0	// need to match appearing order in the OpenSCENARIO file

static ScenarioPlayer *player = 0;

static char **argv;
static int argc = 0;
static std::vector<std::string> args_v;

static void resetScenario(void)
{
	if (player)
	{
		delete player;
		player = 0;
	}
}

static void AddArgument(const char *str)
{
	// split separate argument strings
	std::vector<std::string> args = SplitString(std::string(str), ' ');

	for (size_t i = 0; i < args.size(); i++)
	{
		args_v.push_back(args[i]);
	}
}

static void ConvertArguments()
{
	argc = args_v.size();
	argv = (char**)malloc(argc * sizeof(char*));
	for (int i = 0; i < argc; i++)
	{
		argv[i] = (char*)malloc((args_v[i].size() + 1) * sizeof(char));
		strcpy(argv[i], args_v[i].c_str());
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

static int GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo *r_data, int lookAheadMode)
{
	roadmanager::SteeringTargetInfo s_data;

	if (player == 0)
	{
		return -1;
	}

	if (object_id >= player->scenarioGateway->getNumberOfObjects())
	{
		LOG("Object %d not available, only %d registered", object_id, player->scenarioGateway->getNumberOfObjects());
		return -1;
	}

	roadmanager::Position *pos = &player->scenarioGateway->getObjectStatePtrByIdx(object_id)->state_.pos;
	
	if (pos->GetSteeringTargetInfo(lookahead_distance, &s_data, (roadmanager::Position::LookAheadMode)lookAheadMode) != 0)
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

	if (player == 0)
	{
		return -1;
	}

	if (object_id >= player->scenarioGateway->getNumberOfObjects())
	{
		LOG("Object %d not available, only %d registered", object_id, player->scenarioGateway->getNumberOfObjects());
		return -1;
	}

	Object *obj = player->scenarioEngine->entities.object_[object_id];
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
	SE_DLL_API int SE_Init(const char *oscFilename, int control, int use_viewer, int record, float headstart_time)
	{
		resetScenario();

#ifndef _SCENARIO_VIEWER
		if (use_viewer)
		{
			LOG("use_viewer flag set, but no viewer available (compiled without -D _SCENARIO_VIEWWER");
		}
#endif	

		AddArgument("viewer");  // name of application
		AddArgument("--osc");
		AddArgument(oscFilename);
		AddArgument("--control");
		if (control == 0)
		{
			AddArgument("osc");
		}
		else if (control == 1)
		{
			AddArgument("internal");
		}
		else if (control == 2)
		{
			AddArgument("external");
		}
		else if (control == 3)
		{
			AddArgument("hybrid");
		}
		if (record)
		{
			AddArgument("--record scenario.dat");
		}
		AddArgument("--window 30 30 800 400");
		ConvertArguments();

		// Create scenario engine
		try
		{
			// Initialize the scenario engine and viewer
			player = new ScenarioPlayer(argc, argv);

			// Fast forward to time == 0 - launching hybrid ghost vehicles
			//while (player->scenarioEngine->getSimulationTime() < 0)
			//{
			//	player->Frame(0.05);
			//}

		}
		catch (const std::exception& e)
		{
			LOG(e.what());
			resetScenario();
			return -1;
		}

		return 0;
	}

	SE_DLL_API void SE_Close()
	{
		resetScenario();
	}

	SE_DLL_API int SE_Step()
	{
		if (player)
		{
			player->Frame();
		}

		return 0;
	}

	SE_DLL_API int SE_StepDT(float dt)
	{
		if (player)
		{
			player->Frame(dt);
		}

		return 0;
	}
	
	SE_DLL_API float SE_GetSimulationTime()
	{
		return (float)player->scenarioEngine->getSimulationTime();
	}

	SE_DLL_API int SE_ReportObjectPos(int id, float timestamp, float x, float y, float z, float h, float p, float r, float speed)
	{
		if (player)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = player->scenarioEngine->entities.object_[id];
				int control = obj->control_ == Object::Control::EXTERNAL || obj->control_ == Object::Control::HYBRID_EXTERNAL;
				player->scenarioGateway->reportObject(ObjectState(id, obj->name_, obj->model_id_, control, timestamp, x, y, z, h, p, r, speed, 0, 0), true);
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectRoadPos(int id, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed)
	{
		if (player)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = player->scenarioEngine->entities.object_[id];
				int control = obj->control_ == Object::Control::EXTERNAL || obj->control_ == Object::Control::HYBRID_EXTERNAL;
				player->scenarioGateway->reportObject(ObjectState(id, obj->name_, obj->model_id_, control, timestamp, roadId, laneId, laneOffset, s, speed, 0, 0), true);
			}
		}

		return 0;
	}

	SE_DLL_API int SE_GetNumberOfObjects()
	{
		if (player)
		{
			return player->scenarioGateway->getNumberOfObjects();
		}
		else
		{
			return 0;
		}
	}

	SE_DLL_API int SE_GetObjectState(int index, SE_ScenarioObjectState *state)
	{
		if (player)
		{
			copyStateFromScenarioGateway(state, &player->scenarioGateway->getObjectStatePtrByIdx(index)->state_);
		}

		return 0;
	}

	SE_DLL_API int SE_GetObjectGhostState(int index, SE_ScenarioObjectState *state)
	{
		if (player)
		{
			if (index < player->scenarioEngine->entities.object_.size())
			{
				for (size_t i = 0; i < player->scenarioEngine->entities.object_.size(); i++)  // ghost index always higher than external buddy
				{
					if (player->scenarioEngine->entities.object_[index]->ghost_)
					{
						scenarioengine::ObjectState obj_state;
						player->scenarioGateway->getObjectStateById(player->scenarioEngine->entities.object_[index]->ghost_->id_, obj_state);
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

		if (player)
		{
			for (i = 0; i < *nObjects && i < player->scenarioGateway->getNumberOfObjects(); i++)
			{
				copyStateFromScenarioGateway(&state[i], &player->scenarioGateway->getObjectStatePtrByIdx(i)->state_);
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
		if (player)
		{

			if (object_id < 0 || object_id >= player->scenarioEngine->entities.object_.size())
			{
				LOG("Invalid object_id (%d/%d)", object_id, player->scenarioEngine->entities.object_.size());
				return -1;
			}

			player->AddObjectSensor(object_id, x, y, z, rangeNear, rangeFar, fovH, maxObj);
		}
		return 0;
	}

	SE_DLL_API int SE_FetchSensorObjectList(int object_id, int *list)
	{
		if (player)
		{
			if (object_id < 0 || object_id >= player->scenarioEngine->entities.object_.size())
			{
				LOG("Invalid object_id (%d/%d)", object_id, player->scenarioEngine->entities.object_.size());
				return -1;
			}

			for (int i = 0; i < player->sensor[object_id]->nObj_; i++)
			{
				list[i] = player->sensor[object_id]->hitList_[i].obj_->id_;
			}

			return player->sensor[object_id]->nObj_;
		}
		return 0;
	}

	SE_DLL_API int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo *data, int along_road_center)
	{
		if (player == 0 || object_id >= player->scenarioGateway->getNumberOfObjects())
		{
			return -1;
		}

		if (GetRoadInfoAtDistance(object_id, lookahead_distance, data, along_road_center) != 0)
		{
			return -1;
		}

//		Set_se_steering_target_pos(object_id, data->global_pos_x, data->global_pos_y, data->global_pos_z);

		return 0;
	}

	SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *data, float *speed_ghost)
	{
		if (player == 0 || object_id >= player->scenarioGateway->getNumberOfObjects())
		{
			return -1;
		}

		if (GetRoadInfoAlongGhostTrail(object_id, lookahead_distance, data, speed_ghost) != 0)
		{
			return -1;
		}

//		Set_se_ghost_pos(object_id, data->global_pos_x, data->global_pos_y, data->global_pos_z);
		//LOG("id %d dist %.2f x %.2f y %.2f z %.2f", object_id, lookahead_distance, data->global_pos_x, data->global_pos_y, data->global_pos_z);
		return 0;
	}
}
