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

#include <string>
#include <clocale>

#include "CommonMini.hpp"
#include "playerbase.hpp"
#include "esminiLib.hpp"
#include "IdealSensor.hpp"
#include "osi_sensordata.pb.h"
#include "vehicle.hpp"

using namespace scenarioengine;

#define EGO_ID 0	// need to match appearing order in the OpenSCENARIO file

static ScenarioPlayer *player = 0;

static char **argv = 0;
static int argc = 0;
static std::vector<std::string> args_v;
static std::string returnString;  // use this for returning strings

typedef struct
{
	int id;
	void (*func)(SE_ScenarioObjectState*, void*);
	void* data;
} SE_ObjCallback;

static std::vector<SE_ObjCallback> objCallback;

static void log_callback(const char* str)
{
	printf("%s\n", str);
}

static void resetScenario(void)
{
	if (player)
	{
		delete player;
		player = 0;
	}
	if (argv)
	{
		for (int i = 0; i < argc; i++)
		{
			free(argv[i]);
		}
		free(argv);
		argv = 0;
		argc = 0;
	}
	args_v.clear();
	returnString = "";
}

static void AddArgument(const char *str, bool split=true)
{
	std::vector<std::string> args;
	if (split)
	{
		// split separate argument strings
		args = SplitString(std::string(str), ' ');
	}
	else
	{
		args.push_back(std::string(str));
	}

	for (size_t i = 0; i < args.size(); i++)
	{
		args_v.push_back(args[i]);
	}
}

static void ConvertArguments()
{
	argc = (int)args_v.size();
	argv = (char**)malloc(argc * sizeof(char*));
	std::string argument_list;
	for (int i = 0; i < argc; i++)
	{
		argv[i] = (char*)malloc((args_v[i].size() + 1) * sizeof(char));
		strcpy(argv[i], args_v[i].c_str());
		argument_list += std::string(" ") + argv[i];
	}
	LOG("Player arguments: %s", argument_list.c_str());
}

static void copyStateFromScenarioGateway(SE_ScenarioObjectState *state, ObjectStateStruct *gw_state)
{
	state->id = gw_state->id;
	state->model_id = gw_state->model_id;
	state->ctrl_type = gw_state->ctrl_type;
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
	state->centerOffsetX = gw_state->boundingbox.center_.x_;
	state->centerOffsetY = gw_state->boundingbox.center_.y_;
	state->centerOffsetZ = gw_state->boundingbox.center_.z_;
	state->width = gw_state->boundingbox.dimensions_.width_;
	state->length = gw_state->boundingbox.dimensions_.length_;
	state->height = gw_state->boundingbox.dimensions_.height_;
}

static void copyStateToScenarioGateway(ObjectStateStruct* gw_state, SE_ScenarioObjectState* state)
{
	gw_state->id = state->id;
	gw_state->model_id = state->model_id;
	gw_state->ctrl_type = state->ctrl_type;
	//	strncpy(state->name, gw_state->name, NAME_LEN);
	gw_state->timeStamp = state->timestamp;
	gw_state->pos.SetX(state->x);
	gw_state->pos.SetY(state->y);
	gw_state->pos.SetZ(state->z);
	gw_state->pos.SetH(state->h);
	gw_state->pos.SetP(state->p);
	gw_state->pos.SetR(state->r);
	gw_state->speed = state->speed;
	gw_state->pos.SetTrackPos(state->roadId, state->s, state->t, roadmanager::Position::UpdateTrackPosMode::UPDATE_NOT_XYZH);
	gw_state->pos.SetLaneId(state->laneId);
	gw_state->pos.SetS(state->s);
	gw_state->pos.SetOffset(state->laneOffset);
	gw_state->boundingbox.center_.x_ = state->centerOffsetX;
	gw_state->boundingbox.center_.y_ = state->centerOffsetY;
	gw_state->boundingbox.center_.z_ = state->centerOffsetZ;
	gw_state->boundingbox.dimensions_.width_ = state->width;
	gw_state->boundingbox.dimensions_.length_ = state->length;
	gw_state->boundingbox.dimensions_.height_ = state->height;
}

static int GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo *r_data, int lookAheadMode)
{
	roadmanager::RoadProbeInfo s_data;

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

	if (pos->GetProbeInfo(lookahead_distance, &s_data, (roadmanager::Position::LookAheadMode)lookAheadMode) != 0)
	{
		return -1;
	}
	else
	{
		// Copy data
		r_data->local_pos_x = (float)s_data.relative_pos[0];
		r_data->local_pos_y = (float)s_data.relative_pos[1];
		r_data->local_pos_z = (float)s_data.relative_pos[2];
		r_data->global_pos_x = (float)s_data.road_lane_info.pos[0];
		r_data->global_pos_y = (float)s_data.road_lane_info.pos[1];
		r_data->global_pos_z = (float)s_data.road_lane_info.pos[2];
		r_data->angle = (float)s_data.relative_h;
		r_data->curvature = (float)s_data.road_lane_info.curvature;
		r_data->road_heading = (float)s_data.road_lane_info.heading;
		r_data->road_pitch = (float)s_data.road_lane_info.pitch;
		r_data->road_roll = (float)s_data.road_lane_info.roll;
		r_data->trail_heading = r_data->road_heading;
		r_data->speed_limit = (float)s_data.road_lane_info.speed_limit;

		// Add visualization of forward looking road sensor probe
		#ifdef _SCENARIO_VIEWER
			if (player->viewer_ && player->viewer_->entities_[object_id]->GetType() == viewer::EntityModel::ENTITY_TYPE_VEHICLE)
			{
				viewer::CarModel* model = (viewer::CarModel*)player->viewer_->entities_[object_id];
				model->steering_sensor_->Show();
				player->viewer_->SensorSetPivotPos(model->steering_sensor_, pos->GetX(), pos->GetY(), pos->GetZ());
				player->viewer_->SensorSetTargetPos(model->steering_sensor_, 
					s_data.road_lane_info.pos[0], 
					s_data.road_lane_info.pos[1],
					s_data.road_lane_info.pos[2]);
				player->viewer_->UpdateSensor(model->steering_sensor_);
			}
		#endif
		return 0;
	}
}

static int GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *r_data, float *speed_ghost)
{
	roadmanager::RoadProbeInfo s_data;

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
	Object* ghost = 0;
	if (obj->GetAssignedControllerType() != Controller::Type::CONTROLLER_TYPE_DEFAULT)
	{
		ghost = obj->GetGhost();
		if (ghost == 0)
		{
			LOG("Ghost object not available for object id %d", object_id);
			return -1;
		}
	}

	double s_out, x, y, z;
	int index_out;

	if (ghost->trail_.FindClosestPoint(obj->pos_.GetX(), obj->pos_.GetY(), x, y,
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
	state.h_ = (float)obj->pos_.GetH();  // Set default trail heading aligned with road - in case trail is less than two points (no heading)
	ghost->trail_.FindPointAhead(obj->trail_follow_index_, obj->trail_follow_s_, lookahead_distance, state, index_out, s_out);

	roadmanager::Position pos(state.x_, state.y_, 0, 0, 0, 0);
	obj->pos_.CalcProbeTarget(&pos, &s_data);

	// Copy data
	r_data->local_pos_x = (float)s_data.relative_pos[0];
	r_data->local_pos_y = (float)s_data.relative_pos[1];
	r_data->local_pos_z = (float)s_data.relative_pos[2];
	r_data->global_pos_x = (float)s_data.road_lane_info.pos[0];
	r_data->global_pos_y = (float)s_data.road_lane_info.pos[1];
	r_data->global_pos_z = (float)s_data.road_lane_info.pos[2];
	r_data->angle = (float)s_data.relative_h;
	r_data->curvature = (float)s_data.road_lane_info.curvature;
	r_data->road_heading = (float)s_data.road_lane_info.heading;
	r_data->trail_heading = (float)state.h_;
	r_data->road_pitch = (float)s_data.road_lane_info.pitch;
	r_data->road_roll = (float)s_data.road_lane_info.roll;
	r_data->speed_limit = (float)s_data.road_lane_info.speed_limit;

	*speed_ghost = (float)state.speed_;

	return 0;
}

static int InitScenario()
{
	// Harmonize parsing and printing of floating point numbers. I.e. 1.57e+4 == 15700.0 not 15,700.0 or 1 or 1.57
	std::setlocale(LC_ALL, "C.UTF-8");

	Logger::Inst().SetCallback(log_callback);

	ConvertArguments();

	// Create scenario engine
	try
	{
		// Initialize the scenario engine and viewer
		player = new ScenarioPlayer(argc, argv);
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		resetScenario();
		return -1;
	}

	return 0;
}

extern "C"
{
	SE_DLL_API int SE_AddPath(const char* path)
	{
		SE_Env::Inst().AddPath(path);
		return 0;
	}

	SE_DLL_API void SE_ClearPaths()
	{
		SE_Env::Inst().ClearPaths();
	}

	SE_DLL_API int SE_InitWithArgs(int argc, char* argv[])
	{
		resetScenario();

		for (int i = 0; i < argc; i++)
		{
			AddArgument(argv[i]);
		}

		return InitScenario();
	}

	SE_DLL_API int SE_Init(const char *oscFilename, int disable_ctrls, int use_viewer, int threads, int record)
	{
#ifndef _SCENARIO_VIEWER
		if (use_viewer)
		{
			LOG("use_viewer flag set, but no viewer available (compiled without -D _SCENARIO_VIEWER");
		}
#endif
		resetScenario();

		AddArgument("viewer");  // name of application
		AddArgument("--osc");
		AddArgument(oscFilename, false);

		if (record)
		{
			AddArgument("--record");
			std::string datFilename = FileNameWithoutExtOf(oscFilename) + ".dat";
			AddArgument(datFilename.c_str(), false);
		}
		if (use_viewer)
		{
			AddArgument("--window 50 50 800 400", true);
		}
		else
		{
			AddArgument("--headless");
		}

		if (threads)
		{
			AddArgument("--threads");
		}

		if (disable_ctrls)
		{
			AddArgument("--disable_controllers");
		}

		return InitScenario();
	}

	SE_DLL_API int SE_GetQuitFlag()
	{
		int quit_flag;
		if(player)
		{
			if(player->IsQuitRequested())
				quit_flag = 1;
			else
				quit_flag = 0;
		}
		else
			quit_flag = 2;

		return quit_flag;

	}

	SE_DLL_API const char* SE_GetODRFilename()
	{
		if (!player)
		{
			return 0;
		}
		returnString = player->scenarioEngine->getOdrFilename().c_str();
		return returnString.c_str();
	}

	SE_DLL_API const char* SE_GetSceneGraphFilename()
	{
		if (!player)
		{
			return 0;
		}
		returnString = player->scenarioEngine->getSceneGraphFilename().c_str();
		return returnString.c_str();
	}

	SE_DLL_API int SE_SetParameter(SE_Parameter parameter)
	{
		if (player)
		{
			return player->SetParameterValue(parameter.name, parameter.value);
		}

		return -1;
	}

	SE_DLL_API int SE_GetParameter(SE_Parameter* parameter)
	{
		if (player)
		{
			return player->GetParameterValue(parameter->name, parameter->value);
		}

		return -1;
	}

	SE_DLL_API void SE_Close()
	{
		resetScenario();
	}

	SE_DLL_API int SE_OpenOSISocket(char* ipaddr)
	{
		player->osiReporter->OpenSocket(ipaddr);
		return 0;
	}

	SE_DLL_API int SE_Step()
	{
		if (player)
		{
			player->Frame();
			return 0;
		}
		else
		{
			return -1;
		}
	}

	SE_DLL_API int SE_StepDT(float dt)
	{
		if (player)
		{
			player->Frame(dt);
			return 0;
		}
		else
		{
			return -1;
		}
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
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_holder_, obj->model_id_, 
					obj->GetActivatedControllerType(), obj->boundingbox_, timestamp, speed, 0, 0, x, y, z, h, p, r);
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
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_holder_, obj->model_id_, 
					obj->GetActivatedControllerType(), obj->boundingbox_, timestamp, speed, 0, 0, roadId, laneId, laneOffset, s);
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectSpeed(int id, float speed)
	{
		if (player)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				Object* obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_holder_, obj->model_id_,
					obj->GetActivatedControllerType(), obj->boundingbox_, 0, speed, obj->wheel_angle_, obj->wheel_rot_, &obj->pos_);
			}
			else
			{
				return -1;
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectLateralPosition(int id, float t)
	{
		if (player)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object* obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_holder_, obj->model_id_,
					obj->GetActivatedControllerType(), obj->boundingbox_, 0, obj->GetSpeed(), obj->wheel_angle_,
					obj->wheel_rot_, obj->pos_.GetTrackId(), t, obj->pos_.GetS());
			}
			else
			{
				return -1;
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectLateralLanePosition(int id, int laneId, float laneOffset)
	{
		if (player)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object* obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_holder_, obj->model_id_,
					obj->GetActivatedControllerType(), obj->boundingbox_, 0, obj->GetSpeed(), obj->wheel_angle_,
					obj->wheel_rot_, obj->pos_.GetTrackId(), obj->pos_.GetLaneId(), laneOffset, obj->pos_.GetS());
			}
			else
			{
				return -1;
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

	SE_DLL_API const char* SE_GetOSIGroundTruth(int* size)
	{
		if (player)
		{
			return player->osiReporter->GetOSIGroundTruth(size);
		}

		*size = 0;
		return 0;
	}

	SE_DLL_API const char* SE_GetOSIGroundTruthRaw()
	{
		if (player)
		{
			return (const char*) player->osiReporter->GetOSIGroundTruthRaw();
		}

		return 0;
	}

	SE_DLL_API const char* SE_GetOSIRoadLane(int* size, int object_id)
	{
		if (player)
		{
			return player->osiReporter->GetOSIRoadLane(player->scenarioGateway->objectState_, size, object_id);
		}

		*size = 0;
		return 0;
	}

	SE_DLL_API const char* SE_GetOSILaneBoundary(int* size, int global_id)
	{
		if (player)
		{
			return player->osiReporter->GetOSIRoadLaneBoundary(size, global_id);
		}

		*size = 0;
		return 0;
	}

	SE_DLL_API void SE_GetOSILaneBoundaryIds(int object_id, SE_LaneBoundaryId* ids)
	{
		if (player)
		{
			std::vector<int> ids_vector;
			player->osiReporter->GetOSILaneBoundaryIds(player->scenarioGateway->objectState_, ids_vector, object_id);
			if (!ids_vector.empty())
			{
				ids->far_left_lb_id = ids_vector[0];
				ids->left_lb_id = ids_vector[1];
				ids->right_lb_id = ids_vector[2];
				ids->far_right_lb_id = ids_vector[3];
			}
		}
		return;
	}

	SE_DLL_API int SE_UpdateOSIGroundTruth()
	{
		if (player)
		{
			return player->osiReporter->UpdateOSIGroundTruth(player->scenarioGateway->objectState_);
		}

		return 0;
	}

    SE_DLL_API const char* SE_GetOSISensorDataRaw()
	{
		if (player)
		{
			return (const char*) player->osiReporter->GetOSISensorDataRaw();
		}

		return 0;
	}

	SE_DLL_API bool SE_OSIFileOpen()
	{
		if (player)
		{
			return player->osiReporter->OpenOSIFile();
		}

		return false;
	}

	SE_DLL_API bool SE_OSIFileWrite(bool flush)
	{
		bool retval = false;

		if (player)
		{
			retval = player->osiReporter->WriteOSIFile();
			if (flush)
			{
				player->osiReporter->FlushOSIFile();
			}
		}

		return retval;
	}

	SE_DLL_API void SE_LogMessage(char* message)
	{
		LOG(message);
	}


	SE_DLL_API int SE_ObjectHasGhost(int index)
	{
		Object* ghost = 0;
		if(player->scenarioEngine->entities.object_[index]->GetAssignedControllerType() != Controller::Type::CONTROLLER_TYPE_DEFAULT)
		{
			ghost = player->scenarioEngine->entities.object_[index]->GetGhost();
		}
		return ghost != 0 ? 1 : 0;
	}

	SE_DLL_API int SE_GetObjectGhostState(int index, SE_ScenarioObjectState *state)
	{
		Object* ghost = 0;
		if (player)
		{
			if (index < player->scenarioEngine->entities.object_.size())
			{
				for (size_t i = 0; i < player->scenarioEngine->entities.object_.size(); i++)  // ghost index always higher than external buddy
				{
					if (player->scenarioEngine->entities.object_[index]->GetAssignedControllerType() != Controller::Type::CONTROLLER_TYPE_DEFAULT)
					{
						ghost = player->scenarioEngine->entities.object_[index]->GetGhost();
					}
					if (ghost)
					{
						scenarioengine::ObjectState obj_state;
						player->scenarioGateway->getObjectStateById(ghost->id_, obj_state);
						copyStateFromScenarioGateway(state, &obj_state.state_);
					}
				}
			}
		}

		return ghost != 0 ? 0 : -1;
	}

	/*SE_DLL_API int SE_GetObjectGhostStateFromOSI(const char* output, int index)
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
						copyStateFromScenarioGatewayToOSI(&output, &obj_state.state_);
					}
				}
			}
		}

		return 0;

	}*/

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

	SE_DLL_API int SE_AddObjectSensor(int object_id, float x, float y, float z, float h, float rangeNear, float rangeFar, float fovH, int maxObj)
	{
		if (player)
		{

			if (object_id < 0 || object_id >= player->scenarioEngine->entities.object_.size())
			{
				LOG("Invalid object_id (%d/%d)", object_id, player->scenarioEngine->entities.object_.size());
				return -1;
			}

			player->AddObjectSensor(object_id, x, y, z, h, rangeNear, rangeFar, fovH, maxObj);
		}

		player->ShowObjectSensors(true);

		return 0;
	}

	SE_DLL_API void SE_DisableOSIFile()
	{
		if (player)
		{
			player->SetOSIFileStatus(false);
		}
	}

	SE_DLL_API void SE_EnableOSIFile()
	{
		if (player)
		{
			player->SetOSIFileStatus(true);
		}
	}

	SE_DLL_API int SE_FetchSensorObjectList(int sensor_id, int *list)
	{
		if (player)
		{
			if (sensor_id < 0 || sensor_id >= player->sensor.size())
			{
				LOG("Invalid sensor_id (%d specified / %d available)", sensor_id, player->sensor.size());
				return -1;
			}

			for (int i = 0; i < player->sensor[sensor_id]->nObj_; i++)
			{
				list[i] = player->sensor[sensor_id]->hitList_[i].obj_->id_;
			}

			return player->sensor[sensor_id]->nObj_;
		}

		return -1;
	}

	SE_DLL_API int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo* data, int along_road_center)
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

	SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo* data, float* speed_ghost)
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

	void callback(ObjectStateStruct* state, void* my_data)
	{
		for (size_t i = 0; i < objCallback.size(); i++)
		{
			if (objCallback[i].id == state->id)
			{
				SE_ScenarioObjectState se_state;
				copyStateFromScenarioGateway(&se_state, state);
				objCallback[i].func(&se_state, my_data);
			}
		}
	}

	SE_DLL_API void SE_RegisterObjectCallback(int object_id, void (*fnPtr)(SE_ScenarioObjectState*, void*), void *user_data)
	{
		if (player == 0 || object_id >= player->scenarioGateway->getNumberOfObjects())
		{
			return;
		}
		SE_ObjCallback cb;
		cb.id = object_id;
		cb.func = fnPtr;
		objCallback.push_back(cb);
		player->RegisterObjCallback(object_id, callback, user_data);
	}

	#ifdef _SCENARIO_VIEWER
		SE_DLL_API void SE_ViewerShowFeature(int featureType, bool enable)
		{
			if (player && player->viewer_)
			{
				player->viewer_->SetNodeMaskBits(featureType, enable ? featureType : 0x0);
			}
		}
	#endif

	// Simple vehicle 
	SE_DLL_API void* SE_SimpleVehicleCreate(float x, float y, float h, float length)
	{
		vehicle::Vehicle* v = new vehicle::Vehicle(x, y, h, length);
		return (void*)v;
	}

	SE_DLL_API void SE_SimpleVehicleDelete(void* handleSimpleVehicle)
	{
		if (handleSimpleVehicle)
		{
			free((vehicle::Vehicle*)handleSimpleVehicle);
			handleSimpleVehicle = 0;
		}
	}

	SE_DLL_API void SE_SimpleVehicleControlBinary(void* handleSimpleVehicle, double dt, int throttle, int steering)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}

		((vehicle::Vehicle*)handleSimpleVehicle)->DrivingControlBinary(dt,
			(vehicle::THROTTLE)throttle, (vehicle::STEERING)steering);
	}

	SE_DLL_API void SE_SimpleVehicleControlAnalog(void* handleSimpleVehicle, double dt, double throttle, double steering)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}

		((vehicle::Vehicle*)handleSimpleVehicle)->DrivingControlAnalog(dt, throttle, steering);
	}

	SE_DLL_API void SE_SimpleVehicleSetMaxSpeed(void* handleSimpleVehicle, float speed)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		((vehicle::Vehicle*)handleSimpleVehicle)->SetMaxSpeed(speed);
	}

	SE_DLL_API void SE_SimpleVehicleGetState(void* handleSimpleVehicle, SE_SimpleVehicleState* state)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		state->x = ((vehicle::Vehicle*)handleSimpleVehicle)->posX_;
		state->y = ((vehicle::Vehicle*)handleSimpleVehicle)->posY_;
		state->z = ((vehicle::Vehicle*)handleSimpleVehicle)->posZ_;
		state->h = ((vehicle::Vehicle*)handleSimpleVehicle)->heading_;
		state->p = ((vehicle::Vehicle*)handleSimpleVehicle)->pitch_;
		state->speed = ((vehicle::Vehicle*)handleSimpleVehicle)->speed_;
	}
}
