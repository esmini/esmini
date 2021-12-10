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
#include "pugixml.hpp"

using namespace scenarioengine;

#define EGO_ID 0 // need to match appearing order in the OpenSCENARIO file

static ScenarioPlayer *player = 0;

static char **argv_ = 0;
static int argc_ = 0;
static std::vector<std::string> args_v;
static bool logToConsole = true;

typedef struct
{
	int id;
	void (*func)(SE_ScenarioObjectState *, void *);
	void *data;
} SE_ObjCallback;

static std::vector<SE_ObjCallback> objCallback;

static void log_callback(const char *str)
{
	if (logToConsole)
	{
		printf("%s\n", str);
	}
}

static void resetScenario(void)
{
	if (player != nullptr)
	{
		delete player;
		player = nullptr;
	}
	if (argv_)
	{
		for (int i = 0; i < argc_; i++)
		{
			free(argv_[i]);
		}
		free(argv_);
		argv_ = 0;
		argc_ = 0;
	}
	args_v.clear();
}

static void AddArgument(const char *str, bool split = true)
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
	argc_ = (int)args_v.size();
	argv_ = (char **)malloc(argc_ * sizeof(char *));
	std::string argument_list;
	for (int i = 0; i < argc_; i++)
	{
		argv_[i] = (char *)malloc((args_v[i].size() + 1) * sizeof(char));
		strcpy(argv_[i], args_v[i].c_str());
		argument_list += std::string(" ") + argv_[i];
	}
	LOG("Player arguments: %s", argument_list.c_str());
}

static void copyStateFromScenarioGateway(SE_ScenarioObjectState *state, ObjectStateStruct *gw_state)
{
	state->id = gw_state->info.id;
	state->model_id = gw_state->info.model_id;
	state->ctrl_type = gw_state->info.ctrl_type;
	//	strncpy(state->name, gw_state->info.name, NAME_LEN);
	state->timestamp = gw_state->info.timeStamp;
	state->x = (float)gw_state->pos.GetX();
	state->y = (float)gw_state->pos.GetY();
	state->z = (float)gw_state->pos.GetZ();
	state->h = (float)gw_state->pos.GetH();
	state->p = (float)gw_state->pos.GetP();
	state->r = (float)gw_state->pos.GetR();
	state->speed = (float)gw_state->info.speed;
	state->roadId = (int)gw_state->pos.GetTrackId();
	state->junctionId = (int)gw_state->pos.GetJunctionId();
	state->t = (float)gw_state->pos.GetT();
	state->laneId = (int)gw_state->pos.GetLaneId();
	state->s = (float)gw_state->pos.GetS();
	state->laneOffset = (float)gw_state->pos.GetOffset();
	state->centerOffsetX = gw_state->info.boundingbox.center_.x_;
	state->centerOffsetY = gw_state->info.boundingbox.center_.y_;
	state->centerOffsetZ = gw_state->info.boundingbox.center_.z_;
	state->width = gw_state->info.boundingbox.dimensions_.width_;
	state->length = gw_state->info.boundingbox.dimensions_.length_;
	state->height = gw_state->info.boundingbox.dimensions_.height_;
	state->objectType = gw_state->info.obj_type;
	state->objectCategory = gw_state->info.obj_category;
}

static int copyOverrideActionListfromScenarioEngine(SE_OverrideActionList *list, Object *obj)
{
	if (obj == 0)
	{
		return -1;
	}

	list->throttle.active = obj->overrideActionList[Object::OverrideType::OVERRIDE_THROTTLE].active;
	list->throttle.value = obj->overrideActionList[Object::OverrideType::OVERRIDE_THROTTLE].value;
	list->brake.active = obj->overrideActionList[Object::OverrideType::OVERRIDE_BRAKE].active;
	list->brake.value = obj->overrideActionList[Object::OverrideType::OVERRIDE_BRAKE].value;
	list->clutch.active = obj->overrideActionList[Object::OverrideType::OVERRIDE_CLUTCH].active;
	list->clutch.value = obj->overrideActionList[Object::OverrideType::OVERRIDE_CLUTCH].value;
	list->parkingBrake.active = obj->overrideActionList[Object::OverrideType::OVERRIDE_PARKING_BRAKE].active;
	list->parkingBrake.value = obj->overrideActionList[Object::OverrideType::OVERRIDE_PARKING_BRAKE].value;
	list->steeringWheel.active = obj->overrideActionList[Object::OverrideType::OVERRIDE_STEERING_WHEEL].active;
	list->steeringWheel.value = obj->overrideActionList[Object::OverrideType::OVERRIDE_STEERING_WHEEL].value;
	list->gear.active = obj->overrideActionList[Object::OverrideType::OVERRIDE_GEAR].active;
	list->gear.value = obj->overrideActionList[Object::OverrideType::OVERRIDE_GEAR].value;

	return 0;
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

	if (pos->GetProbeInfo(lookahead_distance, &s_data, (roadmanager::Position::LookAheadMode)lookAheadMode) != roadmanager::Position::ErrorCode::ERROR_GENERIC)
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
		r_data->roadId = (int)s_data.road_lane_info.roadId;
		r_data->junctionId = (int)s_data.road_lane_info.junctionId;
		r_data->laneId = (int)s_data.road_lane_info.laneId;
		r_data->laneOffset = (float)s_data.road_lane_info.laneOffset;
		r_data->s = (float)s_data.road_lane_info.s;
		r_data->t = (float)s_data.road_lane_info.t;

// Add visualization of forward looking road sensor probe
#ifdef _USE_OSG
		if (player->viewer_ && player->viewer_->entities_[object_id]->GetType() == viewer::EntityModel::EntityType::VEHICLE)
		{
			viewer::CarModel *model = (viewer::CarModel *)player->viewer_->entities_[object_id];
			model->steering_sensor_->Show();
			player->viewer_->SensorSetPivotPos(model->steering_sensor_, pos->GetX(), pos->GetY(), pos->GetZ());
			player->viewer_->SensorSetTargetPos(model->steering_sensor_,
												s_data.road_lane_info.pos[0],
												s_data.road_lane_info.pos[1],
												s_data.road_lane_info.pos[2]);
			player->viewer_->UpdateSensor(model->steering_sensor_);
		}
#endif

		if (pos->GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROAD))
		{
			return static_cast<int>(roadmanager::Position::ErrorCode::ERROR_END_OF_ROAD);
		}
		else if (pos->GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROUTE))
		{
			return static_cast<int>(roadmanager::Position::ErrorCode::ERROR_END_OF_ROUTE);
		}
		else
		{
			return 0;  // OK
		}
	}

	return -1;  // Error
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
	Object *ghost = 0;
	if (obj->GetAssignedControllerType() != Controller::Type::CONTROLLER_TYPE_DEFAULT)
	{
		ghost = obj->GetGhost();
		if (ghost == 0)
		{
			LOG("Ghost object not available for object id %d", object_id);
			return -1;
		}
	}

	double x, y, z;
	int index_out;

	if (ghost->trail_.FindClosestPoint(obj->pos_.GetX(), obj->pos_.GetY(), obj->trail_closest_pos_,
									   obj->trail_follow_index_, obj->trail_follow_index_) == 0)
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

	roadmanager::TrajVertex trailPos;
	trailPos.h = (float)obj->pos_.GetH(); // Set default trail heading aligned with road - in case trail is less than two points (no heading)
	ghost->trail_.FindPointAhead(obj->trail_closest_pos_.s, lookahead_distance, trailPos, index_out, obj->trail_follow_index_);

	roadmanager::Position pos(trailPos.x, trailPos.y, 0, 0, 0, 0);
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
	r_data->trail_heading = (float)trailPos.h;
	r_data->road_pitch = (float)s_data.road_lane_info.pitch;
	r_data->road_roll = (float)s_data.road_lane_info.roll;
	r_data->speed_limit = (float)s_data.road_lane_info.speed_limit;

	*speed_ghost = (float)trailPos.speed;

	// Update object sensor position for visualization
	if (obj->sensor_pos_)
	{
		obj->sensor_pos_[0] = trailPos.x;
		obj->sensor_pos_[1] = trailPos.y;
		obj->sensor_pos_[2] = trailPos.z;
	}

	return 0;
}

static int InitScenario()
{
	// Harmonize parsing and printing of floating point numbers. I.e. 1.57e+4 == 15700.0 not 15,700.0 or 1 or 1.57
	std::setlocale(LC_ALL, "C.UTF-8");

	Logger::Inst().SetCallback(log_callback);
	Logger::Inst().OpenLogfile();
	Logger::Inst().LogVersion();

	ConvertArguments();

	// Create scenario engine
	try
	{
		// Initialize the scenario engine and viewer
		player = new ScenarioPlayer(argc_, argv_);
	}
	catch (const std::exception &e)
	{
		LOG(e.what());
		resetScenario();
		return -1;
	}

	return 0;
}

extern "C"
{
	SE_DLL_API int SE_AddPath(const char *path)
	{
		SE_Env::Inst().AddPath(path);
		return 0;
	}

	SE_DLL_API void SE_ClearPaths()
	{
		SE_Env::Inst().ClearPaths();
	}

	SE_DLL_API void SE_SetLogFilePath(const char *logFilePath)
	{
		SE_Env::Inst().SetLogFilePath(logFilePath);
	}

	SE_DLL_API unsigned int SE_GetSeed()
	{
		return SE_Env::Inst().GetSeed();
	}

	SE_DLL_API void SE_SetSeed(unsigned int seed)
	{
		SE_Env::Inst().SetSeed(seed);
	}

	SE_DLL_API int SE_SetOSITolerances(double maxLongitudinalDistance, double maxLateralDeviation)
	{
		SE_Env::Inst().SetOSIMaxLongitudinalDistance(maxLongitudinalDistance);
		SE_Env::Inst().SetOSIMaxLateralDeviation(maxLateralDeviation);
		return 0;
	}

	SE_DLL_API int SE_InitWithArgs(int argc, char *argv[])
	{
		resetScenario();

		for (int i = 0; i < argc; i++)
		{
			AddArgument(argv[i]);
		}

		return InitScenario();
	}

	SE_DLL_API int SE_InitWithString(const char *oscAsXMLString, int disable_ctrls, int use_viewer, int threads, int record)
	{
#ifndef _USE_OSG
		if (use_viewer)
		{
			LOG("use_viewer flag set, but no viewer available (compiled without -D _USE_OSG");
		}
#endif
		resetScenario();

		AddArgument("viewer"); // name of application
		AddArgument("--osc_str");
		AddArgument(oscAsXMLString, false);

		if (record)
		{
			AddArgument("--record");
			AddArgument("simulation.dat", false);
		}
		if (use_viewer)
		{
			AddArgument("--window 60 60 800 400", true);
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

	SE_DLL_API void SE_RegisterParameterDeclarationCallback(void (*fnPtr)(void*), void* user_data)
	{
		RegisterParameterDeclarationCallback(fnPtr, user_data);
	}

	SE_DLL_API int SE_Init(const char* oscFilename, int disable_ctrls, int use_viewer, int threads, int record)
	{
#ifndef _USE_OSG
		if (use_viewer)
		{
			LOG("use_viewer flag set, but no viewer available (compiled without -D _USE_OSG");
		}
#endif
		resetScenario();

		AddArgument("viewer"); // name of application
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
			AddArgument("--window 60 60 800 400", true);
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
		int quit_flag = -1;

		if (player != nullptr)
		{
			if (player->IsQuitRequested())
			{
				quit_flag = 1;
			}
			else
			{
				quit_flag = 0;
			}
		}

		return quit_flag;
	}

	SE_DLL_API const char *SE_GetODRFilename()
	{
		static std::string returnString;
		if (player == nullptr)
		{
			return 0;
		}
		returnString = player->scenarioEngine->getOdrFilename().c_str();
		return returnString.c_str();
	}

	SE_DLL_API const char *SE_GetSceneGraphFilename()
	{
		static std::string returnString;

		if (player == nullptr)
		{
			return 0;
		}

		returnString = player->scenarioEngine->getSceneGraphFilename().c_str();
		return returnString.c_str();
	}

	SE_DLL_API int SE_GetNumberOfParameters()
	{
		if (player == nullptr)
		{
			return -1;
		}

		return player->GetNumberOfParameters();
	}

	SE_DLL_API const char *SE_GetParameterName(int index, int *type)
	{
		static std::string returnString;

		if (player == nullptr)
		{
			return 0;
		}

		returnString = player->GetParameterName(index, (OSCParameterDeclarations::ParameterType*)type);

		return returnString.c_str();
	}

	SE_DLL_API int SE_GetNumberOfProperties(int index)
	{
		if (player != nullptr && index >= 0 && index < player->scenarioGateway->getNumberOfObjects())
		{
			return player->GetNumberOfProperties(index);
		}

		return -1;
	}

	SE_DLL_API const char *SE_GetObjectPropertyName(int index, int propertyIndex)
	{
		if (player != nullptr && index >= 0 && index < player->scenarioGateway->getNumberOfObjects())
		{
			int number = player->GetNumberOfProperties(index);
			if (number > 0 && propertyIndex < number && propertyIndex>=0)
			{
				return player->GetPropertyName(index, propertyIndex);
			}
		}

		return "";
	}

	SE_DLL_API const char *SE_GetObjectPropertyValue(int index, const char *objectPropertyName)
	{
		if (player != nullptr && index >= 0 && index < player->scenarioGateway->getNumberOfObjects())
		{
			for (int i = 0; i < player->GetNumberOfProperties(index); i++)
			{
				if (strcmp(player->GetPropertyName(index, i), objectPropertyName) == 0)
				{
					return player->GetPropertyValue(index, i);
				}
			}
		}

		return "";
	}

	SE_DLL_API int SE_SetParameter(SE_Parameter parameter)
	{
		return ScenarioReader::parameters.setParameterValue(parameter.name, parameter.value);
	}

	SE_DLL_API int SE_GetParameter(SE_Parameter *parameter)
	{
		return ScenarioReader::parameters.getParameterValue(parameter->name, parameter->value);
	}

	SE_DLL_API int SE_GetParameterInt(const char *parameterName, int *value)
	{
		return ScenarioReader::parameters.getParameterValueInt(parameterName, *value);
	}

	SE_DLL_API int SE_GetParameterDouble(const char *parameterName, double *value)
	{
		return ScenarioReader::parameters.getParameterValueDouble(parameterName, *value);
	}

	SE_DLL_API int SE_GetParameterString(const char *parameterName, const char **value)
	{
		return ScenarioReader::parameters.getParameterValueString(parameterName, *value);
	}

	SE_DLL_API int SE_GetParameterBool(const char *parameterName, bool *value)
	{
		return ScenarioReader::parameters.getParameterValueBool(parameterName, *value);
	}

	SE_DLL_API int SE_SetParameterInt(const char *parameterName, int value)
	{
		return ScenarioReader::parameters.setParameterValue(parameterName, value);
	}

	SE_DLL_API int SE_SetParameterDouble(const char *parameterName, double value)
	{
		return ScenarioReader::parameters.setParameterValue(parameterName, value);
	}

	SE_DLL_API int SE_SetParameterString(const char *parameterName, const char *value)
	{
		return ScenarioReader::parameters.setParameterValue(parameterName, value);
	}

	SE_DLL_API int SE_SetParameterBool(const char *parameterName, bool value)
	{
		return ScenarioReader::parameters.setParameterValue(parameterName, value);
	}

	SE_DLL_API void *SE_GetODRManager()
	{
		if (player != nullptr)
		{
			return (void *)player->GetODRManager();
		}

		return NULL;
	}

	SE_DLL_API void SE_Close()
	{
		resetScenario();
	}

	SE_DLL_API void SE_LogToConsole(bool mode)
	{
		logToConsole = mode;
	}

	SE_DLL_API int SE_OpenOSISocket(const char *ipaddr)
	{
		if (player == nullptr)
		{
			return -1;
		}

#ifdef _USE_OSI
		player->osiReporter->OpenSocket(ipaddr);
#endif  // USE_OSI
		return 0;
	}

	SE_DLL_API int SE_Step()
	{
		if (player != nullptr)
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
		if (player != nullptr)
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
		if (player == nullptr)
		{
			return 0.0f;
		}

		return (float)player->scenarioEngine->getSimulationTime();
	}

	SE_DLL_API float SE_GetSimTimeStep()
	{
		if (player == nullptr)
		{
			return 0.0f;
		}

		static __int64 time_stamp = 0;

		return (float)SE_getSimTimeStep(time_stamp, 0.001, 0.1);
	}

	SE_DLL_API void RM_SetAlignMode(int id, int mode)
	{
		if (player != nullptr)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioEngine->entities.object_[id]->pos_.SetAlignMode((roadmanager::Position::ALIGN_MODE)mode);
			}
		}
	}

	SE_DLL_API void RM_SetAlignModeH(int id, int mode)
	{
		if (player != nullptr)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioEngine->entities.object_[id]->pos_.SetAlignModeH((roadmanager::Position::ALIGN_MODE)mode);
			}
		}
	}

	SE_DLL_API void RM_SetAlignModeP(int id, int mode)
	{
		if (player != nullptr)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioEngine->entities.object_[id]->pos_.SetAlignModeP((roadmanager::Position::ALIGN_MODE)mode);
			}
		}
	}

	SE_DLL_API void RM_SetAlignModeR(int id, int mode)
	{
		if (player != nullptr)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioEngine->entities.object_[id]->pos_.SetAlignModeR((roadmanager::Position::ALIGN_MODE)mode);
			}
		}
	}

	SE_DLL_API void RM_SetAlignModeZ(int id, int mode)
	{
		if (player!= nullptr)
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioEngine->entities.object_[id]->pos_.SetAlignModeZ((roadmanager::Position::ALIGN_MODE)mode);
			}
		}
	}

	SE_DLL_API int SE_ReportObjectPos(int id, float timestamp, float x, float y, float z, float h, float p, float r, float speed)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_, obj->model_id_,
					obj->GetActivatedControllerType(), obj->boundingbox_, static_cast<int>(obj->scaleMode_), obj->visibilityMask_,
					timestamp, speed, obj->wheel_angle_, obj->wheel_rot_, x, y, z, h, p, r);
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectPosXYH(int id, float timestamp, float x, float y, float h, float speed)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_, obj->model_id_,
					obj->GetActivatedControllerType(), obj->boundingbox_, static_cast<int>(obj->scaleMode_), obj->visibilityMask_,
					timestamp, speed, obj->wheel_angle_, obj->wheel_rot_, x, y, h);
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectRoadPos(int id, float timestamp, int roadId, int laneId, float laneOffset, float s, float speed)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_, obj->model_id_,
					obj->GetActivatedControllerType(), obj->boundingbox_, static_cast<int>(obj->scaleMode_), obj->visibilityMask_,
					timestamp, speed, obj->wheel_angle_, obj->wheel_rot_, roadId, laneId, laneOffset, s);
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectSpeed(int id, float speed)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				Object *obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_, obj->model_id_,
					obj->GetActivatedControllerType(), obj->boundingbox_, static_cast<int>(obj->scaleMode_),
					obj->visibilityMask_, 0.0, speed, obj->wheel_angle_, obj->wheel_rot_, &obj->pos_);
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
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_, obj->model_id_,
					obj->GetActivatedControllerType(), obj->boundingbox_, static_cast<int>(obj->scaleMode_), obj->visibilityMask_,
					0.0, obj->GetSpeed(), obj->wheel_angle_, obj->wheel_rot_, obj->pos_.GetTrackId(), t, obj->pos_.GetS());
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
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id < player->scenarioEngine->entities.object_.size())
			{
				// reuse some values
				Object *obj = player->scenarioEngine->entities.object_[id];
				player->scenarioGateway->reportObject(id, obj->name_, obj->type_, obj->category_, obj->model_id_,
													  obj->GetActivatedControllerType(), obj->boundingbox_, static_cast<int>(obj->scaleMode_),
													  obj->visibilityMask_, 0.0, obj->GetSpeed(), obj->wheel_angle_,
													  obj->wheel_rot_, obj->pos_.GetTrackId(), obj->pos_.GetLaneId(), laneOffset, obj->pos_.GetS());
			}
			else
			{
				return -1;
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectVel(int id, float timestamp, float x_vel, float y_vel, float z_vel)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id >= 0 && id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioGateway->updateObjectVel(id, 0.0, x_vel, y_vel, z_vel);
				// Also update accelerations directly in scenario object, in case we're in a callback
				player->scenarioEngine->entities.object_[id]->SetVel(x_vel, y_vel, z_vel);
			}
			else
			{
				return -1;
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectAngularVel(int id, float timestamp, float h_rate, float p_rate, float r_rate)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id >= 0 && id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioGateway->updateObjectAngularVel(id, 0.0, h_rate, p_rate, r_rate);
				// Also update accelerations directly in scenario object, in case we're in a callback
				player->scenarioEngine->entities.object_[id]->SetAngularVel(h_rate, p_rate, r_rate);
			}
			else
			{
				return -1;
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectAcc(int id, float timestamp, float x_acc, float y_acc, float z_acc)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id >= 0 && id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioGateway->updateObjectAcc(id, 0.0, x_acc, y_acc, z_acc);
				// Also update accelerations directly in scenario object, in case we're in a callback
				player->scenarioEngine->entities.object_[id]->SetAcc(x_acc, y_acc, z_acc);
			}
			else
			{
				return -1;
			}
		}

		return 0;
	}

	SE_DLL_API int SE_ReportObjectAngularAcc(int id, float timestamp, float h_acc, float p_acc, float r_acc)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id >= 0 && id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioGateway->updateObjectAngularAcc(id, 0.0, h_acc, p_acc, r_acc);
				// Also update accelerations directly in scenario object, in case we're in a callback
				player->scenarioEngine->entities.object_[id]->SetAngularAcc(h_acc, p_acc, r_acc);
			}
			else
			{
				return -1;
			}
		}

		return 0;
	}

	SE_DLL_API int SE_SetLockOnLane(int id, bool mode)
	{
		if (player == nullptr)
		{
			return -1;
		}
		else
		{
			if (id >= 0 && id < player->scenarioEngine->entities.object_.size())
			{
				player->scenarioGateway->getObjectStatePtrByIdx(id)->state_.pos.SetLockOnLane(mode);
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
		if (player == nullptr)
		{
			return -1;
		}

		return player->scenarioGateway->getNumberOfObjects();
	}

	SE_DLL_API int SE_GetObjectState(int index, SE_ScenarioObjectState *state)
	{
		if (player != nullptr && index >= 0 && index < player->scenarioGateway->getNumberOfObjects())
		{
			copyStateFromScenarioGateway(state, &player->scenarioGateway->getObjectStatePtrByIdx(index)->state_);
			return 0;
		}

		return -1;
	}

	SE_DLL_API int SE_GetOverrideActionStatus(int objectId, SE_OverrideActionList *list)
	{
		if (player != nullptr)
		{
			return copyOverrideActionListfromScenarioEngine(list, player->scenarioEngine->entities.GetObjectById(objectId));
		}

		return -1;
	}

	SE_DLL_API const char* SE_GetObjectTypeName(int index)
	{
		static std::string returnString;
		if (player != nullptr && index >= 0 && index < player->scenarioGateway->getNumberOfObjects())
		{
			returnString = player->scenarioEngine->entities.object_[index]->GetTypeName();
			return returnString.c_str();
		}

		return 0;
	}

	SE_DLL_API const char *SE_GetObjectName(int index)
	{
		static std::string returnString;
		if (player != nullptr && index >= 0 && index < player->scenarioGateway->getNumberOfObjects())
		{
			returnString = player->scenarioGateway->getObjectStatePtrByIdx(index)->state_.info.name;
			return returnString.c_str();
		}

		return 0;
	}

	SE_DLL_API const char* SE_GetObjectModelFileName(int index)
	{
		static std::string returnString;
		if (player != nullptr && index >= 0 && index < player->scenarioGateway->getNumberOfObjects())
		{
			returnString = player->scenarioEngine->entities.object_[index]->GetModelFileName();
			return returnString.c_str();
		}

		return 0;
	}

	SE_DLL_API const char *SE_GetOSIGroundTruth(int *size)
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return player->osiReporter->GetOSIGroundTruth(size);
#endif  // USE_OSI
		}

		*size = 0;
		return 0;
	}

	SE_DLL_API const char *SE_GetOSIGroundTruthRaw()
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return (const char *)player->osiReporter->GetOSIGroundTruthRaw();
#endif  // USE_OSI
		}

		return 0;
	}

	SE_DLL_API int SE_SetOSISensorDataRaw(const char* sensordata)
	{
		if (player != nullptr)
		{
#ifdef _USE_OSG
			if (player->viewer_)
			{
	#ifdef _USE_OSI
				const osi3::SensorData *sd = reinterpret_cast<const osi3::SensorData *>(sensordata);
				player->osiReporter->CreateSensorViewFromSensorData(*sd);
				if(player->osiReporter->GetSensorView())
				{
					if(player->OSISensorDetection)
					{
						player->OSISensorDetection->Update(player->osiReporter->GetSensorView());
					}
				}
	#endif  // USE_OSI
			}
#endif
		}
		return 0;
	}

	SE_DLL_API const char *SE_GetOSIRoadLane(int *size, int object_id)
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return player->osiReporter->GetOSIRoadLane(player->scenarioGateway->objectState_, size, object_id);
#endif  // USE_OSI
		}

		*size = 0;
		return 0;
	}

	SE_DLL_API const char *SE_GetOSILaneBoundary(int *size, int global_id)
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return player->osiReporter->GetOSIRoadLaneBoundary(size, global_id);
#endif  // USE_OSI
		}

		*size = 0;
		return 0;
	}

	SE_DLL_API void SE_GetOSILaneBoundaryIds(int object_id, SE_LaneBoundaryId *ids)
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			std::vector<int> ids_vector;
			player->osiReporter->GetOSILaneBoundaryIds(player->scenarioGateway->objectState_, ids_vector, object_id);
			if (!ids_vector.empty())
			{
				ids->far_left_lb_id = ids_vector[0];
				ids->left_lb_id = ids_vector[1];
				ids->right_lb_id = ids_vector[2];
				ids->far_right_lb_id = ids_vector[3];
			}
#endif  // USE_OSI
		}
		return;
	}

	SE_DLL_API int SE_ClearOSIGroundTruth()
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return player->osiReporter->ClearOSIGroundTruth();
#endif  // USE_OSI
		}

		return 0;
	}

	SE_DLL_API int SE_UpdateOSIGroundTruth()
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return player->osiReporter->UpdateOSIGroundTruth(player->scenarioGateway->objectState_);
#endif  // USE_OSI
		}

		return 0;
	}

	SE_DLL_API int SE_UpdateOSIStaticGroundTruth()
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return player->osiReporter->UpdateOSIStaticGroundTruth(player->scenarioGateway->objectState_);
#endif  // USE_OSI
		}

		return 0;
	}

	SE_DLL_API int SE_UpdateOSIDynamicGroundTruth(bool reportGhost)
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return player->osiReporter->UpdateOSIDynamicGroundTruth(player->scenarioGateway->objectState_, reportGhost);
#endif  // USE_OSI
		}

		return 0;
	}

	SE_DLL_API const char *SE_GetOSISensorDataRaw()
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return (const char *)player->osiReporter->GetOSISensorDataRaw();
#endif  // USE_OSI
		}

		return 0;
	}

	SE_DLL_API bool SE_OSIFileOpen(const char *filename)
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			return player->osiReporter->OpenOSIFile(filename);
#endif  // USE_OSI
		}

		return false;
	}

	SE_DLL_API bool SE_OSIFileWrite(bool flush)
	{
		bool retval = false;

		if (player != nullptr)
		{
#ifdef _USE_OSI
			retval = player->osiReporter->WriteOSIFile();
			if (flush)
			{
				player->osiReporter->FlushOSIFile();
			}
#endif  // USE_OSI
		}

		return retval;
	}

	SE_DLL_API int SE_OSISetTimeStamp(unsigned long long int nanoseconds)
	{
		if (player != nullptr)
		{
#ifdef _USE_OSI
			player->osiReporter->SetOSITimeStampExplicit(nanoseconds);
			return 0;
#endif  // USE_OSI
		}

		return -1;
	}

	SE_DLL_API void SE_LogMessage(const char *message)
	{
		LOG(message);
	}

	SE_DLL_API int SE_ObjectHasGhost(int index)
	{
		if (player == nullptr)
		{
			return -1;
		}

		Object *ghost = 0;
		if (player->scenarioEngine->entities.object_[index]->GetAssignedControllerType() != Controller::Type::CONTROLLER_TYPE_DEFAULT)
		{
			ghost = player->scenarioEngine->entities.object_[index]->GetGhost();
		}
		return ghost != 0 ? 1 : 0;
	}

	SE_DLL_API int SE_GetObjectGhostState(int index, SE_ScenarioObjectState *state)
	{
		Object *ghost = 0;

		if (player == nullptr)
		{
			return -1;
		}

		if (index < player->scenarioEngine->entities.object_.size())
		{
			for (size_t i = 0; i < player->scenarioEngine->entities.object_.size(); i++) // ghost index always higher than external buddy
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

	SE_DLL_API int SE_GetObjectStates(int *nObjects, SE_ScenarioObjectState *state)
	{
		int i;
		*nObjects = 0;

		if (player == nullptr)
		{
			return -1;
		}

		for (i = 0; i < *nObjects && i < player->scenarioGateway->getNumberOfObjects(); i++)
		{
			copyStateFromScenarioGateway(&state[i], &player->scenarioGateway->getObjectStatePtrByIdx(i)->state_);
		}
		*nObjects = i;

		return 0;
	}

	SE_DLL_API int SE_AddObjectSensor(int object_id, float x, float y, float z, float h, float rangeNear, float rangeFar, float fovH, int maxObj)
	{
		if (player == nullptr)
		{
			return -1;
		}

		if (object_id < 0 || object_id >= player->scenarioEngine->entities.object_.size())
		{
			LOG("Invalid object_id (%d/%d)", object_id, player->scenarioEngine->entities.object_.size());
			return -1;
		}

		player->AddObjectSensor(object_id, x, y, z, h, rangeNear, rangeFar, fovH, maxObj);

		return 0;
	}

	SE_DLL_API int SE_ViewSensorData(int object_id)
	{
		if (player)
		{

			if (object_id < 0 || object_id >= player->scenarioEngine->entities.object_.size())
			{
				LOG("Invalid object_id (%d/%d)", object_id, player->scenarioEngine->entities.object_.size());
				return -1;
			}

			player->AddOSIDetection(object_id);
			player->ShowObjectSensors(false);

			return 0;
		}

		return -1;
	}

	SE_DLL_API void SE_DisableOSIFile()
	{
		if (player == nullptr)
		{
			return;
		}

		player->SetOSIFileStatus(false);
	}

	SE_DLL_API void SE_EnableOSIFile(const char *filename)
	{
		if (player != nullptr)
		{
			player->SetOSIFileStatus(true, filename);
		}
	}

	SE_DLL_API int SE_FetchSensorObjectList(int sensor_id, int *list)
	{
		if (player != nullptr)
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

	SE_DLL_API int SE_GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo *data, int lookAheadMode, bool inRoadDrivingDirection)
	{
		if (player == nullptr || object_id >= player->scenarioGateway->getNumberOfObjects())
		{
			return -1;
		}

		Object *obj = player->scenarioEngine->entities.object_[object_id];

		float adjustedLookaheadDistance = lookahead_distance;

		if (!inRoadDrivingDirection)
		{
			// Find out what direction to look in
			if (fabs(obj->pos_.GetHRelativeDrivingDirection()) > M_PI_2)
			{
				adjustedLookaheadDistance = -lookahead_distance;
			}
		}

		if (GetRoadInfoAtDistance(object_id, adjustedLookaheadDistance, data, lookAheadMode) != 0)
		{
			return -1;
		}

		return 0;
	}

	SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *data, float *speed_ghost)
	{
		if (player == nullptr || object_id >= player->scenarioGateway->getNumberOfObjects())
		{
			return -1;
		}

		if (GetRoadInfoAlongGhostTrail(object_id, lookahead_distance, data, speed_ghost) != 0)
		{
			return -1;
		}

		return 0;
	}

	void objCallbackFn(ObjectStateStruct *state, void *my_data)
	{
		for (size_t i = 0; i < objCallback.size(); i++)
		{
			if (objCallback[i].id == state->info.id)
			{
				SE_ScenarioObjectState se_state;
				copyStateFromScenarioGateway(&se_state, state);
				objCallback[i].func(&se_state, my_data);
			}
		}
	}

	SE_DLL_API void SE_RegisterObjectCallback(int object_id, void (*fnPtr)(SE_ScenarioObjectState *, void *), void *user_data)
	{
		if (player == nullptr || object_id >= player->scenarioGateway->getNumberOfObjects())
		{
			return;
		}
		SE_ObjCallback cb;
		cb.id = object_id;
		cb.func = fnPtr;
		objCallback.push_back(cb);
		player->RegisterObjCallback(object_id, objCallbackFn, user_data);
	}

	SE_DLL_API int SE_GetNumberOfRoadSigns(int road_id)
	{
		if (player != nullptr)
		{
			roadmanager::Road *road = player->odr_manager->GetRoadById(road_id);
			if (road != NULL)
			{
				return road->GetNumberOfSignals();
			}
		}
		return 0;
	}

	SE_DLL_API int SE_GetRoadSign(int road_id, int index, SE_RoadSign *road_sign)
	{
		static std::string returnString;

		if (player != nullptr)
		{
			roadmanager::Road *road = player->odr_manager->GetRoadById(road_id);
			if (road != NULL)
			{
				roadmanager::Signal *s = road->GetSignal(index);

				if (s)
				{
					// Resolve global cartesian position (x, y, z, h) from the road coordinate
					roadmanager::Position pos;
					pos.SetTrackPos(road_id, s->GetS(), s->GetT());

					road_sign->id = s->GetId();
					returnString = s->GetName();
					road_sign->name = returnString.c_str();
					road_sign->x = (float)pos.GetX();
					road_sign->y = (float)pos.GetY();
					road_sign->z = (float)pos.GetZ();
					road_sign->h = (float)pos.GetH();
					road_sign->s = (float)pos.GetS();
					road_sign->t = (float)pos.GetT();
					road_sign->orientation = s->GetOrientation() == roadmanager::Signal::Orientation::NEGATIVE ? -1 : 1;
					road_sign->z_offset = (float)s->GetZOffset();
					road_sign->length = (float)s->GetLength();
					road_sign->height = (float)s->GetHeight();
					road_sign->width = (float)s->GetWidth();

					return 0;
				}
			}
		}

		// Couldn't find the sign
		return -1;
	}

	SE_DLL_API int SE_GetNumberOfRoadSignValidityRecords(int road_id, int index)
	{
		if (player != nullptr)
		{
			roadmanager::Road* road = player->odr_manager->GetRoadById(road_id);
			if (road != nullptr)
			{
				roadmanager::Signal* s = road->GetSignal(index);
				return (int)s->validity_.size();
			}
		}

		return 0;
	}

	SE_DLL_API int SE_GetRoadSignValidityRecord(int road_id, int signIndex, int validityIndex, SE_RoadObjValidity* validity)
	{
		if (player != nullptr)
		{
			roadmanager::Road* road = player->odr_manager->GetRoadById(road_id);
			if (road != NULL)
			{
				roadmanager::Signal* s = road->GetSignal(signIndex);
				if (validityIndex >= 0 && validityIndex < s->validity_.size())
				{
					validity->fromLane = s->validity_[validityIndex].fromLane_;
					validity->toLane = s->validity_[validityIndex].toLane_;
					return 0;
				}
			}
		}

		return -1;
	}

	SE_DLL_API void SE_ViewerShowFeature(int featureType, bool enable)
	{
#ifdef _USE_OSG
		if (player != nullptr && player->viewer_)
		{
			player->viewer_->SetNodeMaskBits(featureType, enable ? featureType : 0x0);
		}
#endif
	}

	// Simple vehicle
	SE_DLL_API void *SE_SimpleVehicleCreate(float x, float y, float h, float length, float speed)
	{
		vehicle::Vehicle *v = new vehicle::Vehicle(x, y, h, length, speed);
		return (void *)v;
	}

	SE_DLL_API void SE_SimpleVehicleDelete(void *handleSimpleVehicle)
	{
		if (handleSimpleVehicle)
		{
			free((vehicle::Vehicle *)handleSimpleVehicle);
			handleSimpleVehicle = 0;
		}
	}

	SE_DLL_API void SE_SimpleVehicleControlBinary(void *handleSimpleVehicle, double dt, int throttle, int steering)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}

		((vehicle::Vehicle *)handleSimpleVehicle)->DrivingControlBinary(dt, (vehicle::THROTTLE)throttle, (vehicle::STEERING)steering);
	}

	SE_DLL_API void SE_SimpleVehicleControlAnalog(void *handleSimpleVehicle, double dt, double throttle, double steering)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}

		((vehicle::Vehicle *)handleSimpleVehicle)->DrivingControlAnalog(dt, throttle, steering);
	}

	SE_DLL_API void SE_SimpleVehicleControlTarget(void* handleSimpleVehicle, double dt, double target_speed, double heading_to_target)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}

		((vehicle::Vehicle*)handleSimpleVehicle)->DrivingControlTarget(dt, target_speed, heading_to_target);
	}

	SE_DLL_API void SE_SimpleVehicleSetMaxSpeed(void *handleSimpleVehicle, float speed)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		((vehicle::Vehicle *)handleSimpleVehicle)->SetMaxSpeed(speed/3.6);
	}

	SE_DLL_API void SE_SimpleVehicleSetMaxAcceleration(void* handleSimpleVehicle, float maxAcceleration)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		((vehicle::Vehicle*)handleSimpleVehicle)->SetMaxAcc(maxAcceleration);
	}

	SE_DLL_API void SE_SimpleVehicleSetEngineBrakeFactor(void* handleSimpleVehicle, float engineBrakeFactor)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		((vehicle::Vehicle*)handleSimpleVehicle)->SetEngineBrakeFactor(engineBrakeFactor);
	}

	SE_DLL_API void SE_SimpleVehicleSteeringScale(void* handleSimpleVehicle, float steeringScale)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		((vehicle::Vehicle*)handleSimpleVehicle)->SetSteeringScale(steeringScale);
	}

	SE_DLL_API void SE_SimpleVehicleSteeringReturnFactor(void* handleSimpleVehicle, float steeringReturnFactor)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		((vehicle::Vehicle*)handleSimpleVehicle)->SetSteeringReturnFactor(steeringReturnFactor);
	}

	SE_DLL_API void SE_SimpleVehicleSteeringRate(void* handleSimpleVehicle, float steeringRate)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		((vehicle::Vehicle*)handleSimpleVehicle)->SetSteeringRate(steeringRate);
	}

	SE_DLL_API void SE_SimpleVehicleGetState(void *handleSimpleVehicle, SE_SimpleVehicleState *state)
	{
		if (handleSimpleVehicle == 0)
		{
			return;
		}
		state->x = (float)((vehicle::Vehicle *)handleSimpleVehicle)->posX_;
		state->y = (float)((vehicle::Vehicle *)handleSimpleVehicle)->posY_;
		state->z = (float)((vehicle::Vehicle *)handleSimpleVehicle)->posZ_;
		state->h = (float)((vehicle::Vehicle *)handleSimpleVehicle)->heading_;
		state->p = (float)((vehicle::Vehicle *)handleSimpleVehicle)->pitch_;
		state->speed = (float)((vehicle::Vehicle *)handleSimpleVehicle)->speed_;
	}

	SE_DLL_API int SE_CaptureNextFrame()
	{
#ifdef _USE_OSG
		if (player)
		{
			player->CaptureNextFrame();
		}
		else
		{
			return -1;
		}

		return 0;
#else
		return -1;
#endif
	}

	SE_DLL_API int SE_CaptureContinuously(bool state)
	{
#ifdef _USE_OSG
		if (player)
		{
			player->CaptureContinuously(state);
		}
		else
		{
			return -1;
		}

		return 0;
#else
		return -1;
#endif
	}

	SE_DLL_API int SE_AddCustomCamera(double x, double y, double z, double h, double p)
	{
#ifdef _USE_OSG
		if (player)
		{
			player->AddCustomCamera(x, y, z, h, p);
		}
		else
		{
			return -1;
		}

		return 0;
#else
		return -1;
#endif
	}

SE_DLL_API int SE_GetNumberOfRoutePoints(int object_id)
{
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
	if (obj->pos_.GetRoute())
	{
		return obj->pos_.GetRoute()->all_waypoints_.size();
	}
	else
	{
		return 0;
	}
}

SE_DLL_API int SE_GetRoutePoint(int object_id, int route_index, SE_RouteInfo *routeinfo)
{
	if (player == 0)
	{
		return -1;
	}

	if (object_id >= player->scenarioGateway->getNumberOfObjects())
	{
		LOG("Object %d not available, only %d registered", object_id, player->scenarioGateway->getNumberOfObjects());
		return -1;
	}

	Object* obj = player->scenarioEngine->entities.object_[object_id];

	if (route_index >= obj->pos_.GetRoute()->all_waypoints_.size())
	{
		LOG("Requested waypoint index %d invalid, only %d registered", route_index, obj->pos_.GetRoute()->all_waypoints_.size());
		return -1;
	}

	routeinfo->x = obj->pos_.GetRoute()->all_waypoints_[route_index].GetX();
	routeinfo->y = obj->pos_.GetRoute()->all_waypoints_[route_index].GetY();
	routeinfo->z = obj->pos_.GetRoute()->all_waypoints_[route_index].GetZ();
	routeinfo->roadId = obj->pos_.GetRoute()->all_waypoints_[route_index].GetTrackId();
	routeinfo->junctionId = obj->pos_.GetRoute()->all_waypoints_[route_index].GetJunctionId();
	routeinfo->laneId = obj->pos_.GetRoute()->all_waypoints_[route_index].GetLaneId();
	routeinfo->laneOffset = obj->pos_.GetRoute()->all_waypoints_[route_index].GetOffset();
	routeinfo->s = obj->pos_.GetRoute()->all_waypoints_[route_index].GetS();
	routeinfo->t = obj->pos_.GetRoute()->all_waypoints_[route_index].GetT();

	return 0;
}
}