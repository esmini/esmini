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
#ifdef USE_OSI
#include "osi_sensordata.pb.h"
#endif
#include "vehicle.hpp"
#include "pugixml.hpp"
#include "ControllerExternal.hpp"
#include "OSCCondition.hpp"
#include "OSCManeuver.hpp"
#include "OSCParameterDistribution.hpp"

using namespace scenarioengine;

#define EGO_ID 0  // need to match appearing order in the OpenSCENARIO file

static ScenarioPlayer *player = 0;

static char                   **argv_ = 0;
static int                      argc_ = 0;
static std::vector<std::string> args_v;
static bool                     logToConsole = true;
static __int64                  time_stamp   = 0;

static struct
{
    int x;
    int y;
    int w;
    int h;
} winDim = {60, 60, 800, 400};

typedef struct
{
    int id;
    void (*func)(SE_ScenarioObjectState *, void *);
    void *data;
} SE_ObjCallback;

static std::vector<SE_ObjCallback> objCallback;

// List of 3D models populated from any found found model_ids.txt file
static std::map<int, std::string> entity_model_map;

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
        SE_Env::Inst().ClearModelFilenames();
    }
    if (argv_)
    {
        for (int i = 0; i < static_cast<int>(args_v.size()); i++)
        {
            free(argv_[i]);
        }
        free(argv_);
        argv_ = 0;
        argc_ = 0;
    }
    args_v.clear();

    // Reset (global) callbacks
    OSCCondition::conditionCallback        = nullptr;
    StoryBoardElement::stateChangeCallback = nullptr;

    time_stamp = 0;
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
    argc_ = static_cast<int>(args_v.size());
    argv_ = reinterpret_cast<char **>(malloc(args_v.size() * sizeof(char *)));
    std::string argument_list;
    for (unsigned int i = 0; i < static_cast<unsigned int>(argc_); i++)
    {
        argv_[i] = reinterpret_cast<char *>(malloc((args_v[i].size() + 1) * sizeof(char)));
        StrCopy(argv_[i], args_v[i].c_str(), static_cast<unsigned int>(args_v[i].size()) + 1);
        argument_list += std::string(" ") + argv_[i];
    }
    LOG("Player arguments: %s", argument_list.c_str());
}

static void copyStateFromScenarioGateway(SE_ScenarioObjectState *state, ObjectStateStruct *gw_state)
{
    state->id        = gw_state->info.id;
    state->model_id  = gw_state->info.model_id;
    state->ctrl_type = gw_state->info.ctrl_type;
    //	strncpy(state->name, gw_state->info.name, NAME_LEN);
    state->timestamp      = static_cast<float>(gw_state->info.timeStamp);
    state->x              = static_cast<float>(gw_state->pos.GetX());
    state->y              = static_cast<float>(gw_state->pos.GetY());
    state->z              = static_cast<float>(gw_state->pos.GetZ());
    state->h              = static_cast<float>(gw_state->pos.GetH());
    state->p              = static_cast<float>(gw_state->pos.GetP());
    state->r              = static_cast<float>(gw_state->pos.GetR());
    state->speed          = static_cast<float>(gw_state->info.speed);
    state->roadId         = gw_state->pos.GetTrackId();
    state->junctionId     = gw_state->pos.GetJunctionId();
    state->t              = static_cast<float>(gw_state->pos.GetT());
    state->laneId         = gw_state->pos.GetLaneId();
    state->s              = static_cast<float>(gw_state->pos.GetS());
    state->laneOffset     = static_cast<float>(gw_state->pos.GetOffset());
    state->centerOffsetX  = gw_state->info.boundingbox.center_.x_;
    state->centerOffsetY  = gw_state->info.boundingbox.center_.y_;
    state->centerOffsetZ  = gw_state->info.boundingbox.center_.z_;
    state->width          = gw_state->info.boundingbox.dimensions_.width_;
    state->length         = gw_state->info.boundingbox.dimensions_.length_;
    state->height         = gw_state->info.boundingbox.dimensions_.height_;
    state->objectType     = gw_state->info.obj_type;
    state->objectCategory = gw_state->info.obj_category;
    state->wheel_angle    = static_cast<float>(gw_state->info.wheel_angle);
    state->wheel_rot      = static_cast<float>(gw_state->info.wheel_rot);
}

static int getObjectById(int object_id, Object *&obj)
{
    if (player == nullptr)
    {
        return -1;
    }
    else
    {
        obj = player->scenarioEngine->entities_.GetObjectById(object_id);
        if (obj == nullptr)
        {
            LOG("Invalid object_id (%d)", object_id);
            return -1;
        }
    }
    return 0;
}
static int copyOverrideActionListfromScenarioEngine(SE_OverrideActionList *list, Object *obj)
{
    if (obj == 0)
    {
        return -1;
    }

    list->brake.active     = obj->overrideActionList[Object::OverrideType::OVERRIDE_BRAKE].active;
    list->brake.maxRate    = obj->overrideActionList[Object::OverrideType::OVERRIDE_BRAKE].maxRate;
    list->brake.type       = obj->overrideActionList[Object::OverrideType::OVERRIDE_BRAKE].type;
    list->brake.value      = obj->overrideActionList[Object::OverrideType::OVERRIDE_BRAKE].value;
    list->brake.value_type = obj->overrideActionList[Object::OverrideType::OVERRIDE_BRAKE].value_type;

    list->parkingBrake.active     = obj->overrideActionList[Object::OverrideType::OVERRIDE_PARKING_BRAKE].active;
    list->parkingBrake.maxRate    = obj->overrideActionList[Object::OverrideType::OVERRIDE_PARKING_BRAKE].maxRate;
    list->parkingBrake.type       = obj->overrideActionList[Object::OverrideType::OVERRIDE_PARKING_BRAKE].type;
    list->parkingBrake.value      = obj->overrideActionList[Object::OverrideType::OVERRIDE_PARKING_BRAKE].value;
    list->parkingBrake.value_type = obj->overrideActionList[Object::OverrideType::OVERRIDE_PARKING_BRAKE].value_type;

    list->gear.active     = obj->overrideActionList[Object::OverrideType::OVERRIDE_GEAR].active;
    list->gear.number     = obj->overrideActionList[Object::OverrideType::OVERRIDE_GEAR].number;
    list->gear.type       = obj->overrideActionList[Object::OverrideType::OVERRIDE_GEAR].type;
    list->gear.value_type = obj->overrideActionList[Object::OverrideType::OVERRIDE_GEAR].value_type;

    list->clutch.active  = obj->overrideActionList[Object::OverrideType::OVERRIDE_CLUTCH].active;
    list->clutch.value   = obj->overrideActionList[Object::OverrideType::OVERRIDE_CLUTCH].value;
    list->clutch.maxRate = obj->overrideActionList[Object::OverrideType::OVERRIDE_CLUTCH].maxRate;

    list->throttle.active  = obj->overrideActionList[Object::OverrideType::OVERRIDE_THROTTLE].active;
    list->throttle.value   = obj->overrideActionList[Object::OverrideType::OVERRIDE_THROTTLE].value;
    list->throttle.maxRate = obj->overrideActionList[Object::OverrideType::OVERRIDE_THROTTLE].maxRate;

    list->steeringWheel.active    = obj->overrideActionList[Object::OverrideType::OVERRIDE_STEERING_WHEEL].active;
    list->steeringWheel.maxRate   = obj->overrideActionList[Object::OverrideType::OVERRIDE_STEERING_WHEEL].maxRate;
    list->steeringWheel.maxTorque = obj->overrideActionList[Object::OverrideType::OVERRIDE_STEERING_WHEEL].maxTorque;
    list->steeringWheel.value     = obj->overrideActionList[Object::OverrideType::OVERRIDE_STEERING_WHEEL].value;

    return 0;
}

static int copyVehicleLightStatesListFromScenarioEngine(SE_VehicleLightState *states, Object *obj, int lightIndex)
{
    if (obj == 0)
    {
        return -1;
    }

    states->colorName = static_cast<int>(obj->vehicleLightActionStatusList[lightIndex].colorName);
    states->intensity = obj->vehicleLightActionStatusList[lightIndex].luminousIntensity;
    states->lightMode = static_cast<int>(obj->vehicleLightActionStatusList[lightIndex].mode);
    states->lightType = obj->vehicleLightActionStatusList[lightIndex].type;
    states->rgb[0]    = obj->vehicleLightActionStatusList[lightIndex].rgb[0];
    states->rgb[1]    = obj->vehicleLightActionStatusList[lightIndex].rgb[1];
    states->rgb[2]    = obj->vehicleLightActionStatusList[lightIndex].rgb[2];

    return 0;
}

static int GetRoadInfoAtDistance(int object_id, float lookahead_distance, SE_RoadInfo *r_data, int lookAheadMode)
{
    roadmanager::RoadProbeInfo s_data;
    Object                    *main_object = nullptr;
    if (getObjectById(object_id, main_object) == -1)
    {
        return -1;
    }

    roadmanager::Position            *pos = &player->scenarioGateway->getObjectStatePtrByIdx(object_id)->state_.pos;
    roadmanager::Position::ReturnCode retval =
        pos->GetProbeInfo(lookahead_distance, &s_data, static_cast<roadmanager::Position::LookAheadMode>(lookAheadMode));

    if (retval != roadmanager::Position::ReturnCode::ERROR_GENERIC)
    {
        // Copy data
        r_data->local_pos_x   = static_cast<float>(s_data.relative_pos[0]);
        r_data->local_pos_y   = static_cast<float>(s_data.relative_pos[1]);
        r_data->local_pos_z   = static_cast<float>(s_data.relative_pos[2]);
        r_data->global_pos_x  = static_cast<float>(s_data.road_lane_info.pos[0]);
        r_data->global_pos_y  = static_cast<float>(s_data.road_lane_info.pos[1]);
        r_data->global_pos_z  = static_cast<float>(s_data.road_lane_info.pos[2]);
        r_data->angle         = static_cast<float>(s_data.relative_h);
        r_data->curvature     = static_cast<float>(s_data.road_lane_info.curvature);
        r_data->road_heading  = static_cast<float>(s_data.road_lane_info.heading);
        r_data->road_pitch    = static_cast<float>(s_data.road_lane_info.pitch);
        r_data->road_roll     = static_cast<float>(s_data.road_lane_info.roll);
        r_data->trail_heading = r_data->road_heading;
        r_data->speed_limit   = static_cast<float>(s_data.road_lane_info.speed_limit);
        r_data->roadId        = s_data.road_lane_info.roadId;
        r_data->junctionId    = s_data.road_lane_info.junctionId;
        r_data->laneId        = s_data.road_lane_info.laneId;
        r_data->laneOffset    = static_cast<float>(s_data.road_lane_info.laneOffset);
        r_data->s             = static_cast<float>(s_data.road_lane_info.s);
        r_data->t             = static_cast<float>(s_data.road_lane_info.t);

        // Visualize forward looking road sensor probe
        main_object->SetSensorPosition(s_data.road_lane_info.pos[0], s_data.road_lane_info.pos[1], s_data.road_lane_info.pos[2]);
        player->SteeringSensorSetVisible(object_id, true);
    }

    return static_cast<int>(retval);
}

static int GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *r_data, float *speed_ghost)
{
    roadmanager::RoadProbeInfo s_data;

    Object *obj = nullptr;
    if (getObjectById(object_id, obj) == -1)
    {
        return -1;
    }

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

    double x{};
    double y{};
    double z{};
    int    index_out;

    if (ghost->trail_
            .FindClosestPoint(obj->pos_.GetX(), obj->pos_.GetY(), obj->trail_closest_pos_, obj->trail_follow_index_, obj->trail_follow_index_) == 0)
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

    (void)x;
    (void)y;
    (void)z;

    roadmanager::TrajVertex trailPos;
    trailPos.h =
        static_cast<float>(obj->pos_.GetH());  // Set default trail heading aligned with road - in case trail is less than two points (no heading)
    if (ghost->trail_.FindPointAhead(obj->trail_closest_pos_.s, lookahead_distance, trailPos, index_out, obj->trail_follow_index_) != 0)
    {
        return -1;
    }

    roadmanager::Position pos(trailPos.x, trailPos.y, 0, 0, 0, 0);
    obj->pos_.CalcProbeTarget(&pos, &s_data);

    // Copy data
    r_data->local_pos_x   = static_cast<float>(s_data.relative_pos[0]);
    r_data->local_pos_y   = static_cast<float>(s_data.relative_pos[1]);
    r_data->local_pos_z   = static_cast<float>(s_data.relative_pos[2]);
    r_data->global_pos_x  = static_cast<float>(s_data.road_lane_info.pos[0]);
    r_data->global_pos_y  = static_cast<float>(s_data.road_lane_info.pos[1]);
    r_data->global_pos_z  = static_cast<float>(s_data.road_lane_info.pos[2]);
    r_data->angle         = static_cast<float>(s_data.relative_h);
    r_data->curvature     = static_cast<float>(s_data.road_lane_info.curvature);
    r_data->road_heading  = static_cast<float>(s_data.road_lane_info.heading);
    r_data->trail_heading = static_cast<float>(trailPos.h);
    r_data->road_pitch    = static_cast<float>(s_data.road_lane_info.pitch);
    r_data->road_roll     = static_cast<float>(s_data.road_lane_info.roll);
    r_data->speed_limit   = static_cast<float>(s_data.road_lane_info.speed_limit);

    *speed_ghost = static_cast<float>(trailPos.speed);

    // Update object sensor position for visualization
    if (obj->sensor_pos_)
    {
        obj->sensor_pos_[0] = trailPos.x;
        obj->sensor_pos_[1] = trailPos.y;
        obj->sensor_pos_[2] = trailPos.z;
    }

    return 0;
}

static int GetRoadInfoAtGhostTrailTime(int object_id, float time, SE_RoadInfo *r_data, float *speed_ghost)
{
    roadmanager::RoadProbeInfo s_data;

    Object *obj = nullptr;
    if (getObjectById(object_id, obj) == -1)
    {
        return -1;
    }

    Object *ghost = obj->GetGhost();
    if (ghost == nullptr)
    {
        if (obj->GetAssignedControllerType() != Controller::Type::CONTROLLER_TYPE_DEFAULT)
        {
            LOG("Ghost object not available for object id %d", object_id);
        }

        return -1;
    }

    int index_out;

    roadmanager::TrajVertex trailPos;
    trailPos.h =
        static_cast<float>(obj->pos_.GetH());  // Set default trail heading aligned with road - in case trail is less than two points (no heading)

    if (ghost->trail_.FindPointAtTime(static_cast<double>(time) - ghost->GetHeadstartTime(), trailPos, index_out, obj->trail_follow_index_) != 0)
    {
        LOG("Failed to lookup point at time %.2f (time arg = %.2f) along ghost (%d) trail",
            player->scenarioEngine->getSimulationTime() - ghost->GetHeadstartTime() + static_cast<double>(time),
            static_cast<double>(time),
            ghost->GetId());
        return -1;
    }
    else
    {
        obj->trail_follow_index_ = index_out;
    }

    roadmanager::Position pos(trailPos.x, trailPos.y, 0, 0, 0, 0);
    obj->pos_.CalcProbeTarget(&pos, &s_data);

    // Copy data
    r_data->local_pos_x   = static_cast<float>(s_data.relative_pos[0]);
    r_data->local_pos_y   = static_cast<float>(s_data.relative_pos[1]);
    r_data->local_pos_z   = static_cast<float>(s_data.relative_pos[2]);
    r_data->global_pos_x  = static_cast<float>(s_data.road_lane_info.pos[0]);
    r_data->global_pos_y  = static_cast<float>(s_data.road_lane_info.pos[1]);
    r_data->global_pos_z  = static_cast<float>(s_data.road_lane_info.pos[2]);
    r_data->angle         = static_cast<float>(s_data.relative_h);
    r_data->curvature     = static_cast<float>(s_data.road_lane_info.curvature);
    r_data->road_heading  = static_cast<float>(s_data.road_lane_info.heading);
    r_data->trail_heading = static_cast<float>(trailPos.h);
    r_data->road_pitch    = static_cast<float>(s_data.road_lane_info.pitch);
    r_data->road_roll     = static_cast<float>(s_data.road_lane_info.roll);
    r_data->speed_limit   = static_cast<float>(s_data.road_lane_info.speed_limit);

    *speed_ghost = static_cast<float>(trailPos.speed);

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
    Logger::Inst().OpenLogfile(SE_Env::Inst().GetLogFilePath());
    Logger::Inst().LogVersion();

    ConvertArguments();

    // Create scenario engine
    try
    {
        // Initialize the scenario engine and viewer
        player     = new ScenarioPlayer(argc_, argv_);
        int retval = player->Init();
        if (retval == -1)
        {
            LOG("Failed to initialize scenario player");
        }
        else if (retval == -2)
        {
            LOG("Skipped initialize scenario player");
        }

        if (retval != 0)
        {
            resetScenario();
            return -1;
        }
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

    SE_DLL_API void SE_SetDatFilePath(const char *datFilePath)
    {
        SE_Env::Inst().SetDatFilePath(datFilePath);
    }

    SE_DLL_API unsigned int SE_GetSeed()
    {
        return SE_Env::Inst().GetRand().GetSeed();
    }

    SE_DLL_API void SE_SetSeed(unsigned int seed)
    {
        SE_Env::Inst().GetRand().SetSeed(seed);
    }

    SE_DLL_API int SE_SetParameterDistribution(const char *filename)
    {
        return OSCParameterDistribution::Inst().Load(filename);
    }

    SE_DLL_API void SE_ResetParameterDistribution()
    {
        OSCParameterDistribution::Inst().Reset();
    }

    SE_DLL_API int SE_GetNumberOfPermutations()
    {
        return static_cast<int>(OSCParameterDistribution::Inst().GetNumPermutations());
    }

    SE_DLL_API int SE_SelectPermutation(int index)
    {
        return OSCParameterDistribution::Inst().SetRequestedIndex(static_cast<unsigned int>(index));
    }

    SE_DLL_API int SE_GetPermutationIndex()
    {
        return OSCParameterDistribution::Inst().GetIndex();
    }

    SE_DLL_API void SE_SetWindowPosAndSize(int x, int y, int w, int h)
    {
        winDim = {x, y, w, h};
    }

    SE_DLL_API int SE_SetOSITolerances(double maxLongitudinalDistance, double maxLateralDeviation)
    {
        SE_Env::Inst().SetOSIMaxLongitudinalDistance(maxLongitudinalDistance);
        SE_Env::Inst().SetOSIMaxLateralDeviation(maxLateralDeviation);
        return 0;
    }

    SE_DLL_API int SE_InitWithArgs(int argc, const char *argv[])
    {
        resetScenario();

        if (argv && !strncmp(argv[0], "--", 2))
        {
            // Application name argument missing. Add something.
            AddArgument("esmini");
        }

        for (int i = 0; i < argc; i++)
        {
            AddArgument(argv[i], false);
        }

        return InitScenario();
    }

    static int AddCommonArguments(int disable_ctrls, int use_viewer, int threads, int record)
    {
        (void)record;
        if ((use_viewer & 1) == 0)
        {
            AddArgument("--headless");
        }
        else  // Viewer bit set, create a window for on and/or off-screen rendering
        {
            static char winArg[64];
            snprintf(winArg, sizeof(winArg), "--window %d %d %d %d", winDim.x, winDim.y, winDim.w, winDim.h);
            AddArgument(winArg, true);

            if (use_viewer & 2)  // off_screen
            {
                AddArgument("--headless");
            }

            if (use_viewer & 4)  // capture-to-file
            {
                AddArgument("--capture_screen");
            }

            if (use_viewer & 8)  // disable info-text
            {
                AddArgument("--info_text disable");
            }

            if (use_viewer & ~(0xf))  // check for invalid bits 0xf == 1+2+4+8
            {
                LOG("Unexpected use_viewer value: %d. Valid range: (0, %d) / (0x0, 0x%x) (%d, %d)", use_viewer, 0, 0xf, 0xf);
            }
        }

        if (threads)
        {
            AddArgument("--threads");
        }

        if (disable_ctrls)
        {
            AddArgument("--disable_controllers");
        }

        return 0;
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

        AddArgument("esmini(lib)");  // name of application
        AddArgument("--osc_str");
        AddArgument(oscAsXMLString, false);

        if (record)
        {
            AddArgument("--record");
            AddArgument(SE_Env::Inst().GetDatFilePath().c_str(), false);
        }

        AddCommonArguments(disable_ctrls, use_viewer, threads, record);

        return InitScenario();
    }

    SE_DLL_API void SE_RegisterParameterDeclarationCallback(void (*fnPtr)(void *), void *user_data)
    {
        RegisterParameterDeclarationCallback(fnPtr, user_data);
    }

    SE_DLL_API int SE_Init(const char *oscFilename, int disable_ctrls, int use_viewer, int threads, int record)
    {
#ifndef _USE_OSG
        if (use_viewer)
        {
            LOG("use_viewer flag set, but no viewer available (compiled without -D _USE_OSG");
        }
#endif
        resetScenario();

        AddArgument("esmini(lib)");  // name of application
        AddArgument("--osc");
        AddArgument(oscFilename, false);

        if (record)
        {
            AddArgument("--record");
            std::string datFilename;
            if (SE_Env::Inst().GetDatFilePath().empty())
            {
                datFilename = FileNameWithoutExtOf(oscFilename) + ".dat";
            }
            else
            {
                datFilename = SE_Env::Inst().GetDatFilePath();
            }
            AddArgument(datFilename.c_str(), false);
        }

        AddCommonArguments(disable_ctrls, use_viewer, threads, record);

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

    SE_DLL_API int SE_GetPauseFlag()
    {
        int pause_flag = -1;

        if (player != nullptr)
        {
            if (player->IsPaused())
            {
                pause_flag = 1;
            }
            else
            {
                pause_flag = 0;
            }
        }

        return pause_flag;
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

        returnString = player->GetParameterName(index, (OSCParameterDeclarations::ParameterType *)type);

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
            if (number > 0 && propertyIndex < number && propertyIndex >= 0)
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

    SE_DLL_API int SE_SetVariable(SE_Variable variable)
    {
        return ScenarioReader::variables.setParameterValue(variable.name, variable.value);
    }

    SE_DLL_API int SE_GetVariable(SE_Variable *variable)
    {
        return ScenarioReader::variables.getParameterValue(variable->name, variable->value);
    }

    SE_DLL_API int SE_GetVariableInt(const char *variableName, int *value)
    {
        return ScenarioReader::variables.getParameterValueInt(variableName, *value);
    }

    SE_DLL_API int SE_GetVariableDouble(const char *variableName, double *value)
    {
        return ScenarioReader::variables.getParameterValueDouble(variableName, *value);
    }

    SE_DLL_API int SE_GetVariableString(const char *variableName, const char **value)
    {
        return ScenarioReader::variables.getParameterValueString(variableName, *value);
    }

    SE_DLL_API int SE_GetVariableBool(const char *variableName, bool *value)
    {
        return ScenarioReader::variables.getParameterValueBool(variableName, *value);
    }

    SE_DLL_API int SE_SetVariableInt(const char *variableName, int value)
    {
        return ScenarioReader::variables.setParameterValue(variableName, value);
    }

    SE_DLL_API int SE_SetVariableDouble(const char *variableName, double value)
    {
        return ScenarioReader::variables.setParameterValue(variableName, value);
    }

    SE_DLL_API int SE_SetVariableString(const char *variableName, const char *value)
    {
        return ScenarioReader::variables.setParameterValue(variableName, value);
    }

    SE_DLL_API int SE_SetVariableBool(const char *variableName, bool value)
    {
        return ScenarioReader::variables.setParameterValue(variableName, value);
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
        RegisterParameterDeclarationCallback(nullptr, nullptr);
    }

    SE_DLL_API void SE_LogToConsole(bool mode)
    {
        logToConsole = mode;
    }

    SE_DLL_API void SE_CollisionDetection(bool mode)
    {
        SE_Env::Inst().SetCollisionDetection(mode);
    }

    SE_DLL_API int SE_Step()
    {
        if (player != nullptr)
        {
            player->SetFixedTimestep(-1.0);
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
            player->SetFixedTimestep(dt);
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

        return static_cast<float>(player->scenarioEngine->getSimulationTime());
    }

    SE_DLL_API double SE_GetSimulationTimeDouble()
    {
        if (player == nullptr)
        {
            return 0.0;
        }

        return player->scenarioEngine->getSimulationTime();
    }

    SE_DLL_API float SE_GetSimTimeStep()
    {
        if (player == nullptr)
        {
            return 0.0f;
        }

        return static_cast<float>(SE_getSimTimeStep(time_stamp, 0.001, 0.1));
    }

    SE_DLL_API void SE_SetAlignMode(int object_id, int mode)
    {
        if (player != nullptr)
        {
            Object *obj = nullptr;
            if (getObjectById(object_id, obj) == -1)
            {
                return;
            }

            player->scenarioGateway->setObjectAlignModeH(object_id, mode);
            player->scenarioGateway->setObjectAlignModeP(object_id, mode);
            player->scenarioGateway->setObjectAlignModeR(object_id, mode);
            player->scenarioGateway->setObjectAlignModeZ(object_id, mode);
        }
    }

    SE_DLL_API void SE_SetAlignModeH(int object_id, int mode)
    {
        if (player != nullptr)
        {
            Object *obj = nullptr;
            if (getObjectById(object_id, obj) == -1)
            {
                return;
            }

            player->scenarioGateway->setObjectAlignModeH(object_id, mode);
        }
    }

    SE_DLL_API void SE_SetAlignModeP(int object_id, int mode)
    {
        if (player != nullptr)
        {
            Object *obj = nullptr;
            if (getObjectById(object_id, obj) == -1)
            {
                return;
            }

            player->scenarioGateway->setObjectAlignModeP(object_id, mode);
        }
    }

    SE_DLL_API void SE_SetAlignModeR(int object_id, int mode)
    {
        if (player != nullptr)
        {
            Object *obj = nullptr;
            if (getObjectById(object_id, obj) == -1)
            {
                return;
            }

            player->scenarioGateway->setObjectAlignModeR(object_id, mode);
        }
    }

    SE_DLL_API void SE_SetAlignModeZ(int object_id, int mode)
    {
        if (player != nullptr)
        {
            Object *obj = nullptr;
            if (getObjectById(object_id, obj) == -1)
            {
                return;
            }

            player->scenarioGateway->setObjectAlignModeZ(object_id, mode);
        }
    }

    SE_DLL_API int SE_AddObject(const char *object_name, int object_type, int object_category, int object_role, int model_id)
    {
        SE_OSCBoundingBox bb = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        return SE_AddObjectWithBoundingBox(object_name,
                                           object_type,
                                           object_category,
                                           object_role,
                                           model_id,
                                           bb,
                                           static_cast<int>(EntityScaleMode::BB_TO_MODEL));
    }

    SE_DLL_API int SE_AddObjectWithBoundingBox(const char       *object_name,
                                               int               object_type,
                                               int               object_category,
                                               int               object_role,
                                               int               model_id,
                                               SE_OSCBoundingBox bounding_box,
                                               int               scale_mode)
    {
        int object_id = -1;

        // Add missing object
        if (player != nullptr)
        {
            std::string name;
            if (object_name == nullptr)
            {
                name = "obj_" + std::to_string(object_id);
            }
            else
            {
                name = object_name;
            }

            if (object_type < 1 || object_type >= scenarioengine::Object::Type::N_OBJECT_TYPES)
            {
                object_type = scenarioengine::Object::Type::VEHICLE;
            }

            Vehicle                       *vehicle = nullptr;
            scenarioengine::OSCBoundingBox bb;
            bb.center_.x_          = bounding_box.center_.x_;
            bb.center_.y_          = bounding_box.center_.y_;
            bb.center_.z_          = bounding_box.center_.z_;
            bb.dimensions_.height_ = bounding_box.dimensions_.height_;
            bb.dimensions_.length_ = bounding_box.dimensions_.length_;
            bb.dimensions_.width_  = bounding_box.dimensions_.width_;

            if (object_type == scenarioengine::Object::Type::VEHICLE)
            {
                vehicle               = new Vehicle();
                object_id             = player->scenarioEngine->entities_.addObject(vehicle, true);
                vehicle->name_        = name;
                vehicle->scaleMode_   = static_cast<EntityScaleMode>(scale_mode);
                vehicle->model_id_    = model_id;
                vehicle->model3d_     = SE_Env::Inst().GetModelFilenameById(model_id);
                vehicle->category_    = object_category;
                vehicle->role_        = object_role;
                vehicle->boundingbox_ = bb;

                Controller::InitArgs args = {"", "", 0, 0, 0, 0};
                args.type                 = ControllerExternal::GetTypeNameStatic();
                vehicle->controller_      = InstantiateControllerExternal(&args);
                vehicle->controller_->Activate(ControlDomains::DOMAIN_BOTH);
            }
            else
            {
                LOG("SE_AddObject: Object type %d not supported yet", object_type);
                return -1;
            }

            if (player->scenarioGateway->reportObject(object_id,
                                                      name,
                                                      object_type,
                                                      object_category,
                                                      object_role,
                                                      model_id,
                                                      vehicle->GetActivatedControllerType(),
                                                      bb,
                                                      scale_mode,
                                                      0xff,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      0.0,
                                                      nullptr) == 0)
            {
                return object_id;
            }
        }

        return -1;
    }

    SE_DLL_API int SE_DeleteObject(int object_id)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            // Object does not exist
            return -1;
        }

        if (player != nullptr)
        {
            player->scenarioEngine->entities_.removeObject(object_id);
            player->scenarioGateway->removeObject(object_id);
            return 0;
        }

        return -1;
    }

    SE_DLL_API int SE_ReportObjectPos(int object_id, float timestamp, float x, float y, float z, float h, float p, float r)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        player->scenarioGateway->updateObjectWorldPos(object_id, timestamp, x, y, z, h, p, r);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectPosXYH(int object_id, float timestamp, float x, float y, float h)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        player->scenarioGateway->updateObjectWorldPosXYH(object_id, timestamp, x, y, h);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectRoadPos(int object_id, float timestamp, int roadId, int laneId, float laneOffset, float s)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        player->scenarioGateway->updateObjectLanePos(object_id, timestamp, roadId, laneId, laneOffset, s);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectSpeed(int object_id, float speed)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }
        player->scenarioGateway->updateObjectSpeed(object_id, 0.0, speed);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectLateralPosition(int object_id, float t)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        player->scenarioGateway->reportObject(object_id,
                                              obj->name_,
                                              obj->type_,
                                              obj->category_,
                                              obj->role_,
                                              obj->model_id_,
                                              obj->GetActivatedControllerType(),
                                              obj->boundingbox_,
                                              static_cast<int>(obj->scaleMode_),
                                              obj->visibilityMask_,
                                              0.0,
                                              obj->GetSpeed(),
                                              obj->wheel_angle_,
                                              obj->wheel_rot_,
                                              obj->rear_axle_.positionZ,
                                              obj->pos_.GetTrackId(),
                                              t,
                                              obj->pos_.GetS(),
                                              obj->vehicleLightActionStatusList);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectLateralLanePosition(int object_id, int laneId, float laneOffset)
    {
        (void)laneId;
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        player->scenarioGateway->reportObject(object_id,
                                              obj->name_,
                                              obj->type_,
                                              obj->category_,
                                              obj->role_,
                                              obj->model_id_,
                                              obj->GetActivatedControllerType(),
                                              obj->boundingbox_,
                                              static_cast<int>(obj->scaleMode_),
                                              obj->visibilityMask_,
                                              0.0,
                                              obj->GetSpeed(),
                                              obj->wheel_angle_,
                                              obj->wheel_rot_,
                                              obj->rear_axle_.positionZ,
                                              obj->pos_.GetTrackId(),
                                              obj->pos_.GetLaneId(),
                                              laneOffset,
                                              obj->pos_.GetS(),
                                              obj->vehicleLightActionStatusList);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectVel(int object_id, float timestamp, float x_vel, float y_vel, float z_vel)
    {
        (void)timestamp;
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }
        player->scenarioGateway->updateObjectVel(object_id, 0.0, x_vel, y_vel, z_vel);
        // Also update velocities directly in scenario object, in case we're in a callback
        obj->SetVel(x_vel, y_vel, z_vel);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectAngularVel(int object_id, float timestamp, float h_rate, float p_rate, float r_rate)
    {
        (void)timestamp;
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }
        player->scenarioGateway->updateObjectAngularVel(object_id, 0.0, h_rate, p_rate, r_rate);
        // Also update accelerations directly in scenario object, in case we're in a callback
        obj->SetAngularVel(h_rate, p_rate, r_rate);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectAcc(int object_id, float timestamp, float x_acc, float y_acc, float z_acc)
    {
        (void)timestamp;
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }
        player->scenarioGateway->updateObjectAcc(object_id, 0.0, x_acc, y_acc, z_acc);
        // Also update accelerations directly in scenario object, in case we're in a callback
        obj->SetAcc(x_acc, y_acc, z_acc);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectAngularAcc(int object_id, float timestamp, float h_acc, float p_acc, float r_acc)
    {
        (void)timestamp;
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }
        player->scenarioGateway->updateObjectAngularAcc(object_id, 0.0, h_acc, p_acc, r_acc);
        // Also update accelerations directly in scenario object, in case we're in a callback
        obj->SetAngularAcc(h_acc, p_acc, r_acc);

        return 0;
    }

    SE_DLL_API int SE_ReportObjectWheelStatus(int object_id, float rotation, float angle)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }
        player->scenarioGateway->updateObjectWheelRotation(object_id, 0, rotation);
        player->scenarioGateway->updateObjectWheelAngle(object_id, 0, angle);

        return 0;
    }

    SE_DLL_API int SE_SetSnapLaneTypes(int object_id, int laneTypes)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        if (object_id >= 0 && object_id < static_cast<int>(player->scenarioEngine->entities_.object_.size()))
        {
            player->scenarioGateway->getObjectStatePtrByIdx(object_id)->state_.pos.SetSnapLaneTypes(laneTypes);
        }
        else
        {
            return -1;
        }

        return 0;
    }

    SE_DLL_API int SE_SetLockOnLane(int object_id, bool mode)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        if (object_id >= 0 && object_id < static_cast<int>(player->scenarioEngine->entities_.object_.size()))
        {
            player->scenarioGateway->getObjectStatePtrByIdx(object_id)->state_.pos.SetLockOnLane(mode);
        }
        else
        {
            return -1;
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

    SE_DLL_API int SE_GetId(int index)
    {
        if (player == nullptr || index < 0 || index >= player->scenarioGateway->getNumberOfObjects())
        {
            return -1;
        }

        return player->scenarioGateway->getObjectStatePtrByIdx(index)->state_.info.id;
    }

    SE_DLL_API int SE_GetIdByName(const char *name)
    {
        if (player == nullptr)
        {
            return -1;
        }

        for (size_t i = 0; player->scenarioEngine && i < player->scenarioEngine->entities_.object_.size(); i++)
        {
            if (player->scenarioEngine->entities_.object_[i]->GetName() == name)
            {
                return player->scenarioEngine->entities_.object_[i]->GetId();
            }
        }

        return -1;
    }

    SE_DLL_API int SE_GetObjectState(int object_id, SE_ScenarioObjectState *state)
    {
        scenarioengine::ObjectState obj_state;
        if (player->scenarioGateway->getObjectStateById(object_id, obj_state) != -1)
        {
            copyStateFromScenarioGateway(state, &obj_state.state_);
            return 0;
        }

        return -1;
    }

    SE_DLL_API int SE_GetOverrideActionStatus(int object_id, SE_OverrideActionList *list)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        return copyOverrideActionListfromScenarioEngine(list, obj);
    }

    SE_DLL_API int SE_GetNumberOfVehicleLights(int objectId)
    {
        Object *obj = nullptr;
        if (getObjectById(objectId, obj) == -1)
        {
            return -1;
        }
        return player->scenarioEngine->entities_.GetNumOfVehicleLight(objectId);
    }

    SE_DLL_API int SE_GetVehicleLightStatus(int object_id, int lightIndex, SE_VehicleLightState *lightState)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        return copyVehicleLightStatesListFromScenarioEngine(lightState, obj, lightIndex);
    }

    SE_DLL_API const char *SE_GetObjectTypeName(int object_id)
    {
        static std::string returnString;
        Object            *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return 0;
        }

        returnString = obj->GetTypeName();
        return returnString.c_str();
    }

    SE_DLL_API const char *SE_GetObjectName(int object_id)
    {
        static std::string returnString;
        Object            *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return 0;
        }
        returnString = obj->name_;
        return returnString.c_str();
    }

    SE_DLL_API const char *SE_GetObjectModelFileName(int object_id)
    {
        static std::string returnString;
        Object            *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return 0;
        }
        returnString = obj->GetModelFileName();
        return returnString.c_str();
    }

    SE_DLL_API int SE_OpenOSISocket(const char *ipaddr)
    {
#ifdef _USE_OSI
        if (player == nullptr)
        {
            return -1;
        }

        player->osiReporter->OpenSocket(ipaddr);
#else
        (void)ipaddr;
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API const char *SE_GetOSIGroundTruth(int *size)
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->GetOSIGroundTruth(size);
        }

        *size = 0;
#else
        (void)size;
#endif  // _USE_OSI
        return 0;
    }

    SE_DLL_API const char *SE_GetOSIGroundTruthRaw()
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->GetOSIGroundTruthRaw();
        }
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API int SE_SetOSISensorDataRaw(const char *sensordata)
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
#ifdef _USE_OSG
            if (player->viewer_)
            {
                const osi3::SensorData *sd = reinterpret_cast<const osi3::SensorData *>(sensordata);
                player->osiReporter->CreateSensorViewFromSensorData(*sd);
                if (player->osiReporter->GetSensorView())
                {
                    if (player->OSISensorDetection)
                    {
                        player->OSISensorDetection->Update(player->osiReporter->GetSensorView());
                    }
                }
            }
#endif
        }
#else
        (void)sensordata;
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API const char *SE_GetOSIRoadLane(int *size, int object_id)
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->GetOSIRoadLane(player->scenarioGateway->objectState_, size, object_id);
        }

        *size = 0;
#else
        (void)size;
        (void)object_id;
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API const char *SE_GetOSILaneBoundary(int *size, int global_id)
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->GetOSIRoadLaneBoundary(size, global_id);
        }

        *size = 0;
#else
        (void)size;
        (void)global_id;
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API void SE_GetOSILaneBoundaryIds(int object_id, SE_LaneBoundaryId *ids)
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            std::vector<int> ids_vector;
            player->osiReporter->GetOSILaneBoundaryIds(player->scenarioGateway->objectState_, ids_vector, object_id);
            if (!ids_vector.empty())
            {
                ids->far_left_lb_id  = ids_vector[0];
                ids->left_lb_id      = ids_vector[1];
                ids->right_lb_id     = ids_vector[2];
                ids->far_right_lb_id = ids_vector[3];
            }
        }
#else
        (void)object_id;
        (void)ids;
#endif  // _USE_OSI

        return;
    }

    SE_DLL_API int SE_ClearOSIGroundTruth()
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->ClearOSIGroundTruth();
        }
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API int SE_UpdateOSIGroundTruth()
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->UpdateOSIGroundTruth(player->scenarioGateway->objectState_);
        }
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API int SE_UpdateOSIStaticGroundTruth()
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->UpdateOSIStaticGroundTruth(player->scenarioGateway->objectState_);
        }
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API int SE_UpdateOSIDynamicGroundTruth(bool reportGhost)
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->UpdateOSIDynamicGroundTruth(player->scenarioGateway->objectState_, reportGhost);
        }
#else
        (void)reportGhost;
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API const char *SE_GetOSISensorDataRaw()
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            return player->osiReporter->GetOSISensorDataRaw();
        }
#endif  // _USE_OSI

        return 0;
    }

    SE_DLL_API bool SE_OSIFileOpen(const char *filename)
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            if (OSCParameterDistribution::Inst().GetNumPermutations() > 0)
            {
                return player->osiReporter->OpenOSIFile(OSCParameterDistribution::Inst().AddInfoToFilename(filename).c_str());
            }
            else
            {
                return player->osiReporter->OpenOSIFile(filename);
            }
        }
#else
        (void)filename;
#endif  // _USE_OSI

        return false;
    }

    SE_DLL_API bool SE_OSIFileWrite(bool flush)
    {
#ifdef _USE_OSI
        bool retval = false;

        if (player != nullptr)
        {
            retval = player->osiReporter->WriteOSIFile();
            if (flush)
            {
                player->osiReporter->FlushOSIFile();
            }
        }
        return retval;
#else
        (void)flush;
        return false;
#endif  // _USE_OSI
    }

    SE_DLL_API int SE_OSISetTimeStamp(unsigned long long int nanoseconds)
    {
#ifdef _USE_OSI
        if (player != nullptr)
        {
            player->osiReporter->SetOSITimeStampExplicit(nanoseconds);
            return 0;
        }
#else
        (void)nanoseconds;
#endif  // _USE_OSI

        return -1;
    }

    SE_DLL_API void SE_LogMessage(const char *message)
    {
        LOG(message);
    }

    SE_DLL_API int SE_ObjectHasGhost(int object_id)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return 0;
        }

        Object *ghost = 0;
        if (obj->GetAssignedControllerType() != Controller::Type::CONTROLLER_TYPE_DEFAULT)
        {
            ghost = obj->GetGhost();
        }
        return ghost != 0 ? 1 : 0;
    }

    SE_DLL_API int SE_GetObjectGhostState(int object_id, SE_ScenarioObjectState *state)
    {
        Object *ghost = nullptr;
        Object *obj   = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        if (obj->GetAssignedControllerType() != Controller::Type::CONTROLLER_TYPE_DEFAULT)
        {
            ghost = obj->GetGhost();
        }
        if (ghost)
        {
            scenarioengine::ObjectState obj_state;
            player->scenarioGateway->getObjectStateById(ghost->id_, obj_state);
            copyStateFromScenarioGateway(state, &obj_state.state_);
        }

        return ghost != 0 ? 0 : -1;
    }

    SE_DLL_API int SE_GetSpeedUnit()
    {
        roadmanager::OpenDrive *odr = roadmanager::Position::GetOpenDrive();
        if (odr != nullptr)
        {
            return static_cast<int>(odr->GetSpeedUnit());
        }

        return -1;
    }

    SE_DLL_API int SE_GetObjectNumberOfCollisions(int object_id)
    {
        Object *obj = nullptr;

        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        return static_cast<int>(obj->collisions_.size());
    }

    SE_DLL_API int SE_GetObjectCollision(int object_id, int index)
    {
        Object *obj = nullptr;

        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        if (index < 0 || index >= static_cast<int>(obj->collisions_.size()))
        {
            return -1;
        }

        return obj->collisions_[static_cast<unsigned int>(index)]->GetId();
    }

    /*SE_DLL_API int SE_GetObjectGhostStateFromOSI(const char* output, int index)
    {
            if (player)
            {
                    if (index < player->scenarioEngine->entities_.object_.size())
                    {
                            for (size_t i = 0; i < player->scenarioEngine->entities_.object_.size(); i++)  // ghost index always higher than external
    buddy
                            {
                                    if (player->scenarioEngine->entities_.object_[index]->ghost_)
                                    {
                                            scenarioengine::ObjectState obj_state;
                                            player->scenarioGateway->getObjectStateById(player->scenarioEngine->entities_.object_[index]->ghost_->id_,
    obj_state); copyStateFromScenarioGatewayToOSI(&output, &obj_state.state_);
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
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        player->AddObjectSensor(object_id, x, y, z, h, rangeNear, rangeFar, fovH, maxObj);

        return 0;
    }

    SE_DLL_API int SE_ViewSensorData(int object_id)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        player->AddOSIDetection(object_id);
        player->ShowObjectSensors(false);

        return 0;
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

    SE_DLL_API void SE_FlushOSIFile()
    {
#ifdef _USE_OSI
        if (player != nullptr && player->osiReporter != nullptr)
        {
            player->osiReporter->FlushOSIFile();
        }
#endif  // _USE_OSI
    }

    SE_DLL_API int SE_FetchSensorObjectList(int sensor_id, int *list)
    {
        if (player != nullptr)
        {
            if (sensor_id < 0 || sensor_id >= static_cast<int>(player->sensor.size()))
            {
                LOG("Invalid sensor_id (%d specified / %d available)", sensor_id, player->sensor.size());
                return -1;
            }

            for (int i = 0; i < player->sensor[static_cast<unsigned int>(sensor_id)]->nObj_; i++)
            {
                list[i] = player->sensor[static_cast<unsigned int>(sensor_id)]->hitList_[i].obj_->id_;
            }

            return player->sensor[static_cast<unsigned int>(sensor_id)]->nObj_;
        }

        return -1;
    }

    SE_DLL_API int SE_GetRoadInfoAtDistance(int          object_id,
                                            float        lookahead_distance,
                                            SE_RoadInfo *data,
                                            int          lookAheadMode,
                                            bool         inRoadDrivingDirection)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        float adjustedLookaheadDistance = lookahead_distance;

        if (inRoadDrivingDirection)
        {
            // Look in the driving direction of current lane
            if (obj->pos_.GetHRelativeDrivingDirection() > M_PI_2 && obj->pos_.GetHRelativeDrivingDirection() < 3 * M_PI_2)
            {
                adjustedLookaheadDistance = -lookahead_distance;
            }
        }

        return GetRoadInfoAtDistance(object_id, adjustedLookaheadDistance, data, lookAheadMode);
    }

    SE_DLL_API int SE_GetRoadInfoAlongGhostTrail(int object_id, float lookahead_distance, SE_RoadInfo *data, float *speed_ghost)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        if (GetRoadInfoAlongGhostTrail(object_id, lookahead_distance, data, speed_ghost) != 0)
        {
            return -1;
        }

        return 0;
    }

    SE_DLL_API int SE_GetRoadInfoGhostTrailTime(int object_id, float time, SE_RoadInfo *data, float *speed_ghost)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        if (GetRoadInfoAtGhostTrailTime(object_id, time, data, speed_ghost) != 0)
        {
            return -1;
        }

        return 0;
    }

    SE_DLL_API int SE_GetDistanceToObject(int object_a_id, int object_b_id, bool free_space, SE_PositionDiff *pos_diff)
    {
        bool obj_found = false;

        Object *obj_a = nullptr;
        if (getObjectById(object_a_id, obj_a) == -1)
        {
            return -1;
        }

        Object *obj_b = nullptr;
        if (getObjectById(object_b_id, obj_b) == -1)
        {
            return -1;
        }

        roadmanager::PositionDiff diff;

        if (free_space)
        {
            obj_found = (obj_a->FreeSpaceDistanceObjectRoadLane(obj_b, &diff, roadmanager::CoordinateSystem::CS_ROAD) == 0);
        }
        else
        {
            obj_found = obj_a->pos_.Delta(&obj_b->pos_, diff, true, 1000);  // look only 1000 meters
        }

        if (obj_found)
        {
            pos_diff->dLaneId       = diff.dLaneId;
            pos_diff->ds            = static_cast<float>(diff.ds);
            pos_diff->dt            = static_cast<float>(diff.dt);
            pos_diff->dx            = static_cast<float>(diff.dx);
            pos_diff->dy            = static_cast<float>(diff.dy);
            pos_diff->oppositeLanes = diff.dOppLane;
        }

        return obj_found ? 0 : -2;
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
        SE_ObjCallback cb;
        cb.id   = object_id;
        cb.func = fnPtr;
        objCallback.push_back(cb);
        player->RegisterObjCallback(object_id, objCallbackFn, user_data);
    }

    SE_DLL_API void SE_RegisterConditionCallback(void (*fnPtr)(const char *name, double timestamp))
    {
        OSCCondition::conditionCallback = fnPtr;
    }

    SE_DLL_API void SE_RegisterStoryBoardElementStateChangeCallback(void (*fnPtr)(const char *name, int type, int state))
    {
        StoryBoardElement::stateChangeCallback = fnPtr;
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

                    road_sign->id          = s->GetId();
                    returnString           = s->GetName();
                    road_sign->name        = returnString.c_str();
                    road_sign->x           = static_cast<float>(pos.GetX());
                    road_sign->y           = static_cast<float>(pos.GetY());
                    road_sign->z           = static_cast<float>(pos.GetZ());
                    road_sign->h           = static_cast<float>(pos.GetH());
                    road_sign->s           = static_cast<float>(pos.GetS());
                    road_sign->t           = static_cast<float>(pos.GetT());
                    road_sign->orientation = s->GetOrientation() == roadmanager::Signal::Orientation::NEGATIVE ? -1 : 1;
                    road_sign->z_offset    = static_cast<float>(s->GetZOffset());
                    road_sign->length      = static_cast<float>(s->GetLength());
                    road_sign->height      = static_cast<float>(s->GetHeight());
                    road_sign->width       = static_cast<float>(s->GetWidth());

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
            roadmanager::Road *road = player->odr_manager->GetRoadById(road_id);
            if (road != nullptr)
            {
                roadmanager::Signal *s = road->GetSignal(index);
                return static_cast<int>(s->validity_.size());
            }
        }

        return 0;
    }

    SE_DLL_API int SE_GetRoadSignValidityRecord(int road_id, int signIndex, int validityIndex, SE_RoadObjValidity *validity)
    {
        if (player != nullptr)
        {
            roadmanager::Road *road = player->odr_manager->GetRoadById(road_id);
            if (road != NULL)
            {
                roadmanager::Signal *s = road->GetSignal(signIndex);
                if (validityIndex >= 0 && validityIndex < static_cast<int>(s->validity_.size()))
                {
                    validity->fromLane = s->validity_[static_cast<unsigned int>(validityIndex)].fromLane_;
                    validity->toLane   = s->validity_[static_cast<unsigned int>(validityIndex)].toLane_;
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
#else
        (void)featureType;
        (void)enable;
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
            delete ((vehicle::Vehicle *)handleSimpleVehicle);
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

    SE_DLL_API void SE_SimpleVehicleControlTarget(void *handleSimpleVehicle, double dt, double target_speed, double heading_to_target)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }

        ((vehicle::Vehicle *)handleSimpleVehicle)->DrivingControlTarget(dt, target_speed, heading_to_target);
    }

    SE_DLL_API void SE_SimpleVehicleSetMaxSpeed(void *handleSimpleVehicle, float speed)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }
        ((vehicle::Vehicle *)handleSimpleVehicle)->SetMaxSpeed(static_cast<double>(speed) / 3.6);
    }

    SE_DLL_API void SE_SimpleVehicleSetMaxAcceleration(void *handleSimpleVehicle, float maxAcceleration)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }
        ((vehicle::Vehicle *)handleSimpleVehicle)->SetMaxAcc(maxAcceleration);
    }

    SE_DLL_API void SE_SimpleVehicleSetMaxDeceleration(void *handleSimpleVehicle, float maxDeceleration)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }
        ((vehicle::Vehicle *)handleSimpleVehicle)->SetMaxDec(maxDeceleration);
    }

    SE_DLL_API void SE_SimpleVehicleSetEngineBrakeFactor(void *handleSimpleVehicle, float engineBrakeFactor)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }
        ((vehicle::Vehicle *)handleSimpleVehicle)->SetEngineBrakeFactor(engineBrakeFactor);
    }

    SE_DLL_API void SE_SimpleVehicleSteeringScale(void *handleSimpleVehicle, float steeringScale)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }
        ((vehicle::Vehicle *)handleSimpleVehicle)->SetSteeringScale(steeringScale);
    }

    SE_DLL_API void SE_SimpleVehicleSteeringReturnFactor(void *handleSimpleVehicle, float steeringReturnFactor)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }
        ((vehicle::Vehicle *)handleSimpleVehicle)->SetSteeringReturnFactor(steeringReturnFactor);
    }

    SE_DLL_API void SE_SimpleVehicleSteeringRate(void *handleSimpleVehicle, float steeringRate)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }
        ((vehicle::Vehicle *)handleSimpleVehicle)->SetSteeringRate(steeringRate);
    }

    SE_DLL_API void SE_SimpleVehicleGetState(void *handleSimpleVehicle, SE_SimpleVehicleState *state)
    {
        if (handleSimpleVehicle == 0)
        {
            return;
        }
        state->x              = static_cast<float>(((vehicle::Vehicle *)handleSimpleVehicle)->posX_);
        state->y              = static_cast<float>(((vehicle::Vehicle *)handleSimpleVehicle)->posY_);
        state->z              = static_cast<float>(((vehicle::Vehicle *)handleSimpleVehicle)->posZ_);
        state->h              = static_cast<float>(((vehicle::Vehicle *)handleSimpleVehicle)->heading_);
        state->p              = static_cast<float>(((vehicle::Vehicle *)handleSimpleVehicle)->pitch_);
        state->speed          = static_cast<float>(((vehicle::Vehicle *)handleSimpleVehicle)->speed_);
        state->wheel_rotation = static_cast<float>(((vehicle::Vehicle *)handleSimpleVehicle)->wheelRotation_);
        state->wheel_angle    = static_cast<float>(((vehicle::Vehicle *)handleSimpleVehicle)->wheelAngle_);
    }

    SE_DLL_API int SE_SetOffScreenRendering(bool state)
    {
        SE_Env::Inst().SetOffScreenRendering(state);
        return 0;
    }

    SE_DLL_API int SE_SaveImagesToRAM(bool state)
    {
#ifdef _USE_OSG
        if (player)
        {
            player->SaveImagesToRAM(state);
            return 0;
        }
        else
        {
            return -1;
        }
#else
        (void)state;
        return -1;
#endif
    }

    SE_DLL_API int SE_SaveImagesToFile(int nrOfFrames)
    {
#ifdef _USE_OSG
        if (player)
        {
            player->SaveImagesToFile(nrOfFrames);
            return 0;
        }
        else
        {
            return -1;
        }
#else
        (void)nrOfFrames;
        return -1;
#endif
    }

    SE_DLL_API int SE_FetchImage(SE_Image *img)
    {
#ifdef _USE_OSG
        if (player)
        {
            OffScreenImage *offScrImg = nullptr;
            if ((offScrImg = player->FetchCapturedImagePtr()) == nullptr)
            {
                return -1;
            }
            img->width       = offScrImg->width;
            img->height      = offScrImg->height;
            img->pixelSize   = offScrImg->pixelSize;
            img->pixelFormat = offScrImg->pixelFormat;
            img->data        = offScrImg->data;
        }

        return 0;
#else
        (void)img;
        return -1;
#endif
    }

    SE_DLL_API void SE_RegisterImageCallback(void (*fnPtr)(SE_Image *, void *), void *user_data)
    {
#ifdef _USE_OSG
        RegisterImageCallback((viewer::ImageCallbackFunc)fnPtr, user_data);  // ensure SE_Image and OffScrImage is compatible
#else
        (void)fnPtr;
        (void)user_data;
#endif
    }

    SE_DLL_API int
    SE_WritePPMImage(const char *filename, int width, int height, const unsigned char *data, int pixelSize, int pixelFormat, bool upsidedown)
    {
        return SE_WritePPM(filename, width, height, data, pixelSize, pixelFormat, upsidedown);
    }

    SE_DLL_API int
    SE_WriteTGAImage(const char *filename, int width, int height, const unsigned char *data, int pixelSize, int pixelFormat, bool upsidedown)
    {
        return SE_WriteTGA(filename, width, height, data, pixelSize, pixelFormat, upsidedown);
    }

    SE_DLL_API int SE_AddCustomCamera(double x, double y, double z, double h, double p)
    {
#ifdef _USE_OSG
        if (player)
        {
            player->AddCustomCamera(x, y, z, h, p, false);
        }
        else
        {
            return -1;
        }

        return 0;
#else
        (void)x;
        (void)y;
        (void)z;
        (void)h;
        (void)p;
        return -1;
#endif
    }

    SE_DLL_API int SE_AddCustomFixedCamera(double x, double y, double z, double h, double p)
    {
#ifdef _USE_OSG
        if (player)
        {
            player->AddCustomCamera(x, y, z, h, p, true);
        }
        else
        {
            return -1;
        }

        return 0;
#else
        (void)x;
        (void)y;
        (void)z;
        (void)h;
        (void)p;
        return -1;
#endif
    }

    SE_DLL_API int SE_AddCustomAimingCamera(double x, double y, double z)
    {
#ifdef _USE_OSG
        if (player)
        {
            player->AddCustomCamera(x, y, z, false);
        }
        else
        {
            return -1;
        }

        return 0;
#else
        (void)x;
        (void)y;
        (void)z;
        return -1;
#endif
    }

    SE_DLL_API int SE_AddCustomFixedAimingCamera(double x, double y, double z)
    {
#ifdef _USE_OSG
        if (player)
        {
            player->AddCustomCamera(x, y, z, true);
        }
        else
        {
            return -1;
        }

        return 0;
#else
        (void)x;
        (void)y;
        (void)z;
        return -1;
#endif
    }

    SE_DLL_API int SE_AddCustomFixedTopCamera(double x, double y, double z, double rot)
    {
#ifdef _USE_OSG
        if (player)
        {
            player->AddCustomFixedTopCamera(x, y, z, rot);
        }
        else
        {
            return -1;
        }

        return 0;
#else
        (void)x;
        (void)y;
        (void)z;
        (void)rot;
        return -1;
#endif
    }

    SE_DLL_API int SE_SetCameraMode(int mode)
    {
#ifdef _USE_OSG
        if (player && player->viewer_)
        {
            player->viewer_->SetCameraMode(mode);
            return 0;
        }
#else
        (void)mode;
#endif
        return -1;
    }

    SE_DLL_API int SE_SetCameraObjectFocus(int object_id)
    {
#ifdef _USE_OSG
        if (player && player->viewer_)
        {
            for (size_t i = 0; i < player->scenarioEngine->entities_.object_.size(); i++)
            {
                if (player->scenarioEngine->entities_.object_[i]->GetId() == object_id)
                {
                    player->viewer_->SetVehicleInFocus(static_cast<int>(i));
                }
            }
            return 0;
        }
#else
        (void)object_id;
#endif
        return -1;
    }

    SE_DLL_API int SE_GetNumberOfRoutePoints(int object_id)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        if (obj->pos_.GetRoute())
        {
            return static_cast<int>(obj->pos_.GetRoute()->all_waypoints_.size());
        }
        else
        {
            return 0;
        }
    }

    SE_DLL_API int SE_GetRoutePoint(int object_id, int route_index, SE_RouteInfo *routeinfo)
    {
        Object *obj = nullptr;
        if (getObjectById(object_id, obj) == -1)
        {
            return -1;
        }

        if (route_index >= static_cast<int>(obj->pos_.GetRoute()->all_waypoints_.size()))
        {
            LOG("Requested waypoint index %d invalid, only %d registered", route_index, obj->pos_.GetRoute()->all_waypoints_.size());
            return -1;
        }

        roadmanager::Road *road =
            player->odr_manager->GetRoadById(obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetTrackId());

        routeinfo->x          = static_cast<float>(obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetX());
        routeinfo->y          = static_cast<float>(obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetY());
        routeinfo->z          = static_cast<float>(obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetZ());
        routeinfo->roadId     = obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetTrackId();
        routeinfo->junctionId = obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetJunctionId();
        routeinfo->laneId     = obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetLaneId();
        routeinfo->osiLaneId  = road->GetDrivingLaneById(obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetS(),
                                                        obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetLaneId())
                                   ->GetGlobalId();
        routeinfo->laneOffset = static_cast<float>(obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetOffset());
        routeinfo->s          = static_cast<float>(obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetS());
        routeinfo->t          = static_cast<float>(obj->pos_.GetRoute()->all_waypoints_[static_cast<unsigned int>(route_index)].GetT());

        return 0;
    }
}