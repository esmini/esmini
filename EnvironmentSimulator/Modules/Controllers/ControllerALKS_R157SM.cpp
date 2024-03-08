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

#include "ControllerALKS_R157SM.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioEngine.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

#define R157_LOG(level, format, ...)                                                                 \
    {                                                                                                \
        if (level > 0 && level <= GetLogLevel())                                                     \
        {                                                                                            \
            LOG((std::string("ALKS R157 ") + GetModelName() + " " + format).c_str(), ##__VA_ARGS__); \
        }                                                                                            \
        else                                                                                         \
        {                                                                                            \
            (void)0;                                                                                 \
        }                                                                                            \
    }

#define R157_LOG_SIMPLE(level, format)                                                \
    {                                                                                 \
        if (level > 0 && level <= GetLogLevel())                                      \
        {                                                                             \
            LOG((std::string("ALKS R157 ") + GetModelName() + " " + format).c_str()); \
        }                                                                             \
        else                                                                          \
        {                                                                             \
            (void)0;                                                                  \
        }                                                                             \
    }

std::map<ControllerALKS_R157SM::ScenarioType, std::string> ControllerALKS_R157SM::ScenarioTypeName = {
    {ControllerALKS_R157SM::ScenarioType::None, "None"},
    {ControllerALKS_R157SM::ScenarioType::CutIn, "CutIn"},
    {ControllerALKS_R157SM::ScenarioType::CutOut, "CutOut"},
    {ControllerALKS_R157SM::ScenarioType::Deceleration, "Deceleration"}};

std::map<ControllerALKS_R157SM::ModelType, std::string> ControllerALKS_R157SM::ModelTypeName = {
    {ControllerALKS_R157SM::ModelType::Regulation, "Regulation"},
    {ControllerALKS_R157SM::ModelType::ReferenceDriver, "ReferenceDriver"},
    {ControllerALKS_R157SM::ModelType::RSS, "RSS"},
    {ControllerALKS_R157SM::ModelType::FSM, "FSM"}};

std::map<ControllerALKS_R157SM::ReferenceDriver::Phase, std::string> ControllerALKS_R157SM::ReferenceDriver::PhaseName = {
    {ControllerALKS_R157SM::ReferenceDriver::Phase::INACTIVE, "INACTIVE"},
    {ControllerALKS_R157SM::ReferenceDriver::Phase::PERCEIVE, "PERCEIVE"},
    {ControllerALKS_R157SM::ReferenceDriver::Phase::REACT, "REACT"},
    {ControllerALKS_R157SM::ReferenceDriver::Phase::BRAKE_REF, "BRAKE_REF"},
    {ControllerALKS_R157SM::ReferenceDriver::Phase::BRAKE_AEB, "BRAKE_AEB"}};

Controller* scenarioengine::InstantiateControllerALKS_R157SM(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerALKS_R157SM(initArgs);
}

ControllerALKS_R157SM::ControllerALKS_R157SM(InitArgs* args) : Controller(args), model_(0), entities_(0)
{
    if (args && args->properties)
    {
        if (args->properties->GetValueStr("model") == "Regulation")
        {
            model_ = reinterpret_cast<ControllerALKS_R157SM::Model*>(new Regulation());
        }
        else if (args->properties->GetValueStr("model") == "FSM")
        {
            model_ = reinterpret_cast<ControllerALKS_R157SM::Model*>(new FSM());
        }
        else if (args->properties->GetValueStr("model") == "ReferenceDriver")
        {
            ReferenceDriver* ref_driver = new ReferenceDriver();
            if (args->properties->ValueExists("cutInPerceptionDelayMode"))
            {
                if (args->properties->GetValueStr("cutInPerceptionDelayMode") == "Dist")
                {
                    ref_driver->cut_in_perception_delay_mode_ = ReferenceDriver::CutInPerceptionDelayMode::DIST;
                }
                else if (args->properties->GetValueStr("cutInPerceptionDelayMode") == "Time")
                {
                    ref_driver->cut_in_perception_delay_mode_ = ReferenceDriver::CutInPerceptionDelayMode::TIME;
                }
                else
                {
                    LOG("ControllerALKS_R157SM: Unexpected cutInPerceptionDelayMode: %s",
                        args->properties->GetValueStr("cutInPerceptionDelayMode").c_str());
                }
            }
            LOG("ALKS_R157SM ReferenceDriver perceptionDelayMode: %s",
                ref_driver->cut_in_perception_delay_mode_ == ReferenceDriver::CutInPerceptionDelayMode::TIME ? "Time" : "Dist");

            if (args->properties->ValueExists("pedestrianRiskEvaluationTime"))
            {
                ref_driver->SetPedestrianRiskEvaluationTime(strtod(args->properties->GetValueStr("pedestrianRiskEvaluationTime")));
            }
            LOG("ALKS_R157SM PedestrianRiskEvaluationTime: %.2f", ref_driver->GetPedestrianRiskEvaluationTime());

            if (args->properties->ValueExists("aebTTC"))
            {
                ref_driver->aeb_.ttc_critical_aeb_ = strtod(args->properties->GetValueStr("aebTTC"));
            }
            LOG("ALKS_R157SM AEB TTC: %.2f", ref_driver->aeb_.ttc_critical_aeb_);

            if (args->properties->ValueExists("lateralTrigDistance"))
            {
                ref_driver->lateral_dist_trigger_             = new ReferenceDriver::LateralDistTrigger(ref_driver);
                ref_driver->lateral_dist_trigger_->threshold_ = strtod(args->properties->GetValueStr("lateralTrigDistance"));
                ref_driver->lateral_dist_trigger_->SetName("LateralDistTrigger");
                LOG("ALKS_R157SM AEB LateralTrigDistance: %.2f", ref_driver->lateral_dist_trigger_->threshold_);
            }

            if (args->properties->ValueExists("overlapTolerance"))
            {
                ref_driver->overlap_tolerance_ = strtod(args->properties->GetValueStr("overlapTolerance"));
            }
            LOG("ALKS_R157SM Overlap tolerance: %.2f", ref_driver->overlap_tolerance_);

            if (!ref_driver->lateral_dist_trigger_)
            {
                ref_driver->wandering_trigger_             = new ReferenceDriver::WanderingTrigger(ref_driver);
                ref_driver->wandering_trigger_->threshold_ = ref_driver->wandering_threshold_;
                ref_driver->wandering_trigger_->SetName("WanderingTrigger");
            }

            model_ = reinterpret_cast<ControllerALKS_R157SM::Model*>(ref_driver);
        }
        else if (args->properties->GetValueStr("model") == "RSS")
        {
            model_ = reinterpret_cast<ControllerALKS_R157SM::Model*>(new RSS());
        }
        else
        {
            LOG_AND_QUIT("ControllerALKS_R157SM unexpected model %s", args->properties->GetValueStr("model").c_str());
        }
        LOG("ALKS_R157SM model: %s", model_->GetModelName().c_str());

        if (args->properties->ValueExists("logLevel"))
        {
            model_->SetLogging(strtoi(args->properties->GetValueStr("logLevel")));
        }
        LOG("ALKS_R157SM logLevel: %d", model_->log_level_);

        if (args->properties->ValueExists("fullStop"))
        {
            model_->SetFullStop(args->properties->GetValueStr("fullStop") == "true" ? true : false);
        }
        LOG("ALKS_R157SM fullStop: %s", model_->GetFullStop() ? "true" : "false");

        if (args->properties->ValueExists("alwaysTrigOnScenario"))
        {
            model_->SetAlwaysTrigOnScenario(args->properties->GetValueStr("alwaysTrigOnScenario") == "true" ? true : false);
        }
        LOG("ALKS_R157SM alwaysTrigOnScenario: %s", model_->GetAlwaysTrigOnScenario() ? "true" : "false");

        if (args->properties->ValueExists("cruise"))
        {
            if (args->properties->GetValueStr("cruise") == "true" || args->properties->GetValueStr("cruise") == "True")
            {
                model_->SetCruise(true);
            }
            else if (args->properties->GetValueStr("cruise") == "false" || args->properties->GetValueStr("cruise") == "False")
            {
                model_->SetCruise(false);
            }
        }
        LOG("ALKS_R157SM cruise: %s", model_->cruise_ ? "true" : "false");
    }
}

ControllerALKS_R157SM::~ControllerALKS_R157SM()
{
    if (model_ != nullptr)
    {
        delete model_;
    }
}

void ControllerALKS_R157SM::Init()
{
    Controller::Init();
}

void ControllerALKS_R157SM::Step(double timeStep)
{
    double speed = model_->Step(timeStep);

    if (mode_ == ControlOperationMode::MODE_OVERRIDE)
    {
        object_->MoveAlongS(speed * timeStep);
        gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    }

    gateway_->updateObjectSpeed(object_->GetId(), 0.0, speed);

    Controller::Step(timeStep);
}

void ControllerALKS_R157SM::LinkObject(Object* object)
{
    if (!object)
    {
        return;
    }

    if (object->type_ != Object::Type::VEHICLE)
    {
        LOG("Failed attempt to assign ControllerALKS_R157SM controller to a non vehicle object %s", object->GetName().c_str());
        return;
    }

    if (model_)
    {
        model_->SetVehicle(static_cast<Vehicle*>(object));
    }

    Controller::LinkObject(object);
}

int ControllerALKS_R157SM::Activate(ControlActivationMode lat_activation_mode,
                                    ControlActivationMode long_activation_mode,
                                    ControlActivationMode light_activation_mode,
                                    ControlActivationMode anim_activation_mode)
{
    if (model_)
    {
        model_->set_speed_ = object_->GetSpeed();
    }

    return Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);
}

void ControllerALKS_R157SM::SetScenarioEngine(ScenarioEngine* scenario_engine)
{
    scenario_engine_ = scenario_engine;
    if (model_)
    {
        model_->SetScenarioEngine(scenario_engine);
    }
}

void ControllerALKS_R157SM::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}

int ControllerALKS_R157SM::Model::Detect()
{
    ObjectInfo candidate_obj_info, tmp_obj_info;

    if (entities_ == 0)
    {
        R157_LOG_SIMPLE(1, "No entities! Register scenarioengine - SetScenarioEngine()");
        return -1;
    }

    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        tmp_obj_info.obj = entities_->object_[i];

        if (Process(tmp_obj_info) != 0)
        {
            continue;
        }

        if (!CheckSafety(&tmp_obj_info))
        {
            // Check whether it is the closest of potential threats
            if (tmp_obj_info.dist_long <
                    candidate_obj_info.dist_long ||  // closest so far
                                                     // OR object in own lane is detected beyond a cutting-out one and with potentially less ttc
                (tmp_obj_info.dLaneId == 0 && candidate_obj_info.obj->GetType() == Object::Type::VEHICLE &&
                 candidate_obj_info.action == ScenarioType::CutOut))
            {
                if (tmp_obj_info.obj->GetType() == Object::Type::VEHICLE && tmp_obj_info.action == ScenarioType::CutOut && candidate_obj_info.obj &&
                    candidate_obj_info.dLaneId == 0)
                {
                    // Vehicle is cutting out revealing another object beyond in same lane
                    // Focus on the one with least TTC
                    if (tmp_obj_info.ttc < candidate_obj_info.ttc)
                    {
                        candidate_obj_info = tmp_obj_info;
                    }
                }
                else if (tmp_obj_info.dLaneId == 0 && candidate_obj_info.obj && candidate_obj_info.obj->GetType() == Object::Type::VEHICLE &&
                         candidate_obj_info.action == ScenarioType::CutOut)
                {
                    // Detected object is in same lane and beyond a cutting out vehicle
                    // Focus on the one with least TTC
                    if (tmp_obj_info.ttc < candidate_obj_info.ttc)
                    {
                        Object* cut_out_vehicle            = candidate_obj_info.obj;
                        candidate_obj_info                 = tmp_obj_info;
                        candidate_obj_info.cut_out_vehicle = cut_out_vehicle;
                        candidate_obj_info.action          = ScenarioType::CutOut;
                    }
                }
                else
                {
                    // For all other cases simply detect the closest object
                    candidate_obj_info = tmp_obj_info;
                }
            }
        }
    }

    if (candidate_obj_info.obj && candidate_obj_info.action != ScenarioType::None &&
        (candidate_obj_info.obj != object_in_focus_.obj || candidate_obj_info.action != object_in_focus_.action))
    {
        // New object or scenario detected, register scenario type
        SetScenarioType(candidate_obj_info.action);

        R157_LOG(1,
                 "Detected object: %s Scenario: %s",
                 candidate_obj_info.obj->GetName().c_str(),
                 ScenarioType2Str(candidate_obj_info.action).c_str());
    }

    object_in_focus_ = candidate_obj_info;

    return object_in_focus_.obj ? 0 : -1;
}

int ControllerALKS_R157SM::Model::Process(ObjectInfo& info)
{
    // Find closest object
    // Consider all vehicles:
    //   - in front AND
    //   - in own lane OR
    //   - in neighbors lanes changing into own lane

    if (info.obj == nullptr || info.obj == veh_)
    {
        return -1;
    }

    roadmanager::PositionDiff diff;

    // Measure longitudinal distance to all vehicles, don't utilize costly freespace option, instead measure ref point to ref point
    if (veh_->pos_.Delta(&info.obj->pos_, diff, false, GetMaxRange()) == true)
    {
        // Adjust delta lane id in case vehicles are on either side of center lane
        if (diff.dLaneId == 2 && info.obj->pos_.GetLaneId() == 1 && veh_->pos_.GetLaneId() == -1)
        {
            diff.dLaneId = 1;
        }
        else if (diff.dLaneId == -2 && info.obj->pos_.GetLaneId() == -1 && veh_->pos_.GetLaneId() == 1)
        {
            diff.dLaneId = -1;
        }

        if (diff.ds > SMALL_NUMBER &&                            // consider only entities in front of ego
            (info.obj->GetType() == Object::Type::PEDESTRIAN ||  // consider pedestrians regardless of lane
             (diff.dLaneId > -2 && diff.dLaneId < 2)))           // consider other entities in own or adjecent lanes
        {
            info.dLaneId = diff.dLaneId;
            roadmanager::PositionDiff pos_diff;

            // Now calculate the exact longitudinal and lateral distance
            if (veh_->FreeSpaceDistanceObjectRoadLane(info.obj, &pos_diff, roadmanager::CoordinateSystem::CS_ROAD) == 0)
            {
                info.dist_lat  = pos_diff.dt;
                info.dist_long = pos_diff.ds;
                double veh_v_s, veh_v_t, obj_v_s, obj_v_t;
                veh_->pos_.GetVelTS(veh_v_t, veh_v_s);
                info.obj->pos_.GetVelTS(obj_v_t, obj_v_s);

                // Calculate relative speed along road (s axis), from ego point of view
                info.dv_s = (veh_v_s - obj_v_s) *
                            (abs(GetAngleInIntervalMinusPIPlusPI(veh_->pos_.GetHRelative())) > M_PI_2 ? -1.0 : 1.0);  // ignore road direction

                // Calculate relative speed across road (t axis), from ego point of view
                info.dv_t = (veh_v_t - obj_v_t) * SIGN(info.obj->pos_.GetT() - veh_->pos_.GetT());  // ignore side

                if (info.dv_s > 0.0)
                {
                    info.ttc = info.dist_long / info.dv_s;
                    info.thw = info.dist_long / veh_v_s;
                }
                else
                {
                    info.ttc = LARGE_NUMBER;
                    info.thw = LARGE_NUMBER;
                }

                R157_LOG(3,
                         "%s relative speed s, t: %.2f, %.2f dist: %.2f, %.2f dLane %d TTC: %.2f",
                         info.obj->GetName().c_str(),
                         info.dv_s,
                         info.dv_t,
                         info.dist_long,
                         info.dist_lat,
                         info.dLaneId,
                         info.ttc);

                return 0;
            }
        }
    }

    return -1;
}

double ControllerALKS_R157SM::Model::Step(double timeStep)
{
    dt_ = timeStep;

    if (object_in_focus_.obj && GetModelType() == ModelType::ReferenceDriver &&
        (reinterpret_cast<ReferenceDriver*>(this))->GetPhase() != ControllerALKS_R157SM::ReferenceDriver::Phase::INACTIVE && GetFullStop())
    {
        // reference driver ongoing action in combination with FullStop mode
        // skip full re-scan, just update measurements for object already in focus
        Process(object_in_focus_);
    }
    else
    {
        // do a full re-scan for object in focus
        Detect();
    }

    if (CheckCritical())
    {
        return ReactCritical();
    }

    return Cruise();
}

void ControllerALKS_R157SM::Model::ResetObjectInFocus()
{
    object_in_focus_ = {nullptr, nullptr, LARGE_NUMBER, LARGE_NUMBER, 0.0, 0.0, LARGE_NUMBER, LARGE_NUMBER, 0, ScenarioType::None};
}

void ControllerALKS_R157SM::Model::ResetReactionTime()
{
    R157_LOG(2, "Reaction timer (%.2fs) started", GetReactionTime());
    rt_counter_ = GetReactionTime();
}

ControllerALKS_R157SM::Model::Model(ModelType type, double reaction_time, double max_dec, double max_range)
    : type_(type),
      veh_(nullptr),
      entities_(0),
      cut_in_detected_timestamp_(0.0),
      rt_(reaction_time),
      rt_counter_(0.0),
      max_dec_(max_dec),
      max_range_(max_range),
      max_acc_(3.0),
      max_acc_lat_(1.0),
      set_speed_(0.0),
      dt_(0.01),
      acc_(0.0),
      cruise_comfort_acc_(2.0),
      cruise_comfort_dec_(2.0),
      cruise_max_acc_(3.0),
      cruise_max_dec_(4.0),
      model_mode_(ModelMode::NO_TARGET),
      scenario_type_(ScenarioType::None),
      log_level_(1),
      cruise_(true),
      full_stop_(false),
      always_trig_on_scenario_(false),
      scenario_engine_(nullptr)
{
    ResetObjectInFocus();
}

double ControllerALKS_R157SM::Model::Cruise()
{
    if (cruise_)
    {
        if (object_in_focus_.obj == nullptr)
        {
            // No object in sight - Simple cruise control
            SetModelMode(ModelMode::NO_TARGET);

            if (NEAR_NUMBERS(veh_->GetSpeed(), set_speed_))
            {
                acc_ = 0.0;
            }
            else
            {
                if (veh_->GetSpeed() > set_speed_)
                {
                    acc_ = -cruise_comfort_dec_;
                }
                else
                {
                    acc_ = cruise_comfort_acc_;
                }
            }
            return CLAMP(veh_->GetSpeed() + acc_ * dt_, 0, set_speed_);
        }
        // TTCLaneIntrusion > vrel / (2 x 6 ms^2) + 0.35s?
        else if (object_in_focus_.dist_long > 0 && object_in_focus_.ttc > object_in_focus_.dv_s / 12.0 + 0.35)
        {
            SetModelMode(ModelMode::TARGET_IN_SIGHT);

            // A simple car following model:
            //   strive for equal speed and a distance dependning on lead vehicle speed
            //   acc = A * -dv / ds + B * (ds - minimum_dist)
            //   where A and B are constants (that can/should be tuned)

            if (NEAR_NUMBERS(veh_->GetSpeed(), 0.0))
            {
                // If come to a stop, release brake
                acc_ = 0.0;
            }
            else
            {
                double A = 2.0;
                double B = 1.0;

                acc_ = A * -object_in_focus_.dv_s + B * (object_in_focus_.dist_long - MinDist());
                acc_ = CLAMP(acc_, -cruise_max_dec_, cruise_max_acc_);
            }

            R157_LOG(3, "Cruise with target, acc: %.2f", acc_);
            return CLAMP(veh_->GetSpeed() + acc_ * dt_, 0, set_speed_);
        }
    }

    // no change
    acc_ = 0.0;
    return veh_->GetSpeed();
}

std::string ControllerALKS_R157SM::Model::Mode2Str(ModelMode mode)
{
    if (mode == ModelMode::CRITICAL)
    {
        return "CRITICAL";
    }
    else if (mode == ModelMode::NO_TARGET)
    {
        return "NO_TARGET";
    }
    else if (mode == ModelMode::TARGET_IN_SIGHT)
    {
        return "TARGET_IN_SIGHT";
    }

    return "Unknown mode " + std::to_string(static_cast<int>(mode));
}

void ControllerALKS_R157SM::Model::SetModelMode(ModelMode mode, bool log)
{
    if (mode != model_mode_)
    {
        if (log)
        {
            R157_LOG(1, "R157_Model mode: %s -> %s", Mode2Str(model_mode_).c_str(), Mode2Str(mode).c_str());
        }
        model_mode_ = mode;
    }
}

void ControllerALKS_R157SM::Model::SetScenarioEngine(ScenarioEngine* scenario_engine)
{
    scenario_engine_ = scenario_engine;
    entities_        = &scenario_engine_->entities_;
}

void ControllerALKS_R157SM::Model::SetScenarioType(ScenarioType type)
{
    scenario_type_ = type;
}

double ControllerALKS_R157SM::Regulation::MinDist()
{
    //  This function will return minimum following distance according to R157 regulation

    double time_gap = 0.0;

    // E/ECE/TRANS/505/Rev.3/Add.151 5.2.3.3.
    if (veh_->GetSpeed() * 3.6 < 10.0)
    {
        time_gap = 1.0 + (MAX(2.0, veh_->GetSpeed()) - 2.0) / 7.8;
    }
    else
    {
        time_gap = 1.0 + veh_->GetSpeed() * 3.6 / 100.0;
    }

    // At least two meter distance
    return MAX(veh_->GetSpeed() * time_gap, 2.0);
}

//   Return true if safe
bool ControllerALKS_R157SM::Regulation::CheckSafety(ObjectInfo* info)
{
    if (info->obj == nullptr || info->dist_lat == LARGE_NUMBER)
    {
        return true;
    }

    if (info->dLaneId == 0)  // same lane
    {
        info->action = ScenarioType::Deceleration;
        return false;
    }

    // Check if car's front wheel has moved 0.3 meter inside own vehicle's lane
    roadmanager::Road* road = veh_->pos_.GetOpenDrive()->GetRoadById(info->obj->pos_.GetTrackId());
    if (road && info->dist_lat > -SMALL_NUMBER)
    {
        double lane_width = road->GetLaneWidthByS(info->obj->pos_.GetS(), info->obj->pos_.GetLaneId());
        double x          = 0.0;
        if (info->obj->GetType() == Object::Type::VEHICLE)
        {
            // Find out position of front wheel (axis)
            x = (static_cast<Vehicle*>(info->obj))->front_axle_.positionX;
        }
        else
        {
            // Use front side of bounding box
            x = static_cast<double>(info->obj->boundingbox_.dimensions_.length_) / 2.0;
        }

        // Look at side towards Ego vehicle lane
        double y  = -1 * SIGN(info->dLaneId) * static_cast<double>(info->obj->boundingbox_.dimensions_.width_) / 2.0;
        double xr = 0.0, yr = 0.0;
        RotateVec2D(x, y, info->obj->pos_.GetHRelative(), xr, yr);
        double offset = info->obj->pos_.GetOffset() + yr;

        // adjust sign of offset wrt side towards Ego vehicle
        offset *= -1 * SIGN(info->dLaneId);

        // Check if wheel is 0.3 meter inside Ego lane
        if (offset > lane_width / 2.0 + 0.3)
        {
            info->action = ScenarioType::CutIn;
            return false;
        }
    }

    return true;
}

bool ControllerALKS_R157SM::Regulation::CheckCritical()
{
    if (object_in_focus_.obj == nullptr)
    {
        SetModelMode(ModelMode::NO_TARGET);
        return false;
    }
    else if (GetModelMode() == ModelMode::CRITICAL && full_stop_)
    {
        // if already entered critical state and full stop is requested, stay in critical state regardless
        return true;
    }
    else if (object_in_focus_.dv_s > 0.0 && object_in_focus_.dist_long > 0)
    {
        // TTCLaneIntrusion < vrel / (2 x 6 ms^2) + 0.35s?
        if (object_in_focus_.ttc < object_in_focus_.dv_s / (2 * GetMaxDec()) + GetReactionTime() + 0.1)
        {
            SetModelMode(ModelMode::CRITICAL);
            return true;
        }
    }

    SetModelMode(ModelMode::TARGET_IN_SIGHT);
    return false;
}

double ControllerALKS_R157SM::Regulation::ReactCritical()
{
#if 0  // should really reaction time be applied here?
    if (scenario_engine_->getSimulationTime() < cut_in_detected_timestamp_ + GetReactionTime())
    {
        return veh_->GetSpeed();  // no reaction yet
    }
#endif
    acc_         = -GetMaxDec();
    double speed = MAX(0.0, veh_->GetSpeed() + acc_ * dt_);

    R157_LOG(3, "Critical: acc %.2f speed %.2f", acc_, speed);

    return speed;
}

ControllerALKS_R157SM::ReferenceDriver::~ReferenceDriver()
{
    if (lateral_dist_trigger_)
    {
        delete lateral_dist_trigger_;
    }

    if (wandering_trigger_)
    {
        delete wandering_trigger_;
    }
}

void ControllerALKS_R157SM::ReferenceDriver::UpdateAEB(Vehicle* ego, ObjectInfo* info)
{
    if (!aeb_.active_ && info->ttc < aeb_.ttc_critical_aeb_ &&
        ego->OverlappingFront(info->obj, overlap_tolerance_) > Object::OverlapType::PART)  // object fully inside or covering ego front extension
    {
        R157_LOG(2, "AEB activated at ttc %.2f (< critical ttc %.2f)", info->ttc, aeb_.ttc_critical_aeb_);
        aeb_.active_ = true;
    }
}

void ControllerALKS_R157SM::ReferenceDriver::LateralDistTrigger::Update(ObjectInfo* info)
{
    if (info->obj == nullptr || info->dv_t < SMALL_NUMBER)  // moving away from ego lateral position
    {
        if (active_)
        {
            R157_LOG(2, "%s deactivated", name_.c_str());
            Reset();
        }
        return;
    }

    if (info->obj != obj_)
    {
        obj_    = info->obj;
        active_ = true;
        t0_     = info->obj->pos_.GetT();

        R157_LOG(2, "%s activated on %s at t %.3f with dist delta threshold %.3f", name_.c_str(), info->obj->GetName().c_str(), t0_, threshold_);
    }
}

void ControllerALKS_R157SM::ReferenceDriver::WanderingTrigger::Update(ObjectInfo* info)
{
    if (info->obj == nullptr || info->dv_t < SMALL_NUMBER)  // moving away from ego lateral position
    {
        if (active_)
        {
            R157_LOG(2, "%s deactivated", name_.c_str());
            Reset();
        }
        return;
    }

    if (info->obj != obj_)
    {
        obj_    = info->obj;
        active_ = true;
        t0_     = info->obj->pos_.GetT();

        threshold_ = model_->wandering_threshold_;

        bool mc = obj_->GetType() == Object::Type::VEHICLE && (reinterpret_cast<Vehicle*>(obj_))->category_ == Vehicle::Category::MOTORBIKE;

        if ((obj_->pos_.GetT() > model_->veh_->pos_.GetT() &&  // target is to the left of Ego, moving along negative T
             obj_->pos_.GetOffset() > 0) ||                    // target is to the left (other side) of lane center
            (obj_->pos_.GetT() < model_->veh_->pos_.GetT() &&  // target is to the right of Ego, moving along positive T
             obj_->pos_.GetOffset() < 0))                      // target is to the right (other side) of lane center
        {
            // add current lane offset to end up at wandering_threshold
            threshold_ += abs(obj_->pos_.GetOffset());
        }
        else if (!mc)
        {
            // Target is not a mc and is already at same side of lane center as ego
            // remove current lane offset to end up at wandering_threshold.
            // MC is exception, it will move wandering_threshold regardless of
            // initial lane offset when on same side as Ego.
            threshold_ = MAX(0.0, threshold_ - abs(obj_->pos_.GetOffset()));
        }

        R157_LOG(2,
                 "%s activated on %s (%s) at t %.3f with delta dist threshold %.3f",
                 name_.c_str(),
                 info->obj->GetName().c_str(),
                 mc ? "MC" : "non MC",
                 t0_,
                 threshold_);
    }
}

bool ControllerALKS_R157SM::ReferenceDriver::LateralDistTrigger::Evaluate()
{
    if (active_ == false || obj_ == nullptr)
    {
        return false;
    }

    return abs(obj_->pos_.GetT() - t0_) > threshold_;
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckSafety(ObjectInfo* info)
{
    if (info->obj == nullptr || info->dist_lat == LARGE_NUMBER)
    {
        return true;
    }

    // Calculate lane offset of object center, transform OpenSCENARIO reference point
    c_lane_offset_ = info->obj->pos_.GetOffset() + SE_Vector(info->obj->boundingbox_.center_.x_, 0.0).Rotate(info->obj->pos_.GetH()).y();

    if (lateral_dist_trigger_)
    {
        lateral_dist_trigger_->Update(info);
    }
    else
    {
        wandering_trigger_->Update(info);
    }

    if (info->obj->GetType() == Object::Type::PEDESTRIAN)
    {
        if (info->dv_t > SMALL_NUMBER)
        {
            info->action = ScenarioType::CutIn;
            return false;  // always consider pedestrians moving laterally towards ego
        }
    }
    else if (info->dLaneId == 0)  // same lane
    {
        if (abs(c_lane_offset_) > wandering_threshold_ && info->dv_t < -SMALL_NUMBER)  // moving away from ego
        {
            info->action = ScenarioType::CutOut;  // indicate that car is potentially moving out of own lane
        }
        else if (-SIGN(info->obj->pos_.GetLaneId()) * info->obj->pos_.GetAccS() < -5.0 + SMALL_NUMBER)
        {
            info->action = ScenarioType::Deceleration;  // vehicle is decelerating
        }
        return false;  // car is in own lane, so far
    }
    else
    {
        if ((lateral_dist_trigger_ && lateral_dist_trigger_->Evaluate() == true) ||
            (!lateral_dist_trigger_ && wandering_trigger_->Evaluate() == true))  // check distance at the side facing ego
        {
            info->action = ScenarioType::CutIn;
            return false;
        }
        else if (info->action == ScenarioType::CutIn)
        {
            info->action = ScenarioType::None;
        }
    }

    return true;  // object is not a lateral threat
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckPerceptionCutIn()
{
    if (GetPhase() == Phase::INACTIVE)
    {
        if (object_in_focus_.obj && object_in_focus_.obj->GetType() == Object::Type::PEDESTRIAN && object_in_focus_.dv_s > SMALL_NUMBER)
        {
            // Calculate perception distance based on default or specified risk perception/evaluation time
            timer_ = GetPedestrianRiskEvaluationTime();
            SetPhase(Phase::PERCEIVE);  // always consider pedestrians moving laterally towards ego
        }
        else
        {
            bool trig = false;
            if (lateral_dist_trigger_ && lateral_dist_trigger_->Evaluate() == true)
            {
                R157_LOG(2,
                         "Trig perception on lateral distance: %.2f (>%.2f)",
                         lateral_dist_trigger_->GetDistance(),
                         lateral_dist_trigger_->threshold_);
                trig = true;
            }
            else if (wandering_trigger_ && wandering_trigger_->Evaluate() == true)
            {
                R157_LOG(2,
                         "Trig perception on lateral wandering threshold: %.3f (>%.3f)",
                         abs(wandering_trigger_->obj_->pos_.GetT() - wandering_trigger_->t0_),
                         wandering_trigger_->threshold_);
                trig = true;
            }

            if (trig)
            {
                perception_t_ = object_in_focus_.obj->pos_.GetT();

                // Establish additional lateral perception distance
                if (cut_in_perception_delay_mode_ == CutInPerceptionDelayMode::DIST)
                {
                    perception_dist_ = 0.72;  // ALKS Reg 157 3.4.1 default
                }
                else if (cut_in_perception_delay_mode_ == CutInPerceptionDelayMode::TIME)
                {
                    // Calculate perception distance based on 0.4 seconds delay instead
                    timer_ = perception_time_;
                }

                SetPhase(Phase::PERCEIVE);
            }
        }
    }
    else if (GetPhase() == Phase::PERCEIVE)
    {
        if (object_in_focus_.obj && object_in_focus_.obj->GetType() == Object::Type::PEDESTRIAN)
        {
            if (object_in_focus_.dv_s > SMALL_NUMBER)
            {
                timer_ -= dt_;
                if (timer_ < SMALL_NUMBER)
                {
                    SetPhase(Phase::REACT);
                    R157_LOG(2, "Pedestrian %s perceived after %.2fs", object_in_focus_.obj->GetName().c_str(), GetPedestrianRiskEvaluationTime());
                }
            }
        }
        else
        {
            if (cut_in_perception_delay_mode_ == CutInPerceptionDelayMode::DIST)
            {
                if (object_in_focus_.obj == nullptr)
                {
                    R157_LOG_SIMPLE(3, "In perceive phase, but lost object in focus");
                }
                else if (object_in_focus_.obj->pos_.GetT() > veh_->pos_.GetT() &&  // target moving along negative T
                         object_in_focus_.obj->pos_.GetT() < perception_t_ - perception_dist_)
                {
                    R157_LOG(2,
                             "Reached lateral perception distance (t %.3f < t0 %.3f - perc dist %.2f)",
                             object_in_focus_.obj->pos_.GetT(),
                             perception_t_,
                             perception_dist_);
                    SetPhase(Phase::REACT);
                }
                else if (object_in_focus_.obj->pos_.GetT() < veh_->pos_.GetT() &&  // target moving along positive T
                         object_in_focus_.obj->pos_.GetT() > perception_t_ + perception_dist_)
                {
                    R157_LOG(2,
                             "Reached lateral perception distance (t %.3f > t0 %.3f + perc dist %.2f)",
                             object_in_focus_.obj->pos_.GetT(),
                             perception_t_,
                             perception_dist_);
                    SetPhase(Phase::REACT);
                }
            }
            else if (cut_in_perception_delay_mode_ == CutInPerceptionDelayMode::TIME)
            {
                timer_ -= dt_;
                if (timer_ < SMALL_NUMBER)
                {
                    SetPhase(Phase::REACT);
                    R157_LOG(2,
                             "Reached lateral perception time at t %.2fm (t0 %.3fm + perc time %.2f)",
                             object_in_focus_.obj->pos_.GetT(),
                             perception_t_,
                             perception_time_);
                }
            }
        }
        if (GetPhase() == Phase::REACT)
        {
            timer_ = rt_;
            return true;
        }
    }

    return false;
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckPerceptionCutOut()
{
    if (GetPhase() == Phase::INACTIVE)
    {
        if (object_in_focus_.dv_s > SMALL_NUMBER)
        {
            SetPhase(Phase::PERCEIVE);
            timer_ = perception_time_;
        }
    }
    else if (GetPhase() == Phase::PERCEIVE)
    {
        timer_ -= dt_;
        return timer_ < SMALL_NUMBER;
    }

    return false;
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckPerceptionDeceleration()
{
    if (GetPhase() == Phase::INACTIVE)
    {
        if (object_in_focus_.obj && object_in_focus_.obj->pos_.GetAccS() < -5.0)
        {
            SetPhase(Phase::PERCEIVE);
            timer_ = perception_time_;
        }
    }
    else if (GetPhase() == Phase::PERCEIVE)
    {
        timer_ -= dt_;
        return timer_ < SMALL_NUMBER;
    }

    return false;
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckCriticalCutIn()
{
    bool retval = false;

    if (GetAlwaysTrigOnScenario() || object_in_focus_.ttc < critical_ttc_)
    {
        if (GetPhase() == Phase::REACT)
        {
            if (timer_ < SMALL_NUMBER)
            {
                R157_LOG(2,
                         "Reacting to cut-in at TTC %.2f (< %.2f) (%s)",
                         object_in_focus_.ttc,
                         critical_ttc_,
                         object_in_focus_.thw < critical_thw_ ? "Critical" : "Non critical");
                SetModelMode(ModelMode::CRITICAL);
                retval = true;
            }
        }
    }
    else
    {
        if (GetPhase() == Phase::REACT)
        {
            // not considered a cut-in scenario
            R157_LOG(2, "Perceived cut-in at TTC %.2f (> %.2f) => non critical", object_in_focus_.ttc, critical_ttc_);
            Reset();
        }
        else if (GetModelMode() == ModelMode::CRITICAL)
        {
            // Critical until out of minimum distance
            if (object_in_focus_.dist_long > MinDist())
            {
                SetModelMode(object_in_focus_.obj ? ModelMode::TARGET_IN_SIGHT : ModelMode::NO_TARGET);
                if (object_in_focus_.dv_s < SMALL_NUMBER)
                {
                    Reset();  // gap increasing - exit scenario
                }
            }
        }
    }

    return retval;
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckCriticalCutOut()
{
    bool retval = false;

    if (GetAlwaysTrigOnScenario() || object_in_focus_.thw < critical_thw_)
    {
        if (GetPhase() == Phase::REACT)
        {
            if (timer_ < SMALL_NUMBER)
            {
                R157_LOG(2,
                         "Reacting to cut-out THW = %.2f (< %.2f) (%s)",
                         object_in_focus_.thw,
                         critical_thw_,
                         object_in_focus_.thw < critical_thw_ ? "Critical" : "Non critical");
                SetModelMode(ModelMode::CRITICAL);
                retval = true;
            }
        }
    }
    else
    {
        if (GetPhase() == Phase::REACT)
        {
            // not considered a cut-out scenario
            R157_LOG(2, "Reacting to cut-out but THW = %.2f (>%.2f) => non critical", object_in_focus_.thw, critical_thw_);
            Reset();
        }
        else if (GetModelMode() == ModelMode::CRITICAL)
        {
            // Critical until out of minimum distance
            if (object_in_focus_.dist_long > MinDist())
            {
                SetModelMode(object_in_focus_.obj ? ModelMode::TARGET_IN_SIGHT : ModelMode::NO_TARGET);
                if (object_in_focus_.dv_s < SMALL_NUMBER)
                {
                    Reset();  // gap increasing - exit scenario
                }
            }
        }
    }

    return retval;
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckCriticalDeceleration()
{
    bool retval = false;

    if (GetAlwaysTrigOnScenario() || object_in_focus_.thw < critical_thw_)
    {
        if (GetPhase() == Phase::REACT)
        {
            if (timer_ < SMALL_NUMBER)
            {
                R157_LOG(2,
                         "Reacting to dececleration THW < %.2f (%.2f) (%s)",
                         object_in_focus_.thw,
                         critical_thw_,
                         object_in_focus_.thw < critical_thw_ ? "Critical" : "Non critical");
                SetModelMode(ModelMode::CRITICAL);
                retval = true;
            }
        }
    }
    else
    {
        if (GetPhase() == Phase::REACT)
        {
            // not considered a cut-out scenario
            R157_LOG(2, "Reacting to dececleration but THW > %.2f (%.2f) => non critical", object_in_focus_.thw, critical_thw_);
            Reset();
        }
        else if (GetModelMode() == ModelMode::CRITICAL)
        {
            // Critical until out of minimum distance
            if (object_in_focus_.dist_long > MinDist())
            {
                SetModelMode(object_in_focus_.obj ? ModelMode::TARGET_IN_SIGHT : ModelMode::NO_TARGET);
                if (object_in_focus_.dv_s < SMALL_NUMBER)
                {
                    Reset();  // gap increasing - exit scenario
                }
            }
        }
    }

    return retval;
}

void ControllerALKS_R157SM::ReferenceDriver::SetPhase(Phase phase)
{
    if (phase != phase_)
    {
        R157_LOG(2, "phase: %s -> %s", Phase2Str(phase_).c_str(), Phase2Str(phase).c_str());
        phase_ = phase;
    }
}

std::string ControllerALKS_R157SM::ReferenceDriver::Phase2Str(Phase phase)
{
    return PhaseName[phase];
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckCritical()
{
    if (GetPhase() == Phase::INACTIVE || GetPhase() == Phase::PERCEIVE)
    {
        if (CheckPerception())
        {
            R157_LOG(3,
                     "Perceived critical %s scenario ttc %.2f hwt %.2f",
                     ScenarioType2Str(GetScenarioType()).c_str(),
                     object_in_focus_.ttc,
                     object_in_focus_.thw);
            SetPhase(Phase::REACT);
            timer_ = rt_;
        }
    }
    else if (GetPhase() == Phase::REACT)
    {
        timer_ -= dt_;
    }

    if (GetModelMode() == ModelMode::NO_TARGET && object_in_focus_.obj)
    {
        SetModelMode(ModelMode::TARGET_IN_SIGHT);
    }

    if (GetFullStop() && GetModelMode() == ModelMode::CRITICAL)
    {
        // only get out of critical mode when come to a full stop
        if (veh_->GetSpeed() < SMALL_NUMBER)
        {
            Reset();
            SetModelMode(object_in_focus_.obj ? ModelMode::TARGET_IN_SIGHT : ModelMode::NO_TARGET);
        }
    }
    else if (!object_in_focus_.obj)
    {
        Reset();  // lost sight of object
        SetModelMode(ModelMode::NO_TARGET);
    }
    else if ((veh_->GetSpeed() > SMALL_NUMBER && object_in_focus_.dist_long > 0) ||
             GetFullStop()  // fulfill the scenario even if ego passed the perceived vehicle
    )
    {
        CheckCriticalCondition();
    }

    UpdateAEB(veh_, &object_in_focus_);

    return GetModelMode() == ModelMode::CRITICAL || aeb_.active_;
}

double ControllerALKS_R157SM::ReferenceDriver::ReactCritical()
{
    if (phase_ == Phase::REACT && timer_ < SMALL_NUMBER)
    {
        SetPhase(Phase::BRAKE_REF);
    }

    if (GetPhase() == Phase::BRAKE_REF || aeb_.active_)
    {
        if (veh_->GetSpeed() < SMALL_NUMBER)
        {
            SetPhase(Phase::INACTIVE);
            aeb_.Reset();
            timer_ = 0.0;
        }
    }

    // Act
    if (GetPhase() == Phase::REACT)
    {
        // if already decelerating, keep deceleration rate
        // otherwise apply idle deceleration from lifting foot from gas pedal
        acc_ = MIN(-release_deceleration_, acc_);
    }
    else if (GetPhase() == Phase::BRAKE_REF)
    {
        acc_ -= dt_ * 0.774 * g / 0.6;
    }

    if (aeb_.active_)
    {
        acc_ -= dt_ * 0.85 * g / 0.6;
    }

    (aeb_.active_) ? acc_ = MAX(acc_, -0.85 * g) : acc_ = MAX(acc_, -0.774 * g);

    double speed = MAX(0.0, veh_->GetSpeed() + acc_ * dt_);

    R157_LOG(3, "React critical: acc %.2f vel %.2f timer %.2f", acc_, speed, timer_);

    return speed;
}

double ControllerALKS_R157SM::ReferenceDriver::MinDist()
{
    double min_dist = 2.0 + veh_->GetSpeed() * 2.0;

    // R157_LOG(1, "Min dist: %.2f", min_dist);

    return min_dist;
}

bool ControllerALKS_R157SM::RSS::CheckSafety(ObjectInfo* info)
{
    if (info->obj == nullptr || info->dist_lat == LARGE_NUMBER)
    {
        return true;
    }

    if (info->dist_lat < SMALL_NUMBER || info->dLaneId == 0)
    {
        info->action = ScenarioType::Deceleration;
        return false;
    }

    double cut_in_lat = abs(info->obj->pos_.GetVelT());
    double d_safe_rss_lat =
        mu_ + abs((2.0 * cut_in_lat + max_acc_lat_ * rt_) * rt_ / 2.0) + pow(cut_in_lat + max_acc_lat_ * rt_, 2) / (2 * max_acc_lat_);

    R157_LOG(3, "CheckSafety: dist_lat % .2f d_safe_rss_lat %.2f", info->dist_lat, d_safe_rss_lat);

    if (abs(info->dist_lat) < d_safe_rss_lat)
    {
        info->action = ScenarioType::CutIn;
        return false;
    }

    return true;
}

bool ControllerALKS_R157SM::RSS::CheckCritical()
{
    if (object_in_focus_.obj == nullptr)
    {
        SetModelMode(ModelMode::NO_TARGET);
        return false;
    }

    if (object_in_focus_.dv_s > 0.0 && object_in_focus_.dist_long > 0)
    {
        double d_safe_rss_long = veh_->GetSpeed() * rt_ + max_acc_ * pow(rt_, 2) / 2.0 + pow(veh_->GetSpeed() + rt_ * max_acc_, 2) / (2 * max_dec_) -
                                 pow(object_in_focus_.obj->GetSpeed(), 2) / (2 * max_dec_);

        R157_LOG(3, "CheckCritical: dist %.2f d_safe_rss_long %.2f", object_in_focus_.dist_long, d_safe_rss_long);

        if (object_in_focus_.dist_long < d_safe_rss_long)
        {
            if (GetModelMode() != ModelMode::CRITICAL)
            {
                ResetReactionTime();
            }
            SetModelMode(ModelMode::CRITICAL);
            return true;
        }
    }

    SetModelMode(ModelMode::TARGET_IN_SIGHT);
    return false;
}

double ControllerALKS_R157SM::RSS::ReactCritical()
{
    if (rt_counter_ > 0)
    {
        acc_ = MIN(acc_, 0.0);  // release gas pedal
        rt_counter_ -= dt_;
        return veh_->GetSpeed() + acc_ * dt_;
    }

    acc_ = MIN(acc_ - min_jerk_ * dt_, max_dec_);
    R157_LOG(3, "critical acc %.2f", acc_);

    return MAX(veh_->GetSpeed() + acc_ * dt_, 0);
}

double ControllerALKS_R157SM::RSS::MinDist()
{
    double min_dist = veh_->GetSpeed() * rt_ * rt_ + max_acc_ * pow(rt_, 2) / 2 + pow(veh_->GetSpeed() + rt_ * max_acc_, 2) / (2 * max_dec_) +
                      -pow(MAX(veh_->GetSpeed() - rt_ * max_acc_ * max_dec_, 0), 2) / (2 * max_dec_) + 1.0 / 2 * max_dec_ * pow(rt_, 2);

    return min_dist;
}

double ControllerALKS_R157SM::FSM::MinDist()
{
    double min_dist = margin_dist_ + veh_->GetSpeed() * rt_ + pow(veh_->GetSpeed(), 2) / (2 * br_min_) -
                      pow(object_in_focus_.obj->GetSpeed(), 2) / (2 * bl_) + margin_safe_dist_;

    R157_LOG(3, "Min dist: %.2f", min_dist);

    return min_dist;
}

double ControllerALKS_R157SM::FSM::PFS(double dist,
                                       double speed_rear,
                                       double speed_lead,
                                       double rt,
                                       double br_min,
                                       double br_max,
                                       double bl,
                                       double margin_dist,
                                       double margin_safe_dist)
{
    dist = dist - margin_dist;

    double dsafe = speed_rear * rt + pow(speed_rear, 2) / (2 * br_min) - pow(speed_lead, 2) / (2 * bl) + margin_safe_dist;

    if (dist > dsafe)
    {
        return 0.0;
    }
    else
    {
        double dunsafe = speed_rear * rt + pow(speed_rear, 2) / (2 * br_max) - pow(speed_lead, 2) / (2 * bl);
        if (dist < dunsafe)
        {
            return 1.0;
        }
        else
        {
            return (dist - dsafe) / (dunsafe - dsafe);
        }
    }
}

double ControllerALKS_R157SM::FSM::CFS(double dist, double speed_rear, double speed_lead, double rt, double br_min, double br_max, double acc)
{
    double arF   = MAX(acc, -br_min);      // Do not take into account very hard decelerations
    double u_new = speed_rear + rt * arF;  // Estimate rear vehicle new speed
    double dsafe = 0.0;

    if (speed_rear <= speed_lead)
    {
        return 0.0;
    }

    if (u_new < speed_lead)
    {
        dsafe = pow(speed_rear - speed_lead, 2) / abs(acc * 2);
        if (dist < dsafe)
        {
            return 1.0;
        }
        else
        {
            return 0.0;
        }
    }
    else
    {
        dsafe = (speed_rear + arF * rt / 2 - speed_lead) * rt + pow(speed_rear + arF * rt - speed_lead, 2) / (br_min * 2);

        if (dist > dsafe)
        {
            return 0.0;
        }
        else
        {
            double dunsafe = (speed_rear + arF * rt / 2 - speed_lead) * rt + pow(speed_rear + arF * rt - speed_lead, 2) / (br_max * 2);

            if (dist < dunsafe)
            {
                return 1.0;
            }
            else
            {
                return (dist - dsafe) / (dunsafe - dsafe);
            }
        }
    }
}

bool ControllerALKS_R157SM::FSM::CheckSafety(ObjectInfo* info)
{
    if (info->obj == nullptr || info->dist_lat == LARGE_NUMBER)
    {
        return true;
    }

    if (info->dLaneId == 0)  // same lane
    {
        info->action = ScenarioType::Deceleration;
        return false;
    }

    if (info->dist_lat > -SMALL_NUMBER)
    {
        double cutin_speed = -SIGN(info->dLaneId) * info->obj->pos_.GetVelT();
        if (cutin_speed > 0)
        {
            double headway_lat        = abs(info->dist_lat) / cutin_speed;
            double headway_long_gross = abs(info->dist_long) / MAX(SMALL_NUMBER, veh_->GetSpeed() - info->obj->GetSpeed());

            if (headway_lat > headway_long_gross + 0.1)
            {
                return true;
            }
        }
        else
        {
            return true;
        }
    }

    info->action = ScenarioType::CutIn;
    return false;
}

bool ControllerALKS_R157SM::FSM::CheckCritical()
{
    if (object_in_focus_.obj == nullptr)
    {
        SetModelMode(ModelMode::NO_TARGET);
        return false;
    }
    else if (dt_ > SMALL_NUMBER && object_in_focus_.dv_s > 0.0 && object_in_focus_.dist_long > 0)
    {
        cfs_ = CFS(object_in_focus_.dist_long, veh_->GetSpeed(), object_in_focus_.obj->GetSpeed(), rt_, br_min_, br_max_, veh_->pos_.GetAccS());
        pfs_ = PFS(object_in_focus_.dist_long,
                   veh_->GetSpeed(),
                   object_in_focus_.obj->GetSpeed(),
                   rt_,
                   br_min_,
                   br_max_,
                   bl_,
                   margin_dist_,
                   margin_safe_dist_);

        R157_LOG(3, "cfs %.2f pfs %.2f", cfs_, pfs_);

        if (cfs_ + pfs_ < SMALL_NUMBER)
        {
            SetModelMode(ModelMode::TARGET_IN_SIGHT);
            return false;
        }

        if (GetModelMode() != ModelMode::CRITICAL)
        {
            ResetReactionTime();
        }
        SetModelMode(ModelMode::CRITICAL);
        return true;
    }

    SetModelMode(ModelMode::TARGET_IN_SIGHT);
    return false;
}

double ControllerALKS_R157SM::FSM::ReactCritical()
{
    if (rt_counter_ > 0.0)
    {
        acc_ = MIN(acc_, 0.0);  // release gas pedal
        rt_counter_ -= dt_;
        return veh_->GetSpeed() + acc_ * dt_;  // no reaction yet
    }

    double acc = 0.0;
    if (cfs_ > SMALL_NUMBER)
    {
        acc = cfs_ * (br_max_ - br_min_) + br_min_;
    }
    else
    {
        acc = pfs_ * br_min_;
    }

    acc_ = MAX(MAX(acc_ - min_jerk_ * dt_, -max_dec_), -acc);

    R157_LOG(3, "acc %.2f", acc_);

    return MAX(veh_->GetSpeed() + acc_ * dt_, 0);
}
