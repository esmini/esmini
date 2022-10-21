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

#define R157_LOG(level, format, ...)                                                                  \
    {                                                                                                 \
        if (level > 0 && level <= log_level_)                                                         \
        {                                                                                             \
            LOG((std::string("ALKS R157 ") + GetModelName() + " " + format).c_str(), ##__VA_ARGS__);  \
        }                                                                                             \
        else                                                                                          \
        {                                                                                             \
            (void)0;                                                                                  \
        }                                                                                             \
    }

Controller* scenarioengine::InstantiateControllerALKS_R157SM(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerALKS_R157SM(initArgs);
}

ControllerALKS_R157SM::ControllerALKS_R157SM(InitArgs* args) : model_(0), entities_(0), Controller(args)
{
    if (args && args->properties)
    {
        if (args->properties->GetValueStr("model") == "Regulation")
        {
            model_ = (ControllerALKS_R157SM::Model*) new Regulation();
        }
        else if (args->properties->GetValueStr("model") == "FSM")
        {
            model_ = (ControllerALKS_R157SM::Model*) new FSM();
        }
        else if (args->properties->GetValueStr("model") == "ReferenceDriver")
        {
            model_ = (ControllerALKS_R157SM::Model*) new ReferenceDriver();
        }
        else if (args->properties->GetValueStr("model") == "RSS")
        {
            model_ = (ControllerALKS_R157SM::Model*) new RSS();
        }
        else
        {
            LOG_AND_QUIT("ControllerALKS_R157SM unexpected model %s",
                args->properties->GetValueStr("model").c_str());
        }


        if (args->properties->ValueExists("logLevel"))
        {
            model_->SetLogging(strtoi(args->properties->GetValueStr("logLevel")));
        }

        if (args->properties->ValueExists("cruise"))
        {
            if (args->properties->GetValueStr("cruise") == "true" ||
                args->properties->GetValueStr("cruise") == "True")
            {
                model_->SetCruise(true);
            }
            else if (args->properties->GetValueStr("cruise") == "false" ||
                args->properties->GetValueStr("cruise") == "False")
            {
                model_->SetCruise(false);
            }
        }
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

    if (mode_ == Mode::MODE_OVERRIDE)
    {
        object_->MoveAlongS(speed * timeStep);
        gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    }

    gateway_->updateObjectSpeed(object_->GetId(), 0.0, speed);

    Controller::Step(timeStep);
}

void ControllerALKS_R157SM::Assign(Object* object)
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
        model_->SetVehicle((Vehicle*)object);
    }

    Controller::Assign(object);
}

void ControllerALKS_R157SM::Activate(ControlDomains domainMask)
{
    if (model_)
    {
        model_->set_speed_ = object_->GetSpeed();
    }
    Controller::Activate(domainMask);
}

void ControllerALKS_R157SM::SetScenarioEngine(ScenarioEngine* scenario_engine)
{
    scenario_engine_ = scenario_engine;
    if (model_)
    {
        model_->SetScenarioEngine(scenario_engine);
    }
};

void ControllerALKS_R157SM::ReportKeyEvent(int key, bool down)
{
}

void ControllerALKS_R157SM::Model::Scan()
{
    // Find closest object
    // Consider all vehicles:
    //   - in front AND
    //   - in own lane OR
    //   - in neighbors lanes changing into own lane

    roadmanager::PositionDiff diff;
    double min_dist = LARGE_NUMBER;
    Object* old_object_in_focus = object_in_focus_.obj;
    int index = -1;

    ResetObjectInFocus();

    if (entities_ == 0)
    {
        R157_LOG(1, "No entities! Register scenarioengine - SetScenarioEngine()");
        return;
    }

    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        Object* obj = entities_->object_[i];

        if (obj == nullptr || obj == veh_)
        {
            continue;
        }

        // Measure longitudinal distance to all vehicles, don't utilize costly freespace option, instead measure ref point to ref point
        if (veh_->pos_.Delta(&obj->pos_, diff, false, GetMaxRange()) == true)
        {
            // Adjust delta lane id in case vehicles are on either side of center lane
            if (diff.dLaneId == 2 && obj->pos_.GetLaneId() == 1 && veh_->pos_.GetLaneId() == -1)
            {
                diff.dLaneId = 1;
            }
            else if (diff.dLaneId == -2 && obj->pos_.GetLaneId() == -1 && veh_->pos_.GetLaneId() == 1)
            {
                diff.dLaneId = -1;
            }

            // Accept only objects in front of myself and in my own or neighbor lanes
            if (diff.ds > SMALL_NUMBER && diff.dLaneId > -2 && diff.dLaneId < 2)
            {
                // Calculate exact distance, long and lat
                double dist_lat = LARGE_NUMBER, dist_long = LARGE_NUMBER;
                if (veh_->FreeSpaceDistanceObjectRoadLane(obj, &dist_lat, &dist_long, roadmanager::CoordinateSystem::CS_ROAD) == 0)
                {
                    if (diff.dLaneId == 0 || !CheckLateralSafety(obj, diff.dLaneId, dist_long, dist_lat))
                    {
                        // If closest so far, register it as object to focus at
                        if (dist_long < min_dist)
                        {
                            object_in_focus_.obj = obj;
                            object_in_focus_.dist_long = dist_long;
                            object_in_focus_.dist_lat = dist_lat;
                            object_in_focus_.dv = veh_->GetSpeed() - obj->GetSpeed();
                            object_in_focus_.dLaneId = diff.dLaneId;
                            if (object_in_focus_.dv > 0.0)
                            {
                                object_in_focus_.ttc = dist_long / (veh_->GetSpeed() - obj->GetSpeed());
                            }
                            else
                            {
                                object_in_focus_.ttc = LARGE_NUMBER;
                            }
                            index = (int)i;
                            min_dist = dist_long;
                        }
                    }
                }
            }
        }
    }

    if (object_in_focus_.obj && object_in_focus_.obj != old_object_in_focus)
    {
        if (object_in_focus_.dLaneId != 0)
        {
            cut_in_detected_timestamp_ = scenario_engine_->getSimulationTime();
            R157_LOG(2, "Starting cut-in timer");
        }
        else
        {
            cut_in_detected_timestamp_ = 0.0;
        }
    }
}

double ControllerALKS_R157SM::Model::Step(double timeStep)
{
    dt_ = timeStep;

    Scan();

    if (CheckCritical())
    {
        return ReactCritical();
    }
    else
    {
        return Cruise();
    }
}

void ControllerALKS_R157SM::Model::ResetObjectInFocus()
{
    object_in_focus_ = { nullptr, LARGE_NUMBER, LARGE_NUMBER, 0.0, LARGE_NUMBER, 0 };
}

void ControllerALKS_R157SM::Model::ResetReactionTime()
{
    R157_LOG(2, "Reaction timer (%.2fs) started", GetReactionTime());
    rt_counter_ = GetReactionTime();
}


ControllerALKS_R157SM::Model::Model(ModelType type, double reaction_time,
    double max_dec_, double max_range_) : type_(type), rt_(reaction_time), entities_(0),
    rt_counter_(0.0), max_dec_(max_dec_), max_range_(max_range_), max_acc_(3.0), max_acc_lat_(1.0), veh_(nullptr),
    set_speed_(0.0), log_level_(1),model_mode_(ModelMode::CRUISE_NO_TARGET), acc_(0.0), cruise_comfort_acc_(2.0),
    cut_in_detected_timestamp_(0.0), cruise_comfort_dec_(2.0), cruise_max_acc_(3.0), cruise_max_dec_(4.0), cruise_(true)
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
            SetMode(ModelMode::CRUISE_NO_TARGET);

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
        else if (object_in_focus_.dist_long > 0 && object_in_focus_.ttc > object_in_focus_.dv / 12.0 + 0.35)
        {
            SetMode(ModelMode::CRUISE_WITH_TARGET);

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

                acc_ = A * -object_in_focus_.dv + B * (object_in_focus_.dist_long - MinDist());
                acc_ = CLAMP(acc_, -cruise_max_dec_, cruise_max_acc_);
            }

            R157_LOG(2, "Cruise with target, acc: %.2f", acc_);
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
    else if (mode == ModelMode::CRUISE_NO_TARGET)
    {
        return "CRUISE_NO_TARGET";
    }
    else if (mode == ModelMode::CRUISE_WITH_TARGET)
    {
        return "CRUISE_WITH_TARGET";
    }

    return "Unknown mode " + std::to_string(static_cast<int>(mode));
}

void ControllerALKS_R157SM::Model::SetMode(ModelMode mode, bool log)
{
    if (log)
    {
        if (mode != model_mode_)
        {
            R157_LOG(1, "R157_Model mode: %s", Mode2Str(mode).c_str());
        }
    }
    model_mode_ = mode;
}

void ControllerALKS_R157SM::Model::SetScenarioEngine(ScenarioEngine* scenario_engine)
{
    scenario_engine_ = scenario_engine;
    entities_ = &scenario_engine_->entities_;
};

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
bool ControllerALKS_R157SM::Regulation::CheckLateralSafety(Object* obj, int dLaneId, double dist_long, double dist_lat)
{
    if (obj == nullptr || dist_lat == LARGE_NUMBER)
    {
        return true;
    }

    // Check if car's front wheel has moved 0.3 meter inside own vehicle's lane
    roadmanager::Road* road = veh_->pos_.GetOpenDrive()->GetRoadById(obj->pos_.GetTrackId());
    if (road && dist_lat > -SMALL_NUMBER)
    {
        double lane_width = road->GetLaneWidthByS(obj->pos_.GetS(), obj->pos_.GetLaneId());
        double x = 0.0;
        if (obj->type_ == Object::Type::VEHICLE)
        {
            // Find out position of front wheel (axis)
            x = ((Vehicle*)obj)->front_axle_.positionX;
        }
        else
        {
            // Use front side of bounding box
            x = obj->boundingbox_.dimensions_.length_ / 2.0;
        }

        // Look at side towards Ego vehicle lane
        double y = -1 * SIGN(dLaneId) * obj->boundingbox_.dimensions_.width_ / 2.0;
        double xr = 0.0, yr = 0.0;
        RotateVec2D(x, y, obj->pos_.GetHRelative(), xr, yr);
        double offset = obj->pos_.GetOffset() + yr;

        // adjust sign of offset wrt side towards Ego vehicle
        offset *= -1 * SIGN(dLaneId);

        // Check if wheel is 0.3 meter inside Ego lane
        if (offset > lane_width / 2.0 + 0.3)
        {
            return false;
        }
    }

    return true;
}

bool ControllerALKS_R157SM::Regulation::CheckCritical()
{
    if (object_in_focus_.obj == nullptr)
    {
        SetMode(ModelMode::CRUISE_NO_TARGET);
        return false;
    }
    else if (object_in_focus_.dv > 0.0 && object_in_focus_.dist_long > 0)
    {
        // TTCLaneIntrusion < vrel / (2 x 6 ms^2) + 0.35s?
        if (object_in_focus_.ttc < object_in_focus_.dv / (2 * GetMaxDec()) + GetReactionTime() + 0.1)
        {
            SetMode(ModelMode::CRITICAL);
            return true;
        }
    }

    SetMode(ModelMode::CRUISE_WITH_TARGET);
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
    acc_ = -GetMaxDec();
    double speed = MAX(0.0, veh_->GetSpeed() + acc_ * dt_);

    R157_LOG(2, "Critical: acc %.2f speed %.2f", acc_, speed);

    return speed;
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckLateralSafety(Object* obj, int dLaneId, double dist_long, double dist_lat)
{
    if (obj == nullptr || dist_lat == LARGE_NUMBER)
    {
        return true;
    }

    if (dist_lat < SMALL_NUMBER)
    {
        return false;
    }

    // Check if lead car cut-in has been perceived (wandering zone 0.375m + perception distance 0.72m)
    if (SIGN(dLaneId) * obj->pos_.GetOffset() < -(0.375 + 0.72))
    {
        return false;
    }

    return true;
}

void ControllerALKS_R157SM::ReferenceDriver::SetPhase(Phase phase)
{
    phase_ = phase;
    R157_LOG(2, "phase: %s", Phase2Str(phase).c_str());
}

std::string ControllerALKS_R157SM::ReferenceDriver::Phase2Str(Phase phase)
{
    if (phase == Phase::INACTIVE)
    {
        return "INACTIVE";
    }
    else if (phase == Phase::REACT)
    {
        return "REACT";
    }
    else if (phase == Phase::BRAKE_REF)
    {
        return "BRAKE_REF";
    }
    else if (phase == Phase::BRAKE_AEB)
    {
        return "BRAKE_AEB";
    }

    return "Unknown phase";
}

bool ControllerALKS_R157SM::ReferenceDriver::CheckCritical()
{
    if (object_in_focus_.obj == nullptr)
    {
        SetMode(ModelMode::CRUISE_NO_TARGET);
        return false;
    }
    else if (veh_->GetSpeed() > SMALL_NUMBER && object_in_focus_.dist_long > 0)
    {
        if (model_mode_ == ModelMode::CRITICAL && object_in_focus_.dist_long < MinDist())
        {
            // Critical until min distance achieved
            return true;
        }

        // TTC < 2.0s?
        if (object_in_focus_.ttc < 2.0)
        {
            if (model_mode_ != ModelMode::CRITICAL)
            {
                Reset();
                SetMode(ModelMode::CRITICAL);
            }
            return true;
        }
    }

    SetMode(ModelMode::CRUISE_WITH_TARGET);
    return false;
}

double ControllerALKS_R157SM::ReferenceDriver::ReactCritical()
{
    // step state machine
    if (phase_ == Phase::INACTIVE)
    {
        SetPhase(Phase::REACT);
        timer_ = 0.75;
    }
    else if (phase_ == Phase::REACT)
    {
        timer_ -= dt_;
        if (timer_ < SMALL_NUMBER)
        {
            SetPhase(Phase::BRAKE_REF);
        }
    }
    else if (phase_ == Phase::BRAKE_REF || phase_ == Phase::BRAKE_AEB)
    {
        if (veh_->GetSpeed() < SMALL_NUMBER)
        {
            SetPhase(Phase::INACTIVE);
            timer_ = 0.0;
        }
        else if (phase_ == Phase::BRAKE_REF)
        {
            // Time for AEB? Calculate required deceleration to reach same speed without collision
            // Equations:
            //   1. s = dv0 * 0.6 + dv 0 * t + 1/2 a0 * t^2
            //   2. dv = 0 = dv0 + a0 * t
            //   a = -(5 v ^ 2) / (10 s - 6 v)
            double required_deceleration = (5 * pow(object_in_focus_.dv, 2)) / (10 * object_in_focus_.dist_long - 6 * object_in_focus_.dv);
            R157_LOG(2, "req dec: %.2f", required_deceleration);
            if (required_deceleration > 0.85 * g - SMALL_NUMBER)
            {
                SetPhase(Phase::BRAKE_AEB);
            }
        }
    }

    // Act
    if (phase_ == Phase::REACT)
    {
        // if already decelerating, keep deceleration rate
        // otherwise apply idle deceleration from lifting foot from gas pedal
        acc_ = MIN(-release_deceleration_, acc_);
    }
    if (phase_ == Phase::BRAKE_REF)
    {
        acc_ -= dt_ * 0.774 * g / 0.6;

        if (acc_ < -0.774 * g)
        {
            acc_ = -0.774 * g;
        }
    }
    else if (phase_ == Phase::BRAKE_AEB)
    {
        acc_ -= dt_ * 0.85 * g / 0.6;

        if (acc_ < -0.85 * g)
        {
            acc_ = -0.85 * g;
        }
    }

    double speed = MAX(0.0, veh_->GetSpeed() + acc_ * dt_);

    R157_LOG(2, "critical, acc %.2f vel %.2f timer %.2f", acc_, speed, timer_);

    return speed;
}

double ControllerALKS_R157SM::ReferenceDriver::MinDist()
{
    double min_dist = 2.0 + veh_->GetSpeed() * 2.0;

    //R157_LOG(1, "Min dist: %.2f", min_dist);

    return min_dist;
}

bool ControllerALKS_R157SM::RSS::CheckLateralSafety(Object* obj, int dLaneId, double dist_long, double dist_lat)
{
    if (obj == nullptr || dist_lat == LARGE_NUMBER)
    {
        return true;
    }

    if (dist_lat < SMALL_NUMBER)
    {
        return false;
    }

    double cut_in_lat = abs(obj->pos_.GetVelT());
    double d_safe_rss_lat = mu_ + abs(
        (2.0 * cut_in_lat + max_acc_lat_ * rt_) * rt_ / 2.0) +
        pow(cut_in_lat + max_acc_lat_ * rt_, 2) / (2 * max_acc_lat_);

    R157_LOG(2, "CheckLateralSafety: dist_lat % .2f d_safe_rss_lat %.2f", dist_lat, d_safe_rss_lat);

    if (abs(dist_lat) < d_safe_rss_lat)
    {
        return false;
    }

    return true;
}

bool ControllerALKS_R157SM::RSS::CheckCritical()
{
    if (object_in_focus_.obj == nullptr)
    {
        SetMode(ModelMode::CRUISE_NO_TARGET);
        return false;
    }

    if (object_in_focus_.dv > 0.0 && object_in_focus_.dist_long > 0)
    {
        double d_safe_rss_long = veh_->GetSpeed() * rt_ + max_acc_ * pow(rt_, 2) / 2.0 +
            pow(veh_->GetSpeed() + rt_ * max_acc_, 2) / (2 * max_dec_) -
            pow(object_in_focus_.obj->GetSpeed(), 2) / (2 * max_dec_);

        R157_LOG(2, "CheckCritical: dist %.2f d_safe_rss_long %.2f", object_in_focus_.dist_long, d_safe_rss_long);

        if (object_in_focus_.dist_long < d_safe_rss_long)
        {
            if (model_mode_ != ModelMode::CRITICAL)
            {
                ResetReactionTime();
            }
            SetMode(ModelMode::CRITICAL);
            return true;
        }
    }

    SetMode(ModelMode::CRUISE_WITH_TARGET);
    return false;
}

double ControllerALKS_R157SM::RSS::ReactCritical()
{
    if (rt_counter_ > 0)
    {
        acc_ = MIN(acc_, 0.0);    // release gas pedal
        rt_counter_ -= dt_;
        return veh_->GetSpeed() + acc_ * dt_;
    }

    acc_ = MIN(acc_ - min_jerk_ * dt_, max_dec_);
    R157_LOG(2, "critical acc %.2f", acc_);

    return MAX(veh_->GetSpeed() + acc_ * dt_, 0);
}

double ControllerALKS_R157SM::RSS::MinDist()
{
    double min_dist = veh_->GetSpeed() * rt_ * rt_ + max_acc_ * pow(rt_, 2) / 2 +
        pow(veh_->GetSpeed() + rt_ * max_acc_, 2) / (2 * max_dec_) +
        -pow(MAX(veh_->GetSpeed() - rt_ * max_acc_ * max_dec_, 0), 2) /
        (2 * max_dec_) + 1.0 / 2 * max_dec_ * pow(rt_, 2);

    return min_dist;
}

double ControllerALKS_R157SM::FSM::MinDist()
{
    double min_dist = margin_dist_ +
        veh_->GetSpeed() * rt_ + pow(veh_->GetSpeed(), 2) / (2 * br_min_) -
        pow(object_in_focus_.obj->GetSpeed(), 2) / (2 * bl_) + margin_safe_dist_;

    R157_LOG(2, "Min dist: %.2f", min_dist);

    return min_dist;
}

double ControllerALKS_R157SM::FSM::PFS(double dist, double speed_rear, double speed_lead, double rt, double br_min, double br_max,
    double bl, double margin_dist, double margin_safe_dist)
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

double ControllerALKS_R157SM::FSM::CFS(double dist, double speed_rear, double speed_lead,
    double rt, double br_min, double br_max, double acc)
{
    double arF = MAX(acc, -br_min);  // Do not take into account very hard decelerations
    double u_new = speed_rear + rt * arF;   // Estimate rear vehicle new speed
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

bool ControllerALKS_R157SM::FSM::CheckLateralSafety(Object* obj, int dLaneId, double dist_long, double dist_lat)
{
    if (obj == nullptr || dist_lat == LARGE_NUMBER)
    {
        return true;
    }

    if (dist_lat > -SMALL_NUMBER)
    {
        double cutin_speed = -SIGN(dLaneId) * obj->pos_.GetVelT();
        if (cutin_speed > 0)
        {
            double headway_lat = abs(dist_lat) / cutin_speed;
            double headway_long_gross = abs(dist_long) /
                MAX(SMALL_NUMBER, veh_->GetSpeed() - obj->GetSpeed());

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

    return false;
}

bool ControllerALKS_R157SM::FSM::CheckCritical()
{
    if (object_in_focus_.obj == nullptr)
    {
        SetMode(ModelMode::CRUISE_NO_TARGET);
        return false;
    }
    else if (dt_ > SMALL_NUMBER && object_in_focus_.dv > 0.0 && object_in_focus_.dist_long > 0)
    {
        cfs_ = CFS(object_in_focus_.dist_long, veh_->GetSpeed(), object_in_focus_.obj->GetSpeed(),
            rt_, br_min_, br_max_, veh_->pos_.GetAccS());
        pfs_ = PFS(object_in_focus_.dist_long, veh_->GetSpeed(), object_in_focus_.obj->GetSpeed(),
            rt_, br_min_, br_max_, bl_, margin_dist_, margin_safe_dist_);

        R157_LOG(2, "cfs %.2f pfs %.2f", cfs_, pfs_);

        if (cfs_ + pfs_ < SMALL_NUMBER)
        {
            SetMode(ModelMode::CRUISE_WITH_TARGET);
            return false;
        }

        if (model_mode_ != ModelMode::CRITICAL)
        {
            ResetReactionTime();
        }
        SetMode(ModelMode::CRITICAL);
        return true;
    }

    SetMode(ModelMode::CRUISE_WITH_TARGET);
    return false;
}


double ControllerALKS_R157SM::FSM::ReactCritical()
{
    if (rt_counter_ > 0.0)
    {
        acc_ = MIN(acc_, 0.0);    // release gas pedal
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

    R157_LOG(2, "acc %.2f", acc_);

    return MAX(veh_->GetSpeed() + acc_ * dt_, 0);
}
