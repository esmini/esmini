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

/*
 * This controller simulates a bad or dizzy driver by manipulating
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

#include "ControllerFollowRoad.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioEngine.hpp"
#include "logger.hpp"
#include "playerbase.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerFollowRoad(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerFollowRoad(initArgs);
}

ControllerFollowRoad::ControllerFollowRoad(InitArgs* args) : Controller(args)
{
    if (args->properties->ValueExists("lookaheadSpeedDistFactor"))
    {
        lookahead_speed_dist_factor_ = strtod(args->properties->GetValueStr("lookaheadSpeedDistFactor"));
    }

    if (args->properties->ValueExists("speedChangeFactor"))
    {
        speed_change_factor_ = strtod(args->properties->GetValueStr("speedChangeFactor"));
    }

    if (args->properties->ValueExists("lookaheadSteerDistFactor"))
    {
        lookahead_steer_dist_factor_ = strtod(args->properties->GetValueStr("lookaheadSteerDistFactor"));
    }

    if (args->properties->ValueExists("steerFactor"))
    {
        steer_factor_ = strtod(args->properties->GetValueStr("steerFactor"));
    }

    if (args->properties->ValueExists("acceleration"))
    {
        acc_ = strtod(args->properties->GetValueStr("acceleration"));
    }

    LOG_INFO("ControllerFollowRoad: lookaheadSpeedDistFactor: {:.2f}, lookaheadSteerDistFactor: {:.2f}, speed_change_factor: {:.2f}",
             lookahead_speed_dist_factor_,
             lookahead_steer_dist_factor_,
             speed_change_factor_);
}

void ControllerFollowRoad::Init()
{
    if (object_ == nullptr)
    {
        LOG_ERROR("FollowRoad controller requires a valid object to control");
        return;
    }

    if (!NEAR_ZERO(speed_change_factor_))
    {
        sensor_idx_ = object_->AddCustomSensor(SE_Color::Color2RBG(SE_Color::Color::RED),
                                               {object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ()},
                                               0.5);
    }

    Controller::Init();
}

void ControllerFollowRoad::Step(double timeStep)
{
    if (object_ == nullptr)
    {
        return;
    }

    roadmanager::RoadProbeInfo lookahead_info       = {};
    double                     lookahead_speed_dist = 5.0;
    double                     slowdown_factor      = 1.0;
    double                     target_speed         = 0.0;
    double                     lookahead_steer_dist = 0.0;
    double                     acc                  = 0.0;

    if (!NEAR_ZERO(speed_change_factor_))
    {
        if (abs(object_->GetSpeed() - current_speed_) > 1e-3)
        {
            LOG_INFO("New setspeed: {:5.2f} -> {:5.2f}", set_speed_, object_->GetSpeed());
            set_speed_ = object_->GetSpeed();
        }

        // lookahead for speed control
        double speed_dist_tune = 1.0;
        lookahead_speed_dist   = 5.0 + speed_dist_tune * lookahead_speed_dist_factor_ * object_->GetSpeed() + 0.05 * pow(object_->GetSpeed(), 2);
        if (object_->pos_.GetProbeInfo(lookahead_speed_dist,
                                       &lookahead_info,
                                       roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER,
                                       true) < roadmanager::Position::ReturnCode::OK)
        {
            LOG_WARN("Failed lookahead for speed control");
        }
        else
        {
            // visualize lookahead speed sensor point
            object_->SetCustomSensorPosition(sensor_idx_,
                                             lookahead_info.road_lane_info.pos[0],
                                             lookahead_info.road_lane_info.pos[1],
                                             lookahead_info.road_lane_info.pos[2]);
        }

        // calculate speed based on steering angle and road curvature ahead
        double max_lat_acc = 1.5;

        // for speed control consider curvatures 1. at lookahead point, 2. at current position, 3. given by heading to targetpoint, pick maximum
        double pure_pursuit_curv =
            2 * sin(abs(lookahead_info.relative_h)) / lookahead_speed_dist;  // based on circle tangenting current and lookahead point
        double max_curv           = MAX(pure_pursuit_curv, MAX(abs(lookahead_info.road_lane_info.curvature), abs(object_->pos_.GetCurvature())));
        double max_speed_wrt_curv = NEAR_ZERO(max_curv) ? LARGE_NUMBER : sqrt(max_lat_acc / max_curv);
        if (object_->GetSpeed() > max_speed_wrt_curv)
        {
            LOG_DEBUG("speed {:.2f} > max_speed_wrt_curv {:.2f} dh {:.2f} curvatures: pure_pursuit {:.2f}, lookahead {:.2f}, current {:.2f}",
                      object_->GetSpeed(),
                      max_speed_wrt_curv,
                      lookahead_info.relative_h,
                      pure_pursuit_curv,
                      abs(lookahead_info.road_lane_info.curvature),
                      object_->pos_.GetCurvature());
            slowdown_factor = MIN(1.0, exp(speed_change_factor_ * -5 * max_curv));
        }
        target_speed = set_speed_ * slowdown_factor;

        if (vehicle_.speed_ < target_speed - SMALL_NUMBER)
        {
            acc = acc_;
            vehicle_.SetSpeed(MIN(target_speed, vehicle_.speed_ + timeStep * acc));
        }
        else if (vehicle_.speed_ > target_speed + SMALL_NUMBER)
        {
            acc = -acc_;
            vehicle_.SetSpeed(MAX(target_speed, vehicle_.speed_ + timeStep * acc));
        }
        else
        {
            vehicle_.SetSpeed(target_speed);
        }

        object_->SetSpeed(vehicle_.speed_);
        current_speed_ = vehicle_.speed_;
        object_->dirty_.SetBits(Object::DirtyBit::LONGITUDINAL);
    }
    else
    {
        // just apply current speed
        target_speed = object_->GetSpeed();
        vehicle_.SetSpeed(target_speed);
    }

    // steering angle based on relative heading to lookahead point
    lookahead_steer_dist = 1 + lookahead_steer_dist_factor_ * sqrt(object_->GetSpeed());
    if (object_->pos_.GetProbeInfo(lookahead_steer_dist, &lookahead_info, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER, true) <
        roadmanager::Position::ReturnCode::OK)
    {
        LOG_WARN("Failed lookahead for steering angle");
    }
    else
    {
        // visualize lookahead point for steering
        object_->SetLookaheadSensorPosition(lookahead_info.road_lane_info.pos[0],
                                            lookahead_info.road_lane_info.pos[1],
                                            lookahead_info.road_lane_info.pos[2]);
    }
    double tuning_scale_factor = 3.0;
    vehicle_.SetWheelAngle(tuning_scale_factor * steer_factor_ * lookahead_info.relative_h);

    vehicle_.Update(timeStep);

    LOG_DEBUG(
        "road follow ctrl: set_speed: {:.2f} speed_lookahead_dist {:.2f} slowdown factor {:.2f} target speed {:.2f} acc {:.2f} speed {:.2f} steer_lookahead_dist {:.2f} steer_angle {:.2f}",
        set_speed_,
        lookahead_speed_dist,
        slowdown_factor,
        target_speed,
        acc,
        vehicle_.speed_,
        lookahead_steer_dist,
        lookahead_info.relative_h);

    // Register updated vehicle position
    object_->pos_.SetInertiaPos(vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
    object_->dirty_.SetBits(Object::DirtyBit::LATERAL);

    // Fetch Z and Pitch from road position
    vehicle_.posZ_  = object_->pos_.GetZ();
    vehicle_.pitch_ = object_->pos_.GetP();

    // Update wheels wrt domains
    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)))
    {
        object_->wheel_rot_ = vehicle_.wheelRotation_;
    }

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        object_->wheel_angle_ = vehicle_.wheelAngle_;
    }

    Controller::Step(timeStep);
}

int ControllerFollowRoad::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    if (mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)] != mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)])
    {
        LOG_ERROR("{} activation mode: lat {} long {}, but is only valid on both domains in combination. Expect strange result.",
                  GetName(),
                  mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)] == ControlActivationMode::ON ? "On" : "Off",
                  mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)] == ControlActivationMode::ON ? "On" : "Off");
    }

    if (object_)
    {
        vehicle_.Reset();
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
        vehicle_.SetMaxSpeed(object_->GetMaxSpeed());
        vehicle_.SetMaxAcc(object_->GetMaxAcceleration());
        vehicle_.SetMaxDec(object_->GetMaxDeceleration());
        vehicle_.SetSteeringRate(10.0);
        set_speed_ = current_speed_ = vehicle_.speed_ = object_->GetSpeed();
        object_->SetJunctionSelectorStrategy(roadmanager::Junction::JunctionStrategyType::SELECTOR_ANGLE);
        object_->SetJunctionSelectorAngle(0.0);
        player_->LookaheadSensorSetVisible(object_->GetId(), true);
    }

    return Controller::Activate(mode);
}

void ControllerFollowRoad::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}
