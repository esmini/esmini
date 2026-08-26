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

// minimum distance from front axle to the steering lookahead point
static const double LOOKAHEAD_STEER_FRONT_AXLE_MARGIN = 1.0;

// limit for the angle towards the lookahead point, as seen from front axle
static const double LOOKAHEAD_STEER_MAX_ANGLE = M_PI_4;

// settings for steering filter, modeled as a damped spring
static const double MIN_STEERING_TENSION = 10.0;
static const double MAX_STEERING_TENSION = 200.0;

Controller* scenarioengine::InstantiateControllerFollowRoad(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerFollowRoad(initArgs);
}

ControllerFollowRoad::ControllerFollowRoad(InitArgs* args) : Controller(args), steering_filter_(0.0, 0.0, 100 * (1 - steer_filter_))
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

    if (args->properties->ValueExists("steerFilter"))
    {
        steer_filter_ = strtod(args->properties->GetValueStr("steerFilter"));
        if (steer_filter_ < 0.0 || steer_filter_ > 1.0)
        {
            LOG_WARN("ControllerFollowRoad: steerFilter must be in [0,1], got {:.2f}, clamping to {:.2f}",
                     steer_filter_,
                     CLAMP(steer_filter_, 0.0, 1.0));
            steer_filter_ = CLAMP(steer_filter_, 0.0, 1.0);
        }
        double tension = MIN_STEERING_TENSION + (MAX_STEERING_TENSION - MIN_STEERING_TENSION) * (1 - steer_filter_);
        LOG_INFO("ControllerFollowRoad: steerFilter: {:.2f} -> steering tension: {:.2f}", steer_filter_, tension);
        steering_filter_.SetTension(tension);
    }

    if (args->properties->ValueExists("acceleration"))
    {
        acc_ = strtod(args->properties->GetValueStr("acceleration"));
    }

    LOG_INFO(
        "ControllerFollowRoad: lookaheadSpeedDistFactor: {:.2f}, lookaheadSteerDistFactor: {:.2f}, speed_change_factor: {:.2f}, steer_filter: {:.2f}",
        lookahead_speed_dist_factor_,
        lookahead_steer_dist_factor_,
        speed_change_factor_,
        steer_filter_);
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
        speed_sensor_idx_ = object_->AddCustomSensor(SE_Color::Color2RBG(SE_Color::Color::RED),
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
            object_->SetCustomSensorPosition(speed_sensor_idx_,
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

    // steering angle based on angle to lookahead point, as seen from mid front axle
    double wheelbase              = object_->front_axle_.positionX;
    double min_lookahead_distance = wheelbase + LOOKAHEAD_STEER_FRONT_AXLE_MARGIN;
    lookahead_steer_dist          = MAX(1 + lookahead_steer_dist_factor_ * sqrt(object_->GetSpeed()), min_lookahead_distance);
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

    // lookahead point in vehicle coordinate system, origin at mid rear axle
    double target_x = lookahead_info.relative_pos[0];
    double target_y = lookahead_info.relative_pos[1];

    if (target_x < min_lookahead_distance)
    {
        // move the point along the line from reference point, until it ends up ahead of the front axle
        if (target_x < SMALL_NUMBER)
        {
            // point is not ahead at all, no line to move along, so aim aside
            target_x = min_lookahead_distance;
        }
        else
        {
            double scale_factor = min_lookahead_distance / target_x;
            target_x *= scale_factor;
            target_y *= scale_factor;
        }
    }

    double aim_angle          = atan2(target_y, target_x - wheelbase);  // from front axle to target point
    double steer_target_angle = CLAMP(aim_angle, -LOOKAHEAD_STEER_MAX_ANGLE, LOOKAHEAD_STEER_MAX_ANGLE);

    if (steer_filter_ > SMALL_NUMBER)
    {
        steering_filter_.SetTargetValue(steer_target_angle);
        steering_filter_.Update(timeStep);
        vehicle_.SetWheelAngle(steering_filter_.GetValue());
    }
    else
    {
        vehicle_.SetWheelAngle(steer_target_angle);
    }

    vehicle_.Update(timeStep);

    LOG_DEBUG(
        "road follow ctrl: set_speed: {:.2f} speed_lookahead_dist {:.2f} slowdown factor {:.2f} target speed {:.2f} acc {:.2f} speed {:.2f} steer filter: {} steer_lookahead_dist {:.2f} steer_angle {:.2f}",
        set_speed_,
        lookahead_speed_dist,
        slowdown_factor,
        target_speed,
        acc,
        vehicle_.speed_,
        steer_filter_ > SMALL_NUMBER ? "On" : "Off",
        lookahead_steer_dist,
        steer_target_angle);

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
        object_->dirty_.SetBits(Object::DirtyBit::WHEEL_ROTATION);
    }

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        object_->wheel_angle_ = vehicle_.wheelAngle_;
        object_->dirty_.SetBits(Object::DirtyBit::WHEEL_ANGLE);
    }

    Controller::Step(timeStep);
}

int ControllerFollowRoad::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    if (mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)] == ControlActivationMode::ON &&
        mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)] == ControlActivationMode::OFF)
    {
        LOG_ERROR("{} activation mode: lat {} long {}, controller not designed for only longitudinal control. Expect strange result.",
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
