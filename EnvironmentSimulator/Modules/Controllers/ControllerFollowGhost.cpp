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

// #include "playerbase.hpp"
#include "ControllerFollowGhost.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "ScenarioEngine.hpp"
#include "logger.hpp"

using namespace scenarioengine;

#define FOLLOW_GHOST_BY_POSITION  // comment out to run time based following instead

Controller* scenarioengine::InstantiateControllerFollowGhost(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerFollowGhost(initArgs);
}

ControllerFollowGhost::ControllerFollowGhost(InitArgs* args)
    : Controller(args),
      headstart_time_(3.0),
      follow_mode_(FollowMode::FOLLOW_MODE_TIME),
      lookahead_speed_(0.1),
      min_lookahead_speed_(0.1),
      lookahead_steering_(1.0),
      min_lookahead_steering_(1.0),
      steering_speed_inertia_(0.01)
{
    if (args->properties->ValueExists("headstartTime"))
    {
        headstart_time_ = strtod(args->properties->GetValueStr("headstartTime"));
    }

    if (args->properties->ValueExists("followMode"))
    {
        std::string follow_mode = args->properties->GetValueStr("followMode");
        if (follow_mode == "time")
        {
            follow_mode_ = FollowMode::FOLLOW_MODE_TIME;
        }
        else if (follow_mode == "position")
        {
            follow_mode_ = FollowMode::FOLLOW_MODE_POSITION;
        }
        else
        {
            LOG_INFO("Unexpected follow mode \"{}\", falling back to default \"time\"", follow_mode);
            follow_mode_ = FollowMode::FOLLOW_MODE_TIME;
        }
    }

    if (args->properties->ValueExists("lookaheadSpeed"))
    {
        lookahead_speed_ = strtod(args->properties->GetValueStr("lookaheadSpeed"));
    }

    if (args->properties->ValueExists("minLookaheadSpeed"))
    {
        min_lookahead_speed_ = strtod(args->properties->GetValueStr("minLookaheadSpeed"));
    }

    if (args->properties->ValueExists("lookaheadSteering"))
    {
        lookahead_steering_ = strtod(args->properties->GetValueStr("lookaheadSteering"));
    }

    if (args->properties->ValueExists("minLookaheadSteering"))
    {
        min_lookahead_steering_ = strtod(args->properties->GetValueStr("minLookaheadSteering"));
    }

    if (args->properties->ValueExists("steeringSpeedInertia"))
    {
        steering_speed_inertia_ = strtod(args->properties->GetValueStr("steeringSpeedInertia"));
    }
}

void ControllerFollowGhost::Init()
{
    // FollowGhost controller forced into override mode - will not perform any scenario actions
    if (mode_ != ControlOperationMode::MODE_OVERRIDE)
    {
        LOG_INFO("FollowGhost controller mode \"{}\" not applicable. Using override mode instead.", Mode2Str(mode_));
        mode_ = ControlOperationMode::MODE_OVERRIDE;
    }

    object_->SetHeadstartTime(headstart_time_);

    Controller::Init();
}

void ControllerFollowGhost::Step(double timeStep)
{
    if (!object_->GetGhost())
    {
        // No ghost associated
        return;
    }

    double currentTime = scenario_engine_->getSimulationTime();
    int    ret_val     = 0;

    if (follow_mode_ == FollowMode::FOLLOW_MODE_POSITION)
    {
        ret_val = object_->GetGhost()->trail_.FindClosestPoint(object_->pos_.GetX(),
                                                               object_->pos_.GetY(),
                                                               object_->trail_closest_pos_,
                                                               object_->trail_follow_index_,
                                                               object_->trail_follow_index_);

        if (ret_val != static_cast<int>(roadmanager::PolyLineBase::GhostTrailReturnCode::GHOST_TRAIL_NO_VERTICES) &&
            ret_val != static_cast<int>(roadmanager::PolyLineBase::GhostTrailReturnCode::GHOST_TRAIL_ERROR))
        {
            object_->trail_closest_pos_.z = object_->pos_.GetZ();
        }
        else
        {
            // Failed find point along trail, copy entity position
            object_->trail_closest_pos_.x = object_->pos_.GetX();
            object_->trail_closest_pos_.y = object_->pos_.GetY();
            object_->trail_closest_pos_.z = object_->pos_.GetZ();
        }
    }

    // Find out a steering target along ghost vehicle trail
    idx_t                   index_out;
    roadmanager::TrajVertex steering_target_point, speed_point;

    // Locate a point at given distance from own vehicle along the ghost trajectory
    if (follow_mode_ == FollowMode::FOLLOW_MODE_POSITION)
    {
        // Set steering target point at a distance ahead proportional to the speed
        double probe_target_distance = MAX(min_lookahead_speed_, lookahead_speed_ * object_->speed_);

        ret_val = object_->GetGhost()->trail_.FindPointAhead(object_->trail_closest_pos_.s,
                                                             probe_target_distance,
                                                             speed_point,
                                                             index_out,
                                                             object_->trail_follow_index_);

        if (ret_val != static_cast<int>(roadmanager::PolyLineBase::GhostTrailReturnCode::GHOST_TRAIL_NO_VERTICES) &&
            ret_val != static_cast<int>(roadmanager::PolyLineBase::GhostTrailReturnCode::GHOST_TRAIL_ERROR))
        {
            probe_target_distance = MAX(min_lookahead_steering_, lookahead_steering_ * object_->speed_);

            ret_val = object_->GetGhost()->trail_.FindPointAhead(object_->trail_closest_pos_.s,
                                                                 probe_target_distance,
                                                                 steering_target_point,
                                                                 object_->trail_follow_index_);
        }
    }
    else
    {
        ret_val = object_->GetGhost()->trail_.FindPointAtTime(currentTime - headstart_time_ + MAX(min_lookahead_speed_, lookahead_speed_),
                                                              speed_point,
                                                              object_->trail_follow_index_);

        if (ret_val != static_cast<int>(roadmanager::PolyLineBase::GhostTrailReturnCode::GHOST_TRAIL_NO_VERTICES) &&
            ret_val != static_cast<int>(roadmanager::PolyLineBase::GhostTrailReturnCode::GHOST_TRAIL_ERROR))
        {
            ret_val = object_->GetGhost()->trail_.FindPointAtTime(currentTime - headstart_time_ + MAX(min_lookahead_steering_, lookahead_steering_),
                                                                  steering_target_point,
                                                                  object_->trail_follow_index_);
        }
    }

    if (ret_val == static_cast<int>(roadmanager::PolyLineBase::GhostTrailReturnCode::GHOST_TRAIL_NO_VERTICES) ||
        ret_val == static_cast<int>(roadmanager::PolyLineBase::GhostTrailReturnCode::GHOST_TRAIL_ERROR))
    {
        steering_target_point.x = static_cast<float>(object_->pos_.GetX());
        steering_target_point.y = static_cast<float>(object_->pos_.GetY());
        steering_target_point.z = static_cast<float>(object_->pos_.GetZ());
        speed_point.speed       = 0;
    }

    // Update object sensor position for visualization
    object_->sensor_pos_[0] = steering_target_point.x;
    object_->sensor_pos_[1] = steering_target_point.y;
    object_->sensor_pos_[2] = steering_target_point.z;

    double diffGlobal[2] = {steering_target_point.x - object_->pos_.GetX(), steering_target_point.y - object_->pos_.GetY()};
    double len           = sqrt(diffGlobal[0] * diffGlobal[0] + diffGlobal[1] * diffGlobal[1]);
    if (len > SMALL_NUMBER)
    {
        diffGlobal[0] /= len;
        diffGlobal[1] /= len;
    }
    else
    {
        diffGlobal[0] = 0;
        diffGlobal[1] = 0;
    }

    // Find heading to the point
    double egoDirGlobal[2];
    RotateVec2D(1.0, 0.0, object_->pos_.GetH(), egoDirGlobal[0], egoDirGlobal[1]);

    double diffH = asin(GetCrossProduct2D(egoDirGlobal[0], egoDirGlobal[1], diffGlobal[0], diffGlobal[1]));

    // Update driver model target values
    vehicle_.DrivingControlTarget(timeStep, speed_point.speed, diffH);

    // Register updated vehicle position

    // Fetch Z and Pitch from road position
    vehicle_.posZ_  = object_->pos_.GetZ();
    vehicle_.pitch_ = object_->pos_.GetP();

    // Register updated vehicle position
    gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
    gateway_->updateObjectSpeed(object_->id_, 0.0, vehicle_.speed_);

    // Update wheels wrt domains
    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)))
    {
        gateway_->updateObjectWheelRotation(object_->id_, 0.0, vehicle_.wheelRotation_);
    }

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        gateway_->updateObjectWheelAngle(object_->id_, 0.0, vehicle_.wheelAngle_);
    }

    Controller::Step(timeStep);
}

int ControllerFollowGhost::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    if (object_)
    {
        vehicle_.Reset();
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
        vehicle_.speed_ = object_->GetSpeed();
        vehicle_.SetMaxSpeed(100);  // just set a random high value
        vehicle_.SetMaxAcc(10.0);
        vehicle_.SetSteeringScale(steering_speed_inertia_);

        object_->sensor_pos_[0] = object_->pos_.GetX();
        object_->sensor_pos_[1] = object_->pos_.GetY();
        object_->sensor_pos_[2] = object_->pos_.GetZ();
    }

    return Controller::Activate(mode);
}

void ControllerFollowGhost::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}