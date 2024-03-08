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

#include "ControllerOffroadFollower.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

#include <random>

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerOffroadFollower(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerOffroadFollower(initArgs);
}

ControllerOffroadFollower::ControllerOffroadFollower(InitArgs* args)
    : Controller(args),
      follow_entity_(nullptr),
      target_distance_(20.0),
      steering_rate_(4.0),
      speed_factor_(1.0)
{
    if (args && args->properties)
    {
        if (args->properties->ValueExists("followEntity"))
        {
            std::string follow_entity_str = args->properties->GetValueStr("followEntity");

            follow_entity_ = entities_->GetObjectByName(follow_entity_str.c_str());
            if (follow_entity_ == nullptr)
            {
                LOG("Failed to find followEntity %s", follow_entity_str.c_str());
            }
        }

        if (args->properties->ValueExists("targetDistance"))
        {
            target_distance_ = strtod(args->properties->GetValueStr("targetDistance"));
        }

        if (args->properties->ValueExists("steeringRate"))
        {
            steering_rate_ = strtod(args->properties->GetValueStr("steeringRate"));
        }

        if (args->properties->ValueExists("speedFactor"))
        {
            speed_factor_ = strtod(args->properties->GetValueStr("speedFactor"));
        }
    }
}

void ControllerOffroadFollower::Init()
{
    Controller::Init();
}

void ControllerOffroadFollower::Step(double timeStep)
{
    if (follow_entity_ == nullptr)
    {
        return;
    }

    // allow follower vehicle to slightly drive faster than speed limit in order to catch lead vehicle
    vehicle_.SetMaxSpeed(MIN(1.2 * object_->pos_.GetSpeedLimit(), object_->performance_.maxSpeed));

    // Find relative angle to lead vehicle
    double direction =
        GetAngleDifference(GetAngleOfVector(follow_entity_->pos_.GetX() - object_->pos_.GetX(), follow_entity_->pos_.GetY() - object_->pos_.GetY()),
                           object_->pos_.GetH());

    // Find simple distance to lead vehicle (between reference points)
    double dist = GetLengthOfVector2D(follow_entity_->pos_.GetX() - object_->pos_.GetX(), follow_entity_->pos_.GetY() - object_->pos_.GetY());

    // Update vehicle motion
    double steering = ABS_LIMIT(direction, 1.0);
    double throttle = ABS_LIMIT(0.5 * (dist - target_distance_) - 0.5 * (object_->GetSpeed() - follow_entity_->GetSpeed()), 1.0);

    vehicle_.DrivingControlAnalog(timeStep, throttle, steering);

    // Finally, report updated entity data
    gateway_->updateObjectWorldPosMode(object_->id_,
                                       0.0,
                                       vehicle_.posX_,
                                       vehicle_.posY_,
                                       0.0,
                                       vehicle_.heading_,
                                       0.0,
                                       0.0,
                                       roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_ABS |
                                           roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_ABS);

    gateway_->updateObjectSpeed(object_->id_, 0.0, vehicle_.speed_);
    gateway_->updateObjectWheelAngle(object_->id_, 0.0, vehicle_.wheelAngle_);

    // run step method of the base class
    Controller::Step(timeStep);
}

int ControllerOffroadFollower::Activate(ControlActivationMode lat_activation_mode,
                                        ControlActivationMode long_activation_mode,
                                        ControlActivationMode light_activation_mode,
                                        ControlActivationMode anim_activation_mode)
{
    if (object_)
    {
        vehicle_.Reset();
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
        vehicle_.speed_ = object_->GetSpeed();
        vehicle_.SetMaxAcc(object_->GetMaxAcceleration());
        vehicle_.SetMaxDec(object_->GetMaxDeceleration());
        vehicle_.SetSteeringRate(steering_rate_);
    }

    object_->SetJunctionSelectorStrategy(roadmanager::Junction::JunctionStrategyType::SELECTOR_ANGLE);
    object_->SetJunctionSelectorAngle(0.0);

    return Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);
}

void ControllerOffroadFollower::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}