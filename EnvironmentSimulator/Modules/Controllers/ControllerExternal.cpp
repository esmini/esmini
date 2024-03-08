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

#include "ControllerExternal.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerExternal(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerExternal(initArgs);
}

ControllerExternal::ControllerExternal(InitArgs* args) : Controller(args), useGhost_(false), headstart_time_(3.0)
{
    if (args && args->properties && args->properties->ValueExists("useGhost"))
    {
        useGhost_ = (args->properties->GetValueStr("useGhost") == "true" || args->properties->GetValueStr("useGhost") == "True");
    }

    if (args && args->properties && args->properties->ValueExists("headstartTime"))
    {
        headstart_time_ = strtod(args->properties->GetValueStr("headstartTime"));
    }
}

void ControllerExternal::Init()
{
    if (object_)
    {
        object_->SetHeadstartTime(headstart_time_);
    }
    else if (useGhost_)
    {
        LOG("External controller with ghost needs to be assigned by ObjectController");
        LOG("  in the Init section, in order for the headstart time to be correctly registered.");
        LOG("  -> ignoring the headstartTime attribute.");
    }

    Controller::Init();
}

void ControllerExternal::Step(double timeStep)
{
    if (object_->ghost_)
    {
        if (object_->ghost_->trail_.FindClosestPoint(object_->pos_.GetX(),
                                                     object_->pos_.GetY(),
                                                     object_->trail_closest_pos_,
                                                     object_->trail_follow_index_,
                                                     object_->trail_follow_index_) == 0)
        {
            /*if (object_->ghost_->trail_.FindClosestPoint(object_->pos_.GetX(), object_->pos_.GetY(),
            object_->trail_closest_pos_[0], object_->trail_closest_pos_[1],
            object_->trail_follow_s_, object_->trail_follow_index_, object_->trail_follow_index_) == 0)
    {*/
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

    Controller::Step(timeStep);
}

int ControllerExternal::Activate(ControlActivationMode lat_activation_mode,
                                 ControlActivationMode long_activation_mode,
                                 ControlActivationMode light_activation_mode,
                                 ControlActivationMode anim_activation_mode)
{
    return Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);
}

void ControllerExternal::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}