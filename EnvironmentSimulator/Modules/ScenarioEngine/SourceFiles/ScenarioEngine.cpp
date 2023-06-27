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

#include "ScenarioEngine.hpp"
#include "CommonMini.hpp"
#include "ControllerFollowGhost.hpp"
#include "ControllerExternal.hpp"
#include "ControllerRel2Abs.hpp"
#include "ControllerFollowRoute.hpp"
#include "OSCParameterDistribution.hpp"

#define WHEEL_RADIUS          0.35
#define STAND_STILL_THRESHOLD 1e-3  // meter per second

using namespace scenarioengine;

static CallBack paramDeclCallback = {0, 0};

namespace scenarioengine
{
    void RegisterParameterDeclarationCallback(ParamDeclCallbackFunc func, void* data)
    {
        paramDeclCallback.func = func;
        paramDeclCallback.data = data;
    }
}  // namespace scenarioengine

ScenarioEngine::ScenarioEngine(std::string oscFilename, bool disable_controllers)
{
    init_status_ = InitScenario(oscFilename, disable_controllers);
}

ScenarioEngine::ScenarioEngine(const pugi::xml_document& xml_doc, bool disable_controllers)
{
    init_status_ = InitScenario(xml_doc, disable_controllers);
}

void ScenarioEngine::InitScenarioCommon(bool disable_controllers)
{
    quit_flag            = false;
    init_status_         = 0;
    disable_controllers_ = disable_controllers;
    headstart_time_      = 0;
    simulationTime_      = 0;
    trueTime_            = 0;
    frame_nr_            = 0;
    ghost_mode_          = GhostMode::NORMAL;
    scenarioReader       = new ScenarioReader(&entities_, &catalogs, disable_controllers);
}

int ScenarioEngine::InitScenario(std::string oscFilename, bool disable_controllers)
{
    InitScenarioCommon(disable_controllers);

    std::vector<std::string> file_name_candidates;

    // Filename as is - look in current directory
    file_name_candidates.push_back(oscFilename);

    // Finally check registered paths
    for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
    {
        file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], oscFilename));
    }
    size_t i;
    for (i = 0; i < file_name_candidates.size(); i++)
    {
        if (FileExists(file_name_candidates[i].c_str()))
        {
            if (scenarioReader->loadOSCFile(file_name_candidates[i].c_str()) != 0)
            {
                LOG(("Failed to load OpenSCENARIO file " + oscFilename).c_str());
                return -3;
            }
            else
            {
                break;
            }
        }
    }

    if (i == file_name_candidates.size())
    {
        LOG(("Couldn't locate OpenSCENARIO file " + oscFilename).c_str());
        return -1;
    }

    if (!scenarioReader->IsLoaded())
    {
        LOG(("Couldn't load OpenSCENARIO file " + oscFilename).c_str());
        return -2;
    }

    return parseScenario();
}

int ScenarioEngine::InitScenario(const pugi::xml_document& xml_doc, bool disable_controllers)
{
    InitScenarioCommon(disable_controllers);

    if (scenarioReader->loadOSCMem(xml_doc) != 0)
    {
        return -3;
    }

    return parseScenario();
}

ScenarioEngine::~ScenarioEngine()
{
    scenarioReader->UnloadControllers();
    delete scenarioReader;
    scenarioReader = 0;
    LOG("Closing");
}

void ScenarioEngine::UpdateGhostMode()
{
    if (ghost_mode_ == GhostMode::RESTART)
    {
        simulationTime_ -= headstart_time_;
        ghost_mode_ = GhostMode::RESTARTING;
    }
    else if (ghost_mode_ == GhostMode::RESTARTING)
    {
        if (simulationTime_ > trueTime_ - SMALL_NUMBER)
        {
            ghost_mode_ = GhostMode::NORMAL;
        }
    }
}

int ScenarioEngine::step(double deltaSimTime)
{
    UpdateGhostMode();

    if (frame_nr_ == 0)
    {
        // kick off init actions
        for (size_t i = 0; i < init.private_action_.size(); i++)
        {
            init.private_action_[i]->Start(simulationTime_, deltaSimTime);
            init.private_action_[i]->UpdateState();
        }
        for (size_t i = 0; i < init.global_action_.size(); i++)
        {
            init.global_action_[i]->Start(simulationTime_, deltaSimTime);
            init.global_action_[i]->UpdateState();
        }

        // Set initial values for speed and acceleration derivation
        for (size_t i = 0; i < entities_.object_.size(); i++)
        {
            Object* obj = entities_.object_[i];

            obj->state_old.pos_x  = obj->pos_.GetX();
            obj->state_old.pos_y  = obj->pos_.GetY();
            obj->state_old.pos_z  = obj->pos_.GetZ();
            obj->state_old.vel_x  = obj->pos_.GetVelX();
            obj->state_old.vel_y  = obj->pos_.GetVelY();
            obj->state_old.vel_z  = obj->pos_.GetVelZ();
            obj->state_old.h      = obj->pos_.GetH();
            obj->state_old.h_rate = obj->pos_.GetHRate();
            obj->reset_           = true;
        }
    }
    else
    {
        // reset update bits and indicators of applied control
        for (size_t i = 0; i < entities_.object_.size(); i++)
        {
            Object* obj = entities_.object_[i];

            obj->ClearDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
                                Object::DirtyBit::WHEEL_ROTATION);
            obj->reset_ = false;

            // Fetch dirty bits from gateway, indicating what has been reported externally and needs to be protected
            ObjectState* o = scenarioGateway.getObjectStatePtrById(obj->id_);
            if (o == nullptr)
            {
                LOG("Gateway did not provide state for external car %d", obj->id_);
            }
            else
            {
                if (o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL))
                {
                    obj->SetDirtyBits(o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL));
                }
                if (o->dirty_ & Object::DirtyBit::SPEED)
                {
                    obj->SetDirtyBits(Object::DirtyBit::SPEED);
                }
                if (o->dirty_ & Object::DirtyBit::WHEEL_ANGLE)
                {
                    obj->SetDirtyBits(Object::DirtyBit::WHEEL_ANGLE);
                }
                if (o->dirty_ & Object::DirtyBit::WHEEL_ROTATION)
                {
                    obj->SetDirtyBits(Object::DirtyBit::WHEEL_ROTATION);
                }
            }
        }
    }

    // First evaluate StoryBoard stopTrigger
    if (storyBoard.stop_trigger_ && storyBoard.stop_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
    {
        quit_flag = true;
        return 0;
    }

    // Step inital actions - might be extened in time (more than one step)
    for (size_t i = 0; i < init.private_action_.size(); i++)
    {
        Object* obj = init.private_action_[i]->object_;
        if (obj && init.private_action_[i]->IsActive())
        {
            // Add action to object initActions vector if it doesn't contain the action
            if (std::find(init.private_action_[i]->object_->initActions_.begin(),
                          init.private_action_[i]->object_->initActions_.end(),
                          init.private_action_[i]) == init.private_action_[i]->object_->initActions_.end())
            {
                init.private_action_[i]->object_->initActions_.push_back(init.private_action_[i]);
            }
            // LOG("Stepping action of type %d", init.private_action_[i]->action_[j]->type_)
            init.private_action_[i]->Step(getSimulationTime(), deltaSimTime);
        }
        init.private_action_[i]->UpdateState();
    }

    for (size_t i = 0; i < init.global_action_.size(); i++)
    {
        if (init.global_action_[i]->IsActive())
        {
            init.global_action_[i]->Step(getSimulationTime(), deltaSimTime);
        }
        init.global_action_[i]->UpdateState();
    }

    // Check for collisions/overlap after first initialization
    if (SE_Env::Inst().GetCollisionDetection() && frame_nr_ == 0)
    {
        DetectCollisions();
    }

    // Then evaluate all stories
    bool all_done = true;  // This flag will indicate whether all acts are done or not
    for (size_t i = 0; i < storyBoard.story_.size(); i++)
    {
        Story* story = storyBoard.story_[i];

        for (size_t j = 0; j < story->act_.size(); j++)
        {
            Act* act = story->act_[j];

            if (act->IsTriggable())
            {
                // Check start conditions
                if (!act->start_trigger_ ||  // Start act even if there's no trigger
                    act->start_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
                {
                    act->Start(simulationTime_, deltaSimTime);
                }
            }

            if (act->IsActive())
            {
                for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
                {
                    ManeuverGroup* mg = act->maneuverGroup_[k];
                    if (mg && mg->IsTriggable())
                    {
                        mg->Start(simulationTime_, deltaSimTime);
                    }
                }
                if (act->stop_trigger_)
                {
                    if (act->stop_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
                    {
                        act->End(simulationTime_);
                    }
                }
            }

            act->UpdateState();

            // Check whether this act is done - and update flag for all acts
            all_done = all_done && act->state_ == Act::State::COMPLETE;

            // Maneuvers
            if (act->IsActive())
            {
                for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
                {
                    ManeuverGroup* mg = act->maneuverGroup_[k];
                    if (mg->IsActive())
                    {
                        for (size_t l = 0; l < mg->maneuver_.size(); l++)
                        {
                            Maneuver* maneuver = mg->maneuver_[l];

                            for (size_t m = 0; m < maneuver->event_.size(); m++)
                            {
                                Event* event = maneuver->event_[m];

                                // add event to objectEvents vector
                                if (event->IsTriggable() || event->IsActive())
                                {
                                    for (size_t n = 0; n < event->action_.size(); n++)
                                    {
                                        OSCAction* action = event->action_[n];
                                        if (action->base_type_ == OSCAction::BaseType::PRIVATE)
                                        {
                                            OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);
                                            if (!pa->object_->containsEvent(event))
                                            {
                                                pa->object_->addEvent(event);
                                                break;
                                            }
                                        }
                                    }
                                }

                                // First evaluate which events are active
                                if (event->IsTriggable())
                                {
                                    // Check event conditions
                                    if (event->start_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
                                    {
                                        bool startEvent = false;

                                        // Check priority
                                        if (event->priority_ == Event::Priority::OVERWRITE)
                                        {
                                            // Activate trigged event
                                            if (event->IsActive())
                                            {
                                                LOG("Can't overwrite own running event (%s) - skip trig", event->name_.c_str());
                                            }
                                            else
                                            {
                                                // Deactivate any currently active event
                                                for (size_t n = 0; n < maneuver->event_.size(); n++)
                                                {
                                                    if (maneuver->event_[n]->IsActive())
                                                    {
                                                        // remove event from objectEvents vector
                                                        for (size_t o = 0; o < maneuver->event_[n]->action_.size(); o++)
                                                        {
                                                            OSCAction* action = maneuver->event_[n]->action_[o];
                                                            if (action->base_type_ == OSCAction::BaseType::PRIVATE)
                                                            {
                                                                OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);
                                                                pa->object_->removeEvent(event);

                                                                break;
                                                            }
                                                        }

                                                        maneuver->event_[n]->End(simulationTime_);
                                                        LOG("Event %s ended, overwritten by event %s",
                                                            maneuver->event_[n]->name_.c_str(),
                                                            event->name_.c_str());
                                                    }
                                                }

                                                startEvent = true;
                                            }
                                        }
                                        else if (event->priority_ == Event::Priority::SKIP)
                                        {
                                            if (maneuver->IsAnyEventActive())
                                            {
                                                LOG("Event is running, skipping trigged %s", event->name_.c_str());
                                            }
                                            else
                                            {
                                                startEvent = true;
                                            }
                                        }
                                        else if (event->priority_ == Event::Priority::PARALLEL)
                                        {
                                            // Don't care if any other action is ongoing, launch anyway
                                            if (event->IsActive())
                                            {
                                                LOG("Event %s already running, trigger ignored", event->name_.c_str());
                                            }
                                            else if (maneuver->IsAnyEventActive())
                                            {
                                                LOG("Event(s) ongoing, %s will run in parallel", event->name_.c_str());
                                            }

                                            startEvent = true;
                                        }
                                        else
                                        {
                                            LOG("Unknown event priority: %d", event->priority_);
                                        }

                                        if (startEvent)
                                        {
                                            event->Start(simulationTime_, deltaSimTime);
                                        }
                                    }
                                }
                            }
                        }
                        if (mg->AreAllManeuversComplete())
                        {
                            mg->End(simulationTime_);
                        }
                    }
                }
            }
        }

        for (size_t j = 0; j < story->act_.size(); j++)
        {
            // Then step events
            Act* act = story->act_[j];

            // Maneuvers
            if (act->IsActive())
            {
                for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
                {
                    for (size_t l = 0; l < act->maneuverGroup_[k]->maneuver_.size(); l++)
                    {
                        Maneuver* maneuver = act->maneuverGroup_[k]->maneuver_[l];

                        for (size_t m = 0; m < maneuver->event_.size(); m++)
                        {
                            Event* event = maneuver->event_[m];

                            // Update (step) all active actions, for all objects connected to the action
                            if (event->IsActive())
                            {
                                bool active = false;

                                for (size_t n = 0; n < event->action_.size(); n++)
                                {
                                    if (event->action_[n]->IsActive())
                                    {
                                        OSCAction* action           = event->action_[n];
                                        bool       is_private_ghost = [&]()
                                        {
                                            if (action->base_type_ == OSCAction::BaseType::PRIVATE)
                                            {
                                                return (static_cast<OSCPrivateAction*>(action)->object_->IsGhost());
                                            }

                                            return false;
                                        }();
                                        if (ghost_mode_ != GhostMode::RESTARTING || is_private_ghost)
                                        {
                                            if (ghost_mode_ == GhostMode::RESTART && is_private_ghost)
                                            {
                                                // The very step during which the ghost is restarting the
                                                // simulation time has not yet been adjusted (need to keep
                                                // same simulation time all actions throughout the step)
                                                // special case for the restarting ghost, which needs the adjusted time
                                                event->action_[n]->Step(simulationTime_ - headstart_time_, deltaSimTime);
                                            }
                                            else
                                            {
                                                event->action_[n]->Step(simulationTime_, deltaSimTime);
                                            }

                                            active = active || (event->action_[n]->IsActive());
                                        }
                                        else
                                        {
                                            active = true;
                                        }
                                    }
                                }
                                if (!active)
                                {
                                    // remove event from objectEvents vector
                                    for (size_t n = 0; n < event->action_.size(); n++)
                                    {
                                        OSCAction* action = event->action_[n];
                                        if (action->base_type_ == OSCAction::BaseType::PRIVATE)
                                        {
                                            OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);
                                            pa->object_->removeEvent(event);
                                            break;
                                        }
                                    }

                                    // Actions done -> Set event done
                                    event->End(simulationTime_);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // This timestep calculation is due to the Ghost vehicle
    // If both times are equal, it is a normal scenario, or no Ghost teleportation is ongoing -> Step as usual
    // Else if we can take a step, and still not reach the point of teleportation -> Step only simulationTime (That the Ghost runs on)
    // Else, the only thing left is that the next step will take us above the point of teleportation -> Step to that point instead and go on from
    // there

    simulationTime_ += deltaSimTime;
    if (simulationTime_ < 0.0 && simulationTime_ > -SMALL_NUMBER)
    {
        // Avoid -0.000
        simulationTime_ = 0.0;
    }

    if (simulationTime_ > trueTime_)
    {
        trueTime_ = simulationTime_;
    }

    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object* obj = entities_.object_[i];

        // Fetch states from gateway (if available), indicated by dirty bits
        ObjectState* o = scenarioGateway.getObjectStatePtrById(obj->id_);
        if (o != nullptr)
        {
            if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_H)
            {
                obj->pos_.SetAlignModeH(o->state_.pos.GetAlignModeH());
            }
            if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_P)
            {
                obj->pos_.SetAlignModeP(o->state_.pos.GetAlignModeP());
            }
            if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_R)
            {
                obj->pos_.SetAlignModeR(o->state_.pos.GetAlignModeR());
            }
            if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_Z)
            {
                obj->pos_.SetAlignModeZ(o->state_.pos.GetAlignModeZ());
            }
            if (o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL))
            {
                obj->pos_ = o->state_.pos;
            }
            if (o->dirty_ & Object::DirtyBit::SPEED)
            {
                obj->speed_ = o->state_.info.speed;
            }
            if (o->dirty_ & Object::DirtyBit::WHEEL_ANGLE)
            {
                obj->wheel_angle_ = o->state_.info.wheel_angle;
            }
            if (o->dirty_ & Object::DirtyBit::WHEEL_ROTATION)
            {
                obj->wheel_rot_ = o->state_.info.wheel_rot;
            }
            o->clearDirtyBits();
        }

        // Do not move objects when speed is zero,
        // and only ghosts allowed to execute during ghost (restart
        if (!(obj->IsControllerActiveOnDomains(ControlDomains::DOMAIN_BOTH) && obj->GetControllerMode() == Controller::Mode::MODE_OVERRIDE) &&
            fabs(obj->speed_) > SMALL_NUMBER &&
            // Skip update for non ghost objects during ghost restart
            !(!obj->IsGhost() && ghost_mode_ == GhostMode::RESTARTING) && !obj->TowVehicle())  // update trailers later
        {
            defaultController(obj, deltaSimTime);
        }

        if (!obj->pos_.GetRoute())
        {
            if (obj->GetJunctionSelectorStrategy() == roadmanager::Junction::JunctionStrategyType::RANDOM && obj->pos_.IsInJunction() &&
                obj->GetJunctionSelectorAngle() >= 0)
            {
                // Set junction selector angle as undefined during junction
                obj->SetJunctionSelectorAngle(std::nan(""));
            }
            else if (obj->GetJunctionSelectorStrategy() == roadmanager::Junction::JunctionStrategyType::RANDOM && !obj->pos_.IsInJunction() &&
                     std::isnan(obj->GetJunctionSelectorAngle()))
            {
                // Set new random junction selector after coming out of junction
                obj->SetJunctionSelectorAngleRandom();
            }
        }

        if (obj->pos_.GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROAD) ||
            obj->pos_.GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROUTE))
        {
            if (!obj->IsEndOfRoad())
            {
                obj->SetEndOfRoad(true, simulationTime_);
            }
        }
        else
        {
            obj->SetEndOfRoad(false);
        }

        // Report updated state to the gateway
        if (scenarioGateway.isObjectReported(obj->id_))
        {
            if (obj->CheckDirtyBits(Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL))
            {
                scenarioGateway.updateObjectPos(obj->id_, simulationTime_, &obj->pos_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::SPEED))
            {
                scenarioGateway.updateObjectSpeed(obj->id_, simulationTime_, obj->speed_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ANGLE))
            {
                scenarioGateway.updateObjectWheelAngle(obj->id_, simulationTime_, obj->wheel_angle_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ROTATION))
            {
                scenarioGateway.updateObjectWheelRotation(obj->id_, simulationTime_, obj->wheel_rot_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::VISIBILITY))
            {
                scenarioGateway.updateObjectVisibilityMask(obj->id_, obj->visibilityMask_);
            }
        }
        else
        {
            // Object not reported yet, do that
            scenarioGateway.reportObject(obj->id_,
                                         obj->name_,
                                         static_cast<int>(obj->type_),
                                         obj->category_,
                                         obj->role_,
                                         obj->model_id_,
                                         obj->model3d_,
                                         obj->GetActivatedControllerType(),
                                         obj->boundingbox_,
                                         static_cast<int>(obj->scaleMode_),
                                         obj->visibilityMask_,
                                         simulationTime_,
                                         obj->speed_,
                                         obj->wheel_angle_,
                                         obj->wheel_rot_,
                                         obj->rear_axle_.positionZ,
                                         obj->front_axle_.positionX,
                                         obj->front_axle_.positionZ,
                                         &obj->pos_,
                                         obj->vehicleLightActionStatusList);
        }
    }

    for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
    {
        if (scenarioReader->controller_[i]->Active())
        {
            if (ghost_mode_ != GhostMode::RESTARTING)
            {
                scenarioReader->controller_[i]->Step(deltaSimTime);
            }
        }
    }

    // Update any trailers now that tow vehicles have been updated by Default or custom controllers
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object*  obj     = entities_.object_[i];
        Vehicle* trailer = static_cast<Vehicle*>(obj->TrailerVehicle());

        if (!obj->TowVehicle() && obj->TrailerVehicle())
        {
            // Found a front tow vehicle, update trailers
            Vehicle* tow_vehicle = static_cast<Vehicle*>(obj);
            while (trailer)
            {
                // Calculate new trailer position and orientation
                ObjectState* o = scenarioGateway.getObjectStatePtrById(tow_vehicle->id_);
                SE_Vector    v0(tow_vehicle->trailer_hitch_->dx_, 0.0);

                // Fetch updated state of tow vehicle from gateway
                roadmanager::Position* tow_pos = &o->state_.pos;
                v0                             = v0.Rotate(tow_pos->GetH()) + SE_Vector(tow_pos->GetX(), tow_pos->GetY());
                SE_Vector v1                   = SE_Vector(trailer->pos_.GetX(), trailer->pos_.GetY()) - v0;
                v1.SetLength(trailer->trailer_coupler_->dx_);
                scenarioGateway.updateObjectWorldPosXYH(trailer->GetId(),
                                                        getSimulationTime(),
                                                        v0.x() + v1.x(),
                                                        v0.y() + v1.y(),
                                                        GetAngleInInterval2PI(atan2(v1.y(), v1.x()) + M_PI));
                trailer->SetSpeed(tow_vehicle->GetSpeed());

                tow_vehicle = trailer;
                trailer     = static_cast<Vehicle*>(trailer->TrailerVehicle());
            }
        }
    }

    // Check some states
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object* obj = entities_.object_[i];

        // Off road?
        if (obj->pos_.IsOffRoad())
        {
            if (!obj->IsOffRoad())
            {
                obj->SetOffRoad(true, simulationTime_);
            }
        }
        else
        {
            obj->SetOffRoad(false);
        }

        // Stand still?
        if (obj->GetSpeed() > -STAND_STILL_THRESHOLD && obj->GetSpeed() < STAND_STILL_THRESHOLD)
        {
            if (!obj->IsStandStill())
            {
                obj->SetStandStill(true, simulationTime_);
            }
        }
        else
        {
            obj->SetStandStill(false);
        }
    }

    // Check for collisions
    if (SE_Env::Inst().GetCollisionDetection() && frame_nr_ > 0)
    {
        DetectCollisions();
    }

    if (all_done)
    {
        LOG("All acts are done, quit now");
        quit_flag = true;
    }

    frame_nr_++;

    return 0;
}

void ScenarioEngine::printSimulationTime()
{
    LOG("simulationTime = %.2f", simulationTime_);
}

ScenarioGateway* ScenarioEngine::getScenarioGateway()
{
    return &scenarioGateway;
}

int ScenarioEngine::parseScenario()
{
    SetSimulationTime(0);
    SetTrueTime(0);

    scenarioReader->LoadControllers();

    scenarioReader->SetGateway(&scenarioGateway);

    scenarioReader->parseOSCHeader();
    if (scenarioReader->GetVersionMajor() < 1)
    {
        LOG_AND_QUIT("OpenSCENARIO v%d.%d not supported. Please migrate scenario to v1.0 or v1.1 and try again.",
                     scenarioReader->GetVersionMajor(),
                     scenarioReader->GetVersionMinor());
    }
    LOG("Loading %s (v%d.%d)", scenarioReader->getScenarioFilename().c_str(), scenarioReader->GetVersionMajor(), scenarioReader->GetVersionMinor());

    scenarioReader->parseGlobalParameterDeclarations();
    scenarioReader->variables.Print("variables");  // All variables parsed at this point (not the case with parameters)

    // Now that parameter declaration has been parsed, call any registered callbacks before applying the parameters
    if (paramDeclCallback.func != nullptr)
    {
        paramDeclCallback.func(paramDeclCallback.data);
    }

    // Init road manager
    scenarioReader->parseRoadNetwork(roadNetwork);

    if (getOdrFilename().empty())
    {
        LOG("No OpenDRIVE file specified, continue without");
    }
    else
    {
        std::vector<std::string> file_name_candidates;
        // absolute path or relative to current directory
        file_name_candidates.push_back(getOdrFilename());
        // relative path to scenario directory
        file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(scenarioReader->getScenarioFilename()), getOdrFilename()));
        // Remove all directories from path and look in current directory
        file_name_candidates.push_back(FileNameOf(getOdrFilename()));
        // Finally check registered paths
        for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
        {
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], getOdrFilename()));
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(getOdrFilename())));
        }
        size_t i;
        bool   located = false;
        for (i = 0; i < file_name_candidates.size(); i++)
        {
            if (FileExists(file_name_candidates[i].c_str()))
            {
                located = true;
                if (roadmanager::Position::LoadOpenDrive(file_name_candidates[i].c_str()) == true)
                {
                    LOG("Loaded OpenDRIVE: %s", file_name_candidates[i].c_str());
                    break;
                }
                else
                {
                    LOG("Failed to load OpenDRIVE file: %s", file_name_candidates[i].c_str());
                    if (i < file_name_candidates.size() - 1)
                    {
                        LOG("  -> trying: %s", file_name_candidates[i + 1].c_str());
                    }
                }
            }
        }

        if (i == file_name_candidates.size())
        {
            LOG((std::string("Failed to ") + (located ? "load" : "find") + " OpenDRIVE file " + getOdrFilename().c_str()).c_str());
            return -1;
        }
    }

    odrManager = roadmanager::Position::GetOpenDrive();

    scenarioReader->parseCatalogs();
    scenarioReader->parseEntities();

    scenarioReader->parseInit(init);
    scenarioReader->parseStoryBoard(storyBoard);
    storyBoard.entities_ = &entities_;

    // Now when all entities have been loaded, initialize the controllers
    if (!disable_controllers_)
    {
        for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
        {
            scenarioReader->controller_[i]->Init();
            scenarioReader->controller_[i]->SetScenarioEngine(this);
        }

        // find out maximum headstart time for ghosts
        for (size_t i = 0; i < entities_.object_.size(); i++)
        {
            Object* obj = entities_.object_[i];

            if (obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_FOLLOW_GHOST ||
                (obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_EXTERNAL &&
                 (static_cast<ControllerExternal*>((obj->controller_))->UseGhost())))
            {
                SetupGhost(obj);

                if (obj->ghost_)
                {
                    LOG_ONCE("NOTE: Ghost feature activated. Consider headstart time offset (-%.2f s) when reading log.",
                             obj->ghost_->GetHeadstartTime());

                    if (obj->ghost_->GetHeadstartTime() > GetHeadstartTime())
                    {
                        SetHeadstartTime(obj->ghost_->GetHeadstartTime());
                        SetGhostRestart();
                    }
                }
            }
        }
    }

    // Align trailers
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object* obj = entities_.object_[i];
        if (!obj->TowVehicle() && obj->TrailerVehicle())
        {
            // Found a front tow vehicle, update trailers
            (static_cast<Vehicle*>(obj)->AlignTrailers());
        }
    }

    return 0;
}

int ScenarioEngine::defaultController(Object* obj, double dt)
{
    int    retval  = 0;
    double steplen = obj->speed_ * dt;

    if (!obj->CheckDirtyBits(Object::DirtyBit::LONGITUDINAL))  // No action has updated longitudinal dimension
    {
        if (obj->GetControllerMode() == Controller::Mode::MODE_ADDITIVE || !obj->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LONG))
        {
            Vehicle* tow_vehicle = static_cast<Vehicle*>(obj->TowVehicle());
            if (tow_vehicle == nullptr)
            {
                retval = static_cast<int>(obj->MoveAlongS(steplen, true));
                if (retval == -1)
                {
                    // Something went wrong, couldn't move vehicle forward. Stop.
                    obj->SetSpeed(0.0);
                }
                obj->SetDirtyBits(Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::SPEED);
            }
        }
    }

    return retval == -1 ? -1 : 0;
}

void ScenarioEngine::prepareGroundTruth(double dt)
{
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        // Fetch external states from gateway
        Object*      obj = entities_.object_[i];
        ObjectState* o   = scenarioGateway.getObjectStatePtrById(obj->id_);

        if (o == nullptr)
        {
            LOG("Gateway did not provide state for external car %d", obj->id_);
        }
        else
        {
            if (o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL))
            {
                obj->pos_ = o->state_.pos;
                obj->SetDirtyBits(o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL));
            }
            if (o->dirty_ & Object::DirtyBit::SPEED)
            {
                obj->speed_ = o->state_.info.speed;
                obj->SetDirtyBits(Object::DirtyBit::SPEED);
            }
            if (o->dirty_ & Object::DirtyBit::WHEEL_ANGLE)
            {
                obj->wheel_angle_ = o->state_.info.wheel_angle;
                obj->SetDirtyBits(Object::DirtyBit::WHEEL_ANGLE);
            }
            if (o->dirty_ & Object::DirtyBit::WHEEL_ROTATION)
            {
                obj->wheel_rot_ = o->state_.info.wheel_rot;
                obj->SetDirtyBits(Object::DirtyBit::WHEEL_ROTATION);
            }
        }

        // Calculate resulting updated velocity, acceleration and heading rate (rad/s) NOTE: in global coordinate sys
        double dx = obj->pos_.GetX() - obj->state_old.pos_x;
        double dy = obj->pos_.GetY() - obj->state_old.pos_y;
        double dz = obj->pos_.GetZ() - obj->state_old.pos_z;

        if (frame_nr_ == 1 || (obj->IsGhost() && ghost_mode_ != GhostMode::RESTART) || (!obj->IsGhost() && ghost_mode_ != GhostMode::RESTARTING))
        {
            if (dt > SMALL_NUMBER)
            {
                // If velocity has not been reported, calculate it based on movement
                if (!obj->CheckDirtyBits(Object::DirtyBit::VELOCITY))
                {
                    // If not already reported, calculate linear velocity
                    obj->SetVel(dx / dt, dy / dt, dz / dt);
                }

                // If speed has not been reported or set by any controller, calculate it based on velocity
                if (!obj->CheckDirtyBits(Object::DirtyBit::SPEED))
                {
                    obj->SetSpeed(GetLengthOfVector2D(obj->pos_.GetVelX(), obj->pos_.GetVelY()));
                }

                if (!obj->CheckDirtyBits(Object::DirtyBit::ACCELERATION))
                {
                    // If not already reported, calculate linear acceleration
                    obj->SetAcc((obj->pos_.GetVelX() - obj->state_old.vel_x) / dt,
                                (obj->pos_.GetVelY() - obj->state_old.vel_y) / dt,
                                (obj->pos_.GetVelZ() - obj->state_old.vel_z) / dt);
                }

                double heading_rate_new = GetAngleDifference(obj->pos_.GetH(), obj->state_old.h) / dt;
                if (!obj->CheckDirtyBits(Object::DirtyBit::ANGULAR_RATE))
                {
                    // If not already reported, calculate angular velocity/rate
                    obj->SetAngularVel(heading_rate_new, 0.0, 0.0);
                }

                if (!obj->CheckDirtyBits(Object::DirtyBit::ANGULAR_ACC))
                {
                    // If not already reported, calculate angular acceleration
                    obj->SetAngularAcc(GetAngleDifference(heading_rate_new, obj->state_old.h_rate) / dt, 0.0, 0.0);
                }

                // Update wheel rotations of internal scenario objects
                if (!obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ANGLE))
                {
                    // An improvised calculation of a steering angle based on yaw rate and enitity speed
                    double steeringAngleTarget = SIGN(obj->GetSpeed()) * M_PI * heading_rate_new / MAX(fabs(obj->GetSpeed()), SMALL_NUMBER);
                    double steeringAngleDiff   = steeringAngleTarget - obj->wheel_angle_;

                    // Turn wheel gradually towards target
                    double steeringAngleStep = SIGN(steeringAngleDiff) * MIN(abs(steeringAngleDiff), 0.5 * dt);

                    obj->wheel_angle_ += steeringAngleStep;
                    obj->SetDirtyBits(Object::DirtyBit::WHEEL_ANGLE);
                }

                if (!obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ROTATION))
                {
                    obj->wheel_rot_ = fmod(obj->wheel_rot_ + obj->speed_ * dt / WHEEL_RADIUS, 2 * M_PI);
                    obj->SetDirtyBits(Object::DirtyBit::WHEEL_ROTATION);
                }
            }
            else
            {
                // calculate approximated velocity vector based on current heading
                if (!obj->CheckDirtyBits(Object::DirtyBit::VELOCITY))
                {
                    // If not already reported, calculate approximated velocity vector based on current heading
                    obj->SetVel(obj->speed_ * cos(obj->pos_.GetH()), obj->speed_ * sin(obj->pos_.GetH()), 0.0);
                }
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ANGLE))
            {
                scenarioGateway.updateObjectWheelAngle(obj->id_, simulationTime_, obj->wheel_angle_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ROTATION))
            {
                scenarioGateway.updateObjectWheelRotation(obj->id_, simulationTime_, obj->wheel_rot_);
            }

            // store current values for next loop
            obj->state_old.pos_x  = obj->pos_.GetX();
            obj->state_old.pos_y  = obj->pos_.GetY();
            obj->state_old.pos_z  = obj->pos_.GetZ();
            obj->state_old.vel_x  = obj->pos_.GetVelX();
            obj->state_old.vel_y  = obj->pos_.GetVelY();
            obj->state_old.vel_z  = obj->pos_.GetVelZ();
            obj->state_old.h      = obj->pos_.GetH();
            obj->state_old.h_rate = obj->pos_.GetHRate();

            if (!obj->reset_)
            {
                obj->odometer_ += abs(sqrt(dx * dx + dy * dy));  // odometer always measure all movements as positive, I guess...
            }

            if (!(obj->IsGhost() && GetGhostMode() == GhostMode::RESTART))  // skip ghost sample during restart
            {
                if (obj->trail_.GetNumberOfVertices() == 0 || simulationTime_ - obj->trail_.GetVertex(-1)->time > GHOST_TRAIL_SAMPLE_TIME)
                {
                    // Only add trail vertex when speed is not stable at 0
                    if (obj->trail_.GetNumberOfVertices() == 0 || fabs(obj->trail_.GetVertex(-1)->speed) > SMALL_NUMBER ||
                        fabs(obj->GetSpeed()) > SMALL_NUMBER)
                    {
                        obj->trail_.AddVertex({0.0,
                                               obj->pos_.GetX(),
                                               obj->pos_.GetY(),
                                               obj->pos_.GetZ(),
                                               obj->pos_.GetH(),
                                               simulationTime_,
                                               obj->GetSpeed(),
                                               0.0,
                                               false});
                    }
                }
            }
        }

        // Report updated pos values to the gateway
        scenarioGateway.updateObjectPos(obj->id_, simulationTime_, &obj->pos_);

        // Now that frame is complete, reset dirty bits to avoid circulation
        if (o)
            o->clearDirtyBits();

        // Clear dirty/update bits for any reported velocity and acceleration values
        obj->ClearDirtyBits(Object::DirtyBit::VELOCITY | Object::DirtyBit::ANGULAR_RATE | Object::DirtyBit::ACCELERATION |
                            Object::DirtyBit::ANGULAR_ACC);
    }
}

void ScenarioEngine::ReplaceObjectInTrigger(Trigger* trigger, Object* obj1, Object* obj2, double timeOffset, Event* event)
{
    if (trigger == 0)
    {
        return;
    }
    for (size_t i = 0; i < trigger->conditionGroup_.size(); i++)
    {
        for (size_t j = 0; j < trigger->conditionGroup_[i]->condition_.size(); j++)
        {
            OSCCondition* cond = trigger->conditionGroup_[i]->condition_[j];
            if (cond->base_type_ == OSCCondition::ConditionType::BY_ENTITY)
            {
                TrigByEntity* trig = static_cast<TrigByEntity*>(cond);

                if (trig->type_ == TrigByEntity::EntityConditionType::COLLISION || trig->type_ == TrigByEntity::EntityConditionType::REACH_POSITION ||
                    trig->type_ == TrigByEntity::EntityConditionType::TRAVELED_DISTANCE || trig->type_ == TrigByEntity::EntityConditionType::SPEED ||
                    trig->type_ == TrigByEntity::EntityConditionType::ACCELERATION || trig->type_ == TrigByEntity::EntityConditionType::END_OF_ROAD ||
                    trig->type_ == TrigByEntity::EntityConditionType::OFF_ROAD || trig->type_ == TrigByEntity::EntityConditionType::STAND_STILL)
                {
                    LOG("Handing over trigger %s to ghost", cond->name_.c_str());

                    for (size_t k = 0; k < trig->triggering_entities_.entity_.size(); k++)
                    {
                        if (trig->triggering_entities_.entity_[k].object_ == obj1)
                        {
                            trig->triggering_entities_.entity_[k].object_ = obj2;
                        }
                        else
                        {
                            CreateGhostTeleport(obj1, obj2, event);
                            LOG("Created new teleport action for ghost and %s trigger (entity %s)",
                                cond->name_.c_str(),
                                trig->triggering_entities_.entity_[k].object_->GetName().c_str());
                        }
                    }
                }
                else if (event != nullptr)
                {
                    CreateGhostTeleport(obj1, obj2, event);
                    LOG("Created new teleport action for ghost and %s trigger", cond->name_.c_str());
                }
            }
            else if (cond->base_type_ == OSCCondition::ConditionType::BY_VALUE)
            {
                TrigByValue* trig = static_cast<TrigByValue*>(cond);
                if (trig->type_ == TrigByValue::Type::SIMULATION_TIME)
                {
                    (static_cast<TrigBySimulationTime*>((trig)))->value_ += timeOffset;
                }
                else if (event != nullptr)
                {
                    CreateGhostTeleport(obj1, obj2, event);
                    LOG("Created new teleport action for ghost and %s trigger", cond->name_.c_str());
                }
            }
        }
    }
}

void ScenarioEngine::CreateGhostTeleport(Object* obj1, Object* obj2, Event* event)
{
    TeleportAction*        myNewAction = new TeleportAction;
    roadmanager::Position* pos         = new roadmanager::Position();
    pos->SetOrientationType(roadmanager::Position::OrientationType::ORIENTATION_RELATIVE);
    pos->SetInertiaPos(0, 0, 0);
    pos->SetRelativePosition(&obj1->pos_, roadmanager::Position::PositionType::RELATIVE_OBJECT);

    myNewAction->position_       = pos;
    myNewAction->type_           = OSCPrivateAction::ActionType::TELEPORT;
    myNewAction->object_         = obj2;
    myNewAction->scenarioEngine_ = this;
    myNewAction->name_           = "AddedGhostTeleport";
    myNewAction->SetGhostRestart(true);

    event->action_.insert(event->action_.begin(), myNewAction);
}

void ScenarioEngine::SetupGhost(Object* object)
{
    // FollowGhostController special treatment:
    // Create a new (ghost) vehicle and copy all actions from base object

    Vehicle* ghost = new Vehicle(*(static_cast<Vehicle*>(object)));
    object->SetGhost(ghost);
    ghost->name_ += "_ghost";
    ghost->ghost_      = 0;
    ghost->ghost_Ego_  = object;
    ghost->controller_ = 0;
    ghost->isGhost_    = true;
    ghost->SetHeadstartTime(object->headstart_time_);
    entities_.addObject(ghost, true);
    object->SetHeadstartTime(0);

    int numberOfInitActions = static_cast<int>(init.private_action_.size());
    for (int i = 0; i < numberOfInitActions; i++)
    {
        OSCPrivateAction* action = init.private_action_[static_cast<unsigned int>(i)];
        if (action->object_ == object)
        {
            // Copy all actions except ActivateController
            if (action->type_ != OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER)
            {
                OSCPrivateAction* newAction = action->Copy();
                action->name_ += "_ghost-copy";
                newAction->object_         = ghost;
                newAction->scenarioEngine_ = this;
                init.private_action_.push_back(newAction);
            }
        }
    }

    for (size_t i = 0; i < storyBoard.story_.size(); i++)
    {
        Story* story = storyBoard.story_[i];

        for (size_t j = 0; j < story->act_.size(); j++)
        {
            Act* act = story->act_[j];
            ReplaceObjectInTrigger(act->start_trigger_, object, ghost, -ghost->GetHeadstartTime());
            for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
            {
                ManeuverGroup* mg = act->maneuverGroup_[k];
                for (size_t l = 0; l < mg->actor_.size(); l++)
                {
                    if (mg->actor_[l]->object_ == object)
                    {
                        // Replace actor
                        mg->actor_[l]->object_ = ghost;
                    }
                }
                for (size_t l = 0; l < act->maneuverGroup_[k]->maneuver_.size(); l++)
                {
                    Maneuver* maneuver = act->maneuverGroup_[k]->maneuver_[l];
                    for (size_t m = 0; m < maneuver->event_.size(); m++)
                    {
                        Event* event        = maneuver->event_[m];
                        bool   ghostIsActor = false;
                        for (size_t n = 0; n < event->action_.size(); n++)
                        {
                            OSCAction* action = event->action_[n];
                            if (action->base_type_ == OSCAction::BaseType::PRIVATE)
                            {
                                OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);
                                pa->scenarioEngine_  = this;
                                if (pa->object_ == object)
                                {
                                    // If at least one of the event actions is of relevant subset of action types
                                    // then move the action to the ghost object instance, and also make needed
                                    // changes to the event trigger
                                    if (pa->type_ == OSCPrivateAction::ActionType::LONG_SPEED ||
                                        pa->type_ == OSCPrivateAction::ActionType::LONG_SPEED_PROFILE ||
                                        pa->type_ == OSCPrivateAction::ActionType::LAT_LANE_CHANGE ||
                                        pa->type_ == OSCPrivateAction::ActionType::LAT_LANE_OFFSET ||
                                        pa->type_ == OSCPrivateAction::ActionType::SYNCHRONIZE_ACTION ||
                                        pa->type_ == OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY ||
                                        pa->type_ == OSCPrivateAction::ActionType::ASSIGN_ROUTE ||
                                        pa->type_ == OSCPrivateAction::ActionType::TELEPORT)
                                    {
                                        // Replace object
                                        pa->ReplaceObjectRefs(object, ghost);
                                        ghostIsActor = true;
                                    }
                                }
                            }
                        }
                        if (ghostIsActor)
                        {
                            ReplaceObjectInTrigger(event->start_trigger_, object, ghost, -ghost->GetHeadstartTime(), event);
                        }
                    }
                }
            }
        }
    }
}
// Reset events ongoing or finished by ghost
void ScenarioEngine::ResetEvents()
{
    for (size_t i = 0; i < storyBoard.story_.size(); i++)
    {
        Story* story = storyBoard.story_[i];

        for (size_t j = 0; j < story->act_.size(); j++)
        {
            Act* act = story->act_[j];

            for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
            {
                for (size_t l = 0; l < act->maneuverGroup_[k]->maneuver_.size(); l++)
                {
                    Maneuver* maneuver = act->maneuverGroup_[k]->maneuver_[l];

                    for (size_t m = 0; m < maneuver->event_.size(); m++)
                    {
                        Event* event = maneuver->event_[m];

                        if (event->state_ == StoryBoardElement::State::COMPLETE || event->next_state_ == StoryBoardElement::State::COMPLETE)
                        {
                            bool NoTele = true;
                            for (size_t n = 0; n < event->action_.size(); n++)
                            {
                                OSCAction* action = event->action_[n];
                                if (action->base_type_ == OSCAction::BaseType::PRIVATE)
                                {
                                    OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);
                                    if (pa->type_ == OSCPrivateAction::ActionType::TELEPORT)
                                    {
                                        NoTele = false;
                                    }
                                }
                            }
                            for (size_t n = 0; n < event->action_.size(); n++)
                            {
                                OSCAction* action = event->action_[n];
                                if (action->base_type_ == OSCAction::BaseType::PRIVATE)
                                {
                                    OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);

                                    // If the event doesnt contain a teleport action, and the trigger is not triggable, we reser it, making it able to
                                    // tigger again
                                    if (NoTele && pa->object_->IsGhost() && event->start_trigger_->Evaluate(&storyBoard, simulationTime_) == false)
                                    {
                                        LOG("Reset event %s: ", event->name_.c_str());
                                        event->Reset();
                                    }
                                    // if (event->start_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
                                    // {
                                    // 	printf("End event %s: \n", event->name_.c_str());
                                    // 	event->End();
                                    // }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

int ScenarioEngine::DetectCollisions()
{
    collision_pair_.clear();
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object* obj0 = entities_.object_[i];
        for (size_t j = i + 1; j < entities_.object_.size(); j++)
        {
            Object* obj1 = entities_.object_[j];
            if (obj0->Collision(obj1))
            {
                collision_pair_.push_back({obj0, obj1});
                if (std::find(obj0->collisions_.begin(), obj0->collisions_.end(), obj1) == obj0->collisions_.end())
                {
                    // was not overlapping last timestep, but are now
                    LOG("Collision between %s and %s", obj0->GetName().c_str(), obj1->GetName().c_str());
                    obj0->collisions_.push_back(obj1);
                    obj1->collisions_.push_back(obj0);
                }
            }
            else
            {
                if (std::find(obj0->collisions_.begin(), obj0->collisions_.end(), obj1) != obj0->collisions_.end())
                {
                    // was overlapping last frame, but not anymore
                    LOG("Collision between %s and %s dissolved", obj0->GetName().c_str(), obj1->GetName().c_str());
                    obj0->collisions_.erase(std::remove(obj0->collisions_.begin(), obj0->collisions_.end(), obj1), obj0->collisions_.end());
                    obj1->collisions_.erase(std::remove(obj1->collisions_.begin(), obj1->collisions_.end(), obj0), obj1->collisions_.end());
                }
            }
        }
    }

    // Check for and clear any vanished objects from collision lists
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        for (size_t j = 0; j < entities_.object_[i]->collisions_.size(); j++)
        {
            Object* obj = entities_.object_[i];
            if (std::find(entities_.object_.begin(), entities_.object_.end(), obj->collisions_[j]) == entities_.object_.end())
            {
                // object previously collided with pivot object has vanished from the set of entities, remove it from collision list
                LOG("Unregister collision between %s and vanished entity", obj->GetName().c_str());
                obj->collisions_.erase(obj->collisions_.begin() + static_cast<int>(j));
                j--;
            }
        }
    }

    return 0;
}
