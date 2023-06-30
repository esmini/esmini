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

#include "OSCManeuver.hpp"

using namespace scenarioengine;

void Event::Start(double simTime, double dt)
{
    double adjustedTime = simTime;

    for (size_t i = 0; i < action_.size(); i++)
    {
        // Terminate any ongoing action on same object and domain
        if (action_[i]->base_type_ == OSCAction::BaseType::PRIVATE)
        {
            OSCPrivateAction* pa  = static_cast<OSCPrivateAction*>(action_[i]);
            Object*           obj = pa->object_;
            if (obj != nullptr)
            {
                // First check init actions
                for (size_t j = 0; j < obj->initActions_.size(); j++)
                {
                    if (obj->initActions_[j]->base_type_ == OSCAction::BaseType::PRIVATE &&
                        obj->initActions_[j]->state_ == StoryBoardElement::State::RUNNING)
                    {
                        if (static_cast<int>(obj->initActions_[j]->GetDomain()) & static_cast<int>(pa->GetDomain()))
                        {
                            // Domains overlap, at least one domain in common. Terminate old action.
                            LOG("Stopping %s on conflicting %s domain(s)",
                                obj->initActions_[j]->name_.c_str(),
                                ControlDomain2Str(obj->initActions_[j]->GetDomain()).c_str());
                            obj->initActions_[j]->End(simTime);
                        }
                    }
                }

                // Then check Storyboard event actions
                for (size_t j = 0; j < pa->object_->objectEvents_.size(); j++)
                {
                    for (size_t k = 0; k < obj->objectEvents_[j]->action_.size(); k++)
                    {
                        // Make sure the object's action is of private type
                        if (obj->objectEvents_[j]->action_[k]->base_type_ == OSCAction::BaseType::PRIVATE)
                        {
                            OSCPrivateAction* pa2 = static_cast<OSCPrivateAction*>(obj->objectEvents_[j]->action_[k]);
                            if (pa2 != pa && pa2->object_->GetId() == pa->object_->GetId() && pa2->IsActive() &&
                                pa2->base_type_ == OSCAction::BaseType::PRIVATE)
                            {
                                if (static_cast<int>(pa2->GetDomain()) & static_cast<int>(pa->GetDomain()))
                                {
                                    // Domains overlap, at least one domain in common. Terminate old action.
                                    LOG("Stopping object %s %s on conflicting %s domain(s)",
                                        obj->name_.c_str(),
                                        pa2->name_.c_str(),
                                        ControlDomain2Str(pa2->GetDomain()).c_str());
                                    pa2->End(simTime);
                                }
                                if (static_cast<int>(pa2->GetLightDomain()) & static_cast<int>(pa->GetLightDomain()))
                                {
                                    // light Domains overlap, at least one domain in common. Terminate old action.
                                    LOG("Stopping object %s %s on conflicting light domain(s)",
                                        obj->name_.c_str(),
                                        pa2->name_.c_str());
                                    pa2->End(simTime);
                                }
                            }
                        }
                    }
                }
            }
        }
        // Restart actions
        action_[i]->Reset();
        action_[i]->Start(adjustedTime, dt);

        if (action_[i]->base_type_ == OSCAction::BaseType::PRIVATE)
        {
            // When using a TeleportAction for the Ghost-vehicle, we need to set back the starting simTime for other Actions in the same Event.
            // This is an easy solution. A nicer one could be to access ScenarioEngines getSimulationTime() when calling action Start.
            OSCAction*        action = action_[i];
            OSCPrivateAction* pa     = static_cast<OSCPrivateAction*>(action);
            if (pa->object_->IsGhost() && pa->type_ == OSCPrivateAction::ActionType::TELEPORT)
            {
                adjustedTime = simTime - pa->object_->GetHeadstartTime();
            }
        }
    }

    StoryBoardElement::Start(adjustedTime, dt);
}

void Event::End(double simTime)
{
    for (size_t i = 0; i < action_.size(); i++)
    {
        if (action_[i]->IsActive())
        {
            action_[i]->End(simTime);
        }
    }

    StoryBoardElement::End(simTime);
}

void Event::Stop()
{
    for (size_t i = 0; i < action_.size(); i++)
    {
        action_[i]->Stop();
    }
    StoryBoardElement::Stop();
}

void Event::UpdateState()
{
    for (size_t n = 0; n < action_.size(); n++)
    {
        action_[n]->UpdateState();
    }
    StoryBoardElement::UpdateState();
}

bool Maneuver::IsAnyEventActive()
{
    for (size_t i = 0; i < event_.size(); i++)
    {
        if (event_[i]->IsActive())
        {
            return true;
        }
    }
    return false;
}

bool Maneuver::AreAllEventsComplete()
{
    for (size_t i = 0; i < event_.size(); i++)
    {
        if (event_[i]->state_ != StoryBoardElement::State::COMPLETE)
        {
            return false;
        }
    }
    return true;
}

void Maneuver::UpdateState()
{
    // Update state of sub elements - moving from transitions to stable states
    for (size_t k = 0; k < event_.size(); k++)
    {
        event_[k]->UpdateState();
    }

    StoryBoardElement::UpdateState();
}

void Maneuver::Reset()
{
    // Reset child events
    for (size_t k = 0; k < event_.size(); k++)
    {
        event_[k]->Reset();
    }

    StoryBoardElement::Reset();
}
