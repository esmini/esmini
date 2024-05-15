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

#include "Storyboard.hpp"
#include "CommonMini.hpp"

using namespace scenarioengine;

StoryBoardElement* StoryBoardElement::FindChildByName(std::string name)
{
    for (auto child : *GetChildren())
    {
        if (child->GetName() == name)
        {
            return child;
        }
    }

    return nullptr;
}

std::vector<StoryBoardElement*> StoryBoardElement::FindChildByTypeAndName(ElementType type, std::string name)
{
    std::vector<StoryBoardElement*> elements;

    if (type == element_type_ &&                // types should match
        name.size() <= GetFullPath().size() &&  // size of given name path must not be longer than element full path
        name.size() >= GetName().size())        // size of given name path must not be shorter than element name
    {
        // compare rightmost part of element name with the given name to check
        if (GetFullPath().substr(GetFullPath().size() - name.size()) == name)
        {
            elements.push_back(this);
            return elements;
        }
    }
    else if (type > element_type_)
    {
        for (auto child : *GetChildren())
        {
            std::vector<StoryBoardElement*> elements_tmp = child->FindChildByTypeAndName(type, name);

            if (elements_tmp.size() > 0)
            {
                elements.insert(elements.end(), elements_tmp.begin(), elements_tmp.end());
            }
        }
    }

    return elements;
}

void Story::Print()
{
    LOG("Story: %s", GetName().c_str());
}

void StoryBoard::Print()
{
    LOG("Storyboard:");
    for (size_t i = 0; i < story_.size(); i++)
    {
        story_[i]->Print();
    }
}

void StoryBoard::Start(double simTime)
{
    // kick off init actions
    for (auto action : init_.private_action_)
    {
        action->Start(simTime);
    }
    for (auto action : init_.global_action_)
    {
        action->Start(simTime);
    }

    StoryBoardElement::Start(simTime);
}

void StoryBoard::Step(double simTime, double dt)
{
    EvalTriggers(simTime);

    for (auto action : init_.private_action_)
    {
        if (action->GetCurrentState() == StoryBoardElement::State::RUNNING &&
            // skip update for non ghost objects during ghost restart phases
            !(!action->object_->IsGhost() && SE_Env::Inst().GetGhostMode() == GhostMode::RESTARTING))
        {
            action->Step(simTime, dt);
        }
    }
    for (auto action : init_.global_action_)
    {
        // skip update for during ghost restart phases
        if (action->GetCurrentState() == StoryBoardElement::State::RUNNING && !(SE_Env::Inst().GetGhostMode() == GhostMode::RESTARTING))
        {
            action->Step(simTime, dt);
        }
    }

    StoryBoardElement::Step(simTime, dt);
}

void Event::Start(double simTime)
{
    double adjustedTime = simTime;

    // Check priority
    Maneuver* maneuver    = static_cast<Maneuver*>(parent_);
    bool      start_event = false;
    if (priority_ == Event::Priority::OVERWRITE)
    {
        // Activate trigged event
        if (GetCurrentState() == StoryBoardElement::State::RUNNING)
        {
            LOG("Event already running, can't overwrite itself (%s) - skip trig", GetName().c_str());
        }
        else
        {
            for (auto tmp_event : maneuver->event_)
            {
                if (tmp_event != this && tmp_event->GetCurrentState() == StoryBoardElement::State::RUNNING)
                {
                    tmp_event->End();
                    LOG("Event %s ended, overwritten by event %s", tmp_event->GetName().c_str(), GetName().c_str());
                }
            }
            start_event = true;
        }
    }
    else if (priority_ == Event::Priority::SKIP)
    {
        if (maneuver->AnyChildRunning())
        {
            LOG("Some event is already running within the maneuver %s, skipping trigged %s", maneuver->GetName().c_str(), GetName().c_str());
        }
        else
        {
            start_event = true;
        }
    }
    else if (priority_ == Event::Priority::PARALLEL)
    {
        start_event = true;

        // Don't care if any other action is ongoing, launch anyway
        if (GetCurrentState() == StoryBoardElement::State::RUNNING)
        {
            LOG("Event %s already running, trigger ignored", GetName().c_str());
            start_event = false;
        }
        else if (maneuver->AnyChildRunning())
        {
            LOG("Other events ongoing, %s will run in parallel", GetName().c_str());
        }
    }
    else
    {
        LOG("Unknown event priority: %d", priority_);
    }

    if (!start_event)
    {
        return;
    }

    StoryBoardElement::Start(adjustedTime);

    if (GetCurrentState() != State::RUNNING)
    {
        return;
    }

    if (action_.size() == 0)
    {
        StoryBoardElement::End();
        return;
    }

    for (size_t i = 0; i < action_.size(); i++)
    {
        // Terminate any ongoing action on same object and domain
        if (action_[i]->GetBaseType() == OSCAction::BaseType::PRIVATE)
        {
            OSCPrivateAction* pa  = static_cast<OSCPrivateAction*>(action_[i]);
            Object*           obj = pa->object_;
            if (obj != nullptr)
            {
                // First check init actions
                for (size_t j = 0; j < obj->initActions_.size(); j++)
                {
                    if (obj->initActions_[j]->GetBaseType() == OSCAction::BaseType::PRIVATE &&
                        obj->initActions_[j]->GetCurrentState() == StoryBoardElement::State::RUNNING)
                    {
                        if (static_cast<int>(obj->initActions_[j]->GetDomains()) & static_cast<int>(pa->GetDomains()))
                        {
                            // Domains overlap, at least one domain in common. Terminate old action.
                            LOG("Stopping %s on conflicting %s domain(s)",
                                obj->initActions_[j]->GetName().c_str(),
                                ControlDomain2Str(obj->initActions_[j]->GetDomains()).c_str());
                            obj->initActions_[j]->End();
                        }
                    }
                }

                // Then check Storyboard event actions
                for (size_t j = 0; j < pa->object_->objectEvents_.size(); j++)
                {
                    for (size_t k = 0; k < obj->objectEvents_[j]->action_.size(); k++)
                    {
                        // Make sure the object's action is of private type
                        if (obj->objectEvents_[j]->action_[k]->GetBaseType() == OSCAction::BaseType::PRIVATE)
                        {
                            OSCPrivateAction* pa2 = static_cast<OSCPrivateAction*>(obj->objectEvents_[j]->action_[k]);
                            if (pa2 != pa && pa2->object_->GetId() == pa->object_->GetId() &&
                                pa2->GetCurrentState() == StoryBoardElement::State::RUNNING && pa2->GetBaseType() == OSCAction::BaseType::PRIVATE)
                            {
                                if (static_cast<int>(pa2->GetDomains()) & static_cast<int>(pa->GetDomains()))
                                {
                                    // Domains overlap, at least one domain in common. Terminate old action.
                                    LOG("Stopping object %s %s on conflicting %s domain(s)",
                                        obj->GetName().c_str(),
                                        pa2->GetName().c_str(),
                                        ControlDomain2Str(pa2->GetDomains()).c_str());
                                    pa2->End();
                                }
                            }
                        }
                    }
                }
            }
        }
        // Restart actions
        action_[i]->Start(adjustedTime);

        if (action_[i]->GetBaseType() == OSCAction::BaseType::PRIVATE)
        {
            // When using a TeleportAction for the Ghost-vehicle, we need to set back the starting simTime for other Actions in the same Event.
            // This is an easy solution. A nicer one could be to access ScenarioEngines getSimulationTime() when calling action Start.
            OSCAction*        action = action_[i];
            OSCPrivateAction* pa     = static_cast<OSCPrivateAction*>(action);
            if (pa->object_->IsGhost() && pa->action_type_ == OSCPrivateAction::ActionType::TELEPORT)
            {
                adjustedTime = simTime - pa->object_->GetHeadstartTime();
            }
        }
    }
}

void Event::Step(double simTime, double dt)
{
    for (auto action : action_)
    {
        if (action->GetCurrentState() == StoryBoardElement::State::RUNNING)
        {
            bool is_private_ghost = [&]()
            {
                if (action->GetBaseType() == OSCAction::BaseType::PRIVATE)
                {
                    return (static_cast<OSCPrivateAction*>(action)->object_->IsGhost());
                }

                return false;
            }();
            if (SE_Env::Inst().GetGhostMode() != GhostMode::RESTARTING || is_private_ghost)
            {
                if (SE_Env::Inst().GetGhostMode() == GhostMode::RESTART && is_private_ghost)
                {
                    // The very step during which the ghost is restarting the
                    // simulation time has not yet been adjusted (need to keep
                    // same simulation time all actions throughout the step)
                    // special case for the restarting ghost, which needs the adjusted time
                    action->Step(simTime - SE_Env::Inst().GetGhostHeadstart(), dt);
                }
                else
                {
                    action->Step(simTime, dt);
                }
            }
        }
    }
}
