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

#include "Story.hpp"
#include "CommonMini.hpp"

using namespace scenarioengine;

Act* Story::FindActByName(std::string name)
{
    for (size_t i = 0; i < act_.size(); i++)
    {
        if (name == act_[i]->GetName())
        {
            return act_[i];
        }
    }

    return nullptr;
}

ManeuverGroup* Story::FindManeuverGroupByName(std::string name)
{
    for (size_t i = 0; i < act_.size(); i++)
    {
        for (size_t j = 0; j < act_[i]->maneuverGroup_.size(); j++)
        {
            ManeuverGroup* mg = act_[i]->maneuverGroup_[j];
            if (name == mg->GetName())
            {
                return mg;
            }
        }
    }

    return nullptr;
}

Maneuver* Story::FindManeuverByName(std::string name)
{
    for (size_t i = 0; i < act_.size(); i++)
    {
        for (size_t j = 0; j < act_[i]->maneuverGroup_.size(); j++)
        {
            for (size_t k = 0; k < act_[i]->maneuverGroup_[j]->maneuver_.size(); k++)
            {
                Maneuver* maneuver = act_[i]->maneuverGroup_[j]->maneuver_[k];
                if (maneuver->GetName() == name)
                {
                    return maneuver;
                }
            }
        }
    }

    return nullptr;
}

Event* Story::FindEventByName(std::string name)
{
    for (size_t i = 0; i < act_.size(); i++)
    {
        for (size_t j = 0; j < act_[i]->maneuverGroup_.size(); j++)
        {
            for (size_t k = 0; k < act_[i]->maneuverGroup_[j]->maneuver_.size(); k++)
            {
                for (size_t l = 0; l < act_[i]->maneuverGroup_[j]->maneuver_[k]->event_.size(); l++)
                {
                    Event* event = act_[i]->maneuverGroup_[j]->maneuver_[k]->event_[l];
                    if (name == event->GetName())
                    {
                        return event;
                    }
                }
            }
        }
    }

    return nullptr;
}

OSCAction* Story::FindActionByName(std::string name)
{
    for (size_t i = 0; i < act_.size(); i++)
    {
        for (size_t j = 0; j < act_[i]->maneuverGroup_.size(); j++)
        {
            for (size_t k = 0; k < act_[i]->maneuverGroup_[j]->maneuver_.size(); k++)
            {
                for (size_t l = 0; l < act_[i]->maneuverGroup_[j]->maneuver_[k]->event_.size(); l++)
                {
                    for (size_t m = 0; m < act_[i]->maneuverGroup_[j]->maneuver_[k]->event_[l]->action_.size(); m++)
                    {
                        OSCAction* action = act_[i]->maneuverGroup_[j]->maneuver_[k]->event_[l]->action_[m];
                        if (name == action->GetName())
                        {
                            return action;
                        }
                    }
                }
            }
        }
    }

    return nullptr;
}

void Story::Print()
{
    LOG("Story: %s", GetName().c_str());
}

bool Story::IsComplete()
{
    for (size_t i = 0; i < act_.size(); i++)
    {
        if (act_[i]->state_ != State::COMPLETE)
        {
            return false;
        }
    }
    return true;
}

Act* StoryBoard::FindActByName(std::string name)
{
    Act* act = 0;
    for (size_t i = 0; i < story_.size(); i++)
    {
        if ((act = story_[i]->FindActByName(name)) != 0)
        {
            return act;
        }
    }

    return 0;
}

ManeuverGroup* StoryBoard::FindManeuverGroupByName(std::string name)
{
    ManeuverGroup* mg = 0;
    for (size_t i = 0; i < story_.size(); i++)
    {
        if ((mg = story_[i]->FindManeuverGroupByName(name)) != 0)
        {
            return mg;
        }
    }

    return 0;
}

Maneuver* StoryBoard::FindManeuverByName(std::string name)
{
    Maneuver* m = 0;
    for (size_t i = 0; i < story_.size(); i++)
    {
        if ((m = story_[i]->FindManeuverByName(name)) != 0)
        {
            return m;
        }
    }

    return 0;
}

Event* StoryBoard::FindEventByName(std::string name)
{
    Event* event = 0;
    for (size_t i = 0; i < story_.size(); i++)
    {
        if ((event = story_[i]->FindEventByName(name)) != 0)
        {
            return event;
        }
    }

    return 0;
}

OSCAction* StoryBoard::FindActionByName(std::string name)
{
    OSCAction* action = 0;
    for (size_t i = 0; i < story_.size(); i++)
    {
        if ((action = story_[i]->FindActionByName(name)) != 0)
        {
            return action;
        }
    }

    return 0;
}

void StoryBoard::Print()
{
    LOG("Storyboard:");
    for (size_t i = 0; i < story_.size(); i++)
    {
        story_[i]->Print();
    }
}

bool StoryBoard::IsComplete()
{
    for (size_t i = 0; i < story_.size(); i++)
    {
        if (story_[i]->state_ != State::COMPLETE)
        {
            return false;
        }
    }

    return true;
}

void StoryBoard::UpdateState()
{
    // Update state of sub elements - moving from transitions to stable states
    for (size_t k = 0; k < story_.size(); k++)
    {
        story_[k]->UpdateState();
    }

    StoryBoardElement::UpdateState();
}

void Story::UpdateState()
{
    // Update state of sub elements - moving from transitions to stable states
    for (size_t k = 0; k < act_.size(); k++)
    {
        act_[k]->UpdateState();
    }

    StoryBoardElement::UpdateState();
}

void Act::UpdateState()
{
    // Update state of sub elements - moving from transitions to stable states
    for (size_t k = 0; k < maneuverGroup_.size(); k++)
    {
        maneuverGroup_[k]->UpdateState();
    }

    StoryBoardElement::UpdateState();
}

bool Act::IsComplete()
{
    for (size_t i = 0; i < maneuverGroup_.size(); i++)
    {
        if (maneuverGroup_[i]->state_ != State::COMPLETE)
        {
            return false;
        }
    }
    return true;
}

bool ManeuverGroup::IsComplete()
{
    for (size_t i = 0; i < maneuver_.size(); i++)
    {
        if (maneuver_[i]->state_ != State::COMPLETE)
        {
            return false;
        }
    }
    return true;
}

void ManeuverGroup::UpdateState()
{
    // Update state of sub elements - moving from transitions to stable states
    for (size_t k = 0; k < maneuver_.size(); k++)
    {
        maneuver_[k]->UpdateState();
    }

    StoryBoardElement::UpdateState();
}

void ManeuverGroup::Start(double simTime, double dt)
{
    // Reset all child manuevers, getting them ready for (re)start
    for (size_t k = 0; k < maneuver_.size(); k++)
    {
        maneuver_[k]->Reset();
        maneuver_[k]->Start(simTime, dt);
    }

    // Make sure to call base class Start method
    StoryBoardElement::Start(simTime, dt);
}
#if 0
void ManeuverGroup::End(double simTime)
{
    for (size_t k = 0; k < maneuver_.size(); k++)
    {
        if (maneuver_[k]->IsActive())
        {
            maneuver_[k]->End(simTime);
        }
    }

    StoryBoardElement::End(simTime);
}

void ManeuverGroup::Stop()
{
    for (size_t k = 0; k < maneuver_.size(); k++)
    {
        if (maneuver_[k]->IsActive())
        {
            maneuver_[k]->Stop();
        }
    }

    StoryBoardElement::Stop();
}
#endif
