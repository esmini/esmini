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

#include "CommonMini.hpp"
#include "StoryboardElement.hpp"
#include "OSCCondition.hpp"
#include "Action.hpp"

#ifdef _USE_OSI
#include "OSIReporter.hpp"
#include "OSITrafficCommand.hpp"
#endif  // _USE_OSI

using namespace scenarioengine;

void (*StoryBoardElement::stateChangeCallback)(const char* name, int type, int state, const char* full_path) = nullptr;

#ifdef _USE_OSI
OSIReporter* StoryBoardElement::osi_reporter_ = nullptr;
#endif  // _USE_OSI

std::string StoryBoardElement::state2str(StoryBoardElement::State state)
{
    if (state == StoryBoardElement::State::INIT)
    {
        return "initState";
    }
    else if (state == StoryBoardElement::State::STANDBY)
    {
        return "standbyState";
    }
    else if (state == StoryBoardElement::State::RUNNING)
    {
        return "runningState";
    }
    else if (state == StoryBoardElement::State::COMPLETE)
    {
        return "completeState";
    }
    else
    {
        LOG("Undefined element state: %d", state);
    }

    return "Undefined";
}

std::string StoryBoardElement::transition2str(StoryBoardElement::Transition transition)
{
    if (transition == StoryBoardElement::Transition::START_TRANSITION)
    {
        return "startTransition";
    }
    else if (transition == StoryBoardElement::Transition::END_TRANSITION)
    {
        return "endTransition";
    }
    else if (transition == StoryBoardElement::Transition::STOP_TRANSITION)
    {
        return "stopTransition";
    }
    else if (transition == StoryBoardElement::Transition::SKIP_TRANSITION)
    {
        return "skipTransition";
    }
    else if (transition == StoryBoardElement::Transition::INIT_TO_STBY_TRANSITION)
    {
        return "initToStandbyTransition";
    }
    else if (transition == StoryBoardElement::Transition::UNDEFINED_ELEMENT_TRANSITION)
    {
        return "undefinedTransition";
    }
    else
    {
        LOG("Unexpected transition: %d", transition);
    }

    return "Undefined";
}

void StoryBoardElement::PropagateStateFromChildren()
{
    if (element_type_ != StoryBoardElement::ElementType::STORY_BOARD && GetCurrentState() != State::COMPLETE)
    {
        if (AllChildrenComplete())
        {
            End();
        }
    }
}

bool StoryBoardElement::AllChildrenComplete()
{
    for (auto child : *GetChildren())
    {
        if (child->GetCurrentState() != State::COMPLETE)
        {
            return false;
        }
    }

    return true;
}

bool StoryBoardElement::AnyChildRunning()
{
    for (auto child : *GetChildren())
    {
        if (child->GetCurrentState() == State::RUNNING)
        {
            return true;
        }
    }

    return false;
}

void StoryBoardElement::Start(double simTime)
{
    if (GetCurrentState() == State::INIT && start_trigger_ != nullptr)
    {
        SetTransition(Transition::INIT_TO_STBY_TRANSITION);
        SetState(State::STANDBY);
    }
    else if ((GetCurrentState() == State::INIT && start_trigger_ == nullptr) || GetCurrentState() == State::STANDBY || num_executions_)
    {
        SetTransition(Transition::START_TRANSITION);
        SetState(State::RUNNING);
        num_executions_++;
    }
    else
    {
        LOG("%s Invalid Start transition request from %s to %s, %sstart_trigger",
            name_.c_str(),
            state2str(GetCurrentState()).c_str(),
            state2str(State::RUNNING).c_str(),
            start_trigger_ ? "" : "no ");
    }

    // Start children for all element types except events
    // which will handle execution of actions based on domain and priority
    if (element_type_ != ElementType::EVENT && GetCurrentState() == State::RUNNING)
    {
        // Start chilren

        for (auto child : *GetChildren())
        {
            child->Start(simTime);
        }
    }
}

void StoryBoardElement::Step(double simTime, double dt)
{
    if (state_ == State::RUNNING)
    {
        for (auto child : *GetChildren())
        {
            child->Step(simTime, dt);
        }
    }
}

void StoryBoardElement::EvalTriggers(double simTime)
{
    if (GetCurrentState() == State::RUNNING && stop_trigger_)
    {
        if (stop_trigger_->Evaluate(simTime))
        {
            Stop();

            if (element_type_ == ElementType::STORY_BOARD)
            {
                // states are not propagated from children to story board, stop explicitly
                StoryBoardElement::Stop();
            }
        }
    }

    if (GetCurrentState() == State::STANDBY)
    {
        if (!start_trigger_ || start_trigger_->Evaluate(simTime))
        {
            Start(simTime);
        }
    }

    if (GetCurrentState() == State::RUNNING)
    {
        for (auto child : *GetChildren())
        {
            child->EvalTriggers(simTime);
        }
    }
}

void StoryBoardElement::Stop()
{
    for (auto child : *GetChildren())
    {
        child->Stop();
    }

    if (GetCurrentState() == State::INIT)
    {
        return;  // not even standby yet
    }

    if (GetCurrentState() == State::COMPLETE)
    {
        return;  // already complete
    }

    if (GetCurrentState() == State::STANDBY || GetCurrentState() == State::RUNNING)
    {
        SetTransition(Transition::STOP_TRANSITION);
        SetState(State::COMPLETE);
    }
    else
    {
        LOG("%s Invalid Stop transition requested from %s to %s",
            name_.c_str(),
            state2str(GetCurrentState()).c_str(),
            state2str(State::COMPLETE).c_str());
    }

    if (parent_)
    {
        parent_->PropagateStateFromChildren();
    }
}

void StoryBoardElement::End()
{
    if (GetCurrentState() == State::COMPLETE)
    {
        return;  // already complete
    }

    for (auto child : *GetChildren())
    {
        if (child->GetCurrentState() == StoryBoardElement::State::RUNNING)
        {
            child->End();
        }
    }

    if (GetCurrentState() == State::COMPLETE)
    {
        // element was already ended by a child state propagated to parent
        return;
    }

    // Allow element to move directly from standby to complete -
    //   some actions are atomic, and don't need run time
    if (GetCurrentState() == State::RUNNING || GetCurrentState() == State::STANDBY)
    {
        if (element_type_ == ElementType::MANEUVER_GROUP || element_type_ == ElementType::EVENT)
        {
            if (max_num_executions_ != -1 && num_executions_ >= max_num_executions_)
            {
                LOG("%s complete after %d execution%s", name_.c_str(), num_executions_, num_executions_ > 1 ? "s" : "");
                SetTransition(Transition::END_TRANSITION);
                SetState(State::COMPLETE);
            }
            else
            {
                // reset any triggers for potential next execution
                if (start_trigger_ != nullptr)
                {
                    start_trigger_->Reset();
                }

                if (stop_trigger_ != nullptr)
                {
                    stop_trigger_->Reset();
                }

                LOG("%s completed run %d (of max %d)", name_.c_str(), num_executions_, max_num_executions_);
                SetTransition(Transition::END_TRANSITION);
                SetState(State::STANDBY);

                // reset children
                for (auto child : *GetChildren())
                {
                    child->Reset();
                }
            }
        }
        else
        {
            SetTransition(Transition::END_TRANSITION);
            SetState(State::COMPLETE);  // no number_of_execution attribute, just execute once
        }
    }
    else
    {
        LOG("%s Invalid End transition requested from %s to %s or %s",
            name_.c_str(),
            state2str(GetCurrentState()).c_str(),
            state2str(State::STANDBY).c_str(),
            state2str(State::COMPLETE).c_str());
    }

    if (parent_)
    {
        parent_->PropagateStateFromChildren();
    }
}

StoryBoardElement::~StoryBoardElement()
{
    if (start_trigger_)
    {
        delete start_trigger_;
    }

    if (stop_trigger_)
    {
        delete stop_trigger_;
    }
}

void StoryBoardElement::SetState(StoryBoardElement::State state)
{
    if (GetCurrentState() != state)
    {
        LOG("%s %s -> %s -> %s",
            name_.c_str(),
            state2str(GetCurrentState()).c_str(),
            transition2str(GetCurrentTransition()).c_str(),
            state2str(state).c_str());

        if (stateChangeCallback != nullptr)
        {
            stateChangeCallback(GetName().c_str(), static_cast<int>(element_type_), static_cast<int>(state), GetFullPath().c_str());
        }

        for (size_t i = 0; i < trigger_ref_.size(); i++)
        {
            // For all monitoring triggers, register this state change
            trigger_ref_[i]->RegisterStateChange(this, state, GetCurrentTransition());
        }

#ifdef _USE_OSI
        // register all events for private actions to OSI reporter
        if (osi_reporter_ != nullptr && this->element_type_ == StoryBoardElement::ElementType::ACTION &&
            (reinterpret_cast<OSCAction*>(this))->GetBaseType() == OSCAction::BaseType::PRIVATE)
        {
            osi_reporter_->RegisterTrafficCommandStateChange(reinterpret_cast<OSCPrivateAction*>(this), state, GetCurrentTransition());
        }
#endif

        state_ = state;
    }
}

void StoryBoardElement::Reset(State state)
{
    for (auto child : *GetChildren())
    {
        child->Reset(state);
    }

    ResetState(state);
    ResetTransition();
}

void StoryBoardElement::SetName(std::string name)
{
    name_ = name;
    if (element_type_ == STORY_BOARD)
    {
        full_path_ = "";
    }
    else if (parent_ == nullptr)
    {
        full_path_ = name;
    }
    else
    {
        if (parent_->element_type_ == STORY_BOARD)
        {
            full_path_ = name;  // skip storyboard level
        }
        else
        {
            full_path_ = parent_->GetFullPath() + "::" + name;
        }
    }
}
