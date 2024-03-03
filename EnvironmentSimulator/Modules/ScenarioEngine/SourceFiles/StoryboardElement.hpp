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

#pragma once

#include "CommonMini.hpp"

#include <string>
#include <vector>

#ifdef _USE_OSI
class OSIReporter;  // Forward declaration
#endif              // _USE_OSI

namespace scenarioengine
{
    class TrigByState;  // Forward declaration
    class Trigger;      // Forward declaration

    class StoryBoardElement
    {
    public:
        static void (*stateChangeCallback)(const char* name, int type, int state, const char* full_path);

        /**
         * Take note, changing this enum will alter the public API in esminiLib.hpp
         */
        typedef enum
        {
            STORY_BOARD            = 1,
            STORY                  = 2,
            ACT                    = 3,
            MANEUVER_GROUP         = 4,
            MANEUVER               = 5,
            EVENT                  = 6,
            ACTION                 = 7,
            UNDEFINED_ELEMENT_TYPE = 0
        } ElementType;

        /**
         * Take note, changing this enum will alter the public API in esminiLib.hpp
         */
        typedef enum
        {
            INIT                    = 0,
            STANDBY                 = 1,
            RUNNING                 = 2,
            COMPLETE                = 3,
            UNDEFINED_ELEMENT_STATE = 4
        } State;

        typedef enum
        {
            START_TRANSITION,  // Transitions last for one step
            END_TRANSITION,
            STOP_TRANSITION,
            SKIP_TRANSITION,
            INIT_TO_STBY_TRANSITION,
            UNDEFINED_ELEMENT_TRANSITION
        } Transition;

        ElementType        element_type_;
        StoryBoardElement* parent_;
        int                num_executions_;
        int                max_num_executions_;

        std::vector<TrigByState*> trigger_ref_;

        Trigger* start_trigger_;
        Trigger* stop_trigger_;

#ifdef _USE_OSI
        static OSIReporter* osi_reporter_;
        static void         SetOSIReporter(OSIReporter* osi_reporter)
        {
            osi_reporter_ = osi_reporter;
        };

        OSIReporter* GetOSIReporter()
        {
            return osi_reporter_;
        };
#endif  // _USE_OSI

        State GetCurrentState()
        {
            return state_;
        }

        void ResetState(State state = State::INIT)
        {
            state_ = state;
        }

        void SetTransition(Transition transition)
        {
            transition_ = transition;
        }

        Transition GetCurrentTransition()
        {
            return transition_;
        }

        void ResetTransition()
        {
            transition_ = Transition::UNDEFINED_ELEMENT_TRANSITION;
        }

        void AddTriggerRef(TrigByState* trigger)
        {
            trigger_ref_.push_back(trigger);
        }

        StoryBoardElement(ElementType type, StoryBoardElement* parent, int max_num_executions = -1)
            : element_type_(type),
              parent_(parent),
              num_executions_(0),
              max_num_executions_(max_num_executions),
              start_trigger_(nullptr),
              stop_trigger_(nullptr)
        {
            ResetState();
            ResetTransition();
            // children_ = nullptr;
        }

        virtual ~StoryBoardElement();

        void        SetState(State state);  // perform state change via already set transition
        std::string state2str(State state);
        std::string transition2str(StoryBoardElement::Transition state);

        bool AllChildrenComplete();
        bool AnyChildRunning();

        void PropagateStateFromChildren();

        bool IsTriggable()
        {
            return GetCurrentState() == State::STANDBY;
        }

        virtual void Start(double simTime);

        virtual void Step(double simTime, double dt);

        virtual void EvalTriggers(double simTime);

        virtual void Stop();

        virtual void End();

        void Standby()
        {
            if (GetCurrentState() == State::STANDBY)
            {
                SetTransition(Transition::SKIP_TRANSITION);
                SetState(State::STANDBY);
            }
            else if (GetCurrentState() == State::RUNNING)
            {
                SetTransition(Transition::END_TRANSITION);
                SetState(State::STANDBY);
            }
            else
            {
                LOG("Invalid transition requested from %s to %s", state2str(GetCurrentState()).c_str(), state2str(State::STANDBY).c_str());
            }
        }

        virtual void Reset(State state = State::INIT);

        void SetName(std::string name);

        const std::string GetName() const
        {
            return name_;
        };

        const std::string GetFullPath() const
        {
            return full_path_;
        };

        StoryBoardElement*              FindChildByName(std::string name);
        std::vector<StoryBoardElement*> FindChildByTypeAndName(ElementType type, std::string name);

        virtual std::vector<StoryBoardElement*>* GetChildren() = 0;

    private:
        std::string name_;
        std::string full_path_;

        State      state_;
        Transition transition_;
    };

}  // namespace scenarioengine