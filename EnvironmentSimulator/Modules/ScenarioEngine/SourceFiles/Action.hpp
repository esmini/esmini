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

#include "StoryboardElement.hpp"

namespace scenarioengine
{
    class OSCAction : public StoryBoardElement
    {
    public:
        enum class ActionType
        {
            UNDEFINED = 0,

            // Private actions
            PRIVATE_ACTION_BASE_INDEX = 1,
            LONG_SPEED                = 2,
            LONG_SPEED_PROFILE        = 3,
            LONG_DISTANCE             = 4,
            LAT_LANE_CHANGE           = 5,
            LAT_LANE_OFFSET           = 6,
            LAT_DISTANCE              = 7,
            VISIBILITY                = 8,
            CONTROLLER                = 9,
            ASSIGN_CONTROLLER         = 10,
            ACTIVATE_CONTROLLER       = 11,
            OVERRIDE_CONTROLLER       = 12,
            TELEPORT                  = 13,
            ASSIGN_ROUTE              = 14,
            FOLLOW_TRAJECTORY         = 15,
            Acquire_POSITION          = 16,
            SYNCHRONIZE_ACTION        = 17,
            CONNECT_TRAILER_ACTION    = 18,
            DISCONNECT_TRAILER_ACTION = 19,

            // Global actions
            GLOBAL_ACTION_BASE_INDEX = 30,
            ENVIRONMENT              = 31,  // not supported yet
            ADD_ENTITY               = 32,
            DELETE_ENTITY            = 33,
            PARAMETER_SET            = 34,
            VARIABLE_SET             = 35,
            INFRASTRUCTURE           = 36,  // not supported yet
            SWARM_TRAFFIC            = 37,

            // User defined action
            USER_DEFINED_ACTION_BASE_INDEX = 50,
            USER_DEFINED                   = 51,
        };

        typedef enum
        {
            GLOBAL,
            USER_DEFINED,
            PRIVATE,
        } BaseType;

        OSCAction(ActionType action_type, StoryBoardElement* parent)
            : StoryBoardElement(StoryBoardElement::ElementType::ACTION, parent),
              action_type_(action_type)
        {
            id_ = OSCAction::CreateUniqeActionId();
        }

        virtual ~OSCAction()
        {
        }

        std::string         BaseType2Str();
        virtual std::string Type2Str()
        {
            return BaseType2Str();
        }

        BaseType GetBaseType()
        {
            if (action_type_ < ActionType::GLOBAL_ACTION_BASE_INDEX)
            {
                return BaseType::PRIVATE;
            }
            else if (action_type_ < ActionType::USER_DEFINED_ACTION_BASE_INDEX)
            {
                return BaseType::GLOBAL;
            }
            else
            {
                return BaseType::USER_DEFINED;
            }
        }

        unsigned int GetNumChildren()
        {
            return 0;
        }

        StoryBoardElement* GetChild(unsigned int index)
        {
            (void)index;
            return nullptr;
        }

        std::vector<StoryBoardElement*>* GetChildren() override
        {
            return &dummy_child_list_;
        }

        unsigned int GetId() const
        {
            return id_;
        }

        ActionType action_type_;

    private:
        // add dummy child list to avoid nullptr checks - don't add elments to this list
        std::vector<StoryBoardElement*> dummy_child_list_;
        unsigned int                    id_;  // unique ID for each action
        static unsigned int             n_actions_;
        static unsigned int             CreateUniqeActionId()
        {
            return n_actions_++;
        }
    };

    class OSCUserDefinedAction : public OSCAction
    {
    public:
        OSCUserDefinedAction(StoryBoardElement* parent) : OSCAction(OSCAction::ActionType::USER_DEFINED, parent)
        {
        }

        OSCUserDefinedAction(const OSCUserDefinedAction& action) : OSCAction(OSCAction::ActionType::USER_DEFINED, action.parent_)
        {
            SetName(action.GetName());
            user_action_type_ = action.user_action_type_;
            content_          = action.content_;
        }

        ~OSCUserDefinedAction()
        {
        }

        OSCUserDefinedAction* Copy()
        {
            OSCUserDefinedAction* new_action = new OSCUserDefinedAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "UserDefinedAction";
        };

        void Start(double simTime) override
        {
            LOG("Starting %s type: %s content: %s", Type2Str().c_str(), user_action_type_.c_str(), content_.c_str());
            OSCAction::Start(simTime);
        }

        void Step(double, double) override
        {
        }

        std::string user_action_type_;
        std::string content_;
    };

}  // namespace scenarioengine