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
#include "Action.hpp"
#include "OSCPrivateAction.hpp"
#include "OSCGlobalAction.hpp"
#include "OSCParameterDeclarations.hpp"
#include "CommonMini.hpp"
#include "OSCCondition.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <OSCPrivateAction.hpp>
#include <OSCGlobalAction.hpp>

class OSIReporter;  // Forward declaration

namespace scenarioengine
{
    class TrigByState;  // Forward declaration
    class Trigger;      // Forward declaration

    class Init
    {
    public:
        ~Init()
        {
            for (auto* entry : private_action_)
            {
                delete entry;
            }

            for (auto* entry : global_action_)
            {
                delete entry;
            }

            for (auto* entry : user_defined_action_)
            {
                delete entry;
            }
        }

        std::vector<OSCPrivateAction*>     private_action_;
        std::vector<OSCGlobalAction*>      global_action_;
        std::vector<OSCUserDefinedAction*> user_defined_action_;
    };

    class Event : public StoryBoardElement
    {
    public:
        typedef enum
        {
            OVERWRITE,
            SKIP,
            PARALLEL,
            UNDEFINED_PRIORITY
        } Priority;

        Priority priority_;

        std::vector<OSCAction*> action_;

        Event(StoryBoardElement* parent)
            : StoryBoardElement(StoryBoardElement::ElementType::EVENT, parent),
              priority_(Event::Priority::UNDEFINED_PRIORITY)
        {
        }

        ~Event()
        {
            for (auto* entry : action_)
            {
                delete entry;
            }
        }

        void Start(double simTime) override;

        void Step(double simTime, double dt) override;

        std::vector<StoryBoardElement*>* GetChildren() override
        {
            return reinterpret_cast<std::vector<StoryBoardElement*>*>(&action_);
        }
    };

    class Maneuver : public StoryBoardElement
    {
    public:
        OSCParameterDeclarations parameter_declarations_;
        std::vector<Event*>      event_;

        Maneuver(StoryBoardElement* parent) : StoryBoardElement(StoryBoardElement::ElementType::MANEUVER, parent)
        {
        }
        ~Maneuver()
        {
            for (auto* entry : event_)
            {
                delete entry;
            }
        }

        std::vector<StoryBoardElement*>* GetChildren() override
        {
            return reinterpret_cast<std::vector<StoryBoardElement*>*>(&event_);
        }

        void Print()
        {
            LOG_INFO("\tname = {}", GetName());
        };
    };

    class ManeuverGroup : public StoryBoardElement
    {
    public:
        typedef struct
        {
            Object* object_;
        } Actor;

        ManeuverGroup(StoryBoardElement* parent) : StoryBoardElement(StoryBoardElement::ElementType::MANEUVER_GROUP, parent)
        {
        }
        ~ManeuverGroup()
        {
            for (auto* entry : actor_)
            {
                delete entry;
            }
            for (auto* entry : maneuver_)
            {
                delete entry;
            }
        }

        Object* FindActorByName(std::string name)
        {
            for (size_t i = 0; i < actor_.size(); i++)
            {
                if (actor_[i]->object_->name_ == name)
                {
                    return actor_[i]->object_;
                }
            }
            return 0;
        }

        bool IsObjectActor(Object* object)
        {
            for (size_t i = 0; i < actor_.size(); i++)
            {
                if (actor_[i]->object_ == object)
                {
                    return true;
                }
            }
            return false;
        }

        std::vector<StoryBoardElement*>* GetChildren() override
        {
            return reinterpret_cast<std::vector<StoryBoardElement*>*>(&maneuver_);
        }

        std::vector<Actor*>    actor_;
        std::vector<Maneuver*> maneuver_;
    };

    class Act : public StoryBoardElement
    {
    public:
        std::vector<ManeuverGroup*> maneuverGroup_;

        Act(StoryBoardElement* parent) : StoryBoardElement(StoryBoardElement::ElementType::ACT, parent)
        {
        }
        ~Act()
        {
            for (auto* entry : maneuverGroup_)
            {
                delete entry;
            }
        }

        std::vector<StoryBoardElement*>* GetChildren() override
        {
            return reinterpret_cast<std::vector<StoryBoardElement*>*>(&maneuverGroup_);
        }
    };

    class Story : public StoryBoardElement
    {
    public:
        Story(StoryBoardElement* parent) : StoryBoardElement(StoryBoardElement::ElementType::STORY, parent)
        {
        }
        Story(std::string name, StoryBoardElement* parent) : StoryBoardElement(StoryBoardElement::ElementType::STORY, parent)
        {
            SetName(name);
        }
        ~Story()
        {
            for (auto* entry : act_)
            {
                delete entry;
            }
        }

        OSCParameterDeclarations parameter_declarations_;
        Act*                     FindActByName(std::string name);
        ManeuverGroup*           FindManeuverGroupByName(std::string name);
        Maneuver*                FindManeuverByName(std::string name);
        Event*                   FindEventByName(std::string name);
        OSCAction*               FindActionByName(std::string name);
        void                     Print();

        std::vector<StoryBoardElement*>* GetChildren() override
        {
            return reinterpret_cast<std::vector<StoryBoardElement*>*>(&act_);
        }

        std::vector<Act*> act_;
    };

    class StoryBoard : public StoryBoardElement
    {
    public:
        StoryBoard() : StoryBoardElement(StoryBoardElement::ElementType::STORY_BOARD, nullptr)
        {
            SetName("storyBoard");  // don't need name for root level
        }
        ~StoryBoard()
        {
            for (auto* entry : story_)
            {
                delete entry;
            }
        }
        Story*         FindStoryByName(std::string name);
        Act*           FindActByName(std::string name);
        ManeuverGroup* FindManeuverGroupByName(std::string name);
        Maneuver*      FindManeuverByName(std::string name);
        Event*         FindEventByName(std::string name);
        OSCAction*     FindActionByName(std::string name);
        Init           init_;
        Entities*      entities_;
        void           Print();
        void           Start(double simTime) override;
        void           Step(double simTime, double dt) override;

        std::vector<StoryBoardElement*>* GetChildren() override
        {
            return reinterpret_cast<std::vector<StoryBoardElement*>*>(&story_);
        }

        std::vector<Story*> story_;
    };
}  // namespace scenarioengine