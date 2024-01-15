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

#include "OSCManeuver.hpp"
#include "OSCCondition.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace scenarioengine
{

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

        bool IsComplete() override;

        void UpdateState();
        void Start(double simTime, double dt);
#if 0
        void End(double simTime);
        void Stop();
#endif
        std::vector<Actor*>    actor_;
        std::vector<Maneuver*> maneuver_;
    };

    class Act : public StoryBoardElement
    {
    public:
        std::vector<ManeuverGroup*> maneuverGroup_;
        Trigger*                    start_trigger_;
        Trigger*                    stop_trigger_;

        Act(StoryBoardElement* parent) : StoryBoardElement(StoryBoardElement::ElementType::ACT, parent), start_trigger_(0), stop_trigger_(0)
        {
        }
        ~Act()
        {
            for (auto* entry : maneuverGroup_)
            {
                delete entry;
            }
            delete start_trigger_;
            delete stop_trigger_;
        }

        bool IsComplete() override;

        void UpdateState();
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

        bool IsComplete() override;
        void UpdateState() override;

        std::vector<Act*> act_;
    };

    class StoryBoard : public StoryBoardElement
    {
    public:
        StoryBoard() : StoryBoardElement(StoryBoardElement::ElementType::STORY_BOARD, nullptr), stop_trigger_(0)
        {
            SetName("storyBoard");  // don't need name for root level
        }
        ~StoryBoard()
        {
            for (auto* entry : story_)
            {
                delete entry;
            }
            delete stop_trigger_;
        }
        Act*           FindActByName(std::string name);
        ManeuverGroup* FindManeuverGroupByName(std::string name);
        Maneuver*      FindManeuverByName(std::string name);
        Event*         FindEventByName(std::string name);
        OSCAction*     FindActionByName(std::string name);
        Entities*      entities_;
        void           Print();
        bool           IsComplete() override;
        void           UpdateState() override;

        std::vector<Story*> story_;
        Trigger*            stop_trigger_;
    };
}  // namespace scenarioengine