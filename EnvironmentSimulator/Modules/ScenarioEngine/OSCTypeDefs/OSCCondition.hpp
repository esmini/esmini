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

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "OSCCommon.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "OSCPosition.hpp"
#include "Parameters.hpp"
#include "StoryboardElement.hpp"

namespace scenarioengine
{
    // Forward declaration
    class StoryBoard;
    class StoryBoardElement;

    class OSCCondition
    {
    public:
        static void (*conditionCallback)(const char* name, double timestamp);

        enum class ConditionState
        {
            IDLE,
            EVALUATED,
            TIMER,
            TRIGGERED
        };

        typedef enum
        {
            BY_ENTITY,
            BY_STATE,
            BY_VALUE
        } ConditionType;

        typedef enum
        {
            RISING,
            FALLING,
            RISING_OR_FALLING,
            NONE,
            UNDEFINED
        } ConditionEdge;

        ConditionType      base_type_;
        std::string        name_;
        double             delay_;
        bool               last_result_;  // result from last evaluation
        ConditionEdge      edge_;
        SE_SimulationTimer timer_;
        ConditionState     state_;

        OSCCondition(ConditionType base_type) : base_type_(base_type), last_result_(false), edge_(ConditionEdge::NONE), state_(ConditionState::IDLE)
        {
        }
        virtual ~OSCCondition() = default;

        bool         Evaluate(double sim_time);
        virtual bool CheckCondition(double sim_time) = 0;
        virtual void Log();
        bool         CheckEdge(bool new_value, bool old_value, OSCCondition::ConditionEdge edge);
        std::string  Edge2Str();
        virtual void Reset();
    };

    class ConditionGroup
    {
    public:
        std::vector<OSCCondition*> condition_;

        ~ConditionGroup()
        {
            for (auto* entry : condition_)
            {
                delete entry;
            }
        }

        bool Evaluate(double sim_time);
    };

    class Trigger
    {
    public:
        std::vector<ConditionGroup*> conditionGroup_;

        Trigger(bool defaultValue) : defaultValue_(defaultValue)
        {
        }
        virtual ~Trigger()
        {
            for (auto* entry : conditionGroup_)
            {
                delete entry;
            }
        }

        bool         Evaluate(double sim_time);
        virtual void Reset();

    private:
        bool defaultValue_;  // applied on empty conditions
    };

    class TrigByEntity : public OSCCondition
    {
    public:
        struct Entity
        {
            Object* object_;
        };

        typedef enum
        {
            ANY,
            ALL
        } TriggeringEntitiesRule;

        struct TriggeringEntities
        {
            std::vector<Entity>    entity_;
            TriggeringEntitiesRule rule_;
        };

        typedef enum
        {
            TIME_HEADWAY,
            DISTANCE,
            RELATIVE_DISTANCE,
            REACH_POSITION,
            TRAVELED_DISTANCE,
            END_OF_ROAD,
            TIME_TO_COLLISION,
            COLLISION,
            OFF_ROAD,
            ACCELERATION,
            STAND_STILL,
            SPEED,
            RELATIVE_SPEED,
            RELATIVE_CLEARANCE
        } EntityConditionType;

        TriggeringEntitiesRule triggering_entity_rule_;
        TriggeringEntities     triggering_entities_;
        EntityConditionType    type_;
        std::vector<Object*>   triggered_by_entities_;

        TrigByEntity(EntityConditionType type) : OSCCondition(OSCCondition::ConditionType::BY_ENTITY), type_(type)
        {
        }

        void print()
        {
        }
    };

    class TrigByTimeHeadway : public TrigByEntity
    {
    public:
        Object*                           object_;
        double                            value_;
        bool                              freespace_;
        roadmanager::CoordinateSystem     cs_;
        roadmanager::RelativeDistanceType relDistType_;
        Rule                              rule_;
        double                            hwt_;

        bool CheckCondition(double sim_time);
        TrigByTimeHeadway() : TrigByEntity(TrigByEntity::EntityConditionType::TIME_HEADWAY), hwt_(0)
        {
        }
        void Log();
    };

    class TrigByTimeToCollision : public TrigByEntity
    {
    public:
        Object*                           object_;
        std::unique_ptr<OSCPosition>      position_;
        double                            value_;
        bool                              freespace_;
        roadmanager::CoordinateSystem     cs_;
        roadmanager::RelativeDistanceType relDistType_;
        Rule                              rule_;
        double                            ttc_;

        bool CheckCondition(double sim_time);
        TrigByTimeToCollision() : TrigByEntity(TrigByEntity::EntityConditionType::TIME_TO_COLLISION), object_(0), ttc_(-1)
        {
        }
        void Log();
    };

    class TrigByReachPosition : public TrigByEntity
    {
    public:
        std::unique_ptr<OSCPosition> position_;
        double                       tolerance_;
        double                       dist_;
        double                       angularTolerance_;
        bool                         checkOrientation_;

        bool CheckCondition(double sim_time);
        TrigByReachPosition()
            : TrigByEntity(TrigByEntity::EntityConditionType::REACH_POSITION),
              tolerance_(1.0),
              dist_(0),
              angularTolerance_(0.05),
              checkOrientation_(false)
        {
        }
        void Log();
    };

    class TrigByDistance : public TrigByEntity
    {
    public:
        std::unique_ptr<OSCPosition>      position_;
        double                            value_;
        bool                              freespace_;
        roadmanager::CoordinateSystem     cs_;
        roadmanager::RelativeDistanceType relDistType_;
        Rule                              rule_;
        double                            dist_;

        bool CheckCondition(double sim_time);
        TrigByDistance()
            : TrigByEntity(TrigByEntity::EntityConditionType::DISTANCE),
              value_(0),
              freespace_(false),
              cs_(roadmanager::CoordinateSystem::CS_UNDEFINED),
              relDistType_(roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED),
              dist_(0)
        {
        }
        void Log();
    };

    class TrigByTraveledDistance : public TrigByEntity
    {
    public:
        double value_;
        double odom_;

        bool CheckCondition(double sim_time);
        TrigByTraveledDistance() : TrigByEntity(TrigByEntity::EntityConditionType::TRAVELED_DISTANCE), value_(0), odom_(0)
        {
        }
        void Log();
    };

    class TrigByRelativeDistance : public TrigByEntity
    {
    public:
        Object*                           object_;
        double                            value_;
        bool                              freespace_;
        roadmanager::CoordinateSystem     cs_;
        roadmanager::RelativeDistanceType relDistType_;
        Rule                              rule_;
        double                            rel_dist_;

        bool CheckCondition(double sim_time);
        TrigByRelativeDistance() : TrigByEntity(TrigByEntity::EntityConditionType::RELATIVE_DISTANCE), object_(0), value_(0.0), rel_dist_(0)
        {
        }
        void Log();
    };

    class TrigByCollision : public TrigByEntity
    {
    public:
        typedef enum
        {
            LONGITUDINAL,
            LATERAL,
            INTERIAL
        } RelativeDistanceType;

        Object*      object_;
        Object::Type type_;
        Rule         rule_;
        StoryBoard*  storyBoard_;
        typedef struct
        {
            Object* object0;
            Object* object1;
        } CollisionPair;
        std::vector<CollisionPair> collision_pair_;

        bool CheckCondition(double sim_time);
        TrigByCollision() : TrigByEntity(TrigByEntity::EntityConditionType::COLLISION), object_(0), type_(Object::Type::TYPE_NONE), storyBoard_(0)
        {
        }
        void Log();
    };

    class TrigByEndOfRoad : public TrigByEntity
    {
    public:
        Object* object_;
        double  duration_;
        Rule    rule_;
        double  current_duration_;

        bool CheckCondition(double sim_time);
        TrigByEndOfRoad() : TrigByEntity(TrigByEntity::EntityConditionType::END_OF_ROAD), current_duration_(0)
        {
        }
        void Log();

    private:
        double elapsed_time_;
    };

    class TrigByOffRoad : public TrigByEntity
    {
    public:
        Object* object_;
        double  duration_;
        Rule    rule_;
        double  current_duration_;

        bool CheckCondition(double sim_time);
        TrigByOffRoad() : TrigByEntity(TrigByEntity::EntityConditionType::OFF_ROAD), current_duration_(0)
        {
        }
        void Log();

    private:
        double elapsed_time_;
    };

    class TrigByAcceleration : public TrigByEntity
    {
    public:
        double    value_;
        Rule      rule_;
        Direction direction_;
        double    current_acceleration_;

        bool CheckCondition(double sim_time);
        TrigByAcceleration()
            : TrigByEntity(TrigByEntity::EntityConditionType::ACCELERATION),
              value_(0),
              direction_(Direction::UNDEFINED_DIRECTION),
              current_acceleration_(0)
        {
        }
        void Log();
    };

    class TrigBySpeed : public TrigByEntity
    {
    public:
        double    value_;
        Rule      rule_;
        Direction direction_;
        double    current_speed_;

        bool CheckCondition(double sim_time);
        TrigBySpeed()
            : TrigByEntity(TrigByEntity::EntityConditionType::SPEED),
              value_(0),
              direction_(Direction::UNDEFINED_DIRECTION),
              current_speed_(0)
        {
        }
        void Log();
    };

    class TrigByRelativeSpeed : public TrigByEntity
    {
    public:
        Object*   object_;
        double    value_;
        Rule      rule_;
        Direction direction_;
        double    current_rel_speed_;

        bool CheckCondition(double sim_time);
        TrigByRelativeSpeed()
            : TrigByEntity(TrigByEntity::EntityConditionType::RELATIVE_SPEED),
              value_(0),
              direction_(Direction::UNDEFINED_DIRECTION),
              current_rel_speed_(0)
        {
        }
        void Log();
    };

    class TrigByRelativeClearance : public TrigByEntity
    {
    public:
        std::vector<Object*> objects_;
        // Object* object_;
        double      distanceForward_;
        double      distanceBackward_;
        int         from_;
        int         to_;
        bool        freeSpace_;
        bool        oppositeLanes_;
        StoryBoard* storyBoard_;

        bool CheckCondition(double sim_time);
        TrigByRelativeClearance()
            : TrigByEntity(TrigByEntity::EntityConditionType::RELATIVE_CLEARANCE),
              distanceForward_(0),
              distanceBackward_(0),
              from_(-LARGE_NUMBER_INT),  // -inf
              to_(LARGE_NUMBER_INT),     // +inf
              freeSpace_(false),
              oppositeLanes_(false),
              storyBoard_(0)
        {
        }
        roadmanager::Position* pos_;
        void                   Log();
    };
    class TrigByStandStill : public TrigByEntity
    {
    public:
        Object* object_;
        double  duration_;
        Rule    rule_;
        double  current_duration_;

        bool CheckCondition(double sim_time);
        TrigByStandStill() : TrigByEntity(TrigByEntity::EntityConditionType::STAND_STILL), current_duration_(0)
        {
        }
        void Log();

    private:
        double elapsed_time_;
    };

    class TrigByState : public OSCCondition
    {
    public:
        typedef enum
        {
            STANDBY,
            RUNNING,
            COMPLETE,
            UNDEFINED_ELEMENT_STATE,
            START_TRANSITION,
            END_TRANSITION,
            STOP_TRANSITION,
            SKIP_TRANSITION,
            UNDEFINED_ELEMENT_TRANSITION,
            UNDEFINED
        } CondElementState;

        typedef struct
        {
            StoryBoardElement*            element;
            StoryBoardElement::State      state;
            StoryBoardElement::Transition transition;
        } StateChange;

        CondElementState         target_element_state_;
        StoryBoardElement*       element_;
        std::vector<StateChange> state_change_;
        StateChange              latest_state_change_;

        bool CheckCondition(double sim_time);
        TrigByState() : OSCCondition(BY_STATE), target_element_state_(CondElementState::UNDEFINED), element_(nullptr)
        {
            latest_state_change_.element    = nullptr;
            latest_state_change_.state      = StoryBoardElement::State::UNDEFINED_ELEMENT_STATE;
            latest_state_change_.transition = StoryBoardElement::Transition::UNDEFINED_ELEMENT_TRANSITION;
        }

        void        RegisterStateChange(StoryBoardElement* element, StoryBoardElement::State state, StoryBoardElement::Transition transition);
        bool        CheckState(StateChange state_change);
        std::string CondElementState2Str(CondElementState state);
        void        Log();
        void        Reset();
    };

    class TrigByValue : public OSCCondition
    {
    public:
        typedef enum
        {
            PARAMETER,
            VARIABLE,
            TIME_OF_DAY,
            SIMULATION_TIME,
            UNDEFINED
        } Type;

        Type type_;
        Rule rule_;

        TrigByValue(Type type) : OSCCondition(BY_VALUE), type_(type)
        {
        }
    };

    class TrigBySimulationTime : public TrigByValue
    {
    public:
        double value_;
        double sim_time_;

        bool CheckCondition(double sim_time);
        TrigBySimulationTime() : TrigByValue(TrigByValue::Type::SIMULATION_TIME), sim_time_(0)
        {
        }
        void Log();
    };

    class TrigByParameter : public TrigByValue
    {
    public:
        Object*     object_;
        std::string name_;
        std::string value_;
        Rule        rule_;
        Parameters* parameters_;
        std::string current_value_str_;

        bool CheckCondition(double sim_time);
        TrigByParameter() : TrigByValue(TrigByValue::Type::PARAMETER)
        {
        }
        void Log();
    };

    class TrigByVariable : public TrigByValue
    {
    public:
        Object*     object_;
        std::string name_;
        std::string value_;
        Rule        rule_;
        Parameters* variables_;
        std::string current_value_str_;

        bool CheckCondition(double sim_time);
        TrigByVariable() : TrigByValue(TrigByValue::Type::VARIABLE)
        {
        }
        void Log();
    };

}  // namespace scenarioengine
