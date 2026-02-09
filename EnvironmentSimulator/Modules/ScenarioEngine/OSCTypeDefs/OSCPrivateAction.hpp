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

#include "Action.hpp"
#include "OSCPosition.hpp"
#include "Entities.hpp"
#include "CommonMini.hpp"
#include "Controller.hpp"
#include "logger.hpp"

#include <iostream>
#include <string>
#include <math.h>
#include <memory>

namespace scenarioengine
{

#define DISTANCE_TOLERANCE       (0.5)  // meter
#define SYNCH_DISTANCE_TOLERANCE (0.0)  // meter
#define IS_ZERO(x)               (x < SMALL_NUMBER && x > -SMALL_NUMBER)

    class ScenarioEngine;

    class DynamicConstraints
    {
    public:
        double max_acceleration_;       //  Maximum acceleration the distance controller is allowed to use for keeping the distance
        double max_acceleration_rate_;  // Maximum acceleration rate the distance controller is allowed to use for keeping the distance
        double max_deceleration_;       //  Maximum deceleration the distance controller is allowed to use for keeping the distance
        double max_deceleration_rate_;  // Maximum deceleration rate the distance controller is allowed to use for keeping the distance
        double max_speed_;              // Maximum speed the distance controller is allowed to use for keeping the distance

        DynamicConstraints()
            : max_acceleration_(LARGE_NUMBER),
              max_acceleration_rate_(LARGE_NUMBER),
              max_deceleration_(LARGE_NUMBER),
              max_deceleration_rate_(LARGE_NUMBER),
              max_speed_(LARGE_NUMBER)
        {
        }
    };

    class OSCPrivateAction : public OSCAction
    {
    public:
        enum class DynamicsDimension
        {
            RATE,
            TIME,
            DISTANCE,
            DIMENSION_UNDEFINED
        };

        enum class DynamicsShape
        {
            LINEAR,
            CUBIC,
            SINUSOIDAL,
            STEP,
            SHAPE_UNDEFINED
        };

        class TransitionDynamics
        {
        public:
            DynamicsShape     shape_;
            DynamicsDimension dimension_;

            TransitionDynamics()
                : shape_(DynamicsShape::STEP),
                  dimension_(DynamicsDimension::TIME),
                  start_val_(0),
                  target_val_(0),
                  param_target_val_(0),
                  scale_factor_(1.0),
                  param_val_(0),
                  rate_(0)
            {
            }
            void Reset();

            double Evaluate(DynamicsShape shape = DynamicsShape::SHAPE_UNDEFINED) const;  // 0 = start_value, 1 = end_value
            double EvaluatePrim();
            double EvaluateScaledPrim();
            double EvaluatePrimPeak();
            int    Step(double delta_param_val);
            double GetTargetParamValByPrimPeak(double prim_peak);
            double GetTargetParamValByPrimPrimPeak(double prim_prim_peak);

            double GetParamVal() const
            {
                return param_val_;
            }
            void   SetStartVal(double start_val);
            double GetStartVal() const
            {
                return start_val_;
            }
            void   SetTargetVal(double target_val);
            double GetTargetVal() const
            {
                return target_val_;
            }

            void   SetParamTargetVal(double target_value);
            double GetParamTargetVal() const
            {
                return param_target_val_;
            }
            void   SetMaxRate(double max_rate);
            void   SetRate(double rate);
            void   UpdateRate();
            double GetRate() const
            {
                return rate_;
            }
            double GetScaleFactor() const
            {
                return scale_factor_;
            }

        private:
            double start_val_;
            double target_val_;
            double param_target_val_;
            double scale_factor_;
            double param_val_;
            double rate_;
        };

        ActionType      type_;
        unsigned int    domains_;
        Object*         object_;
        ScenarioEngine* scenarioEngine_;

        OSCPrivateAction(OSCAction::ActionType action_type, StoryBoardElement* parent, unsigned int domains)
            : OSCAction(action_type, parent),
              domains_(domains),
              object_(0),
              scenarioEngine_(0)
        {
        }

        virtual ~OSCPrivateAction() = default;

        virtual void print()
        {
            LOG_WARN("Virtual, should be overridden");
        };

        virtual OSCPrivateAction* Copy()
        {
            LOG_WARN("Virtual, should be overridden");
            return 0;
        };

        virtual std::string Type2Str()
        {
            return "OSCPrivateAction base class";
        };

        unsigned int GetDomains() const
        {
            return domains_;
        }

        void SetScenarioEngine(ScenarioEngine* scenarioEngine)
        {
            scenarioEngine_ = scenarioEngine;
        }

        virtual void ReplaceObjectRefs(Object*, Object*){};

        const std::string DomainActivation2Str(ControlActivationMode mode) const
        {
            switch (mode)
            {
                case ControlActivationMode::UNDEFINED:
                    return "UNDEFINED";
                case ControlActivationMode::OFF:
                    return "OFF";
                case ControlActivationMode::ON:
                    return "ON";
            }
            return "UNKNOWN";
        }
    };

    class LongSpeedAction : public OSCPrivateAction
    {
    public:
        TransitionDynamics transition_;

        class Target
        {
        public:
            enum class TargetType
            {
                ABSOLUTE_SPEED,
                RELATIVE_SPEED
            };

            TargetType type_;
            double     value_;

            Target(TargetType type) : type_(type), value_(0)
            {
            }
            virtual ~Target()
            {
            }
            virtual double GetValue() = 0;
        };

        class TargetAbsolute : public Target
        {
        public:
            TargetAbsolute() : Target(TargetType::ABSOLUTE_SPEED)
            {
            }

            double GetValue()
            {
                return value_;
            }
        };

        class TargetRelative : public Target
        {
        public:
            typedef enum
            {
                DELTA,
                FACTOR
            } ValueType;

            Object*   object_;
            ValueType value_type_;
            bool      continuous_;

            TargetRelative() : Target(TargetType::RELATIVE_SPEED), object_(0), value_type_(ValueType::DELTA), continuous_(false)
            {
            }

            double GetValue();
            void   Reset();
        };

        std::shared_ptr<Target> target_;
        bool                    target_speed_reached_;

        LongSpeedAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED, parent, static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)),
              target_(0),
              target_speed_reached_(false)
        {
        }

        LongSpeedAction(const LongSpeedAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG))
        {
            SetName(action.GetName());
            target_               = action.target_;
            transition_           = action.transition_;
            target_speed_reached_ = action.target_speed_reached_;
        }

        OSCPrivateAction* Copy()
        {
            LongSpeedAction* new_action = new LongSpeedAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "SpeedAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void print()
        {
        }

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
    };

    class LongSpeedProfileAction : public OSCPrivateAction
    {
    public:
        class Entry
        {
        public:
            double time_;
            double speed_;
            Entry() : time_(-1.0), speed_(0.0)
            {
            }
            Entry(double time, double speed) : time_(time), speed_(speed)
            {
            }
        };

        class EntryVertex
        {
        public:
            double t_;
            double v_;
            double k_;
            double m_;
            void   SetK(double k)
            {
                k_ = k;
                m_ = v_ - k_ * t_;
            }
            EntryVertex(double t, double v) : t_(t), v_(v), k_(0.0), m_(0.0)
            {
            }
            EntryVertex(double t, double v, double k) : t_(t), v_(v), k_(k)
            {
                m_ = v_ - k_ * t_;
            }
        };

        typedef struct
        {
            double t;
            double v;
            double k;
            double j;
        } SpeedSegment;

        std::vector<Entry>        entry_;
        FollowingMode             following_mode_;
        DynamicConstraints        dynamics_;
        Object*                   entity_ref_;
        std::vector<SpeedSegment> segment_;
        int                       cur_index_;

        LongSpeedProfileAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED_PROFILE,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)),
              following_mode_(FollowingMode::POSITION),
              entity_ref_(nullptr),
              cur_index_(0),
              speed_(0.0),
              acc_(0.0)
        {
        }

        LongSpeedProfileAction(FollowingMode follow_mode, StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED_PROFILE,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)),
              following_mode_(follow_mode),
              entity_ref_(nullptr),
              cur_index_(0),
              speed_(0.0),
              acc_(0.0)
        {
        }

        LongSpeedProfileAction(const LongSpeedProfileAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED_PROFILE,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG))
        {
            SetName(action.GetName());
            following_mode_ = action.following_mode_;
            entry_          = action.entry_;
            entity_ref_     = action.entity_ref_;
            speed_          = action.speed_;
            acc_            = action.acc_;
            cur_index_      = action.cur_index_;
            segment_        = action.segment_;
        }

        OSCPrivateAction* Copy()
        {
            LongSpeedProfileAction* new_action = new LongSpeedProfileAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "SpeedProfileAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt = 0.0);

        void print()
        {
            LOG_INFO("");
        }

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

        void AddEntry(LongSpeedProfileAction::Entry entry)
        {
            entry_.push_back(entry);
        }
        double GetStartTime() const
        {
            return start_time_;
        }
        double GetElapsedTime() const
        {
            return elapsed_;
        }
        double GetSpeed() const
        {
            return speed_;
        }
        void CheckAcceleration(double acc);
        void CheckAccelerationRate(double acc_rate);
        void CheckSpeed(double speed);
        void AddSpeedSegment(double t, double v, double k, double j);

    private:
        double start_time_;
        double elapsed_;
        double speed_;
        double acc_;
        double init_acc_;  // initial acceleration at start of the action
    };

    class LongDistanceAction : public OSCPrivateAction
    {
    public:
        typedef enum
        {
            DISTANCE,
            TIME_GAP,
        } DistType;

        typedef enum
        {
            NONE,
            TRAILING,
            LEADING,
            ANY
        } DisplacementType;

        Object*                       target_object_;
        double                        distance_;
        DistType                      dist_type_;
        double                        freespace_;
        bool                          continuous_;
        DisplacementType              displacement_;
        DynamicConstraints            dynamics_;
        roadmanager::CoordinateSystem cs_;

        LongDistanceAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_DISTANCE, parent, static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)),
              target_object_(0),
              distance_(0),
              dist_type_(DistType::DISTANCE),
              freespace_(0),
              displacement_(DisplacementType::NONE),
              cs_(roadmanager::CoordinateSystem::CS_ENTITY),
              acceleration_(0)
        {
        }

        LongDistanceAction(const LongDistanceAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_DISTANCE,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG))
        {
            SetName(action.GetName());
            target_object_ = action.target_object_;
            dynamics_      = action.dynamics_;
            distance_      = action.distance_;
            dist_type_     = action.dist_type_;
            freespace_     = action.freespace_;
            acceleration_  = action.acceleration_;
            displacement_  = action.displacement_;
            cs_            = action.cs_;
            continuous_    = action.continuous_;
        }

        OSCPrivateAction* Copy()
        {
            LongDistanceAction* new_action = new LongDistanceAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "LongitudinalDistanceAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void print()
        {
        }

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

    private:
        double acceleration_;
    };

    class LatDistanceAction : public OSCPrivateAction
    {
    public:
        typedef enum
        {
            INIT,
            MOVE_RIGID,
            MOVE_DYNAMIC
        } MoveState;

        typedef enum
        {
            DISTANCE,
        } DistType;

        typedef enum
        {
            NONE,  // Needed?
            LEFT_TO_REFERENCED_ENTITY,
            RIGHT_TO_REFERENCED_ENTITY,
            ANY
        } DisplacementType;

        Object*                       target_object_;
        double                        distance_;
        DistType                      dist_type_;
        double                        freespace_;
        bool                          continuous_;
        DisplacementType              displacement_;
        DynamicConstraints            dynamics_;
        roadmanager::CoordinateSystem cs_;

        LatDistanceAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_DISTANCE, parent, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)),
              target_object_(0),
              distance_(0),
              dist_type_(DistType::DISTANCE),
              freespace_(0),
              displacement_(DisplacementType::NONE),
              cs_(roadmanager::CoordinateSystem::CS_ENTITY),
              lat_vel_(0.0),
              acceleration_(0.0),
              spring_(0.0, 0.0, 0.0),
              old_x_(0.0),
              old_y_(0.0),
              sign_(0.0)
        {
        }

        LatDistanceAction(const LatDistanceAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_DISTANCE, action.parent_, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT))
        {
            SetName(action.GetName());
            target_object_ = action.target_object_;
            dynamics_      = action.dynamics_;
            distance_      = action.distance_;
            dist_type_     = action.dist_type_;
            freespace_     = action.freespace_;
            continuous_    = action.continuous_;
            displacement_  = action.displacement_;
            cs_            = action.cs_;
            lat_vel_       = action.lat_vel_;
            acceleration_  = action.acceleration_;
            spring_        = action.spring_;
            old_x_         = action.old_x_;
            old_y_         = action.old_y_;
            sign_          = action.sign_;
        }

        OSCPrivateAction* Copy()
        {
            LatDistanceAction* new_action = new LatDistanceAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "LateralDistanceAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void print()
        {
        }

        void GetDistanceError(roadmanager::Position& pos1, roadmanager::Position& pos2, double& distance_error);
        void GetDesiredRoadPos(const double distance_error, roadmanager::Position& internal_pos);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

    private:
        MoveState    move_state_;
        double       lat_vel_;
        double       acceleration_;
        DampedSpring spring_;
        double       old_x_;
        double       old_y_;
        double       sign_;
    };

    class LatLaneChangeAction : public OSCPrivateAction
    {
    public:
        class Target
        {
        public:
            enum class Type
            {
                ABSOLUTE_LANE,
                RELATIVE_LANE
            };

            Type type_;
            int  value_;

            Target(Type type) : type_(type)
            {
            }

            virtual ~Target() = default;
        };

        class TargetAbsolute : public Target
        {
        public:
            TargetAbsolute() : Target(Target::Type::ABSOLUTE_LANE)
            {
            }

            TargetAbsolute(const TargetAbsolute& target) : Target(target.type_)
            {
                value_ = target.value_;
            }
        };

        class TargetRelative : public Target
        {
        public:
            Object* object_;

            TargetRelative() : Target(Target::Type::RELATIVE_LANE), object_(0)
            {
            }

            TargetRelative(const TargetRelative& target) : Target(target.type_)
            {
                value_  = target.value_;
                object_ = target.object_;
            }
        };

        Target*            target_;
        TransitionDynamics transition_;
        double             target_lane_offset_;

        LatLaneChangeAction(StoryBoardElement* parent, LatLaneChangeAction::DynamicsDimension timing_type = DynamicsDimension::TIME)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_CHANGE, parent, static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)),
              target_(0),
              target_lane_offset_(0.0),
              start_offset_(0.0),
              heading_agnostic_(0.0)
        {
            transition_.dimension_ = timing_type;
        }

        LatLaneChangeAction(const LatLaneChangeAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_CHANGE,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)),
              transition_(action.transition_),
              target_lane_offset_(action.target_lane_offset_),
              start_offset_(action.start_offset_),
              heading_agnostic_(action.heading_agnostic_)
        {
            if (action.target_ != nullptr)
            {
                if (action.target_->type_ == Target::Type::ABSOLUTE_LANE)
                {
                    target_ = new TargetAbsolute(*static_cast<TargetAbsolute*>(action.target_));
                }
                else if (action.target_->type_ == Target::Type::RELATIVE_LANE)
                {
                    target_ = new TargetRelative(*static_cast<TargetRelative*>(action.target_));
                }
            }
            SetName(action.GetName());
            internal_pos_ = action.internal_pos_;
        }

        OSCPrivateAction* Copy()
        {
            LatLaneChangeAction* new_action = new LatLaneChangeAction(*this);
            return new_action;
        }

        ~LatLaneChangeAction()
        {
            if (target_ != nullptr)
            {
                delete target_;
                target_ = nullptr;
            }
        }

        virtual std::string Type2Str()
        {
            return "LaneChangeAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

    private:
        double                start_offset_;
        roadmanager::Position internal_pos_;  // Internal position representation
        double                heading_agnostic_;
    };

    class LatLaneOffsetAction : public OSCPrivateAction
    {
    public:
        class Target
        {
        public:
            enum class Type
            {
                ABSOLUTE_OFFSET,
                RELATIVE_OFFSET
            };

            Type   type_;
            double value_;

            Target(Type type) : type_(type)
            {
            }
            virtual ~Target() = default;
        };

        class TargetAbsolute : public Target
        {
        public:
            TargetAbsolute() : Target(Target::Type::ABSOLUTE_OFFSET)
            {
            }
        };

        class TargetRelative : public Target
        {
        public:
            Object* object_;

            TargetRelative() : Target(Target::Type::RELATIVE_OFFSET)
            {
            }
        };

        std::shared_ptr<Target> target_;
        TransitionDynamics      transition_;
        double                  max_lateral_acc_;

        LatLaneOffsetAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_OFFSET, parent, static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT))
        {
            max_lateral_acc_ = 0;
            target_          = 0;
        }

        LatLaneOffsetAction(const LatLaneOffsetAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_OFFSET,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT))
        {
            SetName(action.GetName());
            target_          = action.target_;
            max_lateral_acc_ = action.max_lateral_acc_;
            transition_      = action.transition_;
        }

        OSCPrivateAction* Copy()
        {
            LatLaneOffsetAction* new_action = new LatLaneOffsetAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "LaneOffsetAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
    };

    class SynchronizeAction : public OSCPrivateAction
    {
    public:
        typedef enum
        {
            STEADY_STATE_NONE,
            STEADY_STATE_POS,
            STEADY_STATE_DIST,
            STEADY_STATE_TIME
        } SteadyStateType;

        typedef enum
        {
            MODE_NONE,
            MODE_LINEAR,
            MODE_NON_LINEAR,
            MODE_STOPPED,
            MODE_STOP_IMMEDIATELY,
            MODE_WAITING,
            MODE_STEADY_STATE
        } SynchMode;

        typedef enum
        {
            SUBMODE_NONE,
            SUBMODE_CONVEX,
            SUBMODE_CONCAVE
        } SynchSubmode;

        struct
        {
            SteadyStateType       type_;
            roadmanager::Position pos_;
            double                time_;
            double                dist_;
        } steadyState_;

        SynchMode    mode_;
        SynchSubmode submode_;

        roadmanager::Position                    target_position_master_;
        roadmanager::Position                    target_position_;
        Object*                                  master_object_;
        std::shared_ptr<LongSpeedAction::Target> final_speed_;
        double                                   tolerance_;
        double                                   tolerance_master_;

        // Store calculated distances to use for comparison
        double lastDist_;
        double lastMasterDist_;

        SynchronizeAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::SYNCHRONIZE_ACTION,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG))
        {
            master_object_     = 0;
            final_speed_       = 0;
            mode_              = SynchMode::MODE_NONE;
            submode_           = SynchSubmode::SUBMODE_NONE;
            lastDist_          = LARGE_NUMBER;
            lastMasterDist_    = LARGE_NUMBER;
            tolerance_         = SYNCH_DISTANCE_TOLERANCE;
            tolerance_master_  = SYNCH_DISTANCE_TOLERANCE;
            steadyState_.type_ = SteadyStateType::STEADY_STATE_NONE;
        }

        SynchronizeAction(const SynchronizeAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::SYNCHRONIZE_ACTION,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG))
        {
            SetName(action.GetName());
            master_object_          = action.master_object_;
            final_speed_            = action.final_speed_;
            target_position_master_ = action.target_position_master_;
            target_position_        = action.target_position_;
            mode_                   = action.mode_;
            submode_                = action.submode_;
            lastDist_               = LARGE_NUMBER;
            lastMasterDist_         = LARGE_NUMBER;
            tolerance_              = SYNCH_DISTANCE_TOLERANCE;
            tolerance_master_       = SYNCH_DISTANCE_TOLERANCE;
            if (steadyState_.type_ == SteadyStateType::STEADY_STATE_DIST)
            {
                steadyState_.dist_ = action.steadyState_.dist_;
            }
            else if (steadyState_.type_ == SteadyStateType::STEADY_STATE_TIME)
            {
                steadyState_.dist_ = action.steadyState_.time_;
            }
            else if (steadyState_.type_ == SteadyStateType::STEADY_STATE_POS)
            {
                steadyState_.pos_ = action.steadyState_.pos_;
            }
            steadyState_.type_ = action.steadyState_.type_;
        }

        ~SynchronizeAction();

        OSCPrivateAction* Copy()
        {
            SynchronizeAction* new_action = new SynchronizeAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "SynchronizeAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime);

        const char* Mode2Str(SynchMode mode) const;

    private:
        double      CalcSpeedForLinearProfile(double v_final, double time, double dist);
        void        PrintStatus(const char* custom_msg);
        void        SetMode(SynchMode mode, std::string msg = "");
        void        SetSubMode(SynchSubmode submode, std::string msg = "");
        const char* SubMode2Str(SynchSubmode submode) const;

        void ReplaceObjectRefs(Object* obj1, Object* obj2)
        {
            if (object_ == obj1)
            {
                object_ = obj2;
            }

            if (master_object_ == obj1)
            {
                master_object_ = obj2;
            }
        }
    };

    class TeleportAction : public OSCPrivateAction
    {
    public:
        roadmanager::Position position_;

        TeleportAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::TELEPORT,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT_AND_LONG)),
              ghost_restart_(false)
        {
        }

        TeleportAction(const TeleportAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::TELEPORT,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT_AND_LONG))
        {
            SetName(action.GetName());
            position_      = action.position_;
            ghost_restart_ = action.ghost_restart_;
        }

        ~TeleportAction()
        {
            if (position_.route_ != nullptr)
            {
                delete position_.route_;
                position_.route_ = nullptr;
            }
        }

        OSCPrivateAction* Copy()
        {
            TeleportAction* new_action = new TeleportAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "TeleportAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
        void SetGhostRestart(bool value)
        {
            ghost_restart_ = value;
        }

    private:
        bool ghost_restart_;

        bool IsGhostRestart() const
        {
            return ghost_restart_;
        }
    };

    class ConnectTrailerAction : public OSCPrivateAction
    {
    public:
        ConnectTrailerAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::CONNECT_TRAILER_ACTION,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE))
        {
        }

        ConnectTrailerAction(const ConnectTrailerAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::CONNECT_TRAILER_ACTION,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE))
        {
            SetName(action.GetName());
            trailer_object_ = action.trailer_object_;
        }

        OSCPrivateAction* Copy()
        {
            ConnectTrailerAction* new_action = new ConnectTrailerAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "ConnectTrailerAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

        Object* trailer_object_ = nullptr;
    };

    class DisconnectTrailerAction : public OSCPrivateAction
    {
    public:
        DisconnectTrailerAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::DISCONNECT_TRAILER_ACTION,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE))
        {
        }

        DisconnectTrailerAction(const DisconnectTrailerAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::DISCONNECT_TRAILER_ACTION,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE))
        {
            SetName(action.GetName());
        }

        OSCPrivateAction* Copy()
        {
            DisconnectTrailerAction* new_action = new DisconnectTrailerAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "ConnectTrailerAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
    };

    class AssignRouteAction : public OSCPrivateAction
    {
    public:
        roadmanager::Route* route_;

        ~AssignRouteAction();

        AssignRouteAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_ROUTE, parent, static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE)),
              route_(0)
        {
        }

        AssignRouteAction(const AssignRouteAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_ROUTE,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE)),
              route_(0)
        {
            SetName(action.GetName());
            if (action.route_ != nullptr)
            {
                route_ = new roadmanager::Route;
                route_->CopyFrom(*action.route_);
            }
            else
            {
                route_ = nullptr;
            }
        }

        OSCPrivateAction* Copy()
        {
            AssignRouteAction* new_action = new AssignRouteAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "AssignRouteAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
    };

    class FollowTrajectoryAction : public OSCPrivateAction
    {
    public:
        enum class TimingDomain
        {
            NONE,
            TIMING_RELATIVE,
            TIMING_ABSOLUTE
        };

        roadmanager::RMTrajectory* traj_;
        TimingDomain               timing_domain_;
        FollowingMode              following_mode_;
        double                     timing_scale_;
        double                     timing_offset_;
        double                     time_;
        double                     initialDistanceOffset_;
        int                        initialHeadingSign_;
        int                        movingDirection_;
        bool                       explicit_h_active_;
        bool                       ignore_heading_for_motion_;

        FollowTrajectoryAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT_AND_LONG)),
              traj_(0),
              timing_domain_(TimingDomain::NONE),
              following_mode_(FollowingMode::FOLLOW),
              timing_scale_(1),
              timing_offset_(0),
              time_(0),
              initialDistanceOffset_(0),
              initialHeadingSign_(1),
              movingDirection_(1),
              explicit_h_active_(false),
              ignore_heading_for_motion_(false)
        {
        }

        FollowTrajectoryAction(const FollowTrajectoryAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT_AND_LONG))
        {
            SetName(action.GetName());
            traj_                      = action.traj_->Copy();
            timing_domain_             = action.timing_domain_;
            timing_scale_              = action.timing_scale_;
            timing_offset_             = action.timing_offset_;
            initialDistanceOffset_     = action.timing_offset_;
            following_mode_            = action.following_mode_;
            time_                      = 0;
            initialHeadingSign_        = action.initialHeadingSign_;
            movingDirection_           = action.movingDirection_;
            explicit_h_active_         = action.explicit_h_active_;
            ignore_heading_for_motion_ = action.ignore_heading_for_motion_;
        }

        ~FollowTrajectoryAction();
        void SetIgnoreHeadingForMotion(bool value)
        {
            ignore_heading_for_motion_ = value;
        }

        OSCPrivateAction* Copy()
        {
            FollowTrajectoryAction* new_action = new FollowTrajectoryAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "FollowTrajectoryAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime);
        void End();

        void Move(double simTime, double dt);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
    };

    class AcquirePositionAction : public OSCPrivateAction
    {
    public:
        roadmanager::Position target_position_;
        roadmanager::Route*   route_;

        ~AcquirePositionAction();

        AcquirePositionAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ACQUIRE_POSITION,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)),
              route_(0)
        {
        }

        AcquirePositionAction(const AcquirePositionAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ACQUIRE_POSITION,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)),
              route_(0)
        {
            SetName(action.GetName());
            target_position_ = action.target_position_;
            route_           = action.route_;
        }

        OSCPrivateAction* Copy()
        {
            AcquirePositionAction* new_action = new AcquirePositionAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "AcquirePositionAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
    };

    class AssignControllerAction : public OSCPrivateAction
    {
    public:
        Controller*           controller_;
        ControlActivationMode lat_activation_mode_   = ControlActivationMode::UNDEFINED;
        ControlActivationMode long_activation_mode_  = ControlActivationMode::UNDEFINED;
        ControlActivationMode light_activation_mode_ = ControlActivationMode::UNDEFINED;
        ControlActivationMode anim_activation_mode_  = ControlActivationMode::UNDEFINED;

        AssignControllerAction(Controller*           controller,
                               ControlActivationMode lat_activation_mode,
                               ControlActivationMode long_activation_mode,
                               ControlActivationMode light_activation_mode,
                               ControlActivationMode anim_activation_mode,
                               StoryBoardElement*    parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_CONTROLLER,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE)),
              controller_(controller),
              lat_activation_mode_(lat_activation_mode),
              long_activation_mode_(long_activation_mode),
              light_activation_mode_(light_activation_mode),
              anim_activation_mode_(anim_activation_mode)
        {
        }

        AssignControllerAction(const AssignControllerAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_CONTROLLER,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE))
        {
            SetName(action.GetName());
            controller_            = action.controller_;
            lat_activation_mode_   = action.lat_activation_mode_;
            long_activation_mode_  = action.long_activation_mode_;
            light_activation_mode_ = action.light_activation_mode_;
            anim_activation_mode_  = action.anim_activation_mode_;
        };

        OSCPrivateAction* Copy()
        {
            AssignControllerAction* new_action = new AssignControllerAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "AssignControllerAction";
        };

        void Step(double, double)
        {
        }

        void Start(double simTime);
    };

    class ActivateControllerAction : public OSCPrivateAction
    {
    public:
        std::string           ctrl_name_;
        Controller*           controller_;
        ControlActivationMode activation_mode_[static_cast<unsigned int>(ControlDomains::COUNT)] = {ControlActivationMode::OFF,
                                                                                                    ControlActivationMode::OFF,
                                                                                                    ControlActivationMode::OFF,
                                                                                                    ControlActivationMode::OFF};

        /**
        Constructor with domain specification
        @param domainMask bitmask according to Controller::Domain type
        */
        ActivateControllerAction(std::string           ctrl_name,
                                 ControlActivationMode lat_activation_mode,
                                 ControlActivationMode long_activation_mode,
                                 ControlActivationMode light_activation_mode,
                                 ControlActivationMode anim_activation_mode,
                                 StoryBoardElement*    parent);

        ActivateControllerAction(const ActivateControllerAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE))
        {
            SetName(action.GetName());
            ctrl_name_  = action.ctrl_name_;
            controller_ = action.controller_;
            for (unsigned int i = 0; i < static_cast<unsigned int>(ControlDomains::COUNT); ++i)
            {
                activation_mode_[i] = action.activation_mode_[i];
            }
        }

        OSCPrivateAction* Copy()
        {
            ActivateControllerAction* new_action = new ActivateControllerAction(*this);
            return new_action;
        }

        void Start(double simTime);

        virtual std::string Type2Str()
        {
            return "ActivateControllerAction";
        };

        void Step(double, double)
        {
        }

        void End();
    };

    class VisibilityAction : public OSCPrivateAction
    {
    public:
        bool graphics_;
        bool traffic_;
        bool sensors_;

        VisibilityAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::VISIBILITY, parent, static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE)),
              graphics_(true),
              traffic_(true),
              sensors_(true)
        {
        }

        VisibilityAction(const VisibilityAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::VISIBILITY,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE)),
              graphics_(true),
              traffic_(true),
              sensors_(true)
        {
            SetName(action.GetName());
            graphics_ = action.graphics_;
            traffic_  = action.traffic_;
            sensors_  = action.sensors_;
        }

        OSCPrivateAction* Copy()
        {
            VisibilityAction* new_action = new VisibilityAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "VisibilityAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime);
    };

    class LightStateAction : public OSCPrivateAction
    {
    public:
        LightStateAction(StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LIGHT_STATE_ACTION,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LIGHT)),
              transitionTime_(0.0),
              flashingOffDuration_(0.5),
              flashingOnDuration_(0.5),
              transitionTimer_(0.0),
              flashingTimer_(0.0),
              cmyk_{-1.0, -1.0, -1.0, -1.0},
              rgb_{-1.0, -1.0, -1.0},
              minRgb_{-1.0, -1.0, -1.0},
              maxRgb_{-1.0, -1.0, -1.0},
              previousMinRgb_{-1.0, -1.0, -1.0},
              previousMaxRgb_{-1.0, -1.0, -1.0},
              RGB_ARRAY_SIZE_(3),
              CMYK_ARRAY_SIZE_(4),
              rgbDeducedFromLightType_(false),
              RGB_OFFSET_(0.4),
              DEFAULT_LUMINOUS_INTENSITY_(6000.0),
              LUMINOUS_MAX_(0.9),
              vehicleLightMode_(Object::VehicleLightMode::UNKNOWN),
              previousMode_(vehicleLightMode_),
              luminousIntensity_(0.0),
              previousIntensity_(luminousIntensity_),
              vehicleLightType_(Object::VehicleLightType::UNDEFINED),
              vehicleLightColor_(Object::VehicleLightColor::UNKNOWN),
              actionVehicleLightStatus_({}),
              flashStatus_(FlashingStatus::UNDEFINED),
              colorSet_(false),
              vehicleLight_(nullptr)
        {
        }

        enum class FlashingStatus
        {
            LOW = 0,
            HIGH,
            UNDEFINED
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        Object::VehicleLightType  GetVehicleLightTypeFromStr(const std::string& lightType);
        Object::VehicleLightMode  GetVehicleLightModeFromStr(const std::string& mode);
        Object::VehicleLightColor GetVehicleLightColorFromStr(const std::string& colorType);
        std::vector<double>       GetRgbFromColorEnum(const Object::VehicleLightColor& color);
        void                      CmykToRgb(const double* cmyk, double* rgb);
        void                      SetRgbFromColorEnum(const Object::VehicleLightColor& color);
        void                      SetRgbFromTypeEnum(const Object::VehicleLightType& type);
        int                       CheckAndSetColorError(double* value, int n);
        void                      LightsFlashing(double dt);
        void                      SetLightTransitionValues(const Object::VehicleLightMode& mode);
        void                      RapidTransition();
        void                      SmoothTransition();
        void                      SetRgbMinMaxColor();
        void                      UpdateArray(double* arr, size_t size, const std::vector<double>& vals);
        void                      SetVehicleLightState(Object::VehicleLightStatus* vehicleLight, double luminousity, double transitionFactor = 1.0);
        // Object::VehicleLightType GetLightType();

        // Getters
        double* GetCmyk()
        {
            return cmyk_;
        }
        double* GetRgb()
        {
            return rgb_;
        }
        double GetRgbOffset() const
        {
            return RGB_OFFSET_;
        }
        Object::VehicleLightColor GetVehicleLightColor() const
        {
            return vehicleLightColor_;
        }
        Object::VehicleLightType GetVehicleLightType() const
        {
            return vehicleLightType_;
        }
        bool GetColorSet() const
        {
            return colorSet_;
        }

        // Setters
        void SetTransitionTime(const double& time)
        {
            transitionTime_ = time;
        }
        void SetFlashingOffDuration(const double& duration)
        {
            flashingOffDuration_ = duration;
        }
        void SetFlashingOnDuration(const double& duration)
        {
            flashingOnDuration_ = duration;
        }
        void SetCMYK(const double& c, const double& m, const double& y, const double& k)
        {
            UpdateArray(cmyk_, CMYK_ARRAY_SIZE_, {c, m, y, k});
        }
        void SetRGB(const double& r, const double& g, const double& b)
        {
            UpdateArray(rgb_, RGB_ARRAY_SIZE_, {r, g, b});
        }
        void SetVehicleLightType(const Object::VehicleLightType& type)
        {
            vehicleLightType_ = type;
        }
        void SetLuminousIntensity(const double& intensity)
        {
            luminousIntensity_ = intensity;
        }
        void SetVehicleLightMode(const Object::VehicleLightMode& mode)
        {
            vehicleLightMode_ = mode;
        }
        void SetVehicleLightColor(const Object::VehicleLightColor& color)
        {
            vehicleLightColor_ = color;
        }
        void SetDeducedRgbFromLightType(bool val)
        {
            rgbDeducedFromLightType_ = val;
        }
        void SetVehicleLightInitStatus()
        {
            actionVehicleLightStatus_.type              = this->vehicleLightType_;
            actionVehicleLightStatus_.luminousIntensity = this->luminousIntensity_;
            actionVehicleLightStatus_.mode              = this->vehicleLightMode_;
            actionVehicleLightStatus_.color             = this->vehicleLightColor_;
            std::copy_n(rgb_, RGB_ARRAY_SIZE_, actionVehicleLightStatus_.rgb);
        }
        void SetColorSet(bool val)
        {
            colorSet_ = val;
        }

        std::unordered_map<std::string, Object::VehicleLightType> lightTypeMap = {
            {"daytimeRunningLights", Object::VehicleLightType::DAYTIME_RUNNING_LIGHTS},
            {"lowBeam", Object::VehicleLightType::LOW_BEAM},
            {"highBeam", Object::VehicleLightType::HIGH_BEAM},
            {"fogLights", Object::VehicleLightType::FOG_LIGHTS},
            {"fogLightsFront", Object::VehicleLightType::FOG_LIGHTS_FRONT},
            {"fogLightsRear", Object::VehicleLightType::FOG_LIGHTS_REAR},
            {"brakeLights", Object::VehicleLightType::BRAKE_LIGHTS},
            {"warningLights", Object::VehicleLightType::WARNING_LIGHTS},
            {"indicatorLeft", Object::VehicleLightType::INDICATOR_LEFT},
            {"indicatorRight", Object::VehicleLightType::INDICATOR_RIGHT},
            {"reversingLights", Object::VehicleLightType::REVERSING_LIGHTS},
            {"licensePlateIllumination", Object::VehicleLightType::LICENSE_PLATE_ILLUMINATION},
            {"specialPurposeLights", Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS}};

        std::unordered_map<std::string, Object::VehicleLightColor> lightColorMap = {{"other", Object::VehicleLightColor::OTHER},
                                                                                    {"red", Object::VehicleLightColor::RED},
                                                                                    {"yellow", Object::VehicleLightColor::YELLOW},
                                                                                    {"green", Object::VehicleLightColor::GREEN},
                                                                                    {"blue", Object::VehicleLightColor::BLUE},
                                                                                    {"violet", Object::VehicleLightColor::VIOLET},
                                                                                    {"orange", Object::VehicleLightColor::ORANGE},
                                                                                    {"brown", Object::VehicleLightColor::BROWN},
                                                                                    {"black", Object::VehicleLightColor::BLACK},
                                                                                    {"grey", Object::VehicleLightColor::GREY},
                                                                                    {"white", Object::VehicleLightColor::WHITE}};

        std::unordered_map<std::string, Object::VehicleLightMode> lightModeMap = {{"on", Object::VehicleLightMode::ON},
                                                                                  {"off", Object::VehicleLightMode::OFF},
                                                                                  {"flashing", Object::VehicleLightMode::FLASHING}};

        std::unordered_map<Object::VehicleLightColor, std::vector<double>> baseColorMap = {
            {Object::VehicleLightColor::RED, {0.5, 0.0, 0.0}},
            {Object::VehicleLightColor::YELLOW, {0.5, 0.5, 0.3}},
            {Object::VehicleLightColor::GREEN, {0.0, 0.5, 0.0}},
            {Object::VehicleLightColor::BLUE, {0.0, 0.0, 0.5}},
            {Object::VehicleLightColor::VIOLET, {0.53, 0.31, 0.02}},
            {Object::VehicleLightColor::ORANGE, {0.5, 0.15, 0.0}},
            {Object::VehicleLightColor::BROWN, {0.15, 0.06, 0.06}},
            {Object::VehicleLightColor::BLACK, {0.0, 0.0, 0.0}},
            {Object::VehicleLightColor::GREY, {0.3, 0.3, 0.3}},
            {Object::VehicleLightColor::WHITE, {0.6, 0.6, 0.6}},
        };

    private:
        double                      transitionTime_;
        double                      flashingOffDuration_;
        double                      flashingOnDuration_;
        double                      transitionTimer_;
        double                      flashingTimer_;
        double                      cmyk_[4];
        double                      rgb_[3];
        double                      minRgb_[3];
        double                      maxRgb_[3];
        double                      previousMinRgb_[3];
        double                      previousMaxRgb_[3];
        const size_t                RGB_ARRAY_SIZE_;
        const size_t                CMYK_ARRAY_SIZE_;
        bool                        rgbDeducedFromLightType_;
        const double                RGB_OFFSET_;
        const double                DEFAULT_LUMINOUS_INTENSITY_;
        const double                LUMINOUS_MAX_;
        Object::VehicleLightMode    vehicleLightMode_;
        Object::VehicleLightMode    previousMode_;
        double                      luminousIntensity_;
        double                      previousIntensity_;
        Object::VehicleLightType    vehicleLightType_;
        Object::VehicleLightColor   vehicleLightColor_;
        Object::VehicleLightStatus  actionVehicleLightStatus_;
        FlashingStatus              flashStatus_;
        bool                        colorSet_;
        Object::VehicleLightStatus* vehicleLight_;
    };

    class OverrideControlAction : public OSCPrivateAction
    {
    public:
        Object::OverrideType overrideType_;

        // assume both domains
        OverrideControlAction(double value, bool active, Object::OverrideType type, StoryBoardElement* parent)
            : OSCPrivateAction(OSCPrivateAction::ActionType::OVERRIDE_CONTROLLER,
                               parent,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE)),
              overrideType_(type)
        {
            (void)value;
            (void)active;
        }

        OverrideControlAction(StoryBoardElement* parent) : OverrideControlAction(0, false, Object::OverrideType::OVERRIDE_UNDEFINED, parent)
        {
        }

        OverrideControlAction(const OverrideControlAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::OVERRIDE_CONTROLLER,
                               action.parent_,
                               static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE))
        {
            SetName(action.GetName());
            overrideType_      = action.overrideType_;
            overrideActionList = action.overrideActionList;
        }

        ~OverrideControlAction()
        {
        }

        void Step(double simTime, double dt);
        void Start(double simTime);

        OSCPrivateAction* Copy()
        {
            OverrideControlAction* new_action = new OverrideControlAction(*this);
            return new_action;
        }

        virtual std::string Type2Str()
        {
            return "OverrideControlAction";
        };

        int AddOverrideStatus(Object::OverrideActionStatus status);

        // Input value range: [0..1] for Throttle, Brake, Clutch and ParkingBrake. [-2*PI..2*PI] for SteeringWheel. [-1,0,1,2,3,4,5,6,7,8] for Gear.
        // Function will cut the value to the near limit if the value is beyond limit and round the value in Gear case.
        double RangeCheckAndErrorLog(Object::OverrideType type,
                                     double               valueCheck,
                                     double               lowerLimit = 0.0,
                                     double               upperLimit = 1.0,
                                     bool                 ifRound    = false);

    private:
        std::vector<Object::OverrideActionStatus> overrideActionList;
    };

}  // namespace scenarioengine
