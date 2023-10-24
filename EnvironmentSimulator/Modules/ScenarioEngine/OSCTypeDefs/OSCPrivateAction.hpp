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

#include "OSCAction.hpp"
#include "OSCPosition.hpp"
#include "Entities.hpp"
#include "CommonMini.hpp"
#include "Controller.hpp"

#include <iostream>
#include <string>
#include <math.h>
#include <memory>

namespace scenarioengine
{

#define DISTANCE_TOLERANCE       (0.5)  // meter
#define SYNCH_DISTANCE_TOLERANCE (1.0)  // meter
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
        enum class ActionType
        {
            LONG_SPEED,
            LONG_SPEED_PROFILE,
            LONG_DISTANCE,
            LAT_LANE_CHANGE,
            LAT_LANE_OFFSET,
            LAT_DISTANCE,
            VISIBILITY,
            CONTROLLER,
            ASSIGN_CONTROLLER,
            ACTIVATE_CONTROLLER,
            OVERRIDE_CONTROLLER,
            TELEPORT,
            ASSIGN_ROUTE,
            FOLLOW_TRAJECTORY,
            Acquire_POSITION,
            SYNCHRONIZE_ACTION,
            CONNECT_TRAILER_ACTION,
            LIGHT_STATE_ACTION
        };

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

            double Evaluate();  // 0 = start_value, 1 = end_value
            double EvaluatePrim();
            double EvaluateScaledPrim();
            double EvaluatePrimPeak();
            int    Step(double delta_param_val);
            double GetTargetParamValByPrimPeak(double prim_peak);
            double GetTargetParamValByPrimPrimPeak(double prim_prim_peak);

            double GetParamVal()
            {
                return param_val_;
            }
            void   SetStartVal(double start_val);
            double GetStartVal()
            {
                return start_val_;
            }
            void   SetTargetVal(double target_val);
            double GetTargetVal()
            {
                return target_val_;
            }

            void   SetParamTargetVal(double target_value);
            double GetParamTargetVal()
            {
                return param_target_val_;
            }
            void   SetMaxRate(double max_rate);
            void   SetRate(double rate);
            void   UpdateRate();
            double GetRate()
            {
                return rate_;
            }
            double GetScaleFactor()
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
        ControlDomains  domain_;
        Object*         object_;
        ScenarioEngine* scenarioEngine_;

        OSCPrivateAction(OSCPrivateAction::ActionType type, ControlDomains domain)
            : OSCAction(OSCAction::BaseType::PRIVATE),
              type_(type),
              domain_(domain),
              object_(0),
              scenarioEngine_(0)
        {
        }

        virtual ~OSCPrivateAction() = default;

        virtual void print()
        {
            LOG("Virtual, should be overridden");
        };

        virtual OSCPrivateAction* Copy()
        {
            LOG("Virtual, should be overridden");
            return 0;
        };

        std::string Type2Str()
        {
            return "OSCPrivateAction base class";
        };

        ControlDomains GetDomain()
        {
            return domain_;
        }

        virtual void ReplaceObjectRefs(Object*, Object*){};

        const std::string DomainActivation2Str(Controller::DomainActivation mode) const
        {
            switch (mode)
            {
                case Controller::DomainActivation::UNDEFINED:
                    return "UNDEFINED";
                case Controller::DomainActivation::OFF:
                    return "OFF";
                case Controller::DomainActivation::ON:
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

            TargetRelative() : Target(TargetType::RELATIVE_SPEED), continuous_(false), consumed_(false), object_speed_(0)
            {
            }

            double GetValue();

        private:
            bool   consumed_;
            double object_speed_;
        };

        std::shared_ptr<Target> target_;
        bool                    target_speed_reached_;

        LongSpeedAction()
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED, ControlDomains::DOMAIN_LONG),
              target_(0),
              target_speed_reached_(false)
        {
        }

        LongSpeedAction(const LongSpeedAction& action) : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED, ControlDomains::DOMAIN_LONG)
        {
            name_                 = action.name_;
            target_               = action.target_;
            transition_           = action.transition_;
            target_speed_reached_ = action.target_speed_reached_;
        }

        OSCPrivateAction* Copy()
        {
            LongSpeedAction* new_action = new LongSpeedAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "SpeedAction";
        };

        void Start(double simTime, double dt);
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

        LongSpeedProfileAction()
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED_PROFILE, ControlDomains::DOMAIN_LONG),
              following_mode_(FollowingMode::POSITION),
              entity_ref_(nullptr),
              cur_index_(0),
              speed_(0.0),
              acc_(0.0)
        {
        }

        LongSpeedProfileAction(FollowingMode follow_mode)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED_PROFILE, ControlDomains::DOMAIN_LONG),
              following_mode_(follow_mode),
              entity_ref_(nullptr),
              cur_index_(0),
              speed_(0.0),
              acc_(0.0)
        {
        }

        LongSpeedProfileAction(const LongSpeedProfileAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED_PROFILE, ControlDomains::DOMAIN_LONG)
        {
            name_           = action.name_;
            following_mode_ = action.following_mode_;
            entry_          = action.entry_;
            entity_ref_     = action.entity_ref_;
            speed_          = action.speed_;
            acc_            = action.acc_;
            cur_index_      = action.cur_index_;
        }

        OSCPrivateAction* Copy()
        {
            LongSpeedProfileAction* new_action = new LongSpeedProfileAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "SpeedProfileAction";
        };

        void Start(double simTime, double dt = 0.0);
        void Step(double simTime, double dt = 0.0);

        void print()
        {
            LOG("");
        }

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

        void AddEntry(LongSpeedProfileAction::Entry entry)
        {
            entry_.push_back(entry);
        }
        double GetStartTime()
        {
            return start_time_;
        }
        double GetElapsedTime()
        {
            return elapsed_;
        }
        double GetSpeed()
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

        Object*            target_object_;
        double             distance_;
        DistType           dist_type_;
        double             freespace_;
        bool               continuous_;
        double             sim_time_;
        DisplacementType   displacement_;
        DynamicConstraints dynamics_;

        LongDistanceAction()
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_DISTANCE, ControlDomains::DOMAIN_LONG),
              target_object_(0),
              distance_(0),
              dist_type_(DistType::DISTANCE),
              freespace_(0),
              sim_time_(0),
              displacement_(DisplacementType::NONE),
              acceleration_(0)
        {
        }

        LongDistanceAction(const LongDistanceAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_DISTANCE, ControlDomains::DOMAIN_LONG)
        {
            name_          = action.name_;
            target_object_ = action.target_object_;
            dynamics_      = action.dynamics_;
            distance_      = action.distance_;
            dist_type_     = action.dist_type_;
            freespace_     = action.freespace_;
            acceleration_  = action.acceleration_;
            sim_time_      = action.sim_time_;
            displacement_  = action.displacement_;
        }

        OSCPrivateAction* Copy()
        {
            LongDistanceAction* new_action = new LongDistanceAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "LongitudinalDistanceAction";
        };

        void Start(double simTime, double dt);
        void Step(double simTime, double dt);

        void print()
        {
        }

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

    private:
        double acceleration_;
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
        };

        class TargetRelative : public Target
        {
        public:
            Object* object_;

            TargetRelative() : Target(Target::Type::RELATIVE_LANE), object_(0)
            {
            }
        };

        std::shared_ptr<Target> target_;
        TransitionDynamics      transition_;
        double                  target_lane_offset_;

        LatLaneChangeAction(LatLaneChangeAction::DynamicsDimension timing_type = DynamicsDimension::TIME)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_CHANGE, ControlDomains::DOMAIN_LAT),
              target_(0),
              target_lane_offset_(0.0),
              start_offset_(0.0)
        {
            transition_.dimension_ = timing_type;
        }

        LatLaneChangeAction(const LatLaneChangeAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_CHANGE, ControlDomains::DOMAIN_LAT),
              target_(action.target_),
              transition_(action.transition_),
              target_lane_offset_(action.target_lane_offset_),
              start_offset_(action.start_offset_)
        {
            name_ = action.name_;
        }

        OSCPrivateAction* Copy()
        {
            LatLaneChangeAction* new_action = new LatLaneChangeAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "LaneChangeAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime, double dt);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

    private:
        double                start_offset_;
        roadmanager::Position internal_pos_;  // Internal position representation
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

        LatLaneOffsetAction() : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_OFFSET, ControlDomains::DOMAIN_LAT)
        {
            max_lateral_acc_ = 0;
            target_          = 0;
        }

        LatLaneOffsetAction(const LatLaneOffsetAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_OFFSET, ControlDomains::DOMAIN_LAT)
        {
            name_            = action.name_;
            target_          = action.target_;
            max_lateral_acc_ = action.max_lateral_acc_;
            transition_      = action.transition_;
        }

        OSCPrivateAction* Copy()
        {
            LatLaneOffsetAction* new_action = new LatLaneOffsetAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "LaneOffsetAction";
        };

        void Start(double simTime, double dt);
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

        std::shared_ptr<OSCPosition> steadyState_OSCPosition_;
        struct
        {
            SteadyStateType type_;
            union
            {
                roadmanager::Position* pos_;
                double                 time_;
                double                 dist_;
            };
        } steadyState_;

        SynchMode    mode_;
        SynchSubmode submode_;

        std::shared_ptr<OSCPosition>             target_position_master_OSCPosition_;
        std::shared_ptr<OSCPosition>             target_position_OSCPosition_;
        roadmanager::Position*                   target_position_master_;
        roadmanager::Position*                   target_position_;
        Object*                                  master_object_;
        std::shared_ptr<LongSpeedAction::Target> final_speed_;
        double                                   tolerance_;
        double                                   tolerance_master_;

        // Store calculated distances to use for comparison
        double lastDist_;
        double lastMasterDist_;

        SynchronizeAction() : OSCPrivateAction(OSCPrivateAction::ActionType::SYNCHRONIZE_ACTION, ControlDomains::DOMAIN_LONG)
        {
            steadyState_OSCPosition_ = nullptr;
            master_object_           = 0;
            final_speed_             = 0;
            target_position_master_  = 0;
            target_position_         = 0;
            mode_                    = SynchMode::MODE_NONE;
            submode_                 = SynchSubmode::SUBMODE_NONE;
            lastDist_                = LARGE_NUMBER;
            lastMasterDist_          = LARGE_NUMBER;
            tolerance_               = SYNCH_DISTANCE_TOLERANCE;
            tolerance_master_        = SYNCH_DISTANCE_TOLERANCE;
            steadyState_.type_       = SteadyStateType::STEADY_STATE_NONE;
        }

        SynchronizeAction(const SynchronizeAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::SYNCHRONIZE_ACTION, ControlDomains::DOMAIN_LONG)
        {
            name_                               = action.name_;
            steadyState_OSCPosition_            = action.steadyState_OSCPosition_;
            target_position_master_OSCPosition_ = action.target_position_master_OSCPosition_;
            target_position_OSCPosition_        = action.target_position_OSCPosition_;
            master_object_                      = action.master_object_;
            final_speed_                        = action.final_speed_;
            target_position_master_             = action.target_position_master_;
            target_position_                    = action.target_position_;
            mode_                               = action.mode_;
            submode_                            = action.submode_;
            lastDist_                           = LARGE_NUMBER;
            lastMasterDist_                     = LARGE_NUMBER;
            tolerance_                          = SYNCH_DISTANCE_TOLERANCE;
            tolerance_master_                   = SYNCH_DISTANCE_TOLERANCE;
            steadyState_                        = action.steadyState_;
        }

        OSCPrivateAction* Copy()
        {
            SynchronizeAction* new_action = new SynchronizeAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "SynchronizeAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime, double dt);

        const char* Mode2Str(SynchMode mode);

    private:
        double CalcSpeedForLinearProfile(double v_final, double time, double dist);
        void   PrintStatus(const char* custom_msg);
        // const char* Mode2Str(SynchMode mode);
        const char* SubMode2Str(SynchSubmode submode);

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
        std::shared_ptr<OSCPosition> position_OSCPosition_;
        roadmanager::Position*       position_;

        TeleportAction() : OSCPrivateAction(OSCPrivateAction::ActionType::TELEPORT, ControlDomains::DOMAIN_BOTH), ghost_restart_(false)
        {
        }

        TeleportAction(const TeleportAction& action) : OSCPrivateAction(OSCPrivateAction::ActionType::TELEPORT, ControlDomains::DOMAIN_BOTH)
        {
            name_                 = action.name_;
            position_OSCPosition_ = action.position_OSCPosition_;
            position_             = action.position_;
            ghost_restart_        = action.ghost_restart_;
        }

        ~TeleportAction()
        {
            if (!position_OSCPosition_ && position_ != nullptr)
            {
                delete position_;
            }
        }

        OSCPrivateAction* Copy()
        {
            TeleportAction* new_action = new TeleportAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "TeleportAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime, double dt);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
        void SetGhostRestart(bool value)
        {
            ghost_restart_ = value;
        }

    private:
        bool ghost_restart_;

        bool IsGhostRestart()
        {
            return ghost_restart_;
        }
    };

    class ConnectTrailerAction : public OSCPrivateAction
    {
    public:
        ConnectTrailerAction() : OSCPrivateAction(OSCPrivateAction::ActionType::CONNECT_TRAILER_ACTION, ControlDomains::DOMAIN_NONE)
        {
        }

        ConnectTrailerAction(const ConnectTrailerAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::CONNECT_TRAILER_ACTION, ControlDomains::DOMAIN_NONE)
        {
            name_           = action.name_;
            trailer_object_ = action.trailer_object_;
        }

        OSCPrivateAction* Copy()
        {
            ConnectTrailerAction* new_action = new ConnectTrailerAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "ConnectTrailerAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime, double dt);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);

        Object* trailer_object_ = nullptr;
    };

    class AssignRouteAction : public OSCPrivateAction
    {
    public:
        roadmanager::Route* route_;

        AssignRouteAction() : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_ROUTE, ControlDomains::DOMAIN_NONE), route_(0)
        {
        }

        AssignRouteAction(const AssignRouteAction& action) : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_ROUTE, ControlDomains::DOMAIN_NONE)
        {
            name_  = action.name_;
            route_ = new roadmanager::Route(*action.route_);
        }

        ~AssignRouteAction()
        {
            delete route_;
        }

        OSCPrivateAction* Copy()
        {
            AssignRouteAction* new_action = new AssignRouteAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "AssignRouteAction";
        };

        void Start(double simTime, double dt);
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

        std::shared_ptr<roadmanager::RMTrajectory> traj_;
        TimingDomain                               timing_domain_;
        FollowingMode                              following_mode_;
        double                                     timing_scale_;
        double                                     timing_offset_;
        double                                     time_;
        double                                     initialDistanceOffset_;
        bool                                       reverse_;

        FollowTrajectoryAction()
            : OSCPrivateAction(OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY, ControlDomains::DOMAIN_BOTH),
              traj_(0),
              timing_domain_(TimingDomain::NONE),
              following_mode_(FollowingMode::FOLLOW),
              timing_scale_(1),
              timing_offset_(0),
              time_(0),
              initialDistanceOffset_(0),
              reverse_(false)
        {
        }

        FollowTrajectoryAction(const FollowTrajectoryAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY, ControlDomains::DOMAIN_BOTH)
        {
            name_                  = action.name_;
            traj_                  = action.traj_;
            timing_domain_         = action.timing_domain_;
            timing_scale_          = action.timing_scale_;
            timing_offset_         = action.timing_offset_;
            initialDistanceOffset_ = action.timing_offset_;
            following_mode_        = action.following_mode_;
            reverse_               = action.reverse_;
            time_                  = 0;
        }

        OSCPrivateAction* Copy()
        {
            FollowTrajectoryAction* new_action = new FollowTrajectoryAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "FollowTrajectoryAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime, double dt);
        void End(double simTime);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
    };

    class AcquirePositionAction : public OSCPrivateAction
    {
    public:
        std::shared_ptr<OSCPosition>        target_position_OSCPosition_;
        roadmanager::Position*              target_position_;
        std::shared_ptr<roadmanager::Route> route_;

        AcquirePositionAction()
            : OSCPrivateAction(OSCPrivateAction::ActionType::Acquire_POSITION, ControlDomains::DOMAIN_LONG),
              target_position_(0),
              route_(0)
        {
        }

        AcquirePositionAction(const AcquirePositionAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::Acquire_POSITION, ControlDomains::DOMAIN_LONG)
        {
            name_                        = action.name_;
            target_position_OSCPosition_ = action.target_position_OSCPosition_;
            target_position_             = action.target_position_;
            route_                       = action.route_;
        }

        OSCPrivateAction* Copy()
        {
            AcquirePositionAction* new_action = new AcquirePositionAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "AcquirePositionAction";
        };

        void Start(double simTime, double dt);
        void Step(double simTime, double dt);

        void ReplaceObjectRefs(Object* obj1, Object* obj2);
    };

    class AssignControllerAction : public OSCPrivateAction
    {
    public:
        Controller*                  controller_;
        Controller::DomainActivation lateral_      = Controller::DomainActivation::OFF;
        Controller::DomainActivation longitudinal_ = Controller::DomainActivation::OFF;

        AssignControllerAction(Controller* controller, Controller::DomainActivation lateral, Controller::DomainActivation longitudinal)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_CONTROLLER, ControlDomains::DOMAIN_NONE),
              controller_(controller),
              lateral_(lateral),
              longitudinal_(longitudinal)
        {
        }

        AssignControllerAction(const AssignControllerAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_CONTROLLER, ControlDomains::DOMAIN_NONE)
        {
            name_         = action.name_;
            controller_   = action.controller_;
            lateral_      = action.lateral_;
            longitudinal_ = action.longitudinal_;
        };

        OSCPrivateAction* Copy()
        {
            AssignControllerAction* new_action = new AssignControllerAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "AssignControllerAction";
        };

        void Step(double, double)
        {
        }

        void Start(double simTime, double dt);
    };

    class ActivateControllerAction : public OSCPrivateAction
    {
    public:
        Controller::DomainActivation lateral_      = Controller::DomainActivation::OFF;
        Controller::DomainActivation longitudinal_ = Controller::DomainActivation::OFF;

        /**
        Default constructor assuming both domains (lat/long) activated
        @param domainMask bitmask according to Controller::Domain type
        */
        ActivateControllerAction()
            : OSCPrivateAction(OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER, ControlDomains::DOMAIN_BOTH),
              lateral_(Controller::DomainActivation::ON),
              longitudinal_(Controller::DomainActivation::ON)
        {
        }

        /**
        Constructor with domain specification
        @param domainMask bitmask according to Controller::Domain type
        */
        ActivateControllerAction(Controller::DomainActivation lateral, Controller::DomainActivation longitudinal)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER, ControlDomains::DOMAIN_NONE),
              lateral_(lateral),
              longitudinal_(longitudinal)
        {
        }

        ActivateControllerAction(const ActivateControllerAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER, ControlDomains::DOMAIN_NONE)
        {
            name_         = action.name_;
            lateral_      = action.lateral_;
            longitudinal_ = action.longitudinal_;
        }

        OSCPrivateAction* Copy()
        {
            ActivateControllerAction* new_action = new ActivateControllerAction(*this);
            return new_action;
        }

        void Start(double simTime, double dt)
        {
            if (object_->GetAssignedControllerType() != 0)
            {
                object_->controller_->Activate(lateral_, longitudinal_);
                LOG("Controller %s activated (lat %s, long %s), domain mask=0x%X",
                    object_->controller_->GetName().c_str(),
                    DomainActivation2Str(lateral_).c_str(),
                    DomainActivation2Str(longitudinal_).c_str(),
                    object_->controller_->GetDomain());
                OSCAction::Start(simTime, dt);
            }
            else
            {
                LOG("No controller assigned to object %s!", object_->name_.c_str());
            }
        }

        std::string Type2Str()
        {
            return "ActivateControllerAction";
        };

        void Step(double, double)
        {
        }

        void End(double simTime)
        {
            if (object_->GetActivatedControllerType() != 0 && object_->controller_ != nullptr)
            {
                object_->controller_->Deactivate();
            }
            // Make sure heading is aligned with road driving direction
            object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
            OSCAction::End(simTime);
        }
    };

    class VisibilityAction : public OSCPrivateAction
    {
    public:
        bool graphics_;
        bool traffic_;
        bool sensors_;

        VisibilityAction()
            : OSCPrivateAction(OSCPrivateAction::ActionType::VISIBILITY, ControlDomains::DOMAIN_NONE),
              graphics_(true),
              traffic_(true),
              sensors_(true)
        {
        }

        VisibilityAction(const VisibilityAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::VISIBILITY, ControlDomains::DOMAIN_NONE),
              graphics_(true),
              traffic_(true),
              sensors_(true)
        {
            name_     = action.name_;
            graphics_ = action.graphics_;
            traffic_  = action.traffic_;
            sensors_  = action.sensors_;
        }

        OSCPrivateAction* Copy()
        {
            VisibilityAction* new_action = new VisibilityAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "VisibilityAction";
        };

        void Step(double simTime, double dt);
        void Start(double simTime, double dt);
    };

    class LightStateAction : public OSCPrivateAction
    {
    public:
        double transitionTime_;
        double flashingOffDuration_;
        double flashingOnDuration_;
        double cmyk_[4];

        LightStateAction()
            : OSCPrivateAction(OSCPrivateAction::ActionType::LIGHT_STATE_ACTION, ControlDomains::DOMAIN_LIGHT),
              transitionTime_(0.0),
              flashingOffDuration_(0.5),
              flashingOnDuration_(0.5),
              cmyk_{-1.0, -1.0, -1.0, -1.0}
        {
        }

        double transitionTimer_ = SMALL_NUMBER;
        double flashingTimer_   = SMALL_NUMBER;
        double initialDiffRgb_[3];
        double finalDiffRgb_[3];
        double initialEmissionRgb_[3];
        double finalEmissionRgb_[3];

        bool                     isUserSetRgb       = true;
        bool                     isRgbFromLightType = false;
        double                   lum_max            = 0.9;
        double                   lum_default        = 0.5;
        Object::VehicleLightMode perviousMode;
        double                   perviousIntensity;
        double                   baseRgb[3];

        enum class flashingStatus
        {
            LOW       = 1,
            HIGH      = 2,
            UNDEFINED = 3
        };
        flashingStatus flashStatus;

        int parseVehicleLightType(std::string typeObject, Object::VehicleLightActionStatus& lightStatus);
        int parseVehicleLightMode(std::string mode, Object::VehicleLightActionStatus& lightStatus);
        int parseVehicleLightColor(std::string colorType, Object::VehicleLightActionStatus& lightStatus);

        void Step(double simTime, double dt);
        void Start(double simTime, double dt);
        void AddVehicleLightActionStatus(Object::VehicleLightActionStatus lightStatus);
        int  setLightTransistionValues(Object::VehicleLightMode mode);

        int  checkColorError(double* value, int n);
        void convertColorAndSetBaseRgb(Object::VehicleLightActionStatus& lightStatus);
        void convertLightTypeAndSetBaseRgb(Object::VehicleLightActionStatus& lightStatus);

        int setbaseRgbAndPrepare(Object::VehicleLightActionStatus& lightStatus);

        Object::VehicleLightType GetLightType()
        {
            return vehicleLightActionStatus.type;
        }

    private:
        Object::VehicleLightActionStatus vehicleLightActionStatus;
    };

    class OverrideControlAction : public OSCPrivateAction
    {
    public:
        Object::OverrideType type_;

        // assume both domains
        OverrideControlAction(double value, bool active, Object::OverrideType type)
            : OSCPrivateAction(OSCPrivateAction::ActionType::OVERRIDE_CONTROLLER, ControlDomains::DOMAIN_NONE),
              type_(type)
        {
            (void)value;
            (void)active;
        }

        OverrideControlAction() : OverrideControlAction(0, false, Object::OverrideType::OVERRIDE_UNDEFINED)
        {
        }

        OverrideControlAction(const OverrideControlAction& action)
            : OSCPrivateAction(OSCPrivateAction::ActionType::OVERRIDE_CONTROLLER, ControlDomains::DOMAIN_NONE)
        {
            name_              = action.name_;
            type_              = action.type_;
            overrideActionList = action.overrideActionList;
        }

        ~OverrideControlAction()
        {
        }

        void Step(double simTime, double dt);
        void Start(double simTime, double dt);

        OSCPrivateAction* Copy()
        {
            OverrideControlAction* new_action = new OverrideControlAction(*this);
            return new_action;
        }

        std::string Type2Str()
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
