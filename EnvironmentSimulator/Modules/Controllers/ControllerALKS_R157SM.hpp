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

/*
 * This controller implements four different human drivers models proposed as
 * benchmark safety/performance models for ALKS function evaluation.
 *
 * The implementaiton is higly inspired by the open source project:
 * https://github.com/ec-jrc/JRC-FSM
 * Some code has been reused with attempt to generalize wrt number of vehicles,
 * lane widths and driving direction.
 * Also a generic ACC cruise control has been implemented to handle driving
 * during no critical phases.
 *
 * Rough logic flow:
 *   1. Find vehicle to focus on
 *      - consider closest lead vehicle AND
 *      - any vehicles cutting in (CheckSafety()) from neighbor lanes
 *      - pick the one which is closest ahead (probably too simple)
 *   2. Check if relation to vehicle in focus is critical (CheckCritical())
 *   3. If critical, react by adapting own speed (ReactCritical())
 *
 *
 * Limitations:
 *   - Cut-out scenarios not supported yet
 *   -
 */

#pragma once

#include <string>
#include <map>
#include "Controller.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_ALKS_R157SM_TYPE_NAME "ALKS_R157SM_Controller"

namespace scenarioengine
{

    class ControllerALKS_R157SM : public Controller
    {
    public:
        enum class ScenarioType
        {
            None,
            CutIn,
            CutOut,
            Deceleration
        };
        static std::map<ScenarioType, std::string> ScenarioTypeName;

        enum class ModelType
        {
            Regulation,
            ReferenceDriver,
            RSS,
            FSM
        };
        static std::map<ModelType, std::string> ModelTypeName;

        enum class ModelMode
        {
            CRITICAL,
            TARGET_IN_SIGHT,
            NO_TARGET
        };

        class Model
        {
        public:
            struct ObjectInfo
            {
                Object*      obj             = 0;                   // pointer reference to the detected object
                Object*      cut_out_vehicle = 0;                   // pointer reference to cut-out vehicle revealing obj above
                double       dist_long       = LARGE_NUMBER;        // longitudinal distance (freespace) to object in focus
                double       dist_lat        = LARGE_NUMBER;        // lateral distance (freespace) to object in focus
                double       dv_s            = 0.0;                 // delta speed along road direction (ego speed relative this object)
                double       dv_t            = 0.0;                 // delta speed across road direction (ego speed relative this object)
                double       ttc             = LARGE_NUMBER;        // time-to-collision (from ego to this object)
                double       thw             = LARGE_NUMBER;        // time-headway (from ego to this object)
                int          dLaneId         = 0;                   // lane delta (from ego). -1, 0 or 1 (according to OpenDRIVE standard)
                ScenarioType action          = ScenarioType::None;  // set if lane change in or out from ego lane detected
            };

            Model(ModelType type, double reaction_time, double max_dec_, double max_range_);
            virtual ~Model() = default;

            void SetVehicle(Vehicle* vehicle)
            {
                veh_ = vehicle;
            }
            double GetReactionTime()
            {
                return rt_;
            }
            double GetReactionTimeCounter()
            {
                return rt_counter_;
            }
            double GetMaxDec()
            {
                return max_dec_;
            }
            double GetMaxRange()
            {
                return max_range_;
            }
            double Cruise();
            void   SetLogging(int logLevel)
            {
                log_level_ = logLevel;
            }
            void SetCruise(bool cruise_active)
            {
                cruise_ = cruise_active;
            }
            void        ResetObjectInFocus();
            void        ResetReactionTime();
            std::string Mode2Str(ModelMode mode);
            std::string ScenarioType2Str(ScenarioType type)
            {
                return ScenarioTypeName[type];
            }
            std::string ModelType2Str(ModelType type)
            {
                return ModelTypeName[type];
            }
            ModelType GetModelType()
            {
                return type_;
            }
            void      SetModelMode(ModelMode mode, bool log = true);
            ModelMode GetModelMode()
            {
                return model_mode_;
            }
            void         SetScenarioEngine(ScenarioEngine* scenario_engine);
            void         SetScenarioType(ScenarioType type);
            ScenarioType GetScenarioType()
            {
                return scenario_type_;
            }
            void SetFullStop(bool full_stop)
            {
                full_stop_ = full_stop;
            }
            bool GetFullStop()
            {
                return full_stop_;
            }
            void SetAlwaysTrigOnScenario(bool value)
            {
                always_trig_on_scenario_ = value;
            }
            bool GetAlwaysTrigOnScenario()
            {
                return always_trig_on_scenario_;
            }

            // Returns new speed
            double Step(double dt);

            // Scan traffic and select object to focus on, if any
            int Detect();

            int Process(ObjectInfo& info);

            // Returns true if object is not in or intruding the ego lane, else false
            virtual bool CheckSafety(ObjectInfo* info)
            {
                (void)info;
                return false;
            }

            // Returns true if critical situation and intervention needed
            virtual bool CheckCritical()
            {
                return false;
            };

            // Apply ALKS to react on critical situation
            virtual double ReactCritical() = 0;

            // Model dependent minimal distance, for the generic cruise function
            virtual double MinDist() = 0;

            std::string GetModelName()
            {
                return ModelType2Str(type_);
            }

            int GetLogLevel()
            {
                return log_level_;
            }

            virtual void Reset()
            {
            }

            ModelType  type_;
            Vehicle*   veh_;
            Entities*  entities_;
            ObjectInfo object_in_focus_;
            double     cut_in_detected_timestamp_;

            // driver parameters
            double rt_;          // reaction time
            double rt_counter_;  // reaction timer (counting down)

            // vehicle parameters
            double max_dec_;
            double max_range_;  // for detection
            double max_acc_;
            double max_acc_lat_;
            double set_speed_;
            double dt_;
            double acc_;
            double cruise_comfort_acc_;
            double cruise_comfort_dec_;
            double cruise_max_acc_;
            double cruise_max_dec_;

            ModelMode    model_mode_;
            ScenarioType scenario_type_;

            int  log_level_;  // 0 (none), 1 (log), 2 (log + debug)
            bool cruise_;
            bool full_stop_;
            bool always_trig_on_scenario_;

            const int       deltaLaneId[3] = {-1, 0, 1};
            const double    g              = 9.8;
            ScenarioEngine* scenario_engine_;

            virtual bool CheckPerceptionCutIn()
            {
                return false;
            }
            virtual bool CheckPerceptionCutOut()
            {
                return false;
            }
            virtual bool CheckPerceptionDeceleration()
            {
                return false;
            }
            virtual bool CheckCriticalCutIn()
            {
                return false;
            }
            virtual bool CheckCriticalCutOut()
            {
                return false;
            }
            virtual bool CheckCriticalDeceleration()
            {
                return false;
            }
        };

        class Regulation : Model
        {
        public:
            Regulation() : Model(ModelType::Regulation, 0.35, 6.0, 46.0)
            {
            }

            bool   CheckSafety(ObjectInfo* info) override;
            bool   CheckCritical() override;
            double ReactCritical() override;
            double MinDist() override;

            bool CheckSafety(Vehicle* obj, int dLaneId, double dist_long, double dist_lat)
            {
                (void)obj;
                (void)dLaneId;
                (void)dist_long;
                (void)dist_lat;
                return false;
            }
            bool   CheckSafetyCutIn(Vehicle* obj, double speed_long, double speed_lat, double dt);
            double React(double speed_long, double dt);
        };

        // Assumptions and design choices:
        //   Conditions for an enitity to be "detected":
        //      - is in front of ego and within max range of 100 meter
        //      - pedestrians: is overlapping ego laterally OR moving towards ego laterally
        //      - other entities: is in same or adjecent lane to ego
        //   Velocity calculations:
        //      - Longitudinal velocity is based on latest movement and projected along road direction (s axis)
        //      - Lateral velocity is based on latest movement projected across road direction (t axis)
        //   Free space distance calculations:
        //      - overlap (collision) detected and reported as zero distance
        //      - distance calculations are mapped to the road coordinate system (s, t):
        //          s = longitudinal distance along (projected to) road reference line
        //          t = lateral distance across (perpendicular to) the road reference line
        //      - pros: Longitudinal distance will consider curvature (not a straight line)
        //      - cons: Longitudinal distance will not consider lateral offset from reference line
        //              E.g. the distance between two vehicles in an outer lane going through
        //              a curve will appear smaller or larger when projected on the reference
        //              line, depending on left or right curve.
        //      - Alternative solution would be to use either euclidian distance (straight lines) or
        //        lane coordinate system which would consider lateral offset. Euclidian is currently
        //        supported in esmini while lane coordinate system is not yet for distance calculations.
        //   TTC calculation:
        //      - Freespace distance (as above) divided by relative speed
        //   Lateral lane offset representing cut-in perception time:
        //      Sum of lane wandering threshold 0.375m according to regulation and additional perception delay:
        //      - option 1: 0.72 meter ("worst case" based on 0.4 s perception time according to regulation)
        //      - option 2: 0.4 seconds delay
        //   Main flow:
        //      Perception time starts as soon as object and action is detected, not waiting for critical criteria
        //      Criticality check (ttc < 2.0 or hwt < 2.0) is done after the situation has been perceived
        //      If critical, the reaction times is started
        //      When reaction is done, criticality is checked once again
        //      If still critical, enter critical reaction phase
        //   Brake behavior: Option to either brake until full stop OR until critical situation avoided
        //   Pedestrian wrap = 100% intepreted as lateral distance == 0 (overlapping to some degree)
        //
        class ReferenceDriver : Model
        {
        public:
            enum class CutInPerceptionDelayMode
            {
                DIST,
                TIME
            };

            enum class Phase
            {
                INACTIVE,
                PERCEIVE,
                REACT,
                BRAKE_REF,
                BRAKE_AEB
            };
            static std::map<Phase, std::string> PhaseName;

            struct AEB
            {
                AEB() : ttc_critical_aeb_(1.5)
                {
                    Reset();
                }

                void Reset()
                {
                    active_ = false;
                }

                double ttc_critical_aeb_;
                bool   active_;
            };

            class LateralDistTrigger
            {
            public:
                LateralDistTrigger(ReferenceDriver* ref_driver) : model_(ref_driver), threshold_(0.0), name_("LateralTrigger")
                {
                    Reset();
                }

                virtual ~LateralDistTrigger() = default;

                bool Active()
                {
                    return active_;
                }
                double GetDistance()
                {
                    return obj_ ? abs(obj_->pos_.GetT() - t0_) : 0.0;
                }
                void Reset()
                {
                    active_ = false;
                    obj_    = nullptr;
                    t0_     = 0.0;
                }
                virtual void Update(ObjectInfo* info);
                bool         Evaluate();
                std::string  GetModelName()
                {
                    return model_ ? model_->GetModelName() : "";
                }
                int GetLogLevel()
                {
                    return model_ ? model_->GetLogLevel() : 0;
                }
                void SetName(std::string name)
                {
                    name_ = name;
                }

                ReferenceDriver* model_;
                double           threshold_;
                bool             active_;
                Object*          obj_;
                double           t0_;  // road coordinate t value at detection time
                std::string      name_;
            };

            class WanderingTrigger : public LateralDistTrigger
            {
            public:
                WanderingTrigger(ReferenceDriver* ref_driver) : LateralDistTrigger(ref_driver)
                {
                }
                ~WanderingTrigger() = default;
                void Update(ObjectInfo* info);
            };

            // set look ahead distance to 100m
            ReferenceDriver()
                : Model(ModelType::ReferenceDriver, 0.75, 0.774 * 9.81, 100.0),
                  c_lane_offset_(0.0),
                  min_jerk_(12.65),
                  release_deceleration_(0.4),
                  critical_ttc_(2.0),
                  critical_thw_(2.0),
                  phase_(Phase::INACTIVE),
                  timer_(0.0),
                  cut_in_perception_delay_mode_(CutInPerceptionDelayMode::DIST),
                  perception_dist_(0.72),
                  perception_time_(0.4),
                  wandering_threshold_(0.375),
                  overlap_tolerance_(0.1),
                  pedestrian_risk_eval_time_(0.4),
                  perception_t_(0.0),
                  lateral_dist_trigger_(0),
                  wandering_trigger_(0)
            {
            }

            ~ReferenceDriver();

            bool   CheckSafety(ObjectInfo* info) override;
            bool   CheckCritical() override;
            double ReactCritical() override;
            double MinDist() override;
            void   SetPhase(Phase phase);
            Phase  GetPhase()
            {
                return phase_;
            }
            std::string Phase2Str(Phase phase);
            void        Reset() override
            {
                SetPhase(Phase::INACTIVE);
                SetScenarioType(ScenarioType::None);
            }
            void UpdateAEB(Vehicle* ego, ObjectInfo* info);

            double                   c_lane_offset_;
            double                   min_jerk_;
            double                   release_deceleration_;  // deceleration when not stepping on the accelerator pedal(I think)
            double                   critical_ttc_;
            double                   critical_thw_;
            Phase                    phase_;
            double                   timer_;
            CutInPerceptionDelayMode cut_in_perception_delay_mode_;
            double                   perception_dist_;
            double                   perception_time_;
            double                   wandering_threshold_;
            double                   overlap_tolerance_;
            double                   pedestrian_risk_eval_time_;
            double                   perception_t_;  // t-value when target has been perceived
            AEB                      aeb_;
            LateralDistTrigger*      lateral_dist_trigger_;
            WanderingTrigger*        wandering_trigger_;

            bool CheckPerception()
            {
                if (GetScenarioType() == ScenarioType::CutIn)
                {
                    return CheckPerceptionCutIn();
                }
                else if (GetScenarioType() == ScenarioType::CutOut)
                {
                    return CheckPerceptionCutOut();
                }
                else if (GetScenarioType() == ScenarioType::Deceleration)
                {
                    return CheckPerceptionDeceleration();
                }
                else
                {
                    return false;
                }
            }
            bool CheckPerceptionCutIn() override;
            bool CheckPerceptionCutOut() override;
            bool CheckPerceptionDeceleration() override;

            bool CheckCriticalCondition()
            {
                if (GetScenarioType() == ScenarioType::CutIn)
                {
                    return CheckCriticalCutIn();
                }
                else if (GetScenarioType() == ScenarioType::CutOut)
                {
                    return CheckCriticalCutOut();
                }
                else if (GetScenarioType() == ScenarioType::Deceleration)
                {
                    return CheckCriticalDeceleration();
                }
                else
                {
                    return false;
                }
            }
            bool CheckCriticalCutIn() override;
            bool CheckCriticalCutOut() override;
            bool CheckCriticalDeceleration() override;
            void SetPedestrianRiskEvaluationTime(double value)
            {
                pedestrian_risk_eval_time_ = value;
            }
            double GetPedestrianRiskEvaluationTime()
            {
                return pedestrian_risk_eval_time_;
            }
        };

        class RSS : Model
        {
        public:
            RSS() : Model(ModelType::RSS, 0.75, 0.774 * 9.81, 100.0), min_jerk_(12.65), mu_(0.3)
            {
            }

            bool   CheckSafety(ObjectInfo* info) override;
            bool   CheckCritical() override;
            double ReactCritical() override;
            double MinDist() override;

            double min_jerk_;
            double mu_;
        };

        class FSM : Model
        {
        public:
            FSM()
                : Model(ModelType::FSM, 0.75, 0.774 * 9.81, 46.0),
                  min_jerk_(12.65),
                  br_min_(4.0),
                  br_max_(6.0),
                  bl_(7.0),
                  ar_(2.0),
                  margin_dist_(2.0),
                  margin_safe_dist_(2.0),
                  cfs_(0.0),
                  pfs_(0.0)
            {
            }

            bool   CheckSafety(ObjectInfo* info) override;
            bool   CheckCritical() override;
            double ReactCritical() override;
            double MinDist() override;

            double PFS(double dist,
                       double speed_rear,
                       double speed_lead,
                       double rt,
                       double br_min,
                       double br_max,
                       double bl,
                       double margin_dist,
                       double margin_safe_dist);
            double CFS(double dist, double speed_rear, double speed_lead, double rt, double br_min, double br_max, double ar);

            double min_jerk_;
            double br_min_;
            double br_max_;
            double bl_;
            double ar_;
            double margin_dist_;
            double margin_safe_dist_;
            double cfs_;
            double pfs_;
        };

        Model*    model_;
        Entities* entities_;

        ControllerALKS_R157SM(InitArgs* args);
        ~ControllerALKS_R157SM();

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_ALKS_R157SM_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return CONTROLLER_ALKS_R157SM;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
        }

        void Init();
        void Step(double timeStep);
        void LinkObject(Object* object);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);
        void ReportKeyEvent(int key, bool down);
        void SetScenarioEngine(ScenarioEngine* scenario_engine) override;
    };

    Controller* InstantiateControllerALKS_R157SM(void* args);
}  // namespace scenarioengine