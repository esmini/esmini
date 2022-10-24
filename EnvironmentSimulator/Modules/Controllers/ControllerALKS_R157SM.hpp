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
  *      - any vehicles cutting in (CheckLateralSafety()) from neighbor lanes
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
#include "Controller.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"


#define CONTROLLER_ALKS_R157SM_TYPE_NAME "ALKS_R157SM_Controller"


namespace scenarioengine
{


	class ControllerALKS_R157SM : public Controller
	{
	public:

        enum class ModelType {
            Regulation,
            ReferenceDriver,
            RSS,
            FSM
        };

        enum class ModelMode
        {
            CRITICAL,
            CRUISE_WITH_TARGET,
            CRUISE_NO_TARGET
        };

        class Model
        {
        public:

            typedef struct {
                Object* obj;
                double dist_long;  // distance (headway) to object in focus
                double dist_lat;   // distance (headway) to object in focus
                double dv;         // delta speed (ego speed relative this object)
                double ttc;        // time-to-collision (from ego to this object)
                int dLaneId;       // lane delta (from ego). -1, 0 or 1 (according to OpenDRIVE standard)
            } ObjectInfo;

            Model(ModelType type, double reaction_time, double max_dec_, double max_range_);
            virtual ~Model() = default;

            void SetVehicle(Vehicle* vehicle) { veh_ = vehicle; }
            bool Distance(Vehicle* vh, double& dist_long, double& dist_lat);
            double GetReactionTime() { return rt_; }
            double GetReactionTimeCounter() { return rt_counter_; }
            double GetMaxDec() { return max_dec_; }
            double GetMaxRange() { return max_range_; }
            double Cruise();
            void SetLogging(int logLevel) { log_level_ = logLevel; }
            void SetCruise(bool cruise_active) { cruise_ = cruise_active; }
            void ResetObjectInFocus();
            void ResetReactionTime();
            std::string Mode2Str(ModelMode mode);
            void SetMode(ModelMode mode, bool log = true);
            void SetScenarioEngine(ScenarioEngine* scenario_engine);

            // Returns new speed
            double Step(double dt);

            // Scan traffic and select object to focus on, if any
            void Scan();

            // Returns true if object is considered cutting-in to ego lane
            virtual bool CheckLateralSafety(Object* obj, int dLaneId, double dist_long, double dist_lat) { return false; }

            // Returns true if critical situation and intervention needed
            virtual bool CheckCritical() { return false; };

            // Apply ALKS to react on critical situation
            virtual double ReactCritical() = 0;

            // Model dependent minimal distance, for the generic cruise function
            virtual double MinDist() = 0;

            virtual std::string GetModelName() = 0;

            ModelType type_;
            Vehicle* veh_;
            Entities* entities_;
            ObjectInfo object_in_focus_;
            double cut_in_detected_timestamp_;

            // driver parameters
            double rt_;          // reaction time
            double rt_counter_;  // reaction timer (counting down)

            // vehicle parameters
            double max_dec_;
            double max_range_;   // for detection
            double max_acc_;
            double max_acc_lat_;
            double set_speed_;
            double dt_;
            double acc_;
            double cruise_comfort_acc_;
            double cruise_comfort_dec_;
            double cruise_max_acc_;
            double cruise_max_dec_;

            ModelMode model_mode_;

            int log_level_;  // 0 (none), 1 (log), 2 (log + debug)
            bool cruise_;

            const int deltaLaneId[3] = { -1, 0, 1 };
            const double g = 9.8;
            ScenarioEngine* scenario_engine_;
        };

        class Regulation : Model
        {
        public:
            Regulation() : Model(ModelType::Regulation, 0.35, 6.0, 46.0) {}

            bool CheckLateralSafety(Object* obj, int deltaLaneId, double dist_long, double dist_lat) override;
            bool CheckCritical() override;
            double ReactCritical() override;
            double MinDist() override;

            bool CheckSafety(Vehicle* obj, int dLaneId, double dist_long, double dist_lat) { return false; }
            bool CheckSafetyCutIn(Vehicle* obj, double speed_long, double speed_lat, double dt);
            double React(double speed_long, double dt);
            std::string GetModelName() override { return "Regulation"; }
        };


        class ReferenceDriver : Model
        {
        public:

            enum class Phase
            {
                INACTIVE,
                REACT,
                BRAKE_REF,
                BRAKE_AEB
            };

            // set look ahead distance to 100m
            ReferenceDriver() : Model(ModelType::ReferenceDriver, 0.75, 0.774 * 9.81, 100.0),
                min_jerk_(12.65), release_deceleration_(0.4), critical_ttc_(2.0), phase_(Phase::INACTIVE), timer_(0.0) {}

            bool CheckLateralSafety(Object* obj, int deltaLaneId, double dist_long, double dist_lat) override;
            bool CheckCritical() override;
            double ReactCritical() override;
            double MinDist() override;
            std::string GetModelName() override { return "ReferenceDriver"; }
            void SetPhase(Phase phase);
            std::string Phase2Str(Phase phase);
            void Reset()
            {
                phase_ = Phase::INACTIVE;
            }

            //bool CheckSafety(Vehicle* cutting_in_veh, double speed_long, double speed_lat, double dt) { return false; }
            //bool CheckSafetyCutIn(Vehicle* cutting_in_veh, double speed_long, double speed_lat, double dt);
            //bool Distance(double ur) { return 2 * ur; }
            //double React(double speed_long, double dt);

            double min_jerk_;
            double release_deceleration_;  // deceleration when not stepping on the accelerator pedal(I think)
            double critical_ttc_;
            double timer_;
            Phase phase_;
        };

        class RSS : Model
        {
        public:
            RSS() : Model(ModelType::RSS, 0.75, 0.774 * 9.81, 100.0),
                min_jerk_(12.65), mu_(0.3) {}

            bool CheckLateralSafety(Object* obj, int deltaLaneId, double dist_long, double dist_lat) override;
            bool CheckCritical() override;
            double ReactCritical() override;
            double MinDist() override;
            std::string GetModelName() override { return "RSS"; }

            double min_jerk_;
            double mu_;
        };

        class FSM : Model
        {
        public:
            FSM() : Model(ModelType::FSM, 0.75, 0.774 * 9.81, 46.0), min_jerk_(12.65),
                br_min_(4.0), br_max_(6.0), bl_(7.0), ar_(2.0), margin_dist_(2.0), margin_safe_dist_(2.0),
                cfs_(0.0), pfs_(0.0) {}

            bool CheckLateralSafety(Object* obj, int deltaLaneId, double dist_long, double dist_lat) override;
            bool CheckCritical() override;
            double ReactCritical() override;
            double MinDist() override;
            std::string GetModelName() override { return "FSM"; }

            double PFS(double dist, double speed_rear, double speed_lead, double rt, double br_min, double br_max,
                double bl, double margin_dist, double margin_safe_dist);
            double CFS(double dist, double speed_rear, double speed_lead, double rt,
                double br_min, double br_max, double ar);

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

		Model* model_;
        Entities* entities_;

        ControllerALKS_R157SM(InitArgs* args);
        ~ControllerALKS_R157SM();

		static const char* GetTypeNameStatic() { return CONTROLLER_ALKS_R157SM_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_ALKS_R157SM; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double timeStep);
        void Assign(Object* object);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);
        void SetScenarioEngine(ScenarioEngine* scenario_engine) override;
	};

	Controller* InstantiateControllerALKS_R157SM(void* args);
}