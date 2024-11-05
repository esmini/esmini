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

#include "Controller.hpp"
#include "Entities.hpp"
#include <array>
#include <unordered_map>

#define CONTROLLER_NATURAL_DRIVER_TYPE_NAME "NaturalDriver"

namespace scenarioengine
{
    struct VoI  // VehicleOfInterest
    {
        scenarioengine::Object*   vehicle;
        roadmanager::PositionDiff diff;
    };

    enum State
    {
        DRIVE = 0,
        CHANGE_LANE,
    };

    enum VoIType  // VehicleOfInterestType
    {
        LEAD = 0,
        FOLLOWING,
        LEFT_LEAD,
        LEFT_FOLLOW,
        RIGHT_LEAD,
        RIGHT_FOLLOW,
    };

    class ControllerNaturalDriver : public Controller
    {
    public:
        ControllerNaturalDriver(InitArgs* args);

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_NATURAL_DRIVER_TYPE_NAME;
        }
        const char* GetTypeName() override
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return CONTROLLER_TYPE_NATURAL_DRIVER;
        }
        int GetType() override
        {
            return GetTypeStatic();
        }

        void Init();
        void InitPostPlayer();
        void Step(double dt);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);

        bool   AdjacentLanesAvailable();
        void   FilterSurroundingVehicles();
        void   UpdateSurroundingVehicles();
        void   FindClosestAhead(scenarioengine::Object* object, roadmanager::PositionDiff& diff, VoIType type);
        void   FindClosestBehind(scenarioengine::Object* object, roadmanager::PositionDiff& diff, VoIType type);
        bool   CheckLaneChangePossible(const int lane_id);
        bool   AbortLaneChange() const;
        double EstimateFreespace(const scenarioengine::Object* follow, const scenarioengine::Object* target, const double ds) const;

        ControllerNaturalDriver* GetOtherDriver(scenarioengine::Object* object) const;
        void                     GetVehicleOfInterestType(int lane_id, VoIType& lead, VoIType& follow);
        double                   GetAcceleration(scenarioengine::Object* follow, scenarioengine::Object* lead) const;
        double                   GetDesiredGap(double max_acceleration,
                                               double max_deceleration,
                                               double follow_speed,
                                               double lead_speed,
                                               double desired_distance,
                                               double desired_thw) const;

        void ReportKeyEvent(int key, bool down);

    private:
        bool                             active_;
        double                           desired_distance_;
        double                           desired_speed_;
        double                           current_speed_;
        double                           lane_change_duration_;
        double                           lookahead_dist_;
        double                           max_deceleration_;
        double                           max_acceleration_;
        std::array<int, 2>               lane_ids_available_;
        std::vector<Object*>             vehicles_in_radius_;
        std::unordered_map<VoIType, VoI> vehicles_of_interest_;
        bool                             lane_change_injected;
        State                            state_;
        double                           lane_change_delay_;
        double                           lane_change_cooldown_;
        int                              target_lane_;
        double                           desired_thw_;
        double                           max_imposed_braking_;
        double                           politeness_;
        double                           lane_change_acc_gain_;
        int                              route_;
        bool                             initiate_lanechange_;
    };

    Controller* InstantiateNaturalDriver(void* args);
}  // namespace scenarioengine