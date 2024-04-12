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

#include <string>
#include "Controller.hpp"
#include "pugixml.hpp"
#include "Parameters.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"
#include <queue>

// Enable test mode, which stops the vehicle when reaching a target
// or in case of path not found

#define CONTROLLER_FOLLOW_ROUTE_TYPE_NAME "FollowRouteController"

namespace scenarioengine
{
    typedef enum
    {
        MISSED_WAYPOINT,
        PASSED_WAYPOINT,
        WAYPOINT_NOT_REACHED
    } WaypointStatus;

    // base class for controllers
    class ControllerFollowRoute : public Controller
    {
    public:
        ControllerFollowRoute(InitArgs *args);

        static const char *GetTypeNameStatic()
        {
            return CONTROLLER_FOLLOW_ROUTE_TYPE_NAME;
        }
        virtual const char *GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return CONTROLLER_TYPE_FOLLOW_ROUTE;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
        }

        void Init();
        void Step(double timeStep);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);
        void ReportKeyEvent(int key, bool down);
        void SetScenarioEngine(ScenarioEngine *scenarioEngine)
        {
            scenarioEngine_ = scenarioEngine;
        };

        /**
         * @brief Get the time for a lane change
         *
         * @return double, (s)
         */
        double GetLaneChangeTime()
        {
            return laneChangeTime_;
        };
        /**
         * @brief Get the Minimum distance for collision detection
         *
         * @return double, (m)
         */
        double GetMinDistForCollision()
        {
            return minDistForCollision_;
        };

    private:
        /**
         * @brief Create a lateral lane change action
         *
         * @param lane the lane id of the desired lane
         */
        void CreateLaneChange(int lane);
        /**
         * @brief Runs the pathfinder and the waypoint creator for the current scenariowaypoint, and checks if a path has been found.
         *
         */
        void CalculateWaypoints();
        /**
         * @brief Check if a lane change is allowed or not.
         * Checking: if lanechange is ongoing, if lane exists, or collision risk
         *
         * @param lane the desired lane id
         * @return true (lane change is allowed) or false (lane change ongoing, lane not exists, or risk for collision)
         */
        bool CanChangeLane(int lane);
        /**
         * @brief Performs the lateral lane change action if defined
         *
         * @param timeStep the simulation time step
         *
         */
        void ChangeLane(double timeStep);
        /**
         * @brief Checks the waypoint status: update waypoints based on the status.
         *
         * @param vehiclePos positon of the vehicle
         * @param nextWaypoint the next waypoint
         */
        void UpdateWaypoints(roadmanager::Position vehiclePos, roadmanager::Position nextWaypoint);
        /**
         * @brief Return the distance between two positions
         *
         * @param p1
         * @param p2
         * @return double, (m)
         */
        double DistanceBetween(roadmanager::Position p1, roadmanager::Position p2);
        /**
         * @brief Wrapper for controller::deactivate, to include testMode
         *
         */
        void Deactivate() override;

        /**
         * @brief Get the Waypoint Status, checks if entity has missed, reached, or passed waypoint.
         *
         * @param vehiclePos position of entity
         * @param waypoint current waypoint
         * @return WaypointStatus
         */
        WaypointStatus GetWaypointStatus(roadmanager::Position vehiclePos, roadmanager::Position waypoint);

        ScenarioEngine                    *scenarioEngine_;
        vehicle::Vehicle                   vehicle_;
        OSCPrivateAction                  *laneChangeAction_;
        roadmanager::OpenDrive            *odr_;
        std::vector<roadmanager::Position> waypoints_;
        int                                currentWaypointIndex_;
        int                                scenarioWaypointIndex_;
        bool                               changingLane_;
        bool                               pathCalculated_;
        std::vector<roadmanager::Position> allWaypoints_;
        double                             laneChangeTime_      = 5;
        double                             minDistForCollision_ = 10;
        double                             minLaneWidth_        = 0.5;
        bool                               testMode_;
    };

    Controller *InstantiateControllerFollowRoute(void *args);
}  // namespace scenarioengine