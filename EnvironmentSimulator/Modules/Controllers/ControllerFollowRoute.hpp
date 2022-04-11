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

#define CONTROLLER_FOLLOW_ROUTE_TYPE_NAME "FollowRouteController"

namespace scenarioengine
{
	class ScenarioPlayer;
	class ScenarioEngine;

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

		static const char *GetTypeNameStatic() { return CONTROLLER_FOLLOW_ROUTE_TYPE_NAME; }
		virtual const char *GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_TYPE_FOLLOW_ROUTE; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double timeStep);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);
		void SetScenarioEngine(ScenarioEngine *scenarioEngine) { scenarioEngine_ = scenarioEngine; };

	private:
		void ChangeLane(int lane, double time);
		void CalculateWaypoints();
		bool CanChangeLane(int lane);
		double DistanceBetween(roadmanager::Position p1, roadmanager::Position p2);
		WaypointStatus GetWaypointStatus(roadmanager::Position vehiclePos, roadmanager::Position waypoint);
		ScenarioEngine *scenarioEngine_;
		vehicle::Vehicle vehicle_;
		std::vector<OSCPrivateAction *> actions_;
		roadmanager::OpenDrive *odr_;
		std::vector<roadmanager::Position> waypoints_;
		int currentWaypointIndex_;
		bool changingLane_;
		bool pathCalculated_;
		std::vector<roadmanager::Position> allWaypoints_;
		const double MIN_DIST_TO_WAYPOINT_LANE_CHANGE = 25;
		const double MIN_DIST_FOR_COLLISION = 20;
		bool temp = false;
	};

	Controller *InstantiateControllerFollowRoute(void *args);
}