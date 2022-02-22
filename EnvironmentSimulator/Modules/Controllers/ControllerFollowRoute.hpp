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

#define CONTROLLER_FOLLOW_ROUTE_TYPE_NAME "FollowRouteController"

namespace scenarioengine
{
	class ScenarioPlayer;
	class ScenarioEngine;

	typedef enum {
		SHORTEST,
		FASTEST,
		MIN_INTERSECTIONS
	} RouteStrategy;

	// base class for controllers
	class ControllerFollowRoute: public Controller
	{
	public:
		ControllerFollowRoute(InitArgs *args);

		static const char* GetTypeNameStatic() { return CONTROLLER_FOLLOW_ROUTE_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_TYPE_FOLLOW_ROUTE; }
		virtual int GetType() { return GetTypeStatic(); }

		void Init();
		void Step(double timeStep);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);
		void SetScenarioEngine(ScenarioEngine* scenarioEngine) { scenarioEngine_ = scenarioEngine; };

	private:
		std::vector<roadmanager::RoadPath::PathNode*> CalculatePath(roadmanager::Position* targetWaypoint, RouteStrategy routeStrategy);
		void ChangeLane(int lane,  double time);
		bool TargetLaneIsInDrivingDirection(roadmanager::RoadPath::PathNode* pNode, roadmanager::Road *nextRoad, int targetLaneId);
		roadmanager::RoadPath::PathNode* GetNextNode(roadmanager::Road* nextRoad, roadmanager::RoadPath::PathNode* srcNode);
		// Sign function: Return 1 if positive, 0 if zero, -1 if negative
		template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }
		vehicle::Vehicle vehicle_;
		ScenarioEngine* scenarioEngine_;
		std::vector<OSCPrivateAction*> actions_;

	};

	Controller* InstantiateControllerFollowRoute(void* args);
}