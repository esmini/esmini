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
		SHORTEST,
		FASTEST,
		MIN_INTERSECTIONS
	} RouteStrategy;

	typedef struct Node
	{
		roadmanager::Road *road;
		int laneId;
		double weight;
		roadmanager::RoadLink *link;
		Node *previous;
	} Node;

	struct WeightCompare
	{
	public:
		bool operator()(Node *a, Node *b) // overloading both operators
		{
			return a->weight > b->weight;
		}
	};

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
		std::vector<Node *> CalculatePath(RouteStrategy routeStrategy);
		void ChangeLane(int lane, double time);
		Node *CreateTargetNode(Node *currentNode, roadmanager::Road *nextRoad, RouteStrategy routeStrategy);
		void UpdateDistanceVector(std::vector<Node *> nextNodes);
		bool TargetLaneIsInDrivingDirection(Node *pNode, roadmanager::Road *nextRoad);
		std::vector<Node *> GetNextNodes(roadmanager::Road *nextRoad,roadmanager::Road *targetRoad, Node *srcNode, RouteStrategy routeStrategy);
		std::vector<int> GetConnectingLanes(Node *srcNode, roadmanager::Road *nextRoad);
		bool FindGoal(roadmanager::OpenDrive *odr, RouteStrategy routeStrategy);
		double CalcAverageSpeed(roadmanager::Road *road);
		double CalcWeight(RouteStrategy routeStrategy, double roadLength,roadmanager::Road *road);
		double CalcWeightWithPos(roadmanager::ContactPointType contactPointType, roadmanager::Position pos, roadmanager::Road *road,RouteStrategy routeStrategy);
		bool IsTargetValid(roadmanager::OpenDrive *odr);
		template <class Q>
		void clearQueue(Q &q) { q = Q(); }

		vehicle::Vehicle vehicle_;
		ScenarioEngine *scenarioEngine_;
		std::vector<OSCPrivateAction *> actions_;
		std::priority_queue<Node *, std::vector<Node *>, WeightCompare> unvisited_;
		std::vector<Node *> visited_;
		std::vector<Node *> distance_;
		roadmanager::Position targetWaypoint_;
	};

	Controller *InstantiateControllerFollowRoute(void *args);
}