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
 * This controller simulates a bad or dizzy driver by manipulating
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

//#include "playerbase.hpp"
#include <algorithm>
#include "ControllerFollowRoute.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "OSCManeuver.hpp"
#include "ScenarioEngine.hpp"
#include "LaneIndependentRouter.hpp"

using namespace scenarioengine;

Controller *scenarioengine::InstantiateControllerFollowRoute(void *args)
{
	Controller::InitArgs *initArgs = (Controller::InitArgs *)args;

	return new ControllerFollowRoute(initArgs);
}

ControllerFollowRoute::ControllerFollowRoute(InitArgs *args) : Controller(args)
{
}

void ControllerFollowRoute::Init()
{
	LOG("FollowRoute init");

	Controller::Init();
}

void ControllerFollowRoute::Step(double timeStep)
{
	if (object_->pos_.GetRoute() == nullptr)
	{
		LOG("Route = nullptr");
		Controller::Step(timeStep);
		return;
	}

	if (!pathCalculated_)
	{
		CalculateWaypoints();
	}

	// Check if all waypoints have been passed
	if (currentWaypointIndex_ >= waypoints_.size())
	{
		object_->SetSpeed(0);
		Controller::Step(timeStep);
		return;
	}

	roadmanager::Position vehiclePos = object_->pos_;
	roadmanager::Position nextWaypoint = waypoints_[currentWaypointIndex_];
	roadmanager::Road *nextRoad = odr_->GetRoadById(nextWaypoint.GetTrackId());

	bool drivingWithRoadDirection = vehiclePos.GetDrivingDirectionRelativeRoad() == 1;
	bool sameRoad = nextWaypoint.GetTrackId() == vehiclePos.GetTrackId();
	bool sameLane = nextWaypoint.GetLaneId() == vehiclePos.GetLaneId();
	if (sameRoad)
	{
		bool nearSPos = abs(vehiclePos.GetS() - nextWaypoint.GetS()) < MIN_DIST_TO_WAYPOINT_LANE_CHANGE;
		if (!sameLane && nearSPos && CanChangeLane(nextWaypoint.GetLaneId()))
		{
			CreateLaneChange(nextWaypoint.GetLaneId(), 1);
			changingLane_ = true;
		}
	}

	UpdateWaypoints(vehiclePos, nextWaypoint);
	ChangeLane(timeStep);

	Controller::Step(timeStep);
}

void ControllerFollowRoute::Activate(ControlDomains domainMask)
{
	LOG("FollowRoute activate");

	this->mode_ = Controller::Mode::MODE_ADDITIVE;
	if (object_ != nullptr)
	{
		odr_ = object_->pos_.GetOpenDrive();
	}
	currentWaypointIndex_ = 0;
	scenarioWaypointIndex_ = 0;
	pathCalculated_ = false;
	changingLane_ = false;
	waypoints_ = {};
	laneChangeAction_ = nullptr;
	Controller::Activate(domainMask);
}

void ControllerFollowRoute::ReportKeyEvent(int key, bool down)
{
}

void ControllerFollowRoute::UpdateWaypoints(roadmanager::Position vehiclePos, roadmanager::Position nextWaypoint)
{
	WaypointStatus waypointStatus = GetWaypointStatus(vehiclePos, nextWaypoint);
	switch (waypointStatus)
	{
	case PASSED_WAYPOINT:
	{
		LOG("Passed waypoint: r=%d, l=%d, s=%f", nextWaypoint.GetTrackId(), nextWaypoint.GetLaneId(), nextWaypoint.GetS());
		currentWaypointIndex_++;
		if (nextWaypoint.GetTrackId() == waypoints_.back().GetTrackId())
		{
			bool scenarioWaypointsLeft = scenarioWaypointIndex_ < object_->pos_.GetRoute()->scenario_waypoints_.size() - 1;
			if (scenarioWaypointsLeft)
			{
				scenarioWaypointIndex_++;
				pathCalculated_ = false;
				CalculateWaypoints();
				currentWaypointIndex_ = 0;
			}
			return;
		}

		object_->pos_.GetRoute()->minimal_waypoints_.clear();
		object_->pos_.GetRoute()->minimal_waypoints_ = {vehiclePos, waypoints_[currentWaypointIndex_]};
		object_->pos_.CalcRoutePosition(); // Reset route object according to new route waypoints and current position

		return;
	}
	case MISSED_WAYPOINT:
		LOG("Missed waypoint: r=%d, l=%d, s=%f", nextWaypoint.GetTrackId(), nextWaypoint.GetLaneId(), nextWaypoint.GetS());
		if (object_->pos_.GetRoute() != nullptr)
		{
			pathCalculated_ = false;
			CalculateWaypoints();
			currentWaypointIndex_ = 0;
		}
		return;
	case WAYPOINT_NOT_REACHED:
		return;
	}
}

void ControllerFollowRoute::CalculateWaypoints()
{
	roadmanager::LaneIndependentRouter router(odr_);

	roadmanager::Position startPos = object_->pos_;
	roadmanager::Position targetPos = object_->pos_.GetRoute()->scenario_waypoints_[scenarioWaypointIndex_];

	// If start and target is on same road, set next waypoint as target
	if (startPos.GetTrackId() == targetPos.GetTrackId())
	{
		scenarioWaypointIndex_++;
		if (scenarioWaypointIndex_ >= object_->pos_.GetRoute()->scenario_waypoints_.size())
		{
			LOG("Error: start and target on same road, scenarioWaypointIndex out of bounds, deactivating controller");
			object_->SetSpeed(0);
			Controller::Deactivate();
			return;
		}
		targetPos = object_->pos_.GetRoute()->scenario_waypoints_[scenarioWaypointIndex_];
	}

	std::vector<roadmanager::Node *> pathToGoal = router.CalculatePath(startPos, targetPos);
	LOG("Path calculated");
	if (pathToGoal.empty())
	{
		LOG("Error: Path not found, deactivating controller");
		object_->SetSpeed(0);
		Controller::Deactivate();
	}
	else
	{
		LOG("Path found");
		waypoints_ = router.GetWaypoints(pathToGoal, startPos, targetPos);

		object_->pos_.GetRoute()->minimal_waypoints_.clear();
		object_->pos_.GetRoute()->minimal_waypoints_ = {waypoints_[0], waypoints_[1]};
		object_->pos_.CalcRoutePosition(); // Reset route object according to new route waypoints and current position
		pathCalculated_ = true;
	}
}

void ControllerFollowRoute::CreateLaneChange(int lane, double time)
{
	LatLaneChangeAction *action_lanechange = new LatLaneChangeAction;
	action_lanechange->name_ = "LaneChange";
	action_lanechange->object_ = object_;
	action_lanechange->transition_.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
	action_lanechange->transition_.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
	action_lanechange->transition_.SetParamTargetVal(time);
	action_lanechange->max_num_executions_ = 1;

	LatLaneChangeAction::TargetAbsolute *target = new LatLaneChangeAction::TargetAbsolute;
	target->value_ = lane;
	action_lanechange->target_ = target;

	// Set lane change to be performed by ChangeLane function
	laneChangeAction_ = action_lanechange;
}

void ControllerFollowRoute::ChangeLane(double timeStep)
{
	if (laneChangeAction_ == nullptr)
	{
		return;
	}

	if (laneChangeAction_->state_ == OSCAction::State::COMPLETE)
	{
		changingLane_ = false;
		return;
	}

	if (!laneChangeAction_->IsActive())
	{
		laneChangeAction_->Start(scenarioEngine_->getSimulationTime(), timeStep);
	}
	else
	{
		laneChangeAction_->Step(scenarioEngine_->getSimulationTime(), timeStep);
		if (laneChangeAction_->state_ != OSCAction::State::COMPLETE)
		{
			laneChangeAction_->UpdateState();
		}
		// Fetch updated position after lane change step
		vehicle_.posX_ = object_->pos_.GetX();
		vehicle_.posY_ = object_->pos_.GetY();
		vehicle_.heading_ = object_->pos_.GetH();

		gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
	}
}

bool ControllerFollowRoute::CanChangeLane(int lane)
{
	roadmanager::Position vehiclePos = object_->pos_;
	std::vector<Object *> allVehicles = scenarioEngine_->entities_.object_;
	if (changingLane_)
	{
		return false;
	}

	for (Object *otherVehicle : allVehicles)
	{
		bool sameRoad = otherVehicle->pos_.GetTrackId() == vehiclePos.GetTrackId();
		bool sameLane = otherVehicle->pos_.GetLaneId() == vehiclePos.GetLaneId();
		if (otherVehicle == object_ || (sameRoad && sameLane))
		{
			continue;
		}
		bool collisionRisk = DistanceBetween(vehiclePos, otherVehicle->pos_) < MIN_DIST_FOR_COLLISION;
		bool sameSideOfRoad = SIGN(vehiclePos.GetLaneId()) == SIGN(otherVehicle->pos_.GetLaneId());

		if (!sameLane && collisionRisk && sameSideOfRoad)
		{
			int n = vehiclePos.GetLaneId();
			auto inc = [&n]
			{ return ++n; };
			auto dec = [&n]
			{ return --n; };

			int lanesBetween = abs(vehiclePos.GetLaneId() - otherVehicle->pos_.GetLaneId());
			bool lanesIncreasing = otherVehicle->pos_.GetLaneId() > vehiclePos.GetLaneId();

			std::vector<int> laneIdsToCheck(lanesBetween);
			if (lanesIncreasing)
			{
				std::generate(laneIdsToCheck.begin(), laneIdsToCheck.end(), inc);
			}
			else
			{
				std::generate(laneIdsToCheck.begin(), laneIdsToCheck.end(), dec);
			}

			bool collides = std::find(laneIdsToCheck.begin(), laneIdsToCheck.end(), otherVehicle->pos_.GetLaneId()) != laneIdsToCheck.end();
			if (collides)
			{
				return false;
			}
		}
	}
	return true;
}

double ControllerFollowRoute::DistanceBetween(roadmanager::Position p1, roadmanager::Position p2)
{
	double x1 = p1.GetX();
	double y1 = p1.GetY();
	double x2 = p2.GetX();
	double y2 = p2.GetY();

	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

WaypointStatus ControllerFollowRoute::GetWaypointStatus(roadmanager::Position vehiclePos, roadmanager::Position waypoint)
{
	using namespace roadmanager;
	bool drivingWithRoadDirection = vehiclePos.GetDrivingDirectionRelativeRoad() == 1;
	bool sameRoad = waypoint.GetTrackId() == vehiclePos.GetTrackId();
	bool sameLane = waypoint.GetLaneId() == vehiclePos.GetLaneId();

	if (sameRoad && ((drivingWithRoadDirection && vehiclePos.GetS() > waypoint.GetS()) ||
					 (!drivingWithRoadDirection && vehiclePos.GetS() < waypoint.GetS())))
	{
		return sameLane ? PASSED_WAYPOINT : MISSED_WAYPOINT;
	}
	if (sameRoad)
	{
		return WAYPOINT_NOT_REACHED;
	}
	Road *currentRoad = odr_->GetRoadById(vehiclePos.GetTrackId());
	std::vector<Road *> possiblePreviousRoads;
	RoadLink *link;
	if (drivingWithRoadDirection)
	{
		link = currentRoad->GetLink(LinkType::PREDECESSOR);
	}
	else
	{
		link = currentRoad->GetLink(LinkType::SUCCESSOR);
	}

	if (link == nullptr)
	{
		return WAYPOINT_NOT_REACHED;
	}

	if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		possiblePreviousRoads.push_back(odr_->GetRoadById(link->GetElementId()));
	}
	else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		Junction *junction = odr_->GetJunctionById(link->GetElementId());
		for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(currentRoad->GetId()); j++)
		{
			int roadId = junction->GetConnectingRoadIdFromIncomingRoadId(currentRoad->GetId(), (int)j);
			possiblePreviousRoads.push_back(odr_->GetRoadById(roadId));
		}
	}

	for (Road *previousRoad : possiblePreviousRoads)
	{
		if (previousRoad->GetId() == waypoint.GetTrackId())
		{
			int previousLaneId = currentRoad->GetConnectingLaneId(link, vehiclePos.GetLaneId(), previousRoad->GetId());
			return previousLaneId == waypoint.GetLaneId() ? PASSED_WAYPOINT : MISSED_WAYPOINT;
		}
	}

	return WAYPOINT_NOT_REACHED;
}
