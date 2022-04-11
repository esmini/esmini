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
	if (pathCalculated_ && currentWaypointIndex_ >= waypoints_.size())
	{
		object_->SetSpeed(0);
		Controller::Step(timeStep);
		return;
	}
	if (object_->pos_.GetRoute() != nullptr && !temp)
	{
		allWaypoints_ = object_->pos_.GetRoute()->all_waypoints_;
		LOG("MINIMAL WAYPOINT LIST:");
		for (roadmanager::Position p : object_->pos_.GetRoute()->minimal_waypoints_)
		{
			LOG("r=%d, l=%d, s=%f", p.GetTrackId(), p.GetLaneId(), p.GetS());
		}
		temp = true;
	}

	if (!pathCalculated_ && temp)
	{
		CalculateWaypoints();
	}

	// LOG("current pos: r=%d, l=%d, s=%f, cwi=%d, cwis=%d", object_->pos_.GetTrackId(), object_->pos_.GetLaneId(), object_->pos_.GetS(), currentWaypointIndex_, waypoints_.size());
	if (pathCalculated_ && !changingLane_)
	{
		roadmanager::Position vehiclePos = object_->pos_;
		bool drivingWithRoadDirection = vehiclePos.GetDrivingDirectionRelativeRoad() == 1;
		roadmanager::Position nextWaypoint = waypoints_[currentWaypointIndex_];
		roadmanager::Road *nextRoad = odr_->GetRoadById(nextWaypoint.GetTrackId());

		// LOG("nextWaypoint: r=%d, l=%d, s=%f", nextWaypoint.GetTrackId(), nextWaypoint.GetLaneId(), nextWaypoint.GetS());
		//  LOG("vehiclePos: r=%d, l=%d, s=%f", vehiclePos.GetTrackId(), vehiclePos.GetLaneId(), vehiclePos.GetS());

		bool sameRoad = nextWaypoint.GetTrackId() == vehiclePos.GetTrackId();
		bool sameLane = nextWaypoint.GetLaneId() == vehiclePos.GetLaneId();
		if (sameRoad)
		{
			bool nearSPos = abs(vehiclePos.GetS() - nextWaypoint.GetS()) < MIN_DIST_TO_WAYPOINT_LANE_CHANGE;
			if (!sameLane && nearSPos && CanChangeLane(nextWaypoint.GetLaneId()) && !changingLane_)
			{
				ChangeLane(nextWaypoint.GetLaneId(), 1);
				changingLane_ = true;
			}
		}

		WaypointStatus waypointStatus = GetWaypointStatus(vehiclePos, nextWaypoint);
		switch (waypointStatus)
		{
		case PASSED_WAYPOINT:
		{
			LOG("Passed waypoint:");
			currentWaypointIndex_++;
			LOG("currentWaypointIndex_: %d", currentWaypointIndex_);
			if (nextWaypoint.GetTrackId() == waypoints_.back().GetTrackId())
			{
				break;
			}
			LOG("Previous waypoint, r: %d l: %d s: %f", waypoints_[currentWaypointIndex_ - 1].GetTrackId(), waypoints_[currentWaypointIndex_ - 1].GetLaneId(), waypoints_[currentWaypointIndex_ - 1].GetS());
			LOG("Next waypoint, r: %d l: %d s: %f", waypoints_[currentWaypointIndex_].GetTrackId(), waypoints_[currentWaypointIndex_].GetLaneId(), waypoints_[currentWaypointIndex_].GetS());
			object_->pos_.GetRoute()->minimal_waypoints_.clear();
			object_->pos_.GetRoute()->minimal_waypoints_ = {vehiclePos, waypoints_[currentWaypointIndex_]};
			LOG("waypoint_idx: %d", object_->pos_.GetRoute()->waypoint_idx_);
			LOG("Updated minimal waypoints");
			LOG("MINIMAL WAYPOINT LIST:");
			for (roadmanager::Position p : object_->pos_.GetRoute()->minimal_waypoints_)
			{
				LOG("r=%d, l=%d, s=%f", p.GetTrackId(), p.GetLaneId(), p.GetS());
			}
			break;
		}
		case MISSED_WAYPOINT:
			LOG("Missed waypoint");
			if (object_->pos_.GetRoute() != nullptr)
			{
				LOG("Calculating new path...");
				pathCalculated_ = false;
				CalculateWaypoints();
				currentWaypointIndex_ = 0;
			}
			break;
		case WAYPOINT_NOT_REACHED:
			break;
		}
	}

	for (size_t i = 0; i < actions_.size(); i++)
	{
		OSCPrivateAction *action = actions_[i];
		if (action->name_ != "LaneChange")
		{
			continue;
		}

		if (!action->IsActive())
		{
			action->Start(scenarioEngine_->getSimulationTime(), timeStep);
		}
		else if (action->IsActive())
		{
			action->Step(scenarioEngine_->getSimulationTime(), timeStep);
			if (action->state_ != OSCAction::State::COMPLETE)
			{
				action->UpdateState();
			}
			// Fetch updated position
			vehicle_.posX_ = object_->pos_.GetX();
			vehicle_.posY_ = object_->pos_.GetY();
			vehicle_.heading_ = object_->pos_.GetH();

			gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
		}

		if (action->state_ == OSCAction::State::COMPLETE)
		{
			actions_.erase(actions_.begin() + i);
			changingLane_ = false;
		}
	}

	// double steplen = object_->GetSpeed()*timeStep;
	// object_->MoveAlongS(steplen);

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
	pathCalculated_ = false;
	changingLane_ = false;
	waypoints_ = {};
	Controller::Activate(domainMask);
}

void ControllerFollowRoute::ReportKeyEvent(int key, bool down)
{
}

void ControllerFollowRoute::CalculateWaypoints()
{
	roadmanager::Position startPos = object_->pos_;
	roadmanager::Position targetPos = allWaypoints_.back();
	roadmanager::LaneIndependentRouter router(odr_);

	std::vector<roadmanager::Node *> pathToGoal = router.CalculatePath(startPos, targetPos, roadmanager::RouteStrategy::SHORTEST);
	LOG("Path calculated");
	if (pathToGoal.empty())
	{
		LOG("Path not found");
		Controller::Deactivate();
	}
	else
	{
		LOG("Path found");
		waypoints_ = router.GetWaypoints(pathToGoal, startPos, targetPos);
		object_->pos_.GetRoute()->minimal_waypoints_.clear();
		object_->pos_.GetRoute()->minimal_waypoints_ = {waypoints_[0], waypoints_[1]};

		LOG("Waypoints created");
		for (roadmanager::Position p : waypoints_)
		{
			LOG("r=%d, l=%d, s=%f", p.GetTrackId(), p.GetLaneId(), p.GetS());
		}
		LOG("MINIMAL WAYPOINT LIST:");
		for (roadmanager::Position p : object_->pos_.GetRoute()->minimal_waypoints_)
		{
			LOG("r=%d, l=%d, s=%f", p.GetTrackId(), p.GetLaneId(), p.GetS());
		}
		pathCalculated_ = true;
	}
}

void ControllerFollowRoute::ChangeLane(int lane, double time)
{
	LOG("Add changing lane event to lane %d", lane);
	LatLaneChangeAction *action_lanechange = new LatLaneChangeAction();
	action_lanechange->name_ = "LaneChange";
	action_lanechange->object_ = object_;
	action_lanechange->transition_.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
	action_lanechange->transition_.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
	action_lanechange->transition_.SetParamTargetVal(time);
	action_lanechange->max_num_executions_ = 1;

	LatLaneChangeAction::TargetAbsolute *test = new LatLaneChangeAction::TargetAbsolute;
	test->value_ = lane;
	action_lanechange->target_ = test;
	actions_.push_back(action_lanechange);
}

bool ControllerFollowRoute::CanChangeLane(int lane)
{
	roadmanager::Position vehiclePos = object_->pos_;
	std::vector<Object *> allVehicles = scenarioEngine_->entities_.object_;
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
