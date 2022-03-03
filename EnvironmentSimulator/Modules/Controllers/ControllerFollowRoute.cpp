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
#include "ControllerFollowRoute.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "OSCManeuver.hpp"
#include "ScenarioEngine.hpp"

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

double testtime = 0;
bool pathCalculated = false;
void ControllerFollowRoute::Step(double timeStep)
{
	// LOG("FollowRoute step");
	// object_->MoveAlongS(timeStep * object_->GetSpeed());
	roadmanager::Route *test = nullptr;
	if (object_->pos_.GetRoute() != nullptr)
	{
		if (!pathCalculated)
		{
			// roadmanager::Position targetWaypoint(5,-1,15,0);
			targetWaypoint_ = object_->pos_.GetRoute()->all_waypoints_.back();
			targetWaypoint_.Print();

			std::vector<Node *> pathToGoalS = CalculatePath(RouteStrategy::SHORTEST);
			LOG("Path to goal (SHORTEST) size: %d", pathToGoalS.size());
			for (Node *node : pathToGoalS)
			{
				LOG("%d", node->road->GetId());
			}

			// std::vector<Node *> pathToGoalF = CalculatePath(RouteStrategy::FASTEST);
			// LOG("Path to goal (FASTEST) size: %d", pathToGoalF.size());
			// for (Node *node : pathToGoalF)
			// {
			// 	LOG("%d", node->road->GetId());
			// }
			pathCalculated = true;
		}

		// test = object_->pos_.GetRoute();
		// if (test->GetWaypoint(-1)->GetLaneId() != object_->pos_.GetLaneId() &&
		// test->GetWaypoint(-1)->GetTrackId() == object_->pos_.GetTrackId())
		// {
		// 	int laneid = test->GetWaypoint(-1)->GetLaneId();
		// 	LOG("ADD ACTION");
		// 	ChangeLane(laneid, 3);
		// }
	}
	testtime += timeStep;
	if (testtime > 1)
	{
		LOG("Nr of actions: %d", actions_.size());
		testtime = 0;
	}
	for (size_t i = 0; i < actions_.size(); i++)
	{
		OSCPrivateAction *action = actions_[i];
		if (!action->IsActive())
		{
			LOG("ACTION START");
			action->Start(scenarioEngine_->getSimulationTime(), timeStep);
		}
		if (action->IsActive())
		{
			// LOG("ACTION STEP");
			action->Step(scenarioEngine_->getSimulationTime(), timeStep);
			if (action->state_ != OSCAction::State::COMPLETE)
			{
				action->UpdateState();
			}
		}
		if (action->state_ == OSCAction::State::COMPLETE)
		{
			LOG("ACTION COMPLETED");
			LOG("actions before end: %d", actions_.size());
			// action->End();
			actions_.erase(actions_.begin() + i);
			LOG("actions after erase: %d", actions_.size());
		}
	}

	Controller::Step(timeStep);
}

void ControllerFollowRoute::Activate(ControlDomains domainMask)
{
	LOG("FollowRoute activate");

	this->mode_ = Controller::Mode::MODE_ADDITIVE;
	// Trigger* trigger = new Trigger(0);
	// ConditionGroup* conGroup = new ConditionGroup();
	// TrigBySimulationTime* condition = new TrigBySimulationTime();
	// condition->value_ = 2;

	// conGroup->condition_.push_back(condition);
	// trigger->conditionGroup_.push_back(conGroup);
	// event_lanechange->start_trigger_ = trigger;

	// Grab and inspect road network
	// roadmanager::OpenDrive *odr = nullptr;
	// if (object_ != nullptr)
	// {
	// 	odr = object_->pos_.GetOpenDrive();
	// }

	// if (odr != nullptr)
	// {
	// 	for (int i = 0; i < odr->GetNumOfRoads(); i++)
	// 	{
	// 		roadmanager::Road *road = odr->GetRoadByIdx(i);
	// 		LOG("road[%d] id: %d length: %.2f", i, road->GetId(), road->GetLength());
	// 	}
	// }

	Controller::Activate(domainMask);
}

void ControllerFollowRoute::ChangeLane(int lane, double time)
{
	LatLaneChangeAction *action_lanechange = new LatLaneChangeAction();
	action_lanechange->object_ = object_;
	action_lanechange->transition_.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
	action_lanechange->transition_.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
	action_lanechange->transition_.SetParamTargetVal(time);
	action_lanechange->max_num_executions_ = 1;

	LatLaneChangeAction::TargetAbsolute *test = new LatLaneChangeAction::TargetAbsolute;
	test->value_ = lane;
	action_lanechange->target_ = test;
	actions_.push_back(action_lanechange);

	// Event* event_lanechange = new Event();
	// event_lanechange->action_.push_back(action_lanechange);
	// event_lanechange->priority_ = Event::Priority::OVERWRITE;
	// event_lanechange->name_="HelperLaneChange";
	// event_lanechange->max_num_executions_ = 1;
	// object_->addEvent(event_lanechange);
}

void ControllerFollowRoute::ReportKeyEvent(int key, bool down)
{
}

// Gets the next pathnode for the nextroad based on current srcnode
std::vector<Node *> ControllerFollowRoute::GetNextNodes(roadmanager::Road *nextRoad, roadmanager::Road *targetRoad, Node *currentNode, RouteStrategy routeStrategy)
{

	// Register length of this road and find node in other end of the road (link)
	roadmanager::RoadLink *nextLink = 0;

	if (currentNode->link->GetElementType() == roadmanager::RoadLink::ELEMENT_TYPE_ROAD)
	{
		// node link is a road, find link in the other end of it
		if (currentNode->link->GetContactPointType() == roadmanager::ContactPointType::CONTACT_POINT_END)
		{
			nextLink = nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR);
		}
		else
		{
			nextLink = nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR);
		}
	}
	else if (currentNode->link->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		roadmanager::Junction *junction = roadmanager::Position::GetOpenDrive()->GetJunctionById(currentNode->link->GetElementId());
		int elementId;
		if (junction && junction->GetType() == roadmanager::Junction::JunctionType::DIRECT)
		{
			elementId = junction->GetId();
		}
		else
		{
			// Default junction
			elementId = currentNode->road->GetId();
		}

		if (nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR) &&
			nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == elementId)
		{

			nextLink = nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR);
		}
		else if (nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR) &&
				 nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR)->GetElementId() == elementId)
		{
			nextLink = nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR);
		}
	}

	if (nextLink == 0)
	{
		// end of road
		return {};
	}
	std::vector<int> connectingLaneIds = GetConnectingLanes(currentNode, nextRoad);
	if (connectingLaneIds.empty())
	{
		// no existing lanes
		return {};
	}

	std::vector<Node *> nextNodes;
	for (int nextLaneId : connectingLaneIds)
	{
		Node *pNode = new Node;
		int targetLaneId = targetWaypoint_.GetLaneId();
		bool TargetWaypointInDrivingDirection = SIGN(nextLaneId) == SIGN(targetLaneId);
		if (nextRoad == targetRoad && TargetWaypointInDrivingDirection)
		{
			// Target road found and driving in same direction, create a target node.
			pNode = CreateTargetNode(currentNode, nextRoad, routeStrategy);
		}
		else
		{
			// create next non target node
			pNode->link = nextLink;
			pNode->road = nextRoad;
			pNode->laneId = nextLaneId;
			pNode->previous = currentNode;
			//FIX TARGETWAYPOINT_ ....
			double nextWeight = CalcWeight(routeStrategy,nextRoad->GetLength(),nextRoad);
			pNode->weight = currentNode->weight + nextWeight;
		}
		if (pNode)
		{
			nextNodes.push_back(pNode);
		}
	}

	return nextNodes;
}

std::vector<int> ControllerFollowRoute::GetConnectingLanes(Node *srcNode, roadmanager::Road *nextRoad)
{
	roadmanager::LaneSection *lanesection = nullptr;
	if (srcNode->link->GetType() == roadmanager::LinkType::SUCCESSOR)
	{
		int nrOfLanesection = srcNode->road->GetNumberOfLaneSections();
		lanesection = srcNode->road->GetLaneSectionByIdx(nrOfLanesection - 1);
	}
	else
	{
		lanesection = srcNode->road->GetLaneSectionByIdx(0);
	}

	std::vector<int> connectingLaneIds;
	int nrOfLanes = lanesection->GetNumberOfLanes();
	for (size_t i = 0; i < nrOfLanes; i++)
	{
		roadmanager::Lane *lane = lanesection->GetLaneByIdx(i);
		int currentlaneId = lane->GetId();
		if (lane->IsDriving() && SIGN(currentlaneId) == SIGN(srcNode->laneId) && lane->GetId() != 0)
		{
			int nextLaneId = srcNode->road->GetConnectingLaneId(srcNode->link, currentlaneId, nextRoad->GetId());
			if (nextLaneId != 0)
			{
				connectingLaneIds.push_back(nextLaneId);
			}
		}
	}
	return connectingLaneIds;
}

Node *ControllerFollowRoute::CreateTargetNode(Node *currentNode, roadmanager::Road *nextRoad, RouteStrategy routeStrategy)
{
	// Create last node (targetnode)
	int nextLaneId = currentNode->road->GetConnectingLaneId(currentNode->link, currentNode->laneId, nextRoad->GetId());
	if (nextLaneId == 0)
	{
		// nextLaneId = 0 -- error
		return nullptr;
	}
	Node *targetNode = new Node;
	targetNode->previous = currentNode;
	targetNode->road = nextRoad;
	targetNode->laneId = nextLaneId;
	double nextWeight = CalcWeightWithPos(currentNode->link->GetContactPointType(),targetWaypoint_,nextRoad,routeStrategy);
	targetNode->weight = currentNode->weight + nextWeight;
	return targetNode;
}

void ControllerFollowRoute::UpdateDistanceVector(std::vector<Node *> nextNodes)
{

	for (Node *nextNode : nextNodes)
	{
		// Check if next node is already in distance_ vector
		size_t i;
		for (i = 0; i < distance_.size(); i++)
		{
			bool sameRoadId = distance_[i]->road->GetId() == nextNode->road->GetId();
			bool sameLaneId = distance_[i]->laneId == nextNode->laneId;
			bool sameLink = distance_[i]->link == nextNode->link;
			if (sameRoadId && sameLaneId && sameLink)
			{
				// Consider it, i.e. calc distance_ and potentially store it (if less than old)
				if (nextNode->weight < distance_[i]->weight)
				{
					// Replace current node with updated node
					distance_[i] = nextNode;
				}
				break;
			}
		}
		if (i == distance_.size())
		{
			distance_.push_back(nextNode);
			unvisited_.push(nextNode);
		}
	}
}

double ControllerFollowRoute::CalcAverageSpeed(roadmanager::Road *road)
{
	int roadTypeCount = road->GetNumberOfRoadTypes();
	double totalSpeed = 0;
	for (size_t i = 0; i < roadTypeCount; i++)
	{
		totalSpeed += road->GetRoadType(i)->speed_;
	}
	if (totalSpeed == 0 || roadTypeCount == 0)
	{
		// Default arbitrary speed is 20 m/s (72 km/h)
		return 20;
	}

	return totalSpeed / roadTypeCount;
}

double ControllerFollowRoute::CalcWeightWithPos(roadmanager::ContactPointType contactPointType, roadmanager::Position pos, roadmanager::Road *road, RouteStrategy routeStrategy)
{
	double roadLength = 0;
	
		if (contactPointType == roadmanager::ContactPointType::CONTACT_POINT_START)
		{
			roadLength = pos.GetS();
		}
		else if (contactPointType == roadmanager::ContactPointType::CONTACT_POINT_END)
		{
			roadLength = road->GetLength() - pos.GetS();
		}
	
	return CalcWeight(routeStrategy,roadLength,road);
}

double ControllerFollowRoute::CalcWeight(RouteStrategy routeStrategy, double roadLength, roadmanager::Road *road){
	if (routeStrategy == RouteStrategy::SHORTEST)
	{
		return roadLength;
	}
	else if (routeStrategy == RouteStrategy::FASTEST)
	{
		double averageSpeed = CalcAverageSpeed(road);

		return roadLength / averageSpeed;
	}
}

bool ControllerFollowRoute::FindGoal(roadmanager::OpenDrive *odr, RouteStrategy routeStrategy)
{
	while (!unvisited_.empty())
	{
		Node *currentNode = unvisited_.top();
		unvisited_.pop();
		bool nodeIsVisited = std::find(visited_.begin(), visited_.end(), currentNode) != visited_.end();
		if (nodeIsVisited)
		{
			continue;
		}
		visited_.push_back(currentNode);
		roadmanager::Road *targetRoad = odr->GetRoadById(targetWaypoint_.GetTrackId());
		int targetLaneId = targetWaypoint_.GetLaneId();
		bool TargetWaypointInDrivingDirection = SIGN(currentNode->laneId) == SIGN(targetLaneId);
		// Checks if current road is target road and if waypoint is the diving direction of the vehicle
		if (currentNode->road == targetRoad && TargetWaypointInDrivingDirection)
		{
			return true;
		}
		roadmanager::RoadLink *link = currentNode->link;
		roadmanager::Road *pivotRoad = currentNode->road;
		int pivotLaneId = currentNode->laneId;
		std::vector<roadmanager::Road *> nextRoads;
		if (link->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_ROAD)
		{
			nextRoads.push_back(odr->GetRoadById(link->GetElementId()));
		}
		else if (link->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
		{
			// check all junction links (connecting roads) that has pivot road as incoming road
			roadmanager::Junction *junction = odr->GetJunctionById(link->GetElementId());
			for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(pivotRoad->GetId()); j++)
			{
				int roadId = junction->GetConnectingRoadIdFromIncomingRoadId(pivotRoad->GetId(), (int)j);
				nextRoads.push_back(odr->GetRoadById(roadId));
			}
		}

		for (roadmanager::Road *nextRoad : nextRoads)
		{
			if (!nextRoad)
			{
				continue;
			}
			std::vector<Node *> nextNodes = GetNextNodes(nextRoad, targetRoad, currentNode, routeStrategy);
			UpdateDistanceVector(nextNodes);
		}
	}
	return false;
}

bool ControllerFollowRoute::IsTargetValid(roadmanager::OpenDrive *odr)
{
	int roadId = targetWaypoint_.GetTrackId();
	roadmanager::Road *targetRoad = odr->GetRoadById(targetWaypoint_.GetTrackId());
	if (!targetRoad)
	{
		return false;
	}
	if (targetWaypoint_.GetS() > targetRoad->GetLength() && targetWaypoint_.GetS() < 0)
	{
		return false;
	}
	roadmanager::LaneSection *laneSection = targetRoad->GetLaneSectionByS(targetWaypoint_.GetS());
	roadmanager::Lane *lane = laneSection->GetLaneById(targetWaypoint_.GetLaneId());
	if (!lane)
	{
		return false;
	}
	return lane->IsDriving();
}

//Calculate path to target and returns it as a vector of pathnodes
std::vector<Node *> ControllerFollowRoute::CalculatePath(RouteStrategy routeStrategy)
{
	using namespace roadmanager;
	visited_.clear();
	distance_.clear();
	clearQueue(unvisited_);

	OpenDrive *odr = nullptr;
	if (object_ != nullptr)
	{
		odr = object_->pos_.GetOpenDrive();
	}
	if (!IsTargetValid(odr))
	{
		LOG("Targetwaypoint is invalid");
		return {};
	}
	Road *startRoad = odr->GetRoadById(object_->pos_.GetTrackId());
	int startLaneId = object_->pos_.GetLaneId();
	Position startPos = object_->pos_;

	Road *targetRoad = odr->GetRoadById(targetWaypoint_.GetTrackId());
	int targetLaneId = targetWaypoint_.GetLaneId();

	ContactPointType contactPoint = ContactPointType::CONTACT_POINT_UNDEFINED;
	RoadLink *nextElement = nullptr;
	bool isInForwardDirection = startPos.GetHRelative() < M_PI_2 || startPos.GetHRelative() > 3 * M_PI_2;

	if (isInForwardDirection)
	{
		// Along road direction
		contactPoint = ContactPointType::CONTACT_POINT_END;
		nextElement = startRoad->GetLink(LinkType::SUCCESSOR); // Find link to next road or junction
	}
	else
	{
		// Opposite road direction
		contactPoint = ContactPointType::CONTACT_POINT_START;
		nextElement = startRoad->GetLink(LinkType::PREDECESSOR); // Find link to previous road or junction
	}

	// If start and end waypoint are on the same road and same lane,
	// no pathToGoal are needed
	if (startRoad == targetRoad && startLaneId == targetLaneId)
	{
		return {};
	}

	if (!nextElement)
	{
		// No link (next road element) found
		return {};
	}

	Node *startNode = new Node;
	startNode->link = nextElement;
	startNode->road = startRoad;
	startNode->laneId = startLaneId;
	startNode->previous = 0;
	double nextWeight = CalcWeightWithPos(contactPoint,startPos,startRoad,routeStrategy);
	startNode->weight = nextWeight;

	unvisited_.push(startNode);

	bool found = FindGoal(odr, routeStrategy);
	std::vector<Node *> pathToGoal;
	if (found)
	{
		LOG("Goal found");
		Node *nodeIterator = visited_.back();
		LOG("Total weight: %f", nodeIterator->weight);
		while (nodeIterator != 0)
		{
			pathToGoal.push_back(nodeIterator);
			nodeIterator = nodeIterator->previous;
		}
		// LOG("distance_ VECTOR:");
		// for (size_t i = 0; i < distance_.size(); i++)
		// {
		// 	LOG("Idx: %d Road:%d Lane: %d Dist:%f Prev: %d",
		// 		i, distance_[i]->road->GetId(), distance_[i]->laneId, distance_[i]->weight, distance_[i]->previous->road->GetId());
		// }
		// LOG("visited_ VECTOR:");
		// for (size_t i = 0; i < visited_.size(); i++)
		// {
		// 	LOG("Idx: %d Road:%d Dist:%f ",
		// 		i, visited_[i]->road->GetId(), visited_[i]->weight);
		// }
	}
	else
	{
		LOG("Path to target not found");
		// LOG("distance_ VECTOR:");
		// for (size_t i = 0; i < distance_.size(); i++)
		// {
		// 	LOG("Idx: %d Road:%d Lane: %d Dist:%f PrevR: %d PrevL: %d",
		// 		i, distance_[i]->road->GetId(), distance_[i]->laneId, distance_[i]->weight, distance_[i]->previous->road->GetId(), distance_[i]->previous->laneId);
		// }
		// LOG("visited_ VECTOR:");
		// for (size_t i = 0; i < visited_.size(); i++)
		// {
		// 	LOG("Idx: %d Road:%d Dist:%f ",
		// 		i, visited_[i]->road->GetId(), visited_[i]->weight);
		// }
	}

	return pathToGoal;
}