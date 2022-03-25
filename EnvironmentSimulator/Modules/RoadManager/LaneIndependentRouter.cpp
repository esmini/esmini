#include <algorithm>
#include "CommonMini.hpp"
#include "pugixml.hpp"
#include "LaneIndependentRouter.hpp"

using namespace roadmanager;

LaneIndependentRouter::LaneIndependentRouter(OpenDrive *odr) : odr_(odr)
{
}

// Gets the next pathnode for the nextroad based on current srcnode
std::vector<Node *> LaneIndependentRouter::GetNextNodes(Road *nextRoad, Road *targetRoad, Node *currentNode, RouteStrategy routeStrategy)
{
	std::vector<std::pair<int, int>> connectingLaneIds = GetConnectingLanes(currentNode, nextRoad);
	if (connectingLaneIds.empty())
	{
		// no existing lanes
		// LOG("No lanes");
		return {};
	}

	std::vector<Node *> nextNodes;
	//LOG("Road: %d", currentNode->road->GetId());
	//LOG("Lane: %d", currentNode->laneId);
	//LOG("NextRoad: %d", nextRoad->GetId())
	int targetLaneId = targetWaypoint_.GetLaneId();;
	for (std::pair<int, int> lanePair : connectingLaneIds)
	{
		Node *pNode = new Node;
		// bool TargetWaypointInDrivingDirection = SIGN(nextLaneId) == SIGN(targetLaneId);
		if (nextRoad == targetRoad && lanePair.second == targetLaneId)
		{
			// Target road found and driving in same direction, create a target node.
			pNode = CreateTargetNode(currentNode, nextRoad, lanePair, routeStrategy);
		}
		else
		{
			RoadLink *nextLink = GetNextLink(currentNode, nextRoad);
			if (!nextLink)
			{
				// Dont add node if it does not have a link. (end of road)
				continue;
			}
			// create next non target node

			//LOG("clID = %d, lID = %d", lanePair.first, lanePair.second);
			pNode->link = nextLink;
			pNode->road = nextRoad;
			pNode->currentLaneId = lanePair.second;
			pNode->fromLaneId = lanePair.first;
			pNode->previous = currentNode;
			// FIX TARGETWAYPOINT_ ....
			double nextWeight = CalcWeight(routeStrategy, nextRoad->GetLength(), nextRoad);
			pNode->weight = currentNode->weight + nextWeight;
		}
		if (pNode)
		{
			nextNodes.push_back(pNode);
		}
	}
	return nextNodes;
}

RoadLink *LaneIndependentRouter::GetNextLink(Node *currentNode, Road *nextRoad)
{
	if (currentNode->link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
	{
		// node link is a road, find link in the other end of it
		if (currentNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END)
		{
			return nextRoad->GetLink(LinkType::PREDECESSOR);
		}
		else
		{
			return nextRoad->GetLink(LinkType::SUCCESSOR);
		}
	}
	else if (currentNode->link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		Junction *junction = Position::GetOpenDrive()->GetJunctionById(currentNode->link->GetElementId());
		int elementId;
		if (junction && junction->GetType() == Junction::JunctionType::DIRECT)
		{
			elementId = junction->GetId();
		}
		else
		{
			// Default junction
			elementId = currentNode->road->GetId();
		}

		if (nextRoad->GetLink(LinkType::SUCCESSOR) &&
			nextRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == elementId)
		{
			return nextRoad->GetLink(LinkType::PREDECESSOR);
		}
		else if (nextRoad->GetLink(LinkType::PREDECESSOR) &&
				 nextRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == elementId)
		{
			return nextRoad->GetLink(LinkType::SUCCESSOR);
		}
	}

	// end of road
	// LOG("nextLink null");
	return nullptr;
}

std::vector<std::pair<int, int>> LaneIndependentRouter::GetConnectingLanes(Node *srcNode, Road *nextRoad)
{
	LaneSection *lanesection = nullptr;
	if (srcNode->link->GetType() == LinkType::SUCCESSOR)
	{
		int nrOfLanesection = srcNode->road->GetNumberOfLaneSections();
		lanesection = srcNode->road->GetLaneSectionByIdx(nrOfLanesection - 1);
	}
	else
	{
		lanesection = srcNode->road->GetLaneSectionByIdx(0);
	}

	std::vector<std::pair<int, int>> connectingLaneIds;
	int nrOfLanes = lanesection->GetNumberOfLanes();
	for (size_t i = 0; i < nrOfLanes; i++)
	{
		Lane *lane = lanesection->GetLaneByIdx((int)i);
		int currentlaneId = lane->GetId();
		if (lane->IsDriving() && SIGN(currentlaneId) == SIGN(srcNode->currentLaneId) && lane->GetId() != 0)
		{

			// LOG("CurrrentLaneId: %d", currentlaneId);
			int nextLaneId = srcNode->road->GetConnectingLaneId(srcNode->link, currentlaneId, nextRoad->GetId());
			// LOG("NextLaneId: %d", nextLaneId);
			if (nextLaneId != 0)
			{
				connectingLaneIds.push_back({currentlaneId, nextLaneId});
			}
		}
	}

	return connectingLaneIds;
}

Node *LaneIndependentRouter::CreateTargetNode(Node *currentNode, Road *nextRoad, std::pair<int, int> laneIds, RouteStrategy routeStrategy)
{
	// Create last node (targetnode)
	Node *targetNode = new Node;
	targetNode->previous = currentNode;
	targetNode->road = nextRoad;
	targetNode->currentLaneId = laneIds.second;
	targetNode->fromLaneId = laneIds.first;
	targetNode->link = nullptr;
	double nextWeight = CalcWeightWithPos(currentNode->link->GetContactPointType(), targetWaypoint_, nextRoad, routeStrategy);
	targetNode->weight = currentNode->weight + nextWeight;
	return targetNode;
}

void LaneIndependentRouter::UpdateDistanceVector(std::vector<Node *> nextNodes)
{

	for (Node *nextNode : nextNodes)
	{
		// Check if next node is already in distance_ vector
		size_t i;
		for (i = 0; i < distance_.size(); i++)
		{
			bool sameRoadId = distance_[i]->road->GetId() == nextNode->road->GetId();
			bool sameLaneId = distance_[i]->currentLaneId == nextNode->currentLaneId;
			bool sameFromLaneId = distance_[i]->fromLaneId == nextNode->fromLaneId;
			bool sameLink = distance_[i]->link == nextNode->link;
			if (sameRoadId && sameLaneId && sameLink && sameFromLaneId)
			{
				// Consider it, i.e. calc distance_ and potentially store it (if less than old)
				if (nextNode->weight < distance_[i]->weight)
				{
					// Replace current node with updated node
					LOG("UpdateVector: r=%d,cl=%d,fl=%d", nextNode->road->GetId(), nextNode->currentLaneId,nextNode->fromLaneId);
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

double LaneIndependentRouter::CalcAverageSpeed(Road *road)
{
	int roadTypeCount = road->GetNumberOfRoadTypes();
	double totalSpeed = 0;
	for (size_t i = 0; i < roadTypeCount; i++)
	{
		totalSpeed += road->GetRoadType((int)i)->speed_;
	}
	if (totalSpeed == 0 || roadTypeCount == 0)
	{
		// Default arbitrary speed is 20 m/s (72 km/h)
		return 20;
	}

	return totalSpeed / roadTypeCount;
}

double LaneIndependentRouter::CalcWeightWithPos(ContactPointType contactPointType, Position pos, Road *road, RouteStrategy routeStrategy)
{
	double roadLength = 0;

	if (contactPointType == ContactPointType::CONTACT_POINT_START)
	{
		roadLength = pos.GetS();
	}
	else if (contactPointType == ContactPointType::CONTACT_POINT_END)
	{
		roadLength = road->GetLength() - pos.GetS();
	}

	return CalcWeight(routeStrategy, roadLength, road);
}

double LaneIndependentRouter::CalcWeight(RouteStrategy routeStrategy, double roadLength, Road *road)
{
	if (routeStrategy == RouteStrategy::SHORTEST)
	{
		return roadLength;
	}
	else if (routeStrategy == RouteStrategy::FASTEST)
	{
		double averageSpeed = CalcAverageSpeed(road);

		return roadLength / averageSpeed;
	}
	else
	{
		LOG("RouteStrategy does not exist");
		return 0;
	}
}

bool LaneIndependentRouter::FindGoal(RouteStrategy routeStrategy)
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
		Road *targetRoad = odr_->GetRoadById(targetWaypoint_.GetTrackId());
		int targetLaneId = targetWaypoint_.GetLaneId();
		// bool TargetWaypointInDrivingDirection = SIGN(currentNode->laneId) == SIGN(targetLaneId);
		//  Checks if current road is target road and if waypoint is the diving direction of the vehicle
		if (currentNode->road == targetRoad && currentNode->currentLaneId == targetLaneId)
		{
			return true;
		}
		if (!currentNode->link)
		{
			continue;
		}
		RoadLink *link = currentNode->link;
		Road *currentRoad = currentNode->road;
		std::vector<Road *> nextRoads;
		if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
		{
			nextRoads.push_back(odr_->GetRoadById(link->GetElementId()));
		}
		else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
		{
			// check all junction links (connecting roads) that has pivot road as incoming road
			Junction *junction = odr_->GetJunctionById(link->GetElementId());
			for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(currentRoad->GetId()); j++)
			{
				int roadId = junction->GetConnectingRoadIdFromIncomingRoadId(currentRoad->GetId(), (int)j);
				nextRoads.push_back(odr_->GetRoadById(roadId));
			}
		}

		for (Road *nextRoad : nextRoads)
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

bool LaneIndependentRouter::IsTargetValid(Position target)
{
	Road *targetRoad = odr_->GetRoadById(target.GetTrackId());
	if (!targetRoad)
	{
		//LOG("1");
		return false;
	}
	if (target.GetS() > targetRoad->GetLength() || target.GetS() < 0)
	{
		//LOG("2");
		return false;
	}
	LaneSection *laneSection = targetRoad->GetLaneSectionByS(target.GetS());
	Lane *lane = laneSection->GetLaneById(target.GetLaneId());
	if (!lane)
	{
		//LOG("3");
		return false;
	}
	return lane->IsDriving();
}

// Calculate path to target and returns it as a vector of pathnodes
std::vector<Node *> LaneIndependentRouter::CalculatePath(Position start, Position target, RouteStrategy routeStrategy)
{
	visited_.clear();
	distance_.clear();
	clearQueue(unvisited_);

	if (!IsTargetValid(target))
	{
		LOG("Targetwaypoint is invalid");
		return {};
	}
	if (routeStrategy == RouteStrategy::MIN_INTERSECTIONS)
	{
		LOG("MIN_INTERSECTION not yet implemented");
		return {};
	}

	Road *startRoad = odr_->GetRoadById(start.GetTrackId());
	int startLaneId = start.GetLaneId();

	targetWaypoint_ = target;
	Road *targetRoad = odr_->GetRoadById(targetWaypoint_.GetTrackId());
	int targetLaneId = targetWaypoint_.GetLaneId();

	ContactPointType contactPoint = ContactPointType::CONTACT_POINT_UNDEFINED;
	RoadLink *nextElement = nullptr;
	bool isInForwardDirection = start.GetHRelative() < M_PI_2 || start.GetHRelative() > 3 * M_PI_2;

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
		LOG("start is target");
		return {};
	}

	if (!nextElement)
	{
		// No link (next road element) found
		LOG("No link from start");
		return {};
	}

	Node *startNode = new Node;
	startNode->link = nextElement;
	startNode->road = startRoad;
	startNode->currentLaneId = startLaneId;
	startNode->fromLaneId = 0;
	startNode->previous = 0;
	double nextWeight = CalcWeightWithPos(contactPoint, start, startRoad, routeStrategy);
	startNode->weight = nextWeight;

	unvisited_.push(startNode);

	bool found = FindGoal(routeStrategy);
	std::vector<Node *> pathToGoal;
	if (found)
	{
		// LOG("Goal found");
		Node *nodeIterator = visited_.back();
		// LOG("Total weight: %f", nodeIterator->weight);
		while (nodeIterator != 0)
		{
			pathToGoal.push_back(nodeIterator);
			nodeIterator = nodeIterator->previous;
		}
	}
	else
	{
		LOG("Path to target not found");
	}
	std::reverse(pathToGoal.begin(), pathToGoal.end());
	LOG("Path to goal:");
	for (Node *n : pathToGoal)
	{
		LOG("r=%d,cl=%d,fl=%d", n->road->GetId(), n->currentLaneId,n->fromLaneId);
	}
	LOG("Distance:");
	for (Node *n : distance_)
	{
		LOG("r=%d,cl=%d,fl=%d", n->road->GetId(), n->currentLaneId,n->fromLaneId);
	}
	LOG("Visited:");
	for (Node *n : visited_)
	{
		LOG("r=%d,cl=%d,fl=%d", n->road->GetId(), n->currentLaneId,n->fromLaneId);
	}
	return pathToGoal;
}

std::vector<Position> LaneIndependentRouter::GetWaypoints(std::vector<Node *> path, Position target)
{
	std::vector<Position> waypoints;
	return waypoints;
}