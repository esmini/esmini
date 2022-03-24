#include <algorithm>
#include "CommonMini.hpp"
#include "pugixml.hpp"
#include "LaneIndependentRouter.hpp"



using namespace roadmanager;

LaneIndependentRouter::LaneIndependentRouter(OpenDrive* odr): odr_(odr){

}

// Gets the next pathnode for the nextroad based on current srcnode
std::vector<Node *> LaneIndependentRouter::GetNextNodes(Road *nextRoad, Road *targetRoad, Node *currentNode, RouteStrategy routeStrategy)
{

	// Register length of this road and find node in other end of the road (link)
	RoadLink *nextLink = 0;

	if (currentNode->link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
	{
		// node link is a road, find link in the other end of it
		if (currentNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END)
		{
			nextLink = nextRoad->GetLink(LinkType::PREDECESSOR);
		}
		else
		{
			nextLink = nextRoad->GetLink(LinkType::SUCCESSOR);
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

			nextLink = nextRoad->GetLink(LinkType::PREDECESSOR);
		}
		else if (nextRoad->GetLink(LinkType::PREDECESSOR) &&
				 nextRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == elementId)
		{
			nextLink = nextRoad->GetLink(LinkType::SUCCESSOR);
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

std::vector<int> LaneIndependentRouter::GetConnectingLanes(Node *srcNode, Road *nextRoad)
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

	std::vector<int> connectingLaneIds;
	int nrOfLanes = lanesection->GetNumberOfLanes();
	for (size_t i = 0; i < nrOfLanes; i++)
	{
		Lane *lane = lanesection->GetLaneByIdx((int)i);
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

Node *LaneIndependentRouter::CreateTargetNode(Node *currentNode, Road *nextRoad, RouteStrategy routeStrategy)
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
		bool TargetWaypointInDrivingDirection = SIGN(currentNode->laneId) == SIGN(targetLaneId);
		// Checks if current road is target road and if waypoint is the diving direction of the vehicle
		if (currentNode->road == targetRoad && TargetWaypointInDrivingDirection)
		{
			return true;
		}
		RoadLink *link = currentNode->link;
		Road *pivotRoad = currentNode->road;

		std::vector<Road *> nextRoads;
		if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
		{
			nextRoads.push_back(odr_->GetRoadById(link->GetElementId()));
		}
		else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
		{
			// check all junction links (connecting roads) that has pivot road as incoming road
			Junction *junction = odr_->GetJunctionById(link->GetElementId());
			for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(pivotRoad->GetId()); j++)
			{
				int roadId = junction->GetConnectingRoadIdFromIncomingRoadId(pivotRoad->GetId(), (int)j);
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
		return false;
	}
	if (target.GetS() > targetRoad->GetLength() || target.GetS() < 0)
	{
		return false;
	}
	LaneSection *laneSection = targetRoad->GetLaneSectionByS(target.GetS());
	Lane *lane = laneSection->GetLaneById(target.GetLaneId());
	if (!lane)
	{
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
	double nextWeight = CalcWeightWithPos(contactPoint, start, startRoad, routeStrategy);
	startNode->weight = nextWeight;

	unvisited_.push(startNode);

	bool found = FindGoal(routeStrategy);
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
	}
	else
	{
		LOG("Path to target not found");
	}

	return pathToGoal;
}
