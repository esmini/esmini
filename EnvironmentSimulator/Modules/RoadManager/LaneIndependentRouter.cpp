#include <algorithm>
#include "CommonMini.hpp"
#include "pugixml.hpp"
#include "LaneIndependentRouter.hpp"

using namespace roadmanager;

LaneIndependentRouter::LaneIndependentRouter(OpenDrive *odr) : odr_(odr), roadCalculations_(RoadCalculations())
{
}

LaneIndependentRouter::~LaneIndependentRouter()
{
    clearQueue(unvisited_);
    clearVector(visited_);
}

// Gets the next pathnode for the nextroad based on current srcnode
std::vector<Node *> LaneIndependentRouter::GetNextNodes(Road *nextRoad, Road *targetRoad, Node *currentNode)
{
    std::vector<std::pair<int, int>> connectingLaneIds = GetConnectingLanes(currentNode, nextRoad);
    if (connectingLaneIds.empty())
    {
        return {};
    }

    std::vector<Node *> nextNodes;
    int                 targetLaneId = targetWaypoint_.GetLaneId();
    ;
    for (std::pair<int, int> lanePair : connectingLaneIds)
    {
        Node *pNode = nullptr;
        if (nextRoad == targetRoad && lanePair.second == targetLaneId)
        {
            // Target road found and driving in same direction, create a target node.
            pNode = CreateTargetNode(currentNode, nextRoad, lanePair);
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
            pNode                = new Node;
            pNode->link          = nextLink;
            pNode->road          = nextRoad;
            pNode->currentLaneId = lanePair.second;
            pNode->fromLaneId    = lanePair.first;
            pNode->previous      = currentNode;
            double nextWeight    = roadCalculations_.CalcWeight(currentNode, routeStrategy_, nextRoad->GetLength(), nextRoad);
            pNode->weight        = currentNode->weight + nextWeight;
        }
        if (pNode)
        {
            nextNodes.push_back(pNode);
        }
    }
    return nextNodes;
}

std::vector<Road *> LaneIndependentRouter::GetNextRoads(RoadLink *link, Road *currentRoad)
{
    std::vector<Road *> nextRoads;
    Road               *nextRoad;
    if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
    {
        nextRoad = odr_->GetRoadById(link->GetElementId());
        if (nextRoad)  // Dont push nullptr
        {
            nextRoads.push_back(nextRoad);
        }
    }
    else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
    {
        // check all junction links (connecting roads) that has pivot road as incoming road
        Junction *junction = odr_->GetJunctionById(link->GetElementId());
        for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(currentRoad->GetId()); j++)
        {
            int roadId = junction->GetConnectingRoadIdFromIncomingRoadId(currentRoad->GetId(), (int)j);
            nextRoad   = odr_->GetRoadById(roadId);
            if (nextRoad)  // Dont push nullptr
            {
                nextRoads.push_back(nextRoad);
            }
        }
    }
    return nextRoads;
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
        int       elementId;
        if (junction && junction->GetType() == Junction::JunctionType::DIRECT)
        {
            elementId = junction->GetId();
        }
        else
        {
            // Default junction
            elementId = currentNode->road->GetId();
        }

        if (nextRoad->GetLink(LinkType::SUCCESSOR) && nextRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == elementId)
        {
            return nextRoad->GetLink(LinkType::PREDECESSOR);
        }
        else if (nextRoad->GetLink(LinkType::PREDECESSOR) && nextRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == elementId)
        {
            return nextRoad->GetLink(LinkType::SUCCESSOR);
        }
    }
    // end of road
    return nullptr;
}

std::vector<std::pair<int, int>> LaneIndependentRouter::GetConnectingLanes(Node *currentNode, Road *nextRoad)
{
    LaneSection *lanesection = nullptr;
    if (currentNode->link->GetType() == LinkType::SUCCESSOR)
    {
        int nrOfLanesection = currentNode->road->GetNumberOfLaneSections();
        lanesection         = currentNode->road->GetLaneSectionByIdx(nrOfLanesection - 1);
    }
    else
    {
        lanesection = currentNode->road->GetLaneSectionByIdx(0);
    }

    std::vector<std::pair<int, int>> connectingLaneIds;
    int                              nrOfLanes = lanesection->GetNumberOfLanes();
    for (size_t i = 0; i < nrOfLanes; i++)
    {
        Lane *lane          = lanesection->GetLaneByIdx((int)i);
        int   currentlaneId = lane->GetId();
        if (lane->IsDriving() && SIGN(currentlaneId) == SIGN(currentNode->currentLaneId) && lane->GetId() != 0)
        {
            int nextLaneId = currentNode->road->GetConnectingLaneId(currentNode->link, currentlaneId, nextRoad->GetId());
            if (nextLaneId != 0)
            {
                connectingLaneIds.push_back({currentlaneId, nextLaneId});
            }
        }
    }

    return connectingLaneIds;
}

Node *LaneIndependentRouter::CreateTargetNode(Node *currentNode, Road *nextRoad, std::pair<int, int> laneIds)
{
    // Create last node (targetnode)
    Node *targetNode          = new Node;
    targetNode->previous      = currentNode;
    targetNode->road          = nextRoad;
    targetNode->currentLaneId = laneIds.second;
    targetNode->fromLaneId    = laneIds.first;
    targetNode->link          = nullptr;
    double nextWeight         = roadCalculations_.CalcWeightWithPos(currentNode, targetWaypoint_, nextRoad, routeStrategy_);
    targetNode->weight        = currentNode->weight + nextWeight;
    return targetNode;
}

bool LaneIndependentRouter::FindGoal()
{
    while (!unvisited_.empty())
    {
        Node *currentNode = unvisited_.top();
        unvisited_.pop();
        bool nodeIsVisited = std::find_if(visited_.begin(), visited_.end(), [currentNode](Node *n) { return *n == *currentNode; }) != visited_.end();
        if (nodeIsVisited)
        {
            delete currentNode;
            continue;
        }
        visited_.push_back(currentNode);
        Road *targetRoad   = odr_->GetRoadById(targetWaypoint_.GetTrackId());
        int   targetLaneId = targetWaypoint_.GetLaneId();
        if (currentNode->road == targetRoad && currentNode->currentLaneId == targetLaneId)
        {
            return true;
        }
        if (!currentNode->link)
        {
            continue;
        }
        std::vector<Road *> nextRoads = GetNextRoads(currentNode->link, currentNode->road);
        for (Road *nextRoad : nextRoads)
        {
            std::vector<Node *> nextNodes = GetNextNodes(nextRoad, targetRoad, currentNode);
            for (Node *n : nextNodes)
            {
                unvisited_.push(n);
            }
        }
    }
    return false;
}

bool LaneIndependentRouter::IsPositionValid(Position pos)
{
    Road *road = odr_->GetRoadById(pos.GetTrackId());
    if (!road)
    {
        return false;
    }
    if (pos.GetS() > road->GetLength() || pos.GetS() < 0)
    {
        return false;
    }
    LaneSection *laneSection = road->GetLaneSectionByS(pos.GetS());
    Lane        *lane        = laneSection->GetLaneById(pos.GetLaneId());
    if (!lane)
    {
        return false;
    }
    return lane->IsDriving();  // true if lane is defined as drivable
}

Node *LaneIndependentRouter::CreateStartNode(RoadLink *link, Road *road, int laneId, ContactPointType contactPoint, Position pos)
{
    Node *startNode          = new Node;
    startNode->link          = link;
    startNode->road          = road;
    startNode->currentLaneId = laneId;
    startNode->fromLaneId    = 0;
    startNode->previous      = 0;

    double roadLength = 0;

    if (contactPoint == ContactPointType::CONTACT_POINT_START)
    {
        roadLength = pos.GetS();
    }
    else if (contactPoint == ContactPointType::CONTACT_POINT_END)
    {
        roadLength = road->GetLength() - pos.GetS();
    }

    double nextWeight = 0;
    if (routeStrategy_ == Position::RouteStrategy::SHORTEST)
    {
        nextWeight = roadLength;
    }
    else if (routeStrategy_ == Position::RouteStrategy::FASTEST)
    {
        double averageSpeed = roadCalculations_.CalcAverageSpeed(road);
        nextWeight          = roadLength / averageSpeed;
    }
    else if (routeStrategy_ == Position::RouteStrategy::MIN_INTERSECTIONS)
    {
        nextWeight = 0;
    }
    startNode->weight = nextWeight;
    return startNode;
}

std::vector<Node> LaneIndependentRouter::CalculatePath(Position start, Position target)
{
    clearQueue(unvisited_);
    clearVector(visited_);

    if (!IsPositionValid(start))
    {
        LOG("(LaneIndependentRouter::CalculatePath) Error: Start position is invalid");
        return {};
    }
    if (!IsPositionValid(target))
    {
        LOG("(LaneIndependentRouter::CalculatePath) Error: Target position is invalid");
        return {};
    }

    Road *startRoad   = odr_->GetRoadById(start.GetTrackId());
    int   startLaneId = start.GetLaneId();

    targetWaypoint_    = target;
    Road *targetRoad   = odr_->GetRoadById(targetWaypoint_.GetTrackId());
    int   targetLaneId = targetWaypoint_.GetLaneId();

    // Get routestrategy from traget position
    routeStrategy_ = target.GetRouteStrategy();

    ContactPointType contactPoint         = ContactPointType::CONTACT_POINT_UNDEFINED;
    RoadLink        *nextElement          = nullptr;
    bool             isInForwardDirection = start.GetHRelative() < M_PI_2 || start.GetHRelative() > 3 * M_PI_2;

    if (isInForwardDirection)
    {
        // Along road direction
        contactPoint = ContactPointType::CONTACT_POINT_END;
        nextElement  = startRoad->GetLink(LinkType::SUCCESSOR);  // Find link to next road or junction
    }
    else
    {
        // Opposite road direction
        contactPoint = ContactPointType::CONTACT_POINT_START;
        nextElement  = startRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
    }

    // If start and end waypoint are on the same road and same lane,
    // no pathToGoal is needed
    if (startRoad == targetRoad && startLaneId == targetLaneId)
    {
        LOG("(LaneIndependentRouter::CalculatePath) Error: start pos and target pos on same road and lane");
        return {};
    }

    if (!nextElement)
    {
        // No link (next road element) found
        LOG("(LaneIndependentRouter::CalculatePath) Error: No link from start pos");
        return {};
    }

    Node *startNode = CreateStartNode(nextElement, startRoad, startLaneId, contactPoint, start);
    unvisited_.push(startNode);

    bool              found = FindGoal();
    std::vector<Node> pathToGoal;
    if (found)
    {
        Node *nodeIterator = visited_.back();
        while (nodeIterator != 0)
        {
            pathToGoal.push_back(*nodeIterator);
            nodeIterator = nodeIterator->previous;
        }
    }
    else
    {
        LOG("(LaneIndependentRouter::CalculatePath) Warning: Path to target not found");
    }
    std::reverse(pathToGoal.begin(), pathToGoal.end());
    return pathToGoal;
}

std::vector<Position> LaneIndependentRouter::GetWaypoints(std::vector<Node> path, Position start, Position target)
{
    std::vector<Position> waypoints;
    for (int idx = 0; idx < path.size() - 1; idx++)
    {
        Node  *current    = &path[idx];
        Node  *next       = &path[idx + 1];
        double laneLength = 0;
        double sPos       = 0;
        double heading    = 0;
        if (current->link->GetType() == LinkType::SUCCESSOR)
        {
            for (int i = current->road->GetNumberOfLaneSections() - 1; i >= 0; i--)
            {
                Lane *lane = current->road->GetLaneSectionByIdx(i)->GetLaneById(next->fromLaneId);
                if (!lane || !lane->IsDriving())
                {
                    break;
                }
                laneLength += current->road->GetLaneSectionByIdx(i)->GetLength();
            }

            heading = 0;
            // calculate sPos to be the middle of the road
            sPos = current->road->GetLength() - (laneLength / 2);
            if (idx == 0 && start.GetS() > current->road->GetLength() - laneLength)
            {
                sPos = start.GetS();
            }
        }
        else if (current->link->GetType() == LinkType::PREDECESSOR)
        {
            for (int i = 0; i < current->road->GetNumberOfLaneSections(); i++)
            {
                Lane *lane = current->road->GetLaneSectionByIdx(i)->GetLaneById(next->fromLaneId);
                if (!lane || !lane->IsDriving())
                {
                    break;
                }
                laneLength += current->road->GetLaneSectionByIdx(i)->GetLength();
            }

            heading = M_PI;
            // calculate sPos to be the middle of the road
            sPos = laneLength / 2;
            if (idx == 0 && start.GetS() < laneLength)
            {
                sPos = start.GetS();
            }
        }
        Position p(current->road->GetId(), next->fromLaneId, sPos, 0.0);
        p.SetHeadingRelativeRoadDirection(heading);
        waypoints.push_back(p);
    }

    waypoints.push_back(target);
    return waypoints;
}

double RoadCalculations::CalcAverageSpeed(Road *road)
{
    int roadTypeCount = road->GetNumberOfRoadTypes();
    if (roadTypeCount == 0)
    {
        // Assume road is rural
        LOG("Warning: Road %d has no road types (and speed limit)", road->GetId());
        return roadTypeToSpeed[Road::RoadType::ROADTYPE_RURAL];
    }

    double totalSpeed = 0;
    for (size_t i = 0; i < roadTypeCount; i++)
    {
        if (road->GetRoadType((int)i)->speed_ > SMALL_NUMBER)
        {
            totalSpeed += road->GetRoadType((int)i)->speed_;
        }
        else
        {
            bool hasDefinedSpeedForRoadType = roadTypeToSpeed.find(road->GetRoadType((int)i)->road_type_) != roadTypeToSpeed.end();
            if (hasDefinedSpeedForRoadType)
            {
                totalSpeed += roadTypeToSpeed[road->GetRoadType((int)i)->road_type_];
            }
            else
            {
                LOG("Error: Road %d has undefined road type", road->GetId());
            }
        }
    }

    return totalSpeed / (double)roadTypeCount;
}

double RoadCalculations::CalcWeightWithPos(Node *previousNode, Position pos, Road *road, Position::RouteStrategy routeStrategy)
{
    double roadLength = 0;

    if (previousNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_START)
    {
        roadLength = pos.GetS();
    }
    else if (previousNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END)
    {
        roadLength = road->GetLength() - pos.GetS();
    }

    return CalcWeight(previousNode, routeStrategy, roadLength, road);
}

double RoadCalculations::CalcWeight(Node *previousNode, Position::RouteStrategy routeStrategy, double roadLength, Road *road)
{
    if (routeStrategy == Position::RouteStrategy::SHORTEST)
    {
        return roadLength;
    }
    else if (routeStrategy == Position::RouteStrategy::FASTEST)
    {
        double averageSpeed = CalcAverageSpeed(road);
        if (averageSpeed == 0)
        {
            // If average speed is 0, road can't be traveled on
            return LARGE_NUMBER;
        }

        return roadLength / averageSpeed;
    }
    else if (routeStrategy == Position::RouteStrategy::MIN_INTERSECTIONS)
    {
        if (previousNode->link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
        {
            return 1;
        }
        return 0;
    }
    else
    {
        LOG("Error: Position::RouteStrategy weight calculation is not defined");
        return 0;
    }
}