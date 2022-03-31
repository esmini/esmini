#pragma once

#include <string>
#include <queue>
#include "pugixml.hpp"
#include "CommonMini.hpp"
#include "RoadManager.hpp"
#include <unordered_map>

namespace roadmanager
{
    typedef enum
    {
        SHORTEST,
        FASTEST,
        MIN_INTERSECTIONS
    } RouteStrategy;

    typedef struct Node
    {
        Road *road;
        int currentLaneId;
        int fromLaneId;
        // std::pair<int,int> connectingLanes;
        double weight;
        RoadLink *link;
        Node *previous;
        void Print()
        {
            LOG("road=%d, cl=%d, fl=%d, w=%f", road->GetId(), currentLaneId, fromLaneId, weight);
        }
        bool operator==(const Node& rhs){
            bool sameRoadId = rhs.road->GetId() == road->GetId();
			bool sameLaneId = rhs.currentLaneId == currentLaneId;
			bool sameFromLaneId = rhs.fromLaneId == fromLaneId;
			bool sameLink = rhs.link == link;
            return sameRoadId && sameLaneId && sameFromLaneId && sameLink;
        }
    } Node;

    struct WeightCompare
    {
    public:
        bool operator()(Node *a, Node *b) // overloading both operators
        {
            if (a->weight == b->weight)
            {
                int aAbs = abs(a->currentLaneId - a->previous->currentLaneId);
                int bAbs = abs(b->currentLaneId - b->previous->currentLaneId);
                // int aAbs = abs(a->currentLaneId);
                // int bAbs = abs(b->currentLaneId);
                return aAbs > bAbs;
            }
            else
            {
                return a->weight > b->weight;
            }
        }
    };

    class RoadCalculations
    {
    public:
        double CalcAverageSpeed(Road *road);
        double CalcWeight(Node *previousNode, RouteStrategy routeStrategy, double roadLength, Road *road);
        double CalcWeightWithPos(Node *previousNode, Position pos, Road *road, RouteStrategy routeStrategy);

    private:
        // roadTypeToSpeed gives speed in m/s for a specific road type
        std::unordered_map<Road::RoadType, double> roadTypeToSpeed = {
            {Road::RoadType::ROADTYPE_BICYCLE, 1.389},
            {Road::RoadType::ROADTYPE_PEDESTRIAN, 1.389}, 
            {Road::RoadType::ROADTYPE_LOWSPEED, 8.333},
            {Road::RoadType::ROADTYPE_TOWN, 13.888},
            {Road::RoadType::ROADTYPE_RURAL, 19.444},
            {Road::RoadType::ROADTYPE_MOTORWAY, 25},
            // An unknown road is assumed to be rural
            {Road::RoadType::ROADTYPE_UNKNOWN, 19.444},
        };
    };

    class LaneIndependentRouter
    {
    public:
        LaneIndependentRouter(OpenDrive *odr);
        std::vector<Node *> CalculatePath(Position start, Position target, RouteStrategy routeStrategy = RouteStrategy::SHORTEST);
        std::vector<Position> GetWaypoints(std::vector<Node *> path, Position target);

    private:
        RoadLink *GetNextLink(Node *currentNode, Road *nextRoad);
        Node *CreateTargetNode(Node *currentNode, Road *nextRoad, std::pair<int, int> laneIds, RouteStrategy routeStrategy);
        Node *CreateStartNode(RoadLink* link,Road* road, int laneId,ContactPointType contactPoint,RouteStrategy routeStrategy,Position pos);
        std::vector<Node *> GetNextNodes(Road *nextRoad, Road *targetRoad, Node *srcNode, RouteStrategy routeStrategy);
        std::vector<std::pair<int, int>> GetConnectingLanes(Node *srcNode, Road *nextRoad);
        bool FindGoal(RouteStrategy routeStrategy);
        bool IsPositionValid(Position pos);
        template <class Q>
        void clearQueue(Q &q) { q = Q(); }

        std::priority_queue<Node *, std::vector<Node *>, WeightCompare> unvisited_;
        std::vector<Node *> visited_;
        Position targetWaypoint_;
        OpenDrive *odr_;
        RoadCalculations roadCalculations_;
    };

}