#pragma once


#include <string>
#include <queue>
#include "pugixml.hpp"
#include "CommonMini.hpp"
#include "RoadManager.hpp"

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
        int laneId;
        double weight;
        RoadLink *link;
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

    class LaneIndependentRouter
    {
    public:
        LaneIndependentRouter(OpenDrive* odr);
        std::vector<Node *> CalculatePath(Position start, Position target, RouteStrategy routeStrategy = RouteStrategy::SHORTEST);

    private:
        Node *CreateTargetNode(Node *currentNode, Road *nextRoad, RouteStrategy routeStrategy);
        void UpdateDistanceVector(std::vector<Node *> nextNodes);
        bool TargetLaneIsInDrivingDirection(Node *pNode, Road *nextRoad);
        std::vector<Node *> GetNextNodes(Road *nextRoad, Road *targetRoad, Node *srcNode, RouteStrategy routeStrategy);
        std::vector<int> GetConnectingLanes(Node *srcNode, Road *nextRoad);
        bool FindGoal(RouteStrategy routeStrategy);
        double CalcAverageSpeed(Road *road);
        double CalcWeight(RouteStrategy routeStrategy, double roadLength, Road *road);
        double CalcWeightWithPos(ContactPointType contactPointType, Position pos, Road *road, RouteStrategy routeStrategy);
        bool IsTargetValid(Position target);
        template <class Q>
        void clearQueue(Q &q) { q = Q(); }

        std::priority_queue<Node *, std::vector<Node *>, WeightCompare> unvisited_;
        std::vector<Node *> visited_;
        std::vector<Node *> distance_;
        Position targetWaypoint_;
        OpenDrive* odr_;
    };
}