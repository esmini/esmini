#pragma once

#include <string>
#include <queue>
#include "pugixml.hpp"
#include "CommonMini.hpp"
#include "RoadManager.hpp"
#include <unordered_map>

namespace roadmanager
{
    /**
     * @brief Contains information for the lane independent pathfinder
     *
     */
    typedef struct Node
    {
        Road     *road;
        int       currentLaneId;
        int       fromLaneId;
        double    weight;
        RoadLink *link;
        Node     *previous;
        void      Print()
        {
            LOG("road=%d, cl=%d, fl=%d, w=%f", road->GetId(), currentLaneId, fromLaneId, weight);
        }
        bool operator==(const Node &rhs)
        {
            bool sameRoadId     = rhs.road->GetId() == road->GetId();
            bool sameLaneId     = rhs.currentLaneId == currentLaneId;
            bool sameFromLaneId = rhs.fromLaneId == fromLaneId;
            bool sameLink       = rhs.link == link;
            return sameRoadId && sameLaneId && sameFromLaneId && sameLink;
        }
    } Node;

    /**
     * @brief operator for comparing weights of nodes in the priority queue
     *
     */
    struct WeightCompare
    {
    public:
        bool operator()(Node *a, Node *b)  // overloading both operators
        {
            if (a->weight == b->weight)  // sort after lanes if weight is same.
            {
                // Changes lane as soon as possible:
                int aAbs = abs(a->currentLaneId - a->previous->currentLaneId);
                int bAbs = abs(b->currentLaneId - b->previous->currentLaneId);
                // Changes lane as late as possible:
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
    /**
     * @brief Contains the functionality for calculating weights for the lane independent pathfinder
     *
     */
    class RoadCalculations
    {
    public:
        /**
         * @brief calculates the average speed for a given road.
         *          Checks if road as defined speed, if not checks road type
         *
         * @param road
         * @return double (m/s)
         */
        double CalcAverageSpeed(Road *road);
        /**
         * @brief Calculate the weight for a given node.
         *
         * @param previousNode
         * @param routeStrategy
         * @param roadLength
         * @param road
         * @return double ((m) or (s) or (nr of intersection) depending on routestrategy)
         */
        double CalcWeight(Node *previousNode, Position::RouteStrategy routeStrategy, double roadLength, Road *road);
        /**
         * @brief Calculate the weight for a given node, where a position is considered. (start or target)
         *
         * @param previousNode
         * @param pos
         * @param road
         * @param routeStrategy
         * @return double ((m) or (s) or (nr of intersection) depending on routestrategy)
         */
        double CalcWeightWithPos(Node *previousNode, Position pos, Road *road, Position::RouteStrategy routeStrategy);

    private:
        /**
         * @brief roadTypeToSpeed gives speed in (m/s) for a specific road type.
         *        roadtype_unknown is assumed to be rural
         */
        std::unordered_map<Road::RoadType, double> roadTypeToSpeed = {
            {Road::RoadType::ROADTYPE_BICYCLE, 1.389},
            {Road::RoadType::ROADTYPE_PEDESTRIAN, 1.389},
            {Road::RoadType::ROADTYPE_LOWSPEED, 8.333},
            {Road::RoadType::ROADTYPE_TOWN, 13.888},
            {Road::RoadType::ROADTYPE_RURAL, 19.444},
            {Road::RoadType::ROADTYPE_MOTORWAY, 25},
            {Road::RoadType::ROADTYPE_UNKNOWN, 19.444},
        };
    };
    /**
     * @brief The lane independent pathfinder
     *
     */
    class LaneIndependentRouter
    {
    public:
        /**
         * @brief Construct a new Lane Independent Router object
         *
         * @param odr the opendrive road network
         */
        LaneIndependentRouter(OpenDrive *odr);

        ~LaneIndependentRouter();

        /**
         * @brief Calculates the path between two positions.
         *
         * @param start
         * @param target
         * @return std::vector<Node *>, empty list if path not found
         */
        std::vector<Node> CalculatePath(Position start, Position target);
        /**
         * @brief Translate a list of nodes (path) in to waypoints
         *
         * @param path list of nodes
         * @param start starting waypoint
         * @param target target waypoint
         * @return std::vector<Position>
         */
        std::vector<Position> GetWaypoints(std::vector<Node> path, Position start, Position target);

    private:
        /**
         * @brief Get the Next Link between two roads
         *
         * @param currentNode
         * @param nextRoad
         * @return RoadLink*, if no link exist returns nullptr
         */
        RoadLink *GetNextLink(Node *currentNode, Road *nextRoad);
        /**
         * @brief Get the nextroads from a link
         *
         * @param link roadlink
         * @param currentRoad
         * @return std::vector<Road *>, empty if no roads exists
         */
        std::vector<Road *> GetNextRoads(RoadLink *link, Road *currentRoad);
        /**
         * @brief Creates a Target Node
         *
         * @param currentNode
         * @param nextRoad
         * @param laneIds a pair containing the current and from lane id.
         * @return Node*
         */
        Node *CreateTargetNode(Node *currentNode, Road *nextRoad, std::pair<int, int> laneIds);
        /**
         * @brief Creates a Start Node
         *
         * @param link
         * @param road
         * @param laneId
         * @param contactPoint
         * @param pos
         * @return Node*
         */
        Node *CreateStartNode(RoadLink *link, Road *road, int laneId, ContactPointType contactPoint, Position pos);
        /**
         * @brief Get the next nodes (one for each lane) for the next road
         *
         * @param nextRoad
         * @param targetRoad
         * @param currentNode currentNode
         * @return std::vector<Node *>
         */
        std::vector<Node *> GetNextNodes(Road *nextRoad, Road *targetRoad, Node *currentNode);
        /**
         * @brief Get the Connecting Lanes between a node (road,lane) and the next road
         *
         * @param currentNode
         * @param nextRoad
         * @return std::vector<std::pair<int, int>>  <fromlaneId,currentLaneId>
         */
        std::vector<std::pair<int, int>> GetConnectingLanes(Node *currentNode, Road *nextRoad);
        /**
         * @brief The main loop of the lane independent pathfinder
         *
         * @return true if path is found
         */
        bool FindGoal();
        /**
         * @brief Checks if a position is valid on the OpenDRIVE network.
         *
         * @param pos
         * @return true
         * @return false
         */
        bool IsPositionValid(Position pos);
        template <class Q>
        void clearQueue(Q &q)
        {
            while (!q.empty())
            {
                auto p = q.top();
                q.pop();
                delete p;
            }
            q = Q();
        }
        template <class V>
        void clearVector(V &v)
        {
            for (auto p : v)
            {
                delete p;
            }
            v.clear();
        }

        struct InspectionPriorityQueue : public std::priority_queue<Node *, std::vector<Node *>, WeightCompare>
        {
            using BaseClass = std::priority_queue<Node *, std::vector<Node *>, WeightCompare>;
            using BaseClass::BaseClass;

        public:
            std::vector<Node *> &GetUnderlyingContainer()
            {
                return c;
            }
        };

        InspectionPriorityQueue unvisited_;
        std::vector<Node *>     visited_;
        Position                targetWaypoint_;
        OpenDrive              *odr_;
        RoadCalculations        roadCalculations_;
        Position::RouteStrategy routeStrategy_;
    };

}  // namespace roadmanager