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
#include <iostream>
#include <random>
#include "Action.hpp"
#include "CommonMini.hpp"
#include "Parameters.hpp"
#include "ScenarioGateway.hpp"
#include "OSCEnvironment.hpp"
#include "OSCAABBTree.hpp"
#include <vector>
#include "OSCUtils.hpp"
#include "OSCPosition.hpp"
#include "logger.hpp"
#include "VehiclePool.hpp"

namespace scenarioengine
{

    using aabbTree::Solutions;
    using std::vector;
    using namespace Utils;
    class VehiclePool;

    class OSCGlobalAction : public OSCAction
    {
    public:
        OSCGlobalAction(OSCAction::ActionType action_type, StoryBoardElement* parent) : OSCAction(action_type, parent)
        {
        }
        virtual ~OSCGlobalAction() = default;

        virtual void print()
        {
            LOG_WARN("Virtual, should be overridden");
        }

        virtual OSCGlobalAction* Copy()
        {
            LOG_WARN("Virtual, should be overridden");
            return 0;
        };

        virtual std::string Type2Str()
        {
            return "OSCGlobalAction base class";
        };
    };

    class ParameterSetAction : public OSCGlobalAction
    {
    public:
        std::string name_;
        std::string value_;
        Parameters* parameters_;

        ParameterSetAction(StoryBoardElement* parent) : OSCGlobalAction(ActionType::PARAMETER_SET, parent), name_(""), value_(""), parameters_(0){};

        ParameterSetAction(const ParameterSetAction& action) : OSCGlobalAction(ActionType::PARAMETER_SET, action.parent_)
        {
            name_   = action.name_;
            value_  = action.value_;
            parent_ = action.parent_;
        }

        OSCGlobalAction* Copy()
        {
            ParameterSetAction* new_action = new ParameterSetAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "ParameterSetAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void print()
        {
        }
    };

    class VariableSetAction : public OSCGlobalAction
    {
    public:
        std::string name_;
        std::string value_;
        Parameters* variables_;

        VariableSetAction(StoryBoardElement* parent) : OSCGlobalAction(ActionType::VARIABLE_SET, parent), name_(""), value_(""), variables_(0){};

        VariableSetAction(const ParameterSetAction& action) : OSCGlobalAction(ActionType::VARIABLE_SET, action.parent_)
        {
            name_  = action.name_;
            value_ = action.value_;
        }

        OSCGlobalAction* Copy()
        {
            VariableSetAction* new_action = new VariableSetAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "VariableSetAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void print()
        {
        }
    };

    class VariableAddAction : public OSCGlobalAction
    {
    public:
        std::string name_;
        double      value_;
        Parameters* variables_;

        VariableAddAction(StoryBoardElement* parent) : OSCGlobalAction(ActionType::VARIABLE_ADD, parent), name_(""), value_(0), variables_(0){};

        VariableAddAction(const VariableAddAction& action) : OSCGlobalAction(ActionType::VARIABLE_ADD, action.parent_)
        {
            name_  = action.name_;
            value_ = action.value_;
        }

        OSCGlobalAction* Copy()
        {
            VariableAddAction* new_action = new VariableAddAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "VariableAddAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void print()
        {
        }
    };

    class VariableMultiplyByAction : public OSCGlobalAction
    {
    public:
        std::string name_;
        double      value_;
        Parameters* variables_;

        VariableMultiplyByAction(StoryBoardElement* parent)
            : OSCGlobalAction(ActionType::VARIABLE_MULTIPLY_BY, parent),
              name_(""),
              value_(0),
              variables_(0){};

        VariableMultiplyByAction(const VariableMultiplyByAction& action) : OSCGlobalAction(ActionType::VARIABLE_MULTIPLY_BY, action.parent_)
        {
            name_  = action.name_;
            value_ = action.value_;
        }

        OSCGlobalAction* Copy()
        {
            VariableMultiplyByAction* new_action = new VariableMultiplyByAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "VariableMultiplyByAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);

        void print()
        {
        }
    };

    class AddEntityAction : public OSCGlobalAction
    {
    public:
        Object*                entity_;
        roadmanager::Position* pos_;
        Entities*              entities_;

        AddEntityAction(StoryBoardElement* parent) : OSCGlobalAction(ActionType::ADD_ENTITY, parent), entity_(nullptr), pos_(0), entities_(nullptr){};

        AddEntityAction(Object* entity, StoryBoardElement* parent)
            : OSCGlobalAction(ActionType::ADD_ENTITY, parent),
              entity_(entity),
              pos_(0),
              entities_(nullptr){};

        AddEntityAction(const AddEntityAction& action) : OSCGlobalAction(ActionType::ADD_ENTITY, action.parent_)
        {
            entity_   = action.entity_;
            entities_ = action.entities_;
            pos_      = action.pos_;
        }
        ~AddEntityAction()
        {
            if (pos_ != nullptr)
            {
                delete pos_;
                pos_ = nullptr;
            }
        }
        OSCGlobalAction* Copy()
        {
            AddEntityAction* new_action = new AddEntityAction(*this);
            return new_action;
        }

        void Start(double simTime);
        void Step(double simTime, double dt);

        void SetEntities(Entities* entities)
        {
            entities_ = entities;
        }

        void print()
        {
        }
    };

    class DeleteEntityAction : public OSCGlobalAction
    {
    public:
        Object*          entity_;
        Entities*        entities_;
        ScenarioGateway* gateway_;

        DeleteEntityAction(StoryBoardElement* parent)
            : OSCGlobalAction(ActionType::DELETE_ENTITY, parent),
              entity_(nullptr),
              entities_(nullptr),
              gateway_(nullptr){};

        DeleteEntityAction(Object* entity, StoryBoardElement* parent)
            : OSCGlobalAction(ActionType::DELETE_ENTITY, parent),
              entity_(entity),
              entities_(nullptr),
              gateway_(nullptr){};

        DeleteEntityAction(const DeleteEntityAction& action, StoryBoardElement* parent) : OSCGlobalAction(ActionType::DELETE_ENTITY, parent)
        {
            entity_   = action.entity_;
            entities_ = action.entities_;
            gateway_  = action.gateway_;
        }

        OSCGlobalAction* Copy()
        {
            DeleteEntityAction* new_action = new DeleteEntityAction(*this);
            return new_action;
        }

        void Start(double simTime);
        void Step(double simTime, double dt);

        void SetEntities(Entities* entities)
        {
            entities_ = entities;
        }
        void SetGateway(ScenarioGateway* gateway)
        {
            gateway_ = gateway;
        }

        void print()
        {
        }
    };

    class EnvironmentAction : public OSCGlobalAction
    {
    public:
        OSCEnvironment new_environment_;

        EnvironmentAction(StoryBoardElement* parent) : OSCGlobalAction(ActionType::ENVIRONMENT, parent){};

        EnvironmentAction(const EnvironmentAction& action, StoryBoardElement* parent) : OSCGlobalAction(ActionType::ENVIRONMENT, parent)
        {
            new_environment_ = action.new_environment_;
        }

        OSCGlobalAction* Copy()
        {
            EnvironmentAction* new_action = new EnvironmentAction(*this);
            return new_action;
        }

        std::string Type2Str()
        {
            return "EnvironmentAction";
        };

        void Start(double simTime);
        void Step(double simTime, double dt);
        void SetEnvironment(OSCEnvironment* environment)
        {
            environment_ = environment;
        }
        void print()
        {
            LOG_INFO("EnvironmentAction");
        }

    private:
        OSCEnvironment* environment_;
    };

    class ScenarioReader;
    class ScenarioEngine;

    class TrafficActionContext
    {
    public:
        TrafficActionContext(ScenarioEngine& engine, ScenarioGateway& gateway, ScenarioReader& reader, roadmanager::OpenDrive& odrManager)
            : scenario_engine_(&engine),
              gateway_(&gateway),
              reader_(&reader),
              odrManager_(&odrManager){};
        TrafficActionContext(roadmanager::OpenDrive& odrManager) : odrManager_(&odrManager){};

        ScenarioEngine& GetScenarioEngine()
        {
            return *scenario_engine_;
        };

        ScenarioGateway& GetScenarioGateway()
        {
            return *gateway_;
        };
        ScenarioReader& GetScenarioReader()
        {
            return *reader_;
        };
        roadmanager::OpenDrive& GetOdrManager()
        {
            return *odrManager_;
        };

    private:
        ScenarioEngine*         scenario_engine_{};
        ScenarioGateway*        gateway_{};
        ScenarioReader*         reader_{};
        roadmanager::OpenDrive* odrManager_{};
    };

    class TrafficAction : public OSCGlobalAction
    {
    public:
        TrafficAction(ActionType type, StoryBoardElement* parent, std::shared_ptr<TrafficActionContext> context)
            : OSCGlobalAction(type, parent),
              context_(context)
        {
        }

        ~TrafficAction() = default;

        void SetName(std::string_view name)
        {
            name_ = name;
        }
        void Print()
        {
        }

        /**
         * Tries to spawn an entity at the given position, if there is enough free space.
         * @param pos Position to spawn the entity at.
         */
        void SpawnEntity(roadmanager::Position* pos);

        /**
         * Checks if there is enough free space at the given position to spawn an entity.
         * @param pos Position to check.
         * @return True if there is enough free space, false otherwise.
         */
        bool FreePositionToSpawn(roadmanager::Position* pos);

        /**
         * Despawns the given entity, along with controllers attached.
         * @param object Entity to despawn.
         */
        void DespawnEntity(Object* object);

        protected:
            std::shared_ptr<TrafficActionContext> context_;
            std::string       name_;
            int spawned_count_ = 0;
            std::vector<int> spawned_entity_ids_;
            std::string action_type_;
            VehiclePool                        vehicle_pool_;
            double                             spawn_speed_ = 0.0;
            Entities*              entities_;
    };
    class TrafficSwarmAction : public TrafficAction
    {
    public:
        struct SpawnInfo
        {
            int    vehicleID;
            int    outMidAreaCount;
            id_t   roadID;
            int    lane;
            double simTime;
        };

        typedef struct
        {
            roadmanager::Position pos;
            roadmanager::Road*    road;
            unsigned int          nLanes;
        } SelectInfo;

        TrafficSwarmAction(StoryBoardElement* parent, std::shared_ptr<TrafficActionContext> context)
            : TrafficAction(ActionType::SWARM_TRAFFIC, parent, context)
        {
            spawnedV.clear();
            centralObject_ = action.centralObject_;
        }

        OSCGlobalAction* Copy()
        {
            TrafficSwarmAction* new_action = new TrafficSwarmAction(*this);
            return new_action;
        }

        void Start(double simTime);

        void Step(double simTime, double dt);

        void print()
        {
        }

        void SetCentralObject(Object* centralObj)
        {
            centralObject_ = centralObj;
        }
        void SetInnerRadius(double innerRadius)
        {
            innerRadius_ = innerRadius;
        }
        void SetSemiMajorAxes(double axes)
        {
            semiMajorAxis_ = axes;
        }
        void SetSemiMinorAxes(double axes)
        {
            semiMinorAxis_ = axes;
        }
        void SetScenarioEngine(ScenarioEngine* scenario_engine);

        void SetGateway(ScenarioGateway* gateway)
        {
            gateway_ = gateway;
        }
        void SetReader(ScenarioReader* reader)
        {
            reader_ = reader;
        }
        void SetNumberOfVehicles(int number)
        {
            numberOfVehicles = static_cast<unsigned long>(number);
        }
        void Setvelocity(double velocity)
        {
            speedRange = false;
            velocity_ = velocity;
        }
        void SetInitialSpeedRange(double lowerLimit, double upperLimit)
        {
            initialSpeedLowerLimit_ = lowerLimit;
            initialSpeedUpperLimit_ = upperLimit;
        }
        void SetDirectionOfTravelDistribution(double opposite, double same)
        {
            dot_set_ = true;
            dotOpposite_ = opposite;
            dotSame_     = same;
        }

    private:
        double                 velocity_;
        double                 initialSpeedLowerLimit_;
        double                 initialSpeedUpperLimit_;
        bool                   speedRange = true;
        double                 dotOpposite_;
        double                 dotSame_;
        bool                   dot_set_ = false;
        Entities*              swarm_entities_;
        Object*                centralObject_;
        aabbTree::ptTree       rTree;
        unsigned long          numberOfVehicles;
        std::vector<SpawnInfo> spawnedV;
        double                 innerRadius_, semiMajorAxis_, semiMinorAxis_, midSMjA, midSMnA, minSize_, lastTime;
        static int             counter_;

        int         despawn(double simTime);
        void        createRoadSegments(aabbTree::BBoxVec& vec);
        void        spawn(Solutions sols, int replace, double simTime);
        inline bool ensureDistance(roadmanager::Position pos, int lane, double dist);
        void        createEllipseSegments(aabbTree::BBoxVec& vec, double SMjA, double SMnA);
        inline void sampleRoads(int minN, int maxN, Solutions& sols, vector<SelectInfo>& info);
        double getInitialSpeed() const;
    };

    class TrafficSourceAction : public OSCGlobalAction
    {
    public:
        TrafficSourceAction(StoryBoardElement* parent, std::shared_ptr<TrafficActionContext> context)
            : TrafficAction(ActionType::SOURCE_TRAFFIC, parent, context)
        {
        }

        OSCGlobalAction* Copy()
        {
            TrafficSourceAction* new_action = new TrafficSourceAction(*this);
            return new_action;
        }

        void Start(double simTime);

        void Step(double simTime, double dt);

        void print()
        {
        }

        void SpawnEntity();

        void SetScenarioEngine(ScenarioEngine* scenario_engine);

        void SetGateway(ScenarioGateway* gateway)
        {
            gateway_ = gateway;
        }
        void SetReader(ScenarioReader* reader)
        {
            reader_ = reader;
        }

        void SetActionTriggerTime(double simTime)
        {
            action_trigger_time_ = simTime;
        }
        void SetRadius(double radius)
        {
            radius_ = radius;
        }
        void SetRate(double rate)
        {
            rate_ = rate;
        }
        void SetSpeed(double speed)
        {
            speed_ = speed;
        }
        void SetPosition(roadmanager::Position* pos)
        {
            pos_ = pos;
        }

        /**
         * Get a random position within the spawn radius from the source position.
         * @return A pointer to the random spawn position.
         */
        roadmanager::Position* GetRandomSpawnPosition();

        // std::vector<TrafficDistributionEntry> traffic_distribution_entry_;

    private:
        double                 action_trigger_time_;
        double                 radius_;
        double                 rate_;
        roadmanager::Position* pos_;
    };

    class TrafficSinkAction : public TrafficAction
    {
    public:
        TrafficSinkAction(StoryBoardElement* parent, std::shared_ptr<TrafficActionContext> context)
            : TrafficAction(ActionType::SINK_TRAFFIC, parent, context)
        {
        }

        OSCGlobalAction* Copy()
        {
            return new TrafficSinkAction(*this);
        }

        void Start(double simTime);

        void Step(double simTime, double dt);

        void SetActionTriggerTime(double simTime)
        {
            action_trigger_time_ = simTime;
        }
        void SetRadius(double radius)
        {
            radius_ = radius;
        }
        void SetRate(double rate, bool constantDespawn = false)
        {
            rate_             = rate;
            constant_despawn_ = constantDespawn;
        }
        void SetPosition(roadmanager::Position* pos)
        {
            pos_ = pos;
        }

        /**
         * Despawn entities that are within the sink radius.
         */
        void FindPossibleDespawn();

        /**
         * Check if a given position is inside the sink radius.
         * @param object_pos The position to check.
         * @return True if the position is inside the sink radius, false otherwise.
         */
        bool InsideSink(roadmanager::Position object_pos);

    private:
        double                 action_trigger_time_;
        double                 radius_;
        double                 rate_;
        roadmanager::Position* pos_;
        bool                   constant_despawn_ = true;
        double                 time_accumulator_ = 0.0;
    };

    // helper for floating-point comparison
    inline bool almost_equal(double a, double b, double eps = 1e-9)
    {
        return std::fabs(a - b) < eps;
    }

    struct LaneSegment
    {
        int    roadId;
        int    laneId;
        double minS;
        double maxS;
        double length;
    };

    inline bool operator==(const LaneSegment& a, const LaneSegment& b)
    {
        return a.roadId == b.roadId && a.laneId == b.laneId && almost_equal(a.minS, b.minS) && almost_equal(a.maxS, b.maxS) &&
               almost_equal(a.length, b.length);
    }

    struct RoadCursor
    {
        int              roadId;
        double           s = 0.0;
        std::vector<int> laneIds;
        bool             last = false;
        double           road_length;

        explicit RoadCursor() : roadId(), s(0.0), laneIds{}, last(false), road_length(-1)
        {
        }

        explicit RoadCursor(int id) : roadId(id), s(0.0), laneIds{}, last(false), road_length(-1)
        {
        }

        RoadCursor(int id, double s_, std::vector<int> lanes = {}, bool last_ = false, double len = 0.0)
            : roadId(id),
              s(s_),
              laneIds(std::move(lanes)),
              last(last_),
              road_length(len)
        {
        }
    };

    inline bool operator==(const RoadCursor& a, const RoadCursor& b)
    {
        return a.roadId == b.roadId && almost_equal(a.s, b.s) && a.laneIds == b.laneIds && a.last == b.last &&
               almost_equal(a.road_length, b.road_length);
    }

    struct RoadRange
    {
        double                  length;
        std::vector<RoadCursor> roadCursors;
    };

    inline bool operator==(const RoadRange& a, const RoadRange& b)
    {
        return almost_equal(a.length, b.length) && a.roadCursors == b.roadCursors;
    }

    class TrafficAreaAction : public TrafficAction
    {
    public:
        TrafficAreaAction(StoryBoardElement* parent, std::shared_ptr<TrafficActionContext> context)
            : TrafficAction(ActionType::AREA_TRAFFIC, parent, context)
        {
        }

        OSCGlobalAction* Copy()
        {
            return new TrafficAreaAction(*this);
        }

        void Start(double simTime);

        void Step(double simTime, double dt);

        void SpawnEntities(int number_of_entities_to_spawn);

        void DespawnEntities();

        bool InsideArea(roadmanager::Position object_pos);

        void SetContinuous(bool continuous)
        {
            continuous_ = continuous;
        }

        void SetNumberOfEntities(int number_of_entities)
        {
            number_of_entities_ = number_of_entities;
        }

        void SetPolygonPoints(const std::vector<roadmanager::Position> points)
        {
            polygon_points_ = points;
        }

        void SortPolygonPoints(std::vector<roadmanager::Position>& points);

        void SetRoadRanges(const std::vector<RoadRange> road_ranges)
        {
            road_ranges_ = road_ranges;
        }
        std::vector<RoadRange> GetRoadRanges() const
        {
            return road_ranges_;
        }
        std::vector<LaneSegment> GetLaneSegments() const
        {
            return lane_segments_;
        }

        void UpdateRoadRanges();
        void SetRoadRangeLength(RoadRange& road_range);
        void SetAdditionalRoadCursorInfo(RoadCursor& road_cursor);
        void AddComplementaryRoadCursors();
        void SetLaneSegments(RoadRange& road_range);
        void LaneSegments();
        void LaneSegmentsForRoad(std::vector<RoadCursor> road_cursors_to_road, double& accumulated_length, const double max_length);
        void HandleLastRoadCursor(std::vector<RoadCursor> last_road_cursors, double& accumulated_length, const double max_length);

        roadmanager::Position* GetRandomSpawnPosition();
        roadmanager::Position* pos_;

    private:
        double ClampMax(double value, double accumulated, double max_length);

        bool                               continuous_;
        int                                number_of_entities_;
        std::vector<roadmanager::Position> polygon_points_;
        std::vector<RoadRange>             road_ranges_;
        std::vector<LaneSegment>           lane_segments_;
        bool                               first_spawn_ = false;
    };

    class TrafficStopAction : public TrafficAction
    {
    public:
        TrafficStopAction(StoryBoardElement* parent, std::shared_ptr<TrafficActionContext> context)
            : TrafficAction(ActionType::STOP_TRAFFIC, parent, context)
        {
        }

        OSCGlobalAction* Copy()
        {
            return new TrafficStopAction(*this);
        }

        void Start(double simTime);

        void Step(double simTime, double dt);

        void print()
        {
        }

        void SetTrafficActionToStop(const std::string& action)
        {
            traffic_action_to_stop_ = action;
        }

    private:
        std::string traffic_action_to_stop_;
    };

}  // namespace scenarioengine
