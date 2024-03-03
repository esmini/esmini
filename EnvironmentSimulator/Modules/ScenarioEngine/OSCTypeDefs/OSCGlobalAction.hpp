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
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "OSCAABBTree.hpp"
#include <vector>
#include "OSCUtils.hpp"
#include "OSCPosition.hpp"

namespace scenarioengine
{

    using aabbTree::Solutions;
    using std::vector;
    using namespace Utils;

    class OSCGlobalAction : public OSCAction
    {
    public:
        OSCGlobalAction(OSCAction::ActionType action_type, StoryBoardElement* parent) : OSCAction(action_type, parent)
        {
        }
        virtual ~OSCGlobalAction() = default;

        virtual void print()
        {
            LOG("Virtual, should be overridden");
        }

        virtual OSCGlobalAction* Copy()
        {
            LOG("Virtual, should be overridden");
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

    class AddEntityAction : public OSCGlobalAction
    {
    public:
        Object*                      entity_;
        std::shared_ptr<OSCPosition> pos_OSCPosition_;
        roadmanager::Position*       pos_;
        Entities*                    entities_;

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

    class ScenarioReader;

    class SwarmTrafficAction : public OSCGlobalAction
    {
    public:
        struct SpawnInfo
        {
            int    vehicleID;
            int    outMidAreaCount;
            int    roadID;
            int    lane;
            double simTime;
        };

        typedef struct
        {
            roadmanager::Position pos;
            roadmanager::Road*    road;
            int                   nLanes;
        } SelectInfo;

        SwarmTrafficAction(StoryBoardElement* parent);
        ~SwarmTrafficAction();

        SwarmTrafficAction(const SwarmTrafficAction& action, StoryBoardElement* parent) : OSCGlobalAction(ActionType::SWARM_TRAFFIC, parent)
        {
            spawnedV.clear();
            centralObject_ = action.centralObject_;
        }

        OSCGlobalAction* Copy()
        {
            SwarmTrafficAction* new_action = new SwarmTrafficAction(*this);
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
        void SetEntities(Entities* entities)
        {
            entities_ = entities;
        }
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
            velocity_ = velocity;
        }

    private:
        double                  velocity_;
        Entities*               entities_;
        ScenarioGateway*        gateway_;
        ScenarioReader*         reader_;
        Object*                 centralObject_;
        aabbTree::ptTree        rTree;
        unsigned long           numberOfVehicles;
        std::vector<SpawnInfo>  spawnedV;
        roadmanager::OpenDrive* odrManager_;
        double                  innerRadius_, semiMajorAxis_, semiMinorAxis_, midSMjA, midSMnA, minSize_, lastTime;
        std::vector<Vehicle*>   vehicle_pool_;
        static int              counter_;

        int         despawn(double simTime);
        void        createRoadSegments(aabbTree::BBoxVec& vec);
        void        spawn(Solutions sols, int replace, double simTime);
        inline bool ensureDistance(roadmanager::Position pos, int lane, double dist);
        void        createEllipseSegments(aabbTree::BBoxVec& vec, double SMjA, double SMnA);
        inline void sampleRoads(int minN, int maxN, Solutions& sols, vector<SelectInfo>& info);
    };

}  // namespace scenarioengine
