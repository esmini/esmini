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

#include "OSCGlobalAction.hpp"
#include "Controller.hpp"
#include "OSCSwarmTrafficGeometry.hpp"
#include <memory>
#include <cmath>
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>
#include <numeric>
#include <set>
#include "VehiclePool.hpp"
#include "ControllerACC.hpp"
#include "ScenarioReader.hpp"
#include "ScenarioEngine.hpp"

using namespace scenarioengine;
using namespace STGeometry;
using aabbTree::BBox;
using aabbTree::BBoxVec;
using aabbTree::curve2triangles;
using aabbTree::makeTriangleAndBbx;
using aabbTree::ptBBox;
using aabbTree::ptTriangle;
using std::make_shared;
using std::vector;

#define USELESS_THRESHOLD     5    // Max check count before deleting uneffective vehicles
#define VEHICLE_DISTANCE      7   // Freespace needed to a potential spawn point to allow vehicle spawn
#define SWARM_TIME_INTERVAL   0.1  // Sleep time between update steps
#define SWARM_SPAWN_FREQUENCY 1.1  // Sleep time between spawns
#define MAX_CARS              1000
#define MAX_LANES             32

int TrafficSwarmAction::counter_ = 0;

void EnvironmentAction::Start(double simTime)
{
    environment_->UpdateEnvironment(new_environment_);
    OSCAction::Start(simTime);
}

void EnvironmentAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;
    OSCAction::Stop();
}

void ParameterSetAction::Start(double simTime)
{
    LOG_INFO("Set parameter {} = {}", name_, value_);
    parameters_->setParameterValueByString(name_, value_);
    OSCAction::Start(simTime);
}

void ParameterSetAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::Stop();
}

void VariableSetAction::Start(double simTime)
{
    LOG_INFO("Set variable {} = {}", name_, value_);
    variables_->setParameterValueByString(name_, value_);
    OSCAction::Start(simTime);
}

void VariableSetAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::Stop();
}

void VariableAddAction::Start(double simTime)
{
    OSCParameterDeclarations::ParameterStruct* ps = variables_->getParameterEntry(name_);
    if (!ps)
    {
        return;
    }

    if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        LOG_INFO("Add variable {} += {:.0f}", name_, value_);
        int v = 0;
        variables_->getParameterValueInt(name_, v);
        v += static_cast<int>(value_);
        variables_->setParameterValue(name_, v);
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        LOG_INFO("Add variable {} += {}", name_, value_);
        double v = 0.0;
        variables_->getParameterValueDouble(name_, v);
        v += value_;
        variables_->setParameterValue(name_, v);
    }

    OSCAction::Start(simTime);
}

void VariableAddAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::Stop();
}

void VariableMultiplyByAction::Start(double simTime)
{
    OSCParameterDeclarations::ParameterStruct* ps = variables_->getParameterEntry(name_);
    if (!ps)
    {
        return;
    }

    LOG_INFO("Multiply variable {} *= {}", name_, value_);
    if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        int v = 0;
        variables_->getParameterValueInt(name_, v);
        v = static_cast<int>(v * value_);
        variables_->setParameterValue(name_, v);
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        double v = 0.0;
        variables_->getParameterValueDouble(name_, v);
        v *= value_;
        variables_->setParameterValue(name_, v);
    }

    OSCAction::Start(simTime);
}

void VariableMultiplyByAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::Stop();
}

void AddEntityAction::Start(double simTime)
{
    if (entity_ == nullptr)
    {
        LOG_ERROR("AddEntityAction missing entity");
        return;
    }

    if (entities_->activateObject(entity_) != 0)
    {
        LOG_WARN("AddEntityAction: Entity already active. Skipping action.");
        return;
    }

    entity_->pos_.TeleportTo(pos_);

    LOG_INFO("Added entity {}", entity_->GetName());

    OSCAction::Start(simTime);
}

void AddEntityAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::Stop();
}

void DeleteEntityAction::Start(double simTime)
{
    if (entity_ == nullptr)
    {
        LOG_ERROR("DeleteEntityAction missing entity");
        return;
    }

    if (entities_->deactivateObject(entity_) != 0)
    {
        LOG_WARN("DeleteEntityAction: Entity already deactivated. Skipping action.");
        return;
    }

    gateway_->removeObject(entity_->name_);

    LOG_INFO("Deleted entity {}", entity_->GetName());

    OSCAction::Start(simTime);
}

void DeleteEntityAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;
    OSCAction::Stop();
}

void TrafficAction::SpawnEntity(roadmanager::Position* pos)
{
    EntityWithController ewc;
    int laneID = pos->GetLaneId();

    if (traffic_distribution_entry_.empty())
    {
        // fallback: use vehicle pool
        ewc.object = new Vehicle(*vehicle_pool_.GetRandomVehicle());
    }
    else
    {
        size_t idx = rand() % traffic_distribution_entry_.size();
        ewc        = traffic_distribution_entry_[idx].GetRandomEntity();

        if (ewc.object)
        {
            if (ewc.object->type_ == Object::Type::VEHICLE)
                ewc.object = new Vehicle(*static_cast<Vehicle*>(ewc.object));
            else if (ewc.object->type_ == Object::Type::PEDESTRIAN)
                ewc.object = new Pedestrian(*static_cast<Pedestrian*>(ewc.object));
        }
        else
        {
            LOG_ERROR("TrafficAction: No entity found in traffic distribution entry for {}", name_);
        }
    }

    ewc.object->pos_.SetLanePos(pos->GetTrackId(), laneID, pos->GetS(), 0.0);
    ewc.object->pos_.SetHeadingRelativeRoadDirection(laneID < 0 ? 0.0 : M_PI);
    ewc.object->SetSpeed(spawn_speed_);
    ewc.object->name_ = action_type_ + std::to_string(spawned_count_);
    context_->GetScenarioEngine().entities_.addObject(ewc.object, true);

    // align trailers if entity is a vehicle
    if (ewc.object->type_ == Object::Type::VEHICLE)
    {
        Vehicle* v = static_cast<Vehicle*>(ewc.object);
        if (!v->TowVehicle() && v->TrailerVehicle())
        {
            v->AlignTrailers();
        }
    }

    if (ewc.controller)
    {
        context_->GetScenarioReader().AddController(ewc.controller);
        ewc.object->AssignController(ewc.controller);
        ewc.controller->LinkObject(ewc.object);
        ewc.controller->Activate({ControlActivationMode::ON, ControlActivationMode::OFF, ControlActivationMode::OFF, ControlActivationMode::OFF});
    }

    spawned_entity_ids_.push_back(ewc.object->GetId());
    spawned_count_++;
    LOG_INFO("Spawned entity {} with speed {}", ewc.object->GetName(), ewc.object->GetSpeed());
}

bool TrafficAction::FreePositionToSpawn(roadmanager::Position* pos)
{
    entities_        = context_->GetScenarioEngine().entities_;
    double x = pos->GetX();
    double y = pos->GetY();
    double latDist;
    double longDist;
    double freespacedistance;

    for (auto& entity : entities_.object_)
    {
        freespacedistance = entity->FreeSpaceDistancePoint(x, y, &latDist, &longDist);
        if (freespacedistance < VEHICLE_DISTANCE)
        {
            return false;
        }
    }

    return true;
}

void TrafficAction::DespawnEntity(Object* object)
{
    LOG_INFO("Despawning entity {}", (object)->name_);

    for (auto& ctrl : (object)->controllers_)
    {
        object->UnassignController(ctrl);
        ctrl->UnlinkObject();
        context_->GetScenarioEngine().scenarioReader->RemoveController(ctrl);
    }
    context_->GetScenarioEngine().entities_.removeObject(object->GetId());
    context_->GetScenarioGateway().removeObject(object->GetId());
}

void TrafficSwarmAction::Start(double simTime)
{
    LOG_INFO("Swarm IR: {:.2f}, SMjA: {:.2f}, SMnA: {:.2f}, maxV: {}, initialSpeedLowerLimit: {:.2f}, initialSpeedUpperLimit: {:.2f}",
             innerRadius_,
             semiMajorAxis_,
             semiMinorAxis_,
             numberOfVehicles,
             initialSpeedLowerLimit_,
             initialSpeedUpperLimit_);
    double x0, y0, x1, y1;
    entities_        = &context_->GetScenarioEngine().entities_;
    spawnedV.clear();

    midSMjA  = (semiMajorAxis_ + innerRadius_) / 2.0;
    midSMnA  = (semiMinorAxis_ + innerRadius_) / 2.0;
    lastTime = -1;

    paramEllipse(0, 0, 0, midSMjA, midSMnA, 0, x0, y0);
    paramEllipse(M_PI / 36, 0, 0, midSMjA, midSMnA, 0, x1, y1);

    minSize_    = ceil(sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) * 100) / 100.0;
    if (minSize_ == 0)
        minSize_ = 1.0;

    aabbTree::ptTree  tree = std::make_shared<aabbTree::Tree>();
    aabbTree::BBoxVec vec;
    vec.clear();
    createRoadSegments(vec);

    tree->build(vec);
    rTree = tree;

    // Register model filenames from vehicle catalog
    // if no catalog loaded, use same model as central object
    vehicle_pool_.Initialize(&context_->GetScenarioReader(), nullptr, true);

    if (vehicle_pool_.GetVehicles().size() == 0)
    {
        if (centralObject_ && centralObject_->type_ == Object::Type::VEHICLE)
        {
            // Create a copy of central object
            Vehicle* vehicle = new Vehicle(*static_cast<Vehicle*>(centralObject_));

            // remove any duplicate controller references
            vehicle->controllers_.clear();

            vehicle_pool_.AddVehicle(vehicle);
        }
        else
        {
            LOG_ERROR("TrafficSwarmAction: No vehicles available to populate swarm traffic. Missing both Vehicle catalog and central vehicle object");
        }
    }

    OSCAction::Start(simTime);
}

void TrafficSwarmAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    // Executes the step at each TIME_INTERVAL
    if (lastTime < 0 || abs(simTime - lastTime) > SWARM_TIME_INTERVAL)
    {
        double SMjA = midSMjA;
        double SMnA = midSMnA;

        BBoxVec                 vec;
        aabbTree::Candidates    candidates;
        std::vector<ptTriangle> triangle;
        Solutions               sols;
        vec.clear();
        candidates.clear();
        triangle.clear();
        sols.clear();

        EllipseInfo info = {SMjA, SMnA, centralObject_->pos_};

        createEllipseSegments(vec, SMjA, SMnA);
        aabbTree::Tree eTree;
        eTree.build(vec);
        rTree->intersect(eTree, candidates);
        aabbTree::processCandidates(candidates, triangle);
        aabbTree::findPoints(triangle, info, sols);

        spawn(sols, despawn(simTime), simTime);
        lastTime = simTime;
    }
}

void TrafficSwarmAction::createRoadSegments(BBoxVec& vec)
{
    for (unsigned int i = 0; i < context_->GetOdrManager().GetNumOfRoads(); i++)
    {
        roadmanager::Road* road = context_->GetOdrManager().GetRoadByIdx(i);

        for (unsigned int j = 0; j < road->GetNumberOfGeometries(); j++)
        {
            roadmanager::Geometry* gm = road->GetGeometry(j);

            switch (gm->GetType())
            {
                case roadmanager::Geometry::GeometryType::GEOMETRY_TYPE_UNKNOWN:
                {
                    break;
                }
                case roadmanager::Geometry::GeometryType::GEOMETRY_TYPE_LINE:
                {
                    auto const length = gm->GetLength();
                    for (double dist = gm->GetS(); dist < length;)
                    {
                        double ds = dist + minSize_;
                        if (ds > length)
                            ds = length;
                        double x0, y0, x1, y1, x2, y2, dummy, l;
                        gm->EvaluateDS(dist, &x0, &y0, &dummy);
                        gm->EvaluateDS(ds, &x1, &y1, &dummy);
                        l  = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
                        x2 = (x1 + x0) / 2 + l / 4.0;
                        y2 = (y1 + y0) / 2 + l / 4.0;
                        Point      a(x0, y0), b(x1, y1), c(x2, y2);
                        ptTriangle triangle = make_shared<Triangle>(gm);
                        triangle->a         = a;
                        triangle->b         = b;
                        triangle->c         = c;
                        triangle->sI        = dist;
                        triangle->sF        = ds;
                        ptBBox bbox         = make_shared<BBox>(triangle);
                        vec.push_back(bbox);
                        dist = ds;
                    }
                    break;
                }
                default:
                {
                    curve2triangles(gm, minSize_, M_PI / 36, vec);
                    break;
                }
            }
        }
    }
}

void TrafficSwarmAction::createEllipseSegments(aabbTree::BBoxVec& vec, double SMjA, double SMnA)
{
    double alpha  = -M_PI / 72.0;
    double dAlpha = M_PI / 36.0;
    auto   pos    = centralObject_->pos_;
    double x0, y0, x1, y1, x2, y2;

    while (alpha < (2 * M_PI - M_PI / 72.0))
    {
        double da = alpha + dAlpha;
        if (da > 2 * M_PI - M_PI / 72.0)
        {
            da = 2 * M_PI - M_PI / 72.0;
        }

        paramEllipse(alpha, pos.GetX(), pos.GetY(), SMjA, SMnA, pos.GetH(), x0, y0);
        paramEllipse(da, pos.GetX(), pos.GetY(), SMjA, SMnA, pos.GetH(), x1, y1);

        double theta0, theta1;
        theta0 = angleTangentEllipse(SMjA, SMnA, alpha, pos.GetH());
        theta1 = angleTangentEllipse(SMjA, SMnA, da, pos.GetH());

        tangentIntersection(x0, y0, alpha, theta0, x1, y1, da, theta1, x2, y2);

        ptBBox bbx = makeTriangleAndBbx(x0, y0, x1, y1, x2, y2);
        vec.push_back(bbx);

        alpha = da;
    }
}

inline void TrafficSwarmAction::sampleRoads(int minN, int maxN, Solutions& sols, vector<SelectInfo>& info)
{
    // printf("Entered road selection\n");
    // printf("Min: %d, Max: %d\n", minN, maxN);

    // Sample the number of cars to spawn
    if (maxN < minN)
    {
        LOG_ERROR("Unstable behavior detected (maxN < minN)");
        return;
    }

    unsigned int nCarsToSpawn = static_cast<unsigned int>(SE_Env::Inst().GetRand().GetNumberBetween(minN, maxN - 1));
    if (nCarsToSpawn == 0)
    {
        return;
    }

    info.reserve(nCarsToSpawn);
    info.clear();
    // We have more points than number of vehicles to spawn.
    // We sample the selected number and each point will be assigned a lane
    if (nCarsToSpawn <= sols.size() && nCarsToSpawn > 0)
    {
        // Shuffle and randomly select the points
        // Solutions selected(nCarsToSpawn);
        static Point selected[MAX_CARS];  // Remove macro when/if found a solution for dynamic array
        std::shuffle(sols.begin(), sols.end(), SE_Env::Inst().GetRand().GetGenerator());
        sample(sols.begin(), sols.end(), selected, nCarsToSpawn, SE_Env::Inst().GetRand().GetGenerator());

        for (unsigned int i = 0; i < nCarsToSpawn; i++)
        {
            Point& pt = selected[i];
            // Find road
            roadmanager::Position pos(pt.x, pt.y, 0.0, pt.h, 0.0, 0.0);
            if (pos.IsInJunction())
            {
                continue;  // avoid put vehicles in the middle of junctions
            }
            // pos.XYZH2TrackPos(pt.x, pt.y, 0, pt.h);
            // Peek road
            roadmanager::Road* road = context_->GetOdrManager().GetRoadById(pos.GetTrackId());
            if (road->GetNumberOfDrivingLanes(pos.GetS()) == 0)
                continue;
            // Since the number of points is equal to the number of vehicles to spaw,
            // only one lane is selected
            SelectInfo sInfo = {pos, road, 1};
            info.push_back(sInfo);
        }
    }
    else
    {  // Less points than vehicles to spawn
        // We use all the spawnable points and we ensure that each obtains
        // a lane at least. The remaining ones will be randomly distributed.
        // The algorithms does not ensure to saturate the selected number of vehicles.
        unsigned int lanesLeft = nCarsToSpawn - static_cast<unsigned int>(sols.size());
        for (Point pt : sols)
        {
            roadmanager::Position pos(pt.x, pt.y, 0.0, pt.h, 0.0, 0.0);
            // pos.XYZH2TrackPos(pt.x, pt.y, 0, pt.h);

            roadmanager::Road* road          = context_->GetOdrManager().GetRoadById(pos.GetTrackId());
            unsigned int       nDrivingLanes = road->GetNumberOfDrivingLanes(pos.GetS());
            if (nDrivingLanes == 0)
            {
                lanesLeft++;
                continue;
            }

            unsigned int lanesN;
            if (lanesLeft > 0)
            {
                std::uniform_int_distribution<unsigned int> laneDist(0, std::min(lanesLeft, nDrivingLanes));
                lanesN = laneDist(SE_Env::Inst().GetRand().GetGenerator());
                lanesN = (lanesN == 0 ? 0 : lanesN - 1);
            }
            else
            {
                lanesN = 0;
            }

            SelectInfo sInfo = {pos, road, 1 + lanesN};
            info.push_back(sInfo);
            lanesLeft -= lanesN;
        }
    }
}

void TrafficSwarmAction::spawn(Solutions sols, int replace, double simTime)
{
    int maxCars = static_cast<int>(
        MIN(MAX_CARS, static_cast<unsigned int>(numberOfVehicles) - spawnedV.size()));  // Remove MIN check when/if found a solution for dynamic array
    if (maxCars <= 0)
    {
        return;
    }

    vector<SelectInfo> info;
    sampleRoads(replace, maxCars, sols, info);

    for (SelectInfo inf : info)
    {
        unsigned int        lanesNo = MIN(MAX_LANES, inf.road->GetNumberOfDrivingLanes(inf.pos.GetS()));
        static unsigned int elements[MAX_LANES];
        std::iota(elements, elements + lanesNo, 0);

        static idx_t lanes[MAX_LANES];

        sample(elements, elements + lanesNo, lanes, MIN(MAX_LANES, inf.nLanes), SE_Env::Inst().GetRand().GetGenerator());

        for (unsigned int i = 0; i < MIN(MAX_LANES, inf.nLanes); i++)
        {
            auto Lane = inf.road->GetDrivingLaneByIdx(inf.pos.GetS(), lanes[i]);
            int  laneID;

            double vel = getInitialSpeed();

            if (!Lane)
            {
                LOG_WARN("Warning: invalid lane index");
                continue;
            }
            else
            {
                laneID = Lane->GetId();
            }

            if (!ensureDistance(inf.pos, laneID, MIN(MAX(40.0, vel * 2.0), 0.7 * semiMajorAxis_)))
                continue;  // distance = speed * 2 seconds

            Controller::InitArgs args;
            args.name            = "Swarm ACC controller";
            args.type            = CONTROLLER_ACC_TYPE_NAME;
            args.scenario_engine = &context_->GetScenarioEngine();
            args.gateway         = &context_->GetScenarioGateway();
            args.parameters      = 0;
            args.properties      = 0;

#if 0  // This is one way of setting the ACC setSpeed property
            args.properties = new OSCProperties();
            OSCProperties::Property property;
            property.name_ = "setSpeed";
            property.value_ = std::to_string(vel);
            args.properties->property_.push_back(property);
#endif
            Controller* acc = InstantiateControllerACC(&args);

#if 1  // This is another way of setting the ACC setSpeed property
            (static_cast<ControllerACC*>(acc))->SetSetSpeed(vel);
#endif
            context_->GetScenarioReader().AddController(acc);

            // Pick random model from vehicle catalog
            Vehicle* vehicle = new Vehicle(*vehicle_pool_.GetRandomVehicle());
            vehicle->pos_.SetLanePos(inf.pos.GetTrackId(), laneID, inf.pos.GetS(), 0.0);

            // Set swarm traffic direction based on RHT or LHT
            if (inf.road->GetRule() == roadmanager::Road::RoadRule::RIGHT_HAND_TRAFFIC)
            {
                vehicle->pos_.SetHeadingRelativeRoadDirection(laneID < 0 ? 0.0 : M_PI);
            }
            else if (inf.road->GetRule() == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC)
            {
                vehicle->pos_.SetHeadingRelativeRoadDirection(laneID > 0 ? 0.0 : M_PI);
            }
            else
            {
                // do something if undefined... maybe default to RHT?
                vehicle->pos_.SetHeadingRelativeRoadDirection(laneID < 0 ? 0.0 : M_PI);
            }

            vehicle->SetSpeed(vel);
            // vehicle->scaleMode_ = EntityScaleMode::BB_TO_MODEL;
            vehicle->name_ = "swarm_" + std::to_string(counter_++);
            int id         = entities_->addObject(vehicle, true);

            // align trailers
            Vehicle* v = vehicle;
            if (!v->TowVehicle() && v->TrailerVehicle())
            {
                v->AlignTrailers();
            }

            vehicle->AssignController(acc);
            acc->LinkObject(vehicle);
            acc->Activate({ControlActivationMode::ON, ControlActivationMode::OFF, ControlActivationMode::OFF, ControlActivationMode::OFF});

            SpawnInfo sInfo = {
                id,                    // Vehicle ID
                0,                     // Useless detection counter
                inf.pos.GetTrackId(),  // Road ID
                laneID,                // Lane
                simTime                // Simulation time
            };
            spawnedV.push_back(sInfo);
        }
    }
}

inline bool TrafficSwarmAction::ensureDistance(roadmanager::Position pos, int lane, double dist)
{
    for (SpawnInfo info : spawnedV)
    {
        Object* vehicle = entities_->GetObjectById(info.vehicleID);

        // First apply minimal radius filter to avoid vehicles appear too close, e.g. next to each other in neighbor lanes
        if (PointDistance2D(pos.GetX(), pos.GetY(), vehicle->pos_.GetX(), vehicle->pos_.GetY()) < 20)
        {
            return false;
        }

        pos.SetLaneId(lane);
        roadmanager::PositionDiff posDiff;
        if (pos.Delta(&vehicle->pos_, posDiff, true, 100.0))  // potentially expensive since trying to resolve path between vehicles...
        {
            // If close and in same lane -> NOK
            if (posDiff.dLaneId == 0 && fabs(posDiff.ds) < dist)
            {
                return false;
            }
        }
    }

    return true;
}

int TrafficSwarmAction::despawn(double simTime)
{
    (void)simTime;
    auto infoPtr       = spawnedV.begin();
    bool increase      = true;
    bool deleteVehicle = false;
    int  count         = 0;

    roadmanager::Position cPos = centralObject_->pos_;

    while (infoPtr != spawnedV.end())
    {
        Object* vehicle = entities_->GetObjectById(infoPtr->vehicleID);

        if (vehicle->IsOffRoad() || vehicle->IsEndOfRoad())
        {
            deleteVehicle = true;
        }
        else
        {
            roadmanager::Position vPos = vehicle->pos_;
            auto                  e0   = ellipse(cPos.GetX(), cPos.GetY(), cPos.GetH(), semiMajorAxis_, semiMinorAxis_, vPos.GetX(), vPos.GetY());
            auto                  e1   = ellipse(cPos.GetX(), cPos.GetY(), cPos.GetH(), midSMjA, midSMnA, vPos.GetX(), vPos.GetY());

            if (e0 > 0.001)  // outside major ellipse
            {
                deleteVehicle = true;
            }
            else if (e1 > 0.001 || (0 <= e1 && e1 <= 0.001))  // outside middle ellipse or on the border
            {
                infoPtr->outMidAreaCount++;
                if (infoPtr->outMidAreaCount > USELESS_THRESHOLD)
                {
                    deleteVehicle = true;
                }
            }
            else
            {
                infoPtr->outMidAreaCount = 0;
            }
        }

        if (deleteVehicle)
        {
            for (auto ctrl : vehicle->controllers_)
            {
                vehicle->UnassignController(ctrl);
                ctrl->UnlinkObject();
                context_->GetScenarioReader().RemoveController(ctrl);
            }

            if (vehicle->type_ == Object::Type::VEHICLE)
            {
                Vehicle* v       = static_cast<Vehicle*>(vehicle);
                Vehicle* trailer = nullptr;
                while (v)  // remove all linked trailers
                {
                    trailer = static_cast<Vehicle*>(v->TrailerVehicle());

                    context_->GetScenarioGateway().removeObject(v->name_);

                    if (v->objectEvents_.size() > 0 || v->initActions_.size() > 0)
                    {
                        entities_->deactivateObject(v);
                    }
                    else
                    {
                        entities_->removeObject(v, false);
                    }
                    v = trailer;

                    vehicle = nullptr;  // indicate vehicle removed
                }
            }

            if (vehicle)
            {
                context_->GetScenarioGateway().removeObject(vehicle->name_);
                if (vehicle->objectEvents_.size() > 0 || vehicle->initActions_.size() > 0)
                {
                    entities_->deactivateObject(vehicle);
                }
                else
                {
                    entities_->removeObject(vehicle, false);
                }
            }

            infoPtr  = spawnedV.erase(infoPtr);
            increase = deleteVehicle = false;
            count++;
        }

        if (increase)
        {
            ++infoPtr;
        }

        increase = true;
    }

    return count;
}

double TrafficSwarmAction::getInitialSpeed() const
{
    if (!speedRange)
    {
        return velocity_;
    }
    return SE_Env::Inst().GetRand().GetRealBetween(initialSpeedLowerLimit_, initialSpeedUpperLimit_);
}

EntityWithController TrafficDistributionEntry::GetRandomEntity() const
{
    if (entityDistribution.entries.empty())
    {
        return {nullptr, nullptr};
    }

    // Compute total weight of all entity entries
    double totalWeight = std::accumulate(entityDistribution.entries.begin(),
                                         entityDistribution.entries.end(),
                                         0.0,
                                         [](double sum, const EntityDistributionEntry& ede) { return sum + ede.weight; });

    double pick = SE_Env::Inst().GetRand().GetRealBetween(0.0, totalWeight);

    for (const auto& ede : entityDistribution.entries)
    {
        if (pick <= ede.weight)
        {
            Controller* ctrl = ede.controllers.empty() ? nullptr : ede.controllers[0];
            return {ede.object, ctrl};  // Return pointer to the actual vehicle
        }
        pick -= ede.weight;
    }

    // Defensive: return last entry's object/controller
    const auto& ede  = entityDistribution.entries.back();
    Controller* ctrl = ede.controllers.empty() ? nullptr : ede.controllers[0];
    return {ede.object, ctrl};  // Defensive: return last entry's vehicle if not found
}

void TrafficSourceAction::Start(double simTime)
{
    LOG_INFO("Traffic Source Radius: {:.2f}, Rate: {:.2f}, Speed: {:.2f}", radius_, rate_, spawn_speed_);
    action_type_ = "source_";

    SetActionTriggerTime(simTime);

    // Should be exchanged for the vehicle definition/distribution
    vehicle_pool_.Initialize(&context_->GetScenarioReader(), nullptr, true);

    if (vehicle_pool_.GetVehicles().size() == 0)
    {
        LOG_ERROR("TrafficSwarmAction: No vehicles available to populate swarm traffic. Missing both Vehicle catalog and central vehicle object");
    }

    OSCAction::Start(simTime);
}

void TrafficSourceAction::Step(double simTime, double dt)
{
    (void)dt;

    double next_spawn_time = action_trigger_time_ + spawned_count_ / rate_;
    if (simTime >= next_spawn_time) {
        SpawnEntity(GetRandomSpawnPosition());
    }
}

roadmanager::Position* TrafficSourceAction::GetRandomSpawnPosition()
{
    double x = pos_->GetX();
    double y = pos_->GetY();

    double angle  = SE_Env::Inst().GetRand().GetRealBetween(0, 2 * M_PI);
    double radius = SE_Env::Inst().GetRand().GetRealBetween(0, 1) * radius_;

    double newX = x + radius * std::cos(angle);
    double newY = y + radius * std::sin(angle);

    return new roadmanager::Position(newX, newY, 0.0, 0.0, 0.0, 0.0);
}

void TrafficSinkAction::Start(double simTime)
{
    LOG_INFO("Traffic Source Radius: {:.2f}, Rate: {:.2f}", radius_, rate_);

    SetActionTriggerTime(simTime);

    OSCAction::Start(simTime);
}

void TrafficSinkAction::Step(double simTime, double dt)
{
    if (constant_despawn_)
    {
        FindPossibleDespawn();
        return;
    }

    time_accumulator_ += dt;
    double interval = 1.0 / rate_;  // seconds per vehicle

    while (time_accumulator_ >= interval)
    {
        FindPossibleDespawn();
        time_accumulator_ -= interval;
    }
}

bool TrafficSinkAction::InsideSink(roadmanager::Position object_pos)
{
    double dx       = object_pos.GetX() - pos_->GetX();
    double dy       = object_pos.GetY() - pos_->GetY();
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance <= radius_;
}

void TrafficSinkAction::FindPossibleDespawn()
{
    auto it = std::find_if(context_->GetScenarioEngine().entities_.object_.begin(),
                           context_->GetScenarioEngine().entities_.object_.end(),
                           [this](Object* obj) { return InsideSink(obj->pos_); });

    if (it != context_->GetScenarioEngine().entities_.object_.end())
    {
        DespawnEntity(*it);
    }
}

void TrafficAreaAction::Start(double simTime)
{
    LOG_INFO("Traffic Area Continuous: {}, Number of entities: {}", continuous_, number_of_entities_);

    // Should be exchanged for the vehicle definition/distribution
    vehicle_pool_.Initialize(&context_->GetScenarioReader(), nullptr, true);

    action_type_ = "area_";

    if (!polygon_points_.empty())
    {
        SortPolygonPoints(polygon_points_);
    }
    if (!road_ranges_.empty())
    {
        UpdateRoadRanges();
    }

    OSCAction::Start(simTime);
}

void TrafficAreaAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    if (!continuous_)
    {
        SpawnEntities(number_of_entities_);

        OSCAction::Stop();
    }
    else
    {
        DespawnEntities();
        SpawnEntities(number_of_entities_ - spawned_entity_ids_.size());
    }
}

// Sorts polygon points in clockwise order
void TrafficAreaAction::SortPolygonPoints(std::vector<roadmanager::Position>& points)
{
    // Compute centroid
    double cx = 0.0, cy = 0.0;
    for (const auto& pt : points)
    {
        cx += pt.GetX();
        cy += pt.GetY();
    }
    cx /= points.size();
    cy /= points.size();

    // Sort by angle from centroid
    std::sort(points.begin(),
              points.end(),
              [cx, cy](const roadmanager::Position& a, const roadmanager::Position& b)
              {
                  double angleA = std::atan2(a.GetY() - cy, a.GetX() - cx);
                  double angleB = std::atan2(b.GetY() - cy, b.GetX() - cx);
                  return angleA > angleB;
              });
}

void TrafficAreaAction::UpdateRoadRanges()
{
    for (auto& rr : road_ranges_)
    {
        for (auto& rc : rr.roadCursors)
        {
            SetAdditionalRoadCursorInfo(rc);
        }
        SetRoadRangeLength(rr);
        SetLaneSegments(rr);
    }
    // AddComplementaryRoadCursors();
    
}

void TrafficAreaAction::SetRoadRangeLength(RoadRange& road_range)
{
        std::unordered_set<int> road_ids_in_range;
        for (const auto& rc : road_range.roadCursors)
        {
            road_ids_in_range.insert(rc.roadId);
        }

        if (road_ids_in_range.size() == 1)
        {
            double minS = std::numeric_limits<double>::max();
            double maxS = 0;
            for (const auto& rc : road_range.roadCursors)
            {
                if (rc.s < minS)
                    minS = rc.s;
                if (rc.s > maxS)
                    maxS = rc.s;
            }
            road_range.length = road_range.length == 0 ? maxS - minS : std::min(road_range.length, maxS - minS);
        }
        else
        {
            double total_length = 0.0;
            for (const auto& road_id : road_ids_in_range)
            {
                double minS = std::numeric_limits<double>::max();
                double maxS;
                double lastS = 0.0;
                for (const auto& rc : road_range.roadCursors)
                {
                    if (rc.roadId == road_id)
                    {
                        maxS = rc.road_length;
                        if (rc.s < minS)
                            minS = rc.s;
                        if (rc.last)
                            lastS = maxS - rc.s;
                    }
                }
                total_length += maxS - minS - lastS;
            }
            road_range.length = road_range.length == 0 ? total_length : std::min(road_range.length, total_length);
        }
}

void TrafficAreaAction::SetAdditionalRoadCursorInfo(RoadCursor& road_cursor)
{
    roadmanager::Road* road = context_->GetOdrManager().GetRoadById(road_cursor.roadId);
    if (!road)
    {
        LOG_ERROR("TrafficAreaAction: Road ID '{}' not found in OpenDRIVE data", road_cursor.roadId);
        return;
    }

    road_cursor.road_length = road->GetLength();

    // Clamp s
    double old_s  = road_cursor.s;
    road_cursor.s = std::clamp(road_cursor.s, 0.0, road_cursor.road_length);
    if (road_cursor.s != old_s)
    {
        LOG_WARN("TrafficAreaAction: RoadCursor s value {} was clamped to {} for road ID {}.", old_s, road_cursor.s, road_cursor.roadId);
    }

    // Collect driving lanes
    std::vector<int> lane_ids;
    auto             section = road->GetLaneSectionByS(road_cursor.s);
    if (!section)
    {
        LOG_ERROR("TrafficAreaAction: No lane section found for road ID {} at s={}", road_cursor.roadId, road_cursor.s);
        return;
    }

    lane_ids.reserve(road->GetNumberOfLanes(road_cursor.s));
    for (unsigned int i = 0; i < road->GetNumberOfLanes(road_cursor.s); i++)
    {
        roadmanager::Lane* lane = section->GetLaneByIdx(i);
        if (!lane)
        {
            LOG_ERROR("TrafficAreaAction: Lane index {} not found in road ID {} at s={}", i, road_cursor.roadId, road_cursor.s);
            continue;
        }
        if (lane->GetLaneType() == roadmanager::Lane::LaneType::LANE_TYPE_DRIVING)
        {
            lane_ids.push_back(lane->GetId());
        }
    }

    // Sync laneIds
    if (road_cursor.laneIds.empty())
    {
        road_cursor.laneIds = lane_ids;
    }
    else
    {
        std::set<int> allowed(lane_ids.begin(), lane_ids.end());
        bool          all_present = true;

        for (int id : road_cursor.laneIds)
        {
            if (allowed.find(id) == allowed.end())
            {
                all_present = false;
                break;
            }
        }

        if (!all_present)
        {
            LOG_WARN("TrafficAreaAction: Some lane IDs in RoadCursor not found in road ID {} at s={}. Updating lane IDs to match road data.",
                     road_cursor.roadId,
                     road_cursor.s);
            road_cursor.laneIds = lane_ids;
        }
    }
}

// [hlindst9] Since we desided to "force" the user to specify all possible road cursors, this is not needed
// As a safety, however, let's keep it for now
void TrafficAreaAction::AddComplementaryRoadCursors()
{
    std::vector<std::pair<id_t, std::string>> all_road_ids = context_->GetOdrManager().GetRoadIds();

    std::vector<int> full_road_id_list(all_road_ids.size());
    std::transform(all_road_ids.begin(),
                   all_road_ids.end(),
                   full_road_id_list.begin(),
                   [](const std::pair<id_t, std::string>& id_pair) { return id_pair.first; });

    for (auto& rr : road_ranges_)
    {
        int current_road_id = rr.roadCursors.front().roadId;
        int last_road_id    = rr.roadCursors.back().roadId;

        std::unordered_set<int> road_ids_in_range;
        for (const auto& rc : rr.roadCursors)
        {
            road_ids_in_range.insert(rc.roadId);
        }

        // If there is only one road, it's containing first and last road cursor, and no extra cursor will be needed
        if (road_ids_in_range.size() > 1)
        {
            bool                    last_found = false;
            std::unordered_set<int> visited_roads;
            bool                    stuck = false;
            while (!last_found && !stuck)
            {
                bool progressed = false;
                for (const auto& next_road_id : full_road_id_list)
                {
                    if (visited_roads.count(next_road_id))
                        continue;

                    roadmanager::Road* current_road = context_->GetOdrManager().GetRoadById(current_road_id);
                    roadmanager::Road* next_road    = context_->GetOdrManager().GetRoadById(next_road_id);
                    if (!current_road->IsSuccessor(next_road))
                    {
                        continue;
                    }

                    auto       it      = std::max_element(rr.roadCursors.begin(),
                                               rr.roadCursors.end(),
                                               [current_road_id](const RoadCursor& a, const RoadCursor& b)
                                               {
                                                   if (a.roadId != current_road_id)
                                                       return true;
                                                   if (b.roadId != current_road_id)
                                                       return false;
                                                   return a.s < b.s;
                                               });
                    RoadCursor last_rc = (it != rr.roadCursors.end()) ? *it : RoadCursor{};

                    // Find all road cursors to next_road_id
                    std::vector<RoadCursor> road_cursors_to_next_road;
                    std::copy_if(rr.roadCursors.begin(),
                                 rr.roadCursors.end(),
                                 std::back_inserter(road_cursors_to_next_road),
                                 [next_road_id](const RoadCursor& rc) { return rc.roadId == next_road_id; });

                    if (std::none_of(road_cursors_to_next_road.begin(),
                                     road_cursors_to_next_road.end(),
                                     [](const RoadCursor& rc) { return rc.s == 0.0; }))
                    {
                        // No road cursor to this road, add one at s=0.0 for full road, same lanes as previous roads lasr rc
                        last_rc.roadId      = next_road_id;
                        last_rc.s           = 0.0;
                        last_rc.last        = false;
                        last_rc.road_length = next_road->GetLength();
                        rr.roadCursors.push_back(last_rc);
                    }
                    else if (next_road_id == last_road_id && road_cursors_to_next_road.size() == 1)
                    {
                        // Special case, if last road cursor is alone on last road, move last to end of road
                        auto last_it = std::find_if(rr.roadCursors.begin(),
                                                    rr.roadCursors.end(),
                                                    [next_road_id](const RoadCursor& rc) { return rc.roadId == next_road_id && rc.s == 0; });
                        if (last_it != rr.roadCursors.end())
                        {
                            last_it->s = next_road->GetLength();
                        }

                        // Also add a cursor at s=0 for last road
                        last_rc.roadId      = next_road_id;
                        last_rc.s           = 0.0;
                        last_rc.last        = false;
                        last_rc.road_length = next_road->GetLength();
                        rr.roadCursors.push_back(last_rc);
                    }

                    if (next_road_id == last_road_id)
                    {
                        last_found = true;
                        break;
                    }
                    else
                    {
                        current_road_id = next_road_id;
                        visited_roads.insert(next_road_id);
                        progressed = true;
                        break;
                    }
                }

                if (!progressed && !last_found)
                {
                    // No new road found in this iteration, so we're stuck
                    stuck = true;
                    LOG_ERROR("Could not find a path to last_road_id {} after visiting all roads.", last_road_id);
                }
            }
        }
    }
}

void TrafficAreaAction::SetLaneSegments(RoadRange& road_range)
{
    std::unordered_set<int> road_ids_set;
    for (const auto& rc : road_range.roadCursors)
    {
        road_ids_set.insert(rc.roadId);
    }

    // Sorting the road IDs, but if they are not in order this would not work, need update
    std::vector<int> road_ids(road_ids_set.begin(), road_ids_set.end());
    std::sort(road_ids.begin(), road_ids.end());

    std::vector<RoadCursor> road_cursors_to_road;
    double                  accumulated_length = 0.0;
    for (const auto& road_id : road_ids)
    {
        std::copy_if(road_range.roadCursors.begin(),
                     road_range.roadCursors.end(),
                     std::back_inserter(road_cursors_to_road),
                     [road_id](const RoadCursor& rc) { return rc.roadId == road_id; });

        LaneSegmentsForRoad(road_cursors_to_road, accumulated_length, road_range.length);
        if (accumulated_length >= road_range.length)
        {
            break;
        }
        road_cursors_to_road.clear();
    }
}

void TrafficAreaAction::LaneSegmentsForRoad(std::vector<RoadCursor> road_cursors_to_road, double& accumulated_length, const double max_length)
{
    if (std::any_of(road_cursors_to_road.begin(), road_cursors_to_road.end(), [](const RoadCursor& rc) { return rc.last; }))
    {
        HandleLastRoadCursor(road_cursors_to_road, accumulated_length, max_length);
        return;
    }

    double add_length;

    if (road_cursors_to_road.size() == 1)
    {
        for (const auto& laneId : road_cursors_to_road[0].laneIds)
        {
            LaneSegment ls;
            ls.roadId = road_cursors_to_road[0].roadId;
            ls.laneId = laneId;
            ls.minS   = road_cursors_to_road[0].s;
            ls.maxS   = std::min(road_cursors_to_road[0].road_length, max_length - accumulated_length);
            ls.length = ls.maxS - ls.minS;
            lane_segments_.push_back(ls);
            add_length = ls.length;
        }
        accumulated_length += add_length;
    }
    else
    {
        // Sort the road cursors by their s value
        std::sort(road_cursors_to_road.begin(), road_cursors_to_road.end(), [](const RoadCursor& a, const RoadCursor& b) { return a.s < b.s; });

        for (size_t i = 0; i < road_cursors_to_road.size() - 1; ++i)
        {
            const auto& current = road_cursors_to_road[i];
            const auto& next    = road_cursors_to_road[i + 1];

            for (const auto& laneId : current.laneIds)
            {
                LaneSegment ls;
                ls.roadId = current.roadId;
                ls.laneId = laneId;
                ls.minS   = current.s;
                ls.maxS   = next.s > max_length - accumulated_length ? ls.minS + (max_length - accumulated_length) : next.s;
                ls.length = ls.maxS - ls.minS;
                lane_segments_.push_back(ls);
                add_length = ls.length;
            }
            accumulated_length += add_length;
            if (accumulated_length >= max_length)
            {
                return;
            }
        }

        // Handle the last segment to the end of the road
        const auto& last = road_cursors_to_road.back();
        for (const auto& laneId : last.laneIds)
        {
            LaneSegment ls;
            ls.roadId = last.roadId;
            ls.laneId = laneId;
            ls.minS   = last.s;
            ls.maxS   = last.road_length > max_length - accumulated_length ? ls.minS + (max_length - accumulated_length) : last.road_length;
            ls.length = ls.maxS - ls.minS;
            lane_segments_.push_back(ls);
            add_length = ls.length;
        }
        accumulated_length += add_length;
    }
}

void TrafficAreaAction::HandleLastRoadCursor(std::vector<RoadCursor> last_road_cursors, double& accumulated_length, const double max_length)
{
    if (last_road_cursors.size() == 1)
    {
        for (const auto& laneId : last_road_cursors[0].laneIds)
        {
            LaneSegment ls;
            ls.roadId = last_road_cursors[0].roadId;
            ls.laneId = laneId;
            ls.minS   = 0.0;
            ls.maxS   = last_road_cursors[0].s == 0.0 ? last_road_cursors[0].road_length : last_road_cursors[0].s;
            ls.maxS   = std::min(ls.maxS, max_length - accumulated_length);
            ls.length = ls.maxS - ls.minS;
            lane_segments_.push_back(ls);
        }
    }
    else
    {
        auto it = std::find_if(last_road_cursors.begin(), last_road_cursors.end(), [](const RoadCursor& rc) { return rc.last; });
        if (it != last_road_cursors.end())
        {
            it->s = it->s == 0.0 ? it->road_length : it->s;
        }
        // Sort the road cursors by their s value
        std::sort(last_road_cursors.begin(), last_road_cursors.end(), [](const RoadCursor& a, const RoadCursor& b) { return a.s < b.s; });
        double second_last_s = 0.0;
        double add_length;
        for (size_t i = 0; i < last_road_cursors.size() - 1; ++i)
        {
            const auto& current = last_road_cursors[i];
            const auto& next    = last_road_cursors[i + 1];
            if (next.last)
            {
                second_last_s = current.s;
                LOG_INFO("Second last RoadCursor on last road is ignored");
                continue;
            }

            for (const auto& laneId : current.laneIds)
            {
                LaneSegment ls;
                ls.roadId = current.roadId;
                ls.laneId = laneId;
                ls.minS   = current.s;
                ls.maxS   = std::min(next.s, ls.minS + max_length - accumulated_length);
                ls.length = ls.maxS - ls.minS;
                lane_segments_.push_back(ls);
                add_length = ls.length;
            }
            accumulated_length += add_length;
            if (accumulated_length >= max_length)
            {
                return;
            }
        }

        const auto& last = last_road_cursors.back();
        for (const auto& laneId : last.laneIds)
        {
            LaneSegment ls;
            ls.roadId = last.roadId;
            ls.laneId = laneId;
            ls.minS   = second_last_s;
            ls.maxS   = std::min(last.s, ls.minS + max_length - accumulated_length);
            ls.length = ls.maxS - ls.minS;
            lane_segments_.push_back(ls);
        }
    }
}

void TrafficAreaAction::DespawnEntities()
{
    for (auto it = spawned_entity_ids_.begin(); it != spawned_entity_ids_.end();)
    {
        Object* obj = context_->GetScenarioEngine().entities_.GetObjectById(*it);
        if (!obj || !InsideArea(obj->pos_))
        {
            DespawnEntity(obj);
            it = spawned_entity_ids_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

bool TrafficAreaAction::InsideArea(roadmanager::Position object_pos)
{
    if (!polygon_points_.empty())
    {
        // Ray-casting algorithm to determine if the point is inside the polygon
        int  n      = polygon_points_.size();
        bool inside = false;

        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            double xi = polygon_points_[i].GetX(), yi = polygon_points_[i].GetY();
            double xj = polygon_points_[j].GetX(), yj = polygon_points_[j].GetY();

            bool intersect =
                ((yi > object_pos.GetY()) != (yj > object_pos.GetY())) && (object_pos.GetX() < (xj - xi) * (object_pos.GetY() - yi) / (yj - yi) + xi);
            if (intersect)
                inside = !inside;
        }

        return inside;
    }
    else if (!road_ranges_.empty())
    {
        int    obj_current_road = object_pos.GetTrackId();
        int    obj_current_lane = object_pos.GetLaneId();
        double obj_current_s    = object_pos.GetS();
        for (const auto& ls : lane_segments_)
        {
            if (ls.roadId == obj_current_road && ls.laneId == obj_current_lane)
            {
                if (obj_current_s >= ls.minS && obj_current_s <= ls.maxS)
                {
                    return true;
                }
            }
        }
        return false;
    }
    LOG_ERROR("Both polygon points and road range are empty in TrafficAreaAction");
    return false;
}

void TrafficAreaAction::SpawnEntities(int number_of_entities_to_spawn)
{
    for (int i = 0; i < number_of_entities_to_spawn; i++)
    {
        roadmanager::Position* vehicle_spawn_position = GetRandomSpawnPosition();
        if (vehicle_spawn_position == nullptr)
        {
            continue;
        }
        spawn_speed_ = vehicle_spawn_position->GetSpeedLimit();
        SpawnEntity(vehicle_spawn_position);
    }
}

roadmanager::Position* TrafficAreaAction::GetRandomSpawnPosition()
{
    // Check we have a valid polygon
    if (!polygon_points_.empty())
    {
        // Triangulate (fan method, assumes convex polygon)
        struct Triangle
        {
            roadmanager::Position a, b, c;
            double                area;
        };
        std::vector<Triangle> triangles;
        double                total_area = 0.0;

        for (size_t i = 1; i < polygon_points_.size() - 1; ++i)
        {
            const auto& a = polygon_points_[0];
            const auto& b = polygon_points_[i];
            const auto& c = polygon_points_[i + 1];

            // Compute area using cross product
            double area = 0.5 * std::abs((b.GetX() - a.GetX()) * (c.GetY() - a.GetY()) - (c.GetX() - a.GetX()) * (b.GetY() - a.GetY()));
            triangles.push_back({a, b, c, area});
            total_area += area;
        }

        // Pick a triangle weighted by area
        double pick = SE_Env::Inst().GetRand().GetRealBetween(0.0, total_area);

        size_t tri_idx = 0;
        for (; tri_idx < triangles.size(); ++tri_idx)
        {
            if (pick <= triangles[tri_idx].area)
                break;
            pick -= triangles[tri_idx].area;
        }
        if (tri_idx >= triangles.size())
            tri_idx = triangles.size() - 1;
        const Triangle& tri = triangles[tri_idx];

        // Pick a random point inside the triangle using barycentric coordinates
        double u = SE_Env::Inst().GetRand().GetReal();
        double v = SE_Env::Inst().GetRand().GetReal();
        if (u + v > 1.0)
        {
            u = 1.0 - u;
            v = 1.0 - v;
        }
        double w = 1.0 - u - v;

        double x = u * tri.a.GetX() + v * tri.b.GetX() + w * tri.c.GetX();
        double y = u * tri.a.GetY() + v * tri.b.GetY() + w * tri.c.GetY();

        // You can set other fields (z, heading, etc.) as needed
        roadmanager::Position* spawn_position = new roadmanager::Position(x, y, 0.0, 0.0, 0.0, 0.0);

        spawn_position->GotoClosestDrivingLaneAtCurrentPosition();
        
        spawn_position->SetLanePos(spawn_position->GetTrackId(), spawn_position->GetLaneId(), spawn_position->GetS(), 0.0);

        if (!InsideArea(*spawn_position) || !FreePositionToSpawn(spawn_position))
        {
            LOG_INFO("Generated spawn position is not inside polygon, or to close to other vehicle. Skipping spawn");
            delete spawn_position;
            return nullptr;
        }
        return spawn_position;
    }
    else if (!road_ranges_.empty())
    {
        if (!lane_segments_.empty())
        {
            const LaneSegment& seg = lane_segments_[SE_Env::Inst().GetRand().GetNumberBetween(0, lane_segments_.size() - 1)];
            double             s   = SE_Env::Inst().GetRand().GetRealBetween(seg.minS, seg.maxS);

            roadmanager::Position* spawn_position = new roadmanager::Position(seg.roadId, seg.laneId, s, 0.0);
            if (!FreePositionToSpawn(spawn_position))
        {
            LOG_INFO("Generated spawn position is to close to other vehicle. Skipping spawn");
            delete spawn_position;
            return nullptr;
        }
        return spawn_position;
        }
    }

    LOG_ERROR_AND_QUIT("Both polygon points and road range are empty in TrafficAreaAction");
    return nullptr;
}

void TrafficStopAction::Start(double simTime)
{
    LOG_INFO("Traffic Stop Action to stop: {}", traffic_action_to_stop_);

    OSCAction::Start(simTime);
}

void TrafficStopAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;
    auto traffic_to_stop = context_->GetScenarioEngine().storyBoard.FindChildByName(traffic_action_to_stop_);
    traffic_to_stop->Stop();
}