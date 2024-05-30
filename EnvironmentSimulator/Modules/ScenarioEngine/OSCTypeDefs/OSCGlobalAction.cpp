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
#include "OSCSwarmTrafficGeometry.hpp"
#include <memory>
#include <cmath>
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>
#include <numeric>
#include "ControllerACC.hpp"
#include "ScenarioReader.hpp"

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
#define VEHICLE_DISTANCE      12   // Min distance between two spawned vehicles
#define SWARM_TIME_INTERVAL   0.1  // Sleep time between update steps
#define SWARM_SPAWN_FREQUENCY 1.1  // Sleep time between spawns
#define MAX_CARS              1000
#define MAX_LANES             32

int SwarmTrafficAction::counter_ = 0;

void ParameterSetAction::Start(double simTime)
{
    LOG("Set parameter %s = %s", name_.c_str(), value_.c_str());
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
    LOG("Set variable %s = %s", name_.c_str(), value_.c_str());
    variables_->setParameterValueByString(name_, value_);
    OSCAction::Start(simTime);
}

void VariableSetAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::Stop();
}

void AddEntityAction::Start(double simTime)
{
    if (entity_ == nullptr)
    {
        LOG("AddEntityAction missing entity");
        return;
    }

    if (entities_->activateObject(entity_) != 0)
    {
        LOG("AddEntityAction: Entity already active. Skipping action.");
        return;
    }

    entity_->pos_.TeleportTo(pos_);

    LOG("Added entity %s", entity_->GetName().c_str());

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
        LOG("DeleteEntityAction missing entity");
        return;
    }

    if (entities_->deactivateObject(entity_) != 0)
    {
        LOG("DeleteEntityAction: Entity already deactivated. Skipping action.");
        return;
    }

    gateway_->removeObject(entity_->name_);

    LOG("Deleted entity %s", entity_->GetName().c_str());

    OSCAction::Start(simTime);
}

void DeleteEntityAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;
    OSCAction::Stop();
}

void print_triangles(BBoxVec& vec, char const filename[])
{
    std::ofstream file;
    file.open(filename);
    for (auto const& bbx : vec)
    {
        auto trPtr = bbx->triangle();
        auto pt    = trPtr->a;
        file << pt.x << "," << pt.y;
        file << "," << trPtr->b.x << "," << trPtr->b.y;
        file << "," << trPtr->c.x << "," << trPtr->c.y << "\n";
    }
    file.close();
}

void print_bbx(BBoxVec& vec, char const filename[])
{
    std::ofstream file;
    file.open(filename);
    for (auto const& bbx : vec)
    {
        auto pt = bbx->blhCorner();
        file << pt.x << "," << pt.y;
        file << "," << bbx->urhCorner().x << "," << bbx->urhCorner().y << "\n";
    }
    file.close();
}

void printTree(aabbTree::Tree& tree, char filename[])
{
    std::ofstream file;
    file.open(filename);
    std::vector<aabbTree::ptTree> v1, v2;
    v1.clear();
    v2.clear();

    if (tree.empty())
    {
        file.close();
        return;
    }

    auto bbx = tree.BBox();
    file << bbx->blhCorner().x << "," << bbx->blhCorner().y << "," << bbx->urhCorner().x << "," << bbx->urhCorner().y << "\n";
    v1.insert(v1.end(), tree.Children().begin(), tree.Children().end());

    while (!v1.empty())
    {
        for (auto const& tr : v1)
        {
            if (!tr->empty())
            {
                auto bbox = tr->BBox();
                file << bbox->blhCorner().x << "," << bbox->blhCorner().y << "," << bbox->urhCorner().x << "," << bbox->urhCorner().y << ",";
                v2.insert(v2.end(), tr->Children().begin(), tr->Children().end());
            }
        }
        file << "\n";
        v1.clear();
        v1 = v2;
        v2.clear();
    }

    file.close();
}

SwarmTrafficAction::SwarmTrafficAction(StoryBoardElement* parent) : OSCGlobalAction(ActionType::SWARM_TRAFFIC, parent), centralObject_(0)
{
    spawnedV.clear();
    counter_ = 0;
}

SwarmTrafficAction::~SwarmTrafficAction()
{
    auto RecursiveDeleteTrailers = [](Vehicle* vehicle, auto& recurseLambda) -> void
    {
        if (vehicle->trailer_hitch_ && vehicle->trailer_hitch_->trailer_vehicle_)
        {
            recurseLambda(static_cast<Vehicle*>(vehicle->trailer_hitch_->trailer_vehicle_), recurseLambda);
        }

        delete vehicle;
    };

    for (auto* entry : vehicle_pool_)
    {
        if (entry != centralObject_)
        {
            if (entry->trailer_hitch_ && entry->trailer_hitch_->trailer_vehicle_)
            {
                RecursiveDeleteTrailers(static_cast<Vehicle*>(entry->trailer_hitch_->trailer_vehicle_), RecursiveDeleteTrailers);
            }

            delete entry;
        }
    }
}

void SwarmTrafficAction::Start(double simTime)
{
    LOG("Swarm IR: %.2f, SMjA: %.2f, SMnA: %.2f, maxV: %i vel: %.2f", innerRadius_, semiMajorAxis_, semiMinorAxis_, numberOfVehicles, velocity_);
    double x0, y0, x1, y1;

    midSMjA  = (semiMajorAxis_ + innerRadius_) / 2.0;
    midSMnA  = (semiMinorAxis_ + innerRadius_) / 2.0;
    lastTime = -1;

    paramEllipse(0, 0, 0, midSMjA, midSMnA, 0, x0, y0);
    paramEllipse(M_PI / 36, 0, 0, midSMjA, midSMnA, 0, x1, y1);

    odrManager_ = roadmanager::Position::GetOpenDrive();
    minSize_    = ceil(sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) * 100) / 100.0;
    if (minSize_ == 0)
        minSize_ = 1.0;

    aabbTree::ptTree  tree = std::make_shared<aabbTree::Tree>();
    aabbTree::BBoxVec vec;
    vec.clear();
    createRoadSegments(vec);

    tree->build(vec);
    rTree = tree;

    // Register model filesnames from first vehicle catalog
    // if no catalog loaded, use same model as central object
    Catalogs* catalogs = reader_->GetCatalogs();
    for (size_t i = 0; i < catalogs->catalog_.size(); i++)
    {
        if (catalogs->catalog_[i]->GetType() == CatalogType::CATALOG_VEHICLE)
        {
            for (size_t j = 0; j < catalogs->catalog_[i]->entry_.size(); j++)
            {
                Vehicle* vehicle = reader_->parseOSCVehicle(catalogs->catalog_[i]->entry_[j]->GetNode());
                if (vehicle->category_ == Vehicle::Category::CAR || vehicle->category_ == Vehicle::Category::BUS ||
                    vehicle->category_ == Vehicle::Category::TRUCK || vehicle->category_ == Vehicle::Category::VAN ||
                    vehicle->category_ == Vehicle::Category::MOTORBIKE)
                {
                    vehicle_pool_.push_back(vehicle);
                }
                else
                {
                    delete vehicle;
                }
            }
        }
    }

    if (vehicle_pool_.size() == 0)
    {
        if (centralObject_->type_ == Object::Type::VEHICLE)
        {
            vehicle_pool_.push_back(static_cast<Vehicle*>(centralObject_));
        }
        else
        {
            LOG_AND_QUIT("No vehicles available to populate swarm traffic. Vehicle catalog empty?");
        }
    }

    OSCAction::Start(simTime);
}

void SwarmTrafficAction::Step(double simTime, double dt)
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

void SwarmTrafficAction::createRoadSegments(BBoxVec& vec)
{
    for (int i = 0; i < odrManager_->GetNumOfRoads(); i++)
    {
        roadmanager::Road* road = odrManager_->GetRoadByIdx(i);

        for (int j = 0; j < road->GetNumberOfGeometries(); j++)
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

void SwarmTrafficAction::createEllipseSegments(aabbTree::BBoxVec& vec, double SMjA, double SMnA)
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

inline void SwarmTrafficAction::sampleRoads(int minN, int maxN, Solutions& sols, vector<SelectInfo>& info)
{
    // printf("Entered road selection\n");
    // printf("Min: %d, Max: %d\n", minN, maxN);

    // Sample the number of cars to spawn
    if (maxN < minN)
    {
        LOG("Unstable behavior detected (maxN < minN)");
        return;
    }

    int nCarsToSpawn = SE_Env::Inst().GetRand().GetNumberBetween(minN, maxN - 1);
    if (nCarsToSpawn <= 0)
    {
        return;
    }

    info.reserve(static_cast<unsigned int>(nCarsToSpawn));
    info.clear();
    // We have more points than number of vehicles to spawn.
    // We sample the selected number and each point will be assigned a lane
    if (static_cast<unsigned int>(nCarsToSpawn) <= sols.size() && nCarsToSpawn > 0)
    {
        // Shuffle and randomly select the points
        // Solutions selected(nCarsToSpawn);
        static Point selected[MAX_CARS];  // Remove macro when/if found a solution for dynamic array
        std::shuffle(sols.begin(), sols.end(), SE_Env::Inst().GetRand().GetGenerator());
        sample(sols.begin(), sols.end(), selected, nCarsToSpawn, SE_Env::Inst().GetRand().GetGenerator());

        for (int i = 0; i < nCarsToSpawn; i++)
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
            roadmanager::Road* road = odrManager_->GetRoadById(pos.GetTrackId());
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
        int lanesLeft = nCarsToSpawn - static_cast<int>(sols.size());
        for (Point pt : sols)
        {
            roadmanager::Position pos(pt.x, pt.y, 0.0, pt.h, 0.0, 0.0);
            // pos.XYZH2TrackPos(pt.x, pt.y, 0, pt.h);

            roadmanager::Road* road          = odrManager_->GetRoadById(pos.GetTrackId());
            int                nDrivingLanes = road->GetNumberOfDrivingLanes(pos.GetS());
            if (nDrivingLanes == 0)
            {
                lanesLeft++;
                continue;
            }

            int lanesN;
            if (lanesLeft > 0)
            {
                std::uniform_int_distribution<int> laneDist(0, std::min(lanesLeft, nDrivingLanes));
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

void SwarmTrafficAction::spawn(Solutions sols, int replace, double simTime)
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
        int        lanesNo = MIN(MAX_LANES, inf.road->GetNumberOfDrivingLanes(inf.pos.GetS()));
        static int elements[MAX_LANES];
        std::iota(elements, elements + lanesNo, 0);

        static int lanes[MAX_LANES];

        sample(elements, elements + lanesNo, lanes, MIN(MAX_LANES, inf.nLanes), SE_Env::Inst().GetRand().GetGenerator());

        for (int i = 0; i < MIN(MAX_LANES, inf.nLanes); i++)
        {
            auto Lane = inf.road->GetDrivingLaneByIdx(inf.pos.GetS(), lanes[i]);
            int  laneID;

            if (!Lane)
            {
                LOG("Warning: invalid lane index");
                continue;
            }
            else
            {
                laneID = Lane->GetId();
            }

            if (!ensureDistance(inf.pos, laneID, MIN(MAX(40.0, velocity_ * 2.0), 0.7 * semiMajorAxis_)))
                continue;  // distance = speed * 2 seconds

            Controller::InitArgs args;
            args.name       = "Swarm ACC controller";
            args.type       = ControllerACC::GetTypeNameStatic();
            args.entities   = entities_;
            args.gateway    = gateway_;
            args.parameters = 0;
            args.properties = 0;

#if 0  // This is one way of setting the ACC setSpeed property
            args.properties = new OSCProperties();
            OSCProperties::Property property;
            property.name_ = "setSpeed";
            property.value_ = std::to_string(velocity_);
            args.properties->property_.push_back(property);
#endif
            Controller* acc = InstantiateControllerACC(&args);

#if 1  // This is another way of setting the ACC setSpeed property
            (static_cast<ControllerACC*>(acc))->SetSetSpeed(velocity_);
#endif
            reader_->AddController(acc);

            // Pick random model from vehicle catalog
            std::uniform_int_distribution<int> dist(0, static_cast<int>((vehicle_pool_.size() - 1)));
            int                                number = dist(SE_Env::Inst().GetRand().GetGenerator());

            Vehicle* vehicle = new Vehicle(*vehicle_pool_[static_cast<unsigned int>(number)]);
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

            vehicle->SetSpeed(velocity_);
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
            acc->Activate(ControlActivationMode::OFF, ControlActivationMode::ON, ControlActivationMode::OFF, ControlActivationMode::OFF);

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

inline bool SwarmTrafficAction::ensureDistance(roadmanager::Position pos, int lane, double dist)
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

int SwarmTrafficAction::despawn(double simTime)
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
                reader_->RemoveController(ctrl);
            }

            if (vehicle->type_ == Object::Type::VEHICLE)
            {
                Vehicle* v       = static_cast<Vehicle*>(vehicle);
                Vehicle* trailer = nullptr;
                while (v)  // remove all linked trailers
                {
                    trailer = static_cast<Vehicle*>(v->TrailerVehicle());

                    gateway_->removeObject(v->name_);

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
                gateway_->removeObject(vehicle->name_);
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