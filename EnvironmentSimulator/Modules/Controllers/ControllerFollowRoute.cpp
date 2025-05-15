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

/*
 * This controller enables a lane independent pathfinder and the lanechanges necessary for following the path
 */

#include <algorithm>
#include "ControllerFollowRoute.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "Storyboard.hpp"
#include "ScenarioEngine.hpp"
#include "LaneIndependentRouter.hpp"
#include "logger.hpp"

using namespace scenarioengine;

Controller *scenarioengine::InstantiateControllerFollowRoute(void *args)
{
    Controller::InitArgs *initArgs = static_cast<Controller::InitArgs *>(args);

    return new ControllerFollowRoute(initArgs);
}

ControllerFollowRoute::ControllerFollowRoute(InitArgs *args) : Controller(args), testMode_(false)
{
    if (args->properties)
    {
        if (args->properties->ValueExists("minDistForCollision"))
        {
            minDistForCollision_ = strtod(args->properties->GetValueStr("minDistForCollision"));
        }

        if (args->properties->ValueExists("laneChangeTime"))
        {
            laneChangeTime_ = strtod(args->properties->GetValueStr("laneChangeTime"));
        }

        if (args->properties->ValueExists("testMode"))
        {
            if (args->properties->GetValueStr("testMode") == "true")
            {
                testMode_ = true;
            }
            else
            {
                testMode_ = false;
            }
        }
    }
}

ControllerFollowRoute::~ControllerFollowRoute()
{
    if (laneChangeAction_ != nullptr)
    {
        delete laneChangeAction_;
        laneChangeAction_ = nullptr;
    }
}

void ControllerFollowRoute::Init()
{
    // FollowRoute controller forced into additive mode - will perform scenario actions, except during lane changes
    if (mode_ != ControlOperationMode::MODE_ADDITIVE)
    {
        LOG_WARN("FollowRoute controller mode \"{}\" not applicable. Using additive mode (except during lane changes).", Mode2Str(mode_));
        mode_ = ControlOperationMode::MODE_ADDITIVE;
    }

    LOG_INFO("FollowRoute init");

    Controller::Init();
}

void ControllerFollowRoute::Step(double timeStep)
{
    if (object_->pos_.GetRoute() == nullptr)
    {
        LOG_DEBUG("Route = nullptr");
        Controller::Step(timeStep);
        return;
    }

    if (!pathCalculated_)
    {
        CalculateWaypoints();
    }

    // Check if all waypoints have been passed
    if (static_cast<unsigned int>(currentWaypointIndex_) >= waypoints_.size())
    {
        Deactivate();
        return;
    }

    roadmanager::Position vehiclePos   = object_->pos_;
    roadmanager::Position nextWaypoint = waypoints_[static_cast<unsigned int>(currentWaypointIndex_)];

    bool sameRoad = nextWaypoint.GetTrackId() == vehiclePos.GetTrackId();

    // Check if lane with different ids are connected between lane sections
    int connectedLaneID =
        odr_->GetRoadById(vehiclePos.GetTrackId())->GetConnectedLaneIdAtS(vehiclePos.GetLaneId(), vehiclePos.GetS(), nextWaypoint.GetS());
    bool lsecConnectedLane = connectedLaneID == nextWaypoint.GetLaneId();

    bool sameLane = (nextWaypoint.GetLaneId() == vehiclePos.GetLaneId()) || lsecConnectedLane;
    if (sameRoad)
    {
        double distToLaneChange = MAX(laneChangeTime_ * object_->GetSpeed(), 25);
        bool   nearSPos         = abs(vehiclePos.GetS() - nextWaypoint.GetS()) < distToLaneChange;
        if (!sameLane && nearSPos && CanChangeLane(nextWaypoint.GetLaneId()))
        {
            CreateLaneChange(nextWaypoint.GetLaneId());
            changingLane_ = true;
        }
    }

    ChangeLane(timeStep);
    UpdateWaypoints(vehiclePos, nextWaypoint);

    Controller::Step(timeStep);
}

int ControllerFollowRoute::Activate(ControlActivationMode lat_activation_mode,
                                    ControlActivationMode long_activation_mode,
                                    ControlActivationMode light_activation_mode,
                                    ControlActivationMode anim_activation_mode)
{
    LOG_INFO("FollowRoute activate");

    if (object_ != nullptr)
    {
        odr_ = object_->pos_.GetOpenDrive();
    }
    currentWaypointIndex_  = 0;
    scenarioWaypointIndex_ = 0;
    pathCalculated_        = false;
    changingLane_          = false;
    waypoints_             = {};
    laneChangeAction_      = nullptr;

    return Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);
}

void ControllerFollowRoute::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}

void ControllerFollowRoute::UpdateWaypoints(roadmanager::Position vehiclePos, roadmanager::Position nextWaypoint)
{
    WaypointStatus waypointStatus = GetWaypointStatus(vehiclePos, nextWaypoint);
    switch (waypointStatus)
    {
        case PASSED_WAYPOINT:  // Get next waypoint and calculate path to it
        {
            LOG_DEBUG("Passed waypoint: r={}, l={}, s={}", nextWaypoint.GetTrackId(), nextWaypoint.GetLaneId(), nextWaypoint.GetS());
            currentWaypointIndex_++;
            if (nextWaypoint.GetTrackId() == waypoints_.back().GetTrackId())
            {
                bool scenarioWaypointsLeft =
                    static_cast<unsigned int>(scenarioWaypointIndex_) < object_->pos_.GetRoute()->scenario_waypoints_.size() - 1;
                if (scenarioWaypointsLeft)
                {
                    scenarioWaypointIndex_++;
                    pathCalculated_ = false;
                    CalculateWaypoints();
                    currentWaypointIndex_ = 0;
                }
                return;
            }

            object_->pos_.GetRoute()->ReplaceMinimalWaypoints({vehiclePos, waypoints_[static_cast<unsigned int>(currentWaypointIndex_)]});
            object_->SetDirtyBits(Object::DirtyBit::ROUTE);  // Set dirty bit to notify that route has changed

            return;
        }
        case MISSED_WAYPOINT:  // Missed waypoint, re-calculate path to waypoint
            LOG_DEBUG("Missed waypoint: r={}, l={}, s={}", nextWaypoint.GetTrackId(), nextWaypoint.GetLaneId(), nextWaypoint.GetS());
            if (object_->pos_.GetRoute() != nullptr)
            {
                pathCalculated_ = false;
                CalculateWaypoints();
                currentWaypointIndex_ = 0;
            }
            return;
        case WAYPOINT_NOT_REACHED:
            return;
    }
}

void ControllerFollowRoute::CalculateWaypoints()
{
    roadmanager::LaneIndependentRouter router(odr_);

    roadmanager::Position startPos(object_->pos_);
    roadmanager::Position targetPos(object_->pos_.GetRoute()->scenario_waypoints_[static_cast<unsigned int>(scenarioWaypointIndex_)]);

    // If current road is found along route, set successor waypoint as target
    unsigned int i = 0;
    unsigned int j = 0;
    for (auto &scwp : object_->pos_.GetRoute()->all_waypoints_)
    {
        if (scwp.GetTrackId() == object_->pos_.GetRoute()->scenario_waypoints_[j].GetTrackId())
        {
            // j represents the index of the next scenario waypoint corresponding to the pivot detailed waypoint
            j++;
        }

        if (scwp.GetTrackId() == startPos.GetTrackId())
        {
            // found the detailed waypoint at startpos road
            // check whether the startpos is on route (not past last waypoint)
            bool drivingWithRoadDirection = startPos.GetDrivingDirectionRelativeRoad() == 1;
            if ((i < object_->pos_.GetRoute()->all_waypoints_.size() - 1) ||
                ((drivingWithRoadDirection && startPos.GetS() < scwp.GetS() - SMALL_NUMBER) ||
                 (!drivingWithRoadDirection && object_->pos_.GetS() > scwp.GetS() + SMALL_NUMBER)))
            {
                // either we can pick next scenario waypoint, or the last since it is ahead
                scenarioWaypointIndex_ = MIN(static_cast<int>(j), static_cast<int>(object_->pos_.GetRoute()->scenario_waypoints_.size() - 1));
                targetPos              = object_->pos_.GetRoute()->scenario_waypoints_[MAX(0, static_cast<unsigned int>(scenarioWaypointIndex_))];
                break;
            }
        }
        i++;
    }

    std::vector<roadmanager::Node> pathToGoal = router.CalculatePath(startPos, targetPos);
    if (pathToGoal.empty())
    {
        LOG_ERROR("Error: Path not found, deactivating controller");
        Deactivate();
    }
    else
    {
        waypoints_ = router.GetWaypoints(pathToGoal, startPos, targetPos);

        object_->pos_.GetRoute()->ReplaceMinimalWaypoints({waypoints_[0], waypoints_[1]});
        object_->SetDirtyBits(Object::DirtyBit::ROUTE);  // Set dirty bit to notify that route has changed
        pathCalculated_ = true;
    }
}

void ControllerFollowRoute::CreateLaneChange(int lane)
{
    LatLaneChangeAction *action_lanechange = new LatLaneChangeAction(nullptr);
    action_lanechange->SetName("LaneChange");
    action_lanechange->object_                = object_;
    action_lanechange->transition_.shape_     = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
    action_lanechange->transition_.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
    action_lanechange->transition_.SetParamTargetVal(laneChangeTime_);
    action_lanechange->max_num_executions_ = 10;

    LatLaneChangeAction::TargetAbsolute *target = new LatLaneChangeAction::TargetAbsolute;
    target->value_                              = lane;

    action_lanechange->target_ = target;

    // Set lane change to be performed by ChangeLane function
    laneChangeAction_ = action_lanechange;
}

void ControllerFollowRoute::ChangeLane(double timeStep)
{
    if (laneChangeAction_ == nullptr || laneChangeAction_->GetCurrentState() == OSCAction::State::COMPLETE)
    {
        return;
    }

    if (!(laneChangeAction_->GetCurrentState() == StoryBoardElement::State::RUNNING))
    {
        laneChangeAction_->Start(scenarioEngine_->getSimulationTime());

        mode_ = ControlOperationMode::MODE_OVERRIDE;  // override mode to prevent default controller from moving the entity

        // skip step at this timestep since default controller already update the entity
    }
    else
    {
        mode_ = ControlOperationMode::MODE_ADDITIVE;  // disable override mode to enable use of default controller actions
        laneChangeAction_->Step(scenarioEngine_->getSimulationTime(), timeStep);
        mode_ = ControlOperationMode::MODE_OVERRIDE;  // restore override mode to prevent default controller from moving the entity

        if (laneChangeAction_->GetCurrentState() == OSCAction::State::COMPLETE)
        {
            changingLane_ = false;
            delete laneChangeAction_;
            laneChangeAction_ = nullptr;
            mode_             = ControlOperationMode::MODE_ADDITIVE;
            return;
        }

        // Fetch updated position after lane change step
        vehicle_.posX_    = object_->pos_.GetX();
        vehicle_.posY_    = object_->pos_.GetY();
        vehicle_.heading_ = object_->pos_.GetH();
    }
}

bool ControllerFollowRoute::CanChangeLane(int lane)
{
    roadmanager::Position vehiclePos  = object_->pos_;
    std::vector<Object *> allVehicles = scenarioEngine_->entities_.object_;
    if (changingLane_)
    {
        return false;
    }
    roadmanager::Road        *road = odr_->GetRoadById(vehiclePos.GetTrackId());
    roadmanager::LaneSection *ls   = road->GetLaneSectionByS(vehiclePos.GetS());
    if (ls->GetLaneById(lane) == 0)  // lane does not exist on road in current lanesection
    {
        return false;
    }

    // Disable collision handling
    if (minDistForCollision_ <= 0)
    {
        return true;
    }

    // Check width of target lane at this point (s-value) along the road
    if (road->GetLaneWidthByS(vehiclePos.GetS(), lane) < minLaneWidth_)
    {
        return false;
    }

    // Check collision risk to every other vehicle on the same side of road.
    for (Object *otherVehicle : allVehicles)
    {
        bool sameRoad = otherVehicle->pos_.GetTrackId() == vehiclePos.GetTrackId();
        bool sameLane = otherVehicle->pos_.GetLaneId() == vehiclePos.GetLaneId();
        if (otherVehicle == object_ || (sameRoad && sameLane))
        {
            continue;
        }
        bool collisionRisk  = DistanceBetween(vehiclePos, otherVehicle->pos_) < minDistForCollision_;
        bool sameSideOfRoad = SIGN(vehiclePos.GetLaneId()) == SIGN(otherVehicle->pos_.GetLaneId());

        if (!sameLane && collisionRisk && sameSideOfRoad && sameRoad)
        {
            int  n   = vehiclePos.GetLaneId();
            auto inc = [&n] { return ++n; };
            auto dec = [&n] { return --n; };

            int  lanesBetween    = abs(vehiclePos.GetLaneId() - otherVehicle->pos_.GetLaneId());
            bool lanesIncreasing = otherVehicle->pos_.GetLaneId() > vehiclePos.GetLaneId();

            std::vector<int> laneIdsToCheck(static_cast<unsigned int>(lanesBetween));
            if (lanesIncreasing)
            {
                std::generate(laneIdsToCheck.begin(), laneIdsToCheck.end(), inc);
            }
            else
            {
                std::generate(laneIdsToCheck.begin(), laneIdsToCheck.end(), dec);
            }

            bool collides = std::find(laneIdsToCheck.begin(), laneIdsToCheck.end(), otherVehicle->pos_.GetLaneId()) != laneIdsToCheck.end();
            if (collides)
            {
                return false;
            }
        }
    }
    return true;
}

double ControllerFollowRoute::DistanceBetween(roadmanager::Position p1, roadmanager::Position p2)
{
    double x1 = p1.GetX();
    double y1 = p1.GetY();
    double x2 = p2.GetX();
    double y2 = p2.GetY();

    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void ControllerFollowRoute::Deactivate()
{
    if (testMode_)
    {
        gateway_->updateObjectSpeed(object_->GetId(), 0.0, 0.0);
    }
    LOG_INFO("ControllerFollowRoute - Deactivated");
    object_->pos_.SetRoute(nullptr);
    Controller::Deactivate();
}

WaypointStatus ControllerFollowRoute::GetWaypointStatus(roadmanager::Position vehiclePos, roadmanager::Position waypoint)
{
    using namespace roadmanager;
    bool drivingWithRoadDirection = vehiclePos.GetDrivingDirectionRelativeRoad() == 1;
    bool sameRoad                 = waypoint.GetTrackId() == vehiclePos.GetTrackId();
    bool sameLane                 = waypoint.GetLaneId() == vehiclePos.GetLaneId();

    if (sameRoad && ((drivingWithRoadDirection && vehiclePos.GetS() > waypoint.GetS() - SMALL_NUMBER) ||
                     (!drivingWithRoadDirection && vehiclePos.GetS() < waypoint.GetS() + SMALL_NUMBER)))
    {
        return sameLane ? PASSED_WAYPOINT : MISSED_WAYPOINT;
    }
    if (sameRoad)
    {
        return WAYPOINT_NOT_REACHED;
    }
    Road               *currentRoad = odr_->GetRoadById(vehiclePos.GetTrackId());
    std::vector<Road *> possiblePreviousRoads;
    RoadLink           *link;
    if (drivingWithRoadDirection)
    {
        link = currentRoad->GetLink(LinkType::PREDECESSOR);
    }
    else
    {
        link = currentRoad->GetLink(LinkType::SUCCESSOR);
    }

    if (link == nullptr)
    {
        return WAYPOINT_NOT_REACHED;
    }

    if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
    {
        possiblePreviousRoads.push_back(odr_->GetRoadById(link->GetElementId()));
    }
    else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
    {
        Junction *junction = odr_->GetJunctionById(link->GetElementId());
        for (unsigned j = 0; j < junction->GetNoConnectionsFromRoadId(currentRoad->GetId()); j++)
        {
            id_t roadId = junction->GetConnectingRoadIdFromIncomingRoadId(currentRoad->GetId(), j);
            possiblePreviousRoads.push_back(odr_->GetRoadById(roadId));
        }
    }

    std::vector<Road *>::iterator itr = std::find_if(possiblePreviousRoads.begin(),
                                                     possiblePreviousRoads.end(),
                                                     [&waypoint](Road *road) { return road->GetId() == waypoint.GetTrackId(); });
    if (itr != possiblePreviousRoads.end())
    {
        int previousLaneId = currentRoad->GetConnectingLaneId(link, vehiclePos.GetLaneId(), waypoint.GetTrackId());
        return previousLaneId == waypoint.GetLaneId() ? PASSED_WAYPOINT : MISSED_WAYPOINT;
    }
    return WAYPOINT_NOT_REACHED;
}
