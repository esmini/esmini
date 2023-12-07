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

#include "OSCPosition.hpp"

using namespace scenarioengine;

OSCPositionWorld::OSCPositionWorld(double x, double y, double z, double h, double p, double r, OSCPosition *base_on_pos)
    : OSCPosition(PositionType::WORLD)
{
    if (base_on_pos != nullptr && base_on_pos->type_ == PositionType::WORLD)
    {
        this->position_ = *base_on_pos->GetRMPos();
    }

    // nan indicates value not set -> mark relative to align to road or curve
    position_.SetMode(roadmanager::Position::PosModeType::INIT,
                      (std::isnan(z) ? roadmanager::Position::PosMode::Z_REL : roadmanager::Position::PosMode::Z_ABS) |
                          (std::isnan(h) ? roadmanager::Position::PosMode::H_REL : roadmanager::Position::PosMode::H_ABS) |
                          (std::isnan(p) ? roadmanager::Position::PosMode::P_REL : roadmanager::Position::PosMode::P_ABS) |
                          (std::isnan(r) ? roadmanager::Position::PosMode::R_REL : roadmanager::Position::PosMode::R_ABS));

    position_.SetInertiaPosMode(x, y, z, h, p, r, position_.GetMode(roadmanager::Position::PosModeType::INIT));
}

OSCPositionLane::OSCPositionLane(int roadId, int laneId, double s, double offset, OSCOrientation orientation) : OSCPosition(PositionType::LANE)
{
    position_.SetOrientationType(orientation.type_);

    position_.SetMode(
        roadmanager::Position::PosModeType::INIT,
        roadmanager::Position::PosMode::Z_REL |
            (std::isnan(orientation.h_)
                 ? roadmanager::Position::PosMode::H_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::H_REL
                                                                                                      : roadmanager::Position::PosMode::H_ABS)) |
            (std::isnan(orientation.p_)
                 ? roadmanager::Position::PosMode::P_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::P_REL
                                                                                                      : roadmanager::Position::PosMode::P_ABS)) |
            (std::isnan(orientation.r_)
                 ? roadmanager::Position::PosMode::R_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::R_REL
                                                                                                      : roadmanager::Position::PosMode::R_ABS)));

    if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
    {
        // Set temporary position to find out driving direction
        position_.SetLanePosMode(roadId, laneId, s, offset, position_.GetMode(roadmanager::Position::PosModeType::INIT));

        // Adjust heading to road direction also considering traffic rule (left/right hand traffic)
        if (position_.GetDrivingDirectionRelativeRoad() > 0)
        {
            position_.SetHeadingRelative(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        }
        else
        {
            position_.SetHeadingRelative(GetAngleSum(M_PI, std::isnan(orientation.h_) ? 0.0 : orientation.h_));
        }
        position_.SetPitchRelative(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetRollRelative(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE)
    {
        position_.SetH(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetP(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetR(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else
    {
        LOG("Unexpected orientation type: %d", orientation.type_);
    }

    position_.SetLanePosMode(roadId, laneId, s, offset, position_.GetMode(roadmanager::Position::PosModeType::INIT));
}

OSCPositionRoad::OSCPositionRoad(int roadId, double s, double t, OSCOrientation orientation) : OSCPosition(PositionType::ROAD)
{
    if (position_.GetRoadById(roadId) == nullptr)
    {
        LOG_AND_QUIT("Reffered road ID %d not available in road network", roadId);
    }

    position_.SetOrientationType(orientation.type_);

    position_.SetMode(
        roadmanager::Position::PosModeType::INIT,
        roadmanager::Position::PosMode::Z_REL |
            (std::isnan(orientation.h_)
                 ? roadmanager::Position::PosMode::H_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::H_REL
                                                                                                      : roadmanager::Position::PosMode::H_ABS)) |
            (std::isnan(orientation.p_)
                 ? roadmanager::Position::PosMode::P_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::P_REL
                                                                                                      : roadmanager::Position::PosMode::P_ABS)) |
            (std::isnan(orientation.r_)
                 ? roadmanager::Position::PosMode::R_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::R_REL
                                                                                                      : roadmanager::Position::PosMode::R_ABS)));

    if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
    {
        position_.SetHeadingRelative(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetPitchRelative(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetRollRelative(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE)
    {
        position_.SetH(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetP(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetR(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else
    {
        LOG("Unexpected orientation type: %d", orientation.type_);
    }

    position_.SetTrackPosMode(roadId, s, t, position_.GetMode(roadmanager::Position::PosModeType::INIT));
}

OSCPositionRelativeObject::OSCPositionRelativeObject(Object *object, double dx, double dy, double dz, OSCOrientation orientation)
    : OSCPosition(PositionType::RELATIVE_OBJECT),
      object_(object)
{
    position_.SetOrientationType(orientation.type_);

    position_.SetMode(
        roadmanager::Position::PosModeType::INIT,
        roadmanager::Position::PosMode::Z_REL |
            (std::isnan(orientation.h_)
                 ? roadmanager::Position::PosMode::H_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::H_REL
                                                                                                      : roadmanager::Position::PosMode::H_ABS)) |
            (std::isnan(orientation.p_)
                 ? roadmanager::Position::PosMode::P_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::P_REL
                                                                                                      : roadmanager::Position::PosMode::P_ABS)) |
            (std::isnan(orientation.r_)
                 ? roadmanager::Position::PosMode::R_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::R_REL
                                                                                                      : roadmanager::Position::PosMode::R_ABS)));

    if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
    {
        position_.SetHeadingRelative(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetPitchRelative(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetRollRelative(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE)
    {
        position_.SetH(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetP(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetR(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else
    {
        LOG("Unexpected orientation type: %d", orientation.type_);
    }

    position_.SetX(dx);
    position_.SetY(dy);
    position_.SetZ(std::isnan(dz) ? 0.0 : dz);

    position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_OBJECT);
}

void OSCPositionRelativeObject::Print()
{
    object_->pos_.Print();
}

OSCPositionRelativeWorld::OSCPositionRelativeWorld(Object *object, double dx, double dy, double dz, OSCOrientation orientation)
    : OSCPosition(PositionType::RELATIVE_WORLD),
      object_(object)
{
    position_.SetOrientationType(orientation.type_);

    position_.SetMode(
        roadmanager::Position::PosModeType::INIT,
        roadmanager::Position::PosMode::Z_REL |
            (std::isnan(orientation.h_)
                 ? roadmanager::Position::PosMode::H_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::H_REL
                                                                                                      : roadmanager::Position::PosMode::H_ABS)) |
            (std::isnan(orientation.p_)
                 ? roadmanager::Position::PosMode::P_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::P_REL
                                                                                                      : roadmanager::Position::PosMode::P_ABS)) |
            (std::isnan(orientation.r_)
                 ? roadmanager::Position::PosMode::R_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::R_REL
                                                                                                      : roadmanager::Position::PosMode::R_ABS)));

    position_.SetX(dx);
    position_.SetY(dy);
    position_.SetZ(std::isnan(dz) ? 0.0 : dz);

    if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
    {
        position_.SetHeadingRelative(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetPitchRelative(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetRollRelative(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE)
    {
        position_.SetH(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetP(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetR(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else
    {
        LOG("Unexpected orientation type: %d", orientation.type_);
    }

    position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_WORLD);
}

void OSCPositionRelativeWorld::Print()
{
    object_->pos_.Print();
}

OSCPositionRelativeLane::OSCPositionRelativeLane(Object                              *object,
                                                 int                                  dLane,
                                                 double                               ds,
                                                 double                               offset,
                                                 OSCOrientation                       orientation,
                                                 roadmanager::Position::DirectionMode direction_mode)
    : OSCPosition(PositionType::RELATIVE_LANE),
      object_(object)
{
    position_.SetOrientationType(orientation.type_);

    position_.SetMode(
        roadmanager::Position::PosModeType::INIT,
        roadmanager::Position::PosMode::Z_REL |
            (std::isnan(orientation.h_)
                 ? roadmanager::Position::PosMode::H_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::H_REL
                                                                                                      : roadmanager::Position::PosMode::H_ABS)) |
            (std::isnan(orientation.p_)
                 ? roadmanager::Position::PosMode::P_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::P_REL
                                                                                                      : roadmanager::Position::PosMode::P_ABS)) |
            (std::isnan(orientation.r_)
                 ? roadmanager::Position::PosMode::R_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::R_REL
                                                                                                      : roadmanager::Position::PosMode::R_ABS)));

    position_.SetLaneId(dLane);
    position_.SetS(ds);
    position_.SetOffset(offset);
    position_.SetDirectionMode(direction_mode);

    if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
    {
        position_.SetHeadingRelative(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetPitchRelative(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetRollRelative(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else
    {
        position_.SetH(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetP(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetR(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }

    position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_LANE);
}

void OSCPositionRelativeLane::Print()
{
    object_->pos_.Print();
}

OSCPositionRelativeRoad::OSCPositionRelativeRoad(Object *object, double ds, double dt, OSCOrientation orientation)
    : OSCPosition(PositionType::RELATIVE_ROAD),
      object_(object)
{
    position_.SetOrientationType(orientation.type_);

    position_.SetMode(
        roadmanager::Position::PosModeType::INIT,
        roadmanager::Position::PosMode::Z_REL |
            (std::isnan(orientation.h_)
                 ? roadmanager::Position::PosMode::H_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::H_REL
                                                                                                      : roadmanager::Position::PosMode::H_ABS)) |
            (std::isnan(orientation.p_)
                 ? roadmanager::Position::PosMode::P_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::P_REL
                                                                                                      : roadmanager::Position::PosMode::P_ABS)) |
            (std::isnan(orientation.r_)
                 ? roadmanager::Position::PosMode::R_REL
                 : (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE ? roadmanager::Position::PosMode::R_REL
                                                                                                      : roadmanager::Position::PosMode::R_ABS)));

    position_.SetS(ds);
    position_.SetT(dt);

    if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
    {
        position_.SetHeadingRelative(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetPitchRelative(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetRollRelative(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE)
    {
        position_.SetH(std::isnan(orientation.h_) ? 0.0 : orientation.h_);
        position_.SetP(std::isnan(orientation.p_) ? 0.0 : orientation.p_);
        position_.SetR(std::isnan(orientation.r_) ? 0.0 : orientation.r_);
    }
    else
    {
        LOG("Unexpected orientation type: %d", orientation.type_);
    }

    position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_ROAD);
}

void OSCPositionRelativeRoad::Print()
{
    object_->pos_.Print();
}

OSCPositionRoute::OSCPositionRoute(roadmanager::Route *route, double s, int laneId, double laneOffset)
{
    (void)s;
    (void)laneId;
    (void)laneOffset;
    position_.SetRoute(route);
}

void OSCPositionRoute::SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset, OSCOrientation *orientation)
{
    position_.SetRouteLanePosition(route, pathS, laneId, laneOffset);

    // Adjust heading to road direction also considering traffic rule (left/right hand traffic)
    if (position_.GetDrivingDirectionRelativeRoad() < 0)
    {
        position_.SetHeadingRelative(GetAngleSum(M_PI, std::isnan(orientation->h_) ? 0.0 : orientation->h_));
        position_.SetPitchRelative(std::isnan(orientation->p_) ? 0.0 : -orientation->p_);
        position_.SetRollRelative(std::isnan(orientation->r_) ? 0.0 : -orientation->r_);
    }
    else
    {
        position_.SetHeadingRelative(std::isnan(orientation->h_) ? 0.0 : orientation->h_);
        position_.SetPitchRelative(std::isnan(orientation->p_) ? 0.0 : orientation->p_);
        position_.SetRollRelative(std::isnan(orientation->r_) ? 0.0 : orientation->r_);
    }
}

void OSCPositionRoute::SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset)
{
    position_.SetRouteLanePosition(route, pathS, laneId, laneOffset);
}

OSCPositionTrajectory::OSCPositionTrajectory(roadmanager::RMTrajectory *traj, double s, double t, OSCOrientation orientation)
{
    (void)orientation;
    (void)t;

    // do initial and temporary evaluation of the trajectory
    // it will be recalculated when related action is triggered
    // so that relative positions will be evaluated correctly
    traj->shape_->CalculatePolyLine();

    position_.SetTrajectory(traj);
    position_.SetTrajectoryS(s);
}
