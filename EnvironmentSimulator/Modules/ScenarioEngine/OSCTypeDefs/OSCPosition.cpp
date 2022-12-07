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

OSCPositionWorld::OSCPositionWorld(double x, double y, double z, double h, double p, double r, OSCPosition* base_on_pos) : OSCPosition(PositionType::WORLD)
{
	if (base_on_pos != nullptr && base_on_pos->type_ == PositionType::WORLD)
	{
		this->position_ = *base_on_pos->GetRMPos();
	}

	if (!std::isnan(z) || !std::isnan(p) || !std::isnan(r))
	{
		if (std::isnan(z) || std::isnan(p) || std::isnan(r))
		{
			LOG("At least one of z, pitch (p) and roll (r) is set. Remaining will be set to zero.");
		}

		if (std::isnan(z))
		{
			z = 0.0;
		}

		if (std::isnan(p))
		{
			// Indicate that this attribute has not been set explicitly in the scenario
			position_.SetOrientationTypeSetBit(roadmanager::Position::OrientationSetMask::P, 0);
			p = 0.0;  // set default value
		}

		if (std::isnan(r))
		{
			// Indicate that this attribute has not been set explicitly in the scenario
			position_.SetOrientationTypeSetBit(roadmanager::Position::OrientationSetMask::R, 0);
			r = 0.0;  // set default value
		}

		if (std::isnan(h))
		{
			// Indicate that this attribute has not been set explicitly in the scenario
			position_.SetOrientationTypeSetBit(roadmanager::Position::OrientationSetMask::H, 0);
			h = 0.0;  // set default value
		}

		position_.SetInertiaPos(x, y, z, h, p, r);
	}
	else
	{
		if (std::isnan(h))
		{
			// Indicate that this attribute has not been set explicitly in the scenario
			position_.SetOrientationTypeSetBit(roadmanager::Position::OrientationSetMask::H, 0);
			h = 0.0;  // set default value
		}

		position_.SetInertiaPos(x, y, h);
	}
}

OSCPositionLane::OSCPositionLane(int roadId, int laneId, double s, double offset, OSCOrientation orientation) :
	OSCPosition(PositionType::LANE)
{
	position_.SetOrientationType(orientation.type_);

	position_.SetLanePos(roadId, laneId, s, offset);

	if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
	{
		// Adjust heading to road direction also considering traffic rule (left/right hand traffic)
		if (position_.GetDrivingDirectionRelativeRoad() < 0)
		{
			position_.SetHeadingRelative(GetAngleSum(M_PI, orientation.h_));
			position_.SetPitchRelative(-orientation.p_);
			position_.SetRollRelative(-orientation.r_);
		}
		else
		{
			position_.SetHeadingRelative(orientation.h_);
			position_.SetPitchRelative(orientation.p_);
			position_.SetRollRelative(orientation.r_);
		}
	}
	else if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE)
	{
		position_.SetHeading(orientation.h_);
	}
	else
	{
		LOG("Unexpected orientation type: %d", orientation.type_);
	}
}

OSCPositionRoad::OSCPositionRoad(int roadId, double s, double t, OSCOrientation orientation) :
	OSCPosition(PositionType::ROAD)
{
	if (position_.GetRoadById(roadId) == nullptr)
	{
		LOG_AND_QUIT("Reffered road ID %d not available in road network", roadId);
	}

	position_.SetOrientationType(orientation.type_);

	position_.SetTrackPos(roadId, s, t);

	if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
	{
		position_.SetHeadingRelative(orientation.h_);
		position_.SetPitchRelative(orientation.p_);
		position_.SetRollRelative(orientation.r_);
		position_.EvaluateOrientation();
	}
	else if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE)
	{
		position_.SetHeading(orientation.h_);
		position_.SetPitch(orientation.p_);
		position_.SetRoll(orientation.r_);
	}
	else
	{
		LOG("Unexpected orientation type: %d", orientation.type_);
	}
}

OSCPositionRelativeObject::OSCPositionRelativeObject(Object *object, double dx, double dy, double dz, OSCOrientation orientation) :
	OSCPosition(PositionType::RELATIVE_OBJECT), object_(object)
{
	position_.SetX(dx);
	position_.SetY(dy);
	position_.SetZ(dz);
	position_.SetOrientationType(orientation.type_);
	position_.SetH(orientation.h_);
	position_.SetP(orientation.p_);
	position_.SetR(orientation.r_);

	position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_OBJECT);
}

void OSCPositionRelativeObject::Print()
{
	object_->pos_.Print();
}


OSCPositionRelativeWorld::OSCPositionRelativeWorld(Object* object, double dx, double dy, double dz, OSCOrientation orientation) :
	OSCPosition(PositionType::RELATIVE_WORLD), object_(object)
{
	position_.SetX(dx);
	position_.SetY(dy);
	position_.SetZ(dz);
	position_.SetOrientationType(orientation.type_);
	position_.SetH(orientation.h_);
	position_.SetP(orientation.p_);
	position_.SetR(orientation.r_);

	position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_WORLD);
}

void OSCPositionRelativeWorld::Print()
{
	object_->pos_.Print();
}

OSCPositionRelativeLane::OSCPositionRelativeLane(Object *object, int dLane, double ds, double offset, OSCOrientation orientation) :
	OSCPosition(PositionType::RELATIVE_LANE), object_(object)
{
	position_.SetLaneId(dLane);
	position_.SetS(ds);
	position_.SetOffset(offset);
	position_.SetOrientationType(orientation.type_);

	if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
	{
		position_.SetHeadingRelative(orientation.h_);
		position_.SetPitchRelative(orientation.p_);
		position_.SetRollRelative(orientation.r_);
		position_.EvaluateOrientation();
	}
	else
	{
		position_.SetHeading(orientation.h_);
		position_.SetPitch(orientation.p_);
		position_.SetRoll(orientation.r_);
	}

	position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_LANE);
}

void OSCPositionRelativeLane::Print()
{
	object_->pos_.Print();
}

OSCPositionRelativeRoad::OSCPositionRelativeRoad(Object* object, double ds, double dt, OSCOrientation orientation) :
	OSCPosition(PositionType::RELATIVE_ROAD), object_(object)
{
	position_.SetS(ds);
	position_.SetT(dt);
	position_.SetOrientationType(orientation.type_);

	if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
	{
		position_.SetHeadingRelative(orientation.h_);
		position_.SetPitchRelative(orientation.p_);
		position_.SetRollRelative(orientation.r_);
		position_.EvaluateOrientation();
	}
	else if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE)
	{
		position_.SetHeading(orientation.h_);
		position_.SetPitch(orientation.p_);
		position_.SetRoll(orientation.r_);
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
		position_.SetHeadingRelative(GetAngleSum(M_PI, orientation->h_));
		position_.SetPitchRelative(-orientation->p_);
		position_.SetRollRelative(-orientation->r_);
	}
	else
	{
		position_.SetHeadingRelative(orientation->h_);
		position_.SetPitchRelative(orientation->p_);
		position_.SetRollRelative(orientation->r_);
	}
}

void OSCPositionRoute::SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset)
{
	position_.SetRouteLanePosition(route, pathS, laneId, laneOffset);
}

OSCPositionTrajectory::OSCPositionTrajectory(roadmanager::RMTrajectory* traj, double s, double t, OSCOrientation orientation)
{
	(void)orientation;
	(void)t;
	position_.SetTrajectory(traj);
	position_.SetTrajectoryS(s);
}
