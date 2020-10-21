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

OSCPositionWorld::OSCPositionWorld(double x, double y, double z, double h, double p, double r) : OSCPosition(PositionType::WORLD)
{
	position_.SetInertiaPos(x, y, z, h, p, r);
}

OSCPositionLane::OSCPositionLane(int roadId, int laneId, double s, double offset, OSCOrientation orientation) : 
	OSCPosition(PositionType::LANE)
{
	position_.SetOrientationType(orientation.type_);

	position_.SetLanePos(roadId, laneId, s, offset);

	if (orientation.type_ == roadmanager::Position::OrientationType::ORIENTATION_RELATIVE)
	{
		// Adjust heading to road direction 
		if (position_.GetLaneId() < 0)
		{
			position_.SetHeadingRelative(orientation.h_);
		}
		else
		{
			position_.SetHeadingRelative(GetAngleSum(M_PI, orientation.h_));
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
	position_.SetP(orientation.p_);
	position_.SetR(orientation.r_);
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
	LOG("");
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
	LOG("");
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
	}
	else
	{
		position_.SetH(orientation.h_);
	}
	position_.SetP(orientation.p_);
	position_.SetR(orientation.r_);

	position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_LANE);
}

void OSCPositionRelativeLane::Print()
{
	LOG("");
	object_->pos_.Print();
}

OSCPositionRoute::OSCPositionRoute(roadmanager::Route *route, double s, int laneId, double laneOffset)
{
	position_.SetRoute(route);
}

void OSCPositionRoute::SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset, OSCOrientation *orientation)
{
	position_.SetRouteLanePosition(route, pathS, laneId, laneOffset);
	position_.SetHeading(orientation->h_);
}

void OSCPositionRoute::SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset)
{
	position_.SetRouteLanePosition(route, pathS, laneId, laneOffset);
}
