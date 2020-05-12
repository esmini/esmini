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

OSCPositionLane::OSCPositionLane(int roadId, int laneId, double s, double offset, OSCOrientation orientation) : OSCPosition(PositionType::LANE)
{
	position_.SetLanePos(roadId, laneId, s, offset);

	// If heading not set explicitly, then align relative heading to driving direction, assuming right-hand traffic
	if (orientation.type_ == OSCOrientation::OrientationType::UNDEFINED)
	{
		if (laneId < 0)
		{
			position_.SetHeadingRelative(0);
		}
		else
		{
			position_.SetHeadingRelative(M_PI);
		}
	}
	else if (orientation.type_ == OSCOrientation::OrientationType::ABSOLUTE)
	{
		position_.SetHeading(orientation.h_);
	}
	else if (orientation.type_ == OSCOrientation::OrientationType::RELATIVE)
	{
		LOG("Relative orientation not supported yet");
	}
	else
	{
		LOG("Unexpected orientation type: %d", orientation.type_);
	}

}

OSCPositionRelativeObject::OSCPositionRelativeObject(Object *object, double dx, double dy, double dz, OSCOrientation orientation) : 
	OSCPosition(PositionType::RELATIVE_OBJECT), object_(object), dx_(dx), dy_(dy), dz_(dz), o_(orientation)
{
	position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_OBJECT);
}

void OSCPositionRelativeObject::Evaluate()
{
	return;
}

void OSCPositionRelativeObject::Print()
{
	LOG("");
	object_->pos_.Print();
	LOG("dx: %.2f dy: %.2f dz: %.2f", dx_, dy_, dz_);
	LOG("orientation: h %.2f p %.2f r %.2f %s", o_.h_, o_.p_, o_.r_, o_.type_ == OSCOrientation::OrientationType::ABSOLUTE ? "Absolute" : "Relative");
}


OSCPositionRelativeWorld::OSCPositionRelativeWorld(Object* object, double dx, double dy, double dz, OSCOrientation orientation) :
	OSCPosition(PositionType::RELATIVE_WORLD), object_(object), dx_(dx), dy_(dy), dz_(dz), o_(orientation)
{
	position_.SetRelativePosition(&object->pos_, roadmanager::Position::PositionType::RELATIVE_WORLD);
}

void OSCPositionRelativeWorld::Evaluate()
{
	if (o_.type_ == OSCOrientation::OrientationType::ABSOLUTE)
	{
		position_.SetInertiaPos(object_->pos_.GetX() + dx_, object_->pos_.GetY() + dy_, object_->pos_.GetZ() + dz_,
			o_.h_, o_.p_, o_.r_);
	}
	else
	{
		position_.SetInertiaPos(object_->pos_.GetX() + dx_, object_->pos_.GetY() + dy_, object_->pos_.GetZ() + dz_,
			object_->pos_.GetH() + o_.h_, object_->pos_.GetP() + o_.p_, object_->pos_.GetR() + o_.r_);
	}
}

void OSCPositionRelativeWorld::Print()
{
	LOG("");
	object_->pos_.Print();
	LOG("dx: %.2f dy: %.2f dz: %.2f", dx_, dy_, dz_);
	LOG("orientation: h %.2f p %.2f r %.2f %s", o_.h_, o_.p_, o_.r_, o_.type_ == OSCOrientation::OrientationType::ABSOLUTE ? "Absolute" : "Relative");
}

OSCPositionRelativeLane::OSCPositionRelativeLane(Object *object, int dLane, double ds, double offset, OSCOrientation orientation) :
	OSCPosition(PositionType::RELATIVE_LANE), object_(object), dLane_(dLane), ds_(ds), offset_(offset), o_(orientation)
{
}

void OSCPositionRelativeLane::Evaluate()
{
	position_.SetLanePos(object_->pos_.GetTrackId(), object_->pos_.GetLaneId() + dLane_, object_->pos_.GetS() + ds_, offset_);

	if (o_.type_ == OSCOrientation::OrientationType::ABSOLUTE)
	{
		position_.SetHeading(o_.h_);
	}
	else if (o_.type_ == OSCOrientation::OrientationType::RELATIVE)
	{
		position_.SetHeading(GetAngleSum(o_.h_, object_->pos_.GetH()));
	}
}

void OSCPositionRelativeLane::Print()
{
	LOG("");
	object_->pos_.Print();
	LOG("dLane: %d ds: %.2f offset: %.2f", dLane_, ds_, offset_);
	LOG("orientation: h %.2f p %.2f r %.2f %s", o_.h_, o_.p_, o_.r_, o_.type_ == OSCOrientation::OrientationType::ABSOLUTE ? "Absolute" : "Relative");
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
