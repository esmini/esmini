#include "OSCPosition.hpp"

using namespace scenarioengine;

OSCPositionWorld::OSCPositionWorld(double x, double y, double z, double h, double p, double r) : OSCPosition(PositionType::WORLD)
{
	position_.SetInertiaPos(x, y, z, h, p, r);
}

OSCPositionLane::OSCPositionLane(int roadId, int laneId, double s, double offset, OSCOrientation orientation) : OSCPosition(PositionType::LANE)
{
	position_.SetLanePos(roadId, laneId, s, offset);
}

OSCPositionRelativeObject::OSCPositionRelativeObject(Object *object, double dx, double dy, double dz, OSCOrientation orientation) : 
	OSCPosition(PositionType::RELATIVE_OBJECT), object_(object), dx_(dx), dy_(dy), dz_(dz), o_(orientation)
{
	Evaluate();	
}

void OSCPositionRelativeObject::Evaluate()
{
	position_.SetInertiaPos(object_->pos_.GetX() + dx_, object_->pos_.GetY() + dy_, object_->pos_.GetZ() + dz_,
		object_->pos_.GetH() + o_.h_, object_->pos_.GetP() + o_.p_, object_->pos_.GetR() + o_.r_);
}

void OSCPositionRelativeObject::Print()
{
	LOG("");
	object_->pos_.Print();
	LOG("dx: %.2f dy: %.2f dz: %.2f", dx_, dy_, dz_);
	LOG("orientation: h %.2f p %.2f r %.2f %s", o_.h_, o_.p_, o_.r_, o_.type_ == OSCOrientation::OrientationType::ABSOLUTE ? "Absolute" : "Relative");
}

OSCPositionRoute::OSCPositionRoute(roadmanager::Route *route, double s, int laneId, double laneOffset)
{
	position_.SetRoute(route);

}