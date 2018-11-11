#include "Car.hpp"
#include "CommonMini.hpp"

Car::Car(){
	extControlled = false;
	followRoute = false;
	speed = 0;
}

void Car::setObjectId(int objectId)
{
	this->objectId = objectId;

}

void Car::setName(std::string objectName)
{
	this->objectName = objectName;
}

void Car::setPosition(roadmanager::Position position)
{
	this->position = position;
}

void Car::setExtControlled(bool boolean)
{
	extControlled = boolean;
}

void Car::setRoute(roadmanager::Route &route)
{
	this->route = route;
}

roadmanager::Route * Car::getRoute()
{
	return &route;
}

bool Car::getExtControlled()
{
	return extControlled;
}

void Car::step(double dt)
{
	if (followRoute)
	{
		route.MoveDS(speed * dt);
		route.GetPosition(&position);
	}
	else
	{
		position.MoveAlongS(speed * dt);
	}

}

void Car::setSpeed(double speed)
{
	this->speed = speed;
}

void Car::setObjectStruct(Entities::ObjectStruct objectStruct)
{
	this->objectStruct = objectStruct;
}

void Car::setOffset(double offset)
{

	int roadId = position.GetTrackId();
	int laneId = position.GetLaneId();
	double s = position.GetS();

	position.SetLanePos(roadId, laneId, s, offset);
}

bool Car::getFollowRoute()
{
	return followRoute;
}

void Car::setFollowRoute(bool followRoute)
{
	this->followRoute = followRoute;
}

double Car::getSpeed()
{
	return speed;
}

roadmanager::Position Car::getPosition()
{
	return position;
}

roadmanager::Position * Car::getPositionPtr()
{
	return &position;
}

int Car::getObjectId()
{
	return objectId;
}

std::string Car::getObjectName()
{
	return objectName;
}

void Car::printState()
{
	LOG("name: %s", objectName);
	LOG("actorId: %s", objectId);

	std::string posType = "Lane";

	// Can be done with cases instead
	if (posType == "World")
	{
		LOG("Position - World");

		LOG("\tx = %.2f", position.GetX());
		LOG("\ty = %.2f", position.GetY());
		LOG("\tz = %.2f", position.GetZ());
		LOG("\th = %.2f", position.GetH());
		LOG("\tp = %.2f", position.GetP());
		LOG("\tr = %.2f", position.GetR());

	}
	else if (posType == "Lane")
	{
		LOG("Position - Lane");

		LOG("\troadId = %d", position.GetTrackId());
		LOG("\tlaneId = %d", position.GetLaneId());
		LOG("\toffset = %.2f", position.GetOffset());
		LOG("\ts = %.2f", position.GetS());
	}

	LOG("\tspeed = ", speed);

	LOG("---------------------------------------");
}

