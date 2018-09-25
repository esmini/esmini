#include "Car.hpp"

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
	//std::cout << "Car: setPosition started" << std::endl;
	this->position = position;
	//std::cout << "Car: setPosition finished" << std::endl;
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
	//std::cout << "Car: setSpeed started" << std::endl;
	this->speed = speed;
	//std::cout << "Car: setSpeed finished" << std::endl;

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
	std::cout << "name: " << objectName << std::endl;
	std::cout << "actorId: " << objectId << std::endl;
	std::cout << std::endl;

	std::string posType = "Lane";

	// Can be done with cases instead
	if (posType == "World")
	{
		std::cout << "Position - World" << "\n" << std::endl;

		std::cout << "\t" << "x = " << position.GetX() << std::endl;
		std::cout << "\t" << "y = " << position.GetY() << std::endl;
		std::cout << "\t" << "z = " << position.GetZ() << std::endl;
		std::cout << "\t" << "h = " << position.GetH() << std::endl;
		std::cout << "\t" << "p = " << position.GetP() << std::endl;
		std::cout << "\t" << "r = " << position.GetR() << std::endl;

	}
	else if (posType == "Lane")
	{
		std::cout << "Position - Lane" << std::endl;

		std::cout << "\t" << "roadId = " << position.GetTrackId() << std::endl;
		std::cout << "\t" << "laneId = " << position.GetLaneId() << std::endl;
		std::cout << "\t" << "offset = " << position.GetOffset() << std::endl;
		std::cout << "\t" << "s = " << position.GetS() << std::endl;
	}

	std::cout << "\t" << "speed = " << speed << std::endl;

	std::cout << "---------------------------------------" << std::endl;
	std::cout << std::endl;
}

