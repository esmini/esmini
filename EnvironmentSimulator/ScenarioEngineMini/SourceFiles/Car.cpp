#include "Car.hpp"

Car::Car(){}

void Car::setObjectId(int objectId)
{
	this->objectId = objectId;

}

void Car::setName(std::string objectName)
{
	this->objectName = objectName;
}

void Car::setPosition(OSCPosition position, std::string posType)
{
	std::cout << "Car: setPosition started" << std::endl;
	this->position = position;
	this->posType = posType;
	std::cout << "Car: setPosition finished" << std::endl;
}


void Car::step(double dt)
{
	// Can be done with cases instead
	if (posType == "World")
	{
		position.World.x = position.World.x + cos(position.World.h) * (speed/3.6) * dt;
		position.World.y = position.World.y + sin(position.World.h) * (speed/3.6) * dt;
	}
	else if (posType == "Lane")
	{
		position.Lane.s = position.Lane.s + (speed/3.6) * dt;
	}
}

void Car::setSpeed(double speed)
{
	std::cout << "Car: setSpeed started" << std::endl;
	this->speed = speed;
	std::cout << "Car: setSpeed finished" << std::endl;

}

void Car::setOffset(double offset)
{
	position.Lane.offset = offset;
}

double Car::getSpeed()
{
	return speed;
}

OSCPosition Car::getPosition()
{
	return position;
}

std::string Car::getPositionType()
{
	return posType;
}

double Car::getOffset()
{
	return position.Lane.offset;
}

double Car::getObjectId()
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


	// Can be done with cases instead
	if (posType == "World")
	{
		std::cout << "Position - World" << "\n" << std::endl;

		std::cout << "\t" << "x = " << position.World.x << std::endl;
		std::cout << "\t" << "y = " << position.World.y << std::endl;
		std::cout << "\t" << "z = " << position.World.z << std::endl;
		std::cout << "\t" << "h = " << position.World.h << std::endl;
		std::cout << "\t" << "p = " << position.World.p << std::endl;
		std::cout << "\t" << "r = " << position.World.r << std::endl;

	}
	else if (posType == "Lane")
	{
		std::cout << "Position - Lane" << std::endl;

		std::cout << "\t" << "roadId = " << position.Lane.roadId << std::endl;
		std::cout << "\t" << "laneId = " << position.Lane.laneId << std::endl;
		std::cout << "\t" << "offset = " << position.Lane.offset << std::endl;
		std::cout << "\t" << "s = " << position.Lane.s << std::endl;
	}

	std::cout << "\t" << "speed = " << speed << std::endl;



	std::cout << "---------------------------------------" << std::endl;
	std::cout << std::endl;
}

