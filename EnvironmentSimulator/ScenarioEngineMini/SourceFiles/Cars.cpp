#include "Cars.hpp"


Cars::Cars()
{
}

void Cars::addCar(Car car)
{
	cars.push_back(car);
}

Car Cars::getCar(std::string objectName)
{
	int idx = Cars::getIdx(objectName);
	return cars[idx];
}

Car Cars::getCar(int objectId)
{
	int idx = Cars::getIdx(objectId);
	return cars[idx];
}

int Cars::getId(std::string objectName)
{
	for (size_t i = 0; i < cars.size(); i++)
	{
		if (cars[i].getObjectName() == objectName)
		{
			return cars[i].getObjectId();
		}
	}
}

std::string Cars::getName(int objectId)
{
	for (size_t i = 0; i < cars.size(); i++)
	{
		if (cars[i].getObjectId() == objectId)
		{
			return cars[i].getObjectName();
		}
	}
}

int Cars::getIdx(std::string objectName)
{
	for (size_t i = 0; i < cars.size(); i++)
	{
		if (cars[i].getObjectName() == objectName)
		{
			return i;
		}
	}
}

int Cars::getIdx(int objectId)
{
	for (size_t i = 0; i < cars.size(); i++)
	{
		if (cars[i].getObjectId() == objectId)
		{
			return i;
		}
	}
}

roadmanager::Position Cars::getPosition(std::string objectName)
{
	int idx = Cars::getIdx(objectName);
	return cars[idx].getPosition();
}

roadmanager::Position Cars::getPosition(int objectId)
{
	int idx = Cars::getIdx(objectId);
	return cars[idx].getPosition();
}

double Cars::getSpeed(std::string objectName)
{
	int idx = Cars::getIdx(objectName);
	return cars[idx].getSpeed();
}

double Cars::getSpeed(int objectId)
{
	int idx = Cars::getIdx(objectId);
	return cars[idx].getSpeed();
}

void Cars::setSpeed(std::string objectName, double speed)
{
	int idx = Cars::getIdx(objectName);
	cars[idx].setSpeed(speed);
}

void Cars::setSpeed(int objectId, double speed)
{
	int idx = Cars::getIdx(objectId);
	cars[idx].setSpeed(speed);
}

void Cars::setPosition(std::string objectName, roadmanager::Position position)
{
	int idx = Cars::getIdx(objectName);
	cars[idx].setPosition(position);
}

void Cars::setPosition(int objectId, roadmanager::Position position)
{
	int idx = Cars::getIdx(objectId);
	cars[idx].setPosition(position);
}

void Cars::step(double dt)
{
	for (size_t i = 0; i < cars.size(); i++)
	{
		cars[i].step(dt);
	}
}

void Cars::step(std::string objectName, double dt)
{
	int idx = Cars::getIdx(objectName);
	cars[idx].step(dt);
}

void Cars::step(int objectId, double dt)
{
	int idx = Cars::getIdx(objectId);
	cars[idx].step(dt);
}

int Cars::getNum()
{
	return cars.size();
}