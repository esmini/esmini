#include "Cars.hpp"


Cars::Cars()
{
	//scenarioGateway = new ScenarioGateway();
}

void Cars::addCar(Car car)
{
	cars.push_back(car);
}

void Cars::addScenarioGateway(ScenarioGateway &scenarioGateway)
{
	this->scenarioGateway = &scenarioGateway;
}

Car Cars::getCar(std::string objectName)
{
	int idx = Cars::getIdx(objectName);
	return cars[idx];
}

Car * Cars::getCarPtr(std::string objectName)
{
	int idx = Cars::getIdx(objectName);
	return &cars[idx];
}

Car Cars::getCar(int objectId)
{
	int idx = Cars::getIdx(objectId);
	return cars[idx];
}

Car * Cars::getCarPtr(int objectId)
{
	int idx = Cars::getIdx(objectId);
	return &cars[idx];
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

	return -1;
}

void Cars::setObjectId(int objectId, int newObjectId)
{
	int idx = Cars::getIdx(objectId);
	return cars[idx].setObjectId(newObjectId);
}

void Cars::setObjectId(std::string objectName, int newObjectId)
{
	int idx = Cars::getIdx(objectName);
	return cars[idx].setObjectId(newObjectId);
}

void Cars::setExtControlled(int objectId, bool boolean)
{
	int idx = Cars::getIdx(objectId);
	cars[idx].setExtControlled(boolean);
}

void Cars::setExtControlled(std::string objectName, bool boolean)
{
	int idx = Cars::getIdx(objectName);
	cars[idx].setExtControlled(boolean);
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
	return "";
}

int Cars::getIdx(std::string objectName)
{
	for (size_t i = 0; i < cars.size(); i++)
	{
		if (cars[i].getObjectName() == objectName)
		{
			return (int)i;
		}
	}
	return -1;
}

int Cars::getIdx(int objectId)
{
	for (size_t i = 0; i < cars.size(); i++)
	{
		if (cars[i].getObjectId() == objectId)
		{
			return (int)i;
		}
	}
	return -1;
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


// Follow route
bool Cars::getFollowRoute(std::string objectName)
{
	int idx = Cars::getIdx(objectName);
	return cars[idx].getFollowRoute();
}

bool Cars::getFollowRoute(int objectId)
{
	int idx = Cars::getIdx(objectId);
	return cars[idx].getFollowRoute();
}

void Cars::setFollowRoute(std::string objectName, bool followRoute)
{
	int idx = Cars::getIdx(objectName);
	return cars[idx].setFollowRoute(followRoute);
}

void Cars::setFollowRoute(int objectId, bool followRoute)
{
	int idx = Cars::getIdx(objectId);
	return cars[idx].setFollowRoute(followRoute);
}


void Cars::step(double dt, double simulationTime)
{

	for (size_t i = 0; i < cars.size(); i++)
	{
		if (cars[i].getExtControlled())
		{
			int objectId = cars[i].getObjectId();
			ObjectState o;

			if (scenarioGateway->getObjectStateById(objectId, o) != 0)
			{
				std::cout << "Cars: Gateway did not provide state for external car " << objectId << std::endl;
			}
			else
			{
				if (o.getPosType() == GW_POS_TYPE_ROAD)
				{
					cars[i].getPositionPtr()->SetLanePos(o.getRoadId(), o.getLaneId(), o.getS(), o.getLaneOffset());
				}
				else if (o.getPosType() == GW_POS_TYPE_XYH)
				{
					cars[i].getPositionPtr()->SetXYH(o.getPosX(), o.getPosY(), o.getRotH());
				}
					
				// Calculate magnitude of speed
				double speed = sqrt(o.getVelX() * o.getVelX() + o.getVelY() * o.getVelY());
				
				// Find out direction of speed, going forward or backwards? Compare with heading
				double rotatedVelX = o.getVelX() * cos(-o.getRotH()) - o.getVelY() * sin(-o.getRotH());
				double rotatedVelY = o.getVelX() * sin(-o.getRotH()) + o.getVelY() * cos(-o.getRotH());
				
				int sign = rotatedVelX < 0 ? -1 : 1;

				cars[i].setSpeed(sign * speed);
			}
		}
		else {
			Car *c = &cars[i];

			c->step(dt);
		}
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
	return (int)cars.size();
}

void Cars::printCar()
{
	for (size_t i = 0; i < cars.size(); i++)
	{
		cars[i].printState();
	}
}

void Cars::printCar(int objectId)
{
	int idx = Cars::getIdx(objectId);
	cars[idx].printState();
}

