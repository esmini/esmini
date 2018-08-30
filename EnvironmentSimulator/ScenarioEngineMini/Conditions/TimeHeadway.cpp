#include "TimeHeadway.hpp"


TimeHeadway::TimeHeadway(OSCCondition &condition, std::vector<Car> &carVector, std::vector<int> storyId, std::vector<std::string> &actionEntities)
{
	this->condition = condition;
	carVectorPtr = &carVector;
	this->storyId = storyId;
	this->actionEntities = actionEntities;

	N = condition.ByEntity.TriggeringEntities.Entity.size();

	// Triggering entities
	triggeringEntityIds.resize(N); 
	triggeringEntityPos.resize(N);

	// Headwaytime
	headwayTimeOld.resize(N);
	headwayTimeNew.resize(N);
	triggs.resize(N, false);

	// Get Ids
	for (size_t i = 0; i < N; i++) { triggeringEntityIds[i] = getObjectId(condition.ByEntity.TriggeringEntities.Entity[i].name); }
	entityId = getObjectId(condition.ByEntity.EntityCondition.TimeHeadway.entity);
}


int TimeHeadway::getObjectId(std::string objectName)
{
	int objectId = -1;

	for (size_t i = 0; i < (*carVectorPtr).size(); i++)
	{
		if ((*carVectorPtr)[i].getObjectName() == objectName)
		{
			objectId = (*carVectorPtr)[i].getObjectId();
		}
	}

	return objectId;
}


bool TimeHeadway::checkTimeHeadway()
{

	for (size_t i = 0; i < N; i++)
	{
		triggeringEntityPos[i] = (*carVectorPtr)[triggeringEntityIds[i]].getPosition();
	}

	//  objectId of objects that are measuaring from
	entityPos = (*carVectorPtr)[entityId].getPosition();
	entitySpeed = (*carVectorPtr)[entityId].getSpeed();


	// Check which object that are allowed to cause a trigg
	headwayTimeOld = headwayTimeNew;


	if (condition.ByEntity.EntityCondition.TimeHeadway.alongRoute == "true")
	{

		// Take into account the dimensions of the vehicle
		if (condition.ByEntity.EntityCondition.TimeHeadway.freespace == "true")
		{
			// Go for road coordinates
			if (condition.ByEntity.EntityCondition.TimeHeadway.alongRoute == "true")
			{
				for (size_t i = 0; i < N; i++)
				{
					headwayTimeNew[i] = (entityPos.GetS() - triggeringEntityPos[i].GetS()) / (entitySpeed / 3.6);
					std::cout << "ScenarioEngine: headwayTime is " << headwayTimeNew[i] << std::endl;

					if (condition.ByEntity.EntityCondition.TimeHeadway.rule == "greater_than")
					{

						if (headwayTimeNew[i] > std::stod(condition.ByEntity.EntityCondition.TimeHeadway.value))
						{
							if (condition.edge == "rising")
							{
								if (headwayTimeOld[i] < std::stod(condition.ByEntity.EntityCondition.TimeHeadway.value))
								{
									triggs[i] = true;
								}
							}
						}
					}
				}
			}
		}
	}


	if (condition.ByEntity.TriggeringEntities.rule == "any")
	{
		for (size_t i = 0; i < N; i++)
		{
			if (triggs[i])
			{
				std::cout << "ScenarioEngine: " << condition.name << " has triggered" << std::endl;
				std::fill(triggs.begin(), triggs.end(), false);
				return true;
			}
		}
	}
	return false;
}

//TimeHeadway::~TimeHeadway()
//{
//}
