#include "TimeHeadway.hpp"


TimeHeadway::TimeHeadway(OSCCondition &condition, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities)
{
	this->condition = condition;
	carsPtr = &cars;
	this->storyId = storyId;
	this->actionEntities = actionEntities;

	N = condition.ByEntity.TriggeringEntities.Entity.size();

	// Triggering entities
	triggeringEntityPos.resize(N);
	triggeringEntities.resize(N);

	// Headwaytime
	headwayTimeOld.resize(N);
	headwayTimeNew.resize(N);
	triggs.resize(N, false);

	// Get triggering entities
	for (size_t i = 0; i < N; i++) 
	{
		triggeringEntities[i] = condition.ByEntity.TriggeringEntities.Entity[i].name;
	}

	entity = condition.ByEntity.EntityCondition.TimeHeadway.entity;
}


bool TimeHeadway::checkTimeHeadway()
{

	for (size_t i = 0; i < N; i++)
	{
		triggeringEntityPos[i] = (*carsPtr).getPosition(triggeringEntities[i]);
	}

	//  objectId of objects that are measuaring from
	entityPos = (*carsPtr).getPosition(entity);
	entitySpeed = (*carsPtr).getSpeed(entity);

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

