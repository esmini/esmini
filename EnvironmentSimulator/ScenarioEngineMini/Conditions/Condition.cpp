#include "Condition.hpp"


Condition::Condition(OSCCondition &condition, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities)
{
	this->condition = condition;
	this->carsPtr = &cars;
	this->storyId = storyId;
	this->actionEntities = actionEntities;
	identifyConditionType(condition);
}

void Condition::identifyConditionType(OSCCondition &condition)
{
	if (!condition.ByEntity.EntityCondition.TimeHeadway.entity.empty())
	{
		this->conditionType = "TimeHeadway";

		// Triggering entities		(TriggeringEntities -> Entity -> name="Ego")
		this->N = condition.ByEntity.TriggeringEntities.Entity.size();
		this->triggeringEntityPos.resize(N);
		this->triggeringEntities.resize(N);

		// Get triggering entities
		for (size_t i = 0; i < N; i++)
		{
			this->triggeringEntities[i] = condition.ByEntity.TriggeringEntities.Entity[i].name;
		}

		// Get entity				(EntityCondition -> TimeHeadway -> entity="$owner" (LaneChanger))
		this->entity = condition.ByEntity.EntityCondition.TimeHeadway.entity;

		// Headwaytime
		headwayTimeOld.resize(N);
		headwayTimeNew.resize(N);
		triggs.resize(N, false);
	}
}

bool Condition::checkCondition()
{
	if (conditionType == "TimeHeadway")
	{
		return checkTimeHeadway();
	}
	else if (conditionType == "SomeThingElse")
	{
		return false;
	}

	return false;
}

bool Condition::checkTimeHeadway()
{

	for (size_t i = 0; i < N; i++)
	{
		triggeringEntityPos[i] = (*carsPtr).getPosition(triggeringEntities[i]);
	}

	entityPos = (*carsPtr).getPosition(entity);
	entitySpeed = (*carsPtr).getSpeed(entity);

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
					//std::cout << "ScenarioEngine: headwayTime is " << headwayTimeNew[i] << std::endl;

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

std::vector<int> Condition::getStoryId()
{
	return storyId;
}

