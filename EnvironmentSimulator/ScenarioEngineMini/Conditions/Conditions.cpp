#include "Conditions.hpp"
#include "CommonMini.hpp"


Conditions::Conditions()
{
}

void Conditions::addCondition(Condition condition)
{
	conditions.push_back(condition);
}

bool Conditions::checkConditions()
{
	bool conditionTrigged = false;

	for (size_t i = 0; i < conditions.size(); i++)
	{

		conditionTrigged = conditions[i].checkCondition();

		if (conditionTrigged)
		{
			lastTriggeredStoryId = conditions[i].getStoryId();
			LOG("Condition %d is removed from conditions", i);
			conditions.erase(conditions.begin() + i);
			return conditionTrigged;
		}
	}
	return conditionTrigged;
}

std::vector<int> Conditions::getLastTriggeredStoryId()
{
	return lastTriggeredStoryId;
}

