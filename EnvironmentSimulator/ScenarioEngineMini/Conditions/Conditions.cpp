#include "Conditions.hpp"


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
			std::cout << "Conditions: " << " condition " << i << " is removed from conditions" << "\n" << std::endl;
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

