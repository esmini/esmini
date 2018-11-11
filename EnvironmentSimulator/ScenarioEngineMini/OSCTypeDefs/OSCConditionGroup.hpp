#pragma once
#include "OSCCondition.hpp"

#include <iostream>
#include <string>
#include <vector>


class OSCConditionGroup
{
public:
	std::vector<OSCCondition> Condition;

	void printOSCConditionGroup()
	{
		LOG("\t - ConditionGroup");

		for (size_t i = 0; i < Condition.size(); i++)
		{
			LOG("\t - ConditionGroup - Condition");
			Condition[i].printOSCCondition();
		}
	}
};
