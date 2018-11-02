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
		std::cout << "\t" << " - ConditionGroup" << std::endl;

		for (size_t i = 0; i < Condition.size(); i++)
		{
			std::cout << "\t" << " - ConditionGroup - Condition" << std::endl;
			Condition[i].printOSCCondition();
		}
	}
};
