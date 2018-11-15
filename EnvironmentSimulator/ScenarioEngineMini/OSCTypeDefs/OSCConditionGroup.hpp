#pragma once
#include "OSCCondition.hpp"

#include <iostream>
#include <string>
#include <vector>


class OSCConditionGroup
{
public:
	std::vector<OSCCondition*> condition_;

	void Print()
	{
		LOG("\t - ConditionGroup");
	}
};
