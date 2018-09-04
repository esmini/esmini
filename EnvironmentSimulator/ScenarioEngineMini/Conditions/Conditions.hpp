#pragma once
#include "OSCCondition.hpp"
#include "Condition.hpp"
#include "Cars.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>


class Conditions
{
public:
	Conditions();

	void addCondition(Condition condition);
	bool checkConditions();
	std::vector<int> getLastTriggeredStoryId();

private:
	std::vector<Condition> conditions;
	std::vector<int> lastTriggeredStoryId;
};

