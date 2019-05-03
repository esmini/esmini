#pragma once
#include "OSCCondition.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace scenarioengine
{

	// Forward declaration
	class OSCCondition;

	class OSCConditionGroup
	{
	public:
		std::vector<OSCCondition*> condition_;

		void Print()
		{
			LOG("\t - ConditionGroup");
		}
	};

}