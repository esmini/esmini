#pragma once
#include "OSCCatalogReference.hpp"
#include "OSCManeuver.hpp"
#include "OSCConditionGroup.hpp"

#include <iostream>
#include <string>
#include <vector>

class Story
{
public:
	Story();
	void Print();

	struct Sequence
	{
		typedef struct 
		{
			std::string entity_;
			// By condition not supported yet
		} Actor;
		std::vector<Actor*> actor_;

		// std::vector<OSCCatalogReference> catalog_reference_;  // not supported yet
		std::vector<OSCManeuver*> maneuver_;

		int number_of_executions_;
		std::string name_;
	};

	struct Act
	{
		std::vector<Sequence*> sequence_;
		std::vector<OSCConditionGroup*> start_condition_group_;
		std::vector<OSCConditionGroup*> end_condition_group_;
		std::vector<OSCConditionGroup*> cancel_condition_group_;
		
		std::string name_;
	};

	std::vector<Act*> act_;

	std::string owner_;
	std::string name_;

	void Step(double dt);
};	



