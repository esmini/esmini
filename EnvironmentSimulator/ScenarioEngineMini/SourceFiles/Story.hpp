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
	void printStory();

	struct EntityStruct
	{
		std::string name;
	};

	struct SequenceStruct
	{
		struct 
		{
			std::vector<EntityStruct> Entity;

			struct
			{
				std::string actor; // Wrong type
			}ByCondition;

		}Actors;

		std::vector<OSCCatalogReference> CatalogReference;
		std::vector<OSCManeuver> Maneuver;

		int numberOfExecutions = 0;
		std::string name;
	};

	struct ActStruct
	{
		std::vector<SequenceStruct> Sequence;
		struct
		{
			struct
			{
				std::vector<OSCConditionGroup> ConditionGroup;
			}start;
			struct
			{
				std::vector<OSCConditionGroup> ConditionGroup;
			}End;
			struct
			{
				std::vector<OSCConditionGroup> ConditionGroup;
			}Cancel;
		}Conditions;
		
		std::string name;
	};

	std::vector<ActStruct> Act;
	std::string owner;
	std::string name;



};	



