#pragma once
#include "OSCGlobalAction.hpp"
#include "OSCUserDefinedAction.hpp"
#include "OSCPrivateAction.hpp"
#include "OSCParameterDeclaration.hpp"
#include "OSCConditionGroup.hpp"
#include "CommonMini.hpp"

#include <iostream>
#include <string>
#include <vector>

struct ActionStruct
{
	OSCGlobalAction Global;
	OSCUserDefinedAction UserDefined;
	OSCPrivateAction Private;

	std::string name;
};

struct EventStruct
{
	std::string name;
	std::string priority; // Wrong type

	std::vector<ActionStruct> Action;
	
	struct
	{
		std::vector<OSCConditionGroup> ConditionGroup;
	}StartConditions;

};


class OSCManeuver
{
public:
	OSCParameterDeclaration ParameterDeclaration;
	std::vector<EventStruct> Event;
	std::string name;

	void printOSCManeuver()
	{
		LOG("\tname = %s", name.c_str());

		for (size_t i = 0; i < Event.size(); i++)
		{
			LOG("\t - Event ");
			LOG("\tname = %s", Event[i].name.c_str());
			LOG("\tpriority = %s", Event[i].priority.c_str());

			for (size_t j = 0; j < Event[i].Action.size(); j++)
			{
				LOG("\t - Event - Action");
				LOG("\tname = %s", Event[i].Action[j].name.c_str());
				LOG("\t - Event - Action - Private");
				Event[i].Action[j].Private.printOSCPrivateAction();
			}
		}
	};
};