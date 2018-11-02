#pragma once
#include "OSCGlobalAction.hpp"
#include "OSCUserDefinedAction.hpp"
#include "OSCPrivateAction.hpp"
#include "OSCParameterDeclaration.hpp"
#include "OSCConditionGroup.hpp"

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
		std::cout << "\t" << "name = " << name << std::endl;
		std::cout << std::endl;

		for (size_t i = 0; i < Event.size(); i++)
		{
			std::cout << "\t" << " - Event " << std::endl;
			std::cout << "\t" << "name = " << Event[i].name << std::endl;
			std::cout << "\t" << "priority = " << Event[i].priority << std::endl;
			std::cout << std::endl;

			for (size_t j = 0; j < Event[i].Action.size(); j++)
			{
				std::cout << "\t" << " - Event - Action" << std::endl;
				std::cout << "\t" << "name = " << Event[i].Action[j].name << std::endl;
				std::cout << std::endl;

				std::cout << "\t" << " - Event - Action - Private" << std::endl;
				Event[i].Action[j].Private.printOSCPrivateAction();
			}
		}
	};
};