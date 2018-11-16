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

class Event
{
public:
	bool active;
	std::string name_;
	std::string priority_; // Wrong type

	std::vector<OSCAction*> action_;
	
	std::vector<OSCConditionGroup*> start_condition_group_;

	Event() : active(false) {}
};


class OSCManeuver
{
public:
	OSCParameterDeclaration parameter_declaration_;
	std::vector<Event*> event_;
	std::string name_;

	void Print()
	{
		LOG("\tname = %s", name_.c_str());
	};
};