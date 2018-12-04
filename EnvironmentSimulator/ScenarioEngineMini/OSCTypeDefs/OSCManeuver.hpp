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

namespace scenarioengine
{

	class Event
	{
	public:

		typedef enum
		{
			OVERWRITE,
			FOLLOWING,
			SKIP
		} Priority;

		typedef enum
		{
			NOT_TRIGGED,
			ACTIVE,
			DONE,
			WAITING,  // Following
			SKIPPED,
			CANCELLED,
			JUST_TERMINATED,
		} State;

		State state_;
		std::string name_;
		Priority priority_;

		std::vector<OSCAction*> action_;

		std::vector<OSCConditionGroup*> start_condition_group_;

		Event() : state_(State::NOT_TRIGGED) {}

		void Activate();
		void Deactivate();
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

}