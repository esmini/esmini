/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once
#include <iostream>
#include "OSCAction.hpp"
#include "CommonMini.hpp"
#include "Parameters.hpp"

namespace scenarioengine
{

	class OSCGlobalAction : public OSCAction
	{
	public:
		typedef enum
		{
			ENVIRONMENT,     // not supported yet
			ENTITY,          // not supported yet
			PARAMETER_SET,
			INFRASTRUCTURE,  // not supported yet
			TRAFFIC,         // not supported yet
		} Type;

		Type type_;

		OSCGlobalAction(OSCGlobalAction::Type type) : OSCAction(OSCAction::BaseType::GLOBAL), type_(type)
		{
			LOG("");
		}

		virtual void print()
		{
			LOG("Virtual, should be overridden");
		}

		virtual OSCGlobalAction* Copy()
		{
			LOG("Virtual, should be overridden");
			return 0;
		};

		virtual std::string Type2Str()
		{
			return "OSCGlobalAction base class";
		};

	};

	class ParameterSetAction : public OSCGlobalAction
	{
	public:
		std::string name_;
		std::string value_;
		Parameters* parameters_;

		ParameterSetAction() : OSCGlobalAction(OSCGlobalAction::Type::PARAMETER_SET), name_(""), value_(""), parameters_(0) {};

		ParameterSetAction(const ParameterSetAction& action) : OSCGlobalAction(OSCGlobalAction::Type::PARAMETER_SET)
		{
			name_ = action.name_;
			value_ = action.value_;
		}

		OSCGlobalAction* Copy()
		{
			ParameterSetAction* new_action = new ParameterSetAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "ParameterSetAction";
		};

		void Start(double simTime, double dt);
		void Step(double simTime, double dt);

		void print()
		{
			LOG("");
		}

	};
}

