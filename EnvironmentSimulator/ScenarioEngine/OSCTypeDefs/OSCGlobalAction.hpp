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

namespace scenarioengine
{

	class OSCGlobalAction : public OSCAction
	{
	public:
		typedef enum
		{
			ENVIRONMENT,     // not supported yet
			ENTITY,          // not supported yet
			PARAMETER,       // not supported yet
			INFRASTRUCTURE,  // not supported yet
			TRAFFIC,         // not supported yet
			EXT_QUIT,
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

	};

	class EXT_QuitAction : public OSCGlobalAction
	{
	public:
		EXT_QuitAction() : OSCGlobalAction(OSCGlobalAction::Type::EXT_QUIT) {}

		void print()
		{
			LOG("");
		}

		void Step(double dt)
		{
			OSCAction::Stop();
		}

		void Trig()
		{
			OSCAction::Trig();
		}
	};
}

