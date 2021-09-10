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

#include "Controller.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"


using namespace scenarioengine;

Controller* scenarioengine::InstantiateController(void* args)
{
	LOG("The base class should not be instantiated");

	return new Controller((Controller:: InitArgs*)args);
}

Controller::Controller(InitArgs* args) : name_(args->name), type_name_(args->type), entities_(args->entities),
	gateway_(args->gateway), domain_(ControlDomains::DOMAIN_NONE), mode_(Controller::Mode::MODE_OVERRIDE), object_(0)
{
	if (args->properties->ValueExists("mode"))
	{
		std::string mode = args->properties->GetValueStr("mode");
		if (mode == "override")
		{
			mode_ = Mode::MODE_OVERRIDE;
		}
		else if (mode == "additive")
		{
			mode_ = Mode::MODE_ADDITIVE;
		}
		else
		{
			LOG("Unexpected mode \"%s\", falling back to default \"override\"", mode.c_str());
			mode_ = Mode::MODE_OVERRIDE;
		}
	}
}

void Controller::Step(double timeStep)
{
	if (object_)
	{
		if (mode_ == Mode::MODE_OVERRIDE)
		{
			if (IsActiveOnDomains(ControlDomains::DOMAIN_LAT))
			{
				object_->SetDirtyBits(Object::DirtyBit::LATERAL);
			}

			if (IsActiveOnDomains(ControlDomains::DOMAIN_LONG))
			{
				object_->SetDirtyBits(Object::DirtyBit::LONGITUDINAL);

			}
		}
		else
		{
			object_->ClearDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
		}
	}
}

void Controller::Assign(Object* object)
{
	if (object == 0)
	{
		if (object_)
		{
			// Detach any existing object from controller
			object_->SetAssignedController(0);
		}
		object_ = 0;
	}
	else
	{
		object_ = object;

		// Attach controller to object
		object_->SetAssignedController(this);
	}
}

void Controller::ReportKeyEvent(int key, bool down)
{
	LOG("Key %c %s", key, down ? "down" : "up");
}

std::string Controller::Mode2Str(int mode)
{
	if (mode == Controller::Mode::MODE_OVERRIDE)
	{
		return "override";
	}
	else if (mode == Controller::Mode::MODE_ADDITIVE)
	{
		return "additive";
	}
	else if (mode == Controller::Mode::MODE_NONE)
	{
		return "none";
	}
	else
	{
		LOG("Unexpected mode \"%d\"", mode);
		return "invalid mode";
	}
}

bool Controller::IsActiveOnDomains(ControlDomains domainMask)
{
	return (static_cast<int>(GetDomain()) & static_cast<int>(domainMask)) == static_cast<int>(domainMask);
}

bool Controller::IsActiveOnAnyOfDomains(ControlDomains domainMask)
{
	return (static_cast<int>(GetDomain()) & static_cast<int>(domainMask)) != 0;
}

bool Controller::IsActive()
{
	return GetDomain() != ControlDomains::DOMAIN_NONE;
}