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

#include <string>
#include "CommonMini.hpp"
#include "OSCProperties.hpp"
#include "Parameters.hpp"

#define CONTROLLER_BASE_TYPE_NAME "ControllerClass"
#define CONTROLLER_BASE_TYPE_ID -1

namespace scenarioengine
{
	// Forward declarations
	class ScenarioGateway;
	class Entities;
	class Object;

	// base class for controllers
	class Controller
	{
	public:

		enum Type
		{
			CONTROLLER_TYPE_DEFAULT,
			CONTROLLER_TYPE_EXTERNAL,
			CONTROLLER_TYPE_FOLLOW_GHOST,
			CONTROLLER_TYPE_INTERACTIVE,
			CONTROLLER_TYPE_SLOPPY_DRIVER,
			CONTROLLER_TYPE_SUMO,
			CONTROLLER_TYPE_REL2ABS,
			CONTROLLER_TYPE_ACC,
			N_CONTROLLER_TYPES,
			GHOST_RESERVED_TYPE = 100,
			USER_CONTROLLER_TYPE_BASE = 1000,
		};

		enum  // copy key enums from OSG GUIEventAdapter
		{
			KEY_Left = 0xFF51,        /* Left arrow */
			KEY_Up = 0xFF52,          /* Up arrow */
			KEY_Right = 0xFF53,       /* Right arrow */
			KEY_Down = 0xFF54,        /* Down arrow */
		};

		typedef enum
		{
			MODE_NONE,     // Controller not available or it is not active
			MODE_OVERRIDE, // Actions from the scenario are not applied, default
			MODE_ADDITIVE, // Actions from the scenario are applied
		} Mode;

		typedef struct
		{
			std::string name;
			std::string type;
			OSCProperties* properties;
			Entities* entities;
			ScenarioGateway* gateway;
			Parameters* parameters;
		} InitArgs;

		Controller() : entities_(0), gateway_(0) {}
		Controller(InitArgs* args);

		static const char* GetTypeNameStatic() { return CONTROLLER_BASE_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_BASE_TYPE_ID; }
		virtual int GetType() { return GetTypeStatic(); }

		virtual void Assign(Object* object);
		virtual void Activate(ControlDomains domainMask) { domain_ = domainMask; };
		virtual void Deactivate() { domain_ = ControlDomains::DOMAIN_NONE; };
		virtual void Init() {};
		virtual void ReportKeyEvent(int key, bool down);

		// Base class Step function should be called from derived classes
		virtual void Step(double timeStep);

		bool Active() { return static_cast<int>(domain_) != 0; };
		std::string GetName() { return name_; }
		ControlDomains GetDomain() { return domain_; }
		int GetMode() { return mode_; }
		std::string Mode2Str(int mode);
		Object* GetObject() { return object_; }

		bool IsActiveOnDomains(ControlDomains domainMask);
		bool IsActiveOnAnyOfDomains(ControlDomains domainMask);
		bool IsActive();

	protected:
		ControlDomains domain_;  // bitmask according to ControllerDomain type
		std::string name_;
		std::string type_name_;
		Entities* entities_;
		ScenarioGateway* gateway_;
		Object* object_;  // The object to which the controller is attached and hence controls
		int mode_; // add to scenario actions or replace
	};

	typedef Controller* (*ControllerInstantiateFunction) (void* args);
	Controller* InstantiateController(void* args);
}