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
#include "OSCProperties.hpp"


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
			CONTROLLER_DEFAULT,
			CONTROLLER_EXTERNAL,
			CONTROLLER_GHOST,
			CONTROLLER_FOLLOW_GHOST,
			CONTROLLER_INTERACTIVE,
			CONTROLLER_SLOPPY_DRIVER,
			CONTROLLER_SUMO,
			CONTROLLER_N_CONTROLLERS
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
			CTRL_NONE = 0,
			CTRL_LONGITUDINAL = 1,
			CTRL_LATERAL = 2,
			CTRL_BOTH = 3  // Also works as bitmask combi of LONG/LAT (1 | 2)
		} ControllerDomain;

		std::string name_;
		Entities* entities_;
		ScenarioGateway* gateway_;
		Object* object_;  // The object to which the controller is attached and hence controls
		Object* ghost_;  // Some controllers follows another ghost vehicle
		int domain_;  // bitmask according to ControllerDomain type
		Controller::Type type_;
		double headstart_time_;

		Controller() : type_(CONTROLLER_DEFAULT), name_(""), entities_(0), gateway_(0) {}
		Controller(Controller::Type type, std::string name, Entities* entities, ScenarioGateway* gateway) :
			type_(type), name_(name), entities_(entities), gateway_(gateway), object_(0), domain_(0),
			ghost_(0), headstart_time_(0) {}
		
		std::string GetName() { return name_; }
		Controller::Type GetType() { return type_; }
		Object* GetGhost() { return ghost_; }
		double GetHeadstartTime() { return headstart_time_; }
		virtual void Assign(Object* object);
		virtual void Activate(int domainMask) { domain_ = domainMask; };
		virtual void Deactivate() { domain_ = 0; };
		virtual void Init() {};
		
		// Base class Step function should be called from derived classes
		virtual void Step(double timeStep);

		virtual void PostFrame() {};

		virtual void ReportKeyEvent(int key, bool down);

		bool Active();

	};

}