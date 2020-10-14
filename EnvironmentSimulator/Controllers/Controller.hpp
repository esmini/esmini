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
#include "Parameters.hpp"

#define CONTROLLER_BASE_NAME "BaseClass"
#define CONTROLLER_BASE_TYPE -1

namespace scenarioengine
{


	// Forward declarations
//	class ScenarioPlayer;
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
			CONTROLLER_TYPE_GHOST,
			CONTROLLER_TYPE_FOLLOW_GHOST,
			CONTROLLER_TYPE_INTERACTIVE,
			CONTROLLER_TYPE_SLOPPY_DRIVER,
			CONTROLLER_TYPE_SUMO,
			N_CONTROLLER_TYPES,
			USER_CONTROLLER_TYPE_BASE = 1000
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


		Controller() : entities_(0), gateway_(0) {}
		Controller(std::string name, Entities* entities, ScenarioGateway* gateway, Parameters* parameters, OSCProperties* properties) :
			name_(name), entities_(entities), gateway_(gateway), object_(0), domain_(0), ghost_(0), headstart_time_(0) {}

		static const char* GetTypeNameStatic() { return CONTROLLER_BASE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static const int GetTypeStatic() { return CONTROLLER_BASE_TYPE; }
		virtual int GetType() { return GetTypeStatic(); }

		void SetGhost(Object* ghost) { ghost_ = ghost; }
		Object* GetGhost() { return ghost_; }
		double GetHeadstartTime() { return headstart_time_; }
		std::string GetName() { return name_; }
		int GetDomain() { return domain_; }
		
		virtual void Assign(Object* object);
		virtual void Activate(int domainMask) { domain_ = domainMask; };
		virtual void Deactivate() { domain_ = 0; };
		virtual void Init() {};
		virtual void PostFrame() {};
		virtual void ReportKeyEvent(int key, bool down);

		// Base class Step function should be called from derived classes
		virtual void Step(double timeStep);

		bool Active();

	protected:
		//		ScenarioPlayer* player_;
		Entities* entities_;
		ScenarioGateway* gateway_;
		Object* object_;  // The object to which the controller is attached and hence controls
		Object* ghost_;  // Some controllers follows another ghost vehicle
		int domain_;  // bitmask according to ControllerDomain type
		double headstart_time_;
		std::string name_;

	};

	typedef Controller* (*ControllerInstantiateFunction) (
		std::string name,
		Entities* player,
		ScenarioGateway* gateway,
		Parameters* parameters,
		OSCProperties* properties
		);


	Controller* InstantiateController(std::string name, Entities* entities, ScenarioGateway* gateway, Parameters* parameters, OSCProperties* properties);
}