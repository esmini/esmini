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

 /*
  * This controller let the user control the vehicle interactively by the arrow keys
  */


#pragma once

#define BT_EULER_DEFAULT_ZYX

#include <string>
#include "Controller.hpp"
#include "Parameters.hpp"
#include "btBulletDynamicsCommon.h"


#define CONTROLLER_DYNAMICS_TYPE_NAME "DynamicsController"

namespace scenarioengine
{

	// base class for controllers
	class ControllerDynamics: public Controller
	{
	public:

		ControllerDynamics(InitArgs* args);
		~ControllerDynamics();

		void Init();
		void Step(double timeStep);
		void Activate(ControlDomains domainMask);
		void ReportKeyEvent(int key, bool down);

		static const char* GetTypeNameStatic() { return CONTROLLER_DYNAMICS_TYPE_NAME; }
		virtual const char* GetTypeName() { return GetTypeNameStatic(); }
		static int GetTypeStatic() { return Controller::Type::CONTROLLER_TYPE_DYNAMICS; }
		virtual int GetType() { return GetTypeStatic(); }


		btBoxShape* veh_shape_;
		btCompoundShape compound_;
		btTransform veh_xform_;
		btBroadphaseInterface* pair_cache_;
		btDefaultCollisionConfiguration* collision_config_;
		btCollisionDispatcher* dispatcher_;
		btConstraintSolver* constraint_solver_;
		btDiscreteDynamicsWorld* dynamics_world_;
		btRigidBody* veh_body_;
		btVehicleRaycaster* veh_ray_caster_;
		btRaycastVehicle* vehicle_;
		btDefaultMotionState* motion_state_;
		double suspension_stiffness_;
		double friction_slip_;
		double roll_influence_;
	private:
	};

	Controller* InstantiateControllerDynamics(void* args);
}