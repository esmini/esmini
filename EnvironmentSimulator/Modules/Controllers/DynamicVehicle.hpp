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

#define BT_EULER_DEFAULT_ZYX

#include "btBulletDynamicsCommon.h"


namespace roadmanager
{
	class OpenDrive;
}

namespace dynamicvehicle
{
	typedef enum
	{
		THROTTLE_BRAKE = -1,
		THROTTLE_NONE = 0,
		THROTTLE_ACCELERATE = 1,
	} THROTTLE;

	typedef enum
	{
		STEERING_RIGHT = -1,
		STEERING_NONE = 0,
		STEERING_LEFT = 1,
	} STEERING;

	class DynamicVehicle
	{
	public:

		DynamicVehicle() : vehicle_(nullptr), dynamics_world_(nullptr), odr_(nullptr) {}
		~DynamicVehicle();

		void Init(
			double length,
			double width,
			double height,
			double mass,
			roadmanager::OpenDrive* odr
		)
		{
			Init(length, width, height, mass, odr, 20.0, 100.0, 0.5);
		}

		void Init(
			double length,
			double width,
			double height,
			double mass,
			roadmanager::OpenDrive* odr,
			double suspension_stiffness,
			double friction_slip,
			double roll_influence
		);

		void Step(double dt);

		void DrivingControlTarget(double dt, double target_speed, double heading_to_target);
		void DrivingControlBinary(double dt, THROTTLE throttle, STEERING steering);
		/**
			Update vehicle in terms of continuous throttle, brake and steering values
			@param dt Timestep (sec)
			@param throttle Acceleration (>0) or deceleration (<0) level in the range(-1, 1)
			@param steering Steering output left (>0) or right (<0) in the range(-1, 1)
		*/
		void DrivingControlAnalog(double dt, double throttle, double steering);

		void GetPosition(double& x, double& y, double& z);
		void GetRotation(double& h, double& p, double& r);

		void Reset() { Reset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }
		void Reset(double x, double y, double z, double h, double p, double r, double speed);

		void SetupFlatGround(double size);

		btRaycastVehicle* vehicle_;
	private:
		btDiscreteDynamicsWorld* dynamics_world_;
		roadmanager::OpenDrive* odr_;
		btAlignedObjectArray<btCollisionShape*> collision_shapes_;
	};

}