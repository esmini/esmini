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

namespace vehicle
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

	class Vehicle
	{
	public:
		Vehicle() { Reset(); }
		Vehicle(double x, double y, double h, double length);
		void Update(double dt);
		void DrivingControlTarget(double dt, double heading_to_target, double target_speed);
		void DrivingControlBinary(double dt, THROTTLE throttle, STEERING steering);
		void DrivingControlAnalog(double dt, double throttle, double steering);
		void SetWheelAngle(double angle);
		void SetWheelRotation(double rotation);
		void SetLength(double length) { length_ = length; }
		void SetPos(double x, double y, double z, double h)
		{
			posX_ = x;
			posY_ = y;
			posZ_ = z;
			heading_ = h;
		}
		void SetMaxSpeed(double speed);
		double GetMaxSpeed() { return max_speed_; }
		void SetAccelerationScale(double accScale);
		bool GetThrottleDisabled() { return steering_disabled_; }
		void SetThrottleDisabled(bool value) { throttle_disabled_ = value; }
		bool GetSteeringDisabled() { return steering_disabled_; }
		void SetSteeringDisabled(bool value) { steering_disabled_ = value; }
		void Reset();

		double posX_;
		double posY_;
		double posZ_;
		double heading_;
		double pitch_;

		double velX_;
		double velY_;
		double velAngle_;
		double velAngleRelVehicleLongAxis_;
		double speed_;
		double wheelAngle_;
		double wheelRotation_;
		double headingDot_;
		bool handbrake_;

		double target_speed_;

		double length_;

	private:
		double max_speed_;
		double acc_scale_;
		bool throttle_disabled_;
		bool steering_disabled_;
	};

}