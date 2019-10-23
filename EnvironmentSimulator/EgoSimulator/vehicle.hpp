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

namespace vehicle
{ 
	enum THROTTLE
	{
		THROTTLE_BRAKE = -1,
		THROTTLE_NONE = 0,
		THROTTLE_ACCELERATE = 1
	};

	enum STEERING
	{
		STEERING_RIGHT = -1,
		STEERING_NONE = 0,
		STEERING_LEFT = 1
	};

	class Vehicle
	{
	public:
		Vehicle(double x, double y, double h, double length);
		void Vehicle::Update(double dt);
		void DrivingControlTarget(double dt, double heading_to_target, double headway_time_to_target);
		void DrivingControlBinary(double dt, THROTTLE throttle, STEERING steering);
		void SetWheelAngle(double angle);
		void SetWheelRotation(double rotation);
		void SetPos(double x, double y, double z, double h)
		{
			posX_ = x;
			posY_ = y;
			posZ_ = z;
			heading_ = h;
		}
		void SetMaxSpeed(double speed) { max_speed_ = speed; }
		double GetMaxSpeed() { return max_speed_; }

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

		double target_hwt_;

		double length_;

	private:
		double max_speed_;
	};

}