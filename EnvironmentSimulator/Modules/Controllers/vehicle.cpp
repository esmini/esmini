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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "vehicle.hpp"
#include "CommonMini.hpp"

using namespace vehicle;

#define STEERING_RATE 5.0
#define LENGTH_DEFAULT 5.0
#define STEERING_MAX_ANGLE (60 * M_PI / 180)
#define ACCELERATION_SCALE_DEFAULT 40
#define MAX_SPEED_DEFAULT 70
#define SPEED_DECLINE 0.001
#define WHEEL_RADIUS 0.35
#define TARGET_HWT 1.0

Vehicle::Vehicle(double x, double y, double h, double length)
{
	Reset();
	posX_ = x;
	posY_ = y;
	heading_ = h;
	length_ = length;
	throttle_disabled_ = false;
	steering_disabled_ = false;
}
#define MAX_WHEEL_ANGLE (60 * M_PI / 180)

void Vehicle::SetWheelAngle(double angle)
{
	wheelAngle_ = angle;
}

void Vehicle::SetWheelRotation(double rotation)
{
	wheelRotation_ = rotation;
}

void Vehicle::SetMaxSpeed(double maxSpeed)
{
	max_speed_ = maxSpeed;
}

void Vehicle::SetAccelerationScale(double accScale)
{
	accScale = CLAMP(accScale, SMALL_NUMBER, 10);
	acc_scale_ = accScale * ACCELERATION_SCALE_DEFAULT;
}

void Vehicle::DrivingControlTarget(double dt, double heading_to_target, double target_speed)
{
	double acceleration = CLAMP(acc_scale_ * (target_speed - speed_), -30, 30);

	speed_ += acceleration * dt;
	speed_ *= (1 - SPEED_DECLINE);

	double steering_scale = 1.0 / (1 + 0.015 * speed_ * speed_);
	wheelAngle_ = heading_to_target;
	wheelAngle_ = CLAMP(wheelAngle_, -steering_scale * STEERING_MAX_ANGLE, steering_scale * STEERING_MAX_ANGLE);

	Update(dt);
}

void Vehicle::DrivingControlBinary(double dt, THROTTLE throttle, STEERING steering)
{
	double oldSpeed = speed_;

	if (!throttle_disabled_)
	{
		if (handbrake_ == true)
		{
			if (throttle == THROTTLE::THROTTLE_NONE)
			{
				handbrake_ = false;
			}
		}
		else
		{
			speed_ += acc_scale_ * throttle * dt;

			if (oldSpeed > 0 && speed_ < 0)
			{
				speed_ = 0;
				handbrake_ = true;
			}
			else
			{
				speed_ *= (1 - SPEED_DECLINE);
				speed_ = CLAMP(speed_, -1.2 * max_speed_, 1.2 * max_speed_);
			}
		}
	}

	// Calculate steering
	if (!steering_disabled_)
	{
		// Make steering wheel speed dependent
		double steering_scale = 1.0 / (1 + 0.02 * speed_ * speed_);
		wheelAngle_ = wheelAngle_ + steering_scale * STEERING_RATE * steering * dt;

		// Self-aligning
		wheelAngle_ *= 0.92;

		// Limit wheel angle
		wheelAngle_ = CLAMP(wheelAngle_, -steering_scale * STEERING_MAX_ANGLE, steering_scale * STEERING_MAX_ANGLE);
	}

	Update(dt);
}

void Vehicle::DrivingControlAnalog(double dt, double throttle, double steering)
{
	double oldSpeed = speed_;

	if (!throttle_disabled_)
	{
		if (handbrake_ == true)
		{
			if (fabs(throttle) < SMALL_NUMBER || throttle > SMALL_NUMBER)
			{
				handbrake_ = false;
			}
		}
		else
		{
			speed_ += acc_scale_ * throttle * dt;

			if (oldSpeed > 0 && speed_ < 0)
			{
				speed_ = 0.0;
				handbrake_ = true;
			}
			else
			{
				speed_ *= (1 - SPEED_DECLINE);
				speed_ = CLAMP(speed_, -1.2 * max_speed_, 1.2 * max_speed_);
			}
		}
	}

	// Calculate steering
	if (!steering_disabled_)
	{
		// Make steering slightly wheel speed dependent
		double steering_scale = 1.0 / (1 + 0.005 * speed_ * speed_);
		wheelAngle_ = wheelAngle_ + steering_scale * STEERING_RATE * steering * dt;

		// Self-aligning
		wheelAngle_ *= 0.95;

		// Limit wheel angle
		wheelAngle_ = CLAMP(wheelAngle_, -steering_scale * STEERING_MAX_ANGLE, steering_scale * STEERING_MAX_ANGLE);
	}

	Update(dt);
}

void Vehicle::Update(double dt)
{
	// Calculate wheel rot: https://en.wikipedia.org/wiki/Arc_(geometry)
	wheelRotation_ = fmod(wheelRotation_ + speed_ * dt / WHEEL_RADIUS, 2 * M_PI);

	// Calculate vehicle kinematics according to simple bicycle model, see
	// http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf

	velAngleRelVehicleLongAxis_ = atan(0.15 * tan(wheelAngle_));  // Origo is between rear wheel axles on ground level, estimated 15% along X axis
	velAngle_ = velAngleRelVehicleLongAxis_ + heading_;
	headingDot_ = speed_ * sin(velAngleRelVehicleLongAxis_) / (length_ * 0.15);

	velX_ = speed_ * cos(velAngle_);
	velY_ = speed_ * sin(velAngle_);
	heading_ = fmod(heading_ + dt * headingDot_, 2 * M_PI);
	if (heading_ < 0)
	{
		heading_ += 2 * M_PI;
	}

	posX_ += dt * velX_;
	posY_ += dt * velY_;
}

void Vehicle::Reset()
{
	posX_ = 0;
	posY_ = 0;
	posZ_ = 0;
	length_ = LENGTH_DEFAULT;
	heading_ = 0;
	pitch_ = 0;
	velX_ = 0;
	velY_ = 0;
	velAngle_ = 0;
	velAngleRelVehicleLongAxis_ = 0;
	speed_ = 0;
	wheelAngle_ = 0;
	wheelRotation_ = 0;
	headingDot_ = 0;
	handbrake_ = false;
	target_speed_ = 0;
	max_speed_ = MAX_SPEED_DEFAULT;
	acc_scale_ = ACCELERATION_SCALE_DEFAULT;
}
