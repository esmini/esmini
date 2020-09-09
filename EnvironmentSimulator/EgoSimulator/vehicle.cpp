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

using namespace vehicle;

#define STEERING_RATE 5.0
#define STEERING_MAX_ANGLE (60 * M_PI / 180)
#define ACCELERATION_SCALE 40
#define SPEED_DECLINE 0.001
#define WHEEL_RADIUS 0.35
#define SIGN(X) (X<0?-1:1)
#define MAX(a, b) (a>b ? a : b)
#define MIN(a, b) (a<b ? a : b)
#define CLAMP(x, lo, hi) MIN(hi, MAX(lo, x))
#define TARGET_HWT 1.0

Vehicle::Vehicle(double x, double y, double h, double length)
{
	posX_ = x;
	posY_ = y;
	posZ_ = 0;
	heading_ = h;
	pitch_ = 0;
	target_speed_ = 0;
	velX_ = 0;
	velY_ = 0;
	velAngle_ = 0;
	velAngleRelVehicleLongAxis_ = 0;
	speed_ = 0;
	handbrake_ = false;
	wheelAngle_ = 0.0;
	wheelRotation_ = 0.0;
	headingDot_ = 0.0;
	max_speed_ = 70;
	length_ = length;
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

void Vehicle::DrivingControlTarget(double dt, double heading_to_target, double target_speed)
{
	double acceleration = CLAMP(ACCELERATION_SCALE * (target_speed - speed_), -30, 30);

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

	if (handbrake_ == true)
	{
		if (throttle == THROTTLE::THROTTLE_NONE)
		{
			handbrake_ = false;
		}
	}
	else 
	{
		speed_ += ACCELERATION_SCALE * throttle * dt;

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

	// Calculate steering

	// Make steering wheel speed dependent
	double steering_scale = 1.0 / (1 + 0.02 * speed_ * speed_);
	wheelAngle_ = wheelAngle_ + steering_scale * STEERING_RATE * steering  * dt;
	
	// Self-aligning
	wheelAngle_ *= 0.92;

	// Limit wheel angle
	wheelAngle_ = CLAMP(wheelAngle_, -steering_scale * STEERING_MAX_ANGLE, steering_scale * STEERING_MAX_ANGLE);

	Update(dt);
}

void Vehicle::Update(double dt)
{
	double criticalB = 0;

	// Calculate wheel rot: https://en.wikipedia.org/wiki/Arc_(geometry)
	wheelRotation_ += speed_ * dt / WHEEL_RADIUS;

	// Calculate vehicle kinematics according to simple bicycle model, see
	// http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf

	velAngleRelVehicleLongAxis_ = atan(0.15 * tan(wheelAngle_));  // Origo is between rear wheel axles on ground level, estimated 15% along X axis
	velAngle_ = velAngleRelVehicleLongAxis_ + heading_;
	headingDot_ = speed_ * sin(velAngleRelVehicleLongAxis_) / (length_ * 0.15);  

	velX_ = speed_ * cos(velAngle_);
	velY_ = speed_ * sin(velAngle_);
	heading_ += dt * headingDot_;
	posX_ += dt * velX_;
	posY_ += dt * velY_;
}

