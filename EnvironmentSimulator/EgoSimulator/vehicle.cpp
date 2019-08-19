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
#define STEERING_MAX_ANGLE (45 * M_PI / 180)
#define ACCELERATION_SCALE 20
#define SPEED_MAX (100 / 3.6)
#define SPEED_DECLINE 0.001
#define WHEEL_RADIUS 0.35
#define SIGN(X) (X<0?-1:1)
#define MAX(a, b) (a>b ? a : b)
#define MIN(a, b) (a<b ? a : b)
#define CLAMP(x, lo, hi) MIN(hi, MAX(lo, x))
//#define REAR_AXLE_TO_LENGTH

Vehicle::Vehicle(double x, double y, double h, double length)
{
	posX_ = x;
	posY_ = y;
	posZ_ = 0;
	heading_ = h;
	pitch_ = 0;

	velX_ = 0;
	velY_ = 0;
	velAngle_ = 0;
	velAngleRelVehicleLongAxis_ = 0;
	speed_ = 0;
	wheelAngle_ = 0.0;
	wheelRotation_ = 0.0;
	headingDot_ = 0.0;
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

void Vehicle::Update(double dt, THROTTLE throttle, STEERING steering)
{
	double criticalB = 0;

	speed_ = (1.0 - SPEED_DECLINE) * speed_ + ACCELERATION_SCALE * throttle * dt;

	if (speed_ > SPEED_MAX)
	{
		speed_ = SPEED_MAX;
	}
	else if (speed_ < -SPEED_MAX)
	{
		speed_ = -SPEED_MAX;
	}

	// Calculate wheel rot: https://en.wikipedia.org/wiki/Arc_(geometry)
	wheelRotation_ += speed_ * dt / WHEEL_RADIUS;

	// Calculate steering
	wheelAngle_ = wheelAngle_ + STEERING_RATE * steering * dt;
	double selfAlign = -SIGN(wheelAngle_) * 0.5 * STEERING_RATE * dt;

	if (wheelAngle_ < 0)
	{
		wheelAngle_ = MIN(wheelAngle_ + selfAlign, 0);
	}
	else
	{
		wheelAngle_ = MAX(wheelAngle_ + selfAlign, 0);
	}
	// Scale down steering angle in proportion to the speed
	double dynamicMaxAngle = STEERING_MAX_ANGLE / (1 + 0.5 * fabs(speed_));
	wheelAngle_ = CLAMP(wheelAngle_, -dynamicMaxAngle, dynamicMaxAngle);

	// Calculate vehicle kinematics according to simple bicycle model, see
	// http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf

	velAngleRelVehicleLongAxis_ = atan(0.5 * tan(wheelAngle_));
	velAngle_ = velAngleRelVehicleLongAxis_ + heading_;
	headingDot_ = speed_ * sin(velAngleRelVehicleLongAxis_) / (length_ / 2);

	velX_ = speed_ * cos(velAngle_);
	velY_ = speed_ * sin(velAngle_);
	heading_ += dt * headingDot_;
	posX_ += dt * velX_;
	posY_ += dt * velY_;
}

