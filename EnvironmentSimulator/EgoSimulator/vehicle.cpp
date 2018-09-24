// ExampleConsole.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "vehicle.hpp"

using namespace vehicle;

#define STEERING_RATE 3.0
#define STEERING_MAX_ANGLE (30 * M_PI / 180)
#define ACCELERATION_SCALE 15
#define SPEED_MAX (200 / 3.6)
#define SPEED_DECLINE 1E-2
#define WHEEL_RADIUS 0.35
#define SIGN(X) (X<0?-1:1)
#define MAX(a, b) (a>b ? a : b)
#define MIN(a, b) (a<b ? a : b)
#define CLAMP(x, lo, hi) MIN(hi, MAX(lo, x))

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
	double steerDamping = 1 - fabs(speed_) / SPEED_MAX;
	double steerRate = STEERING_RATE * steerDamping;

	wheelAngle_ += steerRate * steering * dt;
	double selfAlign = -SIGN(wheelAngle_) * 0.5 * steerRate * dt;

	if (wheelAngle_ < 0)
	{
		wheelAngle_ = MIN(wheelAngle_ + selfAlign, 0);
	}
	else
	{
		wheelAngle_ = MAX(wheelAngle_ + selfAlign, 0);
	}
	wheelAngle_ = CLAMP(wheelAngle_, -STEERING_MAX_ANGLE * steerDamping, STEERING_MAX_ANGLE * steerDamping);

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

