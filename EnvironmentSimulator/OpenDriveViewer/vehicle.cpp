// ExampleConsole.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include "vehicle.hpp"

#define STEERING_STEP 3.0
#define STEERING_MAX_ANGLE (50 * M_PI / 180)
#define STEERING_DECLINE 0.2
#define ACCELERATION_STEP 15
#define SPEED_MAX (150 / 3.6)
#define SPEED_DECLINE 1E-2
#define WHEEL_RADIUS 0.35

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

void Vehicle::Update(double dt, int acceleration, int steering)
{
	double criticalB = 0;

	speed_ = (1.0 - SPEED_DECLINE) * speed_ + ACCELERATION_STEP * acceleration * dt;
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

	double wheelAngleDelta = STEERING_STEP * steering * dt;

	// Max steering angle depending on speed
	double scaleFactor = (SPEED_MAX - fabs(speed_)) / SPEED_MAX;
	wheelAngle_ = (1.0 - STEERING_DECLINE * scaleFactor) * wheelAngle_ + wheelAngleDelta * scaleFactor;

	if (wheelAngle_ > STEERING_MAX_ANGLE)
	{
		wheelAngle_ = STEERING_MAX_ANGLE;
	}
	else if (wheelAngle_ < -STEERING_MAX_ANGLE)
	{
		wheelAngle_ = -STEERING_MAX_ANGLE;
	}

	//printf("speed %.2f steer %.2f\n", speed_ * 3.6, wheelAngle_ * 180 / M_PI);

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



