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
 * This controller simulates a bad or dizzy driver by manipulating
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

#include "ControllerACC.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerACC(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerACC(initArgs);
}

ControllerACC::ControllerACC(InitArgs* args) : active_(false), timeGap_(1.5), setSpeed_(0), currentSpeed_(0),
	Controller(args)
{
	if (args && args->properties && args->properties->ValueExists("timeGap"))
	{
		timeGap_ = strtod(args->properties->GetValueStr("timeGap"));
	}
	if (args && args->properties && args->properties->ValueExists("setSpeed"))
	{
		setSpeed_ = strtod(args->properties->GetValueStr("setSpeed"));
	}
	mode_= Mode::MODE_ADDITIVE;
}

void ControllerACC::Init()
{
	Controller::Init();
}

void ControllerACC::Step(double timeStep)
{
	double minGapLength = LARGE_NUMBER;
	double minSpeedDiff = 0.0;
	int minObjIndex = -1;
	const double minDist = 3.0;  // minimum distance to keep to lead vehicle
	const double minLateralDist = 5.0;
	const double accelerationFactor = 0.7;

	// Lookahead distance is at least 50m or twice the distance required to stop
	// https://www.symbolab.com/solver/equation-calculator/s%5Cleft(t%5Cright)%3D2%5Cleft(m%2Bvt%2B%5Cfrac%7B1%7D%7B2%7Dat%5E%7B2%7D%5Cright)%2C%20t%3D%5Cfrac%7B-v%7D%7Ba%7D
	double lookaheadDist = MAX(50.0, 2 * minDist - pow(object_->GetSpeed(), 2) / -object_->GetMaxDeceleration());  // (m)
	for (size_t i = 0; i < entities_->object_.size(); i++)
	{
		Object* pivot_obj = entities_->object_[i];
		if (pivot_obj == nullptr || pivot_obj == object_)
		{
			continue;
		}

		// Measure longitudinal distance to all vehicles, don't utilize costly freespace option, instead measure ref point to ref point
		roadmanager::PositionDiff diff;
		if (object_->pos_.Delta(&pivot_obj->pos_, diff, false, lookaheadDist) == true)   // look only double timeGap ahead
		{
			// path exists between position objects

			// adjust longitudinal dist wrt bounding boxes
			double adjustedGapLength = diff.ds;
			double dHeading = GetAbsAngleDifference(object_->pos_.GetH(), pivot_obj->pos_.GetH());
			if (dHeading < M_PI_2)   // objects are pointing roughly in the same direction
			{
				adjustedGapLength -=
					(object_->boundingbox_.dimensions_.length_ / 2.0 + object_->boundingbox_.center_.x_) +
					(pivot_obj->boundingbox_.dimensions_.length_ / 2.0 - pivot_obj->boundingbox_.center_.x_);
			}
			else   // objects are pointing roughly in the opposite direction
			{
				adjustedGapLength -=
					(object_->boundingbox_.dimensions_.length_ / 2.0 + object_->boundingbox_.center_.x_) +
					(pivot_obj->boundingbox_.dimensions_.length_ / 2.0 + pivot_obj->boundingbox_.center_.x_);
			}

			// dLaneId == 0 indicates there is linked path between object lanes, i.e. no lane changes needed
			if (diff.dLaneId == 0 && adjustedGapLength > 0 && adjustedGapLength < minGapLength && abs(diff.dt) < minLateralDist)
			{
				minGapLength = adjustedGapLength;
				minSpeedDiff = object_->GetSpeed() - pivot_obj->GetSpeed();
				minObjIndex = (int)i;
			}
		}

		// Also check for really close entities in front
		if (minObjIndex != i)
		{
			double x_local, y_local;
			object_->FreeSpaceDistance(pivot_obj, &y_local, &x_local);

			if (x_local > 0 && x_local < 1.0 + pivot_obj->boundingbox_.dimensions_.length_ +
				0.5 * MAX(0.0, object_->GetSpeed() - pivot_obj->GetSpeed())
				&& y_local < 0.2 && y_local > -1) // yield some more for right hand traffic
			{
				minGapLength = x_local;
				minSpeedDiff = object_->GetSpeed() - pivot_obj->GetSpeed();
				minObjIndex = (int)i;
			}
		}
	}

	if (object_->CheckDirtyBits(Object::DirtyBit::SPEED))
	{
		// Speed has been set from somewhere else (another action or controller), respect it
		setSpeed_ = object_->GetSpeed();
	}

	if (minObjIndex > -1)
	{
		if (minGapLength < 1)
		{
			currentSpeed_ = 0.0;
		}
		else
		{
			// Follow distance = minimum distance + timeGap_ seconds
			double speedForTimeGap = MAX(object_->GetSpeed(), entities_->object_[minObjIndex]->GetSpeed());
			double followDist = minDist + timeGap_ * fabs(speedForTimeGap);  // (m)
			double dist = minGapLength - followDist;
			double dv = object_->GetSpeed() - entities_->object_[minObjIndex]->GetSpeed();

			double acc = 0.5 * dist - 0.5 * dv;   // weighted combination of relative distance and speed

			acc = CLAMP(acc, -object_->GetMaxDeceleration(), object_->GetMaxAcceleration());

			currentSpeed_ = MIN(object_->GetSpeed() + acc * timeStep, setSpeed_);
			currentSpeed_ = MAX(0.0, currentSpeed_);
		}
	}
	else
	{
		// no lead vehicle to adapt to, adjust according to setSpeed
		double tmpSpeed = object_->GetSpeed() + SIGN(setSpeed_ - currentSpeed_) * accelerationFactor * object_->GetMaxAcceleration() * timeStep;
		if (SIGN(setSpeed_ - tmpSpeed) != SIGN(setSpeed_ - currentSpeed_))
		{
			// passed target speed
			currentSpeed_ = setSpeed_;
		}
		else
		{
			currentSpeed_ = tmpSpeed;
		}
	}

	object_->SetSpeed(currentSpeed_);
	gateway_->reportObjectSpeed(object_->GetId(), object_->GetSpeed());

	Controller::Step(timeStep);
}

void ControllerACC::Activate(ControlDomains domainMask)
{
	setSpeed_ = object_->GetSpeed();
	Controller::Activate(domainMask);
}

void ControllerACC::ReportKeyEvent(int key, bool down)
{
}