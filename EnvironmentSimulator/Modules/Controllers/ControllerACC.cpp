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

ControllerACC::ControllerACC(InitArgs* args) : active_(false), timeGap_(2.0), setSpeed_(0), currentSpeed_(0), Controller(args)
{
	if (args->properties->ValueExists("timeGap"))
	{
		timeGap_ = strtod(args->properties->GetValueStr("timeGap"));
	}
	mode_ = Mode::MODE_ADDITIVE;
}

void ControllerACC::Init()
{
	Controller::Init();
}

void ControllerACC::Step(double timeStep)
{
	double minGapLength = LARGE_NUMBER;
	double minSpeedDiff = 0.0;
	double minDist = 15.0;  // minimum distance to keep to lead vehicle
	int minObjIndex = -1;
	double followDist = 0.0;
	double maxDeceleration = -10.0;
	double normalAcceleration = 3.0;

	// Lookahead distance is at least 50m or twice the distance required to stop
	// https://www.symbolab.com/solver/equation-calculator/s%5Cleft(t%5Cright)%3D2%5Cleft(m%2Bvt%2B%5Cfrac%7B1%7D%7B2%7Dat%5E%7B2%7D%5Cright)%2C%20t%3D%5Cfrac%7B-v%7D%7Ba%7D
	double lookaheadDist = MAX(50.0, minDist - pow(object_->GetSpeed(), 2) / maxDeceleration);  // (m)

	// Find any vehicle within 3 x timeGap seconds headway
	for (size_t i = 0; i < entities_->object_.size(); i++)
	{
		if (entities_->object_[i] == object_)
		{
			continue;
		}

		// Measure longitudinal distance to all vehicles, don't utilize costly freespace option, instead measure ref point to ref point
		roadmanager::PositionDiff diff;
		if (object_->pos_.Delta(&entities_->object_[i]->pos_, diff, lookaheadDist) == true)   // look only double timeGap ahead
		{
			// path exists between position objects
			if (diff.dLaneId == 0 && diff.ds > 0)  // dLaneId == 0 indicates there is linked path between object lanes, i.e. no lane changes needed
			{
				minGapLength = diff.ds;
				minSpeedDiff = object_->GetSpeed() - entities_->object_[i]->GetSpeed();
				minObjIndex = (int)i;
			}
		}
	}

	if (object_->CheckDirtyBits(Object::DirtyBit::SPEED))
	{
		// Speed has been set from somewhere else (another action or controller), respect it
		setSpeed_ = object_->GetSpeed();
	}

	double acc = 0.0;
	if (minObjIndex > -1)
	{
		// Follow distance = minimum distance + timeGap_ seconds
		followDist = minDist + timeGap_ * entities_->object_[minObjIndex]->GetSpeed();  // (m)

		double dv = object_->GetSpeed() - entities_->object_[minObjIndex]->GetSpeed();
		regulator_.SetV(dv);
		regulator_.SetValue(followDist - minGapLength);
		regulator_.Update(timeStep);
		acc = CLAMP(regulator_.GetA(), -100, normalAcceleration);
		currentSpeed_ = MIN(object_->GetSpeed() + acc * timeStep, setSpeed_);
	}
	else
	{
		// no lead vehicle to adapt to, adjust according to setSpeed
		double tmpSpeed = object_->GetSpeed() + SIGN(setSpeed_ - currentSpeed_) * normalAcceleration * timeStep;
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
	regulator_.SetTension(5);
	regulator_.SetOptimalDamping();
}

void ControllerACC::ReportKeyEvent(int key, bool down)
{
}