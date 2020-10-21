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

#include "ControllerSloppyDriver.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

#include <random>

using namespace scenarioengine;

static std::mt19937 mt_rand;

#define WHEEL_RADIUS 0.35

Controller* scenarioengine::InstantiateControllerSloppyDriver(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerSloppyDriver(initArgs);
}

ControllerSloppyDriver::ControllerSloppyDriver(InitArgs* args) : sloppiness_(0.5),
	Controller(args)
{
	if (args->properties->ValueExists("sloppiness"))
	{
		sloppiness_ = strtod(args->properties->GetValueStr("sloppiness"));
	}
}

void ControllerSloppyDriver::Init()
{
	mt_rand.seed((unsigned int)time(0));

	Controller::Init();
}

void ControllerSloppyDriver::Step(double timeStep)
{
	if (object_ == 0)
	{
		return;
	}
	
	// Do modification to a local position object and then report to gateway
	double lateral_offset = initT_;
	double hRate = 0;
	double oldSpeed = speedFilter_.GetValue();

	if (object_ && domain_ & Controller::Domain::CTRL_LONGITUDINAL)
	{
		if (speedTimer_ < 0)
		{
			// vary next speed target value in proportion to current speed
			double targetValue = initSpeed_ + object_->GetSpeed() * sloppiness_ * (1.0 * mt_rand() / mt_rand.max() - 0.5);
			speedFilter_.SetTargetValue(targetValue);

			// restart timer - 50% variation
			speedTimer_ = speedTimerDuration_ + speedTimerDuration_ * (1.0 * mt_rand() / mt_rand.max() - 0.5);
		}
		speedFilter_.Update(timeStep);
		
		if (mode_ == Mode::MODE_OVERRIDE)
		{
			object_->SetSpeed(MAX(speedFilter_.GetValue(), 0));  // Don't go reverse
						
			// Adjustment movement to heading and road direction
			if (GetAbsAngleDifference(object_->pos_.GetH(), object_->pos_.GetDrivingDirection()) > M_PI_2)
			{
				// If pointing in other direction
				object_->pos_.MoveAlongS(-timeStep * object_->GetSpeed());
			}
			else
			{
				object_->pos_.MoveAlongS(timeStep * object_->GetSpeed());
			}
		}
		else
		{
			object_->SetSpeed(object_->GetSpeed() + speedFilter_.GetValue() - oldSpeed);
		}
		
		speedTimer_ -= timeStep;
	}

	if (object_ && domain_ & Controller::Domain::CTRL_LATERAL)
	{
		double oldT = lateralFilter_.GetValue();
		double oldRelativeH = relativeH_;

		if (lateralTimer_ < 0)
		{
			// max lateral displacement is about half lane width (7/2) 
			double targetValue = initT_ + 7.0 * sloppiness_ * (1.0 * mt_rand() / mt_rand.max() - 0.5);
			lateralFilter_.SetTargetValue(targetValue);

			// restart timer - 50% variation
			lateralTimer_ = lateralTimerDuration_ + lateralTimerDuration_ * (1.0 * mt_rand() / mt_rand.max() - 0.5);
		}

		lateralFilter_.Update(timeStep);
		
		if (fabs(object_->GetSpeed()) > SMALL_NUMBER)
		{
			// Adjust heading to lateral motion 
			relativeH_ = atan(lateralFilter_.GetV() / object_->GetSpeed());
		}
		else
		{
			// Keep heading
			relativeH_ = object_->pos_.GetHRelative();
		}

		if (mode_ == Mode::MODE_OVERRIDE)
		{
			object_->pos_.SetTrackPos(object_->pos_.GetTrackId(), object_->pos_.GetS(), lateralFilter_.GetValue());
			object_->pos_.SetHeadingRelative(relativeH_);
		}
		else
		{
			object_->pos_.SetTrackPos(object_->pos_.GetTrackId(), object_->pos_.GetS(),
				object_->pos_.GetT() + lateralFilter_.GetValue() - oldT);
			
			// Relative heading is reset by Default Controller each frame
			// Calculate new relative heading (vehicles heading relative the road direction)
			// by finding out the sum lateral movement (per unit longitudinal movement) from 
			// Default Controller and this controller
			double dTDefaultCtrl = tan(object_->pos_.GetHRelative());
			double dTThisCtrl = (lateralFilter_.GetValue() - oldT) / (object_->GetSpeed() * timeStep);
			object_->pos_.SetHeadingRelative(atan(dTDefaultCtrl + dTThisCtrl));
		}

		lateralTimer_ -= timeStep;
	}

	Controller::Step(timeStep);
}

void ControllerSloppyDriver::Activate(int domainMask)
{
	if (object_)
	{
		if (sloppiness_ < 0 || sloppiness_ > 1)
		{
			LOG("Warning, sloppiness is %.2f recommended range is [0:1]", sloppiness_);
		}
		speedTimerDuration_ = 3;
		speedTimer_ = speedTimerDuration_;
		speedFilter_.SetTension(0.4);
		speedFilter_.SetOptimalDamping();
		if (mode_ == Mode::MODE_OVERRIDE)
		{
			initSpeed_ = object_->GetSpeed();
			speedFilter_.SetTargetValue(object_->GetSpeed());
			speedFilter_.SetValue(object_->GetSpeed());
		}
		else
		{
			initSpeed_ = 0;
			speedFilter_.SetTargetValue(0);
			speedFilter_.SetValue(0);
		}


		lateralTimerDuration_ = 0.7;
		lateralTimer_ = lateralTimerDuration_;
		lateralFilter_.SetTension(0.3);
		lateralFilter_.SetOptimalDamping();
		if (mode_ == Mode::MODE_OVERRIDE)
		{
			lateralFilter_.SetTargetValue(object_->pos_.GetT());
			lateralFilter_.SetValue(object_->pos_.GetT());
			initT_ = object_->pos_.GetT();
			relativeH_ = object_->pos_.GetH();
		}
		else
		{
			lateralFilter_.SetTargetValue(0);
			lateralFilter_.SetValue(0);
			initT_ = 0;
			relativeH_ = 0;
		}
		lateralFilter_.SetV(tan(object_->pos_.GetHRelative()) * object_->GetSpeed());
	}

	Controller::Activate(domainMask);
}

void ControllerSloppyDriver::ReportKeyEvent(int key, bool down)
{
}