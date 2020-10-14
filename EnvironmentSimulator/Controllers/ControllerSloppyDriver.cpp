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

Controller* scenarioengine::InstantiateControllerSloppyDriver(std::string name, Entities* entities, ScenarioGateway* gateway,
	Parameters* parameters, OSCProperties* properties)
{
	return new ControllerSloppyDriver(name, entities, gateway, parameters, properties);
}

ControllerSloppyDriver::ControllerSloppyDriver(std::string name, Entities* entities, 
	ScenarioGateway* gateway, Parameters* parameters, OSCProperties* properties) : sloppiness_(0.5),
	Controller(name, entities, gateway, parameters, properties)
{
	if (properties->ValueExists("sloppiness"))
	{
		sloppiness_ = strtod(properties->GetValueStr("sloppiness"));
	}
}

void ControllerSloppyDriver::Init()
{
	mt_rand.seed((unsigned int)time(0));

	Controller::Init();
}

void ControllerSloppyDriver::PostFrame()
{
	Controller::PostFrame();
}

void ControllerSloppyDriver::Step(double timeStep)
{
	if (object_ == 0)
	{
		return;
	}
	
	// Do modification to a local position object and then report to gateway
	roadmanager::Position pos = object_->pos_;
	double speed = initSpeed_;
	double lateral_offset = initT_;
	double heading = pos.GetHRelative();
	double hRate = 0;

	if (object_ && domain_ & ControllerDomain::CTRL_LONGITUDINAL)
	{
		if (speedTimer_ < 0)
		{
			double targetValue = initSpeed_ + sloppiness_ * initSpeed_ * (1.0 * mt_rand() / mt_rand.max() - 0.5);
			speedFilter_.SetTargetValue(targetValue);

			// restart timer - 50% variation
			speedTimer_ = speedTimerDuration_ + speedTimerDuration_ * (1.0 * mt_rand() / mt_rand.max() - 0.5);
		}
		speedFilter_.Update(timeStep);
		
		speed = MAX(speedFilter_.GetValue(), 0);  // Don't go reverse
		
		// Adjustment movement to heading and road direction
		if (GetAbsAngleDifference(object_->pos_.GetH(), object_->pos_.GetDrivingDirection()) > M_PI_2)
		{
			// If pointing in other direction
			pos.MoveAlongS(-timeStep * object_->GetSpeed());
		}
		else
		{
			pos.MoveAlongS(timeStep * object_->GetSpeed());
		}
		
		speedTimer_ -= timeStep;
	}

	if (object_ && domain_ & ControllerDomain::CTRL_LATERAL)
	{
		if (lateralTimer_ < 0)
		{
			double targetValue = initT_ + 20.0 * sloppiness_ * (1.0 * mt_rand() / mt_rand.max() - 0.5);
			lateralFilter_.SetTargetValue(targetValue);

			// restart timer - 50% variation
			lateralTimer_ = lateralTimerDuration_ + lateralTimerDuration_ * (1.0 * mt_rand() / mt_rand.max() - 0.5);
		}
		lateralFilter_.Update(timeStep);
		
		if (fabs(object_->GetSpeed()) > SMALL_NUMBER)
		{
			// Adjust heading to lateral motion 
			heading = atan(lateralFilter_.GetV() / object_->GetSpeed());
		}
		else
		{
			// Keep heading
			heading = pos.GetHRelative();
		}

		if (fabs(object_->pos_.GetHRelative()) > SMALL_NUMBER)
		{
			hRate = GetAngleDifference(heading, object_->pos_.GetHRelative()) / timeStep;
		}


		pos.SetTrackPos(pos.GetTrackId(), pos.GetS(), lateralFilter_.GetValue());
		pos.SetHeadingRelative(heading);

		lateralTimer_ -= timeStep;
	}

	// Report updated state to scenario gateway
	gateway_->reportObject(object_->id_, object_->name_, static_cast<int>(Object::Type::VEHICLE), static_cast<int>(Vehicle::Category::CAR), 
		0, object_->GetControllerType(), object_->boundingbox_, 0,
		speed, hRate/2, fmod(object_->wheel_rot_ + object_->GetSpeed() * timeStep / WHEEL_RADIUS, 2 * M_PI),
		pos.GetX(), pos.GetY(), pos.GetZ(),
		pos.GetH(), pos.GetP(), 0);

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
		initSpeed_ = object_->GetSpeed();
		speedTimerDuration_ = 3;
		speedTimer_ = speedTimerDuration_;
		speedFilter_.SetTension(0.4);
		speedFilter_.SetOptimalDamping();
		speedFilter_.SetTargetValue(object_->GetSpeed());
		speedFilter_.SetValue(object_->GetSpeed());

		initT_ = object_->pos_.GetT();
		lateralTimerDuration_ = 0.7;
		lateralTimer_ = lateralTimerDuration_;
		lateralFilter_.SetTension(0.3);
		lateralFilter_.SetOptimalDamping();
		lateralFilter_.SetTargetValue(object_->pos_.GetT());
		lateralFilter_.SetValue(object_->pos_.GetT());
		lateralFilter_.SetV(tan(object_->pos_.GetHRelative()) * object_->GetSpeed());
	}

	Controller::Activate(domainMask);
}

void ControllerSloppyDriver::ReportKeyEvent(int key, bool down)
{
}