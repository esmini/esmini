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

#include "ControllerInteractive.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

#include <random>

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerInteractive(std::string name, Entities* entities, ScenarioGateway* gateway,
	Parameters* parameters, OSCProperties* properties)
{
	return new ControllerInteractive(name, entities, gateway, parameters, properties);
}

ControllerInteractive::ControllerInteractive(std::string name, Entities* entities, ScenarioGateway* gateway,
	Parameters* parameters, OSCProperties* properties) :
	Controller(name, entities, gateway, parameters, properties)
{
	LOG("");
}

void ControllerInteractive::Init()
{

	Controller::Init();
}

void ControllerInteractive::PostFrame()
{
	Controller::PostFrame();
}

void ControllerInteractive::Step(double timeStep)
{
	double speed_limit = object_->pos_.GetSpeedLimit(); 

	if (speed_limit < SMALL_NUMBER)
	{
		// no speed limit defined, set something with regards to number of lanes
		if (object_->pos_.GetRoadById(object_->pos_.GetTrackId())->GetNumberOfDrivingLanesSide(
			object_->pos_.GetS(), SIGN(object_->pos_.GetLaneId())) > 1)
		{
			speed_limit = 110 / 3.6;
		}
		else
		{
			speed_limit = 60 / 3.6;
		}
	}
	vehicle_.SetMaxSpeed(speed_limit);

	// Update vehicle motion
	vehicle_.DrivingControlBinary(timeStep, 
		(domain_ & ControllerDomain::CTRL_LONGITUDINAL) ? accelerate : vehicle::THROTTLE_NONE, 
		(domain_ & ControllerDomain::CTRL_LATERAL) ? steer : vehicle::STEERING_NONE);

	// Set OpenDRIVE position
	object_->pos_.XYZH2TrackPos(vehicle_.posX_, vehicle_.posY_, vehicle_.posZ_, vehicle_.heading_);

	// Fetch Z and Pitch from OpenDRIVE position
	vehicle_.posZ_ = object_->pos_.GetZ();
	vehicle_.pitch_ = object_->pos_.GetP();

	// Report updated state to scenario gateway

	gateway_->reportObject(object_->id_, object_->name_, static_cast<int>(Object::Type::VEHICLE), 
		static_cast<int>(Vehicle::Category::CAR), 0, object_->GetControllerType(), object_->boundingbox_, 0,
		vehicle_.speed_, vehicle_.wheelAngle_, vehicle_.wheelRotation_,
		vehicle_.posX_, vehicle_.posY_, vehicle_.posZ_,
		vehicle_.heading_, vehicle_.pitch_, 0);

	Controller::Step(timeStep);
}

void ControllerInteractive::Activate(int domainMask)
{
	if (object_)
	{
		vehicle_.Reset();
		vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
		vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
		vehicle_.speed_ = object_->GetSpeed();
	}

	steer = vehicle::STEERING_NONE;
	accelerate = vehicle::THROTTLE_NONE;

	Controller::Activate(domainMask);
}

void ControllerInteractive::ReportKeyEvent(int key, bool down)
{
	if (key == Controller::KEY_Left)
	{
		if (down)
		{
			steer = vehicle::STEERING_LEFT;
		}
		else
		{
			steer = vehicle::STEERING_NONE;
		}
	}
	else if(key == Controller::KEY_Right)
	{
		if (down)
		{
			steer = vehicle::STEERING_RIGHT;
		}
		else
		{
			steer = vehicle::STEERING_NONE;
		}
	}
	else if (key == Controller::KEY_Up)
	{
		if (down)
		{
			accelerate = vehicle::THROTTLE_ACCELERATE;
		}
		else
		{
			accelerate = vehicle::THROTTLE_NONE;
		}
	}
	else if (key == Controller::KEY_Down)
	{
		if (down)
		{
			accelerate = vehicle::THROTTLE_BRAKE;
		}
		else
		{
			accelerate = vehicle::THROTTLE_NONE;
		}
	}
}
