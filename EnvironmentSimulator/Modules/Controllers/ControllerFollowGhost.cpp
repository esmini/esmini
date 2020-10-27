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

//#include "playerbase.hpp"
#include "ControllerFollowGhost.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerFollowGhost(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerFollowGhost(initArgs);
}

ControllerFollowGhost::ControllerFollowGhost(InitArgs* args) : Controller(args)
{
	if (args->properties->ValueExists("headstartTime"))
	{
		headstart_time_ = strtod(args->properties->GetValueStr("headstartTime"));
	}

	// FollowGhost controller forced into override mode - will not perform any scenario actions
	if (mode_ != Mode::MODE_OVERRIDE)
	{
		LOG("FollowGhost controller mode \"%s\" not applicable. Using override mode instead.", Mode2Str(mode_).c_str());
		mode_ = Controller::Mode::MODE_OVERRIDE;
	}
}

void ControllerFollowGhost::Init()
{
	object_->SetHeadstartTime(headstart_time_);
	
	Controller::Init();
}

void ControllerFollowGhost::Step(double timeStep)
{
	// Set steering target point at a distance ahead proportional to the speed
	double probe_target_distance = MAX(7, 0.5 * object_->speed_);

	// Update position along ghost trails
	if (object_->GetActivatedControllerType() == Controller::Type::CONTROLLER_TYPE_EXTERNAL ||
		object_->GetActivatedControllerType() == Controller::Type::CONTROLLER_TYPE_FOLLOW_GHOST)
	{
		if (object_->GetGhost())
		{
			if (object_->GetGhost()->trail_.FindClosestPoint(object_->pos_.GetX(), object_->pos_.GetY(),
				object_->trail_closest_pos_[0], object_->trail_closest_pos_[1],
				object_->trail_follow_s_, object_->trail_follow_index_, object_->trail_follow_index_) == 0)
			{
				object_->trail_closest_pos_[2] = object_->pos_.GetZ();
			}
			else
			{
				// Failed find point along trail, copy entity position
				object_->trail_closest_pos_[0] = object_->pos_.GetX();
				object_->trail_closest_pos_[1] = object_->pos_.GetY();
				object_->trail_closest_pos_[2] = object_->pos_.GetZ();
			}
		}
	}

	// Find out a steering target along ghost vehicle trail
	double s_out;
	int index_out;
	ObjectTrailState state;

	// Locate a point at given distance from own vehicle along the ghost trajectory
	if (object_->GetGhost() && object_->GetGhost()->trail_.FindPointAhead(
		object_->trail_follow_index_, object_->trail_follow_s_, probe_target_distance, state, index_out, s_out) != 0)
	{
		state.x_ = (float)object_->pos_.GetX();
		state.y_ = (float)object_->pos_.GetY();
		state.z_ = (float)object_->pos_.GetZ();
		state.speed_ = 0;
	}

	// Update object sensor position for visualization
	object_->sensor_pos_[0] = state.x_;
	object_->sensor_pos_[1] = state.y_;
	object_->sensor_pos_[2] = state.z_;

	double diffGlobal[2] = { state.x_ - object_->pos_.GetX(), state.y_ - object_->pos_.GetY() };
	double len = sqrt(diffGlobal[0] * diffGlobal[0] + diffGlobal[1] * diffGlobal[1]);
	if (len > SMALL_NUMBER)
	{
		diffGlobal[0] /= len;
		diffGlobal[1] /= len;
	}
	else
	{
		diffGlobal[0] = 0;
		diffGlobal[1] = 0;
	}

	// Find heading to the point
	double egoDirGlobal[2];
	RotateVec2D(1.0, 0.0, object_->pos_.GetH(), egoDirGlobal[0], egoDirGlobal[1]);

	double diffH = asin(GetCrossProduct2D(egoDirGlobal[0], egoDirGlobal[1], diffGlobal[0], diffGlobal[1]));

	// Update driver model target values
	vehicle_.DrivingControlTarget(timeStep, diffH, state.speed_);

	// Register updated vehicle position 
	object_->pos_.XYZH2TrackPos(vehicle_.posX_, vehicle_.posY_, vehicle_.posZ_, vehicle_.heading_);

	object_->SetSpeed(vehicle_.speed_);

	// Fetch Z and Pitch from road position
	vehicle_.posZ_ = object_->pos_.GetZ();
	vehicle_.pitch_ = object_->pos_.GetP();
	
	// Update wheels wrt domains
	if (domain_ & Controller::Domain::CTRL_LONGITUDINAL)
	{
		object_->wheel_rot_ = vehicle_.wheelRotation_;
		object_->SetDirtyBits(Object::DirtyBit::WHEEL_ROTATION);
	}

	if (domain_ & Controller::Domain::CTRL_LATERAL)
	{
		object_->wheel_angle_ = vehicle_.wheelAngle_;
		object_->SetDirtyBits(Object::DirtyBit::WHEEL_ANGLE);
	}

	gateway_->reportObject(object_->id_, object_->name_, static_cast<int>(object_->type_), object_->category_holder_, object_->model_id_,
		object_->GetActivatedControllerType(), object_->boundingbox_, 0, object_->speed_, object_->wheel_angle_, object_->wheel_rot_, &object_->pos_);

	Controller::Step(timeStep);
}

void ControllerFollowGhost::Activate(int domainMask)
{
	if (object_)
	{
		vehicle_.Reset();
		vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
		vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
		vehicle_.speed_ = object_->GetSpeed();
		vehicle_.SetMaxSpeed(100);  // just set a random high value 

		object_->sensor_pos_[0] = object_->pos_.GetX();
		object_->sensor_pos_[1] = object_->pos_.GetY();
		object_->sensor_pos_[2] = object_->pos_.GetZ();
	}

	Controller::Activate(domainMask);
}

void ControllerFollowGhost::ReportKeyEvent(int key, bool down)
{

}