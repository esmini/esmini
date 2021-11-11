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
	if (!object_->GetGhost())
	{
		// No ghost associated
		return;
	}

	// Set steering target point at a distance ahead proportional to the speed
	double probe_target_distance = MAX(7, 0.5 * object_->speed_);

	if (object_->GetGhost()->trail_.FindClosestPoint(object_->pos_.GetX(), object_->pos_.GetY(),
		object_->trail_closest_pos_, object_->trail_follow_index_, object_->trail_follow_index_) == 0)
	{
		object_->trail_closest_pos_.z = object_->pos_.GetZ();
	}
	else
	{
		// Failed find point along trail, copy entity position
		object_->trail_closest_pos_.x = object_->pos_.GetX();
		object_->trail_closest_pos_.y = object_->pos_.GetY();
		object_->trail_closest_pos_.z = object_->pos_.GetZ();
	}

	// Find out a steering target along ghost vehicle trail
	int index_out;
	roadmanager::TrajVertex point;

	// Locate a point at given distance from own vehicle along the ghost trajectory
	if (object_->GetGhost()->trail_.FindPointAhead(
		object_->trail_closest_pos_.s, probe_target_distance, point, index_out, object_->trail_follow_index_) != 0)
	{
		point.x = (float)object_->pos_.GetX();
		point.y = (float)object_->pos_.GetY();
		point.z = (float)object_->pos_.GetZ();
		point.speed = 0;
	}

	// Update object sensor position for visualization
	object_->sensor_pos_[0] = point.x;
	object_->sensor_pos_[1] = point.y;
	object_->sensor_pos_[2] = point.z;

	double diffGlobal[2] = { point.x - object_->pos_.GetX(), point.y - object_->pos_.GetY() };
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
	vehicle_.DrivingControlTarget(timeStep, point.speed, diffH);

	// Register updated vehicle position

	// Fetch Z and Pitch from road position
	vehicle_.posZ_ = object_->pos_.GetZ();
	vehicle_.pitch_ = object_->pos_.GetP();

	// Register updated vehicle position
	gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
	gateway_->updateObjectSpeed(object_->id_, 0.0, vehicle_.speed_);

	// Update wheels wrt domains
	if (IsActiveOnDomains(ControlDomains::DOMAIN_LONG))
	{
		gateway_->updateObjectWheelRotation(object_->id_, 0.0, vehicle_.wheelRotation_);
	}

	if (IsActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		gateway_->updateObjectWheelAngle(object_->id_, 0.0, vehicle_.wheelAngle_);
	}

	Controller::Step(timeStep);
}

void ControllerFollowGhost::Activate(ControlDomains domainMask)
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

		object_->pos_.SetAlignModeZ(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
		object_->pos_.SetAlignModeP(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
	}

	Controller::Activate(domainMask);
}

void ControllerFollowGhost::ReportKeyEvent(int key, bool down)
{

}