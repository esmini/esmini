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

ControllerFollowGhost::ControllerFollowGhost(Controller::Type type, std::string name, Entities* entities,
	ScenarioGateway* gateway, Parameters* parameters, OSCProperties properties) :
	Controller(type, name, entities, gateway)
{
	if (properties.ValueExists("ghost"))
	{
		std::string ghost_name = properties.GetValueStr("ghost");
		ghost_ = entities->GetObjectByName(ghost_name);
		if (ghost_ == 0)
		{
			LOG("Error: Failed to find ghost %s", ghost_name.c_str());
		}
	}
}

void ControllerFollowGhost::Init()
{
	Controller::Init();
}

void ControllerFollowGhost::PostFrame()
{
	Controller::PostFrame();
}

void ControllerFollowGhost::Step(double timeStep)
{
	roadmanager::RoadProbeInfo probe_info;

	// Set steering target point at a distance ahead proportional to the speed
	double probe_target_distance = MAX(7, 0.5 * object_->speed_);

	// find out what direction is forward, according to vehicle relative road heading
	if (GetAbsAngleDifference(object_->pos_.GetH(), object_->pos_.GetHRoadInDrivingDirection()) > M_PI_2)
	{
		probe_target_distance *= -1;
	}

	// Find out a steering target along ghost vehicle trail
	double s_out;
	int index_out;
	ObjectTrailState state;

	// Locate a point at given distance from own vehicle along the ghost trajectory
	if (ghost_ && ghost_->trail_.FindPointAhead(
		object_->trail_follow_index_, object_->trail_follow_s_, probe_target_distance, state, index_out, s_out) != 0)
	{
		state.x_ = (float)object_->pos_.GetX();
		state.y_ = (float)object_->pos_.GetY();
		state.z_ = (float)object_->pos_.GetX();
		state.speed_ = 0;
	}

	// Define a position object at that position and probe to find out more details
	roadmanager::Position pos(state.x_, state.y_, 0, 0, 0, 0);
	object_->pos_.GetProbeInfo(&pos, &probe_info);

	// Update object sensor position for visualization
	for (int i = 0; i < 3; i++) object_->sensor_pos_[i] = probe_info.road_lane_info.pos[i];

	// Update driver model target values
	vehicle_.DrivingControlTarget(timeStep, probe_info.relative_h, state.speed_);

	// Register updated vehicle position 
	object_->pos_.XYZH2TrackPos(vehicle_.posX_, vehicle_.posY_, vehicle_.posZ_, vehicle_.heading_);

	// Fetch Z and Pitch from road position
	vehicle_.posZ_ = object_->pos_.GetZ();
	vehicle_.pitch_ = object_->pos_.GetP();

	// Report updated state to scenario gateway
	gateway_->reportObject(object_->id_, object_->name_, static_cast<int>(Object::Type::VEHICLE), static_cast<int>(Vehicle::Category::CAR),
		0, object_->GetControllerType(), object_->boundingbox_, 0,
		vehicle_.speed_, vehicle_.wheelAngle_, vehicle_.wheelRotation_,
		vehicle_.posX_, vehicle_.posY_, vehicle_.posZ_,
		vehicle_.heading_, vehicle_.pitch_, 0);

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