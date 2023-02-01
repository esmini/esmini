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
 * This controller simulates a simple Adaptive Cruise Control
 */

#include "ControllerLooming.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioEngine.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerLooming(void* args)
{
	Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

	return new ControllerLooming(initArgs);
}

ControllerLooming::ControllerLooming(InitArgs* args) : Controller(args), active_(false), timeGap_(1.5), setSpeed_(0),
	currentSpeed_(0), setSpeedSet_(false), steering_rate_(4.0)
{
	if (args && args->properties && args->properties->ValueExists("timeGap"))
	{
		timeGap_ = strtod(args->properties->GetValueStr("timeGap"));
	}
	if (args && args->properties && args->properties->ValueExists("setSpeed"))
	{
		setSpeed_ = strtod(args->properties->GetValueStr("setSpeed"));
		setSpeedSet_ = true;
	}
}


void ControllerLooming::Step(double timeStep)
{
	// looming controller properties
	double const nearPointDistance = 10.0;
	double const farPointDistance = 80.0;
	double const carFollowingThreshold = 1.0;
	double k0 = 1.8;
	double k1 = 1.3;
	double k2 = 1.5;


	double minGapLength = LARGE_NUMBER;
	int minObjIndex = -1;
	const double minDist = 3.0;  // minimum distance to keep to lead vehicle
	const double minLateralDist = 5.0;

	double far_x = 0.0;
	double far_y = 0.0;
	double near_x = 0.0;
	double near_y = 0.0;

	currentSpeed_ = object_->GetSpeed();


	roadmanager::RoadProbeInfo s_data;
	// fixed near point
	object_->pos_.GetProbeInfo(nearPointDistance, &s_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
	nearAngle = s_data.relative_h;

	near_x = s_data.road_lane_info.pos[0];
	near_y = s_data.road_lane_info.pos[1];

	// dynamic far point-default points are road points
	object_->pos_.GetProbeInfo(farPointDistance, &s_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
	farAngle = s_data.relative_h;

	far_x = s_data.road_lane_info.pos[0];
	far_y = s_data.road_lane_info.pos[1];
	LOG("near: %.2f, %.2f", near_x, near_y);
	


	for (size_t i = 0; i < entities_->object_.size(); i++)
	{

		Object* pivot_obj = entities_->object_[i];
		if (pivot_obj == nullptr || pivot_obj == object_)
		{
			continue;
		}

		double lookaheadDist = 130;
		// Measure longitudinal distance to all vehicles, don't utilize costly free-space option, instead measure ref point to ref point
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
					(static_cast<double>(object_->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(object_->boundingbox_.center_.x_)) +
					(static_cast<double>(pivot_obj->boundingbox_.dimensions_.length_) / 2.0 - static_cast<double>(pivot_obj->boundingbox_.center_.x_));
			}
			else   // objects are pointing roughly in the opposite direction
			{
				adjustedGapLength -=
					(static_cast<double>(object_->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(object_->boundingbox_.center_.x_)) +
					(static_cast<double>(pivot_obj->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(pivot_obj->boundingbox_.center_.x_));
			}
			

			// dLaneId == 0 indicates there is linked path between object lanes, i.e. no lane changes needed
			if (diff.dLaneId == 0 && adjustedGapLength > 0 && adjustedGapLength < minGapLength && abs(diff.dt) < minLateralDist)
			{
				minGapLength = adjustedGapLength;
				minObjIndex = static_cast<int>(i); // TODO: size_t to int
				
				// find far point from lead as reference, if lead <= farPointDistance(80) m
				if (minGapLength <= farPointDistance)
				{
					double angle = atan2(object_->pos_.GetX() - pivot_obj->pos_.GetX(), object_->pos_.GetY() - pivot_obj->pos_.GetY()) - object_->pos_.GetH();
					farAngle = checkAngle(angle);
					LOG("new far: %.2f, %.2f\n", pivot_obj->pos_.GetX(), pivot_obj->pos_.GetY());
				}
				else
				{
					LOG("far: %.2f, %.2f\n", far_x, far_y);
				}

				if (minGapLength < carFollowingThreshold)
				{
					double angle = atan2(object_->pos_.GetX() - pivot_obj->pos_.GetX(), object_->pos_.GetY() - pivot_obj->pos_.GetY()) - object_->pos_.GetH();
					farAngle = checkAngle(angle);
				}
			}
		}
	}

	if (minObjIndex > -1)
	{
		if (minGapLength < 1)
		{
			acc = 0.0;
		}
		else
		{
			double speedForTimeGap = MAX(currentSpeed_, entities_->object_[static_cast<unsigned int>(minObjIndex)]->GetSpeed());
			double followDist = minDist + timeGap_ * fabs(speedForTimeGap);  // (m)
			double dist = minGapLength - followDist;
			double distFactor = MIN(1.0, dist / followDist);

			double dvMin = currentSpeed_ - MIN(setSpeed_, entities_->object_[static_cast<unsigned int>(minObjIndex)]->GetSpeed());
			double dvSet = currentSpeed_ - setSpeed_;

			acc = 2.5 * distFactor - distFactor * dvSet - (1-distFactor) * dvMin;   // weighted combination of relative distance and speed

		}
	}
	else
	{
		// no lead vehicle to adapt to, adjust according to setSpeed
		acc = setSpeed_ - currentSpeed_;
		LOG("far: %.2f, %.2f\n", far_x, far_y);

	}

	LOG("Current speed: %.2f", currentSpeed_);
	LOG("Setspeed: %.2f", setSpeed_);
	LOG("acc %.2f", acc);
	

	if (timeStep > SMALL_NUMBER)
	{
		double nearAngleDot = (nearAngle - prevNearAngle)/timeStep;
		double farAngleDot = (farAngle - prevFarAngle)/timeStep;

		steering = k0 * nearAngle + k1 * nearAngleDot + k2 * farAngleDot;
		// scale (90 degrees) and truncate 
		steering = steering / M_PI_4;
		steering = CLAMP(steering, -1, 1);
		LOG("steering %.2f", steering);
	}
	prevNearAngle = nearAngle;
	prevFarAngle = farAngle;
	
	gateway_->getObjectStatePtrByIdx(object_->GetId())->state_.pos.SetLockOnLane(true);
	vehicle_.DrivingControlAnalog(timeStep, CLAMP(acc,-1,1), steering);		

	gateway_->updateObjectWorldPosXYH(object_->GetId(), 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
	gateway_->updateObjectWheelAngle(object_->GetId(), 0.0, vehicle_.wheelAngle_);

	gateway_->updateObjectSpeed(object_->GetId(), 0.0, vehicle_.speed_);
	object_->sensor_pos_[0] = far_x;
	object_->sensor_pos_[1] = far_y;



	Controller::Step(timeStep);
	
}


void ControllerLooming::Init()
{
	Controller::Init();
}

void ControllerLooming::Activate(ControlDomains domainMask)
{
	currentSpeed_ = object_->GetSpeed();
	if (mode_ == Mode::MODE_ADDITIVE || setSpeedSet_ == false)
	{
		setSpeed_ = object_->GetSpeed();
	}

	if (object_)
	{
		vehicle_.Reset();
		vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
		vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
		vehicle_.speed_ = object_->GetSpeed();
		vehicle_.SetMaxAcc(object_->GetMaxAcceleration());
		vehicle_.SetMaxDec(object_->GetMaxDeceleration());
		vehicle_.SetSteeringRate(steering_rate_);
	}

	Controller::Activate(domainMask);

	if (IsActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// Make sure heading is aligned with road driving direction
		object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
	}
}

void ControllerLooming::ReportKeyEvent(int key, bool down)
{
	(void)key;
	(void)down;
}

double ControllerLooming::checkAngle(double angle)
{
	double const pi = 3.1415926535;
	while (angle < -pi)
	{
        angle += 2.0 * pi;
    }
    while (angle > pi)
	{
        angle -= 2.0 * pi;
    }
    return angle;
}