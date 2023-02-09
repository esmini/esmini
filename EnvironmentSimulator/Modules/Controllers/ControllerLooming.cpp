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
#include "playerbase.hpp"

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
	double k0 = 1.8;
	double k1 = 1.3;
	double k2 = 1.5;
	double const nearPointDistance = 10.0;
	double const farPointDistance = 80.0;
	double const carFollowingThreshold = 1.0;

	double farAngle = 0.0;
	double far_x = 0.0;
	double far_y = 0.0;
	double farTanS = 0.0;

	double minGapLength = LARGE_NUMBER;
	int minObjIndex = -1;
	const double minDist = 3.0;  // minimum distance to keep to lead vehicle
	const double minLateralDist = 5.0;
	bool hasFarTan = false;


	currentSpeed_ = object_->GetSpeed();
	
	roadmanager::RoadProbeInfo s_data;
	// fixed near point
	object_->pos_.GetProbeInfo(nearPointDistance, &s_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
	double nearAngle = s_data.relative_h;

	// double near_x = s_data.road_lane_info.pos[0];
	// double near_y = s_data.road_lane_info.pos[1];

	roadmanager::OpenDrive* odr = roadmanager::Position::GetOpenDrive();
	if (odr != NULL)
	{
		roadmanager::Road* road = odr->GetRoadById(object_->pos_.GetTrackId());
		double dist = 0.0;
		roadmanager::Road* roadTemp = road;
		while ( dist < farPointDistance)
		{
			roadmanager::LaneSection* lsec = nullptr;
			if (dist < SMALL_NUMBER) // first time 
			{
				lsec = roadTemp->GetLaneSectionByS(object_->pos_.GetS());
			} 
			else
			{
				roadmanager::RoadLink* roadLink = roadTemp->GetLink(roadmanager::LinkType::SUCCESSOR);
				roadTemp = odr->GetRoadById(roadLink->GetElementId());
				lsec = roadTemp->GetLaneSectionByS(0);
				// todo depand on the direction pick s value
			}
			roadmanager::Lane* lane = lsec->GetLaneById(object_->pos_.GetLaneId());
			std::vector<roadmanager::PointStruct> osi_points = lane->GetOSIPoints()->GetPoints();
			double disS_left = -1.0;
			
			for (int m = 1; m < 3; m++)  // m==1: Left, m==2: Right
			{
				int counter = 0;
				double prevAngleDif = 0.0;
				double tmpPrevFarAngle = 0.0;

				for (size_t k=0; k < osi_points.size(); k++)
				{
					if (dist + osi_points[k].s > farPointDistance + object_->pos_.GetS())
					{
						break;
					}
					else if (m == 2 && disS_left > 0 && dist + osi_points[k].s > disS_left)
					{
						break;
					}
					else if (osi_points[k].s  > object_->pos_.GetS())
					{
						LOG("egoPosi: %.10f", object_->pos_.GetS());
						LOG("disBtwFarAndEgo: %.2f",osi_points[k].s  - object_->pos_.GetS());
						double xr = 0.0, yr = 0.0;
						double local_x = 0.0;
						double local_y = (m == 1 ? +1 : -1) * road->GetLaneWidthByS(osi_points[k].s , object_->pos_.GetLaneId())/2;

						RotateVec2D(local_x, local_y, osi_points[k].h, xr, yr);
						double farX = osi_points[k].x + xr;
						double farY = osi_points[k].y + yr;
						farAngle = GetAngleInIntervalMinusPIPlusPI(atan2(farY - object_->pos_.GetY() , farX - object_->pos_.GetX()));
						printf("far x %.2f y %.2f angle %.2f\n", farX, farY, farAngle);
				
						if (counter > 0)
						{
							angleDif = GetAngleDifference(farAngle, tmpPrevFarAngle);
							LOG("angleDiff: %.10f", angleDif);
							if (counter > 1)
							{
								LOG("prevAngleDif: %.10f", prevAngleDif);
								if (SIGN(angleDif)!=SIGN(prevAngleDif))
								{
									LOG("farFinalTanAngle: %.2f", farAngle);
									if (m == 1)
									{
										disS_left = osi_points[k].s ;
									}

									far_x = farX;
									far_y = farY;
									farTanS = osi_points[k].s - object_->pos_.GetS();
									hasFarTan = true;
									break;
								}
							}
							prevAngleDif = angleDif;
						}
						tmpPrevFarAngle = farAngle;
						LOG("farTanAngle: %.10f", farAngle);
						LOG("farTanPrevAngle: %.10f", tmpPrevFarAngle);
						counter += 1;
					}
				}
			}
			if (dist < SMALL_NUMBER) // first time 
			{
				dist += roadTemp->GetLength() - object_->pos_.GetS();
				// todo depand on the direction pick s value
			} 
			else
			{
				// todo depand on the direction pick s value
				dist += roadTemp->GetLength();
			}
		}
	}

	if (!hasFarTan)
	{
		// dynamic far point-default points are road points
		object_->pos_.GetProbeInfo(farPointDistance, &s_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
		farAngle = s_data.relative_h;
		LOG("farAngleNoTan: %.10f", farAngle);

		far_x = s_data.road_lane_info.pos[0];
		far_y = s_data.road_lane_info.pos[1];
		// LOG("near: %.2f, %.2f", near_x, near_y);
	}
	
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
					if (hasFarTan && farTanS < minGapLength) // use lowest point if both lead points and road tan points detected
					{
						break;
					}
					farAngle = GetAngleInIntervalMinusPIPlusPI(atan2(object_->pos_.GetY() - pivot_obj->pos_.GetY() , object_->pos_.GetX() - pivot_obj->pos_.GetX()) - object_->pos_.GetH());
					LOG("new far: %.2f, %.2f\n", pivot_obj->pos_.GetX(), pivot_obj->pos_.GetY());
						far_x = pivot_obj->pos_.GetX();
						far_y = pivot_obj->pos_.GetY();
				}
				
				// for close vehicle
				if (minGapLength < carFollowingThreshold)
				{
					farAngle = GetAngleInIntervalMinusPIPlusPI(atan2(object_->pos_.GetX() - pivot_obj->pos_.GetX(), object_->pos_.GetY() - pivot_obj->pos_.GetY()) - object_->pos_.GetH());
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

			acc = distFactor - distFactor * dvSet - (1-distFactor) * dvMin;   // weighted combination of relative distance and speed

		}
	}
	else
	{
		// no lead vehicle to adapt to, adjust according to setSpeed
		acc = setSpeed_ - currentSpeed_;
		// LOG("far: %.2f, %.2f\n", far_x, far_y);

	}

	// LOG("Current speed: %.2f", currentSpeed_);
	// LOG("Setspeed: %.2f", setSpeed_);
	// LOG("acc %.2f", CLAMP(acc,-1,1));
	
	if (timeStep > SMALL_NUMBER)
	{
		double nearAngleDot = (nearAngle - prevNearAngle)/timeStep;
		double farAngleDot = (farAngle - prevFarAngle)/timeStep;

		steering = k0 * nearAngle + k1 * nearAngleDot + k2 * farAngleDot;
		// scale (90 degrees) and truncate 
		steering = steering / M_PI_4;
		steering = CLAMP(steering, -1, 1);
		LOG("steering %.2f\n", steering);
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

	object_->SetSensorPosition(
		far_x,
		far_y,
		0.0
	);
	
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
	if (player_)
	{
		player_->SteeringSensorSetVisible(object_->GetId(), true);
	}
}

void ControllerLooming::ReportKeyEvent(int key, bool down)
{
	(void)key;
	(void)down;
}
