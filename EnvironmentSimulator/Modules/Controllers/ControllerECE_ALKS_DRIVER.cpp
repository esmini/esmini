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
 * This controller simulates the driver from ECE ALKS regulation.
 */

#include "ControllerECE_ALKS_DRIVER.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerECE_ALKS_DRIVER(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerECE_ALKS_DRIVER(initArgs);
}

ControllerECE_ALKS_DRIVER::ControllerECE_ALKS_DRIVER(InitArgs* args) : active_(false), timeGap_(2.0), setSpeed_(0), currentSpeed_(0), Controller(args)
{
	if (args->properties->ValueExists("timeGap"))
	{
		timeGap_ = strtod(args->properties->GetValueStr("timeGap"));
	}
	mode_ = Mode::MODE_ADDITIVE;
}

void ControllerECE_ALKS_DRIVER::Init()
{
	Controller::Init();
}

void ControllerECE_ALKS_DRIVER::Step(double timeStep)
{
	double minGapLength = LARGE_NUMBER;
	double minSpeedDiff = 0.0;
	double minDist = 15.0;  // minimum distance to keep to lead vehicle
	int minObjIndex = -1;
	double followDist = 0.0;
	double maxDeceleration = -10.0;
	double normalAcceleration = 3.0;
	double tCutOut = -1.0;
	double dtFreeCutOut = -LARGE_NUMBER;
	double bb_l = object_->boundingbox_.dimensions_.length_;
	double bb_w = object_->boundingbox_.dimensions_.width_;
	double bb_cx = object_->boundingbox_.center_.x_;

	// Lookahead distance is at least 50m or twice the distance required to stop
	// https://www.symbolab.com/solver/equation-calculator/s%5Cleft(t%5Cright)%3D2%5Cleft(m%2Bvt%2B%5Cfrac%7B1%7D%7B2%7Dat%5E%7B2%7D%5Cright)%2C%20t%3D%5Cfrac%7B-v%7D%7Ba%7D
	double lookaheadDist = MAX(50.0, minDist - pow(object_->GetSpeed(), 2) / maxDeceleration);  // (m)

	// Find any vehicle within 3 x timeGap seconds headway

	// from default ACC one can estimate, that the entities are sorted at least by their ds and the smallest ds values are coming at last positions
	// if this is not the case, then the code would not work, because it's very important to check first the cut-out vehicle and afterwards the vehicle in front of cut-out vehicle!!!
	for (int i = static_cast<int>(entities_->object_.size()) - 1; i >= 0; i--)
	{
		if (entities_->object_[i] == object_)
		{
			continue;
		}
		double bbi_l = entities_->object_[i]->boundingbox_.dimensions_.length_;
		double bbi_w = entities_->object_[i]->boundingbox_.dimensions_.width_;
		double bbi_cx = entities_->object_[i]->boundingbox_.center_.x_;

		// Measure longitudinal distance to all vehicles, don't utilize costly freespace option, instead measure ref point to ref point
		roadmanager::PositionDiff diff;
		if (object_->pos_.Delta(&entities_->object_[i]->pos_, diff, lookaheadDist) == true)   // look only double timeGap ahead
		{
			// path exists between position objects

			// object on adjacent lane (possible cut-in or cut-out)
			if (fabs(entities_->object_[i]->pos_.GetVelY()) > SMALL_NUMBER && entities_->object_[i]->GetSpeed() <= object_->GetSpeed() + SMALL_NUMBER &&
				fabs(diff.dLaneId) == 1)
			{
				// cut-in, lateral velocity in the direction of ego
				if ((diff.dLaneId == -1 && entities_->object_[i]->pos_.GetVelY() > 0 && entities_->object_[i]->pos_.GetOffset() > 0.375) ||
					(diff.dLaneId == 1 && entities_->object_[i]->pos_.GetVelY() < 0 && entities_->object_[i]->pos_.GetOffset() < -0.375))
				{
					double tCutIn = MAX(0, 0.5 * (bb_w + bbi_w) - fabs(diff.dt)) / fabs(entities_->object_[i]->pos_.GetVelY());  // lateral free space / lateral velocity
					double dsFreeAfterCutIn = diff.ds - (0.5 * bb_l + bb_cx) - (0.5 * bbi_l - bbi_cx) - (object_->GetSpeed() - entities_->object_[i]->GetSpeed()) * tCutIn;
					// check if the cut-in vehicle would be in front of ego after cut-in, otherwise it can be ignored
					if (dsFreeAfterCutIn >= 0)
					{
						// check if TTC after cut-in is smaller or equal 2sec
						if (dsFreeAfterCutIn / (object_->GetSpeed() - entities_->object_[i]->GetSpeed()) <= 2) // TTC Cut-In <= 2sec
						{
							driverFullBrake_ = true;
						}
					}
				}

				// Normally this case for cut-out should not be needed, it should start in the lower if-branch for vehicles on same lane and a deviation from their lane center of more than 0.375m
				// also the egoFullBrake is stored as property
				else if ((diff.dLaneId == -1 && entities_->object_[i]->pos_.GetVelY() > 0) || (diff.dLaneId == 1 && entities_->object_[i]->pos_.GetVelY() < 0))
				{

					dtFreeCutOut = fabs(diff.dt) - 0.5 * (bb_w + bbi_w);
					tCutOut = MAX(0, -dtFreeCutOut) / fabs(entities_->object_[i]->pos_.GetVelY());  // lateral free space / lateral velocity
				}
			}
			// object in front on same lane
			else if (diff.dLaneId == 0 && diff.ds > 0)  // dLaneId == 0 indicates there is linked path between object lanes, i.e. no lane changes needed
			{
				// cut-out with object on same lane, but with a distance of more than 0.375m from lane center and lateral velocity into opposite direction of ego
				if ((entities_->object_[i]->pos_.GetOffset() > 0.375 && entities_->object_[i]->pos_.GetVelY() > 0) || (entities_->object_[i]->pos_.GetOffset() < -0.375 && entities_->object_[i]->pos_.GetVelY() < 0))
				{
					dtFreeCutOut = fabs(diff.dt) - 0.5 * (bb_w + bbi_w);
					tCutOut = MAX(0, -dtFreeCutOut) / fabs(entities_->object_[i]->pos_.GetVelY());  // lateral free space / lateral velocity
				}

				// Normally this case for cut-in should not be needed, it should start in the upper if-branch for vehicles on adjacent lane and a deviation from their lane center of more than 0.375m
				// also the egoFullBrake is stored as property
				else if ((entities_->object_[i]->pos_.GetOffset() > 0 && entities_->object_[i]->pos_.GetVelY() < 0) || (entities_->object_[i]->pos_.GetOffset() < 0 && entities_->object_[i]->pos_.GetVelY() > 0))
				{
					double tCutIn = MAX(0, 0.5 * (bb_w + bbi_w) - fabs(diff.dt)) / fabs(entities_->object_[i]->pos_.GetVelY());  // lateral free space / lateral velocity
					double dsFreeAfterCutIn = diff.ds - (0.5 * bb_l + bb_cx) - (0.5 * bbi_l - bbi_cx) - (object_->GetSpeed() - entities_->object_[i]->GetSpeed()) * tCutIn;
					// check if the cut-in vehicle would be in front of ego after cut-in, otherwise it can be ignored
					if (dsFreeAfterCutIn >= 0)
					{
						// check if TTC after cut-in is smaller or equal 2sec
						if (dsFreeAfterCutIn / (object_->GetSpeed() - entities_->object_[i]->GetSpeed()) <= 2) // TTC Cut-In <= 2sec
						{
							driverFullBrake_ = true;
						}
					}
				}

				double dsFree = diff.ds - (0.5 * bb_l + bb_cx) - (0.5 * bbi_l - bbi_cx);
				// deceleration
				if (entities_->object_[i]->pos_.GetAccX() <= -5 && dsFree >= 0 && dsFree / (object_->GetSpeed() - entities_->object_[i]->GetSpeed()) <= 2) // TTC of object in front <= 2sec
				{
					driverFullBrake_ = true;
					// from cut-in and cut-out one can estimate, that AEB is only braking in case of full wrap, right decision???
					if (fabs(diff.dt) < SMALL_NUMBER)
					{
						aebFullBrake_ = true;
					}
				}
				else
				{
					// There exist a full wrap of ego with the next object in front and TTC <= 2sec and if there was a cut-out before, then the cut-out vehicle has already left ego's driving path (lateral deviations omitted)
					if (fabs(diff.dt) < SMALL_NUMBER && (dtFreeCutOut >= 0 || dtFreeCutOut == LARGE_NUMBER) && entities_->object_[i]->GetSpeed() < object_->GetSpeed())
					{
						if (dsFree >= 0 && dsFree / (object_->GetSpeed() - entities_->object_[i]->GetSpeed()) <= 2) // TTC of object in front <= 2sec, AEB TTC not fully clear???
						{
							aebFullBrake_ = true;
						}
					}

					// There was a cut-out registered of another vehicle between this vehicle and ego
					if (tCutOut > 0)
					{
						double dsFreeAfterCutOut = diff.ds - (0.5 * bb_l + bb_cx) - (0.5 * bbi_l - bbi_cx) - (object_->GetSpeed() - entities_->object_[i]->GetSpeed()) * tCutOut;
						if (dsFreeAfterCutOut >= 0 && dsFreeAfterCutOut / (object_->GetSpeed() - entities_->object_[i]->GetSpeed()) <= 2) // TTC between ego and object in front of cut-out object <= 2sec
						{
							driverFullBrake_ = true;
						}
					}
				}

				// default ACC, in case of deceleration maneuver and deceleration < 5m/s2 it is not really clear how the reference driver would react
				// in that case it is estimated, that such cases are uncritical for each reference driver and we can use the default ACC here
				// the timeGap property of the ACC here should be chosen such that all deceleration maneuvers from ECE ALKS regulation with deceleration smaller or equal to 5m/s2 would lead to no collision
				if (diff.ds <= minGapLength)
				{
					minGapLength = diff.ds;
					minSpeedDiff = object_->GetSpeed() - entities_->object_[i]->GetSpeed();
					minObjIndex = (int)i;
				}
			}
		}
	}

	if (object_->CheckDirtyBits(Object::DirtyBit::SPEED))
	{
		// Speed has been set from somewhere else (another action or controller), respect it
		setSpeed_ = object_->GetSpeed();
	}

	// now the reference driver would brake
	if (driverFullBrake_)
	{
		// if there was no wait time set, then set it to 0.75sec
		if (waitTime_ == -1.0)
		{
			waitTime_ = 0.75;
		}

		currentSpeed_ = setSpeed_;  // during wait time take the actual speed or the speed from other contoller
		// the ego driver starts to react when the full wait time left and is braking with a jerk of 0.744G during 0.6sec
		if (waitTime_ == 0.0)
		{
			currentSpeed_ = MAX(0.0, object_->GetSpeed() - 0.774 * 9.81 * timeStep / 0.6);  // jerk time of 0.6sec to decelerate with 0.774G

			// the full braking has finished, now reset this controller again
			if (fabs(currentSpeed_) < SMALL_NUMBER)
			{
				ControllerECE_ALKS_DRIVER::Reset();
			}
		}
		waitTime_ = MAX(0.0, waitTime_ - timeStep);  // reduce waitTime by current timeStep each time this if branch is called
	}

	// the AEB has no reaction time, thus the AEB is directly braking with a jerk of 0.85G during 0.6sec
	if (aebFullBrake_)
	{
		currentSpeed_ = MAX(0, object_->GetSpeed() - 0.85 * 9.81 * timeStep / 0.6);  // jerk time of 0.6sec to decelerate with 0.85G

		// the full braking has finished, now reset this controller again
		if (fabs(currentSpeed_) < SMALL_NUMBER)
		{
			ControllerECE_ALKS_DRIVER::Reset();
		}

	}

	// taken from ACC controller, the behaviur of reference driver not really defined for all uncritical situations
	double acc = 0.0;
	if (!driverFullBrake_ && !aebFullBrake_)
	{
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
	}

	object_->SetSpeed(currentSpeed_);
	gateway_->reportObjectSpeed(object_->GetId(), object_->GetSpeed());

	Controller::Step(timeStep);
}

void ControllerECE_ALKS_DRIVER::Activate(ControlDomains domainMask)
{
	ControllerECE_ALKS_DRIVER::Reset();
	Controller::Activate(domainMask);
	regulator_.SetTension(5);
	regulator_.SetOptimalDamping();
}

void ControllerECE_ALKS_DRIVER::Reset()
{
	waitTime_ = -1.0;
	driverFullBrake_ = false;
	aebFullBrake_ = false;
	setSpeed_ = object_->GetSpeed();
}

void ControllerECE_ALKS_DRIVER::ReportKeyEvent(int key, bool down)
{
}
