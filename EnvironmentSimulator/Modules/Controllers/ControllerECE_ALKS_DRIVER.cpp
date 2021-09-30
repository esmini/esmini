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
 * Written by: Thaddaeus Menzel, IDIADA Fahrzeugtechnik GmbH
 * This controller simulates the reference driver model from ECE ALKS regulation.
 * For more information look at:
 * https://unece.org/sites/default/files/2021-03/R157e.pdf, pages 43-45
 */

#include "ControllerECE_ALKS_DRIVER.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

#define ALKS_LOG(...) {if (logging_) { LOG(__VA_ARGS__); } else { __noop(); }}

Controller* scenarioengine::InstantiateControllerECE_ALKS_DRIVER(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerECE_ALKS_DRIVER(initArgs);
}

ControllerECE_ALKS_DRIVER::ControllerECE_ALKS_DRIVER(InitArgs* args) : active_(false), setSpeed_(0), currentSpeed_(0),
	logging_(false), Controller(args)
{
	mode_ = Mode::MODE_ADDITIVE;

	if (args->properties->ValueExists("logging"))
	{
		if (args->properties->GetValueStr("logging") == "true")
		{
			logging_ = true;
		}
	}
}

void ControllerECE_ALKS_DRIVER::Init()
{
	Controller::Init();
}

void ControllerECE_ALKS_DRIVER::Step(double timeStep)
{
//	double minGapLength = LARGE_NUMBER;
	double minDist = 15.0;  // minimum distance to keep to lead vehicle
	double maxDeceleration = -10.0;
	double normalAcceleration = 3.0;

	double egoL = object_->boundingbox_.dimensions_.length_;
	double egoW = object_->boundingbox_.dimensions_.width_;
	double egoCx = object_->boundingbox_.center_.x_;
	double egoV = object_->GetSpeed();

	double targetL = 0.0;
	double targetW = 0.0;
	double targetCx = 0.0;
	double targetO = 0.0;
	double targetV = 0.0;
	double targetVT = 0.0;
	double targetAS = 0.0;

	double tCutIn = 0.0;
	double dsFree = 0.0;
	double acc = 0.0;
	double TTC = 0.0;

	// Lookahead distance is at least 50m or twice the distance required to stop
	// https://www.symbolab.com/solver/equation-calculator/s%5Cleft(t%5Cright)%3D2%5Cleft(m%2Bvt%2B%5Cfrac%7B1%7D%7B2%7Dat%5E%7B2%7D%5Cright)%2C%20t%3D%5Cfrac%7B-v%7D%7Ba%7D
	double lookaheadDist = MAX(50.0, minDist - pow(egoV, 2) / maxDeceleration);  // (m)

	// The entities with the smallest ds are coming first, this is important for cut-out scenario
	// In case of cut-out scenario it is crucial to detect first the cut-out vehicle and afterwards to check if the next vehicle
	// in front of cut-out vehicle is at TTC < 2sec.
	for (int i = static_cast<int>(entities_->object_.size()) - 1; i >= 0; i--)
	{
		if (entities_->object_[i] == object_)
		{
			continue;
		}
		if (aebBraking_)
		{
			// AEB brakes harder than driver, no need to continue checking for scenario if aeb is already braking
			break;
		}
		targetL = entities_->object_[i]->boundingbox_.dimensions_.length_;
		targetW = entities_->object_[i]->boundingbox_.dimensions_.width_;
		targetCx = entities_->object_[i]->boundingbox_.center_.x_;
		targetO = entities_->object_[i]->pos_.GetOffset();
		targetV = entities_->object_[i]->GetSpeed();
		targetVT = entities_->object_[i]->pos_.GetVelT();
		targetAS = entities_->object_[i]->pos_.GetAccS();

		// Measure longitudinal distance to all vehicles, don't utilize costly freespace option, instead measure ref point to ref point
		roadmanager::PositionDiff diff;
		if (object_->pos_.Delta(&entities_->object_[i]->pos_, diff, lookaheadDist) == true)
		{
			// path exists between position objects

			// object on adjacent lane, slower than ego and with a deviation of more than 0.375m in direction of ego from its lane center
			if (!driverBraking_ && fabs(targetVT) > SMALL_NUMBER && targetV < egoV + SMALL_NUMBER &&
				fabs(diff.dLaneId) == 1 && ((diff.dLaneId == -1 && targetVT > 0 && targetO > 0.375) ||
				(diff.dLaneId == 1 && targetVT < 0 && targetO < -0.375)))
			{
				tCutIn = 0.4;  //see risk perception time, this means after this time the cut-in was detected

				// relative heading angles are playing no role for reference driver
				dsFree = diff.ds - (0.5 * egoL + egoCx) - (0.5 * targetL - targetCx) - (egoV - targetV) * tCutIn;
				// check if the cut-in vehicle would be in front of ego after cut-in and within a TTC <=2sec, otherwise it can be ignored
				TTC = dsFree / fabs(egoV - targetV);
				if (egoV > 0 && dsFree >= 0 && targetV < egoV && TTC <= 2)
				{
					// cut-in would be in the red zone in front of ego vehicle after lane change
					// TTC (<= 2sec) + offset (> 0.375) + perception time (0.4sec, see plot of regulation)
					waitTime_ = 1.15;  // 0.75sec braking delay + 0.4sec risk perception time (distance a and b in plot of regulation)
					ALKS_LOG("ECE ALKS driver -> cut-in detected (TTC: %.2f) -> start braking after %.2f sec (braking delay + risk perception time)",
						TTC, waitTime_);
					driverBraking_ = true;
				}
			}
			// object in front on same lane
			else if (diff.dLaneId == 0 && diff.ds > 0)  // dLaneId == 0 indicates there is linked path between object lanes, i.e. no lane changes needed
			{
				// cut-out with object on same lane, but with a distance of more than 0.375m from lane center and lateral velocity into opposite direction of ego
				if (dtFreeCutOut_ < 0 && ((targetO > 0.375 && targetVT > 0) || (targetO < -0.375 && targetVT < 0)))
				{
					// relative heading angles are playing no role for reference driver
					if (dtFreeCutOut_ == -LARGE_NUMBER)
					{
						ALKS_LOG("ECE ALKS driver -> cut-out detected");
					}
					dtFreeCutOut_ = fabs(diff.dt) - 0.5 * (egoW + targetW);
				}

				// relative heading angles are playing no role for reference driver
				dsFree = diff.ds - (0.5 * egoL + egoCx) - (0.5 * targetL - targetCx);
				TTC = dsFree / fabs(egoV - targetV);
				// deceleration
				if (targetAS < 0 && dsFree >= 0 && TTC <= 2) // TTC of object in front <= 2sec
				{
					if (!driverBraking_)
					{
						waitTime_ = 0.75;  // 0.75sec braking delay
						if (targetAS < -5)
						{
							waitTime_ += 0.4;  // + 0.4sec risk perception time which begins when leading vehicle exceeds a deceleration of 5m/s2
							ALKS_LOG("ECE ALKS driver -> deceleration detected (as: %.2f, TTC: %.2f) -> start braking after %.2f sec (braking delay + risk perception time)",
								fabs(targetAS), TTC, waitTime_);
						}
						else
						{
							LOG("ECE ALKS driver -> deceleration detected (as: %.2f, TTC: %.2f) -> start braking after %.2f sec (braking delay, no risk perception time)",
								fabs(targetAS), TTC, waitTime_);
						}
						driverBraking_ = true;
					}

					// from cut-in and cut-out one can conclude for deceleration scenario, that AEB is only braking in case of full wrap, right decision???
					// AEB has here no delay, would directly brake as long as TTC <= TTC AEB (which is estimated to be the same as for driver 2sec)
					// But by this definition the driver braking delay and perception time will play no role
					// But in total one must not find here the exact definition because from regulation one knows that the driver always avoids a collision.
					// Thus one can directly brake with AEB
					if (!aebBraking_ && fabs(diff.dt) < SMALL_NUMBER)
					{
						ALKS_LOG("ECE ALKS AEB -> full wrap of ego and target (TTC: %.2f) -> start braking", TTC);
						aebBraking_ = true;
						// AEB brakes harder than driver, no need to continue checking for scenario if aeb is already braking
						break;
					}
				}
				else
				{
					// There exist a full wrap of ego with the next object in front and TTC <= 2sec and if there was a cut-out before,
					// then the cut-out vehicle has already left ego's driving path (lateral deviations omitted)
					if (!aebBraking_ && fabs(diff.dt) < SMALL_NUMBER && (dtFreeCutOut_ >= 0 || dtFreeCutOut_ == -LARGE_NUMBER) &&
						targetV < egoV && dsFree >= 0 && TTC <= 2)
					{
						ALKS_LOG("ECE ALKS AEB -> full wrap of ego and target (TTC: %.2f) -> start braking", TTC);
						aebBraking_ = true;
						// AEB brakes harder than driver, no need to continue checking for scenario if aeb is already braking
						break;
					}

					// There was a cut-out detected of another vehicle between this vehicle and ego
					// TTC between ego and object in front of cut-out object <= 2sec
					// Thus this vehicle (in front of cut-out vehicle) is in the red zone in the plot of regulation
					if (!driverBraking_ && dtFreeCutOut_ > -LARGE_NUMBER && dsFree >= 0 && dsFree / fabs(egoV - targetV) <= 2)
					{
						waitTime_ = 1.15;  // 0.75sec braking delay + 0.4sec risk perception time (distance a and b in plot of regulation)
						ALKS_LOG("ECE ALKS driver -> cut-out and next vehicle in front detected (TTC: %.2f) -> "
							"start braking after %.2f sec (braking delay + risk perception time)", TTC, waitTime_);
						driverBraking_ = true;
					}
				}
			}
		}
	}

	if (object_->CheckDirtyBits(Object::DirtyBit::SPEED))
	{
		// Speed has been set from somewhere else (another action or controller), respect it
		setSpeed_ = object_->GetSpeed();
	}

	currentSpeed_ = setSpeed_; // only needed if there is no driver
	if (egoV > 0)
	{
		// the AEB has no reaction time, thus the AEB is directly braking with a jerk of 0.85G during 0.6sec
		if (aebBraking_)
		{
			timeSinceBraking_ = MIN(timeSinceBraking_ + timeStep, 0.6);
			// jerk time of 0.6sec to decelerate with 0.85G
			// MAX comparing to 0, because it makes no sense to drive backwards
			acc = timeSinceBraking_ / 0.6 * 0.85;
			currentSpeed_ = MAX(0, egoV - acc * 9.81 * timeStep);
			ALKS_LOG("ECE ALKS AEB -> braking from %.2f to %.2f (acc: %.3fG)", egoV, currentSpeed_, MIN(acc, egoV / timeStep / 9.81));
		}
		// now the reference driver would brake
		else if (driverBraking_)
		{
			// the ego driver starts to react when the full wait time left and is braking with a jerk of 0.744G during 0.6sec
			if (waitTime_ == 0.0)
			{
				timeSinceBraking_ = MIN(timeSinceBraking_ + timeStep, 0.6);
				// jerk within time range of 0.6sec to decelerate with 0.774G
				// MAX comparing to 0, because it makes no sense to drive backwards
				acc = timeSinceBraking_ / 0.6 * 0.774;
				currentSpeed_ = MAX(0, egoV - acc * 9.81 * timeStep);
				ALKS_LOG("ECE ALKS driver -> wait time passed -> braking from %.2f to %.2f (acc: %.3fG)", egoV, currentSpeed_, MIN(acc, egoV / timeStep / 9.81));
			}
			waitTime_ = MAX(0.0, waitTime_ - timeStep);  // reduce waitTime by current timeStep each time this if branch is called
		}
	}

	if (!driverBraking_ && !aebBraking_)
	{
		// no lead vehicle to adapt to, adjust according to setSpeed
		double tmpSpeed = egoV + SIGN(setSpeed_ - currentSpeed_) * normalAcceleration * timeStep;
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

	if (currentSpeed_ == 0.0)
	{
		// after standstill by AEB or driver hold the velocity zero until a new velocity is set by a new controller
		setSpeed_ = 0.0;
		waitTime_ = -1.0;
		timeSinceBraking_ = 0.0;
		driverBraking_ = false;
		aebBraking_ = false;
	}

	Controller::Step(timeStep);
}

void ControllerECE_ALKS_DRIVER::Activate(ControlDomains domainMask)
{
	setSpeed_ = object_->GetSpeed();
	dtFreeCutOut_ = -LARGE_NUMBER;
	waitTime_ = -1.0;
	driverBraking_ = false;
	aebBraking_ = false;
	timeSinceBraking_ = 0.0;

	Controller::Activate(domainMask);
}

void ControllerECE_ALKS_DRIVER::ReportKeyEvent(int key, bool down)
{
}
