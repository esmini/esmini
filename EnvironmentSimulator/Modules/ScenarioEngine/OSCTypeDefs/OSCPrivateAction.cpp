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

#define _USE_MATH_DEFINES
#include <math.h>

#include "OSCPrivateAction.hpp"
#include "ScenarioEngine.hpp"

#define MAX_DECELERATION -8.0
#define LONGITUDINAL_DISTANCE_THRESHOLD 0.1

using namespace scenarioengine;

// Equations used in TransitionDynamics
// ------------------------------------
// Step: Trivial
// Linear: Trivial
// Sinusoidal:
//		f(x) = a-b*(cos(pi*x/c)-1)/2
//		f'(x) = pi*b*sin(pi*x/c)/(2*c)
//		Slope peaks midway, at x=c/2 =>
//		f'peak = pi*b/(2*c)
//		f''(x): b*pi^2*cos(pi*x/c)/(2c^2)
//		Acceleration peaks at endpoints (x=0, x=c)
//      f''peak: f''(0) = pi^2*b/(2c^2), f''(c) = -b*pi^2/(2c^2)
//      plot: "https://www.desmos.com/calculator/ikrmaen0mm"
//		f': "https://www.wolframalpha.com/input/?i=d/dx(a-b*(cos(pi*x/c)-1)/2)"
//		f'peak: "https://www.wolframalpha.com/input/?i=d/dx(a-b*(cos(pi*x/c)-1)/2),x=c/2"
//		f'': "https://www.wolframalpha.com/input/?i=d/dx(pi*b*sin(pi*x/c)/(2*c))"
// Cubic:
//		f(x) = a+3b(x/c)^2-2b(x/c)^3
//		f'(x) = 6bx(c-x)/c^3
//		Slope (rate) peaks midway, at x=c/2 =>
//      f'peak = f'(c/2) = 3*b/(2*c)
//      f''(x) = 6b(c-2x)/c^3
//		Acceleration peaks at endpoints (x=0, x=c)
//      f''peak: f''(0) = 6b/c^2, f''(c) = -6b/c^2
//		plot: "https://www.desmos.com/calculator/6t6wbeeos8"
//		f': "https://www.wolframalpha.com/input/?i=d/dx(a%2B3b(x/c)^2-2b(x/c)^3)"
//		f'peak: "https://www.wolframalpha.com/input/?i=d/dx(a%2B3b(x/c)^2-2b(x/c)^3),x=c/2"
//      f'': Trivial

double OSCPrivateAction::TransitionDynamics::EvaluatePrimPeak()
{
	if (dimension_ == DynamicsDimension::RATE)
	{
		return GetRate();
	}
	else
	{
		if (shape_ == DynamicsShape::STEP)
		{
			return LARGE_NUMBER * SIGN(GetTargetVal() - GetStartVal());
		}
		else if (shape_ == DynamicsShape::LINEAR)
		{
			return (GetTargetVal() - GetStartVal()) / AVOID_ZERO(GetParamTargetVal());
		}
		else if (shape_ == DynamicsShape::SINUSOIDAL)
		{
			return M_PI * (GetTargetVal() - GetStartVal()) / (2 * AVOID_ZERO(GetParamTargetVal()));
		}
		else if (shape_ == DynamicsShape::CUBIC)
		{
			return 1.5 * (GetTargetVal() - GetStartVal()) / AVOID_ZERO(GetParamTargetVal());
		}
		else
		{
			LOG("Invalid Dynamics shape: %d", shape_);
		}
	}

	return 0;
}

double OSCPrivateAction::TransitionDynamics::GetTargetParamValByPrimPeak(double prim_peak)
{
	if (shape_ == DynamicsShape::STEP)
	{
		return 0.0;
	}
	else if (shape_ == DynamicsShape::LINEAR)
	{
		return (GetTargetVal() - GetStartVal()) / prim_peak;
	}
	else if (shape_ == DynamicsShape::SINUSOIDAL)
	{
		return M_PI * (GetTargetVal() - GetStartVal()) / (2 * prim_peak);
	}
	else if (shape_ == DynamicsShape::CUBIC)
	{
		return 1.5 * (GetTargetVal() - GetStartVal()) / prim_peak;
	}
	else
	{
		LOG("Invalid Dynamics shape: %d", shape_);
	}

	return 0.0;
}

double OSCPrivateAction::TransitionDynamics::GetTargetParamValByPrimPrimPeak(double prim_prim_peak)
{
	if (shape_ == DynamicsShape::STEP)
	{
		return 0.0;
	}
	else if (shape_ == DynamicsShape::LINEAR)
	{
		// Acceleration is infinite at start and end, anything else should result in flat line
		// Just to have something reasonable, re-use acc equation from CUBIC case
		return sqrt(6 * abs(GetTargetVal() - GetStartVal()) / prim_prim_peak);
	}
	else if (shape_ == DynamicsShape::SINUSOIDAL)
	{
		// pi*sqrt(abs(b)/(2*prim_prim_peak))
		return M_PI * sqrt(abs(GetTargetVal() - GetStartVal()) / (2 * prim_prim_peak));
	}
	else if (shape_ == DynamicsShape::CUBIC)
	{
		// sqrt(6*abs(b)/y)
		return sqrt(6 * abs(GetTargetVal() - GetStartVal()) / prim_prim_peak);
	}
	else
	{
		LOG("Invalid Dynamics shape: %d", shape_);
	}

	return 0.0;
}

double OSCPrivateAction::TransitionDynamics::EvaluatePrim()
{
	if (shape_ == DynamicsShape::STEP)
	{
		return LARGE_NUMBER * SIGN(GetTargetVal() - GetStartVal());
	}
	else if (shape_ == DynamicsShape::LINEAR)
	{
		return (GetTargetVal() - GetStartVal()) / AVOID_ZERO(GetParamTargetVal());
	}
	else if (shape_ == DynamicsShape::SINUSOIDAL)
	{
		return M_PI * (GetTargetVal() - GetStartVal()) * sin(M_PI * param_val_ / GetParamTargetVal()) /
			(2 * AVOID_ZERO(GetParamTargetVal()));
	}
	else if (shape_ == DynamicsShape::CUBIC)
	{
		return 6 * (GetTargetVal() - GetStartVal()) * (GetParamTargetVal() - param_val_) * param_val_ /
			pow(AVOID_ZERO(GetParamTargetVal()), 3);
	}
	else
	{
		LOG("Invalid Dynamics shape: %d", shape_);
	}

	return 0;
}

double OSCPrivateAction::TransitionDynamics::EvaluateScaledPrim()
{
	return EvaluatePrim() / AVOID_ZERO(scale_factor_);
}

double OSCPrivateAction::TransitionDynamics::Evaluate()
{
	if (shape_ == DynamicsShape::STEP)
	{
		return GetTargetVal();
	}
	else if (shape_ == DynamicsShape::LINEAR)
	{
		return GetStartVal() + GetParamVal() * (GetTargetVal() - GetStartVal()) / (AVOID_ZERO(GetParamTargetVal()));
	}
	else if (shape_ == DynamicsShape::SINUSOIDAL)
	{
		return GetStartVal() - (GetTargetVal() - GetStartVal()) * (cos(M_PI * GetParamVal() / AVOID_ZERO(GetParamTargetVal())) - 1) / 2;
	}
	else if (shape_ == DynamicsShape::CUBIC)
	{
		return GetStartVal() + (GetTargetVal() - GetStartVal()) * pow(GetParamVal() / AVOID_ZERO(GetParamTargetVal()), 2) *
			(3 - 2 * GetParamVal() / AVOID_ZERO(GetParamTargetVal()));
	}
	else
	{
		LOG("Invalid Dynamics shape: %d", shape_);
	}

	return GetTargetVal();
}

void OSCPrivateAction::TransitionDynamics::Reset()
{
	scale_factor_ = 1.0;
	param_val_ = 0.0;
	start_val_ = 0.0;
	target_val_ = 0.0;
}

int OSCPrivateAction::TransitionDynamics::Step(double delta_param_val)
{
	param_val_ += delta_param_val / scale_factor_;

	return 0;
}

void OSCPrivateAction::TransitionDynamics::SetStartVal(double start_val)
{
	start_val_ = start_val;
	UpdateRate();
}

void OSCPrivateAction::TransitionDynamics::SetTargetVal(double target_val)
{
	target_val_ = target_val;
	UpdateRate();
}

void OSCPrivateAction::TransitionDynamics::SetParamTargetVal(double target_value)
{
	if (dimension_ != DynamicsDimension::RATE)
	{
		param_target_val_ = AVOID_ZERO(target_value);
	}
	else
	{
		// Interpret the target parameter value as rate
		SetRate(target_value);
	}
}

void OSCPrivateAction::TransitionDynamics::SetMaxRate(double max_rate)
{
	// Check max rate
	double peak_rate = EvaluatePrimPeak();

	if (abs(peak_rate) > abs(max_rate))
	{
		scale_factor_ = abs(peak_rate) / abs(AVOID_ZERO(max_rate));
	}
	else
	{
		scale_factor_ = 1.0;
	}
}

void OSCPrivateAction::TransitionDynamics::SetRate(double rate)
{
	// Adapt sign
	rate_ = rate;

	UpdateRate();
}

void OSCPrivateAction::TransitionDynamics::UpdateRate()
{
	// Adapt sign
	rate_ = SIGN(GetTargetVal() - GetStartVal()) * abs(rate_);

	if (dimension_ == DynamicsDimension::RATE)
	{
		// Find out parameter range from rate
		param_target_val_ = AVOID_ZERO(GetTargetParamValByPrimPeak(rate_));
	}
}

void AssignRouteAction::Start(double simTime, double dt)
{
	object_->pos_.SetRoute(route_);

	OSCAction::Start(simTime, dt);

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// lateral motion controlled elsewhere
		return;
	}
}

void AssignRouteAction::Step(double, double)
{
	OSCAction::End();
}

void AssignRouteAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}
	for (size_t i = 0; i < route_->minimal_waypoints_.size(); i++)
	{
		route_->minimal_waypoints_[i].ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
	}
}

void FollowTrajectoryAction::Start(double simTime, double dt)
{
	OSCAction::Start(simTime, dt);

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// lateral motion controlled elsewhere
		// other action or controller already updated lateral dimension of object
		// potentially longitudinal dimension could be updated separatelly - but skip that for now
		return;
	}

	traj_->Freeze();
	object_->pos_.SetTrajectory(traj_);

	object_->pos_.SetTrajectoryS(initialDistanceOffset_);
	time_ = traj_->GetTimeAtS(initialDistanceOffset_);

	// We want the trajectory to be projected on road surface.
	object_->pos_.SetAlignMode(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);

	// But totally decouple trajectory positioning from road heading
	object_->pos_.SetAlignModeH(roadmanager::Position::ALIGN_MODE::ALIGN_SOFT);

	object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
}

void FollowTrajectoryAction::End()
{
	OSCAction::End();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		return;
	}

	// Disconnect trajectory
	object_->pos_.SetTrajectory(0);

	// And reset align mode
	object_->pos_.SetAlignMode(roadmanager::Position::ALIGN_MODE::ALIGN_SOFT);
}

void FollowTrajectoryAction::Step(double simTime, double dt)
{
	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// lateral motion controlled elsewhere
		// other action or controller already updated lateral dimension of object
		// potentially longitudinal dimension could be updated separatelly - but skip that for now
		return;
	}

	double old_s = object_->pos_.GetTrajectoryS();

	time_ += timing_scale_ * dt;

	// Adjust time for any ghost headstart
	double timeOffset = object_->IsGhost() ? object_->GetHeadstartTime() : 0.0;

	// Move along trajectory
	if (
		// Ignore any timing info in trajectory
		timing_domain_ == TimingDomain::NONE ||
		// Speed is controlled elsewhere - just follow trajectory with current speed
		(object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
			object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LONG)))
	{
		object_->pos_.MoveTrajectoryDS(object_->speed_ * dt);
		object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
	}
	else if (timing_domain_ == TimingDomain::TIMING_RELATIVE)
	{
		object_->pos_.SetTrajectoryPosByTime(time_ + timeOffset + timing_offset_);
		if (time_ + timeOffset <= traj_->GetStartTime() + traj_->GetDuration())
		{
			// don't calculate and update actual speed when reached end of trajectory,
			// since the movement is based on remaining length of trajectory, not speed
			object_->SetSpeed((object_->pos_.GetTrajectoryS() - old_s) / MAX(SMALL_NUMBER, dt));
		}
		object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
	}
	else if (timing_domain_ == TimingDomain::TIMING_ABSOLUTE)
	{
		double s_old = object_->pos_.GetTrajectoryS();
		object_->pos_.SetTrajectoryPosByTime(simTime * timing_scale_ + timeOffset + timing_offset_);
		if (time_ + timeOffset <= traj_->GetStartTime() + traj_->GetDuration())
		{
			// don't calculate and update actual speed when reached end of trajectory,
			// since the movement is based on remaining length of trajectory, not speed
			object_->SetSpeed((object_->pos_.GetTrajectoryS() - s_old) / MAX(SMALL_NUMBER, dt));
		}
		object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
	}

	// Check end conditions:
	// Trajectories with no time stamps:
	//     closed trajectories have no end
	//     open trajectories simply ends when s >= length of trajectory
	// Trajectories with time stamps:
	//     always ends when time >= trajectory duration (last timestamp)
	if (((timing_domain_ == TimingDomain::NONE && !traj_->closed_ && object_->pos_.GetTrajectoryS() > (traj_->GetLength() - SMALL_NUMBER)) ||
		 (timing_domain_ != TimingDomain::NONE && time_ + timeOffset >= traj_->GetStartTime() + traj_->GetDuration())))
	{
		// Reached end of trajectory
		// Calculate road coordinates from final inertia (X, Y) coordinates
		object_->pos_.XYZH2TrackPos(object_->pos_.GetX(), object_->pos_.GetY(), 0, object_->pos_.GetH());

		double remaningDistance = 0.0;
		if (timing_domain_ == TimingDomain::NONE && !traj_->closed_ && object_->pos_.GetTrajectoryS() > (traj_->GetLength() - SMALL_NUMBER))
		{
			// Move the remaning distance along road at current lane offset
			remaningDistance = object_->speed_ * dt - (object_->pos_.GetTrajectoryS() - old_s);
		}
		else if (timing_domain_ != TimingDomain::NONE && time_ + timeOffset >= traj_->GetStartTime() + traj_->GetDuration())
		{
			// Move the remaning distance along road at current lane offset
			double remaningTime = time_ + timeOffset - (traj_->GetStartTime() + traj_->GetDuration());
			remaningDistance = remaningTime * object_->speed_;
		}

		// Move the remainder of distance along the current heading
		double dx = remaningDistance * cos(object_->pos_.GetH());
		double dy = remaningDistance * sin(object_->pos_.GetH());

		object_->pos_.SetInertiaPos(object_->pos_.GetX() + dx, object_->pos_.GetY() + dy, object_->pos_.GetH());
		object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);

		End();
	}
}

void FollowTrajectoryAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}
	if (traj_->shape_->type_ == roadmanager::Shape::ShapeType::CLOTHOID)
	{
		roadmanager::ClothoidShape* cl = (roadmanager::ClothoidShape*)traj_->shape_;
		cl->pos_.ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
	}
	else if (traj_->shape_->type_ == roadmanager::Shape::ShapeType::POLYLINE)
	{
		roadmanager::PolyLineShape* pl = (roadmanager::PolyLineShape*)traj_->shape_;
		for (size_t i = 0; i < pl->vertex_.size(); i++)
		{
			pl->vertex_[i]->pos_.ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
		}
	}

}

void AcquirePositionAction::Start(double simTime, double dt)
{
	// Resolve route
	route_ = new roadmanager::Route;
	route_->setName("AcquirePositionRoute");

	route_->AddWaypoint(&object_->pos_);
	route_->AddWaypoint(target_position_);

	object_->pos_.SetRoute(route_);

	OSCAction::Start(simTime, dt);

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// lateral motion controlled elsewhere
		return;
	}
}

void AcquirePositionAction::Step(double, double)
{
	OSCAction::End();
}

void AcquirePositionAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}
}

void AssignControllerAction::Start(double simTime, double dt)
{
	if (controller_ == 0)
	{
		// Detach any controller from object
		if (object_->controller_)
		{
			Controller* ctrl = (Controller*)object_->controller_;
			ctrl->Assign(0);
			object_->SetAssignedController(0);
		}
	}
	else
	{
		controller_->Assign(object_);

		if (object_->controller_)
		{
			if (!object_->controller_->Active())
			{
				if (domainMask_ != ControlDomains::DOMAIN_NONE)
				{
					object_->controller_->Activate(domainMask_);
					LOG("Controller %s activated, domain mask=0x%X", object_->controller_->GetName().c_str(), domainMask_);
				}
			}
			else
			{
				LOG("Controller %s already active (domainmask 0x%X), should not happen when just assigned!",
					object_->controller_->GetName().c_str(), domainMask_);
			}
		}
	}

	OSCAction::Start(simTime, dt);
}

void LatLaneChangeAction::Start(double simTime, double dt)
{
	OSCAction::Start(simTime, dt);
	int target_lane_id_ = 0;

	transition_.Reset();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// lateral motion controlled elsewhere
		return;
	}

	if (target_->type_ == Target::Type::ABSOLUTE_LANE)
	{
		target_lane_id_ = target_->value_;
	}
	else if (target_->type_ == Target::Type::RELATIVE_LANE)
	{
		// Find out target lane relative referred vehicle
		target_lane_id_ = ((TargetRelative*)target_)->object_->pos_.GetLaneId() + target_->value_;

		if (target_lane_id_ == 0 || SIGN(((TargetRelative*)target_)->object_->pos_.GetLaneId()) != SIGN(target_lane_id_))
		{
			// Skip reference lane (id == 0)
			target_lane_id_ = SIGN(target_lane_id_ - object_->pos_.GetLaneId()) * (abs(target_lane_id_) + 1);
		}
	}

	// Switch internal position to the target lane
	internal_pos_ = object_->pos_;
	internal_pos_.ForceLaneId(target_lane_id_);

	// Make offsets agnostic to lane sign
	transition_.SetStartVal(SIGN(internal_pos_.GetLaneId()) * internal_pos_.GetOffset());
	transition_.SetTargetVal(SIGN(target_lane_id_) * target_lane_offset_);

	// Set initial state
	internal_pos_.SetLanePos(internal_pos_.GetTrackId(), internal_pos_.GetLaneId(), internal_pos_.GetS(),
		SIGN(internal_pos_.GetLaneId()) * transition_.Evaluate());
}

void LatLaneChangeAction::Step(double simTime, double dt)
{
	double offset_agnostic = internal_pos_.GetOffset() * internal_pos_.GetLaneId();
	double angle = 0;

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// lateral motion controlled elsewhere
		return;
	}

	if (abs(object_->GetSpeed()) < SMALL_NUMBER)
	{
		return;
	}

	// Add a constraint that lateral speed may not exceed longitudinal
	transition_.SetMaxRate(object_->GetSpeed());
	offset_agnostic = transition_.Evaluate();
	double rate = transition_.EvaluateScaledPrim();

	// Fetch any assigned route and/or trajectory
	internal_pos_.SetTrajectory(object_->pos_.GetTrajectory());
	internal_pos_.SetTrajectoryS(object_->pos_.GetTrajectoryS());
	internal_pos_.SetTrajectoryT(object_->pos_.GetTrajectoryT());
	internal_pos_.SetRoute(object_->pos_.GetRoute());

	// Update internal position with new offset
	internal_pos_.SetLanePos(internal_pos_.GetTrackId(), internal_pos_.GetLaneId(), internal_pos_.GetS(), offset_agnostic * SIGN(internal_pos_.GetLaneId()));

	// Update longitudinal position
	double ds = object_->pos_.DistanceToDS(object_->speed_ * dt);
	roadmanager::Position::ErrorCode retval = roadmanager::Position::ErrorCode::ERROR_NO_ERROR;
	if (internal_pos_.GetRoute() && !internal_pos_.GetRoute()->invalid_route_)
	{
		retval = internal_pos_.MoveRouteDS(ds, false);
		object_->pos_ = internal_pos_;
	}
	else
	{
		retval = internal_pos_.MoveAlongS(ds, 0.0, -1.0);
		object_->pos_ = internal_pos_;

		// Attach object position to closest road and lane, look up via inertial coordinates
		object_->pos_.XYZH2TrackPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
	}

	if (object_->pos_.GetRoute())
	{
		// Check whether updated position still is on the route, i.e. same road ID, or not
		if (!object_->pos_.IsInJunction() && !internal_pos_.GetTrackId() && object_->pos_.GetTrackId() != internal_pos_.GetTrackId())
		{
			LOG("Warning/Info: LaneChangeAction moved away from route (track id %d -> track id %d), disabling route",
				object_->pos_.GetTrackId(), internal_pos_.GetTrackId());
			object_->pos_.SetRoute(nullptr);
		}
	}

	if (transition_.GetParamVal() > transition_.GetParamTargetVal() - SMALL_NUMBER ||
		// Close enough?
		fabs(offset_agnostic - transition_.GetTargetVal()) < SMALL_NUMBER ||
		// Passed target value?
		transition_.GetParamVal() > 0 && SIGN(offset_agnostic - transition_.GetTargetVal()) != SIGN(transition_.GetStartVal() - transition_.GetTargetVal()))
	{
		OSCAction::End();
		object_->pos_.SetHeadingRelativeRoadDirection(0);
	}
	else
	{
		if (transition_.dimension_ == DynamicsDimension::DISTANCE)
		{
			angle = atan(rate);
		}
		else
		{
			// Convert rate (lateral-movment/time) to lateral-movement/long-movement
			angle = atan(rate / AVOID_ZERO(object_->GetSpeed()));
		}
		object_->pos_.SetHeadingRelativeRoadDirection((IsAngleForward(internal_pos_.GetHRelative()) ? 1 : -1) * SIGN(internal_pos_.GetLaneId()) * angle);
	}

	if (transition_.dimension_ == DynamicsDimension::DISTANCE)
	{
		transition_.Step(dt * object_->GetSpeed());
	}
	else
	{
		transition_.Step(dt);
	}

	if (retval == roadmanager::Position::ErrorCode::ERROR_END_OF_ROAD)
	{
		object_->SetSpeed(0.0);
	}

	object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
}

void LatLaneChangeAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}

	if (target_->type_ == Target::Type::RELATIVE_LANE)
	{
		if (((TargetRelative*)target_)->object_ == obj1)
		{
			((TargetRelative*)target_)->object_ = obj2;
		}
	}
}

void LatLaneOffsetAction::Start(double simTime, double dt)
{
	OSCAction::Start(simTime, dt);
	transition_.Reset();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// lateral motion controlled elsewhere
		return;
	}

	if (target_->type_ == Target::Type::ABSOLUTE_OFFSET)
	{
		transition_.SetTargetVal(SIGN(object_->pos_.GetLaneId()) * target_->value_);
	}
	else if (target_->type_ == Target::Type::RELATIVE_OFFSET)
	{
		// Register what lane action object belongs to
		int lane_id = object_->pos_.GetLaneId();

		// Find out referred object track position
		roadmanager::Position refpos = ((TargetRelative*)target_)->object_->pos_;
		refpos.SetTrackPos(refpos.GetTrackId(), refpos.GetS(), refpos.GetT() + target_->value_);
		refpos.ForceLaneId(lane_id);

		// Target lane offset = t value of requested lane + offset relative t value of current lane without offset
		transition_.SetTargetVal(SIGN(object_->pos_.GetLaneId()) * (refpos.GetT() - (object_->pos_.GetT() - object_->pos_.GetOffset())));
	}

	transition_.SetStartVal(SIGN(object_->pos_.GetLaneId()) * object_->pos_.GetOffset());
	transition_.SetParamTargetVal(transition_.GetTargetParamValByPrimPrimPeak(max_lateral_acc_));
}

void LatLaneOffsetAction::Step(double simTime, double dt)
{
	double offset_agnostic;

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LAT))
	{
		// lateral motion controlled elsewhere
		return;
	}

	offset_agnostic = transition_.Evaluate();

	object_->pos_.SetLanePos(object_->pos_.GetTrackId(), object_->pos_.GetLaneId(), object_->pos_.GetS(), SIGN(object_->pos_.GetLaneId()) * offset_agnostic);

	if (transition_.GetParamVal() > transition_.GetParamTargetVal() - SMALL_NUMBER ||
		// Close enough?
		fabs(offset_agnostic - transition_.GetTargetVal()) < SMALL_NUMBER ||
		// Passed target value?
		transition_.GetParamVal() > 0 && SIGN(offset_agnostic - transition_.GetTargetVal()) != SIGN(transition_.GetStartVal() - transition_.GetTargetVal()))
	{
		OSCAction::End();
		object_->pos_.SetHeadingRelativeRoadDirection(0);
	}
	else
	{
		// Convert rate (lateral-movment/time) to lateral-movement/long-movement
		double angle = atan(transition_.EvaluatePrim() / AVOID_ZERO(object_->GetSpeed()));
		object_->pos_.SetHeadingRelativeRoadDirection((IsAngleForward(object_->pos_.GetHRelative()) ? 1 : -1) * SIGN(object_->pos_.GetLaneId()) * angle);
	}

	object_->SetDirtyBits(Object::DirtyBit::LATERAL);

	transition_.Step(dt);
}

void LatLaneOffsetAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}

	if (target_->type_ == Target::Type::RELATIVE_OFFSET)
	{
		if (((TargetRelative*)target_)->object_ == obj1)
		{
			((TargetRelative*)target_)->object_ = obj2;
		}
	}
}
double LongSpeedAction::TargetRelative::GetValue()
{
	if (!continuous_)
	{
		// sample relative object speed once
		if (!consumed_)
		{
			object_speed_ = object_->speed_;
			consumed_ = true;
		}
	}
	else
	{
		object_speed_ = object_->speed_;
	}

	if (value_type_ == ValueType::DELTA)
	{
		return object_speed_ + value_;
	}
	else if (value_type_ == ValueType::FACTOR)
	{
		return object_speed_ * value_;
	}
	else
	{
		LOG("Invalid value type: %d", value_type_);
	}

	return 0;
}

void LongSpeedAction::Start(double simTime, double dt)
{
	OSCAction::Start(simTime, dt);
	transition_.Reset();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LONG))
	{
		// longitudinal motion controlled elsewhere
		OSCAction::End();
		return;
	}

	transition_.SetStartVal(object_->GetSpeed());

	if (transition_.dimension_ == DynamicsDimension::DISTANCE)
	{
		// Convert to time, since speed shape is expected over time, not distance (as in lane change case)
		// integrated distance = time(v_init + v_delta/2) = time(v_init + v_end)/2 => time = 2*distance/(v_init + v_end)
		transition_.SetParamTargetVal(2 * transition_.GetParamTargetVal() / (transition_.GetStartVal() + target_->GetValue()));
	}

	transition_.SetTargetVal(target_->GetValue());

	// Set initial state
	object_->SetSpeed(transition_.Evaluate());
}

void LongSpeedAction::Step(double simTime, double dt)
{
	double new_speed = 0;
	bool done = false;

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LONG))
	{
		// longitudinal motion controlled elsewhere
		OSCAction::End();
		return;
	}

	// Get target speed, which might be dynamic (relative other entitity)
	transition_.SetTargetVal(ABS_LIMIT(target_->GetValue(), object_->performance_.maxSpeed));
	if (transition_.GetTargetVal() > transition_.GetStartVal())
	{
		// Acceleration
		transition_.SetMaxRate(object_->performance_.maxAcceleration);
	}
	else
	{
		// Deceleration
		transition_.SetMaxRate(object_->performance_.maxDeceleration);
	}

	// Make sure sign of rate is correct
	if (transition_.dimension_ == DynamicsDimension::RATE)
	{
		transition_.SetRate(SIGN(transition_.GetTargetVal() - transition_.GetStartVal()) * abs(transition_.GetRate()));
	}

	transition_.Step(dt);
	new_speed = transition_.Evaluate();

	if (!(target_->type_ == Target::TargetType::RELATIVE_SPEED && ((TargetRelative*)target_)->continuous_ == true) &&
		 (transition_.GetParamVal() > transition_.GetParamTargetVal() - SMALL_NUMBER ||
		  // Close enough?
		  abs(new_speed - transition_.GetTargetVal()) < SMALL_NUMBER ||
		  // Already passed target value (perhaps due to strange initial conditions)?
		  SIGN(target_->GetValue() - transition_.GetStartVal()) != SIGN(target_->GetValue() - object_->GetSpeed())))
	{
		done = true;
		new_speed = ABS_LIMIT(target_->GetValue(), object_->performance_.maxSpeed);
	}

	object_->SetSpeed(new_speed);

	if (done)
	{
		OSCAction::End();
	}
}

void LongSpeedAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}

	if (target_->type_ == Target::TargetType::RELATIVE_SPEED)
	{
		if (((TargetRelative*)target_)->object_ == obj1)
		{
			((TargetRelative*)target_)->object_ = obj2;
		}
	}
}

void LongDistanceAction::Start(double simTime, double dt)
{
	sim_time_ = simTime;
	if (target_object_ == 0)
	{
		LOG("Can't trig without set target object ");
		return;
	}

	// Resolve displacement
	if (displacement_ == DisplacementType::ANY)
	{
		// Find out current displacement, and apply it
		double distance;
		if (freespace_)
		{
			double latDist = 0;
			double longDist = 0;
			object_->FreeSpaceDistance(target_object_, &latDist, &longDist);
			distance = longDist;
		}
		else
		{
			double x, y;
			distance = object_->pos_.getRelativeDistance(target_object_->pos_.GetX(), target_object_->pos_.GetY(), x, y);

			// Just interested in the x-axis component of the distance
			distance = x;
		}

		if (distance < 0.0)
		{
			displacement_ = DisplacementType::LEADING;
		}
		else
		{
			displacement_ = DisplacementType::TRAILING;
		}
	}

	OSCAction::Start(simTime, dt);
}

void LongDistanceAction::Step(double simTime, double)
{
	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LONG))
	{
		// longitudinal motion controlled elsewhere
		return;
	}

	double dt = simTime - sim_time_;
	sim_time_ = simTime;

	// Find out current distance
	double distance;
	if (freespace_)
	{
		double latDist = 0;
		double longDist = 0;
		object_->FreeSpaceDistance(target_object_, &latDist, &longDist);
		distance = longDist;
	}
	else
	{
		double x, y;
		distance = object_->pos_.getRelativeDistance(target_object_->pos_.GetX(), target_object_->pos_.GetY(), x, y);

		// Just interested in the x-axis component of the distance
		distance = x;
	}

	double speed_diff = object_->speed_ - target_object_->speed_;
	double acc;
	double spring_constant = 4;
	double dc;
	double requested_dist = 0;

	if (dist_type_ == DistType::DISTANCE)
	{
		requested_dist = distance_;
	}
	if (dist_type_ == DistType::TIME_GAP)
	{
		// Convert requested time gap (seconds) to distance (m)
		requested_dist = abs(target_object_->speed_) * distance_;
	}

	if (displacement_ == DisplacementType::TRAILING)
	{
		requested_dist = abs(requested_dist);
	}
	else if (displacement_ == DisplacementType::LEADING)
	{
		requested_dist = -abs(requested_dist);
	}

	double distance_diff = distance - requested_dist;

	if (continuous_ == false && fabs(distance_diff) < LONGITUDINAL_DISTANCE_THRESHOLD)
	{
		// Reached requested distance, quit action
		OSCAction::End();
	}

	if (dynamics_.none_ == true)
	{
		// Set position according to distance and copy speed of target vehicle
		object_->pos_.MoveAlongS(distance_diff);
		object_->SetSpeed(target_object_->speed_);
	}
	else
	{
		// Apply damped spring model with critical/optimal damping factor
		// Adjust tension in spring in proportion to max acceleration. Experimental, may be removed.
		double spring_constant_adjusted = 0.1 * dynamics_.max_acceleration_ * spring_constant;
		dc = 2 * sqrt(spring_constant_adjusted);
		acc = distance_diff * spring_constant_adjusted - speed_diff * dc;
		if (acc > dynamics_.max_acceleration_)
		{
			acc = dynamics_.max_acceleration_;
		}
		else if (acc < -dynamics_.max_deceleration_)
		{
			acc = -dynamics_.max_deceleration_;
		}

		object_->SetSpeed(object_->GetSpeed() + acc * dt);

		if (object_->GetSpeed() > dynamics_.max_speed_)
		{
			object_->SetSpeed(dynamics_.max_speed_);
		}
		else if (object_->GetSpeed() < -dynamics_.max_speed_)
		{
			object_->SetSpeed(-dynamics_.max_speed_);
		}
	}
}

void LongDistanceAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}

	if (target_object_ == obj1)
	{
		target_object_ = obj2;
	}
}

void TeleportAction::Start(double simTime, double dt)
{
	OSCAction::Start(simTime, dt);
	LOG("Starting teleport Action");

	if (object_->IsGhost() && scenarioEngine_->getSimulationTime() > 0)
	{
		scenarioEngine_->SetSimulationTime(scenarioEngine_->getSimulationTime() - scenarioEngine_->GetHeadstartTime());
		object_->trail_.Reset();

		if (object_->ghost_Ego_ != 0)
		{
			object_->SetSpeed(object_->ghost_Ego_->GetSpeed());
		}

		scenarioEngine_->ResetEvents(); // Ghost-project. Reset events finished by ghost.
	}

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActive())
	{
		// motion controlled elsewhere
		return;
	}

	object_->pos_.TeleportTo(position_);

	LOG("%s New position:", object_->name_.c_str());
	object_->pos_.Print();
	object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
	object_->reset_ = true;
}

void TeleportAction::Step(double, double)
{
	OSCAction::End();
}

void TeleportAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}
}

double SynchronizeAction::CalcSpeedForLinearProfile(double v_final, double time, double dist)
{
	if (time < 0.001 || dist < 0.001)
	{
		// Avoid division by zero, fall back to zero acceleration
		return 0;
	}

	// Compute current speed needed to reach given final speed in given time
	double v0 = 2 * dist / time - v_final;

	return v0;
}

const char* SynchronizeAction::Mode2Str(SynchMode mode)
{
	if (mode == SynchMode::MODE_NONE)
	{
		return "MODE_NONE";
	}
	else if (mode == SynchMode::MODE_NON_LINEAR)
	{
		return "MODE_NON_LINEAR";
	}
	else if (mode == SynchMode::MODE_LINEAR)
	{
		return "MODE_LINEAR";
	}
	else if (mode == SynchMode::MODE_STOPPED)
	{
		return "MODE_STOPPED";
	}
	else if (mode == SynchMode::MODE_STOP_IMMEDIATELY)
	{
		return "MODE_STOP_IMMEDIATELY";
	}
	else if (mode == SynchMode::MODE_WAITING)
	{
		return "MODE_WAITING";
	}
	else if (mode == SynchMode::MODE_STEADY_STATE)
	{
		return "MODE_STEADY_STATE";
	}
	else
	{
		return "Unknown mode";
	}
}

const char* SynchronizeAction::SubMode2Str(SynchSubmode submode)
{
	if (submode == SynchSubmode::SUBMODE_CONCAVE)
	{
		return "SUBMODE_CONCAVE";
	}
	else if (submode == SynchSubmode::SUBMODE_CONVEX)
	{
		return "SUBMODE_CONVEX";
	}
	else if (submode == SynchSubmode::SUBMODE_NONE)
	{
		return "SUBMODE_NONE";
	}
	else
	{
		return "Unknown sub-mode";
	}
}

void SynchronizeAction::PrintStatus(const char* custom_msg)
{
	LOG("%s, mode=%s (%d) sub-mode=%s (%d)", custom_msg,
		Mode2Str(mode_), mode_, SubMode2Str(submode_), submode_);
}

void SynchronizeAction::Start(double simTime, double dt)
{
	sim_time_ = simTime;

	// resolve steady state -> translate into dist
	if (steadyState_.type_ == SteadyStateType::STEADY_STATE_TIME)
	{
		steadyState_.dist_ = steadyState_.time_ * final_speed_->GetValue();
		steadyState_.type_ = SteadyStateType::STEADY_STATE_DIST;
	}
	else if (steadyState_.type_ == SteadyStateType::STEADY_STATE_POS)
	{
		// Find out distance between steady state position and final destination
		roadmanager::PositionDiff diff;
		target_position_->Delta(steadyState_.pos_, diff);
		steadyState_.dist_ = diff.ds;
		steadyState_.type_ = SteadyStateType::STEADY_STATE_DIST;
	}

	OSCAction::Start(simTime, dt);

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LONG))
	{
		// longitudinal motion controlled elsewhere
		return;
	}
}

void SynchronizeAction::Step(double simTime, double)
{
	bool done = false;

	double dt = simTime - sim_time_;
	sim_time_ = simTime;

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LONG))
	{
		// longitudinal motion controlled elsewhere
		return;
	}

	// Calculate distance along road/route
	double masterDist, dist;
	roadmanager::PositionDiff diff;

	if (master_object_->pos_.GetTrajectory() || !master_object_->pos_.Delta(target_position_master_, diff))
	{
		// No road network path between master vehicle and master target pos - using world coordinate distance
		diff.ds = GetLengthOfLine2D(master_object_->pos_.GetX(), master_object_->pos_.GetY(),
			target_position_master_->GetX(), target_position_master_->GetY());
	}
	masterDist = fabs(diff.ds);

	if (object_->pos_.GetTrajectory() || !object_->pos_.Delta(target_position_, diff))
	{
		// No road network path between action vehicle and action target pos - using world coordinate distance
		diff.ds = GetLengthOfLine2D(object_->pos_.GetX(), object_->pos_.GetY(),
			target_position_->GetX(), target_position_->GetY());
	}
	dist = fabs(diff.ds);

	// Done when distance increases, indicating that destination just has been reached or passed
	if (dist < tolerance_ + SMALL_NUMBER)
	{
		LOG("Synchronize dist (%.2f) < tolerance (%.2f)", dist, tolerance_);
		if (final_speed_)
		{
			object_->SetSpeed(final_speed_->GetValue());
		}
		done = true;
	}
	else if (masterDist < tolerance_master_ + SMALL_NUMBER)
	{
		LOG("Synchronize masterDist (%.2f) < tolerance (%.2f)", masterDist, tolerance_master_);
		if (final_speed_)
		{
			object_->SetSpeed(final_speed_->GetValue());
		}
		done = true;
	}
	else if (dist > lastDist_)
	{
		LOG("Synchronize dist increasing (%.2f > %.2f) - missed destination", dist, lastDist_);
		done = true;
	}
	else if (masterDist > lastMasterDist_)
	{
		LOG("Synchronize masterDist increasing (%.2f > %.2f) - missed destination", masterDist, lastMasterDist_);
		done = true;
	}

	lastDist_ = dist;
	lastMasterDist_ = masterDist;

	// for calculations, measure distance to toleration area/radius
	dist = MAX(dist - tolerance_, SMALL_NUMBER);
	masterDist = MAX(masterDist - tolerance_master_, SMALL_NUMBER);

	if (done)
	{
		OSCAction::End();
	}
	else
	{
		double masterTimeToDest = LARGE_NUMBER;
		if (master_object_->speed_ > SMALL_NUMBER)
		{
			masterTimeToDest = masterDist / master_object_->speed_;
		}
		double average_speed = dist / masterTimeToDest;
		double acc = 0;

		if (final_speed_)
		{
			if (steadyState_.type_ != SteadyStateType::STEADY_STATE_NONE && mode_ != SynchMode::MODE_STEADY_STATE)
			{
				double time_to_ss = steadyState_.dist_ / final_speed_->GetValue();

				if (dist - steadyState_.dist_ < SMALL_NUMBER || masterTimeToDest - time_to_ss < SMALL_NUMBER)
				{
					mode_ = SynchMode::MODE_STEADY_STATE;
					submode_ = SynchSubmode::SUBMODE_NONE;
					if (time_to_ss > masterTimeToDest && (time_to_ss - masterTimeToDest) * final_speed_->GetValue() > tolerance_)
					{
						LOG("Entering Stead State according to criteria but not enough time to reach destination");
					}
					//PrintStatus("SteadyState");
				}
				else
				{
					// subtract steady state distance
					dist -= steadyState_.dist_;

					// subtract steady state duration
					masterTimeToDest -= time_to_ss;
				}
			}

			// For more information about calculations, see
			// https://docs.google.com/document/d/1dEBUWlJVLUz6Rp9Ol1l90iG0LfNtcsgLyJ0kDdwgPzA/edit?usp=sharing
			//
			// Interactive Python script plotting calculation result based on various input values
			// https://drive.google.com/file/d/1z902gRYogkLhUAV1pZLc9gcgwnak7TBH/view?usp=sharing
			// (the method described below is "Spedified final speed - alt 1")
			//
			// Here follow a brief description:
			//
			// Calculate acceleration needed to reach the destination in due time
			// Four cases
			//   1  Linear. Reach final speed with constant acceleration
			//	 2a Non-linear convex (rush). First accelerate, then decelerate.
			//	 2b Non-linear concave (linger). First decelerate, then accelerate.
			//   3  Non-linear with stop. Decelerate to a stop. Wait. Then accelerate.
			//
			//   Case 2-3 involves two (case 2a, 2b) or three (case 3) phases with constant acceleration/deceleration
			//   Last phase in case 2-3 is actually case 1 - a linear change to final speed
			//
			// Symbols
			//   given:
			//     s = distance to destination
			//     t = master object time to destination
			//     v0 = current speed
			//     v1 = final speed
			//     va = Average speed needed to reach destination
			//   variables:
			//     s1 = distance first phase
			//     s2 = distance second phase
			//     x = end time for first phase
			//     y = end time for second (last) phase
			//     vx = speed at x
			//
			// Equations
			//   case 1
			//     v1 = 2 * s / t - v2
			//     a = (v2 - v1) / t
			//
			//   case 2 (a & b)
			//      system:
			//        s1 = v1 * x + (vx - v1) * x / 2
			//        s2 = v2 * y + (vx - v2) * y / 2
			//        t = x + y
			//        s = s1 + s2
			// 		  (vx - v1) / x = (vx - v2) / y
			//
			//      solve for x and vx
			//      a = (vx - v1) / x
			//
			//   case 3
			//      system:
			//        s1 = x * v1 / 2
			//        s2 = y * v2 / 2
			//        s = s1 + s2
			//        v1 / v2 = x / y
			//
			//      solve for x
			//      a = -v1 / x

			if (mode_ == SynchMode::MODE_STEADY_STATE)
			{
				object_->speed_ = final_speed_->GetValue();
				return;
			}
			if (mode_ == SynchMode::MODE_WAITING)
			{
				if (masterTimeToDest >= LARGE_NUMBER)
				{
					// Continue waiting
					return;
				}
				else
				{
					// Reset mode
					mode_ = SynchMode::MODE_NONE;
				}
			}
			if (mode_ == SynchMode::MODE_STOP_IMMEDIATELY)
			{
				acc = MAX_DECELERATION;
				object_->speed_ += acc * dt;
				if (object_->speed_ < 0)
				{
					object_->SetSpeed(0);
					mode_ = SynchMode::MODE_WAITING;  // wait for master to move
					//PrintStatus("Waiting");
				}

				return;
			}
			else if (mode_ == SynchMode::MODE_STOPPED)
			{
				if (masterTimeToDest < 2 * dist / final_speed_->GetValue())
				{
					// Time to move again after the stop
					mode_ = SynchMode::MODE_LINEAR;
					//PrintStatus("Restart");
				}
				else
				{
					// Stay still
					object_->SetSpeed(0);
					return;
				}
			}

			if (mode_ == SynchMode::MODE_LINEAR)
			{
				if (masterTimeToDest > LARGE_NUMBER - 1)
				{
					// Master in effect standing still, do not move
					object_->SetSpeed(0);
				}
				else
				{
					object_->SetSpeed(MAX(0, CalcSpeedForLinearProfile(MAX(0, final_speed_->GetValue()), masterTimeToDest, dist)));
				}
				return;
			}
			else if (mode_ == SynchMode::MODE_NON_LINEAR && masterTimeToDest < LARGE_NUMBER)
			{
				// Check if case 1, i.e. on a straight speed profile line
				double v0_onLine = 2 * dist / masterTimeToDest - final_speed_->GetValue();

				if (fabs(object_->speed_ - v0_onLine) < 0.1)
				{
					// Passed apex. Switch to linear mode (constant acc) to reach final destination and speed
					mode_ = SynchMode::MODE_LINEAR;

					// Keep current speed for this time step
					return;
				}
			}

			if (mode_ == SynchMode::MODE_NONE)
			{
				// Since not linear mode, go into non-linear mode
				mode_ = SynchMode::MODE_NON_LINEAR;

				// Find out submode, convex or concave
				if ((final_speed_->GetValue() + object_->speed_) / 2 < dist / masterTimeToDest)
				{
					submode_ = SynchSubmode::SUBMODE_CONVEX;
				}
				else
				{
					submode_ = SynchSubmode::SUBMODE_CONCAVE;
				}
				//PrintStatus("Non-linear");
			}

			// Now, calculate x and vx according to default method oulined in the documentation
			double s = dist;
			double t = masterTimeToDest;
			double v0 = object_->speed_;
			double v1 = final_speed_->GetValue();

			double signed_term = sqrt(2.0) * sqrt(2.0 * s*s - 2 * (v1 + v0)*t*s + (v1*v1 + v0 * v0)*t*t);

			// Calculate both solutions from the quadratic equation
			double vx = 0;
			if (fabs(v1 - v0) < SMALL_NUMBER)
			{
				// When v0 == v1, x is simply t/2
				// s = (T / 2) * v_cur + (T / 2) * vx -> vx = (s - (T / 2) * v_cur) / (T / 2)
				vx = (s - (t / 2) * v0) / (t / 2);
				acc = (vx - v0) / (t / 2);
			}
			else
			{
				double x1 = -(signed_term + 2 * s - 2 * v1*t) / (2 * (v1 - v0));
				double x2 = -(-signed_term + 2 * s - 2 * v1*t) / (2 * (v1 - v0));
				double vx1 = (2 * s - signed_term) / (2 * t);
				double vx2 = (2 * s + signed_term) / (2 * t);
				double a1 = (vx1 - v0) / x1;
				double a2 = (vx2 - v0) / x2;

				// Choose solution, only one is found within the given time span [0:masterTimeToDest]
				if (x1 > 0 && x1 < t)
				{
					vx = vx1;
					acc = a1;
				}
				else if (x2 > 0 && x2 < t)
				{
					vx = vx2;
					acc = a2;
				}
				else
				{
					// No solution
					acc = 0;
				}
			}

			if (mode_ == SynchMode::MODE_NON_LINEAR &&
				((submode_ == SynchSubmode::SUBMODE_CONCAVE && acc > 0) ||
				(submode_ == SynchSubmode::SUBMODE_CONVEX && acc < 0)))
			{
				// Reached the apex of the speed profile, switch mode and phase
				mode_ = SynchMode::MODE_LINEAR;
				//PrintStatus("Reached apex");

				// Keep speed for this time step
				acc = 0;
			}

			// Check for case 3, where target speed(vx) < 0
			if (mode_ == SynchMode::MODE_NON_LINEAR && vx < 0)
			{
				// In phase one, decelerate to 0, then stop
				// Calculate time needed to cover distance proportional to current speed / final speed
				double t1 = 2 * v0*s / (v0*v0 + v1 * v1);
				if (fabs(t1) > SMALL_NUMBER)
				{
					acc = -v0 / t1;
				}
				else
				{
					acc = 0;
				}

				if (t1 * v0 / 2 > s / 2)
				{
					// If more than half distance to destination needed, then stop immediatelly
					acc = MAX_DECELERATION;
					mode_ = SynchMode::MODE_STOP_IMMEDIATELY;
					//PrintStatus("Stop immediately");
				}

				if (v0 + acc * dt < 0)
				{
					// Reached to a stop
					object_->SetSpeed(0);
					mode_ = SynchMode::MODE_STOPPED;
					//PrintStatus("Stopped");

					return;
				}
			}
		}
		else
		{
			// No final speed specified. Calculate it based on current speed and available time
			double final_speed = 2 * average_speed - object_->speed_;
			acc = (final_speed - object_->speed_) / masterTimeToDest;
		}

		object_->SetSpeed(object_->GetSpeed() + acc * dt);
	}
}

void VisibilityAction::Start(double simTime, double dt)
{
	OSCAction::Start(simTime, dt);
	object_->SetVisibilityMask(
		(graphics_ ? Object::Visibility::GRAPHICS : 0) |
		(traffic_ ? Object::Visibility::TRAFFIC : 0) |
		(sensors_ ? Object::Visibility::SENSORS : 0)
	);
}

void VisibilityAction::Step(double, double)
{
	OSCAction::End();
}

int OverrideControlAction::AddOverrideStatus(Object::OverrideActionStatus status)
{
	overrideActionList.push_back(status);
	if (status.type < Object::OverrideType::OVERRIDE_NR_TYPES)
	{
		if (status.type == Object::OverrideType::OVERRIDE_STEERING_WHEEL)
		{
			if (status.active)
			{
				domain_ = static_cast<ControlDomains>(static_cast<int>(domain_) | static_cast<int>(ControlDomains::DOMAIN_LAT));
			}
			else
			{
				domain_ = static_cast<ControlDomains>(static_cast<int>(domain_) & ~static_cast<int>(ControlDomains::DOMAIN_LAT));
			}
		}
		else
		{
			if (status.active)
			{
				domain_ = static_cast<ControlDomains>(static_cast<int>(domain_) | static_cast<int>(ControlDomains::DOMAIN_LONG));
			}
			else
			{
				domain_ = static_cast<ControlDomains>(static_cast<int>(domain_) & ~static_cast<int>(ControlDomains::DOMAIN_LONG));
			}
		}
	}
	else
	{
		LOG_AND_QUIT("Unexpected override type: %d", status.type);
	}

	return 0;
}

void OverrideControlAction::Start(double simTime, double dt)
{
	for (size_t i = 0; i < overrideActionList.size(); i++)
	{
		object_->overrideActionList[overrideActionList[i].type] = overrideActionList[i];
	}
	OSCAction::Start(simTime, dt);
}

void OverrideControlAction::Step(double simTime, double dt)
{
	OSCAction::End();
}

double OverrideControlAction::RangeCheckAndErrorLog(Object::OverrideType type, double valueCheck, double lowerLimit, double upperLimit, bool ifRound)
{
	double temp = valueCheck;
	if(valueCheck<=upperLimit&&valueCheck>=lowerLimit)
	{
		if(!ifRound){
			LOG("%d value %.2f is within range.", type, valueCheck);
		}
		else
		{
			valueCheck = round(temp);
			LOG("%d value %.1f is within range and the value is rounded to %.1f.", type, temp,valueCheck);
		}
	}
	else
	{
		valueCheck = (valueCheck>upperLimit)?upperLimit:lowerLimit;
		LOG("%d value is not within range and is modified from %f to %.1f.", type, temp, valueCheck);
	}
	return valueCheck;
}