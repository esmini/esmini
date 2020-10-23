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

#define MAX(x, y) (y > x ? y : x)
#define MIN(x, y) (y < x ? y : x)
#define MAX_DECELERATION -8.0

using namespace scenarioengine;

double OSCPrivateAction::TransitionDynamics::Evaluate(double factor, double start_value, double end_value)
{
	if (factor > 1.0)
	{
		factor = 1.0;
	}
	if (shape_ == DynamicsShape::STEP)
	{
		return end_value;
	}
	else if (shape_ == DynamicsShape::LINEAR)
	{
		return start_value + factor * (end_value - start_value);
	}
	else if (shape_ == DynamicsShape::SINUSOIDAL)
	{
		// cosine(angle + PI) gives a value in interval [-1 : 1], add 1 and normalize (divide by 2)
		double val = start_value + (end_value - start_value) * (1 + cos(M_PI * (1 + factor))) / 2.0;
		return val;
	}
	else if (shape_ == DynamicsShape::CUBIC)
	{
		LOG("Dynamics Cubic not implemented");
		return end_value;
	}
	else
	{
		LOG("Invalid Dynamics shape: %d", shape_);
	}
	
	return end_value;
}

void AssignRouteAction::Start()
{
	object_->pos_.SetRoute(route_);

	OSCAction::Start();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))
	{
		// lateral motion controlled elsewhere
		return;
	}
}

void AssignRouteAction::End()
{
	// Disconnect route
	object_->pos_.SetRoute(0);

	OSCAction::End();
}

void AssignRouteAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}
	for (size_t i = 0; i < route_->waypoint_.size(); i++)
	{
		route_->waypoint_[i]->ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
	}
}

void FollowTrajectoryAction::Start()
{
	OSCAction::Start();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))
	{
		// lateral motion controlled elsewhere
		// other action or controller already updated lateral dimension of object 
		// potentially longitudinal dimension could be updated separatelly - but skip that for now
		return;
	}

	traj_->Freeze();
	object_->pos_.SetTrajectory(traj_);
	
	object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
}

void FollowTrajectoryAction::End()
{
	OSCAction::End();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))

	// Disconnect trajectory
	object_->pos_.SetTrajectory(0);
}

void FollowTrajectoryAction::Step(double dt, double simTime)
{
	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))
	{
		// lateral motion controlled elsewhere
		// other action or controller already updated lateral dimension of object 
		// potentially longitudinal dimension could be updated separatelly - but skip that for now
		return;
	}

	time_ += timing_scale_ * dt;

	// Measure length of movement for odometer
	double x0 = object_->pos_.GetX();
	double y0 = object_->pos_.GetY();

	if (!traj_->closed_ && object_->pos_.GetTrajectoryS() > (traj_->shape_->length_ - DISTANCE_TOLERANCE))
	{
		// Reached end of trajectory
		// Calculate road coordinates from final inertia (X, Y) coordinates
		object_->pos_.XYZH2TrackPos(object_->pos_.GetX(), object_->pos_.GetY(), 0, object_->pos_.GetH(), false);
		
		End();
	}
	else
	{
		// Move along trajectory
		if (timing_domain_ == TimingDomain::NONE || 
			object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))  
		{
			object_->pos_.MoveTrajectoryDS(object_->speed_ * dt);
		}
		else if (timing_domain_ == TimingDomain::TIMING_RELATIVE)
		{
			double s_old = object_->pos_.GetTrajectoryS();
			object_->pos_.SetTrajectoryPosByTime(traj_, time_ + timing_offset_);
			object_->speed_ = (object_->pos_.GetTrajectoryS() - s_old) / dt;
		}
		else if (timing_domain_ == TimingDomain::TIMING_ABSOLUTE)
		{
			double s_old = object_->pos_.GetTrajectoryS();
			object_->pos_.SetTrajectoryPosByTime(traj_, simTime * timing_scale_ + timing_offset_);
			object_->speed_ = (object_->pos_.GetTrajectoryS() - s_old) / dt;
		}
	}

	object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
}

void FollowTrajectoryAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}
	if (traj_->shape_->type_ == roadmanager::Shape::ShapeType::CLOTHOID)
	{
		roadmanager::Clothoid* cl = (roadmanager::Clothoid*)traj_->shape_;
		cl->pos_.ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
	}
	else if (traj_->shape_->type_ == roadmanager::Shape::ShapeType::POLYLINE)
	{
		roadmanager::PolyLine* pl = (roadmanager::PolyLine*)traj_->shape_;
		for (size_t i = 0; i < pl->vertex_.size(); i++)
		{
			pl->vertex_[i]->pos_.ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
		}
	}

}

void LatLaneChangeAction::Start()
{
	OSCAction::Start();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))
	{
		// lateral motion controlled elsewhere
		return;
	}

	if (target_->type_ == Target::Type::ABSOLUTE)
	{
		target_lane_id_ = target_->value_;
	}
	else if (target_->type_ == Target::Type::RELATIVE)
	{
		// Find out number of lanes to move left (positive) or right (negative) relative own vehicle direction
		int left_direction = object_->pos_.GetHRelative() < M_PI_2 || object_->pos_.GetHRelative() > 3 * M_PI_2 ? 1 : -1;
		target_lane_id_ = ((TargetRelative*)target_)->object_->pos_.GetLaneId() + left_direction * target_->value_;
		
		if (target_lane_id_ == 0 || (SIGN(target_lane_id_) != SIGN(object_->pos_.GetLaneId())))
		{
			// Skip reference lane (id == 0)
			target_lane_id_ += left_direction * SIGN(target_->value_);
		}
	}

	target_t_ =
		SIGN(target_lane_id_) *
		object_->pos_.GetOpenDrive()->GetRoadById(object_->pos_.GetTrackId())->GetCenterOffset(object_->pos_.GetS(), target_lane_id_) +
		target_lane_offset_;

	// if dynamics dimension is rate, transform into distance
	if (transition_dynamics_.dimension_ == DynamicsDimension::RATE)
	{
		double lat_distance = object_->pos_.GetT() - target_t_;
		double rate = transition_dynamics_.target_value_;

		if (transition_dynamics_.shape_ == DynamicsShape::LINEAR)
		{
			// longitudinal distance = long_speed * time = long_speed * lat_dist / lat_speed 
			if (fabs(rate) > SMALL_NUMBER)
			{
				transition_dynamics_.target_value_ = fabs(object_->speed_ * lat_distance / rate);
			}
			else
			{
				// rate close to zero. Choose a random large distance.
				transition_dynamics_.target_value_ = 500;
			}
		}
		else if (transition_dynamics_.shape_ == DynamicsShape::SINUSOIDAL)
		{
			// Calculate corresponding distance with a magic formula
			transition_dynamics_.target_value_ = fabs(0.5 * lat_distance * M_PI *
				sqrt(object_->speed_ * object_->speed_ - rate * rate) / rate);
		}
	}

	t_ = start_t_ = object_->pos_.GetT();

	elapsed_ = 0;
}

void LatLaneChangeAction::Step(double dt, double simTime)
{
	double t_old = t_;
	double factor;
	double angle = 0;

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))
	{
		// lateral motion controlled elsewhere
		return;
	}

	if (transition_dynamics_.dimension_ == DynamicsDimension::TIME)
	{
		double dt_adjusted = dt;

		// Set a limit for lateral speed not to exceed longitudinal speed
		if (transition_dynamics_.target_value_ * object_->speed_ < fabs(target_t_ - start_t_))
		{
			dt_adjusted = dt * object_->speed_ * transition_dynamics_.target_value_ / fabs(target_t_ - start_t_);
		}
		elapsed_ += dt_adjusted;
	}
	else if (transition_dynamics_.dimension_ == DynamicsDimension::DISTANCE ||
			transition_dynamics_.dimension_ == DynamicsDimension::RATE)
	{
		elapsed_ += object_->speed_ * dt;
	}
	else
	{
		LOG("Unexpected timing type: %d", transition_dynamics_.dimension_);
	}

	factor = elapsed_ / transition_dynamics_.target_value_;
	t_ = transition_dynamics_.Evaluate(factor, start_t_, target_t_);

	if (object_->pos_.GetRoute())
	{
		// If on a route, stay in original lane
		int lane_id = object_->pos_.GetLaneId();
		object_->pos_.SetTrackPos(object_->pos_.GetTrackId(), object_->pos_.GetS(), t_);
		object_->pos_.ForceLaneId(lane_id);
	}
	else
	{
		object_->pos_.SetTrackPos(object_->pos_.GetTrackId(), object_->pos_.GetS(), t_);
	}
		

	if (factor > 1.0 || abs(t_ - target_t_) < SMALL_NUMBER || SIGN(t_ - start_t_) != SIGN(target_t_ - start_t_))
	{
		OSCAction::End();
		object_->pos_.SetHeadingRelativeRoadDirection(0);
	}
	else
	{
		if (object_->speed_ > SMALL_NUMBER)
		{
			angle = atan((t_ - t_old) / (object_->speed_ * dt));
			object_->pos_.SetHeadingRelativeRoadDirection(angle);
		}
	}

	object_->SetDirtyBits(Object::DirtyBit::LATERAL);
}

void LatLaneChangeAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}

	if (target_->type_ == Target::Type::RELATIVE)
	{
		if (((TargetRelative*)target_)->object_ == obj1)
		{
			((TargetRelative*)target_)->object_ = obj2;
		}
	}
}

void LatLaneOffsetAction::Start()
{
	OSCAction::Start();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))
	{
		// lateral motion controlled elsewhere
		return;
	}

	start_lane_offset_ = object_->pos_.GetOffset();
}

void LatLaneOffsetAction::Step(double dt, double simTime)
{
	double factor, lane_offset;
	double angle = 0;
	double old_lane_offset = object_->pos_.GetOffset();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LATERAL))
	{
		// lateral motion controlled elsewhere
		return;
	}

	elapsed_ += dt;
	factor = elapsed_ / dynamics_.duration_;

	lane_offset = dynamics_.transition_.Evaluate(factor, start_lane_offset_, target_->value_);

	object_->pos_.SetLanePos(object_->pos_.GetTrackId(), object_->pos_.GetLaneId(), object_->pos_.GetS(), lane_offset);

	if (object_->speed_ > SMALL_NUMBER)
	{
		angle = atan((lane_offset - old_lane_offset) / (object_->speed_ * dt));
	}

	if (factor > 1.0)
	{
		OSCAction::End();
		angle = 0;
	}

	object_->pos_.SetHeadingRelativeRoadDirection(angle);

	object_->SetDirtyBits(Object::DirtyBit::LATERAL);
}

void LatLaneOffsetAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}

	if (target_->type_ == Target::Type::RELATIVE)
	{
		if (((TargetRelative*)target_)->object_ = obj1)
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

void LongSpeedAction::Start()
{
	OSCAction::Start();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LONGITUDINAL))
	{
		// longitudinal motion controlled elsewhere
		return;
	}

	if (transition_dynamics_.shape_ == DynamicsShape::STEP)
	{
		object_->SetSpeed(target_->GetValue());
		if (!(target_->type_ == Target::TargetType::RELATIVE && ((TargetRelative*)target_)->continuous_ == true))
		{
			OSCAction::End();
		}
	}
	else
	{
		start_speed_ = object_->speed_;
	}
 }

void LongSpeedAction::Step(double dt, double simTime)
{
	double factor = 0.0;
	double new_speed = 0;
	bool target_speed_reached = false;

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LONGITUDINAL))
	{
		// longitudinal motion controlled elsewhere
		return;
	}

	if (transition_dynamics_.dimension_ == DynamicsDimension::RATE)
	{
		elapsed_ += dt;
		double speed_diff = target_->GetValue() - object_->speed_;
		new_speed = object_->speed_ + SIGN(speed_diff) * fabs(transition_dynamics_.target_value_) * dt;

		// Check if speed changed passed target value
		if ((object_->speed_ > target_->GetValue() && new_speed < target_->GetValue()) ||
			(object_->speed_ < target_->GetValue() && new_speed > target_->GetValue()))
		{
			new_speed = target_->GetValue();
			target_speed_reached = true;
		}
	}
	else if (transition_dynamics_.dimension_ == DynamicsDimension::TIME)
	{
		elapsed_ += dt;
		factor = elapsed_ / (transition_dynamics_.target_value_);

		if(factor > 1.0)
		{
			new_speed = target_->GetValue();
			target_speed_reached = true;
		}
		else
		{
			new_speed = transition_dynamics_.Evaluate(factor, start_speed_, target_->GetValue());
		}
	}
	else
	{
		LOG("Timing type %d not supported yet", transition_dynamics_.dimension_);
		new_speed = target_->GetValue();
		OSCAction::Stop();

		return;
	}

	if (target_speed_reached && !(target_->type_ == Target::TargetType::RELATIVE && ((TargetRelative*)target_)->continuous_ == true))
	{
		OSCAction::End();
	}

	object_->speed_ = new_speed;
}

void LongSpeedAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
	if (object_ == obj1)
	{
		object_ = obj2;
	}

	if (target_->type_ == Target::TargetType::RELATIVE)
	{
		if (((TargetRelative*)target_)->object_ == obj1)
		{
			((TargetRelative*)target_)->object_ = obj2;
		}
	}
}

void LongDistanceAction::Start()
{
	if (target_object_ == 0)
	{
		LOG("Can't trig without set target object ");
		return;
	}

	OSCAction::Start();
}

void LongDistanceAction::Step(double dt, double simTime)
{
	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LONGITUDINAL))
	{
		// longitudinal motion controlled elsewhere
		return;
	}

	// Find out current distance
	double x, y;
	double distance = object_->pos_.getRelativeDistance(target_object_->pos_, x, y);
	double speed_diff = object_->speed_ - target_object_->speed_;
	double acc;
	double spring_constant = 4;
	double dc;
	double requested_dist;

	// Just interested in the x-axis component of the distance
	distance = x;

	if (dist_type_ == DistType::DISTANCE)
	{
		requested_dist = distance_;
	}
	if (dist_type_ == DistType::TIME_GAP)
	{
		// Convert requested time gap (seconds) to distance (m)
		requested_dist = object_->speed_ * distance_;
	}

	double distance_diff = distance - requested_dist;

	if (dynamics_.none_ == true)
	{
		// Set position according to distance and copy speed of target vehicle
		object_->pos_.MoveAlongS(distance_diff);
		object_->speed_ = target_object_->speed_;

		object_->SetDirtyBits(Object::DirtyBit::LONGITUDINAL);
	}
	else
	{
		// Apply damped spring model with critical/optimal damping factor		
		dc = 2 * sqrt(spring_constant);
		acc = distance_diff * spring_constant - speed_diff * dc;
		if (acc > dynamics_.max_acceleration_)
		{
			acc = dynamics_.max_acceleration_;
		}
		else if (acc < -dynamics_.max_deceleration_)
		{
			acc = -dynamics_.max_deceleration_;
		}

		object_->speed_ += acc * dt;

		if (object_->speed_ > dynamics_.max_speed_)
		{
			object_->speed_ = dynamics_.max_speed_;
		}
		else if (object_->speed_ < -dynamics_.max_speed_)
		{
			object_->speed_ = -dynamics_.max_speed_;
		}
	}

	//	LOG("Dist %.2f diff %.2f acc %.2f speed %.2f", distance, distance_diff, acc, object_->speed_);
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

void TeleportAction::Start()
{
	OSCAction::Start();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LONGITUDINAL | Controller::Domain::CTRL_LONGITUDINAL))
	{
		// motion controlled elsewhere
		return;
	}

	roadmanager::Position tmpPos;

	if (position_->GetRelativePosition() == &object_->pos_)
	{
		// Special case: Relative to itself - need to make a copy before reseting
		tmpPos = object_->pos_;
		position_->SetRelativePosition(&tmpPos, position_->GetType());
	}

	object_->pos_.CopyRMPos(position_);

	// Resolve any relative positions
	object_->pos_.ReleaseRelation();

	if (object_->pos_.GetType() == roadmanager::Position::PositionType::ROUTE)
	{
		object_->pos_.CalcRoutePosition();
	}

	LOG("%s pos: ", object_->name_.c_str());
	object_->pos_.Print();
	object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
	object_->reset_ = true;
}

void TeleportAction::Step(double dt, double simTime)
{
	(void)dt;
	(void)simTime;

	OSCAction::Stop();
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

void SynchronizeAction::Start()
{
	OSCAction::Start();

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LONGITUDINAL))
	{
		// longitudinal motion controlled elsewhere
		return;
	}
}

void SynchronizeAction::Step(double dt, double simTime)
{
	(void)dt;

	if (object_->GetControllerMode() == Controller::Mode::MODE_OVERRIDE &&
		object_->IsControllerActiveOnDomains(Controller::Domain::CTRL_LONGITUDINAL))
	{
		// longitudinal motion controlled elsewhere
		return;
	}

	// Calculate distance along road/route
	double masterDist, dist;
	roadmanager::PositionDiff diff;

	if (!master_object_->pos_.Delta(*target_position_master_, diff))
	{
		LOG("No road network path between master vehicle and master target pos");
		return;
	}
	masterDist = diff.ds;

	if (!object_->pos_.Delta(*target_position_, diff))
	{
		LOG("No road network path between master vehicle and master target pos");
		return;
	}
	dist = diff.ds;

	double masterTimeToDest = LARGE_NUMBER;

	if (master_object_->speed_ > SMALL_NUMBER)
	{
		masterTimeToDest = masterDist / master_object_->speed_;
	}

	// Done when either of the vehicles reaches the destination
	if (dist < DISTANCE_TOLERANCE || (masterTimeToDest) < SMALL_NUMBER)
	{
		OSCAction::End();
	}
	else
	{
		double average_speed = dist / masterTimeToDest;
		double acc = 0;

		//LOG("dist: %.2f time: %.2f final speed: %.2f", dist, masterTimeToDest, final_speed_->GetValue());

		if (final_speed_)
		{
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
					object_->speed_ = 0;
					mode_ = SynchMode::MODE_WAITING;  // wait for master to move
					PrintStatus("Waiting");
				}

				return;
			}
			else if (mode_ == SynchMode::MODE_STOPPED)
			{
				if (masterTimeToDest < 2 * dist / final_speed_->GetValue())
				{
					// Time to move again after the stop
					mode_ = SynchMode::MODE_LINEAR;
					PrintStatus("Restart");
				}
				else
				{
					// Stay still
					object_->speed_ = 0;
					return;
				}
			}
			
			if (mode_ == SynchMode::MODE_LINEAR)
			{
				object_->speed_ = MAX(0, CalcSpeedForLinearProfile(MAX(0, final_speed_->GetValue()), masterTimeToDest, dist));
				return;
			}
			else if (masterTimeToDest < LARGE_NUMBER) 
			{
				// Check if case 1, i.e. on a straight speed profile line
				double v0_onLine = 2 * dist / masterTimeToDest - final_speed_->GetValue();

				if (fabs(object_->speed_ - v0_onLine) < 0.1)
				{
					// Switch to linear mode (constant acc) to reach final destination and speed
					mode_ = SynchMode::MODE_LINEAR;
					PrintStatus("Passed apex");

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
				PrintStatus("Non-linear");
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
				PrintStatus("Reached apex");

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
					PrintStatus("Stop immediately");
				}

				if (v0 + acc * dt < 0)
				{
					// Reached to a stop
					object_->speed_ = 0;
					mode_ = SynchMode::MODE_STOPPED;
					PrintStatus("Stopped");

					return;
				}
			}
		}
		else
		{
			// No final speed specified. Calculate it based on current speed and available time
			double final_speed_ = 2 * average_speed - object_->speed_;
			acc = (final_speed_ - object_->speed_) / masterTimeToDest;
		}

		object_->speed_ += acc * dt;
	}
}

void VisibilityAction::Start()
{
	OSCAction::Start();
	object_->SetVisibilityMask(
		(graphics_ ? Object::Visibility::GRAPHICS : 0) |
		(traffic_ ? Object::Visibility::TRAFFIC : 0) |
		(sensors_ ? Object::Visibility::SENSORS : 0)
	);
}

void VisibilityAction::Step(double dt, double simTime)
{
	(void)dt;
	(void)simTime;

	OSCAction::Stop();
}
