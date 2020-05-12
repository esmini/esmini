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

void FollowRouteAction::Start()
{
	if (object_->control_ == Object::Control::EXTERNAL ||
		object_->control_ == Object::Control::HYBRID_EXTERNAL)
	{
		// motion control handed over 
		return;
	}

	object_->pos_.SetRoute(route_);

	OSCAction::Start();
}

void FollowTrajectoryAction::Start()
{
	if (object_->control_ == Object::Control::EXTERNAL ||
		object_->control_ == Object::Control::HYBRID_EXTERNAL)
	{
		// motion control handed over 
		return;
	}

	traj_->Freeze();
	object_->pos_.SetTrajectory(traj_);
	
	OSCAction::Start();
}

void FollowTrajectoryAction::Step(double dt, double simTime)
{
	time_ += timing_scale_ * dt;

	if (!traj_->closed_ && object_->pos_.GetTrajectoryS() > (traj_->shape_->length_ - DISTANCE_TOLERANCE))
	{
		// Disconnect trajectory
		object_->pos_.SetTrajectory(0);

		// Calculate road coordinates from final inertia (X, Y) coordinates
		roadmanager::Position* p = &object_->pos_;
		p->XYZH2TrackPos(object_->pos_.GetX(), object_->pos_.GetY(), 0, object_->pos_.GetHRoad(), true);

		OSCAction::End();
	}
	else
	{
		// Move along trajectory
		if (timing_domain_ == TimingDomain::NONE)
		{
			object_->pos_.MoveTrajectoryDS(object_->speed_ * dt);
		}
		else if (timing_domain_ == TimingDomain::TIMING_RELATIVE)
		{
			object_->pos_.SetTrajectoryPosByTime(traj_, time_ + timing_offset_);
		}
		else if (timing_domain_ == TimingDomain::TIMING_ABSOLUTE)
		{
			object_->pos_.SetTrajectoryPosByTime(traj_, simTime * timing_scale_ + timing_offset_);
		}
	}
}

void LatLaneChangeAction::Start()
{
	if (object_->control_ == Object::Control::EXTERNAL ||
		object_->control_ == Object::Control::HYBRID_EXTERNAL)
	{
		// motion control handed over 
		return;
	}
	OSCAction::Start();

	if (target_->type_ == Target::Type::ABSOLUTE)
	{
		target_lane_id_ = target_->value_;
	}
	else if (target_->type_ == Target::Type::RELATIVE)
	{
		target_lane_id_ = ((TargetRelative*)target_)->object_->pos_.GetLaneId() + target_->value_;
	}
	start_t_ = object_->pos_.GetT();
}

void LatLaneChangeAction::Step(double dt, double simTime)
{
	double target_t;
	double t, t_old;
	double factor;
	double angle = 0;

	target_t =
		SIGN(target_lane_id_) *
		object_->pos_.GetOpenDrive()->GetRoadById(object_->pos_.GetTrackId())->GetCenterOffset(object_->pos_.GetS(), target_lane_id_) +
		target_lane_offset_;

	if (transition_dynamics_.dimension_ == DynamicsDimension::TIME || transition_dynamics_.dimension_ == DynamicsDimension::DISTANCE)
	{
		if (transition_dynamics_.dimension_ == DynamicsDimension::TIME)
		{
			double dt_adjusted = dt;

			// Set a limit for lateral speed not to exceed longitudinal speed
			if (transition_dynamics_.target_value_ * object_->speed_ < fabs(target_t - start_t_))
			{
				dt_adjusted = dt * object_->speed_ * transition_dynamics_.target_value_ / fabs(target_t - start_t_);
			}
			elapsed_ += dt_adjusted;
		}
		else if (transition_dynamics_.dimension_ == DynamicsDimension::DISTANCE)
		{
			elapsed_ += object_->speed_ * dt;
		}
		else
		{
			LOG("Unexpected timing type: %d", transition_dynamics_.dimension_);
		}

		factor = elapsed_ / transition_dynamics_.target_value_;
		t_old = object_->pos_.GetT();

		t = transition_dynamics_.Evaluate(factor, start_t_, target_t);
		
		if (object_->pos_.GetRoute())
		{
			// If on a route, stay in original lane
			int lane_id = object_->pos_.GetLaneId();
			object_->pos_.SetTrackPos(object_->pos_.GetTrackId(), object_->pos_.GetS(), t);
			object_->pos_.ForceLaneId(lane_id);
		}
		else
		{
			object_->pos_.SetTrackPos(object_->pos_.GetTrackId(), object_->pos_.GetS(), t);
		}
		

		if (factor > 1.0)
		{
			OSCAction::End();
			object_->pos_.SetHeadingRelativeRoadDirection(0);
		}
		else
		{
			if (object_->speed_ > SMALL_NUMBER)
			{
				angle = atan((t - t_old) / (object_->speed_ * dt));
				object_->pos_.SetHeadingRelativeRoadDirection(angle);
			}
		}
	}
	else
	{
		LOG("Timing type %d not supported yet", transition_dynamics_.dimension_);
	}
}

void LatLaneOffsetAction::Start()
{
	if (object_->control_ == Object::Control::EXTERNAL ||
		object_->control_ == Object::Control::HYBRID_EXTERNAL)
	{
		// motion control handed over 
		return;
	}

	OSCAction::Start();
	start_lane_offset_ = object_->pos_.GetOffset();
}

void LatLaneOffsetAction::Step(double dt, double simTime)
{
	double factor, lane_offset;
	double angle = 0;
	double old_lane_offset = object_->pos_.GetOffset();

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
	if (object_->control_ == Object::Control::EXTERNAL ||
		object_->control_ == Object::Control::HYBRID_EXTERNAL)
	{
		// motion control handed over 
		return;
	}

	OSCAction::Start();

	start_speed_ = object_->speed_;
}

void LongSpeedAction::Step(double dt, double simTime)
{
	double factor = 0.0;
	double new_speed = 0;
	bool target_speed_reached = false;

	if (transition_dynamics_.shape_ == DynamicsShape::STEP)
	{
		new_speed = target_->GetValue();
		target_speed_reached = true;
	}
	else if (transition_dynamics_.dimension_ == DynamicsDimension::RATE)
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

	if (target_speed_reached && !(target_->type_ == Target::Type::RELATIVE && ((TargetRelative*)target_)->continuous_ == true))
	{
		OSCAction::End();
	}

	object_->speed_ = new_speed;
}

void LongDistanceAction::Step(double dt, double simTime)
{
	// Find out current distance
	double x, y;
	double distance = object_->pos_.getRelativeDistance(target_object_->pos_, x, y);
	double speed_diff = object_->speed_ - target_object_->speed_;
	double acc;
	double spring_constant = 4;
	double dc;
	double requested_dist;

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

void LongDistanceAction::Start()
{
	if (target_object_ == 0)
	{
		LOG("Can't trig without set target object ");
		return;
	}

	OSCAction::Start();
}

void PositionAction::Start()
{
	// Evaluate position, potentially dependent on other entities
	if (position_->type_ == OSCPosition::PositionType::RELATIVE_LANE)
	{
		OSCPositionRelativeLane* osc_pos = (OSCPositionRelativeLane*)position_;
		roadmanager::Position* pos = position_->GetRMPos();
		pos->CopyRMPos(&osc_pos->object_->pos_);
		pos->SetLanePos(pos->GetTrackId(), pos->GetLaneId() + osc_pos->dLane_, pos->GetS() + osc_pos->ds_, pos->GetOffset() + osc_pos->offset_);
	}
	OSCAction::Start();
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

void SynchronizeAction::Step(double dt, double simTime)
{
	(void)dt;

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
