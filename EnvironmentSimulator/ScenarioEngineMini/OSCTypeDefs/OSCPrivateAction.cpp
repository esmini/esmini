#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include "OSCPrivateAction.hpp"

#define SIGN(x) (x < 0 ? -1 : 1)
#define MAX(x, y) (y > x ? y : x)
#define MIN(x, y) (y < x ? y : x)


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

void LatLaneChangeAction::Trig()
{
	if (object_->extern_control_)
	{
		// motion control handed over 
		return;
	}

	OSCAction::Trig();

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

void LatLaneChangeAction::Step(double dt)
{
	double target_t;
	double t, t_old;
	double factor;
	double angle = 0;

	target_t =
		SIGN(target_lane_id_) *
		object_->pos_.GetOpenDrive()->GetRoadById(object_->pos_.GetTrackId())->GetCenterOffset(object_->pos_.GetS(), target_lane_id_) +
		target_lane_offset_;

	if (dynamics_.timing_type_ == Timing::TIME)
	{
		elapsed_ += dt;
		factor = elapsed_ / dynamics_.timing_target_value_;
		t_old = object_->pos_.GetT();

		t = dynamics_.transition_.Evaluate(factor, start_t_, target_t);
		
		object_->pos_.SetTrackPos(object_->pos_.GetTrackId(), object_->pos_.GetS(), t);
		

		if (object_->speed_ > SMALL_NUMBER)
		{
			angle = atan((t - t_old) / (object_->speed_ * dt));
		}

		if (factor > 1.0)
		{
			OSCAction::Done();
			angle = 0;
		}

		object_->pos_.SetHeadingRelative(angle);
	}
	else
	{
		LOG("Timing type %d not supported yet", dynamics_.timing_type_);
	}
}

void LatLaneOffsetAction::Trig()
{
	if (object_->extern_control_)
	{
		// motion control handed over 
		return;
	}
	OSCAction::Trig();
	start_lane_offset_ = object_->pos_.GetOffset();
}

void LatLaneOffsetAction::Step(double dt)
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
		OSCAction::Done();
		angle = 0;
	}

	object_->pos_.SetHeadingRelative(angle);
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

void LongSpeedAction::Trig()
{
	if (object_->extern_control_)
	{
		// motion control handed over 
		return;
	}
	OSCAction::Trig();

	start_speed_ = object_->speed_;
}

void LongSpeedAction::Step(double dt)
{
	double factor = 0.0;
	double target_speed = 0;
	double new_speed = 0;
	bool target_speed_reached = false;

	if (dynamics_.transition_.shape_ == DynamicsShape::STEP)
	{
		new_speed = target_->GetValue();
		target_speed_reached = true;
	}
	else if (dynamics_.timing_type_ == Timing::RATE)
	{
		elapsed_ += dt;
		double speed_diff = target_->GetValue() - object_->speed_;
		new_speed = object_->speed_ + SIGN(speed_diff) * fabs(dynamics_.timing_target_value_) * dt;

		// Check if speed changed passed target value
		if ((object_->speed_ > target_->GetValue() && new_speed < target_->GetValue()) ||
			(object_->speed_ < target_->GetValue() && new_speed > target_->GetValue()))
		{
			new_speed = target_->GetValue();
			target_speed_reached = true;
		}
	}
	else if (dynamics_.timing_type_ == Timing::TIME)
	{
		elapsed_ += dt;
		factor = elapsed_ / (dynamics_.timing_target_value_);

		if(factor > 1.0)
		{
			new_speed = target_->GetValue();
			target_speed_reached = true;
		}
		else
		{
			new_speed = dynamics_.transition_.Evaluate(factor, start_speed_, target_->GetValue());
		}
	}
	else
	{
		LOG("Timing type %d not supported yet", dynamics_.timing_type_);
		new_speed = target_->GetValue();
		OSCAction::Done();

		return;
	}

	if (target_speed_reached && !(target_->type_ == Target::Type::RELATIVE && ((TargetRelative*)target_)->continuous_ == true))
	{
		OSCAction::Done();
	}

	object_->speed_ = new_speed;
}

void LongDistanceAction::Step(double dt)
{
	// Find out current distance
	double distance = object_->pos_.getRelativeDistance(target_object_->pos_);
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

//	LOG("Dist %.2f diff %.2f acc %.2f speed %.2f", distance, distance_diff, acc, object_->speed_);
}

void LongDistanceAction::Trig()
{
	if (target_object_ == 0)
	{
		LOG("Can't trig without set target object ");
		return;
	}

	OSCAction::Trig();
}

void MeetingRelativeAction::Step(double dt)
{
	// Calculate straight distance, not along road/route. To be improved.
	double pivotDist = object_->pos_.getRelativeDistance(*own_target_position_);
	double targetTimeToDest = INFINITY;
	double relativeDist = MAX(0, relative_object_->pos_.getRelativeDistance(*relative_target_position_));

	if (relative_object_->speed_ > SMALL_NUMBER)
	{
		targetTimeToDest = relativeDist / relative_object_->speed_;
	}

	// Done when either of the vehicles reaches the destination
	if (relativeDist < DISTANCE_TOLERANCE || (targetTimeToDest + offsetTime_) < SMALL_NUMBER)
	{
		OSCAction::Done();
	}
	else
	{
		object_->speed_ = pivotDist / (targetTimeToDest + offsetTime_);
	}
}
