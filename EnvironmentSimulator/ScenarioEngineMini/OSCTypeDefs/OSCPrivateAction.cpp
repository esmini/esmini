#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include "OSCPrivateAction.hpp"

#define SIGN(x) (x < 0 ? -1 : 1)


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

	if (target_->ABSOLUTE)
	{
		target_lane_id_ = target_->value_;
	}
	else if (target_->RELATIVE)
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
		object_->pos_.SetHeadingRelative(atan((t - t_old) / (object_->speed_ * dt)));

		if (factor > 1.0)
		{
			state_ = OSCAction::State::DONE;
		}
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

	double old_lane_offset = object_->pos_.GetOffset();

	elapsed_ += dt;
	factor = elapsed_ / dynamics_.duration_;

	lane_offset = dynamics_.transition_.Evaluate(factor, start_lane_offset_, target_->value_);
	
	object_->pos_.SetLanePos(object_->pos_.GetTrackId(), object_->pos_.GetLaneId(), object_->pos_.GetS(), lane_offset);

	object_->pos_.SetHeadingRelative(atan((lane_offset - old_lane_offset) / (object_->speed_ * dt)));

	if (factor > 1.0)
	{
		object_->pos_.SetHeadingRelative(0.0);
		state_ = State::DONE;
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

void LongSpeedAction::Step(double dt)
{
	double factor = 0.0;
	double target_speed = 0;
	double new_speed = object_->speed_;

	if (dynamics_.timing_type_ == Timing::RATE)
	{
		elapsed_ += dt;
		new_speed += dynamics_.timing_target_value_ * dt;

		if ((dynamics_.timing_target_value_ < 0 && new_speed < target_->GetValue()) ||
			(dynamics_.timing_target_value_ > 0 && new_speed > target_->GetValue()))
		{
			if (new_speed < target_->GetValue())
			{
				new_speed = target_->GetValue();
				state_ = State::DONE;
			}
		}
	}
	else if (dynamics_.timing_type_ == Timing::TIME)
	{
		elapsed_ += dt;
		factor = elapsed_ / (dynamics_.timing_target_value_);

		if(factor > 1.0)
		{
			new_speed = target_->GetValue();
			if (target_->type_ == Target::Type::RELATIVE && ((TargetRelative*)target_)->continuous_ != true)
			{
				state_ = State::DONE;
			}
		}
		else
		{
			new_speed = dynamics_.transition_.Evaluate(factor, start_speed_, target_->GetValue());
		}
	}
	else
	{
		LOG("Timing type %d not supported yet", dynamics_.timing_type_);
		state_ = State::DONE;
		return;
	}

	object_->speed_ = new_speed;
}

void MeetingRelativeAction::Step(double dt)
{
	// Calculate straight distance, not along road/route. To be improved.
	double pivotDist = object_->pos_.getRelativeDistance(*own_target_position_);
	double relativeDist = relative_object_->pos_.getRelativeDistance(*relative_target_position_);

	double targetTimeToDest = INFINITY;

	if (relative_object_->speed_ > SMALL_NUMBER)
	{
		targetTimeToDest = relativeDist / relative_object_->speed_;
	}

	object_->speed_ = pivotDist / (targetTimeToDest + offsetTime_);

	if (relativeDist < DISTANCE_TOLERANCE)
	{
		state_ = State::DONE;
	}
}
