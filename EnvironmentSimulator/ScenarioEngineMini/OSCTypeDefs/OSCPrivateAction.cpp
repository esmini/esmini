#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include "OSCPrivateAction.hpp"


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

void LatLaneChangeAction::Step(double dt)
{
	double lane_offset;
	if (dynamics_.timing_type_ == Timing::TIME)
	{
		elapsed_ += dt;
		lane_offset = dynamics_.transition_.Evaluate(elapsed_ / dynamics_.timing_target_value_, 0, dynamics_.timing_target_value_);
	}
	else
	{
		LOG("Timing type %d not supported yet", dynamics_.timing_type_);
	}

	LOG("Step %s elapsed: %.2f target: %.2f current: %.2f", object_->name_.c_str(),
		elapsed_, dynamics_.timing_target_value_, lane_offset);

}

void LatLaneOffsetAction::Trig()
{
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

void LongSpeedAction::Step(double dt)
{
	double factor = 0.0;
	double new_speed = object_->speed_;

	if (dynamics_.timing_type_ == Timing::RATE)
	{
		elapsed_ += dt;
		new_speed += dynamics_.timing_target_value_ * dt;

		if (dynamics_.timing_target_value_ < 0)
		{
			if (new_speed < target_->value_)
			{
				new_speed = target_->value_;
				state_ = State::DONE;
			}
		}
		else
		{
			if (new_speed > target_->value_)
			{
				new_speed = target_->value_;
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
			new_speed = target_->value_;
			state_ =  State::DONE;
		}
		else
		{
			new_speed = dynamics_.transition_.Evaluate(factor, start_speed_, target_->value_);
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
