#pragma once

#include "OSCCondition.hpp"

bool TrigByState::Evaluate(Act *act, double sim_time)
{

	return false;
}

bool TrigAtStart::Evaluate(Act *act, double sim_time)
{

	return false;
}

bool TrigAfterTermination::Evaluate(Act *act, double sim_time)
{

	return false;
}

bool TrigByValue::Evaluate(Act *act, double sim_time)
{

	return false;
}

bool TrigBySimulationTime::Evaluate(Act *act, double sim_time)
{
	if (sim_time >= value_)
	{
		LOG("Trigged %s (sim_time: %.2f >= condition: %.2f", name_.c_str(), sim_time, value_);
		act->active = true;
		return true;
	}

	return false;
}

bool EvaluateRule(double a, double b, Rule rule)
{
	if (rule == Rule::EQUAL_TO)
	{
		return a == b;
	}
	else if (rule == Rule::GREATER_THAN)
	{
		return a > b;
	}
	else if (rule == Rule::LESS_THAN)
	{
		return a < b;
	}
	else
	{
		LOG("Undefined Rule: %d", rule);
	}
	return false;
}

bool TrigByTimeHeadway::Evaluate(Act *act, double sim_time)
{
	bool trig = false;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		double rel_dist = triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(object_->pos_);
		double hwt;

		if (object_->speed_ < SMALL_NUMBER)
		{
			hwt = INFINITY;
		}
		else
		{
			hwt = fabs(rel_dist / object_->speed_);
		}
		
		trig = EvaluateRule(hwt, value_, rule_);
		if (trig == false && triggering_entity_rule_ == TriggeringEntitiesRule::ALL)
		{
			return false;
		}
		else if (trig == true && triggering_entity_rule_ == TriggeringEntitiesRule::ANY)
		{
			return true;
		}
		// else just continue
	}

	return trig;
}
