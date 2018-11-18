#pragma once

#include "OSCCondition.hpp"


std::string Rule2Str(Rule rule)
{
	if (rule == Rule::EQUAL_TO)
	{
		return "=";
	}
	else if (rule == Rule::GREATER_THAN)
	{
		return ">";
	}
	else if (rule == Rule::LESS_THAN)
	{
		return "<";
	}
	else
	{
		LOG("Undefined Rule: %d", rule);
	}
	return "unknown";
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

bool OSCCondition::CheckEdge(double a, double b, OSCCondition::ConditionEdge edge)
{
	if (edge == OSCCondition::ConditionEdge::ANY)
	{
		return true;
	}
	else if (evaluated_ && edge == OSCCondition::ConditionEdge::FALLING)
	{
		if (a < b)
		{
			return true;
		}
	}
	else if (evaluated_ && edge == OSCCondition::ConditionEdge::RISING)
	{
		if (a > b)
		{
			return true;
		}
	}
	else if(evaluated_)
	{
		LOG("Invalid edge: %d", edge);
	}

	return false;
}

std::string Edge2Str(OSCCondition::ConditionEdge edge)
{
	if (edge == OSCCondition::ConditionEdge::FALLING)
	{
		return "Falling";
	}
	else if (edge == OSCCondition::ConditionEdge::RISING)
	{
		return "Rising";
	}
	else if (edge == OSCCondition::ConditionEdge::ANY)
	{
		return "Any";
	}

	return "Unknown edge";
}

bool EvalDone(bool trig, TrigByEntity::TriggeringEntitiesRule rule)
{
	if (trig == false && rule == TrigByEntity::TriggeringEntitiesRule::ALL)
	{
		return true;  // One false is enough
	}
	else if (trig == true && rule == TrigByEntity::TriggeringEntitiesRule::ANY)
	{
		return true;  // One true is enough
	}

	return false;
}

bool TrigByState::Evaluate(Act *act, double sim_time)
{
	evaluated_ = true;

	return false;
}

bool TrigAtStart::Evaluate(Act *act, double sim_time)
{
	evaluated_ = true;

	return false;
}

bool TrigAfterTermination::Evaluate(Act *act, double sim_time)
{
	evaluated_ = true;

	return false;
}

bool TrigByValue::Evaluate(Act *act, double sim_time)
{

	evaluated_ = true;

	return false;
}

bool TrigBySimulationTime::Evaluate(Act *act, double sim_time)
{
	evaluated_ = true;

	if (sim_time >= value_)
	{
		LOG("Trigged %s (sim_time: %.2f >= condition: %.2f", name_.c_str(), sim_time, value_);
		return true;
	}

	return false;
}

bool TrigByTimeHeadway::Evaluate(Act *act, double sim_time)
{
	bool trig = false;
	double rel_dist, hwt;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		rel_dist = triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(object_->pos_);

		if (object_->speed_ < SMALL_NUMBER)
		{
			hwt = INFINITY;
		}
		else
		{
			hwt = fabs(rel_dist / object_->speed_);
		}

		trig = EvaluateRule(hwt, value_, rule_) && CheckEdge(hwt, headway_time_last_value_, edge_);

		headway_time_last_value_ = hwt;

		if (EvalDone(trig, triggering_entity_rule_))
		{
			break;
		}
	}

	if (trig)
	{
		LOG("Trigged %s hwt: %.2f %s %.2f, %s", name_.c_str(), hwt, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	}

	evaluated_ = true;

	return trig;
}
