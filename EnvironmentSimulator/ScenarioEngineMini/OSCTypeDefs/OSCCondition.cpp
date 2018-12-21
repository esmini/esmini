#pragma once

#include "OSCCondition.hpp"
#include "Story.hpp"

using namespace scenarioengine;

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

bool TrigByState::Evaluate(Story *story, double sim_time)
{
	evaluated_ = true;


	return false;
}

bool TrigAtStart::Evaluate(Story *story, double sim_time)
{
	evaluated_ = true;

	if (element_type_ == StoryElementType::SCENE)
	{
		return true;
	}
	else if (element_type_ == StoryElementType::ACTION)
	{
		OSCAction *action = story->FindActionByName(element_name_);
		if ( action && action->state_ == OSCAction::State::ACTIVE)
		{
			LOG("Trigged");
			return true;
		}
	}
	else if (element_type_ == StoryElementType::ACT)
	{
		Act *act = story->FindActByName(element_name_);
		if (act && act->active_)
		{
			LOG("Trigged");
			return true;
		}
	}
	else
	{
		LOG("Story element type %d not supported yet", element_type_);
	}

	return false;
}

bool TrigAfterTermination::Evaluate(Story *story, double sim_time)
{
	evaluated_ = true;
	bool trig = false;

	if (element_type_ == StoryElementType::SCENE)
	{
		trig = true;
	}
	else if (element_type_ == StoryElementType::ACTION)
	{
		OSCAction *action = story->FindActionByName(element_name_);
		if (action && action->state_ == OSCAction::State::DONE)
		{
			trig = true;
		}
	}
	else
	{
		LOG("Story element type %d not supported yet", element_type_);
	}

	if (trig)
	{
		LOG("Trigged");
	}
	return trig;
}

bool TrigByValue::Evaluate(Story *story, double sim_time)
{

	evaluated_ = true;

	return false;
}

bool TrigBySimulationTime::Evaluate(Story *story, double sim_time)
{
	evaluated_ = true;

	if (sim_time >= value_)
	{
		LOG("Trigged %s (sim_time: %.2f >= condition: %.2f", name_.c_str(), sim_time, value_);
		return true;
	}

	return false;
}

bool TrigByTimeHeadway::Evaluate(Story *story, double sim_time)
{
	bool trig = false;
	double rel_dist, hwt;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		double x, y;
		rel_dist = triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(object_->pos_, x, y);

		// Headway time not defined for cases:
		//  - when target object is behind 
		//  - when object is still or going reverse 
		if (rel_dist < 0 || object_->speed_ < SMALL_NUMBER)
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

	//LOG("Trig? %s hwt: %.2f %s %.2f, %s (dist: %.2f)", name_.c_str(), hwt, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str(), rel_dist);
	if (trig)
	{
		LOG("Trigged %s hwt: %.2f %s %.2f, %s", name_.c_str(), hwt, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	}

	evaluated_ = true;

	return trig;
}

bool TrigByReachPosition::Evaluate(Story *story, double sim_time)
{
	bool trig = false;
	double x, y;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (fabs(triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(position_, x, y)) < tolerance_)
		{
			trig = true;
		}

		if (EvalDone(trig, triggering_entity_rule_))
		{
			break;
		}
	}

	evaluated_ = true;

	return trig;
}

bool TrigByRelativeDistance::Evaluate(Story *story, double sim_time)
{
	bool trig = false;
	double rel_dist, rel_intertial_dist, x, y;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		rel_intertial_dist = triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(object_->pos_, x, y);

		if (type_ == RelativeDistanceType::LONGITUDINAL)
		{
			rel_dist = fabs(y);
		}
		else if (type_ == RelativeDistanceType::LATERAL)
		{
			rel_dist = fabs(x);
		}
		else if (type_ == RelativeDistanceType::INTERIAL)
		{
			rel_dist = fabs(rel_intertial_dist);
		}
		else
		{
			LOG("Unsupported RelativeDistance type: %d", type_);
		}

		trig = EvaluateRule(rel_dist, value_, rule_) && CheckEdge(rel_dist, relative_dist_last_value_, edge_);

		relative_dist_last_value_ = rel_dist;

		if (EvalDone(trig, triggering_entity_rule_))
		{
			break;
		}
	}

	//LOG("RelDist Trig? %s rel_dist: %.2f %s %.2f, %s", name_.c_str(), rel_dist, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	if (trig)
	{
		LOG("Trigged %s rel_dist: %.2f %s %.2f, %s", name_.c_str(), rel_dist, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	}

	evaluated_ = true;

	return trig;
}
