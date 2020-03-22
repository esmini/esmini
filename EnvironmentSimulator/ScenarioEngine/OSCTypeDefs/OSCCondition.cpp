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

bool OSCCondition::CheckEdge(bool new_value, bool old_value, OSCCondition::ConditionEdge edge)
{
	if (evaluated_ && edge == OSCCondition::ConditionEdge::ANY)
	{
		return new_value != old_value;
	}
	else if (evaluated_ && edge == OSCCondition::ConditionEdge::FALLING)
	{
		if (new_value == false && old_value == true)
		{
			return true;
		}
	}
	else if (evaluated_ && edge == OSCCondition::ConditionEdge::RISING)
	{
		if (new_value == true && old_value == false)
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

bool EvalDone(bool result, TrigByEntity::TriggeringEntitiesRule rule)
{
	if (result == false && rule == TrigByEntity::TriggeringEntitiesRule::ALL)
	{
		return true;  // One false is enough
	}
	else if (result == true && rule == TrigByEntity::TriggeringEntitiesRule::ANY)
	{
		return true;  // One true is enough
	}

	return false;
}

bool TrigByState::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;
	bool result = false;

	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	last_result_ = result;
	evaluated_ = true;

	if (result && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return result;
}

bool TrigAtStart::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)sim_time;
	bool trig = false;

	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	if (element_type_ == StoryElementType::SCENE)
	{
		if (edge_ == ConditionEdge::RISING || edge_ == ConditionEdge::ANY)
		{
			trig = true;
		}
	}
	else if (element_type_ == StoryElementType::ACTION)
	{
		OSCAction *action = storyBoard->FindActionByName(element_name_);

		if ( action )
		{
			OSCAction::State state = action->state_;
			//LOG("Action %s state %d", action->name_.c_str(), state);
			if(edge_ == ConditionEdge::RISING && state == OSCAction::State::ACTIVATED)
			{
				trig = true;
			}
			else if (edge_ == ConditionEdge::FALLING && state == OSCAction::State::DEACTIVATED)
			{
				trig = true;
			}
			else if(edge_ == ConditionEdge::ANY && (state == OSCAction::State::ACTIVATED || state == OSCAction::State::DEACTIVATED))
			{
				trig = true;
			}
		}
	}
	else if (element_type_ == StoryElementType::ACT)
	{
		Act *act = storyBoard->FindActByName(element_name_);
		
		if (act)
		{
			Act::State state = act->state_;
			if (edge_ == ConditionEdge::RISING)
			{
				if (state == Act::State::ACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::FALLING)
			{
				if (state == Act::State::DEACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::ANY)
			{
				if (state == Act::State::ACTIVATED || state == Act::State::DEACTIVATED)
				{
					trig = true;
				}
			}
			else
			{
				LOG("Unknown edge type: %d", edge_);
			}
		}
	}
	else if (element_type_ == StoryElementType::EVENT)
	{
		Event *event = storyBoard->FindEventByName(element_name_);

		if (event)
		{
			Event::State state = event->state_;
			if (edge_ == ConditionEdge::RISING)
			{
				if(state == Event::State::ACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::FALLING)
			{
				if (state == Event::State::DEACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::ANY)
			{
				if (state == Event::State::ACTIVATED || state == Event::State::DEACTIVATED)
				{

					trig = true;
				}
			}
			else
			{
				LOG("Unknown edge type: %d", edge_);
			}
		}
	}
	else
	{
		LOG("Story element type %d not supported yet", element_type_);
	}

	evaluated_ = true;

	if (trig)
	{
		LOG("Trigged %s (edge: %s", name_.c_str(), Edge2Str(edge_).c_str());
	}

	if (trig && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return trig;
}

bool TrigAfterTermination::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)sim_time;
	bool trig = false;

	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	if (element_type_ == StoryElementType::SCENE)
	{
		trig = false;
	}
	else if (element_type_ == StoryElementType::ACTION)
	{
		OSCAction *action = storyBoard->FindActionByName(element_name_);

		if (action)
		{
			OSCAction::State state = action->state_;
			if (edge_ == ConditionEdge::RISING)
			{
				if (state == OSCAction::State::DEACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::FALLING)
			{
				if(state == OSCAction::State::ACTIVATED)
				{ 
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::ANY)
			{
				if (state == OSCAction::State::ACTIVATED || state == OSCAction::State::DEACTIVATED)
				{
					trig = true;
				}
			}
			else
			{
				LOG("Unknown edge type: %d", edge_);
			}
		}
	}
	else if (element_type_ == StoryElementType::ACT)
	{
		Act *act = storyBoard->FindActByName(element_name_);

		if (act)
		{
			Act::State state = act->state_;
			if (edge_ == ConditionEdge::RISING)
			{
				if (state == Act::State::DEACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::FALLING)
			{
				if (state == Act::State::ACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::ANY)
			{
				if (state == Act::State::ACTIVATED || state == Act::State::DEACTIVATED)
				{
					trig = true;
				}
			}
			else
			{
				LOG("Unknown edge type: %d", edge_);
			}
		}
	}
	else if (element_type_ == StoryElementType::EVENT)
	{
		Event *event = storyBoard->FindEventByName(element_name_);

		if (event)
		{
			Event::State state = event->state_;
			if (edge_ == ConditionEdge::RISING)
			{
				if (state == Event::DEACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::FALLING)
			{
				if (state == Event::State::ACTIVATED)
				{
					trig = true;
				}
			}
			else if (edge_ == ConditionEdge::ANY)
			{
				if (state == Event::State::ACTIVATED || state == Event::State::DEACTIVATED)
				{
					trig = true;
				}
			}
			else
			{
				LOG("Unknown edge type: %d", edge_);
			}
		}
	}
	else
	{
		LOG("Story element type %d not supported yet", element_type_);
	}

	evaluated_ = true;

	if (trig)
	{
		LOG("Trigged %s (edge: %s)", name_.c_str(), Edge2Str(edge_).c_str());
	}

	if (trig && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return trig;
}

bool TrigByValue::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	bool result = false;

	last_result_ = result;
	evaluated_ = true;

	if (result && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return result;
}

bool TrigBySimulationTime::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;

	bool result = false;
	bool trig = false;
	
	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	// Special case for simulation time: Since many scenarios use simtime == 0 as start condition, 
	// consider simtime == 0 as a change (rising from false) by setting evaluated_ = true from start
	evaluated_ = true;

	result = EvaluateRule(sim_time, value_, rule_);

	trig = CheckEdge(result, last_result_, edge_);

	if (trig)
	{
		LOG("Trigged %s (sim_time: %.2f >= condition: %.2f result: %d last_result: %d edge: %s)", name_.c_str(), sim_time, value_, result, last_result_, Edge2Str(edge_).c_str());
	}

	last_result_ = result;

	if (trig && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return trig;
}

bool TrigByTimeHeadway::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	bool trig = false;
	double rel_dist, hwt;

	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		double x, y;
		rel_dist = triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(object_->pos_, x, y);

		// Headway time not defined for cases:
		//  - when target object is behind 
		//  - when object is still or going reverse 
		if (rel_dist < 0 || object_->speed_ < SMALL_NUMBER)
		{
			hwt = LARGE_NUMBER;
		}
		else
		{
			hwt = fabs(rel_dist / object_->speed_);
		}

		result = EvaluateRule(hwt, value_, rule_);
		trig = CheckEdge(result, last_result_, edge_);

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

	last_result_ = result;
	evaluated_ = true;

	if (trig && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return trig;
}

bool TrigByReachPosition::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	bool trig = false;
	double x, y;

	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (fabs(triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(*position_->GetRMPos(), x, y)) < tolerance_)
		{
			result = true;
		}

		if (EvalDone(trig, triggering_entity_rule_))
		{
			break;
		}
	}

	trig = CheckEdge(result, last_result_, edge_);

	last_result_ = result;
	evaluated_ = true;

	if (trig)
	{
		LOG("Trigged %s (result: %d last_result: %d edge: %s", name_.c_str(), result, last_result_, Edge2Str(edge_).c_str());
	}

	if (trig && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return trig;
}

bool TrigByDistance::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	bool trig = false;
	double x, y;
	double dist;

	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		dist = fabs(triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(*position_->GetRMPos(), x, y));

		if (EvalDone(trig, triggering_entity_rule_))
		{
			break;
		}
	}

	result = EvaluateRule(dist, value_, rule_);

	trig = CheckEdge(result, last_result_, edge_);

	last_result_ = result;
	evaluated_ = true;

	if (trig)
	{
		LOG("Trigged %s (dist: %.2f condition: %.2f result: %d last_result: %d edge: %s", name_.c_str(), dist, value_, result, last_result_, Edge2Str(edge_).c_str());
	}

	if (trig && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return trig;
}

bool TrigByRelativeDistance::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	bool trig = false;
	double rel_dist, rel_intertial_dist, x, y;

	if (timer_.Started())
	{
		if (timer_.DurationS() > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS());
			timer_.Reset();
			return true;
		}
		return false;
	}

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		rel_intertial_dist = triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(object_->pos_, x, y);

		if (type_ == RelativeDistanceType::LONGITUDINAL)
		{
			rel_dist = fabs(x);
		}
		else if (type_ == RelativeDistanceType::LATERAL)
		{
			rel_dist = fabs(y);
		}
		else if (type_ == RelativeDistanceType::INTERIAL)
		{
			rel_dist = fabs(rel_intertial_dist);
		}
		else
		{
			LOG("Unsupported RelativeDistance type: %d", type_);
		}

		result = EvaluateRule(rel_dist, value_, rule_);
		trig = CheckEdge(result, last_result_, edge_);
		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	//LOG("RelDist Trig? %s rel_dist: %.2f %s %.2f, %s", name_.c_str(), rel_dist, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	if (trig)
	{
		LOG("Trigged %s rel_dist: %.2f %s %.2f, %s (%d, %d)", name_.c_str(), rel_dist, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str(), result, last_result_);
	}

	last_result_ = result;
	evaluated_ = true;

	if (trig && delay_ > 0)
	{
		timer_.Start();
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	return trig;
}
