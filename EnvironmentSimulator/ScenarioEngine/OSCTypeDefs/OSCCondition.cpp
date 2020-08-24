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
	if (edge == OSCCondition::ConditionEdge::NONE)
	{
		return new_value;
	}
	else if (evaluated_ && edge == OSCCondition::ConditionEdge::RISING_OR_FALLING)
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
	else if (edge == OSCCondition::ConditionEdge::RISING_OR_FALLING)
	{
		return "RisingOrFalling";
	}
	else if (edge == OSCCondition::ConditionEdge::NONE)
	{
		return "NONE";
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

bool OSCCondition::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	if (timer_.Started())
	{
		if (timer_.DurationS(sim_time) > delay_)
		{
			LOG("Timer expired at %.2f seconds", timer_.DurationS(sim_time));
			timer_.Reset();
			return true;
		}
		return false;
	}

	bool result = CheckCondition(storyBoard, sim_time, false);
	bool trig = CheckEdge(result, last_result_, edge_);

	last_result_ = result;
	evaluated_ = true;

	if (trig && delay_ > 0)
	{
		timer_.Start(sim_time);
		LOG("Timer %.2fs started", delay_);
		return false;
	}

	// Print a log message first time condition evaluates to true
	if (trig && trig != last_trig_)
	{
		CheckCondition(storyBoard, sim_time, true);
	}
	
	last_trig_ = trig;

	return trig;
}

bool ConditionGroup::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	if (condition_.size() == 0)
	{
		return false;
	}

	bool result = true;
	for (size_t i = 0; i < condition_.size(); i++)
	{
		// AND operator, all must be true
		result &= condition_[i]->Evaluate(storyBoard, sim_time);
	}

	return result;
}

bool Trigger::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	bool result = false;

	for (size_t i = 0; i < conditionGroup_.size(); i++)
	{
		// OR operator, at least one must be true
		result |= conditionGroup_[i]->Evaluate(storyBoard, sim_time);
	}

	return result;
}

bool TrigByState::CheckCondition(StoryBoard *storyBoard, double sim_time, bool log)
{
	(void)sim_time;
	bool result = false;
	StoryBoardElement *element = 0;


	if (element_type_ == StoryBoardElement::ElementType::STORY)
	{
		if (evaluated_ == false)
		{
			result = state_ == CondElementState::START_TRANSITION;
		}
		else
		{
			result = state_ == CondElementState::RUNNING;
		}
	}
	else
	{
		if (element_type_ == StoryBoardElement::ElementType::ACTION)
		{
			element = storyBoard->FindActionByName(element_name_);
		}
		else if (element_type_ == StoryBoardElement::ElementType::ACT)
		{
			element = storyBoard->FindActByName(element_name_);
		}
		else if (element_type_ == StoryBoardElement::ElementType::EVENT)
		{
			element = storyBoard->FindEventByName(element_name_);
		}
		else
		{
			LOG("Story element type %d not supported yet", element_type_);
			return false;
		}

		if (state_ == CondElementState::STANDBY)
		{
			result = element->state_ == StoryBoardElement::State::STANDBY;
		}
		else if (state_ == CondElementState::RUNNING)
		{
			result = element->state_ == StoryBoardElement::State::RUNNING;
		}
		else if (state_ == CondElementState::COMPLETE)
		{
			result = element->state_ == StoryBoardElement::State::COMPLETE;
		}
		else if (state_ == CondElementState::END_TRANSITION)
		{
			result = element->transition_ == StoryBoardElement::Transition::END_TRANSITION;
		}
		else if (state_ == CondElementState::SKIP_TRANSITION)
		{
			result = element->transition_ == StoryBoardElement::Transition::SKIP_TRANSITION;
		}
		else if (state_ == CondElementState::START_TRANSITION)
		{
			result = element->transition_ == StoryBoardElement::Transition::START_TRANSITION;
		}
		else if (state_ == CondElementState::STOP_TRANSITION)
		{
			result = element->transition_ == StoryBoardElement::Transition::STOP_TRANSITION;
		}
		else
		{
			LOG("Invalid state: %d", state_);
		}
	}

	if (log) 
	{
		LOG("%s == %s, element: %s state: %s, edge: %s", name_.c_str(), result ? "true" : "false", 
			element_name_.c_str(), CondElementState2Str(state_).c_str(), Edge2Str(edge_).c_str());
	}

	return result;
}

std::string TrigByState::CondElementState2Str(CondElementState state)
{
	if (state == STANDBY)
	{
		return "STANDBY";
	}
	else if (state == RUNNING)
	{
		return "RUNNING";
	}
	else if (state == COMPLETE)
	{
		return "COMPLETE";
	}
	else if (state == UNDEFINED_ELEMENT_STATE)
	{
		return "UNDEFINED_ELEMENT_STATE";
	}
	else if (state == START_TRANSITION)
	{
		return "START_TRANSITION";
	}
	else if (state == END_TRANSITION)
	{
		return "END_TRANSITION";
	}
	else if (state == STOP_TRANSITION)
	{
		return "STOP_TRANSITION";
	}
	else if (state == SKIP_TRANSITION)
	{
		return "SKIP_TRANSITION";
	}
	else if (state == COMPLETE_TRANSITION)
	{
		return "COMPLETE_TRANSITION";
	}
	else if (state == UNDEFINED_ELEMENT_TRANSITION)
	{
		return "UNDEFINED_ELEMENT_TRANSITION";
	}
	else
	{
		LOG("Unknown state: %d", state);
	}
	
	return "Unknown state";
}

bool TrigBySimulationTime::CheckCondition(StoryBoard *storyBoard, double sim_time, bool log)
{
	(void)storyBoard;
	bool result = EvaluateRule(sim_time, value_, rule_);

	if (log)
	{
		LOG("%s == %s, sim_time: %.4f %s %.2f edge: %s", name_.c_str(), result ? "true" : "false",
			sim_time, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	}

	return result;
}

bool TrigByTimeHeadway::CheckCondition(StoryBoard *storyBoard, double sim_time, bool log)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	double rel_dist, hwt = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (along_route_ == true)
		{
			roadmanager::PositionDiff diff;
			triggering_entities_.entity_[i].object_->pos_.Delta(object_->pos_, diff);
			rel_dist = diff.ds;
		}
		else
		{
			double x, y;
			rel_dist = triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(object_->pos_, x, y);
			// Only consider X-component of distance vector
			rel_dist = x;
		}

		// Headway time not defined for cases:
		//  - when target object is behind 
		//  - when object is still or going reverse 
		if (rel_dist < 0 || triggering_entities_.entity_[i].object_->speed_ < SMALL_NUMBER)
		{
			hwt = -1;
		}
		else
		{
			hwt = fabs(rel_dist / triggering_entities_.entity_[i].object_->speed_);

			result = EvaluateRule(hwt, value_, rule_);

			if (EvalDone(result, triggering_entity_rule_))  
			{
				break;
			}
		}
	}
	if (log)
	{
		LOG("%s == %s, HWT: %.2f %s %.2f, edge %s", name_.c_str(), result ? "true" : "false",
			hwt, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	}

	return result;
}

bool TrigByReachPosition::CheckCondition(StoryBoard *storyBoard, double sim_time, bool log)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	double x, y, dist = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		dist = fabs(triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(*position_->GetRMPos(), x, y));
		if (dist < tolerance_)
		{
			result = true;
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	if (log)
	{
		LOG("%s == %s, distance %.2f < tolerance (%.2f), edge: %s", name_.c_str(), result ? "true" : "false", 
			dist, tolerance_, Edge2Str(edge_).c_str());
	}

	return result;
}

bool TrigByDistance::CheckCondition(StoryBoard *storyBoard, double sim_time, bool log)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	double dist = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (along_route_ == true)
		{
			roadmanager::PositionDiff diff;
			triggering_entities_.entity_[i].object_->pos_.Delta(*position_->GetRMPos(), diff);
			dist = fabs(diff.ds);
		}
		else
		{
			double x, y;
			dist = fabs(triggering_entities_.entity_[i].object_->pos_.getRelativeDistance(*position_->GetRMPos(), x, y));
		}

		result = EvaluateRule(dist, value_, rule_);

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}
	
	if (log)
	{
		LOG("%s == %s, dist: %.2f %s %.2f, edge: %s", name_.c_str(), result ? "true" : "false", 
			dist, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	}

	return result;
}

bool TrigByRelativeDistance::CheckCondition(StoryBoard *storyBoard, double sim_time, bool log)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	double rel_dist = 0, rel_intertial_dist, x, y;

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

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	if (log)
	{
		LOG("%s == %s, rel_dist: %.2f %s %.2f, edge: %s", name_.c_str(), result ? "true" : "false",
			rel_dist, Rule2Str(rule_).c_str(), value_, Edge2Str(edge_).c_str());
	}

	return result;
}

bool TrigByTraveledDistance::CheckCondition(StoryBoard* storyBoard, double sim_time, bool log)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	double odom = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		odom = triggering_entities_.entity_[i].object_->odometer_;
		result = odom >= value_;

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	if (log)
	{
		LOG("%s == %s, traveled_dist: %.2f >= %.2f, edge: %s", name_.c_str(), result ? "true" : "false",
			odom, value_, Edge2Str(edge_).c_str());
	}

	return result;
}

bool TrigByEndOfRoad::CheckCondition(StoryBoard* storyBoard, double sim_time, bool log)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	double current_duration = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (triggering_entities_.entity_[i].object_->IsEndOfRoad())
		{
			current_duration = sim_time - triggering_entities_.entity_[i].object_->GetEndOfRoadTimestamp();
		}
		
		result = current_duration > duration_;

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	if (log)
	{
		LOG("%s == %s, end_of_road duration: %.2f >= %.2f, edge: %s", name_.c_str(), result ? "true" : "false",
			current_duration, duration_, Edge2Str(edge_).c_str());
	}

	return result;
}
