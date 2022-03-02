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
using namespace roadmanager;

void (*OSCCondition::conditionCallback)(const char* name, double timestamp) = nullptr;

std::string Rule2Str(Rule rule)
{
	if (rule == Rule::GREATER_THAN)
	{
		return ">";
	}
	else if (rule == Rule::GREATER_OR_EQUAL)
	{
		return ">=";
	}
	else if (rule == Rule::LESS_THAN)
	{
		return "<";
	}
	else if (rule == Rule::LESS_OR_EQUAL)
	{
		return "<=";
	}
	else if (rule == Rule::EQUAL_TO)
	{
		return "==";
	}
	else if (rule == Rule::NOT_EQUAL_TO)
	{
		return "!=";
	}
	else
	{
		LOG("Undefined Rule: %d", rule);
	}
	return "unknown";
}

bool EvaluateRule(double a, double b, Rule rule)
{
	if (rule == Rule::GREATER_THAN)
	{
		return a > b + SMALL_NUMBER;
	}
	else if (rule == Rule::GREATER_OR_EQUAL)
	{
		return a >= b - SMALL_NUMBER;
	}
	else if (rule == Rule::LESS_THAN)
	{
		return a < b - SMALL_NUMBER;
	}
	else if (rule == Rule::LESS_OR_EQUAL)
	{
		return a <= b + SMALL_NUMBER;
	}
	else if (rule == Rule::EQUAL_TO)
	{
		return (a == b || (a > b - SMALL_NUMBER && a < b + SMALL_NUMBER));
	}
	else if (rule == Rule::NOT_EQUAL_TO)
	{
		return !(a == b || (a > b - SMALL_NUMBER && a < b + SMALL_NUMBER));
	}
	else
	{
		LOG("Undefined Rule: %d", rule);
	}
	return false;
}

bool EvaluateRule(int a, int b, Rule rule)
{
	if (rule == Rule::GREATER_THAN)
	{
		return a > b;
	}
	else if (rule == Rule::GREATER_OR_EQUAL)
	{
		return a >= b;
	}
	else if (rule == Rule::LESS_THAN)
	{
		return a < b;
	}
	else if (rule == Rule::LESS_OR_EQUAL)
	{
		return a <= b;
	}
	else if (rule == Rule::EQUAL_TO)
	{
		return (a == b);
	}
	else if (rule == Rule::NOT_EQUAL_TO)
	{
		return !(a == b);
	}
	else
	{
		LOG("Undefined Rule: %d", rule);
	}
	return false;
}

bool EvaluateRule(std::string a, std::string b, Rule rule)
{
	if (rule == Rule::GREATER_THAN)
	{
		return a > b;
	}
	else if (rule == Rule::GREATER_OR_EQUAL)
	{
		return a >= b;
	}
	else if (rule == Rule::LESS_THAN)
	{
		return a < b;
	}
	else if (rule == Rule::LESS_OR_EQUAL)
	{
		return a <= b;
	}
	else if (rule == Rule::EQUAL_TO)
	{
		return (a == b);
	}
	else if (rule == Rule::NOT_EQUAL_TO)
	{
		return !(a == b);
	}
	else
	{
		LOG("Undefined Rule: %d", rule);
	}
	return false;
}

bool EvaluateRule(bool a, bool b, Rule rule)
{
	if (rule == Rule::GREATER_THAN)
	{
		return a == true && b == false;
	}
	else if (rule == Rule::GREATER_OR_EQUAL)
	{
		return a == b || (a == true && b == false);
	}
	else if (rule == Rule::LESS_THAN)
	{
		return a == false && b == true;
	}
	else if (rule == Rule::LESS_OR_EQUAL)
	{
		return a == b || (a == false && b == true);
	}
	else if (rule == Rule::EQUAL_TO)
	{
		return (a == b);
	}
	else if (rule == Rule::NOT_EQUAL_TO)
	{
		return !(a == b);
	}
	else
	{
		LOG("Undefined Rule: %d", rule);
	}
	return false;
}

void OSCCondition::Log()
{
	LOG("%s result: %d", name_.c_str(), last_result_);
}

bool OSCCondition::CheckEdge(bool new_value, bool old_value, OSCCondition::ConditionEdge edge)
{
	if (edge == OSCCondition::ConditionEdge::NONE)
	{
		return new_value;
	}
	else if (state_ == ConditionState::EVALUATED)
	{
		if (edge == OSCCondition::ConditionEdge::RISING_OR_FALLING)
		{
			return new_value != old_value;
		}
		else if (edge == OSCCondition::ConditionEdge::FALLING)
		{
			if (new_value == false && old_value == true)
			{
				return true;
			}
		}
		else if (edge == OSCCondition::ConditionEdge::RISING)
		{
			if (new_value == true && old_value == false)
			{
				return true;
			}
		}
		else
		{
			LOG("Invalid edge: %d", edge);
		}
	}

	return false;
}

std::string OSCCondition::Edge2Str()
{
	if (edge_ == OSCCondition::ConditionEdge::FALLING)
	{
		return "falling";
	}
	else if (edge_ == OSCCondition::ConditionEdge::RISING)
	{
		return "rising";
	}
	else if (edge_ == OSCCondition::ConditionEdge::RISING_OR_FALLING)
	{
		return "risingOrFalling";
	}
	else if (edge_ == OSCCondition::ConditionEdge::NONE)
	{
		return "none";
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

	if (state_ == ConditionState::TIMER)
	{
		if (timer_.Expired(sim_time))
		{
			LOG("%s timer expired at %.2f seconds", name_.c_str(), timer_.Elapsed(sim_time));
			timer_.Reset();
			state_ = ConditionState::TRIGGERED;

			// Trigger the global condition callback
			if (conditionCallback != nullptr)
			{
				conditionCallback(name_.c_str(), sim_time);
			}

			return true;
		}
		else
		{
			return false;
		}
	}

	bool result = CheckCondition(storyBoard, sim_time);
	bool trig = CheckEdge(result, last_result_, edge_);
	last_result_ = result;

	if (state_ < ConditionState::EVALUATED)
	{
		state_ = ConditionState::EVALUATED;
	}

	if (delay_ > 0 && trig && state_ < ConditionState::TIMER)
	{
		timer_.Start(sim_time, delay_);
		state_ = ConditionState::TIMER;
		LOG("%s timer %.2fs started", name_.c_str(), delay_);
		return false;
	}

	if (trig)
	{
		state_ = ConditionState::TRIGGERED;

		// Trigger the global condition callback
		if (conditionCallback != nullptr)
		{
			conditionCallback(name_.c_str(), sim_time);
		}
	}

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
		if (result == true || condition_[i]->delay_ > 0.0)
		{
			// When at least one condition in the group is false,
			// only conditions with timer needs to be evaluated
			result &= condition_[i]->Evaluate(storyBoard, sim_time);
		}
	}

	return result;
}

bool Trigger::Evaluate(StoryBoard *storyBoard, double sim_time)
{
	bool result = false;

	if (conditionGroup_.size() == 0)
	{
		result = defaultValue_;
	}
	else
	{
		for (size_t i = 0; i < conditionGroup_.size(); i++)
		{
			// OR operator, at least one must be true
			result |= conditionGroup_[i]->Evaluate(storyBoard, sim_time);
		}
	}

	if (result)
	{
		// Log
		LOG("Trigger /------------------------------------------------");
		for (size_t i = 0; i < conditionGroup_.size(); i++)
		{
			for (size_t j = 0; j < conditionGroup_[i]->condition_.size(); j++)
			{
				conditionGroup_[i]->condition_[j]->Log();
				if (conditionGroup_[i]->condition_[j]->base_type_ == OSCCondition::ConditionType::BY_ENTITY)
				{
					TrigByEntity* trigger = (TrigByEntity*)conditionGroup_[i]->condition_[j];
					for (size_t k = 0; k < trigger->triggered_by_entities_.size(); k++)
					{
						LOG("Triggering entity %d: %s", k, trigger->triggered_by_entities_[k]->name_.c_str());
					}
				}
			}
		}
		LOG("Trigger  ------------------------------------------------/");
	}

	return result;
}

bool TrigByState::CheckCondition(StoryBoard *storyBoard, double sim_time)
{
	(void)sim_time;
	bool result = false;
	StoryBoardElement *element = 0;


	if (element_type_ == StoryBoardElement::ElementType::STORY)
	{
		if (state_ < ConditionState::EVALUATED)
		{
			result = element_state_ == CondElementState::START_TRANSITION;
		}
		else
		{
			result = element_state_ == CondElementState::RUNNING;
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

		if (element == 0)
		{
			LOG("Story board element \"%s\" not found", element_name_.c_str());
			return false;
		}

		if (element_state_ == CondElementState::STANDBY)
		{
			result = element->state_ == StoryBoardElement::State::STANDBY;
		}
		else if (element_state_ == CondElementState::RUNNING)
		{
			result = element->state_ == StoryBoardElement::State::RUNNING;
		}
		else if (element_state_ == CondElementState::COMPLETE)
		{
			result = element->state_ == StoryBoardElement::State::COMPLETE;
		}
		else if (element_state_ == CondElementState::END_TRANSITION)
		{
			result = element->transition_ == StoryBoardElement::Transition::END_TRANSITION;
		}
		else if (element_state_ == CondElementState::SKIP_TRANSITION)
		{
			result = element->transition_ == StoryBoardElement::Transition::SKIP_TRANSITION;
		}
		else if (element_state_ == CondElementState::START_TRANSITION)
		{
			result = element->transition_ == StoryBoardElement::Transition::START_TRANSITION;
		}
		else if (element_state_ == CondElementState::STOP_TRANSITION)
		{
			result = element->transition_ == StoryBoardElement::Transition::STOP_TRANSITION;
		}
		else
		{
			LOG("Invalid state: %d", element_state_);
		}
	}

	return result;
}

void TrigByState::Log()
{
	LOG("%s == %s, element: %s state: %s, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		element_name_.c_str(), CondElementState2Str(element_state_).c_str(), Edge2Str().c_str());
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

bool TrigBySimulationTime::CheckCondition(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	sim_time_ = sim_time;
	bool result = EvaluateRule(sim_time_, value_, rule_);

	return result;
}

void TrigBySimulationTime::Log()
{
	LOG("%s == %s, %.4f %s %.2f edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		sim_time_, Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
}

bool TrigByParameter::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	bool result = false;
	current_value_str_ = "";

	OSCParameterDeclarations::ParameterStruct* pe = parameters_->getParameterEntry(name_);
	if (pe == 0)
	{
		if (state_ < ConditionState::EVALUATED)  // print only once
		{
			LOG("Parameter %s not found", name_.c_str());
		}
		return result;
	}

	current_value_str_ = std::to_string(pe->value._int).c_str();
	if (pe->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
	{
		result = EvaluateRule(pe->value._int, strtoi(value_), rule_);

	}
	else if (pe->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
	{
		result = EvaluateRule(pe->value._double, strtod(value_), rule_);
	}
	else if (pe->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING)
	{
		result = EvaluateRule(pe->value._string, value_, rule_);
	}
	else if (pe->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL)
	{
		result = EvaluateRule(pe->value._bool, value_ == "true" ? true : false, rule_);
	}
	else
	{
		LOG("Unexpected parameter type: %d", pe->type);
	}

	return result;
}

void TrigByParameter::Log()
{
	LOG("parameter %s %s %s %s edge: %s", name_.c_str(), current_value_str_.c_str(),
			Rule2Str(rule_).c_str(), value_.c_str(), Edge2Str().c_str());
}

bool TrigByTimeHeadway::CheckCondition(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	double rel_dist;
	hwt_ = 0;

	triggered_by_entities_.clear();

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		Object* trigObj = triggering_entities_.entity_[i].object_;
		if (!trigObj->IsActive() || object_ && !object_->IsActive())
		{
			continue;
		}

		if (trigObj->Distance(object_, cs_, relDistType_, freespace_, rel_dist) != 0)
		{
			rel_dist = LARGE_NUMBER;
		}

		// Headway time not defined for cases:
		//  - when target object is behind
		//  - when object is still or going reverse
		if (rel_dist < 0 || trigObj->speed_ < SMALL_NUMBER)
		{
			hwt_ = -1;
		}
		else
		{
			hwt_ = fabs(rel_dist / trigObj->speed_);

			result = EvaluateRule(hwt_, value_, rule_);

			if (result == true)
			{
				triggered_by_entities_.push_back(trigObj);
			}

			if (EvalDone(result, triggering_entity_rule_))
			{
				break;
			}
		}
	}

	return result;
}

void TrigByTimeHeadway::Log()
{
	LOG("%s == %s, HWT: %.2f %s %.2f, edge %s", name_.c_str(), last_result_ ? "true" : "false",
		hwt_, Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
}

bool TrigByTimeToCollision::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	double rel_dist, rel_speed;

	ttc_ = -1;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		Object* trigObj = triggering_entities_.entity_[i].object_;
		if (!trigObj->IsActive())
		{
			continue;
		}

		int retVal = 0;

		if (object_ != nullptr)
		{
			if (!object_->IsActive())
			{
				continue;
			}
			retVal = trigObj->Distance(object_, cs_, relDistType_, freespace_, rel_dist);
		}
		else
		{
			roadmanager::Position* pos = position_->GetRMPos();
			retVal = trigObj->Distance(pos->GetX(), pos->GetY(), cs_, relDistType_, freespace_, rel_dist);
		}
		if (retVal != 0)
		{
			rel_dist = LARGE_NUMBER;
		}

		if (object_)
		{
			rel_speed = trigObj->speed_ - object_->speed_;
		}
		else
		{
			rel_speed = trigObj->speed_;
		}

		// TimeToCollision (TTC) not defined for cases:
		//  - when target object is behind
		//  - when triggering entity speed is <=0 (still or going reverse)
		//  - when distance is constant or increasing
		if (rel_dist < 0 || trigObj->speed_ < SMALL_NUMBER || rel_speed <= SMALL_NUMBER)
		{
			ttc_ = -1;
		}
		else
		{
			ttc_ = fabs(rel_dist / rel_speed);

			result = EvaluateRule(ttc_, value_, rule_);

			if (result == true)
			{
				triggered_by_entities_.push_back(trigObj);
			}

			if (EvalDone(result, triggering_entity_rule_))
			{
				break;
			}
		}
	}

	return result;
}

void TrigByTimeToCollision::Log()
{
	if (ttc_ < 0)
	{
		LOG("%s == %s, TTC: %s %s %.2f, edge %s", name_.c_str(), last_result_ ? "true" : "false",
			"Inf", Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
	}
	else
	{
		LOG("%s == %s, TTC: %.2f %s %.2f, edge %s", name_.c_str(), last_result_ ? "true" : "false",
			ttc_, Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
	}
}

bool TrigByReachPosition::CheckCondition(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	double x, y;
	dist_ = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		Object* trigObj = triggering_entities_.entity_[i].object_;
		if (!trigObj->IsActive())
		{
			continue;
		}

		Position* pos = position_->GetRMPos();
		if (pos == nullptr)
		{
			LOG_AND_QUIT("missing road manager position");
		}

		dist_ = fabs(trigObj->pos_.getRelativeDistance(pos->GetX(), pos->GetY(), x, y));
		if (dist_ < tolerance_)
		{
			// Check for any orientation condition
			if (checkOrientation_)
			{
				if (abs(trigObj->pos_.GetH() - pos->GetH()) < angularTolerance_ &&
					abs(trigObj->pos_.GetP() - pos->GetP()) < angularTolerance_ &&
					abs(trigObj->pos_.GetR() - pos->GetR()) < angularTolerance_)
				{
					result = true;
				}
				else
				{
					result = false;
				}
			}
			else
			{
				result = true;
			}
		}

		if (result == true)
		{
			triggered_by_entities_.push_back(trigObj);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByReachPosition::Log()
{
	if (checkOrientation_)
	{
		if (position_->GetRMPos()->GetOrientationType() == Position::OrientationType::ORIENTATION_ABSOLUTE)
		{
			LOG("%s == %s, distance %.2f < tolerance (%.2f), abs orientation [%.2f, %.2f, %.2f] (tolerance %.2f), edge: % s",
				name_.c_str(), last_result_ ? "true" : "false", dist_, tolerance_,
				triggered_by_entities_[0]->pos_.GetH(), triggered_by_entities_[0]->pos_.GetP(), triggered_by_entities_[0]->pos_.GetR(),
				angularTolerance_, Edge2Str().c_str());
		}
		else
		{
			LOG("%s == %s, distance %.2f < tolerance (%.2f), rel orientation [%.2f, %.2f, %.2f] (tolerance %.2f), edge: % s",
				name_.c_str(), last_result_ ? "true" : "false", dist_, tolerance_,
				triggered_by_entities_[0]->pos_.GetHRelative(), triggered_by_entities_[0]->pos_.GetPRelative(), triggered_by_entities_[0]->pos_.GetRRelative(),
				angularTolerance_, Edge2Str().c_str());
		}
	}
	else
	{
		LOG("%s == %s, distance %.2f < tolerance (%.2f), edge: %s", name_.c_str(), last_result_ ? "true" : "false",
			dist_, tolerance_, Edge2Str().c_str());
	}
}

bool TrigByDistance::CheckCondition(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	dist_ = 0;
	roadmanager::Position* pos = position_->GetRMPos();

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		Object* trigObj = triggering_entities_.entity_[i].object_;
		if (!trigObj->IsActive())
		{
			continue;
		}

		if (trigObj->Distance(pos->GetX(), pos->GetY(), cs_, relDistType_, freespace_, dist_) != 0)
		{
			dist_ = LARGE_NUMBER;
		}
		else
		{
			// in conditions only consider absolute distances for now
			dist_ = fabs(dist_);
		}

		result = EvaluateRule(dist_, value_, rule_);

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByDistance::Log()
{
	LOG("%s == %s, dist: %.2f %s %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		dist_, Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
}

bool TrigByRelativeDistance::CheckCondition(StoryBoard *storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	rel_dist_ = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		Object* trigObj = triggering_entities_.entity_[i].object_;
		if (!(trigObj->IsActive() && object_->IsActive()))
		{
			continue;
		}

		roadmanager::CoordinateSystem cs = cs_;

		if (trigObj->Distance(object_, cs, relDistType_, freespace_, rel_dist_) != 0)
		{
			rel_dist_ = LARGE_NUMBER;
		}
		else
		{
			// in conditions only consider absolute distances for now
			rel_dist_ = fabs(rel_dist_);
		}

		result = EvaluateRule(rel_dist_, value_, rule_);

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByRelativeDistance::Log()
{
	LOG("%s == %s, rel_dist: %.2f %s %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		rel_dist_, Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
}

bool TrigByCollision::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;

	triggered_by_entities_.clear();
	collision_pair_.clear();

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		Object* trigObj = triggering_entities_.entity_[i].object_;
		if (!trigObj->IsActive())
		{
			continue;
		}

		if (object_ && object_->IsActive())
		{
			if (trigObj->Collision(object_))
			{
				CollisionPair p = { trigObj, object_ };
				collision_pair_.push_back(p);
				result = true;
			}
		}
		if (type_ != Object::Type::TYPE_NONE)
		{
			// check all instances of specifed object type
			for (size_t j = 0; j < storyBoard->entities_->object_.size(); j++)
			{
				if (storyBoard->entities_->object_[j] != trigObj &&
					storyBoard->entities_->object_[j]->type_ == type_ &&
					storyBoard->entities_->object_[j]->IsActive())
				{
					if (SE_Env::Inst().GetCollisionDetection() == false)
					{
						if (trigObj->Collision(storyBoard->entities_->object_[j]))
						{
							result = true;
						}
					}
					else
					{
						// reuse results from global collision detection
						for (size_t k = 0; k < trigObj->collisions_.size(); k++)
						{
							if (trigObj->collisions_[k] == storyBoard->entities_->object_[j])
							{
								result = true;
							}
						}
					}
					if (result == true)
					{
						CollisionPair p = { trigObj, storyBoard->entities_->object_[j] };
						collision_pair_.push_back(p);
					}
				}
			}
		}

		if (result == true)
		{
			triggered_by_entities_.push_back(trigObj);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByCollision::Log()
{
	for (size_t i = 0; i < collision_pair_.size(); i++)
	{
		LOG("collision %d between %s and %s", i, collision_pair_[i].object0->name_.c_str(), collision_pair_[i].object1->name_.c_str());
	}
	LOG("%s == %s edge: %s",
		name_.c_str(), last_result_ ? "true" : "false", Edge2Str().c_str());
}

bool TrigByTraveledDistance::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	bool result = false;
	odom_ = 0;

	triggered_by_entities_.clear();

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (!triggering_entities_.entity_[i].object_->IsActive())
		{
			continue;
		}

		odom_ = triggering_entities_.entity_[i].object_->odometer_;
		result = odom_ >= value_;

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByTraveledDistance::Log()
{
	LOG("%s == %s, traveled_dist: %.2f >= %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		odom_, value_, Edge2Str().c_str());
}

bool TrigByEndOfRoad::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	current_duration_ = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (!triggering_entities_.entity_[i].object_->IsActive())
		{
			continue;
		}

		if (triggering_entities_.entity_[i].object_->IsEndOfRoad())
		{
			current_duration_ = sim_time - triggering_entities_.entity_[i].object_->GetEndOfRoadTimestamp();
		}

		result = current_duration_ > duration_;

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByEndOfRoad::Log()
{
	LOG("%s == %s, end_of_road duration: %.2f >= %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		current_duration_, duration_, Edge2Str().c_str());
}

bool TrigByStandStill::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	current_duration_ = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (!triggering_entities_.entity_[i].object_->IsActive())
		{
			continue;
		}

		if (triggering_entities_.entity_[i].object_->IsStandStill())
		{
			current_duration_ = sim_time - triggering_entities_.entity_[i].object_->GetStandStillTimestamp();
		}

		result = current_duration_ > duration_;

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByStandStill::Log()
{
	LOG("%s == %s, stand_still duration: %.2f >= %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		current_duration_, duration_, Edge2Str().c_str());
}

bool TrigByOffRoad::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	current_duration_ = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (!triggering_entities_.entity_[i].object_->IsActive())
		{
			continue;
		}

		if (triggering_entities_.entity_[i].object_->IsOffRoad())
		{
			current_duration_ = sim_time - triggering_entities_.entity_[i].object_->GetOffRoadTimestamp();
		}

		result = current_duration_ > duration_;

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByOffRoad::Log()
{
	LOG("%s == %s, off road duration: %.2f >= %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		current_duration_, duration_, Edge2Str().c_str());
}

bool TrigByAcceleration::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	current_acceleration_ = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (!triggering_entities_.entity_[i].object_->IsActive())
		{
			continue;
		}

		current_acceleration_ = sqrt(pow(triggering_entities_.entity_[i].object_->pos_.GetAccX(), 2) +
			pow(triggering_entities_.entity_[i].object_->pos_.GetAccY(), 2));

		result = EvaluateRule(current_acceleration_, value_, rule_);

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByAcceleration::Log()
{
	LOG("%s == %s, acceleration: %.2f %s %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		current_acceleration_, Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
}

bool TrigBySpeed::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	current_speed_ = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (!triggering_entities_.entity_[i].object_->IsActive())
		{
			continue;
		}

		current_speed_ = triggering_entities_.entity_[i].object_->GetSpeed();

		result = EvaluateRule(current_speed_, value_, rule_);

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigBySpeed::Log()
{
	LOG("%s == %s, speed: %.2f %s %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		current_speed_, Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
}

bool TrigByRelativeSpeed::CheckCondition(StoryBoard* storyBoard, double sim_time)
{
	(void)storyBoard;
	(void)sim_time;

	triggered_by_entities_.clear();
	bool result = false;
	current_rel_speed_ = 0;

	for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
	{
		if (!triggering_entities_.entity_[i].object_->IsActive())
		{
			continue;
		}

		if (object_)
		{
			current_rel_speed_ = triggering_entities_.entity_[i].object_->GetSpeed() - object_->GetSpeed();
		}
		else
		{
			current_rel_speed_ = triggering_entities_.entity_[i].object_->GetSpeed();
		}

		result = EvaluateRule(current_rel_speed_, value_, rule_);

		if (result == true)
		{
			triggered_by_entities_.push_back(triggering_entities_.entity_[i].object_);
		}

		if (EvalDone(result, triggering_entity_rule_))
		{
			break;
		}
	}

	return result;
}

void TrigByRelativeSpeed::Log()
{
	LOG("%s == %s, relative_speed: %.2f %s %.2f, edge: %s", name_.c_str(), last_result_ ? "true" : "false",
		current_rel_speed_, Rule2Str(rule_).c_str(), value_, Edge2Str().c_str());
}