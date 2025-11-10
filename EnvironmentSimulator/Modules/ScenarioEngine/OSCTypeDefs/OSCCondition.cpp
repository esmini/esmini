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

#include <sstream>
#include "OSCCondition.hpp"
#include "Storyboard.hpp"
#include "logger.hpp"

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
        LOG_ERROR("Undefined Rule: {}", rule);
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
        LOG_ERROR("Undefined Rule: {}", rule);
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
        LOG_ERROR("Undefined Rule: {}", rule);
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
        LOG_ERROR("Undefined Rule: {}", rule);
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
        LOG_ERROR("Undefined Rule: {}", rule);
    }
    return false;
}

void OSCCondition::Log(bool trig, bool full)
{
    if (full)
    {
        LOG_INFO("{}: {}, delay: {:.2f}, {}", name_, trig, delay_, GetAdditionalLogInfo());
    }
    else
    {
        LOG_INFO("{}: {}", name_, trig);
    }
}

bool OSCCondition::GetValue() const
{
    return cond_value_;
}

bool OSCCondition::CheckEdge(bool new_value, bool old_value, OSCCondition::ConditionEdge edge) const
{
    if (edge == OSCCondition::ConditionEdge::NONE)
    {
        return new_value;
    }
    else if (state_ >= ConditionState::EVALUATED)
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
            LOG_ERROR("Invalid edge: {}", edge);
        }
    }

    return false;
}

std::string OSCCondition::Edge2Str() const
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

void OSCCondition::Reset()
{
    history_.Reset();
    cond_value_ = false;
}

bool OSCCondition::Evaluate(double sim_time)
{
    bool result        = CheckCondition(sim_time);
    bool current_value = CheckEdge(result, last_result_, edge_);
    last_result_       = result;

    if (state_ < ConditionState::EVALUATED)
    {
        state_ = ConditionState::EVALUATED;
    }

    if (history_.RegisterValue(sim_time, current_value))
    {
        LOG_DEBUG("Registered {} value {}", name_, current_value);
        if (current_value || history_.GetNumberOfEntries() > 1)
        {
            Log(current_value, true);
        }
    }

    // adjust value based on delay
    bool adjusted_value = history_.GetValueAtTime(sim_time - delay_);
    if (adjusted_value && adjusted_value != cond_value_)
    {
        // Whenever value changes to true, trigger the global condition callback
        if (conditionCallback != nullptr)
        {
            conditionCallback(name_.c_str(), sim_time);
        }
    }

    cond_value_ = adjusted_value;

    return cond_value_;
}

bool ConditionGroup::Evaluate(double sim_time)
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
            result &= condition_[i]->Evaluate(sim_time);
        }
    }

    return result;
}

bool Trigger::Evaluate(double sim_time)
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
            result |= conditionGroup_[i]->Evaluate(sim_time);
        }
    }

    if (result)
    {
        // Log
        LOG_INFO("Trigger /------------------------------------------------");
        for (size_t i = 0; i < conditionGroup_.size(); i++)
        {
            LOG_INFO("Group {}:", i);
            for (size_t j = 0; j < conditionGroup_[i]->condition_.size(); j++)
            {
                conditionGroup_[i]->condition_[j]->Log(conditionGroup_[i]->condition_[j]->GetValue());

                // restart the condition state for next execution iteration
                conditionGroup_[i]->condition_[j]->state_ = OSCCondition::ConditionState::IDLE;
                if (conditionGroup_[i]->condition_[j]->base_type_ == OSCCondition::ConditionType::BY_ENTITY)
                {
                    TrigByEntity* trigger = static_cast<TrigByEntity*>(conditionGroup_[i]->condition_[j]);
                    for (size_t k = 0; k < trigger->triggered_by_entities_.size(); k++)
                    {
                        LOG_INFO("Triggering entity {}: {}", k, trigger->triggered_by_entities_[k]->name_);
                    }
                }
            }
        }
        LOG_INFO("Trigger  ------------------------------------------------/");
    }

    return result;
}

void Trigger::Reset()
{
    for (auto cg : conditionGroup_)
    {
        for (auto c : cg->condition_)
        {
            c->Reset();
        }
    }
}

bool TrigByState::CheckCondition(double sim_time)
{
    (void)sim_time;
    bool result = false;

    // reset any registered transition, which is only valid for one frame
    latest_state_change_.transition = StoryBoardElement::Transition::UNDEFINED_ELEMENT_TRANSITION;

    if (element_ == nullptr)
    {
        return false;
    }

    if (state_change_.empty())
    {
        // if no state change, check state from last change
        result = CheckState(latest_state_change_);
    }
    else
    {
        for (auto state_change : state_change_)
        {
            if (state_change.element != element_)
            {
                continue;
            }
            else
            {
                result = CheckState(state_change);
                if (result == true)
                {
                    break;
                }
            }
        }

        // register latest state change
        latest_state_change_ = state_change_.back();

        state_change_.clear();
    }

    return result;
}

std::string TrigByState::StateChangeToStr(StateChange state_change)
{
    std::string str;
    if (state_change.state == StoryBoardElement::State::STANDBY)
    {
        str = "STANDBY / ";
    }
    else if (state_change.state == StoryBoardElement::State::RUNNING)
    {
        str = "RUNNING / ";
    }
    else if (state_change.state == StoryBoardElement::State::COMPLETE)
    {
        str = "COMPLETE / ";
    }
    else
    {
        str = "UNDEFINED_STATE / ";
    }

    if (state_change.transition == StoryBoardElement::Transition::END_TRANSITION)
    {
        str += "END_TRANSITION";
    }
    else if (state_change.transition == StoryBoardElement::Transition::SKIP_TRANSITION)
    {
        str += "SKIP_TRANSITION";
    }
    else if (state_change.transition == StoryBoardElement::Transition::START_TRANSITION)
    {
        str += "START_TRANSITION";
    }
    else if (state_change.transition == StoryBoardElement::Transition::STOP_TRANSITION)
    {
        str += "STOP_TRANSITION";
    }
    else
    {
        str += "UNDEFINED_TRANSITION";
    }

    return str;
}

bool TrigByState::CheckState(StateChange state_change)
{
    if (target_element_state_ == CondElementState::STANDBY)
    {
        return state_change.state == StoryBoardElement::State::STANDBY;
    }
    else if (target_element_state_ == CondElementState::RUNNING)
    {
        return state_change.state == StoryBoardElement::State::RUNNING;
    }
    else if (target_element_state_ == CondElementState::COMPLETE)
    {
        return state_change.state == StoryBoardElement::State::COMPLETE;
    }
    else if (target_element_state_ == CondElementState::END_TRANSITION)
    {
        return state_change.transition == StoryBoardElement::Transition::END_TRANSITION;
    }
    else if (target_element_state_ == CondElementState::SKIP_TRANSITION)
    {
        return state_change.transition == StoryBoardElement::Transition::SKIP_TRANSITION;
    }
    else if (target_element_state_ == CondElementState::START_TRANSITION)
    {
        return state_change.transition == StoryBoardElement::Transition::START_TRANSITION;
    }
    else if (target_element_state_ == CondElementState::STOP_TRANSITION)
    {
        return state_change.transition == StoryBoardElement::Transition::STOP_TRANSITION;
    }
    else
    {
        LOG_ERROR("Invalid state: {}", target_element_state_);
    }

    return false;
}

std::string TrigByState::GetAdditionalLogInfo()
{
    return fmt::format("{}, {} == {}, edge: {}",
                       element_->GetName(),
                       StateChangeToStr(latest_state_change_),
                       CondElementState2Str(target_element_state_),
                       Edge2Str());
}

void TrigByState::RegisterStateChange(StoryBoardElement* element, StoryBoardElement::State state, StoryBoardElement::Transition transition)
{
    state_change_.push_back({element, state, transition});
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
    else if (state == UNDEFINED_ELEMENT_TRANSITION)
    {
        return "UNDEFINED_ELEMENT_TRANSITION";
    }
    else
    {
        LOG_ERROR("Unknown state: {}", state);
    }

    return "Unknown state";
}

void TrigByState::Reset()
{
    state_change_.clear();
    OSCCondition::Reset();
}

bool TrigBySimulationTime::CheckCondition(double sim_time)
{
    sim_time_   = sim_time;
    bool result = EvaluateRule(sim_time_, value_, rule_);

    return result;
}

std::string TrigBySimulationTime::GetAdditionalLogInfo()
{
    return fmt::format("{:.4f} {} {:.4f}, edge: {}", sim_time_, Rule2Str(rule_), value_, Edge2Str());
}

bool TrigByParameter::CheckCondition(double sim_time)
{
    (void)sim_time;
    bool result = false;

    OSCParameterDeclarations::ParameterStruct* pe = parameters_->getParameterEntry(parameterRef_);
    if (pe == 0)
    {
        if (state_ < ConditionState::EVALUATED)  // print only once
        {
            LOG_ERROR("Parameter {} not found", parameterRef_);
        }
        return result;
    }

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
        LOG_ERROR("Unexpected parameter type: {}", pe->type);
    }

    return result;
}

std::string TrigByParameter::GetAdditionalLogInfo()
{
    OSCParameterDeclarations::ParameterStruct* pe = parameters_->getParameterEntry(parameterRef_);
    return fmt::format("{} {} {} {}, edge: {}", parameterRef_, pe ? pe->value._string : "NOT_FOUND", Rule2Str(rule_), value_, Edge2Str());
}

bool TrigByVariable::CheckCondition(double sim_time)
{
    (void)sim_time;
    bool result = false;

    OSCParameterDeclarations::ParameterStruct* pe = variables_->getParameterEntry(variableRef_);
    if (pe == 0)
    {
        if (state_ < ConditionState::EVALUATED)  // print only once
        {
            LOG_WARN("Variable {} not found", variableRef_);
        }
        return result;
    }

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
        LOG_ERROR("Unexpected variable type: {}", pe->type);
    }

    return result;
}

std::string TrigByVariable::GetAdditionalLogInfo()
{
    OSCParameterDeclarations::ParameterStruct* ve = variables_->getParameterEntry(variableRef_);
    return fmt::format("variable {} {} {} {}, edge: {}", variableRef_, ve ? ve->value._string : "NOT_FOUND", Rule2Str(rule_), value_, Edge2Str());
}

void TrigByTrafficSignal::SetSignal()
{
    auto odr = roadmanager::Position::GetOpenDrive();
    for (const auto& signal : odr->GetDynamicSignals())
    {
        if (signal == nullptr)
        {
            continue;
        }

        if (signal->GetId() == std::stoi(this->signalName_))
        {
            // Add some switch here to cast to other type depending on tl->GetTrafficSignalType() if necessary
            this->traffic_light_ = dynamic_cast<roadmanager::TrafficLight*>(signal);
            if (traffic_light_ == nullptr)
            {
                LOG_ERROR_AND_QUIT("TrafficSignalCondition: Traffic signal with id {} isn't a valid signal", this->signalName_);
            }

            traffic_light_->CheckValidLampModes(this->signalState_);
            return;
        }
    }
    LOG_ERROR_AND_QUIT("TrafficSignalCondition: Couldn't find traffic signal with id {}", this->signalName_);
}

bool TrigByTrafficSignal::CheckCondition(double sim_time)
{
    (void)sim_time;

    return signalState_ == traffic_light_->GetStateString();
}

std::string TrigByTrafficSignal::GetAdditionalLogInfo()
{
    return fmt::format("TrafficSignal signalName={} signalState={}", signalName_, signalState_);
}

bool TrigByTimeHeadway::CheckCondition(double sim_time)
{
    (void)sim_time;

    bool   result = false;
    double rel_dist;
    hwt_ = 0;

    triggered_by_entities_.clear();

    for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
    {
        result          = false;
        Object* trigObj = triggering_entities_.entity_[i].object_;
        if (!trigObj->IsActive() || (object_ && !object_->IsActive()))
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

std::string TrigByTimeHeadway::GetAdditionalLogInfo()
{
    return fmt::format("HWT: {:.2f} {} {:.2f}, edge {}", hwt_, Rule2Str(rule_), value_, Edge2Str());
}

bool TrigByTimeToCollision::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool   result   = false;
    double rel_dist = LARGE_NUMBER, rel_speed = 0.0;

    ttc_ = -1;

    for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
    {
        result          = false;
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
            retVal                     = trigObj->Distance(pos->GetX(), pos->GetY(), cs_, relDistType_, freespace_, rel_dist);
        }

        if (retVal != 0)
        {
            rel_dist = LARGE_NUMBER;
        }

        if (object_)
        {
            if (fabs(object_->pos_.GetVelX()) < SMALL_NUMBER && fabs(object_->pos_.GetVelY()) < SMALL_NUMBER)
            {
                // object standing still, consider only speed of triggering entity
                rel_speed = trigObj->GetSpeed();
            }
            else
            {
                double rel_vel[2] = {0.0, 0.0};
                // Calculate relative speed of triggering entity along object's velocity direction
                double proj_speed = ProjectPointOnVector2DSignedLength(trigObj->pos_.GetVelX(),
                                                                       trigObj->pos_.GetVelY(),
                                                                       object_->pos_.GetVelX(),
                                                                       object_->pos_.GetVelY(),
                                                                       rel_vel[0],
                                                                       rel_vel[1]);

                // calculate trig object relative speed as projected velocity absolute difference considering
                rel_speed = SIGN(trigObj->GetSpeed()) * SIGN(proj_speed) * (proj_speed - fabs(object_->GetSpeed()));
            }
            // printf("rel_dist %.2f obj vel (%.2f, %.2f) speed %.2f trig_obj vel (%.2f, %.2f) speed %.2f proj_speed %.2f rel_speed %.2f\n",
            //     rel_dist, object_->pos_.GetVelX(),
            //     object_->pos_.GetVelY(),
            //     object_->GetSpeed(),
            //     trigObj->pos_.GetVelX(),
            //     trigObj->pos_.GetVelY(),
            //     trigObj->GetSpeed(),
            //     proj_speed,
            //     rel_speed);
        }
        else
        {
            rel_speed = trigObj->speed_;
        }

        // TimeToCollision (TTC) not defined for cases:
        //  - no distance between entities
        //  - moving away from each other
        if (fabs(rel_dist) < SMALL_NUMBER || fabs(rel_speed) < SMALL_NUMBER)
        {
            ttc_ = -1;
        }
        else
        {
            ttc_ = rel_dist / rel_speed;

            if (ttc_ < 0.0)
            {
                ttc_ = -1.0;
            }
            else
            {
                result = EvaluateRule(ttc_, value_, rule_);
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
    }

    return result;
}

std::string TrigByTimeToCollision::GetAdditionalLogInfo()
{
    if (ttc_ < 0)
    {
        return fmt::format("TTC: {} {} {:.2f}, edge {}", "Inf", Rule2Str(rule_), value_, Edge2Str());
    }
    else
    {
        return fmt::format("TTC: {:.2f} {} {:.2f}, edge {}", ttc_, Rule2Str(rule_), value_, Edge2Str());
    }
}

bool TrigByReachPosition::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool   result = false;
    double dist_x, dist_y;
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
            LOG_ERROR_AND_QUIT("missing road manager position");
        }
        pos->EvaluateRelation();

        dist_ = fabs(trigObj->pos_.getRelativeDistance(pos->GetX(), pos->GetY(), dist_x, dist_y));
        if (dist_ < tolerance_)  // dist may reach half lane width, since offset of relative position is 0
        {
            // Check for any orientation condition
            if (checkOrientation_)
            {
                if (abs(trigObj->pos_.GetH() - pos->GetH()) < angularTolerance_ && abs(trigObj->pos_.GetP() - pos->GetP()) < angularTolerance_ &&
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

std::string TrigByReachPosition::GetAdditionalLogInfo()
{
    if (checkOrientation_)
    {
        return fmt::format("distance {:.2f} < tolerance ({:.2f}), orientation [{:.2f}, {:.2f}, {:.2f}] (tolerance {:.2f}), edge: {}",
                           dist_,
                           tolerance_,
                           triggered_by_entities_[0]->pos_.GetH(),
                           triggered_by_entities_[0]->pos_.GetP(),
                           triggered_by_entities_[0]->pos_.GetR(),
                           angularTolerance_,
                           Edge2Str());
    }
    else
    {
        return fmt::format("distance {:.2f} < tolerance ({:.2f}), edge: {}", dist_, tolerance_, Edge2Str());
    }
}

bool TrigByDistance::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool result                = false;
    dist_                      = 0;
    roadmanager::Position* pos = position_->GetRMPos();

    pos->EvaluateRelation();

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

std::string scenarioengine::TrigByDistance::GetAdditionalLogInfo()
{
    return fmt::format("dist: {:.2f} {} {:.2f}, edge: {}", dist_, Rule2Str(rule_), value_, Edge2Str());
}

bool TrigByRelativeDistance::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool result = false;
    rel_dist_   = 0;

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

std::string TrigByRelativeDistance::GetAdditionalLogInfo()
{
    return fmt::format("rel_dist: {:.2f} {} {:.2f}, edge: {}", rel_dist_, Rule2Str(rule_), value_, Edge2Str());
}

bool TrigByCollision::CheckCondition(double sim_time)
{
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
                CollisionPair p = {trigObj, object_};
                collision_pair_.push_back(p);
                result = true;
            }
        }
        if (objectType_ != Object::Type::TYPE_NONE)
        {
            // check all instances of specifed object type
            for (size_t j = 0; j < storyBoard_->entities_->object_.size(); j++)
            {
                // Do not collide with ghosts.
                if (storyBoard_->entities_->object_[j]->isGhost_)
                {
                    continue;
                }
                if (storyBoard_->entities_->object_[j] != trigObj && storyBoard_->entities_->object_[j]->type_ == objectType_ &&
                    storyBoard_->entities_->object_[j]->IsActive())
                {
                    bool local_result = false;
                    if (SE_Env::Inst().GetCollisionDetection() == false)
                    {
                        if (trigObj->Collision(storyBoard_->entities_->object_[j]))
                        {
                            local_result = true;
                        }
                    }
                    else
                    {
                        // reuse results from global collision detection
                        for (size_t k = 0; k < trigObj->collisions_.size(); k++)
                        {
                            if (trigObj->collisions_[k] == storyBoard_->entities_->object_[j])
                            {
                                local_result = true;
                            }
                        }
                    }
                    if (local_result == true)
                    {
                        CollisionPair p = {trigObj, storyBoard_->entities_->object_[j]};
                        collision_pair_.push_back(p);
                        result = true;
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

std::string TrigByCollision::GetAdditionalLogInfo()
{
    std::string str;

    if (!collision_pair_.empty())
    {
        str = fmt::format("{} collision(s): ", collision_pair_.size(), collision_pair_.size());
    }

    for (size_t i = 0; i < collision_pair_.size(); i++)
    {
        str += fmt::format("{} and {}, ", collision_pair_[i].object0->name_, collision_pair_[i].object1->name_);
    }
    str += fmt::format("edge: {}", Edge2Str());

    return str;
}

bool TrigByTraveledDistance::CheckCondition(double sim_time)
{
    (void)sim_time;

    bool result = false;
    odom_       = 0;

    triggered_by_entities_.clear();

    for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
    {
        if (!triggering_entities_.entity_[i].object_->IsActive())
        {
            continue;
        }

        odom_  = triggering_entities_.entity_[i].object_->odometer_;
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

std::string TrigByTraveledDistance::GetAdditionalLogInfo()
{
    return fmt::format("traveled_dist: {:.2f} >= {:.2f}, edge: {}", odom_, value_, Edge2Str());
}

bool TrigByEndOfRoad::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool result       = false;
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
            if (current_duration_ > duration_ - SMALL_NUMBER)
            {
                result = true;
            }
        }

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

std::string TrigByEndOfRoad::GetAdditionalLogInfo()
{
    return fmt::format("end_of_road duration: {:.2f} >= {:.2f}, edge: {}", current_duration_, duration_, Edge2Str());
}

bool TrigByStandStill::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool result       = false;
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

std::string TrigByStandStill::GetAdditionalLogInfo()
{
    return fmt::format("stand_still duration: {:.2f} >= {:.2f}, edge: {}", current_duration_, duration_, Edge2Str());
}

bool TrigByOffRoad::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool result       = false;
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

std::string TrigByOffRoad::GetAdditionalLogInfo()
{
    return fmt::format("off road duration: {:.2f} >= {:.2f}, edge: {}", current_duration_, duration_, Edge2Str());
}

bool TrigByAcceleration::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool result           = false;
    current_acceleration_ = 0;

    for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
    {
        if (!triggering_entities_.entity_[i].object_->IsActive())
        {
            continue;
        }

        if (direction_ == LONGITUDINAL)
        {
            current_acceleration_ = triggering_entities_.entity_[i].object_->pos_.GetAccLong();
        }
        else if (direction_ == LATERAL)
        {
            current_acceleration_ = triggering_entities_.entity_[i].object_->pos_.GetAccLat();
        }
        else if (direction_ == VERTICAL)
        {
            current_acceleration_ = triggering_entities_.entity_[i].object_->pos_.GetAccZ();
        }
        else
        {
            current_acceleration_ = triggering_entities_.entity_[i].object_->pos_.GetAcc();
        }

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

std::string TrigByAcceleration::GetAdditionalLogInfo()
{
    return fmt::format("acceleration: {:.2f} {} {:.2f}, edge: {}", current_acceleration_, Rule2Str(rule_), value_, Edge2Str());
}

bool TrigBySpeed::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();
    bool result    = false;
    current_speed_ = 0;

    for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
    {
        if (!triggering_entities_.entity_[i].object_->IsActive())
        {
            continue;
        }

        if (direction_ == LONGITUDINAL)
        {
            current_speed_ = triggering_entities_.entity_[i].object_->pos_.GetVelLong();
        }
        else if (direction_ == LATERAL)
        {
            current_speed_ = triggering_entities_.entity_[i].object_->pos_.GetVelLat();
        }
        else if (direction_ == VERTICAL)
        {
            current_speed_ = triggering_entities_.entity_[i].object_->pos_.GetVelZ();
        }
        else
        {
            current_speed_ = triggering_entities_.entity_[i].object_->GetSpeed();
        }

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

std::string TrigBySpeed::GetAdditionalLogInfo()
{
    return fmt::format("speed: {:.2f} {} {:.2f}, edge: {}", current_speed_, Rule2Str(rule_), value_, Edge2Str());
}

bool TrigByRelativeSpeed::CheckCondition(double sim_time)
{
    (void)sim_time;

    if (object_ == nullptr)
    {
        LOG_ERROR("TrigByRelativeSpeed: Couldn't resolve refEntity object");
        return false;
    }
    else if (!object_->IsActive())
    {
        LOG_ERROR("TrigByRelativeSpeed: refEntity not active");
        return false;
    }

    triggered_by_entities_.clear();
    bool result        = false;
    current_rel_speed_ = 0;

    for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
    {
        if (!triggering_entities_.entity_[i].object_->IsActive())
        {
            continue;
        }

        if (direction_ == LONGITUDINAL)
        {
            // find out triggering entity speed along longitudinal axis in the reference vehicle coordinate system
            // i.e. project it's speed to reference vehicle longitudinal axis
            double vel_x = cos(-object_->pos_.GetH()) * triggering_entities_.entity_[i].object_->pos_.GetVelX() -
                           sin(-object_->pos_.GetH()) * triggering_entities_.entity_[i].object_->pos_.GetVelY();

            // calculate the relative longitudinal speed (in reference vehicle coordinate system)
            current_rel_speed_ = vel_x - object_->pos_.GetVelLong();
        }
        else if (direction_ == LATERAL)
        {
            double vel_y = sin(-object_->pos_.GetH()) * triggering_entities_.entity_[i].object_->pos_.GetVelX() +
                           cos(-object_->pos_.GetH()) * triggering_entities_.entity_[i].object_->pos_.GetVelY();
            current_rel_speed_ = vel_y - object_->pos_.GetVelLat();
        }
        else if (direction_ == VERTICAL)
        {
            current_rel_speed_ = triggering_entities_.entity_[i].object_->pos_.GetVelZ() - object_->pos_.GetVelZ();
        }
        else
        {
            current_rel_speed_ = triggering_entities_.entity_[i].object_->GetSpeed() - object_->GetSpeed();
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

std::string TrigByRelativeSpeed::GetAdditionalLogInfo()
{
    return fmt::format("relative_speed: {:.2f} {} {:.2f}, edge: {}", current_rel_speed_, Rule2Str(rule_), value_, Edge2Str());
}

bool TrigByRelativeClearance::CheckCondition(double sim_time)
{
    (void)sim_time;

    triggered_by_entities_.clear();

    bool   result   = false;
    bool   objFound = false;
    double maxDist  = MAX(distanceForward_, distanceBackward_);

    roadmanager::OpenDrive* odr = roadmanager::Position::GetOpenDrive();

    for (size_t i = 0; i < triggering_entities_.entity_.size(); i++)
    {
        Object*            entityObject = triggering_entities_.entity_[i].object_;
        roadmanager::Road* road         = odr->GetRoadById(entityObject->pos_.GetTrackId());

        if (road == nullptr)
        {
            return result;
        }

        Object* refObject_;
        result = false;

        // First check all entities within lane range
        for (size_t j = 0; j < storyBoard_->entities_->object_.size(); j++)
        {
            refObject_ = storyBoard_->entities_->object_[j];
            if ((refObject_ == entityObject) ||
                ((objects_.size() != 0) && ((std::find(objects_.begin(), objects_.end(), refObject_) == objects_.end()))))
            {  // ignore the entity which in triggering itself, entity which in not in reference entity list
                continue;
            }

            if (from_ > to_)
            {  // quit execution if to value is less than from value
                LOG_ERROR_AND_QUIT("QUITTING, Wrong from and to value in RelativeLaneRange element");
            }

            PositionDiff diff;
            if (freeSpace_)
            {
                objFound = (entityObject->FreeSpaceDistanceObjectRoadLane(refObject_, &diff, CoordinateSystem::CS_ROAD) == 0);
            }
            else
            {
                objFound = entityObject->pos_.Delta(&refObject_->pos_, diff, true, maxDist);
            }

            if (objFound)
            {
                if (diff.ds < -distanceBackward_ || diff.ds > distanceForward_)
                {
                    // ds is not within range (-distanceBackward_ : distanceForward_)
                    objFound = false;
                }
                else
                {
                    // found object is within specified distance range, check lane range and opposite direction condition
                    if ((diff.dLaneId >= from_) && (diff.dLaneId <= to_) &&
                        !(!oppositeLanes_ && diff.dOppLane))  // reject objects in opposite lane if we don't want them
                    {
                        // We found at least ONE object in the specified clearance area (lane and distance range)
                        break;
                    }
                    else
                    {  // ds is within range but not within lane range and opposite direction condition
                        objFound = false;
                    }
                }
            }
        }

        if (objFound)
        {
            result = false;
        }
        else
        {
            // Then, if lanes in the specified lane range are free of entities also check that the specified area exists
            // check at location of current position (road, lane and s)
            result = false;

            for (unsigned int j = 0; j < road->GetNumberOfDrivingLanes(entityObject->pos_.GetS()); j++)
            {
                int lane_id = road->GetDrivingLaneByIdx(entityObject->pos_.GetS(), j)->GetId();
                if (IS_IN_SPAN(lane_id, entityObject->pos_.GetLaneId() + from_, entityObject->pos_.GetLaneId() + to_) &&  // lane is within range
                    ((oppositeLanes_ == true) ||  // consider any lane when opposite lane flag is false
                     (oppositeLanes_ == false &&
                      SIGN(lane_id) == SIGN(entityObject->pos_.GetLaneId()))))  // consider only same direction when opposite lane flag is false
                {
                    // at least one lane found within the specified range, which is enough
                    result = true;
                    break;
                }
            }
        }

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

std::string TrigByRelativeClearance::GetAdditionalLogInfo()
{
    return fmt::format(", edge: {}", Edge2Str());
}

bool ConditionDelay::RegisterValue(double time, bool value)
{
    if (values_.empty())
    {
        // add first value
        values_.push_back({time, value});
        return true;
    }
    else if (time < values_.back().time_ + SMALL_NUMBER)
    {
        if (time < values_.back().time_ - SMALL_NUMBER)
        {
            LOG_DEBUG("Unexpected time value {:.2f} < latest registered {:.2f}. Probably ghost restart. Insert value based on timestamp.",
                      time,
                      values_.back().time_);
        }

        auto it = std::lower_bound(values_.begin(),
                                   values_.end(),
                                   time,
                                   [](const ConditionValue& pivot_value, const double& t) { return pivot_value.time_ < t; });

        // Insert new value at the correct position wrt timestamp
        // the algorithm is based on that every element is different value than any previous and next
        // the element found by lower_bound is referred to as pivot and is the first element with same or larger time stamp
        if (it != values_.end())
        {
            if (it->time_ == time)
            {
                if (value != it->value_)
                {
                    if (it == values_.begin())
                    {
                        // change value of first element
                        it->value_ = value;
                    }
                    else
                    {
                        // pivot element is obsolete, remove it and the next which also becomes obsolete
                        if (it + 1 != values_.end())
                        {
                            values_.erase(it + 1);
                        }
                        values_.erase(it);
                    }
                }
            }
            else if (it->value_ == value)
            {
                // values are same, update the timestamp of pivot element
                it->time_ = time;
            }
            else  // pivot element has different value
            {
                if (it + 1 != values_.end())
                {
                    // next element value is same, just update it's timestamp
                    (it + 1)->time_ = time;
                }
                else
                {
                    // if there is a previous element it has the same value and we can skip this, else add it
                    if (it == values_.begin())
                    {
                        values_.insert(it, {time, value});
                    }
                }
            }
        }
        else
        {
            // If the time is beyond the last entry, add a new one
            values_.push_back({time, value});
        }
        return true;
    }
    else if (value != values_.back().value_)
    {
        // register new value at given time
        values_.push_back({time, value});
        return true;
    }

    return false;
}

void ConditionDelay::Reset()
{
    values_.clear();
    current_index_ = 0;
}

void ConditionDelay::ResetCurrentIndex(double time)
{
    current_index_ = 0;

    // Check if value has become true in the time window since last check
    for (unsigned int i = 0; i < values_.size(); i++)
    {
        ConditionValue& v = values_[i];
        if (v.time_ < time + SMALL_NUMBER)
        {
            // found value at given timestamp
            current_index_ = i;
            break;
        }
    }
}

bool ConditionDelay::GetValueAtTime(double time)
{
    bool retval = false;

    if (!values_.empty())
    {
        // detect ghost restart by checking for decreased time value
        if (current_index_ > 0 && time < values_[current_index_ - 1].time_ - SMALL_NUMBER)
        {
            LOG_DEBUG("ConditionDelay::GetValueAtTime: Time value of past {:.2f} / {:.2f}, probably Ghost restart. Reset index.",
                      time,
                      values_[current_index_ - 1].time_);
            ResetCurrentIndex(time);
        }

        if (current_index_ >= values_.size())
        {
            // passed last registered value, pick it
            retval = values_.back().value_;
        }
        else if (time > values_[current_index_].time_ - SMALL_NUMBER)
        {
            // Check if value has become true in the time window since last check
            for (; current_index_ < values_.size() && time > values_[current_index_].time_ - SMALL_NUMBER; current_index_++)
            {
                retval |= values_[current_index_].value_;
            }
        }
        else if (current_index_ > 0)
        {
            // use value from current time window
            retval = values_[current_index_ - 1].value_;
        }
    }

    return retval;
}

size_t ConditionDelay::GetNumberOfEntries() const
{
    return values_.size();
}
