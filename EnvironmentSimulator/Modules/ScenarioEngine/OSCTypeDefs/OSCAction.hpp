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

#pragma once

#include "Entities.hpp"

namespace scenarioengine
{

	class StoryBoardElement
	{
	public:

		typedef enum
		{
			STORY,
			ACT,
			MANEUVER_GROUP,
			MANEUVER,
			EVENT,
			ACTION,
			UNDEFINED_ELEMENT_TYPE
		} ElementType;

		typedef enum
		{
			STANDBY,
			RUNNING,
			COMPLETE,
			UNDEFINED_ELEMENT_STATE
		} State;

		typedef enum
		{
			START_TRANSITION,  // Transitions last for one step
			END_TRANSITION,
			STOP_TRANSITION,
			SKIP_TRANSITION,
			UNDEFINED_ELEMENT_TRANSITION
		} Transition;

		ElementType type_;
		State state_;
		State next_state_;
		Transition transition_;
		std::string name_;
		int num_executions_;
		int max_num_executions_;

		StoryBoardElement(ElementType type) :
			type_(type),
			state_(State::STANDBY),
			next_state_(State::STANDBY),
			transition_(Transition::UNDEFINED_ELEMENT_TRANSITION),
			num_executions_(0),
			max_num_executions_(-1) {}

		StoryBoardElement(ElementType type, int max_num_executions) :
			type_(type),
			state_(State::STANDBY),
			next_state_(State::STANDBY),
			transition_(Transition::UNDEFINED_ELEMENT_TRANSITION),
			num_executions_(0),
			max_num_executions_(max_num_executions) {}

		void UpdateState();
		void SetState(State state);
		std::string state2str(State state);
		std::string transition2str(StoryBoardElement::Transition state);

		bool IsActive()
		{
			// Also consider when just being activated - indicated by next_state == running
			// to avoid single frames of no updates (zero motion)
			// Elements on transition to end or stop states also considered not active
			return ((state_ == State::RUNNING || next_state_ == State::RUNNING) &&
				(transition_ != Transition::END_TRANSITION && transition_ != Transition::STOP_TRANSITION));
		}

		bool IsTriggable()
		{
			return state_ == State::STANDBY;
		}

		virtual void Start(double simTime, double dt)
		{
			if (state_ == State::STANDBY || next_state_ == State::STANDBY)
			{
				transition_ = Transition::START_TRANSITION;
				next_state_ = State::RUNNING;
				num_executions_++;
			}
			else
			{
				LOG("%s Invalid Start transition request from %s to %s", name_.c_str(), state2str(state_).c_str(), state2str(State::RUNNING).c_str());
			}
		}

		virtual void Stop()
		{
			if (state_ == State::STANDBY || state_ == State::RUNNING)
			{
				transition_ = Transition::STOP_TRANSITION;
				next_state_ = State::COMPLETE;
			}
			else
			{
				LOG("%s Invalid Stop transition requested from %s to %s", name_.c_str(), state2str(state_).c_str(), state2str(State::COMPLETE).c_str());
			}
		}

		virtual void End()
		{
			// Allow elements to move directly from standby to complete
			// Some actions are atomic, and don't need run time
			if (state_ == State::RUNNING || state_ == State::STANDBY )
			{
				transition_ = Transition::END_TRANSITION;
				if (type_ == ElementType::ACT || type_ == ElementType::ACTION)
				{
					next_state_ = State::COMPLETE;
				}
				else if (max_num_executions_ != -1 && num_executions_ >= max_num_executions_)
				{
					LOG("%s complete after %d execution%s", name_.c_str(), num_executions_, num_executions_ > 1 ? "s" : "");
					next_state_ = State::COMPLETE;
				}
				else
				{
					next_state_ = State::STANDBY;
				}
			}
			else
			{
				LOG("%s Invalid End transition requested from %s to %s or %s", name_.c_str(), state2str(state_).c_str(),
					state2str(State::STANDBY).c_str(), state2str(State::COMPLETE).c_str());
			}
		}

		void Standby()
		{
			if (state_ == State::STANDBY)
			{
				transition_ = Transition::SKIP_TRANSITION;
				next_state_ = State::STANDBY;
			}
			else if (state_ == State::RUNNING)
			{
				transition_ = Transition::END_TRANSITION;
				next_state_ = State::STANDBY;
			}
			else
			{
				LOG("Invalid transition requested from %s to %s", state2str(state_).c_str(), state2str(State::STANDBY).c_str());
			}
		}

		void Reset()
		{
			state_ = State::STANDBY;
			next_state_ = State::STANDBY;
			transition_ = Transition::UNDEFINED_ELEMENT_TRANSITION;
			num_executions_ = 0;
		}
	};

	class OSCAction: public StoryBoardElement
	{
	public:
		typedef enum
		{
			GLOBAL,
			USER_DEFINED,
			PRIVATE,
		} BaseType;

		BaseType base_type_;

		OSCAction(BaseType type) : base_type_(type), StoryBoardElement(StoryBoardElement::ElementType::ACTION) {}
		virtual ~OSCAction() {}

		std::string BaseType2Str();
		virtual std::string Type2Str() { return BaseType2Str(); }

		virtual void Step(double simTime, double dt) = 0;
	};

}
