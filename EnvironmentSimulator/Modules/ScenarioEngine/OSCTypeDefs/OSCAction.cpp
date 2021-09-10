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

#include "OSCAction.hpp"

using namespace scenarioengine;

std::string StoryBoardElement::state2str(StoryBoardElement::State state)
{
	if (state == StoryBoardElement::State::COMPLETE)
	{
		return "completeState";
	}
	else if (state == StoryBoardElement::State::RUNNING)
	{
		return "runningState";
	}
	else if (state == StoryBoardElement::State::STANDBY)
	{
		return "standbyState";
	}
	else
	{
		LOG("Undefined element state: %d", state);
	}

	return "Undefined";
}

std::string StoryBoardElement::transition2str(StoryBoardElement::Transition transition)
{
	if (transition == StoryBoardElement::Transition::START_TRANSITION)
	{
		return "startTransition";
	}
	else if (transition == StoryBoardElement::Transition::END_TRANSITION)
	{
		return "endTransition";
	}
	else if (transition == StoryBoardElement::Transition::STOP_TRANSITION)
	{
		return "stopTransition";
	}
	else if (transition == StoryBoardElement::Transition::SKIP_TRANSITION)
	{
		return "skipTransition";
	}
	else
	{
		LOG("Undefined transition: %d", transition);
	}

	return "Undefined";
}


std::string OSCAction::BaseType2Str()
{
	if (base_type_ == BaseType::GLOBAL)
	{
		return "Global";
	}
	else if(base_type_ == BaseType::PRIVATE)
	{
		return "Private";
	}
	else if (base_type_ == BaseType::USER_DEFINED)
	{
		return "User defined";
	}
	else
	{
		LOG("Undefined Base Type: %d", base_type_);
	}

	return "Undefined";
}

void StoryBoardElement::SetState(StoryBoardElement:: State state)
{
	if (state != state_)
	{
		LOG("%s %s -> %s -> %s", name_
			.c_str(),
			state2str(state_).c_str(),
			transition2str(transition_).c_str(),
			state2str(state).c_str());
	}
	state_ = state;
}

void StoryBoardElement::UpdateState()
{
	if (next_state_ != state_)
	{
		SetState(next_state_);
	}
	else if (transition_ != Transition::UNDEFINED_ELEMENT_TRANSITION)
	{
		// Reset transition indicator
		transition_ = Transition::UNDEFINED_ELEMENT_TRANSITION;
	}

}
