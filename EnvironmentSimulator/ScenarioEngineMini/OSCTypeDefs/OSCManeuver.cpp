#include "OSCManeuver.hpp"

using namespace scenarioengine;

void Event::Activate()
{
	for (size_t i = 0; i < action_.size(); i++)
	{
		action_[i]->Trig();
	}
	state_ = Event::State::ACTIVE;
}

void Event::Deactivate()
{
	state_ = Event::State::DEACTIVATED;
}

bool scenarioengine::Event::Triggable()
{
	if (state_ == Event::State::NOT_TRIGGED ||
		state_ == Event::State::DEACTIVATED ||
		state_ == Event::State::DONE)
	{
		return true;
	}
	return false;
}



