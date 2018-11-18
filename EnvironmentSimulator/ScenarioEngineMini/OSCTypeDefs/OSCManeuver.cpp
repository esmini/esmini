#include "OSCManeuver.hpp"

void Event::Activate()
{
	for (size_t i = 0; i < action_.size(); i++)
	{
		action_[i]->Trig();
	}
	state_ = Event::State::ACTIVE;
}

