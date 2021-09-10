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

#include "OSCManeuver.hpp"

using namespace scenarioengine;

void Event::Start(double simTime, double dt)
{
	for (size_t i = 0; i < action_.size(); i++)
	{

		// Terminate any ongoing action on same object and domain
		if (action_[i]->base_type_ == OSCAction::BaseType::PRIVATE)
		{
			OSCPrivateAction* pa = (OSCPrivateAction*)action_[i];
			Object* obj = pa->object_;
			if (obj != nullptr)
			{
				// First check init actions
				for (size_t j = 0; j < obj->initActions_.size(); j++)
				{
					if (obj->initActions_[j]->base_type_ == OSCAction::BaseType::PRIVATE &&
						obj->initActions_[j]->state_ == StoryBoardElement::State::RUNNING)
					{
						if (static_cast<int>(obj->initActions_[j]->GetDomain()) & static_cast<int>(pa->GetDomain()))
						{
							// Domains overlap, at least one domain in common. Terminate old action.
							LOG("Stopping %s on conflicting %s domain(s)",
								obj->initActions_[j]->name_.c_str(), ControlDomain2Str(obj->initActions_[j]->GetDomain()).c_str());
							obj->initActions_[j]->End();
						}
					}
				}

				// Then check Storyboard event actions
				for (size_t j = 0; j < pa->object_->objectEvents_.size(); j++)
				{
					for (size_t k = 0; k < obj->objectEvents_[j]->action_.size(); k++)
					{
						if (pa->object_->objectEvents_[j]->action_[k] != pa &&
							pa->object_->objectEvents_[j]->action_[k]->state_ == StoryBoardElement::State::RUNNING &&
							pa->object_->objectEvents_[j]->action_[k]->base_type_ == OSCAction::BaseType::PRIVATE)
						{
							OSCPrivateAction* pa2 = (OSCPrivateAction*)obj->objectEvents_[j]->action_[k];
							if (static_cast<int>(pa2->GetDomain()) & static_cast<int>(pa->GetDomain()))
							{
								// Domains overlap, at least one domain in common. Terminate old action.
								LOG("Stopping object %s %s on conflicting %s domain(s)",
									obj->name_.c_str(), pa2->name_.c_str(), ControlDomain2Str(pa2->GetDomain()).c_str());
								pa2->End();
							}
						}
					}
				}
			}
		}
		// Restart actions
		action_[i]->Reset();
		action_[i]->Start(simTime, dt);
	}
	StoryBoardElement::Start(simTime, dt);
}

void Event::End()
{
	for (size_t i = 0; i < action_.size(); i++)
	{
		if (action_[i]->IsActive())
		{
			action_[i]->End();
		}
	}
	StoryBoardElement::End();
}

void Event::Stop()
{
	for (size_t i = 0; i < action_.size(); i++)
	{
		action_[i]->Stop();
	}
	StoryBoardElement::Stop();
}

bool scenarioengine::OSCManeuver::IsAnyEventActive()
{
	for (size_t i = 0; i < event_.size(); i++)
	{
		if (event_[i]->IsActive())
		{
			return true;
		}
	}
	return false;
}
