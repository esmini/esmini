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

/*
 * This controller simulates a bad or dizzy driver by manipulating
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

//#include "playerbase.hpp"
#include "ControllerFollowRoute.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "OSCManeuver.hpp"
#include "ScenarioEngine.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerFollowRoute(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerFollowRoute(initArgs);
}

ControllerFollowRoute::ControllerFollowRoute(InitArgs* args) : Controller(args)
{
}

void ControllerFollowRoute::Init()
{
	LOG("FollowRoute init");

	Controller::Init();
}

void ControllerFollowRoute::Step(double timeStep)
{
	LOG("FollowRoute step");
	//object_->MoveAlongS(timeStep * object_->GetSpeed());

	for(Event* e : object_->getEvents()){
		for(OSCAction* action : e->action_){
			if(action->name_ == "Test"){
				if(!action->IsActive()){
					LOG("Starting lane change action");
					action->Start(scenarioEngine_->getSimulationTime(), timeStep);
				}
				else{
					action->Step(scenarioEngine_->getSimulationTime(), timeStep);
				}
			}
		}
	}

	Controller::Step(timeStep);
}

void ControllerFollowRoute::Activate(ControlDomains domainMask)
{
	LOG("FollowRoute activate");

	this->mode_ = Controller::Mode::MODE_ADDITIVE;

	LatLaneChangeAction* action_lanechange = new LatLaneChangeAction();
	action_lanechange->object_ = object_;
	action_lanechange->transition_.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
	action_lanechange->transition_.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
	action_lanechange->transition_.SetParamTargetVal(5);

	LatLaneChangeAction::TargetAbsolute* test = new LatLaneChangeAction::TargetAbsolute;
	test->value_ = -3;
	action_lanechange->target_ = test;
	action_lanechange->name_="Test";

	Event* event_lanechange = new Event();
	event_lanechange->action_.push_back(action_lanechange);
	event_lanechange->priority_ = Event::Priority::OVERWRITE;
	object_->addEvent(event_lanechange);

	// Trigger* trigger = new Trigger(0);
	// ConditionGroup* conGroup = new ConditionGroup();
	// TrigBySimulationTime* condition = new TrigBySimulationTime();
	// condition->value_ = 2;

	// conGroup->condition_.push_back(condition);
	// trigger->conditionGroup_.push_back(conGroup);
	// event_lanechange->start_trigger_ = trigger;

	// Grab and inspect road network
	roadmanager::OpenDrive* odr = nullptr;
	if (object_ != nullptr)
	{
		odr = object_->pos_.GetOpenDrive();
	}

	if (odr != nullptr)
	{
		for (int i = 0; i < odr->GetNumOfRoads(); i++)
		{
			roadmanager::Road* road = odr->GetRoadByIdx(i);
			LOG("road[%d] id: %d length: %.2f", i, road->GetId(), road->GetLength());
		}
	}

	Controller::Activate(domainMask);
}

void ControllerFollowRoute::ReportKeyEvent(int key, bool down)
{

}