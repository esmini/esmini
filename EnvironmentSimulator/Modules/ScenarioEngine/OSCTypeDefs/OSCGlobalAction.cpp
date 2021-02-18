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

#include "OSCGlobalAction.hpp"

using namespace scenarioengine;

void ParameterSetAction::Start(double simTime, double dt)
{
	LOG("Set parameter %s = %s", name_.c_str(), value_.c_str());
	parameters_->setParameterValueByString(name_, value_);
	OSCAction::Start(simTime, dt);
}

void ParameterSetAction::Step(double, double dt)
{
	OSCAction::Stop();
}

void SwarmTrafficAction::Start()
{
	LOG("SwarmTrafficAction Start");

	// Get handle to road network
	roadmanager::OpenDrive* odrManager = roadmanager::Position::GetOpenDrive();
	for (size_t i = 0; i < odrManager->GetNumOfRoads(); i++)
	{
		roadmanager::Road* road = odrManager->GetRoadByIdx((int)i);
		printf("Road %d length: %.2f\n", (int)i, road->GetLength());
	}

	OSCAction::Start();
}

void SwarmTrafficAction::Step(double dt, double simTime)
{
	LOG("SwarmTrafficAction Step");
	printf("Central object world pos (x, y): %.2f, %.2f\n", centralObject_->pos_.GetX(), centralObject_->pos_.GetY());
	printf("Central object road pos (roadId, s): %d, %.2f\n", centralObject_->pos_.GetTrackId(), centralObject_->pos_.GetS());
}
