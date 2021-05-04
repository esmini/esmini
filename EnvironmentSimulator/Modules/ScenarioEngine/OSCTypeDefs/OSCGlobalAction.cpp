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


