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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>


#include "ScenarioEngine.hpp"

#define DEFAULT_INPORT 48199


typedef struct
{
	float x;		// m
	float y;		// m
	float z;		// m
	float h;		// rad
	float p;		// rad
	float r;		// rad
	float speed;	// m/s
	float wheel_angle; // rad
} EgoStateBuffer_t;


namespace scenarioengine
{
	void StartServer(ScenarioEngine *scenarioEngine);
	void StopServer();
}
