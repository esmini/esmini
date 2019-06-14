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

#include "roadmanagerdll.hpp"
#include "roadmanager.hpp"

using namespace roadmanager;

extern "C"
{
	RM_DLL_API int RM_Init(const char *odrFilename)
	{
		return 0;
	}

}
