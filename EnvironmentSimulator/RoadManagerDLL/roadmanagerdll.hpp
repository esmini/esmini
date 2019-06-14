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

#ifdef WIN32
  #define RM_DLL_API __declspec(dllexport)
#else
  #define RM_DLL_API  // Leave empty on Mac
#endif


#ifdef __cplusplus
extern "C"
{
#endif

	RM_DLL_API int RM_Init(const char *odrFilename);


#ifdef __cplusplus
}
#endif
