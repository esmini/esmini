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

 // expression implementation is based on https://github.com/zserge/expr

#ifdef __cplusplus
extern "C" {
#endif

/**
* Evaluate expression
* @param str Experssion
* @return evaluated resulting value as float
*/
float eval_expr(const char* str);

#ifdef __cplusplus
} /* extern "C" */
#endif

