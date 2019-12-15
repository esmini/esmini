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

#include "ScenarioEngine.hpp"

namespace scenarioengine
{
	typedef struct
	{
		double x;
		double y;
		double z;
		double h;
		double p;
		double r;
	} SensorPosition;

	class ObjectSensor
	{
	public:

		double near_;         // Near limit field of view, from position of sensor
		double far_;          // Far limit field of view, from position of sensor
		double fovH_;         // Horizontal field of view, in degrees
		int maxObj_;          // Maximum length of object list
					         
		Object *host_;        // Entity to which the sensor is attached
		SensorPosition pos_;  // Position, relative host object
		int *objList_;        // List of identified objects
		int nObj_;            // Size of object list, i.e. number of identified objects

		ObjectSensor(Entities *entities, Object *refobj, double near, double far, double fovH, int maxObj);
		ObjectSensor::~ObjectSensor();
		void Update();

	private:

		Entities *entities_;   // Reference to the global collection of objects within the scenario

	};

}
