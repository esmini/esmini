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

#include "IdealSensor.hpp"

using namespace scenarioengine;

ObjectSensor::ObjectSensor(Entities *entities, Object *refobj, double near, double far, double fovH, int maxObj)
{
	entities_ = entities;
	near_ = near;
	far_ = far;
	fovH_ = fovH;
	maxObj_ = maxObj;
	host_ = refobj;
	nObj_ = 0;
	objList_ = (int*)malloc(maxObj * sizeof(int));
}

ObjectSensor::~ObjectSensor()
{
	free(objList_);
}

void ObjectSensor::Update()
{
	nObj_ = 0;

	for (size_t i = 0; i < entities_->object_.size(); i++)
	{
		Object *obj = entities_->object_[i];
		if (obj == host_)
		{
			// skip own vehicle
			continue;
		}

		// Check whether object is within field of view
		// find out angle between heading vector and line to object
		double hx = 1.0;
		double hy = 0.0;
		double hx2, hy2;
		RotateVec2D(hx, hy, host_->pos_.GetH(), hx2, hy2);

		// Find vector from host to object
		double xo = obj->pos_.GetX() - host_->pos_.GetX();
		double yo = obj->pos_.GetY() - host_->pos_.GetY();
		double xon, yon;
		NormalizeVec2D(xo, yo, xon, yon);

		// Find angle to object
		double angle = acos(GetDotProduct2D(hx2, hy2, xon, yon));

		if (angle < fovH_)
		{
			objList_[nObj_++] = obj->id_;
		}
	}
}