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

BaseSensor::BaseSensor(BaseSensor::Type type, double pos_x, double pos_y)
{
	type_ = type;
	pos_.x = pos_x;
	pos_.y = pos_y;
	pos_.z = 0.5;
	pos_.h = 0;
	pos_.p = 0;
	pos_.r = 0;
}

ObjectSensor::ObjectSensor(Entities *entities, Object *refobj, double pos_x, double pos_y, double near, double far, double fovH, int maxObj):
	BaseSensor(BaseSensor::Type::SENSOR_TYPE_OBJECT, pos_x, pos_y)
{
	entities_ = entities;
	near_ = near;
	near_sq_ = near_ * near_;
	far_ = far;
	far_sq_ = far_ * far_;
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

		double sensor_pos_x, sensor_pos_y;
		RotateVec2D(pos_.x, pos_.y, host_->pos_.GetH(), sensor_pos_x, sensor_pos_y);

		// Find vector from host to object
		double xo = obj->pos_.GetX() - (host_->pos_.GetX() + sensor_pos_x);
		double yo = obj->pos_.GetY() - (host_->pos_.GetY() + sensor_pos_y);

		// First check distance
		double dist_sq = (xo*xo + yo * yo);
		if (dist_sq < near_sq_ || dist_sq > far_sq_)
		{
			// Not within near and far radius/distance
			continue;
		}

		double xon, yon;
		NormalizeVec2D(xo, yo, xon, yon);

		// Find angle to object
		double angle = acos(GetDotProduct2D(hx2, hy2, xon, yon));
		//LOG("angle: %.2f", 180*angle/M_PI);
		if (angle < fovH_/2)
		{
			objList_[nObj_++] = obj->id_;
		}
	}
}