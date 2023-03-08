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

BaseSensor::BaseSensor(BaseSensor::Type type, double pos_x, double pos_y, double pos_z, double heading)
{
    type_         = type;
    pos_.x        = pos_x;
    pos_.y        = pos_y;
    pos_.z        = pos_z;
    pos_.h        = heading;
    pos_.p        = 0;
    pos_.r        = 0;
    pos_.x_global = 0;
    pos_.y_global = 0;
    pos_.z_global = 0;
}

ObjectSensor::ObjectSensor(Entities *entities,
                           Object   *refobj,
                           double    pos_x,
                           double    pos_y,
                           double    pos_z,
                           double    heading,
                           double    nearClip,
                           double    farClip,
                           double    fovH,
                           int       maxObj)
    : BaseSensor(BaseSensor::Type::SENSOR_TYPE_OBJECT, pos_x, pos_y, pos_z, heading)
{
    entities_ = entities;
    near_     = nearClip;
    near_sq_  = near_ * near_;
    far_      = farClip;
    far_sq_   = far_ * far_;
    fovH_     = fovH;
    maxObj_   = maxObj;
    host_     = refobj;
    nObj_     = 0;
    hitList_  = static_cast<ObjectHit *>(malloc(static_cast<unsigned int>(maxObj) * sizeof(ObjectHit)));
}

ObjectSensor::~ObjectSensor()
{
    free(hitList_);
}

void ObjectSensor::Update()
{
    nObj_ = 0;

    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        Object *obj = entities_->object_[i];
        if (obj == host_ || obj->IsGhost())
        {
            // skip own vehicle and any ghost vehicles
            continue;
        }

        if (!(obj->visibilityMask_ & Object::Visibility::SENSORS))
        {
            // Object is not visible for sensors
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
        pos_.x_global = host_->pos_.GetX() + sensor_pos_x;
        pos_.y_global = host_->pos_.GetY() + sensor_pos_y;
        pos_.z_global = host_->pos_.GetZ() + pos_.z;

        // Find vector from host to object
        double xo = obj->pos_.GetX() - pos_.x_global;
        double yo = obj->pos_.GetY() - pos_.y_global;

        // First check distance
        double dist_sq = (xo * xo + yo * yo);
        if (dist_sq < near_sq_ || dist_sq > far_sq_)
        {
            // Not within near and far radius/distance
            continue;
        }

        double xon, yon;
        NormalizeVec2D(xo, yo, xon, yon);

        // Find angle to object
        double angle     = acos(GetDotProduct2D(hx2, hy2, xon, yon));
        double rel_angle = GetAbsAngleDifference(angle, pos_.h);
        if (rel_angle < fovH_ / 2)
        {
            hitList_[nObj_].obj_ = obj;

            // Calculate hit object position in sensor local coordinates
            double xl, yl;
            RotateVec2D(xo, yo, -GetAngleSum(host_->pos_.GetH(), pos_.h), xl, yl);

            hitList_[nObj_].x_ = xl;
            hitList_[nObj_].y_ = yl;
            hitList_[nObj_].z_ = obj->pos_.GetZ() - pos_.z_global + 0.7;

            // Calculate hit object velocity in sensor local coordinates
            double xVelTarget = obj->pos_.GetVelX();
            double yVelTarget = obj->pos_.GetVelY();
            double xVelHost   = host_->pos_.GetVelX();
            double yVelHost   = host_->pos_.GetVelY();
            double angleHost  = -GetAngleSum(host_->pos_.GetH(), pos_.h);
            double targetVelXforHost, targetVelYforHost;
            Global2LocalCoordinates(xVelTarget, yVelTarget, xVelHost, yVelHost, angleHost, targetVelXforHost, targetVelYforHost);
            hitList_[nObj_].velX_ = targetVelXforHost;
            hitList_[nObj_].velY_ = targetVelYforHost;

            // Calculate hit object acceleration in sensor local coordinates
            double xAccTarget = obj->pos_.GetAccX();
            double yAccTarget = obj->pos_.GetAccY();
            double xAccHost   = host_->pos_.GetAccX();
            double yAccHost   = host_->pos_.GetAccY();
            double targetAccXforHost, targetAccYforHost;
            Global2LocalCoordinates(xAccTarget, yAccTarget, xAccHost, yAccHost, angleHost, targetAccXforHost, targetAccYforHost);
            hitList_[nObj_].accX_ = targetAccXforHost;
            hitList_[nObj_].accY_ = targetAccYforHost;

            // Calculate hit object yaw, yaw rate and yaw acceleration in sensor local coordinates
            double yawTarget     = obj->pos_.GetH();
            double yawHost       = GetAngleSum(host_->pos_.GetH(), pos_.h);
            hitList_[nObj_].yaw_ = GetAngleDifference(yawTarget, yawHost);

            double yawRateTarget     = obj->pos_.GetHRate();
            double yawRateHost       = host_->pos_.GetHRate();
            hitList_[nObj_].yawRate_ = GetAngleDifference(yawRateTarget, yawRateHost);

            double yawAccTarget     = obj->pos_.GetHAcc();
            double yawAccHost       = host_->pos_.GetHAcc();
            hitList_[nObj_].yawAcc_ = GetAngleDifference(yawAccTarget, yawAccHost);

            nObj_++;
        }
    }
}