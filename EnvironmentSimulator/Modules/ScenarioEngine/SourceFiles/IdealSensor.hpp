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
        double x_global;
        double y_global;
        double z_global;
    } SensorPosition;

    class BaseSensor
    {
    public:
        typedef enum
        {
            SENSOR_TYPE_UNDEFINED,
            SENSOR_TYPE_OBJECT,
        } Type;

        Type           type_;
        SensorPosition pos_;  // Position, relative host object

        BaseSensor(BaseSensor::Type type, double pos_x, double pos_y, double pos_z, double heading);
        virtual ~BaseSensor() = default;

        virtual void Update()
        {
            LOG("Virtual, should be overridden");
        };
    };

    class ObjectSensor : public BaseSensor
    {
    public:
        typedef struct
        {
            Object *obj_;  // Identified object
            double  x_;    // Position of object, in local coordinates from sensor
            double  y_;
            double  z_;
            double  velX_;  // Velocity of object in local coordinates from sensor
            double  velY_;
            double  velZ_;
            double  accX_;  // Acceleration of object in local coordinates from sensor
            double  accY_;
            double  accZ_;
            double  yaw_;  // Yaw of object in local coordinates from sensor
            double  yawRate_;
            double  yawAcc_;
        } ObjectHit;

        double     near_;     // Near limit field of view, from position of sensor
        double     near_sq_;  // Near squared - for performance purpose
        double     far_;      // Far limit field of view, from position of sensor
        double     far_sq_;   // Far squared - for performance purpose
        double     fovH_;     // Horizontal field of view, in degrees
        double     fovV_;     // Vertical field of view, in degrees
        int        maxObj_;   // Maximum length of object list
        ObjectHit *hitList_;  // List of identified objects
        Object    *host_;     // Entity to which the sensor is attached
        int        nObj_;     // Size of object list, i.e. number of identified objects

        ObjectSensor(Entities *entities,
                     Object   *refobj,
                     double    pos_x,
                     double    pos_y,
                     double    pos_z,
                     double    heading,
                     double    nearClip,
                     double    farClip,
                     double    fovH,
                     int       maxObj);
        ~ObjectSensor();
        void Update();

    private:
        Entities *entities_;  // Reference to the global collection of objects within the scenario
    };

}  // namespace scenarioengine
