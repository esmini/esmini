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

namespace vehicle
{
    typedef enum
    {
        THROTTLE_BRAKE      = -1,
        THROTTLE_NONE       = 0,
        THROTTLE_ACCELERATE = 1,
    } THROTTLE;

    typedef enum
    {
        STEERING_RIGHT = -1,
        STEERING_NONE  = 0,
        STEERING_LEFT  = 1,
    } STEERING;

    class Vehicle
    {
    public:
        Vehicle()
        {
            Reset();
        }
        Vehicle(double x, double y, double h, double length, double speed = 0.0);
        void Update(double dt);
        void DrivingControlTarget(double dt, double target_speed, double heading_to_target);
        void DrivingControlBinary(double dt, THROTTLE throttle, STEERING steering);

        /**
                Update vehicle in terms of continuous throttle, brake and steering values
                @param dt Timestep (sec)
                @param throttle Acceleration (>0) or deceleration (<0) level in the range(-1, 1)
                @param steering Steering output left (>0) or right (<0) in the range(-1, 1)
        */
        void DrivingControlAnalog(double dt, double throttle, double steering);
        void SetWheelAngle(double angle);
        void SetWheelRotation(double rotation);
        void SetSpeed(double speed);
        void SetLength(double length)
        {
            length_ = length;
        }
        void SetPos(double x, double y, double z, double h)
        {
            posX_    = x;
            posY_    = y;
            posZ_    = z;
            heading_ = h;
        }
        void SetZ(double z)
        {
            posZ_ = z;
        }
        void SetPitch(double pitch)
        {
            pitch_ = pitch;
        }
        void   SetMaxSpeed(double speed);
        double GetMaxSpeed()
        {
            return max_speed_;
        }
        void SetMaxAcc(double acc)
        {
            max_acc_ = acc;
        }
        double GetMaxAcc()
        {
            return max_acc_;
        }
        void SetMaxDec(double dec)
        {
            max_dec_ = dec;
        }
        double GetMaxDec()
        {
            return max_dec_;
        }
        void SetSteeringRate(double steering_rate)
        {
            steering_rate_ = steering_rate;
        }
        double GetSteeringRate()
        {
            return steering_rate_;
        }
        void SetSteeringReturnFactor(double steering_return_factor)
        {
            steering_return_factor_ = steering_return_factor;
        }
        double GetSteeringReturnFactor()
        {
            return steering_return_factor_;
        }
        void SetSteeringScale(double steering_scale)
        {
            steering_scale_ = steering_scale;
        }
        double GetSteeringScale()
        {
            return steering_scale_;
        }
        bool GetThrottleDisabled()
        {
            return steering_disabled_;
        }
        void SetThrottleDisabled(bool value)
        {
            throttle_disabled_ = value;
        }
        bool GetSteeringDisabled()
        {
            return steering_disabled_;
        }
        void SetSteeringDisabled(bool value)
        {
            steering_disabled_ = value;
        }

        // Set engine brake factor, applied when no throttle is applied. Recommended range = [0.0, 0.01], default = 0.001
        void SetEngineBrakeFactor(double engineBrakeFactor)
        {
            engine_brake_factor_ = engineBrakeFactor;
        }
        void Reset();

        double posX_;
        double posY_;
        double posZ_;
        double heading_;
        double pitch_;

        double velX_;
        double velY_;
        double velAngle_;
        double velAngleRelVehicleLongAxis_;
        double speed_;
        double wheelAngle_;
        double wheelRotation_;
        double headingDot_;
        bool   handbrake_;

        double target_speed_;

        double length_;

    private:
        double max_speed_;
        double max_acc_;
        double max_dec_;
        double steering_rate_;
        double steering_return_factor_;
        double steering_scale_;
        double engine_brake_factor_;
        bool   throttle_disabled_;
        bool   steering_disabled_;
    };

}  // namespace vehicle