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

#include <string>
#include "Controller.hpp"
#include "Parameters.hpp"
#include "vehicle.hpp"
#include "UDP.hpp"

#define CONTROLLER_UDP_DRIVER_TYPE_NAME "UDPDriverController"

#define UDP_DRIVER_MESSAGE_VERSION      1
#define DEFAULT_UDP_DRIVER_PORT         49950
#define UDP_SYNCHRONOUS_MODE_TIMEOUT_MS 500

namespace scenarioengine
{
    // base class for controllers
    class ControllerUDPDriver : public Controller
    {
    public:
        enum class InputMode
        {
            NO_INPUT             = 0,
            DRIVER_INPUT         = 1,
            VEHICLE_STATE_XYZHPR = 2,
            VEHICLE_STATE_XYH    = 3,
            VEHICLE_STATE_H      = 4
        };

        enum class ExecMode
        {
            EXEC_MODE_ASYNCHRONOUS = 0,
            EXEC_MODE_SYNCHRONOUS  = 1,
        };

        typedef struct
        {
            unsigned int version;
            unsigned int inputMode;
            unsigned int objectId;
            unsigned int frameNumber;
        } DMHeader;

        typedef struct
        {
            double        x;
            double        y;
            double        h;
            double        speed;
            double        wheelAngle;
            unsigned char deadReckon;
        } DMMSGVehicleStateXYH;

        typedef struct
        {
            double        h;
            double        speed;
            double        wheelAngle;
            unsigned char deadReckon;
        } DMMSGVehicleStateH;

        typedef struct
        {
            double        x;
            double        y;
            double        z;
            double        h;
            double        p;
            double        r;
            double        speed;
            double        wheelAngle;
            unsigned char deadReckon;
        } DMMSGVehicleStateXYZHPR;

        typedef struct
        {
            double throttle;       // range [0, 1]
            double brake;          // range [0, 1]
            double steeringAngle;  // range [-pi/2, pi/2]
        } DMMSGDriverInput;

        typedef struct
        {
            DMHeader header;
            union
            {
                DMMSGVehicleStateH      stateH;
                DMMSGVehicleStateXYH    stateXYH;
                DMMSGVehicleStateXYZHPR stateXYZHPR;
                DMMSGDriverInput        driverInput;
            } message;
        } DMMessage;

        ControllerUDPDriver(InitArgs* args);
        ~ControllerUDPDriver();
        std::string InputMode2Str(InputMode inputMode);
        std::string ExecMode2Str(ExecMode execMode);

        void Init();
        void Step(double timeStep);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);
        void ReportKeyEvent(int key, bool down);

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_UDP_DRIVER_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return Controller::Type::CONTROLLER_TYPE_UDP_DRIVER;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
        }

    private:
        vehicle::Vehicle  vehicle_;
        vehicle::THROTTLE accelerate = vehicle::THROTTLE_NONE;
        vehicle::STEERING steer      = vehicle::STEERING_NONE;
        InputMode         inputMode_;
        UDPServer*        udpServer_;
        int               port_;
        static int        basePort_;
        ExecMode          execMode_;
        DMMessage         msg;
        DMMessage         lastMsg;
    };

    Controller* InstantiateControllerUDPDriver(void* args);
}  // namespace scenarioengine