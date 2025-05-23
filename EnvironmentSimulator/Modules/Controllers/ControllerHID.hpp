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

/*
 * This controller let the user control the vehicle interactively by the arrow keys
 */

#pragma once

#include <string>
#include "Controller.hpp"
#include "pugixml.hpp"
#include "Parameters.hpp"
#include "vehicle.hpp"

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#include <iostream>
#else
#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>           // For open()
#include <unistd.h>          // For read() and close()
#include <linux/joystick.h>  // For joystick event structure and ioctl commands
#endif

#define CONTROLLER_HID_TYPE_NAME "HIDController"

namespace scenarioengine
{
    enum class HID_AXIS
    {
        HID_R_AXIS,
        HID_U_AXIS,
        HID_V_AXIS,
        HID_X_AXIS,
        HID_Y_AXIS,
        HID_Z_AXIS,
        HID_NR_OF_AXIS
    };

    // base class for controllers
    class ControllerHID : public Controller
    {
    public:
        ControllerHID(InitArgs* args);
        ~ControllerHID() override;

        void Init();
        void Step(double timeStep);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);

        void Deactivate() override;

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_HID_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return Controller::Type::CONTROLLER_TYPE_HID;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
        }
        int  OpenHID(int device_id);
        void CloseHID();
        int  ReadHID(double& throttle, double& steering);
        int  ParseAxis(const std::string& axis, HID_AXIS& axis_type);

    private:
        vehicle::Vehicle vehicle_;
        double           steering_;
        double           throttle_;
        double           steering_rate_;
        int              device_id_;
        HID_AXIS         throttle_axis_;
        HID_AXIS         steering_axis_;
        int              device_id_internal_;
#ifdef _WIN32
        JOYINFOEX joy_info_;
#endif
    };

    Controller* InstantiateControllerHID(void* args);
}  // namespace scenarioengine