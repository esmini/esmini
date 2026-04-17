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
#include <mmsystem.h>
#endif

#define CONTROLLER_HID_TYPE_NAME "HIDController"

namespace scenarioengine
{
    // base class for controllers
    class ControllerHID : public Controller
    {
    public:
        enum HID_INPUT
        {
            HID_AXIS_X  = 0,
            HID_AXIS_Y  = 1,
            HID_AXIS_Z  = 2,
            HID_AXIS_RX = 3,
            HID_AXIS_RY = 4,
            HID_AXIS_RZ = 5,
            // buttons starts on ID 0 in Linux, 1 in Windows where first (0) is unused
            HID_BTN_0        = 6,
            HID_BTN_1        = 7,
            HID_BTN_2        = 8,
            HID_BTN_3        = 9,
            HID_BTN_4        = 10,
            HID_BTN_5        = 11,
            HID_BTN_6        = 12,
            HID_BTN_7        = 13,
            HID_BTN_8        = 14,
            HID_BTN_9        = 15,
            HID_NR_OF_INPUTS = 16
        };

        ControllerHID(InitArgs* args);
        ~ControllerHID() override;

        void Init();
        void Step(double timeStep);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);

        void Deactivate() override;

        virtual const char* GetTypeName() const
        {
            return CONTROLLER_HID_TYPE_NAME;
        }

        virtual Type GetType() const
        {
            return Controller::Type::CONTROLLER_TYPE_HID;
        }

        int  OpenHID(int device_id);
        void CloseHID();
        int  ReadHID(double& throttle, double& steering);
        int  ParseHIDInputType(const std::string& type_str, HID_INPUT& input, int& sign);

    private:
        vehicle::Vehicle vehicle_;
        double           steering_           = 0.0;
        double           throttle_           = 0.0;
        double           steering_rate_      = 0.0;
        int              device_id_          = 0;
        HID_INPUT        steering_input_     = HID_INPUT::HID_AXIS_X;
        HID_INPUT        throttle_input_     = HID_INPUT::HID_AXIS_RZ;
        HID_INPUT        brake_input_        = HID_INPUT::HID_AXIS_RZ;
        int              steering_sign_      = 1;
        int              throttle_sign_      = 1;
        int              brake_sign_         = 1;
        int              device_id_internal_ = -1;
        int64_t          values_[HID_INPUT::HID_NR_OF_INPUTS];
#ifdef _WIN32
        JOYINFOEX joy_info_;
#endif
    };

    Controller* InstantiateControllerHID(void* args);
}  // namespace scenarioengine