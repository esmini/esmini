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

#include "ControllerHID.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerHID(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerHID(initArgs);
}

int ControllerHID::ParseAxis(const std::string& axis, HID_AXIS& axis_type)
{
    if (axis == "X" || axis == "x")
    {
        axis_type = HID_AXIS::HID_X_AXIS;
    }
    else if (axis == "Y" || axis == "y")
    {
        axis_type = HID_AXIS::HID_Y_AXIS;
    }
    else if (axis == "Z" || axis == "z")
    {
        axis_type = HID_AXIS::HID_Z_AXIS;
    }
    else if (axis == "R" || axis == "r")
    {
        axis_type = HID_AXIS::HID_R_AXIS;
    }
    else if (axis == "U" || axis == "u")
    {
        axis_type = HID_AXIS::HID_U_AXIS;
    }
    else if (axis == "V" || axis == "v")
    {
        axis_type = HID_AXIS::HID_V_AXIS;
    }
    else
    {
        LOG_ERROR("Invalid axis type: {}", axis);
        axis_type = HID_AXIS::HID_NR_OF_AXIS;
        return -1;
    }
    return 0;
}

ControllerHID::ControllerHID(InitArgs* args)
    : Controller(args),
      steering_(0.0),
      throttle_(0.0),
      steering_rate_(4.0),
      device_id_(0),
      device_id_internal_(-1)
{
    if (args && args->properties)
    {
        if (args->properties->ValueExists("steeringRate"))
        {
            steering_rate_ = strtod(args->properties->GetValueStr("steeringRate"));
        }

        if (args->properties->ValueExists("deviceID"))
        {
            device_id_ = strtoi(args->properties->GetValueStr("deviceID"));
        }

        if (args->properties->ValueExists("steeringAxis"))
        {
            std::string axis = args->properties->GetValueStr("steeringAxis");
            ParseAxis(args->properties->GetValueStr("steeringAxis"), steering_axis_);
        }

        if (args->properties->ValueExists("throttleAxis"))
        {
            std::string axis = args->properties->GetValueStr("throttleAxis");
            ParseAxis(args->properties->GetValueStr("throttleAxis"), throttle_axis_);
        }
    }
    align_to_road_heading_on_deactivation_ = true;
    align_to_road_heading_on_activation_   = true;
}

ControllerHID::~ControllerHID()
{
    CloseHID();
}

void ControllerHID::Init()
{
    OpenHID(device_id_);
    Controller::Init();
}

void ControllerHID::Step(double timeStep)
{
    if (ReadHID(throttle_, steering_) != 0)
    {
        return;
    }

    // LOG_DEBUG("steering: {:.2f} throttle: {:.2f}  ", steering_, throttle_);

    vehicle_.SetMaxSpeed(object_->GetMaxSpeed());

    // Update vehicle motion
    vehicle_.DrivingControlAnalog(timeStep, throttle_, steering_);

    gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);

    // Fetch Z and Pitch from OpenDRIVE position
    vehicle_.posZ_  = object_->pos_.GetZRoad();
    vehicle_.pitch_ = object_->pos_.GetPRoad();

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)))
    {
        gateway_->updateObjectSpeed(object_->id_, 0.0, vehicle_.speed_);
    }

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        gateway_->updateObjectWheelAngle(object_->id_, 0.0, vehicle_.wheelAngle_);
    }

    Controller::Step(timeStep);
}

int ControllerHID::Activate(ControlActivationMode lat_activation_mode,
                            ControlActivationMode long_activation_mode,
                            ControlActivationMode light_activation_mode,
                            ControlActivationMode anim_activation_mode)
{
    if (object_)
    {
        vehicle_.Reset();
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
        vehicle_.speed_ = object_->GetSpeed();
        vehicle_.SetMaxAcc(object_->GetMaxAcceleration());
        vehicle_.SetMaxDec(object_->GetMaxDeceleration());
        vehicle_.SetSteeringRate(steering_rate_);
    }

    object_->SetJunctionSelectorStrategy(roadmanager::Junction::JunctionStrategyType::SELECTOR_ANGLE);
    object_->SetJunctionSelectorAngle(0.0);

    return Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);
}

void ControllerHID::Deactivate()
{
    CloseHID();
    Controller::Deactivate();
}

#ifdef _WIN32

int ControllerHID::OpenHID(int device_id)
{
    UINT numDevs = joyGetNumDevs();
    if (numDevs == 0)
    {
        LOG_ERROR("No joystick devices available");
        return -1;
    }

    joy_info_         = {};
    joy_info_.dwSize  = sizeof(JOYINFOEX);
    joy_info_.dwFlags = JOY_RETURNALL;

    device_id_internal_ = JOYSTICKID1 + device_id;

    MMRESULT res = joyGetPosEx(static_cast<unsigned int>(JOYSTICKID1 + device_id), &joy_info_);
    if (res != JOYERR_NOERROR)
    {
        LOG_ERROR("Joystick with device id {} not ready or not connected", device_id_internal_);
        device_id_internal_ = -1;  // Reset fd
        return -1;
    }
    else
    {
        LOG_INFO("Opened joystick device id {}", device_id_internal_);
    }

    return 0;
}

int ControllerHID::ReadHID(double& throttle, double& steering)
{
    if (device_id_internal_ < 0)
    {
        return -1;  // Device not opened or invalid
    }

    MMRESULT res = joyGetPosEx(device_id_internal_, &joy_info_);

    DWORD axis_values[static_cast<unsigned int>(HID_AXIS::HID_NR_OF_AXIS)] =
        {joy_info_.dwRpos, joy_info_.dwUpos, joy_info_.dwVpos, joy_info_.dwXpos, joy_info_.dwYpos, joy_info_.dwZpos};

    if (res == JOYERR_NOERROR)
    {
        steering = 1.0 - static_cast<double>(axis_values[static_cast<unsigned int>(steering_axis_)]) / 32768.0;  // Normalize to [-1, 1]
        throttle = 1.0 - static_cast<double>(axis_values[static_cast<unsigned int>(throttle_axis_)]) / 32768.0;  // Normalize to [-1, 1]

        LOG_DEBUG("R: {} U: {} V: {} X: {} Y: {} Z: {} Buttons: {} -> steering: {:.2f} throttle: {:.2f}  ",
                  joy_info_.dwRpos,
                  joy_info_.dwUpos,
                  joy_info_.dwVpos,
                  joy_info_.dwXpos,
                  joy_info_.dwYpos,
                  joy_info_.dwZpos,
                  joy_info_.dwButtons,
                  steering,
                  throttle);
    }
    else
    {
        LOG_ERROR("Error reading joystick data: %d", res);
        return -1;
    }

    return 0;
}

void ControllerHID::CloseHID()
{
}

#else

int ControllerHID::OpenHID(int device_id)
{
    std::string joystick_path;

    joystick_path = "/dev/input/js" + std::to_string(device_id);

    // Try to open the device file
    device_id_internal_ = open(joystick_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (device_id_internal_ < 0)
    {
        // If opening fails, it likely means the device doesn't exist
        // or we don't have permissions, so continue to the next.
        LOG_ERROR("Joystick with device id {} not ready or not connected", device_id);
        return -1;
    }

    // Device opened successfully. Now query its capabilities.
    char name[128];
    if (ioctl(device_id_internal_, JSIOCGNAME(sizeof(name)), name) < 0)
    {
        // Fallback if name cannot be read (e.g., older kernel or device)
        snprintf(name, sizeof(name), "Unknown Joystick %d", device_id);
    }

    int num_axes = 0;
    ioctl(device_id_internal_, JSIOCGAXES, &num_axes);  // Get number of axes

    int num_buttons = 0;
    ioctl(device_id_internal_, JSIOCGBUTTONS, &num_buttons);  // Get number of buttons

    // Consider it a "real" joystick if it has at least one axis or one button
    if (num_axes > 0 || num_buttons > 0)
    {
        LOG_INFO("Opened {} Axis: {} Buttons: {}", joystick_path + " " + name, num_axes);
    }
    else
    {
        // If it has no axes or buttons, it's not a joystick for our purpose
        LOG_ERROR("Skipping {} (no axes/buttons detected).", joystick_path + " " + name);
        close(device_id_internal_);  // Close this non-joystick device
        device_id_internal_ = -1;    // Reset fd
        return -1;
    }

    return 0;
}

int ControllerHID::ReadHID(double& throttle, double& steering)
{
    if (device_id_internal_ < 0)
    {
        return -1;  // Device not opened or invalid
    }

    struct js_event js_event;
    if (read(device_id_internal_, &js_event, sizeof(struct js_event)) == static_cast<ssize_t>(sizeof(struct js_event)))
    {
        // Mask out JS_EVENT_INIT (initial state event)
        switch (js_event.type & ~JS_EVENT_INIT)
        {
            case JS_EVENT_BUTTON:
                LOG_DEBUG("Button {} {}", js_event.number, js_event.value ? "pressed" : "released");
                break;
            case JS_EVENT_AXIS:
                LOG_DEBUG("Axis {} value: {}", js_event.number, js_event.value);

                // hard code axis
                if (js_event.number == 0)
                {
                    steering = -js_event.value / 32768.0;  // Normalize to [-1, 1]
                }
                else if (js_event.number == 1)
                {
                    throttle = 1 - (js_event.value + 32768) / 65536.0;  // Normalize to [0, 1]
                }
                else if (js_event.number == 2)
                {
                    throttle = -1 + (js_event.value + 32768) / 65536.0;  // Normalize to [-1, 0]
                }

                LOG_DEBUG("In ReadHID steering: {:.2f} throttle: {:.2f}  ", steering, throttle);

                // signed short axis_values[static_cast<unsigned int>(HID_AXIS::HID_NR_OF_AXIS)];
                // if (js_event.number < static_cast<unsigned char>(HID_AXIS::HID_NR_OF_AXIS))
                // {
                //     LOG_INFO("num {} value {}", js_event.number, js_event.value);
                //     axis_values[js_event.number] = js_event.value;
                // }

                break;
        }
    }
    return 0;
}

void ControllerHID::CloseHID()
{
    close(device_id_internal_);
}

#endif
