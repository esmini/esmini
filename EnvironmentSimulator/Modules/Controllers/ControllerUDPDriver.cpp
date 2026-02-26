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
 * This controller simulates a bad or dizzy driver by manipulating
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

#include "ControllerUDPDriver.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "logger.hpp"
#include "ScenarioEngine.hpp"

#include <random>

using namespace scenarioengine;

int ControllerUDPDriver::basePort_ = DEFAULT_UDP_DRIVER_PORT;

Controller* scenarioengine::InstantiateControllerUDPDriver(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerUDPDriver(initArgs);
}

ControllerUDPDriver::ControllerUDPDriver(InitArgs* args)
    : Controller(args),
      inputMode_(InputMode::DRIVER_INPUT),
      udpServer_(nullptr),
      port_(0),
      execMode_(ExecMode::EXEC_MODE_ASYNCHRONOUS)
{
    if (args && args->properties && args->properties->ValueExists("inputMode"))
    {
        if (args->properties->GetValueStr("inputMode") == "vehicleStateXYH")
        {
            inputMode_ = InputMode::VEHICLE_STATE_XYH;
        }
        else if (args->properties->GetValueStr("inputMode") == "vehicleStateXYZHPR")
        {
            inputMode_ = InputMode::VEHICLE_STATE_XYZHPR;
        }
        else if (args->properties->GetValueStr("inputMode") == "driverInput")
        {
            inputMode_ = InputMode::DRIVER_INPUT;
        }
        else
        {
            LOG_ERROR_AND_QUIT("ControllerExternalDriverModel unexpected arg inputMode {}", args->properties->GetValueStr("inputMode").c_str());
        }
    }

    if (args && args->properties && args->properties->ValueExists("port"))
    {
        int portTmp = strtoi(args->properties->GetValueStr("port"));
        if (portTmp < 0 || portTmp > 65535)
        {
            LOG_ERROR_AND_QUIT("Invlaid driver model port: {} (valid range is [0, 65535]", portTmp);
        }
        else
        {
            port_ = portTmp;
        }
    }

    if (args && args->properties && args->properties->ValueExists("basePort"))
    {
        int basePortTmp = strtoi(args->properties->GetValueStr("basePort"));
        if (basePortTmp < 0 || basePortTmp > 65535)
        {
            LOG_ERROR_AND_QUIT("Invlaid driver model basePort: {} (valid range is [0, 65535]", basePortTmp);
        }
        else if (basePortTmp != basePort_)
        {
            LOG_INFO("Changing base port to {} (from {})", basePortTmp, basePort_);
            basePort_ = basePortTmp;
        }
    }

    if (args && args->properties && args->properties->ValueExists("execMode"))
    {
        if (args->properties->GetValueStr("execMode") == "asynchronous")
        {
            execMode_ = ExecMode::EXEC_MODE_ASYNCHRONOUS;
        }
        else if (args->properties->GetValueStr("execMode") == "synchronous")
        {
            execMode_ = ExecMode::EXEC_MODE_SYNCHRONOUS;
        }
        else
        {
            LOG_ERROR_AND_QUIT("ControllerExternalDriverModel unexpected arg execMode {}", args->properties->GetValueStr("execMode"));
        }
    }

    // currently only supporting override mode
    if (args && args->properties && args->properties->ValueExists("mode"))
    {
        if (args->properties->GetValueStr("intputMode") == "additive")
        {
            LOG_WARN("ExternalDriverModelController only support override mode, ignoring requested additive mode");
        }
    }
    mode_ = ControlOperationMode::MODE_OVERRIDE;

    memset(static_cast<void*>(&msg), 0, sizeof(msg));
    memset(static_cast<void*>(&lastMsg), 0, sizeof(lastMsg));
}

ControllerUDPDriver::~ControllerUDPDriver()
{
    if (udpServer_ != nullptr)
    {
        delete udpServer_;
    }
}

std::string ControllerUDPDriver::InputMode2Str(InputMode inputMode)
{
    if (inputMode == InputMode::DRIVER_INPUT)
    {
        return "driverInput";
    }
    else if (inputMode == InputMode::VEHICLE_STATE_XYH)
    {
        return "vehicleStateXYH";
    }
    else if (inputMode == InputMode::VEHICLE_STATE_XYZHPR)
    {
        return "vehicleStateXYZHPR";
    }
    else
    {
        return "Unknown";
    }
}

std::string ControllerUDPDriver::ExecMode2Str(ExecMode execMode)
{
    if (execMode == ExecMode::EXEC_MODE_ASYNCHRONOUS)
    {
        return "Asynchronous";
    }
    else if (execMode == ExecMode::EXEC_MODE_SYNCHRONOUS)
    {
        return "Synchronous";
    }
    else
    {
        return "Unknown";
    }
}

void ControllerUDPDriver::Init()
{
    if (basePort_ == -1)
    {
        basePort_ = DEFAULT_UDP_DRIVER_PORT;
        LOG_WARN("ControllerUDPDriver: using default baseport {}", basePort_);
    }
    Controller::Init();
}

void ControllerUDPDriver::Step(double timeStep)
{
    int retval            = 0;
    int receivedNrOfBytes = 0;

    if (execMode_ == ExecMode::EXEC_MODE_ASYNCHRONOUS)
    {
        // Pick all queued messages - store only the last/latest
        while (retval >= 0)
        {
            retval = udpServer_->Receive(reinterpret_cast<char*>(&msg), sizeof(msg));
            if (retval > 0)
            {
                receivedNrOfBytes = retval;
            }
        }
    }
    else
    {
        retval = udpServer_->Receive(reinterpret_cast<char*>(&msg), sizeof(msg));
        if (retval >= 0)
        {
            receivedNrOfBytes = retval;
        }
    }

    Object*                obj = nullptr;
    roadmanager::Position* pos = nullptr;

    if (receivedNrOfBytes > 0)
    {
        // Message received, handle it
        lastMsg = msg;

        // printf("version %d objectId %d framenr %d inputMode %d/%s \n",
        //	msg.header.version,
        //	msg.header.objectId,
        //	msg.header.frameNumber,
        //	msg.header.inputMode,
        //	InputMode2Str(static_cast<InputMode>(msg.header.inputMode)).c_str());

        if (msg.header.version != UDP_DRIVER_MESSAGE_VERSION)
        {
            LOG_ERROR_ONCE("ControllerUDPDriver: Got unsupported msg version {} (only accepting version {})",
                           msg.header.version,
                           UDP_DRIVER_MESSAGE_VERSION);

            return;
        }

        obj = scenario_engine_->entities_.GetObjectById(static_cast<int>(msg.header.objectId));
        if (obj == nullptr)
        {
            LOG_ERROR("ControllerUDPDriver: Received message for unknown object id {}, ignoring", msg.header.objectId);
            return;
        }

        pos = &obj->pos_;

        if (msg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_XYZHPR))
        {
            pos->SetMode(roadmanager::Position::PosModeType::SET,
                         roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_ABS |
                             roadmanager::Position::PosMode::R_ABS);

            // Update object state via gateway
            pos->SetInertiaPos(msg.message.stateXYZHPR.x,
                               msg.message.stateXYZHPR.y,
                               msg.message.stateXYZHPR.z,
                               msg.message.stateXYZHPR.h,
                               msg.message.stateXYZHPR.p,
                               msg.message.stateXYZHPR.r);
            obj->SetSpeed(msg.message.stateXYZHPR.speed);
            obj->wheel_angle_ = msg.message.stateXYZHPR.wheelAngle;

            // Also update the internal vehicle model, in case next message is driver input
            vehicle_.SetPos(msg.message.stateXYZHPR.x, msg.message.stateXYZHPR.y, msg.message.stateXYZHPR.z, msg.message.stateXYZHPR.h);
        }
        else if (msg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_XYH))
        {
            pos->SetMode(roadmanager::Position::PosModeType::SET,
                         roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                             roadmanager::Position::PosMode::R_REL);

            // Update object state via gateway
            pos->SetInertiaPos(msg.message.stateXYH.x, msg.message.stateXYH.y, msg.message.stateXYH.h);
            obj->SetSpeed(msg.message.stateXYH.speed);
            obj->wheel_angle_ = msg.message.stateXYH.wheelAngle;

            // Also update the internal vehicle model, in case next message is driver input
            vehicle_.SetPos(msg.message.stateXYH.x, msg.message.stateXYH.y, pos->GetZ(), msg.message.stateXYH.h);

            // Fetch Pitch from OpenDRIVE position
            vehicle_.SetPitch(pos->GetP());
        }
        else if (msg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_H))
        {
            pos->SetMode(roadmanager::Position::PosModeType::SET,
                         roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                             roadmanager::Position::PosMode::R_REL);

            // Update object state via gateway
            pos->SetInertiaPos(pos->GetX(), pos->GetY(), msg.message.stateH.h);
            obj->SetSpeed(msg.message.stateH.speed);
            obj->wheel_angle_ = msg.message.stateH.wheelAngle;

            // Also update the internal vehicle model, in case next message is driver input
            vehicle_.SetPos(pos->GetX(), pos->GetY(), pos->GetZ(), msg.message.stateH.h);

            // Fetch Pitch from OpenDRIVE position
            vehicle_.SetPitch(pos->GetP());
        }
        else if (msg.header.inputMode == static_cast<int>(InputMode::DRIVER_INPUT))
        {
            pos->SetMode(roadmanager::Position::PosModeType::SET,
                         roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                             roadmanager::Position::PosMode::R_REL);
        }
        else
        {
            LOG_ERROR("ControllerExternalDriverModel received {} bytes and unexpected input mode {}", retval, msg.header.inputMode);
        }
    }

    if (timeStep > SMALL_NUMBER && object_ != nullptr)
    {
        if (
            // following states, apply dead reckoning always (when enabled)
            (lastMsg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_H) && lastMsg.message.stateH.deadReckon) ||
            // following states, only apply dead reckoning when no message has been received
            (receivedNrOfBytes == 0 &&
             ((lastMsg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_XYH) && lastMsg.message.stateXYH.deadReckon) ||
              (lastMsg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_XYZHPR) && lastMsg.message.stateXYH.deadReckon))))
        {
            // No message received, but dead reckoning enabled. Update position based on speed and heading
            double speed = 0.0;
            double h     = 0.0;

            if (lastMsg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_H))
            {
                speed = lastMsg.message.stateH.speed;
                h     = lastMsg.message.stateH.h;
            }
            else if (lastMsg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_XYH))
            {
                speed = lastMsg.message.stateXYH.speed;
                h     = lastMsg.message.stateXYH.h;
            }
            else if (lastMsg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_XYZHPR))
            {
                speed = lastMsg.message.stateXYZHPR.speed;
                h     = lastMsg.message.stateXYZHPR.h;
            }
            else
            {
                LOG_ERROR_AND_QUIT("Unexpected msg type {}", lastMsg.header.inputMode);
            }

            double ds = speed * timeStep;
            double dx = ds * cos(h);
            double dy = ds * sin(h);

            object_->pos_.SetInertiaPos(object_->pos_.GetX() + dx, object_->pos_.GetY() + dy, vehicle_.heading_);
        }
        else if (lastMsg.header.inputMode == static_cast<int>(InputMode::DRIVER_INPUT))
        {
            // In driver input mode the vehicle is updated continuously wrt latest input
            vehicle_.DrivingControlAnalog(timeStep,
                                          lastMsg.message.driverInput.throttle - lastMsg.message.driverInput.brake,
                                          lastMsg.message.driverInput.steeringAngle);

            // Register updated vehicle position
            object_->pos_.SetInertiaPos(vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
            object_->SetSpeed(vehicle_.speed_);
            object_->wheel_angle_ = vehicle_.wheelAngle_;

            // Fetch Z and Pitch from OpenDRIVE position
            vehicle_.SetZ(pos->GetZ());
            vehicle_.SetPitch(pos->GetP());
        }
    }

    Controller::Step(timeStep);
}

int ControllerUDPDriver::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    if (object_)
    {
        if (port_ == 0)  // port not specified, assign default
        {
            port_ = basePort_ + object_->GetId();
        }

        if (udpServer_ == nullptr ||         // not created yet
            udpServer_->GetPort() != port_)  // port nr changed. Need to recreate the socket.
        {
            // Close socket in case the controller is assigned again with different port
            if (udpServer_ != nullptr)
            {
                delete udpServer_;
            }
            if (execMode_ == ExecMode::EXEC_MODE_ASYNCHRONOUS)
            {
                udpServer_ = new UDPServer(static_cast<unsigned short>(port_), 1);
            }
            else
            {
                udpServer_ = new UDPServer(static_cast<unsigned short>(port_), UDP_SYNCHRONOUS_MODE_TIMEOUT_MS);
            }
            LOG_INFO("ExternalDriverModel server listening on port {} execMode: {}", port_, ExecMode2Str(execMode_));
        }

        vehicle_.Reset();
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
        vehicle_.speed_ = object_->GetSpeed();
        vehicle_.SetMaxAcc(20.0);
        vehicle_.SetMaxSpeed(30.0);
    }

    steer      = vehicle::STEERING_NONE;
    accelerate = vehicle::THROTTLE_NONE;

    return Controller::Activate(mode);
}

void ControllerUDPDriver::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}
