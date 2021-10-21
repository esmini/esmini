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

#include "ControllerUDPDriverModel.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

#include <random>

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerUDPDriverModel(void* args)
{
	Controller::InitArgs* initArgs = (Controller::InitArgs*)args;

	return new ControllerUDPDriverModel(initArgs);
}

ControllerUDPDriverModel::ControllerUDPDriverModel(InitArgs* args) :
	inputMode_(InputMode::DRIVER_INPUT), udpServer_(nullptr), port_(0), Controller(args)
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
			LOG_AND_QUIT("ControllerExternalDriverModel unexpected arg inputMode %s",
				args->properties->GetValueStr("inputMode").c_str());
		}
	}
	if (args && args->properties && args->properties->ValueExists("port"))
	{
		port_ = strtoi(args->properties->GetValueStr("port"));
	}

	// currently only supporting override mode
	if (args && args->properties && args->properties->ValueExists("mode"))
	{
		if (args->properties->GetValueStr("intputMode") == "additive")
		{
			LOG("ExternalDriverModelController only support override mode, ignoring requested additive mode");
		}
	}
	mode_ = Mode::MODE_OVERRIDE;
}

ControllerUDPDriverModel::~ControllerUDPDriverModel()
{
	if (udpServer_ != nullptr)
	{
		delete udpServer_;
	}
}

std::string ControllerUDPDriverModel::InputMode2Str(InputMode inputMode)
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

void ControllerUDPDriverModel::Init()
{
	Controller::Init();
}

void ControllerUDPDriverModel::Step(double timeStep)
{
	DMMessage msg;

	int retval = udpServer_->Receive((char*)&msg, sizeof(msg));

	if (retval == -1)
	{
		return;
	}

	//printf("version %d objectId %d framenr %d inputMode %d/%s \n",
	//	msg.header.version,
	//	msg.header.objectId,
	//	msg.header.frameNumber,
	//	msg.header.inputMode,
	//	InputMode2Str(static_cast<InputMode>(msg.header.inputMode)).c_str());

	if (msg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_XYZHPR))
	{
		roadmanager::Position* pos = &gateway_->getObjectStatePtrById(msg.header.objectId)->state_.pos;
		pos->SetAlignModeZ(roadmanager::Position::ALIGN_MODE::ALIGN_NONE);
		pos->SetAlignModeP(roadmanager::Position::ALIGN_MODE::ALIGN_NONE);

		// Update object state via gateway
		gateway_->updateObjectWorldPos(object_->id_, 0.0, msg.message.stateXYZHPR.x, msg.message.stateXYZHPR.y, msg.message.stateXYZHPR.z,
			msg.message.stateXYZHPR.h, msg.message.stateXYZHPR.p, msg.message.stateXYZHPR.r);
		gateway_->updateObjectSpeed(object_->id_, 0.0, msg.message.stateXYZHPR.speed);
		gateway_->updateObjectWheelAngle(object_->id_, 0.0, msg.message.stateXYZHPR.wheelAngle);

		// Also update the internal vehicle model, in case next message is driver input
		vehicle_.SetPos(msg.message.stateXYZHPR.x, msg.message.stateXYZHPR.y, msg.message.stateXYZHPR.z, msg.message.stateXYZHPR.h);
	}
	else if (msg.header.inputMode == static_cast<int>(InputMode::VEHICLE_STATE_XYH))
	{
		roadmanager::Position* pos = &gateway_->getObjectStatePtrById(msg.header.objectId)->state_.pos;
		pos->SetAlignModeZ(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
		pos->SetAlignModeP(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);

		// Update object state via gateway
		gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, msg.message.stateXYH.x, msg.message.stateXYH.y, msg.message.stateXYH.h);
		gateway_->updateObjectSpeed(object_->id_, 0.0, msg.message.stateXYH.speed);
		gateway_->updateObjectWheelAngle(object_->id_, 0.0, msg.message.stateXYH.wheelAngle);

		// Also update the internal vehicle model, in case next message is driver input
		vehicle_.SetPos(msg.message.stateXYH.x, msg.message.stateXYH.y, pos->GetZ(), msg.message.stateXYH.h);

		// Fetch Pitch from OpenDRIVE position
		vehicle_.SetPitch(pos->GetP());
	}
	else if (msg.header.inputMode == static_cast<int>(InputMode::DRIVER_INPUT))
	{
		roadmanager::Position* pos = &gateway_->getObjectStatePtrById(msg.header.objectId)->state_.pos;
		pos->SetAlignModeZ(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
		pos->SetAlignModeP(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);

		// Update vehicle input
		vehicle_.DrivingControlAnalog(timeStep, msg.message.driverInput.throttle - msg.message.driverInput.brake, msg.message.driverInput.steeringAngle);

		// Register updated vehicle position
		gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
		gateway_->updateObjectSpeed(object_->id_, 0.0, vehicle_.speed_);
		gateway_->updateObjectWheelAngle(object_->id_, 0.0, msg.message.driverInput.steeringAngle);

		// Fetch Z and Pitch from OpenDRIVE position
		vehicle_.SetZ(pos->GetZ());
		vehicle_.SetPitch(pos->GetP());
	}
	else
	{
		LOG("ControllerExternalDriverModel received %d bytes and unexpected input mode %d", retval, msg.header.inputMode);
	}

	Controller::Step(timeStep);
}

void ControllerUDPDriverModel::Activate(ControlDomains domainMask)
{
	if (object_)
	{
		if (port_ == 0)   // port not specified, assign default
		{
			port_ = DEFAULT_DRIVER_MODEL_PORT + object_->GetId();
		}

		if (udpServer_ == nullptr ||  // not created yet
			(udpServer_ != nullptr && udpServer_->GetPort() != port_)) // port nr changed. Need to recreate the socket.
		{
			// Close socket in case the controller is assigned again with different port
			if (udpServer_ != nullptr)
			{
				delete udpServer_;
			}
			udpServer_ = new UDPServer((unsigned short)port_);
			LOG("ExternalDriverModel server listening on port %d", port_);
		}

		vehicle_.Reset();
		vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
		vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
		vehicle_.speed_ = object_->GetSpeed();
	}

	steer = vehicle::STEERING_NONE;
	accelerate = vehicle::THROTTLE_NONE;

	Controller::Activate(domainMask);
}

void ControllerUDPDriverModel::ReportKeyEvent(int key, bool down)
{
}
