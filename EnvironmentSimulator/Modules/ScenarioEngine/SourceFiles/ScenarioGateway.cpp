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

#include "ScenarioGateway.hpp"
#include "CommonMini.hpp"
#include "Replay.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#else
 /* Assume that any non-Windows platform uses POSIX-style sockets instead. */
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
#include <unistd.h> /* Needed for close() */
#endif

using namespace scenarioengine;

ObjectState::ObjectState()
{
	memset(&state_, 0, sizeof(ObjectState));
	state_.info.id = -1;
}


ObjectState::ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type,	OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position* pos) : dirty_(0)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.info.id = id;
	state_.info.obj_type = obj_type;
	state_.info.obj_category = obj_category;
	state_.info.model_id = model_id;
	state_.info.ctrl_type = ctrl_type;
	state_.info.timeStamp = (float)timestamp;
	strncpy(state_.info.name, name.c_str(), NAME_LEN);
	state_.pos = *pos;
	state_.info.speed = (float)speed;
	state_.info.wheel_angle = (float)wheel_angle;
	state_.info.wheel_rot = (float)wheel_rot;
	state_.info.boundingbox = boundingbox;
	state_.info.scaleMode = scaleMode;

	dirty_ =
		Object::DirtyBit::LONGITUDINAL |
		Object::DirtyBit::LATERAL |
		Object::DirtyBit::SPEED |
		Object::DirtyBit::WHEEL_ANGLE |
		Object::DirtyBit::WHEEL_ROTATION;
}

ObjectState::ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, double x, double y, double z, double h, double p, double r) : dirty_(0)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.info.id = id;
	state_.info.obj_type = obj_type;
	state_.info.obj_category = obj_category;
	state_.info.model_id = model_id;
	state_.info.ctrl_type = ctrl_type;
	state_.info.name[0] = 0;
	state_.info.timeStamp = (float)timestamp;
	strncpy(state_.info.name, name.c_str(), NAME_LEN);
	state_.pos.Init();
	state_.pos.SetInertiaPos(x, y, z, h, p, r);
	state_.info.speed = (float)speed;
	state_.info.wheel_angle = (float)wheel_angle;
	state_.info.wheel_rot = (float)wheel_rot;
	state_.info.boundingbox = boundingbox;
	state_.info.scaleMode = scaleMode;

	dirty_ =
		Object::DirtyBit::LONGITUDINAL |
		Object::DirtyBit::LATERAL |
		Object::DirtyBit::SPEED |
		Object::DirtyBit::WHEEL_ANGLE |
		Object::DirtyBit::WHEEL_ROTATION;
}

ObjectState::ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, int laneId, double laneOffset, double s) : dirty_(0)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.info.id = id;
	state_.info.obj_type = obj_type;
	state_.info.obj_category = obj_category;
	state_.info.model_id = model_id;
	state_.info.ctrl_type = ctrl_type;
	state_.info.timeStamp = (float)timestamp;
	strncpy(state_.info.name, name.c_str(), NAME_LEN);
	state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
	state_.info.speed = (float)speed;
	state_.info.wheel_angle = (float)wheel_angle;
	state_.info.wheel_rot = (float)wheel_rot;
	state_.info.boundingbox = boundingbox;
	state_.info.scaleMode = scaleMode;

	dirty_ =
		Object::DirtyBit::LONGITUDINAL |
		Object::DirtyBit::LATERAL |
		Object::DirtyBit::SPEED |
		Object::DirtyBit::WHEEL_ANGLE |
		Object::DirtyBit::WHEEL_ROTATION;
}

ObjectState::ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, double lateralOffset, double s)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.info.id = id;
	state_.info.obj_type = obj_type;
	state_.info.obj_category = obj_category;
	state_.info.model_id = model_id;
	state_.info.ctrl_type = ctrl_type;
	state_.info.timeStamp = (float)timestamp;
	strncpy(state_.info.name, name.c_str(), NAME_LEN);
	state_.pos.SetTrackPos(roadId, s, lateralOffset);
	state_.info.speed = (float)speed;
	state_.info.wheel_angle = (float)wheel_angle;
	state_.info.wheel_rot = (float)wheel_rot;
	state_.info.boundingbox = boundingbox;
	state_.info.scaleMode = scaleMode;

	dirty_ =
		Object::DirtyBit::LONGITUDINAL |
		Object::DirtyBit::LATERAL |
		Object::DirtyBit::SPEED |
		Object::DirtyBit::WHEEL_ANGLE |
		Object::DirtyBit::WHEEL_ROTATION;

}

void ObjectState::Print()
{
	LOG("state: \n\tid %d\n\tname %s\n\tmodel_id: %d\n\tctrl_type: %d\n\ttime %.2f\n\tx %.2f\n\ty %.2f\n\th %.2f\n\tspeed %.2f\twheel_angle %.2f type %d category %d",
		state_.info.id,
		state_.info.name,
		state_.info.model_id,
		state_.info.ctrl_type,
		state_.info.timeStamp,
		state_.pos.GetX(),
		state_.pos.GetY(),
		state_.pos.GetZ(),
		state_.info.speed,
		state_.info.wheel_angle,
		state_.info.obj_type,
		state_.info.obj_category
	);
	LOG("state: \n\tbounding box: \ncenter: x: %.2f, y: %.2f, z: %.2f\n\tdimensions: width: %.2f, length: %.2f, height: %.2f scaleMode: %d",
    state_.info.boundingbox.center_.x_,
		state_.info.boundingbox.center_.y_,
		state_.info.boundingbox.center_.z_,
		state_.info.boundingbox.dimensions_.width_,
		state_.info.boundingbox.dimensions_.length_,
		state_.info.boundingbox.dimensions_.height_,
		state_.info.scaleMode
	);
}

// ScenarioGateway

ScenarioGateway::ScenarioGateway()
{
}

ScenarioGateway::~ScenarioGateway()
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		delete objectState_[i];
	}
	objectState_.clear();

	data_file_.flush();
	data_file_.close();
}

ObjectState* ScenarioGateway::getObjectStatePtrById(int id)
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		if (objectState_[i]->state_.info.id == id)
		{
			return objectState_[i];
		}
	}

	return 0;
}

int ScenarioGateway::getObjectStateById(int id, ObjectState& objectState)
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		if (objectState_[i]->state_.info.id == id)
		{
			objectState = *objectState_[i];
			return 0;
		}
	}

	// Indicate not found by returning non zero
	return -1;
}

int ScenarioGateway::updateObjectInfo(ObjectState* obj_state, double timestamp, double speed, double wheel_angle, double wheel_rot)
{
	if (!obj_state)
	{
		return -1;
	}

	obj_state->state_.info.speed = (float)speed;
	obj_state->state_.info.timeStamp = (float)timestamp;
	obj_state->state_.info.wheel_angle = (float)wheel_angle;
	obj_state->state_.info.wheel_rot = (float)wheel_rot;

	obj_state->dirty_ |=
		Object::DirtyBit::SPEED |
		Object::DirtyBit::WHEEL_ANGLE |
		Object::DirtyBit::WHEEL_ROTATION;

	return 0;
}

int ScenarioGateway::reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
	roadmanager::Position* pos)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		obj_state = new ObjectState(id, name,obj_type,obj_category, model_id, ctrl_type, boundingbox,
			scaleMode, timestamp, speed, wheel_angle, wheel_rot, pos);

		// Specify lanes relevant to the object (will snap to them)
		obj_state->state_.pos.SetSnapLaneTypes(roadmanager::Lane::LaneType::LANE_TYPE_ANY_DRIVING);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos = *pos;
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;

		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}

	return 0;
}

int ScenarioGateway::reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
	double x, double y, double z, double h, double p, double r)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, obj_type, obj_category, model_id, ctrl_type, boundingbox, scaleMode,
			timestamp, speed, wheel_angle, wheel_rot, x, y, z, h, p, r);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetInertiaPos(x, y, z, h, p, r);
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;

		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}

	return 0;
}

int ScenarioGateway::reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
	double x, double y, double h)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, obj_type, obj_category, model_id, ctrl_type, boundingbox, scaleMode,
			timestamp, speed, wheel_angle, wheel_rot, x, y, 0, h, 0, 0);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetInertiaPos(x, y, h);
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;

		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}

	return 0;
}

int ScenarioGateway::reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
	int roadId, int laneId, double laneOffset, double s)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, obj_type, obj_category, model_id, ctrl_type, boundingbox, scaleMode,
			timestamp, speed, wheel_angle, wheel_rot, roadId, laneId, laneOffset, s);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;

		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}

	return 0;
}

int ScenarioGateway::reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int ctrl_type, OSCBoundingBox boundingbox,
	int scaleMode, double timestamp, double speed, double wheel_angle, double wheel_rot,
	int roadId, double lateralOffset, double s)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, obj_type, obj_category, model_id, ctrl_type, boundingbox, scaleMode,
			timestamp, speed, wheel_angle, wheel_rot, roadId, lateralOffset, s);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetTrackPos(roadId, s, lateralOffset);
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;

		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}

	return 0;
}

int ScenarioGateway::updateObjectPos(int id, double timestamp, roadmanager::Position* pos)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Object id: %d must be reported before updated", id);
		return -1;
	}
	else
	{
		// Update status
		obj_state->state_.pos = *pos;
		obj_state->state_.info.timeStamp = (float)timestamp;
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;
	}

	return 0;
}

int ScenarioGateway::updateObjectRoadPos(int id, double timestamp, int roadId, double lateralOffset, double s)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Object id: %d must be reported before updated", id);
		return -1;
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetTrackPos(roadId, s, lateralOffset);
		obj_state->state_.info.timeStamp = (float)timestamp;
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;
	}

	return 0;
}

int ScenarioGateway::updateObjectLanePos(int id, double timestamp, int roadId, int laneId, double offset, double s)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Object id: %d must be reported before updated", id);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetLanePos(roadId, laneId, s, offset);
		obj_state->state_.info.timeStamp = (float)timestamp;
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;
	}

	return 0;
}

int ScenarioGateway::updateObjectWorldPosXYH(int id, double timestamp, double x, double y, double h)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Object id: %d must be reported before updated", id);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetInertiaPos(x, y, h);
		obj_state->state_.info.timeStamp = (float)timestamp;
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;
	}

	return 0;
}

int ScenarioGateway::updateObjectWorldPos(int id, double timestamp, double x, double y, double z, double h, double p, double r)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Object id: %d must be reported before updated", id);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetInertiaPos(x, y, z, h, p, r);
		obj_state->state_.info.timeStamp = (float)timestamp;
		obj_state->dirty_ |=
			Object::DirtyBit::LONGITUDINAL |
			Object::DirtyBit::LATERAL;
	}

	return 0;
}

int ScenarioGateway::updateObjectSpeed(int id, double timestamp, double speed)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == nullptr)
	{
		LOG_ONCE("Can't set speed for object %d yet. Please register object using reportObject() first.", id);
		return -1;
	}

	obj_state->state_.info.speed = (float)speed;
	obj_state->dirty_ |= Object::DirtyBit::SPEED;

	return 0;
}

int ScenarioGateway::updateObjectVel(int id, double timestamp, double x_vel, double y_vel, double z_vel)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == nullptr)
	{
		LOG_ONCE("Can't set velocity for object %d yet. Please register object using reportObject() first.", id);
		return -1;
	}

	obj_state->state_.pos.SetVel(x_vel, y_vel, z_vel);
	obj_state->dirty_ |= Object::DirtyBit::VELOCITY;

	return 0;
}

int ScenarioGateway::updateObjectAcc(int id, double timestamp, double x_acc, double y_acc, double z_acc)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == nullptr)
	{
		LOG_ONCE("Can't set acceleration for object %d yet. Please register object using reportObject() first.", id);
		return -1;
	}

	obj_state->state_.pos.SetAcc(x_acc, y_acc, z_acc);
	obj_state->dirty_ |= Object::DirtyBit::ACCELERATION;

	return 0;
}

int ScenarioGateway::updateObjectAngularVel(int id, double timestamp, double h_rate, double p_rate, double r_rate)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == nullptr)
	{
		LOG_ONCE("Can't set angular velocity for object %d yet. Please register object using reportObject() first.", id);
		return -1;
	}

	obj_state->state_.pos.SetAngularVel(h_rate, p_rate, r_rate);
	obj_state->dirty_ |= Object::DirtyBit::ANGULAR_RATE;

	return 0;
}

int ScenarioGateway::updateObjectAngularAcc(int id, double timestamp, double h_acc, double p_acc, double r_acc)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == nullptr)
	{
		LOG_ONCE("Can't set angular acceleration for object %d yet. Please register object using reportObject() first.", id);
		return -1;
	}

	obj_state->state_.pos.SetAngularAcc(h_acc, p_acc, r_acc);
	obj_state->dirty_ |= Object::DirtyBit::ANGULAR_ACC;

	return 0;
}

int ScenarioGateway::updateObjectWheelAngle(int id, double timestamp, double wheelAngle)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == nullptr)
	{
		LOG_ONCE("Can't set wheel angle for object %d yet. Please register object using reportObject() first.", id);
		return -1;
	}

	obj_state->state_.info.wheel_angle = (float)wheelAngle;
	obj_state->dirty_ |= Object::DirtyBit::WHEEL_ANGLE;

	return 0;
}

int ScenarioGateway::updateObjectWheelRotation(int id, double timestamp, double wheelRotation)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == nullptr)
	{
		LOG_ONCE("Can't set wheel rotation for object %d yet. Please register object using reportObject() first.", id);
		return -1;
	}

	obj_state->state_.info.wheel_rot = (float)wheelRotation;
	obj_state->dirty_ |= Object::DirtyBit::WHEEL_ROTATION;

	return 0;
}

bool ScenarioGateway::isObjectReported(int id)
{
	return getObjectStatePtrById(id) != nullptr;
}

void ScenarioGateway::clearDirtyBits()
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		objectState_[i]->clearDirtyBits();
	}
}

void ScenarioGateway::removeObject(int id)
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		if (objectState_[i]->state_.info.id == id)
		{
			objectState_.erase(objectState_.begin() + i);
		}
	}
}

void ScenarioGateway::removeObject(std::string name)
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		if (objectState_[i]->state_.info.name == name)
		{
			objectState_.erase(objectState_.begin() + i);
		}
	}
}

void ScenarioGateway::WriteStatesToFile()
{
	if (data_file_.is_open())
	{
		// Write status to file - for later replay
		for (size_t i = 0; i < objectState_.size(); i++)
		{
			struct ObjectStateStructDat datState;

			datState.info = objectState_[i]->state_.info;
			datState.pos.x = (float)objectState_[i]->state_.pos.GetX();
			datState.pos.y = (float)objectState_[i]->state_.pos.GetY();
			datState.pos.z = (float)objectState_[i]->state_.pos.GetZ();
			datState.pos.h = (float)objectState_[i]->state_.pos.GetH();
			datState.pos.p = (float)objectState_[i]->state_.pos.GetP();
			datState.pos.r = (float)objectState_[i]->state_.pos.GetR();
			datState.pos.roadId = objectState_[i]->state_.pos.GetTrackId();
			datState.pos.laneId = objectState_[i]->state_.pos.GetLaneId();
			datState.pos.offset = (float)objectState_[i]->state_.pos.GetOffset();
			datState.pos.t = (float)objectState_[i]->state_.pos.GetT();
			datState.pos.s = (float)objectState_[i]->state_.pos.GetS();
			data_file_.write((char*)(&datState), sizeof(datState));
		}
	}
}

int ScenarioGateway::RecordToFile(std::string filename, std::string odr_filename, std::string  model_filename)
{
	if (!filename.empty())
	{
		data_file_.open(filename, std::ofstream::binary);
		if (data_file_.fail())
		{
			LOG("Cannot open file: %s", filename.c_str());
			return -1;
		}
		ReplayHeader header;
		header.version = DAT_FILE_FORMAT_VERSION;
		strncpy(header.odr_filename, odr_filename.c_str(), REPLAY_FILENAME_SIZE);
		strncpy(header.model_filename, model_filename.c_str(), REPLAY_FILENAME_SIZE);

		data_file_.write((char*)&header, sizeof(header));
	}

	return 0;
}
