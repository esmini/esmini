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

using namespace scenarioengine;


ObjectState::ObjectState()
{
	memset(&state_, 0, sizeof(ObjectState));
	state_.id = -1;
}

#if 0
ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, roadmanager::Position *pos, double speed, double wheel_angle, int ghost_id)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos = *pos;
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.ghost_id = ghost_id;
}

ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, double x, double y, double z, double h, double p, double r, double speed, double wheel_angle, int ghost_id)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.SetInertiaPos(x, y, z, h, p, r);
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.ghost_id = ghost_id;
}

ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, int roadId, int laneId, double laneOffset, double s, double speed, double wheel_angle, int ghost_id)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.ghost_id = ghost_id;
}
#else
ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, roadmanager::Position *pos, double speed, double wheel_angle)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos = *pos;
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
}

ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, double x, double y, double z, double h, double p, double r, double speed, double wheel_angle)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.SetInertiaPos(x, y, z, h, p, r);
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
}

ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, int roadId, int laneId, double laneOffset, double s, double speed, double wheel_angle)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
}
#endif
void ObjectState::Print()
{
	LOG("state: \n\tid %d\n\tname %s\n\tmodel_id: %d\n\tcontrol: %d\n\ttime %.2f\n\tx %.2f\n\ty %.2f\n\th %.2f\n\tspeed %.2f\twheel_angle %.2f",
		state_.id,
		state_.name,
		state_.model_id,
		state_.control,
		state_.timeStamp,
		state_.pos.GetX(),
		state_.pos.GetY(),
		state_.pos.GetZ(),
		state_.speed,
		state_.wheel_angle
	);
}

// ScenarioGateway

ScenarioGateway::ScenarioGateway()
{
	objectState_.clear();
}

ScenarioGateway::~ScenarioGateway()
{
	for (size_t i=0; i<objectState_.size(); i++)
	{
		delete objectState_[i];
	}
	objectState_.clear();

	data_file_.flush();
	data_file_.close();
}


int ScenarioGateway::getObjectStateById(int id, ObjectState &objectState)
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		if (objectState_[i]->state_.id == id)
		{
			objectState = *objectState_[i];
			return 0;
		}
	}

	// Indicate not found by returning non zero
	return -1;

}

void ScenarioGateway::reportObject(ObjectState objectState, bool update)
{
	bool found = false;
	
	// Check whether the object is already present in the list of active objects
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		if (objectState_[i]->state_.id == objectState.state_.id)
		{
			found = true;
			
			if (update)
			{
				// Just update relevant fields
				objectState_[i]->state_.pos = objectState.state_.pos;
				objectState_[i]->state_.speed = objectState.state_.speed;
				objectState_[i]->state_.timeStamp= objectState.state_.timeStamp;
				objectState_[i]->state_.wheel_angle = objectState.state_.wheel_angle;
			}
			else
			{
				// Copy all fields
				objectState_[i]->state_ = objectState.state_;
			}
			break;
		}
	}

	if (!found)
	{
		// Add object
		LOG("Adding %s state: (%d, %.2f)", objectState.state_.name, objectState.state_.id, objectState.state_.timeStamp);
		ObjectState *os = new ObjectState;
		*os = objectState;
		objectState_.push_back(os);
	}

	// Write status to file - for later replay
	if (data_file_.is_open())
	{
		data_file_.write((char*)&objectState, sizeof(objectState.state_));
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
		strncpy(header.odr_filename, FileNameOf(odr_filename).c_str(), REPLAY_FILENAME_SIZE);
		strncpy(header.model_filename, FileNameOf(model_filename).c_str(), REPLAY_FILENAME_SIZE);

		data_file_.write((char*)&header, sizeof(header));
	}

	return 0;
}
