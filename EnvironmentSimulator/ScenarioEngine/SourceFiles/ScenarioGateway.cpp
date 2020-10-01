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

#include <utils/geom/PositionVector.h>
#include <libsumo/Simulation.h>
#include <libsumo/Vehicle.h>
#include <libsumo/TraCIDefs.h>

using namespace scenarioengine;

ObjectState::ObjectState()
{
	memset(&state_, 0, sizeof(ObjectState));
	state_.id = -1;
}


ObjectState::ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int control,\
 OSCBoundingBox boundingbox, double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position* pos)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.obj_type = obj_type;
	state_.obj_category = obj_category;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos = *pos;
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.wheel_rot = (float)wheel_rot;
	state_.boundingbox = boundingbox;
}

ObjectState::ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int control,\
OSCBoundingBox boundingbox, double timestamp, double speed, double wheel_angle, double wheel_rot, double x, double y, double z, double h, double p, double r)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.obj_type = obj_type;
	state_.obj_category = obj_category;
	state_.model_id = model_id;
	state_.control = control;
	state_.name[0] = 0;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.Init();
	state_.pos.SetInertiaPos(x, y, z, h, p, r);
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.wheel_rot = (float)wheel_rot;
	state_.boundingbox = boundingbox;
}

ObjectState::ObjectState(int id, std::string name, int obj_type, int obj_category, int model_id, int control,\
 OSCBoundingBox boundingbox, double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, int laneId, double laneOffset, double s)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.obj_type = obj_type;
	state_.obj_category = obj_category;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.wheel_rot = (float)wheel_rot;
	state_.boundingbox = boundingbox;
}

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
	LOG("state: \n\tbounding box: \ncenter: x: %.2f, y: %.2f, z: %.2f\n\tdimensions: width: %.2f, length: %.2f, height: %.2f",
    state_.boundingbox.center_.x_,
		state_.boundingbox.center_.y_,
		state_.boundingbox.center_.z_,
		state_.boundingbox.dimensions_.width_,
		state_.boundingbox.dimensions_.length_,
		state_.boundingbox.dimensions_.height_
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
		if (objectState_[i]->state_.id == id)
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
		if (objectState_[i]->state_.id == id)
		{
			objectState = *objectState_[i];
			return 0;
		}
	}

	// Indicate not found by returning non zero
	return -1;
}

void ScenarioGateway::updateObjectInfo(ObjectState* obj_state, double timestamp, double speed, double wheel_angle, double wheel_rot)
{
	if (!obj_state)
	{
		return;
	}

	obj_state->state_.speed = (float)speed;
	obj_state->state_.timeStamp = (float)timestamp;
	obj_state->state_.wheel_angle = (float)wheel_angle;
	obj_state->state_.wheel_rot = (float)wheel_rot;

	// Write status to file - for later replay
	if (data_file_.is_open())
	{
		data_file_.write((char*)(&obj_state->state_), sizeof(obj_state->state_));
	}
}

void ScenarioGateway::reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int control, OSCBoundingBox boundingbox,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	roadmanager::Position* pos)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name,obj_type,obj_category, model_id, control, boundingbox, timestamp, speed, wheel_angle, wheel_rot, pos);

		// Specify lanes relevant to the object (will snap to them)
		obj_state->state_.pos.SetSnapLaneTypes(roadmanager::Lane::LaneType::LANE_TYPE_ANY_DRIVING);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos = *pos;
		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}
}

void ScenarioGateway::reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int control, OSCBoundingBox boundingbox,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	double x, double y, double z, double h, double p, double r)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, obj_type, obj_category, model_id, control, boundingbox, timestamp, speed, wheel_angle, wheel_rot, x, y, z, h, p, r);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetInertiaPos(x, y, z, h, p, r);
		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}
}

void ScenarioGateway::reportObject(int id, std::string name, int obj_type, int obj_category, int model_id, int control, OSCBoundingBox boundingbox,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	int roadId, int laneId, double laneOffset, double s)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, obj_type, obj_category, model_id, control, boundingbox,timestamp, speed, wheel_angle, wheel_rot, roadId, laneId, laneOffset, s);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}
}

void ScenarioGateway::removeObject(int id)
{
	for (size_t i = 0; i < objectState_.size(); i++) 
	{
		if (objectState_[i]->state_.id == id) 
		{
			objectState_.erase(objectState_.begin() + i);
		}
	}
}

void ScenarioGateway::removeObject(std::string name)
{
	for (size_t i = 0; i < objectState_.size(); i++) 
	{
		if (objectState_[i]->state_.name == name) 
		{
			objectState_.erase(objectState_.begin() + i);
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
		strncpy(header.odr_filename, FileNameOf(odr_filename).c_str(), REPLAY_FILENAME_SIZE);
		strncpy(header.model_filename, FileNameOf(model_filename).c_str(), REPLAY_FILENAME_SIZE);

		data_file_.write((char*)&header, sizeof(header));
	}

	return 0;
}

SumoController::SumoController(Entities* entities, ScenarioGateway* scenarioGateway)
{
	// initalize sumo with the configuration file
	entities_ = entities;
	scenarioGateway_ = scenarioGateway;
	std::vector<std::string> options;

	options.push_back("-c " + entities_->sumo_config_path);
	options.push_back("--xml-validation");
	options.push_back("never");
	
	libsumo::Simulation::load(options);
	sumo_used = true;
}

SumoController::SumoController()
{
	sumo_used = false;
}

void SumoController::InitalizeObjects()
{	
	// Adds all vehicles added in openscenario to the sumosimulation
	if (sumo_used)
	{
		for (size_t j = 0; j < entities_->object_.size(); j++)
		{
			if (entities_->object_[j]->control_ != Object::Control::HYBRID_GHOST) 
			{
				libsumo::Vehicle::add(entities_->object_[j]->name_,"");
			}
		}
		updatePositions();
	}
}

void SumoController::updatePositions()
{
	// Updates all positions for non-sumo controlled vehicles
	if (sumo_used)
	{
		for (size_t i = 0; i < entities_->object_.size(); i++)
		{
			if ((entities_->object_[i]->control_ != Object::Control::SUMO) && (entities_->object_[i]->control_ != Object::Control::HYBRID_GHOST))
			{
				libsumo::Vehicle::moveToXY(entities_->object_[i]->name_,"random",0,entities_->object_[i]->pos_.GetX()+entities_->sumo_x_offset,entities_->object_[i]->pos_.GetY()+entities_->sumo_y_offset,entities_->object_[i]->pos_.GetH(),0);
				libsumo::Vehicle::setSpeed(entities_->object_[i]->name_,entities_->object_[i]->speed_);
			}
		}
	}
}

void SumoController::step(double time)
{
	// stepping funciton for sumo, adds/removes vehicles (based on sumo), 
	// updates all positions of vehicles in the simulation that are controlled by sumo
	if (sumo_used)
	{
		// do sumo timestep
		libsumo::Simulation::step(time);

		// check if any new cars has been added by sumo and add them to entities
		if (libsumo::Simulation::getDepartedNumber() > 0) {
			std::vector<std::string> deplist = libsumo::Simulation::getDepartedIDList();
			for (size_t i = 0; i < deplist.size(); i++)
			{
				if (!entities_->nameExists(deplist[i]))
				{
					Vehicle *vehicle = new Vehicle();
					// copy the default vehicle stuff here (add bounding box and so on)
					LOG("Adding new vehicle: %s",deplist[i].c_str());
					vehicle->name_ = deplist[i];
					vehicle->control_ = Object::Control::SUMO;
					vehicle->model_filepath_ = entities_->sumo_vehicle->model_filepath_;
					entities_->addObject(vehicle);
				}
			}
		}

		// check if any cars have been removed by sumo and remove them from scenarioGateway and entities
		if (libsumo::Simulation::getArrivedNumber() > 0) {
			std::vector<std::string> arrivelist = libsumo::Simulation::getArrivedIDList();
			for (size_t i = 0; i < arrivelist.size();i++)
			{
				for (size_t j = 0; j < entities_->object_.size(); j++)
				{
					if (arrivelist[i] == entities_->object_[j]->name_) {
						LOG("Removing vehicle: %s",arrivelist[i].c_str());
						entities_->removeObject(arrivelist[i]);
						scenarioGateway_->removeObject(arrivelist[i]);
					}
				}
			}
		}

		// Update the position of all cars controlled by sumo
		for (size_t i = 0; i < entities_->object_.size(); i++)
		{
			if (entities_->object_[i]->control_ == Object::Control::SUMO)
			{
				std::string sumoid = entities_->object_[i]->name_;
				libsumo::TraCIPosition pos = libsumo::Vehicle::getPosition3D(sumoid);
				entities_->object_[i]->speed_ = libsumo::Vehicle::getSpeed(sumoid);
				entities_->object_[i]->pos_.SetInertiaPos(pos.x-entities_->sumo_x_offset,pos.y-entities_->sumo_y_offset,pos.z,-libsumo::Vehicle::getAngle(sumoid)*3.14159265359/180+ 3.14159265359/2,libsumo::Vehicle::getSlope(sumoid)*3.14159265359/180,0);
			}
        }
	}

}