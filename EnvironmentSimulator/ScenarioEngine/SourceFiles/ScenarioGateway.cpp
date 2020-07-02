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
#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_version.pb.h"

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

#include <Simulation.h>
#include <Vehicle.h>
#include <TraCIDefs.h>

using namespace scenarioengine;


#define OSI_OUT_PORT 48198

static struct {
	osi3::SensorView *sv;
	std::vector<osi3::MovingObject*> mobj;
} mobj_osi_internal;

static struct {
	osi3::Lane *ln; 
} mlane_osi_internal; 

static OSISensorView osiSensorView;
static OSIRoadLane osiRoadLane; 
std::ofstream osi_file;

static int sendSocket;
static struct sockaddr_in recvAddr;


ObjectState::ObjectState()
{
	memset(&state_, 0, sizeof(ObjectState));
	state_.id = -1;
}


ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position* pos)
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
	state_.wheel_rot = (float)wheel_rot;
}

ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, double speed, double wheel_angle, double wheel_rot, double x, double y, double z, double h, double p, double r)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
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
}

ObjectState::ObjectState(int id, std::string name, int model_id, int control, double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, int laneId, double laneOffset, double s)
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
	state_.wheel_rot = (float)wheel_rot;
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
}

// ScenarioGateway

ScenarioGateway::ScenarioGateway()
{
	sendSocket = 0;

	objectState_.clear();

	mobj_osi_internal.sv = new osi3::SensorView();

	mobj_osi_internal.sv->mutable_version()->set_version_major(3);
	mobj_osi_internal.sv->mutable_version()->set_version_minor(0);
	mobj_osi_internal.sv->mutable_version()->set_version_patch(0);

	mobj_osi_internal.sv->mutable_timestamp()->set_seconds(0);
	mobj_osi_internal.sv->mutable_timestamp()->set_seconds(0);



	osi_file = std::ofstream("move_obj.osi", std::ios_base::binary);
	if (!osi_file.good())
	{
		LOG("Failed open osi_s file");
	}
}


ScenarioGateway::~ScenarioGateway()
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		delete objectState_[i];
	}
	objectState_.clear();

	//	free(osiSensorView.sensor_view);
	osiSensorView.size = 0;

	data_file_.flush();
	data_file_.close();

	CloseSocket();

	osi_file.close();

	// Assume osi cleans up allocated data
}

int ScenarioGateway::OpenSocket(std::string ipaddr)
{
#ifdef _WIN32
	WSADATA wsa_data;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (iResult != NO_ERROR)
	{
		wprintf(L"WSAStartup failed with error %d\n", iResult);
		return -1;
	}
#endif

	// create socket for outgoing UDP packages
	sendSocket = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sendSocket < 0) {
		LOG("socket failed");
		return -1;
	}

	// Setup receiver IP address
	recvAddr.sin_family = AF_INET;
	recvAddr.sin_port = htons(OSI_OUT_PORT);
	inet_pton(AF_INET, ipaddr.c_str(), &recvAddr.sin_addr);

	return 0;
}

int ScenarioGateway::CloseSocket()
{
	if (!sendSocket)
	{
		return 0;
	}

#ifdef _WIN32
	if (closesocket(sendSocket) == SOCKET_ERROR)
#else
	if (close(sendSocket) < 0)
#endif
	{
		printf("Failed closing socket");
		return -1;
	}

#ifdef _WIN32
	WSACleanup();
#endif

	return 0;
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

int ScenarioGateway::UpdateOSISensorView()
{

	// Add instances of moving object according to number of objects in the scenario
	while (mobj_osi_internal.mobj.size() < objectState_.size())
	{
		osi3::MovingObject* mobj = mobj_osi_internal.sv->mutable_global_ground_truth()->add_moving_object();

		mobj->mutable_id()->set_value(mobj_osi_internal.mobj.size());
		mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_MEDIUM_CAR);
		mobj->mutable_base()->mutable_dimension()->set_height(1.5);
		mobj->mutable_base()->mutable_dimension()->set_width(1.75);
		mobj->mutable_base()->mutable_dimension()->set_length(4.0);

		mobj_osi_internal.mobj.push_back(mobj);
	}

	if (objectState_.size() > 0)
	{
		// Grab timestamp from first vehicle, if available (otherwise stick with default 0.0 sec)
		mobj_osi_internal.sv->mutable_global_ground_truth()->mutable_timestamp()->set_seconds((int64_t)objectState_[0]->state_.timeStamp);
		mobj_osi_internal.sv->mutable_global_ground_truth()->mutable_timestamp()->set_nanos((uint32_t)(
			(objectState_[0]->state_.timeStamp - (int64_t)objectState_[0]->state_.timeStamp) * 1e9)
		);
	}

	for (size_t i = 0; i < objectState_.size(); i++)
	{
		mobj_osi_internal.mobj[i]->mutable_vehicle_attributes()->mutable_driver_id()->set_value((uint64_t)objectState_[i]->state_.control);  // a placeholder for control mode
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_position()->set_x(objectState_[i]->state_.pos.GetX());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_position()->set_y(objectState_[i]->state_.pos.GetY());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_position()->set_z(objectState_[i]->state_.pos.GetZ());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_orientation()->set_yaw(objectState_[i]->state_.pos.GetH());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_orientation()->set_pitch(objectState_[i]->state_.pos.GetP());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_orientation()->set_roll(objectState_[i]->state_.pos.GetR());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_velocity()->set_x(objectState_[i]->state_.speed * cos(objectState_[i]->state_.pos.GetH()));
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_velocity()->set_y(objectState_[i]->state_.speed * sin(objectState_[i]->state_.pos.GetH()));
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_velocity()->set_z(0);  // assume neglectable speed in z dimension
	}

	mobj_osi_internal.sv->SerializeToString(&osiSensorView.sensor_view);
	osiSensorView.size = (unsigned int)mobj_osi_internal.sv->ByteSizeLong();

	// write to file, first size of message
	osi_file.write((char*)&osiSensorView.size, sizeof(osiSensorView.size));

	// write to file, actual message - the sensorview object including timestamp and moving objects
	osi_file.write(osiSensorView.sensor_view.c_str(), osiSensorView.size);

	// send over udp - skip size (package size == message size)
	if (sendSocket)
	{
		int sendResult = sendto(sendSocket, (char*)osiSensorView.sensor_view.c_str(), osiSensorView.size, 0, (struct sockaddr*)&recvAddr, sizeof(recvAddr));
		if (sendResult != osiSensorView.size)
		{
			LOG("Failed send osi package over UDP");
#ifdef _WIN32
			wprintf(L"send failed with error: %d\n", WSAGetLastError());
#endif
		}
	}

	// Indicate not found by returning non zero
	return -1;
}

int ScenarioGateway::UpdateOSIRoadLane(int object_id, int lane_idx)
{
	// Check if object_id exists
	if (object_id >= getNumberOfObjects())
	{
		LOG("Object %d not available, only %d registered", object_id, getNumberOfObjects());
		return -1;
	}
	
	// Find position of the object 
	roadmanager::Position pos;
	for (size_t i = 0; i < getNumberOfObjects() ; i++)
	{
		if (object_id == objectState_[i]->state_.id)
		{
			pos = objectState_[i]->state_.pos;
		}		
	}
	// Find road, LaneSection and Lane where the object is 
	// STILL TO DO: use the lane_idx to look to adjecent lanes 
	int road_id = pos.GetTrackId();
	roadmanager::Road* road = pos.GetRoadById(road_id);
	roadmanager::LaneSection* lane_section = road->GetLaneSectionByS(pos.GetS());
	double offset; 
	int closest_lane_idx = lane_section->GetClosestLaneIdx(pos.GetS(), pos.GetT(), offset);
	roadmanager::Lane* lane = lane_section->GetLaneByIdx(closest_lane_idx);
	roadmanager::Lane::LaneType lanetype = lane->GetLaneType();	

	//update lane id
	mlane_osi_internal.ln->mutable_id()->set_value((uint64_t)pos.GetLaneId()); 

	// update classification type
	// STILL TO DO: add more types
	osi3::Lane_Classification_Type class_type;
	if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_DRIVING)
	{
		class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING; 
	}
	else
	{
		class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_UNKNOWN; 
	}
	mlane_osi_internal.ln->mutable_classification()->set_type(class_type);

	// update classification is_vehicle_in_lane 
	bool is_veh_on_lane; 
	if (lane_idx == 0 )
	{
		is_veh_on_lane = true;
	}
	else
	{
		is_veh_on_lane = false; 
	}
	mlane_osi_internal.ln->mutable_classification()->set_is_host_vehicle_lane(is_veh_on_lane);

	// STILL TO DO: update lane centerline points
	double x = 0;
	double y = 0;
	double z = 0; 
	osi3::Vector3d* centerLine = mlane_osi_internal.ln->mutable_classification()->add_centerline(); 
	centerLine->set_x(x);
	centerLine->set_y(y);
	centerLine->set_z(z);

	// STILL TO DO: check if object is moving in the same direction of the lane centerline points 
	bool center_is_driving = true; 
	mlane_osi_internal.ln->mutable_classification()->set_centerline_is_driving_direction(center_is_driving); 

	// update lane_id for lanes on the left andlanes on the righ 
	int n_lanes_in_section = lane_section->GetNumberOfLanes();
	std::vector<int> lanes_on_left; 
	std::vector<int> lanes_on_right;
	for (int i = 0; i < n_lanes_in_section; i++)
	{
		if (lane_section->GetLaneIdByIdx(i)< pos.GetLaneId() )
		{
			lanes_on_left.push_back(lane_section->GetLaneIdByIdx(i));
		}
		else if (lane_section->GetLaneIdByIdx(i)> pos.GetLaneId() )
		{
			lanes_on_right.push_back(lane_section->GetLaneIdByIdx(i));
		}
		
	}
	std::sort(lanes_on_left.begin(),lanes_on_left.end());
	std::reverse(lanes_on_left.begin(),lanes_on_left.end());
	std::sort(lanes_on_right.begin(),lanes_on_right.end());

	for (int i = 0; i < lanes_on_left.size(); i++)
	{
		osi3::Identifier* left_id = mlane_osi_internal.ln->mutable_classification()->add_left_adjacent_lane_id();
		left_id->set_value((uint64_t)lanes_on_left[i]);  
	}
	for (int i = 0; i < lanes_on_right.size(); i++)
	{
		osi3::Identifier* right_id = mlane_osi_internal.ln->mutable_classification()->add_right_adjacent_lane_id(); 
		right_id->set_value((uint64_t)lanes_on_right[i]); 
	}

	// update lane paiting 
	// STILL TO DO: when I get a vector of predecessors and successors I need to create all possible combinations
	roadmanager::LaneLink* lane_pre = lane->GetLink(roadmanager::LinkType::PREDECESSOR);
	roadmanager::LaneLink* lane_succ = lane->GetLink(roadmanager::LinkType::SUCCESSOR); 
	osi3::Lane_Classification_LanePairing* lane_pair = mlane_osi_internal.ln->mutable_classification()->add_lane_pairing();
	lane_pair->mutable_antecessor_lane_id()->set_value(lane_pre->GetId()); 
	lane_pair->mutable_successor_lane_id()->set_value(lane_succ->GetId()); 

	// STILL TO DO:
	int right_bound_id = 0; 
	osi3::Identifier* right_lane_bound_id = mlane_osi_internal.ln->mutable_classification()->add_right_lane_boundary_id(); 
	right_lane_bound_id->set_value(right_bound_id); 

	// STILL TO DO: 
	int left_bound_id = 0; 
	osi3::Identifier* left_lane_bound_id = mlane_osi_internal.ln->mutable_classification()->add_left_lane_boundary_id(); 
	left_lane_bound_id->set_value(left_bound_id); 
	
	// STILL TO DO:
	int free_bound_id = 0; 
	osi3::Identifier* free_lane_bound_id = mlane_osi_internal.ln->mutable_classification()->add_free_lane_boundary_id(); 
	free_lane_bound_id->set_value(free_bound_id); 

	// STILL TO DO: 
	double temp = 0; 
	mlane_osi_internal.ln->mutable_classification()->mutable_road_condition()->set_surface_temperature(temp);
	mlane_osi_internal.ln->mutable_classification()->mutable_road_condition()->set_surface_water_film(temp);
	mlane_osi_internal.ln->mutable_classification()->mutable_road_condition()->set_surface_freezing_point(temp);
	mlane_osi_internal.ln->mutable_classification()->mutable_road_condition()->set_surface_ice(temp);
	mlane_osi_internal.ln->mutable_classification()->mutable_road_condition()->set_surface_roughness(temp);
	mlane_osi_internal.ln->mutable_classification()->mutable_road_condition()->set_surface_texture(temp);
	

	
	mlane_osi_internal.ln->SerializeToString(&osiRoadLane.lane_info);
	osiRoadLane.size = (unsigned int)mlane_osi_internal.ln->ByteSizeLong();

	// write to file, first size of message
	osi_file.write((char*)&osiRoadLane.size, sizeof(osiRoadLane.size));

	// write to file, actual message - the sensorview object including timestamp and moving objects
	osi_file.write(osiRoadLane.lane_info.c_str(), osiRoadLane.size);

	// send over udp - skip size (package size == message size)
	if (sendSocket)
	{
		int sendResult = sendto(sendSocket, (char*)osiRoadLane.lane_info.c_str(), osiRoadLane.size, 0, (struct sockaddr*)&recvAddr, sizeof(recvAddr));
		if (sendResult != osiRoadLane.size)
		{
			LOG("Failed send osi package over UDP");
#ifdef _WIN32
			wprintf(L"send failed with error: %d\n", WSAGetLastError());
#endif
		}
	}

	// Indicate not found by returning non zero
	return -1;
}

const char* ScenarioGateway::GetOSISensorView(int* size)
{
	*size = osiSensorView.size;
	return osiSensorView.sensor_view.data();
}

const char* ScenarioGateway::GetOSIRoadLane(int* size, int lane_idx)
{
	*size = osiRoadLane.size;
	return osiRoadLane.lane_info.data();
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

void ScenarioGateway::reportObject(int id, std::string name, int model_id, int control,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	roadmanager::Position* pos)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, model_id, control, timestamp, speed, wheel_angle, wheel_rot, pos);

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

void ScenarioGateway::reportObject(int id, std::string name, int model_id, int control,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	double x, double y, double z, double h, double p, double r)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, model_id, control, timestamp, speed, wheel_angle, wheel_rot, x, y, z, h, p, r);

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

void ScenarioGateway::reportObject(int id, std::string name, int model_id, int control,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	int roadId, int laneId, double laneOffset, double s)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, model_id, control, timestamp, speed, wheel_angle, wheel_rot, roadId, laneId, laneOffset, s);

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




SumoController::SumoController(std::string file)
{
	
	std::vector<std::string> options;
	options.push_back(file);
	
	// libsumo::Simulation::load(options);
	sumo_used = true;
}
SumoController::SumoController()
{
	sumo_used = false;
}
void SumoController::step(double time, Entities* entities, ScenarioGateway* scegw)
{
	if (sumo_used)
	{
		libsumo::Simulation::step(time);
		// if (libsumo::Simulation::getDepartedNumber() > 0) {
		// 	std::vector<std::string> deplist = libsumo::Simulation::getDepartedIDList();
		// 	for (std::vector<std::string>::iterator name = deplist.begin(); name != deplist.end(); ++name) {
		// 		Vehicle *vehicle = new Vehicle();
		// 		// copy the default vehicle stuffs here

		// 		vehicle->name_ = *name;
		// 		entities->addObject(vehicle);

		// 		// scegw->addObject(); // maybe add this in the future?
		// 	}
		// }

		// if (libsumo::Simulation::getArrivedNumber() > 0) {
		// std::vector<std::string> deplist = libsumo::Simulation::getArrivedIDList();
		// 	for (std::vector<std::string>::iterator j = deplist.begin(); j != deplist.end(); ++j) {
		// 		for(std::vector<std::string>::iterator it = entities->object_.begin(); it != entities->object_.end(); ++it) {
		// 			if (*it == *j) {
		// 				// entities->removeObject(*it);
		// 				// scegw->removeObject(*it);
		// 			}
		// 		}
		// 	}
		// }

		for (size_t i = 0; i < entities->object_.size(); i++) 
		{
			if (entities->object_[i]->control_ == Object::Control::SUMO)
			{
				std::string sumoid = entities->object_[i]->name_;
				libsumo::TraCIPosition pos = libsumo::Vehicle::getPosition3D(sumoid);

				entities->object_[i]->speed_ = libsumo::Vehicle::getSpeed(sumoid);
				entities->object_[i]->pos_.SetX(pos.x);
				entities->object_[i]->pos_.SetY(pos.y);
				entities->object_[i]->pos_.SetZ(pos.z);
				entities->object_[i]->pos_.SetH(libsumo::Vehicle::getAngle(sumoid));
				entities->object_[i]->pos_.SetP(libsumo::Vehicle::getSlope(sumoid));
			}
        }

	}
}