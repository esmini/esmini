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

#include "CommonMini.hpp"
#include "OSIReporter.hpp"
#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_sensordata.pb.h"
#include "osi_version.pb.h"
#include "osi_common.pb.h"
#include <cmath>

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

#define OSI_OUT_PORT 48198
#define OSI_MAX_UDP_DATA_SIZE 8200

static struct {
	osi3::SensorData *sd;
	osi3::GroundTruth *gt;
	osi3::StationaryObject *sobj;
	osi3::MovingObject *mobj;
	std::vector<osi3::Lane*> ln;
	std::vector<osi3::LaneBoundary*> lnb;
} obj_osi_internal;

// Large OSI messages needs to be split for UDP transmission
// This struct must be mached on receiver side
static struct {
	int counter;
	unsigned int datasize;
	char data[OSI_MAX_UDP_DATA_SIZE];
} osi_udp_buf;

typedef struct
{
	std::string ground_truth;
	unsigned int size;
} OSIGroundTruth;

typedef struct
{
	std::string osi_lane_info;
	unsigned int size;
} OSIRoadLane;

typedef struct
{
	std::string osi_lane_boundary_info;
	unsigned int size;
} OSIRoadLaneBoundary;

using namespace scenarioengine;

static OSIGroundTruth osiGroundTruth;
static OSIRoadLane osiRoadLane;
static OSIRoadLaneBoundary osiRoadLaneBoundary;

static struct sockaddr_in recvAddr;

// ScenarioGateway

OSIReporter::OSIReporter()
{
	sendSocket = 0;

	obj_osi_internal.gt = new osi3::GroundTruth();

	obj_osi_internal.gt->mutable_version()->set_version_major(3);
	obj_osi_internal.gt->mutable_version()->set_version_minor(0);
	obj_osi_internal.gt->mutable_version()->set_version_patch(0);

	obj_osi_internal.gt->mutable_timestamp()->set_seconds(0);
	obj_osi_internal.gt->mutable_timestamp()->set_nanos(0);

    // Sensor Data
    obj_osi_internal.sd = new osi3::SensorData();

	// Counter for OSI update
	osi_update_counter_ = 0;

	nanosec_ = 0xffffffffffffffff; // indicate not set
}

OSIReporter::~OSIReporter()
{
	if (obj_osi_internal.gt)
	{
		obj_osi_internal.gt->Clear();
		delete obj_osi_internal.gt;
	}

	if (obj_osi_internal.sd)
	{
		obj_osi_internal.sd->Clear();
		delete obj_osi_internal.sd;
	}

	obj_osi_internal.ln.clear();
	obj_osi_internal.lnb.clear();

	osiGroundTruth.size = 0;
	osiRoadLane.size=0;

	CloseSocket();
	if (osi_file.is_open())
	{
		osi_file.close();
	}

}

int OSIReporter::OpenSocket(std::string ipaddr)
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

int OSIReporter::CloseSocket()
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

void OSIReporter::ReportSensors(std::vector<ObjectSensor*> sensor)
{
	const int max_detections = 10;

	if (sensor.size() == 0)
	{
		return;
	}
	while (obj_osi_internal.sd->sensor_view_size() < sensor.size()){
		obj_osi_internal.sd->add_sensor_view();
	}
	for (int i = 0; i < sensor.size(); i++)
	{
        // Clear history
        obj_osi_internal.sd->mutable_sensor_view(i)->mutable_global_ground_truth()->clear_moving_object();
		for (int j = 0; j < sensor[i]->nObj_; j++)
		{
            // Create moving object
            osi3::MovingObject* mobj;
            mobj = obj_osi_internal.sd->mutable_sensor_view(i)->mutable_global_ground_truth()->add_moving_object();

            // Populate sensor data
            mobj->mutable_id()->set_value(sensor[i]->hitList_[j].obj_->id_);
			mobj->mutable_base()->mutable_position()->set_x(sensor[i]->hitList_[j].x_ + sensor[i]->hitList_[j].obj_->boundingbox_.center_.x_*cos(sensor[i]->hitList_[j].yaw_));
			mobj->mutable_base()->mutable_position()->set_y(sensor[i]->hitList_[j].y_ + sensor[i]->hitList_[j].obj_->boundingbox_.center_.x_*sin(sensor[i]->hitList_[j].yaw_));
			mobj->mutable_base()->mutable_position()->set_z(sensor[i]->hitList_[j].z_);
			mobj->mutable_base()->mutable_velocity()->set_x(sensor[i]->hitList_[j].velX_);
			mobj->mutable_base()->mutable_velocity()->set_y(sensor[i]->hitList_[j].velY_);
			mobj->mutable_base()->mutable_velocity()->set_z(sensor[i]->hitList_[j].velZ_);
			mobj->mutable_base()->mutable_acceleration()->set_x(sensor[i]->hitList_[j].accX_);
			mobj->mutable_base()->mutable_acceleration()->set_y(sensor[i]->hitList_[j].accY_);
			mobj->mutable_base()->mutable_acceleration()->set_z(sensor[i]->hitList_[j].accZ_);
			mobj->mutable_base()->mutable_orientation()->set_yaw(sensor[i]->hitList_[j].yaw_);
			mobj->mutable_base()->mutable_orientation_rate()->set_yaw(sensor[i]->hitList_[j].yawRate_);
			mobj->mutable_base()->mutable_orientation_acceleration()->set_yaw(sensor[i]->hitList_[j].yawAcc_);
			mobj->mutable_base()->mutable_dimension()->set_height(sensor[i]->hitList_[j].obj_->boundingbox_.dimensions_.height_);
			mobj->mutable_base()->mutable_dimension()->set_length(sensor[i]->hitList_[j].obj_->boundingbox_.dimensions_.length_);
			mobj->mutable_base()->mutable_dimension()->set_width(sensor[i]->hitList_[j].obj_->boundingbox_.dimensions_.width_);

		}
	}
}

bool OSIReporter::OpenOSIFile(const char* filename)
{
	const char* f = (filename == 0 || !strcmp(filename, "")) ? "ground_truth.osi" : filename;
	osi_file = std::ofstream(f, std::ios_base::binary);
	if (!osi_file.good())
	{
		LOG("Failed open OSI tracefile %s", f);
		return false;
	}
	LOG("OSI tracefile %s opened", f);
	return true;
}

void OSIReporter::CloseOSIFile()
{
	osi_file.close();
}

bool OSIReporter::WriteOSIFile()
{
	if (!osi_file.good())
	{
		return false;
	}

	// write to file, first size of message
	osi_file.write((char*)&osiGroundTruth.size, sizeof(osiGroundTruth.size));

	// write to file, actual message - the groundtruth object including timestamp and moving objects
	osi_file.write(osiGroundTruth.ground_truth.c_str(), osiGroundTruth.size);

	if (!osi_file.good())
	{
		LOG("Failed write osi file");
		return false;
	}
	return true;
}

void OSIReporter::FlushOSIFile()
{
	if (osi_file.good())
	{
		osi_file.flush();
	}
}

int OSIReporter::ClearOSIGroundTruth()
{
	obj_osi_internal.gt->clear_moving_object();
	obj_osi_internal.gt->clear_stationary_object();
	obj_osi_internal.gt->clear_lane();
	obj_osi_internal.gt->clear_lane_boundary();

	return 0;
}

int OSIReporter::UpdateOSIGroundTruth(std::vector<ObjectState*> objectState)
{
	if (osi_update_counter_ == 0)
	{
		UpdateOSIStaticGroundTruth(objectState);
	}
	obj_osi_internal.gt->clear_moving_object();
	UpdateOSIDynamicGroundTruth(objectState);

	if (GetSocket() || IsFileOpen())
	{
		obj_osi_internal.gt->SerializeToString(&osiGroundTruth.ground_truth);
		osiGroundTruth.size = (unsigned int)obj_osi_internal.gt->ByteSizeLong();
	}

	if (sendSocket)
	{
		// send over udp - split large OSI messages in multiple transmissions
		unsigned int sentDataBytes = 0;

		for (osi_udp_buf.counter = 0, osi_udp_buf.datasize = 0; sentDataBytes < osiGroundTruth.size; osi_udp_buf.counter++)
		{
			osi_udp_buf.datasize = MIN(osiGroundTruth.size - sentDataBytes, OSI_MAX_UDP_DATA_SIZE);
			memcpy(&osi_udp_buf.data, (char*)&osiGroundTruth.ground_truth.c_str()[sentDataBytes], osi_udp_buf.datasize);
			int packSize = sizeof(osi_udp_buf) - (OSI_MAX_UDP_DATA_SIZE - osi_udp_buf.datasize);

			if (sentDataBytes + osi_udp_buf.datasize >= osiGroundTruth.size)
			{
				// Last package
				osi_udp_buf.counter = -1;
			}
			int sendResult = sendto(sendSocket, (char*)&osi_udp_buf, packSize, 0, (struct sockaddr*)&recvAddr, sizeof(recvAddr));

			if (sendResult != packSize)
			{
				LOG("Failed send osi package over UDP");
#ifdef _WIN32
				wprintf(L"send failed with error: %d\n", WSAGetLastError());
#endif
				// Give up
				sentDataBytes = osiGroundTruth.size;
			}
			else
			{
				sentDataBytes += osi_udp_buf.datasize;
			}
		}
	}

	if (IsFileOpen())
	{
		WriteOSIFile();
	}
	osi_update_counter_++;

	return 0;
}

int OSIReporter::UpdateOSIStaticGroundTruth(std::vector<ObjectState*> objectState)
{
	for (size_t i = 0; i < objectState.size(); i++)
	{
		if(objectState[i]->state_.info.obj_type==static_cast<int>(Object::Type::VEHICLE) ||
		objectState[i]->state_.info.obj_type==static_cast<int>(Object::Type::PEDESTRIAN))
		{
			// do nothing
		}
		else if(objectState[i]->state_.info.obj_type==static_cast<int>(Object::Type::MISC_OBJECT))
		{
			UpdateOSIStationaryObject(objectState[i]);
		}
		else
		{
			LOG("Warning: Object type %d is not supported in OSIReporter, and hence no OSI update for this object", objectState[i]->state_.info.obj_type);
		}
	}

	UpdateOSIRoadLane(objectState);
	UpdateOSILaneBoundary(objectState);
	UpdateOSIIntersection();
	return 0;
}

int OSIReporter::UpdateOSIDynamicGroundTruth(std::vector<ObjectState*> objectState)
{
	if (IsTimeStampSetExplicit())
	{
		// use excplicit timestamp
		obj_osi_internal.gt->mutable_timestamp()->set_seconds((int64_t)(nanosec_ / 1000000000));
		obj_osi_internal.gt->mutable_timestamp()->set_nanos((uint32_t)(nanosec_ % 1000000000));
	}
	else
	{
		// use timstamp from object state
		obj_osi_internal.gt->mutable_timestamp()->set_seconds((int64_t)objectState[0]->state_.info.timeStamp);
		obj_osi_internal.gt->mutable_timestamp()->set_nanos((uint32_t)(
			(objectState[0]->state_.info.timeStamp - (int64_t)objectState[0]->state_.info.timeStamp) * 1e9)
		);
	}

	for (size_t i = 0; i < objectState.size(); i++)
	{
		if(objectState[i]->state_.info.obj_type==static_cast<int>(Object::Type::VEHICLE) ||
		objectState[i]->state_.info.obj_type==static_cast<int>(Object::Type::PEDESTRIAN))
		{
			UpdateOSIMovingObject(objectState[i]);
		}
		else if(objectState[i]->state_.info.obj_type==static_cast<int>(Object::Type::MISC_OBJECT))
		{
			// do nothing
		}
		else
		{
			LOG("Warning: Object type %d is not supported in OSIReporter, and hence no OSI update for this object", objectState[i]->state_.info.obj_type);
		}
	}

	return 0;
}

int OSIReporter::UpdateOSIHostVehicleData(ObjectState* objectState)
{
	(void)objectState;  // avoid compiler warning
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_position()->set_x(objectState->state_.pos.GetX());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_position()->set_y(objectState->state_.pos.GetY());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_position()->set_z(objectState->state_.pos.GetZ());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_velocity()->set_x(objectState->state_.pos.GetVelX());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_velocity()->set_y(objectState->state_.pos.GetVelY());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_velocity()->set_z(objectState->state_.pos.GetVelZ());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_acceleration()->set_x(objectState->state_.pos.GetAccX());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_acceleration()->set_y(objectState->state_.pos.GetAccY());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_acceleration()->set_z(objectState->state_.pos.GetAccZ());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_orientation()->set_yaw(objectState->state_.pos.GetH());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_orientation_rate()->set_yaw(objectState->state_.pos.GetHRate());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_orientation_acceleration()->set_yaw(objectState->state_.pos.GetHAcc());
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_dimension()->set_height(objectState->state_.boundingbox.dimensions_.height_);
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_dimension()->set_width(objectState->state_.boundingbox.dimensions_.width_);
	// obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_dimension()->set_length(objectState->state_.boundingbox.dimensions_.length_);

	return 0;
}

int OSIReporter::UpdateOSIStationaryObject(ObjectState* objectState)
{
	// Create OSI Stationary Object
	obj_osi_internal.sobj = obj_osi_internal.gt->add_stationary_object();

	// Set OSI Stationary Object Mutable ID
	int sobj_size = obj_osi_internal.gt->mutable_stationary_object()->size();
	obj_osi_internal.sobj->mutable_id()->set_value(sobj_size-1);

	// Set OSI Stationary Object Type and Classification
	if(objectState->state_.info.obj_type == static_cast<int>(Object::Type::MISC_OBJECT))
	{
		if(objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::NONE))
		{
			obj_osi_internal.sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_UNKNOWN);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::POLE))
		{
			obj_osi_internal.sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_POLE);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::TREE))
		{
			obj_osi_internal.sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_TREE);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::VEGETATION))
		{
			obj_osi_internal.sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_VEGETATION);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::BARRIER))
		{
			obj_osi_internal.sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BARRIER);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::BUILDING))
		{
			obj_osi_internal.sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BUILDING);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::OBSTACLE) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::PARKINGSPACE) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::RAILING) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::PATCH) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::TRAFFICISLAND) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::CROSSWALK) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::STREETLAMP) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::GANTRY) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::SOUNDBARRIER) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::WIND) ||
				objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::ROADMARK))
		{
			obj_osi_internal.sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_OTHER);
		}
		else
		{
			obj_osi_internal.sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_UNKNOWN);
			LOG("OSIReporter::UpdateOSIStationaryObject -> Unsupported stationary object category");
		}
	}
	else
	{
		LOG("OSIReporter::UpdateOSIStationaryObject -> Unsupported stationary object type");
	}

	// Set OSI Stationary Object Boundingbox
	obj_osi_internal.sobj->mutable_base()->mutable_dimension()->set_height(objectState->state_.info.boundingbox.dimensions_.height_);
	obj_osi_internal.sobj->mutable_base()->mutable_dimension()->set_width(objectState->state_.info.boundingbox.dimensions_.width_);
	obj_osi_internal.sobj->mutable_base()->mutable_dimension()->set_length(objectState->state_.info.boundingbox.dimensions_.length_);

	// Set OSI Stationary Object Position
	obj_osi_internal.sobj->mutable_base()->mutable_position()->set_x(objectState->state_.pos.GetX() + objectState->state_.info.boundingbox.center_.x_*cos(objectState->state_.pos.GetH()));
	obj_osi_internal.sobj->mutable_base()->mutable_position()->set_y(objectState->state_.pos.GetY() + objectState->state_.info.boundingbox.center_.x_*sin(objectState->state_.pos.GetH()));
	obj_osi_internal.sobj->mutable_base()->mutable_position()->set_z(objectState->state_.pos.GetZ());

	// Set OSI Stationary Object Orientation
	obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_roll(objectState->state_.pos.GetR());
	obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_pitch(objectState->state_.pos.GetP());
	obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_yaw(objectState->state_.pos.GetH());

	return 0;
}

int OSIReporter::UpdateOSIMovingObject(ObjectState* objectState)
{
	// Create OSI Moving object
	obj_osi_internal.mobj = obj_osi_internal.gt->add_moving_object();

	// Set OSI Moving Object Mutable ID
	int mobj_size = obj_osi_internal.gt->mutable_moving_object()->size();
	obj_osi_internal.mobj->mutable_id()->set_value(mobj_size-1);

	// Set OSI Moving Object Type and Classification
	if(objectState->state_.info.obj_type == static_cast<int>(Object::Type::VEHICLE))
	{
		obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_VEHICLE);

		if(objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::CAR))
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_MEDIUM_CAR);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::BICYCLE))
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_BICYCLE);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::BUS))
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_BUS);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::MOTORBIKE))
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_MOTORBIKE);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::SEMITRAILER))
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_SEMITRAILER);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::TRAIN))
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_TRAIN);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::TRAM))
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_TRAM);
		}
		else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::TRUCK))
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_HEAVY_TRUCK);
		}
		else
		{
			obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_UNKNOWN);
			LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object category");
		}
	}
	else if (objectState->state_.info.obj_type == static_cast<int>(Object::Type::PEDESTRIAN))
	{
		if (objectState->state_.info.obj_category==static_cast<int>(Pedestrian::Category::PEDESTRIAN))
		{
			obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_PEDESTRIAN);
		}
		else if (objectState->state_.info.obj_category==static_cast<int>(Pedestrian::Category::ANIMAL))
		{
			obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_ANIMAL);
		}
		else if (objectState->state_.info.obj_category==static_cast<int>(Pedestrian::Category::WHEELCHAIR))
		{
			obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_OTHER);
		}
		else
		{
			obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_UNKNOWN);
			LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object category");
		}
	}
	else
	{
		LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object type");
	}

	// Set OSI Moving Object Control Type
	obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_driver_id()->set_value((uint64_t)objectState->state_.info.ctrl_type);

	// Set OSI Moving Object Boundingbox
	obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_x((double)(objectState->state_.info.boundingbox.center_.x_));
	obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_y((double)(objectState->state_.info.boundingbox.center_.y_));
	obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_z((double)(objectState->state_.info.boundingbox.center_.z_));
	obj_osi_internal.mobj->mutable_base()->mutable_dimension()->set_height(objectState->state_.info.boundingbox.dimensions_.height_);
	obj_osi_internal.mobj->mutable_base()->mutable_dimension()->set_width(objectState->state_.info.boundingbox.dimensions_.width_);
	obj_osi_internal.mobj->mutable_base()->mutable_dimension()->set_length(objectState->state_.info.boundingbox.dimensions_.length_);

	// Set OSI Moving Object Position
	obj_osi_internal.mobj->mutable_base()->mutable_position()->set_x(objectState->state_.pos.GetX() + objectState->state_.info.boundingbox.center_.x_*cos(objectState->state_.pos.GetH()));
	obj_osi_internal.mobj->mutable_base()->mutable_position()->set_y(objectState->state_.pos.GetY() + objectState->state_.info.boundingbox.center_.x_*sin(objectState->state_.pos.GetH()));
	obj_osi_internal.mobj->mutable_base()->mutable_position()->set_z(objectState->state_.pos.GetZ());

	// Set OSI Moving Object Orientation
	obj_osi_internal.mobj->mutable_base()->mutable_orientation()->set_roll(objectState->state_.pos.GetR());
	obj_osi_internal.mobj->mutable_base()->mutable_orientation()->set_pitch(objectState->state_.pos.GetP());
	obj_osi_internal.mobj->mutable_base()->mutable_orientation()->set_yaw(objectState->state_.pos.GetH());
	obj_osi_internal.mobj->mutable_base()->mutable_orientation_rate()->set_yaw(objectState->state_.pos.GetHRate());
	obj_osi_internal.mobj->mutable_base()->mutable_orientation_rate()->set_pitch(objectState->state_.pos.GetPRate());
	obj_osi_internal.mobj->mutable_base()->mutable_orientation_rate()->set_roll(objectState->state_.pos.GetRRate());
	obj_osi_internal.mobj->mutable_base()->mutable_orientation_acceleration()->set_yaw(objectState->state_.pos.GetHAcc());
	obj_osi_internal.mobj->mutable_base()->mutable_orientation_acceleration()->set_pitch(objectState->state_.pos.GetPAcc());
	obj_osi_internal.mobj->mutable_base()->mutable_orientation_acceleration()->set_roll(objectState->state_.pos.GetRAcc());

	// Set OSI Moving Object Velocity
	obj_osi_internal.mobj->mutable_base()->mutable_velocity()->set_x(objectState->state_.pos.GetVelX());
	obj_osi_internal.mobj->mutable_base()->mutable_velocity()->set_y(objectState->state_.pos.GetVelY());
	obj_osi_internal.mobj->mutable_base()->mutable_velocity()->set_z(objectState->state_.pos.GetVelZ());

	// Set OSI Moving Object Acceleration
	obj_osi_internal.mobj->mutable_base()->mutable_acceleration()->set_x(objectState->state_.pos.GetAccX());
	obj_osi_internal.mobj->mutable_base()->mutable_acceleration()->set_y(objectState->state_.pos.GetAccY());
	obj_osi_internal.mobj->mutable_base()->mutable_acceleration()->set_z(objectState->state_.pos.GetAccZ());

	// Set OSI Moving Object Lane ID
	obj_osi_internal.mobj->add_assigned_lane_id()->set_value(objectState->state_.pos.GetLaneGlobalId());

	return 0;
}

int OSIReporter::UpdateOSIIntersection()
{
	// NOTE: for free_lane_boundary this algoritm will only work for open drive solid roadmarks in the junction (or atleast the outest driving lane's roadmark)
	
	// tolerance to check if points are close or not
	double tolerance = 0.01;

	typedef struct
	{
		int id;
		double length;
		int global_id;
		roadmanager::OSIPoints *osipoints;
		
	} LaneLengthStruct;
	
	roadmanager::Junction *junction;
	roadmanager::Connection *connection;
	roadmanager::JunctionLaneLink *junctionlanelink;
	roadmanager::Road *incomming_road;
	roadmanager::Road *outgoing_road;
	roadmanager::Road *connecting_road;
	roadmanager::ContactPointType contactpoint;
	roadmanager::RoadLink *roadlink = 0;
	LaneLengthStruct left_lane_struct;
	LaneLengthStruct right_lane_struct;
	// s values to know where on the road to check for the lanes
	double incomming_s_value;
	double outgoing_s_value;
	double connecting_outgoing_s_value;
	
	// value for the linktype for the connecting road and the outgoing road
	roadmanager::LinkType connecting_road_link_type = roadmanager::LinkType::NONE;
	// value for the linktype for the incomming road and the connecting road
	roadmanager::LinkType incomming_road_link_type = roadmanager::LinkType::NONE;
	// some values used for fixing free lane boundary
	double length;
	bool new_connecting_road;
	int g_id;
	roadmanager::OSIPoints *osipoints;

	static roadmanager::OpenDrive* opendrive = roadmanager::Position::GetOpenDrive();
	osi3::Lane* osi_lane;
	for (int i=0; i<opendrive->GetNumOfJunctions(); i++)
	{
		std::vector<LaneLengthStruct> left_lane_lengths;
		std::vector<LaneLengthStruct> right_lane_lengths;
		std::vector<LaneLengthStruct> lane_lengths;
		std::vector<LaneLengthStruct> tmp_lane_lengths;
		std::set<int> connected_roads;
		// //add check if it is an intersection or an highway exit/entry
		junction = opendrive->GetJunctionByIdx(i);
		
		// check if the first road is of type highway, then assumes it is not a intersection 
		if (junction->IsOsiIntersection())
		{

			// genereric data for the junction
			osi_lane = obj_osi_internal.gt->add_lane();
			osi_lane->mutable_id()->set_value(junction->GetGlobalId());
			osi_lane->mutable_classification()->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION);
			

			// check all connections in the junction
			for (int j=0; j<junction->GetNumberOfConnections();j++)
			{
				connection = junction->GetConnectionByIdx(j);
				incomming_road = connection->GetIncomingRoad();
				connecting_road = connection->GetConnectingRoad();
				new_connecting_road = true;
				

				// check if the connecting road has been used before
				for (int l=0; l<(int)lane_lengths.size(); l++)
				{
					if (lane_lengths[l].id == connecting_road->GetId())
					{
						new_connecting_road = false;
					}
				}
				
				for (int l=0; l<(int)left_lane_lengths.size(); l++)
				{
					if (left_lane_lengths[l].id == connecting_road->GetId())
					{
						new_connecting_road = false;
					}
				}
				

				// get needed info about the incomming road
				if (incomming_road->GetLink(roadmanager::LinkType::SUCCESSOR) != 0) 
				{
					if (incomming_road->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == connecting_road->GetJunction())
					{
						incomming_s_value = incomming_road->GetLength();
						incomming_road_link_type = roadmanager::LinkType::SUCCESSOR;
					}
					else
					{
						incomming_s_value = 0;
						incomming_road_link_type = roadmanager::LinkType::PREDECESSOR;
					}
				}
				else
				{
					incomming_s_value = 0;
					incomming_road_link_type = roadmanager::LinkType::PREDECESSOR;
				}
				
				// Get info about the connecting road, and to get the correct outgoing road
				contactpoint = connection->GetContactPoint();
				if (contactpoint == roadmanager::ContactPointType::CONTACT_POINT_START)
				{
					connecting_road_link_type = roadmanager::LinkType::SUCCESSOR;
					roadlink = connecting_road->GetLink(connecting_road_link_type);
					connecting_outgoing_s_value = connecting_road->GetLength();
					
				} 
				else if (contactpoint == roadmanager::ContactPointType::CONTACT_POINT_END)
				{
					connecting_road_link_type = roadmanager::LinkType::PREDECESSOR;
					roadlink = connecting_road->GetLink(connecting_road_link_type);
					connecting_outgoing_s_value = 0;
					
				}
				else
				{
					LOG("WARNING: Unknow connection detected, can't establish outgoing connection in OSI junction");
					return -1;
				}
				outgoing_road = opendrive->GetRoadById(roadlink->GetElementId());
				connected_roads.insert(incomming_road->GetId());
				connected_roads.insert(outgoing_road->GetId());
				// Get neccesary info about the outgoing road
				if (outgoing_road->GetLink(roadmanager::LinkType::SUCCESSOR) != 0 )
				{
					if (outgoing_road->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == connecting_road->GetJunction())
					{
						outgoing_s_value = outgoing_road->GetLength();
					}
					else
					{
						outgoing_s_value = 0;
					}
				}
				else
				{
					outgoing_s_value = 0;
				}

				if (new_connecting_road)
				{

					left_lane_struct.id = connecting_road->GetId();
					left_lane_struct.length = LARGE_NUMBER; 
					right_lane_struct.id = connecting_road->GetId();
					right_lane_struct.length = LARGE_NUMBER;
					
					for (int l_id=1;l_id<=connecting_road->GetLaneSectionByS(0,0)->GetNUmberOfLanesRight();l_id++)
					{	
						if(connecting_road->GetLaneSectionByS(0,0)->GetLaneById(-l_id)->IsDriving())
						{	
							// check if an roadmark exist or use a laneboundary 
							//NOTE: assumes only simple lines in an intersection
							if (connecting_road->GetLaneSectionByS(0,0)->GetLaneById(-l_id)->GetLaneBoundaryGlobalId() != -1)
							{
								osipoints = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(-l_id)->GetLaneBoundary()->GetOSIPoints();
								length = osipoints->GetLength();
								g_id = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(-l_id)->GetLaneBoundary()->GetGlobalId();
							}
							else
							{
								osipoints = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(-l_id)->GetLaneRoadMarkByIdx(0)->GetLaneRoadMarkTypeByIdx(0)->GetLaneRoadMarkTypeLineByIdx(0)->GetOSIPoints();
								length = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(-l_id)->GetLaneRoadMarkByIdx(0)->GetLaneRoadMarkTypeByIdx(0)->GetLaneRoadMarkTypeLineByIdx(0)->GetOSIPoints()->GetLength();
								g_id = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(-l_id)->GetLaneRoadMarkByIdx(0)->GetLaneRoadMarkTypeByIdx(0)->GetLaneRoadMarkTypeLineByIdx(0)->GetGlobalId();
							}
							if ((right_lane_struct.length > length) || (fabs(right_lane_struct.length - length) < tolerance))
							{
								right_lane_struct.length = length;
								right_lane_struct.global_id = g_id;
								right_lane_struct.osipoints = osipoints;
							}
						}
					}
					for (int l_id=1;l_id<=connecting_road->GetLaneSectionByS(0,0)->GetNUmberOfLanesLeft();l_id++)
					{	
						if(connecting_road->GetLaneSectionByS(0)->GetLaneById(l_id)->IsDriving())
						{	
							if (connecting_road->GetLaneSectionByS(0,0)->GetLaneById(l_id)->GetLaneBoundaryGlobalId() != -1)
							{
								osipoints = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(l_id)->GetLaneBoundary()->GetOSIPoints();
								length = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(l_id)->GetLaneBoundary()->GetOSIPoints()->GetLength();
								g_id = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(l_id)->GetLaneBoundary()->GetGlobalId();
							}
							else
							{
								osipoints = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(l_id)->GetLaneRoadMarkByIdx(0)->GetLaneRoadMarkTypeByIdx(0)->GetLaneRoadMarkTypeLineByIdx(0)->GetOSIPoints();
								length = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(l_id)->GetLaneRoadMarkByIdx(0)->GetLaneRoadMarkTypeByIdx(0)->GetLaneRoadMarkTypeLineByIdx(0)->GetOSIPoints()->GetLength();
								g_id = connecting_road->GetLaneSectionByS(0,0)->GetLaneById(l_id)->GetLaneRoadMarkByIdx(0)->GetLaneRoadMarkTypeByIdx(0)->GetLaneRoadMarkTypeLineByIdx(0)->GetGlobalId();
							}
							if ((left_lane_struct.length > length) || (fabs(right_lane_struct.length - length) < tolerance))
							{
								left_lane_struct.length = length;
								left_lane_struct.global_id = g_id;
								left_lane_struct.osipoints = osipoints;
							}
						}
					}	
					if (fabs(left_lane_struct.length - right_lane_struct.length) < SMALL_NUMBER) 
					{
						left_lane_lengths.push_back(left_lane_struct);
						right_lane_lengths.push_back(right_lane_struct);
					}
					else
					{
						if (left_lane_struct.length < right_lane_struct.length)
						{
							lane_lengths.push_back(left_lane_struct);
						}
						else
						{
							lane_lengths.push_back(right_lane_struct);
						}
					}
				}
				// create all lane parings for the junction
				for (int l=0; l<connection->GetNumberOfLaneLinks();l++){
					junctionlanelink = connection->GetLaneLink(l);
					// check if the connecting road has been checked before, otherwise get the shortest laneboundary
					
					
					
					// TODO: will only work for right hand traffic right now
					if (((incomming_road_link_type == roadmanager::LinkType::SUCCESSOR && junctionlanelink->from_ < 0) ||
						(incomming_road_link_type == roadmanager::LinkType::PREDECESSOR && junctionlanelink->from_ > 0)) && 
						incomming_road->GetDrivingLaneById(incomming_s_value,junctionlanelink->from_) != 0)
					{
						osi3::Lane_Classification_LanePairing *laneparing = osi_lane->mutable_classification()->add_lane_pairing();
						laneparing->mutable_antecessor_lane_id()->set_value(incomming_road->GetDrivingLaneById(incomming_s_value,junctionlanelink->from_)->GetGlobalId());				
						laneparing->mutable_successor_lane_id()->set_value(outgoing_road->GetDrivingLaneById(outgoing_s_value, 
							connecting_road->GetDrivingLaneById(connecting_outgoing_s_value, junctionlanelink->to_)->GetLink(connecting_road_link_type)->GetId())->GetGlobalId());
					}
				}
			}
			// sort the correct free-boundaries
			for (int j=0;j<left_lane_lengths.size();j++)
			{
				// Tolerance for checking if 
				
				bool keep_right = true;
				bool keep_left = true;
				for (int k=0;k<lane_lengths.size();k++)
				{
					int same_left = roadmanager::CheckOverlapingOSIPoints(left_lane_lengths[j].osipoints,lane_lengths[k].osipoints,tolerance);
					if (same_left >0) 
					{
						keep_left = false;
					}
					int same_right = roadmanager::CheckOverlapingOSIPoints(right_lane_lengths[j].osipoints,lane_lengths[k].osipoints,tolerance);
					if (same_right >0)
					{
						keep_right = false;
					}
				}
				if (keep_left) 
				{
					tmp_lane_lengths.push_back(left_lane_lengths[j]);
				}
				else if (keep_right)
				{
					tmp_lane_lengths.push_back(right_lane_lengths[j]);
				}
			}
			for (int j=0;j<tmp_lane_lengths.size();j++)
			{
				lane_lengths.push_back(tmp_lane_lengths[j]);
			} 

			if (lane_lengths.size() == connected_roads.size())
			{
				for (int j=0;j<lane_lengths.size();j++)
				{
					osi3::Identifier* free_lane_id = osi_lane->mutable_classification()->add_free_lane_boundary_id();
					free_lane_id->set_value(lane_lengths[j].global_id);
				}	
			}
			else
			{
				std::vector<int> ids_to_remove;
				for (int j=0;j<lane_lengths.size();j++)
				{
					if (!(std::find(ids_to_remove.begin(), ids_to_remove.end(), j) != ids_to_remove.end()))
					{
						for (int k=0;k<lane_lengths.size();k++)
						{
							if (k!=j)
							{
								int same_points = roadmanager::CheckOverlapingOSIPoints(lane_lengths[k].osipoints,lane_lengths[j].osipoints,tolerance);
								if (same_points >0)
								{
									if (lane_lengths[k].length < lane_lengths[j].length)
									{
										ids_to_remove.push_back(j);
									}
									else
									{
										ids_to_remove.push_back(k);
										continue;
									}

								}
							}
						}
					}
				}
				
				for (int j=0;j<lane_lengths.size();j++)
				{
					if (!(std::find(ids_to_remove.begin(), ids_to_remove.end(), j) != ids_to_remove.end()))
					{
						osi3::Identifier* free_lane_id = osi_lane->mutable_classification()->add_free_lane_boundary_id();
						free_lane_id->set_value(lane_lengths[j].global_id);
					}
				}	
				LOG("Issues with the Intersection %i for the osi free lane boundary, none will be added.",junction->GetId());
			}
			
		}	
	}
	return 0;
}

int OSIReporter::UpdateOSILaneBoundary(std::vector<ObjectState*> objectState)
{
	//Retrieve opendrive class from RoadManager
	static roadmanager::OpenDrive* opendrive = roadmanager::Position::GetOpenDrive();

	//Loop over all roads
	for (int i = 0; i<opendrive->GetNumOfRoads(); i++)
	{

		roadmanager::Road* road = opendrive->GetRoadByIdx(i);

		// loop over all lane sections
		for (int j= 0; j<road->GetNumberOfLaneSections(); j++)
		{
			roadmanager::LaneSection* lane_section = road->GetLaneSectionByIdx(j);

			// loop over all lanes
			for (int k=0; k<lane_section->GetNumberOfLanes(); k++)
			{
				roadmanager::Lane* lane = lane_section->GetLaneByIdx(k);
				
				int n_roadmarks = lane->GetNumberOfRoadMarks();
				if (n_roadmarks != 0) // if there are road marks
				{
					// loop over RoadMarks
					for (int ii = 0; ii < lane->GetNumberOfRoadMarks(); ii++)
					{
						roadmanager::LaneRoadMark* laneroadmark = lane->GetLaneRoadMarkByIdx(ii);

						//loop over road mark types
						for (int jj = 0; jj < laneroadmark->GetNumberOfRoadMarkTypes(); jj++)
						{
							roadmanager::LaneRoadMarkType* laneroadmarktype = laneroadmark->GetLaneRoadMarkTypeByIdx(jj);

							// loop over LaneRoadMarkTypeLine
							for (int kk = 0; kk< laneroadmarktype->GetNumberOfRoadMarkTypeLines(); kk++)
							{
								roadmanager::LaneRoadMarkTypeLine* laneroadmarktypeline = laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(kk);

								osi3::LaneBoundary* osi_laneboundary = 0;
								int line_id = laneroadmarktypeline->GetGlobalId();

								// Check if this line is already pushed to OSI
								for (int h=0; h < obj_osi_internal.lnb.size(); h++)
								{
									if (obj_osi_internal.lnb[h]->mutable_id()->value() == line_id)
									{
										osi_laneboundary = obj_osi_internal.lnb[h];
									}
								}
								if (!osi_laneboundary)
								{
									osi_laneboundary = obj_osi_internal.gt->add_lane_boundary();

									// update id
									osi_laneboundary->mutable_id()->set_value(line_id);

									int n_osi_points = laneroadmarktypeline->GetOSIPoints()->GetNumOfOSIPoints();
									for (int h = 0; h < n_osi_points; h++)
									{
										osi3::LaneBoundary_BoundaryPoint* boundary_point = osi_laneboundary->add_boundary_line();
										boundary_point->mutable_position()->set_x(laneroadmarktypeline->GetOSIPoints()->GetXfromIdx(h));
										boundary_point->mutable_position()->set_y(laneroadmarktypeline->GetOSIPoints()->GetYfromIdx(h));
										boundary_point->mutable_position()->set_z(laneroadmarktypeline->GetOSIPoints()->GetZfromIdx(h));
										boundary_point->set_width(laneroadmarktypeline->GetWidth());
										boundary_point->set_height(laneroadmark->GetHeight());
									}

									// update classification type
									osi3::LaneBoundary_Classification_Type classific_type;
									osi3::LaneBoundary_Classification_Type osi_type = osi_laneboundary->mutable_classification()->type();
									switch(laneroadmark->GetType())
									{
										case roadmanager::LaneRoadMark::RoadMarkType::NONE_TYPE:
										classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_UNKNOWN;
										break;
										case roadmanager::LaneRoadMark::RoadMarkType::SOLID:
										classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
										break;
										case roadmanager::LaneRoadMark::RoadMarkType::BROKEN:
										classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE;
										break;
										default: classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
									}
									osi_laneboundary->mutable_classification()->set_type(classific_type);
									osi_type = osi_laneboundary->mutable_classification()->type();

									// update classification color
									osi3::LaneBoundary_Classification_Color classific_col;
									switch(laneroadmark->GetColor())
									{
										case roadmanager::LaneRoadMark::RoadMarkColor::STANDARD_COLOR:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::BLUE:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_BLUE;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::GREEN:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_GREEN;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::RED:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_RED;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::WHITE:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::YELLOW:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_YELLOW;
										break;
										default: classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
									}
									osi_laneboundary->mutable_classification()->set_color(classific_col);

									// update limiting structure id only if the type of lane boundary is set to TYPE_STRUCTURE - for now it is not implemented
									//osi_laneboundary->mutable_classification()->mutable_limiting_structure_id(0)->set_value(0);

									obj_osi_internal.lnb.push_back(osi_laneboundary);

								}
							}
						}
					}
				}
				else // if there are no road marks I take the lane boundary
				{
					roadmanager::LaneBoundaryOSI* laneboundary = lane->GetLaneBoundary();
					// Check if this line is already pushed to OSI
					int boundary_id = laneboundary->GetGlobalId();
					osi3::LaneBoundary* osi_laneboundary = 0;
					for (int h=0; h < obj_osi_internal.lnb.size(); h++)
					{
						if (obj_osi_internal.lnb[h]->mutable_id()->value() == boundary_id)
						{
							osi_laneboundary = obj_osi_internal.lnb[h];
						}
					}
					if (!osi_laneboundary)
					{
						osi_laneboundary = obj_osi_internal.gt->add_lane_boundary();

						// update id
						osi_laneboundary->mutable_id()->set_value(boundary_id);

						int n_osi_points = laneboundary->GetOSIPoints()->GetNumOfOSIPoints();
						for (int h = 0; h < n_osi_points; h++)
						{
							osi3::LaneBoundary_BoundaryPoint* boundary_point = osi_laneboundary->add_boundary_line();
							boundary_point->mutable_position()->set_x(laneboundary->GetOSIPoints()->GetXfromIdx(h));
							boundary_point->mutable_position()->set_y(laneboundary->GetOSIPoints()->GetYfromIdx(h));
							boundary_point->mutable_position()->set_z(laneboundary->GetOSIPoints()->GetZfromIdx(h));
							//boundary_point->set_width(laneboundary->GetWidth());
							//boundary_point->set_height(laneroadmark->GetHeight());
						}

						osi3::LaneBoundary_Classification_Type classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_NO_LINE;
						osi_laneboundary->mutable_classification()->set_type(classific_type);

						osi3::LaneBoundary_Classification_Color classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_UNKNOWN;
						osi_laneboundary->mutable_classification()->set_color(classific_col);

						obj_osi_internal.lnb.push_back(osi_laneboundary);
					}
				}
			}
		}
	}

	return 0;
}

int OSIReporter::UpdateOSIRoadLane(std::vector<ObjectState*> objectState)
{
	//Retrieve opendrive class from RoadManager
	static roadmanager::OpenDrive* opendrive = roadmanager::Position::GetOpenDrive();

	// Loop over all roads
	for (int i = 0; i<opendrive->GetNumOfRoads(); i++)
	{

		roadmanager::Road* road = opendrive->GetRoadByIdx(i);

		// loop over all lane sections
		for (int j= 0; j<road->GetNumberOfLaneSections(); j++)
		{
			roadmanager::LaneSection* lane_section = road->GetLaneSectionByIdx(j);

			// loop over all lanes
			for (int k=0; k<lane_section->GetNumberOfLanes(); k++)
			{
				roadmanager::Lane* lane = lane_section->GetLaneByIdx(k);
				if ((!lane->IsCenter() && !lane->IsOSIIntersection()))
				{
					osi3::Lane* osi_lane = 0;
					int lane_global_id = lane->GetGlobalId();
					int lane_id = lane->GetId();


					// Check if this lane is already pushed to OSI - if yes just update
					for (int jj=0; jj < obj_osi_internal.ln.size(); jj++)
					{
						if (obj_osi_internal.ln[jj]->mutable_id()->value() == lane_global_id)
						{
							osi_lane = obj_osi_internal.ln[jj];
							break;
						}
					}
					// if the lane is not already in the osi message we add it all
					if (!osi_lane)
					{
						// LANE ID
						osi_lane = obj_osi_internal.gt->add_lane();
						osi_lane->mutable_id()->set_value(lane_global_id);

						// CLASSIFICATION TYPE
						roadmanager::Lane::LaneType lanetype = lane->GetLaneType();
						osi3::Lane_Classification_Type class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_UNKNOWN;

						if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_DRIVING 		||
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_PARKING		||
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BIDIRECTIONAL 	 )
						{
							class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
						}
						else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_STOP 		||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BIKING 		||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SIDEWALK 	||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BORDER   	||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_RESTRICTED   ||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ROADMARKS    ||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_TRAM   		||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_RAIL   			 )
						{
							class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
						}
						else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ENTRY 		||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_EXIT 		||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_OFF_RAMP 	||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ON_RAMP 		||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_MEDIAN 		||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SHOULDER 		 )
						{
							class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
						}
						else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL1 	||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL2 	||
								lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL3 		 )
						{
							class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_OTHER;
						}
						else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_NONE )
						{
							class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_UNKNOWN;
						}
						osi_lane->mutable_classification()->set_type(class_type);

						// CENTERLINE POINTS
						int n_osi_points = lane->GetOSIPoints()->GetNumOfOSIPoints();
						for (int jj = 0; jj < n_osi_points; jj++)
						{
							osi3::Vector3d* centerLine = osi_lane->mutable_classification()->add_centerline();
							centerLine->set_x(lane->GetOSIPoints()->GetXfromIdx(jj));
							centerLine->set_y(lane->GetOSIPoints()->GetYfromIdx(jj));
							centerLine->set_z(lane->GetOSIPoints()->GetZfromIdx(jj));
						}

						// DRIVING DIRECTION
						int driving_side = 1; //right, along side of s-direction
						bool driving_direction = true;
						if (lane_id >= 0)
						{
							driving_side = -1; //left,  against side of s-direction
							driving_direction = false;
						}
						osi_lane->mutable_classification()->set_centerline_is_driving_direction(driving_direction);

						// LEFT AND RIGHT LANE IDS
						std::vector< std::pair <int,int> > globalid_ids_left;
						std::vector< std::pair <int,int> > globalid_ids_right;


						if (lane_section->IsOSILaneById(lane_id-(1*driving_side)))
						{
							globalid_ids_right.push_back( std::make_pair(lane_id-(1*driving_side), lane_section->GetLaneGlobalIdById(lane_id-(1*driving_side)) ) );
						}
						else if (lane_section->IsOSILaneById(lane_id-(2*driving_side)))
						{
							globalid_ids_right.push_back( std::make_pair(lane_id-(2*driving_side) , lane_section->GetLaneGlobalIdById(lane_id-(2*driving_side)) ) );
						}

						if (lane_section->IsOSILaneById(lane_id+(1*driving_side)))
						{
							globalid_ids_left.push_back( std::make_pair(lane_id+(1*driving_side) , lane_section->GetLaneGlobalIdById(lane_id+(1*driving_side)) ) );
						}
						else if (lane_section->IsOSILaneById(lane_id+(2*driving_side)))
						{
							globalid_ids_left.push_back( std::make_pair(lane_id+(2*driving_side) , lane_section->GetLaneGlobalIdById(lane_id+(2*driving_side)) ) );
						}

						// order global id with local id to maintain geographical order
						std::sort(globalid_ids_left.begin(), globalid_ids_left.end());
						std::sort(globalid_ids_right.begin(), globalid_ids_right.end());
						if(driving_direction)
						{
							std::reverse(globalid_ids_right.begin(), globalid_ids_right.end());
						}
						else
						{
							std::reverse(globalid_ids_left.begin(), globalid_ids_left.end());
						}



						for (int jj = 0; jj < globalid_ids_left.size(); jj++)
						{
							osi3::Identifier* left_id = osi_lane->mutable_classification()->add_left_adjacent_lane_id();
							left_id->set_value((uint64_t)globalid_ids_left[jj].second);
						}
						for (int jj = 0; jj < globalid_ids_right.size(); jj++)
						{
							osi3::Identifier* right_id = osi_lane->mutable_classification()->add_right_adjacent_lane_id();
							right_id->set_value((uint64_t)globalid_ids_right[jj].second);
						}



						// LANE BOUNDARY IDS
						if (lane_id == 0) // for central lane I use the laneboundary osi points as right and left boundary so that it can be used from both sides
						{
							// check if lane has road mark
							std::vector<int> line_ids = lane->GetLineGlobalIds();
							if (!line_ids.empty()) // lane has RoadMarks
							{
								for (int jj = 0; jj < line_ids.size(); jj++ )
								{
									osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
									left_lane_bound_id->set_value(line_ids[jj]);
									osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
									right_lane_bound_id->set_value(line_ids[jj]);
								}
							}
							else // no road marks -> we take lane boundary
							{
								int laneboundary_global_id = lane->GetLaneBoundaryGlobalId();
								if (laneboundary_global_id >= 0)
								{
									osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
									left_lane_bound_id->set_value(laneboundary_global_id);
									osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
									right_lane_bound_id->set_value(laneboundary_global_id);
								}
							}
						}
						else
						{
							// Set left/right laneboundary ID for left/right lanes- we use LaneMarks is they exist, if not we take laneboundary
							std::vector<int> line_ids = lane->GetLineGlobalIds();
							if (!line_ids.empty()) // lane has RoadMarks
							{
								for (int jj = 0; jj < line_ids.size(); jj++ )
								{
									if (lane_id > 0 )
									{
										osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
										right_lane_bound_id->set_value(line_ids[jj]);
									}
									if (lane_id < 0 )
									{
										osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
										right_lane_bound_id->set_value(line_ids[jj]);
									}
								}
							}
							else
							{
								int laneboundary_global_id = lane->GetLaneBoundaryGlobalId();
								if (lane_id > 0 && laneboundary_global_id >= 0)
								{
									osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
									right_lane_bound_id->set_value(laneboundary_global_id);
								}
								if (lane_id < 0 && laneboundary_global_id >= 0)
								{
									osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
									right_lane_bound_id->set_value(laneboundary_global_id);
								}
							}

							// Set right/left laneboundary ID for left/right lanes - we look at neightbour lanes
							int next_lane_id = 0;
							if (lane_id < 0) // if lane is on the right, then it contains its right boundary. So I need to look into its left lane for the left boundary
							{
								next_lane_id = lane_id+1;
							}
							else if (lane_id > 0) // if lane is on the left, then it contains its left boundary. So I need to look into its right lane for the right boundary
							{
								next_lane_id = lane_id-1;
							}
							// look at right lane and check if it has Lines for RoadMarks
							roadmanager::Lane* next_lane = lane_section->GetLaneById(next_lane_id);
							std::vector<int> nextlane_line_ids = next_lane->GetLineGlobalIds();
							if (!nextlane_line_ids.empty())
							{
								for (int jj = 0; jj < nextlane_line_ids.size(); jj++ )
								{
									if (lane_id<0)
									{
										osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
										left_lane_bound_id->set_value(nextlane_line_ids[jj]);
									}
									else if (lane_id>0)
									{
										osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
										left_lane_bound_id->set_value(nextlane_line_ids[jj]);
									}
								}
							}
							else // if the neightbour lane does not have Lines for RoadMakrs we take the LaneBoundary
							{
								int next_laneboundary_global_id = next_lane->GetLaneBoundaryGlobalId();
								if (lane_id < 0 && next_laneboundary_global_id >= 0)
								{
									osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
									left_lane_bound_id->set_value(next_laneboundary_global_id);
								}
								if (lane_id > 0 && next_laneboundary_global_id >= 0)
								{
									osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
									left_lane_bound_id->set_value(next_laneboundary_global_id);
								}
							}
						}


						// STILL TO DO:
						double temp = 0;
						osi_lane->mutable_classification()->mutable_road_condition()->set_surface_temperature(temp);
						osi_lane->mutable_classification()->mutable_road_condition()->set_surface_water_film(temp);
						osi_lane->mutable_classification()->mutable_road_condition()->set_surface_freezing_point(temp);
						osi_lane->mutable_classification()->mutable_road_condition()->set_surface_ice(temp);
						osi_lane->mutable_classification()->mutable_road_condition()->set_surface_roughness(temp);
						osi_lane->mutable_classification()->mutable_road_condition()->set_surface_texture(temp);

						obj_osi_internal.ln.push_back(osi_lane);
					}
				}
			}
		}
	}

	return 0;
}

const char* OSIReporter::GetOSIGroundTruth(int* size)
{
	if (!(GetSocket() || IsFileOpen()))
	{
		// Data has not been serialized
		obj_osi_internal.gt->SerializeToString(&osiGroundTruth.ground_truth);
		osiGroundTruth.size = (unsigned int)obj_osi_internal.gt->ByteSizeLong();
	}
	*size = osiGroundTruth.size;
	return osiGroundTruth.ground_truth.data();
}

const char* OSIReporter::GetOSIGroundTruthRaw()
{
	return (const char*) obj_osi_internal.gt;
}

const char* OSIReporter::GetOSIRoadLane(std::vector<ObjectState*> objectState, int* size, int object_id)
{
	// Check if object_id exists
	if (object_id >= objectState.size())
	{
		LOG("Object %d not available, only %d registered", object_id, objectState.size());
		*size = 0;
		return 0;
	}

	// Find position of the object
	roadmanager::Position pos;
	for (size_t i = 0; i < objectState.size() ; i++)
	{
		if (object_id == objectState[i]->state_.info.id)
		{
			pos = objectState[i]->state_.pos;
			break;
		}
	}

	// find the lane in the sensor view and save its index in the sensor view
	int lane_id_of_vehicle = pos.GetLaneGlobalId();
	int idx = -1;
	for (int i = 0; i<obj_osi_internal.ln.size(); i++)
	{
		osi3::Identifier identifier = obj_osi_internal.ln[i]->id();
		int found_id = (int)identifier.value();
		if (found_id == lane_id_of_vehicle)
		{
			idx = i;
			break;
		}
	}
	if (idx < 0)
	{
		LOG("Failed to locate vehicle lane id!");
		return 0;
	}
	// serialize to string the single lane
	obj_osi_internal.ln[idx]->SerializeToString(&osiRoadLane.osi_lane_info);
	osiRoadLane.size = (unsigned int)obj_osi_internal.ln[idx]->ByteSizeLong();
	*size = osiRoadLane.size;
	return osiRoadLane.osi_lane_info.data();
}


const char* OSIReporter::GetOSIRoadLaneBoundary(int* size, int global_id)
{
	// find the lane bounday in the sensor view and save its index
	int idx = -1;
	for (int i = 0; i<obj_osi_internal.lnb.size(); i++)
	{
		osi3::Identifier identifier = obj_osi_internal.lnb[i]->id();
		int found_id = (int)identifier.value();
		if (found_id == global_id)
		{
			idx = i;
			break;
		}
	}

	if (idx == -1)
	{
		return 0;
	}

	// serialize to string the single lane
	obj_osi_internal.lnb[idx]->SerializeToString(&osiRoadLaneBoundary.osi_lane_boundary_info);
	osiRoadLaneBoundary.size = (unsigned int)obj_osi_internal.lnb[idx]->ByteSizeLong();
	*size = osiRoadLaneBoundary.size;
	return osiRoadLaneBoundary.osi_lane_boundary_info.data();
}

bool OSIReporter::IsCentralOSILane(int lane_idx)
{
	// to check if the lane is a central lane we check if the right and left lane boundary have the same global id.
	osi3::Identifier Left_lb_id = obj_osi_internal.ln[lane_idx]->mutable_classification()->left_lane_boundary_id(0);
	int left_lb_id = (int)Left_lb_id.value();

	osi3::Identifier Right_lb_id = obj_osi_internal.ln[lane_idx]->mutable_classification()->right_lane_boundary_id(0);
	int right_lb_id = (int)Right_lb_id.value();

	if (left_lb_id == right_lb_id)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int OSIReporter::GetLaneIdxfromIdOSI(int lane_id)
{
	int idx = -1;
	for (int i = 0; i<obj_osi_internal.ln.size(); i++)
	{
		osi3::Identifier identifier = obj_osi_internal.ln[i]->id();
		int found_id = (int)identifier.value();
		if (found_id == lane_id)
		{
			idx = i;
			break;
		}
	}
	return idx;
}

void OSIReporter::GetOSILaneBoundaryIds(std::vector<ObjectState*> objectState, std::vector<int> &ids, int object_id)
{
	int idx_central, idx_left, idx_right;
	int left_lb_id, right_lb_id;
	int far_left_lb_id, far_right_lb_id;
	std::vector<int> final_lb_ids;

	// Check if object_id exists
	if (object_id >= objectState.size())
	{
		LOG("Object %d not available, only %d registered", object_id, objectState.size());
		ids = {-1, -1, -1, -1};
		return;
	}

	// Find position of the object
	roadmanager::Position pos;
	for (size_t i = 0; i < objectState.size(); i++)
	{
		if (object_id == objectState[i]->state_.info.id)
		{
			pos = objectState[i]->state_.pos;
		}
	}

	// find the lane in the sensor view and save its index
	int lane_id_of_vehicle = pos.GetLaneGlobalId();
	idx_central = GetLaneIdxfromIdOSI(lane_id_of_vehicle);

	// find left and right lane boundary ids of central lane
	if (obj_osi_internal.ln[idx_central]->mutable_classification()->left_lane_boundary_id_size() == 0 )
	{
		left_lb_id = -1;
	}
	else
	{
		osi3::Identifier left_lane = obj_osi_internal.ln[idx_central]->mutable_classification()->left_lane_boundary_id(0);
		left_lb_id = (int)left_lane.value();
	}

	if (obj_osi_internal.ln[idx_central]->mutable_classification()->right_lane_boundary_id_size() == 0 )
	{
		right_lb_id = -1;
	}
	else
	{
		osi3::Identifier right_lane = obj_osi_internal.ln[idx_central]->mutable_classification()->right_lane_boundary_id(0);
		right_lb_id = (int)right_lane.value();
	}

	// find first left lane
	if (obj_osi_internal.ln[idx_central]->mutable_classification()->left_adjacent_lane_id_size() == 0 )
	{
		far_left_lb_id = -1;
	}
	else
	{

		osi3::Identifier Left_lane_id = obj_osi_internal.ln[idx_central]->mutable_classification()->left_adjacent_lane_id(0);
		int left_lane_id = (int)Left_lane_id.value();
		idx_left = GetLaneIdxfromIdOSI(left_lane_id);

		// save left boundary of left lane as far left lane boundary of central lane
		if (obj_osi_internal.ln[idx_left]->mutable_classification()->left_lane_boundary_id_size() == 0 )
		{
			far_left_lb_id = -1;
		}
		else
		{
			if (obj_osi_internal.ln[idx_central]->mutable_classification()->centerline_is_driving_direction() != obj_osi_internal.ln[idx_left]->mutable_classification()->centerline_is_driving_direction()) // if not same drv dir
			{
				osi3::Identifier Far_left_lb_id = obj_osi_internal.ln[idx_left]->mutable_classification()->right_lane_boundary_id(0);
				far_left_lb_id = (int)Far_left_lb_id.value();
			}
			else
			{
				osi3::Identifier Far_left_lb_id = obj_osi_internal.ln[idx_left]->mutable_classification()->left_lane_boundary_id(0);
				far_left_lb_id = (int)Far_left_lb_id.value();
			}
		}
	}


	// now find first right lane
	if (obj_osi_internal.ln[idx_central]->mutable_classification()->right_adjacent_lane_id_size() == 0 )
	{
		far_right_lb_id = -1;
	}
	else
	{
		osi3::Identifier Right_lane_id = obj_osi_internal.ln[idx_central]->mutable_classification()->right_adjacent_lane_id(0);
		int right_lane_id = (int)Right_lane_id.value();
		idx_right = GetLaneIdxfromIdOSI(right_lane_id);

		// save right boundary of right lane as far right lane boundary of central lane
		if (obj_osi_internal.ln[idx_right]->mutable_classification()->right_lane_boundary_id_size() == 0 )
		{
			far_right_lb_id = -1;
		}
		else
		{
			if (obj_osi_internal.ln[idx_central]->mutable_classification()->centerline_is_driving_direction() != obj_osi_internal.ln[idx_right]->mutable_classification()->centerline_is_driving_direction()) // if not same drv dir
			{
				osi3::Identifier Far_right_lb_id = obj_osi_internal.ln[idx_right]->mutable_classification()->left_lane_boundary_id(0);
				far_right_lb_id = (int)Far_right_lb_id.value();
			}
			else
			{
				osi3::Identifier Far_right_lb_id = obj_osi_internal.ln[idx_right]->mutable_classification()->right_lane_boundary_id(0);
				far_right_lb_id = (int)Far_right_lb_id.value();
			}
		}

	}

	// push all ids into output vector
	final_lb_ids.push_back(far_left_lb_id);
	final_lb_ids.push_back(left_lb_id);
	final_lb_ids.push_back(right_lb_id);
	final_lb_ids.push_back(far_right_lb_id);

	ids = final_lb_ids;

	return;

}

const char* OSIReporter::GetOSISensorDataRaw()
{
    return (const char*) obj_osi_internal.sd;
}

int OSIReporter::SetOSITimeStampExplicit(unsigned long long int nanoseconds)
{
	nanosec_ = nanoseconds;

	return 0;
}

