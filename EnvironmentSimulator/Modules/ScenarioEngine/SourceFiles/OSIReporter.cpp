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
#include "OSITrafficCommand.hpp"
#include <cmath>
#include <string>
#include <utility>

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

#define OSI_OUT_PORT          48198
#define OSI_MAX_UDP_DATA_SIZE 8192

// Large OSI messages needs to be split for UDP transmission
// This struct must be mached on receiver side
static struct
{
    int          counter;
    unsigned int datasize;
    char         data[OSI_MAX_UDP_DATA_SIZE];
} osi_udp_buf;

typedef struct
{
    std::string  ground_truth;
    unsigned int size;
} OSIGroundTruth;

typedef struct
{
    std::string  osi_lane_info;
    unsigned int size;
} OSIRoadLane;

typedef struct
{
    std::string  osi_lane_boundary_info;
    unsigned int size;
} OSIRoadLaneBoundary;

typedef struct
{
    std::string  traffic_command;
    unsigned int size;
} OSITrafficCommand;

static struct
{
    osi3::SensorData                 *sd;
    osi3::GroundTruth                *gt;
    osi3::StationaryObject           *sobj;
    osi3::TrafficSign                *ts;
    osi3::MovingObject               *mobj;
    std::vector<osi3::Lane *>         ln;
    std::vector<osi3::LaneBoundary *> lnb;
} obj_osi_internal;

static struct
{
    osi3::GroundTruth    *gt;
    osi3::SensorView     *sv;
    osi3::TrafficCommand *tc;
} obj_osi_external;

using namespace scenarioengine;

static OSIGroundTruth      osiGroundTruth;
static OSIRoadLane         osiRoadLane;
static OSIRoadLaneBoundary osiRoadLaneBoundary;
static OSITrafficCommand   osiTrafficCommand;

// ScenarioGateway

OSIReporter::OSIReporter(ScenarioEngine *scenarioengine)
{
    udp_client_      = nullptr;
    scenario_engine_ = scenarioengine;

    obj_osi_internal.gt = new osi3::GroundTruth();
    obj_osi_external.gt = new osi3::GroundTruth();
    obj_osi_external.sv = new osi3::SensorView();
    obj_osi_external.tc = new osi3::TrafficCommand();

    obj_osi_internal.gt->mutable_version()->set_version_major(3);
#ifdef _OSI_VERSION_3_3_1
    obj_osi_internal.gt->mutable_version()->set_version_minor(3);
    obj_osi_internal.gt->mutable_version()->set_version_patch(1);
#else
    obj_osi_internal.gt->mutable_version()->set_version_minor(5);
    obj_osi_internal.gt->mutable_version()->set_version_patch(0);
#endif

    obj_osi_internal.gt->mutable_timestamp()->set_seconds(0);
    obj_osi_internal.gt->mutable_timestamp()->set_nanos(0);

    obj_osi_external.tc->mutable_timestamp()->set_seconds(0);
    obj_osi_external.tc->mutable_timestamp()->set_nanos(0);

    // Sensor Data
    obj_osi_internal.sd = new osi3::SensorData();

    // Counter for OSI update
    osi_update_counter_ = 0;

    nanosec_ = 0xffffffffffffffff;  // indicate not set
}

OSIReporter::~OSIReporter()
{
    if (obj_osi_internal.gt)
    {
        obj_osi_internal.gt->Clear();
        delete obj_osi_internal.gt;
    }

    if (obj_osi_external.gt)
    {
        obj_osi_external.gt->Clear();
        delete obj_osi_external.gt;
    }

    if (obj_osi_internal.sd)
    {
        obj_osi_internal.sd->Clear();
        delete obj_osi_internal.sd;
    }

    if (obj_osi_external.sv)
    {
        obj_osi_external.sv->Clear();
        delete obj_osi_external.sv;
    }

    if (obj_osi_external.tc)
    {
        obj_osi_external.tc->Clear();
        delete obj_osi_external.tc;
    }

    obj_osi_internal.ln.clear();
    obj_osi_internal.lnb.clear();

    osiGroundTruth.size    = 0;
    osiRoadLane.size       = 0;
    osiTrafficCommand.size = 0;

    delete udp_client_;

    if (osi_file.is_open())
    {
        osi_file.close();
    }
}

SE_SOCKET OSIReporter::OpenSocket(std::string ipaddr)
{
    udp_client_ = new UDPClient(OSI_OUT_PORT, ipaddr);

    return udp_client_->GetStatus();
}

void OSIReporter::ReportSensors(std::vector<ObjectSensor *> sensor)
{
    if (sensor.size() == 0)
    {
        return;
    }
    while (obj_osi_internal.sd->sensor_view_size() < static_cast<int>(sensor.size()))
    {
        obj_osi_internal.sd->add_sensor_view();
    }
    for (unsigned int i = 0; i < sensor.size(); i++)
    {
        // Clear history
        obj_osi_internal.sd->mutable_sensor_view(static_cast<int>(i))->mutable_global_ground_truth()->clear_moving_object();
        for (unsigned int j = 0; j < static_cast<unsigned int>(sensor[i]->nObj_); j++)
        {
            // Create moving object
            osi3::MovingObject *mobj;
            mobj = obj_osi_internal.sd->mutable_sensor_view(static_cast<int>(i))->mutable_global_ground_truth()->add_moving_object();

            // Populate sensor data
            mobj->mutable_id()->set_value(static_cast<unsigned int>(sensor[i]->hitList_[j].obj_->id_));
            mobj->mutable_base()->mutable_position()->set_x(sensor[i]->hitList_[j].x_ +
                                                            static_cast<double>(sensor[i]->hitList_[j].obj_->boundingbox_.center_.x_) *
                                                                cos(sensor[i]->hitList_[j].yaw_));
            mobj->mutable_base()->mutable_position()->set_y(sensor[i]->hitList_[j].y_ +
                                                            static_cast<double>(sensor[i]->hitList_[j].obj_->boundingbox_.center_.x_) *
                                                                sin(sensor[i]->hitList_[j].yaw_));
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

bool OSIReporter::OpenOSIFile(const char *filename)
{
    osi_file.open(filename, std::ios_base::binary);
    if (!osi_file.good())
    {
        LOG("Failed open OSI tracefile %s", filename);
        return false;
    }
    LOG("OSI tracefile %s opened", filename);
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
    osi_file.write(reinterpret_cast<char *>(&osiGroundTruth.size), sizeof(osiGroundTruth.size));

    // write to file, actual message - the groundtruth object including timestamp and moving objects
    osi_file.write(osiGroundTruth.ground_truth.c_str(), osiGroundTruth.size);

    // write to file, first size of message
    // osi_file.write(reinterpret_cast<char *>(&osiTrafficCommand.size), sizeof(osiTrafficCommand.size));

    // write to file, actual message - the groundtruth object including timestamp and moving objects
    // osi_file.write(osiTrafficCommand.traffic_command.c_str(), osiTrafficCommand.size);

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
    obj_osi_external.gt->clear_moving_object();
    obj_osi_external.gt->clear_stationary_object();
    obj_osi_external.gt->clear_lane();
    obj_osi_external.gt->clear_lane_boundary();
    obj_osi_external.gt->clear_traffic_light();
    obj_osi_external.gt->clear_traffic_sign();
    obj_osi_external.gt->clear_road_marking();

    return 0;
}

int OSIReporter::UpdateOSIGroundTruth(const std::vector<std::unique_ptr<ObjectState>> &objectState)
{
    if (GetUpdated() == true)
    {
        // already updated within this scenario frame, skip
        return 0;
    }

    if (GetCounter() == 0)
    {
        UpdateOSIStaticGroundTruth(objectState);
    }
    else if (GetCounter() == 1)
    {
        // Clear the static data now when it has been reported once
        ClearOSIGroundTruth();
    }

    UpdateOSIDynamicGroundTruth(objectState);

    if (GetUDPClientStatus() == 0 || IsFileOpen())
    {
        obj_osi_external.gt->SerializeToString(&osiGroundTruth.ground_truth);
        osiGroundTruth.size = static_cast<unsigned int>(obj_osi_external.gt->ByteSizeLong());
    }

    if (GetUDPClientStatus() == 0)
    {
        // send over udp - split large OSI messages in multiple transmissions
        unsigned int sentDataBytes = 0;

        for (osi_udp_buf.counter = 1; sentDataBytes < osiGroundTruth.size; osi_udp_buf.counter++)
        {
            osi_udp_buf.datasize = MIN(osiGroundTruth.size - sentDataBytes, OSI_MAX_UDP_DATA_SIZE);
            memcpy(osi_udp_buf.data, &osiGroundTruth.ground_truth.c_str()[sentDataBytes], osi_udp_buf.datasize);
            int packSize = static_cast<int>(sizeof(osi_udp_buf)) - static_cast<int>((OSI_MAX_UDP_DATA_SIZE - osi_udp_buf.datasize));

            if (sentDataBytes + osi_udp_buf.datasize >= osiGroundTruth.size)
            {
                // Last package indicated by negative counter number
                osi_udp_buf.counter = -osi_udp_buf.counter;
            }

            int sendResult = udp_client_->Send(reinterpret_cast<char *>(&osi_udp_buf), static_cast<unsigned int>(packSize));  // TODO: @Emil

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

    IncrementCounter();
    SetUpdated(true);

    return 0;
}

int OSIReporter::UpdateOSIStaticGroundTruth(const std::vector<std::unique_ptr<ObjectState>> &objectState)
{
    // First pick objects from the OpenSCENARIO description
    static roadmanager::OpenDrive *opendrive = roadmanager::Position::GetOpenDrive();
    for (size_t i = 0; i < static_cast<unsigned int>(opendrive->GetNumOfRoads()); i++)
    {
        roadmanager::Road *road = opendrive->GetRoadByIdx(static_cast<int>(i));
        if (road)
        {
            for (size_t j = 0; j < static_cast<unsigned int>(road->GetNumberOfObjects()); j++)
            {
                roadmanager::RMObject *object = road->GetRoadObject(static_cast<int>(j));
                if (object)
                {
                    UpdateOSIStationaryObjectODR(road->GetId(), object);
                }
            }
        }
    }

    // Then pick objects from the OpenSCENARIO description
    for (size_t i = 0; i < objectState.size(); i++)
    {
        if (objectState[i]->state_.info.obj_type == static_cast<int>(Object::Type::VEHICLE) ||
            objectState[i]->state_.info.obj_type == static_cast<int>(Object::Type::PEDESTRIAN))
        {
            // do nothing
        }
        else if (objectState[i]->state_.info.obj_type == static_cast<int>(Object::Type::MISC_OBJECT))
        {
            UpdateOSIStationaryObject(objectState[i].get());
        }
        else
        {
            LOG("Warning: Object type %d is not supported in OSIReporter, and hence no OSI update for this object",
                objectState[i]->state_.info.obj_type);
        }
    }

    UpdateOSIRoadLane();
    UpdateOSILaneBoundary();
    UpdateOSIIntersection();
    UpdateTrafficSignals();

    // Set GeoReference in OSI as map_reference
    obj_osi_external.gt->set_map_reference(opendrive->GetGeoReferenceAsString());

    obj_osi_external.gt->mutable_stationary_object()->CopyFrom(*obj_osi_internal.gt->mutable_stationary_object());
    obj_osi_external.gt->mutable_lane()->CopyFrom(*obj_osi_internal.gt->mutable_lane());
    obj_osi_external.gt->mutable_lane_boundary()->CopyFrom(*obj_osi_internal.gt->mutable_lane_boundary());
    obj_osi_external.gt->mutable_traffic_sign()->CopyFrom(*obj_osi_internal.gt->mutable_traffic_sign());
    obj_osi_external.gt->mutable_traffic_light()->CopyFrom(*obj_osi_internal.gt->mutable_traffic_light());
    obj_osi_external.gt->mutable_road_marking()->CopyFrom(*obj_osi_internal.gt->mutable_road_marking());

    obj_osi_external.gt->set_model_reference(stationary_model_reference);

    return 0;
}

int OSIReporter::UpdateOSIDynamicGroundTruth(const std::vector<std::unique_ptr<ObjectState>> &objectState, bool reportGhost)
{
    obj_osi_internal.gt->clear_moving_object();
    obj_osi_internal.gt->clear_timestamp();

    if (IsTimeStampSetExplicit())
    {
        // use excplicit timestamp
        obj_osi_internal.gt->mutable_timestamp()->set_seconds(static_cast<int64_t>((nanosec_ / 1000000000)));
        obj_osi_internal.gt->mutable_timestamp()->set_nanos(static_cast<uint32_t>((nanosec_ % 1000000000)));
    }
    else if (objectState.size() > 0)
    {
        // use timstamp from object state
        obj_osi_internal.gt->mutable_timestamp()->set_seconds(static_cast<int64_t>(objectState[0]->state_.info.timeStamp));
        obj_osi_internal.gt->mutable_timestamp()->set_nanos(
            static_cast<uint32_t>(((objectState[0]->state_.info.timeStamp - floor(objectState[0]->state_.info.timeStamp)) * 1e9)));
    }
    else
    {
        // report time = 0
        obj_osi_internal.gt->mutable_timestamp()->set_seconds(static_cast<int64_t>(0));
        obj_osi_internal.gt->mutable_timestamp()->set_nanos(static_cast<uint32_t>(0));
    }

    for (size_t i = 0; i < objectState.size(); i++)
    {
        if (objectState[i]->state_.info.obj_type == static_cast<int>(Object::Type::VEHICLE) ||
            objectState[i]->state_.info.obj_type == static_cast<int>(Object::Type::PEDESTRIAN))
        {
            if (reportGhost)
            {
                UpdateOSIMovingObject(objectState[i].get());
            }
            else
            {
                if (objectState[i]->state_.info.ctrl_type != Controller::Type::GHOST_RESERVED_TYPE)
                {
                    UpdateOSIMovingObject(objectState[i].get());
                }
            }
        }
        else if (objectState[i]->state_.info.obj_type == static_cast<int>(Object::Type::MISC_OBJECT))
        {
            // do nothing
        }
        else
        {
            LOG("Warning: Object type %d is not supported in OSIReporter, and hence no OSI update for this object",
                objectState[i]->state_.info.obj_type);
        }
    }

    obj_osi_external.gt->mutable_timestamp()->CopyFrom(*obj_osi_internal.gt->mutable_timestamp());
    obj_osi_external.gt->mutable_moving_object()->CopyFrom(*obj_osi_internal.gt->mutable_moving_object());

    return 0;
}

int OSIReporter::UpdateOSIHostVehicleData(ObjectState *objectState)
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
    // obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_orientation()->set_yaw(GetAngleInIntervalMinusPIPlusPI(objectState->state_.pos.GetH()));
    // obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_orientation_rate()->set_yaw(objectState->state_.pos.GetHRate());
    // obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_orientation_acceleration()->set_yaw(objectState->state_.pos.GetHAcc());
    // obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_dimension()->set_height(objectState->state_.boundingbox.dimensions_.height_);
    // obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_dimension()->set_width(objectState->state_.boundingbox.dimensions_.width_);
    // obj_osi_internal.sv->mutable_host_vehicle_data()->mutable_location()->mutable_dimension()->set_length(objectState->state_.boundingbox.dimensions_.length_);

    return 0;
}

int OSIReporter::UpdateOSIStationaryObjectODR(int road_id, roadmanager::RMObject *object)
{
    (void)road_id;
    // Create OSI Stationary Object
    obj_osi_internal.sobj = obj_osi_internal.gt->add_stationary_object();

    // Set OSI Stationary Object Mutable ID
    int sobj_size = obj_osi_internal.gt->mutable_stationary_object()->size();
    obj_osi_internal.sobj->mutable_id()->set_value(static_cast<unsigned int>(sobj_size - 1));

    // Set OSI Stationary Object Type and Classification
    if (object->GetType() == roadmanager::RMObject::ObjectType::POLE)
    {
        obj_osi_internal.sobj->mutable_classification()->set_type(
            osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_POLE);
    }
    else if (object->GetType() == roadmanager::RMObject::ObjectType::TREE)
    {
        obj_osi_internal.sobj->mutable_classification()->set_type(
            osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_TREE);
    }
    else if (object->GetType() == roadmanager::RMObject::ObjectType::VEGETATION)
    {
        obj_osi_internal.sobj->mutable_classification()->set_type(
            osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_VEGETATION);
    }
    else if (object->GetType() == roadmanager::RMObject::ObjectType::BARRIER)
    {
        obj_osi_internal.sobj->mutable_classification()->set_type(
            osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BARRIER);
    }
    else if (object->GetType() == roadmanager::RMObject::ObjectType::BUILDING)
    {
        obj_osi_internal.sobj->mutable_classification()->set_type(
            osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BUILDING);
    }
    else if (object->GetType() == roadmanager::RMObject::ObjectType::PARKINGSPACE)
    {
        obj_osi_internal.sobj->mutable_classification()->set_type(
            osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_OTHER);
        obj_osi_internal.sobj->mutable_classification()->set_material(
            osi3::StationaryObject_Classification_Material::StationaryObject_Classification_Material_MATERIAL_CONCRETE);
        obj_osi_internal.sobj->mutable_classification()->set_density(
            osi3::StationaryObject_Classification_Density::StationaryObject_Classification_Density_DENSITY_SOLID);
        obj_osi_internal.sobj->mutable_classification()->set_color(
            osi3::StationaryObject_Classification_Color::StationaryObject_Classification_Color_COLOR_GREY);

        osi3::ExternalReference *scource_reference = obj_osi_internal.sobj->add_source_reference();
        std::string             *identifier        = scource_reference->add_identifier();

        std::string restrictions_string = object->GetParkingSpace().GetRestrictions();
        identifier->assign(restrictions_string);
    }
    else if (object->GetType() == roadmanager::RMObject::ObjectType::OBSTACLE || object->GetType() == roadmanager::RMObject::ObjectType::RAILING ||
             object->GetType() == roadmanager::RMObject::ObjectType::PATCH || object->GetType() == roadmanager::RMObject::ObjectType::TRAFFICISLAND ||
             object->GetType() == roadmanager::RMObject::ObjectType::CROSSWALK ||
             object->GetType() == roadmanager::RMObject::ObjectType::STREETLAMP || object->GetType() == roadmanager::RMObject::ObjectType::GANTRY ||
             object->GetType() == roadmanager::RMObject::ObjectType::SOUNDBARRIER || object->GetType() == roadmanager::RMObject::ObjectType::WIND ||
             object->GetType() == roadmanager::RMObject::ObjectType::ROADMARK)
    {
        obj_osi_internal.sobj->mutable_classification()->set_type(
            osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_OTHER);
    }
    else
    {
        obj_osi_internal.sobj->mutable_classification()->set_type(
            osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_UNKNOWN);
        LOG("OSIReporter::UpdateOSIStationaryObjectODR -> Unsupported stationary object category");
    }

    // Set OSI Stationary Object Position
    obj_osi_internal.sobj->mutable_base()->mutable_position()->set_x(object->GetX());
    obj_osi_internal.sobj->mutable_base()->mutable_position()->set_y(object->GetY());
    obj_osi_internal.sobj->mutable_base()->mutable_position()->set_z(object->GetZ() + object->GetZOffset());

    if (object->GetNumberOfOutlines() > 0)
    {
        for (size_t k = 0; k < static_cast<unsigned int>(object->GetNumberOfOutlines()); k++)
        {
            roadmanager::Outline *outline = object->GetOutline(static_cast<int>(k));
            if (outline)
            {
                double height = 0;
                for (size_t l = 0; l < outline->corner_.size(); l++)
                {
                    double x, y, z;
                    outline->corner_[l]->GetPosLocal(x, y, z);
                    // printf("outline corner %d, %d: %.2f %.2f\n", (int)k, (int)l, x, y);
                    osi3::Vector2d *vec = obj_osi_internal.sobj->mutable_base()->add_base_polygon();
                    vec->set_x(x);
                    vec->set_y(y);
                    height += outline->corner_[l]->GetHeight() / static_cast<double>(outline->corner_.size());
                }
                obj_osi_internal.sobj->mutable_base()->mutable_dimension()->set_height(height);
            }
        }
    }
    else
    {
        // Set OSI Stationary Object Boundingbox
        obj_osi_internal.sobj->mutable_base()->mutable_dimension()->set_height(object->GetHeight());
        obj_osi_internal.sobj->mutable_base()->mutable_dimension()->set_width(object->GetWidth());
        obj_osi_internal.sobj->mutable_base()->mutable_dimension()->set_length(object->GetLength());
        // only bounding box

        // Set OSI Stationary Object Orientation
        obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_roll(GetAngleInIntervalMinusPIPlusPI(object->GetRoll()));
        obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_pitch(GetAngleInIntervalMinusPIPlusPI(object->GetPitch()));
        obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_yaw(GetAngleInIntervalMinusPIPlusPI(object->GetH() + object->GetHOffset()));
    }

    return 0;
}

int OSIReporter::UpdateOSIStationaryObject(ObjectState *objectState)
{
    // Create OSI Stationary Object
    obj_osi_internal.sobj = obj_osi_internal.gt->add_stationary_object();

    // Set OSI Stationary Object Mutable ID
    int sobj_size = obj_osi_internal.gt->mutable_stationary_object()->size();
    obj_osi_internal.sobj->mutable_id()->set_value(static_cast<unsigned int>(sobj_size - 1));

    // Set OSI Stationary Object Type and Classification
    if (objectState->state_.info.obj_type == static_cast<int>(Object::Type::MISC_OBJECT))
    {
        if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::NONE))
        {
            obj_osi_internal.sobj->mutable_classification()->set_type(
                osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_UNKNOWN);
        }
        else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::POLE))
        {
            obj_osi_internal.sobj->mutable_classification()->set_type(
                osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_POLE);
        }
        else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::TREE))
        {
            obj_osi_internal.sobj->mutable_classification()->set_type(
                osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_TREE);
        }
        else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::VEGETATION))
        {
            obj_osi_internal.sobj->mutable_classification()->set_type(
                osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_VEGETATION);
        }
        else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::BARRIER))
        {
            obj_osi_internal.sobj->mutable_classification()->set_type(
                osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BARRIER);
        }
        else if (objectState->state_.info.obj_category == static_cast<int>(MiscObject::Category::BUILDING))
        {
            obj_osi_internal.sobj->mutable_classification()->set_type(
                osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BUILDING);
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
            obj_osi_internal.sobj->mutable_classification()->set_type(
                osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_OTHER);
        }
        else
        {
            obj_osi_internal.sobj->mutable_classification()->set_type(
                osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_UNKNOWN);
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
    obj_osi_internal.sobj->mutable_base()->mutable_position()->set_x(
        objectState->state_.pos.GetX() + static_cast<double>(objectState->state_.info.boundingbox.center_.x_) * cos(objectState->state_.pos.GetH()));
    obj_osi_internal.sobj->mutable_base()->mutable_position()->set_y(
        objectState->state_.pos.GetY() + static_cast<double>(objectState->state_.info.boundingbox.center_.x_) * sin(objectState->state_.pos.GetH()));
    obj_osi_internal.sobj->mutable_base()->mutable_position()->set_z(objectState->state_.pos.GetZ());

    // Set OSI Stationary Object Orientation
    obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_roll(GetAngleInIntervalMinusPIPlusPI(objectState->state_.pos.GetR()));
    obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_pitch(GetAngleInIntervalMinusPIPlusPI(objectState->state_.pos.GetP()));
    obj_osi_internal.sobj->mutable_base()->mutable_orientation()->set_yaw(GetAngleInIntervalMinusPIPlusPI(objectState->state_.pos.GetH()));

    return 0;
}

int OSIReporter::UpdateOSIMovingObject(ObjectState *objectState)
{
    // Create OSI Moving object
    obj_osi_internal.mobj = obj_osi_internal.gt->add_moving_object();

    // Set OSI Moving Object Mutable ID
    obj_osi_internal.mobj->mutable_id()->set_value(static_cast<unsigned int>(objectState->state_.info.id));

    // Set OSI Moving Object Type and Classification
    if (objectState->state_.info.obj_type == static_cast<int>(Object::Type::VEHICLE))
    {
        obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_VEHICLE);

        if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::CAR))
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
        else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::TRAILER))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_TRAILER);
        }
        else if (objectState->state_.info.obj_category == static_cast<int>(Vehicle::Category::VAN))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_DELIVERY_VAN);
        }
        else
        {
            LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object vehicle category: %d (%s). Set to UNKNOWN.",
                objectState->state_.info.obj_category,
                Vehicle::Category2String(objectState->state_.info.obj_category).c_str());
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_UNKNOWN);
        }

#ifdef _OSI_VERSION_3_3_1
        LOG_ONCE("using OSI 3.3.1, skipping vehicle role attribute");
#else
        if (objectState->state_.info.obj_role == Vehicle::Role::AMBULANCE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_AMBULANCE);
        }
        else if (objectState->state_.info.obj_role == Vehicle::Role::CIVIL)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_CIVIL);
        }
        else if (objectState->state_.info.obj_role == Vehicle::Role::FIRE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_FIRE);
        }
        else if (objectState->state_.info.obj_role == Vehicle::Role::MILITARY)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_MILITARY);
        }
        else if (objectState->state_.info.obj_role == Vehicle::Role::POLICE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_POLICE);
        }
        else if (objectState->state_.info.obj_role == Vehicle::Role::PUBLIC_TRANSPORT)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_PUBLIC_TRANSPORT);
        }
        else if (objectState->state_.info.obj_role == Vehicle::Role::ROAD_ASSISTANCE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_ROAD_ASSISTANCE);
        }
        else if (objectState->state_.info.obj_role == Vehicle::Role::NONE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_UNKNOWN);
        }
        else
        {
            LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object vehicle role: %d (%s). Set classification UNKNOWN.",
                objectState->state_.info.obj_role,
                Vehicle::Role2String(objectState->state_.info.obj_role).c_str());
            obj_osi_internal.mobj->mutable_vehicle_classification()->set_role(osi3::MovingObject_VehicleClassification::ROLE_UNKNOWN);
        }

        // Update states for all lights
        if (objectState->state_.info.light_state[Object::VehicleLightType::BRAKE_LIGHTS].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_brake_light_state(
                osi3::MovingObject_VehicleClassification_LightState::BRAKE_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::BRAKE_LIGHTS].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::BRAKE_LIGHTS].mode == Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_brake_light_state(
                osi3::MovingObject_VehicleClassification_LightState::BRAKE_LIGHT_STATE_NORMAL);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::BRAKE_LIGHTS].mode == Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_brake_light_state(
                osi3::MovingObject_VehicleClassification_LightState::BRAKE_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::LOW_BEAM].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_head_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::LOW_BEAM].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::LOW_BEAM].mode == Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_head_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::LOW_BEAM].mode == Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_head_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::HIGH_BEAM].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_high_beam(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::HIGH_BEAM].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::HIGH_BEAM].mode == Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_high_beam(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::HIGH_BEAM].mode == Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_high_beam(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::DAY_TIME_RUNNING_LIGHTS].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_head_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::DAY_TIME_RUNNING_LIGHTS].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::DAY_TIME_RUNNING_LIGHTS].mode == Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_head_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::DAY_TIME_RUNNING_LIGHTS].mode ==
                 Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_head_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_front_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_rear_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS].mode == Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_front_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_rear_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS].mode == Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_front_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_rear_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS_FRONT].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_front_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS_FRONT].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS_FRONT].mode == Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_front_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS_FRONT].mode == Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_front_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS_REAR].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_rear_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS_REAR].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS_REAR].mode == Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_rear_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::FOG_LIGHTS_REAR].mode == Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_rear_fog_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::REVERSING_LIGHTS].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_reversing_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::REVERSING_LIGHTS].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::REVERSING_LIGHTS].mode == Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_reversing_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::REVERSING_LIGHTS].mode == Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_reversing_light(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::LICENSE_PLATER_ILLUMINATION].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_license_plate_illumination_rear(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::LICENSE_PLATER_ILLUMINATION].mode == Object::VehicleLightMode::ON ||
                 objectState->state_.info.light_state[Object::VehicleLightType::LICENSE_PLATER_ILLUMINATION].mode ==
                     Object::VehicleLightMode::FLASHING)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_license_plate_illumination_rear(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::LICENSE_PLATER_ILLUMINATION].mode ==
                 Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_license_plate_illumination_rear(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if (objectState->state_.info.light_state[Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS].mode == Object::VehicleLightMode::OFF)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_emergency_vehicle_illumination(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_OFF);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS].mode == Object::VehicleLightMode::ON)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_emergency_vehicle_illumination(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_ON);
        }
        else if ((objectState->state_.info.light_state[Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS].mode ==
                  Object::VehicleLightMode::FLASHING) &&
                 (objectState->state_.info.light_state[Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS].colorName ==
                  Object::VehicleLightColor::BLUE))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_emergency_vehicle_illumination(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_FLASHING_BLUE);
        }
        else if ((objectState->state_.info.light_state[Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS].mode ==
                  Object::VehicleLightMode::FLASHING) &&
                 (objectState->state_.info.light_state[Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS].colorName ==
                  Object::VehicleLightColor::ORANGE))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_emergency_vehicle_illumination(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_FLASHING_AMBER);
        }
        else if (objectState->state_.info.light_state[Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS].mode ==
                 Object::VehicleLightMode::UNKNOWN_MODE)
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_emergency_vehicle_illumination(
                osi3::MovingObject_VehicleClassification_LightState::GENERIC_LIGHT_STATE_UNKNOWN);
        }

        if ((objectState->state_.info.light_state[Object::VehicleLightType::WARNING_LIGHTS].mode == Object::VehicleLightMode::OFF) ||
            (objectState->state_.info.light_state[Object::VehicleLightType::INDICATOR_LEFT].mode == Object::VehicleLightMode::OFF) ||
            (objectState->state_.info.light_state[Object::VehicleLightType::INDICATOR_RIGHT].mode == Object::VehicleLightMode::OFF))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_indicator_state(
                osi3::MovingObject_VehicleClassification_LightState::INDICATOR_STATE_OFF);
        }
        else if ((objectState->state_.info.light_state[Object::VehicleLightType::INDICATOR_RIGHT].mode == Object::VehicleLightMode::ON) ||
                 (objectState->state_.info.light_state[Object::VehicleLightType::INDICATOR_RIGHT].mode == Object::VehicleLightMode::FLASHING))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_indicator_state(
                osi3::MovingObject_VehicleClassification_LightState::INDICATOR_STATE_RIGHT);
        }
        else if ((objectState->state_.info.light_state[Object::VehicleLightType::INDICATOR_LEFT].mode == Object::VehicleLightMode::ON) ||
                 (objectState->state_.info.light_state[Object::VehicleLightType::INDICATOR_LEFT].mode == Object::VehicleLightMode::FLASHING))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_indicator_state(
                osi3::MovingObject_VehicleClassification_LightState::INDICATOR_STATE_LEFT);
        }

        else if ((objectState->state_.info.light_state[Object::VehicleLightType::WARNING_LIGHTS].mode == Object::VehicleLightMode::ON) ||
                 (objectState->state_.info.light_state[Object::VehicleLightType::WARNING_LIGHTS].mode == Object::VehicleLightMode::FLASHING))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_indicator_state(
                osi3::MovingObject_VehicleClassification_LightState::INDICATOR_STATE_WARNING);
        }
        else if ((objectState->state_.info.light_state[Object::VehicleLightType::WARNING_LIGHTS].mode == Object::VehicleLightMode::UNKNOWN_MODE) &&
                 (objectState->state_.info.light_state[Object::VehicleLightType::INDICATOR_LEFT].mode == Object::VehicleLightMode::UNKNOWN_MODE) &&
                 (objectState->state_.info.light_state[Object::VehicleLightType::INDICATOR_RIGHT].mode == Object::VehicleLightMode::UNKNOWN_MODE))
        {
            obj_osi_internal.mobj->mutable_vehicle_classification()->mutable_light_state()->set_indicator_state(
                osi3::MovingObject_VehicleClassification_LightState::INDICATOR_STATE_UNKNOWN);
        }

#endif
    }
    else if (objectState->state_.info.obj_type == static_cast<int>(Object::Type::PEDESTRIAN))
    {
        if (objectState->state_.info.obj_category == static_cast<int>(Pedestrian::Category::PEDESTRIAN))
        {
            obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_PEDESTRIAN);
        }
        else if (objectState->state_.info.obj_category == static_cast<int>(Pedestrian::Category::ANIMAL))
        {
            obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_ANIMAL);
        }
        else if (objectState->state_.info.obj_category == static_cast<int>(Pedestrian::Category::WHEELCHAIR))
        {
            obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_OTHER);
        }
        else
        {
            LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object pedestrian category: %d (%s). Set type UNKNOWN.",
                objectState->state_.info.obj_category,
                Pedestrian::Category2String(objectState->state_.info.obj_category).c_str());
            obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_UNKNOWN);
        }
    }
    else
    {
        LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object type: %d (%s). Set UNKNOWN.",
            objectState->state_.info.obj_type,
            Object::Type2String(objectState->state_.info.obj_type).c_str());
        obj_osi_internal.mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_UNKNOWN);
    }

    // Set OSI Moving Object Control Type
    obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_driver_id()->set_value(static_cast<uint64_t>(objectState->state_.info.ctrl_type));

    // Set OSI Moving Object Boundingbox
    obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_x(
        static_cast<double>(-objectState->state_.info.boundingbox.center_.x_));
    obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_y(
        static_cast<double>(-objectState->state_.info.boundingbox.center_.y_));
    obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_z(
        objectState->state_.info.rear_axle_z_pos - static_cast<double>(objectState->state_.info.boundingbox.center_.z_));
    obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_front()->set_x(
        objectState->state_.info.front_axle_x_pos - static_cast<double>(objectState->state_.info.boundingbox.center_.x_));
    obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_front()->set_y(
        static_cast<double>(-objectState->state_.info.boundingbox.center_.y_));
    obj_osi_internal.mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_front()->set_z(
        objectState->state_.info.front_axle_z_pos - static_cast<double>(objectState->state_.info.boundingbox.center_.z_));
    obj_osi_internal.mobj->mutable_base()->mutable_dimension()->set_height(objectState->state_.info.boundingbox.dimensions_.height_);
    obj_osi_internal.mobj->mutable_base()->mutable_dimension()->set_width(objectState->state_.info.boundingbox.dimensions_.width_);
    obj_osi_internal.mobj->mutable_base()->mutable_dimension()->set_length(objectState->state_.info.boundingbox.dimensions_.length_);

    // Set OSI Moving Object Position
    obj_osi_internal.mobj->mutable_base()->mutable_position()->set_x(
        objectState->state_.pos.GetX() + static_cast<double>(objectState->state_.info.boundingbox.center_.x_) * cos(objectState->state_.pos.GetH()));
    obj_osi_internal.mobj->mutable_base()->mutable_position()->set_y(
        objectState->state_.pos.GetY() + static_cast<double>(objectState->state_.info.boundingbox.center_.x_) * sin(objectState->state_.pos.GetH()));
    obj_osi_internal.mobj->mutable_base()->mutable_position()->set_z(
        objectState->state_.pos.GetZ() + static_cast<double>(objectState->state_.info.boundingbox.dimensions_.height_) / 2.0);

    // Set OSI Moving Object Orientation
    obj_osi_internal.mobj->mutable_base()->mutable_orientation()->set_roll(GetAngleInIntervalMinusPIPlusPI(objectState->state_.pos.GetR()));
    obj_osi_internal.mobj->mutable_base()->mutable_orientation()->set_pitch(GetAngleInIntervalMinusPIPlusPI(objectState->state_.pos.GetP()));
    obj_osi_internal.mobj->mutable_base()->mutable_orientation()->set_yaw(GetAngleInIntervalMinusPIPlusPI(objectState->state_.pos.GetH()));
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

    // Set ego lane
    obj_osi_internal.mobj->add_assigned_lane_id()->set_value(static_cast<unsigned int>(objectState->state_.pos.GetLaneGlobalId()));

    // Set OSI Wheel Angle Data (Yaw)
    obj_osi_internal.mobj->mutable_vehicle_attributes()->add_wheel_data()->mutable_orientation()->set_yaw(objectState->state_.info.wheel_angle);

    // Set 3D model file as OSI model reference
    obj_osi_internal.mobj->set_model_reference(objectState->state_.info.model3d);

    return 0;
}

int OSIReporter::UpdateOSIIntersection()
{
    // NOTE: for free_lane_boundary this algoritm will only work for open drive solid roadmarks in the junction (or atleast the outest driving lane's
    // roadmark)

    // tolerance to check if points are close or not
    double tolerance = 0.01;

    typedef struct
    {
        int                     id;
        double                  length;
        int                     global_id;
        roadmanager::OSIPoints *osipoints;
    } LaneLengthStruct;

    roadmanager::Junction         *junction;
    roadmanager::Connection       *connection;
    roadmanager::JunctionLaneLink *junctionlanelink;
    roadmanager::Road             *incomming_road;
    roadmanager::Road             *outgoing_road;
    roadmanager::Road             *connecting_road;
    roadmanager::ContactPointType  contactpoint;
    roadmanager::RoadLink         *roadlink          = 0;
    LaneLengthStruct               left_lane_struct  = {0, 0.0, 0, nullptr};
    LaneLengthStruct               right_lane_struct = {0, 0.0, 0, nullptr};
    // s values to know where on the road to check for the lanes
    double incomming_s_value;
    double outgoing_s_value;
    double connecting_outgoing_s_value;

    // value for the linktype for the connecting road and the outgoing road
    roadmanager::LinkType connecting_road_link_type = roadmanager::LinkType::NONE;
    // value for the linktype for the incomming road and the connecting road
    roadmanager::LinkType incomming_road_link_type = roadmanager::LinkType::NONE;
    // some values used for fixing free lane boundary
    double                  length;
    bool                    new_connecting_road;
    int                     g_id;
    roadmanager::OSIPoints *osipoints;

    static roadmanager::OpenDrive *opendrive = roadmanager::Position::GetOpenDrive();
    osi3::Lane                    *osi_lane;
    for (int i = 0; i < opendrive->GetNumOfJunctions(); i++)
    {
        std::vector<LaneLengthStruct> left_lane_lengths;
        std::vector<LaneLengthStruct> right_lane_lengths;
        std::vector<LaneLengthStruct> lane_lengths;
        std::vector<LaneLengthStruct> tmp_lane_lengths;
        std::set<int>                 connected_roads;
        // //add check if it is an intersection or an highway exit/entry
        junction = opendrive->GetJunctionByIdx(i);

        // check if the first road is of type highway, then assumes it is not a intersection
        if (junction->IsOsiIntersection())
        {
            // genereric data for the junction
            osi_lane = obj_osi_internal.gt->add_lane();
            osi_lane->mutable_id()->set_value(static_cast<unsigned int>(junction->GetGlobalId()));
            osi_lane->mutable_classification()->set_type(osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION);

            // check all connections in the junction
            for (int j = 0; j < junction->GetNumberOfConnections(); j++)
            {
                connection          = junction->GetConnectionByIdx(j);
                incomming_road      = connection->GetIncomingRoad();
                connecting_road     = connection->GetConnectingRoad();
                new_connecting_road = true;

                // check if the connecting road has been used before
                for (unsigned int l = 0; l < lane_lengths.size(); l++)
                {
                    if (lane_lengths[l].id == connecting_road->GetId())
                    {
                        new_connecting_road = false;
                    }
                }

                for (unsigned int l = 0; l < left_lane_lengths.size(); l++)
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
                        incomming_s_value        = incomming_road->GetLength();
                        incomming_road_link_type = roadmanager::LinkType::SUCCESSOR;
                    }
                    else
                    {
                        incomming_s_value        = 0;
                        incomming_road_link_type = roadmanager::LinkType::PREDECESSOR;
                    }
                }
                else
                {
                    incomming_s_value        = 0;
                    incomming_road_link_type = roadmanager::LinkType::PREDECESSOR;
                }

                // Get info about the connecting road, and to get the correct outgoing road
                contactpoint = connection->GetContactPoint();
                if (contactpoint == roadmanager::ContactPointType::CONTACT_POINT_START)
                {
                    connecting_road_link_type   = roadmanager::LinkType::SUCCESSOR;
                    roadlink                    = connecting_road->GetLink(connecting_road_link_type);
                    connecting_outgoing_s_value = connecting_road->GetLength();
                }
                else if (contactpoint == roadmanager::ContactPointType::CONTACT_POINT_END)
                {
                    connecting_road_link_type   = roadmanager::LinkType::PREDECESSOR;
                    roadlink                    = connecting_road->GetLink(connecting_road_link_type);
                    connecting_outgoing_s_value = 0;
                }
                else
                {
                    LOG("WARNING: Unknow connection detected, can't establish outgoing connection in OSI junction");
                    return -1;
                }
                if (roadlink == nullptr)
                {
                    LOG("Failed to resolve %s link of incoming road id %d",
                        roadmanager::OpenDrive::LinkType2Str(connecting_road_link_type).c_str(),
                        incomming_road->GetId());
                    return -1;
                }
                outgoing_road = opendrive->GetRoadById(roadlink->GetElementId());
                connected_roads.insert(incomming_road->GetId());
                connected_roads.insert(outgoing_road->GetId());
                // Get neccesary info about the outgoing road
                if (outgoing_road->GetLink(roadmanager::LinkType::SUCCESSOR) != 0)
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
                    left_lane_struct.id      = connecting_road->GetId();
                    left_lane_struct.length  = LARGE_NUMBER;
                    right_lane_struct.id     = connecting_road->GetId();
                    right_lane_struct.length = LARGE_NUMBER;

                    for (int l_id = 1; l_id <= connecting_road->GetLaneSectionByS(0, 0)->GetNUmberOfLanesRight(); l_id++)
                    {
                        if (connecting_road->GetLaneSectionByS(0, 0)->GetLaneById(-l_id)->IsDriving())
                        {
                            // check if an roadmark exist or use a laneboundary
                            // NOTE: assumes only simple lines in an intersection
                            if (connecting_road->GetLaneSectionByS(0, 0)->GetLaneById(-l_id)->GetLaneBoundaryGlobalId() != -1)
                            {
                                osipoints = connecting_road->GetLaneSectionByS(0, 0)->GetLaneById(-l_id)->GetLaneBoundary()->GetOSIPoints();
                                length    = osipoints->GetLength();
                                g_id      = connecting_road->GetLaneSectionByS(0, 0)->GetLaneById(-l_id)->GetLaneBoundary()->GetGlobalId();
                            }
                            else
                            {
                                osipoints = connecting_road->GetLaneSectionByS(0, 0)
                                                ->GetLaneById(-l_id)
                                                ->GetLaneRoadMarkByIdx(0)
                                                ->GetLaneRoadMarkTypeByIdx(0)
                                                ->GetLaneRoadMarkTypeLineByIdx(0)
                                                ->GetOSIPoints();
                                length = connecting_road->GetLaneSectionByS(0, 0)
                                             ->GetLaneById(-l_id)
                                             ->GetLaneRoadMarkByIdx(0)
                                             ->GetLaneRoadMarkTypeByIdx(0)
                                             ->GetLaneRoadMarkTypeLineByIdx(0)
                                             ->GetOSIPoints()
                                             ->GetLength();
                                g_id = connecting_road->GetLaneSectionByS(0, 0)
                                           ->GetLaneById(-l_id)
                                           ->GetLaneRoadMarkByIdx(0)
                                           ->GetLaneRoadMarkTypeByIdx(0)
                                           ->GetLaneRoadMarkTypeLineByIdx(0)
                                           ->GetGlobalId();
                            }
                            if ((right_lane_struct.length > length) || (fabs(right_lane_struct.length - length) < tolerance))
                            {
                                right_lane_struct.length    = length;
                                right_lane_struct.global_id = g_id;
                                right_lane_struct.osipoints = osipoints;
                            }
                        }
                    }
                    for (int l_id = 1; l_id <= connecting_road->GetLaneSectionByS(0, 0)->GetNUmberOfLanesLeft(); l_id++)
                    {
                        if (connecting_road->GetLaneSectionByS(0)->GetLaneById(l_id)->IsDriving())
                        {
                            if (connecting_road->GetLaneSectionByS(0, 0)->GetLaneById(l_id)->GetLaneBoundaryGlobalId() != -1)
                            {
                                osipoints = connecting_road->GetLaneSectionByS(0, 0)->GetLaneById(l_id)->GetLaneBoundary()->GetOSIPoints();
                                length = connecting_road->GetLaneSectionByS(0, 0)->GetLaneById(l_id)->GetLaneBoundary()->GetOSIPoints()->GetLength();
                                g_id   = connecting_road->GetLaneSectionByS(0, 0)->GetLaneById(l_id)->GetLaneBoundary()->GetGlobalId();
                            }
                            else
                            {
                                osipoints = connecting_road->GetLaneSectionByS(0, 0)
                                                ->GetLaneById(l_id)
                                                ->GetLaneRoadMarkByIdx(0)
                                                ->GetLaneRoadMarkTypeByIdx(0)
                                                ->GetLaneRoadMarkTypeLineByIdx(0)
                                                ->GetOSIPoints();
                                length = connecting_road->GetLaneSectionByS(0, 0)
                                             ->GetLaneById(l_id)
                                             ->GetLaneRoadMarkByIdx(0)
                                             ->GetLaneRoadMarkTypeByIdx(0)
                                             ->GetLaneRoadMarkTypeLineByIdx(0)
                                             ->GetOSIPoints()
                                             ->GetLength();
                                g_id = connecting_road->GetLaneSectionByS(0, 0)
                                           ->GetLaneById(l_id)
                                           ->GetLaneRoadMarkByIdx(0)
                                           ->GetLaneRoadMarkTypeByIdx(0)
                                           ->GetLaneRoadMarkTypeLineByIdx(0)
                                           ->GetGlobalId();
                            }
                            if ((left_lane_struct.length > length) || (fabs(right_lane_struct.length - length) < tolerance))
                            {
                                left_lane_struct.length    = length;
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
                bool right_hand_traffic = (incomming_road->GetRule() == roadmanager::Road::RoadRule::RIGHT_HAND_TRAFFIC ||
                                           connecting_road->GetRule() == roadmanager::Road::RoadRule::RIGHT_HAND_TRAFFIC);
                // create all lane parings for the junction
                for (int l = 0; l < connection->GetNumberOfLaneLinks(); l++)
                {
                    junctionlanelink = connection->GetLaneLink(l);
                    // check if the connecting road has been checked before, otherwise get the shortest laneboundary

                    // TODO: will only work for right hand traffic right now
                    if ((((incomming_road_link_type == roadmanager::LinkType::SUCCESSOR && junctionlanelink->from_ < 0) ||
                          (incomming_road_link_type == roadmanager::LinkType::PREDECESSOR && junctionlanelink->from_ > 0)) &&
                         incomming_road->GetDrivingLaneById(incomming_s_value, junctionlanelink->from_) != 0 && right_hand_traffic) ||
                        (((incomming_road_link_type == roadmanager::LinkType::SUCCESSOR && junctionlanelink->from_ > 0) ||
                          (incomming_road_link_type == roadmanager::LinkType::PREDECESSOR && junctionlanelink->from_ < 0)) &&
                         incomming_road->GetDrivingLaneById(incomming_s_value, junctionlanelink->to_) != 0 && !right_hand_traffic))
                    {
                        osi3::Lane_Classification_LanePairing *laneparing = osi_lane->mutable_classification()->add_lane_pairing();
                        laneparing->mutable_antecessor_lane_id()->set_value(
                            static_cast<unsigned int>(incomming_road->GetDrivingLaneById(incomming_s_value, junctionlanelink->from_)->GetGlobalId()));

                        roadmanager::Lane *lane = connecting_road->GetDrivingLaneById(connecting_outgoing_s_value, junctionlanelink->to_);
                        roadmanager::Lane *successor_lane =
                            lane != nullptr ? outgoing_road->GetDrivingLaneById(outgoing_s_value, lane->GetLink(connecting_road_link_type)->GetId())
                                            : nullptr;
                        if (lane != nullptr && successor_lane != nullptr)
                        {
                            laneparing->mutable_successor_lane_id()->set_value(static_cast<unsigned int>(successor_lane->GetGlobalId()));
                        }
                        else
                        {
                            LOG("Connecting road %d incoming road %d failed get lane by id %d",
                                connecting_road->GetId(),
                                connection->GetIncomingRoad()->GetId(),
                                junctionlanelink->to_);
                        }
                    }
                }
            }
            // sort the correct free-boundaries
            for (unsigned int j = 0; j < left_lane_lengths.size(); j++)
            {
                // Tolerance for checking if

                bool keep_right = true;
                bool keep_left  = true;
                for (unsigned int k = 0; k < lane_lengths.size(); k++)
                {
                    int same_left = roadmanager::CheckOverlapingOSIPoints(left_lane_lengths[j].osipoints, lane_lengths[k].osipoints, tolerance);
                    if (same_left > 0)
                    {
                        keep_left = false;
                    }
                    int same_right = roadmanager::CheckOverlapingOSIPoints(right_lane_lengths[j].osipoints, lane_lengths[k].osipoints, tolerance);
                    if (same_right > 0)
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
            for (unsigned int j = 0; j < tmp_lane_lengths.size(); j++)
            {
                lane_lengths.push_back(tmp_lane_lengths[j]);
            }

            if (lane_lengths.size() == connected_roads.size())
            {
                for (unsigned int j = 0; j < lane_lengths.size(); j++)
                {
                    osi3::Identifier *free_lane_id = osi_lane->mutable_classification()->add_free_lane_boundary_id();
                    free_lane_id->set_value(static_cast<unsigned int>(lane_lengths[j].global_id));
                }
            }
            else
            {
                std::vector<int> ids_to_remove;
                for (unsigned int j = 0; j < lane_lengths.size(); j++)
                {
                    if (!(std::find(ids_to_remove.begin(), ids_to_remove.end(), static_cast<int>(j)) != ids_to_remove.end()))
                    {
                        for (unsigned int k = 0; k < lane_lengths.size(); k++)
                        {
                            if (k != j)
                            {
                                int same_points =
                                    roadmanager::CheckOverlapingOSIPoints(lane_lengths[k].osipoints, lane_lengths[j].osipoints, tolerance);
                                if (same_points > 0)
                                {
                                    if (lane_lengths[k].length < lane_lengths[j].length)
                                    {
                                        ids_to_remove.push_back(static_cast<int>(j));
                                    }
                                    else
                                    {
                                        ids_to_remove.push_back(static_cast<int>(k));
                                        continue;
                                    }
                                }
                            }
                        }
                    }
                }

                for (unsigned int j = 0; j < lane_lengths.size(); j++)
                {
                    if (!(std::find(ids_to_remove.begin(), ids_to_remove.end(), static_cast<int>(j)) != ids_to_remove.end()))
                    {
                        osi3::Identifier *free_lane_id = osi_lane->mutable_classification()->add_free_lane_boundary_id();
                        free_lane_id->set_value(static_cast<unsigned int>(lane_lengths[j].global_id));
                    }
                }
                LOG("Issues with the Intersection %i for the osi free lane boundary, none will be added.", junction->GetId());
            }
        }
    }

    // Lets Update the antecessor and successor lanes of the lanes that are not intersections
    // Get all the intersection lanes, this lanes have the predecessor and successor lanes information
    std::vector<osi3::Lane *> IntersectionLanes;
    for (int i = 0; i < obj_osi_internal.gt->lane_size(); ++i)
    {
        if (obj_osi_internal.gt->lane(i).classification().type() == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION)
        {
            IntersectionLanes.push_back(obj_osi_internal.gt->mutable_lane(i));
        }
    }

    // For each lane in OSI groundTruth
    for (int i = 0; i < obj_osi_internal.gt->lane_size(); ++i)
    {
        // Check if the lane is in the intersection
        for (unsigned int j = 0; j < IntersectionLanes.size(); ++j)
        {
            for (int k = 0; k < IntersectionLanes[j]->classification().lane_pairing_size(); ++k)
            {
                // Check predecessors
                if (IntersectionLanes[j]->classification().lane_pairing()[k].has_antecessor_lane_id())
                {
                    // It lane is in predecesor of the intersection
                    if (obj_osi_internal.gt->lane(i).id().value() ==
                        IntersectionLanes[j]->classification().lane_pairing()[k].antecessor_lane_id().value())
                    {
                        // then we add the intersection ID to the successor of the lane
                        if (obj_osi_internal.gt->mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
                        {
                            obj_osi_internal.gt->mutable_lane(i)
                                ->mutable_classification()
                                ->add_lane_pairing()
                                ->mutable_successor_lane_id()
                                ->set_value(IntersectionLanes[j]->id().value());
                        }
                    }
                }

                // Check successors
                if (IntersectionLanes[j]->classification().lane_pairing()[k].has_successor_lane_id())
                {
                    // It lane is in successor of the intersection
                    if (obj_osi_internal.gt->lane(i).id().value() ==
                        IntersectionLanes[j]->classification().lane_pairing()[k].successor_lane_id().value())
                    {
                        // then we add the intersection ID to the predecessor of the lane
                        if (obj_osi_internal.gt->mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
                        {
                            obj_osi_internal.gt->mutable_lane(i)
                                ->mutable_classification()
                                ->add_lane_pairing()
                                ->mutable_antecessor_lane_id()
                                ->set_value(IntersectionLanes[j]->id().value());
                        }
                    }
                }
            }
        }
    }

    return 0;
}

int OSIReporter::UpdateOSILaneBoundary()
{
    // Retrieve opendrive class from RoadManager
    static roadmanager::OpenDrive *opendrive = roadmanager::Position::GetOpenDrive();

    // Loop over all roads
    for (int i = 0; i < opendrive->GetNumOfRoads(); i++)
    {
        roadmanager::Road *road = opendrive->GetRoadByIdx(i);

        // loop over all lane sections
        for (int j = 0; j < road->GetNumberOfLaneSections(); j++)
        {
            roadmanager::LaneSection *lane_section = road->GetLaneSectionByIdx(j);

            // loop over all lanes
            for (int k = 0; k < lane_section->GetNumberOfLanes(); k++)
            {
                roadmanager::Lane *lane = lane_section->GetLaneByIdx(k);

                int n_roadmarks = lane->GetNumberOfRoadMarks();
                if (n_roadmarks != 0)  // if there are road marks
                {
                    // loop over RoadMarks
                    for (int ii = 0; ii < lane->GetNumberOfRoadMarks(); ii++)
                    {
                        roadmanager::LaneRoadMark *laneroadmark = lane->GetLaneRoadMarkByIdx(ii);

                        // loop over road mark types
                        for (int jj = 0; jj < laneroadmark->GetNumberOfRoadMarkTypes(); jj++)
                        {
                            roadmanager::LaneRoadMarkType *laneroadmarktype = laneroadmark->GetLaneRoadMarkTypeByIdx(jj);

                            int inner_index = -1;
                            if (laneroadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN_SOLID ||
                                laneroadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID_BROKEN)
                            {
                                if (laneroadmarktype->GetNumberOfRoadMarkTypeLines() < 2)
                                {
                                    LOG_AND_QUIT("You need to specify at least 2 line for broken solid or solid broken roadmark type");
                                    break;
                                }
                                std::vector<double> sort_solidbroken_brokensolid;
                                for (int q = 0; q < laneroadmarktype->GetNumberOfRoadMarkTypeLines(); q++)
                                {
                                    sort_solidbroken_brokensolid.push_back(laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(q)->GetTOffset());
                                }

                                if (lane->GetId() < 0 || lane->GetId() == 0)
                                {
                                    inner_index =
                                        static_cast<int>((std::max_element(sort_solidbroken_brokensolid.begin(), sort_solidbroken_brokensolid.end()) -
                                                          sort_solidbroken_brokensolid.begin()));
                                }
                                else
                                {
                                    inner_index =
                                        static_cast<int>((std::min_element(sort_solidbroken_brokensolid.begin(), sort_solidbroken_brokensolid.end()) -
                                                          sort_solidbroken_brokensolid.begin()));
                                }
                            }

                            // loop over LaneRoadMarkTypeLine
                            for (int kk = 0; kk < laneroadmarktype->GetNumberOfRoadMarkTypeLines(); kk++)
                            {
                                roadmanager::LaneRoadMarkTypeLine *laneroadmarktypeline = laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(kk);

                                bool broken = false;
                                if (laneroadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN_SOLID)
                                {
                                    if (inner_index == kk)
                                    {
                                        broken = true;
                                    }
                                }

                                if (laneroadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID_BROKEN)
                                {
                                    broken = true;
                                    if (inner_index == kk)
                                    {
                                        broken = false;
                                    }
                                }

                                osi3::LaneBoundary *osi_laneboundary = 0;

                                int line_id = laneroadmarktypeline->GetGlobalId();

                                // Check if this line is already pushed to OSI
                                for (unsigned int h = 0; h < obj_osi_internal.lnb.size(); h++)
                                {
                                    if (obj_osi_internal.lnb[h]->mutable_id()->value() == static_cast<unsigned int>(line_id))
                                    {
                                        osi_laneboundary = obj_osi_internal.lnb[h];
                                    }
                                }
                                if (!osi_laneboundary)
                                {
                                    osi_laneboundary = obj_osi_internal.gt->add_lane_boundary();

                                    // update id
                                    osi_laneboundary->mutable_id()->set_value(static_cast<unsigned int>(line_id));

                                    int n_osi_points = laneroadmarktypeline->GetOSIPoints()->GetNumOfOSIPoints();
                                    for (int h = 0; h < n_osi_points; h++)
                                    {
                                        osi3::LaneBoundary_BoundaryPoint *boundary_point = osi_laneboundary->add_boundary_line();
                                        boundary_point->mutable_position()->set_x(laneroadmarktypeline->GetOSIPoints()->GetXfromIdx(h));
                                        boundary_point->mutable_position()->set_y(laneroadmarktypeline->GetOSIPoints()->GetYfromIdx(h));
                                        boundary_point->mutable_position()->set_z(laneroadmarktypeline->GetOSIPoints()->GetZfromIdx(h));
                                        boundary_point->set_width(laneroadmarktypeline->GetWidth());
                                        boundary_point->set_height(laneroadmark->GetHeight());
                                    }

                                    // update classification type
                                    osi3::LaneBoundary_Classification_Type classific_type;
                                    switch (laneroadmark->GetType())
                                    {
                                        case roadmanager::LaneRoadMark::RoadMarkType::NONE_TYPE:
                                            classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_NO_LINE;
                                            break;
                                        case roadmanager::LaneRoadMark::RoadMarkType::SOLID:
                                            classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
                                            break;
                                        case roadmanager::LaneRoadMark::RoadMarkType::SOLID_SOLID:
                                            classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
                                            break;
                                        case roadmanager::LaneRoadMark::RoadMarkType::BROKEN:
                                            classific_type =
                                                osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE;
                                            break;
                                        case roadmanager::LaneRoadMark::RoadMarkType::BROKEN_BROKEN:
                                            classific_type =
                                                osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE;
                                            break;
                                        case roadmanager::LaneRoadMark::RoadMarkType::SOLID_BROKEN:
                                            if (broken)
                                            {
                                                classific_type =
                                                    osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE;
                                            }
                                            else
                                            {
                                                classific_type =
                                                    osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
                                            }
                                            break;
                                        case roadmanager::LaneRoadMark::RoadMarkType::BROKEN_SOLID:
                                            if (broken)
                                            {
                                                classific_type =
                                                    osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE;
                                            }
                                            else
                                            {
                                                classific_type =
                                                    osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
                                            }
                                            break;
                                        case roadmanager::LaneRoadMark::RoadMarkType::BOTTS_DOTS:
                                            classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_BOTTS_DOTS;
                                            break;
                                        default:
                                            classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
                                    }
                                    osi_laneboundary->mutable_classification()->set_type(classific_type);

                                    // update classification color
                                    osi3::LaneBoundary_Classification_Color classific_col;
                                    switch (laneroadmark->GetColor())
                                    {
                                        case roadmanager::RoadMarkColor::STANDARD_COLOR:
                                            classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
                                            break;
                                        case roadmanager::RoadMarkColor::BLUE:
                                            classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_BLUE;
                                            break;
                                        case roadmanager::RoadMarkColor::GREEN:
                                            classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_GREEN;
                                            break;
                                        case roadmanager::RoadMarkColor::RED:
                                            classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_RED;
                                            break;
                                        case roadmanager::RoadMarkColor::WHITE:
                                            classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
                                            break;
                                        case roadmanager::RoadMarkColor::YELLOW:
                                            classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_YELLOW;
                                            break;
                                        default:
                                            classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
                                    }
                                    osi_laneboundary->mutable_classification()->set_color(classific_col);

                                    // update limiting structure id only if the type of lane boundary is set to TYPE_STRUCTURE - for now it is not
                                    // implemented
                                    // osi_laneboundary->mutable_classification()->mutable_limiting_structure_id(0)->set_value(0);

                                    obj_osi_internal.lnb.push_back(osi_laneboundary);
                                }
                            }
                        }
                    }
                }
                else  // if there are no road marks I take the lane boundary
                {
                    roadmanager::LaneBoundaryOSI *laneboundary = lane->GetLaneBoundary();
                    // Check if this line is already pushed to OSI
                    int                 boundary_id      = laneboundary->GetGlobalId();
                    osi3::LaneBoundary *osi_laneboundary = 0;
                    for (unsigned int h = 0; h < obj_osi_internal.lnb.size(); h++)
                    {
                        if (obj_osi_internal.lnb[h]->mutable_id()->value() == static_cast<unsigned int>(boundary_id))
                        {
                            osi_laneboundary = obj_osi_internal.lnb[h];
                        }
                    }
                    if (!osi_laneboundary)
                    {
                        osi_laneboundary = obj_osi_internal.gt->add_lane_boundary();

                        // update id
                        osi_laneboundary->mutable_id()->set_value(static_cast<unsigned int>(boundary_id));

                        int n_osi_points = laneboundary->GetOSIPoints()->GetNumOfOSIPoints();
                        for (int h = 0; h < n_osi_points; h++)
                        {
                            osi3::LaneBoundary_BoundaryPoint *boundary_point = osi_laneboundary->add_boundary_line();
                            boundary_point->mutable_position()->set_x(laneboundary->GetOSIPoints()->GetXfromIdx(h));
                            boundary_point->mutable_position()->set_y(laneboundary->GetOSIPoints()->GetYfromIdx(h));
                            boundary_point->mutable_position()->set_z(laneboundary->GetOSIPoints()->GetZfromIdx(h));
                            // boundary_point->set_width(laneboundary->GetWidth());
                            // boundary_point->set_height(laneroadmark->GetHeight());
                        }

                        if (lane->IsRoadEdge())
                        {
                            osi_laneboundary->mutable_classification()->set_type(
                                osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_ROAD_EDGE);
                        }
                        else
                        {
                            osi_laneboundary->mutable_classification()->set_type(
                                osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_NO_LINE);
                        }

                        osi3::LaneBoundary_Classification_Color classific_col =
                            osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_UNKNOWN;
                        osi_laneboundary->mutable_classification()->set_color(classific_col);

                        obj_osi_internal.lnb.push_back(osi_laneboundary);
                    }
                }
            }
        }
    }

    return 0;
}

int OSIReporter::UpdateOSIRoadLane()
{
    // Retrieve opendrive class from RoadManager
    static roadmanager::OpenDrive *opendrive = roadmanager::Position::GetOpenDrive();

    // Loop over all roads
    for (int i = 0; i < opendrive->GetNumOfRoads(); i++)
    {
        roadmanager::Road *road = opendrive->GetRoadByIdx(i);

        // Get predecessor and successor roads if exists
        roadmanager::RoadLink *roadLink = nullptr;

        roadmanager::Road *predecessorRoad = nullptr;
        roadmanager::Road *successorRoad   = nullptr;

        roadmanager::Junction *predecessorJunction = nullptr;
        roadmanager::Junction *successorJunction   = nullptr;

        roadLink = road->GetLink(roadmanager::LinkType::PREDECESSOR);
        if (roadLink)
        {
            if (roadLink->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_ROAD)
            {
                predecessorRoad = opendrive->GetRoadById(roadLink->GetElementId());
            }
            if (roadLink->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
            {
                predecessorJunction = opendrive->GetJunctionById(roadLink->GetElementId());
            }
        }

        roadLink = road->GetLink(roadmanager::LinkType::SUCCESSOR);
        if (roadLink)
        {
            if (roadLink->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_ROAD)
            {
                successorRoad = opendrive->GetRoadById(roadLink->GetElementId());
            }
            if (roadLink->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
            {
                successorJunction = opendrive->GetJunctionById(roadLink->GetElementId());
            }
        }

        // loop over all lane sections
        for (int j = 0; j < road->GetNumberOfLaneSections(); j++)
        {
            roadmanager::LaneSection *lane_section                   = road->GetLaneSectionByIdx(j);
            int                       global_predecessor_junction_id = -1;
            int                       global_successor_junction_id   = -1;
            // Get predecessor and successor lane_sections
            roadmanager::LaneSection *predecessor_lane_section = nullptr;
            roadmanager::LaneSection *successor_lane_section   = nullptr;

            // if there are more than 1 section we use the previous lane section in the same road
            if (j > 0)
            {
                predecessor_lane_section = road->GetLaneSectionByIdx(j - 1);
            }
            else
            {
                // Otherwise we use the last lane section of the predecessor road
                if (predecessorRoad)
                {
                    // get first or last lane section depending on road direction
                    if (predecessorRoad->GetLink(roadmanager::LinkType::PREDECESSOR))
                    {
                        if (predecessorRoad->GetLink(roadmanager::LinkType::PREDECESSOR)->GetElementId() == road->GetId())
                        {
                            predecessor_lane_section = predecessorRoad->GetLaneSectionByIdx(0);
                        }
                        else if (predecessorRoad->GetLink(roadmanager::LinkType::PREDECESSOR)->GetElementId() == road->GetJunction())
                        {
                            predecessor_lane_section = predecessorRoad->GetLaneSectionByIdx(0);
                        }
                    }
                    if (predecessorRoad->GetLink(roadmanager::LinkType::SUCCESSOR))
                    {
                        if (predecessorRoad->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == road->GetId())
                        {
                            predecessor_lane_section = predecessorRoad->GetLaneSectionByIdx(predecessorRoad->GetNumberOfLaneSections() - 1);
                        }
                        else if (predecessorRoad->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == road->GetJunction())
                        {
                            predecessor_lane_section = predecessorRoad->GetLaneSectionByIdx(predecessorRoad->GetNumberOfLaneSections() - 1);
                        }
                    }
                }
                else if (predecessorJunction && predecessorJunction->IsOsiIntersection())
                {
                    global_predecessor_junction_id = predecessorJunction->GetGlobalId();
                }
            }

            // If it is the lane section before to the last one we use the last lane section as successor
            if (j < road->GetNumberOfLaneSections() - 1)
            {
                successor_lane_section = road->GetLaneSectionByIdx(j + 1);
            }
            else
            {
                // Otherwise (is the last lane section) we use the first lane section of the successor road if exists
                if (successorRoad)
                {
                    // get first or last lane section depending on road direction
                    if (successorRoad->GetLink(roadmanager::LinkType::PREDECESSOR))
                    {
                        if (successorRoad->GetLink(roadmanager::LinkType::PREDECESSOR)->GetElementId() == road->GetId())
                        {
                            successor_lane_section = successorRoad->GetLaneSectionByIdx(0);
                        }
                        else if (successorRoad->GetLink(roadmanager::LinkType::PREDECESSOR)->GetElementId() == road->GetJunction())
                        {
                            successor_lane_section = successorRoad->GetLaneSectionByIdx(0);
                        }
                    }
                    if (successorRoad->GetLink(roadmanager::LinkType::SUCCESSOR))
                    {
                        if (successorRoad->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == road->GetId())
                        {
                            successor_lane_section = successorRoad->GetLaneSectionByIdx(successorRoad->GetNumberOfLaneSections() - 1);
                        }
                        else if (successorRoad->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == road->GetJunction())
                        {
                            successor_lane_section = successorRoad->GetLaneSectionByIdx(successorRoad->GetNumberOfLaneSections() - 1);
                        }
                    }
                }
                else if (successorJunction && successorJunction->IsOsiIntersection())
                {
                    global_successor_junction_id = successorJunction->GetGlobalId();
                }
            }

            // loop over all lanes
            for (int k = 0; k < lane_section->GetNumberOfLanes(); k++)
            {
                roadmanager::Lane *lane = lane_section->GetLaneByIdx(k);
                if ((!lane->IsCenter() && !lane->IsOSIIntersection()))
                {
                    osi3::Lane *osi_lane       = 0;
                    int         lane_global_id = lane->GetGlobalId();
                    int         lane_id        = lane->GetId();

                    // Check if this lane is already pushed to OSI - if yes just update
                    for (unsigned int jj = 0; jj < obj_osi_internal.ln.size(); jj++)
                    {
                        if (obj_osi_internal.ln[jj]->mutable_id()->value() == static_cast<unsigned int>(lane_global_id))
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
                        osi_lane->mutable_id()->set_value(static_cast<unsigned int>(lane_global_id));

                        // CLASSIFICATION TYPE
                        roadmanager::Lane::LaneType       lanetype   = lane->GetLaneType();
                        osi3::Lane_Classification_Type    class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_UNKNOWN;
                        osi3::Lane_Classification_Subtype subclass_type =
                            osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_UNKNOWN;
                        if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_DRIVING)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_NORMAL;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_PARKING)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_PARKING;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BIDIRECTIONAL)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_NORMAL;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_STOP)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_STOP;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BIKING)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_BIKING;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SIDEWALK)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_SIDEWALK;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BORDER)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_BORDER;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_RESTRICTED)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_RESTRICTED;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ROADMARKS)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_OTHER;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_TRAM)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_OTHER;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_RAIL)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_OTHER;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ENTRY)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_ENTRY;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_EXIT)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_EXIT;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_OFF_RAMP)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_OFFRAMP;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ON_RAMP)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_ONRAMP;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_MEDIAN)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_OTHER;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SHOULDER)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_SHOULDER;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_CURB)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_BORDER;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_CONNECTING_RAMP)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_CONNECTINGRAMP;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL1 ||
                                 lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL2 ||
                                 lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL3)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_OTHER;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_OTHER;
                        }
                        else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_NONE)
                        {
                            class_type    = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_UNKNOWN;
                            subclass_type = osi3::Lane_Classification_Subtype::Lane_Classification_Subtype_SUBTYPE_UNKNOWN;
                        }
                        osi_lane->mutable_classification()->set_type(class_type);
                        osi_lane->mutable_classification()->set_subtype(subclass_type);

                        // CENTERLINE POINTS
                        int n_osi_points = lane->GetOSIPoints()->GetNumOfOSIPoints();
                        for (int jj = 0; jj < n_osi_points; jj++)
                        {
                            osi3::Vector3d *centerLine = osi_lane->mutable_classification()->add_centerline();
                            centerLine->set_x(lane->GetOSIPoints()->GetXfromIdx(jj));
                            centerLine->set_y(lane->GetOSIPoints()->GetYfromIdx(jj));
                            centerLine->set_z(lane->GetOSIPoints()->GetZfromIdx(jj));
                        }

                        // DRIVING DIRECTION
                        bool driving_direction = true;
                        if ((lane_id >= 0 && road->GetRule() == roadmanager::Road::RoadRule::RIGHT_HAND_TRAFFIC) ||
                            (lane_id < 0 && road->GetRule() == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC))
                        {
                            driving_direction = false;
                        }
                        osi_lane->mutable_classification()->set_centerline_is_driving_direction(driving_direction);

                        // Get the predecessor and successor lanes
                        roadmanager::Lane *predecessorLane = nullptr;
                        roadmanager::Lane *successorLane   = nullptr;

                        osi3::Lane_Classification_LanePairing *lane_pairing = nullptr;
                        if (predecessor_lane_section && lane->GetLink(roadmanager::LinkType::PREDECESSOR))
                        {
                            predecessorLane = predecessor_lane_section->GetLaneById(lane->GetLink(roadmanager::LinkType::PREDECESSOR)->GetId());
                            if (predecessorLane)
                            {
                                if (!lane_pairing)
                                {
                                    lane_pairing = osi_lane->mutable_classification()->add_lane_pairing();
                                }
                                lane_pairing->mutable_antecessor_lane_id()->set_value(static_cast<unsigned int>(predecessorLane->GetGlobalId()));
                            }
                        }

                        if (successor_lane_section && lane->GetLink(roadmanager::LinkType::SUCCESSOR))
                        {
                            successorLane = successor_lane_section->GetLaneById(lane->GetLink(roadmanager::LinkType::SUCCESSOR)->GetId());
                            if (successorLane)
                            {
                                if (!lane_pairing)
                                {
                                    lane_pairing = osi_lane->mutable_classification()->add_lane_pairing();
                                }
                                lane_pairing->mutable_successor_lane_id()->set_value(static_cast<unsigned int>(successorLane->GetGlobalId()));
                            }
                        }

                        if (global_predecessor_junction_id != -1)
                        {
                            if (!lane_pairing)
                            {
                                lane_pairing = osi_lane->mutable_classification()->add_lane_pairing();
                            }
                            lane_pairing->mutable_antecessor_lane_id()->set_value(static_cast<unsigned int>(global_predecessor_junction_id));
                        }

                        if (global_successor_junction_id != -1)
                        {
                            if (!lane_pairing)
                            {
                                lane_pairing = osi_lane->mutable_classification()->add_lane_pairing();
                            }
                            lane_pairing->mutable_successor_lane_id()->set_value(static_cast<unsigned int>(global_successor_junction_id));
                        }
                        // Update lanes that connect with junctions that are not intersections
                        if (road->GetNumberOfRoadTypes() > 0 && road->GetRoadType(0)->road_type_ == roadmanager::Road::RoadType::ROADTYPE_MOTORWAY &&
                            road->GetJunction() > 0)
                        {
                            roadmanager::LaneLink *link_predecessor = lane->GetLink(roadmanager::LinkType::PREDECESSOR);
                            roadmanager::LaneLink *link_successor   = lane->GetLink(roadmanager::LinkType::SUCCESSOR);

                            roadmanager::Lane *driving_lane_predecessor = 0;
                            roadmanager::Lane *driving_lane_successor   = 0;

                            if (link_predecessor)
                            {
                                driving_lane_predecessor =
                                    predecessorRoad->GetDrivingLaneById(predecessor_lane_section->GetS(), link_predecessor->GetId());
                                if (driving_lane_predecessor)
                                {
                                    LOG("Lane %d on predecessor road %d s %.2f is not a driving lane",
                                        lane->GetId(),
                                        predecessorRoad->GetId(),
                                        predecessor_lane_section->GetS());
                                }
                            }
                            else
                            {
                                LOG("Failed to resolve Predecessor link of lane %d of road %d", lane->GetId(), road->GetId());
                            }

                            if (link_successor)
                            {
                                driving_lane_successor = successorRoad->GetDrivingLaneById(successor_lane_section->GetS(), link_successor->GetId());
                                if (driving_lane_successor)
                                {
                                    LOG("Lane %d on successor road %d s %.2f is not a driving lane",
                                        lane->GetId(),
                                        successorRoad->GetId(),
                                        successor_lane_section->GetS());
                                }
                            }
                            else
                            {
                                LOG("Failed to resolve Successor link of lane %d of road %d", lane->GetId(), road->GetId());
                            }

                            for (int l = 0; l < obj_osi_internal.gt->lane_size(); ++l)
                            {
                                if (obj_osi_internal.gt->mutable_lane(l)->mutable_classification()->lane_pairing_size() > 0)
                                {
                                    // there should be only one lane_paring, since only intersections have multiple ones
                                    lane_pairing = obj_osi_internal.gt->mutable_lane(l)->mutable_classification()->mutable_lane_pairing(0);
                                }
                                else
                                {
                                    lane_pairing = obj_osi_internal.gt->mutable_lane(l)->mutable_classification()->add_lane_pairing();
                                }
                                if (predecessorRoad && predecessor_lane_section && link_predecessor && driving_lane_predecessor &&
                                    static_cast<unsigned int>(driving_lane_predecessor->GetGlobalId()) == obj_osi_internal.gt->lane(l).id().value())
                                {
                                    if ((road->GetLink(roadmanager::LinkType::PREDECESSOR) != 0))
                                    {
                                        lane_pairing->mutable_successor_lane_id()->set_value(static_cast<unsigned int>(lane_global_id));
                                    }
                                }
                                if (successorRoad && successor_lane_section && link_successor && driving_lane_successor &&
                                    static_cast<unsigned int>(driving_lane_successor->GetGlobalId()) == obj_osi_internal.gt->lane(l).id().value())
                                {
                                    if ((road->GetLink(roadmanager::LinkType::SUCCESSOR) != 0))
                                    {
                                        lane_pairing->mutable_antecessor_lane_id()->set_value(static_cast<unsigned int>(lane_global_id));
                                    }
                                }
                            }
                        }

                        // LEFT AND RIGHT LANE IDS
                        std::vector<std::pair<int, int>> globalid_ids_left;
                        std::vector<std::pair<int, int>> globalid_ids_right;

                        if (lane_section->IsOSILaneById(lane_id + (1)))
                        {
                            globalid_ids_left.push_back(std::make_pair(lane_id - (1), lane_section->GetLaneGlobalIdById(lane_id + (1))));
                        }
                        else if (lane_section->IsOSILaneById(lane_id + (2)))
                        {
                            globalid_ids_left.push_back(std::make_pair(lane_id - (2), lane_section->GetLaneGlobalIdById(lane_id + (2))));
                        }

                        if (lane_section->IsOSILaneById(lane_id - (1)))
                        {
                            globalid_ids_right.push_back(std::make_pair(lane_id - (1), lane_section->GetLaneGlobalIdById(lane_id - (1))));
                        }
                        else if (lane_section->IsOSILaneById(lane_id - (2)))
                        {
                            globalid_ids_right.push_back(std::make_pair(lane_id - (2), lane_section->GetLaneGlobalIdById(lane_id - (2))));
                        }

                        // order global id with local id to maintain geographical order
                        std::sort(globalid_ids_left.begin(), globalid_ids_left.end());
                        std::sort(globalid_ids_right.begin(), globalid_ids_right.end());

                        for (unsigned int jj = 0; jj < globalid_ids_left.size(); jj++)
                        {
                            osi3::Identifier *left_id = osi_lane->mutable_classification()->add_left_adjacent_lane_id();
                            left_id->set_value(static_cast<uint64_t>(globalid_ids_left[jj].second));
                        }
                        for (unsigned int jj = 0; jj < globalid_ids_right.size(); jj++)
                        {
                            osi3::Identifier *right_id = osi_lane->mutable_classification()->add_right_adjacent_lane_id();
                            right_id->set_value(static_cast<uint64_t>(globalid_ids_right[jj].second));
                        }

                        // LANE BOUNDARY IDS
                        if (lane_id == 0)  // for central lane I use the laneboundary osi points as right and left boundary so that it can be used
                                           // from both sides
                        {
                            // check if lane has road mark
                            std::vector<int> line_ids = lane->GetLineGlobalIds();
                            if (!line_ids.empty())  // lane has RoadMarks
                            {
                                for (unsigned int jj = 0; jj < line_ids.size(); jj++)
                                {
                                    osi3::Identifier *left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
                                    left_lane_bound_id->set_value(static_cast<unsigned int>(line_ids[jj]));
                                    osi3::Identifier *right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
                                    right_lane_bound_id->set_value(static_cast<unsigned int>(line_ids[jj]));
                                }
                            }
                            else  // no road marks -> we take lane boundary
                            {
                                int laneboundary_global_id = lane->GetLaneBoundaryGlobalId();
                                if (laneboundary_global_id >= 0)
                                {
                                    osi3::Identifier *left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
                                    left_lane_bound_id->set_value(static_cast<unsigned int>(laneboundary_global_id));
                                    osi3::Identifier *right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
                                    right_lane_bound_id->set_value(static_cast<unsigned int>(laneboundary_global_id));
                                }
                            }
                        }
                        else
                        {
                            // Set left/right laneboundary ID for left/right lanes- we use LaneMarks is they exist, if not we take laneboundary
                            std::vector<int> line_ids = lane->GetLineGlobalIds();
                            if (!line_ids.empty())  // lane has RoadMarks
                            {
                                for (unsigned int jj = 0; jj < line_ids.size(); jj++)
                                {
                                    if (lane_id < 0)
                                    {
                                        osi3::Identifier *left_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
                                        left_lane_bound_id->set_value(static_cast<unsigned int>(line_ids[jj]));
                                    }
                                    else if (lane_id > 0)
                                    {
                                        osi3::Identifier *left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
                                        left_lane_bound_id->set_value(static_cast<unsigned int>(line_ids[jj]));
                                    }
                                }
                            }
                            else
                            {
                                int laneboundary_global_id = lane->GetLaneBoundaryGlobalId();
                                if (lane_id < 0 && laneboundary_global_id >= 0)
                                {
                                    osi3::Identifier *left_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
                                    left_lane_bound_id->set_value(static_cast<unsigned int>(laneboundary_global_id));
                                }
                                else if (lane_id > 0 && laneboundary_global_id >= 0)
                                {
                                    osi3::Identifier *left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
                                    left_lane_bound_id->set_value(static_cast<unsigned int>(laneboundary_global_id));
                                }
                            }

                            // Set right/left laneboundary ID for left/right lanes - we look at neightbour lanes
                            int next_lane_id = 0;
                            if (lane_id < 0)  // if lane is on the right, then it contains its right boundary. So I need to look into its left lane
                                              // for the left boundary
                            {
                                next_lane_id = lane_id + 1;
                            }
                            else if (lane_id > 0)  // if lane is on the left, then it contains its left boundary. So I need to look into its right
                                                   // lane for the right boundary
                            {
                                next_lane_id = lane_id - 1;
                            }
                            // look at right lane and check if it has Lines for RoadMarks
                            roadmanager::Lane *next_lane = lane_section->GetLaneById(next_lane_id);
                            if (next_lane != nullptr)
                            {
                                std::vector<int> nextlane_line_ids = next_lane->GetLineGlobalIds();
                                if (!nextlane_line_ids.empty())
                                {
                                    for (unsigned int jj = 0; jj < nextlane_line_ids.size(); jj++)
                                    {
                                        if (lane_id < 0)
                                        {
                                            osi3::Identifier *right_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
                                            right_lane_bound_id->set_value(static_cast<unsigned int>(nextlane_line_ids[jj]));
                                        }
                                        else if (lane_id > 0)
                                        {
                                            osi3::Identifier *right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
                                            right_lane_bound_id->set_value(static_cast<unsigned int>(nextlane_line_ids[jj]));
                                        }
                                    }
                                }
                                else  // if the neightbour lane does not have Lines for RoadMakrs we take the LaneBoundary
                                {
                                    int next_laneboundary_global_id = next_lane->GetLaneBoundaryGlobalId();
                                    if (lane_id < 0 && next_laneboundary_global_id >= 0)
                                    {
                                        osi3::Identifier *right_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
                                        right_lane_bound_id->set_value(static_cast<unsigned int>(next_laneboundary_global_id));
                                    }
                                    else if (lane_id > 0 && next_laneboundary_global_id >= 0)
                                    {
                                        osi3::Identifier *right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
                                        right_lane_bound_id->set_value(static_cast<unsigned int>(next_laneboundary_global_id));
                                    }
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
                        // obj_osi_external.gt->mutable_lane()->CopyFrom(*obj_osi_internal.gt->mutable_lane());
                    }
                }
            }
        }
    }

    return 0;
}

int OSIReporter::UpdateTrafficSignals()
{
    // Create OSI Stationary Object
    // obj_osi_internal.ts = obj_osi_internal.gt->add_traffic_sign();

    // Retrieve opendrive class from RoadManager
    static roadmanager::OpenDrive *opendrive = roadmanager::Position::GetOpenDrive();

    // Loop over all roads
    for (int i = 0; i < opendrive->GetNumOfRoads(); i++)
    {
        roadmanager::Road *road = opendrive->GetRoadByIdx(i);
        for (int j = 0; j < road->GetNumberOfSignals(); ++j)
        {
            roadmanager::Signal *signal = road->GetSignal(j);

            if (signal)
            {
                // Is Traffic Light
                if (signal->IsDynamic())
                {
                    osi3::TrafficLight *trafficLight = obj_osi_internal.gt->add_traffic_light();
                    trafficLight->mutable_id()->set_value(static_cast<unsigned int>(signal->GetId()));
                    trafficLight->mutable_base()->mutable_orientation()->set_pitch(GetAngleInIntervalMinusPIPlusPI(signal->GetPitch()));
                    trafficLight->mutable_base()->mutable_orientation()->set_roll(GetAngleInIntervalMinusPIPlusPI(signal->GetRoll()));
                    trafficLight->mutable_base()->mutable_orientation()->set_yaw(GetAngleInIntervalMinusPIPlusPI(
                        signal->GetH() + signal->GetHOffset() + M_PI));  // Add pi to have the yaw angle of actual sign face direction (normally
                                                                         // pointing 180 degrees wrt road construction direction)
                    trafficLight->mutable_base()->mutable_dimension()->set_height(signal->GetHeight());
                    trafficLight->mutable_base()->mutable_dimension()->set_width(signal->GetWidth());

                    trafficLight->mutable_base()->mutable_position()->set_x(signal->GetX());
                    trafficLight->mutable_base()->mutable_position()->set_y(signal->GetY());
                    trafficLight->mutable_base()->mutable_position()->set_z(signal->GetZ() + signal->GetZOffset());
                }
                else
                {
                    // Traffic Sign
                    osi3::TrafficSign *trafficSign = obj_osi_internal.gt->add_traffic_sign();
                    // Set ID, Value, Text
                    trafficSign->mutable_id()->set_value(static_cast<unsigned int>(signal->GetId()));
                    trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value(signal->GetValue());
                    trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_text(signal->GetText());
                    trafficSign->mutable_main_sign()->mutable_classification()->set_type(
                        static_cast<osi3::TrafficSign_MainSign_Classification_Type>(signal->GetOSIType()));

                    // Set Unit
                    if (std::strcmp(signal->GetUnit().c_str(), ""))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_NO_UNIT);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "m"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_METER);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "km"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_KILOMETER);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "ft"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_FEET);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "mile"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_MILE);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "m/s"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_OTHER);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "mph"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_MILE_PER_HOUR);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "km/h"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_KILOMETER_PER_HOUR);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "kg"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_UNKNOWN);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "t"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_METRIC_TON);
                    }
                    else if (std::strcmp(signal->GetUnit().c_str(), "%"))
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_PERCENTAGE);
                    }
                    else
                    {
                        trafficSign->mutable_main_sign()->mutable_classification()->mutable_value()->set_value_unit(
                            osi3::TrafficSignValue_Unit::TrafficSignValue_Unit_UNIT_UNKNOWN);
                    }

                    // Set Pithc, Roll, Height, Width
                    trafficSign->mutable_main_sign()->mutable_base()->mutable_orientation()->set_pitch(
                        GetAngleInIntervalMinusPIPlusPI(signal->GetPitch()));
                    trafficSign->mutable_main_sign()->mutable_base()->mutable_orientation()->set_roll(
                        GetAngleInIntervalMinusPIPlusPI(signal->GetRoll()));
                    trafficSign->mutable_main_sign()->mutable_base()->mutable_orientation()->set_yaw(GetAngleInIntervalMinusPIPlusPI(
                        signal->GetH() + signal->GetHOffset() + M_PI));  // Add pi to have the yaw angle of actual sign face direction (normally
                                                                         // pointing 180 degrees wrt road construction direction)
                    trafficSign->mutable_main_sign()->mutable_base()->mutable_dimension()->set_height(signal->GetHeight());
                    trafficSign->mutable_main_sign()->mutable_base()->mutable_dimension()->set_width(signal->GetWidth());

                    // Set X, Y, Z based on s, t, and zOffset
                    trafficSign->mutable_main_sign()->mutable_base()->mutable_position()->set_x(signal->GetX());
                    trafficSign->mutable_main_sign()->mutable_base()->mutable_position()->set_y(signal->GetY());
                    trafficSign->mutable_main_sign()->mutable_base()->mutable_position()->set_z(signal->GetZ() + signal->GetZOffset());
                }
            }
        }
    }
    return 0;
}

int OSIReporter::UpdateOSITrafficCommand()
{
    obj_osi_external.tc->clear_action();

    if (GetUDPClientStatus() == 0 || IsFileOpen())
    {
        obj_osi_external.tc->SerializeToString(&osiTrafficCommand.traffic_command);
        osiTrafficCommand.size = static_cast<unsigned int>(obj_osi_external.tc->ByteSizeLong());
    }

    for (auto state_change : traffic_command_state_changes_)
    {
        if (state_change.transition == StoryBoardElement::Transition::START_TRANSITION)
        {
            ReportTrafficCommand(obj_osi_external.tc, state_change.action, scenario_engine_->getSimulationTime());
        }
    }

    traffic_command_state_changes_.clear();

    return 0;
}

int OSIReporter::CreateSensorViewFromSensorData(const osi3::SensorData &sd)
{
    obj_osi_external.sv->Clear();
    for (int i = 0; i < sd.moving_object_size(); i++)
    {
        CreateMovingObjectFromSensorData(sd, i);
    }

    for (int i = 0; i < sd.lane_boundary_size(); i++)
    {
        CreateLaneBoundaryFromSensordata(sd, i);
    }
    return 0;
}

void OSIReporter::CreateMovingObjectFromSensorData(const osi3::SensorData &sd, int obj_nr)
{
    osi3::DetectedMovingObject object = sd.moving_object(obj_nr);
    double                     x      = object.base().position().x() + sd.mounting_position().position().x();
    double                     y      = object.base().position().y() + sd.mounting_position().position().y();
    double                     z      = object.base().position().z();
    double                     yaw    = object.base().orientation().yaw();

    yaw = sd.mounting_position().orientation().yaw() + yaw;
    yaw = sd.host_vehicle_location().orientation().yaw() + yaw;

    // Local2GlobalCoordinates(x, y,
    //     sd.mounting_position().position().x(),
    //     sd.mounting_position().position().y(),
    //     sd.mounting_position().orientation().yaw(), x,y);
    //
    //
    // Local2GlobalCoordinates(x, y,
    //     sd.host_vehicle_location().position().x(),
    //     sd.host_vehicle_location().position().y(),
    //     sd.host_vehicle_location().orientation().yaw(), x,y);

    osi3::MovingObject *obj = obj_osi_external.sv->mutable_global_ground_truth()->add_moving_object();

    obj->mutable_id()->set_value(object.header().tracking_id().value());
    obj->mutable_base()->mutable_position()->set_x(x);
    obj->mutable_base()->mutable_position()->set_y(y);
    obj->mutable_base()->mutable_position()->set_z(z);
    obj->mutable_base()->mutable_orientation()->set_yaw(yaw);

    obj->mutable_base()->mutable_dimension()->set_height(object.base().dimension().height());
    obj->mutable_base()->mutable_dimension()->set_length(object.base().dimension().length());
    obj->mutable_base()->mutable_dimension()->set_width(object.base().dimension().width());
}

void OSIReporter::CreateLaneBoundaryFromSensordata(const osi3::SensorData &sd, int lane_boundary_nr)
{
    osi3::DetectedLaneBoundary lane_boundary = sd.lane_boundary(lane_boundary_nr);
    double                     x, y, z;
    osi3::LaneBoundary        *new_lane_boundary = obj_osi_external.sv->mutable_global_ground_truth()->add_lane_boundary();

    for (int i = 0; i < sd.lane_boundary(lane_boundary_nr).boundary_line_size(); i++)
    {
        x = lane_boundary.boundary_line(i).position().x() + sd.mounting_position().position().x();
        y = lane_boundary.boundary_line(i).position().y() + sd.mounting_position().position().y();
        z = lane_boundary.boundary_line(i).position().z();

        // Local2GlobalCoordinates(x, y,
        //     sd.mounting_position().position().x(),
        //     sd.mounting_position().position().y(),
        //     sd.mounting_position().orientation().yaw(), x, y);
        //
        // Local2GlobalCoordinates(x, y,
        //    sd.host_vehicle_location().position().x(),
        //    sd.host_vehicle_location().position().y(),
        //    sd.host_vehicle_location().orientation().yaw(), x,y);

        new_lane_boundary->mutable_id()->set_value(lane_boundary.header().ground_truth_id().at(0).value());
        osi3::LaneBoundary_BoundaryPoint *boundary_point = new_lane_boundary->add_boundary_line();

        boundary_point->mutable_position()->set_x(x);
        boundary_point->mutable_position()->set_y(y);
        boundary_point->mutable_position()->set_z(z);
    }
}

const char *OSIReporter::GetOSIGroundTruth(int *size)
{
    if (!(GetUDPClientStatus() == 0 || IsFileOpen()))
    {
        // Data has not been serialized
        obj_osi_external.gt->SerializeToString(&osiGroundTruth.ground_truth);
        osiGroundTruth.size = static_cast<unsigned int>(obj_osi_external.gt->ByteSizeLong());
    }
    *size = static_cast<int>(osiGroundTruth.size);
    return osiGroundTruth.ground_truth.data();
}

const char *OSIReporter::GetOSIGroundTruthRaw()
{
    return reinterpret_cast<char *>(obj_osi_external.gt);
}

const char *OSIReporter::GetOSITrafficCommandRaw()
{
    return reinterpret_cast<char *>(obj_osi_external.tc);
}

const char *OSIReporter::GetOSIRoadLane(const std::vector<std::unique_ptr<ObjectState>> &objectState, int *size, int object_id)
{
    // Check if object_id exists
    if (static_cast<unsigned int>(object_id) >= objectState.size())
    {
        LOG("Object %d not available, only %d registered", object_id, objectState.size());
        *size = 0;
        return 0;
    }

    // Find position of the object
    roadmanager::Position pos;
    for (size_t i = 0; i < objectState.size(); i++)
    {
        if (object_id == objectState[i]->state_.info.id)
        {
            pos = objectState[i]->state_.pos;
            break;
        }
    }

    // find the lane in the sensor view and save its index in the sensor view
    int lane_id_of_vehicle = pos.GetLaneGlobalId();
    int idx                = -1;
    for (unsigned int i = 0; i < obj_osi_internal.ln.size(); i++)
    {
        osi3::Identifier identifier = obj_osi_internal.ln[i]->id();
        int              found_id   = static_cast<int>(identifier.value());
        if (found_id == lane_id_of_vehicle)
        {
            idx = static_cast<int>(i);
            break;
        }
    }
    if (idx < 0)
    {
        LOG("Failed to locate vehicle lane id!");
        return 0;
    }
    // serialize to string the single lane
    obj_osi_internal.ln[static_cast<unsigned int>(idx)]->SerializeToString(&osiRoadLane.osi_lane_info);
    osiRoadLane.size = static_cast<unsigned int>(obj_osi_internal.ln[static_cast<unsigned int>(idx)]->ByteSizeLong());
    *size            = static_cast<int>(osiRoadLane.size);
    return osiRoadLane.osi_lane_info.data();
}

const char *OSIReporter::GetOSIRoadLaneBoundary(int *size, int global_id)
{
    // find the lane bounday in the sensor view and save its index
    int idx = -1;
    for (unsigned int i = 0; i < obj_osi_internal.lnb.size(); i++)
    {
        osi3::Identifier identifier = obj_osi_internal.lnb[i]->id();
        int              found_id   = static_cast<int>(identifier.value());
        if (found_id == global_id)
        {
            idx = static_cast<int>(i);
            break;
        }
    }

    if (idx == -1)
    {
        return 0;
    }

    // serialize to string the single lane
    obj_osi_internal.lnb[static_cast<unsigned int>(idx)]->SerializeToString(&osiRoadLaneBoundary.osi_lane_boundary_info);
    osiRoadLaneBoundary.size = static_cast<unsigned int>(obj_osi_internal.lnb[static_cast<unsigned int>(idx)]->ByteSizeLong());
    *size                    = static_cast<int>(osiRoadLaneBoundary.size);
    return osiRoadLaneBoundary.osi_lane_boundary_info.data();
}

bool OSIReporter::IsCentralOSILane(int lane_idx)
{
    // to check if the lane is a central lane we check if the right and left lane boundary have the same global id.
    osi3::Identifier Left_lb_id = obj_osi_internal.ln[static_cast<unsigned int>(lane_idx)]->mutable_classification()->left_lane_boundary_id(0);
    int              left_lb_id = static_cast<int>(Left_lb_id.value());

    osi3::Identifier Right_lb_id = obj_osi_internal.ln[static_cast<unsigned int>(lane_idx)]->mutable_classification()->right_lane_boundary_id(0);
    int              right_lb_id = static_cast<int>(Right_lb_id.value());

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
    for (unsigned int i = 0; i < obj_osi_internal.ln.size(); i++)
    {
        osi3::Identifier identifier = obj_osi_internal.ln[i]->id();
        int              found_id   = static_cast<int>(identifier.value());
        if (found_id == lane_id)
        {
            idx = static_cast<int>(i);
            break;
        }
    }
    return idx;
}

void OSIReporter::GetOSILaneBoundaryIds(const std::vector<std::unique_ptr<ObjectState>> &objectState, std::vector<int> &ids, int object_id)
{
    int              idx_central, idx_left, idx_right;
    int              left_lb_id, right_lb_id;
    int              far_left_lb_id, far_right_lb_id;
    std::vector<int> final_lb_ids;

    // Check if object_id exists
    if (static_cast<unsigned int>(object_id) >= objectState.size())
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
    idx_central            = GetLaneIdxfromIdOSI(lane_id_of_vehicle);

    // find left and right lane boundary ids of central lane
    if (obj_osi_internal.ln[static_cast<unsigned int>(idx_central)]->mutable_classification()->left_lane_boundary_id_size() == 0)
    {
        left_lb_id = -1;
    }
    else
    {
        osi3::Identifier left_lane = obj_osi_internal.ln[static_cast<unsigned int>(idx_central)]->mutable_classification()->left_lane_boundary_id(0);
        left_lb_id                 = static_cast<int>(left_lane.value());
    }

    if (obj_osi_internal.ln[static_cast<unsigned int>(idx_central)]->mutable_classification()->right_lane_boundary_id_size() == 0)
    {
        right_lb_id = -1;
    }
    else
    {
        osi3::Identifier right_lane =
            obj_osi_internal.ln[static_cast<unsigned int>(idx_central)]->mutable_classification()->right_lane_boundary_id(0);
        right_lb_id = static_cast<int>(right_lane.value());
    }

    // find first left lane
    if (obj_osi_internal.ln[static_cast<unsigned int>(idx_central)]->mutable_classification()->left_adjacent_lane_id_size() == 0)
    {
        far_left_lb_id = -1;
    }
    else
    {
        osi3::Identifier Left_lane_id =
            obj_osi_internal.ln[static_cast<unsigned int>(idx_central)]->mutable_classification()->left_adjacent_lane_id(0);
        int left_lane_id = static_cast<int>(Left_lane_id.value());
        idx_left         = GetLaneIdxfromIdOSI(left_lane_id);

        // save left boundary of left lane as far left lane boundary of central lane
        if (obj_osi_internal.ln[static_cast<unsigned int>(idx_left)]->mutable_classification()->left_lane_boundary_id_size() == 0)
        {
            far_left_lb_id = -1;
        }
        else
        {
            osi3::Identifier Far_left_lb_id =
                obj_osi_internal.ln[static_cast<unsigned int>(idx_left)]->mutable_classification()->left_lane_boundary_id(0);
            far_left_lb_id = static_cast<int>(Far_left_lb_id.value());
        }
    }

    // now find first right lane
    if (obj_osi_internal.ln[static_cast<unsigned int>(idx_central)]->mutable_classification()->right_adjacent_lane_id_size() == 0)
    {
        far_right_lb_id = -1;
    }
    else
    {
        osi3::Identifier Right_lane_id =
            obj_osi_internal.ln[static_cast<unsigned int>(idx_central)]->mutable_classification()->right_adjacent_lane_id(0);
        int right_lane_id = static_cast<int>(Right_lane_id.value());
        idx_right         = GetLaneIdxfromIdOSI(right_lane_id);

        // save right boundary of right lane as far right lane boundary of central lane
        if (obj_osi_internal.ln[static_cast<unsigned int>(idx_right)]->mutable_classification()->right_lane_boundary_id_size() == 0)
        {
            far_right_lb_id = -1;
        }
        else
        {
            osi3::Identifier Far_right_lb_id =
                obj_osi_internal.ln[static_cast<unsigned int>(idx_right)]->mutable_classification()->right_lane_boundary_id(0);
            far_right_lb_id = static_cast<int>(Far_right_lb_id.value());
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

const char *OSIReporter::GetOSISensorDataRaw()
{
    return reinterpret_cast<const char *>(obj_osi_internal.sd);
}

osi3::SensorView *OSIReporter::GetSensorView()
{
    return obj_osi_external.sv;
}

int OSIReporter::SetOSITimeStampExplicit(unsigned long long int nanoseconds)
{
    nanosec_ = nanoseconds;

    return 0;
}
void OSIReporter::SetStationaryModelReference(std::string model_reference)
{
    // Check registered paths for model3d
    std::string model3d_abs_path;
    for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
    {
        std::string file_name_candidate = CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], model_reference);
        if (FileExists(file_name_candidate.c_str()))
        {
            stationary_model_reference = file_name_candidate;
            break;
        }
    }
}
