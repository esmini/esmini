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

#include <sstream>
#include "ScenarioGateway.hpp"
#include "CommonMini.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#else
/* Assume that any non-Windows platform uses POSIX-style sockets instead. */
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
#include <unistd.h> /* Needed for close() */

#include <utility>
#endif

using namespace scenarioengine;

ObjectState::ObjectState()
{
    state_.info.id = -1;
}

ObjectState::ObjectState(int                               id,
                         std::string                       name,
                         int                               obj_type,
                         int                               obj_category,
                         int                               obj_role,
                         int                               model_id,
                         std::string                       model3d,
                         int                               ctrl_type,
                         OSCBoundingBox                    boundingbox,
                         int                               scaleMode,
                         int                               visibilityMask,
                         double                            timestamp,
                         double                            speed,
                         double                            wheel_angle,
                         double                            wheel_rot,
                         double                            rear_axle_z_pos,
                         double                            front_axle_x_pos,
                         double                            front_axle_z_pos,
                         roadmanager::Position*            pos,
                         Object::VehicleLightActionStatus* light_state)
    : dirty_(0)
{
    state_.info.id           = id;
    state_.info.obj_type     = obj_type;
    state_.info.obj_category = obj_category;
    state_.info.obj_role     = obj_role;
    state_.info.model_id     = model_id;
    state_.info.model3d      = model3d;
    state_.info.ctrl_type    = ctrl_type;
    state_.info.timeStamp    = timestamp;
    StrCopy(state_.info.name, name.c_str(), MIN(name.length() + 1, NAME_LEN));
    state_.pos                   = *pos;
    state_.info.speed            = speed;
    state_.info.rear_axle_z_pos  = rear_axle_z_pos;
    state_.info.front_axle_x_pos = front_axle_x_pos;
    state_.info.front_axle_z_pos = front_axle_z_pos;
    state_.info.boundingbox      = boundingbox;
    state_.info.scaleMode        = scaleMode;
    state_.info.visibilityMask   = visibilityMask;
    if (light_state != nullptr)
    {
        for (int i = 0; i < Object::VehicleLightType::NUMBER_OF_VEHICLE_LIGHTS; i++)
        {
            state_.info.light_state[i] = light_state[i];
        }
    }

    for (auto& w : state_.info.wheel_data)
    {
        w.h = static_cast<float>(wheel_angle);
        w.p = static_cast<float>(wheel_rot);
    }

    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION | Object::DirtyBit::LIGHT_STATE;
}

ObjectState::ObjectState(int                               id,
                         std::string                       name,
                         int                               obj_type,
                         int                               obj_category,
                         int                               obj_role,
                         int                               model_id,
                         int                               ctrl_type,
                         OSCBoundingBox                    boundingbox,
                         int                               scaleMode,
                         int                               visibilityMask,
                         double                            timestamp,
                         double                            speed,
                         double                            wheel_angle,
                         double                            wheel_rot,
                         double                            rear_axle_z_pos,
                         double                            x,
                         double                            y,
                         double                            z,
                         double                            h,
                         double                            p,
                         double                            r,
                         Object::VehicleLightActionStatus* light_state)
    : dirty_(0)
{
    state_.info.id           = id;
    state_.info.obj_type     = obj_type;
    state_.info.obj_category = obj_category;
    state_.info.obj_role     = obj_role;
    state_.info.model_id     = model_id;
    state_.info.ctrl_type    = ctrl_type;
    state_.info.name[0]      = 0;
    state_.info.timeStamp    = timestamp;
    StrCopy(state_.info.name, name.c_str(), MIN(name.length() + 1, NAME_LEN));
    state_.pos.Init();
    state_.pos.SetInertiaPos(x, y, z, h, p, r);
    state_.info.speed           = speed;
    state_.info.rear_axle_z_pos = rear_axle_z_pos;
    state_.info.boundingbox     = boundingbox;
    state_.info.scaleMode       = scaleMode;
    state_.info.visibilityMask  = visibilityMask;

    for (auto& w : state_.info.wheel_data)
    {
        w.h = static_cast<float>(wheel_angle);
        w.p = static_cast<float>(wheel_rot);
    }

    if (light_state != nullptr)
    {
        for (int i = 0; i < Object::VehicleLightType::NUMBER_OF_VEHICLE_LIGHTS; i++)
        {
            state_.info.light_state[i] = light_state[i];
        }
    }
    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION | Object::DirtyBit::LIGHT_STATE;
}

ObjectState::ObjectState(int                               id,
                         std::string                       name,
                         int                               obj_type,
                         int                               obj_category,
                         int                               obj_role,
                         int                               model_id,
                         int                               ctrl_type,
                         OSCBoundingBox                    boundingbox,
                         int                               scaleMode,
                         int                               visibilityMask,
                         double                            timestamp,
                         double                            speed,
                         double                            wheel_angle,
                         double                            wheel_rot,
                         double                            rear_axle_z_pos,
                         int                               roadId,
                         int                               laneId,
                         double                            laneOffset,
                         double                            s,
                         Object::VehicleLightActionStatus* light_state)
    : dirty_(0)
{
    state_.info.id           = id;
    state_.info.obj_type     = obj_type;
    state_.info.obj_category = obj_category;
    state_.info.obj_role     = obj_role;
    state_.info.model_id     = model_id;
    state_.info.ctrl_type    = ctrl_type;
    state_.info.timeStamp    = timestamp;
    StrCopy(state_.info.name, name.c_str(), MIN(name.length() + 1, NAME_LEN));
    state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
    state_.info.speed           = speed;
    state_.info.rear_axle_z_pos = rear_axle_z_pos;
    state_.info.boundingbox     = boundingbox;
    state_.info.scaleMode       = scaleMode;
    state_.info.visibilityMask  = visibilityMask;
    if (light_state != nullptr)
    {
        for (int i = 0; i < Object::VehicleLightType::NUMBER_OF_VEHICLE_LIGHTS; i++)
        {
            state_.info.light_state[i] = light_state[i];
        }
    }

    for (auto& w : state_.info.wheel_data)
    {
        w.h = static_cast<float>(wheel_angle);
        w.p = static_cast<float>(wheel_rot);
    }

    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION | Object::DirtyBit::LIGHT_STATE;
}

ObjectState::ObjectState(int                               id,
                         std::string                       name,
                         int                               obj_type,
                         int                               obj_category,
                         int                               obj_role,
                         int                               model_id,
                         int                               ctrl_type,
                         OSCBoundingBox                    boundingbox,
                         int                               scaleMode,
                         int                               visibilityMask,
                         double                            timestamp,
                         double                            speed,
                         double                            wheel_angle,
                         double                            wheel_rot,
                         double                            rear_axle_z_pos,
                         int                               roadId,
                         double                            lateralOffset,
                         double                            s,
                         Object::VehicleLightActionStatus* light_state)
{
    state_.info.id           = id;
    state_.info.obj_type     = obj_type;
    state_.info.obj_category = obj_category;
    state_.info.obj_role     = obj_role;
    state_.info.model_id     = model_id;
    state_.info.ctrl_type    = ctrl_type;
    state_.info.timeStamp    = timestamp;
    StrCopy(state_.info.name, name.c_str(), MIN(name.length() + 1, NAME_LEN));
    state_.pos.SetTrackPos(roadId, s, lateralOffset);
    state_.info.speed           = speed;
    state_.info.rear_axle_z_pos = rear_axle_z_pos;
    state_.info.boundingbox     = boundingbox;
    state_.info.scaleMode       = scaleMode;
    state_.info.visibilityMask  = visibilityMask;

    for (auto& w : state_.info.wheel_data)
    {
        w.h = static_cast<float>(wheel_angle);
        w.p = static_cast<float>(wheel_rot);
    }

    if (light_state != nullptr)
    {
        for (int i = 0; i < Object::VehicleLightType::NUMBER_OF_VEHICLE_LIGHTS; i++)
        {
            state_.info.light_state[i] = light_state[i];
        }
    }
    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION | Object::DirtyBit::LIGHT_STATE;
}

void ObjectState::Print()
{
    LOG("state: \n\tid %d\n\tname %s\n\tmodel_id: %d\n\tctrl_type: %d\n\ttime %.2f\n\tx %.2f\n\ty %.2f\n\th %.2f\n\tspeed %.2f\twheel_angle %.2f type %d category %d role %d",
        state_.info.id,
        state_.info.name,
        state_.info.model_id,
        state_.info.ctrl_type,
        state_.info.timeStamp,
        state_.pos.GetX(),
        state_.pos.GetY(),
        state_.pos.GetZ(),
        state_.info.speed,
        state_.info.wheel_data[0].h,
        state_.info.obj_type,
        state_.info.obj_category,
        state_.info.obj_role);
    LOG("state: \n\tbounding box: \ncenter: x: %.2f, y: %.2f, z: %.2f\n\tdimensions: width: %.2f, length: %.2f, height: %.2f scaleMode: %d visMask: %d",
        static_cast<double>(state_.info.boundingbox.center_.x_),
        static_cast<double>(state_.info.boundingbox.center_.y_),
        static_cast<double>(state_.info.boundingbox.center_.z_),
        static_cast<double>(state_.info.boundingbox.dimensions_.width_),
        static_cast<double>(state_.info.boundingbox.dimensions_.length_),
        static_cast<double>(state_.info.boundingbox.dimensions_.height_),
        static_cast<double>(state_.info.scaleMode),
        static_cast<double>(state_.info.visibilityMask));
}

// ScenarioGateway

ScenarioGateway::ScenarioGateway()
{
}

ScenarioGateway::~ScenarioGateway()
{
    objectState_.clear();
    delete datLogger;
}

ObjectState* ScenarioGateway::getObjectStatePtrById(int id)
{
    for (size_t i = 0; i < objectState_.size(); i++)
    {
        if (objectState_[i]->state_.info.id == id)
        {
            return objectState_[i].get();
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

int ScenarioGateway::updateObjectInfo(ObjectState*                      obj_state,
                                      double                            timestamp,
                                      int                               visibilityMask,
                                      double                            speed,
                                      double                            wheel_angle,
                                      double                            wheel_rot,
                                      Object::VehicleLightActionStatus* light_state)
{
    if (!obj_state)
    {
        return -1;
    }

    obj_state->state_.info.speed          = speed;
    obj_state->state_.info.timeStamp      = timestamp;
    obj_state->state_.info.visibilityMask = visibilityMask;
    if (light_state != nullptr)
    {
        for (int i = 0; i < Object::VehicleLightType::NUMBER_OF_VEHICLE_LIGHTS; i++)
        {
            obj_state->state_.info.light_state[i] = light_state[i];
        }
    }

    for (auto& w : obj_state->state_.info.wheel_data)
    {
        if (w.axle == 0)
        {
            w.h = static_cast<float>(wheel_angle);
        }
        w.p = static_cast<float>(wheel_rot);
    }

    obj_state->dirty_ |= Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE | Object::DirtyBit::WHEEL_ROTATION | Object::DirtyBit::LIGHT_STATE;

    return 0;
}

int ScenarioGateway::reportObject(int                               id,
                                  std::string                       name,
                                  int                               obj_type,
                                  int                               obj_category,
                                  int                               obj_role,
                                  int                               model_id,
                                  std::string                       model3d,
                                  int                               ctrl_type,
                                  OSCBoundingBox                    boundingbox,
                                  int                               scaleMode,
                                  int                               visibilityMask,
                                  double                            timestamp,
                                  double                            speed,
                                  double                            wheel_angle,
                                  double                            wheel_rot,
                                  double                            rear_axle_z_pos,
                                  double                            front_axle_x_pos,
                                  double                            front_axle_z_pos,
                                  roadmanager::Position*            pos,
                                  Object::VehicleLightActionStatus* light_state)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Check registered paths for model3d
        std::string model3d_abs_path;
        for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
        {
            std::string file_name_candidate = CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], model3d);
            if (FileExists(file_name_candidate.c_str()))
            {
                model3d_abs_path = file_name_candidate;
                break;
            }
        }

        // Create state and set permanent information
        obj_state = new ObjectState(id,
                                    name,
                                    obj_type,
                                    obj_category,
                                    obj_role,
                                    model_id,
                                    model3d_abs_path,
                                    ctrl_type,
                                    boundingbox,
                                    scaleMode,
                                    visibilityMask,
                                    timestamp,
                                    speed,
                                    wheel_angle,
                                    wheel_rot,
                                    rear_axle_z_pos,
                                    front_axle_x_pos,
                                    front_axle_z_pos,
                                    pos,
                                    light_state);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos = *pos;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot, light_state);
    }

    return 0;
}

int ScenarioGateway::reportObject(int                               id,
                                  std::string                       name,
                                  int                               obj_type,
                                  int                               obj_category,
                                  int                               obj_role,
                                  int                               model_id,
                                  int                               ctrl_type,
                                  OSCBoundingBox                    boundingbox,
                                  int                               scaleMode,
                                  int                               visibilityMask,
                                  double                            timestamp,
                                  double                            speed,
                                  double                            wheel_angle,
                                  double                            wheel_rot,
                                  double                            rear_axle_z_pos,
                                  double                            x,
                                  double                            y,
                                  double                            z,
                                  double                            h,
                                  double                            p,
                                  double                            r,
                                  Object::VehicleLightActionStatus* light_state)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
        obj_state = new ObjectState(id,
                                    name,
                                    obj_type,
                                    obj_category,
                                    obj_role,
                                    model_id,
                                    ctrl_type,
                                    boundingbox,
                                    scaleMode,
                                    visibilityMask,
                                    timestamp,
                                    speed,
                                    wheel_angle,
                                    wheel_rot,
                                    rear_axle_z_pos,
                                    x,
                                    y,
                                    z,
                                    h,
                                    p,
                                    r,
                                    light_state);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos.SetInertiaPos(x, y, z, h, p, r);
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot, light_state);
    }

    return 0;
}

int ScenarioGateway::reportObject(int                               id,
                                  std::string                       name,
                                  int                               obj_type,
                                  int                               obj_category,
                                  int                               obj_role,
                                  int                               model_id,
                                  int                               ctrl_type,
                                  OSCBoundingBox                    boundingbox,
                                  int                               scaleMode,
                                  int                               visibilityMask,
                                  double                            timestamp,
                                  double                            speed,
                                  double                            wheel_angle,
                                  double                            wheel_rot,
                                  double                            rear_axle_z_pos,
                                  double                            x,
                                  double                            y,
                                  double                            h,
                                  Object::VehicleLightActionStatus* light_state)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
        obj_state = new ObjectState(id,
                                    name,
                                    obj_type,
                                    obj_category,
                                    obj_role,
                                    model_id,
                                    ctrl_type,
                                    boundingbox,
                                    scaleMode,
                                    visibilityMask,
                                    timestamp,
                                    speed,
                                    wheel_angle,
                                    wheel_rot,
                                    rear_axle_z_pos,
                                    x,
                                    y,
                                    0,
                                    h,
                                    0,
                                    0,
                                    light_state);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos.SetInertiaPos(x, y, h);
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot, light_state);
    }

    return 0;
}

int ScenarioGateway::reportObject(int                               id,
                                  std::string                       name,
                                  int                               obj_type,
                                  int                               obj_category,
                                  int                               obj_role,
                                  int                               model_id,
                                  int                               ctrl_type,
                                  OSCBoundingBox                    boundingbox,
                                  int                               scaleMode,
                                  int                               visibilityMask,
                                  double                            timestamp,
                                  double                            speed,
                                  double                            wheel_angle,
                                  double                            wheel_rot,
                                  double                            rear_axle_z_pos,
                                  int                               roadId,
                                  int                               laneId,
                                  double                            laneOffset,
                                  double                            s,
                                  Object::VehicleLightActionStatus* light_state)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
        obj_state = new ObjectState(id,
                                    name,
                                    obj_type,
                                    obj_category,
                                    obj_role,
                                    model_id,
                                    ctrl_type,
                                    boundingbox,
                                    scaleMode,
                                    visibilityMask,
                                    timestamp,
                                    speed,
                                    wheel_angle,
                                    wheel_rot,
                                    rear_axle_z_pos,
                                    roadId,
                                    laneId,
                                    laneOffset,
                                    s,
                                    light_state);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot, light_state);
    }

    return 0;
}

int ScenarioGateway::reportObject(int            id,
                                  std::string    name,
                                  int            obj_type,
                                  int            obj_category,
                                  int            obj_role,
                                  int            model_id,
                                  int            ctrl_type,
                                  OSCBoundingBox boundingbox,
                                  int            scaleMode,
                                  int            visibilityMask,
                                  double         timestamp,
                                  double         speed,
                                  double         wheel_angle,
                                  double         wheel_rot,
                                  double         rear_axle_z_pos,
                                  int            roadId,
                                  double         lateralOffset,
                                  double         s,
                                  Object::VehicleLightActionStatus* light_state)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
        obj_state = new ObjectState(id,
                                    name,
                                    obj_type,
                                    obj_category,
                                    obj_role,
                                    model_id,
                                    ctrl_type,
                                    boundingbox,
                                    scaleMode,
                                    visibilityMask,
                                    timestamp,
                                    speed,
                                    wheel_angle,
                                    wheel_rot,
                                    rear_axle_z_pos,
                                    roadId,
                                    lateralOffset,
                                    s,
                                    light_state);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos.SetTrackPos(roadId, s, lateralOffset);
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot, light_state);
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
        obj_state->state_.pos            = *pos;
        obj_state->state_.info.timeStamp = timestamp;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;
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
        obj_state->state_.info.timeStamp = timestamp;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;
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
        obj_state->state_.info.timeStamp = timestamp;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;
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
        obj_state->state_.info.timeStamp = timestamp;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;
    }

    return 0;
}

int ScenarioGateway::updateObjectWorldPosXYHMode(int id, double timestamp, double x, double y, double h, int mode)
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
        obj_state->state_.pos.SetInertiaPosMode(x, y, h, mode);
        obj_state->state_.info.timeStamp = timestamp;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;
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
        obj_state->state_.info.timeStamp = timestamp;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;
    }

    return 0;
}

int ScenarioGateway::updateObjectWorldPosMode(int id, double timestamp, double x, double y, double z, double h, double p, double r, int mode)
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
        obj_state->state_.pos.SetInertiaPosMode(x, y, z, h, p, r, mode);
        obj_state->state_.info.timeStamp = timestamp;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;
    }

    return 0;
}

int ScenarioGateway::updateObjectSpeed(int id, double timestamp, double speed)
{
    (void)timestamp;
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set speed for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    obj_state->state_.info.speed = speed;
    obj_state->dirty_ |= Object::DirtyBit::SPEED;

    return 0;
}

int ScenarioGateway::updateObjectVel(int id, double timestamp, double x_vel, double y_vel, double z_vel)
{
    (void)timestamp;
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
    (void)timestamp;
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
    (void)timestamp;
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
    (void)timestamp;
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
    (void)timestamp;
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set wheel angle for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    // update any wheel on front axle
    for (auto& w : obj_state->state_.info.wheel_data)
    {
        if (w.axle == 0)
        {
            w.h = static_cast<float>(wheelAngle);
            obj_state->dirty_ |= Object::DirtyBit::WHEEL_ANGLE;
        }
    }

    return 0;
}

int ScenarioGateway::updateObjectWheelRotation(int id, double timestamp, double wheelRotation)
{
    (void)timestamp;
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set wheel rotation for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    // update all wheels
    for (auto& w : obj_state->state_.info.wheel_data)
    {
        w.p = static_cast<float>(wheelRotation);
        obj_state->dirty_ |= Object::DirtyBit::WHEEL_ROTATION;
    }

    return 0;
}

int ScenarioGateway::updateObjectVisibilityMask(int id, int visibilityMask)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set visibility mask for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    obj_state->state_.info.visibilityMask = visibilityMask;
    obj_state->dirty_ |= Object::DirtyBit::VISIBILITY;

    return 0;
}

int ScenarioGateway::updateObjectControllerType(int id, int controllerType)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set controller type for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    obj_state->state_.info.ctrl_type = controllerType;
    obj_state->dirty_ |= Object::DirtyBit::CONTROLLER;

    return 0;
}

int ScenarioGateway::updateObjectWheelData(int id, std::vector<WheelData> wheel_data)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set friction coefficients for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    for (unsigned int i = 0; i < wheel_data.size(); i++)
    {
        if (obj_state->state_.info.wheel_data.size() <= i)
        {
            // push first time
            obj_state->state_.info.wheel_data.push_back(std::move(wheel_data[i]));
        }
        else
        {
            // update existing
            obj_state->state_.info.wheel_data[i] = std::move(wheel_data[i]);
        }
    }

    obj_state->dirty_ |= Object::DirtyBit::FRICTION | Object::DirtyBit::WHEEL_ANGLE | Object::DirtyBit::WHEEL_ROTATION;

    return 0;
}

int ScenarioGateway::updateObjectLightState(int id, Object::VehicleLightActionStatus* light_state)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set visibility mask for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    for (int i = 0; i < Object::VehicleLightType::NUMBER_OF_VEHICLE_LIGHTS; i++)
    {
        if (light_state[i].type != Object::VehicleLightType::UNDEFINED)
        {
            obj_state->state_.info.light_state[i] = light_state[i];
        }
    }
    obj_state->dirty_ |= Object::DirtyBit::LIGHT_STATE;

    return 0;
}

int ScenarioGateway::setObjectPositionMode(int id, int type, int mode)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set alignment mode for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    if (type < 0 || type > static_cast<int>(roadmanager::Position::PosModeType::UPDATE))
    {
        LOG_ONCE("Unexpected ObjectPositionMode type %d, skipping", type);
        return -1;
    }

    obj_state->state_.pos.SetModes(type, mode);

    if ((mode & roadmanager::Position::PosMode::Z_SET) != 0)
    {
        obj_state->dirty_ |=
            (type == static_cast<int>(roadmanager::Position::PosModeType::SET) ? static_cast<int>(Object::DirtyBit::ALIGN_MODE_Z_SET)
                                                                               : static_cast<int>(Object::DirtyBit::ALIGN_MODE_Z_UPDATE));
    }
    if ((mode & roadmanager::Position::PosMode::H_SET) != 0)
    {
        obj_state->dirty_ |=
            (type == static_cast<int>(roadmanager::Position::PosModeType::SET) ? static_cast<int>(Object::DirtyBit::ALIGN_MODE_H_SET)
                                                                               : static_cast<int>(Object::DirtyBit::ALIGN_MODE_H_UPDATE));
    }
    if ((mode & roadmanager::Position::PosMode::P_SET) != 0)
    {
        obj_state->dirty_ |=
            (type == static_cast<int>(roadmanager::Position::PosModeType::SET) ? static_cast<int>(Object::DirtyBit::ALIGN_MODE_P_SET)
                                                                               : static_cast<int>(Object::DirtyBit::ALIGN_MODE_P_UPDATE));
    }
    if ((mode & roadmanager::Position::PosMode::R_SET) != 0)
    {
        obj_state->dirty_ |=
            (type == static_cast<int>(roadmanager::Position::PosModeType::SET) ? static_cast<int>(Object::DirtyBit::ALIGN_MODE_R_SET)
                                                                               : static_cast<int>(Object::DirtyBit::ALIGN_MODE_R_UPDATE));
    }

    return 0;
}

int ScenarioGateway::setObjectPositionModeDefault(int id, int type)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set alignment mode for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    obj_state->state_.pos.SetModeDefault(static_cast<roadmanager::Position::PosModeType>(type));
    if (type == static_cast<int>(roadmanager::Position::PosModeType::SET))
    {
        obj_state->dirty_ |= (Object::DirtyBit::ALIGN_MODE_H_SET | Object::DirtyBit::ALIGN_MODE_P_SET | Object::DirtyBit::ALIGN_MODE_R_SET |
                              Object::DirtyBit::ALIGN_MODE_Z_SET);
    }
    else
    {
        obj_state->dirty_ |= (Object::DirtyBit::ALIGN_MODE_H_UPDATE | Object::DirtyBit::ALIGN_MODE_P_UPDATE | Object::DirtyBit::ALIGN_MODE_R_UPDATE |
                              Object::DirtyBit::ALIGN_MODE_Z_UPDATE);
    }

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
    for (auto objectIt = std::begin(objectState_); objectIt != std::end(objectState_);)
    {
        if ((*objectIt)->state_.info.id == id)
        {
            objectIt = objectState_.erase(objectIt);
        }
        else
        {
            ++objectIt;
        }
    }
}

void ScenarioGateway::removeObject(std::string name)
{
    for (auto objectIt = std::begin(objectState_); objectIt != std::end(objectState_);)
    {
        if ((*objectIt)->state_.info.name == name)
        {
            objectIt = objectState_.erase(objectIt);
        }
        else
        {
            ++objectIt;
        }
    }
}

int ScenarioGateway::WriteStatesToFile()
{
    // std::cout << "Inside write-->" << std::endl;
    if (datLogger != nullptr && datLogger->IsFileOpen())
    {
        // Write status to file - for later replay
        for (size_t i = 0; i < objectState_.size(); i++)
        {
            int objId              = objectState_[i]->state_.info.id;
            datLogger->simTimeTemp = objectState_[i]->state_.info.timeStamp;
            datLogger->AddObject(objId);
            datLogger->WriteModelId(objId, objectState_[i]->state_.info.model_id);
            datLogger->WriteObjPos(objId,
                                   objectState_[i]->state_.pos.GetX(),
                                   objectState_[i]->state_.pos.GetY(),
                                   objectState_[i]->state_.pos.GetZ(),
                                   objectState_[i]->state_.pos.GetH(),
                                   objectState_[i]->state_.pos.GetP(),
                                   objectState_[i]->state_.pos.GetR());
            datLogger->WriteObjSpeed(objectState_[i]->state_.info.id, objectState_[i]->state_.info.speed);
            datLogger->WriteObjCategory(objId, objectState_[i]->state_.info.obj_category);
            datLogger->WriteObjType(objId, objectState_[i]->state_.info.obj_type);
            datLogger->WriteCtrlType(objId, objectState_[i]->state_.info.ctrl_type);
            datLogger->WriteWheelAngle(objId, objectState_[i]->state_.info.wheel_data.size() > 0 ? static_cast<float>(objectState_[i]->state_.info.wheel_data[0].h) : 0.0f);
            datLogger->WriteWheelRot(objId, objectState_[i]->state_.info.wheel_data.size() > 0 ? static_cast<float>(objectState_[i]->state_.info.wheel_data[0].p) : 0.0f);
            datLogger->WriteBB(objId,
                               objectState_[i]->state_.info.boundingbox.center_.x_,
                               objectState_[i]->state_.info.boundingbox.center_.y_,
                               objectState_[i]->state_.info.boundingbox.center_.z_,
                               objectState_[i]->state_.info.boundingbox.dimensions_.length_,
                               objectState_[i]->state_.info.boundingbox.dimensions_.width_,
                               objectState_[i]->state_.info.boundingbox.dimensions_.height_);
            datLogger->WriteScaleMode(objId, objectState_[i]->state_.info.scaleMode);
            datLogger->WriteVisiblityMask(objId, objectState_[i]->state_.info.visibilityMask);
            datLogger->WriteName(objId, objectState_[i]->state_.info.name);
            datLogger->WriteRoadId(objId, objectState_[i]->state_.pos.GetTrackId());
            datLogger->WriteLaneId(objId, objectState_[i]->state_.pos.GetLaneId());
            datLogger->WritePosOffset(objId, objectState_[i]->state_.pos.GetOffset());
            datLogger->WritePosT(objId, objectState_[i]->state_.pos.GetT());
            datLogger->WritePosS(objId, objectState_[i]->state_.pos.GetS());

            double                rgb_[4];
            datLogger::LightState lightState_;
            for (int j = 0; j < Object::VehicleLightType::NUMBER_OF_VEHICLE_LIGHTS; j++)
            {
                datLogger::LightRGB rgb_value;
                rgb_[0] = objectState_[i]->state_.info.light_state[j].diffuseRgb[0] + objectState_[i]->state_.info.light_state[j].emissionRgb[0];
                rgb_[1] = objectState_[i]->state_.info.light_state[j].diffuseRgb[1] + objectState_[i]->state_.info.light_state[j].emissionRgb[1];
                rgb_[2] = objectState_[i]->state_.info.light_state[j].diffuseRgb[2] + objectState_[i]->state_.info.light_state[j].emissionRgb[2];
                rgb_[3] = objectState_[i]->state_.info.light_state[j].emissionRgb[0] / rgb_[0];

                // Convert doubles [0:1] into bytes (unsigned chars) [0:255]
                // ensure range [0:1]
                rgb_value.red       = static_cast<unsigned char>(MIN(MAX(rgb_[0], 0.0), 255.0) * 255.0);
                rgb_value.green     = static_cast<unsigned char>(MIN(MAX(rgb_[1], 0.0), 255.0) * 255.0);
                rgb_value.blue      = static_cast<unsigned char>(MIN(MAX(rgb_[2], 0.0), 255.0) * 255.0);
                rgb_value.intensity = static_cast<unsigned char>(MIN(MAX(rgb_[3], 0.0), 255.0) * 255.0);
                // printf("Obj[%d]Light[%d]RGB[%d]: %.2f -> %.2f -> %d\n", static_cast<int>(i), j, (j * 4) + k, rgb_[k], adjusted_value,

                switch (static_cast<Object::VehicleLightType>(j))
                {
                    case Object::VehicleLightType::DAY_TIME_RUNNING_LIGHTS:
                        lightState_.day_time_running_lights = rgb_value;
                        break;
                    case Object::VehicleLightType::LOW_BEAM:
                        lightState_.low_beam = rgb_value;
                        break;
                    case Object::VehicleLightType::HIGH_BEAM:
                        lightState_.high_beam = rgb_value;
                        break;
                    case Object::VehicleLightType::FOG_LIGHTS_FRONT:
                        lightState_.fog_lights_front = rgb_value;
                        break;
                    case Object::VehicleLightType::FOG_LIGHTS_REAR:
                        lightState_.fog_lights_rear = rgb_value;
                        break;
                    case Object::VehicleLightType::BRAKE_LIGHTS:
                        lightState_.brake_lights = rgb_value;
                        break;
                    case Object::VehicleLightType::INDICATOR_LEFT:
                        lightState_.indicator_left = rgb_value;
                        break;
                    case Object::VehicleLightType::INDICATOR_RIGHT:
                        lightState_.indicator_right = rgb_value;
                        break;
                    case Object::VehicleLightType::REVERSING_LIGHTS:
                        lightState_.reversing_lights = rgb_value;
                        break;
                    case Object::VehicleLightType::LICENSE_PLATER_ILLUMINATION:
                        lightState_.license_plater_illumination = rgb_value;
                        break;
                    case Object::VehicleLightType::SPECIAL_PURPOSE_LIGHTS:
                        lightState_.special_purpose_lights = rgb_value;
                        break;
                    case Object::VehicleLightType::FOG_LIGHTS:
                        lightState_.fog_lights = rgb_value;
                        break;
                    case Object::VehicleLightType::WARNING_LIGHTS:
                        lightState_.warning_lights = rgb_value;
                        break;
                    default:
                        break;
                }
            }
            datLogger->WriteLightState(objId, lightState_);
            datLogger->ObjIdPkgAdded = false;
        }
        datLogger->deleteObject();
        datLogger->TimePkgAdded = false;
    }
    return 0;
}

int ScenarioGateway::RecordToFile(std::string filename, std::string odr_filename, std::string model_filename)
{
    if (!filename.empty())
    {
        int ver = DAT_FILE_FORMAT_VERSION;
        if (datLogger == nullptr)
        {
            if ((datLogger = new datLogger::DatLogger()) == nullptr)
            {
                return -1;
            }

            if (datLogger->init(filename, ver, odr_filename, model_filename) != 0)
            {
                delete datLogger;
                datLogger = nullptr;
                return -1;
            }
        }
    }
    return 0;
}
