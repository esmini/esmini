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

ObjectState::ObjectState(int                          id,
                         std::string                  name,
                         int                          obj_type,
                         int                          obj_category,
                         int                          obj_role,
                         int                          model_id,
                         std::string                  model3d,
                         int                          ctrl_type,
                         OSCBoundingBox               boundingbox,
                         int                          scaleMode,
                         int                          visibilityMask,
                         double                       timestamp,
                         double                       speed,
                         double                       wheel_angle,
                         double                       wheel_rot,
                         double                       rear_axle_z_pos,
                         double                       front_axle_x_pos,
                         double                       front_axle_z_pos,
                         const roadmanager::Position* pos,
                         std::string                  source_reference,
                         double                       refpoint_x_offset,
                         double                       model_x_offset)
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
    state_.pos                    = *pos;
    state_.info.speed             = speed;
    state_.info.rear_axle_z_pos   = rear_axle_z_pos;
    state_.info.front_axle_x_pos  = front_axle_x_pos;
    state_.info.front_axle_z_pos  = front_axle_z_pos;
    state_.info.boundingbox       = boundingbox;
    state_.info.scaleMode         = scaleMode;
    state_.info.visibilityMask    = visibilityMask;
    state_.info.refpoint_x_offset = refpoint_x_offset;
    state_.info.model_x_offset    = model_x_offset;

    for (auto& w : state_.info.wheel_data)
    {
        w.h = static_cast<float>(wheel_angle);
        w.p = static_cast<float>(wheel_rot);
    }

    state_.info.source_reference = source_reference;

    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION;
}

ObjectState::ObjectState(int            id,
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
                         double         x,
                         double         y,
                         double         z,
                         double         h,
                         double         p,
                         double         r)
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

    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION;
}

ObjectState::ObjectState(int            id,
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
                         id_t           roadId,
                         int            laneId,
                         double         laneOffset,
                         double         s)
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

    for (auto& w : state_.info.wheel_data)
    {
        w.h = static_cast<float>(wheel_angle);
        w.p = static_cast<float>(wheel_rot);
    }

    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION;
}

ObjectState::ObjectState(int            id,
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
                         id_t           roadId,
                         double         lateralOffset,
                         double         s)
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

    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION;
}

void ObjectState::Print()
{
    LOG_INFO(
        "state: \n\tid {}\n\tname {}\n\tmodel_id: {}\n\tctrl_type: {}\n\ttime {:.2f}\n\tx {:.2f}\n\ty {:.2f}\n\th {:.2f}\n\tspeed {:.2f}\twheel_angle {:.2f} type {} category {} role {}",
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
    LOG_INFO(
        "state: \n\tbounding box: \ncenter: x: {:.2f}, y: {:.2f}, z: {:.2f}\n\tdimensions: width: {:.2f}, length: {:.2f}, height: {:.2f} scaleMode: {} visMask: {}",
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

    data_file_.flush();
    data_file_.close();
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

int ScenarioGateway::getObjectStateById(int id, ObjectState& objectState) const
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

int ScenarioGateway::updateObjectInfo(ObjectState* obj_state,
                                      double       timestamp,
                                      int          visibilityMask,
                                      double       speed,
                                      double       wheel_angle,
                                      double       wheel_rot)
{
    if (!obj_state)
    {
        return -1;
    }

    obj_state->state_.info.speed          = speed;
    obj_state->state_.info.timeStamp      = timestamp;
    obj_state->state_.info.visibilityMask = visibilityMask;

    for (auto& w : obj_state->state_.info.wheel_data)
    {
        if (w.axle == 0)
        {
            w.h = static_cast<float>(wheel_angle);
        }
        w.p = static_cast<float>(wheel_rot);
    }

    obj_state->dirty_ |= Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE | Object::DirtyBit::WHEEL_ROTATION;

    return 0;
}

int ScenarioGateway::reportObject(int                    id,
                                  std::string            name,
                                  int                    obj_type,
                                  int                    obj_category,
                                  int                    obj_role,
                                  int                    model_id,
                                  std::string            model3d,
                                  int                    ctrl_type,
                                  OSCBoundingBox         boundingbox,
                                  int                    scaleMode,
                                  int                    visibilityMask,
                                  double                 timestamp,
                                  double                 speed,
                                  double                 wheel_angle,
                                  double                 wheel_rot,
                                  double                 rear_axle_z_pos,
                                  double                 front_axle_x_pos,
                                  double                 front_axle_z_pos,
                                  roadmanager::Position* pos,
                                  std::string            source_reference,
                                  double                 refpoint_x_offset,
                                  double                 model_x_offset)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        obj_state = new ObjectState(id,
                                    name,
                                    obj_type,
                                    obj_category,
                                    obj_role,
                                    model_id,
                                    model3d,
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
                                    source_reference,
                                    refpoint_x_offset,
                                    model_x_offset);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos = *pos;
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot);
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
                                  double         x,
                                  double         y,
                                  double         z,
                                  double         h,
                                  double         p,
                                  double         r)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG_INFO("Creating new object \"{}\" (id {}, timestamp {:.2f})", name, id, timestamp);
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
                                    r);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos.SetInertiaPos(x, y, z, h, p, r);
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot);
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
                                  double         x,
                                  double         y,
                                  double         h)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG_INFO("Creating new object \"{}\" (id {}, timestamp {:.2f})", name, id, timestamp);
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
                                    0);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos.SetInertiaPos(x, y, h);
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot);
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
                                  id_t           roadId,
                                  int            laneId,
                                  double         laneOffset,
                                  double         s)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG_INFO("Creating new object \"{}\" (id {}, timestamp {:.2f})", name, id, timestamp);
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
                                    s);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot);
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
                                  id_t           roadId,
                                  double         lateralOffset,
                                  double         s)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG_INFO("Creating new object \"{}\" (id {}, timestamp {:.2f})", name, id, timestamp);
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
                                    s);

        // Add object to collection
        objectState_.push_back(std::unique_ptr<ObjectState>{obj_state});
    }
    else
    {
        // Update status
        obj_state->state_.pos.SetTrackPos(roadId, s, lateralOffset);
        obj_state->dirty_ |= Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL;

        updateObjectInfo(obj_state, timestamp, visibilityMask, speed, wheel_angle, wheel_rot);
    }

    return 0;
}

int ScenarioGateway::updateObjectPos(int id, double timestamp, const roadmanager::Position* pos)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG_ERROR("Object id: {} must be reported before updated", id);
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

int ScenarioGateway::updateObjectRoadPos(int id, double timestamp, id_t roadId, double lateralOffset, double s)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG_ERROR("Object id: {} must be reported before updated", id);
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

int ScenarioGateway::updateObjectLanePos(int id, double timestamp, id_t roadId, int laneId, double offset, double s)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == 0)
    {
        // Create state and set permanent information
        LOG_ERROR("Object id: {} must be reported before updated", id);
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
        LOG_ERROR("Object id: {} must be reported before updated", id);
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
        LOG_ERROR("Object id: {} must be reported before updated", id);
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
        LOG_ERROR("Object id: {} must be reported before updated", id);
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
        LOG_ERROR("Object id: {} must be reported before updated", id);
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
        LOG_ERROR("Can't set speed for object {} yet. Please register object using reportObject() first.", id);
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
        LOG_ERROR("Can't set velocity for object {} yet. Please register object using reportObject() first.", id);
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
        LOG_ERROR("Can't set acceleration for object {} yet. Please register object using reportObject() first.", id);
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
        LOG_ERROR("Can't set angular velocity for object {} yet. Please register object using reportObject() first.", id);
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
        LOG_ERROR("Can't set angular acceleration for object {} yet. Please register object using reportObject() first.", id);
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
        LOG_ERROR("Can't set wheel angle for object {} yet. Please register object using reportObject() first.", id);
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

int ScenarioGateway::updateObjectLaneTypeSnapMask(int id, double timestamp, int laneTypeMask)
{
    (void)timestamp;
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ERROR("Can't set lane type mask for object {} yet. Please register object using reportObject() first.", id);
        return -1;
    }

    obj_state->state_.pos.SetSnapLaneTypes(laneTypeMask);
    obj_state->dirty_ |= Object::DirtyBit::LANE_TYPE_SNAP_MASK;

    return 0;
}

int ScenarioGateway::updateObjectWheelRotation(int id, double timestamp, double wheelRotation)
{
    (void)timestamp;
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ERROR("Can't set wheel rotation for object {} yet. Please register object using reportObject() first.", id);
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
        LOG_ERROR("Can't set visibility mask for object {} yet. Please register object using reportObject() first.", id);
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
        LOG_ERROR("Can't set controller type for object {} yet. Please register object using reportObject() first.", id);
        return -1;
    }

    obj_state->state_.info.ctrl_type = controllerType;
    obj_state->dirty_ |= Object::DirtyBit::CONTROLLER;

    return 0;
}

int ScenarioGateway::updateObjectBoundingBox(int id, OSCBoundingBox bb)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ERROR("Can't set bounding box for object {} yet. Please register object using reportObject() first.", id);
        return -1;
    }

    obj_state->state_.info.boundingbox = bb;
    obj_state->dirty_ |= Object::DirtyBit::BOUNDING_BOX;

    return 0;
}

int ScenarioGateway::updateObjectWheelData(int id, std::vector<WheelData> wheel_data)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ERROR("Can't set friction coefficients for object {} yet. Please register object using reportObject() first.", id);
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

int ScenarioGateway::setObjectPositionMode(int id, int type, int mode)
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ERROR("Can't set alignment mode for object {} yet. Please register object using reportObject() first.", id);
        return -1;
    }

    if (type < 0 || type > static_cast<int>(roadmanager::Position::PosModeType::UPDATE))
    {
        LOG_ERROR_ONCE("Unexpected ObjectPositionMode type {}, skipping", type);
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
        LOG_ERROR("Can't set alignment mode for object {} yet. Please register object using reportObject() first.", id);
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

void ScenarioGateway::WriteStatesToFile(const double simulation_time, const double dt)
{
    if (dat_writer_.IsWriteFileOpen())
    {
        dat_writer_.SetSimulationTime(simulation_time, dt);
        dat_writer_.WriteDtToDat();
        dat_writer_.WriteTrafficLightsToDat(dynamic_signals_);
        dat_writer_.WriteStoryBoardStateChangesToDat(storyboard_state_changes_);
        dat_writer_.WriteObjectStatesToDat(objectState_);
        dat_writer_.SetTimestampWritten(false);
    }
}

int ScenarioGateway::RecordToFile(std::string filename, std::string odr_filename, std::string model_filename, std::string git_rev)
{
    if (filename.empty())
    {
        LOG_ERROR("Filename is empty");
        return -1;
    }

    return dat_writer_.Init(filename, odr_filename, model_filename, git_rev);
}
