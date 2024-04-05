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
#endif

using namespace scenarioengine;

ObjectState::ObjectState()
{
    state_.info.id = -1;
}

ObjectState::ObjectState(int                    id,
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
                         roadmanager::Position* pos)
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
    state_.info.wheel_angle      = wheel_angle;
    state_.info.wheel_rot        = wheel_rot;
    state_.info.rear_axle_z_pos  = rear_axle_z_pos;
    state_.info.front_axle_x_pos = front_axle_x_pos;
    state_.info.front_axle_z_pos = front_axle_z_pos;
    state_.info.boundingbox      = boundingbox;
    state_.info.scaleMode        = scaleMode;
    state_.info.visibilityMask   = visibilityMask;

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
    state_.info.wheel_angle     = wheel_angle;
    state_.info.wheel_rot       = wheel_rot;
    state_.info.rear_axle_z_pos = rear_axle_z_pos;
    state_.info.boundingbox     = boundingbox;
    state_.info.scaleMode       = scaleMode;
    state_.info.visibilityMask  = visibilityMask;

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
                         int            roadId,
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
    state_.info.wheel_angle     = wheel_angle;
    state_.info.wheel_rot       = wheel_rot;
    state_.info.rear_axle_z_pos = rear_axle_z_pos;
    state_.info.boundingbox     = boundingbox;
    state_.info.scaleMode       = scaleMode;
    state_.info.visibilityMask  = visibilityMask;

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
                         int            roadId,
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
    state_.info.wheel_angle     = wheel_angle;
    state_.info.wheel_rot       = wheel_rot;
    state_.info.rear_axle_z_pos = rear_axle_z_pos;
    state_.info.boundingbox     = boundingbox;
    state_.info.scaleMode       = scaleMode;
    state_.info.visibilityMask  = visibilityMask;

    dirty_ = Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
             Object::DirtyBit::WHEEL_ROTATION;
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
        state_.info.wheel_angle,
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
    obj_state->state_.info.wheel_angle    = wheel_angle;
    obj_state->state_.info.wheel_rot      = wheel_rot;
    obj_state->state_.info.visibilityMask = visibilityMask;

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
                                  roadmanager::Position* pos)
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
                                    pos);

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
                                  int            roadId,
                                  int            laneId,
                                  double         laneOffset,
                                  double         s)
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
                                  double         rear_axle_z_pos,
                                  double         wheel_rot,
                                  int            roadId,
                                  double         lateralOffset,
                                  double         s)
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

    obj_state->state_.info.wheel_angle = wheelAngle;
    obj_state->dirty_ |= Object::DirtyBit::WHEEL_ANGLE;

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

    obj_state->state_.info.wheel_rot = wheelRotation;
    obj_state->dirty_ |= Object::DirtyBit::WHEEL_ROTATION;

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

int ScenarioGateway::updateObjectFrictionCoefficients(int id, double friction[4])
{
    ObjectState* obj_state = getObjectStatePtrById(id);

    if (obj_state == nullptr)
    {
        LOG_ONCE("Can't set friction coefficients for object %d yet. Please register object using reportObject() first.", id);
        return -1;
    }

    for (int i = 0; i < 4; i++)
    {
        obj_state->state_.info.friction[i] = friction[i];
    }
    obj_state->dirty_ |= Object::DirtyBit::FRICTION;

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

void ScenarioGateway::WriteStatesToFile()
{
    if (data_file_.is_open())
    {
        // Write status to file - for later replay
        for (size_t i = 0; i < objectState_.size(); i++)
        {
            struct ObjectStateStructDat datState;

            datState.info.boundingbox = objectState_[i]->state_.info.boundingbox;
            datState.info.ctrl_type   = objectState_[i]->state_.info.ctrl_type;
            datState.info.ctrl_type   = objectState_[i]->state_.info.ctrl_type;
            datState.info.id          = objectState_[i]->state_.info.id;
            datState.info.model_id    = objectState_[i]->state_.info.model_id;
            memcpy(datState.info.name, objectState_[i]->state_.info.name, sizeof(datState.info.name));
            datState.info.obj_category   = objectState_[i]->state_.info.obj_category;
            datState.info.obj_type       = objectState_[i]->state_.info.ctrl_type;
            datState.info.scaleMode      = objectState_[i]->state_.info.scaleMode;
            datState.info.speed          = static_cast<float>(objectState_[i]->state_.info.speed);
            datState.info.timeStamp      = static_cast<float>(objectState_[i]->state_.info.timeStamp);
            datState.info.visibilityMask = objectState_[i]->state_.info.visibilityMask;
            datState.info.wheel_angle    = static_cast<float>(objectState_[i]->state_.info.wheel_angle);
            datState.info.wheel_rot      = static_cast<float>(objectState_[i]->state_.info.wheel_rot);

            datState.pos.x      = static_cast<float>(objectState_[i]->state_.pos.GetX());
            datState.pos.y      = static_cast<float>(objectState_[i]->state_.pos.GetY());
            datState.pos.z      = static_cast<float>(objectState_[i]->state_.pos.GetZ());
            datState.pos.h      = static_cast<float>(objectState_[i]->state_.pos.GetH());
            datState.pos.p      = static_cast<float>(objectState_[i]->state_.pos.GetP());
            datState.pos.r      = static_cast<float>(objectState_[i]->state_.pos.GetR());
            datState.pos.roadId = objectState_[i]->state_.pos.GetTrackId();
            datState.pos.laneId = objectState_[i]->state_.pos.GetLaneId();
            datState.pos.offset = static_cast<float>(objectState_[i]->state_.pos.GetOffset());
            datState.pos.t      = static_cast<float>(objectState_[i]->state_.pos.GetT());
            datState.pos.s      = static_cast<float>(objectState_[i]->state_.pos.GetS());
            data_file_.write(reinterpret_cast<char*>(&datState), sizeof(datState));
        }
    }
}

int ScenarioGateway::RecordToFile(std::string filename, std::string odr_filename, std::string model_filename)
{
    if (!filename.empty())
    {
        data_file_.open(filename, std::ofstream::binary);
        if (data_file_.fail())
        {
            LOG("Cannot open file: %s", filename.c_str());
            return -1;
        }
        DatHeader header;
        header.version = DAT_FILE_FORMAT_VERSION;
        StrCopy(header.odr_filename, odr_filename.c_str(), MIN(odr_filename.length() + 1, DAT_FILENAME_SIZE));
        StrCopy(header.model_filename, model_filename.c_str(), MIN(model_filename.length() + 1, DAT_FILENAME_SIZE));

        data_file_.write(reinterpret_cast<char*>(&header), sizeof(header));
    }

    return 0;
}
