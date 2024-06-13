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

#include <random>
#include "Entities.hpp"
#include "Controller.hpp"
#include "Storyboard.hpp"

using namespace scenarioengine;
using namespace roadmanager;

#define ELEVATION_DIFF_THRESHOLD 2.5

Object::Object(Type type)
    : type_(type),
      id_(0),
      speed_(0),
      wheel_angle_(0),
      wheel_rot_(0),
      model3d_(""),
      ghost_trail_s_(0),
      trail_follow_index_(0),
      odometer_(0),
      end_of_road_timestamp_(0.0),
      off_road_timestamp_(0.0),
      stand_still_timestamp_(0),
      reset_(0),
      headstart_time_(0),
      ghost_(0),
      ghost_Ego_(0),
      visibilityMask_(0xff),
      isGhost_(false),
      junctionSelectorStrategy_(Junction::JunctionStrategyType::RANDOM),
      nextJunctionSelectorAngle_(0.0),
      scaleMode_(EntityScaleMode::NONE),
      dirty_(0),
      is_active_(false)
{
    sensor_pos_[0] = 0;
    sensor_pos_[1] = 0;
    sensor_pos_[2] = 0;

    state_old.pos_x  = 0;
    state_old.pos_y  = 0;
    state_old.vel_x  = 0;
    state_old.vel_y  = 0;
    state_old.h      = 0;
    state_old.h_rate = 0;

    trail_closest_pos_ = {0, 0, 0, 0, 0, 0, false};

    // initialize override vector
    for (int i = 0; i < OVERRIDE_NR_TYPES; i++)
    {
        overrideActionList[i].type   = static_cast<OverrideType>(i);
        overrideActionList[i].value  = 0.0;
        overrideActionList[i].active = false;
    }

    boundingbox_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    if (junctionSelectorStrategy_ == Junction::JunctionStrategyType::RANDOM)
    {
        SetJunctionSelectorAngleRandom();
    }

    front_axle_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    rear_axle_  = {0.0, 0.0, 0.0, 0.0, 0.0};
}

void Object::SetEndOfRoad(bool state, double time)
{
    if (state == true)
    {
        end_of_road_timestamp_ = time;
    }
    else
    {
        end_of_road_timestamp_ = 0.0;
    }
}

void Object::SetOffRoad(bool state, double time)
{
    if (state == true)
    {
        off_road_timestamp_ = time;
    }
    else
    {
        off_road_timestamp_ = 0.0;
    }
}

void Object::SetStandStill(bool state, double time)
{
    if (state == true)
    {
        stand_still_timestamp_ = time;
    }
    else
    {
        stand_still_timestamp_ = 0.0;
    }
}

Position::ReturnCode Object::MoveAlongS(double ds, bool actualDistance)
{
    return pos_.MoveAlongS(ds, 0.0, GetJunctionSelectorAngle(), actualDistance, Position::MoveDirectionMode::HEADING_DIRECTION, true);
}

void Object::AssignController(Controller* controller)
{
    // if already assigned, first remove it from list of assigned controllers
    controllers_.erase(std::remove(controllers_.begin(), controllers_.end(), controller), controllers_.end());

    // add assigned controller to the end of list, which indicates the last assigned controller
    controllers_.push_back(controller);
}

void Object::UnassignController(Controller* controller)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl == controller)
        {
            ctrl->UnlinkObject();
            ctrl->Deactivate();
            break;
        }
    }

    controllers_.erase(std::remove(controllers_.begin(), controllers_.end(), controller), controllers_.end());
}

void Object::UnassignControllers()
{
    for (auto ctrl : controllers_)
    {
        ctrl->Deactivate();
    }
    controllers_.clear();
}

bool Object::IsControllerActiveOnDomains(unsigned int domainMask, Controller::Type type)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl->IsActiveOnDomains(domainMask))
        {
            if (type == Controller::Type::CONTROLLER_TYPE_UNDEFINED || ctrl->GetType() == ctrl->GetType())
            {
                return true;
            }
        }
    }

    return false;
}

bool Object::IsControllerActiveOnAnyOfDomains(unsigned int domainMask, Controller::Type type)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl->IsActiveOnAnyOfDomains(domainMask))
        {
            if (type == Controller::Type::CONTROLLER_TYPE_UNDEFINED || ctrl->GetType() == ctrl->GetType())
            {
                return true;
            }
        }
    }

    return false;
}

bool Object::IsControllerModeOnDomains(ControlOperationMode mode, unsigned int domainMask, Controller::Type type)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl->IsActiveOnDomains(domainMask))
        {
            if (ctrl->GetMode() == mode && (type == Controller::Type::CONTROLLER_TYPE_UNDEFINED || ctrl->GetType() == ctrl->GetType()))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    return false;
}

bool Object::IsControllerModeOnAnyOfDomains(ControlOperationMode mode, unsigned int domainMask, Controller::Type type)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl->IsActiveOnAnyOfDomains(domainMask))
        {
            if (ctrl->GetMode() == mode && (type == Controller::Type::CONTROLLER_TYPE_UNDEFINED || ctrl->GetType() == ctrl->GetType()))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    return false;
}

scenarioengine::Controller* scenarioengine::Object::GetAssignedControllerOftype(Controller::Type type)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl->GetType() == type)
        {
            return ctrl;
        }
    }

    return nullptr;
}

bool scenarioengine::Object::IsAnyAssignedControllerOfType(Controller::Type type)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl->GetType() == type)
        {
            return true;
        }
    }

    return false;
}

bool Object::IsAnyActiveControllerOfType(Controller::Type type)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl->IsActive() && ctrl->GetType() == type)
        {
            return true;
        }
    }

    return false;
}

scenarioengine::Controller* Object::GetControllerActiveOnDomain(ControlDomains domain)
{
    for (auto ctrl : controllers_)
    {
        if (ctrl->IsActiveOnDomains(static_cast<unsigned int>(domain)))
        {
            return ctrl;
        }
    }

    return nullptr;
}

scenarioengine::Controller::Type Object::GetControllerTypeActiveOnDomain(ControlDomains domain)
{
    scenarioengine::Controller* ctrl = GetControllerActiveOnDomain(domain);

    if (ctrl != nullptr)
    {
        return static_cast<Controller::Type>(ctrl->GetType());
    }
    else if (IsGhost())
    {
        return Controller::Type::GHOST_RESERVED_TYPE;
    }

    return Controller::Type::CONTROLLER_TYPE_DEFAULT;
}

scenarioengine::Controller* Object::GetController(std::string name)
{
    for (auto ctrl_tmp : controllers_)
    {
        if (ctrl_tmp->GetName() == name)
        {
            return ctrl_tmp;
        }
    }

    return nullptr;
}

void Object::SetVisibilityMask(int mask)
{
    visibilityMask_ = mask;
    SetDirtyBits(dirty_ | DirtyBit::VISIBILITY);
}

void Object::SetVel(double x_vel, double y_vel, double z_vel)
{
    pos_.SetVel(x_vel, y_vel, z_vel);
    SetDirtyBits(dirty_ | DirtyBit::VELOCITY);
}

void Object::SetAcc(double x_acc, double y_acc, double z_acc)
{
    pos_.SetAcc(x_acc, y_acc, z_acc);
    SetDirtyBits(dirty_ | DirtyBit::ACCELERATION);
}

void Object::SetAngularVel(double h_vel, double p_vel, double r_vel)
{
    pos_.SetAngularVel(h_vel, p_vel, r_vel);
    SetDirtyBits(dirty_ | DirtyBit::ANGULAR_RATE);
}

void Object::SetAngularAcc(double h_acc, double p_acc, double r_acc)
{
    pos_.SetAngularAcc(h_acc, p_acc, r_acc);
    SetDirtyBits(dirty_ | DirtyBit::ANGULAR_ACC);
}

void Object::SetJunctionSelectorAngle(double angle)
{
    if (std::isnan(angle))
    {
        nextJunctionSelectorAngle_ = angle;
    }
    else
    {
        nextJunctionSelectorAngle_ = GetAngleInInterval2PI(angle);
    }
}

void Object::SetJunctionSelectorAngleRandom()
{
    nextJunctionSelectorAngle_ = 2 * M_PI * SE_Env::Inst().GetRand().GetReal();
}

bool Object::CollisionAndRelativeDistLatLong(Object* target, double* distLat, double* distLong)
{
    // Apply method Separating Axis Theorem (SAT)
    // http://www.euclideanspace.com/threed/games/examples/cars/collisions/
    // https://www.sevenson.com.au/actionscript/sat/

    // Idea:
    // For each side of the bounding boxes:
    //   The normal of that edge will be the projection axis
    //   Project all points of the two bounding boxes onto that axis
    //   If we find ONE side with a gap between point clusters from BB1 and BB2,
    //   it's enough to conclude they are not overlapping/colliding
    //
    // Optimization: Since the bounding boxes are boxes with parallel
    // sides, we only need to check half of the sides

    if (target == 0)
    {
        return false;
    }

    Object* obj0 = this;
    Object* obj1 = target;
    bool    gap  = false;
    if (distLong != nullptr)
    {
        *distLong = 0.0;
    }
    if (distLat != nullptr)
    {
        *distLat = 0.0;
    }

    // First do a rough check to rule out potential overlap/collision
    // Compare radial/euclidean distance with sum of the diagonal dimension of the bounding boxes
    double x = 0, y = 0;
    double dist           = fabs(this->pos_.getRelativeDistance(target->pos_.GetX(), target->pos_.GetY(), x, y));
    double max_length     = this->boundingbox_.dimensions_.length_ + target->boundingbox_.dimensions_.length_;
    double max_width      = this->boundingbox_.dimensions_.width_ + target->boundingbox_.dimensions_.width_;
    double dist_threshold = sqrt(max_length * max_length + max_width * max_width);

    if (dist > dist_threshold && distLong == nullptr && distLat == nullptr)
    {
        return false;
    }

    // Also do a Z sanity check, to rule out on different road elevations
    if (fabs(obj0->pos_.GetZ() - obj1->pos_.GetZ()) > ELEVATION_DIFF_THRESHOLD && distLong == nullptr && distLat == nullptr)
    {
        return false;
    }

    for (int i = 0; i < 2; i++)  // for each of the two BBs
    {
        if (i == 1)
        {
            // swap order, now check all edges of target object
            obj0 = target;
            obj1 = this;
        }

        double n0[2] = {0.0, 0.0};
        for (int j = 0; j < 2; j++)  // for longitudinal and lateral sides
        {
            if (j == 0)
            {
                // Normal for longitudinal side (sides) points along lateral direction
                n0[0] = 0.0;
                n0[1] = 1.0;
            }
            else
            {
                // Normal for lateral side (front/rear) points along lateral direction
                n0[0] = 1.0;
                n0[1] = 0.0;
            }

            // Rotate the normal/projection axis to align with bounding box/vehicle
            double n1[2] = {0.0, 0.0};
            RotateVec2D(n0[0], n0[1], obj0->pos_.GetH(), n1[0], n1[1]);

            // Now, project each point of each BB onto the rotated normal
            // And register min and max for point cluster of each BB
            double min[2] = {0.0, 0.0}, max[2] = {0.0, 0.0};
            for (int k = 0; k < 2; k++)
            {
                Object* obj = (k == 0 ? obj0 : obj1);

                // Specify bounding box corner vertices, starting at first quadrant
                double vertices[4][2] = {
                    {static_cast<double>(obj->boundingbox_.center_.x_) + static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                     static_cast<double>(obj->boundingbox_.center_.y_) + static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                    {static_cast<double>(obj->boundingbox_.center_.x_) - static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                     static_cast<double>(obj->boundingbox_.center_.y_) + static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                    {static_cast<double>(obj->boundingbox_.center_.x_) - static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                     static_cast<double>(obj->boundingbox_.center_.y_) - static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                    {static_cast<double>(obj->boundingbox_.center_.x_) + static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                     static_cast<double>(obj->boundingbox_.center_.y_) - static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0}};

                for (int l = 0; l < 4; l++)
                {
                    double point_to_project[2];

                    // Align projection points to object heading
                    RotateVec2D(vertices[l][0], vertices[l][1], obj->pos_.GetH(), point_to_project[0], point_to_project[1]);

                    double dot_p = GetDotProduct2D(obj->pos_.GetX() + point_to_project[0], obj->pos_.GetY() + point_to_project[1], n1[0], n1[1]);

                    if (l == 0)
                    {
                        min[k] = max[k] = dot_p;
                    }
                    else
                    {
                        min[k] = MIN(dot_p, min[k]);
                        max[k] = MAX(dot_p, max[k]);
                    }
                }
            }

            if (((min[0] < min[1] - SMALL_NUMBER) && (max[0] < min[1] - SMALL_NUMBER)) ||
                ((max[0] > max[1] + SMALL_NUMBER) && (min[0] > max[1] + SMALL_NUMBER)))
            {
                // gap found
                gap = true;
                if (distLong == nullptr && distLat == nullptr)
                {
                    return !gap;
                }
                else
                {
                    // measure gap relative pivot vehicle
                    if (i == 0)
                    {
                        if (min[0] < min[1] - SMALL_NUMBER && max[0] < min[1] - SMALL_NUMBER)
                        {
                            if (j == 0)
                            {
                                if (distLat)
                                    *distLat = min[1] - max[0];
                            }
                            else
                            {
                                if (distLong)
                                    *distLong = min[1] - max[0];
                            }
                        }
                        else
                        {
                            if (j == 0)
                            {
                                if (distLat)
                                    *distLat = -(min[0] - max[1]);
                            }
                            else
                            {
                                if (distLong)
                                    *distLong = -(min[0] - max[1]);
                            }
                        }
                    }
                }
            }
        }
    }

    return !gap;
}

double Object::PointCollision(double x, double y)
{
    // Apply method Separating Axis Theorem (SAT)
    // http://www.euclideanspace.com/threed/games/examples/cars/collisions/
    // https://www.sevenson.com.au/actionscript/sat/

    // Idea:
    // For each side of the bounding box:
    //   The normal of that edge will be the projection axis
    //   Project the point and all axis of the bounding boxe onto that axis
    //   If we find ONE side with a gap between point clusters from BB1 and BB2,
    //   it's enough to conclude they are not overlapping/colliding
    //
    // Optimization: Since the bounding boxes are boxes with parallel
    // sides, we only need to check half of the sides

    Object* obj0 = this;

    double n0[2] = {0.0, 0.0};
    for (int j = 0; j < 2; j++)  // for longitudinal and lateral sides
    {
        if (j == 0)
        {
            // Normal for longitudinal side (sides) points along lateral side
            n0[0] = 0.0;
            n0[1] = 1.0;
        }
        else
        {
            // Normal for lateral side (front/rear) points along lateral side
            n0[0] = 1.0;
            n0[1] = 0.0;
        }

        // Rotate the normal/projection axis to align with bounding box/vehicle
        double n1[2] = {0.0, 0.0};
        RotateVec2D(n0[0], n0[1], obj0->pos_.GetH(), n1[0], n1[1]);

        // Now, project each point of each BB onto the rotated normal
        // And register min and max for point cluster of each BB
        double min[2] = {0.0, 0.0}, max[2] = {0.0, 0.0};

        // Specify bounding box corner vertices, starting at first quadrant
        double vertices[4][2] = {
            {static_cast<double>(obj0->boundingbox_.center_.x_) + static_cast<double>(obj0->boundingbox_.dimensions_.length_) / 2.0,
             static_cast<double>(obj0->boundingbox_.center_.y_) + static_cast<double>(obj0->boundingbox_.dimensions_.width_) / 2.0},
            {static_cast<double>(obj0->boundingbox_.center_.x_) - static_cast<double>(obj0->boundingbox_.dimensions_.length_) / 2.0,
             static_cast<double>(obj0->boundingbox_.center_.y_) + static_cast<double>(obj0->boundingbox_.dimensions_.width_) / 2.0},
            {static_cast<double>(obj0->boundingbox_.center_.x_) - static_cast<double>(obj0->boundingbox_.dimensions_.length_) / 2.0,
             static_cast<double>(obj0->boundingbox_.center_.y_) - static_cast<double>(obj0->boundingbox_.dimensions_.width_) / 2.0},
            {static_cast<double>(obj0->boundingbox_.center_.x_) + static_cast<double>(obj0->boundingbox_.dimensions_.length_) / 2.0,
             static_cast<double>(obj0->boundingbox_.center_.y_) - static_cast<double>(obj0->boundingbox_.dimensions_.width_) / 2.0}};

        for (int l = 0; l < 4; l++)
        {
            double point_to_project[2];

            // Align projection points to object heading
            RotateVec2D(vertices[l][0], vertices[l][1], obj0->pos_.GetH(), point_to_project[0], point_to_project[1]);

            double dot_p = GetDotProduct2D(obj0->pos_.GetX() + point_to_project[0], obj0->pos_.GetY() + point_to_project[1], n1[0], n1[1]);

            if (l == 0)
            {
                min[0] = max[0] = dot_p;
            }
            else
            {
                min[0] = MIN(dot_p, min[0]);
                max[0] = MAX(dot_p, max[0]);
            }
        }

        double dot_p = GetDotProduct2D(x, y, n1[0], n1[1]);

        if (((min[0] < dot_p - SMALL_NUMBER) && (max[0] < dot_p - SMALL_NUMBER)) ||
            ((max[0] > dot_p + SMALL_NUMBER) && (min[0] > dot_p + SMALL_NUMBER)))
        {
            // gap found - no collision
            return false;
        }
    }

    return true;
}

double Object::FreeSpaceDistance(Object* target, double* latDist, double* longDist)
{
    double minDist = LARGE_NUMBER;

    if (longDist != nullptr)
    {
        *longDist = LARGE_NUMBER;
    }
    if (latDist != nullptr)
    {
        *latDist = LARGE_NUMBER;
    }

    if (target == 0)
    {
        return minDist;
    }

    if (CollisionAndRelativeDistLatLong(target, latDist, longDist))
    {
        return 0.0;
    }

    // OK, they are not overlapping. Now find the distance.
    // Strategy: Brute force check all vertices of one bounding box
    // against all sides of the other bounding box - then switch to
    // check vertices of the other bounding box against the sides
    // of the first bounding box.

    double vertices[2][4][2];

    for (int i = 0; i < 2; i++)  // for each of the two BBs
    {
        Object* obj = (i == 0 ? this : target);

        // Specify bounding box corner vertices, starting at first quadrant
        double vtmp[4][2] = {{static_cast<double>(obj->boundingbox_.center_.x_) + static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                              static_cast<double>(obj->boundingbox_.center_.y_) + static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                             {static_cast<double>(obj->boundingbox_.center_.x_) - static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                              static_cast<double>(obj->boundingbox_.center_.y_) + static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                             {static_cast<double>(obj->boundingbox_.center_.x_) - static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                              static_cast<double>(obj->boundingbox_.center_.y_) - static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                             {static_cast<double>(obj->boundingbox_.center_.x_) + static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                              static_cast<double>(obj->boundingbox_.center_.y_) - static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0}};

        for (int j = 0; j < 4; j++)  // for all vertices
        {
            // Align points to object heading and position
            RotateVec2D(vtmp[j][0], vtmp[j][1], obj->pos_.GetH(), vertices[i][j][0], vertices[i][j][1]);
            vertices[i][j][0] += obj->pos_.GetX();
            vertices[i][j][1] += obj->pos_.GetY();
        }
    }

    for (int i = 0; i < 2; i++)  // for each of the two BBs
    {
        int vindex = (i == 0 ? 0 : 1);

        for (int j = 0; j < 4; j++)  // for all vertices
        {
            double point[2] = {vertices[vindex][j][0], vertices[vindex][j][1]};

            for (int k = 0; k < 4; k++)  // for all sides/edges in the other bounding box
            {
                double edge[2][2];
                edge[0][0] = vertices[(vindex + 1) % 2][k][0];
                edge[0][1] = vertices[(vindex + 1) % 2][k][1];
                edge[1][0] = vertices[(vindex + 1) % 2][(k + 1) % 4][0];
                edge[1][1] = vertices[(vindex + 1) % 2][(k + 1) % 4][1];

                double xProj   = 0;
                double yProj   = 0;
                double tmpDist = DistanceFromPointToEdge2D(point[0], point[1], edge[0][0], edge[0][1], edge[1][0], edge[1][1], &xProj, &yProj);

                if (tmpDist < minDist)
                {
                    minDist = tmpDist;
                }
            }
        }
    }

    return minDist;
}

double Object::FreeSpaceDistancePoint(double x, double y, double* latDist, double* longDist)
{
    double minDist = LARGE_NUMBER;
    *latDist       = LARGE_NUMBER;
    *longDist      = LARGE_NUMBER;

    if (PointCollision(x, y))
    {
        *latDist  = 0.0;
        *longDist = 0.0;
        return 0.0;
    }

    // OK, they are not overlapping. Now find the distance.
    // Strategy: Brute force check point against all sides
    // of the bounding box

    Object* obj = this;

    double vertices[4][2];

    // Specify bounding box corner vertices, starting at first quadrant
    double vtmp[4][2] = {{static_cast<double>(obj->boundingbox_.center_.x_) + static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                          static_cast<double>(obj->boundingbox_.center_.y_) + static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                         {static_cast<double>(obj->boundingbox_.center_.x_) - static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                          static_cast<double>(obj->boundingbox_.center_.y_) + static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                         {static_cast<double>(obj->boundingbox_.center_.x_) - static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                          static_cast<double>(obj->boundingbox_.center_.y_) - static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                         {static_cast<double>(obj->boundingbox_.center_.x_) + static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                          static_cast<double>(obj->boundingbox_.center_.y_) - static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0}};

    for (int j = 0; j < 4; j++)  // for all vertices
    {
        // Align points to object heading and position
        RotateVec2D(vtmp[j][0], vtmp[j][1], obj->pos_.GetH(), vertices[j][0], vertices[j][1]);
        vertices[j][0] += obj->pos_.GetX();
        vertices[j][1] += obj->pos_.GetY();
    }

    double point[2] = {x, y};

    for (int k = 0; k < 4; k++)  // for all sides/edges of the bounding box
    {
        double edge[2][2];
        edge[0][0] = vertices[k][0];
        edge[0][1] = vertices[k][1];
        edge[1][0] = vertices[(k + 1) % 4][0];
        edge[1][1] = vertices[(k + 1) % 4][1];

        double xProj   = 0;
        double yProj   = 0;
        double tmpDist = DistanceFromPointToEdge2D(point[0], point[1], edge[0][0], edge[0][1], edge[1][0], edge[1][1], &xProj, &yProj);

        if (tmpDist < minDist)
        {
            minDist = tmpDist;

            if (latDist && longDist)
            {
                // Calculate x, y components of the distance in vehicle reference system
                // y points left in vehicle ref system, x forward
                RotateVec2D(point[0] - xProj, point[1] - yProj, -this->pos_.GetH(), *longDist, *latDist);
            }
        }
    }

    return minDist;
}

int Object::FreeSpaceDistancePointRoadLane(double x, double y, double* latDist, double* longDist, CoordinateSystem cs)
{
    *latDist  = LARGE_NUMBER;
    *longDist = LARGE_NUMBER;

    if (cs != CoordinateSystem::CS_LANE && cs != CoordinateSystem::CS_ROAD)
    {
        LOG("Unexpected coordinateSystem (%d). %d or %d expected.", CoordinateSystem::CS_LANE, CoordinateSystem::CS_ROAD);
        return -1;
    }

    if (cs == CoordinateSystem::CS_LANE)
    {
        LOG("freespace LANE coordinateSystem not supported yet, falling back to freespace ROAD");
        cs = CoordinateSystem::CS_ROAD;
    }

    // Specify bounding box corner vertices, starting at first quadrant
    double vtmp[4][2] = {{static_cast<double>(boundingbox_.center_.x_) + static_cast<double>(boundingbox_.dimensions_.length_) / 2.0,
                          static_cast<double>(boundingbox_.center_.y_) + static_cast<double>(boundingbox_.dimensions_.width_) / 2.0},
                         {static_cast<double>(boundingbox_.center_.x_) - static_cast<double>(boundingbox_.dimensions_.length_) / 2.0,
                          static_cast<double>(boundingbox_.center_.y_) + static_cast<double>(boundingbox_.dimensions_.width_) / 2.0},
                         {static_cast<double>(boundingbox_.center_.x_) - static_cast<double>(boundingbox_.dimensions_.length_) / 2.0,
                          static_cast<double>(boundingbox_.center_.y_) - static_cast<double>(boundingbox_.dimensions_.width_) / 2.0},
                         {static_cast<double>(boundingbox_.center_.x_) + static_cast<double>(boundingbox_.dimensions_.length_) / 2.0,
                          static_cast<double>(boundingbox_.center_.y_) - static_cast<double>(boundingbox_.dimensions_.width_) / 2.0}};

    // Align points to object heading and position
    double vertices[4][3];
    for (int i = 0; i < 4; i++)
    {
        RotateVec2D(vtmp[i][0], vtmp[i][1], pos_.GetH(), vertices[i][0], vertices[i][1]);
        vertices[i][0] += pos_.GetX();
        vertices[i][1] += pos_.GetY();
        vertices[i][2] = pos_.GetH();
    }

    // Map XY point to road coordinates, but consider only roads reachable from point
    Position pointPos = pos_;
    pointPos.SetRoute(0);  // don't mess with the route of the original position object
    if (static_cast<int>(pointPos.XYZ2TrackPos(x, y, 0, true)) < 0)
    {
        return -1;
    }

    // Find long and lat max values
    Position     pos[4];
    double       maxDS = 0.0;
    double       minDS = LARGE_NUMBER;
    double       maxDT = 0.0;
    double       minDT = LARGE_NUMBER;
    PositionDiff posDiff;
    for (int j = 0; j < 4; j++)
    {
        pos[j] = pos_;
        pos[j].SetRoute(0);  // don't mess with the route of the original position object
        // Map bounding box points to road coordinates, consider only roads reachable from current position
        if (static_cast<int>(pos[j].XYZ2TrackPos(vertices[j][0], vertices[j][1], 0, roadmanager::Position::PosMode::UNDEFINED, true)) < 0)
        {
            return -1;
        }

        if (pos[j].Delta(&pointPos, posDiff) == false)
        {
            return -1;
        }

        if (j == 0 || fabs(posDiff.ds) < fabs(minDS))
        {
            minDS = posDiff.ds;
        }

        if (j == 0 || fabs(posDiff.dt) < fabs(minDT))
        {
            minDT = posDiff.dt;
        }

        if (j == 0 || fabs(posDiff.ds) > fabs(maxDS))
        {
            maxDS = posDiff.ds;
        }

        if (j == 0 || fabs(posDiff.dt) > fabs(maxDT))
        {
            maxDT = posDiff.dt;
        }
    }

    *longDist = minDS;
    *latDist  = minDT;

    // Check for overlap
    if (SIGN(minDS) != SIGN(maxDS))
    {
        // Overlap
        *longDist = 0.0;
    }
    if (SIGN(minDT) != SIGN(maxDT))
    {
        // Overlap
        *latDist = 0.0;
    }

    return 0;
}

int Object::FreeSpaceDistanceObjectRoadLane(Object* target, PositionDiff* posDiff, CoordinateSystem cs)
{
    if (posDiff == nullptr)
    {
        LOG("FreeSpaceDistanceObjectRoadLane: PositionDiff is NULL");
        return -1;
    }

    posDiff->dLaneId  = LARGE_NUMBER_INT;
    posDiff->dt       = LARGE_NUMBER;
    posDiff->ds       = LARGE_NUMBER;
    posDiff->dx       = LARGE_NUMBER;
    posDiff->dy       = LARGE_NUMBER;
    posDiff->dOppLane = false;

    // First some checks
    if (target == 0)
    {
        return -1;
    }

    if (cs != CoordinateSystem::CS_LANE && cs != CoordinateSystem::CS_ROAD)
    {
        LOG("Unexpected coordinateSystem (%d). %d or %d expected.", CoordinateSystem::CS_LANE, CoordinateSystem::CS_ROAD);
        return -1;
    }

    if (cs == CoordinateSystem::CS_LANE)
    {
        LOG("freespace LANE coordinateSystem not supported yet, falling back to freespace ROAD");
        cs = CoordinateSystem::CS_ROAD;
    }

    if (Collision(target))
    {
        posDiff->dLaneId = 0;
        posDiff->dt      = 0;
        posDiff->ds      = 0;
        posDiff->dx      = 0;
        posDiff->dy      = 0;
        return 0;
    }

    // OK, they are not overlapping (colliding). Now find the distance.
    // Strategy: Brute force check all vertices of one bounding box
    // against all sides of the other bounding box - then switch to
    // check vertices of the other bounding box against the sides
    // of the first bounding box.

    double   vertices[2][4][3] = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
    Position pos[2][4];

    for (int i = 0; i < 2; i++)  // for each of the two BBs
    {
        Object* obj = (i == 0 ? this : target);

        // Specify bounding box corner vertices, starting at first quadrant
        double vtmp[4][2] = {{static_cast<double>(obj->boundingbox_.center_.x_) + static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                              static_cast<double>(obj->boundingbox_.center_.y_) + static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                             {static_cast<double>(obj->boundingbox_.center_.x_) - static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                              static_cast<double>(obj->boundingbox_.center_.y_) + static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                             {static_cast<double>(obj->boundingbox_.center_.x_) - static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                              static_cast<double>(obj->boundingbox_.center_.y_) - static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0},
                             {static_cast<double>(obj->boundingbox_.center_.x_) + static_cast<double>(obj->boundingbox_.dimensions_.length_) / 2.0,
                              static_cast<double>(obj->boundingbox_.center_.y_) - static_cast<double>(obj->boundingbox_.dimensions_.width_) / 2.0}};

        for (int j = 0; j < 4; j++)  // for all vertices
        {
            // Align points to object heading and position
            RotateVec2D(vtmp[j][0], vtmp[j][1], obj->pos_.GetH(), vertices[i][j][0], vertices[i][j][1]);
            vertices[i][j][0] += obj->pos_.GetX();
            vertices[i][j][1] += obj->pos_.GetY();
            vertices[i][j][2] = obj->pos_.GetH();

            // Map XY points to road coordinates, but consider only roads reachable from point
            pos[i][j] = pos_;
            if (static_cast<int>(pos[i][j].XYZ2TrackPos(vertices[i][j][0], vertices[i][j][1], 0, roadmanager::Position::PosMode::UNDEFINED, true)) <
                0)
            {
                return -1;
            }
        }
    }

    // Find long and lat max values
    double maxDS = 0.0;
    double minDS = LARGE_NUMBER;
    double maxDT = 0.0;
    double minDT = LARGE_NUMBER;

    double ds[4][4];  // delta s between every vertex on first bb to every vertex on second bb
    double dt[4][4];  // delta t between every vertex on first bb to every vertex on second bb

    for (int i = 0; i < 4; i++)  // for each vertex of first BBs
    {
        for (int j = 0; j < 4; j++)  // for each vertex of second BBs
        {
            if (pos[0][i].Delta(&pos[1][j], *posDiff) == false)
            {
                return -1;
            }
            ds[i][j] = posDiff->ds;
            dt[i][j] = posDiff->dt;

            if ((i == 0 && j == 0) || fabs(posDiff->ds) < fabs(minDS))
            {
                minDS = posDiff->ds;
            }

            if ((i == 0 && j == 0) || fabs(posDiff->dt) < fabs(minDT))
            {
                minDT = posDiff->dt;
            }

            if ((i == 0 && j == 0) || fabs(posDiff->ds) > fabs(maxDS))
            {
                maxDS = posDiff->ds;
            }

            if ((i == 0 && j == 0) || fabs(posDiff->dt) > fabs(maxDT))
            {
                maxDT = posDiff->dt;
            }
        }
    }

    posDiff->ds = minDS;
    posDiff->dt = minDT;

    // Check for overlap
    for (int i = 0; i < 4; i++)  // for each ds
    {
        if (!((SIGN(ds[i][0]) == SIGN(ds[i][1])) && (SIGN(ds[i][0]) == SIGN(ds[i][2])) && (SIGN(ds[i][0]) == SIGN(ds[i][3]))))
        {
            // Overlap
            posDiff->ds = 0.0;
        }
        if (!((SIGN(dt[i][0]) == SIGN(dt[i][1])) && (SIGN(dt[i][0]) == SIGN(dt[i][2])) && (SIGN(dt[i][0]) == SIGN(dt[i][3]))))
        {
            // Overlap
            posDiff->dt = 0.0;
        }
    }

    return 0;
}

int Object::Distance(Object*                           target,
                     roadmanager::CoordinateSystem     cs,
                     roadmanager::RelativeDistanceType relDistType,
                     bool                              freeSpace,
                     double&                           dist,
                     double                            maxDist)
{
    (void)maxDist;
    if (freeSpace)
    {
        double latDist, longDist;

        if (cs == roadmanager::CoordinateSystem::CS_ENTITY || relDistType == RelativeDistanceType::REL_DIST_EUCLIDIAN ||
            relDistType == RelativeDistanceType::REL_DIST_CARTESIAN)
        {
            // Get Cartesian/Euclidian distance
            dist = FreeSpaceDistance(target, &latDist, &longDist);

            // sign indicates target being in front (+) or behind (-)
            dist *= SIGN(longDist);

            if (cs == roadmanager::CoordinateSystem::CS_ENTITY)
            {
                if (relDistType == roadmanager::RelativeDistanceType::REL_DIST_LATERAL)
                {
                    dist = latDist;
                }
                else if (relDistType == roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL)
                {
                    dist = longDist;
                }
            }
        }
        else if (cs == CoordinateSystem::CS_ROAD || cs == CoordinateSystem::CS_LANE)
        {
            roadmanager::PositionDiff pos_diff;
            if (FreeSpaceDistanceObjectRoadLane(target, &pos_diff, cs) != 0)
            {
                return -1;
            }
            else
            {
                if (relDistType == RelativeDistanceType::REL_DIST_LATERAL)
                {
                    dist = pos_diff.dt;
                }
                else if (relDistType == RelativeDistanceType::REL_DIST_LONGITUDINAL)
                {
                    dist = pos_diff.ds;
                }
                else
                {
                    LOG("Unexpected relativeDistanceType: %d", relDistType);
                    return -1;
                }
            }
        }
        else
        {
            LOG("Unhandled case: cs %d reDistType %d freeSpace %d\n", cs, relDistType, freeSpace);
            return -1;
        }
    }
    else  // not freeSpace
    {
        return pos_.Distance(&target->pos_, cs, relDistType, dist);
    }

    return 0;
}

int Object::Distance(double                            x,
                     double                            y,
                     roadmanager::CoordinateSystem     cs,
                     roadmanager::RelativeDistanceType relDistType,
                     bool                              freeSpace,
                     double&                           dist,
                     double                            maxDist)
{
    if (freeSpace)
    {
        double latDist, longDist;

        if (cs == roadmanager::CoordinateSystem::CS_ENTITY || relDistType == RelativeDistanceType::REL_DIST_EUCLIDIAN ||
            relDistType == RelativeDistanceType::REL_DIST_CARTESIAN)
        {
            // Get Cartesian/Euclidian distance
            dist = FreeSpaceDistancePoint(x, y, &latDist, &longDist);

            // sign indicates target being in front (+) or behind (-)
            dist *= SIGN(longDist);

            if (cs == roadmanager::CoordinateSystem::CS_ENTITY)
            {
                if (relDistType == roadmanager::RelativeDistanceType::REL_DIST_LATERAL)
                {
                    dist = latDist;
                }
                else if (relDistType == roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL)
                {
                    dist = longDist;
                }
            }
        }
        else if (cs == CoordinateSystem::CS_ROAD || cs == CoordinateSystem::CS_LANE)
        {
            if (FreeSpaceDistancePointRoadLane(x, y, &latDist, &longDist, cs) != 0)
            {
                return -1;
            }
            else
            {
                if (relDistType == RelativeDistanceType::REL_DIST_LATERAL)
                {
                    dist = latDist;
                }
                else if (relDistType == RelativeDistanceType::REL_DIST_LONGITUDINAL)
                {
                    dist = longDist;
                }
                else
                {
                    LOG("Unexpected relativeDistanceType: %d", relDistType);
                    return -1;
                }
            }
        }
        else
        {
            LOG("Unhandled case: cs %d reDistType %d freeSpace %d\n", cs, relDistType, freeSpace);
            return -1;
        }
    }
    else  // not freeSpace
    {
        return pos_.Distance(x, y, cs, relDistType, dist, maxDist);
    }

    return 0;
}

Object::OverlapType Object::OverlappingFront(Object* target, double tolerance)
{
    // Strategy:
    // Project vertices of target objects' bounding box
    // on the front line of own bounding box
    // All vertices inside: OVERLAP_INSIDE
    // Some vertices inside: OVERLAP_PARTLY
    // At least one vertex on each side of front line: OVERLAP_FULL

    // Own object front side of bounding box
    SE_Vector front_left(boundingbox_.center_.x_ + boundingbox_.dimensions_.length_ / 2.0f, boundingbox_.dimensions_.width_ / 2.0f);
    SE_Vector front_right(boundingbox_.center_.x_ + boundingbox_.dimensions_.length_ / 2.0f, -boundingbox_.dimensions_.width_ / 2.0f);

    // Rotate and translate front line
    front_left  = front_left.Rotate(pos_.GetH());
    front_right = front_right.Rotate(pos_.GetH());
    front_left += SE_Vector(pos_.GetX(), pos_.GetY());
    front_right += SE_Vector(pos_.GetX(), pos_.GetY());

    // Specify target object bounding box corner vertices, starting at first quadrant going clock wise
    SE_Vector vertex[4] = {{target->boundingbox_.center_.x_ + target->boundingbox_.dimensions_.length_ / 2.0f,
                            target->boundingbox_.center_.y_ + target->boundingbox_.dimensions_.width_ / 2.0f},
                           {target->boundingbox_.center_.x_ - target->boundingbox_.dimensions_.length_ / 2.0f,
                            target->boundingbox_.center_.y_ + target->boundingbox_.dimensions_.width_ / 2.0f},
                           {target->boundingbox_.center_.x_ - target->boundingbox_.dimensions_.length_ / 2.0f,
                            target->boundingbox_.center_.y_ - target->boundingbox_.dimensions_.width_ / 2.0f},
                           {target->boundingbox_.center_.x_ + target->boundingbox_.dimensions_.length_ / 2.0f,
                            target->boundingbox_.center_.y_ - target->boundingbox_.dimensions_.width_ / 2.0f}};

    for (int i = 0; i < 4; i++)  // for all vertices
    {
        // Align points to targetect heading and position
        vertex[i] = vertex[i].Rotate(target->pos_.GetH());
        vertex[i] += SE_Vector(target->pos_.GetX(), target->pos_.GetY());
    }

    int outside_left_count  = 0;
    int outside_right_count = 0;
    int inside_count        = 0;
    for (int i = 0; i < 4; i++)
    {
        double projected_point[2];
        double s_norm = 0.0;

        ProjectPointOnLine2D(vertex[i].x(),
                             vertex[i].y(),
                             front_left.x(),
                             front_left.y(),
                             front_right.x(),
                             front_right.y(),
                             projected_point[0],
                             projected_point[1]);

        bool is_within = PointInBetweenVectorEndpoints(projected_point[0],
                                                       projected_point[1],
                                                       front_left.x(),
                                                       front_left.y(),
                                                       front_right.x(),
                                                       front_right.y(),
                                                       s_norm);

        if (is_within)
        {
            inside_count++;

            if (s_norm * static_cast<double>(boundingbox_.dimensions_.width_) < tolerance)  // s_norm is factor (0..1) along front line
            {
                outside_left_count++;
            }
            else if ((1 - s_norm) * static_cast<double>(boundingbox_.dimensions_.width_) < tolerance)
            {
                outside_right_count++;
            }
        }
        else
        {
            if (abs(s_norm) < tolerance)  // s_norm is actual distance from front line
            {
                inside_count++;
            }

            if (s_norm < 0)
            {
                outside_left_count++;
            }
            else
            {
                outside_right_count++;
            }
        }
    }

    if (inside_count == 4 && outside_left_count > 0 && outside_right_count > 0)
    {
        return OverlapType::INSIDE_AND_FULL;
    }
    else if (inside_count == 4)
    {
        return OverlapType::INSIDE;
    }
    else if (outside_left_count > 0 && outside_right_count > 0)
    {
        return OverlapType::FULL;
    }
    else if (inside_count > 0)
    {
        return OverlapType::PART;
    }

    return OverlapType::NONE;
}

int Entities::addObject(Object* obj, bool activate, int call_index)
{
    const int max_trailers = 100;
    if (call_index >= max_trailers)
    {
        LOG_AND_QUIT("Error: addObject max recursion reached (%d). Check scenario trailer config", max_trailers);
    }

    obj->id_ = getNewId();
    if (activate)
    {
        object_.push_back(obj);
    }
    else
    {
        object_pool_.push_back(obj);
    }

    obj->SetActive(activate);

    Vehicle* trailer_vehicle = static_cast<Vehicle*>(obj->TrailerVehicle());
    if (trailer_vehicle && trailer_vehicle != obj)
    {
        if (trailer_vehicle->name_.empty())
        {
            trailer_vehicle->name_ = obj->GetName() + "+";
        }

        // avoid adding trailers twice
        bool in_active_list  = std::find(object_.begin(), object_.end(), trailer_vehicle) != object_.end();
        bool in_passive_list = std::find(object_pool_.begin(), object_pool_.end(), trailer_vehicle) != object_pool_.end();

        if (in_active_list && !activate)
        {
            deactivateObject(trailer_vehicle, call_index);
        }
        else if (in_passive_list && activate)
        {
            activateObject(trailer_vehicle, call_index);
        }
        else if (!in_active_list && !in_passive_list)
        {
            addObject(trailer_vehicle, activate, call_index + 1);
        }
    }

    return obj->id_;
}

int Entities::activateObject(Object* obj, int call_index)
{
    const int max_trailers = 100;
    if (call_index >= max_trailers)
    {
        LOG_AND_QUIT("Error: activateObject max recursion reached (%d). Check scenario trailer config", max_trailers);
    }

    int n_active_objs = static_cast<int>(std::count(object_.begin(), object_.end(), obj));

    if (n_active_objs == 0)
    {
        object_.push_back(obj);
        obj->SetActive(true);

        int n_objs = static_cast<int>(std::count(object_pool_.begin(), object_pool_.end(), obj));
        if (n_objs == 1)
        {
            object_pool_.erase(std::remove(object_pool_.begin(), object_pool_.end(), obj), object_pool_.end());
        }
        else if (n_objs > 1)
        {
            LOG("Unexpected: %d object instances in pool when activating obj %s. Duplicate names? not supported", n_objs, obj->GetName().c_str());
            return -1;
        }
        else
        {
            LOG("Unexpected finding: Object %s missing in pool empty when activating.", obj->GetName().c_str());
        }

        Vehicle* trailer_vehicle = static_cast<Vehicle*>(obj->TrailerVehicle());
        if (trailer_vehicle && trailer_vehicle != obj)
        {
            activateObject(trailer_vehicle, call_index + 1);
        }
    }
    else
    {
        LOG("Failed to activate obj %s. Already active (%d instances in active list) or duplicate name?", obj->GetName().c_str(), n_active_objs);
        return -1;
    }

    return 0;
}

int Entities::deactivateObject(Object* obj, int call_index)
{
    const int max_trailers = 100;
    if (call_index >= max_trailers)
    {
        LOG_AND_QUIT("Error: deactivateObject max recursion reached (%d). Check scenario trailer config", max_trailers);
    }

    int n_active_objs = static_cast<int>(std::count(object_.begin(), object_.end(), obj));

    if (n_active_objs == 1)
    {
        object_.erase(std::remove(object_.begin(), object_.end(), obj), object_.end());
        obj->SetActive(false);

        int n_objs = static_cast<int>(std::count(object_pool_.begin(), object_pool_.end(), obj));
        if (n_objs == 0)
        {
            object_pool_.push_back(obj);
        }
        else
        {
            LOG("Unexpected: Object %s already in pool (%d instances) when deactivating it.", obj->GetName().c_str(), n_objs);
        }

        Vehicle* trailer_vehicle = static_cast<Vehicle*>(obj->TrailerVehicle());
        if (trailer_vehicle && trailer_vehicle != obj)
        {
            deactivateObject(trailer_vehicle, call_index + 1);
        }
    }
    else if (n_active_objs > 1)
    {
        LOG("Unexpected: %d object instances found when deactivating obj %s. Duplicate names? not supported", n_active_objs, obj->GetName().c_str());
        return -1;
    }
    else
    {
        LOG("Failed to deactivate obj %s. Already inactive (0 in active list).", obj->GetName().c_str());
        return -1;
    }

    return 0;
}

void Entities::removeObject(int id, bool recursive)
{
    for (size_t i = 0; i < object_.size(); i++)
    {
        if (object_[i]->id_ == id)
        {
            removeObject(object_[i], recursive);
            return;
        }
    }
}

void Entities::removeObject(std::string name, bool recursive)
{
    for (size_t i = 0; i < object_.size(); i++)
    {
        if (object_[i]->name_ == name)
        {
            removeObject(object_[i], recursive);
            return;
        }
    }
}

void Entities::removeObject(Object* object, bool recursive)
{
    if (recursive)
    {
        for (size_t i = 0; i < object_.size(); i++)
        {
            if (object_[i] == object)
            {
                if (object->type_ == Object::Type::VEHICLE)
                {
                    Vehicle* v = static_cast<Vehicle*>(object);
                    if (v->trailer_hitch_ && v->trailer_hitch_->trailer_vehicle_)
                    {
                        removeObject(v->trailer_hitch_->trailer_vehicle_, recursive);
                        break;
                    }
                }
            }
        }
    }

    object_.erase(std::remove(object_.begin(), object_.end(), object), object_.end());
    delete object;

    return;
}

bool Entities::nameExists(std::string name)
{
    for (size_t i = 0; i < object_.size(); i++)
    {
        if (object_[i]->name_ == name)
        {
            return true;
        }
    }
    return false;
}

bool Entities::indexExists(int id)
{
    for (size_t i = 0; i < object_.size(); i++)
    {
        if (object_[i]->id_ == id)
        {
            return false;
        }
    }
    return true;
}

int Entities::getNewId()
{
    return nextId_++;
}

Vehicle::Vehicle() : Object(Object::Type::VEHICLE), trailer_coupler_(nullptr), trailer_hitch_(nullptr)
{
    category_                    = static_cast<int>(Category::CAR);
    performance_.maxAcceleration = 10.0;
    performance_.maxDeceleration = 10.0;
    performance_.maxSpeed        = 100.0;
}

Vehicle::Vehicle(const Vehicle& v) : Object(Object::Type::VEHICLE), trailer_coupler_(nullptr), trailer_hitch_(nullptr)
{
    *this = v;

    if (v.trailer_coupler_ && v.trailer_coupler_->tow_vehicle_)
    {
        trailer_coupler_.reset(new TrailerCoupler(*v.trailer_coupler_));
        trailer_coupler_->tow_vehicle_ = nullptr;
    }

    if (v.trailer_hitch_ && v.trailer_hitch_->trailer_vehicle_)
    {
        // make a unique copy of any trailer
        trailer_hitch_.reset(new TrailerHitch(*v.trailer_hitch_));
        Vehicle* trailer = new Vehicle(*(static_cast<Vehicle*>((v.trailer_hitch_->trailer_vehicle_))));
        ConnectTrailer(trailer);
    }
}

// Vehicle& Vehicle::operator=(const Vehicle& v)
// {
// 	*this = v;

// 	if (v.trailer_coupler_ && v.trailer_coupler_->tow_vehicle_)
// 	{
// 		trailer_coupler_.reset(new TrailerCoupler(*v.trailer_coupler_));
// 		trailer_coupler_->tow_vehicle_ = nullptr;
// 	}

// 	if (v.trailer_hitch_ && v.trailer_hitch_->trailer_vehicle_)
// 	{
// 		// make a unique copy of any trailer
// 		trailer_hitch_.reset(new TrailerHitch(*v.trailer_hitch_));
// 		Vehicle* trailer = new Vehicle(*(static_cast<Vehicle*>((v.trailer_hitch_->trailer_vehicle_))));
// 		ConnectTrailer(trailer);
// 	}
// }

Vehicle::~Vehicle()
{
}

int Vehicle::ConnectTrailer(Vehicle* trailer)
{
    if (trailer && trailer->trailer_coupler_)
    {
        trailer->trailer_coupler_->tow_vehicle_ = this;
        if (trailer_hitch_)
        {
            trailer_hitch_->trailer_vehicle_ = trailer;
            return 0;
        }
    }

    return -1;
}

int Vehicle::DisconnectTrailer()
{
    if (trailer_hitch_ && trailer_hitch_->trailer_vehicle_)
    {
        reinterpret_cast<Vehicle*>(trailer_hitch_->trailer_vehicle_)->trailer_coupler_->tow_vehicle_ = nullptr;
        trailer_hitch_->trailer_vehicle_                                                             = nullptr;
    }

    return 0;
}

void Vehicle::AlignTrailers()
{
    // Calculate neutral trailer position and orientation
    Vehicle* v       = this;
    Vehicle* trailer = nullptr;
    int      counter = 0;
    while (v && counter++ < 100)
    {
        trailer = static_cast<Vehicle*>(v->TrailerVehicle());
        if (trailer)
        {
            SE_Vector v0(v->trailer_hitch_->dx_, 0.0);
            v0 = v0.Rotate(pos_.GetH()) + SE_Vector(v->pos_.GetX(), v->pos_.GetY());

            SE_Vector v1(-trailer->trailer_coupler_->dx_, 0.0);
            v1 = v1.Rotate(pos_.GetH()) + v0;

            trailer->pos_.SetInertiaPos(v1.x(), v1.y(), pos_.GetH());
            trailer->SetSpeed(GetSpeed());
        }
        v = trailer;
    }
}

std::string Vehicle::Category2String(int category)
{
    switch (category)
    {
        case Category::BICYCLE:
            return "BICYCLE";
        case Category::BUS:
            return "BUS";
        case Category::CAR:
            return "CAR";
        case Category::MOTORBIKE:
            return "MOTORBIKE";
        case Category::SEMITRAILER:
            return "SEMITRAILER";
        case Category::TRAILER:
            return "TRAILER";
        case Category::TRAIN:
            return "TRAIN";
        case Category::TRAM:
            return "TRAM";
        case Category::TRUCK:
            return "TRUCK";
        case Category::VAN:
            return "VAN";
        default:
            return "Unknown";
    }
}

std::string Vehicle::Role2String(int role)
{
    switch (role)
    {
        case Role::AMBULANCE:
            return "AMBULANCE";
        case Role::CIVIL:
            return "CIVIL";
        case Role::FIRE:
            return "FIRE";
        case Role::MILITARY:
            return "MILITARY";
        case Role::NONE:
            return "NONE";
        case Role::POLICE:
            return "POLICE";
        case Role::PUBLIC_TRANSPORT:
            return "PUBLIC_TRANSPORT";
        case Role::ROAD_ASSISTANCE:
            return "ROAD_ASSISTANCE";
        default:
            return "Unknown";
    }
}

Object* Entities::GetObjectByName(std::string name)
{
    for (size_t i = 0; i < object_.size(); i++)
    {
        if (name == object_[i]->name_)
        {
            return object_[i];
        }
    }

    for (size_t i = 0; i < object_pool_.size(); i++)
    {
        if (name == object_pool_[i]->name_)
        {
            return object_pool_[i];
        }
    }

    LOG("Failed to find object %s", name.c_str());

    return 0;
}

Object* Entities::GetObjectById(int id)
{
    for (size_t i = 0; i < object_.size(); i++)
    {
        if (id == object_[i]->id_)
        {
            return object_[i];
        }
    }

    for (size_t i = 0; i < object_pool_.size(); i++)
    {
        if (id == object_pool_[i]->id_)
        {
            return object_pool_[i];
        }
    }

    LOG("Failed to find object with id %d", id);

    return 0;
}

int Entities::GetObjectIdxById(int id)
{
    for (size_t i = 0; i < object_.size(); i++)
    {
        if (object_[i]->GetId() == id)
        {
            return static_cast<int>(i);
        }
    }

    return -1;
}

void Object::removeEvent(Event* event)
{
    auto it = std::find(objectEvents_.begin(), objectEvents_.end(), event);
    if (it != objectEvents_.end())
        objectEvents_.erase(it);
}

std::vector<OSCPrivateAction*> Object::getPrivateActions()
{
    std::vector<OSCPrivateAction*> actions;
    for (unsigned int i = 0; i < objectEvents_.size(); i++)
    {
        Event* event = objectEvents_[i];
        for (size_t n = 0; n < event->action_.size(); n++)
        {
            OSCAction* action = event->action_[n];
            if (action->GetBaseType() == OSCAction::BaseType::PRIVATE)
            {
                OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);
                if (pa->GetCurrentState() == StoryBoardElement::State::RUNNING || pa->GetCurrentState() == StoryBoardElement::State::STANDBY)
                {
                    actions.push_back(pa);
                }
            }
        }
    }
    return actions;
}

Object* Object::TowVehicle()
{
    Vehicle* tow_vehicle = nullptr;

    if (type_ == Object::Type::VEHICLE)
    {
        Vehicle* vehicle = static_cast<Vehicle*>(this);
        if (vehicle->trailer_coupler_ != nullptr)
        {
            if (vehicle->trailer_coupler_->tow_vehicle_)
            {
                if (vehicle->trailer_coupler_->tow_vehicle_->type_ == Object::Type::VEHICLE)
                {
                    tow_vehicle = static_cast<Vehicle*>(vehicle->trailer_coupler_->tow_vehicle_);
                    if (tow_vehicle != nullptr && tow_vehicle->trailer_hitch_ == nullptr)
                    {
                        LOG_ONCE("Warning: Tow vehicle %s lacks hitch", tow_vehicle->GetName().c_str());
                        tow_vehicle = nullptr;
                    }
                }
            }
        }
    }

    return tow_vehicle;
}

Object* Object::TrailerVehicle()
{
    Vehicle* trailer_vehicle = nullptr;

    if (type_ == Object::Type::VEHICLE)
    {
        Vehicle* vehicle = static_cast<Vehicle*>(this);
        if (vehicle->trailer_hitch_ != nullptr)
        {
            if (vehicle->trailer_hitch_->trailer_vehicle_)
            {
                if (vehicle->trailer_hitch_->trailer_vehicle_->type_ == Object::Type::VEHICLE)
                {
                    trailer_vehicle = static_cast<Vehicle*>(vehicle->trailer_hitch_->trailer_vehicle_);
                    if (trailer_vehicle != nullptr && trailer_vehicle->trailer_coupler_ == nullptr)
                    {
                        LOG_ONCE("Warning: Trailer vehicle %s lacks coupler", trailer_vehicle->GetName().c_str());
                        trailer_vehicle = nullptr;
                    }
                }
            }
        }
    }

    return trailer_vehicle;
}

std::string Object::Type2String(int type)
{
    switch (type)
    {
        case Type::MISC_OBJECT:
            return "MISC_OBJEC";
        case Type::PEDESTRIAN:
            return "PEDESTRIAN";
        case Type::VEHICLE:
            return "VEHICLE";
        case Type::TYPE_NONE:
            return "NONE";
        default:
            return "Unknown";
    }
}

std::string Pedestrian::Category2String(int category)
{
    switch (category)
    {
        case Category::ANIMAL:
            return "ANIMAL";
        case Category::PEDESTRIAN:
            return "PEDESTRIAN";
        case Category::WHEELCHAIR:
            return "WHEELCHAIR";
        default:
            return "Unknown";
    }
}

std::string MiscObject::Category2String(int category)
{
    switch (category)
    {
        case Category::BARRIER:
            return "BARRIER";
        case Category::BUILDING:
            return "BUILDING";
        case Category::CROSSWALK:
            return "CROSSWALK";
        case Category::GANTRY:
            return "GANTRY";
        case Category::NONE:
            return "NONE";
        case Category::OBSTACLE:
            return "OBSTACLE";
        case Category::PARKINGSPACE:
            return "PARKINGSPACE";
        case Category::PATCH:
            return "PATCH";
        case Category::POLE:
            return "POLE";
        case Category::RAILING:
            return "RAILING";
        case Category::ROADMARK:
            return "ROADMARK";
        case Category::SOUNDBARRIER:
            return "SOUNDBARRIER";
        case Category::STREETLAMP:
            return "STREETLAMP";
        case Category::TRAFFICISLAND:
            return "TRAFFICISLAND";
        case Category::TREE:
            return "TREE";
        case Category::VEGETATION:
            return "VEGETATION";
        case Category::WIND:
            return "WIND";
        default:
            return "Unknown";
    }
}
