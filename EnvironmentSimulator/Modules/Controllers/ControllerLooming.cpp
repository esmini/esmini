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

#include "ControllerLooming.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioEngine.hpp"
#include "ScenarioGateway.hpp"
#include "playerbase.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerLooming(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerLooming(initArgs);
}

ControllerLooming::ControllerLooming(InitArgs* args) : Controller(args)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LONG);

    if (args && args->properties && args->properties->ValueExists("timeGap"))
    {
        timeGap_ = strtod(args->properties->GetValueStr("timeGap"));
    }
    if (args && args->properties && args->properties->ValueExists("setSpeed"))
    {
        setSpeed_    = strtod(args->properties->GetValueStr("setSpeed"));
        setSpeedSet_ = true;
    }
}

void ControllerLooming::Step(double timeStep)
{
    // looming controller properties
    double       k0                = 1.8;
    double       k1                = 1.3;
    double       k2                = 1.5;
    double const nearPointDistance = 10.0;
    double       farPointDistance  = 80.0;

    double far_angle = 0.0;
    double far_x     = 0.0;
    double far_y     = 0.0;
    double far_tan_s = 0.0;

    hasFarTan              = false;
    int    road_counter    = -1;
    int    laneSec_counter = -1;
    bool   isIntersection  = false;
    double dist            = 0.0;

    currentSpeed_ = object_->GetSpeed();

    roadmanager::RoadProbeInfo s_data;
    // fixed near point
    object_->pos_.GetProbeInfo(nearPointDistance, &s_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
    double nearAngle = s_data.relative_h;

    roadmanager::OpenDrive* odr = roadmanager::Position::GetOpenDrive();
    if (odr != NULL)
    {
        roadmanager::Road* road = odr->GetRoadById(object_->pos_.GetTrackId());
        if (road == nullptr)
        {
            LOG("Road ID %d", object_->pos_.GetTrackId());
            return;
        }

        int    counter[2]         = {0, 0};      // left, right
        double angle_diff_prev[2] = {0.0, 0.0};  // left, right
        double far_x_prev[2]      = {0.0, 0.0};
        double far_y_prev[2]      = {0.0, 0.0};
        double far_tan_s_prev[2]  = {0.0, 0.0};
        double far_angle_prev[2]  =  // initial angle to points parallel to Ego
            {GetAngleInIntervalMinusPIPlusPI(object_->pos_.GetHRoad() + M_PI_2),
             GetAngleInIntervalMinusPIPlusPI(object_->pos_.GetHRoad() - M_PI_2)};  // left, right

        if (!IsAngleForward(object_->pos_.GetHRelative()))
        {  // According to direction switch initial angle to points parallel to Ego
            far_angle_prev[0] = GetAngleInIntervalMinusPIPlusPI(object_->pos_.GetHRoad() - M_PI_2);
            far_angle_prev[1] = GetAngleInIntervalMinusPIPlusPI(object_->pos_.GetHRoad() + M_PI_2);
        }

        roadmanager::Road* roadTemp       = nullptr;
        int                direction      = IsAngleForward(object_->pos_.GetHRelative());
        int                direction_prev = direction;
        roadmanager::Lane* lane           = nullptr;

        while (!hasFarTan && dist < farPointDistance)
        {
            roadmanager::LaneSection* lsec = nullptr;
            if (road_counter == -1)  // first time, first road, same as ego vehicle is located
            {
                roadTemp = road;
                // On this road, find out whether we're moving along the road s-direction or not
                direction = IsAngleForward(object_->pos_.GetHRelative()) ? 1 : -1;
            }
            else
            {
                // decide whether to pick next road as successor or predecessor
                // based on the current direction
                roadmanager::RoadLink* roadLink =
                    roadTemp->GetLink(direction == 1 ? roadmanager::LinkType::SUCCESSOR : roadmanager::LinkType::PREDECESSOR);

                if (roadLink == nullptr)
                {
                    break;  // no more connecting roads
                }

                if (roadLink->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
                {
                    isIntersection = true;
                    break;  // currently intersection not considered.
                }

                roadTemp = odr->GetRoadById(roadLink->GetElementId());

                // we just entered a new road, and we have a road link.
                // Compare direction of new road with the previous one
                direction_prev = direction;
                if ((direction == 1 && roadLink->GetContactPointType() == roadmanager::ContactPointType::CONTACT_POINT_END) ||
                    (direction == -1 && roadLink->GetContactPointType() == roadmanager::ContactPointType::CONTACT_POINT_START))
                {
                    // New road has opposite direction, change sign of direction variable
                    direction *= -1;
                }
            }

            road_counter++;
            double dist_lsec = 0.0;
            for (int n = 0; !hasFarTan && dist + dist_lsec < farPointDistance && n < roadTemp->GetNumberOfLaneSections(); n++)
            {
                int current_laneSec_idx = n;
                if (direction == -1)
                {  // switch lanesec index according to direction of ego
                    current_laneSec_idx = roadTemp->GetNumberOfLaneSections() - 1 - n;
                }

                lsec = roadTemp->GetLaneSectionByIdx(current_laneSec_idx);  // pick lanesec by lanesec id
                if (lsec == nullptr)
                {
                    LOG("Unexpected missing lane Section");
                    return;
                }
                if (road_counter == 0)
                {
                    // lane = lsec->GetLaneById(object_->pos_.GetLaneId());
                    if (laneSec_counter < 0)
                    {
                        lane = lsec->GetLaneById(object_->pos_.GetLaneId());
                    }
                    else if (laneSec_counter >= 0)
                    {
                        lane = lsec->GetLaneById(
                            lane->GetLink(direction_prev == 1 ? roadmanager::LinkType::SUCCESSOR : roadmanager::LinkType::PREDECESSOR)->GetId());
                    }
                }
                else if (road_counter > 0)
                {
                    // for lane links, we look from previous lane, hence switch direction
                    roadmanager::LaneLink* link =
                        lane->GetLink(direction_prev == 1 ? roadmanager::LinkType::SUCCESSOR : roadmanager::LinkType::PREDECESSOR);
                    if (link == nullptr)
                    {
                        LOG("Unexpected missing lane link");
                        return;
                    }

                    lane = lsec->GetLaneById(link->GetId());
                }
                std::vector<roadmanager::PointStruct> osi_points = lane->GetOSIPoints()->GetPoints();

                double dist_left = -1.0;  // the distance from ego vehicle to the closest found tangent point
                laneSec_counter++;
                for (int m = 0; m < 2; m++)  // m==0: Left, m==1: Right
                {
                    // road 0: start from first OSI point, othe roads: skip first since coinciding with last on previous
                    for (size_t k = road_counter == 0 ? 0 : 1; k < osi_points.size(); k++)
                    {
                        size_t cur_idx = k;
                        if (direction == -1)  // first road
                        {
                            cur_idx = osi_points.size() - 1 - k;
                        }

                        if ((road_counter == 0 &&  // first road
                             ((direction == 1 && osi_points[cur_idx].s - object_->pos_.GetS() > farPointDistance) ||
                              (direction == -1 && object_->pos_.GetS() - osi_points[cur_idx].s > farPointDistance))) ||
                            // Not first road
                            (road_counter > 0 &&  // missed this check
                             ((direction == 1 && dist + osi_points[cur_idx].s > farPointDistance) ||
                              (direction == -1 && dist + dist_lsec - osi_points[cur_idx].s > farPointDistance))))
                        {
                            break;  // looked passed 80 meters, we can quit now
                        }

                        // we're on right side, and already passed a found tangent point on left side
                        // no need to look further. Left side "win".
                        else if (m == 1 && hasFarTan &&
                                 ((road_counter == 0 &&  // first road
                                   ((direction == 1 && osi_points[cur_idx].s > object_->pos_.GetS() + dist_left) ||
                                    (direction == -1 && osi_points[cur_idx].s < object_->pos_.GetS() - dist_left))) ||
                                  (road_counter > 0 &&  // Not first road
                                   ((direction == 1 && osi_points[cur_idx].s > dist_left - dist) ||
                                    (direction == -1 && lsec->GetLength() - osi_points[cur_idx].s > dist_left - dist)))))
                        {
                            break;
                        }

                        else if (road_counter > 0 ||  // Not first round, let's look at OSI points from start of road
                                                      // First round, only check OSI points in front of ego
                                 (direction == 1 && osi_points[cur_idx].s > object_->pos_.GetS()) ||
                                 (direction == -1 && osi_points[cur_idx].s < object_->pos_.GetS()))
                        {
                            double xr = 0.0, yr = 0.0;
                            double local_x = 0.0;
                            double local_y = (m == 0 ? +1 : -1) * road->GetLaneWidthByS(osi_points[cur_idx].s, object_->pos_.GetLaneId()) / 2;
                            local_y        = (direction == 1 ? local_y : -local_y);  // switch y position according to ego driving side
                            RotateVec2D(local_x, local_y, osi_points[cur_idx].h, xr, yr);
                            double far_x_tmp   = osi_points[cur_idx].x + xr;
                            double far_y_tmp   = osi_points[cur_idx].y + yr;
                            double farTanS_tmp = dist + (direction == 1 ? osi_points[cur_idx].s : dist_lsec - osi_points[cur_idx].s);
                            if (road_counter == 0)
                            {
                                // subtract object position
                                farTanS_tmp -= (direction == 1 ? object_->pos_.GetS() : road->GetLength() - object_->pos_.GetS());
                            }

                            double farAngle_tmp =
                                GetAngleInIntervalMinusPIPlusPI(atan2(far_y_tmp - object_->pos_.GetY(), far_x_tmp - object_->pos_.GetX()));
                            angleDiff = GetAngleDifference(farAngle_tmp, far_angle_prev[m]);

                            if (counter[m] > 0)
                            {
                                if (abs(angleDiff) > SMALL_NUMBER &&  // skip points at same angle (e.g. identical position)
                                    SIGN(angleDiff) != SIGN(angle_diff_prev[m]))
                                {
                                    // LOG("has far tan");
                                    hasFarTan = true;
                                    far_x     = far_x_prev[m];
                                    far_y     = far_y_prev[m];
                                    far_tan_s = far_tan_s_prev[m];
                                    far_angle = far_angle_prev[m];

                                    if (m == 0)
                                    {
                                        dist_left = far_tan_s_prev[m];
                                    }
                                    break;
                                }
                            }

                            // Store info on farest point so far on this side of the road
                            far_x_prev[m]     = far_x_tmp;
                            far_y_prev[m]     = far_y_tmp;
                            far_tan_s_prev[m] = farTanS_tmp;
                            far_angle_prev[m] = farAngle_tmp;
                            if (abs(angleDiff) > SMALL_NUMBER)
                            {
                                angle_diff_prev[m] = angleDiff;
                            }

                            counter[m] += 1;  // Switch to right side
                        }
                    }
                }
                dist_lsec += lsec->GetLength();
            }
            if (road_counter == 0)  // first time
            {
                dist += (direction == 1 ? road->GetLength() - object_->pos_.GetS() : object_->pos_.GetS());
            }
            else
            {
                dist += roadTemp->GetLength();
            }
        }
    }

    bool         hasLeadFar   = false;
    int          minObjIndex  = -1;
    double       minGapLength = LARGE_NUMBER;
    const double minDist      = 3.0;  // minimum distance to keep to lead vehicle

    const double minLateralDist = 5.0;
    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        Object* pivot_obj = entities_->object_[i];
        if (pivot_obj == nullptr || pivot_obj == object_)
        {
            continue;
        }

        double lookaheadDist = 130;
        // Measure longitudinal distance to all vehicles, don't utilize costly free-space option, instead measure ref point to ref point
        roadmanager::PositionDiff diff;
        if (object_->pos_.Delta(&pivot_obj->pos_, diff, false, lookaheadDist) == true)  // look only double timeGap ahead
        {
            // path exists between position objects
            // adjust longitudinal dist wrt bounding boxes
            double adjustedGapLength = diff.ds;
            double dHeading          = GetAbsAngleDifference(object_->pos_.GetH(), pivot_obj->pos_.GetH());
            if (dHeading < M_PI_2)  // objects are pointing roughly in the same direction
            {
                adjustedGapLength -=
                    (static_cast<double>(object_->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(object_->boundingbox_.center_.x_)) +
                    (static_cast<double>(pivot_obj->boundingbox_.dimensions_.length_) / 2.0 -
                     static_cast<double>(pivot_obj->boundingbox_.center_.x_));
            }
            else  // objects are pointing roughly in the opposite direction
            {
                adjustedGapLength -=
                    (static_cast<double>(object_->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(object_->boundingbox_.center_.x_)) +
                    (static_cast<double>(pivot_obj->boundingbox_.dimensions_.length_) / 2.0 +
                     static_cast<double>(pivot_obj->boundingbox_.center_.x_));
            }

            // dLaneId == 0 indicates there is linked path between object lanes, i.e. no lane changes needed
            if (diff.dLaneId == 0 && adjustedGapLength > 0 && adjustedGapLength < minGapLength && abs(diff.dt) < minLateralDist)
            {
                minGapLength = adjustedGapLength;
                minObjIndex  = static_cast<int>(i);  // TODO: size_t to int

                // find far point from lead as reference, if lead <= farPointDistance(80) m
                if (minGapLength <= farPointDistance)
                {
                    if (isIntersection && dist < minGapLength)
                    {  // lead is considered till it reach intersection and stop looking ahead.
                        farPointDistance = 0.0;
                        break;
                    }
                    if (hasFarTan && far_tan_s < minGapLength)
                    {  // far tan point wins if lead point greater
                        break;
                    }
                    hasLeadFar = true;
                    far_angle  = GetAngleInIntervalMinusPIPlusPI(
                        atan2(object_->pos_.GetY() - pivot_obj->pos_.GetY(), object_->pos_.GetX() - pivot_obj->pos_.GetX()) - object_->pos_.GetH());
                    // LOG("new far: %.2f, %.2f\n", pivot_obj->pos_.GetX(), pivot_obj->pos_.GetY());
                    far_x = pivot_obj->pos_.GetX();
                    far_y = pivot_obj->pos_.GetY();
                }
            }
        }
    }

    if (!hasFarTan && !hasLeadFar)
    {
        // dynamic far road points
        object_->pos_.GetProbeInfo(farPointDistance, &s_data, roadmanager::Position::LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
        far_angle = s_data.relative_h;

        far_x = s_data.road_lane_info.pos[0];
        far_y = s_data.road_lane_info.pos[1];
    }

    if (minObjIndex > -1)
    {
        if (minGapLength < 1)
        {
            acc = 0.0;
        }
        else
        {
            double speedForTimeGap = MAX(currentSpeed_, entities_->object_[static_cast<unsigned int>(minObjIndex)]->GetSpeed());
            double followDist      = minDist + timeGap_ * fabs(speedForTimeGap);  // (m)
            double distRem         = minGapLength - followDist;
            double distFactor      = MIN(1.0, distRem / followDist);

            double dvMin = currentSpeed_ - MIN(setSpeed_, entities_->object_[static_cast<unsigned int>(minObjIndex)]->GetSpeed());
            double dvSet = currentSpeed_ - setSpeed_;

            acc = distFactor - distFactor * dvSet - (1 - distFactor) * dvMin;  // weighted combination of relative distance and speed
        }
    }
    else
    {
        // no lead vehicle to adapt to, adjust according to setSpeed
        acc = setSpeed_ - currentSpeed_;
    }

    if (timeStep > SMALL_NUMBER)
    {
        double nearAngleDot = (nearAngle - prevNearAngle) / timeStep;
        double farAngleDot  = (far_angle - prevFarAngle) / timeStep;

        steering = k0 * nearAngle + k1 * nearAngleDot + k2 * farAngleDot;
        // scale (90 degrees) and truncate
        steering = steering / M_PI_4;
        steering = CLAMP(steering, -1, 1);
    }
    prevNearAngle = nearAngle;
    prevFarAngle  = far_angle;

    gateway_->getObjectStatePtrByIdx(object_->GetId())->state_.pos.SetLockOnLane(true);
    vehicle_.DrivingControlAnalog(timeStep, CLAMP(acc, -1, 1), steering);

    gateway_->updateObjectWorldPosXYH(object_->GetId(), 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
    gateway_->updateObjectWheelAngle(object_->GetId(), 0.0, vehicle_.wheelAngle_);

    gateway_->updateObjectSpeed(object_->GetId(), 0.0, vehicle_.speed_);
    object_->sensor_pos_[0] = far_x;
    object_->sensor_pos_[1] = far_y;

    object_->SetSensorPosition(far_x, far_y, 0.0);

    Controller::Step(timeStep);
}

void ControllerLooming::Init()
{
    Controller::Init();
}

int ControllerLooming::Activate(ControlActivationMode lat_activation_mode,
                                ControlActivationMode long_activation_mode,
                                ControlActivationMode light_activation_mode,
                                ControlActivationMode anim_activation_mode)
{
    currentSpeed_ = object_->GetSpeed();
    if (mode_ == ControlOperationMode::MODE_ADDITIVE || setSpeedSet_ == false)
    {
        setSpeed_ = object_->GetSpeed();
    }

    if (object_)
    {
        vehicle_.Reset();
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
        vehicle_.speed_ = object_->GetSpeed();
        vehicle_.SetMaxAcc(object_->GetMaxAcceleration());
        vehicle_.SetMaxDec(object_->GetMaxDeceleration());
        vehicle_.SetSteeringRate(steering_rate_);
    }

    Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // Make sure heading is aligned with road driving direction
        object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
    }
    if (player_)
    {
        player_->SteeringSensorSetVisible(object_->GetId(), true);
    }

    return 0;
}

void ControllerLooming::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}
