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
#include "OSITrafficCommand.hpp"

int ReportTrafficCommand(osi3::TrafficCommand *tc, OSCPrivateAction *action, double time)
{
    tc->clear_timestamp();
    tc->mutable_timestamp()->set_seconds(static_cast<int64_t>(time));
    tc->mutable_timestamp()->set_nanos(static_cast<uint32_t>(time - floor(time) * 1e9));

    Object *obj = action->object_;

    if (obj == nullptr)
    {
        LOG_ERROR("Error: Missing object in private action {}", action->GetName());
        return -1;
    }

    switch (action->action_type_)
    {
        case OSCPrivateAction::ActionType::LAT_LANE_CHANGE:
        {
            osi3::TrafficAction *ta = tc->add_action();

            LOG_INFO("OSITrafficCmd: Lane change action {} started for obj {}", action->GetName(), action->object_ ? action->object_->GetId() : -1);
            tc->mutable_traffic_participant_id()->set_value(action->object_ ? static_cast<unsigned int>(action->object_->GetId()) : UINT_MAX);

            ta->mutable_lane_change_action()->mutable_action_header()->mutable_action_id()->set_value(action->GetId());

            LatLaneChangeAction *a = reinterpret_cast<LatLaneChangeAction *>(action);
            if (a->transition_.dimension_ == OSCPrivateAction::DynamicsDimension::DISTANCE)
            {
                ta->mutable_lane_change_action()->set_distance(a->transition_.GetParamTargetVal());
                ta->mutable_lane_change_action()->set_duration(0.0);
            }
            else if (a->transition_.dimension_ == OSCPrivateAction::DynamicsDimension::TIME)
            {
                ta->mutable_lane_change_action()->set_distance(0.0);
                ta->mutable_lane_change_action()->set_duration(a->transition_.GetParamTargetVal());
            }
            else
            {
                // no constraints
                ta->mutable_lane_change_action()->set_distance(0.0);
                ta->mutable_lane_change_action()->set_duration(0.0);
            }

            if (a->transition_.shape_ == OSCPrivateAction::DynamicsShape::CUBIC)
            {
                ta->mutable_lane_change_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_CUBIC);
            }
            else if (a->transition_.shape_ == OSCPrivateAction::DynamicsShape::LINEAR)
            {
                ta->mutable_lane_change_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_LINEAR);
            }
            else if (a->transition_.shape_ == OSCPrivateAction::DynamicsShape::SINUSOIDAL)
            {
                ta->mutable_lane_change_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_SINUSOIDAL);
            }
            else if (a->transition_.shape_ == OSCPrivateAction::DynamicsShape::STEP)
            {
                ta->mutable_lane_change_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_STEP);
            }
            else
            {
                LOG_ERROR("OSITrafficCmd: Unrecognized shape type: {}", a->transition_.shape_);
                ta->mutable_lane_change_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_UNSPECIFIED);
            }

            if (a->target_->type_ == LatLaneChangeAction::Target::Type::ABSOLUTE_LANE)
            {
                ta->mutable_lane_change_action()->set_relative_target_lane(a->target_->value_);
            }
            else  // relative
            {
                ta->mutable_lane_change_action()->set_relative_target_lane(roadmanager::GetLaneIdDelta(obj->pos_.GetLaneId(), a->target_->value_));
            }

            break;
        }

        case OSCPrivateAction::ActionType::LONG_SPEED:
        {
            osi3::TrafficAction *ta = tc->add_action();

            LOG_INFO("OSITrafficCmd: Speed action {} started for obj {}", action->GetName(), action->object_ ? action->object_->GetId() : -1);

            tc->mutable_traffic_participant_id()->set_value(action->object_ ? static_cast<unsigned int>(action->object_->GetId()) : UINT_MAX);

            ta->mutable_lane_change_action()->mutable_action_header()->mutable_action_id()->set_value(action->GetId());

            LongSpeedAction *a = reinterpret_cast<LongSpeedAction *>(action);
            if (a->transition_.dimension_ == OSCPrivateAction::DynamicsDimension::DISTANCE)
            {
                ta->mutable_speed_action()->set_distance(a->transition_.GetParamTargetVal());
                ta->mutable_speed_action()->set_duration(0.0);
            }
            else if (a->transition_.dimension_ == OSCPrivateAction::DynamicsDimension::TIME)
            {
                ta->mutable_speed_action()->set_distance(0.0);
                ta->mutable_speed_action()->set_duration(a->transition_.GetParamTargetVal());
            }
            else
            {
                // no constraints
                ta->mutable_speed_action()->set_distance(0.0);
                ta->mutable_speed_action()->set_duration(0.0);
            }

            if (a->transition_.shape_ == OSCPrivateAction::DynamicsShape::CUBIC)
            {
                ta->mutable_speed_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_CUBIC);
            }
            else if (a->transition_.shape_ == OSCPrivateAction::DynamicsShape::LINEAR)
            {
                ta->mutable_speed_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_LINEAR);
            }
            else if (a->transition_.shape_ == OSCPrivateAction::DynamicsShape::SINUSOIDAL)
            {
                ta->mutable_speed_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_SINUSOIDAL);
            }
            else if (a->transition_.shape_ == OSCPrivateAction::DynamicsShape::STEP)
            {
                ta->mutable_speed_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_STEP);
            }
            else
            {
                LOG_ERROR("Unrecognized shape type: {}", a->transition_.shape_);
                ta->mutable_speed_action()->set_dynamics_shape(osi3::TrafficAction_DynamicsShape_DYNAMICS_SHAPE_UNSPECIFIED);
            }

            if (a->target_->type_ == LongSpeedAction::Target::TargetType::ABSOLUTE_SPEED)
            {
                ta->mutable_speed_action()->set_absolute_target_speed(a->target_->value_);
            }
            else  // relative
            {
                ta->mutable_speed_action()->set_absolute_target_speed(obj->GetSpeed() + a->target_->value_);
            }
            break;
        }

        case OSCPrivateAction::ActionType::TELEPORT:
        {
            osi3::TrafficAction *ta = tc->add_action();

            LOG_INFO("OSITrafficCmd: Teleport action {} started for obj {}", action->GetName(), action->object_ ? action->object_->GetId() : -1);

            tc->mutable_traffic_participant_id()->set_value(action->object_ ? static_cast<unsigned int>(action->object_->GetId()) : UINT_MAX);

            ta->mutable_teleport_action()->mutable_action_header()->mutable_action_id()->set_value(action->GetId());

            TeleportAction *a = reinterpret_cast<TeleportAction *>(action);
            a->position_.EvaluateRelation();

            ta->mutable_teleport_action()->mutable_position()->set_x(a->position_.GetX());
            ta->mutable_teleport_action()->mutable_position()->set_y(a->position_.GetY());
            ta->mutable_teleport_action()->mutable_position()->set_z(a->position_.GetZ());

            ta->mutable_teleport_action()->mutable_orientation()->set_yaw(a->position_.GetH());
            ta->mutable_teleport_action()->mutable_orientation()->set_pitch(a->position_.GetP());
            ta->mutable_teleport_action()->mutable_orientation()->set_roll(a->position_.GetR());

            break;
        }

        default:

            LOG_ERROR("OSITrafficCommand: Unsupported type {} of action {}", action->action_type_, action->GetName());
    }

    return 0;
}