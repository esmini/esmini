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

/*
 * This controller simulates a bad or dizzy driver by manipulating
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

// #include "playerbase.hpp"
#include "ControllerFollowReference.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioEngine.hpp"
#include "logger.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerFollowReference(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerFollowReference(initArgs);
}

ControllerFollowReference::ControllerFollowReference(InitArgs* args) : Controller(args)
{
    if (args->properties->ValueExists("stanleyCrossTrackErrorGain"))
    {
        stanley_cross_track_error_gain_ = strtod(args->properties->GetValueStr("stanleyCrossTrackErrorGain"));
    }

    if (args->properties->ValueExists("stanleySpeedStabilizer"))
    {
        stanley_speed_stabilizer_ = strtod(args->properties->GetValueStr("stanleySpeedStabilizer"));
    }

    if (args->properties->ValueExists("stanleyHeadingGain"))
    {
        stanley_heading_gain_ = strtod(args->properties->GetValueStr("stanleyHeadingGain"));
    }

    if (args->properties->ValueExists("stanleyCurvatureGain"))
    {
        stanley_curvature_gain_ = strtod(args->properties->GetValueStr("stanleyCurvatureGain"));
    }

    if (args->properties->ValueExists("speedControlStiffness"))
    {
        speed_control_stiffness_ = strtod(args->properties->GetValueStr("speedControlStiffness"));
    }

    if (args->properties->ValueExists("referenceEntity"))
    {
        reference_entity_name_ = args->properties->GetValueStr("referenceEntity");
    }

    LOG_INFO(
        "ControllerFollowReference: stanleyCrossTrackErrorGain: {:.2f}, stanleySpeedStabilizer: {:.2f}, stanleyHeadingGain: {:.2f}, stanleyCurvatureGain: {:.2f} speedControlStiffness: {:.2f}",
        stanley_cross_track_error_gain_,
        stanley_speed_stabilizer_,
        stanley_heading_gain_,
        stanley_curvature_gain_,
        speed_control_stiffness_);
}

void ControllerFollowReference::Init()
{
    // FollowGhost controller forced into override mode - will not perform any scenario actions
    if (mode_ != ControlOperationMode::MODE_OVERRIDE)
    {
        LOG_INFO("FollowReference controller mode \"{}\" not applicable. Using override mode instead.", Mode2Str(mode_));
        mode_ = ControlOperationMode::MODE_OVERRIDE;
    }

    Controller::Init();
}

void ControllerFollowReference::Step(double timeStep)
{
    if (reference_entity_ == nullptr)
    {
        // No ghost associated
        return;
    }

    // find lateral distance between Ego and reference vehicle x-axis
    double measure_point[2] = {0.0, 0.0};
    if (object_->GetSpeed() >= 0.0)
    {
        // measure from Ego front axle when driving forward
        RotateVec2D(wheelbase_, 0.0, object_->pos_.GetH(), measure_point[0], measure_point[1]);
    }

    double dy = DistanceFromPointToLine2DWithAngle(object_->pos_.GetX() + measure_point[0],
                                                   object_->pos_.GetY() + measure_point[1],
                                                   reference_entity_->pos_.GetX(),
                                                   reference_entity_->pos_.GetY(),
                                                   reference_entity_->pos_.GetH());

    // find longitudinal distance and heading difference between Ego and reference vehicle
    double dx = DistanceFromPointToLine2DWithAngle(object_->pos_.GetX(),
                                                   object_->pos_.GetY(),
                                                   reference_entity_->pos_.GetX(),
                                                   reference_entity_->pos_.GetY(),
                                                   object_->pos_.GetH() + M_PI_2);

    double dh = GetAngleInIntervalMinusPIPlusPI(reference_entity_->pos_.GetH() - object_->pos_.GetH());

    // approximate reference path curvature from ghost (reference) yaw rate / speed
    double traj_curvature = ABS_LIMIT(reference_entity_->pos_.GetHRate(), 0.5) / ABS_FLOOR(reference_entity_->GetSpeed(), SMALL_NUMBER);
    vehicle_.DrivingControlAccAndSteer(
        timeStep,
        speed_ctrl_.computeAcceleration(dx, object_->GetSpeed() - reference_entity_->GetSpeed()),
        steer_ctrl_.computeSteering(dy, dh * SIGN(object_->GetSpeed()), object_->GetSpeed(), traj_curvature, wheelbase_));

    // Fetch Z and Pitch from road position
    vehicle_.posZ_  = object_->pos_.GetZ();
    vehicle_.pitch_ = object_->pos_.GetP();

    // Register updated vehicle position
    object_->pos_.SetInertiaPos(vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
    object_->SetSpeed(vehicle_.speed_);

    // Update wheels wrt domains
    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)))
    {
        object_->wheel_rot_ = vehicle_.wheelRotation_;
    }

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        object_->wheel_angle_ = vehicle_.wheelAngle_;
    }

    Controller::Step(timeStep);
}

int ControllerFollowReference::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    if (mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)] != mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)])
    {
        LOG_ERROR("{} activation mode: lat {} long {}, but is only valid on both domains in combination",
                  GetName(),
                  mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)] == ControlActivationMode::ON ? "On" : "Off",
                  mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)] == ControlActivationMode::ON ? "On" : "Off");
    }

    if (object_)
    {
        vehicle_.Reset();

        // apply vehicle properties
        vehicle_.SetMaxSpeed(object_->GetMaxSpeed());
        vehicle_.SetMaxAcc(object_->GetMaxAcceleration());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);

        // initialize vehicle state
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.speed_ = object_->GetSpeed();

        object_->sensor_pos_[0] = object_->pos_.GetX();
        object_->sensor_pos_[1] = object_->pos_.GetY();
        object_->sensor_pos_[2] = object_->pos_.GetZ();

        wheelbase_ = object_->front_axle_.positionX;

        if (reference_entity_name_.empty())
        {
            reference_entity_ = object_->GetGhost();
            if (reference_entity_ == nullptr)
            {
                LOG_ERROR_AND_QUIT("No valid reference entity and no ghost!");
            }
            reference_entity_->SetVisibilityMask(SE_Env::Inst().GetOptions().GetOptionSetByEnum(esmini_options::HIDE_GHOST)
                                                     ? 0x0
                                                     : reference_entity_->visibilityMask_ &= ~(Object::Visibility::SENSORS));
        }
        else
        {
            reference_entity_ = scenario_engine_->entities_.GetObjectByName(reference_entity_name_);
            if (reference_entity_ == nullptr)
            {
                LOG_ERROR_AND_QUIT("Reference entity {} not found", reference_entity_name_);
            }
        }
    }

    steer_ctrl_.SetCrossTrackError(stanley_cross_track_error_gain_);
    steer_ctrl_.SetSpeedStabilizer(stanley_speed_stabilizer_);
    steer_ctrl_.SetHeadingGain(stanley_heading_gain_);
    steer_ctrl_.SetCurvatureGain(stanley_curvature_gain_);
    speed_ctrl_.SetStiffness(speed_control_stiffness_);

    return Controller::Activate(mode);
}

void ControllerFollowReference::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}