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

#include "ControllerRel2Abs.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include <iostream>
#include "OSCPrivateAction.hpp"
#include <cmath>
#include <math.h>
#include <algorithm>
#include "Storyboard.hpp"
#include "ScenarioEngine.hpp"
#include "RoadManager.hpp"
#include <ctype.h>

// #define CONTROLLER_REL2ABS_DEBUG

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerRel2Abs(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerRel2Abs(initArgs);
}

ControllerRel2Abs::ControllerRel2Abs(InitArgs* args)
    : Controller(args),
      pred_horizon(1),
      switching_threshold_dist(1.5),
      switching_threshold_speed(1.5)
{
    // ControllerRel2Abs forced into additive mode - will only react on scenario actions
    if (mode_ != ControlOperationMode::MODE_ADDITIVE)
    {
        LOG("ControllerRel2Abs mode \"%s\" not applicable. Using additive mode instead.", Mode2Str(mode_).c_str());
        mode_ = ControlOperationMode::MODE_ADDITIVE;
    }
    if (args->properties->ValueExists("horizon"))
    {
        pred_horizon = strtod(args->properties->GetValueStr("horizon"));
    }
    if (args->properties->ValueExists("thresholdSpeed"))
    {
        switching_threshold_speed = strtod(args->properties->GetValueStr("thresholdSpeed"));
    }
    if (args->properties->ValueExists("thresholdDist"))
    {
        switching_threshold_dist = strtod(args->properties->GetValueStr("thresholdDist"));
    }
}

void ControllerRel2Abs::Init()
{
    Controller::Init();
}

void ControllerRel2Abs::findEgo()
{
    if (ego_obj == -1)
    {
        LOG("Searching for vehicle named \"Ego\".");
        for (unsigned int i = 0; i < entities_->object_.size(); i++)
        {
            if (entities_->object_[i]->type_ == Object::Type::VEHICLE)
            {
                std::string name = entities_->object_[i]->name_;
                std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return static_cast<unsigned char>(tolower(c)); });
                if (name == "ego")
                {
                    ego_obj = static_cast<int>(i);
                    LOG("Object named \"%s\" used as ego vehicle.", entities_->object_[i]->name_.c_str());
                    return;
                }
            }
        }
        LOG("Ego not found, searching for externally controlled vehicles instead.");
        for (unsigned int i = 0; i < entities_->object_.size(); i++)
        {
            if (entities_->object_[i]->type_ == Object::Type::VEHICLE)
            {
                if (entities_->object_[i]->IsAnyActiveControllerOfType(Controller::Type::CONTROLLER_TYPE_EXTERNAL))
                {
                    ego_obj = static_cast<int>(i);
                    LOG("Object named \"%s\" used as ego vehicle due to being controlled externally.", entities_->object_[i]->name_.c_str());
                    return;
                }
            }
        }
        LOG("Ego not found, assuming ego is first object added: \"%s\"", entities_->object_[0]->name_.c_str());
        ego_obj = 0;
    }
}

void ControllerRel2Abs::Step(double timeStep)
{
    double egoSpeed = 0;

    findEgo();
    Object* ego = entities_->object_[static_cast<unsigned int>(ego_obj)];
    egoSpeed    = ego->GetSpeed();

    // ----------------------- prediction & switching algorithm - start -----------------------

    if (!switchNow)
    {
        double currentSpeed_ = object_->GetSpeed();

        double currentTime = scenario_engine_->getSimulationTime();

        if (currentTime - timestamp > pred_horizon)
        {
            timestamp = currentTime;
            csv_iter  = 0;

            // Clear data struct from previous data
            data.time.clear();
            data.posX.clear();
            data.posY.clear();
            data.speeds.clear();

            actualData.time.clear();
            actualData.posX.clear();
            actualData.posY.clear();
            actualData.speeds.clear();

            std::vector<ControllerRel2Abs::position_copy*> positionsCopied;
            // Copy positions, i.e. save original position and speed, keep object reference.
            for (unsigned int i = 0; i < entities_->object_.size(); i++)
            {
                position_copy* obj_copy = new position_copy();
                CopyPosition(entities_->object_[i], obj_copy);
                positionsCopied.push_back(obj_copy);  // Add copy dirty bits?
            }

            data.time.push_back(currentTime);
            data.posX.push_back(object_->pos_.GetX());
            data.posY.push_back(object_->pos_.GetY());
            data.speeds.push_back(object_->GetSpeed());

            // Vector of copies of all active private actions in scenario. Will in reality only contain private actions
            std::vector<OSCAction*> activeActionsCopies;
            for (unsigned int i = 0; i < entities_->object_.size(); i++)
            {
                std::vector<OSCPrivateAction*> actions =
                    entities_->object_[i]->getPrivateActions();  // getActions creates the vector => it's not updated by SE (only event vector is)
                std::vector<OSCPrivateAction*> activeActions;

                // Add init actions as well
                for (size_t j = 0; j < entities_->object_[i]->initActions_.size(); j++)
                {
                    actions.push_back(entities_->object_[i]->initActions_[j]);
                }

                for (size_t j = 0; j < actions.size(); j++)
                {
                    // OBS IsActive will return true if next state is running as well
                    if (actions[j]->GetCurrentState() == StoryBoardElement::State::RUNNING)
                    {
                        activeActions.push_back(actions[j]);
                    }
                }

                for (unsigned int j = 0; j < activeActions.size(); j++)
                {
                    OSCPrivateAction* action_copy = activeActions[j]->Copy();
                    action_copy->object_          = entities_->object_[i];
                    if (std::find(std::begin(action_whitelist), std::end(action_whitelist), activeActions[j]->action_type_) !=
                        std::end(action_whitelist))
                    {
                        activeActionsCopies.push_back(static_cast<OSCAction*>(action_copy));
                    }
                }
            }

            // Start all copied actions
            for (unsigned int i = 0; i < activeActionsCopies.size(); i++)
            {
                activeActionsCopies[i]->Start(currentTime);
            }

            // Simulation loop
            for (int i = 0; i < pred_nbr_timesteps; i++)
            {
                currentTime += pred_timestep;
                data.time.push_back(currentTime);

                for (unsigned int j = 0; j < entities_->object_.size(); j++)
                {
                    Object* object = entities_->object_[j];
                    for (unsigned int k = 0; k < activeActionsCopies.size(); k++)
                    {
                        OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(activeActionsCopies[k]);
                        if (pa->object_ == object)
                        {
                            activeActionsCopies[k]->Step(currentTime, pred_timestep);
                        }
                    }

                    // Default controller - i.e. point mass model w. constant speed.
                    double v       = object->GetSpeed();
                    double steplen = v * pred_timestep;
                    // Add or subtract stepsize according to curvature and offset, in order to keep constant speed
                    double curvature = object->pos_.GetCurvature();
                    double offset    = object->pos_.GetT();
                    if (abs(curvature) > SMALL_NUMBER)
                    {
                        // Approximate delta length by sampling curvature in current position
                        steplen += steplen * curvature * offset;
                    }

                    if (!object->CheckDirtyBits(Object::DirtyBit::LONGITUDINAL))
                    {
                        if (object->pos_.GetRoute())
                        {
                            object->pos_.MoveRouteDS(steplen);
                        }
                        else
                        {
                            // Adjustment movement to heading and road direction
                            if (GetAbsAngleDifference(object->pos_.GetH(), object->pos_.GetDrivingDirection()) > M_PI_2)
                            {
                                // If pointing in other direction
                                steplen *= -1;
                            }
                            object->pos_.MoveAlongS(steplen);
                        }
                    }

                    // LOG("Object[%d] speed = %lf, y: %lf", object->id_, object->GetSpeed(), object->pos_.GetY());

                    object->ClearDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::SPEED |
                                           Object::DirtyBit::WHEEL_ANGLE | Object::DirtyBit::WHEEL_ROTATION);

                    if (object == object_)
                    {
                        double newX = object->pos_.GetX();
                        double newY = object->pos_.GetY();
                        // We leave out caculating the new heading for now
                        data.posX.push_back(newX);
                        data.posY.push_back(newY);
                        data.speeds.push_back(v);
                    }
                }
            }

            for (unsigned int i = 0; i < positionsCopied.size(); i++)
            {
                position_copy* cpy  = positionsCopied[i];
                cpy->object->pos_   = *cpy->pos;
                cpy->object->speed_ = cpy->speed;
                cpy->object->SetDirty(cpy->dirtyBits);
                delete (positionsCopied[i]);
            }

            // Free memory allocated through Action.copy()
            for (unsigned int i = 0; i < activeActionsCopies.size(); i++)
            {
                delete (activeActionsCopies[i]);
            }
        }

        currentTime = scenario_engine_->getSimulationTime();
        actualData.time.push_back(currentTime);
        actualData.posX.push_back(object_->pos_.GetX());
        actualData.posY.push_back(object_->pos_.GetY());
        actualData.speeds.push_back(currentSpeed_);

        double x_act = actualData.posX.back();
        double y_act = actualData.posY.back();
        double v_act = actualData.speeds.back();
        double t     = actualData.time.back();

        double t0;
        double x_est;
        double y_est;
        double errorDist  = 0;
        double errorSpeed = 0;
        double v_est      = 0;

        // Loop through data and find time closest to t (ideally do linear interpolation)
        for (unsigned int i = 0; i < data.time.size(); i++)
        {
            double t1 = data.time.at(i);
            if (t1 == t)
            {
                x_est      = data.posX.at(i);
                y_est      = data.posY.at(i);
                v_est      = data.speeds.at(i);
                errorDist  = sqrt(pow((x_act - x_est), 2) + pow((y_act - y_est), 2));
                errorSpeed = fabs(v_act - v_est);

                if (errorDist > switching_threshold_dist || errorSpeed > switching_threshold_speed)
                {
                    switchNow = true;
                    LOG("Switch now, pos error = %lf, speed error = %lf", errorDist, errorSpeed);
                }
                break;
            }
            else if (t1 > t)
            {
                t0         = data.time.at(i - 1);
                x_est      = ((data.posX.at(i) - data.posX.at(i - 1)) / (t1 - t0)) * (t - t0) + data.posX.at(i - 1);
                y_est      = ((data.posY.at(i) - data.posY.at(i - 1)) / (t1 - t0)) * (t - t0) + data.posY.at(i - 1);
                v_est      = ((data.speeds.at(i) - data.speeds.at(i - 1)) / (t1 - t0)) * (t - t0) + data.speeds.at(i - 1);
                errorDist  = sqrt(pow((x_act - x_est), 2) + pow((y_act - y_est), 2));
                errorSpeed = fabs(v_act - v_est);

                if (errorDist > switching_threshold_dist || errorSpeed > switching_threshold_speed)
                {
                    switchNow = true;
                    LOG("Switch now, pos error = %lf, speed error = %lf", errorDist, errorSpeed);
                }
                break;
            }
        }

#ifdef CONTROLLER_REL2ABS_DEBUG
        if (data.time.size() > 0)
        {
            logData << currentTime << "," << errorDist << "," << actualData.posX.back() << "," << data.time.at(csv_iter) << ","
                    << data.posX.at(csv_iter) << "," << currentSpeed_ << "," << data.speeds.at(csv_iter) << "," << errorSpeed << ",\n";
        }

        csv_iter++;
        if (csv_iter > data.time.size() - 1)
        {
            csv_iter = data.time.size() - 1;
        }
#endif
    }
    // ----------------------- prediction & switching algorithm - end -----------------------

    if (switchNow && mode_ != ControlOperationMode::MODE_OVERRIDE)
    {
        std::vector<OSCPrivateAction*> actions =
            object_->getPrivateActions();  // getActions creates the vector => it's not updated by SE (only event vector is)
        std::vector<OSCPrivateAction*> activeActions;

        // Add init actions as well
        for (size_t i = 0; i < object_->initActions_.size(); i++)
        {
            actions.push_back(object_->initActions_[i]);
        }

        for (size_t i = 0; i < actions.size(); i++)
        {
            // OBS IsActive will return true if next state is running as well
            if (actions[i]->GetCurrentState() == StoryBoardElement::State::RUNNING)
            {
                activeActions.push_back(actions[i]);
            }
        }

        for (unsigned int i = 0; i < activeActions.size(); i++)
        {
            if (activeActions[i]->action_type_ == OSCPrivateAction::ActionType::LONG_SPEED)
            {
                LongSpeedAction* lsa = static_cast<LongSpeedAction*>(activeActions[i]);
                if (lsa->target_->type_ == LongSpeedAction::Target::TargetType::RELATIVE_SPEED)
                {
                    LongSpeedAction::TargetRelative* target = static_cast<LongSpeedAction::TargetRelative*>(lsa->target_.get());
                    if (target->object_ == ego)
                    {
                        double trgSpeed = lsa->target_->GetValue();
                        lsa->target_.reset(new LongSpeedAction::TargetAbsolute);
                        lsa->target_->value_ = trgSpeed;
                        LOG("LongSpeedAction Target has switched to absolute from relative with the value: %lf", trgSpeed);
                    }
                }
            }
            else if (activeActions[i]->action_type_ == OSCPrivateAction::ActionType::LONG_DISTANCE)
            {
                LongDistanceAction* lda = static_cast<LongDistanceAction*>(activeActions[i]);
                if (lda->target_object_ == ego)
                {
                    // Assumes object has reached steady state and want to continue in that state.
                    double           currentSpeed = lda->object_->GetSpeed();
                    LongSpeedAction* lsa          = new LongSpeedAction(nullptr);
                    lsa->object_                  = object_;
                    lsa->transition_.shape_       = OSCPrivateAction::DynamicsShape::LINEAR;
                    lsa->transition_.dimension_   = OSCPrivateAction::DynamicsDimension::RATE;
                    if (lda->dynamics_.max_acceleration_ != 0)
                        lsa->transition_.SetParamTargetVal(lda->dynamics_.max_acceleration_);
                    else if (lda->dynamics_.max_deceleration_ != 0)
                        lsa->transition_.SetParamTargetVal(lda->dynamics_.max_deceleration_);
                    else
                        lsa->transition_.SetParamTargetVal(10.0);
                    LongSpeedAction::TargetAbsolute* target_abs = new LongSpeedAction::TargetAbsolute;
                    target_abs->value_                          = currentSpeed;
                    lsa->target_.reset(target_abs);
                    // get lda's event and add lsa action to it
                    std::vector<Event*> events = lda->object_->getEvents();
                    for (size_t j = 0; j < events.size(); j++)
                    {
                        // Ensure it's correct event
                        for (size_t k = 0; k < events[j]->action_.size(); k++)
                        {
                            if (activeActions[i] == events[j]->action_[k])
                            {
                                events[j]->action_.push_back(static_cast<OSCAction*>(lsa));
                                activeActions.erase(activeActions.begin() + i);
                                activeActions.push_back(lsa);
                            }
                        }
                    }
                    lda->End();
                    lsa->Start(scenario_engine_->getSimulationTime());
                    LOG("Replacing the relative target LongDistanceAction with an absolute target LongSpeedAction and target value: %lf",
                        currentSpeed);
                }
            }
            else if (activeActions[i]->action_type_ == OSCPrivateAction::ActionType::LAT_LANE_CHANGE)
            {
                // Only samples relative lane at start, will not need to be handled
            }
            else if (activeActions[i]->action_type_ == OSCPrivateAction::ActionType::LAT_LANE_OFFSET)
            {
                // No relative object setup possible, will not need to be handled
            }
            else if (activeActions[i]->action_type_ == OSCPrivateAction::ActionType::LAT_DISTANCE)
            {
                // Action type not currently implemented in esmini
            }
            else if (activeActions[i]->action_type_ == OSCPrivateAction::ActionType::SYNCHRONIZE_ACTION)
            {
                SynchronizeAction* sa = static_cast<SynchronizeAction*>(activeActions[i]);
                if (sa->master_object_ == ego)
                {
                    if (sa->final_speed_)
                    {
                        if (sa->mode_ == SynchronizeAction::SynchMode::MODE_STEADY_STATE)
                        {
                            // mode wont change, do nothing (?)
                        }
                        else if (sa->mode_ == SynchronizeAction::SynchMode::MODE_LINEAR)
                        {
                            // This is the last part of the action, assume final speed is to be reached
                            double           trgSpeed   = sa->final_speed_->GetValue();
                            LongSpeedAction* lsa        = new LongSpeedAction(nullptr);
                            lsa->object_                = object_;
                            lsa->transition_.shape_     = OSCPrivateAction::DynamicsShape::LINEAR;
                            lsa->transition_.dimension_ = OSCPrivateAction::DynamicsDimension::DISTANCE;

                            // at this point synchronize action has exec. => lastDist was current dist this step.
                            double currentDist = sa->lastDist_;

                            // decide suitable acceleration
                            double currentAcc = 0;
                            // Check for true linear movement, if ego moves non-linearly target can get non-linear movement in linear mode
                            // since acceleration updates each timestep
                            bool   linear  = true;
                            double lastAcc = 0;
                            // Calculate average acceleration last 5 timesteps
                            for (auto it = speeds.begin(); it != speeds.end(); it++)
                            {
                                double nextSpeed = 0;
                                it++;
                                if (it != speeds.end())
                                {
                                    nextSpeed = it->second;
                                }
                                else
                                {
                                    nextSpeed = object_->GetSpeed();
                                }
                                it--;

                                double tmp = (nextSpeed - it->second) / it->first;
                                // round acceleration to 1 decimals
                                tmp = static_cast<int>(tmp * 10 + 0.5);
                                tmp /= 10;
                                if (it != speeds.begin())
                                {
                                    if (fabs(tmp) > 0 && tmp != lastAcc)
                                    {
                                        linear = false;
                                    }
                                }
                                lastAcc = tmp;
                                currentAcc += tmp;
                            }
                            currentAcc /= static_cast<double>(speeds.size());
                            if (linear || lastAcc > 0.3)
                            {
                                // if speeds increasing linearly / recently passed a minima with a fairly large current acc.
                                // any other dynamicsshape would give jump to zero acc. initially => undesired behaviour
                                lsa->transition_.shape_ = OSCPrivateAction::DynamicsShape::LINEAR;
                            }
                            else if (lastAcc < 0.3)
                            {
                                // Good solution when close to an minima in sine-like or cubic target behaviour
                                // Good enough when acceleration is negative
                                lsa->transition_.shape_ = OSCPrivateAction::DynamicsShape::CUBIC;
                            }
                            lsa->transition_.SetParamTargetVal(currentDist);

                            LongSpeedAction::TargetAbsolute* target_abs = new LongSpeedAction::TargetAbsolute;
                            target_abs->value_                          = trgSpeed;
                            lsa->target_.reset(target_abs);

                            // get sa's event and add lsa action to it
                            std::vector<Event*> events = sa->object_->getEvents();
                            for (size_t j = 0; j < events.size(); j++)
                            {
                                // Find correct event
                                for (size_t k = 0; k < events[j]->action_.size(); k++)
                                {
                                    if (activeActions[i] == events[j]->action_[k])
                                    {
                                        events[j]->action_.push_back(static_cast<OSCAction*>(lsa));
                                        activeActions.erase(activeActions.begin() + i);
                                        activeActions.push_back(lsa);
                                    }
                                }
                            }
                            sa->End();
                            lsa->Start(scenario_engine_->getSimulationTime());
                            LOG("Replacing the SynchronizeAction (with final speed) with an absolute target LongSpeedAction and target value: %lf",
                                trgSpeed);
                        }
                        else if (sa->mode_ == SynchronizeAction::SynchMode::MODE_NON_LINEAR)
                        {
                            // We are currently acc./dec. to reach an apex, this is most likely not the unwanted behaviour to switch from
                        }
                        else if (sa->mode_ == SynchronizeAction::SynchMode::MODE_WAITING)
                        {
                            // Ego is/was standing still for unknown reason, if we were to switch here it's most likely incorrect => do nothing
                        }
                        else if (sa->mode_ == SynchronizeAction::SynchMode::MODE_STOP_IMMEDIATELY)
                        {
                            // We assume that we want to switch close to the end point, this mode only occurs before the halfway point
                        }
                    }
                    else
                    {
                        double           currentSpeed = sa->object_->GetSpeed();
                        LongSpeedAction* lsa          = new LongSpeedAction(nullptr);
                        lsa->object_                  = object_;
                        lsa->transition_.shape_       = OSCPrivateAction::DynamicsShape::LINEAR;
                        lsa->transition_.dimension_   = OSCPrivateAction::DynamicsDimension::RATE;
                        // Assume normal vehicle operation => low max acceleration of 3 m/s^2
                        lsa->transition_.SetParamTargetVal(3);
                        LongSpeedAction::TargetAbsolute* target_abs = new LongSpeedAction::TargetAbsolute;
                        target_abs->value_                          = currentSpeed;
                        lsa->target_.reset(target_abs);

                        // get sa's event and add lsa action to it
                        std::vector<Event*> events = sa->object_->getEvents();
                        for (size_t j = 0; j < events.size(); j++)
                        {
                            // Ensure it's correct event
                            for (size_t k = 0; k < events[j]->action_.size(); k++)
                            {
                                if (activeActions[i] == events[j]->action_[k])
                                {
                                    events[j]->action_.push_back(static_cast<OSCAction*>(lsa));
                                    activeActions.erase(activeActions.begin() + i);
                                    activeActions.push_back(lsa);
                                }
                            }
                        }
                        sa->End();
                        lsa->Start(scenario_engine_->getSimulationTime());
                        LOG("Replacing the SynchronizeAction (no final speed) with an absolute target LongSpeedAction and target value: %lf",
                            currentSpeed);
                    }
                }
            }
            else if (activeActions[i]->action_type_ == OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY)
            {
                // Trajectory frozen at action start, will not need to be handled
            }
        }
    }

    speeds.emplace_back(timeStep, object_->GetSpeed());
    while (speeds.size() > 5)
    {
        speeds.erase(speeds.begin());
    }

    prev_ego_speed    = egoSpeed;
    prev_target_speed = object_->GetSpeed();

    gateway_->reportObject(object_->id_,
                           object_->name_,
                           static_cast<int>(object_->type_),
                           object_->category_,
                           object_->role_,
                           object_->model_id_,
                           object_->model3d_,
                           object_->GetControllerTypeActiveOnDomain(ControlDomains::DOMAIN_LONG),
                           object_->boundingbox_,
                           static_cast<int>(object_->scaleMode_),
                           object_->visibilityMask_,
                           0.0,
                           object_->speed_,
                           object_->wheel_angle_,
                           object_->wheel_rot_,
                           object_->rear_axle_.positionZ,
                           object_->front_axle_.positionX,
                           object_->front_axle_.positionZ,
                           &object_->pos_);

    Controller::Step(timeStep);
}

int ControllerRel2Abs::Activate(ControlActivationMode lat_activation_mode,
                                ControlActivationMode long_activation_mode,
                                ControlActivationMode light_activation_mode,
                                ControlActivationMode anim_activation_mode)
{
#ifdef CONTROLLER_REL2ABS_DEBUG
    logData.open("LogData.csv");
    logData << "Time,Error,X_act,Pred_Time,X_pred,V_act,V_pred,V_error,\n";
#endif
    timestamp = -1000;  // So that presim occurs right away

    ego_obj           = -1;
    prev_ego_speed    = 0;
    prev_target_speed = 0;
    switchNow         = false;
    csv_iter          = 0;

    pred_timestep      = 0.1;
    pred_nbr_timesteps = pred_horizon / pred_timestep;

    return Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);
}

void ControllerRel2Abs::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}

// Method for copying position and saving speed for object. Contains pointer to object which Returns struct with action pointer,
// speed and position objects
void ControllerRel2Abs::CopyPosition(Object* object, position_copy* obj_copy)
{
    roadmanager::Position* saved_pos = new roadmanager::Position();

    *saved_pos = object->pos_;  // Copy object->pos_ into newly alloced mem. pointed at by saved_pos

    double saved_speed = object->speed_;

    obj_copy->object    = object;
    obj_copy->pos       = saved_pos;
    obj_copy->speed     = saved_speed;
    obj_copy->dirtyBits = object->GetDirtyBitMask();
}