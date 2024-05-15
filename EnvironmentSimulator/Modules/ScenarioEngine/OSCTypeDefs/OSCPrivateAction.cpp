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

#define _USE_MATH_DEFINES
#include <math.h>

#include "OSCPrivateAction.hpp"
#include "ScenarioEngine.hpp"

#define MAX_DECELERATION                -8.0
#define LONGITUDINAL_DISTANCE_THRESHOLD 0.1

using namespace scenarioengine;

// Equations used in TransitionDynamics
// ------------------------------------
// Step: Trivial
// Linear: Trivial
// Sinusoidal:
//		f(x) = a-b*(cos(pi*x/c)-1)/2
//		f'(x) = pi*b*sin(pi*x/c)/(2*c)
//		Slope peaks midway, at x=c/2 =>
//		f'peak = pi*b/(2*c)
//		f''(x): b*pi^2*cos(pi*x/c)/(2c^2)
//		Acceleration peaks at endpoints (x=0, x=c)
//      f''peak: f''(0) = pi^2*b/(2c^2), f''(c) = -b*pi^2/(2c^2)
//      plot: "https://www.desmos.com/calculator/ikrmaen0mm"
//		f': "https://www.wolframalpha.com/input/?i=d/dx(a-b*(cos(pi*x/c)-1)/2)"
//		f'peak: "https://www.wolframalpha.com/input/?i=d/dx(a-b*(cos(pi*x/c)-1)/2),x=c/2"
//		f'': "https://www.wolframalpha.com/input/?i=d/dx(pi*b*sin(pi*x/c)/(2*c))"
// Cubic:
//		f(x) = a+3b(x/c)^2-2b(x/c)^3
//		f'(x) = 6bx(c-x)/c^3
//		Slope (rate) peaks midway, at x=c/2 =>
//      f'peak = f'(c/2) = 3*b/(2*c)
//      f''(x) = 6b(c-2x)/c^3
//		Acceleration peaks at endpoints (x=0, x=c)
//      f''peak: f''(0) = 6b/c^2, f''(c) = -6b/c^2
//		plot: "https://www.desmos.com/calculator/6t6wbeeos8"
//		f': "https://www.wolframalpha.com/input/?i=d/dx(a%2B3b(x/c)^2-2b(x/c)^3)"
//		f'peak: "https://www.wolframalpha.com/input/?i=d/dx(a%2B3b(x/c)^2-2b(x/c)^3),x=c/2"
//      f'': Trivial

double OSCPrivateAction::TransitionDynamics::EvaluatePrimPeak()
{
    if (dimension_ == DynamicsDimension::RATE)
    {
        return GetRate();
    }
    else
    {
        if (shape_ == DynamicsShape::STEP)
        {
            return LARGE_NUMBER * SIGN(GetTargetVal() - GetStartVal());
        }
        else if (shape_ == DynamicsShape::LINEAR)
        {
            return (GetTargetVal() - GetStartVal()) / AVOID_ZERO(GetParamTargetVal());
        }
        else if (shape_ == DynamicsShape::SINUSOIDAL)
        {
            return M_PI * (GetTargetVal() - GetStartVal()) / (2 * AVOID_ZERO(GetParamTargetVal()));
        }
        else if (shape_ == DynamicsShape::CUBIC)
        {
            return 1.5 * (GetTargetVal() - GetStartVal()) / AVOID_ZERO(GetParamTargetVal());
        }
        else
        {
            LOG("Invalid Dynamics shape: %d", shape_);
        }
    }

    return 0;
}

double OSCPrivateAction::TransitionDynamics::GetTargetParamValByPrimPeak(double prim_peak)
{
    if (shape_ == DynamicsShape::STEP)
    {
        return 0.0;
    }
    else if (shape_ == DynamicsShape::LINEAR)
    {
        return (GetTargetVal() - GetStartVal()) / prim_peak;
    }
    else if (shape_ == DynamicsShape::SINUSOIDAL)
    {
        return M_PI * (GetTargetVal() - GetStartVal()) / (2 * prim_peak);
    }
    else if (shape_ == DynamicsShape::CUBIC)
    {
        return 1.5 * (GetTargetVal() - GetStartVal()) / prim_peak;
    }
    else
    {
        LOG("Invalid Dynamics shape: %d", shape_);
    }

    return 0.0;
}

double OSCPrivateAction::TransitionDynamics::GetTargetParamValByPrimPrimPeak(double prim_prim_peak)
{
    if (shape_ == DynamicsShape::STEP)
    {
        return 0.0;
    }
    else if (shape_ == DynamicsShape::LINEAR)
    {
        // Acceleration is infinite at start and end, anything else should result in flat line
        // Just to have something reasonable, re-use acc equation from CUBIC case
        return sqrt(6 * abs(GetTargetVal() - GetStartVal()) / prim_prim_peak);
    }
    else if (shape_ == DynamicsShape::SINUSOIDAL)
    {
        // pi*sqrt(abs(b)/(2*prim_prim_peak))
        return M_PI * sqrt(abs(GetTargetVal() - GetStartVal()) / (2 * prim_prim_peak));
    }
    else if (shape_ == DynamicsShape::CUBIC)
    {
        // sqrt(6*abs(b)/y)
        return sqrt(6 * abs(GetTargetVal() - GetStartVal()) / prim_prim_peak);
    }
    else
    {
        LOG("Invalid Dynamics shape: %d", shape_);
    }

    return 0.0;
}

double OSCPrivateAction::TransitionDynamics::EvaluatePrim()
{
    if (shape_ == DynamicsShape::STEP)
    {
        return LARGE_NUMBER * SIGN(GetTargetVal() - GetStartVal());
    }
    else if (shape_ == DynamicsShape::LINEAR)
    {
        return (GetTargetVal() - GetStartVal()) / AVOID_ZERO(GetParamTargetVal());
    }
    else if (shape_ == DynamicsShape::SINUSOIDAL)
    {
        return M_PI * (GetTargetVal() - GetStartVal()) * sin(M_PI * param_val_ / GetParamTargetVal()) / (2 * AVOID_ZERO(GetParamTargetVal()));
    }
    else if (shape_ == DynamicsShape::CUBIC)
    {
        return 6 * (GetTargetVal() - GetStartVal()) * (GetParamTargetVal() - param_val_) * param_val_ / pow(AVOID_ZERO(GetParamTargetVal()), 3);
    }
    else
    {
        LOG("Invalid Dynamics shape: %d", shape_);
    }

    return 0;
}

double OSCPrivateAction::TransitionDynamics::EvaluateScaledPrim()
{
    return EvaluatePrim() / AVOID_ZERO(scale_factor_);
}

double OSCPrivateAction::TransitionDynamics::Evaluate(DynamicsShape shape)
{
    if (shape == DynamicsShape::SHAPE_UNDEFINED)
    {
        shape = shape_;
    }

    if (shape == DynamicsShape::STEP)
    {
        return GetTargetVal();
    }
    else if (shape == DynamicsShape::LINEAR)
    {
        return GetStartVal() + GetParamVal() * (GetTargetVal() - GetStartVal()) / (AVOID_ZERO(GetParamTargetVal()));
    }
    else if (shape == DynamicsShape::SINUSOIDAL)
    {
        return GetStartVal() - (GetTargetVal() - GetStartVal()) * (cos(M_PI * GetParamVal() / AVOID_ZERO(GetParamTargetVal())) - 1) / 2;
    }
    else if (shape == DynamicsShape::CUBIC)
    {
        return GetStartVal() + (GetTargetVal() - GetStartVal()) * pow(GetParamVal() / AVOID_ZERO(GetParamTargetVal()), 2) *
                                   (3 - 2 * GetParamVal() / AVOID_ZERO(GetParamTargetVal()));
    }
    else
    {
        LOG("Invalid Dynamics shape: %d", shape);
    }

    return GetTargetVal();
}

void OSCPrivateAction::TransitionDynamics::Reset()
{
    scale_factor_ = 1.0;
    param_val_    = 0.0;
    start_val_    = 0.0;
    target_val_   = 0.0;
}

int OSCPrivateAction::TransitionDynamics::Step(double delta_param_val)
{
    param_val_ += delta_param_val / scale_factor_;

    return 0;
}

void OSCPrivateAction::TransitionDynamics::SetStartVal(double start_val)
{
    start_val_ = start_val;
    UpdateRate();
}

void OSCPrivateAction::TransitionDynamics::SetTargetVal(double target_val)
{
    target_val_ = target_val;
    UpdateRate();
}

void OSCPrivateAction::TransitionDynamics::SetParamTargetVal(double target_value)
{
    if (dimension_ != DynamicsDimension::RATE)
    {
        param_target_val_ = AVOID_ZERO(target_value);
    }
    else
    {
        // Interpret the target parameter value as rate
        SetRate(target_value);
    }
}

void OSCPrivateAction::TransitionDynamics::SetMaxRate(double max_rate)
{
    // Check max rate
    double peak_rate = EvaluatePrimPeak();

    if (abs(peak_rate) > abs(max_rate))
    {
        scale_factor_ = abs(peak_rate) / abs(AVOID_ZERO(max_rate));
    }
    else
    {
        scale_factor_ = 1.0;
    }
}

void OSCPrivateAction::TransitionDynamics::SetRate(double rate)
{
    // Adapt sign
    rate_ = rate;

    UpdateRate();
}

void OSCPrivateAction::TransitionDynamics::UpdateRate()
{
    // Adapt sign
    rate_ = SIGN(GetTargetVal() - GetStartVal()) * abs(rate_);

    if (dimension_ == DynamicsDimension::RATE)
    {
        // Find out parameter range from rate
        param_target_val_ = AVOID_ZERO(GetTargetParamValByPrimPeak(rate_));
    }
}

void AssignRouteAction::Start(double simTime)
{
    route_->setObjName(object_->GetName());
    object_->pos_.SetRoute(route_);
    object_->SetDirtyBits(Object::DirtyBit::ROUTE);

    OSCAction::Start(simTime);
}

void AssignRouteAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::End();
}

void AssignRouteAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }
    for (size_t i = 0; i < route_->minimal_waypoints_.size(); i++)
    {
        route_->minimal_waypoints_[i].ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
    }
}

void FollowTrajectoryAction::Start(double simTime)
{
    OSCAction::Start(simTime);

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // lateral motion controlled elsewhere
        // other action or controller already updated lateral dimension of object
        // potentially longitudinal dimension could be updated separatelly - but skip that for now
        return;
    }

    reverse_ = (object_->GetSpeed() < 0.0);

    traj_->Freeze(following_mode_, object_->GetSpeed(), &object_->pos_);
    object_->pos_.SetTrajectory(traj_.get());

    object_->pos_.SetTrajectoryS(initialDistanceOffset_);
    time_ = traj_->GetTimeAtS(initialDistanceOffset_);

    // establish initial position
    Move(simTime, 0.0);

    // establish initial speed if time reference is defined
    if (timing_domain_ != TimingDomain::NONE)
    {
        object_->SetSpeed(traj_->GetSpeedAtS(object_->pos_.GetTrajectoryS()));
    }
}

void FollowTrajectoryAction::End()
{
    OSCAction::End();

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        return;
    }

    // Disconnect trajectory
    object_->pos_.SetTrajectory(0);
}

void FollowTrajectoryAction::Step(double simTime, double dt)
{
    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // lateral motion controlled elsewhere
        // other action or controller already updated lateral dimension of object
        // potentially longitudinal dimension could be updated separatelly - but skip that for now
        return;
    }

    // signal that an action owns control
    object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::SPEED);

    if (!object_->IsGhost() && simTime < 0.0)
    {
        // Only ghosts are moving up to and including time == 0
        return;
    }

    int    dir   = reverse_ ? -1 : 1;
    double old_s = object_->pos_.GetTrajectoryS();

    // Adjust absolute time for any ghost headstart
    double timeOffset = (timing_domain_ == TimingDomain::TIMING_ABSOLUTE && object_->IsGhost()) ? object_->GetHeadstartTime() : 0.0;

    Move(simTime, dt);

    // Check end conditions:
    // Trajectories with no time stamps:
    //     closed trajectories have no end
    //     open trajectories simply ends when s >= length of trajectory
    // Trajectories with time stamps:
    //     always ends when time >= trajectory duration (last timestamp)
    if (((timing_domain_ == TimingDomain::NONE && !traj_->closed_) && dt > 0.0 &&
         ((dir * object_->GetSpeed() > 0.0 && object_->pos_.GetTrajectoryS() > (traj_->GetLength() - SMALL_NUMBER)) ||
          (dir * object_->GetSpeed() < 0.0 && object_->pos_.GetTrajectoryS() < SMALL_NUMBER))) ||
        (timing_domain_ != TimingDomain::NONE && time_ + timeOffset >= traj_->GetStartTime() + traj_->GetDuration()))
    {
        // Reached end of trajectory
        double remaningDistance = 0.0;
        if (timing_domain_ == TimingDomain::NONE && !traj_->closed_ && object_->pos_.GetTrajectoryS() > (traj_->GetLength() - SMALL_NUMBER))
        {
            // Move the remaning distance along road at current lane offset
            remaningDistance = object_->speed_ * dt - (object_->pos_.GetTrajectoryS() - old_s);
        }
        else if (timing_domain_ != TimingDomain::NONE && time_ + timeOffset >= traj_->GetStartTime() + traj_->GetDuration())
        {
            // Move the remaning distance along road at current lane offset
            double remaningTime = time_ + timeOffset - (traj_->GetStartTime() + traj_->GetDuration());
            remaningDistance    = remaningTime * object_->speed_;
        }

        // Move the remainder of distance along the current heading
        double dx = remaningDistance * cos(object_->pos_.GetH());
        double dy = remaningDistance * sin(object_->pos_.GetH());

        object_->pos_.SetInertiaPos(object_->pos_.GetX() + dx,
                                    object_->pos_.GetY() + dy,
                                    std::nan(""),   // don't update Z
                                    std::nan(""),   // don't update Heading
                                    std::nan(""),   // don't update Pitch
                                    std::nan(""));  // don't update Roll

        End();
    }
}

void scenarioengine::FollowTrajectoryAction::Move(double simTime, double dt)
{
    int    dir   = reverse_ ? -1 : 1;
    double old_s = object_->pos_.GetTrajectoryS();

    // Adjust absolute time for any ghost headstart
    double timeOffset = (timing_domain_ == TimingDomain::TIMING_ABSOLUTE && object_->IsGhost()) ? object_->GetHeadstartTime() : 0.0;

    // Move along trajectory
    if (
        // Ignore any timing info in trajectory
        timing_domain_ == TimingDomain::NONE ||
        // Speed is controlled elsewhere - just follow trajectory with current speed
        (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LONG))))
    {
        object_->pos_.MoveTrajectoryDS(dir * object_->speed_ * dt);
    }
    else if (timing_domain_ == TimingDomain::TIMING_RELATIVE)
    {
        time_ += timing_scale_ * dt;
        object_->pos_.SetTrajectoryPosByTime(time_ + timing_offset_);

        // calculate and update actual speed only while not reached end of trajectory,
        // since the movement is based on remaining length of trajectory, not speed
        if (time_ + timing_offset_ < traj_->GetStartTime() + traj_->GetDuration() + SMALL_NUMBER)
        {
            if (dt > SMALL_NUMBER)  // only update speed if some time has passed
            {
                object_->SetSpeed((object_->pos_.GetTrajectoryS() - old_s) / dt);
            }
        }
    }
    else if (timing_domain_ == TimingDomain::TIMING_ABSOLUTE)
    {
        if (object_->IsGhost() || simTime > -SMALL_NUMBER)
        {
            time_ = (simTime + dt) * timing_scale_;
        }

        object_->pos_.SetTrajectoryPosByTime(time_ + timeOffset + timing_offset_);

        if ((dt > SMALL_NUMBER) &&  // skip speed update if timestep is zero
            (time_ + timeOffset < traj_->GetStartTime() + traj_->GetDuration() + SMALL_NUMBER))
        {
            // don't calculate and update actual speed when reached end of trajectory,
            // since the movement is based on remaining length of trajectory, not speed
            object_->SetSpeed((object_->pos_.GetTrajectoryS() - old_s) / MAX(SMALL_NUMBER, dt));
        }
    }

    // align heading to driving direction
    if (reverse_)
    {
        object_->pos_.SetHeading(GetAngleInInterval2PI(object_->pos_.GetH() + M_PI));
    }
}

void FollowTrajectoryAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }
    if (traj_->shape_->type_ == roadmanager::Shape::ShapeType::CLOTHOID)
    {
        roadmanager::ClothoidShape* cl = static_cast<roadmanager::ClothoidShape*>(traj_->shape_.get());
        cl->pos_.ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
    }
    else if (traj_->shape_->type_ == roadmanager::Shape::ShapeType::POLYLINE)
    {
        roadmanager::PolyLineShape* pl = static_cast<roadmanager::PolyLineShape*>(traj_->shape_.get());
        for (size_t i = 0; i < pl->vertex_.size(); i++)
        {
            pl->vertex_[i].pos_.ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
        }
    }
}

void AcquirePositionAction::Start(double simTime)
{
    // Resolve route
    route_.reset(new roadmanager::Route);
    route_->setName("AcquirePositionRoute");
    route_->setObjName(object_->GetName());

    route_->AddWaypoint(&object_->pos_);
    route_->AddWaypoint(target_position_);

    object_->pos_.SetRoute(route_);
    object_->SetDirtyBits(Object::DirtyBit::ROUTE);

    OSCAction::Start(simTime);

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // lateral motion controlled elsewhere
        return;
    }
}

void AcquirePositionAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::End();
}

void AcquirePositionAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }
}

void AssignControllerAction::Start(double simTime)
{
    if (controller_)
    {
        if (object_ != nullptr)
        {
            object_->AssignController(controller_);
        }

        if (controller_)
        {
            if (!controller_->Active())
            {
                if (lat_activation_mode_ != ControlActivationMode::UNDEFINED || long_activation_mode_ != ControlActivationMode::UNDEFINED ||
                    light_activation_mode_ != ControlActivationMode::UNDEFINED || anim_activation_mode_ != ControlActivationMode::UNDEFINED)
                {
                    controller_->Activate(lat_activation_mode_, long_activation_mode_, light_activation_mode_, anim_activation_mode_);
                    LOG("Controller %s activated (lat %s, long %s, light %s, anim %s), domain mask=0x%X",
                        controller_->GetName().c_str(),
                        DomainActivation2Str(lat_activation_mode_).c_str(),
                        DomainActivation2Str(long_activation_mode_).c_str(),
                        DomainActivation2Str(light_activation_mode_).c_str(),
                        DomainActivation2Str(anim_activation_mode_).c_str(),
                        controller_->GetActiveDomains());
                }
            }
            else
            {
                LOG("Controller %s already active (domainmask 0x%X), should not happen when just assigned!",
                    controller_->GetName().c_str(),
                    controller_->GetActiveDomains());
            }
        }
    }

    OSCAction::Start(simTime);
}

void ActivateControllerAction::Start(double simTime)
{
    if (object_->controllers_.size() == 0)
    {
        LOG("No controller assigned to object %s!", object_->name_.c_str());
    }
    else
    {
        controller_ = nullptr;

        if (ctrl_name_.empty())
        {
            controller_ = object_->controllers_.back();

            if (scenarioEngine_->GetScenarioReader()->GetVersionMinor() > 2)
            {
                LOG("Warning: No controller name given for activation on object %s, %s: %s",
                    object_->name_.c_str(),
                    object_->GetNrOfAssignedControllers() > 1 ? "pick most recently assigned" : "using assigned controller",
                    controller_->GetName().c_str());
            }
        }
        else
        {
            controller_ = object_->GetController(ctrl_name_);
            if (controller_ == nullptr)
            {
                LOG("Error: Controller %s is not assigned to object %s", ctrl_name_.c_str(), object_->GetName().c_str());
            }
        }

        if (controller_ != nullptr)
        {
            // first deactivate any controller active on the requested domain(s)
            Controller* ctrl = nullptr;
            if (long_activation_mode_ == ControlActivationMode::ON &&
                (ctrl = object_->GetControllerActiveOnDomain(ControlDomains::DOMAIN_LONG)) != nullptr)
            {
                LOG("Deactivating conflicting ctrl %s on domain %s",
                    ctrl->GetName().c_str(),
                    ControlDomain2Str(static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)).c_str());
                ctrl->DeactivateDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LONG));
            }
            if (lat_activation_mode_ == ControlActivationMode::ON &&
                (ctrl = object_->GetControllerActiveOnDomain(ControlDomains::DOMAIN_LAT)) != nullptr)
            {
                LOG("Deactivating conflicting ctrl %s on domain %s",
                    ctrl->GetName().c_str(),
                    ControlDomain2Str(static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)).c_str());
                ctrl->DeactivateDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LAT));
            }
            if (anim_activation_mode_ == ControlActivationMode::ON &&
                (ctrl = object_->GetControllerActiveOnDomain(ControlDomains::DOMAIN_ANIM)) != nullptr)
            {
                LOG("Deactivating conflicting ctrl %s on domain %s",
                    ctrl->GetName().c_str(),
                    ControlDomain2Str(static_cast<unsigned int>(ControlDomains::DOMAIN_ANIM)).c_str());
                ctrl->DeactivateDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_ANIM));
            }
            if (light_activation_mode_ == ControlActivationMode::ON &&
                (ctrl = object_->GetControllerActiveOnDomain(ControlDomains::DOMAIN_LIGHT)) != nullptr)
            {
                LOG("Deactivating conflicting ctrl %s on domain %s",
                    ctrl->GetName().c_str(),
                    ControlDomain2Str(static_cast<unsigned int>(ControlDomains::DOMAIN_LIGHT)).c_str());
                ctrl->DeactivateDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LIGHT));
            }

            if (controller_->Activate(lat_activation_mode_, long_activation_mode_, light_activation_mode_, anim_activation_mode_) == 0)
            {
                object_->SetDirtyBits(Object::DirtyBit::CONTROLLER);
                LOG("Controller %s active on domains: %s (mask=0x%x)",
                    controller_->GetName().c_str(),
                    ControlDomain2Str(controller_->GetActiveDomains()).c_str(),
                    controller_->GetActiveDomains());
                OSCAction::Start(simTime);
                End();  // action ends immediately
            }
            else
            {
                LOG("Error: Failed to activate controller %s assigned to object %s!", controller_->GetName().c_str(), object_->GetName().c_str());
            }
        }
    }
}

void ActivateControllerAction::End()
{
    // Make sure heading is aligned with road driving direction
    object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
    OSCAction::End();
}

void LatLaneChangeAction::Start(double simTime)
{
    OSCAction::Start(simTime);
    int target_lane_id_ = 0;

    transition_.Reset();

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // lateral motion controlled elsewhere
        return;
    }

    if (target_->type_ == Target::Type::ABSOLUTE_LANE)
    {
        target_lane_id_ = target_->value_;
    }
    else if (target_->type_ == Target::Type::RELATIVE_LANE)
    {
        // Find out target lane relative referred vehicle
        Object* ref_entity = (static_cast<TargetRelative*>(target_.get()))->object_;
        if (ref_entity != nullptr)
        {
            target_lane_id_ = ref_entity->pos_.GetLaneId() + target_->value_ * (IsAngleForward(ref_entity->pos_.GetHRelative()) ? 1 : -1);

            if (target_lane_id_ == 0 || SIGN(ref_entity->pos_.GetLaneId()) != SIGN(target_lane_id_))
            {
                // Skip reference lane (id == 0)
                target_lane_id_ = SIGN(target_lane_id_ - ref_entity->pos_.GetLaneId()) * (abs(target_lane_id_) + 1);
            }
        }
        else
        {
            LOG("LaneChange RelativeTarget ref entity not found!");
        }
    }

    // Reset orientation, align to road
    object_->pos_.SetHeadingRelativeRoadDirection(0.0);
    object_->pos_.SetPitchRelative(0.0);
    object_->pos_.SetRollRelative(0.0);

    // Set initial state
    object_->pos_.ForceLaneId(target_lane_id_);
    internal_pos_ = object_->pos_;

    // Make offsets agnostic to lane sign
    transition_.SetStartVal(SIGN(object_->pos_.GetLaneId()) * object_->pos_.GetOffset());
    transition_.SetTargetVal(SIGN(target_lane_id_) * target_lane_offset_);
}

void LatLaneChangeAction::Step(double simTime, double dt)
{
    double offset_agnostic = internal_pos_.GetOffset() * SIGN(internal_pos_.GetLaneId());

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // lateral motion controlled elsewhere
        return;
    }

    // signal that an action owns control
    object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);

    if (!object_->IsGhost() && simTime < 0.0)
    {
        // Only ghosts are moving up to and including time == 0
        return;
    }

    if (abs(object_->GetSpeed()) < SMALL_NUMBER)
    {
        return;
    }

    double rate       = transition_.EvaluateScaledPrim();
    double step_len   = fabs(object_->speed_) * dt;  // travel distance total
    double delta_long = 0.0;
    if (transition_.dimension_ == DynamicsDimension::TIME || transition_.dimension_ == DynamicsDimension::RATE)
    {
        // ds^2 = steplen^2 - dt^2
        double delta_lat = dt * fabs(rate);
        delta_long       = sqrt(pow(step_len, 2.0) - pow(MIN(delta_lat, step_len), 2.0));

        if (delta_lat < step_len + SMALL_NUMBER)
        {
            // requested delta lateral motion is less than total step length
            transition_.Step(dt);
            offset_agnostic   = transition_.Evaluate();
            heading_agnostic_ = GetAngleInInterval2PI(atan2(SIGN(object_->speed_) * SIGN(rate) * delta_lat, delta_long));
        }
        else
        {
            // requested lateral motion too large wrt speed, skip long motion and limit lateral one
            // fall back to linear interpolation
            transition_.Step(step_len * transition_.GetParamTargetVal() / abs(transition_.GetTargetVal() - transition_.GetStartVal()));
            if (!NEAR_NUMBERS(heading_agnostic_, SIGN(object_->speed_) * (M_PI_2 - SMALL_NUMBER)) && transition_.shape_ != DynamicsShape::LINEAR)
            {
                LOG("LatLaneChangeAction: Limiting lateral motion and falling back to linear interpolation");
            }
            offset_agnostic   = transition_.Evaluate(DynamicsShape::LINEAR);
            heading_agnostic_ = SIGN(object_->speed_) * SIGN(rate) * (M_PI_2 - SMALL_NUMBER);  // avoid ambiguous driving direction
        }
    }
    else if (transition_.dimension_ == DynamicsDimension::DISTANCE)
    {
        delta_long = sqrt(pow(step_len, 2.0) / (1.0 + pow(rate, 2.0)));
        transition_.Step(delta_long);
        offset_agnostic   = transition_.Evaluate();
        heading_agnostic_ = atan(rate);  // heading for the step movement
    }
    else
    {
        LOG("Unexpected, unsupported transition dimension: %d", transition_.dimension_);
    }

    // Restore position to target lane and new offset
    object_->pos_.SetLanePos(internal_pos_.GetTrackId(),
                             internal_pos_.GetLaneId(),
                             internal_pos_.GetS(),
                             offset_agnostic * SIGN(internal_pos_.GetLaneId()));

    if (transition_.shape_ == DynamicsShape::STEP)
    {
        // not for step shape, since it is not a continuous function. Maintain longitudinal motion
        delta_long = step_len;
    }

    double ds = object_->pos_.DistanceToDS(SIGN(object_->speed_) * delta_long);  // find correspondning delta s along road reference line

    roadmanager::Position::ReturnCode retval = roadmanager::Position::ReturnCode::OK;
    retval = object_->pos_.MoveAlongS(ds, 0.0, -1.0, false, roadmanager::Position::MoveDirectionMode::HEADING_DIRECTION, true);
    internal_pos_.SetLanePos(object_->pos_.GetTrackId(), object_->pos_.GetLaneId(), object_->pos_.GetS(), object_->pos_.GetOffset());

    if (object_->pos_.GetRoute())
    {
        // Check whether updated position still is on the route, i.e. same road ID, or not
        if (!object_->pos_.IsInJunction() && !internal_pos_.GetTrackId() && object_->pos_.GetTrackId() != internal_pos_.GetTrackId())
        {
            LOG("Warning/Info: LaneChangeAction moved away from route (track id %d -> track id %d), disabling route",
                object_->pos_.GetTrackId(),
                internal_pos_.GetTrackId());
            object_->pos_.SetRoute(nullptr);
            object_->SetDirtyBits(Object::DirtyBit::ROUTE);
        }
    }

    if (transition_.GetParamVal() > transition_.GetParamTargetVal() - SMALL_NUMBER ||
        // Close enough?
        fabs(offset_agnostic - transition_.GetTargetVal()) < SMALL_NUMBER ||
        // Passed target value?
        (transition_.GetParamVal() > 0 &&
         SIGN(offset_agnostic - transition_.GetTargetVal()) != SIGN(transition_.GetStartVal() - transition_.GetTargetVal())))
    {
        OSCAction::End();
        object_->pos_.SetHeadingRelativeRoadDirection(0);
    }
    else
    {
        object_->pos_.SetHeadingRelativeRoadDirection((IsAngleForward(object_->pos_.GetHRelative()) ? 1 : -1) * SIGN(object_->pos_.GetLaneId()) *
                                                      heading_agnostic_);
    }
    object_->pos_.EvaluateZHPR(object_->pos_.GetMode(roadmanager::Position::PosModeType::UPDATE));

    if (retval == roadmanager::Position::ReturnCode::ERROR_END_OF_ROAD)
    {
        object_->SetSpeed(0.0);
    }

    if (!(object_->pos_.GetRoute() && object_->pos_.GetRoute()->IsValid()))
    {
        // Attach object position to closest road and lane, look up via inertial coordinates
        object_->pos_.XYZ2TrackPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), roadmanager::Position::PosMode::Z_ABS);
    }

    object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::SPEED);
}

void LatLaneChangeAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }

    if (target_->type_ == Target::Type::RELATIVE_LANE)
    {
        if ((static_cast<TargetRelative*>(target_.get()))->object_ == obj1)
        {
            (static_cast<TargetRelative*>(target_.get()))->object_ = obj2;
        }
    }
}

void LatLaneOffsetAction::Start(double simTime)
{
    OSCAction::Start(simTime);
    transition_.Reset();

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // lateral motion controlled elsewhere
        return;
    }

    if (target_->type_ == Target::Type::ABSOLUTE_OFFSET)
    {
        transition_.SetTargetVal(SIGN(object_->pos_.GetLaneId()) * target_->value_);
    }
    else if (target_->type_ == Target::Type::RELATIVE_OFFSET)
    {
        // Register what lane action object belongs to
        int lane_id = object_->pos_.GetLaneId();

        // Find out target position based on the referred object
        roadmanager::Position refpos = (static_cast<TargetRelative*>(target_.get()))->object_->pos_;
        refpos.SetTrackPos(refpos.GetTrackId(), refpos.GetS(), refpos.GetT() + target_->value_ * (IsAngleForward(refpos.GetHRelative()) ? 1 : -1));

        // Transform target position into lane position based on current lane id
        refpos.ForceLaneId(lane_id);

        // Target lane offset = target t value - t value of current lane (which is current t - current offset)
        transition_.SetTargetVal(SIGN(object_->pos_.GetLaneId()) * (refpos.GetT() - (object_->pos_.GetT() - object_->pos_.GetOffset())));
    }

    transition_.SetStartVal(SIGN(object_->pos_.GetLaneId()) * object_->pos_.GetOffset());
    transition_.SetParamTargetVal(transition_.GetTargetParamValByPrimPrimPeak(max_lateral_acc_));
}

void LatLaneOffsetAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;
    double offset_agnostic;

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // lateral motion controlled elsewhere
        return;
    }

    offset_agnostic = transition_.Evaluate();

    if (transition_.GetParamVal() > transition_.GetParamTargetVal() - SMALL_NUMBER ||
        // Close enough?
        fabs(offset_agnostic - transition_.GetTargetVal()) < SMALL_NUMBER ||
        // Passed target value?
        (transition_.GetParamVal() > 0 &&
         SIGN(offset_agnostic - transition_.GetTargetVal()) != SIGN(transition_.GetStartVal() - transition_.GetTargetVal())))
    {
        OSCAction::End();
        object_->pos_.SetLanePos(object_->pos_.GetTrackId(),
                                 object_->pos_.GetLaneId(),
                                 object_->pos_.GetS(),
                                 SIGN(object_->pos_.GetLaneId()) * transition_.GetTargetVal());
        object_->pos_.SetHeadingRelativeRoadDirection(0);
    }
    else
    {
        object_->pos_.SetLanePos(object_->pos_.GetTrackId(),
                                 object_->pos_.GetLaneId(),
                                 object_->pos_.GetS(),
                                 SIGN(object_->pos_.GetLaneId()) * offset_agnostic);

        // Convert rate (lateral-movment/time) to lateral-movement/long-movement
        double angle = atan(transition_.EvaluatePrim() / AVOID_ZERO(object_->GetSpeed()));
        object_->pos_.SetHeadingRelativeRoadDirection((IsAngleForward(object_->pos_.GetHRelative()) ? 1 : -1) * SIGN(object_->pos_.GetLaneId()) *
                                                      angle);
    }

    object_->SetDirtyBits(Object::DirtyBit::LATERAL);

    transition_.Step(dt);
}

void LatLaneOffsetAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }

    if (target_->type_ == Target::Type::RELATIVE_OFFSET)
    {
        if ((static_cast<TargetRelative*>(target_.get()))->object_ == obj1)
        {
            (static_cast<TargetRelative*>(target_.get()))->object_ = obj2;
        }
    }
}
double LongSpeedAction::TargetRelative::GetValue()
{
    if (!continuous_)
    {
        // sample relative object speed once
        if (!consumed_)
        {
            object_speed_ = object_->speed_;
            consumed_     = true;
        }
    }
    else
    {
        object_speed_ = object_->speed_;
    }

    if (value_type_ == ValueType::DELTA)
    {
        return object_speed_ + value_;
    }
    else if (value_type_ == ValueType::FACTOR)
    {
        return object_speed_ * value_;
    }
    else
    {
        LOG("Invalid value type: %d", value_type_);
    }

    return 0;
}

void LongSpeedAction::Start(double simTime)
{
    OSCAction::Start(simTime);
    transition_.Reset();
    target_speed_reached_ = false;

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)))
    {
        // longitudinal motion controlled elsewhere
        OSCAction::End();
        return;
    }

    transition_.SetStartVal(object_->GetSpeed());

    if (transition_.dimension_ == DynamicsDimension::DISTANCE)
    {
        // Convert to time, since speed shape is expected over time, not distance (as in lane change case)
        // special cased when speed changes sign:

        double v0   = transition_.GetStartVal();
        double v1   = target_->GetValue();
        double dist = transition_.GetParamTargetVal();

        if (abs(v1 - v0) < SMALL_NUMBER)
        {
            // no change
            transition_.SetParamTargetVal(0.0);
        }
        else if (abs(v0) > SMALL_NUMBER && abs(v1) > SMALL_NUMBER && SIGN(v0) != SIGN(v1))
        {
            // sign of speed changes, add two parts divided by speed = 0
            // find relation dist0 / (dist0 + dist1) = v0^2 / (v0^2 + v1^2)
            double dist_factor = abs(pow(v0, 2) / (pow(v0, 2) + pow(v1, 2)));

            // Find displacement of part 1
            // d0 = (v0 * t0) / 2 => t0 = (2 * d0) / v0
            double d0 = dist_factor * dist;

            // calculate duration of part 1
            double t0 = 2.0 * d0 / abs(v0);

            // calculate duration of remaning part 2
            double t1 = 2.0 * (dist - d0) / abs(v1);

            transition_.SetParamTargetVal(t0 + t1);
        }
        else  // change of speed without changing driving direction
        {
            // integrated distance = time(v_init + v_delta/2) = time(v_init + v_end)/2
            // => time = 2*distance/(v_init + v_end)
            transition_.SetParamTargetVal(2 * dist / (abs(v0 + v1)));
        }
    }

    transition_.SetTargetVal(target_->GetValue());

    // Set initial state
    object_->SetSpeed(transition_.Evaluate());
}

void LongSpeedAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;
    double new_speed = 0;

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)))
    {
        // longitudinal motion controlled elsewhere
        OSCAction::End();
        return;
    }

    // Get target speed, which might be dynamic (relative other entitity)
    transition_.SetTargetVal(ABS_LIMIT(target_->GetValue(), object_->performance_.maxSpeed));

    if (target_speed_reached_)
    {
        new_speed = target_->GetValue();
    }
    else
    {
        if (transition_.GetTargetVal() > transition_.GetStartVal())
        {
            // Acceleration
            transition_.SetMaxRate(object_->performance_.maxAcceleration);
        }
        else
        {
            // Deceleration
            transition_.SetMaxRate(object_->performance_.maxDeceleration);
        }

        // Make sure sign of rate is correct
        if (transition_.dimension_ == DynamicsDimension::RATE)
        {
            transition_.SetRate(SIGN(transition_.GetTargetVal() - transition_.GetStartVal()) * abs(transition_.GetRate()));
        }

        transition_.Step(dt);
        new_speed = transition_.Evaluate();

        if (transition_.GetParamVal() > transition_.GetParamTargetVal() - SMALL_NUMBER ||
            // Close enough?
            abs(new_speed - transition_.GetTargetVal()) < SMALL_NUMBER ||
            // Already passed target value (perhaps due to strange initial conditions)?
            SIGN(target_->GetValue() - transition_.GetStartVal()) != SIGN(target_->GetValue() - object_->GetSpeed()))
        {
            target_speed_reached_ = true;
            new_speed             = target_->GetValue();
        }
    }

    object_->SetSpeed(ABS_LIMIT(new_speed, object_->performance_.maxSpeed));

    if (target_speed_reached_ &&
        !(target_->type_ == Target::TargetType::RELATIVE_SPEED && (static_cast<TargetRelative*>(target_.get()))->continuous_ == true))
    {
        OSCAction::End();
    }
}

void LongSpeedProfileAction::Start(double simTime)
{
    OSCAction::Start(simTime);

    if (entry_.size() == 0 || object_ == nullptr)
    {
        return;
    }

    double speed_offset = entity_ref_ != nullptr ? entity_ref_->GetSpeed() : 0.0;
    double init_time_   = simTime;

    init_acc_ = object_->pos_.GetAccLong();

    std::vector<EntryVertex> vertex;
    unsigned int             index = 0;

    // Check need for initial entry with current speed
    if (entry_[0].time_ > SMALL_NUMBER)
    {
        // insert initial entry at time = 0 with initial/current speed
        vertex.push_back(EntryVertex(init_time_, object_->GetSpeed()));
    }
    else if (entry_[0].time_ > -SMALL_NUMBER)
    {
        if (following_mode_ == FollowingMode::FOLLOW)
        {
            // replace first entry at time = 0.0 with current speed
            vertex.push_back(EntryVertex(init_time_, object_->GetSpeed()));
            index = 1;  // Skip first entry
        }
        else
        {
            // in linear mode use specified initial speed
            vertex.push_back(EntryVertex(init_time_, entry_[0].speed_));
        }
    }
    else  // negative time indicating time attribute omitted
    {
        // Create a first entry at current time and speed
        vertex.push_back(EntryVertex(init_time_, object_->GetSpeed()));
    }

    // First filter out any obsolete middle points on straight acceleration lines
    double delta_k, j = 0.0, time = init_time_;
    for (; index < entry_.size(); index++)
    {
        if (entry_[index].time_ < -SMALL_NUMBER)
        {
            if (abs(entry_[index].speed_ + speed_offset - vertex.back().v_) < SMALL_NUMBER)
            {
                // same speed as previous vertex, skip entry
                continue;
            }
            // negative time is interpreted as missing time stamp
            double dv  = entry_[index].speed_ + speed_offset - vertex.back().v_;
            double acc = 0.0;
            if (dv < 0)
            {
                acc = MIN(-SMALL_NUMBER, -dynamics_.max_deceleration_);
            }
            else
            {
                acc = MAX(SMALL_NUMBER, dynamics_.max_acceleration_);
            }

            vertex.back().t_ = time;
            time += dv / acc;
        }
        else
        {
            time += entry_[index].time_;
        }

        double dt = time - vertex.back().t_;

        if (index < entry_.size() - 1)
        {
            if (dt < SMALL_NUMBER)
            {
                // replace previous entry
                vertex.back() = EntryVertex(time, entry_[index].speed_ + speed_offset);
                continue;  // skip entry
            }
        }

        vertex.back().SetK((entry_[index].speed_ + speed_offset - (vertex.back().v_)) / dt);

        if (index > 1)
        {
            delta_k = vertex.back().k_ - (vertex.rbegin() + 1)->k_;  // last - second last
        }
        else
        {
            delta_k = vertex.back().k_ - init_acc_;  // slope is zero at start - no delta
        }

        if (abs(delta_k) < SMALL_NUMBER && index > 1)
        {
            // replace previous entry
            double new_k  = (entry_[index].speed_ + speed_offset - (vertex.rbegin() + 1)->v_) / (time - (vertex.rbegin() + 1)->t_);
            vertex.back() = EntryVertex(time, entry_[index].speed_ + speed_offset, new_k);
            continue;  // skip entry
        }

        if (index == entry_.size() - 1)
        {
            // Final entry, set k = 0
            vertex.push_back(EntryVertex(time, entry_[index].speed_ + speed_offset, 0.0));
        }
        else
        {
            vertex.push_back(EntryVertex(time, entry_[index].speed_ + speed_offset));
        }
    }

    EntryVertex vtx = vertex[0];
    double      t0, t1, t3 = vtx.t_, v0, v3 = 0.0, m0, m1 = 0.0, k1 = 0.0;
    double      k0 = init_acc_;

    segment_.clear();

    // Some info on the implementation concept and equations systems can be found here:
    // https://drive.google.com/file/d/1DmjVHftcsbU71Ce_GASZ6IArcPA6teNF/view?usp=sharing

    if (vertex.size() == 1)
    {
        AddSpeedSegment(vtx.t_, vtx.v_, 0.0, 0.0);
    }
    else if (following_mode_ == FollowingMode::FOLLOW && vertex.size() == 2)
    {
        // Special case: Single speed target, following mode = follow
        // Reach target speed with given jerk contraint. Add linear segment if needed.

        // Set first jerk segment

        j = (vertex[0].k_ - k0) < 0 ? -dynamics_.max_deceleration_rate_ : dynamics_.max_acceleration_rate_;
        AddSpeedSegment(vtx.t_, vtx.v_, k0, j);

        double j0 = j;
        double j1 = 0.0, t2 = 0.0, v1 = 0.0, v2 = 0.0;

        // Find jerk at endpoint, where acceleration / k is zero
        if (vertex[0].k_ < 0)
        {
            j1 = dynamics_.max_acceleration_rate_;
        }
        else
        {
            j1 = -dynamics_.max_deceleration_rate_;
        }

        t0 = vertex[0].t_;
        v0 = vertex[0].v_;
        t3 = vertex[1].t_;
        v3 = vertex[1].v_;

        if (fabs(j1 - j0) < SMALL_NUMBER)
        {
            // following_mode_ = FollowingMode::POSITION;
            if (fabs(t0 - t3) > SMALL_NUMBER && fabs(j1 * (t0 - t3) - k0) > SMALL_NUMBER)
            {
                t1 = (2 * j1 * (v0 + t0 * j1 * (t0 - t3) - v3) + pow(k0, 2) + 2 * k0 * j1 * (t3 - 2 * t0)) / (2 * j1 * (j1 * (t0 - t3) - k0));
                if (t1 < 0)
                {
                    LOG("SpeedProfile: No solution found (t1) - fallback to Position mode");
                }
                else
                {
                    v1 = v0 + k0 * (t1 - t0) + j0 * 0.5 * pow((t1 - t0), 2);
                    k1 = k0 + j0 * (t1 - t0);
                    t2 = (k1 / j1) + t3;
                    if (t2 < 0)
                    {
                        LOG("SpeedProfile: No solution found (t2)");
                    }
                    else
                    {
                        v2 = v1 + k1 * (t2 - t1);
                        if (k1 < dynamics_.max_acceleration_ && k1 > -dynamics_.max_deceleration_)
                        {
                            // add linear segment
                            AddSpeedSegment(t1, v1, k1, 0.0);
                        }
                    }
                }
            }
            else
            {
                LOG("SpeedProfile: No solution found (t3)");
            }
        }
        else
        {
            double factor = j0 * j1 * (2 * v0 * (j1 - j0) + pow(k0, 2) + 2 * k0 * j1 * (t3 - t0) + j0 * j1 * pow(t0 - t3, 2) + 2 * v3 * (j0 - j1));

            if (factor < 0.0)
            {
                // no room or time for linear segment of constant acceleration
                t2 = (sqrt(j1 * (-(j0 - j1)) * (j0 * (2 * v3 - 2 * v0) + pow(k0, 2))) + (j0 - j1) * (t0 * j0 - k0)) / (j0 * (j0 - j1));
                v2 = v0 + k0 * (t2 - t0) + j0 * 0.5 * pow(t2 - t0, 2);
                k1 = k0 + j0 * (t2 - t0);
                t3 = t2 - k1 / j1;

                if (IS_IN_SPAN(k1, -dynamics_.max_deceleration_, dynamics_.max_acceleration_) && t3 > vertex[1].t_)
                {
                    LOG("SpeedProfile: Can't reach target speed %.2f on target time %.2fs with given jerk constraints, extend to %.2fs",
                        v3,
                        vertex[1].t_,
                        t3);
                }
            }
            else
            {
                // need a linear segment to reach speed at specified time
                LOG("SpeedProfile: Add linear segment to reach target speed on time.");
                t1 = (-sqrt(factor) - j0 * (k0 + t3 * j1) + t0 * pow(j0, 2)) / (j0 * (j0 - j1));
                v1 = v0 + k0 * (t1 - t0) + j0 * 0.5 * pow(t1 - t0, 2);
                k1 = init_acc_ + j0 * (t1 - t0);
                t2 = (k1 / j1) + t3;
                v2 = v1 + k1 * (t2 - t1);

                if (IS_IN_SPAN(k1, -dynamics_.max_deceleration_, dynamics_.max_acceleration_))
                {
                    // add linear segment
                    AddSpeedSegment(t1, v1, k1, 0.0);
                }
            }
        }

        if (IS_IN_SPAN(k1, -dynamics_.max_deceleration_, dynamics_.max_acceleration_))
        {
            // add second jerk segment
            AddSpeedSegment(t2, v2, k1, j1);
            // add end segment
            AddSpeedSegment(t3, v3, 0.0, 0.0);
        }
        else
        {
            if (k1 > dynamics_.max_acceleration_)
            {
                LOG("SpeedProfile: Constraining acceleration from %.2f to %.2f", k1, dynamics_.max_acceleration_);
                k1 = dynamics_.max_acceleration_;
            }
            else if (k1 < -dynamics_.max_deceleration_)
            {
                LOG("SpeedProfile: Constraining deceleration from %.2f to %.2f", -k1, dynamics_.max_deceleration_);
                k1 = -dynamics_.max_deceleration_;
            }

            t1 = t0 + (k1 - k0) / j0;
            v1 = v0 + k0 * (t1 - t0) + 0.5 * j0 * pow(t1 - t0, 2);
            t2 = (v3 - v1) / k1 + t1 + k1 / (2 * j1);
            v2 = v3 + pow(k1, 2) / (2 * j1);
            t3 = -v1 / k1 + v3 / k1 + t1 - k1 / (2 * j1);

            LOG("SpeedProfile: Extend %.2f s", t3 - vertex.back().t_);

            // add linear segment
            AddSpeedSegment(t1, v1, k1, 0.0);
            // add second jerk segment
            AddSpeedSegment(t2, v2, k1, j1);
            // add end segment
            AddSpeedSegment(t3, v3, 0.0, 0.0);
        }
    }
    else
    {
        // Normal case: Follow acceleration (slopes) of multiple speed targets over time

        if (following_mode_ == FollowingMode::FOLLOW)
        {
            if (abs(vtx.k_) > SMALL_NUMBER)
            {
                j = (vertex[0].k_ - k0) < 0 ? -dynamics_.max_deceleration_rate_ : dynamics_.max_acceleration_rate_;
                AddSpeedSegment(vtx.t_, vtx.v_, k0, j);

                t3 = vtx.t_ + (abs(j) > SMALL_NUMBER ? (vtx.k_ - k0) / j : 0.0);  // duration of first jerk t = a / j
            }

            v3 = vtx.v_ + k0 * (t3 - init_time_) + 0.5 * j * pow((t3 - init_time_), 2);  // speed after initial jerk
            k1 = vtx.k_;                                                                 // slope of first linear segment
            m1 = v3 - k1 * t3;                                                           // eq constant for second acceleration line

            // add first linear segment
            AddSpeedSegment(t3, v3, k1, 0.0);
        }

        for (index = 0; index < vertex.size(); index++)
        {
            if (following_mode_ == FollowingMode::POSITION)
            {
                vtx = vertex[index];
                AddSpeedSegment(vtx.t_, vtx.v_, vtx.k_, 0.0);
            }
            else if (index < vertex.size() - 1)
            {
                k0  = k1;
                vtx = vertex[index + 1];
                t0  = t3;
                v0  = v3;
                m0  = m1;
                k1  = vtx.k_;
                m1  = vtx.m_;

                if (index < vertex.size() - 1)
                {
                    j = vtx.k_ - k0 < 0 ? -dynamics_.max_deceleration_rate_ : dynamics_.max_acceleration_rate_;
                }

                if (abs(k0 - k1) < SMALL_NUMBER)
                {
                    // no change in acceleration, skip jerk segment
                    t3 = t0;
                    v3 = v0;
                }
                else
                {
                    // find time for next jerk
                    // https://www.wolframalpha.com/input?i=solve+b%3Dk*g%2Bn%2Cd%3Dl*j%2Bo%2Cd%3Db%2Bk*%28j-g%29%2Bm*%28j-g%29%5E2%2F2%2Cl%3Dk%2Bm*%28j-g%29+for+b%2Cd%2Cg%2Cj
                    // solve b = k * g + n, d = l * j + o, d = b + k * (j - g) + m * (j - g) ^ 2 / 2, l = k + m * (j - g) for b, d, g, j
                    // substitutions: a = v0, b = v1, c = v2, d = v3, f = t0, g = t1, h = t2, j = t3, k = k0, l = k1, m = j, n = m0, o = m1
                    // g = (k ^ 2 - 2 k l + l ^ 2 - 2 m(n - o)) / (2 m(k - l))
                    t1 = (pow(k0, 2) - 2 * k0 * k1 + pow(k1, 2) - 2 * j * (m0 - m1)) / (2 * j * (k0 - k1));

                    if (t1 < t0)
                    {
                        LOG("LongSpeedProfileAction failed at point %d (time=%.2f). Falling back to linear (Position) mode.",
                            index,
                            vertex[index].t_);
                        following_mode_ = FollowingMode::POSITION;
                        continue;
                    }

                    double v1 = v0 + k0 * (t1 - t0);
                    t3        = t1 + (vtx.k_ - k0) / j;
                    v3        = v1 + k0 * (t3 - t1) + 0.5 * j * pow(t3 - t1, 2);

                    // add jerk segment
                    AddSpeedSegment(t1, v1, k0, j);
                }

                // add linear segment
                AddSpeedSegment(t3, v3, vtx.k_, 0.0);
            }
        }
    }

    cur_index_ = 0;
    speed_     = object_->GetSpeed();
}

void LongSpeedProfileAction::Step(double simTime, double dt)
{
    double time = simTime + dt;

    if (time < segment_.back().t + 10 && !(time > segment_.back().t and abs(speed_ - segment_.back().v) < SMALL_NUMBER))
    {
        while (static_cast<unsigned int>(cur_index_) < segment_.size() - 1 &&
               time > segment_[static_cast<unsigned int>(cur_index_) + 1].t - SMALL_NUMBER)
        {
            cur_index_++;
        }

        SpeedSegment* s = &segment_[static_cast<unsigned int>(cur_index_)];

        speed_ = s->v + s->k * (time - s->t) + 0.5 * s->j * pow(time - s->t, 2);
    }

    elapsed_ = MAX(0.0, time - segment_[0].t);

    if (static_cast<unsigned int>(cur_index_) >= entry_.size() - 1 && fabs(speed_ - segment_.back().v) < SMALL_NUMBER)
    {
        speed_ = segment_.back().v;
        OSCAction::End();
    }

    object_->SetSpeed(speed_);
}

void LongSpeedProfileAction::CheckAcceleration(double acc)
{
    if (following_mode_ == FollowingMode::POSITION)
    {
        return;
    }

    if (acc > dynamics_.max_acceleration_ + SMALL_NUMBER || acc < -dynamics_.max_deceleration_ - SMALL_NUMBER)
    {
        LOG("Acceleration %.2f not within constrained span [%.2f:%.2f], revert to linear mode",
            acc,
            dynamics_.max_acceleration_,
            -dynamics_.max_deceleration_);

        following_mode_ = FollowingMode::POSITION;
    }
}

void LongSpeedProfileAction::CheckAccelerationRate(double acc_rate)
{
    if (following_mode_ == FollowingMode::POSITION)
    {
        return;
    }

    if (acc_rate > dynamics_.max_acceleration_rate_ + SMALL_NUMBER || acc_rate < -dynamics_.max_deceleration_rate_ - SMALL_NUMBER)
    {
        LOG("Acceleration rate %.2f not within constrained span [%.2f:%.2f], revert to linear mode",
            acc_rate,
            dynamics_.max_acceleration_rate_,
            -dynamics_.max_deceleration_rate_);

        following_mode_ = FollowingMode::POSITION;
    }
}

void LongSpeedProfileAction::CheckSpeed(double speed)
{
    if (following_mode_ == FollowingMode::POSITION)
    {
        return;
    }

    if (speed > dynamics_.max_speed_ + SMALL_NUMBER || speed < -dynamics_.max_speed_ - SMALL_NUMBER)
    {
        LOG("Speed %.2f not within constrained span [%.2f:%.2f], revert to linear mode", speed, dynamics_.max_speed_, -dynamics_.max_speed_);

        following_mode_ = FollowingMode::POSITION;
    }
}

void LongSpeedProfileAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }

    if (entity_ref_ == obj1)
    {
        entity_ref_ = obj2;
    }
}

void LongSpeedProfileAction::AddSpeedSegment(double t, double v, double k, double j)
{
    CheckSpeed(v);
    CheckAcceleration(k);
    CheckAccelerationRate(j);
    segment_.push_back({t, v, k, j});
}

void LongSpeedAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }

    if (target_->type_ == Target::TargetType::RELATIVE_SPEED)
    {
        if ((static_cast<TargetRelative*>(target_.get()))->object_ == obj1)
        {
            (static_cast<TargetRelative*>(target_.get()))->object_ = obj2;
        }
    }
}

void LongDistanceAction::Start(double simTime)
{
    sim_time_ = simTime;
    if (target_object_ == 0)
    {
        LOG("Can't trig without set target object ");
        return;
    }

    // Resolve displacement
    if (displacement_ == DisplacementType::ANY)
    {
        // Find out current displacement, and apply it
        double distance;
        if (freespace_)
        {
            double latDist  = 0;
            double longDist = 0;
            object_->FreeSpaceDistance(target_object_, &latDist, &longDist);
            distance = longDist;
        }
        else
        {
            double x, y;
            distance = object_->pos_.getRelativeDistance(target_object_->pos_.GetX(), target_object_->pos_.GetY(), x, y);

            // Just interested in the x-axis component of the distance
            distance = x;
        }

        if (distance < 0.0)
        {
            displacement_ = DisplacementType::LEADING;
        }
        else
        {
            displacement_ = DisplacementType::TRAILING;
        }
    }

    OSCAction::Start(simTime);
}

void LongDistanceAction::Step(double simTime, double)
{
    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)))
    {
        // longitudinal motion controlled elsewhere
        return;
    }

    double dt = simTime - sim_time_;
    sim_time_ = simTime;

    // Find out current distance
    double distance;
    if (freespace_)
    {
        double latDist  = 0;
        double longDist = 0;
        object_->FreeSpaceDistance(target_object_, &latDist, &longDist);
        distance = longDist;
    }
    else
    {
        object_->pos_.Distance(&target_object_->pos_, cs_, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, distance);
    }

    double speed_diff = object_->speed_ - target_object_->speed_;
    double acc;
    double jerk            = 0.0;
    double spring_constant = 0.4;
    double dc;
    double requested_dist = 0;

    if (dist_type_ == DistType::DISTANCE)
    {
        requested_dist = distance_;
    }
    if (dist_type_ == DistType::TIME_GAP)
    {
        // Convert requested time gap (seconds) to distance (m)
        requested_dist = abs(target_object_->speed_) * distance_;
    }

    if (displacement_ == DisplacementType::TRAILING)
    {
        requested_dist = abs(requested_dist);
    }
    else if (displacement_ == DisplacementType::LEADING)
    {
        requested_dist = -abs(requested_dist);
    }

    double distance_diff = distance - requested_dist;

    if (continuous_ == false && fabs(distance_diff) < LONGITUDINAL_DISTANCE_THRESHOLD)
    {
        // Reached requested distance, quit action
        OSCAction::End();
    }

    if (dynamics_.max_acceleration_ >= LARGE_NUMBER && dynamics_.max_deceleration_ >= LARGE_NUMBER)
    {
        // Set position according to distance and copy speed of target vehicle
        object_->pos_.MoveAlongS(distance_diff);
        object_->SetSpeed(target_object_->speed_);
    }
    else
    {
        // Apply damped spring model with critical/optimal damping factor
        // Adjust tension in spring in proportion to the max acceleration and max deceleration
        double tension = distance_diff < 0.0 ? dynamics_.max_acceleration_ : dynamics_.max_deceleration_;

        double spring_constant_adjusted = tension * spring_constant;
        dc                              = 2 * sqrt(spring_constant_adjusted);
        acc                             = distance_diff * spring_constant_adjusted - speed_diff * dc;
        if (acc < 0.0)
        {
            jerk = -dynamics_.max_deceleration_rate_;
            if (acc < -dynamics_.max_deceleration_)
            {
                acc = -dynamics_.max_deceleration_;
            }
        }
        else
        {
            jerk = dynamics_.max_acceleration_rate_;
            if (acc > dynamics_.max_acceleration_)
            {
                acc = dynamics_.max_acceleration_;
            }
        }

        // Apply simple linear model for jerk
        if (jerk < 0.0 && acc < acceleration_)
        {
            acc = MAX(acceleration_ + jerk * dt, acc);
        }
        else if (jerk > 0.0 && acc > acceleration_)
        {
            acc = MIN(acceleration_ + jerk * dt, acc);
        }

        acceleration_ = acc;
        object_->SetSpeed(object_->GetSpeed() + acceleration_ * dt);

        if (object_->GetSpeed() > dynamics_.max_speed_)
        {
            object_->SetSpeed(dynamics_.max_speed_);
        }
        else if (object_->GetSpeed() < -dynamics_.max_speed_)
        {
            object_->SetSpeed(-dynamics_.max_speed_);
        }
    }
}

void LongDistanceAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }

    if (target_object_ == obj1)
    {
        target_object_ = obj2;
    }
}

void TeleportAction::Start(double simTime)
{
    OSCAction::Start(simTime);
    LOG("Starting teleport Action");

    if (object_->IsGhost() && IsGhostRestart() && scenarioEngine_->getSimulationTime() > SMALL_NUMBER)
    {
        SE_Env::Inst().SetGhostMode(GhostMode::RESTART);

        object_->trail_.Reset(true);

        // The following code will copy speed from the Ego that ghost relates to
        if (object_->ghost_Ego_ != nullptr)
        {
            object_->SetSpeed(object_->ghost_Ego_->GetSpeed());
        }

        scenarioEngine_->ResetEvents();  // Ghost-project. Reset events finished by ghost.
    }

    if (object_->IsControllerModeOnAnyOfDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT_AND_LONG)))
    {
        // motion controlled elsewhere
        return;
    }

    if (object_->TowVehicle())
    {
        return;  // position controlled by tow vehicle
    }

    // consider any assigned route for relative positions
    position_->CopyRouteSharedPtr(&object_->pos_);
    object_->pos_.TeleportTo(position_);

    if (!object_->TowVehicle() && object_->TrailerVehicle())
    {
        (static_cast<Vehicle*>(object_))->AlignTrailers();
    }

    LOG("%s New position:", object_->name_.c_str());
    object_->pos_.Print();

    object_->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::SPEED | Object::DirtyBit::TELEPORT);
    object_->reset_ = true;
}

void TeleportAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::End();
}

void TeleportAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }

    position_->ReplaceObjectRefs(&obj1->pos_, &obj2->pos_);
}

void ConnectTrailerAction::Start(double simTime)
{
    OSCAction::Start(simTime);

    if (trailer_object_)
    {
        if (object_->TrailerVehicle())
        {
            if (object_->TrailerVehicle() == trailer_object_)
            {
                LOG("Trailer %s already connected to %s - keep connection", trailer_object_->GetName().c_str(), object_->GetName().c_str());
            }
            else
            {
                LOG("Disconnecting currently connected trailer: %s", object_->TrailerVehicle()->GetName().c_str());
                reinterpret_cast<Vehicle*>(object_)->DisconnectTrailer();
            }
        }

        if (trailer_object_ != object_->TrailerVehicle())
        {
            LOG("Connect trailer %s to %s", reinterpret_cast<Vehicle*>(trailer_object_)->GetName().c_str(), object_->GetName().c_str());
            reinterpret_cast<Vehicle*>(object_)->ConnectTrailer(reinterpret_cast<Vehicle*>(trailer_object_));
        }
    }
    else
    {
        LOG("No trailer to disconnect from %s", object_->GetName().c_str());
    }
}

void ConnectTrailerAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::End();
}

void ConnectTrailerAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }

    if (trailer_object_ == obj1)
    {
        trailer_object_ = obj2;
    }
}

void DisconnectTrailerAction::Start(double simTime)
{
    OSCAction::Start(simTime);

    if (object_->TrailerVehicle())
    {
        LOG("Disconnecting %s from %s", object_->TrailerVehicle()->GetName().c_str(), object_->GetName().c_str());
        reinterpret_cast<Vehicle*>(object_)->DisconnectTrailer();
    }
    else
    {
        LOG("DisconnectTrailerAction: No trailer connected, ignoring action");
    }
}

void DisconnectTrailerAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::End();
}

void DisconnectTrailerAction::ReplaceObjectRefs(Object* obj1, Object* obj2)
{
    if (object_ == obj1)
    {
        object_ = obj2;
    }
}

double SynchronizeAction::CalcSpeedForLinearProfile(double v_final, double time, double dist)
{
    if (time < 0.001 || dist < 0.001)
    {
        // Avoid division by zero, fall back to zero acceleration
        return 0;
    }

    // Compute current speed needed to reach given final speed in given time
    double v0 = 2 * dist / time - v_final;

    return v0;
}

const char* SynchronizeAction::Mode2Str(SynchMode mode)
{
    if (mode == SynchMode::MODE_NONE)
    {
        return "MODE_NONE";
    }
    else if (mode == SynchMode::MODE_NON_LINEAR)
    {
        return "MODE_NON_LINEAR";
    }
    else if (mode == SynchMode::MODE_LINEAR)
    {
        return "MODE_LINEAR";
    }
    else if (mode == SynchMode::MODE_STOPPED)
    {
        return "MODE_STOPPED";
    }
    else if (mode == SynchMode::MODE_STOP_IMMEDIATELY)
    {
        return "MODE_STOP_IMMEDIATELY";
    }
    else if (mode == SynchMode::MODE_WAITING)
    {
        return "MODE_WAITING";
    }
    else if (mode == SynchMode::MODE_STEADY_STATE)
    {
        return "MODE_STEADY_STATE";
    }
    else
    {
        return "Unknown mode";
    }
}

const char* SynchronizeAction::SubMode2Str(SynchSubmode submode)
{
    if (submode == SynchSubmode::SUBMODE_CONCAVE)
    {
        return "SUBMODE_CONCAVE";
    }
    else if (submode == SynchSubmode::SUBMODE_CONVEX)
    {
        return "SUBMODE_CONVEX";
    }
    else if (submode == SynchSubmode::SUBMODE_NONE)
    {
        return "SUBMODE_NONE";
    }
    else
    {
        return "Unknown sub-mode";
    }
}

void SynchronizeAction::PrintStatus(const char* custom_msg)
{
    LOG("%s, mode=%s (%d) sub-mode=%s (%d)", custom_msg, Mode2Str(mode_), mode_, SubMode2Str(submode_), submode_);
}

void SynchronizeAction::Start(double simTime)
{
    // resolve steady state -> translate into dist
    if (steadyState_.type_ == SteadyStateType::STEADY_STATE_TIME)
    {
        steadyState_.dist_ = steadyState_.time_ * final_speed_->GetValue();
        steadyState_.type_ = SteadyStateType::STEADY_STATE_DIST;
    }
    else if (steadyState_.type_ == SteadyStateType::STEADY_STATE_POS)
    {
        // Find out distance between steady state position and final destination
        roadmanager::PositionDiff diff;
        target_position_->Delta(steadyState_.pos_, diff);
        steadyState_.dist_ = diff.ds;
        steadyState_.type_ = SteadyStateType::STEADY_STATE_DIST;
    }

    OSCAction::Start(simTime);

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)))
    {
        // longitudinal motion controlled elsewhere
        return;
    }
}

void SynchronizeAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;
    bool done = false;

    if (object_->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)))
    {
        // longitudinal motion controlled elsewhere
        return;
    }

    // Calculate distance along road/route
    double                    masterDist, dist;
    roadmanager::PositionDiff diff;

    if (master_object_->pos_.GetTrajectory() || !master_object_->pos_.Delta(target_position_master_, diff))
    {
        // No road network path between master vehicle and master target pos - using world coordinate distance
        diff.ds = GetLengthOfLine2D(master_object_->pos_.GetX(),
                                    master_object_->pos_.GetY(),
                                    target_position_master_->GetX(),
                                    target_position_master_->GetY());
    }
    masterDist = fabs(diff.ds);

    if (object_->pos_.GetTrajectory() || !object_->pos_.Delta(target_position_, diff))
    {
        // No road network path between action vehicle and action target pos - using world coordinate distance
        diff.ds = GetLengthOfLine2D(object_->pos_.GetX(), object_->pos_.GetY(), target_position_->GetX(), target_position_->GetY());
    }
    dist = fabs(diff.ds);

    // Done when distance increases, indicating that destination just has been reached or passed
    if (dist < tolerance_ + SMALL_NUMBER)
    {
        LOG("Synchronize dist (%.2f) < tolerance (%.2f)", dist, tolerance_);
        done = true;
    }
    else if (masterDist < tolerance_master_ + SMALL_NUMBER)
    {
        LOG("Synchronize masterDist (%.2f) < tolerance (%.2f)", masterDist, tolerance_master_);
        done = true;
    }
    else if (dist > lastDist_)
    {
        LOG("Synchronize dist increasing (%.2f > %.2f) - missed destination", dist, lastDist_);
        done = true;
    }
    else if (masterDist > lastMasterDist_)
    {
        LOG("Synchronize masterDist increasing (%.2f > %.2f) - missed destination", masterDist, lastMasterDist_);
        done = true;
    }

    lastDist_       = dist;
    lastMasterDist_ = masterDist;

    // for calculations, measure distance to toleration area/radius
    dist       = MAX(dist - tolerance_, SMALL_NUMBER);
    masterDist = MAX(masterDist - tolerance_master_, SMALL_NUMBER);

    double masterTimeToDest = LARGE_NUMBER;

    // project speed on road s-axis
    double master_speed = SIGN(master_object_->GetSpeed()) * abs(master_object_->pos_.GetVelS());

    if (master_speed > SMALL_NUMBER)
    {
        masterTimeToDest = masterDist / master_speed;

        if (masterTimeToDest < dt)
        {
            LOG("Synchronize masterTimeToDest (%.3f) reached within this timestep (%.3f)", masterTimeToDest, dt);
            done = true;
        }
    }

    if (done)
    {
        if (final_speed_)
        {
            object_->SetSpeed(final_speed_->GetValue());
        }
        OSCAction::End();
    }
    else
    {
        double average_speed = dist / masterTimeToDest;
        double acc           = 0;

        if (final_speed_)
        {
            if (steadyState_.type_ != SteadyStateType::STEADY_STATE_NONE && mode_ != SynchMode::MODE_STEADY_STATE)
            {
                double time_to_ss = steadyState_.dist_ / final_speed_->GetValue();

                if (dist - steadyState_.dist_ < SMALL_NUMBER || masterTimeToDest - time_to_ss < SMALL_NUMBER)
                {
                    mode_    = SynchMode::MODE_STEADY_STATE;
                    submode_ = SynchSubmode::SUBMODE_NONE;
                    if (time_to_ss > masterTimeToDest && (time_to_ss - masterTimeToDest) * final_speed_->GetValue() > tolerance_)
                    {
                        LOG("Entering Stead State according to criteria but not enough time to reach destination");
                    }
                    // PrintStatus("SteadyState");
                }
                else
                {
                    // subtract steady state distance
                    dist -= steadyState_.dist_;

                    // subtract steady state duration
                    masterTimeToDest -= time_to_ss;
                }
            }

            // For more information about calculations, see
            // https://docs.google.com/document/d/1dEBUWlJVLUz6Rp9Ol1l90iG0LfNtcsgLyJ0kDdwgPzA/edit?usp=sharing
            //
            // Interactive Python script plotting calculation result based on various input values
            // https://drive.google.com/file/d/1z902gRYogkLhUAV1pZLc9gcgwnak7TBH/view?usp=sharing
            // (the method described below is "Spedified final speed - alt 1")
            //
            // Here follow a brief description:
            //
            // Calculate acceleration needed to reach the destination in due time
            // Four cases
            //   1  Linear. Reach final speed with constant acceleration
            //	 2a Non-linear convex (rush). First accelerate, then decelerate.
            //	 2b Non-linear concave (linger). First decelerate, then accelerate.
            //   3  Non-linear with stop. Decelerate to a stop. Wait. Then accelerate.
            //
            //   Case 2-3 involves two (case 2a, 2b) or three (case 3) phases with constant acceleration/deceleration
            //   Last phase in case 2-3 is actually case 1 - a linear change to final speed
            //
            // Symbols
            //   given:
            //     s = distance to destination
            //     t = master object time to destination
            //     v0 = current speed
            //     v1 = final speed
            //     va = Average speed needed to reach destination
            //   variables:
            //     s1 = distance first phase
            //     s2 = distance second phase
            //     x = end time for first phase
            //     y = end time for second (last) phase
            //     vx = speed at x
            //
            // Equations
            //   case 1
            //     v1 = 2 * s / t - v2
            //     a = (v2 - v1) / t
            //
            //   case 2 (a & b)
            //      system:
            //        s1 = v1 * x + (vx - v1) * x / 2
            //        s2 = v2 * y + (vx - v2) * y / 2
            //        t = x + y
            //        s = s1 + s2
            // 		  (vx - v1) / x = (vx - v2) / y
            //
            //      solve for x and vx
            //      a = (vx - v1) / x
            //
            //   case 3
            //      system:
            //        s1 = x * v1 / 2
            //        s2 = y * v2 / 2
            //        s = s1 + s2
            //        v1 / v2 = x / y
            //
            //      solve for x
            //      a = -v1 / x

            if (mode_ == SynchMode::MODE_STEADY_STATE)
            {
                object_->speed_ = final_speed_->GetValue();
                return;
            }
            if (mode_ == SynchMode::MODE_WAITING)
            {
                if (masterTimeToDest >= LARGE_NUMBER)
                {
                    // Continue waiting
                    return;
                }
                else
                {
                    // Reset mode
                    mode_ = SynchMode::MODE_NONE;
                }
            }
            if (mode_ == SynchMode::MODE_STOP_IMMEDIATELY)
            {
                acc = MAX_DECELERATION;
                object_->speed_ += acc * dt;
                if (object_->speed_ < 0)
                {
                    object_->SetSpeed(0);
                    mode_ = SynchMode::MODE_WAITING;  // wait for master to move
                                                      // PrintStatus("Waiting");
                }

                return;
            }
            else if (mode_ == SynchMode::MODE_STOPPED)
            {
                if (masterTimeToDest < 2 * dist / final_speed_->GetValue())
                {
                    // Time to move again after the stop
                    mode_ = SynchMode::MODE_LINEAR;
                    // PrintStatus("Restart");
                }
                else
                {
                    // Stay still
                    object_->SetSpeed(0);
                    return;
                }
            }

            if (mode_ == SynchMode::MODE_LINEAR)
            {
                if (masterTimeToDest > LARGE_NUMBER - 1)
                {
                    // Master in effect standing still, do not move
                    object_->SetSpeed(0);
                }
                else
                {
                    object_->SetSpeed(MAX(0, CalcSpeedForLinearProfile(MAX(0, final_speed_->GetValue()), masterTimeToDest, dist)));
                }
                return;
            }
            else if (mode_ == SynchMode::MODE_NON_LINEAR && masterTimeToDest < LARGE_NUMBER)
            {
                // Check if case 1, i.e. on a straight speed profile line
                double v0_onLine = 2 * dist / masterTimeToDest - final_speed_->GetValue();

                if (fabs(object_->speed_ - v0_onLine) < 0.1)
                {
                    // Passed apex. Switch to linear mode (constant acc) to reach final destination and speed
                    mode_ = SynchMode::MODE_LINEAR;
                    // PrintStatus("Passed apex");

                    // Keep current speed for this time step
                    return;
                }
            }

            if (mode_ == SynchMode::MODE_NONE)
            {
                // Since not linear mode, go into non-linear mode
                mode_ = SynchMode::MODE_NON_LINEAR;

                // Find out submode, convex or concave
                if ((final_speed_->GetValue() + object_->speed_) / 2 < dist / masterTimeToDest)
                {
                    submode_ = SynchSubmode::SUBMODE_CONVEX;
                }
                else
                {
                    submode_ = SynchSubmode::SUBMODE_CONCAVE;
                }
                // PrintStatus("Non-linear");
            }

            // Now, calculate x and vx according to default method oulined in the documentation
            double s = dist;
            double t = masterTimeToDest;

            // project speed on road s-axis
            double v0 = SIGN(object_->GetSpeed()) * abs(object_->pos_.GetVelS());
            double v1 = final_speed_->GetValue();

            double signed_term = sqrt(2.0) * sqrt(2.0 * s * s - 2 * (v1 + v0) * t * s + (v1 * v1 + v0 * v0) * t * t);

            // Calculate both solutions from the quadratic equation
            double vx = 0;
            if (fabs(v1 - v0) < SMALL_NUMBER)
            {
                // When v0 == v1, x is simply t/2
                // s = (T / 2) * v_cur + (T / 2) * vx -> vx = (s - (T / 2) * v_cur) / (T / 2)
                vx  = (s - (t / 2) * v0) / (t / 2);
                acc = (vx - v0) / (t / 2);
            }
            else
            {
                double x1  = -(signed_term + 2 * s - 2 * v1 * t) / (2 * (v1 - v0));
                double x2  = -(-signed_term + 2 * s - 2 * v1 * t) / (2 * (v1 - v0));
                double vx1 = (2 * s - signed_term) / (2 * t);
                double vx2 = (2 * s + signed_term) / (2 * t);
                double a1  = (vx1 - v0) / x1;
                double a2  = (vx2 - v0) / x2;

                // Choose solution, only one is found within the given time span [0:masterTimeToDest]
                if (x1 > 0 && x1 < t)
                {
                    vx  = vx1;
                    acc = a1;
                }
                else if (x2 > 0 && x2 < t)
                {
                    vx  = vx2;
                    acc = a2;
                }
                else
                {
                    // No solution
                    acc = 0;
                }
            }

            if (mode_ == SynchMode::MODE_NON_LINEAR &&
                ((submode_ == SynchSubmode::SUBMODE_CONCAVE && acc > 0) || (submode_ == SynchSubmode::SUBMODE_CONVEX && acc < 0)))
            {
                // Reached the apex of the speed profile, switch mode and phase
                mode_ = SynchMode::MODE_LINEAR;
                // PrintStatus("Reached apex");

                // Keep speed for this time step
                acc = 0;
            }

            // Check for case 3, where target speed(vx) < 0
            if (mode_ == SynchMode::MODE_NON_LINEAR && vx < 0)
            {
                // In phase one, decelerate to 0, then stop
                // Calculate time needed to cover distance proportional to current speed / final speed
                double t1 = 2 * v0 * s / (v0 * v0 + v1 * v1);
                if (fabs(t1) > SMALL_NUMBER)
                {
                    acc = -v0 / t1;
                }
                else
                {
                    acc = 0;
                }

                if (t1 * v0 / 2 > s / 2)
                {
                    // If more than half distance to destination needed, then stop immediatelly
                    acc   = MAX_DECELERATION;
                    mode_ = SynchMode::MODE_STOP_IMMEDIATELY;
                    // PrintStatus("Stop immediately");
                }

                if (v0 + acc * dt < 0)
                {
                    // Reached to a stop
                    object_->SetSpeed(0);
                    mode_ = SynchMode::MODE_STOPPED;
                    // PrintStatus("Stopped");

                    return;
                }
            }
        }
        else
        {
            // No final speed specified. Calculate it based on current speed and available time
            double final_speed = 2 * average_speed - object_->speed_;
            acc                = (final_speed - object_->speed_) / masterTimeToDest;
        }

        object_->SetSpeed(object_->GetSpeed() + acc * dt);
    }
}

void VisibilityAction::Start(double simTime)
{
    OSCAction::Start(simTime);
    object_->SetVisibilityMask((graphics_ ? Object::Visibility::GRAPHICS : 0) | (traffic_ ? Object::Visibility::TRAFFIC : 0) |
                               (sensors_ ? Object::Visibility::SENSORS : 0));
    auto towVehicle = object_->TowVehicle();
    if (towVehicle)
    {
        towVehicle->SetVisibilityMask((graphics_ ? Object::Visibility::GRAPHICS : 0) | (traffic_ ? Object::Visibility::TRAFFIC : 0) |
                                      (sensors_ ? Object::Visibility::SENSORS : 0));
    }
    auto TrailerVehicle = object_->TrailerVehicle();
    if (TrailerVehicle)
    {
        TrailerVehicle->SetVisibilityMask((graphics_ ? Object::Visibility::GRAPHICS : 0) | (traffic_ ? Object::Visibility::TRAFFIC : 0) |
                                          (sensors_ ? Object::Visibility::SENSORS : 0));
    }
}

void VisibilityAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::End();
}

int OverrideControlAction::AddOverrideStatus(Object::OverrideActionStatus status)
{
    overrideActionList.push_back(status);
    if (status.type < Object::OverrideType::OVERRIDE_NR_TYPES)
    {
        if (status.type == Object::OverrideType::OVERRIDE_STEERING_WHEEL)
        {
            if (status.active)
            {
                domains_ = domains_ | static_cast<unsigned int>(ControlDomains::DOMAIN_LAT);
            }
            else
            {
                domains_ = domains_ & ~static_cast<unsigned int>(ControlDomains::DOMAIN_LAT);
            }
        }
        else
        {
            if (status.active)
            {
                domains_ = domains_ | static_cast<unsigned int>(ControlDomains::DOMAIN_LONG);
            }
            else
            {
                domains_ = domains_ & ~static_cast<unsigned int>(ControlDomains::DOMAIN_LONG);
            }
        }
    }
    else
    {
        LOG_AND_QUIT("Unexpected override type: %d", status.type);
    }

    return 0;
}

void OverrideControlAction::Start(double simTime)
{
    for (size_t i = 0; i < overrideActionList.size(); i++)
    {
        object_->overrideActionList[overrideActionList[i].type] = overrideActionList[i];
    }
    OSCAction::Start(simTime);
}

void OverrideControlAction::Step(double simTime, double dt)
{
    (void)simTime;
    (void)dt;

    OSCAction::End();
}

double OverrideControlAction::RangeCheckAndErrorLog(Object::OverrideType type, double valueCheck, double lowerLimit, double upperLimit, bool ifRound)
{
    double temp = valueCheck;
    if (valueCheck <= upperLimit && valueCheck >= lowerLimit)
    {
        if (!ifRound)
        {
            LOG("%d value %.2f is within range.", type, valueCheck);
        }
        else
        {
            valueCheck = round(temp);
            LOG("%d value %.1f is within range and the value is rounded to %.1f.", type, temp, valueCheck);
        }
    }
    else
    {
        valueCheck = (valueCheck > upperLimit) ? upperLimit : lowerLimit;
        LOG("%d value is not within range and is modified from %f to %.1f.", type, temp, valueCheck);
    }
    return valueCheck;
}
