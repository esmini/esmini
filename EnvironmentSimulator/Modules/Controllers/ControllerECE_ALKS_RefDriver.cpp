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
 * Written by: Thaddaeus Menzel, IDIADA Fahrzeugtechnik GmbH
 * This controller simulates the reference driver model from ECE ALKS regulation.
 * For more information look at:
 * https://unece.org/sites/default/files/2021-03/R157e.pdf, pages 43-45
 */

#include "ControllerECE_ALKS_RefDriver.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;

#define ALKS_LOG(...)         \
    {                         \
        if (logging_)         \
        {                     \
            LOG(__VA_ARGS__); \
        }                     \
        else                  \
        {                     \
            (void)0;          \
        }                     \
    }

Controller* scenarioengine::InstantiateControllerECE_ALKS_REF_DRIVER(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerECE_ALKS_REF_DRIVER(initArgs);
}

ControllerECE_ALKS_REF_DRIVER::ControllerECE_ALKS_REF_DRIVER(InitArgs* args)
    : Controller(args),
      active_(false),
      setSpeed_(0),
      currentSpeed_(0),
      logging_(false)
{
    mode_ = ControlOperationMode::MODE_ADDITIVE;

    if (args->properties->ValueExists("logging"))
    {
        if (args->properties->GetValueStr("logging") == "true")
        {
            logging_ = true;
        }
    }
}

void ControllerECE_ALKS_REF_DRIVER::Init()
{
    Controller::Init();
}

void ControllerECE_ALKS_REF_DRIVER::Step(double timeStep)
{
    //	double minGapLength = LARGE_NUMBER;
    double      minDist                  = 15.0;  // minimum distance to keep to lead vehicle
    double      maxDeceleration          = -10.0;
    double      normalAcceleration       = 3.0;
    std::string aebBrakeCandidateName    = "";
    double      candidateTTC             = LARGE_NUMBER;
    std::string driverBrakeCandidateName = "";

    double egoL  = object_->boundingbox_.dimensions_.length_;
    double egoW  = object_->boundingbox_.dimensions_.width_;
    double egoCx = object_->boundingbox_.center_.x_;
    double egoV  = object_->GetSpeed();

    double targetL  = 0.0;
    double targetW  = 0.0;
    double targetCx = 0.0;
    double targetO  = 0.0;
    double targetV  = 0.0;
    double targetVT = 0.0;
    double targetAS = 0.0;

    double dsFree     = 0.0;
    double acc        = 0.0;
    double TTC        = 0.0;
    double lastOffset = 0.0;

    // Lookahead distance is at least 50m or twice the distance required to stop
    // https://www.symbolab.com/solver/equation-calculator/s%5Cleft(t%5Cright)%3D2%5Cleft(m%2Bvt%2B%5Cfrac%7B1%7D%7B2%7Dat%5E%7B2%7D%5Cright)%2C%20t%3D%5Cfrac%7B-v%7D%7Ba%7D
    double lookaheadDist = MAX(50.0, minDist - pow(egoV, 2) / maxDeceleration);  // (m)

    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        if (entities_->object_[i] == object_)
        {
            continue;
        }
        if (aebBraking_ || egoV == 0)
        {
            // ego already stopped or AEB is already braking. AEB brakes harder than driver, no more need to continue checking for scenario
            break;
        }
        targetL  = entities_->object_[i]->boundingbox_.dimensions_.length_;
        targetW  = entities_->object_[i]->boundingbox_.dimensions_.width_;
        targetCx = entities_->object_[i]->boundingbox_.center_.x_;
        targetO  = entities_->object_[i]->pos_.GetOffset();
        targetV  = entities_->object_[i]->GetSpeed();
        targetVT = entities_->object_[i]->pos_.GetVelT();
        targetAS = entities_->object_[i]->pos_.GetAccS();

        // Measure longitudinal distance to all vehicles, don't utilize costly freespace option, instead measure ref point to ref point
        roadmanager::PositionDiff diff;
        if (object_->pos_.Delta(&entities_->object_[i]->pos_, diff, lookaheadDist) == true)
        {
            // path exists between position objects

            // object on adjacent lane with a deviation more than 0.375m from its lane center,
            // don't interprete swearving vehicles on adjecent lane as cut-in or cut-out
            if (fabs(diff.dLaneId) == 1 && egoV > 0)
            {
                // object with a deviation of more than 0.375m in direction of ego from its lane center
                if (!driverBraking_ &&
                    ((diff.dLaneId == -1 && targetVT > 0 && targetO > 0.375) || (diff.dLaneId == 1 && targetVT < 0 && targetO < -0.375)))
                {
                    // relative heading angles are playing no role for reference driver
                    dsFree = diff.ds - (0.5 * egoL + egoCx) - (0.5 * targetL - targetCx);
                    // check if the cut-in vehicle would be in front of ego after cut-in and within a TTC <=2sec, otherwise it can be ignored
                    TTC = dsFree / fabs(egoV - targetV);
                    if (TTC >= 0 && targetV < egoV)
                    {
                        if (TTC >= 2)
                        {
                            dsFree -= (egoV - targetV) * 0.4;  // see risk perception time = 0.4sec
                            TTC = dsFree / fabs(egoV - targetV);
                        }
                        if (TTC < 2)
                        {
                            // cut-in would be in the red zone in front of ego vehicle after lane change (at least after 0.4sec risk perception time
                            // or after 1.15sec including braking delay) TTC (< 2sec) + offset (> 0.375) 0.75sec braking delay + 0.4sec risk
                            // perception time (distance a and b in plot of regulation)
                            waitTime_ = 1.15;
                            ALKS_LOG(
                                "ECE ALKS driver -> cut-in detected of '%s' on adjacent lane (offset: %.3f, TTC: %.2f) in front of '%s' -> driver starts braking after %.2f sec "
                                "(braking delay + risk perception time)",
                                entities_->object_[i]->name_.c_str(),
                                fabs(targetO),
                                TTC,
                                object_->name_.c_str(),
                                waitTime_);
                            driverBraking_ = true;
                            cutInDetected_ = true;
                        }
                    }
                    if (!cutInDetected_)
                    {
                        ALKS_LOG("ECE ALKS driver -> cut-in detected of '%s' on adjacent lane (offset: %.3f, TTC: %.2f) in front of '%s'",
                                 entities_->object_[i]->name_.c_str(),
                                 fabs(targetO),
                                 TTC,
                                 object_->name_.c_str());
                        cutInDetected_ = true;
                    }
                }
                else if (dtFreeCutOut_ != -LARGE_NUMBER && dtFreeCutOut_ < 0 &&
                         ((diff.dLaneId == 1 && targetVT > 0) || (diff.dLaneId == -1 && targetVT < 0)))
                {
                    if (!driverBrakeCandidateName.empty() && candidateTTC < 2)
                    {
                        // 0.75sec braking delay + 0.4sec risk perception time (distance a and b in plot of regulation)
                        waitTime_ = 1.15;
                        ALKS_LOG(
                            "ECE ALKS driver -> cut-out '%s' on adjacent lane (offset: %.3f) and next vehicle in front '%s' detected (TTC: %.2f) in front of '%s' -> "
                            "start braking after %.2f sec (braking delay + risk perception time)",
                            entities_->object_[i]->name_.c_str(),
                            lastOffset,
                            driverBrakeCandidateName.c_str(),
                            candidateTTC,
                            object_->name_.c_str(),
                            waitTime_);
                        driverBraking_ = true;
                    }
                    dtFreeCutOut_ = fabs(diff.dt) - 0.5 * (egoW + targetW);
                }
            }
            // object in front on same lane
            else if (diff.dLaneId == 0 &&
                     diff.ds > 0)  // dLaneId == 0 indicates there is linked path between object lanes, i.e. no lane changes needed
            {
                // relative heading angles are playing no role for reference driver
                dsFree = diff.ds - (0.5 * egoL + egoCx) - (0.5 * targetL - targetCx);
                TTC    = dsFree / fabs(egoV - targetV);

                // cut-out with object on same lane, but with a distance of more than 0.375m from lane center and lateral velocity into opposite
                // direction of ego
                if ((targetO > 0.375 && targetVT > 0) || (targetO < -0.375 && targetVT < 0))
                {
                    if (dtFreeCutOut_ == -LARGE_NUMBER)
                    {
                        ALKS_LOG("ECE ALKS driver -> cut-out detected of '%s' on same lane (offset: %.3f, TTC: %.2f) in front of '%s'",
                                 entities_->object_[i]->name_.c_str(),
                                 fabs(targetO),
                                 TTC,
                                 object_->name_.c_str());
                        if (!driverBrakeCandidateName.empty())
                        {
                            // 0.75sec braking delay + 0.4sec risk perception time (distance a and b in plot of regulation)
                            waitTime_ = 1.15;
                            ALKS_LOG(
                                "ECE ALKS driver -> cut-out '%s' on same lane (offset: %.3f) and next vehicle in front '%s' detected (TTC: %.2f) in front of '%s' -> "
                                "start braking after %.2f sec (braking delay + risk perception time)",
                                entities_->object_[i]->name_.c_str(),
                                lastOffset,
                                driverBrakeCandidateName.c_str(),
                                candidateTTC,
                                object_->name_.c_str(),
                                waitTime_);
                            driverBraking_ = true;
                        }
                        driverBrakeCandidateName = entities_->object_[i]->name_;
                        candidateTTC             = TTC;
                        lastOffset               = fabs(targetO);
                    }
                    // relative heading angles are playing no role for reference driver
                    dtFreeCutOut_ = fabs(diff.dt) - 0.5 * (egoW + targetW);
                    if (!aebBrakeCandidateName.empty() && dtFreeCutOut_ >= 0)
                    {
                        ALKS_LOG("ECE ALKS AEB -> full wrap of '%s' and '%s' (TTC: %.2f) -> AEB starts braking",
                                 object_->name_.c_str(),
                                 aebBrakeCandidateName.c_str(),
                                 candidateTTC);
                        // AEB brakes harder than driver, no need to continue checking for scenario if aeb is already braking
                        aebBraking_ = true;
                        break;
                    }
                }
                // cut-in
                else if (!driverBraking_ && ((targetO > 0.375 && targetVT < 0) || (targetO < -0.375 && targetVT > 0)))
                {
                    if (TTC >= 2)
                    {
                        dsFree -= (egoV - targetV) * 0.4;
                        TTC = dsFree / fabs(egoV - targetV);
                    }
                    if (TTC < 2)
                    {
                        // cut-in would be in the red zone in front of ego vehicle after lane change (at least after 0.4sec risk perception time or
                        // after 1.15sec including braking delay) TTC (< 2sec) + offset (> 0.375) 0.75sec braking delay + 0.4sec risk perception time
                        // (distance a and b in plot of regulation)
                        waitTime_ = 1.15;
                        ALKS_LOG(
                            "ECE ALKS driver -> cut-in detected of '%s' on same lane (offset: %.3f, TTC: %.2f) in front of '%s' -> driver starts braking after %.2f sec "
                            "(braking delay + risk perception time)",
                            entities_->object_[i]->name_.c_str(),
                            fabs(targetO),
                            TTC,
                            object_->name_.c_str(),
                            waitTime_);
                        driverBraking_ = true;
                        cutInDetected_ = true;
                    }
                }
                // deceleration
                if (targetAS < 0 && dsFree >= 0 && TTC < 2)  // TTC of object in front < 2sec
                {
                    // from cut-in and cut-out one can conclude for deceleration scenario, that AEB is only braking in case of full wrap, right
                    // decision here??? AEB has here no delay, would directly brake as long as TTC <= TTC AEB (which is estimated to be the same as
                    // for driver 2sec) But by this definition the driver braking delay and perception time will play no role But in total one must
                    // not find here the exact definition because from regulation one knows that the driver always avoids a collision. Thus one can
                    // directly brake with AEB
                    if (fabs(diff.dt) < SMALL_NUMBER)
                    {
                        ALKS_LOG("ECE ALKS AEB -> full wrap of '%s' and '%s' (TTC: %.2f) -> AEB starts braking",
                                 object_->name_.c_str(),
                                 entities_->object_[i]->name_.c_str(),
                                 TTC);
                        aebBraking_ = true;
                        // AEB brakes harder than driver, no need to continue checking for scenario if aeb is already braking
                        break;
                    }
                    // in case of default scenarios the next is never happening because of full wrap and braking by AEB, that brakes even harder than
                    // the driver.
                    else if (!driverBraking_)
                    {
                        waitTime_ = 0.75;  // 0.75sec braking delay
                        if (targetAS < -5)
                        {
                            waitTime_ += 0.4;  // + 0.4sec risk perception time which begins when leading vehicle exceeds a deceleration of 5m/s2
                            ALKS_LOG(
                                "ECE ALKS driver -> deceleration detected of '%s' (as: %.2f, TTC: %.2f) in front of '%s' -> driver starts braking after %.2f sec "
                                "(braking delay + risk perception time)",
                                entities_->object_[i]->name_.c_str(),
                                fabs(targetAS),
                                TTC,
                                object_->name_.c_str(),
                                waitTime_);
                        }
                        else
                        {
                            ALKS_LOG(
                                "ECE ALKS driver -> deceleration detected of '%s' (as: %.2f, TTC: %.2f) in front of '%s' -> driver starts braking after %.2f sec "
                                "(braking delay, no risk perception time)",
                                entities_->object_[i]->name_.c_str(),
                                fabs(targetAS),
                                TTC,
                                object_->name_.c_str(),
                                waitTime_);
                        }
                        driverBraking_ = true;
                    }
                }
                else
                {
                    // There is a slower object in front at TTC < 2sec and if there was a cut-out before,
                    // then the cut-out vehicle has already left ego's driving path (lateral deviations omitted)
                    if ((dtFreeCutOut_ >= 0 || dtFreeCutOut_ == -LARGE_NUMBER) && targetV < egoV && dsFree >= 0 && TTC < 2)
                    {
                        // There is a full wrap with this object in front
                        // AEB only braking if there is a full wrap, right decision???
                        if (fabs(diff.dt) < SMALL_NUMBER)
                        {
                            // cut-out scenario or cut-in scenario have already been detected before
                            if (dtFreeCutOut_ >= 0 || driverBraking_)
                            {
                                ALKS_LOG("ECE ALKS AEB -> full wrap of '%s' and '%s' (TTC: %.2f) -> AEB starts braking",
                                         object_->name_.c_str(),
                                         entities_->object_[i]->name_.c_str(),
                                         TTC);
                                // AEB brakes harder than driver, no need to continue checking for scenario if aeb is already braking
                                aebBraking_ = true;
                                break;
                            }
                            else
                            {
                                // There is the possibility of cut-out scenario and that the cut-out vehicle was not examined before this vehicle,
                                // which should be in front of cut-out vehicle. But there is also the possibility that there is no cut-out scenario
                                // happening.
                                aebBrakeCandidateName = entities_->object_[i]->name_;
                                candidateTTC          = TTC;
                            }
                        }
                    }

                    // object in front
                    if (!driverBraking_ && dsFree >= 0)
                    {
                        // There was a cut-out vehicle in between with a deviation from its lane center larger than 0.375m (no swearving vehicle) and
                        // TTC < 2sec
                        if (dtFreeCutOut_ > -LARGE_NUMBER && TTC < 2 && !driverBrakeCandidateName.empty())
                        {
                            waitTime_ = 1.15;  // 0.75sec braking delay + 0.4sec risk perception time (distance a and b in plot of regulation)
                            ALKS_LOG(
                                "ECE ALKS driver -> cut-out '%s' on same lane (offset: %.3f) and next vehicle in front '%s' detected (TTC: %.2f) in front of '%s' -> "
                                "driver starts braking after %.2f sec (braking delay + risk perception time)",
                                driverBrakeCandidateName.c_str(),
                                lastOffset,
                                entities_->object_[i]->name_.c_str(),
                                TTC,
                                object_->name_.c_str(),
                                waitTime_);
                            driverBraking_ = true;
                        }
                        else
                        {
                            driverBrakeCandidateName = entities_->object_[i]->name_;
                            candidateTTC             = TTC;
                        }
                    }
                }
            }
        }
    }
    // The following cases are important for slower vehicles in front of ego at TTC < 2sec
    // especially important for cut-in and cut-out maeneuvers completed at TTC >= 2sec
    if (egoV > 0 && !aebBraking_)
    {
        if (!aebBrakeCandidateName.empty() && (dtFreeCutOut_ >= 0 || dtFreeCutOut_ == -LARGE_NUMBER) && candidateTTC < 2)
        {
            ALKS_LOG("ECE ALKS AEB -> full wrap of '%s' and '%s' (TTC: %.2f) -> AEB starts braking",
                     object_->name_.c_str(),
                     aebBrakeCandidateName.c_str(),
                     candidateTTC);
            // AEB brakes harder than driver, no need to continue checking for scenario if aeb is already braking
            aebBraking_ = true;
        }
        else if (!driverBraking_ && !driverBrakeCandidateName.empty() && candidateTTC < 2)
        {
            // 0.75sec braking delay + 0.4sec risk perception time (distance a and b in plot of regulation)
            waitTime_ = 1.15;
            ALKS_LOG(
                "ECE ALKS driver -> next vehicle in front '%s' detected (TTC: %.2f) -> start braking after %.2f sec (braking delay + risk perception time)",
                driverBrakeCandidateName.c_str(),
                candidateTTC,
                waitTime_);
            driverBraking_ = true;
        }
    }

    if (object_->CheckDirtyBits(Object::DirtyBit::SPEED))
    {
        // Speed has been set from somewhere else (another action or controller), respect it
        setSpeed_ = object_->GetSpeed();
    }

    currentSpeed_ = setSpeed_;  // only needed if there is no driver or AEB action
    if (egoV > 0)
    {
        // the AEB has no reaction time, thus the AEB is directly braking with a jerk of 0.85G during 0.6sec
        if (aebBraking_)
        {
            timeSinceBraking_ = MIN(timeSinceBraking_ + timeStep, 0.6);
            // jerk time of 0.6sec to decelerate with 0.85G
            // MAX comparing to 0, because it makes no sense to drive backwards
            acc           = timeSinceBraking_ / 0.6 * 0.85;
            currentSpeed_ = MAX(0, egoV - acc * 9.81 * timeStep);
            ALKS_LOG("ECE ALKS AEB -> '%s' braking from %.2f to %.2f (acc: %.3fG)",
                     object_->name_.c_str(),
                     egoV,
                     currentSpeed_,
                     MIN(acc, egoV / timeStep / 9.81));
        }
        // now the reference driver would brake
        else if (driverBraking_)
        {
            // the ego driver starts to react when the full wait time left and is braking with a jerk of 0.744G during 0.6sec
            if (waitTime_ == 0.0)
            {
                timeSinceBraking_ = MIN(timeSinceBraking_ + timeStep, 0.6);
                // jerk within time range of 0.6sec to decelerate with 0.774G
                // MAX comparing to 0, because it makes no sense to drive backwards
                acc           = timeSinceBraking_ / 0.6 * 0.774;
                currentSpeed_ = MAX(0, egoV - acc * 9.81 * timeStep);
                ALKS_LOG("ECE ALKS driver -> wait time passed -> '%s' braking from %.2f to %.2f (acc: %.3fG)",
                         object_->name_.c_str(),
                         egoV,
                         currentSpeed_,
                         MIN(acc, egoV / timeStep / 9.81));
            }
            waitTime_ = MAX(0.0, waitTime_ - timeStep);  // reduce waitTime by current timeStep each time this if branch is called
        }
    }

    if (!driverBraking_ && !aebBraking_)
    {
        // no lead vehicle to adapt to, adjust according to setSpeed (coming from ACC controller)
        double tmpSpeed = egoV + SIGN(setSpeed_ - currentSpeed_) * normalAcceleration * timeStep;
        if (SIGN(setSpeed_ - tmpSpeed) != SIGN(setSpeed_ - currentSpeed_))
        {
            // passed target speed
            currentSpeed_ = setSpeed_;
        }
        else
        {
            currentSpeed_ = tmpSpeed;
        }
    }

    object_->SetSpeed(currentSpeed_);
    gateway_->updateObjectSpeed(object_->GetId(), 0.0, object_->GetSpeed());

    if (currentSpeed_ == 0.0)
    {
        // after standstill by AEB or driver hold the velocity zero until a new velocity is set by a new controller
        Reset();
    }

    Controller::Step(timeStep);
}

int ControllerECE_ALKS_REF_DRIVER::Activate(ControlActivationMode lat_activation_mode,
                                            ControlActivationMode long_activation_mode,
                                            ControlActivationMode light_activation_mode,
                                            ControlActivationMode anim_activation_mode)
{
    Reset();

    Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);

    return 0;
}

void ControllerECE_ALKS_REF_DRIVER::Reset()
{
    setSpeed_         = object_->GetSpeed();
    dtFreeCutOut_     = -LARGE_NUMBER;
    cutInDetected_    = false;
    waitTime_         = -1.0;
    driverBraking_    = false;
    aebBraking_       = false;
    timeSinceBraking_ = 0.0;
}

void ControllerECE_ALKS_REF_DRIVER::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}
