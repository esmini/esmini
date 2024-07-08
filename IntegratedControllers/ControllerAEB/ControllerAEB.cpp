#include "ControllerAEB.h"

#include "CommonMini.hpp"
#include "OSCProperties.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

using namespace scenarioengine;
using namespace scenarioengine::controller;

ControllerAEB::ControllerAEB(InitArgs* args)
: scenarioengine::controller::ControllerBase(args)
{
    mode_ = ControlOperationMode::MODE_ADDITIVE;
    operating_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LONG);
    if (args && args->properties && args->properties->ValueExists("brakeRate"))
    {
        brakeRate_ = strtod(args->properties->GetValueStr("brakeRate"));
    }

    LOG("AEB controller initialized with brake rate: %f", brakeRate_);
}

Type ControllerAEB::GetType() const
{
    return USER_CONTROLLER_TYPE_BASE;
}

void ControllerAEB::FindNearestObjectAndDistanceAhead(scenarioengine::Object* &nearest, double& distanceToNearest)
{
    constexpr double lookaheadDist = 280;  // We are only interested in objects within 100m range    
    constexpr double lateralDist = 5;
    double minDistance = LARGE_NUMBER;
    for ( const auto& obj : entities_->object_)
    {        
        if (obj == object_)
        {
            continue;
        }
        roadmanager::PositionDiff diff;
        if (object_->pos_.Delta(&obj->pos_, diff, false, lookaheadDist) == true)  // look only double timeGap ahead
        {
            double distance = diff.ds;

            if (diff.dLaneId == 0 && distance > 0 && distance < minDistance && abs(diff.dt) < lateralDist)
            {
                minDistance = distance;
                nearest = obj;
                distanceToNearest = minDistance;          
            }            
        }
    }
}


void ControllerAEB::Step(double timeStep)
{          
    if( object_->GetSpeed() > 0)    // if EGO is moving
    {        
        if( EmergencyBraking_)      // if EGO is already braking then we continue it
        {
            ApplyEmergencyBrake(timeStep);
        }
        else                       // otherwise we will check if emergency braking is needed 
        {
            scenarioengine::Object* nearest = nullptr;
            double distance = 0;
            FindNearestObjectAndDistanceAhead(nearest, distance);    
            if( nearest != nullptr)
            {                
                IsEmergencyBrakingNeeded(nearest, distance);        
            }
        }
    }    
    ControllerBase::Step(timeStep);
}


void ControllerAEB::IsEmergencyBrakingNeeded(scenarioengine::Object* nearest, double distanceToNearest)
{    
    double freeSpaceToNearest = distanceToNearest;
    double speedDiff;// = object_->GetSpeed() - nearest->GetSpeed();
    double dHeading          = GetAbsAngleDifference(object_->pos_.GetH(), nearest->pos_.GetH());
    // assuming vehicle speed in same or opposite direction
    if (dHeading < M_PI_2)  // objects are pointing roughly in the same direction
    {
        freeSpaceToNearest -=
            (static_cast<double>(object_->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(object_->boundingbox_.center_.x_)) +
            (static_cast<double>(nearest->boundingbox_.dimensions_.length_) / 2.0 -
                static_cast<double>(nearest->boundingbox_.center_.x_));
        speedDiff = object_->GetSpeed() - nearest->GetSpeed();        
    }
    else  // objects are pointing roughly in the opposite direction
    {
        freeSpaceToNearest -=
            (static_cast<double>(object_->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(object_->boundingbox_.center_.x_)) +
            (static_cast<double>(nearest->boundingbox_.dimensions_.length_) / 2.0 +
                static_cast<double>(nearest->boundingbox_.center_.x_));
        speedDiff = object_->GetSpeed() + nearest->GetSpeed();        // as they are moving in opposite direction
    }
    
    if( speedDiff <= 0)
    {
        // The nearest vehicle is faster then EGO, so there is no chance of collision under given circumstances
        return;
    }

    const double safetyDistance = 10;    

    // Calculate required required distance to reach delta speed 0
    // solve s=v*t+(a*t^2)/2, v+a*t=0 for s,t
    // https://www.wolframalpha.com/input?i=solve+s%3Dv*t%2B%28a*t%5E2%29%2F2%2C+v%2Ba*t%3D0+for+s%2Ct

    double requiredDistanceToAvoidCollision = -(speedDiff * speedDiff) / (2 * -brakeRate_);
    if ( freeSpaceToNearest < requiredDistanceToAvoidCollision + safetyDistance)
    {
        LOG("AEB controller going to apply emergency brake, freeSpaceToNearest:%f, requiredDistanceToAvoidCollision:%f", freeSpaceToNearest, requiredDistanceToAvoidCollision);
        EmergencyBraking_ = true;
    }

}


void ControllerAEB::ApplyEmergencyBrake( double deltaTime)
{
    double newSpeed = object_->GetSpeed() - (brakeRate_ * deltaTime);
    if( newSpeed < 0)
    {
        newSpeed = 0;
    }
    gateway_->updateObjectSpeed(object_->GetId(), 0.0, newSpeed);
}