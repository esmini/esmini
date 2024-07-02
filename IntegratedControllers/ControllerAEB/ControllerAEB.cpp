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
    else
    {
        std::cout << "brakeRate not found in scenario file, setting default value:" << brakeRate_ << '\n';
    }
}

Type ControllerAEB::GetType() const
{
    return USER_CONTROLLER_TYPE_BASE;
}

void ControllerAEB::FindNearestObjectAndDistanceAhead(scenarioengine::Object* &nearest, double& distanceToNearest)
{
    constexpr double lookaheadDist = 100;  // We are only interested in objects within 100m range    
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
            double dHeading = GetAbsAngleDifference(object_->pos_.GetH(), obj->pos_.GetH());

            if (diff.dLaneId == 0 && distance > 0 && distance < minDistance && abs(diff.dt) < lateralDist)
            {
                minDistance = distance;
                nearest = obj;
                distanceToNearest = minDistance;
                std::cout << "found " << obj->GetName() << " nearby with distance:" << distance << '\n';              
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
    double speedDiff = object_->GetSpeed() - nearest->GetSpeed();
    if( speedDiff <= 0)
    {
        // The nearest vehicle is faster then EGO, so there is no chance of collision under given circumstances
        std::cout << "no danger of collision" << '\n';;
        return;
    }
    const double safetyDistance = 20;
    double collisionAvoidanceDistance = distanceToNearest - safetyDistance;
    double timeToCollision = collisionAvoidanceDistance / speedDiff;    
    double acceleration = -speedDiff/(2*timeToCollision);
    std::cout << "speedDiff:" << speedDiff << ", collisionAvoidanceDistance:" << collisionAvoidanceDistance << ", timeToCollision:" 
        << timeToCollision << ", acceleration:" << acceleration << '\n';;
    if( acceleration < -brakeRate_)
    {
        std::cout << "------!!!going to apply emergency brake!!!--------\n";
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
    // roadmanager::PositionDiff diff;
    // object_->pos_.Delta(&entities_->object_[1]->pos_, diff, false, 200); 
    // std::cout << "Applying AEB, EGO CurrentSpeed:" << object_->GetSpeed() << " newSpeed:" <<  newSpeed << " dist: " << diff.ds << '\n';;
    gateway_->updateObjectSpeed(object_->GetId(), 0.0, newSpeed);
}