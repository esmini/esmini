#pragma once

#include "Controller.hpp"

#include <utility>

using namespace scenarioengine::controller;

class ControllerAEB : public scenarioengine::controller::ControllerBase
{
public:
    ControllerAEB(InitArgs* args);
    //! Public interfaces
    Type GetType() const override;

    void Step(double timeStep) override;

private:

    void FindNearestObjectAndDistanceAhead(scenarioengine::Object* &nearest, double& distanceToNearest);

    void IsEmergencyBrakingNeeded(scenarioengine::Object* nearest, double distanceToNearest);

    void ApplyEmergencyBrake( double deltaTime);
    // Intensity of brake to be applied
    double brakeRate_ = 5;
    bool EmergencyBraking_ = false;
};

extern "C" EXPORTED ControllerBase* InstantiateController(void* args)
{
    InitArgs* initArgs = static_cast<InitArgs*>(args);
    return new ControllerAEB(initArgs);
}
