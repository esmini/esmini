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

#pragma once

#include <string>
#include "CommonMini.hpp"
#include "OSCProperties.hpp"
#include "Parameters.hpp"
#include "Controller.hpp"

#define CONTROLLER_BASE_TYPE_NAME "ControllerClass"
#define CONTROLLER_BASE_TYPE_ID   -1

namespace scenarioengine
{
    /*
    // Forward declarations
    class ScenarioPlayer;
    class ScenarioGateway;
    class ScenarioEngine;
    class Entities;
    class Object;
    */

namespace controller
{

// base class for embedded controllers
class EmbeddedController : public controller::ControllerBase
{
public:

    EmbeddedController(InitArgs* args = nullptr);
    virtual ~EmbeddedController() = default;
    /*
    virtual void LinkObject(Object* object);
    virtual void UnlinkObject();
    virtual void SetScenarioEngine(ScenarioEngine* scenario_engine);
    virtual void SetPlayer(ScenarioPlayer* player);
    */
    Object*     GetRoadObject();
    Object* GetLinkedObject();
    // Base class Step function should be called from derived classes
    virtual void Step(double timeStep) override;      

protected:
    /*
    Object*              object_;             // The object to which the controller is attached and hence controls
    Entities*            entities_;
    ScenarioGateway*     gateway_;
    ScenarioEngine*      scenario_engine_;
    ScenarioPlayer*      player_;
    */
};

typedef ControllerBase* (*ControllerInstantiateFunction)(void* args);
ControllerBase* InstantiateController(void* args);

}   // namespace controller
    
}  // namespace scenarioengine