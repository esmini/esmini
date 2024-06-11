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

#include "EmbeddedController.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"

namespace scenarioengine::controller
{
EmbeddedController::EmbeddedController(InitArgs* args)  // init operatingdomains
    : object_(0),
      entities_(0),
      gateway_(0),
      scenario_engine_(0),
      player_(0)
{
    if (args)
    {
        operating_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LAT_AND_LONG);
        active_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_NONE);
        mode_ = ControlOperationMode::MODE_OVERRIDE;
        // Riz
        /*
        name_      = args->name;
        type_name_ = args->type;
        */
        entities_  = args->entities;
        gateway_   = args->gateway;
    }
    else
    {
        LOG_AND_QUIT("Controller constructor missing args");
    }

    if (args->properties && args->properties->ValueExists("mode"))
    {
        std::string mode = args->properties->GetValueStr("mode");
        if (mode == "override")
        {
            mode_ = ControlOperationMode::MODE_OVERRIDE;
        }
        else if (mode == "additive")
        {
            mode_ = ControlOperationMode::MODE_ADDITIVE;
        }
        else
        {
            LOG("Unexpected mode \"%s\", falling back to default \"override\"", mode.c_str());
            mode_ = ControlOperationMode::MODE_OVERRIDE;
        }
    }
    else
    {
        mode_ = ControlOperationMode::MODE_OVERRIDE;
    }
}

void EmbeddedController::LinkObject(Object* object)
{
    object_ = object;
}

void EmbeddedController::UnlinkObject()
{
    object_ = nullptr;
}

void EmbeddedController::SetScenarioEngine(ScenarioEngine* scenario_engine)
{
    scenario_engine_ = scenario_engine;
}

void EmbeddedController::SetPlayer(ScenarioPlayer* player)
{
    player_ = player;
}

Object* EmbeddedController::GetRoadObject()
{
    return object_;
}

Object* EmbeddedController::GetLinkedObject()
{
    return object_;
}

void EmbeddedController::Step(double timeStep)
{
    (void)timeStep;
    if (object_)
    {
        if (mode_ == ControlOperationMode::MODE_OVERRIDE)
        {
            if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
            {
                object_->SetDirtyBits(Object::DirtyBit::LATERAL);
            }

            if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)))
            {
                object_->SetDirtyBits(Object::DirtyBit::LONGITUDINAL);
            }
        }
        else
        {
            object_->ClearDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);
        }
    }
}
}
// Riz - This should not be needed
/*
EmbeddedController* scenarioengine::InstantiateController(void* args)
{
    LOG("The base class should not be instantiated");

    return new EmbeddedController(static_cast<InitArgs*>(args));
}
*/
// Riz - moved to base class
/*
int EmbeddedController::Activate(ControlActivationMode lat_mode,
                         ControlActivationMode long_mode,
                         ControlActivationMode light_mode,
                         ControlActivationMode anim_mode)
{
    unsigned int control_domains[4] = {static_cast<unsigned int>(ControlDomains::DOMAIN_LAT),
                                       static_cast<unsigned int>(ControlDomains::DOMAIN_LONG),
                                       static_cast<unsigned int>(ControlDomains::DOMAIN_LIGHT),
                                       static_cast<unsigned int>(ControlDomains::DOMAIN_ANIM)};

    ControlActivationMode modes[4] = {lat_mode, long_mode, light_mode, anim_mode};

    for (unsigned int i = 0; i < 4; i++)
    {
        if (modes[i] == ControlActivationMode::OFF)
        {
            active_domains_ &= ~control_domains[i];
        }
        else if (modes[i] == ControlActivationMode::ON)
        {
            if ((operating_domains_ & control_domains[i]) == 0)
            {
                LOG("Warning: Controller %s operating domains: %s. Skipping activation on domain %s",
                    GetName().c_str(),
                    ControlDomain2Str(operating_domains_).c_str(),
                    ControlDomain2Str(control_domains[i]).c_str());
            }
            else
            {
                active_domains_ |= control_domains[i];
            }
        }
    }
    return 0;
}

void EmbeddedController::ReportKeyEvent(int key, bool down)
{
    LOG("Key %c %s", key, down ? "down" : "up");
}


std::string EmbeddedController::Mode2Str(ControlOperationMode mode)
{
    if (mode == ControlOperationMode::MODE_OVERRIDE)
    {
        return "override";
    }
    else if (mode == ControlOperationMode::MODE_ADDITIVE)
    {
        return "additive";
    }
    else if (mode == ControlOperationMode::MODE_NONE)
    {
        return "none";
    }
    else
    {
        LOG("Unexpected mode \"%d\"", mode);
        return "invalid mode";
    }
}

bool EmbeddedController::IsActiveOnDomainsOnly(unsigned int domainMask)
{
    return (GetActiveDomains() == domainMask);
}

bool EmbeddedController::IsActiveOnDomains(unsigned int domainMask)
{
    return (domainMask & GetActiveDomains()) == domainMask;
}

bool EmbeddedController::IsNotActiveOnDomains(unsigned int domainMask)
{
    return (domainMask & GetActiveDomains()) == 0;
}

bool EmbeddedController::IsActiveOnAnyOfDomains(unsigned int domainMask)
{
    return (domainMask & GetActiveDomains()) != 0;
}

bool EmbeddedController::IsActive()
{
    return GetActiveDomains() != static_cast<unsigned int>(ControlDomains::DOMAIN_NONE);
}
*/