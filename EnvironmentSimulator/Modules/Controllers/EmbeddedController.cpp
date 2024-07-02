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
        : ControllerBase(args)
    {
    }
    /*
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

    void EmbeddedController::SetPlayver(ScenarioPlayer* player)
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
    */
    void EmbeddedController::Step(double timeStep)
    {
        ControllerBase::Step(timeStep);
    }
}  // namespace scenarioengine::controller
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