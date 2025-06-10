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

#include "Controller.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "ScenarioEngine.hpp"
#include "logger.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateController(void* args)
{
    LOG_ERROR("The base class should not be instantiated");

    return new Controller(static_cast<Controller::InitArgs*>(args));
}

Controller::Controller(InitArgs* args)  // init operatingdomains
    : operating_domains_(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT_AND_LONG)),
      active_domains_(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE)),
      mode_(ControlOperationMode::MODE_OVERRIDE),
      object_(nullptr),
      entities_(nullptr),
      gateway_(nullptr),
      scenario_engine_(nullptr),
      player_(nullptr)
{
    if (args)
    {
        name_            = args->name;
        type_name_       = args->type;
        gateway_         = args->gateway;
        scenario_engine_ = args->scenario_engine;
        entities_        = scenario_engine_ != nullptr ? &scenario_engine_->entities_ : nullptr;
    }
    else
    {
        LOG_ERROR_AND_QUIT("Controller constructor missing args");
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
            LOG_WARN("Unexpected mode \"{}\", falling back to default \"override\"", mode);
            mode_ = ControlOperationMode::MODE_OVERRIDE;
        }
    }
    else
    {
        mode_ = ControlOperationMode::MODE_OVERRIDE;
    }
}

void Controller::Step(double timeStep)
{
    (void)timeStep;
    if (object_)
    {
        if (mode_ == ControlOperationMode::MODE_OVERRIDE)
        {
            if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
            {
                object_->SetDirtyBits(Object::DirtyBit::LATERAL);
            }

            if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG)))
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

void Controller::LinkObject(Object* object)
{
    object_ = object;
}

void Controller::UnlinkObject()
{
    object_ = nullptr;
}

int Controller::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    if (mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)] == ControlActivationMode::OFF && align_to_road_heading_on_deactivation_)
    {
        // Make sure heading is aligned with driving direction when controller is deactivated on the lateral domain
        if (IsActiveOnDomains(static_cast<int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
        {
            AlignToRoadHeading();
        }
    }

    if (mode[static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)] == ControlActivationMode::ON && align_to_road_heading_on_activation_)
    {
        // Make sure heading is aligned with driving direction when controller is activated on the lateral domain
        AlignToRoadHeading();
    }

    for (unsigned int i = 0; i < static_cast<unsigned int>(ControlDomains::COUNT); i++)
    {
        if (mode[i] == ControlActivationMode::OFF)
        {
            active_domains_ &= ~(static_cast<unsigned int>(ControlDomain2DomainMask(static_cast<ControlDomains>(i))));
        }
        else if (mode[i] == ControlActivationMode::ON)
        {
            if ((operating_domains_ & static_cast<unsigned int>(ControlDomain2DomainMask(static_cast<ControlDomains>(i)))) == 0)
            {
                LOG_WARN("Warning: Controller {} operating domains: {}. Skipping activation on domain {}",
                         GetName(),
                         ControlDomainMask2Str(operating_domains_),
                         ControlDomain2Str(static_cast<ControlDomains>(i)));
            }
            else
            {
                active_domains_ |= static_cast<unsigned int>(ControlDomain2DomainMask(static_cast<ControlDomains>(i)));
            }
        }
    }
    return 0;
}

void scenarioengine::Controller::DeactivateDomains(unsigned int domains)
{
    // Make sure heading is aligned with driving direction when controller is deactivated on the lateral domain
    if (align_to_road_heading_on_deactivation_ && IsActiveOnDomains(static_cast<int>(ControlDomainMasks::DOMAIN_MASK_LAT)) &&
        (domains & static_cast<int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        AlignToRoadHeading();
    }

    active_domains_ = active_domains_ & ~domains;
}

void Controller::ReportKeyEvent(int key, bool down)
{
    LOG_DEBUG("Key {} {}", key, down ? "down" : "up");
}

std::string Controller::Mode2Str(ControlOperationMode mode)
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
        LOG_ERROR("Unexpected mode \"{}\"", std::to_string(static_cast<int>(mode)));
        return "invalid mode";
    }
}

bool Controller::IsActiveOnDomainsOnly(unsigned int domainMask) const
{
    return (GetActiveDomains() == domainMask);
}

bool Controller::IsActiveOnDomains(unsigned int domainMask) const
{
    return (domainMask & GetActiveDomains()) == domainMask;
}

bool Controller::IsNotActiveOnDomains(unsigned int domainMask) const
{
    return (domainMask & GetActiveDomains()) == 0;
}

bool Controller::IsActiveOnAnyOfDomains(unsigned int domainMask) const
{
    return (domainMask & GetActiveDomains()) != 0;
}

bool Controller::IsActive() const
{
    return GetActiveDomains() != static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_NONE);
}

void scenarioengine::Controller::AlignToRoadHeading()
{
    if (object_ != nullptr)
    {
        object_->pos_.SetHeading(object_->pos_.GetHRoadInDrivingDirection());
    }
}
