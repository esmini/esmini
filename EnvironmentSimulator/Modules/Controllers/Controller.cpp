#include "Controller.hpp"

#include "CommonMini.hpp"

namespace scenarioengine::controller
{

std::string ToStr(Type type)
{
    switch (type)
    {
        case CONTROLLER_TYPE_DEFAULT:
            return "CONTROLLER_TYPE_DEFAULT";
        case CONTROLLER_TYPE_EXTERNAL:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_FOLLOW_GHOST:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_FOLLOW_ROUTE:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_INTERACTIVE:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_SLOPPY_DRIVER:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_SUMO:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_REL2ABS:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_ACC:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_ALKS:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_UDP_DRIVER:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_ECE_ALKS_REF_DRIVER:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_ALKS_R157SM:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_LOOMING:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_OFFROAD_FOLLOWER:
            return "CONTROLLER_TYPE_EXTERNAL";
        case N_CONTROLLER_TYPES:
            return "CONTROLLER_TYPE_EXTERNAL";
        case CONTROLLER_TYPE_UNDEFINED:
            return "CONTROLLER_TYPE_EXTERNAL";
        case GHOST_RESERVED_TYPE:
            return "CONTROLLER_TYPE_EXTERNAL";
        case USER_CONTROLLER_TYPE_BASE:
            return "CONTROLLER_TYPE_EXTERNAL";
        default:
            return "CONTROLLER_UNKNOWN";
    }
}


std::string ToStr(ControlOperationMode mode)
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
        //Riz LOG("Unexpected mode \"%d\"", mode);
        return "invalid mode";
    }
}

std::string ToStr(ControlActivationMode mode)
{
    switch(mode)
    {
        case ControlActivationMode::UNDEFINED:
            return "UNDEFINED";
        case ControlActivationMode::OFF:
            return "OFF";
        case ControlActivationMode::ON:
            return "ON";
        default:
            return "MODE_UNKOWN";
    }
}

const std::any& BaseController::GetProperty(const std::string& propertyName) const
{
    return properties_.at(propertyName);
}

void BaseController::SetProperty(const std::string& propertyName, const std::any& propertyValue)
{
    properties_[propertyName] = propertyValue;
}
/*
void Controller::Step(double timeStep)
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
*/
uint32_t BaseController::GetOperatingDomains() const
{
    return operating_domains_;
}

uint32_t BaseController::GetActiveDomains() const 
{
    return active_domains_;
}

controller::ControlOperationMode BaseController::GetMode() const 
{
    return mode_;
}

void BaseController::LinkObjectByID(uint64_t id)
{
    // do we need any validation here that if there is an object present with the given id
    linkedObjectID_ = id;
}

void BaseController::UnlinkObject()
{
    linkedObjectID_ = 0;
}

int BaseController::Activate(ControlActivationMode lat_mode,
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

void BaseController::Deactivate()
{
    active_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_NONE);
}

void BaseController::DeactivateDomains(uint32_t domains)
{
    active_domains_ = active_domains_ & ~domains;
}

void BaseController::Init()
{

}

void BaseController::InitPostPlayer()
{

}

uint64_t BaseController::GetLinkedObjectID() const
{
    return linkedObjectID_;
}

void BaseController::ReportKeyEvent(int key, bool down)
{
    LOG("Key %c %s", key, down ? "down" : "up");
}

bool BaseController::IsActiveOnDomainsOnly(uint32_t domainMask) const
{
    return (GetActiveDomains() == domainMask);
}

bool BaseController::IsActiveOnDomains(uint32_t domainMask) const
{
    return (domainMask & GetActiveDomains()) == domainMask;
}

bool BaseController::IsNotActiveOnDomains(uint32_t domainMask) const
{
    return (domainMask & GetActiveDomains()) == 0;
}

bool BaseController::IsActiveOnAnyOfDomains(uint32_t domainMask) const
{
    return (domainMask & GetActiveDomains()) != 0;
}

bool BaseController::IsActive() const
{
    return GetActiveDomains() != static_cast<unsigned int>(ControlDomains::DOMAIN_NONE);
}

void BaseController::SetName(const std::string& name)
{
    name_ = name;
}

const std::string& BaseController::GetName() const
{
    return name_;
}
}