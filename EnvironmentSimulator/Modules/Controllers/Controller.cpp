#include "Controller.hpp"
#include "CommonMini.hpp"
#include "OSCProperties.hpp"
#include "Parameters.hpp"
#include "Entities.hpp"

namespace scenarioengine::controller
{

    std::string ToStr(Type type)
    {
        switch (type)
        {
            case CONTROLLER_TYPE_DEFAULT:
                return "ControllerClass";
            case CONTROLLER_TYPE_EXTERNAL:
                return "ExternalController";
            case CONTROLLER_TYPE_FOLLOW_GHOST:
                return "FollowGhostController";
            case CONTROLLER_TYPE_FOLLOW_ROUTE:
                return "FollowRouteController";
            case CONTROLLER_TYPE_INTERACTIVE:
                return "InteractiveController";
            case CONTROLLER_TYPE_SLOPPY_DRIVER:
                return "SloppyDriverController";
            case CONTROLLER_TYPE_SUMO:
                return "SumoController";
            case CONTROLLER_TYPE_REL2ABS:
                return "ControllerRel2Abs";
            case CONTROLLER_TYPE_ACC:
                return "ACCController";
            case CONTROLLER_TYPE_ALKS:
                return "ALKSController";
            case CONTROLLER_TYPE_UDP_DRIVER:
                return "UDPDriverController";
            case CONTROLLER_TYPE_ECE_ALKS_REF_DRIVER:
                return "ECE_ALKS_RefDriverController";
            case CONTROLLER_ALKS_R157SM:
                return "ALKS_R157SM_Controller";
            case CONTROLLER_TYPE_LOOMING:
                return "LoomingController";
            case CONTROLLER_TYPE_OFFROAD_FOLLOWER:
                return "OffroadFollower";
            case N_CONTROLLER_TYPES:
                return "NController";
            case CONTROLLER_TYPE_UNDEFINED:
                return "UndefinedController";
            case GHOST_RESERVED_TYPE:
                return "GhostReservedController";
            case USER_CONTROLLER_TYPE_BASE:
                return "UserController";
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
            // Riz LOG("Unexpected mode \"%d\"", mode);
            return "invalid mode";
        }
    }

    std::string ToStr(ControlActivationMode mode)
    {
        switch (mode)
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

    ControllerBase::ControllerBase(InitArgs* args) : object_(0), entities_(0), gateway_(0), scenario_engine_(0), player_(0)
    {
        if (args)
        {
            operating_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LAT_AND_LONG);
            active_domains_    = static_cast<unsigned int>(ControlDomains::DOMAIN_NONE);
            mode_              = ControlOperationMode::MODE_OVERRIDE;
            name_              = args->name;
            // type_name_ = args->type;
            entities_ = args->entities;
            gateway_  = args->gateway;
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

    const std::any& ControllerBase::GetProperty(const std::string& propertyName) const
    {
        return properties_.at(propertyName);
    }

    void ControllerBase::SetProperty(const std::string& propertyName, const std::any& propertyValue)
    {
        properties_[propertyName] = propertyValue;
    }

    uint32_t ControllerBase::GetOperatingDomains() const
    {
        return operating_domains_;
    }

    uint32_t ControllerBase::GetActiveDomains() const
    {
        return active_domains_;
    }

    controller::ControlOperationMode ControllerBase::GetMode() const
    {
        return mode_;
    }

    void ControllerBase::LinkObjectByID(uint64_t id)
    {
        // do we need any validation here that if there is an object present with the given id
        linkedObjectID_ = id;
    }

    void ControllerBase::UnlinkObject()
    {
        linkedObjectID_ = 0;
    }

    int ControllerBase::Activate(ControlActivationMode lat_mode,
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

    void ControllerBase::Deactivate()
    {
        active_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_NONE);
    }

    void ControllerBase::DeactivateDomains(uint32_t domains)
    {
        active_domains_ = active_domains_ & ~domains;
    }

    void ControllerBase::Init()
    {
    }

    void ControllerBase::InitPostPlayer()
    {
    }

    uint64_t ControllerBase::GetLinkedObjectID() const
    {
        return linkedObjectID_;
    }

    void ControllerBase::ReportKeyEvent(int key, bool down)
    {
        LOG("Key %c %s", key, down ? "down" : "up");
    }

    bool ControllerBase::IsActiveOnDomainsOnly(uint32_t domainMask) const
    {
        return (GetActiveDomains() == domainMask);
    }

    bool ControllerBase::IsActiveOnDomains(uint32_t domainMask) const
    {
        return (domainMask & GetActiveDomains()) == domainMask;
    }

    bool ControllerBase::IsNotActiveOnDomains(uint32_t domainMask) const
    {
        return (domainMask & GetActiveDomains()) == 0;
    }

    bool ControllerBase::IsActiveOnAnyOfDomains(uint32_t domainMask) const
    {
        return (domainMask & GetActiveDomains()) != 0;
    }

    bool ControllerBase::IsActive() const
    {
        return GetActiveDomains() != static_cast<unsigned int>(ControlDomains::DOMAIN_NONE);
    }

    void ControllerBase::SetName(const std::string& name)
    {
        name_ = name;
    }

    const std::string& ControllerBase::GetName() const
    {
        return name_;
    }

    void ControllerBase::LinkObject(Object* object)
    {
        object_ = object;
    }

    void ControllerBase::SetScenarioEngine(ScenarioEngine* scenario_engine)
    {
        scenario_engine_ = scenario_engine;
    }

    void ControllerBase::SetPlayer(ScenarioPlayer* player)
    {
        player_ = player;
    }

    Object* ControllerBase::GetRoadObject()
    {
        return object_;
    }

    Object* ControllerBase::GetLinkedObject()
    {
        return object_;
    }

    void ControllerBase::Step(double timeStep)
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

}  // namespace scenarioengine::controller