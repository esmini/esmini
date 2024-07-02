#pragma once

#include <string>
#include <any>
#include <map>

// Define EXPORTED for any platform
#if defined _WIN32 || defined __CYGWIN__
#ifdef WIN_EXPORT
// Exporting...
#ifdef __GNUC__
#define EXPORTED __attribute__((dllexport))
#else
#define EXPORTED __declspec(dllexport)  // Note: actually gcc seems to also supports this syntax.
#endif
#else
#ifdef __GNUC__
#define EXPORTED __attribute__((dllimport))
#else
#define EXPORTED __declspec(dllimport)  // Note: actually gcc seems to also supports this syntax.
#endif
#endif
#define NOT_EXPORTED
#else
#if __GNUC__ >= 4
#define EXPORTED     __attribute__((visibility("default")))
#define NOT_EXPORTED __attribute__((visibility("hidden")))
#else
#define EXPORTED
#define NOT_EXPORTED
#endif
#endif

namespace scenarioengine
{

    // Forward declarations
    class ScenarioPlayer;
    class ScenarioGateway;
    class ScenarioEngine;
    class Entities;
    class Object;
    class OSCProperties;
    class Parameters;

    namespace controller
    {

        typedef struct
        {
            std::string      name;
            std::string      type;
            OSCProperties*   properties;
            Entities*        entities;
            ScenarioGateway* gateway;
            Parameters*      parameters;
        } InitArgs;

        enum Type
        {
            CONTROLLER_TYPE_DEFAULT,
            CONTROLLER_TYPE_EXTERNAL,
            CONTROLLER_TYPE_FOLLOW_GHOST,
            CONTROLLER_TYPE_FOLLOW_ROUTE,
            CONTROLLER_TYPE_INTERACTIVE,
            CONTROLLER_TYPE_SLOPPY_DRIVER,
            CONTROLLER_TYPE_SUMO,
            CONTROLLER_TYPE_REL2ABS,
            CONTROLLER_TYPE_ACC,
            CONTROLLER_TYPE_ALKS,
            CONTROLLER_TYPE_UDP_DRIVER,
            CONTROLLER_TYPE_ECE_ALKS_REF_DRIVER,
            CONTROLLER_ALKS_R157SM,
            CONTROLLER_TYPE_LOOMING,
            CONTROLLER_TYPE_OFFROAD_FOLLOWER,
            N_CONTROLLER_TYPES,
            CONTROLLER_TYPE_UNDEFINED,
            GHOST_RESERVED_TYPE       = 100,
            USER_CONTROLLER_TYPE_BASE = 1000,
        };

        std::string ToStr(Type type);

        enum class ControlOperationMode
        {
            MODE_NONE     = 0,  // Controller not available or it is not active
            MODE_OVERRIDE = 1,  // Actions from the scenario are not applied, default
            MODE_ADDITIVE = 2,  // Actions from the scenario are applied
        };
        std::string ToStr(ControlOperationMode mode);

        enum class ControlActivationMode
        {
            UNDEFINED = 0,
            OFF       = 1,
            ON        = 2
        };
        std::string ToStr(ControlActivationMode mode);

        class ControllerBase
        {
        public:
            // public interface
            ControllerBase(InitArgs* args = nullptr);
            virtual ~ControllerBase() = default;
            // Returns name of the controller
            const std::string& GetName() const;
            // Returns type of the controller
            virtual Type GetType() const = 0;
            // Returns specified property in the parameter
            const std::any&      GetProperty(const std::string& propertyName) const;
            uint32_t             GetOperatingDomains() const;
            uint32_t             GetActiveDomains() const;
            ControlOperationMode GetMode() const;
            bool                 IsActiveOnDomainsOnly(unsigned int domainMask) const;
            bool                 IsActiveOnDomains(unsigned int domainMask) const;
            bool                 IsNotActiveOnDomains(unsigned int domainMask) const;
            bool                 IsActiveOnAnyOfDomains(unsigned int domainMask) const;
            bool                 IsActive() const;
            uint64_t             GetLinkedObjectID() const;

            void SetName(const std::string& name);
            // Set property
            void         SetProperty(const std::string& propertyName, const std::any& propertyValue);
            void         LinkObjectByID(uint64_t id);
            void         UnlinkObject();
            virtual int  Activate(ControlActivationMode lat_mode,
                                  ControlActivationMode long_mode,
                                  ControlActivationMode light_mode,
                                  ControlActivationMode anim_mode);
            virtual void Deactivate();  // only one drive class had its own implementation
            void         DeactivateDomains(uint32_t domains);
            virtual void Init();
            virtual void InitPostPlayer();
            virtual void ReportKeyEvent(int key, bool down);

            virtual void LinkObject(Object* object);
            virtual void SetScenarioEngine(ScenarioEngine* scenario_engine);
            virtual void SetPlayer(ScenarioPlayer* player);
            Object*      GetRoadObject();
            Object*      GetLinkedObject();

            // Base class Step function should be called from derived classes
            virtual void Step(double timeStep);

        protected:
            std::map<std::string, std::any> properties_;
            uint32_t                        operating_domains_;  // bitmask representing domains controller is operating on
            uint32_t                        active_domains_;     // bitmask representing domains controller is currently active on
            ControlOperationMode            mode_;               // add to scenario actions or replace
            uint64_t                        linkedObjectID_ = 0;
            std::string                     name_;

            Object*          object_;  // The object to which the controller is attached and hence controls
            Entities*        entities_;
            ScenarioGateway* gateway_;
            ScenarioEngine*  scenario_engine_;
            ScenarioPlayer*  player_;
        };

    }  // namespace controller

}  // namespace scenarioengine