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
#include "RoadNetwork.hpp"
#include "Catalogs.hpp"
#include "Entities.hpp"
#include "Storyboard.hpp"
#include "OSCPosition.hpp"
#include "OSCProperties.hpp"
#include "pugixml.hpp"
#include "OSCGlobalAction.hpp"
#include "OSCBoundingBox.hpp"
#include "Parameters.hpp"
#include "Controller.hpp"
#include "ScenarioGateway.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace scenarioengine
{
    class ControllerPool
    {
    public:
        typedef struct
        {
            std::string                   type;
            ControllerInstantiateFunction instantiateFunction;
        } ControllerEntry;

        ControllerPool(){};

        std::vector<ControllerEntry> controller_;

        void AddController(std::string name, ControllerInstantiateFunction function)
        {
            ControllerEntry entry = {name, function};
            controller_.push_back(entry);
        }

        ControllerEntry* GetControllerByType(std::string type)
        {
            for (size_t i = 0; i < controller_.size(); i++)
            {
                if (controller_[i].type == type)
                {
                    return &controller_[i];
                }
            }
            return 0;
        }

        void Clear()
        {
            controller_.clear();
        }
    };

    class ScenarioReader
    {
    public:
        ScenarioReader(Entities* entities, Catalogs* catalogs, bool disable_controllers = false);
        ~ScenarioReader();
        int  loadOSCFile(const char* path);
        int  loadOSCMem(const pugi::xml_document& xml_doch);
        void SetGateway(ScenarioGateway* gateway)
        {
            gateway_ = gateway;
        }
        void SetScenarioEngine(ScenarioEngine* scenarioEngine)
        {
            scenarioEngine_ = scenarioEngine;
        }
        int RegisterCatalogDirectory(pugi::xml_node catalogDirChild);

        int parseOSCHeader();

        // RoadNetwork
        void                       parseRoadNetwork(RoadNetwork& roadNetwork);
        void                       parseOSCFile(OSCFile& file, pugi::xml_node fileNode);
        roadmanager::RMTrajectory* parseTrajectory(pugi::xml_node node);

        // Catalogs
        void      parseCatalogs();
        Catalog*  LoadCatalog(std::string name);
        Catalogs* GetCatalogs()
        {
            return catalogs_;
        }
        roadmanager::Route*               parseOSCRoute(pugi::xml_node routeNode);
        roadmanager::RMTrajectory*        parseTrajectoryRef(pugi::xml_node trajNode);
        void                              ParseOSCProperties(OSCProperties& properties, pugi::xml_node& xml_node);
        roadmanager::CoordinateSystem     ParseCoordinateSystem(pugi::xml_node node, roadmanager::CoordinateSystem defaultValue);
        roadmanager::RelativeDistanceType ParseRelativeDistanceType(pugi::xml_node node, roadmanager::RelativeDistanceType defaultValue);
        void                              ParseOSCBoundingBox(OSCBoundingBox& boundingbox, pugi::xml_node& xml_node);
        Vehicle*                          parseOSCVehicle(pugi::xml_node vehicleNode);
        Pedestrian*                       parseOSCPedestrian(pugi::xml_node pedestrianNode);
        MiscObject*                       parseOSCMiscObject(pugi::xml_node miscObjectNode);
        Vehicle*                          createRandomOSCVehicle(std::string name);
        Controller*                       parseOSCObjectController(pugi::xml_node vehicleNode);
        void                              parseGlobalParameterDeclarations()
        {
            parameters.parseGlobalParameterDeclarations(osc_root_.child("ParameterDeclarations"));
            variables.parseGlobalParameterDeclarations(osc_root_.child("VariableDeclarations"));
        }

        // Enitites
        int    parseEntities();
        Entry* ResolveCatalogReference(pugi::xml_node node);
        bool   CheckModelId(Object* object);

        // Storyboard - Init
        void                      parseInit(Init& init);
        ActivateControllerAction* parseActivateControllerAction(pugi::xml_node actionNode, Event* parent);
        int                       parseDynamicConstraints(pugi::xml_node dynamics_node, DynamicConstraints& dc, Object* obj);
        OSCPrivateAction*         parseOSCPrivateAction(pugi::xml_node actionNode, Object* object, Event* parent);
        OSCGlobalAction*          parseOSCGlobalAction(pugi::xml_node actionNode, Event* parent);
        OSCUserDefinedAction*     parseOSCUserDefinedAction(pugi::xml_node actionNode, Event* parent);
        void                      parseOSCOrientation(OSCOrientation& orientation, pugi::xml_node orientationNode);
        OSCPosition*              parseOSCPosition(pugi::xml_node positionNode, OSCPosition* base_on_pos = nullptr);

        // Storyboard - Story
        OSCCondition* parseOSCCondition(pugi::xml_node conditionNode);
        Trigger*      parseTrigger(pugi::xml_node triggerNode, bool defaultValue);
        //	void parseOSCConditionGroup(OSCConditionGroup *conditionGroup, pugi::xml_node conditionGroupNode);
        int  parseStoryBoard(StoryBoard& storyBoard);
        void parseOSCManeuver(Maneuver* maneuver, pugi::xml_node maneuverNode, ManeuverGroup* mGroup);

        std::string getScenarioFilename()
        {
            return oscFilename_;
        }
        bool IsLoaded()
        {
            return !osc_root_.empty();
        }

        static void RegisterController(std::string type_name, ControllerInstantiateFunction function)
        {
            ScenarioReader::controllerPool_.AddController(type_name, function);
        }

        void LoadControllers();
        void UnloadControllers();

        std::string GetDescription()
        {
            return description_;
        }
        int GetVersionMajor()
        {
            return versionMajor_;
        }
        int GetVersionMinor()
        {
            return versionMinor_;
        }

        int  RemoveController(Controller* controller);
        void AddController(Controller* controller)
        {
            controller_.push_back(controller);
        }
        pugi::xml_document* GetDXMLDocument()
        {
            return &doc_;
        }

        std::vector<Controller*> controller_;

        static Parameters parameters;  // static to enable set via callback during creation of object
        static Parameters variables;

    private:
        pugi::xml_document    doc_;
        pugi::xml_node        osc_root_;
        std::string           oscFilename_;
        Entities*             entities_;
        Catalogs*             catalogs_;
        ScenarioGateway*      gateway_;
        ScenarioEngine*       scenarioEngine_;
        bool                  disable_controllers_;
        static ControllerPool controllerPool_;
        int                   versionMajor_;
        int                   versionMinor_;
        std::string           description_;
        StoryBoard*           story_board_;

        int             ParseTransitionDynamics(pugi::xml_node node, OSCPrivateAction::TransitionDynamics& td);
        ConditionGroup* ParseConditionGroup(pugi::xml_node node);
        Object*         ResolveObjectReference(std::string name);
    };

}  // namespace scenarioengine
