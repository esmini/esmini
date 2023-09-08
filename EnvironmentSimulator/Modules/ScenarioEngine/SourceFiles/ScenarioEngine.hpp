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

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include "Catalogs.hpp"
#include "Entities.hpp"
#include "Init.hpp"
#include "Story.hpp"
#include "ScenarioGateway.hpp"
#include "ScenarioReader.hpp"
#include "RoadNetwork.hpp"
#include "ActionServer.hpp"

namespace scenarioengine
{
    typedef void (*ParamDeclCallbackFunc)(void *);

    struct CallBack
    {
        ParamDeclCallbackFunc func;
        void                 *data;
    };

    void RegisterParameterDeclarationCallback(ParamDeclCallbackFunc func, void *data);

    typedef struct
    {
        Object *object0;
        Object *object1;
    } CollisionPair;

    enum class GhostMode
    {
        NORMAL,
        RESTART,    // the frame ghost is requested to restart
        RESTARTING  // ghost restart is ongoing, including the final restart timestep
    };

    class ScenarioEngine
    {
    public:
        Entities                    entities_;
        std::vector<CollisionPair>  collision_pair_;
        actionserver::ServerActions serverActions_;

        ScenarioEngine(std::string oscFilename, bool disable_controllers = false);
        ScenarioEngine(const pugi::xml_document &xml_doc, bool disable_controllers = false);
        ~ScenarioEngine();

        void InitScenarioCommon(bool disable_controllers);
        int  InitScenario(std::string oscFilename, bool disable_controllers = false);
        int  InitScenario(const pugi::xml_document &xml_doc, bool disable_controllers = false);

        /**
        Step scenario, i.e. evaluate conditions and step actions
        @param deltaSimTime timestep
        @return 0 = OK normal, 1 = OK scenario done, -1 = NOK error
        */
        int  step(double deltaSimTime);
        void printSimulationTime();
        void prepareGroundTruth(double dt);
        int  defaultController(Object *obj, double dt);

        void ReplaceObjectInTrigger(Trigger *trigger, Object *obj1, Object *obj2, double timeOffset, Event *event = 0);
        void SetupGhost(Object *object);
        void ResetEvents();
        int  DetectCollisions();

        std::string getScenarioFilename()
        {
            return scenarioReader->getScenarioFilename();
        }
        std::string getSceneGraphFilename()
        {
            return roadNetwork.sceneGraphFile.filepath;
        }
        std::string getOdrFilename()
        {
            return roadNetwork.logicFile.filepath;
        }
        roadmanager::OpenDrive *getRoadManager()
        {
            return odrManager;
        }

        ScenarioGateway *getScenarioGateway();
        double           getSimulationTime()
        {
            return simulationTime_;
        }
        bool GetQuitFlag()
        {
            return quit_flag;
        }
        ScenarioReader *scenarioReader;
        ScenarioReader *GetScenarioReader()
        {
            return scenarioReader;
        }
        void SetHeadstartTime(double headstartTime)
        {
            headstart_time_ = headstartTime;
        }
        double GetHeadstartTime()
        {
            return headstart_time_;
        }
        void SetSimulationTime(double time)
        {
            simulationTime_ = time;
        }
        double *GetSimulationTimePtr()
        {
            return &simulationTime_;
        }

        void SetTrueTime(double time)
        {
            trueTime_ = time;
        }
        double GetTrueTime()
        {
            return trueTime_;
        }
        double *GetTrueTimePtr()
        {
            return &trueTime_;
        }
        void CreateGhostTeleport(Object *obj1, Object *obj2, Event *event);
        void SetGhostRestart()
        {
            ghost_mode_ = GhostMode::RESTART;
        }
        GhostMode GetGhostMode()
        {
            return ghost_mode_;
        }
        void UpdateGhostMode();
        int  GetInitStatus()
        {
            return init_status_;
        }

        double   trueTime_;
        bool     doOnce = true;
        SE_Mutex mutex_;

    private:
        // OpenSCENARIO parameters
        Catalogs                catalogs;
        Init                    init;
        StoryBoard              storyBoard;
        RoadNetwork             roadNetwork;
        roadmanager::OpenDrive *odrManager;
        bool                    disable_controllers_;

        // Simulation parameters
        double          simulationTime_;
        double          headstart_time_;
        GhostMode       ghost_mode_;
        Vehicle         sumotemplate;
        ScenarioGateway scenarioGateway;

        // execution control flags
        bool         quit_flag;
        unsigned int frame_nr_;
        int          init_status_;

        int parseScenario();
    };

}  // namespace scenarioengine
