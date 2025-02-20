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
#include "Storyboard.hpp"
#include "ScenarioGateway.hpp"
#include "ScenarioReader.hpp"
#include "RoadNetwork.hpp"

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

    struct PairHash
    {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& pair) const
        {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    class ScenarioEngine
    {
    public:
        Entities                   entities_;
        std::vector<CollisionPair> collision_pair_;
        std::vector<OSCAction *>  *injected_actions_;

        ScenarioEngine(std::string oscFilename, bool disable_controllers = false);
        ScenarioEngine(const pugi::xml_document &xml_doc, bool disable_controllers = false);
        ~ScenarioEngine();

        void InitScenarioCommon(bool disable_controllers);
        int  InitScenario(std::string oscFilename, bool disable_controllers = false);
        int  InitScenario(const pugi::xml_document &xml_doc, bool disable_controllers = false);
        void SetInjectedActionsPtr(std::vector<OSCAction *> *injected_actions)
        {
            injected_actions_ = injected_actions;
        }

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
        void ParseGlobalDeclarations();
        void EraseCleanParams();
        void EraseCleanVariables();
        void GetIdxsFromIds(const int id_1, const int id_2, int &idx_1, int &idx_2);
        bool CheckTeleported(const std::pair<int, int> pair);
        int  GetDistance(int id_1, int id_2, roadmanager::RelativeDistanceType dist_type, double& distance, double& timestamp);
        void UpdateDistance(const std::pair<int, int> ids, Object* obj_1, Object* obj_2, roadmanager::RelativeDistanceType dist_type);
        bool GetDisableControllersFlag()
        {
            return disable_controllers_;
        }
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
        double           getSimulationTime() const
        {
            return simulationTime_;
        }
        bool GetQuitFlag()
        {
            return storyBoard.GetCurrentState() == StoryBoard::State::COMPLETE;
        }
        ScenarioReader *scenarioReader;
        ScenarioReader *GetScenarioReader()
        {
            return scenarioReader;
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

        void UpdateGhostMode();
        int  GetInitStatus()
        {
            return init_status_;
        }

#ifdef _USE_OSI
        void SetOSIReporter(OSIReporter *osi_reporter)
        {
            storyBoard.SetOSIReporter(osi_reporter);
        }
#endif  // _USE_OSI

        double   trueTime_;
        bool     doOnce = true;
        SE_Mutex mutex_;

        StoryBoard storyBoard;

    private:

        // OpenSCENARIO parameters
        Catalogs                catalogs;
        RoadNetwork             roadNetwork;
        roadmanager::OpenDrive *odrManager;
        bool                    disable_controllers_;

        // Simulation parameters
        double          simulationTime_;
        Vehicle         sumotemplate;
        ScenarioGateway scenarioGateway;
        Object         *ghost_;
        
        // Distance map
        struct Measurement
        {
            double distance_;
            double timestamp_;
        };

        struct Distance
        {
            std::array<Object*, 2> objects_;
            std::unordered_map<roadmanager::RelativeDistanceType, Measurement> distance_;
            double next_update_;
        };

        std::unordered_map<std::pair<int, int>, Distance, PairHash> object_distance_map_;

        // execution control flags
        unsigned int frame_nr_;
        int          init_status_;

        int parseScenario();
    };

}  // namespace scenarioengine
