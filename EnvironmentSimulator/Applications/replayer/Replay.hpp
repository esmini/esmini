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
#include <fstream>
#include "CommonMini.hpp"
#include "ScenarioGateway.hpp"
#include "DatLogger.hpp"

namespace scenarioengine
{
    typedef struct
    {
        ObjectStateStructDat state;
        double               odometer;
    } ReplayEntry;

    typedef struct
    {
        char* pkg;
    } ObjectStateWithPkg;

    struct ObjectStateWithObjId
    {
        int                             id;
        bool                            active;
        double                          odometer = 0.0;
        std::vector<ObjectStateWithPkg> pkgs;
    };

    struct ScenarioState
    {
        double                            sim_time;
        std::vector<ObjectStateWithObjId> obj_states;
    };

    struct RestartTimes
    {
        double       restart_time_;
        double       next_time_ = LARGE_NUMBER;
        unsigned int restart_index_;
        unsigned int next_index_;
    };

    class Replay
    {
    public:
        Replay(const std::string directory, const std::string ScenarioEntities, std::string create_datfile);
        Replay(std::string filename);

        ~Replay();

        // vectors and methods to handle multiple files
        std::vector<std::pair<std::pair<std::string, bool>, std::vector<datLogger::CommonPkg>>> scenarioData;
        std::vector<int>                                                                        objectIds;
        void   AdjustObjectId(std::vector<std::vector<int>>& objectIds);
        void   GetReplaysFromDirectory(const std::string dir, const std::string sce);
        size_t GetNumberOfScenarios();

        int CreateMergedDatfile(const std::string filename);

        // vector and method for record and read pkg
        std::vector<datLogger::CommonPkg> pkgs_;
        ScenarioState                     scenarioState_;
        int                               RecordPkgs(const std::string& fileName);  // check package can be recorded or not
        std::vector<size_t>               GetNumberOfObjectsAtTime();               // till next time forward
        size_t                            GetPkgCntBtwObj(size_t idx);              // till next time forward
        datLogger::PackageId              ReadPkgHdr(char* package);
        double                            GetTimeFromCnt(int count);  // give time for the time
        void                              ExtractRestartTimes();
        std::vector<RestartTimes>         restartTimes_;

        // method for cache
        void InitiateStates();
        void UpdateCache();

        /**
         Go to specified timestamp
         @param time_frame Timestamp
         @param stopAtEachFrame Always stop for each step time frame
        */
        int GoToTime(double time_frame, bool stopAtEachFrame = false);
        /**
         Go to next time frame forward available in the dat file
        */
        void GoToNextFrame();
        /**
         Go to previous frame backward available in the dat file
        */
        void GoToPreviousFrame();
        /**
         * Go to given delta time
         */
        void GoToDeltaTime(double dt, bool stopAtEachFrame = false);
        void GoToStart();
        void GoToEnd();
        bool IsObjAvailableInCache(int Id);  // check in cache
        bool IsObjAvailableActive(int id) const;
        void UpdateObjStatus(int id, bool status);
        void CheckObjAvailabilityForward();
        void CheckObjAvailabilityBackward();

        // method to handle private members
        void   SetStartTime(double time);
        void   SetStopTime(double time);
        double GetStartTime();

        double                GetStopTime();
        double                GetTime();
        int                   GetIndex();
        void                  SetTime(double t);
        void                  SetIndex(unsigned int index);
        void                  SetRepeat(bool repeat);
        void                  UpdateOdaMeter(int obj_id, double value);
        void                  SetShowRestart(bool showRestart);
        void                  SetStopEntries();
        double                deltaTime_ = LARGE_NUMBER;
        datLogger::LightState perviousLightState;
        datLogger::LightState defaultLightState;
        bool                  show_lights     = false;
        bool                  IsLightPkgFound = false;
        // method to read data from cache
        int                     GetModelID(int obj_id);
        int                     GetCtrlType(int obj_id);
        int                     GetObjCategory(int obj_id);
        int                     GetBB(int obj_id, OSCBoundingBox& bb);
        int                     GetScaleMode(int obj_id);
        int                     GetVisibility(int obj_id);
        datLogger::Pos          GetPos(int obj_id);
        double                  GetX(int obj_id);
        double                  GetY(int obj_id);
        double                  GetZ(int obj_id);
        double                  GetH(int obj_id);
        double                  GetR(int obj_id);
        double                  GetP(int obj_id);
        id_t                    GetRoadId(int obj_id);
        int                     GetLaneId(int obj_id);
        double                  GetPosOffset(int obj_id);
        double                  GetPosT(int obj_id);
        double                  GetPosS(int obj_id);
        ObjectPositionStructDat GetComPletePos(int obj_id);
        double                  GetWheelAngle(int obj_id);
        double                  GetWheelRot(int obj_id);
        double                  GetSpeed(int obj_id);
        int                     GetName(int obj_id, std::string& name);
        double                  GetOdaMeter(int obj_id);
        void                    GetRgbValues(int obj_id, Object::VehicleLightActionStatus* light_state);
        void                    GetLightStates(int obj_id, datLogger::LightState& light_states_);

        const datLogger::DatHdr           GetHeader() const;
        std::vector<datLogger::CommonPkg> GetPkgs();

    private:
        std::ifstream            file_;
        std::vector<std::string> scenarios_;
        double                   time_;
        double                   startTime_;
        double                   stopTime_;
        unsigned int             startIndex_;
        unsigned int             stopIndex_;
        unsigned int             index_;
        bool                     repeat_;
        std::string              create_datfile_;
        bool                     show_restart_;
        bool                     IsRestart_    = false;
        double                   previousTime_ = std::nan("");

        datLogger::DatLogger* datLogger_ = nullptr;
        datLogger::DatHdr     header_;

        int    FindIndexAtTimestamp(double timestamp, int startSearchIndex = 0);
        bool   IsValidPocket(id_t id);
        void   BuildData();
        void   AddObjState(size_t objId);  // add the object state for given object id from the current object state
        void   DeleteObjState(int objId);
        bool   IsObjectDeletePkg(size_t index) const;
        bool   IsObjIdAddPkg(size_t index) const;
        bool   IsObjIdPkg(size_t index) const;
        bool   IsTimePkg(size_t index) const;
        double GetDoubleContent(size_t index);
        int    GetIntContent(size_t index);
        int    GetIntFromPkg(datLogger::CommonPkg* pkg);
        double GetDoubleFromPkg(datLogger::CommonPkg* pkg);
        int    GetIntFromScenarioState(int obj_id, datLogger::PackageId id);
        double GetDoubleFromScenarioState(int obj_id, datLogger::PackageId id);
    };

}  // namespace scenarioengine