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

#include "Replay.hpp"
#include "ScenarioGateway.hpp"
#include "CommonMini.hpp"
#include "dirent.h"
#include "DatLogger.hpp"

using namespace scenarioengine;

Replay::Replay(std::string filename) : time_(0.0), index_(0), repeat_(false), show_restart_(false)
{
    RecordPkgs(filename);  // read the dat file and store the data in pkgs_

    // round time step to closest 6:th decimal of a second
    deltaTime_ = std::round(deltaTime_ / SMALL_NUMBER) * SMALL_NUMBER;

    LOG_INFO("Recording {} opened. dat version: {} odr: {} model: {}",
             FileNameOf(filename),
             header_.version,
             FileNameOf(header_.odrFilename.string.data()),
             FileNameOf(header_.modelFilename.string.data()));

    if (header_.version != DAT_FILE_FORMAT_VERSION)
    {
        LOG_ERROR_AND_QUIT("Version mismatch. {} is version {} while supported version is {}. Please re-create dat file.",
                           filename,
                           header_.version,
                           DAT_FILE_FORMAT_VERSION);
    }

    if (pkgs_.size() > 0)
    {
        // Register first entry timestamp as starting time
        time_       = *reinterpret_cast<double*>(pkgs_[0].content.data());
        startTime_  = time_;
        startIndex_ = 0;

        // Register last entry timestamp as stop time
        SetStopEntries();
    }
    // initiate the cache with first time frame
    InitiateCache();
}

Replay::Replay(const std::string directory, const std::string scenario, std::string create_datfile)
    : time_(0.0),
      index_(0),
      repeat_(false),
      show_restart_(false)
{
    GetReplaysFromDirectory(directory, scenario);

    std::vector<std::vector<int>> scenarioObjIds;

    for (size_t i = 0; i < scenarios_.size(); i++)
    {
        RecordPkgs(scenarios_[i]);
        // round time step to closest 6:th decimal of a second
        deltaTime_ = std::round(deltaTime_ / SMALL_NUMBER) * SMALL_NUMBER;
        scenarioObjIds.push_back(objectIds);

        LOG_INFO("Recording {} opened. dat version: {} odr: {} model: {}",
                 FileNameOf(scenarios_[i]),
                 header_.version,
                 FileNameOf(header_.odrFilename.string.data()),
                 FileNameOf(header_.modelFilename.string.data()));

        if (header_.version != DAT_FILE_FORMAT_VERSION)
        {
            LOG_ERROR_AND_QUIT("Version mismatch. {} is version {} while supported version is {}. Please re-create dat file.",
                               scenarios_[i],
                               header_.version,
                               DAT_FILE_FORMAT_VERSION);
        }
        scenarioData.push_back(std::make_pair(std::make_pair(scenarios_[i], true), pkgs_));
        pkgs_ = {};
    }

    if (scenarioData.size() < 2)
    {
        LOG_ERROR_AND_QUIT("Too few scenarios loaded, use single replay feature instead\n");
    }

    AdjustObjectId(scenarioObjIds);

    // Build remaining data in order.
    BuildData();

    if (pkgs_.size() > 0)
    {
        // Register first entry timestamp as starting time
        time_       = *reinterpret_cast<double*>(pkgs_[0].content.data());
        startTime_  = time_;
        startIndex_ = 0;

        // Register last entry timestamp as stop time
        SetStopEntries();
    }
    // initiate the cache with first time frame
    InitiateCache();

    if (!create_datfile.empty())
    {
        CreateMergedDatfile(create_datfile);
    }
}

// Browse through replay-folder and appends strings of absolute path to matching scenario
void Replay::GetReplaysFromDirectory(const std::string dir, const std::string sce)
{
    DIR* directory = opendir(dir.c_str());

    // If no directory found, write error
    if (directory == nullptr)
    {
        LOG_ERROR_AND_QUIT("No valid directory given, couldn't open {}", dir);
    }

    // While directory is open, check the filename
    struct dirent* file;
    while ((file = readdir(directory)) != nullptr)
    {
        std::string filename = file->d_name;
        if (file->d_type == DT_DIR && filename.find(sce) != std::string::npos)
        {
            DIR* nested_dir = opendir((dir + filename).c_str());
            if (nested_dir == nullptr)
            {
                LOG_ERROR("Couldn't open nested directory {}{}", dir, filename);
            }

            struct dirent* nested_file;
            while ((nested_file = readdir(nested_dir)) != nullptr)
            {
                std::string nested_filename = nested_file->d_name;

                if (nested_filename != "." && nested_filename != ".." && nested_filename.find(sce) != std::string::npos &&
                    nested_filename.find(".dat") != std::string::npos)
                {
                    scenarios_.emplace_back(CombineDirectoryPathAndFilepath(dir + filename, nested_filename));
                }
            }
            closedir(nested_dir);
        }

        if (filename != "." && filename != ".." && filename.find(sce) != std::string::npos && filename.find(".dat") != std::string::npos)
        {
            scenarios_.emplace_back(CombineDirectoryPathAndFilepath(dir, filename));
        }
    }
    closedir(directory);

    // Sort list of filenames
    std::sort(scenarios_.begin(), scenarios_.end(), [](std::string const& a, std::string const& b) { return a < b; });

    if (scenarios_.empty())
    {
        LOG_ERROR_AND_QUIT("Couldn't read any scenarios named {} in path {}", sce, dir);
    }
}

size_t Replay::GetNumberOfScenarios()
{
    return scenarios_.size();
}

Replay::~Replay()
{
    delete datLogger_;
}

void Replay::GoToStart()
{
    InitiateCache();
}

void Replay::GoToEnd()
{
    GoToTime(stopTime_);
}

int Replay::FindIndexAtTimestamp(double timestamp, int startSearchIndex)
{
    int i = 0;

    if (timestamp > stopTime_)
    {
        GoToEnd();
        return static_cast<int>(index_);
    }
    else if (timestamp < GetStartTime())
    {
        return static_cast<int>(index_);
    }

    if (timestamp < time_)
    {
        // start search from beginning
        startSearchIndex = 0;
    }

    for (i = startSearchIndex; i < static_cast<int>(pkgs_.size()); i++)
    {
        if (static_cast<datLogger::PackageId>(pkgs_[static_cast<unsigned int>(i)].hdr.id) == datLogger::PackageId::TIME_SERIES)
        {
            double timeTemp = *reinterpret_cast<double*>(pkgs_[static_cast<unsigned int>(i)].content.data());
            if (IsEqualDouble(timeTemp, timestamp) || (timeTemp > timestamp))
            {
                break;
            }
        }
    }

    return MIN(i, static_cast<int>(pkgs_.size()) - 1);
}

bool scenarioengine::Replay::IsValidPocket(id_t id)
{
    for (id_t idPkg = static_cast<id_t>(datLogger::PackageId::HEADER); idPkg <= static_cast<id_t>(datLogger::PackageId::END_OF_SCENARIO); ++idPkg)
    {
        if (id == idPkg)
        {
            return true;
        }
    }
    return false;
}

bool IsEqualLightRgb(const datLogger::LightRGB& rgb1, const datLogger::LightRGB& rgb2)
{
    if (rgb1.red == rgb2.red && rgb1.green == rgb2.green && rgb1.blue == rgb2.blue && rgb1.intensity == rgb2.intensity)
    {
        return true;
    }

    return false;
}

int Replay::RecordPkgs(const std::string& fileName)
{
    std::ifstream file_Read_;
    file_Read_.open(fileName, std::ifstream::binary);
    if (file_Read_.fail())
    {
        LOG_ERROR("Cannot open file: {}", fileName);
    }

    if (file_Read_.is_open())
    {
        LOG_INFO("File Opened for read");
    }

    // Get the file size
    file_Read_.seekg(0, std::ios::end);
    std::streampos file_size = file_Read_.tellg();
    file_Read_.seekg(0, std::ios::beg);
    while (file_Read_.good())
    {
        if (file_Read_.tellg() == file_size)
        {
            file_Read_.close();
            break;
        }
        // read the header for every loop
        datLogger::CommonPkgHdr cmnHdrPkgRead;
        file_Read_.read(reinterpret_cast<char*>(&cmnHdrPkgRead), sizeof(datLogger::CommonPkgHdr));

        if (!IsValidPocket(cmnHdrPkgRead.id))
        {
            LOG_ERROR("Unknown package read->package id : {}", cmnHdrPkgRead.id);
        }
        if ((cmnHdrPkgRead.id == static_cast<id_t>(datLogger::PackageId::HEADER)))
        {
            file_Read_.read(reinterpret_cast<char*>(&header_.version), sizeof(datLogger::DatHdr::version));
            file_Read_.read(reinterpret_cast<char*>(&header_.odrFilename.size), sizeof(datLogger::DatHdr::odrFilename.size));
            header_.odrFilename.string.resize(header_.odrFilename.size);
            file_Read_.read(header_.odrFilename.string.data(), header_.odrFilename.size);
            file_Read_.read(reinterpret_cast<char*>(&header_.modelFilename.size), sizeof(datLogger::DatHdr::modelFilename.size));
            header_.modelFilename.string.resize(header_.modelFilename.size);
            file_Read_.read(header_.modelFilename.string.data(), header_.modelFilename.size);
        }
        else
        {
            datLogger::CommonPkg cmnPkg;
            cmnPkg.hdr = cmnHdrPkgRead;
            cmnPkg.content.resize(cmnPkg.hdr.content_size);                   // Allocate memory for the *copy*
            file_Read_.read(cmnPkg.content.data(), cmnPkg.hdr.content_size);  // Read into the vector
            pkgs_.push_back(cmnPkg);

            if ((cmnPkg.hdr.id == static_cast<id_t>(datLogger::PackageId::TIME_SERIES)))
            {
                double t = *reinterpret_cast<double*>(pkgs_[pkgs_.size() - 1].content.data());

                if (t > 0)  // dont include till ghost reaches 0.0 time
                {
                    if (std::isnan(previousTime_))
                    {
                        previousTime_ = t;
                    }
                    else if (t > previousTime_)
                    {
                        if (fabs(t - previousTime_) < deltaTime_ && fabs(t - previousTime_) > SMALL_NUMBER)
                        {
                            deltaTime_ = fabs(t - previousTime_);
                        }
                    }
                    previousTime_ = t;
                }
            }
            else if (cmnPkg.hdr.id == static_cast<id_t>(datLogger::PackageId::LIGHT_STATES))
            {
                datLogger::LightState lightState = *reinterpret_cast<datLogger::LightState*>(pkgs_[pkgs_.size() - 1].content.data());

                if (IsLightPkgFound && !show_lights)
                {
                    const size_t numLights = sizeof(lightState) / sizeof(datLogger::LightRGB);
                    for (size_t i = 0; i < numLights; ++i)
                    {
                        datLogger::LightRGB* lightNew = reinterpret_cast<datLogger::LightRGB*>(&lightState) + i;
                        datLogger::LightRGB* lightOld = reinterpret_cast<datLogger::LightRGB*>(&perviousLightState) + i;
                        if (lightNew->red != lightOld->red || lightNew->green != lightOld->green || lightNew->blue != lightOld->blue ||
                            lightNew->intensity != lightOld->intensity)
                        {
                            show_lights = true;
                            break;
                        }
                    }
                }
                perviousLightState = lightState;
                if (!IsLightPkgFound)
                {
                    defaultLightState = lightState;
                }
                IsLightPkgFound = true;
            }
        }
    }
    return 0;
}

datLogger::PackageId Replay::ReadPkgHdr(char* package)
{
    datLogger::CommonPkg pkg;
    pkg = *reinterpret_cast<datLogger::CommonPkg*>(package);
    return static_cast<datLogger::PackageId>(pkg.hdr.id);
}

size_t Replay::GetPkgCntBtwObj(size_t idx)
{
    for (size_t i = idx + 1; i < pkgs_.size(); i++)  // start looking from next package
    {
        if (IsTimePkg(i) || IsObjIdPkg(i) || IsEndOfScenarioPkg(i))  // stop looking if time or obj id package found
        {
            return i - idx - 1;
        }
    }
    return 0;
}

std::vector<size_t> Replay::GetNumberOfObjectsAtTime()
{
    std::vector<size_t> Indices;
    bool                timeFound = false;
    for (size_t i = index_; i < pkgs_.size(); i++)
    {
        if (IsTimePkg(i) && !timeFound)
        {
            double timeTemp = GetDoubleContent(i);
            if (IsEqualDouble(timeTemp, time_))
            {
                timeFound = true;
            }
            continue;  // continue till time match found. if time matched then
        }
        if (timeFound && IsObjIdPkg(i))
        {
            Indices.push_back(i);  // time matches
        }
        if (IsTimePkg(i))
        {
            return Indices;  // second time instances
        }
    }
    return Indices;
}

bool Replay::IsObjAvailableInCache(int id)  // check in current state
{
    for (const auto& obj : scenarioState_.obj_states)  // loop current state object id to find the object id
    {
        if (obj.id == id)
        {
            return true;  // obj id present
        }
    }
    return false;
}

bool Replay::IsObjAvailableActive(int id) const  // check in current state
{
    for (const auto& obj : scenarioState_.obj_states)  // loop current state object id to find the object id
    {
        if (obj.id == id)
        {
            return obj.active;  // obj id present
        }
    }
    return false;
}

void Replay::GoToDeltaTime(double dt, bool stopAtEachFrame)
{
    GoToTime(scenarioState_.sim_time + dt, stopAtEachFrame);
}

void Replay::ExtractRestartTimes()
{
    double       perviousTime  = -LARGE_NUMBER;
    unsigned int perviousIndex = 0;
    if (!(restartTimes_.size() > 0))
    {
        for (size_t i = 0; i < pkgs_.size(); i++)
        {
            if (IsTimePkg(i))
            {
                double timeTemp = GetDoubleContent(i);
                if (perviousTime > timeTemp)
                {
                    RestartTimes restartTime;
                    restartTime.restart_time_  = perviousTime;
                    restartTime.restart_index_ = perviousIndex;
                    restartTimes_.push_back(restartTime);
                }
                perviousTime  = timeTemp;
                perviousIndex = static_cast<unsigned int>(i);
                if (restartTimes_.size() > 0)
                {
                    for (size_t j = 0; j < restartTimes_.size(); j++)
                    {
                        if (IsEqualDouble(restartTimes_[j].next_time_, LARGE_NUMBER))  // only once
                        {
                            if (restartTimes_[j].restart_time_ < timeTemp)
                            {
                                restartTimes_[j].next_time_  = timeTemp;
                                restartTimes_[j].next_index_ = static_cast<unsigned int>(i);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}

void Replay::GoToNextFrame()
{
    for (size_t i = static_cast<size_t>(index_) + 1; i < pkgs_.size(); i++)
    {
        if (IsTimePkg(i))
        {
            time_  = GetDoubleContent(i);
            index_ = static_cast<unsigned int>(i);
            break;
        }
    }
}

void Replay::GoToPreviousFrame()
{
    for (size_t i = static_cast<size_t>(index_) - 1; static_cast<int>(i) >= 0; i--)
    {
        if (IsTimePkg(i))
        {
            time_  = GetDoubleContent(i);
            index_ = static_cast<unsigned int>(i);
            break;
        }
    }
}
void Replay::UpdateCache()
{
    scenarioState_.sim_time          = time_;
    std::vector<size_t> objIdIndices = GetNumberOfObjectsAtTime();

    for (const auto objIdIndex : objIdIndices)
    {
        int    obj_id = GetIntContent(objIdIndex);
        size_t pkgCnt = GetPkgCntBtwObj(objIdIndex);
        for (auto& obj : scenarioState_.obj_states)  // loop current state object id to find the object id
        {
            if (obj.id == obj_id)  // found object id
            {
                for (size_t j = 0; j < obj.pkgs.size(); j++)
                {
                    for (size_t k = objIdIndex + 1; k < objIdIndex + pkgCnt + 1;
                         k++)  // start with Index + 1, Index will have object id package index. looking from next package
                    {
                        datLogger::PackageId id_ = ReadPkgHdr(obj.pkgs[j].pkg);
                        if (id_ == static_cast<datLogger::PackageId>(pkgs_[k].hdr.id))
                        {
                            obj.pkgs[j].pkg = reinterpret_cast<char*>(&pkgs_[k]);
                            break;
                        }
                    }
                }
            }
        }
    }
}

void Replay::UpdateObjStatusInCache(int id, bool status)
{
    for (auto& obj : scenarioState_.obj_states)  // Iterate by reference
    {                                            // loop current state object id to find the object id
        if (obj.id == id)
        {
            obj.active = status;
        }
    }
}

void Replay::CheckObjAvailabilityForward()
{
    // check all obj status in current time frame
    for (const auto index : GetNumberOfObjectsAtTime())
    {
        // Add the object's state to the cache. This also initializes the object's state if it's new.
        AddObjStateInCache(index);
        if (IsObjectDeletePkg(index + 1))
        {
            // If the object is marked for deletion, update its status in the cache to inactive.
            // We do NOT delete the object from the cache to preserve its state for reverse playback.
            UpdateObjStatusInCache(GetIntContent(index), false);
        }
    }
}

void Replay::CheckObjAvailabilityBackward()
{
    // check all obj status in current time frame
    for (const auto index : GetNumberOfObjectsAtTime())
    {
        int obj_id = GetIntContent(index);
        if (IsObjectDeletePkg(index + 1))
        {
            // obj added in cache with same state as when its deleted
            UpdateObjStatusInCache(obj_id, true);
        }
        else if (IsObjIdAddPkg(index + 1) && !IsEqualDouble(time_, startTime_))  // ignore first time frame
        {
            UpdateObjStatusInCache(obj_id, false);
        }
    }
}

bool Replay::HandleRestartTimes()
{
    if (!restartTimes_.empty())
    {
        for (const auto& restartTime : restartTimes_)
        {
            if ((restartTime.next_index_ == index_) && (!show_restart_))  // go to restarted time from restart finished next time
            {
                index_ = restartTime.restart_index_;  // jump, skip all time frames belong during restart
                time_  = restartTime.restart_time_;
                return false;  // Time lapsed is false
            }
            else if (restartTime.restart_index_ == index_ && show_restart_)  // go to restarted time from restart first time
            {
                return true;  // Time lapsed is true
            }
        }
    }
    return false;  // Time lapsed is false if no restart time is handled.
}

int Replay::GoToTime(double t, bool stopAtEachFrame)
{
    if (t > stopTime_)  // go to stop time
    {
        t = stopTime_;
    }
    else if (t < startTime_)  // go to start time
    {
        t = startTime_;
    }

    if (!IsEqualDouble(t, scenarioState_.sim_time))
    {
        if (scenarioState_.sim_time < t)
        {
            GoForwardTime(t, stopAtEachFrame);
        }
        else
        {
            GoBackwardTime(t, stopAtEachFrame);
        }
    }
    return 0;
}

int scenarioengine::Replay::GoForwardTime(double t, bool stopAtEachFrame)
{
    while (true)
    {
        double       previousTime  = time_;
        unsigned int previousIndex = index_;

        GoToNextFrame();
        double time_diff = time_ - scenarioState_.sim_time;
        if ((time_ > t + SMALL_NUMBER) || (stopAtEachFrame && time_diff > deltaTime_ && !IsEqualDouble(time_diff, deltaTime_)))
        {  // Gone past requested time or stop at each frame AND the time difference exceeds deltaTime_ and also make sure time diff is not 6 decimal
           // greater.
            time_  = previousTime;
            index_ = previousIndex;
            if (stopAtEachFrame && time_ - scenarioState_.sim_time > deltaTime_)
            {
                scenarioState_.sim_time += deltaTime_;
            }
            else
            {
                scenarioState_.sim_time = t;
            }
            return 0;
        }

        CheckObjAvailabilityForward();
        UpdateCache();

        if (stopAtEachFrame || (time_ + SMALL_NUMBER < previousTime && show_restart_) || IsEqualDouble(t, time_))
        {  // stop at each frame or requested time reached or restart
            return 0;
        }
    }
}

int scenarioengine::Replay::GoBackwardTime(double t, bool stopAtEachFrame)
{
    bool timeLapsed = false;
    while (!timeLapsed)
    {
        if (scenarioState_.sim_time < time_)
        {
            CheckObjAvailabilityBackward();
        }

        double       previousTime  = time_;
        unsigned int previousIndex = index_;
        GoToPreviousFrame();

        timeLapsed = HandleRestartTimes();

        if (stopAtEachFrame && scenarioState_.sim_time - time_ > deltaTime_)
        {  // stop at each frame AND the time difference exceeds deltaTime_.
            time_  = previousTime;
            index_ = previousIndex;
            scenarioState_.sim_time += deltaTime_;
            break;
        }

        UpdateCache();
        CheckObjAvailabilityBackward();

        if (time_ < t + SMALL_NUMBER)  // gone past requested time, sim time should be given time
        {
            time_                   = previousTime;
            index_                  = previousIndex;
            scenarioState_.sim_time = t;
            break;
        }

        if (stopAtEachFrame || IsEqualDouble(t, time_))  // ran till requested time
        {
            timeLapsed = true;
        }
    }
    return 0;
}

void Replay::SetStopEntries()
{
    for (size_t i = pkgs_.size() - 1; static_cast<int>(i) >= 0; i--)
    {
        if (IsTimePkg(i))
        {
            stopTime_  = GetDoubleContent(i);
            stopIndex_ = static_cast<unsigned int>(i);
            break;
        }
    }
}

double Replay::GetTimeFromCnt(int count)
{
    double timeTemp = -1.0;
    int    count_   = 0;
    for (size_t i = 0; i < pkgs_.size(); i++)  // return time if idx is already time pkg or find the respective time pkg for given pkg idx
    {
        if (IsTimePkg(i))  // find time pkg for given pkg idx
        {
            count_ += 1;
            if (count == count_)
            {
                return GetDoubleContent(i);
            }
        }
    }
    return timeTemp;
}

void Replay::AddObjStateInCache(size_t idx)
{
    ObjectStateWithObjId stateObjId;
    if (!IsObjIdPkg(idx))
    {
        LOG_ERROR_AND_QUIT(" Initialization error->Stop replay ");
    }

    stateObjId.id = GetIntContent(idx);
    if (IsObjAvailableActive(stateObjId.id))
    {
        return;  // object already active in cache
    }
    if (IsObjAvailableInCache(stateObjId.id))
    {
        // make sure same obj deleted before adding the same object(object re-added)
        DeleteObjStateInCache(stateObjId.id);
    }

    stateObjId.active = true;
    size_t pkgCount   = GetPkgCntBtwObj(idx);
    for (size_t i = idx + 1; i < pkgCount + idx + 1; i++)
    {  // GetPkgCntBtwObj will return count of package

        if (IsObjIdAddPkg(i))
        {
            continue;  // skip packages.
        }
        ObjectStateWithPkg statePkg;
        statePkg.pkg = reinterpret_cast<char*>(&pkgs_[i]);
        stateObjId.pkgs.push_back(statePkg);
    }
    scenarioState_.obj_states.push_back(stateObjId);
}

void Replay::DeleteObjStateInCache(int objId)
{
    for (size_t i = 0; i < scenarioState_.obj_states.size(); i++)  // loop current state object id to find the object id
    {
        if (scenarioState_.obj_states[i].id == objId)  // found object id
        {
            scenarioState_.obj_states.erase(scenarioState_.obj_states.begin() + static_cast<unsigned int>(i));
            break;
        }
    }
}

void Replay::InitiateCache()
{
    // reset the timings
    scenarioState_.obj_states.clear();
    scenarioState_.sim_time = startTime_;
    time_                   = startTime_;
    index_                  = startIndex_;

    std::vector<size_t> objIdIndices = GetNumberOfObjectsAtTime();

    for (const auto index : objIdIndices)
    {
        AddObjStateInCache(index);
    }
}

bool scenarioengine::Replay::IsObjectDeletePkg(size_t index) const
{
    return pkgs_[index].hdr.id == static_cast<int>(datLogger::PackageId::OBJ_DELETED);
}

bool scenarioengine::Replay::IsObjIdAddPkg(size_t index) const
{
    return pkgs_[index].hdr.id == static_cast<int>(datLogger::PackageId::OBJ_ADDED);
}

bool scenarioengine::Replay::IsObjIdPkg(size_t index) const
{
    return static_cast<datLogger::PackageId>(pkgs_[index].hdr.id) == datLogger::PackageId::OBJ_ID;
}

bool scenarioengine::Replay::IsEndOfScenarioPkg(size_t index) const
{
    return static_cast<datLogger::PackageId>(pkgs_[index].hdr.id) == datLogger::PackageId::END_OF_SCENARIO;
}

bool scenarioengine::Replay::IsTimePkg(size_t index) const
{
    return static_cast<datLogger::PackageId>(pkgs_[index].hdr.id) == datLogger::PackageId::TIME_SERIES;
}

double scenarioengine::Replay::GetDoubleContent(size_t index)
{
    return *reinterpret_cast<double*>(pkgs_[index].content.data());
}

int scenarioengine::Replay::GetIntContent(size_t index)
{
    return *reinterpret_cast<int*>(pkgs_[index].content.data());
}

int scenarioengine::Replay::GetIntFromPkg(datLogger::CommonPkg* pkg)
{
    return *reinterpret_cast<int*>(pkg->content.data());
    ;
}

double scenarioengine::Replay::GetDoubleFromPkg(datLogger::CommonPkg* pkg)
{
    return *reinterpret_cast<double*>(pkg->content.data());
    ;
}

int scenarioengine::Replay::GetIntFromScenarioState(int obj_id, datLogger::PackageId id)
{
    for (const auto& obj : scenarioState_.obj_states)
    {
        if (obj.id == obj_id)
        {
            for (const auto& objPkg : obj.pkgs)
            {
                datLogger::CommonPkg* cmnPkg = reinterpret_cast<datLogger::CommonPkg*>(objPkg.pkg);
                if (static_cast<datLogger::PackageId>(cmnPkg->hdr.id) == id)
                {
                    return GetIntFromPkg(cmnPkg);
                }
            }
        }
    }
    return -1;
}

int Replay::GetModelID(int obj_id)
{
    return GetIntFromScenarioState(obj_id, datLogger::PackageId::MODEL_ID);
}

int Replay::GetCtrlType(int obj_id)
{
    return GetIntFromScenarioState(obj_id, datLogger::PackageId::CTRL_TYPE);
}

int Replay::GetObjCategory(int obj_id)
{
    return GetIntFromScenarioState(obj_id, datLogger::PackageId::OBJ_CATEGORY);
}

int Replay::GetScaleMode(int obj_id)
{
    return GetIntFromScenarioState(obj_id, datLogger::PackageId::SCALE_MODE);
}

int Replay::GetVisibility(int obj_id)
{
    return GetIntFromScenarioState(obj_id, datLogger::PackageId::VISIBILITY_MASK);
}

int Replay::GetLaneId(int obj_id)
{
    return GetIntFromScenarioState(obj_id, datLogger::PackageId::LANE_ID);
}

double scenarioengine::Replay::GetDoubleFromScenarioState(int obj_id, datLogger::PackageId id)
{
    for (const auto& obj : scenarioState_.obj_states)
    {
        if (obj.id == obj_id)
        {
            for (const auto& objPkg : obj.pkgs)
            {
                datLogger::CommonPkg* cmnPkg = reinterpret_cast<datLogger::CommonPkg*>(objPkg.pkg);
                if (static_cast<datLogger::PackageId>(cmnPkg->hdr.id) == id)
                {
                    return GetDoubleFromPkg(cmnPkg);
                }
            }
        }
    }
    return 0.0;
}

double Replay::GetPosOffset(int obj_id)
{
    return GetDoubleFromScenarioState(obj_id, datLogger::PackageId::POS_OFFSET);
}

double Replay::GetWheelAngle(int obj_id)
{
    return GetDoubleFromScenarioState(obj_id, datLogger::PackageId::WHEEL_ANGLE);
}

double Replay::GetWheelRot(int obj_id)
{
    return GetDoubleFromScenarioState(obj_id, datLogger::PackageId::WHEEL_ROT);
}

double Replay::GetSpeed(int obj_id)
{
    return GetDoubleFromScenarioState(obj_id, datLogger::PackageId::SPEED);
}

double Replay::GetPosT(int obj_id)
{
    return GetDoubleFromScenarioState(obj_id, datLogger::PackageId::POS_T);
}

double Replay::GetPosS(int obj_id)
{
    return GetDoubleFromScenarioState(obj_id, datLogger::PackageId::POS_S);
}

int Replay::GetBB(int obj_id, OSCBoundingBox& bb)
{
    datLogger::BoundingBox bb_;
    for (const auto& obj : scenarioState_.obj_states)  // loop current state object id to find the object id
    {
        if (obj.id == obj_id)
        {
            for (const auto& objPkg : obj.pkgs)
            {
                datLogger::CommonPkg* cmnPkg = reinterpret_cast<datLogger::CommonPkg*>(objPkg.pkg);
                if (static_cast<datLogger::PackageId>(cmnPkg->hdr.id) == datLogger::PackageId::BOUNDING_BOX)
                {
                    bb_ = *reinterpret_cast<datLogger::BoundingBox*>(cmnPkg->content.data());
                    break;
                }
            }
        }
    }
    bb.center_.x_          = bb_.x;
    bb.center_.y_          = bb_.y;
    bb.center_.z_          = bb_.z;
    bb.dimensions_.height_ = bb_.height;
    bb.dimensions_.length_ = bb_.length;
    bb.dimensions_.width_  = bb_.width;
    return 0;
}

datLogger::Pos Replay::GetPos(int obj_id)
{
    datLogger::Pos pos;
    for (const auto& obj : scenarioState_.obj_states)  // loop current state object id to find the object id
    {
        if (obj.id == obj_id)
        {
            for (const auto& objPkg : obj.pkgs)
            {
                datLogger::CommonPkg* cmnPkg = reinterpret_cast<datLogger::CommonPkg*>(objPkg.pkg);
                if (static_cast<datLogger::PackageId>(cmnPkg->hdr.id) == datLogger::PackageId::POSITIONS)
                {
                    pos = *reinterpret_cast<datLogger::Pos*>(cmnPkg->content.data());
                    break;
                }
            }
        }
    }
    return pos;
}

double Replay::GetX(int obj_id)
{
    datLogger::Pos pos = GetPos(obj_id);
    return pos.x;
}

double Replay::GetY(int obj_id)
{
    datLogger::Pos pos = GetPos(obj_id);
    return pos.y;
}

double Replay::GetZ(int obj_id)
{
    datLogger::Pos pos = GetPos(obj_id);
    return pos.z;
}

double Replay::GetH(int obj_id)
{
    datLogger::Pos pos = GetPos(obj_id);
    return pos.h;
}

double Replay::GetR(int obj_id)
{
    datLogger::Pos pos = GetPos(obj_id);
    return pos.r;
}

double Replay::GetP(int obj_id)
{
    datLogger::Pos pos = GetPos(obj_id);
    return pos.p;
}

id_t Replay::GetRoadId(int obj_id)
{
    id_t road_id = ID_UNDEFINED;
    for (const auto& obj : scenarioState_.obj_states)  // loop current state object id to find the object id
    {
        if (obj.id == obj_id)
        {
            for (const auto& objPkg : obj.pkgs)
            {
                datLogger::CommonPkg* cmnPkg = reinterpret_cast<datLogger::CommonPkg*>(objPkg.pkg);
                if (static_cast<datLogger::PackageId>(cmnPkg->hdr.id) == datLogger::PackageId::ROAD_ID)
                {
                    road_id = *reinterpret_cast<id_t*>(cmnPkg->content.data());
                    break;
                }
            }
        }
    }
    return road_id;
}

ObjectPositionStructDat Replay::GetComPletePos(int obj_id)
{
    ObjectPositionStructDat complete_pos;
    datLogger::Pos          pos;
    pos                 = GetPos(obj_id);
    complete_pos.h      = pos.h;
    complete_pos.x      = pos.x;
    complete_pos.y      = pos.y;
    complete_pos.z      = pos.z;
    complete_pos.p      = pos.p;
    complete_pos.r      = pos.r;
    complete_pos.laneId = GetLaneId(obj_id);
    complete_pos.roadId = GetRoadId(obj_id);
    complete_pos.offset = GetPosOffset(obj_id);
    complete_pos.t      = GetPosT(obj_id);
    complete_pos.s      = GetPosS(obj_id);
    return complete_pos;
}

int Replay::GetName(int obj_id, std::string& name)
{
    for (const auto obj : scenarioState_.obj_states)  // loop current state object id to find the object id
    {
        if (obj.id == obj_id)
        {
            for (const auto objPkg : obj.pkgs)
            {
                datLogger::CommonPkg* cmnPkg = reinterpret_cast<datLogger::CommonPkg*>(objPkg.pkg);
                if (static_cast<datLogger::PackageId>(cmnPkg->hdr.id) == datLogger::PackageId::NAME)
                {
                    name = cmnPkg->content.data();
                    return 0;
                }
            }
        }
    }
    return -1;  // maybe check success or not
}

void Replay::GetRgbValues(int obj_id, Object::VehicleLightActionStatus* vehicleLightActionStatusList)
{
    datLogger::LightState lightState;
    GetLightStates(obj_id, lightState);

    const size_t numLights = sizeof(lightState) / sizeof(datLogger::LightRGB);
    for (size_t i = 0; i < numLights; ++i)
    {
        datLogger::LightRGB* light         = reinterpret_cast<datLogger::LightRGB*>(&lightState) + i;
        datLogger::LightRGB* default_light = reinterpret_cast<datLogger::LightRGB*>(&defaultLightState) + i;
        if (light->red != default_light->red || light->green != default_light->green || light->blue != default_light->blue ||
            light->intensity != default_light->intensity)
        {
            vehicleLightActionStatusList[i].type          = static_cast<Object::VehicleLightType>(i);
            vehicleLightActionStatusList[i].diffuseRgb[0] = light->red / 255.0;
            vehicleLightActionStatusList[i].diffuseRgb[1] = light->green / 255.0;
            vehicleLightActionStatusList[i].diffuseRgb[2] = light->blue / 255.0;

            vehicleLightActionStatusList[i].emissionRgb[0] = vehicleLightActionStatusList[i].diffuseRgb[0] * (light->intensity / 255.0);
            vehicleLightActionStatusList[i].emissionRgb[1] = vehicleLightActionStatusList[i].diffuseRgb[1] * (light->intensity / 255.0);
            vehicleLightActionStatusList[i].emissionRgb[2] = vehicleLightActionStatusList[i].diffuseRgb[2] * (light->intensity / 255.0);
        }
    }
}

void Replay::GetLightStates(int obj_id, datLogger::LightState& light_states_)
{
    for (const auto obj : scenarioState_.obj_states)  // loop current state object id to find the object id
    {
        if (obj.id == obj_id)
        {
            for (const auto objPkg : obj.pkgs)
            {
                datLogger::CommonPkg* cmnPkg = reinterpret_cast<datLogger::CommonPkg*>(objPkg.pkg);
                if (static_cast<datLogger::PackageId>(cmnPkg->hdr.id) == datLogger::PackageId::LIGHT_STATES)
                {
                    light_states_ = *reinterpret_cast<datLogger::LightState*>(cmnPkg->content.data());
                    return;
                }
            }
        }
    }
}

const datLogger::DatHdr scenarioengine::Replay::GetHeader() const
{
    return header_;
}

std::vector<datLogger::CommonPkg> scenarioengine::Replay::GetPkgs()
{
    return pkgs_;
}

void Replay::SetStartTime(double time)
{
    startTime_  = time;
    startIndex_ = static_cast<unsigned int>(FindIndexAtTimestamp(startTime_));
    if (time_ < startTime_)
    {
        time_  = startTime_;
        index_ = startIndex_;
    }
}

void Replay::SetStopTime(double time)
{
    stopTime_  = time;
    stopIndex_ = static_cast<unsigned int>(FindIndexAtTimestamp(stopTime_));
    if (time_ > stopTime_)
    {
        time_  = stopTime_;
        index_ = stopIndex_;
    }
}

double scenarioengine::Replay::GetStartTime()
{
    return startTime_;
}

double scenarioengine::Replay::GetStopTime()
{
    return stopTime_;
}

double scenarioengine::Replay::GetTime()
{
    return scenarioState_.sim_time;
}

int scenarioengine::Replay::GetIndex()
{
    return static_cast<int>(index_);
}

void scenarioengine::Replay::SetTime(double t)
{
    time_ = t;
}

void scenarioengine::Replay::SetIndex(unsigned int index)
{
    index_ = index;
}

void scenarioengine::Replay::SetRepeat(bool repeat)
{
    repeat_ = repeat;
}

void Replay::AdjustObjectId(std::vector<std::vector<int>>& objIds)
{
    int max = 0;
    for (auto ids : objIds)
    {
        for (auto id : ids)
        {
            if (id > max)
            {
                max = id;
            }
        }
    }

    int    multiplier = 1;
    double result     = LARGE_NUMBER;
    for (int i = 10; result > 1; i *= 10)
    {
        result     = max / i;
        multiplier = i;
    }

    // Log which scenario belongs to what ID-group (0, 100, 200 etc.)
    for (size_t i = 0; i < scenarioData.size(); i++)
    {
        // std::string scenario_tmp = scenarioData[i].first;
        std::string scenario_tmp = scenarioData[i].first.first;
        LOG_INFO("Scenarios corresponding to IDs ({}:{}): {}",
                 static_cast<int>(i) * multiplier,
                 ((static_cast<int>(i) + 1) * multiplier) - 1,
                 FileNameOf(scenario_tmp.c_str()));
    }

    for (size_t i = 0; i < scenarioData.size(); i++)
    {
        for (size_t j = 0; j < scenarioData[i].second.size(); j++)
        {
            // Set scenario ID-group (0, 100, 200 etc.)
            if (scenarioData[i].second[j].hdr.id == static_cast<int>(datLogger::PackageId::OBJ_ID))
            {
                int value = *reinterpret_cast<int*>(scenarioData[i].second[j].content.data());
                // value += static_cast<int>(j) * 100;
                value += static_cast<int>(i) * multiplier;
                *reinterpret_cast<int*>(scenarioData[i].second[j].content.data()) = value;  // store it in the same address
            }
        }
    }
}

void Replay::BuildData()
{
    // Scenario with smallest start time first
    std::sort(scenarioData.begin(),
              scenarioData.end(),
              [](const auto& sce1, const auto& sce2) {
                  return *reinterpret_cast<const double*>(sce1.second[0].content.data()) <
                         *reinterpret_cast<const double*>(sce2.second[0].content.data());
              });

    // Keep track of current index of each scenario
    std::vector<size_t> cur_idx;

    for (size_t j = 0; j < scenarioData.size(); j++)
    {
        cur_idx.push_back(0);
    }

    // Populate data based on first (with lowest timestamp) scenario
    double cur_timestamp      = *reinterpret_cast<double*>(scenarioData[0].second[0].content.data());
    double timeTemp           = SMALL_NUMBER;
    bool   timeFound          = false;
    double min_time_stamp     = LARGE_NUMBER;
    bool   timePkgWritten     = false;
    int    endOfScenarioCount = 0;
    bool   firstIteration     = true;

    do
    {
        for (size_t j = 0; j < scenarioData.size(); j++)
        {
            if (j == 0)
            {
                firstIteration = true;  // reset the iteration
            }
            if (scenarioData[j].first.second == false)  // file merged, skip looking
            {
                continue;
            }

            for (size_t k = cur_idx[j]; k < scenarioData[j].second.size(); k++)
            {
                if (scenarioData[j].second[k].hdr.id == static_cast<int>(datLogger::PackageId::TIME_SERIES))
                {
                    timeTemp = *reinterpret_cast<double*>(scenarioData[j].second[k].content.data());
                    if (IsEqualDouble(timeTemp, cur_timestamp))
                    {
                        timeFound = true;
                    }
                    else if (timeTemp > cur_timestamp)
                    {
                        timeFound = false;
                        // find the smallest time in the all scenarios for next iteration
                        if (firstIteration)  // first iteration base time
                        {
                            min_time_stamp = timeTemp;
                            firstIteration = false;  // j wont give iteration number if specific scenario merged.
                        }
                        else if (min_time_stamp > timeTemp)
                        {
                            min_time_stamp = timeTemp;
                        }
                        cur_idx[j] = k;
                        break;
                    }
                }

                datLogger::PackageId pkgId = static_cast<datLogger::PackageId>(scenarioData[j].second[k].hdr.id);

                if ((timeFound && pkgId != datLogger::PackageId::TIME_SERIES) ||
                    (timeFound && pkgId == datLogger::PackageId::TIME_SERIES && !timePkgWritten))
                {
                    if (pkgId == datLogger::PackageId::END_OF_SCENARIO)
                    {
                        scenarioData[j].first.second = false;  // indicate already file merged
                        endOfScenarioCount += 1;
                    }
                    if (pkgId == datLogger::PackageId::TIME_SERIES)  // store time pkg only once
                    {
                        timePkgWritten = true;
                    }

                    if (pkgId != datLogger::PackageId::END_OF_SCENARIO ||
                        (pkgId == datLogger::PackageId::END_OF_SCENARIO &&
                         endOfScenarioCount == static_cast<int>(cur_idx.size())))  // write end of scenario only once
                    {
                        pkgs_.push_back(scenarioData[j].second[k]);
                    }
                }
            }
        }
        cur_timestamp  = min_time_stamp;
        timePkgWritten = false;

    } while (endOfScenarioCount < static_cast<int>(cur_idx.size()));
}

int Replay::CreateMergedDatfile(const std::string filename)
{
    if (!filename.empty())
    {
        if (datLogger_ == nullptr)
        {
            if ((datLogger_ = new datLogger::DatLogger()) == nullptr)
            {
                return -1;
            }

            if (datLogger_->Init(filename, header_.version, header_.odrFilename.string.data(), header_.modelFilename.string.data()) != 0)
            {
                delete datLogger_;
                datLogger_ = nullptr;
                return -1;
            }
        }
    }

    if (datLogger_->IsFileOpen())
    {
        for (size_t i = 0; i < pkgs_.size(); i++)
        {
            if (pkgs_[i].hdr.id == static_cast<int>(datLogger::PackageId::NAME))
            {
                std::string name = pkgs_[i].content.data();
                datLogger_->WriteStringPkg(name, static_cast<datLogger::PackageId>(pkgs_[i].hdr.id));
            }
            else
            {
                datLogger_->writePackage(pkgs_[i]);
            }
        }
    }
    return 0;
}

void Replay::UpdateOdaMeter(int obj_id, double value)
{
    for (auto& obj : scenarioState_.obj_states)
    {
        if (obj.id == obj_id)
        {
            obj.odometer = value;
        }
    }
}

void scenarioengine::Replay::SetShowRestart(bool showRestart)
{
    show_restart_ = showRestart;
}

double Replay::GetOdaMeter(int obj_id)
{
    for (const auto& obj : scenarioState_.obj_states)
    {
        if (obj.id == obj_id)
        {
            return obj.odometer;
        }
    }
    return 0.0;
}
