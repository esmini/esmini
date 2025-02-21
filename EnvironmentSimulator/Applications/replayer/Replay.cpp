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

// new replayer constructor
Replay::Replay(std::string filename) : time_(0.0), index_(0), repeat_(false), show_restart_(false)
{
    RecordPkgs(filename);

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
    InitiateStates();
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
        // pair <scenario name, scenario data>
        // scenarioData.push_back(std::make_pair(scenarios_[i], pkgs_));
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
    InitiateStates();

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
    InitiateStates();
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
            if (isEqualDouble(timeTemp, timestamp) || (timeTemp > timestamp))
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

                if (!(t < 0) || !isEqualDouble(deltaTime_, LARGE_NUMBER))  // dont include till ghost reaches 0.0 time
                {
                    if (!std::isnan(previousTime_))
                    {
                        if (fabs(t - previousTime_) < deltaTime_)
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
    // std::cout << "Found package ID from current state: " << pkgIdTostring( static_cast<PackageId>(pkg.hdr.id)) << std::endl;
    return static_cast<datLogger::PackageId>(pkg.hdr.id);
}

int Replay::GetPkgCntBtwObj(size_t idx)
{
    int count = 0;
    for (size_t i = idx + 1; i < pkgs_.size(); i++)  // start looking from next package
    {
        if (static_cast<datLogger::PackageId>(pkgs_[i].hdr.id) == datLogger::PackageId::TIME_SERIES ||
            static_cast<datLogger::PackageId>(pkgs_[i].hdr.id) == datLogger::PackageId::OBJ_ID)
        {
            break;  // stop looking if time or obj id package found
        }
        count += 1;  // count package
    }
    return count;
}

std::vector<int> Replay::GetNumberOfObjectsAtTime()
{
    std::vector<int> Indices;
    bool             timeFound = false;

    for (size_t i = index_; i < pkgs_.size(); i++)
    {
        if (pkgs_[i].hdr.id == static_cast<int>(datLogger::PackageId::TIME_SERIES) && !timeFound)
        {
            double timeTemp = *reinterpret_cast<double*>(pkgs_[i].content.data());
            if (isEqualDouble(timeTemp, time_))
            {
                timeFound = true;
            }
            continue;  // continue till time match found. if time matched then
        }
        if (timeFound && static_cast<datLogger::PackageId>(pkgs_[i].hdr.id) == datLogger::PackageId::OBJ_ID)
        {
            Indices.push_back(static_cast<int>(i));  // time matches
        }
        if (pkgs_[i].hdr.id == static_cast<int>(datLogger::PackageId::TIME_SERIES))
        {
            return Indices;  // second time instances
        }
    }
    return Indices;
}

bool Replay::IsObjAvailableInCache(int id)  // check in current state
{
    bool status = false;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)  // loop current state object id to find the object id
    {
        if (scenarioState.obj_states[i].id == id)
        {
            status = true;  // obj id present
            break;
        }
    }
    return status;
}

bool Replay::IsObjAvailableActive(int id)  // check in current state
{
    bool status = false;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)  // loop current state object id to find the object id
    {
        if (scenarioState.obj_states[i].id == id)
        {
            status = scenarioState.obj_states[i].active;  // obj id present
            break;
        }
    }
    return status;
}

void Replay::GoToDeltaTime(double dt, bool stopAtEachFrame)
{
    GoToTime(scenarioState.sim_time + dt, stopAtEachFrame);
}

void Replay::GetRestartTimes()
{
    double       perviousTime  = -LARGE_NUMBER;
    unsigned int perviousIndex = 0;
    if (!(restartTimes.size() > 0))
    {
        for (size_t i = 0; i < pkgs_.size(); i++)
        {
            if (pkgs_[i].hdr.id == static_cast<int>(datLogger::PackageId::TIME_SERIES))
            {
                double timeTemp = *reinterpret_cast<double*>(pkgs_[i].content.data());
                if (perviousTime > timeTemp)
                {
                    RestartTimes restartTime;
                    restartTime.restart_time_  = perviousTime;
                    restartTime.restart_index_ = perviousIndex;
                    restartTimes.push_back(restartTime);
                }
                perviousTime  = timeTemp;
                perviousIndex = static_cast<unsigned int>(i);
                if (restartTimes.size() > 0)
                {
                    for (size_t j = 0; j < restartTimes.size(); j++)
                    {
                        if (isEqualDouble(restartTimes[j].next_time_, LARGE_NUMBER))  // only once
                        {
                            if (restartTimes[j].restart_time_ < timeTemp)
                            {
                                restartTimes[j].next_time_  = timeTemp;
                                restartTimes[j].next_index_ = static_cast<unsigned int>(i);
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
        if (pkgs_[i].hdr.id == static_cast<int>(datLogger::PackageId::TIME_SERIES))
        {
            double timeTemp = *reinterpret_cast<double*>(pkgs_[i].content.data());
            index_          = static_cast<unsigned int>(i);
            time_           = timeTemp;
            break;
        }
    }
}

void Replay::GoToPreviousFrame()
{
    for (size_t i = static_cast<size_t>(index_) - 1; static_cast<int>(i) >= 0; i--)
    {
        if (pkgs_[i].hdr.id == static_cast<int>(datLogger::PackageId::TIME_SERIES))
        {
            double timeTemp = *reinterpret_cast<double*>(pkgs_[i].content.data());
            index_          = static_cast<unsigned int>(i);
            time_           = timeTemp;
            break;
        }
    }
}
void Replay::UpdateCache()
{
    scenarioState.sim_time        = time_;
    std::vector<int> objIdIndices = GetNumberOfObjectsAtTime();
    for (size_t l = 0; l < objIdIndices.size(); l++)
    {
        int obj_id = *reinterpret_cast<int*>(pkgs_[static_cast<size_t>(objIdIndices[l])].content.data());
        int pkgCnt = GetPkgCntBtwObj(static_cast<size_t>(objIdIndices[l]));
        for (size_t i = 0; i < scenarioState.obj_states.size(); i++)  // loop current state object id to find the object id
        {
            if (scenarioState.obj_states[i].id == obj_id)  // found object id
            {
                for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
                {
                    for (size_t k = static_cast<size_t>(objIdIndices[l] + 1); k < static_cast<size_t>(objIdIndices[l] + pkgCnt + 1);
                         k++)  // start with Index + 1, Index will have object id package index. looking from next package
                    {
                        datLogger::PackageId id_ = ReadPkgHdr(scenarioState.obj_states[i].pkgs[j].pkg);
                        if (id_ == static_cast<datLogger::PackageId>(pkgs_[k].hdr.id))
                        {
                            scenarioState.obj_states[i].pkgs[j].pkg = reinterpret_cast<char*>(&pkgs_[k]);
                            break;
                        }
                    }
                }
            }
        }
    }
}

void Replay::UpdateObjStatus(int id, bool status)
{
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)  // loop current state object id to find the object id
    {
        if (scenarioState.obj_states[i].id == id)
        {
            scenarioState.obj_states[i].active = status;
        }
    }
}

void Replay::CheckObjAvailabilityForward()
{
    // check all obj status in current time frame
    std::vector<int> objIdIndices = GetNumberOfObjectsAtTime();
    for (size_t Index = 0; Index < objIdIndices.size(); Index++)
    {
        int obj_id = *reinterpret_cast<int*>(pkgs_[static_cast<size_t>(objIdIndices[Index])].content.data());
        if (pkgs_[static_cast<size_t>(objIdIndices[Index] + 1)].hdr.id == static_cast<int>(datLogger::PackageId::OBJ_DELETED))
        {
            UpdateObjStatus(obj_id, false);  // obj not active in cache
            continue;
        }
        else if (pkgs_[static_cast<size_t>(objIdIndices[Index] + 1)].hdr.id == static_cast<int>(datLogger::PackageId::OBJ_ADDED))
        {
            if (IsObjAvailableInCache(obj_id))  // make sure same obj deleted before adding the same object
            {
                DeleteObjState(obj_id);  // make sure same obj deleted before adding the same object
            }
            AddObjState(static_cast<size_t>(objIdIndices[Index]));  // obj added in cache. this will also initiate this obj
            continue;
        }
    }
}

void Replay::CheckObjAvailabilityBackward()
{
    // check all obj status in current time frame
    std::vector<int> objIdIndices = GetNumberOfObjectsAtTime();
    for (size_t Index = 0; Index < objIdIndices.size(); Index++)
    {
        int obj_id = *reinterpret_cast<int*>(pkgs_[static_cast<size_t>(objIdIndices[Index])].content.data());
        if (pkgs_[static_cast<size_t>(objIdIndices[Index] + 1)].hdr.id == static_cast<int>(datLogger::PackageId::OBJ_DELETED))
        {
            UpdateObjStatus(obj_id, true);  // obj added in cache with same state as when its deleted
            continue;
        }
        else if (pkgs_[static_cast<size_t>(objIdIndices[Index] + 1)].hdr.id == static_cast<int>(datLogger::PackageId::OBJ_ADDED) &&
                 !isEqualDouble(time_, startTime_))  // ignore first time frame
        {
            UpdateObjStatus(obj_id, false);
            continue;
        }
    }
}

int Replay::GoToTime(double t, bool stopAtEachFrame)
{
    if ((t > stopTime_) || isEqualDouble(t, stopTime_))  // go to stop time
    {
        t = stopTime_;
    }
    else if ((t < startTime_) || isEqualDouble(t, startTime_))  // go to start time
    {
        t = startTime_;
    }

    bool timeLapsed = false;
    if (isEqualDouble(t, scenarioState.sim_time))
    {
        return 0;  // no update to cache
    }
    else
    {
        if (scenarioState.sim_time < t)
        {
            while (!timeLapsed)
            {
                double       pervious_time_  = time_;
                unsigned int pervious_index_ = index_;
                GoToNextFrame();
                if (time_ > t + SMALL_NUMBER)  // gone past requested time
                {
                    time_                  = pervious_time_;
                    index_                 = pervious_index_;
                    scenarioState.sim_time = t;  // sim time should be given time
                    break;
                }
                else if (stopAtEachFrame)  // stop at each min time frame also(might be some time frame might not written so each frame might not have
                                           // time)
                {
                    if (time_ - scenarioState.sim_time > deltaTime_)
                    {
                        time_  = pervious_time_;
                        index_ = pervious_index_;
                        scenarioState.sim_time += deltaTime_;
                        break;
                    }
                    else
                    {
                        timeLapsed = true;
                    }
                }
                else if ((time_ + SMALL_NUMBER < pervious_time_ &&
                          show_restart_) ||          // next time less than pervious time. break only when show restart
                         (isEqualDouble(t, time_)))  // requested time reached
                {
                    timeLapsed = true;
                }

                CheckObjAvailabilityForward();
                UpdateCache();
            }
        }
        if (scenarioState.sim_time > t)
        {
            while (!timeLapsed)
            {
                GoToPreviousFrame();

                if (restartTimes.size() > 0)
                {
                    for (size_t j = 0; j < restartTimes.size(); j++)
                    {
                        if ((restartTimes[j].next_index_ == index_) && (!show_restart_))  // go to restarted time from restart finished next time
                        {
                            index_ = restartTimes[j].restart_index_;  // jump, skip all time frames belong during restart
                            time_  = restartTimes[j].restart_time_;
                            break;
                        }
                        else if ((restartTimes[j].restart_index_ == index_) && (show_restart_))  // go to restarted time from restart first time
                        {
                            timeLapsed = true;
                            break;
                        }
                    }
                }

                double       pervious_time_  = time_;
                unsigned int pervious_index_ = index_;
                if (stopAtEachFrame)  // stop at each min time frame also(might be some time frame might not written so each frame might not have
                                      // time)
                {
                    if (scenarioState.sim_time - time_ > deltaTime_)
                    {
                        time_  = pervious_time_;
                        index_ = pervious_index_;
                        scenarioState.sim_time += deltaTime_;
                        break;
                    }
                    else
                    {
                        timeLapsed = true;
                    }
                }
                else if ((isEqualDouble(t, time_)) ||  // requested time equal to next time
                         (time_ < t + SMALL_NUMBER))   // gone past requested time
                {
                    timeLapsed = true;
                }

                UpdateCache();
                CheckObjAvailabilityBackward();

                if (time_ < t + SMALL_NUMBER)  // gone past requested time, sim time should be given time
                {
                    scenarioState.sim_time = t;
                }
            }
        }
    }
    return 0;
}

// int scenarioengine::Replay::GoToForwardTime(double time_frame, bool stopAtEachFrame)
// {
//     return 0;
// }

void Replay::SetStopEntries()
{
    for (size_t i = pkgs_.size() - 1; static_cast<int>(i) >= 0; i--)
    {
        if (static_cast<datLogger::PackageId>(pkgs_[i].hdr.id) == datLogger::PackageId::TIME_SERIES)
        {
            stopTime_  = *reinterpret_cast<double*>(pkgs_[i].content.data());
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
        if (static_cast<datLogger::PackageId>(pkgs_[i].hdr.id) == datLogger::PackageId::TIME_SERIES)  // find time pkg for given pkg idx
        {
            count_ += 1;
            if (count == count_)
            {
                timeTemp = *reinterpret_cast<double*>(pkgs_[i].content.data());
                break;
            }
        }
    }
    return timeTemp;
}

void Replay::AddObjState(size_t idx)
{
    ObjectStateWithObjId stateObjId;
    if (static_cast<datLogger::PackageId>(pkgs_[idx].hdr.id) != datLogger::PackageId::OBJ_ID)
    {
        std::cout << " Initialization error->Stop replay " << std::endl;
    }
    stateObjId.id     = *reinterpret_cast<int*>(pkgs_[idx].content.data());
    stateObjId.active = true;
    int pkgCount      = GetPkgCntBtwObj(idx);

    for (size_t i = idx + 1; i < static_cast<size_t>(pkgCount) + idx + 1; i++)
    {  // GetPkgCntBtwObj will return count of package

        if (datLogger::PackageId::OBJ_ADDED == static_cast<datLogger::PackageId>(pkgs_[i].hdr.id) ||
            datLogger::PackageId::OBJ_DELETED == static_cast<datLogger::PackageId>(pkgs_[i].hdr.id))
        {
            continue;  // skip packages.
        }
        ObjectStateWithPkg statePkg;
        statePkg.pkg = reinterpret_cast<char*>(&pkgs_[i]);
        stateObjId.pkgs.push_back(statePkg);
    }
    scenarioState.obj_states.push_back(stateObjId);
}

void Replay::DeleteObjState(int objId)
{
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)  // loop current state object id to find the object id
    {
        if (scenarioState.obj_states[i].id == objId)  // found object id
        {
            // scenarioState.obj_states[i].active = false;
            scenarioState.obj_states.erase(scenarioState.obj_states.begin() + static_cast<unsigned int>(i));
            break;
        }
    }
}

void Replay::InitiateStates()
{
    // reset the timings
    scenarioState.obj_states.clear();
    scenarioState.sim_time = startTime_;
    time_                  = startTime_;
    index_                 = startIndex_;

    std::vector<int> objIdIndices = GetNumberOfObjectsAtTime();

    for (size_t Index = 0; Index < objIdIndices.size(); Index++)
    {
        AddObjState(static_cast<size_t>(objIdIndices[Index]));
    }
}

int Replay::GetModelID(int obj_id)
{
    int model_id = -1;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::MODEL_ID)
            {
                model_id = *reinterpret_cast<int*>(pkg->content.data());
            }
        }
    }
    return model_id;
}

int Replay::GetCtrlType(int obj_id)
{
    int ctrl_type = -1;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::CTRL_TYPE)
            {
                ctrl_type = *reinterpret_cast<int*>(pkg->content.data());
            }
        }
    }
    return ctrl_type;
}

int Replay::GetObjCategory(int obj_id)
{
    int objCategory = -1;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::OBJ_CATEGORY)
            {
                objCategory = *reinterpret_cast<int*>(pkg->content.data());
            }
        }
    }
    return objCategory;
}

int Replay::GetBB(int obj_id, OSCBoundingBox& bb)
{
    datLogger::BoundingBox bb_;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::BOUNDING_BOX)
            {
                bb_ = *reinterpret_cast<datLogger::BoundingBox*>(pkg->content.data());
                break;
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

int Replay::GetScaleMode(int obj_id)
{
    int scale_mode = -1;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::SCALE_MODE)
            {
                scale_mode = *reinterpret_cast<int*>(pkg->content.data());
            }
        }
    }

    return scale_mode;
}

int Replay::GetVisibility(int obj_id)
{
    int vis = -1;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::VISIBILITY_MASK)
            {
                vis = *reinterpret_cast<int*>(pkg->content.data());
            }
        }
    }

    return vis;
}

datLogger::Pos Replay::GetPos(int obj_id)
{
    datLogger::Pos pos;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::POSITIONS)
            {
                pos = *reinterpret_cast<datLogger::Pos*>(pkg->content.data());
                break;
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
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::ROAD_ID)
            {
                road_id = *reinterpret_cast<id_t*>(pkg->content.data());
            }
        }
    }

    return road_id;
}

int Replay::GetLaneId(int obj_id)
{
    int lane_id = -1;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::LANE_ID)
            {
                lane_id = *reinterpret_cast<int*>(pkg->content.data());
            }
        }
    }

    return lane_id;
}

double Replay::GetPosOffset(int obj_id)
{
    double offset = 0.0;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::POS_OFFSET)
            {
                offset = *reinterpret_cast<double*>(pkg->content.data());
            }
        }
    }

    return offset;
}

float Replay::GetPosT(int obj_id)
{
    double pos_t = 0.0;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::POS_T)
            {
                pos_t = *reinterpret_cast<double*>(pkg->content.data());
            }
        }
    }

    return static_cast<float>(pos_t);
}

float Replay::GetPosS(int obj_id)
{
    double pos_s = 0.0;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::POS_S)
            {
                pos_s = *reinterpret_cast<double*>(pkg->content.data());
            }
        }
    }

    return static_cast<float>(pos_s);
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

double Replay::GetWheelAngle(int obj_id)
{
    double wheel_angle = 0.0;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::WHEEL_ANGLE)
            {
                wheel_angle = *reinterpret_cast<double*>(pkg->content.data());
            }
        }
    }

    return wheel_angle;
}

double Replay::GetWheelRot(int obj_id)
{
    double wheel_rot = 0.0;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::WHEEL_ROT)
            {
                wheel_rot = *reinterpret_cast<double*>(pkg->content.data());
            }
        }
    }

    return wheel_rot;
}

double Replay::GetSpeed(int obj_id)
{
    double speed = 0.0;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::SPEED)
            {
                speed = *reinterpret_cast<double*>(pkg->content.data());
            }
        }
    }

    return speed;
}

int Replay::GetName(int obj_id, std::string& name)
{
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::NAME)
            {
                name = pkg->content.data();
            }
        }
    }
    return 0;
}

void Replay::GetRgbValues(int obj_id, Object::VehicleLightActionStatus* vehicleLightActionStatusList)
{
    datLogger::LightState lightState_;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::LIGHT_STATES)
            {
                lightState_ = *reinterpret_cast<datLogger::LightState*>(pkg->content.data());
            }
        }
    }

    const size_t numLights = sizeof(lightState_) / sizeof(datLogger::LightRGB);
    for (size_t i = 0; i < numLights; ++i)
    {
        datLogger::LightRGB* light         = reinterpret_cast<datLogger::LightRGB*>(&lightState_) + i;
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
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            continue;
        }
        for (size_t j = 0; j < scenarioState.obj_states[i].pkgs.size(); j++)
        {
            datLogger::CommonPkg* pkg;
            pkg = reinterpret_cast<datLogger::CommonPkg*>(scenarioState.obj_states[i].pkgs[j].pkg);
            if (static_cast<datLogger::PackageId>(pkg->hdr.id) == datLogger::PackageId::LIGHT_STATES)
            {
                light_states_ = *reinterpret_cast<datLogger::LightState*>(pkg->content.data());
            }
        }
    }
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

    while (true)
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
                    if (isEqualDouble(timeTemp, cur_timestamp))
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

        if (endOfScenarioCount == static_cast<int>(cur_idx.size()))
        {
            break;  // reached end of file
        }
    }
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

            if (datLogger_->init(filename, header_.version, header_.odrFilename.string.data(), header_.modelFilename.string.data()) != 0)
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
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            scenarioState.obj_states[i].odometer = value;
        }
    }
}

double Replay::GetOdaMeter(int obj_id)
{
    double odaMeter = 0.0;
    for (size_t i = 0; i < scenarioState.obj_states.size(); i++)
    {
        if (scenarioState.obj_states[i].id != obj_id)
        {
            odaMeter = scenarioState.obj_states[i].odometer;
        }
    }
    return odaMeter;
}
