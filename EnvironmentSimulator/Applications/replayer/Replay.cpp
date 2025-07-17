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

using namespace scenarioengine;

Replay::Replay(std::string filename, bool clean) : time_(0.0), index_(0), repeat_(false), clean_(clean)
{
    file_.open(filename, std::ifstream::binary);
    if (file_.fail())
    {
        LOG_ERROR_AND_QUIT("Cannot open file: {}", filename);
    }

    LOG_INFO("Datfile {} opened.", FileNameOf(filename));

    // Get the file size
    file_.seekg(0, std::ios::end);
    const auto file_size = file_.tellg();
    file_.seekg(0, std::ios::beg);

    ReplayEntry replay_entry;
    while (file_.tellg() < file_size)
    {
        Dat::PacketHeader header;
        if (!file_.read(reinterpret_cast<char*>(&header), sizeof(header)))
        {
            LOG_ERROR("Failed to read packet header.");
            break;
        }

        switch (header.id)
        {
            case static_cast<int>(Dat::PacketId::DAT_HEADER):
            {
                Dat::DatHeader d_header;

                if (!file_.read(reinterpret_cast<char*>(&header_.version_major), sizeof(header_.version_major)) ||
                    !file_.read(reinterpret_cast<char*>(&header_.version_minor), sizeof(header_.version_minor)))
                {
                    LOG_ERROR("Failed reading header versions.");
                    break;
                }

                if (ReadStringPacket(header_.odr_filename.string) != 0)
                {
                    LOG_ERROR("Failed reading odr filename.");
                    break;
                }

                if (ReadStringPacket(header_.model_filename.string) != 0)
                {
                    LOG_ERROR("Failed reading model filename.");
                    break;
                }

                break;
            }
            case static_cast<id_t>(Dat::PacketId::TIMESTAMP):
            {
                replay_entry.odometer = 0.0;
                if (ReadPacket(header, timestamp_) != 0)
                    LOG_ERROR("Failed reading speed data.");
                /* We set the timestamp when OBJ_ID packet is read, since we always transmit in the following order:
                 - time
                 - obj_id_1
                 - data...
                 - obj_id_2
                 - data_2
                 - time
                 - etc.
                */
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_ID):
            {
                if (object_ids_.count(replay_entry.state.info.id) != 0)  // Have we processed this object before?
                {
                    // Yes we have, so we add the replay entry and store it as cache before reading a new ID and updating all its data.
                    // This works because of the flow described in PacketId::TIMESTAMP
                    obj_events_map_[replay_entry.state.info.id].push_back(replay_entry);
                    object_state_cache_[replay_entry.state.info.id] = replay_entry;
                }

                // Get the new object ID
                int id;
                if (ReadPacket(header, id) != 0)
                    LOG_ERROR("Failed reading object ID.");

                if (object_state_cache_.count(id) != 0)
                {
                    // If the object already exists in the cache, fetch latest known state
                    replay_entry = object_state_cache_[id];
                }

                // We set the latest timestamp which is already fetched
                replay_entry.state.info.timeStamp = static_cast<float>(timestamp_);
                replay_entry.state.info.active    = true;
                replay_entry.state.info.id        = id;  // Could be done earlier I guess, but we do it here for clarity

                object_ids_.insert(replay_entry.state.info.id);  // Add the object ID to the set

                // Proceed with reading the rest of the packet data
                break;
            }
            case static_cast<id_t>(Dat::PacketId::SPEED):
            {
                double speed;
                if (ReadPacket(header, speed) != 0)
                    LOG_ERROR("Failed reading speed data.");
                replay_entry.state.info.speed = static_cast<float>(speed);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POSE):
            {
                Dat::Pose pose;
                if (ReadPacket(header, pose.x, pose.y, pose.z, pose.h, pose.p, pose.r) != 0)
                    LOG_ERROR("Failed reading pose data.");

                replay_entry.state.pos.x = static_cast<float>(pose.x);
                replay_entry.state.pos.y = static_cast<float>(pose.y);
                replay_entry.state.pos.z = static_cast<float>(pose.z);
                replay_entry.state.pos.h = static_cast<float>(pose.h);
                replay_entry.state.pos.p = static_cast<float>(pose.p);
                replay_entry.state.pos.r = static_cast<float>(pose.r);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::MODEL_ID):
            {
                if (ReadPacket(header, replay_entry.state.info.model_id) != 0)
                    LOG_ERROR("Failed reading model ID.");
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_TYPE):
            {
                if (ReadPacket(header, replay_entry.state.info.obj_type) != 0)
                    LOG_ERROR("Failed reading object type.");
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_CATEGORY):
            {
                if (ReadPacket(header, replay_entry.state.info.obj_category) != 0)
                    LOG_ERROR("Failed reading object category.");
                break;
            }
            case static_cast<id_t>(Dat::PacketId::CTRL_TYPE):
            {
                if (ReadPacket(header, replay_entry.state.info.ctrl_type) != 0)
                    LOG_ERROR("Failed reading controller type.");
                break;
            }
            case static_cast<id_t>(Dat::PacketId::WHEEL_ANGLE):
            {
                double wheel_angle;
                if (ReadPacket(header, wheel_angle) != 0)
                    LOG_ERROR("Failed reading wheel angle.");
                replay_entry.state.info.wheel_angle = static_cast<float>(wheel_angle);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::WHEEL_ROT):
            {
                double wheel_rot;
                if (ReadPacket(header, wheel_rot) != 0)
                    LOG_ERROR("Failed reading wheel rotation.");
                replay_entry.state.info.wheel_rot = static_cast<float>(wheel_rot);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::BOUNDING_BOX):
            {
                Dat::BoundingBox boundingbox;
                if (ReadPacket(header, boundingbox.x, boundingbox.y, boundingbox.z, boundingbox.length, boundingbox.width, boundingbox.height) != 0)
                {
                    LOG_ERROR("Failed reading bounding box data.");
                }
                replay_entry.state.info.boundingbox.center_.x_          = static_cast<float>(boundingbox.x);
                replay_entry.state.info.boundingbox.center_.y_          = static_cast<float>(boundingbox.y);
                replay_entry.state.info.boundingbox.center_.z_          = static_cast<float>(boundingbox.z);
                replay_entry.state.info.boundingbox.dimensions_.length_ = static_cast<float>(boundingbox.length);
                replay_entry.state.info.boundingbox.dimensions_.width_  = static_cast<float>(boundingbox.width);
                replay_entry.state.info.boundingbox.dimensions_.height_ = static_cast<float>(boundingbox.height);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::SCALE_MODE):
            {
                if (ReadPacket(header, replay_entry.state.info.scaleMode) != 0)
                    LOG_ERROR("Failed reading scale mode.");
                break;
            }
            case static_cast<id_t>(Dat::PacketId::VISIBILITY_MASK):
            {
                if (ReadPacket(header, replay_entry.state.info.visibilityMask) != 0)
                    LOG_ERROR("Failed reading visibility mask.");
                break;
            }
            case static_cast<id_t>(Dat::PacketId::NAME):
            {
                std::string name;
                if (ReadStringPacket(name) != 0)
                    LOG_ERROR("Failed reading name.");
                id_to_name_[replay_entry.state.info.id] = std::move(name);  // Tranfer ownership to the map, ensuring the name is retained
                replay_entry.state.info.name            = id_to_name_[replay_entry.state.info.id].c_str();
                break;
            }
            case static_cast<id_t>(Dat::PacketId::ROAD_ID):
            {
                if (ReadPacket(header, replay_entry.state.pos.roadId) != 0)
                    LOG_ERROR("Failed reading road ID.");
                break;
            }
            case static_cast<id_t>(Dat::PacketId::LANE_ID):
            {
                if (ReadPacket(header, replay_entry.state.pos.laneId) != 0)
                    LOG_ERROR("Failed reading lane ID.");
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_OFFSET):
            {
                double offset;
                if (ReadPacket(header, offset) != 0)
                    LOG_ERROR("Failed reading position offset.");
                replay_entry.state.pos.offset = static_cast<float>(offset);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_T):
            {
                double t;
                if (ReadPacket(header, t) != 0)
                    LOG_ERROR("Failed reading position T.");
                replay_entry.state.pos.t = static_cast<float>(t);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_S):
            {
                double s;
                if (ReadPacket(header, s) != 0)
                    LOG_ERROR("Failed reading position S.");
                replay_entry.state.pos.s = static_cast<float>(s);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_DELETED):
            {
                LOG_INFO("Object {} deleted at timestamp: {}", replay_entry.state.info.id, timestamp_);
                replay_entry.state.info.active = false;
                break;
            }
            case static_cast<id_t>(Dat::PacketId::END_OF_SCENARIO):
            {
                if (ReadPacket(header, stopTime_) != 0)
                    LOG_ERROR("Failed reading end of scenario timestamp.");
                LOG_INFO("End of scenario reached at timestamp: {}", stopTime_);
                break;
            }
            default:
            {
                LOG_ERROR("Unknown packet id: {}", header.id);
                file_.seekg(header.data_size, std::ios::cur);
                break;
            }
        }
    }

    file_.close();

    // Add entries to data_ vector
    BuildDataFromPackets();
}

void Replay::BuildDataFromPackets()
{
    object_state_cache_.clear();

    double dt  = 0.01;
    stopIndex_ = static_cast<unsigned int>(stopTime_ / dt);

    // Find start time, maybe its earlier than 0.0s
    for (const auto& [id, entry] : obj_events_map_)
    {
        startTime_  = static_cast<double>(std::min(static_cast<float>(startTime_), entry.front().state.info.timeStamp));
        startIndex_ = 0;
    }

    for (double t = startTime_; t <= stopTime_; t += dt)
    {
        for (const int obj_id : object_ids_)
        {
            const auto& events = obj_events_map_[obj_id];

            auto& last_state = object_state_cache_[obj_id];

            if (!events.empty())
            {
                for (const auto& event : events)
                {
                    if (static_cast<double>(event.state.info.timeStamp) <= t)
                    {
                        last_state = event;  // Update the last state to the most recent event before or at time t
                    }
                    else
                    {
                        break;  // No need to check further, as events are sorted by timestamp
                    }
                }
            }

            ReplayEntry entry          = last_state;
            entry.state.info.timeStamp = static_cast<float>(t);
            if (entry.state.info.active)
            {
                data_.push_back(entry);
            }
        }
    }
}

Replay::Replay(const std::string directory, const std::string scenario, std::string create_datfile)
    : time_(0.0),
      index_(0),
      repeat_(false),
      create_datfile_(create_datfile)
{
    GetReplaysFromDirectory(directory, scenario);
    std::vector<std::pair<std::string, std::vector<ReplayEntry>>> scenarioData;

    for (size_t i = 0; i < scenarios_.size(); i++)
    {
        file_.open(scenarios_[i], std::ofstream::binary);
        if (file_.fail())
        {
            LOG_ERROR("Cannot open file: {}", scenarios_[i]);
            throw std::invalid_argument(std::string("Cannot open file: ") + scenarios_[i]);
        }
        file_.read(reinterpret_cast<char*>(&header_), sizeof(header_));
        LOG_INFO("Recording {} opened. dat version: {}.{} odr: {} model: {}",
                 FileNameOf(scenarios_[i]),
                 header_.version_major,
                 header_.version_minor,
                 FileNameOf(header_.odr_filename.string),
                 FileNameOf(header_.model_filename.string));

        if (header_.version_major != DAT_FILE_FORMAT_VERSION_MAJOR || header_.version_minor != DAT_FILE_FORMAT_VERSION_MINOR)
        {
            LOG_ERROR_AND_QUIT("Version mismatch. {} is version {}.{} while supported version is {}.{}. Please re-create dat file.",
                               scenarios_[i],
                               header_.version_major,
                               header_.version_minor,
                               DAT_FILE_FORMAT_VERSION_MAJOR,
                               DAT_FILE_FORMAT_VERSION_MINOR);
        }
        while (!file_.eof())
        {
            ReplayEntry entry;
            entry.odometer = 0.0;
            file_.read(reinterpret_cast<char*>(&entry.state), sizeof(entry.state));

            if (!file_.eof())
            {
                data_.push_back(entry);
            }
        }
        // pair <scenario name, scenario data>
        scenarioData.push_back(std::make_pair(scenarios_[i], data_));
        data_ = {};
        file_.close();
    }

    if (scenarioData.size() < 2)
    {
        LOG_ERROR_AND_QUIT("Too few scenarios loaded, use single replay feature instead\n");
    }

    // Scenario with smallest start time first
    std::sort(scenarioData.begin(),
              scenarioData.end(),
              [](const auto& sce1, const auto& sce2) { return sce1.second[0].state.info.timeStamp < sce2.second[0].state.info.timeStamp; });

    // Log which scenario belongs to what ID-group (0, 100, 200 etc.)
    for (size_t i = 0; i < scenarioData.size(); i++)
    {
        std::string scenario_tmp = scenarioData[i].first;
        LOG_INFO("Scenarios corresponding to IDs ({}:{}): {}", i * 100, (i + 1) * 100 - 1, FileNameOf(scenario_tmp));
    }

    // Ensure increasing timestamps. Remove any other entries.
    for (auto& sce : scenarioData)
    {
        CleanEntries(sce.second);
    }

    // Build remaining data in order.
    BuildData(scenarioData);

    if (data_.size() > 0)
    {
        // Register first entry timestamp as starting time
        time_       = data_[0].state.info.timeStamp;
        startTime_  = time_;
        startIndex_ = 0;

        // Register last entry timestamp as stop time
        stopTime_  = data_.back().state.info.timeStamp;
        stopIndex_ = static_cast<unsigned int>(FindIndexAtTimestamp(stopTime_));
    }

    if (!create_datfile_.empty())
    {
        CreateMergedDatfile(create_datfile_);
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

size_t Replay::GetNumberOfScenarios() const
{
    return scenarios_.size();
}

Replay::~Replay()
{
    data_.clear();
}

void Replay::GoToStart()
{
    index_ = startIndex_;
    time_  = startTime_;
}

void Replay::GoToEnd()
{
    if (repeat_)
    {
        index_ = startIndex_;
        time_  = startTime_;
    }
    else
    {
        index_ = stopIndex_;
        time_  = stopTime_;
    }
}

void Replay::GoToTime(double time, bool stop_at_next_frame)
{
    if (!stop_at_next_frame)
    {
        if (time > stopTime_)
        {
            GoToEnd();
        }
        else if (time < GetStartTime())
        {
            GoToStart();
        }
        else
        {
            index_ = static_cast<unsigned int>(FindIndexAtTimestamp(time, static_cast<int>(index_)));
            time_  = time;
        }
    }
    else
    {
        if (time > time_)
        {
            size_t next_index = FindNextTimestamp();
            if (next_index > index_ && time > static_cast<double>(data_[next_index].state.info.timeStamp) &&
                static_cast<double>(data_[next_index].state.info.timeStamp) <= GetStopTime())
            {
                index_ = static_cast<unsigned int>(next_index);
                time_  = data_[index_].state.info.timeStamp;
            }
            else
            {
                if (time > GetStopTime())
                {
                    GoToEnd();
                }
                else
                {
                    time_ = time;
                }
            }
        }
        else if (time < time_)
        {
            size_t next_index = FindPreviousTimestamp();
            if (next_index < index_ && time < static_cast<double>(data_[next_index].state.info.timeStamp))
            {
                index_ = static_cast<unsigned int>(next_index);
                time_  = data_[index_].state.info.timeStamp;
            }
            else
            {
                if (time < GetStartTime())
                {
                    GoToStart();
                }
                else
                {
                    time_ = time;
                }
            }
        }
    }
}

void Replay::GoToDeltaTime(double dt, bool stop_at_next_frame)
{
    GoToTime(time_ + dt, stop_at_next_frame);
}

int Replay::GoToNextFrame()
{
    if (data_.empty())
    {
        return -1;
    }

    float ctime = data_[index_].state.info.timeStamp;
    for (size_t i = index_ + 1; i < data_.size(); i++)
    {
        if (data_[i].state.info.timeStamp > ctime)
        {
            GoToTime(data_[i].state.info.timeStamp);
            return static_cast<int>(i);
        }
    }
    return -1;
}

void Replay::GoToPreviousFrame()
{
    if (index_ > 0)
    {
        GoToTime(data_[index_ - 1].state.info.timeStamp);
    }
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

    for (i = startSearchIndex; i < static_cast<int>(data_.size()); i++)
    {
        if (static_cast<double>(data_[static_cast<unsigned int>(i)].state.info.timeStamp) >= timestamp)
        {
            break;
        }
    }

    return MIN(i, static_cast<int>(data_.size()) - 1);
}

unsigned int Replay::FindNextTimestamp(bool wrap) const
{
    unsigned int index = index_ + 1;
    for (; index < data_.size(); index++)
    {
        if (data_[index].state.info.timeStamp > data_[index_].state.info.timeStamp)
        {
            break;
        }
    }

    if (index >= data_.size())
    {
        if (wrap)
        {
            return 0;
        }
        else
        {
            return index_;  // stay on current index
        }
    }

    return index;
}

unsigned int Replay::FindPreviousTimestamp(bool wrap) const
{
    int index = static_cast<int>(index_) - 1;

    if (index < 0)
    {
        if (wrap)
        {
            index = static_cast<int>(data_.size()) - 1;
        }
        else
        {
            return 0;
        }
    }

    for (int i = index - 1; i >= 0; i--)
    {
        // go backwards until we identify the first entry with same timestamp
        if (data_[static_cast<unsigned int>(i)].state.info.timeStamp < data_[static_cast<unsigned int>(index)].state.info.timeStamp)
        {
            break;
        }
        index = i;
    }

    return static_cast<unsigned int>(index);
}

ReplayEntry* Replay::GetEntry(int id)
{
    // Read all vehicles at current timestamp
    float timestamp = data_[index_].state.info.timeStamp;
    int   i         = 0;
    while (index_ + static_cast<unsigned int>(i) < data_.size() && !(data_[index_ + static_cast<unsigned int>(i)].state.info.timeStamp > timestamp))
    {
        if (data_[index_ + static_cast<unsigned int>(i)].state.info.id == id)
        {
            return &data_[index_ + static_cast<unsigned int>(i)];
        }
        i++;
    }

    return nullptr;
}

ObjectStateStructDat* Replay::GetState(int id)
{
    ReplayEntry* entry = GetEntry(id);
    if (entry != nullptr)
    {
        return &entry->state;
    }
    else
    {
        return nullptr;
    }
}

void Replay::SetStartTime(double time)
{
    startTime_ = time;
    if (time_ < startTime_)
    {
        time_ = startTime_;
    }

    startIndex_ = static_cast<unsigned int>(FindIndexAtTimestamp(startTime_));
}

void Replay::SetStopTime(double time)
{
    stopTime_ = time;
    if (time_ > stopTime_)
    {
        time_ = stopTime_;
    }

    stopIndex_ = static_cast<unsigned int>(FindIndexAtTimestamp(stopTime_));
}

void Replay::CleanEntries(std::vector<ReplayEntry>& entries)
{
    if (entries.empty())
    {
        return;
    }

    for (unsigned int i = 0; i < entries.size() - 1; i++)
    {
        if (entries[i + 1].state.info.timeStamp < entries[i].state.info.timeStamp)
        {
            entries.erase(entries.begin() + i + 1);
            i--;
        }

        for (unsigned int j = 1; (i + j < entries.size()) && NEAR_NUMBERSF(entries[i + j].state.info.timeStamp, entries[i].state.info.timeStamp); j++)
        {
            // Keep the latest instance of entries with same timestamp
            if (entries[i + j].state.info.id == entries[i].state.info.id)
            {
                entries.erase(entries.begin() + i);
                i--;
                break;
            }
        }
    }
}

void Replay::BuildData(std::vector<std::pair<std::string, std::vector<ReplayEntry>>>& scenarios)
{
    // Keep track of current index of each scenario
    std::vector<int> cur_idx;
    std::vector<int> next_idx;

    for (size_t j = 0; j < scenarios.size(); j++)
    {
        cur_idx.push_back(0);
        next_idx.push_back(0);
    }

    // Set scenario ID-group (0, 100, 200 etc.)
    for (size_t j = 0; j < scenarios.size(); j++)
    {
        for (size_t k = 0; k < scenarios[j].second.size(); k++)
        {
            // Set scenario ID-group (0, 100, 200 etc.)
            scenarios[j].second[k].state.info.id += static_cast<int>(j) * 100;
        }
    }

    // Populate data_ based on first (with lowest timestamp) scenario
    double cur_timestamp = static_cast<double>(scenarios[0].second[0].state.info.timeStamp);
    while (cur_timestamp < LARGE_NUMBER - SMALL_NUMBER)
    {
        // populate entries if all scenarios at current time step
        double min_time_stamp = LARGE_NUMBER;
        for (size_t j = 0; j < scenarios.size(); j++)
        {
            if (next_idx[j] != -1)
            {
                unsigned int k = static_cast<unsigned int>(cur_idx[j]);
                for (; k < scenarios[j].second.size() && static_cast<double>(scenarios[j].second[k].state.info.timeStamp) < cur_timestamp + 1e-6; k++)
                {
                    // push entry with modified timestamp
                    scenarios[j].second[k].state.info.timeStamp = static_cast<float>(cur_timestamp);
                    data_.push_back(scenarios[j].second[k]);
                }

                if (k < scenarios[j].second.size())
                {
                    next_idx[j] = static_cast<int>(k);
                    if (static_cast<double>(scenarios[j].second[k].state.info.timeStamp) < min_time_stamp)
                    {
                        min_time_stamp = static_cast<double>(scenarios[j].second[k].state.info.timeStamp);
                    }
                }
                else
                {
                    next_idx[j] = -1;
                }
            }
        }

        if (min_time_stamp < LARGE_NUMBER - SMALL_NUMBER)
        {
            for (size_t j = 0; j < scenarios.size(); j++)
            {
                if (next_idx[j] > 0 && static_cast<double>(scenarios[j].second[static_cast<unsigned int>(next_idx[j])].state.info.timeStamp) <
                                           min_time_stamp + SMALL_NUMBER)
                {
                    // time has reached next entry, step this scenario
                    cur_idx[j] = next_idx[j];
                }
            }
        }

        cur_timestamp = min_time_stamp;
    }
}

void Replay::CreateMergedDatfile(const std::string filename) const
{
    std::ofstream data_file_;
    data_file_.open(filename, std::ofstream::binary);
    if (data_file_.fail())
    {
        LOG_ERROR("Cannot open file: {}", filename);
        exit(-1);
    }

    data_file_.write(reinterpret_cast<const char*>(&header_), sizeof(header_));

    if (data_file_.is_open())
    {
        // Write status to file - for later replay
        for (size_t i = 0; i < data_.size(); i++)
        {
            data_file_.write(reinterpret_cast<const char*>(&data_[i].state), sizeof(data_[i].state));
        }
    }
}

template <typename... Data>
int Replay::ReadPacket(const Dat::PacketHeader& header, Data&... data)
{
    Dat::PacketGeneric packet;
    packet.header = header;
    packet.data.resize(packet.header.data_size);
    if (!file_.read(packet.data.data(), packet.header.data_size))
    {
        return -1;
    }

    const char* read_ptr = packet.data.data();

    (..., (memcpy(&data, read_ptr, sizeof(data)), read_ptr += sizeof(data)));

    return 0;
}

int Replay::ReadStringPacket(std::string& str)
{
    unsigned int size;
    if (!file_.read(reinterpret_cast<char*>(&size), sizeof(size)))
    {
        return -1;
    }
    str.resize(size);
    if (!file_.read(str.data(), size))
    {
        return -1;
    }

    return 0;
}
