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
#include "PacketHandler.hpp"

using namespace scenarioengine;

Replay::Replay(std::string filename, bool clean, float fixed_timestep)
    : time_(0.0),
      index_(0),
      repeat_(false),
      clean_(clean),
      fixed_timestep_(fixed_timestep)
{
    // Parse the packets from the file
    int ret = ParsePackets(filename);
    if (ret != 0)
    {
        LOG_ERROR("Failed to parse packets from file: {}", filename);
        return;
    }

    // Add entries to data_ vector
    BuildDataFromPackets();

    stopIndex_ = static_cast<unsigned int>(FindIndexAtTimestamp(stopTime_));  // Needs data_ to be filled before this can be called

    if (clean)
    {
        CleanEntries(data_);
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

    if (scenarios_.size() < 2)
    {
        LOG_ERROR_AND_QUIT("Too few scenarios loaded, use single replay feature instead\n");
    }

    for (size_t i = 0; i < scenarios_.size(); i++)
    {
        ParsePackets(scenarios_[i]);
        BuildDataFromPackets();
        scenarioData.emplace_back(scenarios_[i], data_);
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

int Replay::ParsePackets(const std::string& filename)
{
    ClearData();

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

    // Read raw header BEFORE reading packets
    if (FillHeader() != 0)
    {
        LOG_ERROR_AND_QUIT("Failed to read DAT header.");
    }

    LOG_INFO("Datfile header: version {}.{}, odr_filename: {}, model_filename: {}",
             header_.version_major,
             header_.version_minor,
             header_.odr_filename.string,
             header_.model_filename.string);

    ReplayEntry replay_entry;

    // Now parse packets
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
            case static_cast<id_t>(Dat::PacketId::TIMESTAMP):
            {
                replay_entry.odometer = 0.0;
                if (ReadPacket(header, timestamp_) != 0)
                    LOG_ERROR("Failed reading speed data.");

                // If we encounter 2 TIMESTAMP packets in a row, we have non-changed objects
                // This only happens when we dont have a fixed timestep, then we need to process these packets
                if (previous_p_id_ == header.id)
                {
                    if (obj_events_map_.count(replay_entry.state.info.id) == 0)
                    {
                        obj_events_map_[replay_entry.state.info.id].push_back(replay_entry);
                    }
                    for (const auto& [obj_id, entry] : obj_events_map_)
                    {
                        if (entry.empty())
                            continue;

                        auto last_entry                 = entry.back();
                        last_entry.state.info.timeStamp = timestamp_;
                        obj_events_map_[obj_id].push_back(last_entry);
                        object_state_cache_[obj_id] = last_entry;
                    }
                }
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

                    // We'll deduce the minimum timestep, might be useful later (excluding ghost objects (ctrl_type == 100))
                    if (replay_entry.state.info.ctrl_type != 100 && !min_timestep_.has_value())
                    {
                        min_timestep_ = abs(timestamp_ - replay_entry.state.info.timeStamp);
                    }
                    else if (replay_entry.state.info.ctrl_type != 100)
                    {
                        float dt = abs(timestamp_ - replay_entry.state.info.timeStamp);
                        if (!(abs(dt) < SMALL_NUMBERF))
                        {
                            // If we already have a minimum timestep, update it if the current one is smaller
                            dt            = std::round(dt * 1000.0f) / 1000.0f;  // avoid floating point precision issues
                            min_timestep_ = std::min(min_timestep_.value(), dt);
                        }
                    }
                }

                // We set the latest timestamp which is already fetched
                replay_entry.state.info.timeStamp = timestamp_;
                replay_entry.state.info.active    = true;
                replay_entry.state.info.id        = id;  // Could be done earlier I guess, but we do it here for clarity

                object_ids_.insert(replay_entry.state.info.id);  // Add the object ID to the set

                // Proceed with reading the rest of the packet data
                break;
            }
            case static_cast<id_t>(Dat::PacketId::SPEED):
            {
                if (ReadPacket(header, replay_entry.state.info.speed) != 0)
                {
                    LOG_ERROR("Failed reading speed data.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POSE):
            {
                if (ReadPacket(header,
                               replay_entry.state.pos.x,
                               replay_entry.state.pos.y,
                               replay_entry.state.pos.z,
                               replay_entry.state.pos.h,
                               replay_entry.state.pos.p,
                               replay_entry.state.pos.r) != 0)
                {
                    LOG_ERROR("Failed reading pose data.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::MODEL_ID):
            {
                if (ReadPacket(header, replay_entry.state.info.model_id) != 0)
                {
                    LOG_ERROR("Failed reading model ID.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_TYPE):
            {
                if (ReadPacket(header, replay_entry.state.info.obj_type) != 0)
                {
                    LOG_ERROR("Failed reading object type.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_CATEGORY):
            {
                if (ReadPacket(header, replay_entry.state.info.obj_category) != 0)
                {
                    LOG_ERROR("Failed reading object category.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::CTRL_TYPE):
            {
                if (ReadPacket(header, replay_entry.state.info.ctrl_type) != 0)
                {
                    LOG_ERROR("Failed reading controller type.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::WHEEL_ANGLE):
            {
                if (ReadPacket(header, replay_entry.state.info.wheel_angle) != 0)
                {
                    LOG_ERROR("Failed reading wheel angle.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::WHEEL_ROT):
            {
                if (ReadPacket(header, replay_entry.state.info.wheel_rot) != 0)
                {
                    LOG_ERROR("Failed reading wheel rotation.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::BOUNDING_BOX):
            {
                if (ReadPacket(header,
                               replay_entry.state.info.boundingbox.center_.x_,
                               replay_entry.state.info.boundingbox.center_.y_,
                               replay_entry.state.info.boundingbox.center_.z_,
                               replay_entry.state.info.boundingbox.dimensions_.length_,
                               replay_entry.state.info.boundingbox.dimensions_.width_,
                               replay_entry.state.info.boundingbox.dimensions_.height_) != 0)
                {
                    LOG_ERROR("Failed reading bounding box data.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::SCALE_MODE):
            {
                if (ReadPacket(header, replay_entry.state.info.scaleMode) != 0)
                {
                    LOG_ERROR("Failed reading scale mode.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::VISIBILITY_MASK):
            {
                if (ReadPacket(header, replay_entry.state.info.visibilityMask) != 0)
                {
                    LOG_ERROR("Failed reading visibility mask.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::NAME):
            {
                std::string name;
                if (ReadStringPacket(name) != 0)
                {
                    LOG_ERROR("Failed reading name.");
                    return -1;
                }
                id_to_name_[replay_entry.state.info.id] = std::move(name);  // Tranfer ownership to the map, ensuring the name is retained
                replay_entry.state.info.name            = id_to_name_[replay_entry.state.info.id].c_str();
                break;
            }
            case static_cast<id_t>(Dat::PacketId::ROAD_ID):
            {
                if (ReadPacket(header, replay_entry.state.pos.roadId) != 0)
                {
                    LOG_ERROR("Failed reading road ID.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::LANE_ID):
            {
                if (ReadPacket(header, replay_entry.state.pos.laneId) != 0)
                {
                    LOG_ERROR("Failed reading lane ID.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_OFFSET):
            {
                if (ReadPacket(header, replay_entry.state.pos.offset) != 0)
                {
                    LOG_ERROR("Failed reading position offset.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_T):
            {
                if (ReadPacket(header, replay_entry.state.pos.t) != 0)
                {
                    LOG_ERROR("Failed reading position T.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_S):
            {
                if (ReadPacket(header, replay_entry.state.pos.s) != 0)
                {
                    LOG_ERROR("Failed reading position S.");
                    return -1;
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_DELETED):
            {
                replay_entry.state.info.active = false;
                break;
            }
            case static_cast<id_t>(Dat::PacketId::END_OF_SCENARIO):
            {
                float stop_time;
                if (ReadPacket(header, stop_time) != 0)
                {
                    LOG_ERROR("Failed reading end of scenario timestamp.");
                    return -1;
                }
                // We have an unsaved entry, so we add it to the map
                if (replay_entry.state.info.timeStamp >= stop_time)
                {
                    obj_events_map_[replay_entry.state.info.id].push_back(replay_entry);
                }
                stopTime_ = static_cast<double>(stop_time);
                break;
            }
            default:
            {
                LOG_ERROR("Unknown packet id: {}", header.id);
                file_.seekg(header.data_size, std::ios::cur);
                return -1;
            }
        }
        previous_p_id_ = header.id;
    }

    file_.close();

    return 0;
}

void Replay::BuildDataFromPackets()
{
    object_state_cache_.clear();

    // Find start time, maybe its earlier than 0.0s
    for (const auto& [id, entry] : obj_events_map_)
    {
        startTime_  = static_cast<double>(std::min(static_cast<float>(startTime_), entry.front().state.info.timeStamp));
        time_       = startTime_;
        startIndex_ = 0;

        IsDataFixedTimestep(&entry);  // Check if the data has a fixed timestep or not
    }

    if (!logged_timestep_fixed_)
    {
        // If the replay has variable timestep, we build data based on the timestamps found in the packets
        BuildDataVariableTimestep();
    }
    else
    {
        // If the replay has a fixed timestep, we can build data directly
        BuildDataFixedTimestep();
    }
}

void Replay::BuildDataFixedTimestep()
{
    // If fixed timestep is specified, use it, otherwise use the minimum timestep found in the data
    float dt = min_timestep_.value_or(0.01f);  // Default to 10ms if no minimum timestep is found
    if (fixed_timestep_ > 0.0f)
    {
        dt = fixed_timestep_;
    }

    for (const auto& id : object_ids_)
    {
        id_to_search_idx_[id] = 0;  // Initialize search index for each object ID
    }

    float ghost_dt = 0.05f;
    for (float t = static_cast<float>(startTime_); t <= static_cast<float>(stopTime_) + 0.001f;)
    {
        for (const int obj_id : object_ids_)
        {
            const auto& events = obj_events_map_[obj_id];

            auto& last_state = object_state_cache_[obj_id];  // Its ok to be empty, all values will be filled first time object appears

            if (!events.empty())
            {
                for (size_t i = id_to_search_idx_[obj_id]; i < events.size(); i++)
                {
                    if (events[i].state.info.timeStamp <= static_cast<float>(t) + 0.001f)
                    {
                        last_state                = events[i];  // Update the last state to the most recent event before or at time t
                        id_to_search_idx_[obj_id] = i;          // Store the index of the last event for this object
                    }
                    else
                    {
                        break;  // No need to check further, as events are sorted by timestamp
                    }
                }
            }

            ReplayEntry entry = last_state;
            if (entry.state.info.active)
            {
                entry.state.info.timeStamp = static_cast<float>(t);
                data_.push_back(entry);
            }
        }
        if (t < 0.0f - SMALL_NUMBERF)  // If t is negative, we use ghost_dt to avoid going backwards in time
        {
            t += ghost_dt;
        }
        else
        {
            t += dt;
        }
    }
}

void Replay::BuildDataVariableTimestep()
{
    std::set<float> all_timestamps;  // Sorted, unique timestamps

    // Step 1: Collect all timestamps
    for (const auto& [obj_id, events] : obj_events_map_)
    {
        for (const auto& entry : events)
        {
            all_timestamps.insert(entry.state.info.timeStamp);
        }
    }

    // Step 2: Process each timestamp
    for (float t : all_timestamps)
    {
        for (int obj_id : object_ids_)
        {
            const auto& events     = obj_events_map_[obj_id];
            auto&       last_state = object_state_cache_[obj_id];  // OK to be empty initially

            // Update last known state if there's an event at or before time `t`
            for (size_t i = id_to_search_idx_[obj_id]; i < events.size(); ++i)
            {
                if (events[i].state.info.timeStamp <= t + SMALL_NUMBERF)
                {
                    last_state                = events[i];
                    id_to_search_idx_[obj_id] = i;
                }
                else
                {
                    break;  // Since events are sorted
                }
            }

            // Emit entry
            if (last_state.state.info.active)
            {
                ReplayEntry entry          = last_state;
                entry.state.info.timeStamp = t;  // Set current time explicitly
                data_.push_back(entry);
            }
        }
    }
}

void Replay::IsDataFixedTimestep(const std::vector<scenarioengine::ReplayEntry>* entry)
{
    // Code below to deduce if the replay has a fixed timestep or not...
    float dt;
    for (size_t i = 0; i < entry->size() - 1; i++)
    {
        float timestamp = entry->at(i).state.info.timeStamp;
        if (timestamp < 0.0f)
        {
            continue;
        }
        float next_timestamp = entry->at(i + 1).state.info.timeStamp;
        float current_dt     = next_timestamp - timestamp;
        if (i == 0)
        {
            dt = current_dt;
        }
        else
        {
            // If the delta differs by more than a small tolerance, it's not fixed
            if (std::abs(current_dt - dt) > 0.001f)
            {
                logged_timestep_fixed_ = false;
                break;  // No need to check further
            }
        }
    }
}

int Replay::FillHeader()
{
    Dat::DatHeader d_header;

    if (!file_.read(reinterpret_cast<char*>(&header_.version_major), sizeof(header_.version_major)) ||
        !file_.read(reinterpret_cast<char*>(&header_.version_minor), sizeof(header_.version_minor)))
    {
        LOG_ERROR("Failed reading header versions.");
        return -1;
    }

    if (ReadStringPacket(header_.odr_filename.string) != 0)
    {
        LOG_ERROR("Failed reading odr filename.");
        return -1;
    }

    if (ReadStringPacket(header_.model_filename.string) != 0)
    {
        LOG_ERROR("Failed reading model filename.");
        return -1;
    }

    return 0;
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
    if (scenarios.empty() || scenarios[0].second.empty())
    {
        LOG_ERROR("BuildData: No scenario data to process.");
        return;
    }

    data_.clear();  // Rebuilding data_ from scratch

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
    Dat::DatLogger dat_logger;
    dat_logger.Init(filename, header_.odr_filename.string, header_.model_filename.string);

    if (!dat_logger.IsWriteFileOpen())
    {
        LOG_ERROR("Failed to open dat file for writing: {}", filename);
        return;
    }

    std::vector<std::unique_ptr<scenarioengine::ObjectState>> object_states;
    for (size_t i = 0; i < data_.size() - 1; i++)
    {
        auto state = &data_[i].state;
        auto obj   = std::make_unique<scenarioengine::ObjectState>(state->info.id,
                                                                 state->info.name,
                                                                 state->info.obj_type,
                                                                 state->info.obj_category,
                                                                 0,  // No role
                                                                 state->info.model_id,
                                                                 state->info.ctrl_type,
                                                                 state->info.boundingbox,
                                                                 state->info.scaleMode,
                                                                 state->info.visibilityMask,
                                                                 static_cast<double>(state->info.timeStamp),
                                                                 static_cast<double>(state->info.speed),
                                                                 static_cast<double>(state->info.wheel_angle),
                                                                 static_cast<double>(state->info.wheel_rot),
                                                                 0.0,  // No rear axle z pos
                                                                 static_cast<double>(state->pos.x),
                                                                 static_cast<double>(state->pos.y),
                                                                 static_cast<double>(state->pos.z),
                                                                 static_cast<double>(state->pos.h),
                                                                 static_cast<double>(state->pos.p),
                                                                 static_cast<double>(state->pos.r));

        obj->state_.info.wheel_data.emplace_back();  // Initialize wheel_data vector
        obj->state_.info.wheel_data[0].h = static_cast<double>(state->info.wheel_angle);
        obj->state_.info.wheel_data[0].p = static_cast<double>(state->info.wheel_rot);
        obj->state_.pos.SetTrackId(state->pos.roadId);
        obj->state_.pos.SetLaneId(state->pos.laneId);
        obj->state_.pos.SetOffset(static_cast<double>(state->pos.offset));
        obj->state_.pos.SetT(static_cast<double>(state->pos.t));
        obj->state_.pos.SetS(static_cast<double>(state->pos.s));

        object_states.emplace_back(std::move(obj));

        // If the next timestamp is a new entry, we need to send it to PacketHandler for dat creation
        if (data_[i + 1].state.info.timeStamp != data_[i].state.info.timeStamp)
        {
            dat_logger.SetSimulationTime(data_[i].state.info.timeStamp);
            dat_logger.WriteToDat(object_states);

            object_states.clear();  // Clear the states for the next timestamp
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

void Replay::ClearData()
{
    data_.clear();
    obj_events_map_.clear();
    object_state_cache_.clear();
    object_ids_.clear();
    id_to_name_.clear();
    id_to_search_idx_.clear();
    timestamp_ = 0.0;
    min_timestep_.reset();
}