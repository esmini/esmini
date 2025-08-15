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
                if (ReadPacket(header, timestamp_) != 0)
                {
                    LOG_ERROR("Failed reading timestamp data.");
                }
                timestamps_.push_back(timestamp_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_ID):
            {
                if (ReadPacket(header, current_object_id_) != 0)
                {
                    LOG_ERROR("Failed reading object ID.");
                    return -1;
                }

                if (objects_timeline_.count(current_object_id_) == 0)
                {
                    // Initialize timelines for this object
                    objects_timeline_[current_object_id_] = {};
                    current_object_timeline_              = &objects_timeline_[current_object_id_];
                    current_object_timeline_->odometer_.values.emplace_back(timestamp_, 0.0f);
                }
                else
                {
                    current_object_timeline_ = &objects_timeline_[current_object_id_];
                }
                current_object_timeline_->active_.values.emplace_back(timestamp_, true);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::SPEED):
            {
                float speed;
                if (ReadPacket(header, speed) != 0)
                {
                    LOG_ERROR("Failed reading speed data.");
                    return -1;
                }

                current_object_timeline_->speed_.values.emplace_back(timestamp_, speed);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POSE):
            {
                Dat::Pose pose;
                if (ReadPacket(header, pose.x, pose.y, pose.z, pose.h, pose.p, pose.r) != 0)
                {
                    LOG_ERROR("Failed reading pose data.");
                    return -1;
                }

                current_object_timeline_->pose_.values.emplace_back(timestamp_, pose);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::MODEL_ID):
            {
                int model_id;
                if (ReadPacket(header, model_id) != 0)
                {
                    LOG_ERROR("Failed reading model ID.");
                    return -1;
                }
                current_object_timeline_->model_id_.values.emplace_back(timestamp_, model_id);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_TYPE):
            {
                int obj_type;
                if (ReadPacket(header, obj_type) != 0)
                {
                    LOG_ERROR("Failed reading object type.");
                    return -1;
                }
                current_object_timeline_->obj_type_.values.emplace_back(timestamp_, obj_type);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_CATEGORY):
            {
                int obj_category;
                if (ReadPacket(header, obj_category) != 0)
                {
                    LOG_ERROR("Failed reading object category.");
                    return -1;
                }
                current_object_timeline_->obj_category_.values.emplace_back(timestamp_, obj_category);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::CTRL_TYPE):
            {
                int ctrl_type;
                if (ReadPacket(header, ctrl_type) != 0)
                {
                    LOG_ERROR("Failed reading controller type.");
                    return -1;
                }
                current_object_timeline_->ctrl_type_.values.emplace_back(timestamp_, ctrl_type);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::WHEEL_ANGLE):
            {
                float wheel_angle;
                if (ReadPacket(header, wheel_angle) != 0)
                {
                    LOG_ERROR("Failed reading wheel angle.");
                    return -1;
                }
                current_object_timeline_->wheel_angle_.values.emplace_back(timestamp_, wheel_angle);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::WHEEL_ROT):
            {
                float wheel_rot;
                if (ReadPacket(header, wheel_rot) != 0)
                {
                    LOG_ERROR("Failed reading wheel rotation.");
                    return -1;
                }
                current_object_timeline_->wheel_rot_.values.emplace_back(timestamp_, wheel_rot);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::BOUNDING_BOX):
            {
                OSCBoundingBox bounding_box;
                if (ReadPacket(header,
                               bounding_box.center_.x_,
                               bounding_box.center_.y_,
                               bounding_box.center_.z_,
                               bounding_box.dimensions_.length_,
                               bounding_box.dimensions_.width_,
                               bounding_box.dimensions_.height_) != 0)
                {
                    LOG_ERROR("Failed reading bounding box data.");
                    return -1;
                }
                current_object_timeline_->bounding_box_.values.emplace_back(timestamp_, bounding_box);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::SCALE_MODE):
            {
                int scale_mode;
                if (ReadPacket(header, scale_mode) != 0)
                {
                    LOG_ERROR("Failed reading scale mode.");
                    return -1;
                }
                current_object_timeline_->scale_mode_.values.emplace_back(timestamp_, scale_mode);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::VISIBILITY_MASK):
            {
                int visibility_mask;
                if (ReadPacket(header, visibility_mask) != 0)
                {
                    LOG_ERROR("Failed reading visibility mask.");
                    return -1;
                }
                current_object_timeline_->visibility_mask_.values.emplace_back(timestamp_, visibility_mask);
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
                current_object_timeline_->name_.values.emplace_back(timestamp_, std::move(name));
                break;
            }
            case static_cast<id_t>(Dat::PacketId::ROAD_ID):
            {
                id_t road_id;
                if (ReadPacket(header, road_id) != 0)
                {
                    LOG_ERROR("Failed reading road ID.");
                    return -1;
                }
                current_object_timeline_->road_id_.values.emplace_back(timestamp_, road_id);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::LANE_ID):
            {
                int lane_id;
                if (ReadPacket(header, lane_id) != 0)
                {
                    LOG_ERROR("Failed reading lane ID.");
                    return -1;
                }
                current_object_timeline_->lane_id_.values.emplace_back(timestamp_, lane_id);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_OFFSET):
            {
                float offset;
                if (ReadPacket(header, offset) != 0)
                {
                    LOG_ERROR("Failed reading position offset.");
                    return -1;
                }
                current_object_timeline_->pos_offset_.values.emplace_back(timestamp_, offset);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_T):
            {
                float t;
                if (ReadPacket(header, t) != 0)
                {
                    LOG_ERROR("Failed reading position T.");
                    return -1;
                }
                current_object_timeline_->pos_t_.values.emplace_back(timestamp_, t);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_S):
            {
                float s;
                if (ReadPacket(header, s) != 0)
                {
                    LOG_ERROR("Failed reading position S.");
                    return -1;
                }
                current_object_timeline_->pos_s_.values.emplace_back(timestamp_, s);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_DELETED):
            {
                current_object_timeline_->active_.values.emplace_back(timestamp_, false);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::DT):
            {
                float dt;
                if (ReadPacket(header, dt) != 0)
                {
                    LOG_ERROR("Failed reading fixed timestep.");
                    return -1;
                }
                dt_timeline_.values.emplace_back(timestamp_, dt);
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
                // if (replay_entry.state.info.timeStamp >= stop_time)
                // {
                //     obj_events_map_[replay_entry.state.info.id].push_back(replay_entry);
                // }
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

        if (fixed_timestep_ == 0.0f)  // We dont have fixed timestep from commandline
        {
            IsDataFixedTimestep(&entry);  // Check if the data has a fixed timestep or not
        }
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
            time_ = time;
        }
    }
    else
    {
        /* TODO */
    }
}

void Replay::GoToDeltaTime(double dt, bool stop_at_next_frame)
{
    GoToTime(time_ + dt, stop_at_next_frame);
}

int Replay::GoToNextFrame()
{
    if (timestamps_.empty())
    {
        return -1;
    }

    auto it = std::upper_bound(timestamps_.begin(), timestamps_.end(), time_);

    if (it != timestamps_.end())
    {
        time_ = *it;
        return 0;
    }

    return -1;
}

void Replay::GoToPreviousFrame()
{
    if (timestamps_.empty())
    {
        return;
    }

    auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), time_);

    if (it != timestamps_.begin())
    {
        --it;  // Move to the previous timestamp
        time_ = *it;
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