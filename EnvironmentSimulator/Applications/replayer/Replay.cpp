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

Replay::Replay(std::string filename, bool clean) : time_(0.0), index_(0), repeat_(false), clean_(clean)
{
    // Parse the packets from the file
    int ret = ParsePackets(filename);
    if (ret != 0)
    {
        LOG_ERROR("Failed to parse packets from file: {}", filename);
        return;
    }

    if (timestamps_.empty())
    {
        LOG_ERROR("No timestamps found in file: {}", filename);
        return;
    }

    startTime_ = static_cast<float>(timestamps_[0].first);
    time_      = startTime_;
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

    if (!create_datfile_.empty())
    {
        CreateMergedDatfile(create_datfile_);
    }
}

int Replay::ParsePackets(const std::string& filename)
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

                if (timestamps_.empty() || timestamp_ <= SMALL_NUMBERF || timestamp_ > timestamps_.back().first)
                {
                    bool significant = (header.id != previous_packet_id_);
                    timestamps_.emplace_back(timestamp_, significant);
                }
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
                    if (timestamp_ > 0.0f)
                    {
                        current_object_timeline_->active_.values.emplace_back(0.0f, false);  // Object was inactive from start of simulation
                    }
                    else
                    {
                        current_object_timeline_->active_.values.emplace_back(timestamp_, true);  // Object is active at the start of simulation
                    }
                }
                else
                {
                    current_object_timeline_ = &objects_timeline_[current_object_id_];
                    if (current_object_timeline_->active_.values.back().second != true)
                    {
                        current_object_timeline_->active_.values.emplace_back(timestamp_, true);
                    }
                }
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
                AddToTimeline(current_object_timeline_->speed_, speed);
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

                AddToTimeline(current_object_timeline_->pose_, pose);
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
                AddToTimeline(current_object_timeline_->wheel_angle_, wheel_angle);
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
                AddToTimeline(current_object_timeline_->wheel_rot_, wheel_rot);
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
                AddToTimeline(current_object_timeline_->visibility_mask_, visibility_mask);
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
                AddToTimeline(current_object_timeline_->road_id_, road_id);
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
                AddToTimeline(current_object_timeline_->lane_id_, lane_id);
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
                AddToTimeline(current_object_timeline_->pos_offset_, offset);
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
                AddToTimeline(current_object_timeline_->pos_t_, t);
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
                AddToTimeline(current_object_timeline_->pos_s_, s);
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
                fixed_timestep_ = dt;
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
                stopTime_  = static_cast<double>(stop_time);
                stopIndex_ = timestamps_.size() - 1;
                break;
            }
            default:
            {
                LOG_ERROR("Unknown packet id: {}", header.id);
                file_.seekg(header.data_size, std::ios::cur);
                return -1;
            }
        }
        previous_packet_id_ = header.id;
    }

    file_.close();

    return 0;
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
    objects_timeline_.clear();
}

void Replay::GoToStart(bool ignore_repeat)
{
    if (repeat_ && !ignore_repeat)
    {
        index_ = stopIndex_;
        time_  = stopTime_;
    }
    else
    {
        index_ = startIndex_;
        time_  = startTime_;
    }
}

void Replay::GoToEnd(bool ignore_repeat)
{
    if (repeat_ && !ignore_repeat)
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

void Replay::RoundTime()
{
    if (fixed_timestep_ == 1.0f)
    {
        time_ = std::floor(time_);
    }
    else if (fixed_timestep_ > 0.0f)
    {
        auto divisor = std::round((static_cast<float>(time_) / fixed_timestep_));
        time_        = divisor * fixed_timestep_;
    }
}

void Replay::GoToTime(double target_time, bool stop_at_next_frame)
{
    // We dont stop searching until we found the closes timestamp
    if (!stop_at_next_frame)
    {
        if (target_time >= stopTime_)
        {
            GoToEnd();
            return;
        }
        else if (target_time <= GetStartTime())
        {
            GoToStart();
            return;
        }
        else
        {
            index_ = FindIndexAtTimestamp(target_time);
            time_  = static_cast<float>(target_time);
        }
    }
    else
    {
        if (target_time > stopTime_)
        {
            GoToEnd();
            return;
        }
        if (target_time < GetStartTime())
        {
            GoToStart();
            return;
        }
        if (target_time > time_)  // Looking ahead, stopping as soon as we find time greater than current time
        {
            size_t next_index = index_ + 1;
            // Subtract small number so we don't accidentally step ahead of intended time
            if (next_index < timestamps_.size() && static_cast<float>(target_time) >= timestamps_[next_index].first - SMALL_NUMBERF)
            {
                index_ = next_index;
                time_  = static_cast<float>(target_time);
            }
            else
            {
                time_ = static_cast<float>(target_time);
            }
        }
        else if (target_time < time_)  // Same for backwards, but we stop when we find a timestamp less than current time
        {
            if (index_ > 0 && static_cast<float>(target_time) <= timestamps_[index_ - 1].first)
            {
                index_--;
                time_ = timestamps_[index_].first;
            }
            else
            {
                time_ = static_cast<float>(target_time);
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
    if (timestamps_.empty())
    {
        return -1;
    }

    auto it =
        std::upper_bound(timestamps_.begin(), timestamps_.end(), time_, [](float value, const std::pair<float, bool>& p) { return value < p.first; });

    if (it == timestamps_.end())
    {
        GoToEnd();
        return 0;
    }
    if (it->first <= static_cast<float>(stopTime_) + SMALL_NUMBERF)
    {
        index_ = static_cast<size_t>(std::distance(timestamps_.begin(), it));
        time_  = it->first;
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

    auto it =
        std::lower_bound(timestamps_.begin(), timestamps_.end(), time_, [](const std::pair<float, bool>& p, float value) { return p.first < value; });

    if (it == timestamps_.begin())
    {
        GoToStart();
        return;
    }
    else
    {
        --it;  // Move to the previous timestamp
        index_ = static_cast<size_t>(std::distance(timestamps_.begin(), it));
        time_  = it->first;
    }
}

size_t Replay::FindIndexAtTimestamp(double timestamp)
{
    auto start = timestamps_.begin();
    auto end   = timestamps_.end();
    if (timestamp > time_)  // We look ahead, can start to search from current index_
    {
        start += index_;
    }
    else  // else we start from beginning and stop at index + 1 (lower_bound excludes last, so we add +1)
    {
        end = timestamps_.begin() + index_ + 1;
    }
    // Subtract small number so we don't accidentally find index ahead of intended time
    auto it = std::lower_bound(start,
                               end,
                               static_cast<float>(timestamp) - SMALL_NUMBERF,
                               [](const std::pair<float, bool>& p, float value) { return p.first < value; });
    return static_cast<size_t>(std::distance(timestamps_.begin(), it));
}

ObjectStateStructDat* Replay::GetState(int id)
{
    ReplayEntry* entry = &object_state_cache_[id];
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
    index_      = startIndex_;
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

void Replay::GoToSignificantTimestamp(bool search_forward)
{
    if (search_forward)
    {
        auto it = significant_event_start_indices_.begin();
        for (; it != significant_event_start_indices_.end(); ++it)
        {
            if (*it > index_)
            {
                index_ = *it;
                time_  = timestamps_[index_].first;
                return;
            }
        }
        if (it == significant_event_start_indices_.end())
        {
            index_ = timestamps_.size() - 1;
            time_  = timestamps_[index_].first;
        }
    }
    else  // Search backward
    {
        auto it = significant_event_start_indices_.rbegin();
        for (; it != significant_event_start_indices_.rend(); ++it)
        {
            if (*it < index_)
            {
                index_ = *it;
                time_  = timestamps_[index_].first;
                return;
            }
        }
        if (it == significant_event_start_indices_.rend())
        {
            index_ = 0;
            time_  = timestamps_[index_].first;
        }
    }
}

ReplayEntry Replay::GetReplayEntryAtTimeIncremental(int id, float t) const
{
    ReplayEntry entry;

    const auto& timeline = objects_timeline_.at(id);

    entry.state.info.id             = id;
    entry.state.info.timeStamp      = t;
    entry.state.info.model_id       = timeline.model_id_.get_value_incremental(t);
    entry.state.info.obj_type       = timeline.obj_type_.get_value_incremental(t);
    entry.state.info.obj_category   = timeline.obj_category_.get_value_incremental(t);
    entry.state.info.ctrl_type      = timeline.ctrl_type_.get_value_incremental(t);
    entry.state.info.name           = timeline.name_.get_value_incremental(t).c_str();
    entry.state.info.speed          = timeline.speed_.get_value_incremental(t);
    entry.state.info.wheel_angle    = timeline.wheel_angle_.get_value_incremental(t);
    entry.state.info.wheel_rot      = timeline.wheel_rot_.get_value_incremental(t);
    entry.state.info.boundingbox    = timeline.bounding_box_.get_value_incremental(t);
    entry.state.info.scaleMode      = timeline.scale_mode_.get_value_incremental(t);
    entry.state.info.visibilityMask = timeline.visibility_mask_.get_value_incremental(t);
    entry.state.info.active         = timeline.active_.get_value_incremental(t);
    entry.state.pos.x               = timeline.pose_.get_value_incremental(t).x;
    entry.state.pos.y               = timeline.pose_.get_value_incremental(t).y;
    entry.state.pos.z               = timeline.pose_.get_value_incremental(t).z;
    entry.state.pos.h               = timeline.pose_.get_value_incremental(t).h;
    entry.state.pos.p               = timeline.pose_.get_value_incremental(t).p;
    entry.state.pos.r               = timeline.pose_.get_value_incremental(t).r;
    entry.state.pos.roadId          = timeline.road_id_.get_value_incremental(t);
    entry.state.pos.laneId          = timeline.lane_id_.get_value_incremental(t);
    entry.state.pos.offset          = timeline.pos_offset_.get_value_incremental(t);
    entry.state.pos.t               = timeline.pos_t_.get_value_incremental(t);
    entry.state.pos.s               = timeline.pos_s_.get_value_incremental(t);
    entry.odometer                  = timeline.odometer_.get_value_incremental(t);

    return entry;
}

ReplayEntry Replay::GetReplayEntryAtTimeBinary(int id, float t) const
{
    ReplayEntry entry;

    const auto& timeline = objects_timeline_.at(id);

    entry.state.info.id             = id;
    entry.state.info.model_id       = timeline.model_id_.get_value_binary(t);
    entry.state.info.obj_type       = timeline.obj_type_.get_value_binary(t);
    entry.state.info.obj_category   = timeline.obj_category_.get_value_binary(t);
    entry.state.info.ctrl_type      = timeline.ctrl_type_.get_value_binary(t);
    entry.state.info.name           = timeline.name_.get_value_binary(t).c_str();
    entry.state.info.speed          = timeline.speed_.get_value_binary(t);
    entry.state.info.wheel_angle    = timeline.wheel_angle_.get_value_binary(t);
    entry.state.info.wheel_rot      = timeline.wheel_rot_.get_value_binary(t);
    entry.state.info.boundingbox    = timeline.bounding_box_.get_value_binary(t);
    entry.state.info.scaleMode      = timeline.scale_mode_.get_value_binary(t);
    entry.state.info.visibilityMask = timeline.visibility_mask_.get_value_binary(t);
    entry.state.info.active         = timeline.active_.get_value_binary(t);
    entry.state.pos.x               = timeline.pose_.get_value_binary(t).x;
    entry.state.pos.y               = timeline.pose_.get_value_binary(t).y;
    entry.state.pos.z               = timeline.pose_.get_value_binary(t).z;
    entry.state.pos.h               = timeline.pose_.get_value_binary(t).h;
    entry.state.pos.p               = timeline.pose_.get_value_binary(t).p;
    entry.state.pos.r               = timeline.pose_.get_value_binary(t).r;
    entry.state.pos.roadId          = timeline.road_id_.get_value_binary(t);
    entry.state.pos.laneId          = timeline.lane_id_.get_value_binary(t);
    entry.state.pos.offset          = timeline.pos_offset_.get_value_binary(t);
    entry.state.pos.t               = timeline.pos_t_.get_value_binary(t);
    entry.state.pos.s               = timeline.pos_s_.get_value_binary(t);
    entry.odometer                  = timeline.odometer_.get_value_binary(t);

    return entry;
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

template <typename T, typename Data>
void Replay::AddToTimeline(Timeline<T>& timeline, Data data)
{
    // Check if the current object is ghost, if its not, just add the data
    if (!current_object_timeline_->ctrl_type_.values.empty() && current_object_timeline_->ctrl_type_.values.front().second != 100)
    {
        timeline.values.emplace_back(timestamp_, data);
        return;
    }

    if (!timeline.values.empty() && timestamp_ < timeline.values.back().first)
    {
        // We have detected a ghost restart, thus we:
        // first decrement the ghost_ghost_counter to ensure every ghost ghost gets a unique ID.
        // then copy the current object's timeline to the new ghost object's timeline and sets the last state to inactive
        // finally, we need to clear the current ghost object's timeline down to the time where ghost reset began
        auto& obj_tl = objects_timeline_.at(current_object_id_);

        if (!NEAR_NUMBERSF(obj_tl.last_restart_time, timestamp_))
        {
            int  ghost_ghost_id = current_object_id_ * ghost_ghost_counter_;
            auto it             = objects_timeline_.find(ghost_ghost_id);
            if (it == objects_timeline_.end())
            {
                auto [new_it, inserted] = objects_timeline_.emplace(ghost_ghost_id, objects_timeline_[current_object_id_]);
                it                      = new_it;

                if (inserted)
                {
                    it->second.active_.values.front().second = false;  // Object starts as inactive, as we assume no restart happens at start
                    it->second.name_.values.front().second += "_" + std::to_string(ghost_ghost_counter_);

                    // Ghosts ghost active from ghost restart time until latest timestamp
                    it->second.active_.values.emplace_back(timestamp_, true);
                    it->second.active_.values.emplace_back(timestamps_.back().first, false);

                    ghost_ghost_counter_ -= 1;  // Next ghost will have a new id
                }

                obj_tl.last_restart_time = timestamp_;
            }
        }
        timeline.values.resize(timeline.get_index_binary(timestamp_));
    }
    timeline.values.emplace_back(timestamp_, data);
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

template <typename T>
const T& Timeline<T>::get_value_incremental(float time) const noexcept
{
    if (values.empty())
    {
        LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
    }

    size_t idx        = last_index;
    float  moved_dt   = 0.0f;
    float  desired_dt = time - values[last_index].first;

    if (NEAR_NUMBERSF(last_time, time))
    {
        return values[last_index].second;
    }

    if (time >= values[idx].first)  // Requested time is after last searched value, we increment
    {
        while (idx + 1 < values.size())
        {
            float step = values[idx + 1].first - values[idx].first;
            if (moved_dt + step > desired_dt + SMALL_NUMBERF)
                break;

            moved_dt += step;
            idx++;
        }
    }
    else  // Requested time is before last searched value, we decrement
    {
        while (idx > 0)
        {
            float step = values[idx].first - values[idx - 1].first;
            if (moved_dt + step > -desired_dt - SMALL_NUMBERF)
            {
                idx--;
                break;
            }

            moved_dt += step;
            idx--;
        }
    }

    last_index = idx;  // Save the index for next call
    last_time  = time;

    return values[idx].second;
}

template <typename T>
const T& Timeline<T>::get_value_binary(float time) const noexcept
{
    if (values.empty())
    {
        LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
    }

    if (NEAR_NUMBERSF(last_time, time))
    {
        return values[last_index].second;
    }

    auto search_begin = values.begin();
    auto search_end   = values.end();

    if (time >= values[last_index].first)
    {
        // Time moved forward — only search ahead
        search_begin = values.begin() + static_cast<typename std::vector<std::pair<float, T>>::difference_type>(last_index);
    }
    else
    {
        // Time moved backward — only search behind
        search_end = values.begin() + static_cast<typename std::vector<std::pair<float, T>>::difference_type>(last_index) + 1;
    }

    auto it = std::upper_bound(search_begin, search_end, time, [](float t, const std::pair<float, T>& v) { return t < v.first; });

    if (it == values.begin())
    {
        last_index = 0;
        return it->second;
    }

    --it;
    last_index = static_cast<size_t>(std::distance(values.begin(), it));
    last_time  = time;

    return it->second;
}

template <typename T>
size_t Timeline<T>::get_index_binary(float time) const noexcept
{
    if (values.empty())
    {
        LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
    }

    if (NEAR_NUMBERSF(last_time, time))
    {
        return last_index;
    }

    auto search_begin = values.begin();
    auto search_end   = values.end();

    if (time >= values[last_index].first)
    {
        // Time moved forward — only search ahead
        search_begin = values.begin();
    }
    else
    {
        // Time moved backward — only search behind
        search_end = values.begin() + 1;
    }

    auto it = std::upper_bound(search_begin, search_end, time, [](float t, const std::pair<float, T>& v) { return t < v.first; });

    if (it == values.begin())
    {
        return 0;
    }

    return static_cast<size_t>(std::distance(values.begin(), it));
}
