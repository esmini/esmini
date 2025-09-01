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

Replay::Replay(std::string filename) : time_(0.0), index_(0), repeat_(false)
{
    // Parse the packets from the file
    int ret = ParsePackets(filename);
    if (ret != 0)
    {
        LOG_ERROR("Failed to parse packets from file: {}", filename);
        return;
    }

    FillInTimestamps();  // Create timestamps from dt

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
    std::vector<std::pair<std::string, std::map<int, PropertyTimeline, MapComparator>>> scenarioData;
    std::vector<std::vector<std::pair<double, bool>>>                                   timestamps;

    if (scenarios_.size() < 2)
    {
        LOG_ERROR_AND_QUIT("Too few scenarios loaded, use single replay feature instead\n");
    }

    double temp_fixed_timestep = LARGE_NUMBER;
    for (size_t i = 0; i < scenarios_.size(); i++)
    {
        ParsePackets(scenarios_[i]);
        timestamps.push_back(timestamps_);
        scenarioData.emplace_back(scenarios_[i], objects_timeline_);
        if (fixed_timestep_ < temp_fixed_timestep)
        {
            temp_fixed_timestep = fixed_timestep_;
        }

        objects_timeline_.clear();
        timestamps_.clear();
    }

    // Build the objects timeline data structure
    // Log which scenario belongs to what ID-group (0, 100, 200 etc.)
    for (size_t i = 0; i < scenarioData.size(); i++)
    {
        std::string scenario_tmp = scenarioData[i].first;
        LOG_INFO("Scenarios corresponding to IDs ({}:{}): {}", i * 100, (i + 1) * 100 - 1, FileNameOf(scenario_tmp));
        for (auto& [id, timeline] : scenarioData[i].second)
        {
            int new_id = id + static_cast<int>(i * 100);
            objects_timeline_.emplace(new_id, std::move(timeline));
        }
    }
    scenarioData.clear();

    // Build the final timestamps_ vector
    size_t total_size = std::accumulate(timestamps.begin(), timestamps.end(), size_t{0}, [](size_t sum, const auto& v) { return sum + v.size(); });
    timestamps_.reserve(total_size);

    for (const auto& v : timestamps)
    {
        timestamps_.insert(timestamps_.end(), v.begin(), v.end());
    }

    std::sort(timestamps_.begin(), timestamps_.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    fixed_timestep_ = temp_fixed_timestep;
    startTime_      = static_cast<float>(timestamps_[0].first);
    time_           = startTime_;

    if (!create_datfile_.empty())
    {
        LOG_INFO("Creating merged dat file: {}", create_datfile_);
        CreateMergedDatfile(create_datfile_);
    }
}

int Replay::ParsePackets(const std::string& filename)
{
    // Get the file size
    auto dat_reader = Dat::DatReader(filename);

    // Read raw header BEFORE reading packets
    if (dat_reader.FillDatHeader() != 0)
    {
        LOG_ERROR_AND_QUIT("Failed to read DAT header.");
    }

    dat_header_ = dat_reader.GetDatHeader();
    LOG_INFO("Datfile: version {}.{}, odr_filename: {}, model_filename: {}",
             dat_header_.version_major,
             dat_header_.version_minor,
             dat_header_.odr_filename.string,
             dat_header_.model_filename.string);

    // Now parse packets
    Dat::PacketHeader header;
    while (dat_reader.ReadFile(header))
    {
        switch (header.id)
        {
            case static_cast<id_t>(Dat::PacketId::TIMESTAMP):
            {
                if (dat_reader.ReadPacket(header, timestamp_) != 0)
                {
                    LOG_ERROR("Failed reading timestamp data.");
                }

                if (!first_timestamp_.has_value())
                {
                    first_timestamp_ = timestamp_;
                }

                if (timestamps_.empty() || timestamp_ <= SMALL_NUMBER || timestamp_ > timestamps_.back().first)
                {
                    if (previous_packet_id_ != header.id)
                    {
                        timestamps_.emplace_back(timestamp_, true);
                    }
                    else
                    {
                        timestamps_.emplace_back(timestamp_, false);
                    }
                }
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_ID):
            {
                if (dat_reader.ReadPacket(header, current_object_id_) != 0)
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
                    if (timestamp_ > 0.0)
                    {
                        current_object_timeline_->active_.values.emplace_back(0.0f, false);  // Object was inactive from start of simulation
                        current_object_timeline_->active_.values.emplace_back(timestamp_, true);
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
                if (dat_reader.ReadPacket(header, speed) != 0)
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
                if (dat_reader.ReadPacket(header, pose.x, pose.y, pose.z, pose.h, pose.p, pose.r) != 0)
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
                if (dat_reader.ReadPacket(header, model_id) != 0)
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
                if (dat_reader.ReadPacket(header, obj_type) != 0)
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
                if (dat_reader.ReadPacket(header, obj_category) != 0)
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
                if (dat_reader.ReadPacket(header, ctrl_type) != 0)
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
                if (dat_reader.ReadPacket(header, wheel_angle) != 0)
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
                if (dat_reader.ReadPacket(header, wheel_rot) != 0)
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
                if (dat_reader.ReadPacket(header,
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
                if (dat_reader.ReadPacket(header, scale_mode) != 0)
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
                if (dat_reader.ReadPacket(header, visibility_mask) != 0)
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
                if (dat_reader.ReadStringPacket(name) != 0)
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
                if (dat_reader.ReadPacket(header, road_id) != 0)
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
                if (dat_reader.ReadPacket(header, lane_id) != 0)
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
                if (dat_reader.ReadPacket(header, offset) != 0)
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
                if (dat_reader.ReadPacket(header, t) != 0)
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
                if (dat_reader.ReadPacket(header, s) != 0)
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
                double dt;
                if (dat_reader.ReadPacket(header, dt) != 0)
                {
                    LOG_ERROR("Failed reading fixed timestep.");
                    return -1;
                }
                dt_.values.emplace_back(timestamp_, dt);

                // Receiving a new DT is not significant,
                // but we dont know we'll get dt after adding the timestamp
                timestamps_.back().second = false;
                break;
            }
            case static_cast<id_t>(Dat::PacketId::END_OF_SCENARIO):
            {
                double stop_time;
                if (dat_reader.ReadPacket(header, stop_time) != 0)
                {
                    LOG_ERROR("Failed reading end of scenario timestamp.");
                    return -1;
                }
                stopTime_ = stop_time;
                if (!NEAR_NUMBERS(stopTime_, timestamps_.back().first))
                {
                    timestamps_.emplace_back(stopTime_, true);
                }
                else
                {
                    timestamps_.back().second = true;
                }
                // stopIndex_ = timestamps_.size() - 1;
                break;
            }
            default:
            {
                dat_reader.UnknownPacket(header);
                return -1;
            }
        }
        previous_packet_id_ = header.id;
    }

    dat_reader.CloseFile();

    return 0;
}

void Replay::FillInTimestamps()
{
    std::vector<std::pair<double, bool>> filled;

    size_t i         = 0;
    double curr_time = first_timestamp_.value();

    while (i < timestamps_.size() - 1 && curr_time < static_cast<double>(stopTime_) - SMALL_NUMBER)
    {
        filled.emplace_back(curr_time, timestamps_[i].second);  // Save current timestamp

        double next_logged_time = timestamps_[i + 1].first;

        double dt        = dt_.get_value_binary(curr_time, true);  // Get next delta time
        double next_time = curr_time + dt;

        // Case 1: next_time is very close to next_logged_time -> snap
        if (NEAR_NUMBERS(next_time, next_logged_time))
        {
            i++;
            curr_time = next_logged_time;
        }
        // Case 2: next_time goes beyond the next logged timestamp
        else if (next_time > next_logged_time)
        {
            LOG_WARN("Some mismatch in timestamps, overshooting at {}", curr_time);
        }
        // Case 3: Inside a gap bigger than 1 sample -> We need to fill from last known dt
        else
        {
            double prev_dt = dt_.get_value_binary(curr_time);
            curr_time += prev_dt;
        }
    }

    // Always add the last explicit one
    filled.emplace_back(timestamps_.back().first, timestamps_.back().second);
    filled.begin()->second = true;  // Start of scenario is significant

    timestamps_.swap(filled);
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
    if (fixed_timestep_ == 1.0)
    {
        time_ = std::floor(time_);
    }
    else if (fixed_timestep_ > 0.0)
    {
        auto divisor = std::round(time_ / fixed_timestep_);
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
            time_  = target_time;
        }
    }
    else
    {
        if (target_time > time_)  // Looking ahead, stopping as soon as we find time greater than current time
        {
            size_t next_index = index_ + 1;
            // Subtract small number so we don't accidentally step ahead of intended time
            if (next_index < timestamps_.size() && target_time >= timestamps_[next_index].first - SMALL_NUMBER)
            {
                index_ = next_index;
                time_  = timestamps_[index_].first;
            }
            else
            {
                if (target_time > stopTime_)
                {
                    GoToEnd();
                }
                else
                {
                    time_ = target_time;
                }
            }
        }
        else if (target_time < time_)  // Same for backwards, but we stop when we find a timestamp less than current time
        {
            if (index_ > 0 && target_time <= timestamps_[index_ - 1].first)
            {
                index_--;
                time_ = timestamps_[index_].first;
            }
            else
            {
                if (target_time < GetStartTime())
                {
                    GoToStart();
                }
                else
                {
                    time_ = static_cast<float>(target_time);
                }
            }
        }
    }
    if (time_ > GetStopTime())
    {
        GoToEnd();
    }
    else if (time_ < GetStartTime())
    {
        GoToStart();
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

    auto it = std::upper_bound(timestamps_.begin(),
                               timestamps_.end(),
                               time_ + SMALL_NUMBER,
                               [](double value, const std::pair<double, bool>& p) { return value < p.first; });

    if (it == timestamps_.end())
    {
        GoToEnd();
        return 0;
    }
    if (it->first <= stopTime_ + SMALL_NUMBER)
    {
        index_ = static_cast<size_t>(std::distance(timestamps_.begin(), it));
        time_  = it->first;
        return 0;
    }
    else
    {
        GoToEnd();
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
        if (time_ < startTime_)
        {
            GoToStart();
        }
    }
}

size_t Replay::FindIndexAtTimestamp(double timestamp)
{
    auto start = timestamps_.begin();
    auto end   = timestamps_.end();
    if (timestamp >= time_ - SMALL_NUMBER)  // We look ahead, can start to search from current index_
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

void Replay::GoToSignificantTimestamp(bool search_forward)
{
    if (search_forward)
    {
        size_t i;
        for (i = index_ + 1; i < timestamps_.size(); i++)
        {
            if (timestamps_[i].second)
            {
                index_ = i;
                time_  = timestamps_[index_].first;
                break;
            }
        }
        if (i == timestamps_.size())
        {
            index_ = timestamps_.size() - 1;
            time_  = timestamps_[index_].first;
        }
        if (time_ > stopTime_)
        {
            GoToEnd();
            return;
        }
    }
    else  // Search backward
    {
        size_t i;
        for (i = index_ - 1; i > 0; i--)
        {
            if (timestamps_[i].second)
            {
                index_ = i;
                time_  = timestamps_[index_].first;
                break;
            }
        }
        if (i == 0)
        {
            index_ = 0;
            time_  = timestamps_[index_].first;
        }
        if (time_ < startTime_)
        {
            GoToStart();
            return;
        }
    }
}

ReplayEntry Replay::GetReplayEntryAtTimeIncremental(int id, double t) const
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

ReplayEntry Replay::GetReplayEntryAtTimeBinary(int id, double t) const
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
                    it->second.active_.values.emplace_back(timestamp_, true);  // timestamp_ contains the new rewinded time
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

void Replay::CreateMergedDatfile(const std::string filename) const
{
    Dat::DatWriter dat_writer;
    dat_writer.Init(filename, dat_header_.odr_filename.string, dat_header_.model_filename.string);

    if (!dat_writer.IsWriteFileOpen())
    {
        LOG_ERROR("Failed to open dat file for writing: {}", filename);
        return;
    }

    // We re-create the object states vector which then is written to the DAT file
    std::vector<std::unique_ptr<scenarioengine::ObjectState>> object_states;
    for (size_t i = 0; i < timestamps_.size() - 1; i++)
    {
        for (const auto& [id, _] : objects_timeline_)
        {
            ReplayEntry entry = GetReplayEntryAtTimeIncremental(id, timestamps_[i].first);
            auto        state = &entry.state;

            if (!state->info.active)  // Ignore entities which are inactive in the current time
            {
                continue;
            }

            auto obj = std::make_unique<scenarioengine::ObjectState>(state->info.id,
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
        }

        // Same flow as in ScenarioGateway.cpp
        // dat_writer.SetSimulationTime(timestamps_[i].first, dt);
        // dat_writer.WriteGenericDataToDat();  // Writes the fixed timestep
        dat_writer.WriteObjectStatesToDat(object_states);

        object_states.clear();  // Clear the states for the next timestamp
    }
}
