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

#include <numeric>

#include "Replay.hpp"
#include "ScenarioGateway.hpp"
#include "CommonMini.hpp"
#include "dirent.h"
#include "PacketHandler.hpp"

using namespace scenarioengine;

Replay::Replay(std::string filename, bool quiet) : time_(0.0), quiet_(quiet), index_(0), repeat_(false)
{
    // Parse the packets from the file
    dat_writer_ = std::make_unique<Dat::DatWriter>();
    dat_reader_ = std::make_unique<Dat::DatReader>(filename);
    dat_header_ = ParseDatHeader(filename);

    bool has_ghost_restarts = ExtractPacketsAsSlices();

    if (has_ghost_restarts)
    {
        ExtractGhostRestarts();
    }

    if (timestamps_.empty())
    {
        LOG_ERROR("No timestamps available, quitting");
        return;
    }

    if (dts_.values.empty())
    {
        LOG_ERROR("No dt found in file {}", filename);
        return;
    }

    FillInTimestamps();  // Create filled timestamps with help from dt

    FlattenSlices();

    int ret = ParsePackets();
    if (ret != 0)
    {
        LOG_ERROR("Failed to parse packets from file: {}", filename);
        return;
    }

    if (!eos_received_)
    {
        stopTime_ = timestamps_.back();
    }

    startTime_ = timestamps_[0];
    stopIndex_ = static_cast<unsigned int>(timestamps_.size() - 1);
    // stopTime_ set in END_OF_SCENARIO packet

    time_ = startTime_;
}

Replay::Replay(const std::string directory, const std::string scenario, std::string create_datfile)
    : time_(0.0),
      index_(0),
      repeat_(false),
      create_datfile_(create_datfile)
{
    GetReplaysFromDirectory(directory, scenario);

    if (scenarios_.size() < 2)
    {
        LOG_ERROR("Too few scenarios loaded, use single replay feature instead\n");
        return;
    }

    for (size_t s = 0; s < scenarios_.size(); s++)
    {
        LOG_INFO("Scenarios corresponding to IDs ({}:{}): {}", s * 100, (s + 1) * 100 - 1, scenarios_[s]);
    }

    // Make the base scenario for other cars to merge into
    size_t i    = 0;
    dat_writer_ = std::make_unique<Dat::DatWriter>();
    dat_reader_ = std::make_unique<Dat::DatReader>(scenarios_[i]);
    dat_header_ = ParseDatHeader(scenarios_[i]);
    std::vector<std::vector<double>> scenarios_timestamps;
    std::vector<Timeline<double>>    dts;

    bool has_ghost_restarts = ExtractPacketsAsSlices(false);

    if (has_ghost_restarts)
    {
        ExtractGhostRestarts();
    }

    if (timestamps_.empty())
    {
        LOG_ERROR("No timestamps available in scenario: {}", scenarios_[i]);
        return;
    }

    FillInTimestamps();

    scenarios_timestamps.emplace_back(timestamps_);
    dts.emplace_back(dts_);

    // We loop over rest of scenarios and save their timestamps and delta-times as well as create their timestamp
    // vectors to merge later
    for (i = 1; i < scenarios_.size(); i++)
    {
        timestamps_ = {};
        dts_        = {};

        dat_reader_ = std::make_unique<Dat::DatReader>(scenarios_[i]);
        ParseDatHeader(scenarios_[i]);
        has_ghost_restarts = ExtractPacketsAsSlices(false, i);

        if (has_ghost_restarts)
        {
            ExtractGhostRestarts();
        }

        FillInTimestamps();

        scenarios_timestamps.emplace_back(timestamps_);
        dts.emplace_back(dts_);
    }

    MergeAndCleanTimestamps(scenarios_timestamps);  // creates a new timestamp_ vector

    // Calculate new DTs after we have new timestamp array with possibly new delta-times
    CalculateNewDt();

    // Deal with the slices, first sorting, then removing unnecessary timestamp packets then flatten the slices
    std::sort(packet_slices_.begin(),
              packet_slices_.end(),
              [](const PacketSlice& slice_a, const PacketSlice& slice_b) { return slice_a.timestamp < slice_b.timestamp; });

    RemoveDuplicateTimestampsInSlices(dts_);

    if (has_ghost_restarts)
    {
        LOG_WARN("Some scenario has ghost restart, ignoring the restart when merging");
    }

    FlattenSlices(false);

    if (!create_datfile_.empty())
    {
        LOG_INFO("Creating merged dat file: {}", create_datfile_);
        CreateMergedDatfile(create_datfile_);

        return;
    }

    int ret = ParsePackets();

    if (ret != 0)
    {
        LOG_ERROR("Failed to parse packets on merged .dat files");
        return;
    }

    if (!eos_received_)
    {
        stopTime_ = timestamps_.back();
    }

    startTime_ = timestamps_[0];
    stopIndex_ = static_cast<unsigned int>(timestamps_.size() - 1);
    // stopTime_ set in END_OF_SCENARIO packet

    time_ = startTime_;
}

void Replay::CalculateNewDt()
{
    dts_           = {};
    double temp_dt = LARGE_NUMBER;
    for (size_t j = 0; j < timestamps_.size() - 1; j++)
    {
        double dt = timestamps_[j + 1] - timestamps_[j];
        if (!NEAR_NUMBERS(dt, temp_dt))
        {
            dts_.values.emplace_back(timestamps_[j + 1], dt);
            temp_dt = dt;
        }
    }
}

void Replay::MergeAndCleanTimestamps(const std::vector<std::vector<double>>& scenarios_timestamps)
{
    // Flatten
    size_t total_size = std::accumulate(scenarios_timestamps.begin(),
                                        scenarios_timestamps.end(),
                                        size_t{0},
                                        [](size_t sum, const std::vector<double>& t_v) { return sum + t_v.size(); });

    timestamps_.reserve(total_size);

    for (const auto& t_v : scenarios_timestamps)
    {
        timestamps_.insert(timestamps_.end(), t_v.begin(), t_v.end());
    }

    // Sort
    std::sort(timestamps_.begin(), timestamps_.end());

    // Remove duplicates
    timestamps_.erase(std::unique(timestamps_.begin(), timestamps_.end(), [](double a, double b) { return NEAR_NUMBERS(a, b); }), timestamps_.end());
}

void Replay::RemoveDuplicateTimestampsInSlices(const Timeline<double>& dts)
{
    auto   dts_it         = dts.values.begin();
    bool   dt_written     = false;
    double temp_timestamp = LARGE_NUMBER;
    for (auto it = packet_slices_.begin(); it + 1 != packet_slices_.end();)
    {
        // First packet is either empty or just timestamp, i.e. useless packet.
        if (it->packets.size() < 2)
        {
            it = packet_slices_.erase(it);
            continue;
        }

        // Remove duplicate timestamps
        auto next_it = it + 1;
        if (NEAR_NUMBERS(it->timestamp, next_it->timestamp))
        {
            if (!next_it->packets.empty() && next_it->packets.front().header.id == static_cast<id_t>(Dat::PacketId::TIMESTAMP))
            {
                next_it->packets.erase(next_it->packets.begin());
            }

            if (next_it->packets.empty())
            {
                packet_slices_.erase(next_it);
                continue;
            }
        }

        if (!NEAR_NUMBERS(temp_timestamp, it->timestamp))
        {
            dt_written = false;
        }

        if (dts_it != dts.values.end())
        {
            auto [t, dt] = *dts_it;
            if (NEAR_NUMBERS(t, it->timestamp) && !dt_written)
            {
                Dat::PacketGeneric pkt;
                pkt.header = {static_cast<id_t>(Dat::PacketId::DT), {}};
                dat_writer_->ReWriteToBuffer(pkt, dt);
                it->packets.push_back(pkt);

                dts_it++;
                dt_written     = true;
                temp_timestamp = it->timestamp;
            }
        }

        it++;
    }

    // Write end of scenario last
    Dat::PacketGeneric pkt;
    pkt.header = {static_cast<id_t>(Dat::PacketId::END_OF_SCENARIO), {}};
    dat_writer_->ReWriteToBuffer(pkt, timestamps_.back());
    packet_slices_.back().packets.push_back(pkt);
}

bool Replay::ExtractPacketsAsSlices(bool dt_in_slice, size_t scenario_idx)
{
    // Build the packets containing header and data and store in a vector
    PacketSlice* current_slice  = nullptr;
    bool         has_restart    = false;
    double       prev_timestamp = 0.0;

    Dat::PacketHeader header;
    while (dat_reader_->ReadFile(header))
    {
        LOG_DEBUG("Read packet id={}, size={}", header.id, header.data_size);
        auto packet = dat_reader_->CreateGenericPacket(header);

        double timestamp = prev_timestamp;
        if (packet.header.id == static_cast<id_t>(Dat::PacketId::TIMESTAMP))
        {
            packet_slices_.emplace_back();  // Initializes a PacketSlice in place
            current_slice = &packet_slices_.back();

            dat_reader_->ReadPacket(packet, timestamp);
            current_slice->timestamp = timestamp;

            if (timestamps_.empty() || timestamp > timestamps_.back())
            {
                timestamps_.emplace_back(timestamp);
            }
        }
        else if (packet.header.id == static_cast<id_t>(Dat::PacketId::DT))
        {
            double dt;
            dat_reader_->ReadPacket(packet, dt);
            if (NEAR_NUMBERS(dt, 0.0))
            {
                continue;
            }

            dts_.values.emplace_back(current_slice->timestamp, dt);

            // We don't want to save the dt packet in the current slice
            if (!dt_in_slice)
            {
                continue;
            }
        }
        else if (packet.header.id == static_cast<id_t>(Dat::PacketId::END_OF_SCENARIO))
        {
            double stopTime;
            dat_reader_->ReadPacket(packet, stopTime);

            if (stopTime > stopTime_)
            {
                stopTime_ = stopTime;
            }

            if (stopTime_ > timestamps_.back() + SMALL_NUMBER)
            {
                timestamps_.emplace_back(stopTime_);
            }

            eos_received_ = true;

            continue;  // Don't add the packet to any slices
        }
        // If merging datfiles, we need to adjust the ID based on scenario idx
        else if (scenario_idx > 0 && packet.header.id == static_cast<id_t>(Dat::PacketId::OBJ_ID))
        {
            int obj_id;
            dat_reader_->ReadPacket(packet, obj_id);

            obj_id += static_cast<int>(scenario_idx) * 100;

            dat_writer_->ReWriteToBuffer(packet, obj_id);
        }

        // Skip some packages here if its not the first scenario we are parsing (doesn't make sense to merge)
        if (scenario_idx == 0 || (packet.header.id != static_cast<id_t>(Dat::PacketId::TRAFFIC_LIGHT) &&
                                  packet.header.id != static_cast<id_t>(Dat::PacketId::ELEM_STATE_CHANGE)))
        {
            current_slice->packets.push_back(packet);
        }

        if (timestamp < prev_timestamp)
        {
            has_restart = true;
        }

        prev_timestamp = timestamp;
    }

    return has_restart;
}

int Replay::ParsePackets()
{
    // Now parse packets
    for (size_t i = 0; i < generic_packets_.size(); i++)
    {
        for (const auto& gp : generic_packets_[i])
        {
            switch (gp.header.id)
            {
                case static_cast<id_t>(Dat::PacketId::TIMESTAMP):
                {
                    if (dat_reader_->ReadPacket(gp, timestamp_) != 0)
                    {
                        LOG_ERROR("Failed reading timestamp data.");
                    }
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::OBJ_ID):
                {
                    if (dat_reader_->ReadPacket(gp, current_object_id_) != 0)
                    {
                        LOG_ERROR("Failed reading object ID.");
                        return -1;
                    }

                    if (i > 0)
                    {
                        current_object_id_ = -((current_object_id_ / 100) * 100 + static_cast<int>(i));
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
                    if (dat_reader_->ReadPacket(gp, speed) != 0)
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
                    if (dat_reader_->ReadPacket(gp, pose.x, pose.y, pose.z, pose.h, pose.p, pose.r) != 0)
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
                    if (dat_reader_->ReadPacket(gp, model_id) != 0)
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
                    if (dat_reader_->ReadPacket(gp, obj_type) != 0)
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
                    if (dat_reader_->ReadPacket(gp, obj_category) != 0)
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
                    if (dat_reader_->ReadPacket(gp, ctrl_type) != 0)
                    {
                        LOG_ERROR("Failed reading controller type.");
                        return -1;
                    }
                    current_object_timeline_->ctrl_type_.values.emplace_back(timestamp_, ctrl_type);
                    if (ctrl_type == 100)  // Ghost controller, save the id
                    {
                        ghost_controller_id_ = current_object_id_;
                    }
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::WHEEL_ANGLE):
                {
                    float wheel_angle;
                    if (dat_reader_->ReadPacket(gp, wheel_angle) != 0)
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
                    if (dat_reader_->ReadPacket(gp, wheel_rot) != 0)
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
                    if (dat_reader_->ReadPacket(gp,
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
                    if (dat_reader_->ReadPacket(gp, scale_mode) != 0)
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
                    if (dat_reader_->ReadPacket(gp, visibility_mask) != 0)
                    {
                        LOG_ERROR("Failed reading visibility mask.");
                        return -1;
                    }
                    current_object_timeline_->visibility_mask_.values.emplace_back(timestamp_, visibility_mask);
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::NAME):
                {
                    std::string name = dat_reader_->ReadStringPacket(gp);
                    if (name.empty())
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
                    if (dat_reader_->ReadPacket(gp, road_id) != 0)
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
                    if (dat_reader_->ReadPacket(gp, lane_id) != 0)
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
                    if (dat_reader_->ReadPacket(gp, offset) != 0)
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
                    if (dat_reader_->ReadPacket(gp, t) != 0)
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
                    if (dat_reader_->ReadPacket(gp, s) != 0)
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
                case static_cast<id_t>(Dat::PacketId::TRAFFIC_LIGHT):
                {
                    Dat::TrafficLightLamp lamp;
                    if (dat_reader_->ReadPacket(gp, lamp) != 0)
                    {
                        LOG_ERROR("Failed reading traffic light lamp");
                        return -1;
                    }
                    traffic_lights_timeline_[lamp.lamp_id].values.emplace_back(timestamp_, lamp);
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::REFPOINT_X_OFFSET):
                {
                    float refpoint_x_offset;
                    if (dat_reader_->ReadPacket(gp, refpoint_x_offset) != 0)
                    {
                        LOG_ERROR("Failed reading refpoint_x_offset");
                        return -1;
                    }
                    current_object_timeline_->refpoint_x_offset_.values.emplace_back(timestamp_, refpoint_x_offset);
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::MODEL_X_OFFSET):
                {
                    float model_x_offset;
                    if (dat_reader_->ReadPacket(gp, model_x_offset) != 0)
                    {
                        LOG_ERROR("Failed reading model_x_offset");
                        return -1;
                    }
                    current_object_timeline_->model_x_offset_.values.emplace_back(timestamp_, model_x_offset);
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::OBJ_MODEL3D):
                {
                    std::string model3d = dat_reader_->ReadStringPacket(gp);
                    if (model3d.empty())
                    {
                        LOG_ERROR("Failed reading object 3D model filename.");
                        return -1;
                    }
                    current_object_timeline_->model3d_.values.emplace_back(timestamp_, std::move(model3d));
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::ELEM_STATE_CHANGE):
                {
                    std::string state_change = dat_reader_->ReadStringPacket(gp);
                    if (state_change.empty())
                    {
                        LOG_ERROR("Failed to read element state change");
                        return -1;
                    }

                    std::string esc = BuildElementStateChange(state_change);
                    element_state_changes_.values.emplace_back(timestamp_, esc);
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::SHAPE_2D_OUTLINE):
                {
                    std::vector<SE_Point2D> outline;
                    if (dat_reader_->ReadVectorPacket(gp, outline) != 0)
                    {
                        LOG_ERROR("Failed to read 2D outline");
                        return -1;
                    }
                    current_object_timeline_->outline_.values.emplace_back(timestamp_, outline);
                    break;
                }
                case static_cast<id_t>(Dat::PacketId::DT):
                case static_cast<id_t>(Dat::PacketId::END_OF_SCENARIO):
                default:  // Intentially ignored packets
                {
                    dat_reader_->SkipPacket(gp.header);
                    if (std::find(unknown_pids.begin(), unknown_pids.end(), gp.header.id) == unknown_pids.end())
                    {
                        LOG_DEBUG("Unknown packet with id: {}", gp.header.id);
                        unknown_pids.push_back(gp.header.id);
                    }
                    break;
                }
            }
        }

        if (i > 0)
        {
            UpdateGhostsTimelineAfterRestart(i);
        }
    }

    return 0;
}

void Replay::FlattenSlices(bool add_ghost_restart)
{
    generic_packets_.emplace_back();
    for (auto& slice : packet_slices_)
    {
        for (auto& pkt : slice.packets)
        {
            generic_packets_.front().push_back(std::move(pkt));
        }
    }

    if (add_ghost_restart)
    {
        for (auto& restart : ghost_restarts_)
        {
            generic_packets_.emplace_back();
            for (auto& slice : restart)
            {
                for (auto& pkt : slice.packets)
                {
                    generic_packets_.back().push_back(std::move(pkt));
                }
            }
        }
    }
}

void Replay::ExtractGhostRestarts()
{
    /*
        First loop over all slices to see if we have ghost restart, if we do, then we make them to separate vectors
    */
    std::vector<PacketSlice> temp_slices;
    bool                     g_restart      = false;
    double                   prev_timestamp = 0.0;

    for (auto it = packet_slices_.begin(); it != packet_slices_.end();)
    {
        double timestamp = it->timestamp;

        if (timestamp > 0.0 && timestamp < prev_timestamp - SMALL_NUMBER && !g_restart)
        {
            restart_timestamps_.emplace_back(timestamp, prev_timestamp);
            g_restart = true;
        }
        else if (g_restart && timestamp >= prev_timestamp + SMALL_NUMBER)
        {
            ghost_restarts_.emplace_back(std::move(temp_slices));
            temp_slices.clear();
            g_restart = false;
        }

        if (g_restart)
        {
            temp_slices.push_back(std::move(*it));
            it = packet_slices_.erase(it);
        }
        else
        {
            prev_timestamp = timestamp;
            ++it;
        }
    }
    for (const auto& restart : restart_timestamps_)
        LOG_INFO("Restart start: {}. Restart end: {}", restart.first, restart.second);
}

Dat::DatHeader Replay::ParseDatHeader(const std::string& filename)
{
    // Read raw header BEFORE reading packets
    if (dat_reader_->FillDatHeader(quiet_) != 0)
    {
        int old_header = ReadOldDatHeader(filename);
        if (old_header != -1)
        {
            LOG_ERROR_AND_QUIT("Old DAT file version {} found which is not supported.", old_header, DAT_FILE_FORMAT_VERSION_MAJOR);
        }
        else
        {
            LOG_ERROR_AND_QUIT("Unable to read DAT file header, is it really a DAT file?");
        }

        LOG_ERROR_AND_QUIT("Failed to read DAT header.");
    }

    return dat_reader_->GetDatHeader();
}

void Replay::FillInTimestamps()
{
    double              curr_time = timestamps_.front();
    std::vector<double> filled    = {curr_time};

    // Fixed timestep entire scenario
    if (dts_.values.size() == 1)
    {
        double current_dt = dts_.values.front().second;
        FillEmptyTimestamps(curr_time, timestamps_.back(), current_dt, filled);
    }
    // Mixed timesteps in the scenario
    else
    {
        size_t i = 0;
        for (size_t j = 0; j < dts_.values.size() - 1;)
        {
            double next_timestamp = timestamps_[i + 1];
            double next_dt        = dts_.values[j + 1].second;

            // We have reached the time where the next dt should be used
            // Increment j so we get next dt in next iteration
            if (NEAR_NUMBERS(curr_time, dts_.values[j + 1].first - next_dt))
            {
                j++;
            }
            // The next timestamp of a dt change is before our current time
            // We have a ghost reset
            else if (curr_time - SMALL_NUMBER > dts_.values[j + 1].first)
            {
                // stitch together the timestamps
                double start_time = dts_.values[j + 1].first;
                double end_time   = curr_time;
                double restart_dt = dts_.values[j + 1].second;

                auto   it        = std::upper_bound(filled.begin(), filled.end(), start_time);
                size_t start_idx = static_cast<size_t>(std::distance(filled.begin(), (--it)));
                size_t steps     = static_cast<size_t>(std::llround(((end_time - start_time) / restart_dt) + SMALL_NUMBER));
                for (size_t k = 0; k < steps; k++)
                {
                    double t = k * restart_dt + start_time;
                    for (size_t m = start_idx; m < filled.size(); m++)
                    {
                        if (t > filled[m] + SMALL_NUMBER && t < filled[m + 1] - SMALL_NUMBER)
                        {
                            filled.insert(filled.begin() + static_cast<ptrdiff_t>(m) + 1, t);
                            start_idx = m + 1;
                            break;
                        }
                    }
                }
                j++;
                continue;
            }

            double current_dt = dts_.values[j].second;
            // The gap to the next timestamp is more than 1 sample away, we should fill it
            if (curr_time + next_dt < next_timestamp - SMALL_NUMBER && curr_time + current_dt < next_timestamp - SMALL_NUMBER)
            {
                // We have a large gap
                double end_time = next_timestamp - next_dt;
                FillEmptyTimestamps(curr_time, end_time, current_dt, filled);
            }
            // We have reached the last dt_ value, but the current time is not at the end, so we need to fill with current dt until the end
            else if (j == dts_.values.size() - 1 && curr_time + current_dt < timestamps_.back() - SMALL_NUMBER)
            {
                FillEmptyTimestamps(curr_time, timestamps_.back(), current_dt, filled);
            }
            // We are one sample away, just add it
            else
            {
                filled.emplace_back(next_timestamp);
            }
            i++;
            curr_time = filled.back();
        }
    }

    timestamps_.swap(filled);
}

std::string GetElementType(const std::string& s)
{
    switch (std::stoi(s))
    {
        case 1:
            return "StoryBoard";
        case 2:
            return "Story";
        case 3:
            return "Act";
        case 4:
            return "ManeuverGroup";
        case 5:
            return "Maneuver";
        case 6:
            return "Event";
        case 7:
            return "Action";
        case 8:
            return "Undefined type";
        default:
            LOG_ERROR_AND_QUIT("Can't resolve ElementType {}", s);
    }

    return "";
}

std::string GetElementState(const std::string& s)
{
    switch (std::stoi(s))
    {
        case 0:
            return "Init";
        case 1:
            return "Standby";
        case 2:
            return "Running";
        case 3:
            return "Complete";
        case 4:
            return "Undefined state";
        default:
            LOG_ERROR_AND_QUIT("Can't resolve ElementState {}", s);
    }

    return "";
}

std::string Replay::BuildElementStateChange(const std::string& element_state)
{
    std::string        output = "";
    std::istringstream stream(element_state);
    std::string        line;

    bool first = true;

    while (std::getline(stream, line))
    {
        if (line.empty())
            continue;

        std::istringstream ls(line);

        std::string type_str, state_str, name, path;

        if (!std::getline(ls, type_str, ';') || !std::getline(ls, state_str, ';') || !std::getline(ls, name, ';') || !std::getline(ls, path, ';'))
        {
            continue;
        }

        // Add newline ONLY between lines (not after the last one)
        if (!first)
        {
            output += "\n";
        }

        first = false;

        output += fmt::format("   {} '{}' changed state to '{}' (path: {})", GetElementType(type_str), name, GetElementState(state_str), path);
    }

    return output;
}

void Replay::FillEmptyTimestamps(const double start, const double end, const double dt, std::vector<double>& v)
{
    size_t steps = static_cast<size_t>(std::llround(((end - start) / dt) + SMALL_NUMBER));
    for (size_t i = 0; i < steps; i++)
    {
        v.emplace_back((i + 1) * dt + start);
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
    objects_timeline_.clear();
    timestamps_.clear();
    object_state_cache_.clear();
    dts_ = {};
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
            if (next_index < timestamps_.size() && target_time >= timestamps_[next_index] - SMALL_NUMBER)
            {
                index_ = static_cast<unsigned int>(next_index);
                time_  = timestamps_[index_];
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
            if (index_ > 0 && target_time <= timestamps_[index_ - 1])
            {
                index_--;
                time_ = timestamps_[index_];
            }
            else
            {
                if (target_time < GetStartTime())
                {
                    GoToStart();
                }
                else
                {
                    time_ = target_time;
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

    auto it = std::upper_bound(timestamps_.begin(), timestamps_.end(), time_ + SMALL_NUMBER);

    if (it == timestamps_.end())
    {
        GoToEnd();
        return 0;
    }
    if (*it <= stopTime_ + SMALL_NUMBER)
    {
        index_ = static_cast<unsigned int>(std::distance(timestamps_.begin(), it));
        time_  = *it;
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

    auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), time_ - SMALL_NUMBER);

    if (it == timestamps_.begin())
    {
        GoToStart();
        return;
    }
    else
    {
        --it;  // Move to the previous timestamp
        index_ = static_cast<unsigned int>(std::distance(timestamps_.begin(), it));
        time_  = *it;
        if (time_ < startTime_)
        {
            GoToStart();
        }
    }
}

unsigned int Replay::FindIndexAtTimestamp(double timestamp)
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
    auto it = std::lower_bound(start, end, timestamp - SMALL_NUMBER);
    return static_cast<unsigned int>(std::distance(timestamps_.begin(), it));
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

std::vector<int> Replay::GetAllObjectIDs() const
{
    std::vector<int> ids;
    ids.reserve(objects_timeline_.size());
    for (const auto& [id, _] : objects_timeline_)
    {
        ids.push_back(id);
    }
    return ids;
}

void Replay::SetStartTime(double time)
{
    startTime_ = time;
    if (time_ < startTime_)
    {
        time_ = startTime_;
    }

    startIndex_ = FindIndexAtTimestamp(startTime_);
    index_      = startIndex_;
}

void Replay::SetStopTime(double time)
{
    stopTime_ = time;
    if (time_ > stopTime_)
    {
        time_ = stopTime_;
    }

    stopIndex_ = FindIndexAtTimestamp(stopTime_);
}

void Replay::SetTimeToNearestTimestamp()
{
    if (timestamps_.empty())
    {
        return;
    }

    auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), time_);

    if (it == timestamps_.begin())
    {
        time_ = *timestamps_.begin();
    }
    else if (it == timestamps_.end())
    {
        time_ = timestamps_.back();
    }
    else
    {
        time_ = *it;
    }
}

ReplayEntry Replay::GetReplayEntryAtTimeIncremental(int id, double t) const
{
    ReplayEntry entry;

    const auto& timeline = objects_timeline_.at(id);

    entry.state.info.id             = id;
    entry.state.info.timeStamp      = static_cast<float>(t);
    entry.state.info.model_id       = timeline.model_id_.get_value_incremental(t).value_or(-1);
    entry.state.info.obj_type       = timeline.obj_type_.get_value_incremental(t).value();
    entry.state.info.obj_category   = timeline.obj_category_.get_value_incremental(t).value();
    entry.state.info.ctrl_type      = timeline.ctrl_type_.get_value_incremental(t).value();
    entry.state.info.name           = timeline.name_.get_value_incremental(t).value();
    entry.state.info.speed          = timeline.speed_.get_value_incremental(t).value();
    entry.state.info.wheel_angle    = timeline.wheel_angle_.get_value_incremental(t).value();
    entry.state.info.wheel_rot      = timeline.wheel_rot_.get_value_incremental(t).value();
    entry.state.info.boundingbox    = timeline.bounding_box_.get_value_incremental(t).value();
    entry.state.info.scaleMode      = timeline.scale_mode_.get_value_incremental(t).value();
    entry.state.info.visibilityMask = timeline.visibility_mask_.get_value_incremental(t).value();
    entry.state.info.active         = timeline.active_.get_value_incremental(t).value();
    entry.state.pos.x               = timeline.pose_.get_value_incremental(t).value().x;
    entry.state.pos.y               = timeline.pose_.get_value_incremental(t).value().y;
    entry.state.pos.z               = timeline.pose_.get_value_incremental(t).value().z;
    entry.state.pos.h               = timeline.pose_.get_value_incremental(t).value().h;
    entry.state.pos.p               = timeline.pose_.get_value_incremental(t).value().p;
    entry.state.pos.r               = timeline.pose_.get_value_incremental(t).value().r;
    entry.state.pos.roadId          = timeline.road_id_.get_value_incremental(t).value_or(ID_UNDEFINED);
    entry.state.pos.laneId          = timeline.lane_id_.get_value_incremental(t).value();
    entry.state.pos.offset          = timeline.pos_offset_.get_value_incremental(t).value();
    entry.state.pos.t               = timeline.pos_t_.get_value_incremental(t).value();
    entry.state.pos.s               = timeline.pos_s_.get_value_incremental(t).value();
    entry.odometer                  = timeline.odometer_.get_value_incremental(t).value();

    return entry;
}

ReplayEntry Replay::GetReplayEntryAtTimeBinary(int id, double t) const
{
    ReplayEntry entry;

    const auto& timeline = objects_timeline_.at(id);

    entry.state.info.id             = id;
    entry.state.info.timeStamp      = static_cast<float>(t);
    entry.state.info.model_id       = timeline.model_id_.get_value_binary(t).value_or(-1);
    entry.state.info.obj_type       = timeline.obj_type_.get_value_binary(t).value();
    entry.state.info.obj_category   = timeline.obj_category_.get_value_binary(t).value();
    entry.state.info.ctrl_type      = timeline.ctrl_type_.get_value_binary(t).value();
    entry.state.info.name           = timeline.name_.get_value_binary(t).value().c_str();
    entry.state.info.speed          = timeline.speed_.get_value_binary(t).value();
    entry.state.info.wheel_angle    = timeline.wheel_angle_.get_value_binary(t).value();
    entry.state.info.wheel_rot      = timeline.wheel_rot_.get_value_binary(t).value();
    entry.state.info.boundingbox    = timeline.bounding_box_.get_value_binary(t).value();
    entry.state.info.scaleMode      = timeline.scale_mode_.get_value_binary(t).value();
    entry.state.info.visibilityMask = timeline.visibility_mask_.get_value_binary(t).value();
    entry.state.info.active         = timeline.active_.get_value_binary(t).value();
    entry.state.pos.x               = timeline.pose_.get_value_binary(t).value().x;
    entry.state.pos.y               = timeline.pose_.get_value_binary(t).value().y;
    entry.state.pos.z               = timeline.pose_.get_value_binary(t).value().z;
    entry.state.pos.h               = timeline.pose_.get_value_binary(t).value().h;
    entry.state.pos.p               = timeline.pose_.get_value_binary(t).value().p;
    entry.state.pos.r               = timeline.pose_.get_value_binary(t).value().r;
    entry.state.pos.roadId          = timeline.road_id_.get_value_binary(t).value_or(ID_UNDEFINED);
    entry.state.pos.laneId          = timeline.lane_id_.get_value_binary(t).value();
    entry.state.pos.offset          = timeline.pos_offset_.get_value_binary(t).value();
    entry.state.pos.t               = timeline.pos_t_.get_value_binary(t).value();
    entry.state.pos.s               = timeline.pos_s_.get_value_binary(t).value();
    entry.odometer                  = timeline.odometer_.get_value_binary(t).value();

    return entry;
}

void Replay::UpdateGhostsTimelineAfterRestart(size_t idx)
{
    int  ghost_ghost_id          = -(static_cast<int>(idx));
    auto original_ghost_timeline = objects_timeline_.find(ghost_controller_id_);
    if (original_ghost_timeline == objects_timeline_.end())
    {
        LOG_ERROR_AND_QUIT("No ghost controller found even though a ghost restart was detected. Quitting.");
    }
    if (idx == 0)
    {
        LOG_ERROR_AND_QUIT("Trying to adjust ghost timeline without any ghost restarts, exiting");
    }

    // We have detected a ghost restart, thus we:
    // first decrement the ghost_ghost_counter to ensure every ghost ghost gets a unique ID.
    // then copy the current object's timeline to the new ghost object's timeline and sets the last state to inactive
    // finally, we need to clear the current ghost object's timeline down to the time where ghost reset began
    PropertyTimeline temp = objects_timeline_[ghost_ghost_id];

    // Extract ghost data up until restart and put in ghost_ghost
    objects_timeline_[ghost_ghost_id] = original_ghost_timeline->second;
    auto ghost_timeline_properties    = &objects_timeline_[ghost_ghost_id];
    ghost_timeline_properties->lane_id_.values.resize(ghost_timeline_properties->lane_id_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->road_id_.values.resize(ghost_timeline_properties->road_id_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->pos_offset_.values.resize(ghost_timeline_properties->pos_offset_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->pos_t_.values.resize(ghost_timeline_properties->pos_t_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->pos_s_.values.resize(ghost_timeline_properties->pos_s_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->pose_.values.resize(ghost_timeline_properties->pose_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->speed_.values.resize(ghost_timeline_properties->speed_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->wheel_angle_.values.resize(ghost_timeline_properties->wheel_angle_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->wheel_rot_.values.resize(ghost_timeline_properties->wheel_rot_.get_index_binary(timestamp_).value());
    ghost_timeline_properties->visibility_mask_.values.resize(ghost_timeline_properties->visibility_mask_.get_index_binary(timestamp_).value());

    ghost_timeline_properties->name_.values.front().second += fmt::format("_{}", ghost_ghost_id);
    ghost_timeline_properties->active_.values.front().second = false;  // Not active at start
    double restart_begin                                     = restart_timestamps_[idx - 1].first;
    double restart_end                                       = restart_timestamps_[idx - 1].second;
    ghost_timeline_properties->active_.values.emplace_back(restart_begin, true);  // Active at start of restart
    ghost_timeline_properties->active_.values.emplace_back(restart_end, false);   // Not active after restart time has passed

    auto& ghost_timeline = original_ghost_timeline->second;
    ghost_timeline.lane_id_.replace_data_between_times(restart_begin, restart_end, temp.lane_id_.values, true);
    ghost_timeline.road_id_.replace_data_between_times(restart_begin, restart_end, temp.road_id_.values, true);
    ghost_timeline.pos_offset_.replace_data_between_times(restart_begin, restart_end, temp.pos_offset_.values, true);
    ghost_timeline.pos_t_.replace_data_between_times(restart_begin, restart_end, temp.pos_t_.values, true);
    ghost_timeline.pos_s_.replace_data_between_times(restart_begin, restart_end, temp.pos_s_.values, true);
    ghost_timeline.pose_.replace_data_between_times(restart_begin, restart_end, temp.pose_.values, true);
    ghost_timeline.speed_.replace_data_between_times(restart_begin, restart_end, temp.speed_.values, true);
    ghost_timeline.wheel_angle_.replace_data_between_times(restart_begin, restart_end, temp.wheel_angle_.values, true);
    ghost_timeline.wheel_rot_.replace_data_between_times(restart_begin, restart_end, temp.wheel_rot_.values, true);
    ghost_timeline.visibility_mask_.replace_data_between_times(restart_begin, restart_end, temp.visibility_mask_.values, true);
}

void Replay::CreateMergedDatfile(const std::string filename) const
{
    dat_writer_->Init(filename, dat_header_.odr_filename.string, dat_header_.model_filename.string, dat_header_.git_rev.string);

    if (!dat_writer_->IsWriteFileOpen())
    {
        LOG_ERROR("Failed to open dat file for writing: {}", filename);
        return;
    }

    for (auto packet : generic_packets_[0])
    {
        dat_writer_->WritePacket(packet);
    }

    // Set simulation time so END_OF_SCENARIO packet is written correctly in destructor (dt not important)
    dat_writer_->SetSimulationTime(timestamps_.back(), 0.0);
}

int Replay::ReadOldDatHeader(const std::string& filename)
{
    std::ifstream file;
    file.open(filename, std::ofstream::binary);
    if (file.fail())
    {
        return -1;
    }

    typedef struct
    {
        int  version;
        char odr_filename[512];
        char model_filename[512];
    } OldDatHeader;

    OldDatHeader old_header;
    file.read(reinterpret_cast<char*>(&old_header), sizeof(old_header));
    file.close();

    return old_header.version;
}
