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

    Dat::ObjState state;
    while (file_.tellg() < file_size)
    {
        Dat::PacketHeader header;
        if (!file_.read(reinterpret_cast<char*>(&header), sizeof(header)))
        {
            LOG_ERROR("Failed to read packet header.");
            break;
        }

        if (header.id > static_cast<int>(Dat::PacketId::END_OF_SCENARIO))
        {
            LOG_ERROR("Unknown packet id: {}", header.id);
            break;
        }

        switch (header.id)
        {
            case static_cast<id_t>(Dat::PacketId::DAT_HEADER):
            {
                Dat::DatHeader d_header;

                if (!file_.read(reinterpret_cast<char*>(&d_header.version_major), sizeof(d_header.version_major)) ||
                    !file_.read(reinterpret_cast<char*>(&d_header.version_minor), sizeof(d_header.version_minor)))
                {
                    LOG_ERROR("Failed reading header versions.");
                    break;
                }

                if (!file_.read(reinterpret_cast<char*>(&d_header.odr_filename.size), sizeof(d_header.odr_filename.size)))
                {
                    LOG_ERROR("Failed reading odr filename size.");
                    break;
                }
                d_header.odr_filename.string.resize(d_header.odr_filename.size);
                if (!file_.read(d_header.odr_filename.string.data(), d_header.odr_filename.size))
                {
                    LOG_ERROR("Failed reading odr filename string.");
                    break;
                }

                if (!file_.read(reinterpret_cast<char*>(&d_header.model_filename.size), sizeof(d_header.model_filename.size)))
                {
                    LOG_ERROR("Failed reading odr model filename size.");
                    break;
                }
                d_header.model_filename.string.resize(d_header.model_filename.size);
                if (!file_.read(d_header.model_filename.string.data(), d_header.model_filename.size))
                {
                    LOG_ERROR("Failed reading odr model string.");
                    break;
                }

                LOG_INFO("Version: {}.{}", d_header.version_major, d_header.version_minor);
                LOG_INFO("ODR file: {}", d_header.odr_filename.string);
                LOG_INFO("Model file: {}", d_header.model_filename.string);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::TIMESTAMP):
            {
                double timestamp;
                if (ReadPacket(header, timestamp) != 0)
                    LOG_ERROR("Failed reading speed data.");
                LOG_INFO("Timestamp: {:.2f}", timestamp);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_ID):
            {
                if (ReadPacket(header, state.obj_id_) != 0)
                    LOG_ERROR("Failed reading object ID.");
                LOG_INFO("Object ID: {}", state.obj_id_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::SPEED):
            {
                if (ReadPacket(header, state.speed_) != 0)
                    LOG_ERROR("Failed reading speed data.");
                LOG_INFO("Speed: {:.2f}", state.speed_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POSE):
            {
                if (ReadPacket(header, state.pose_.x, state.pose_.y, state.pose_.z, state.pose_.h, state.pose_.p, state.pose_.r) != 0)
                    LOG_ERROR("Failed reading pose data.");
                LOG_INFO("Pose: x={:.2f}, y={:.2f}, z={:.2f}, h={:.2f}, p={:.2f}, r={:.2f}",
                         state.pose_.x,
                         state.pose_.y,
                         state.pose_.z,
                         state.pose_.h,
                         state.pose_.p,
                         state.pose_.r);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::MODEL_ID):
            {
                if (ReadPacket(header, state.model_id_) != 0)
                    LOG_ERROR("Failed reading model ID.");
                LOG_INFO("Model ID: {}", state.model_id_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_TYPE):
            {
                if (ReadPacket(header, state.obj_type_) != 0)
                    LOG_ERROR("Failed reading object type.");
                LOG_INFO("Object Type: {}", state.obj_type_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::OBJ_CATEGORY):
            {
                if (ReadPacket(header, state.obj_category_) != 0)
                    LOG_ERROR("Failed reading object category.");
                LOG_INFO("Object Category: {}", state.obj_category_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::CTRL_TYPE):
            {
                if (ReadPacket(header, state.ctrl_type_) != 0)
                    LOG_ERROR("Failed reading controller type.");
                LOG_INFO("Controller Type: {}", state.ctrl_type_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::WHEEL_ANGLE):
            {
                if (ReadPacket(header, state.wheel_angle_) != 0)
                    LOG_ERROR("Failed reading wheel angle.");
                LOG_INFO("Wheel Angle: {:.2f}", state.wheel_angle_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::WHEEL_ROT):
            {
                if (ReadPacket(header, state.wheel_rot_) != 0)
                    LOG_ERROR("Failed reading wheel rotation.");
                LOG_INFO("Wheel Rotation: {:.2f}", state.wheel_rot_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::BOUNDING_BOX):
            {
                if (ReadPacket(header,
                               state.bounding_box_.x,
                               state.bounding_box_.y,
                               state.bounding_box_.z,
                               state.bounding_box_.length,
                               state.bounding_box_.width,
                               state.bounding_box_.height) != 0)
                {
                    LOG_ERROR("Failed reading bounding box data.");
                }
                LOG_INFO("Bounding Box: x={:.2f}, y={:.2f}, z={:.2f}, length={:.2f}, width={:.2f}, height={:.2f}",
                         state.bounding_box_.x,
                         state.bounding_box_.y,
                         state.bounding_box_.z,
                         state.bounding_box_.length,
                         state.bounding_box_.width,
                         state.bounding_box_.height);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::SCALE_MODE):
            {
                if (ReadPacket(header, state.scale_mode_) != 0)
                    LOG_ERROR("Failed reading scale mode.");
                LOG_INFO("Scale Mode: {}", state.scale_mode_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::VISIBILITY_MASK):
            {
                if (ReadPacket(header, state.visibility_mask_) != 0)
                    LOG_ERROR("Failed reading visibility mask.");
                LOG_INFO("Visibility Mask: {}", state.visibility_mask_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::NAME):
            {
                unsigned int name_size;
                if (!file_.read(reinterpret_cast<char*>(&name_size), sizeof(unsigned int)))
                {
                    LOG_ERROR("Failed reading object name size.");
                    break;
                }
                state.name_.resize(name_size);
                if (!file_.read(state.name_.data(), name_size))
                    LOG_ERROR("Failed reading object name.");
                LOG_INFO("Name: {}", state.name_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::ROAD_ID):
            {
                if (ReadPacket(header, state.road_id_) != 0)
                    LOG_ERROR("Failed reading road ID.");
                LOG_INFO("Road ID: {}", state.road_id_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::LANE_ID):
            {
                if (ReadPacket(header, state.lane_id_) != 0)
                    LOG_ERROR("Failed reading lane ID.");
                LOG_INFO("Lane ID: {}", state.lane_id_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_OFFSET):
            {
                if (ReadPacket(header, state.pos_offset_) != 0)
                    LOG_ERROR("Failed reading position offset.");
                LOG_INFO("Position Offset: {:.2f}", state.pos_offset_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_T):
            {
                if (ReadPacket(header, state.pos_t_) != 0)
                    LOG_ERROR("Failed reading position T.");
                LOG_INFO("Position T: {:.2f}", state.pos_t_);
                break;
            }
            case static_cast<id_t>(Dat::PacketId::POS_S):
            {
                if (ReadPacket(header, state.pos_s_) != 0)
                    LOG_ERROR("Failed reading position S.");
                LOG_INFO("Position S: {:.2f}", state.pos_s_);
                break;
            }
            default:
            {
                // Skip the data for this packet (unknown)
                file_.seekg(header.data_size, std::ios::cur);
                break;
            }
        }
    }

    file_.close();
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
        LOG_INFO("Recording {} opened. dat version: {} odr: {} model: {}",
                 FileNameOf(scenarios_[i]),
                 header_.version,
                 FileNameOf(header_.odr_filename),
                 FileNameOf(header_.model_filename));

        if (header_.version != DAT_FILE_FORMAT_VERSION)
        {
            LOG_ERROR_AND_QUIT("Version mismatch. {} is version {} while supported version is {}. Please re-create dat file.",
                               scenarios_[i],
                               header_.version,
                               DAT_FILE_FORMAT_VERSION);
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