#define DAT_VERSION_MAJOR 1
#define DAT_VERSION_MINOR 0

#include "ScenarioGateway.hpp"
#include "CommonMini.cpp"
#include "PacketHandler.hpp"

Dat::DatWriter::DatWriter()
{
    if (SE_Env::Inst().GetOptions().GetOptionSet("fixed_timestep"))
    {
        fixed_timestep_ = std::stof(SE_Env::Inst().GetOptions().GetOptionArg("fixed_timestep"));
    }
}

Dat::DatWriter::~DatWriter()
{
    // Seems GateWay which owns DatWriter is destroyed before the scenario ends...
    if (IsWriteFileOpen())
    {
        Write(PacketId::END_OF_SCENARIO, static_cast<float>(simulation_time_));
        write_file_.flush();
        write_file_.close();
    }
}

int Dat::DatWriter::Init(const std::string& file_name, const std::string& odr_name, const std::string& model_name)
{
    write_file_.open(file_name, std::ios::binary);
    if (write_file_.fail())
    {
        LOG_ERROR("Cannot open file: {}", file_name);
        return -1;
    }

    // Write version
    unsigned int version_major = DAT_VERSION_MAJOR;
    unsigned int version_minor = DAT_VERSION_MINOR;
    write_file_.write(reinterpret_cast<char*>(&version_major), sizeof(version_major));
    write_file_.write(reinterpret_cast<char*>(&version_minor), sizeof(version_minor));

    // Write odr filename
    unsigned int odr_size = static_cast<unsigned int>(odr_name.size());
    write_file_.write(reinterpret_cast<char*>(&odr_size), sizeof(odr_size));
    write_file_.write(odr_name.data(), odr_size);

    // Write model filename
    unsigned int model_size = static_cast<unsigned int>(model_name.size());
    write_file_.write(reinterpret_cast<char*>(&model_size), sizeof(model_size));
    write_file_.write(model_name.data(), model_size);

    return 0;
}

// Write data not specific to object states (don't forget to update ShouldWriteObjId)
int Dat::DatWriter::WriteGenericDataToDat()
{
    // PacketId::DT
    if (!NEAR_NUMBERSF(object_state_cache_.dt_, fixed_timestep_))
    {
        object_state_cache_.dt_ = fixed_timestep_;
        Write(PacketId::DT, object_state_cache_.dt_);
    }

    return 0;
}

int Dat::DatWriter::WriteObjectStatesToDat(const std::vector<std::unique_ptr<scenarioengine::ObjectState>>& object_states)
{
    // Write objects
    this->ResetCurrentIds();
    for (const auto& object_state : object_states)
    {
        // New object state, check if it exists in the cache, else we add it
        const auto state   = &object_state->state_;
        current_object_id_ = state->info.id;
        current_ids_.insert(current_object_id_);

        auto cache_it = object_state_cache_.state_.find(current_object_id_);
        if (cache_it == object_state_cache_.state_.end())
        {
            // New object state, add it to the cache
            ObjState obj_state;
            obj_state.obj_id_                              = current_object_id_;
            object_state_cache_.state_[current_object_id_] = obj_state;
            cache_it                                       = object_state_cache_.state_.find(current_object_id_);
        }

        // We might want to write a state to datfile, so we set the timestamp
        object_state_cache_.timestamp_ = static_cast<float>(simulation_time_);

        // PacketId::SPEED
        if (!NEAR_NUMBERSF(cache_it->second.speed_, static_cast<float>(state->info.speed)))
        {
            cache_it->second.speed_ = static_cast<float>(state->info.speed);
            Write(PacketId::SPEED, cache_it->second.speed_);
        }
        // PacketId::POSE
        if (!IsPoseEqual(cache_it->second.pose_, state->pos))
        {
            cache_it->second.pose_.x = static_cast<float>(state->pos.GetX());
            cache_it->second.pose_.y = static_cast<float>(state->pos.GetY());
            cache_it->second.pose_.z = static_cast<float>(state->pos.GetZ());
            cache_it->second.pose_.h = static_cast<float>(state->pos.GetH());
            cache_it->second.pose_.p = static_cast<float>(state->pos.GetP());
            cache_it->second.pose_.r = static_cast<float>(state->pos.GetR());

            Write(PacketId::POSE,
                  cache_it->second.pose_.x,
                  cache_it->second.pose_.y,
                  cache_it->second.pose_.z,
                  cache_it->second.pose_.h,
                  cache_it->second.pose_.p,
                  cache_it->second.pose_.r);
        }

        // PacketId::MODEL_ID
        if (cache_it->second.model_id_ != state->info.model_id)
        {
            cache_it->second.model_id_ = state->info.model_id;
            Write(PacketId::MODEL_ID, cache_it->second.model_id_);
        }

        // PacketId::OBJ_TYPE
        if (cache_it->second.obj_type_ != state->info.obj_type)
        {
            cache_it->second.obj_type_ = state->info.obj_type;
            Write(PacketId::OBJ_TYPE, cache_it->second.obj_type_);
        }

        // PacketId::OBJ_CATEGORY
        if (cache_it->second.obj_category_ != state->info.obj_category)
        {
            cache_it->second.obj_category_ = state->info.obj_category;
            Write(PacketId::OBJ_CATEGORY, cache_it->second.obj_category_);
        }

        // PacketId::CTRL_TYPE
        if (cache_it->second.ctrl_type_ != state->info.ctrl_type)
        {
            cache_it->second.ctrl_type_ = state->info.ctrl_type;
            Write(PacketId::CTRL_TYPE, cache_it->second.ctrl_type_);
        }

        // PacketId::WHEEL_ANGLE
        float wheel_angle = (state->info.wheel_data.empty()) ? 0.0f : static_cast<float>(state->info.wheel_data[0].h);
        if (!NEAR_NUMBERSF(cache_it->second.wheel_angle_, wheel_angle))
        {
            cache_it->second.wheel_angle_ = wheel_angle;
            Write(PacketId::WHEEL_ANGLE, cache_it->second.wheel_angle_);
        }

        // PacketId::WHEEL_ROT
        float wheel_rot = (state->info.wheel_data.empty()) ? 0.0f : static_cast<float>(state->info.wheel_data[0].p);
        if (!NEAR_NUMBERSF(cache_it->second.wheel_rot_, wheel_rot))
        {
            cache_it->second.wheel_rot_ = wheel_rot;
            Write(PacketId::WHEEL_ROT, cache_it->second.wheel_rot_);
        }

        // PacketId::BOUNDING_BOX
        if (!IsBoundingBoxEqual(cache_it->second.bounding_box_, state->info.boundingbox))
        {
            cache_it->second.bounding_box_.x      = state->info.boundingbox.center_.x_;
            cache_it->second.bounding_box_.y      = state->info.boundingbox.center_.y_;
            cache_it->second.bounding_box_.z      = state->info.boundingbox.center_.z_;
            cache_it->second.bounding_box_.length = state->info.boundingbox.dimensions_.length_;
            cache_it->second.bounding_box_.width  = state->info.boundingbox.dimensions_.width_;
            cache_it->second.bounding_box_.height = state->info.boundingbox.dimensions_.height_;

            Write(PacketId::BOUNDING_BOX,
                  cache_it->second.bounding_box_.x,
                  cache_it->second.bounding_box_.y,
                  cache_it->second.bounding_box_.z,
                  cache_it->second.bounding_box_.length,
                  cache_it->second.bounding_box_.width,
                  cache_it->second.bounding_box_.height);
        }

        // PacketId::SCALE_MODE
        if (cache_it->second.scale_mode_ != state->info.scaleMode)
        {
            cache_it->second.scale_mode_ = state->info.scaleMode;
            Write(PacketId::SCALE_MODE, cache_it->second.scale_mode_);
        }

        // PacketId::VISIBILITY_MASK
        if (cache_it->second.visibility_mask_ != state->info.visibilityMask)
        {
            cache_it->second.visibility_mask_ = state->info.visibilityMask;
            Write(PacketId::VISIBILITY_MASK, cache_it->second.visibility_mask_);
        }
        // PacketId::NAME
        if (std::strcmp(cache_it->second.name_.c_str(), state->info.name) != 0)
        {
            cache_it->second.name_ = std::string(state->info.name);

            auto name_size = static_cast<unsigned int>(cache_it->second.name_.size());
            Write(PacketId::NAME, name_size, cache_it->second.name_);
        }
        // PacketId::ROAD_ID
        if (cache_it->second.road_id_ != state->pos.GetTrackId())
        {
            cache_it->second.road_id_ = state->pos.GetTrackId();
            Write(PacketId::ROAD_ID, cache_it->second.road_id_);
        }

        // PacketId::LANE_ID
        if (cache_it->second.lane_id_ != state->pos.GetLaneId())
        {
            cache_it->second.lane_id_ = state->pos.GetLaneId();
            Write(PacketId::LANE_ID, cache_it->second.lane_id_);
        }

        // PacketId::POS_OFFSET
        if (!NEAR_NUMBERSF(cache_it->second.pos_offset_, static_cast<float>(state->pos.GetOffset())))
        {
            cache_it->second.pos_offset_ = static_cast<float>(state->pos.GetOffset());
            Write(PacketId::POS_OFFSET, cache_it->second.pos_offset_);
        }

        // PacketId::POS_T
        if (!NEAR_NUMBERSF(cache_it->second.pos_t_, static_cast<float>(state->pos.GetT())))
        {
            cache_it->second.pos_t_ = static_cast<float>(state->pos.GetT());
            Write(PacketId::POS_T, cache_it->second.pos_t_);
        }

        // PacketId::POS_S
        if (!NEAR_NUMBERSF(cache_it->second.pos_s_, static_cast<float>(state->pos.GetS())))
        {
            cache_it->second.pos_s_ = static_cast<float>(state->pos.GetS());
            Write(PacketId::POS_S, cache_it->second.pos_s_);
        }

        this->SetObjectIdWritten(false);  // Indicate we need to write object id for next state
    }

    // In case we have variable timestep, we need to write every time packet
    if (!timestamp_written_ && fixed_timestep_ == -1.0f)
    {
        PacketGeneric packet;
        packet.header.id        = static_cast<id_t>(PacketId::TIMESTAMP);
        packet.header.data_size = sizeof(float);
        packet.data.resize(packet.header.data_size);
        char* write_ptr = packet.data.data();
        WriteToBuffer(write_ptr, static_cast<float>(simulation_time_));
        WritePacket(packet);  // Direct write, avoids recursion and repeated flags
    }

    this->CheckDeletedObjects();
    this->SetTimestampWritten(false);  // Reset timestamp written flag after writing all states

    return 0;
}

void Dat::DatWriter::CheckDeletedObjects()
{
    // Will be empty before first iteration, so we ignore that case
    if (!previous_ids_.empty())
    {
        // Write deleted objects
        for (const auto& previous_id : previous_ids_)
        {
            if (current_ids_.count(previous_id) == 0)
            {
                // Object used to exist but was not found in the current state, so it has been deleted
                current_object_id_ = previous_id;
                Write(PacketId::OBJ_DELETED);
                object_state_cache_.state_.erase(previous_id);
                this->SetObjectIdWritten(false);  // Need to reset this flag so write function will write the object ID
            }
        }
    }

    previous_ids_ = std::move(current_ids_);
}

void Dat::DatWriter::WritePacket(PacketGeneric& packet)
{
    write_file_.write(reinterpret_cast<char*>(&packet.header), sizeof(PacketHeader));
    write_file_.write(packet.data.data(), static_cast<std::streamsize>(packet.data.size()));
}

bool Dat::DatWriter::IsWriteFileOpen() const
{
    return write_file_.is_open();
}

void Dat::DatWriter::SetTimestampWritten(bool state)
{
    timestamp_written_ = state;
}

void Dat::DatWriter::SetObjectIdWritten(bool state)
{
    object_id_written_ = state;
}

void Dat::DatWriter::SetSimulationTime(const double simulation_time)
{
    simulation_time_ = simulation_time;
}

void Dat::DatWriter::ResetCurrentIds()
{
    current_ids_.clear();
}

bool Dat::DatWriter::IsPoseEqual(const Pose& pose, const roadmanager::Position& pos) const
{
    return (pose.x == static_cast<float>(pos.GetX()) && pose.y == static_cast<float>(pos.GetY()) && pose.z == static_cast<float>(pos.GetZ()) &&
            pose.h == static_cast<float>(pos.GetH()) && pose.p == static_cast<float>(pos.GetP()) && pose.r == static_cast<float>(pos.GetR()));
}

bool Dat::DatWriter::IsBoundingBoxEqual(const BoundingBox& bb, const scenarioengine::OSCBoundingBox& osc_bb) const
{
    return (bb.x == osc_bb.center_.x_ && bb.y == osc_bb.center_.y_ && bb.z == osc_bb.center_.z_ && bb.length == osc_bb.dimensions_.length_ &&
            bb.width == osc_bb.dimensions_.width_ && bb.height == osc_bb.dimensions_.height_);
}

size_t Dat::DatWriter::SerializedSize(const std::string& str)
{
    return str.size();
}

void Dat::DatWriter::WriteToBuffer(char*& write_ptr, const std::string& str)
{
    memcpy(write_ptr, str.data(), str.size());
    write_ptr += str.size();
}

constexpr bool Dat::DatWriter::ShouldWriteObjId(PacketId p_id) const noexcept
{
    static_assert(static_cast<int>(PacketId::END_OF_SCENARIO) < 64, "PacketId values must be < 64");

    constexpr uint64_t skip_mask = (uint64_t{1} << static_cast<unsigned int>(PacketId::END_OF_SCENARIO)) |
                                   (uint64_t{1} << static_cast<unsigned int>(PacketId::DT)) |
                                   (uint64_t{1} << static_cast<unsigned int>(PacketId::TIMESTAMP));

    // If the bit for p_id is set, we skip writing
    return ((skip_mask >> static_cast<unsigned int>(p_id)) & uint64_t{1}) == 0;
}

/* DAT READER */

Dat::DatReader::DatReader(const std::string& filename)
{
    file_.open(filename, std::ifstream::binary);
    if (file_.fail())
    {
        LOG_ERROR_AND_QUIT("Cannot open file: {}", filename);
    }

    LOG_INFO("Datfile {} opened.", FileNameOf(filename));

    SetFileSize();
}

void Dat::DatReader::SetFileSize()
{
    file_.seekg(0, std::ios::end);
    file_size_ = file_.tellg();
    file_.seekg(0, std::ios::beg);  // jump back to start
}

bool Dat::DatReader::ReadFile(Dat::PacketHeader& header)
{
    if (file_.tellg() >= file_size_)
    {
        return false;
    }

    if (!file_.read(reinterpret_cast<char*>(&header), sizeof(header)))
    {
        LOG_ERROR("Failed to read packet header.");
        return false;
    }

    return true;
}

void Dat::DatReader::UnknownPacket(const Dat::PacketHeader& header)
{
    LOG_ERROR("Unknown packet id: {}", header.id);
    file_.seekg(header.data_size, std::ios::cur);  // Skips the packet by moving cursor ahead
}

void Dat::DatReader::CloseFile()
{
    if (file_.is_open())
    {
        file_.close();
    }
}

int Dat::DatReader::ReadStringPacket(std::string& str)
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

int Dat::DatReader::FillDatHeader()
{
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