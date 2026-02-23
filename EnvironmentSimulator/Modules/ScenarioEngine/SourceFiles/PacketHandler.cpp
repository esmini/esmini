#include "ScenarioGateway.hpp"
#include "PacketHandler.hpp"

Dat::DatWriter::~DatWriter()
{
    // Seems GateWay which owns DatWriter is destroyed before the scenario ends...
    if (IsWriteFileOpen())
    {
        Write(PacketId::END_OF_SCENARIO, simulation_time_);
        write_file_.flush();
        write_file_.close();
    }
}

int Dat::DatWriter::Init(const std::string& file_name, const std::string& odr_name, const std::string& model_name, const std::string& git_rev)
{
    write_file_.open(file_name, std::ios::binary);
    if (write_file_.fail())
    {
        LOG_ERROR("Cannot open file: {}", file_name);
        return -1;
    }

    // Always write versions first
    unsigned int version_major = DAT_FILE_FORMAT_VERSION_MAJOR;
    unsigned int version_minor = DAT_FILE_FORMAT_VERSION_MINOR;

    write_file_.write(reinterpret_cast<char*>(&version_major), sizeof(version_major));
    write_file_.write(reinterpret_cast<char*>(&version_minor), sizeof(version_minor));

    unsigned int odr_size     = static_cast<unsigned int>(odr_name.size());
    unsigned int model_size   = static_cast<unsigned int>(model_name.size());
    unsigned int git_rev_size = static_cast<unsigned int>(git_rev.size());

    // Size of payload
    unsigned int header_size = sizeof(odr_size) + odr_size + sizeof(model_size) + model_size + sizeof(git_rev_size) + git_rev_size;

    // Write size of dat header packet, excluding version
    write_file_.write(reinterpret_cast<char*>(&header_size), sizeof(header_size));

    // Write payload
    write_file_.write(reinterpret_cast<char*>(&odr_size), sizeof(odr_size));
    write_file_.write(odr_name.data(), odr_size);

    write_file_.write(reinterpret_cast<char*>(&model_size), sizeof(model_size));
    write_file_.write(model_name.data(), model_size);

    write_file_.write(reinterpret_cast<char*>(&git_rev_size), sizeof(git_rev_size));
    write_file_.write(git_rev.data(), git_rev_size);

    return 0;
}

int Dat::DatWriter::WriteDtToDat()
{
    if (!NEAR_NUMBERS(object_state_cache_.dt_, dt_))
    {
        object_state_cache_.dt_ = dt_;
        Write(PacketId::DT, object_state_cache_.dt_);
    }
    return 0;
}

int Dat::DatWriter::WriteTrafficLightsToDat(const std::vector<roadmanager::Signal*>& dynamic_signals)
{
    for (size_t i = 0; i < dynamic_signals.size(); i++)
    {
        auto tl = dynamic_cast<roadmanager::TrafficLight*>(dynamic_signals[i]);
        if (tl == nullptr || !tl->GetHasOSCAction())
        {
            continue;
        }

        for (size_t j = 0; j < tl->GetNrLamps(); j++)
        {
            auto lamp = tl->GetLamp(j);
            if (!lamp->IsDirty())
            {
                continue;
            }

            auto [it, inserted] = object_state_cache_.traffic_lights_lamps_.try_emplace(lamp->GetId());

            if (it->second.lamp_mode != lamp->GetMode())
            {
                it->second = {tl->GetId(), lamp->GetId(), static_cast<unsigned int>(j), static_cast<int>(lamp->GetMode())};
                Write(PacketId::TRAFFIC_LIGHT, it->second);
            }
        }
    }
    return 0;
}

int Dat::DatWriter::WriteStoryBoardStateChangesToDat(const std::vector<std::string>& state_changes)
{
    std::string concat_string;
    for (const auto& state : state_changes)
    {
        if (!concat_string.empty())
        {
            concat_string += "\n";
        }
        concat_string += state;
    }

    if (!concat_string.empty())
    {
        PacketString p_str = {static_cast<unsigned int>(concat_string.size()), concat_string};
        Write(PacketId::ELEM_STATE_CHANGE, p_str);
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
        object_state_cache_.timestamp_ = simulation_time_;

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

            PacketString p_str = {static_cast<unsigned int>(cache_it->second.name_.size()), cache_it->second.name_};
            Write(PacketId::NAME, p_str);
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

        // PacketId::REFPOINT_X_OFFSET
        if (!NEAR_NUMBERSF(cache_it->second.refpoint_x_offset_, static_cast<float>(state->info.refpoint_x_offset)))
        {
            cache_it->second.refpoint_x_offset_ = static_cast<float>(state->info.refpoint_x_offset);
            Write(PacketId::REFPOINT_X_OFFSET, cache_it->second.refpoint_x_offset_);
        }

        // PacketId::MODEL_X_OFFSET
        if (!NEAR_NUMBERSF(cache_it->second.model_x_offset_, static_cast<float>(state->info.model_x_offset)))
        {
            cache_it->second.model_x_offset_ = static_cast<float>(state->info.model_x_offset);
            Write(PacketId::MODEL_X_OFFSET, cache_it->second.model_x_offset_);
        }

        // PacketId::OBJ_MODEL3D
        if (std::strcmp(cache_it->second.model3d_.c_str(), state->info.model3d.c_str()) != 0)
        {
            cache_it->second.model3d_ = state->info.model3d;

            PacketString p_str = {static_cast<unsigned int>(cache_it->second.model3d_.size()), cache_it->second.model3d_};
            Write(PacketId::OBJ_MODEL3D, p_str);
        }

        // PacketId::SHAPE_2D_OUTLINE
        if (cache_it->second.outline_2d.size() == 0 && state->outline.size() > 0)
        {
            cache_it->second.outline_2d = state->outline;

            PacketShape2DOutline packet_shape = {cache_it->second.outline_2d};
            Write(PacketId::SHAPE_2D_OUTLINE, packet_shape.points);
        }

        if (object_state->ReadDirtyBits() & scenarioengine::Object::DirtyBit::LIGHT_STATE)
        {
            for (size_t i = 0; i < cache_it->second.light_state_.size(); i++)
            {
                if (!IsLightStateEqual(cache_it->second.light_state_[i], state->info.light_state[i]))
                {
                    const auto& light                = state->info.light_state[i];
                    cache_it->second.light_state_[i] = {static_cast<int>(light.type),
                                                        light.active,
                                                        light.rgb[0],
                                                        light.rgb[1],
                                                        light.rgb[2],
                                                        light.emission[0],
                                                        light.emission[1],
                                                        light.emission[2]};
                    Write(PacketId::LIGHT_STATE, cache_it->second.light_state_[i]);
                }
            }
        }

        this->SetObjectIdWritten(false);  // Indicate we need to write object id for next state
    }

    this->CheckDeletedObjects();

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
    if (packet.header.data_size != packet.data.size())
    {
        LOG_ERROR_AND_QUIT("packet id {} fails", packet.header.id);
    }
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

void Dat::DatWriter::SetSimulationTime(const double simulation_time, const double dt)
{
    simulation_time_ = simulation_time;
    dt_              = dt;
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

bool Dat::DatWriter::IsLightStateEqual(const LightState& ls, const scenarioengine::Object::VehicleLightStatus& osc_ls) const
{
    return (ls.active == osc_ls.active && ls.r == osc_ls.rgb[0] && ls.g == osc_ls.rgb[1] && ls.b == osc_ls.rgb[2] && ls.e_r == osc_ls.emission[0] &&
            ls.e_g == osc_ls.emission[1] && ls.e_b == osc_ls.emission[2]);
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

    constexpr uint64_t skip_mask =
        (uint64_t{1} << static_cast<unsigned int>(PacketId::END_OF_SCENARIO)) | (uint64_t{1} << static_cast<unsigned int>(PacketId::DT)) |
        (uint64_t{1} << static_cast<unsigned int>(PacketId::TIMESTAMP)) | (uint64_t{1} << static_cast<unsigned int>(PacketId::TRAFFIC_LIGHT)) |
        uint64_t{1} << static_cast<unsigned int>(PacketId::ELEM_STATE_CHANGE);

    // If the bit for p_id is set, we skip writing
    return ((skip_mask >> static_cast<unsigned int>(p_id)) & uint64_t{1}) == 0;
}

/* DAT READER */

Dat::DatReader::DatReader(const std::string& filename) : file_name_(filename)
{
    file_.open(filename, std::ifstream::binary);
    if (file_.fail())
    {
        LOG_ERROR_AND_QUIT("Cannot open file: {}", filename);
    }

    SetFileSize();
}

Dat::DatReader::~DatReader()
{
    CloseFile();
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

void Dat::DatReader::SkipPacket(const Dat::PacketHeader& header)
{
    file_.seekg(header.data_size, std::ios::cur);  // Skips the packet by moving cursor ahead
}

void Dat::DatReader::CloseFile()
{
    if (file_.is_open())
    {
        file_.close();
    }
}

std::string Dat::DatReader::ReadStringPacket(const Dat::PacketGeneric& pkt)
{
    const char*  ptr = pkt.data.data();
    unsigned int size;

    memcpy(&size, ptr, sizeof(size));
    ptr += sizeof(size);

    return std::string(ptr, size);
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

int Dat::DatReader::FillDatHeader(bool quiet)
{
    if (!file_.read(reinterpret_cast<char*>(&header_.version_major), sizeof(header_.version_major)) ||
        !file_.read(reinterpret_cast<char*>(&header_.version_minor), sizeof(header_.version_minor)))
    {
        LOG_ERROR_AND_QUIT("Failed reading header versions.");
    }

    if (header_.version_major != DAT_FILE_FORMAT_VERSION_MAJOR)
    {
        LOG_ERROR_AND_QUIT("Incompatible DAT major file version: {}, supporting version: {}", header_.version_major, DAT_FILE_FORMAT_VERSION_MAJOR);
    }

    unsigned int header_size = 0;
    if (!file_.read(reinterpret_cast<char*>(&header_size), sizeof(header_size)))
    {
        LOG_ERROR_AND_QUIT("Failed reading header size.");
    }

    std::streampos payload_start = file_.tellg();
    std::streampos payload_end   = payload_start + static_cast<std::streampos>(header_size);

    if (ReadStringPacket(header_.odr_filename.string) != 0)
    {
        LOG_ERROR_AND_QUIT("Failed reading odr filename.");
    }

    if (ReadStringPacket(header_.model_filename.string) != 0)
    {
        LOG_ERROR_AND_QUIT("Failed reading model filename.");
    }

    if (ReadStringPacket(header_.git_rev.string) != 0)
    {
        LOG_ERROR_AND_QUIT("Failed reading git rev.");
    }

    if (!quiet)
    {
        LOG_INFO("Datfile {} opened: version {}.{}, odr_filename: {}, model_filename: {}, GIT REV: {}",
                 file_name_,
                 header_.version_major,
                 header_.version_minor,
                 header_.odr_filename.string,
                 header_.model_filename.string,
                 header_.git_rev.string);
    }

    if (header_.version_minor != DAT_FILE_FORMAT_VERSION_MINOR && !quiet)
    {
        LOG_WARN("replayer compiled for version {}.{}. Some inconsistencies are expected.",
                 DAT_FILE_FORMAT_VERSION_MAJOR,
                 DAT_FILE_FORMAT_VERSION_MINOR);
    }

    std::streampos after_read = file_.tellg();

    if (after_read > payload_end)
    {
        LOG_ERROR_AND_QUIT("Unexpected: Dat Header exceeds bounds");
    }

    if (after_read < payload_end)
    {
        auto remaining_payload = payload_end - after_read;
        file_.seekg(remaining_payload, std::ios::cur);
        if (!quiet)
        {
            LOG_WARN("Skipping {} amount of unrecognized .dat header data", static_cast<size_t>(remaining_payload));
        }
    }

    return 0;
}