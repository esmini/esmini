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

int Dat::DatWriter::WriteObjectStatesToDat(const std::vector<scenarioengine::Object*>& objects)
{
    // Write objects
    this->ResetCurrentIds();
    for (const auto& obj : objects)
    {
        // New object state, check if it exists in the cache, else we add it
        current_object_id_ = obj->GetId();
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
        if (!NEAR_NUMBERSF(cache_it->second.speed_, static_cast<float>(obj->GetSpeed())))
        {
            cache_it->second.speed_ = static_cast<float>(obj->GetSpeed());
            Write(PacketId::SPEED, cache_it->second.speed_);
        }
        // PacketId::POSE
        if (!IsPoseEqual(cache_it->second.pose_, obj->pos_))
        {
            cache_it->second.pose_.x = static_cast<float>(obj->pos_.GetX());
            cache_it->second.pose_.y = static_cast<float>(obj->pos_.GetY());
            cache_it->second.pose_.z = static_cast<float>(obj->pos_.GetZ());
            cache_it->second.pose_.h = static_cast<float>(obj->pos_.GetH());
            cache_it->second.pose_.p = static_cast<float>(obj->pos_.GetP());
            cache_it->second.pose_.r = static_cast<float>(obj->pos_.GetR());

            Write(PacketId::POSE,
                  cache_it->second.pose_.x,
                  cache_it->second.pose_.y,
                  cache_it->second.pose_.z,
                  cache_it->second.pose_.h,
                  cache_it->second.pose_.p,
                  cache_it->second.pose_.r);
        }

        // PacketId::MODEL_ID
        if (cache_it->second.model_id_ != obj->model_id_)
        {
            cache_it->second.model_id_ = obj->model_id_;
            Write(PacketId::MODEL_ID, cache_it->second.model_id_);
        }

        // PacketId::OBJ_TYPE
        if (cache_it->second.obj_type_ != obj->GetType())
        {
            cache_it->second.obj_type_ = obj->model_id_;
            Write(PacketId::OBJ_TYPE, cache_it->second.obj_type_);
        }

        // PacketId::OBJ_CATEGORY
        if (cache_it->second.obj_category_ != obj->category_)
        {
            cache_it->second.obj_category_ = obj->category_;
            Write(PacketId::OBJ_CATEGORY, cache_it->second.obj_category_);
        }

        // PacketId::CTRL_TYPE
        scenarioengine::Controller::Type ctrl_type = obj->GetControllerTypeActiveOnDomain(ControlDomains::DOMAIN_LONG);
        // scenarioengine::Controller::Type ctrl_type = scenarioengine::Controller::Type::CONTROLLER_TYPE_DEFAULT;
        if (cache_it->second.ctrl_type_ != ctrl_type)
        {
            cache_it->second.ctrl_type_ = ctrl_type;
            Write(PacketId::CTRL_TYPE, cache_it->second.ctrl_type_);
        }

        // PacketId::WHEEL_ANGLE
        float wheel_angle = static_cast<float>(obj->GetWheelAngle());
        if (!NEAR_NUMBERSF(cache_it->second.wheel_angle_, wheel_angle))
        {
            cache_it->second.wheel_angle_ = wheel_angle;
            Write(PacketId::WHEEL_ANGLE, cache_it->second.wheel_angle_);
        }

        // PacketId::WHEEL_ROT
        float wheel_rot = static_cast<float>(obj->GetWheelRotation());
        if (!NEAR_NUMBERSF(cache_it->second.wheel_rot_, wheel_rot))
        {
            cache_it->second.wheel_rot_ = wheel_rot;
            Write(PacketId::WHEEL_ROT, cache_it->second.wheel_rot_);
        }

        // PacketId::BOUNDING_BOX
        if (!IsBoundingBoxEqual(cache_it->second.bounding_box_, obj->boundingbox_))
        {
            cache_it->second.bounding_box_.x      = obj->boundingbox_.center_.x_;
            cache_it->second.bounding_box_.y      = obj->boundingbox_.center_.y_;
            cache_it->second.bounding_box_.z      = obj->boundingbox_.center_.z_;
            cache_it->second.bounding_box_.length = obj->boundingbox_.dimensions_.length_;
            cache_it->second.bounding_box_.width  = obj->boundingbox_.dimensions_.width_;
            cache_it->second.bounding_box_.height = obj->boundingbox_.dimensions_.height_;

            Write(PacketId::BOUNDING_BOX,
                  cache_it->second.bounding_box_.x,
                  cache_it->second.bounding_box_.y,
                  cache_it->second.bounding_box_.z,
                  cache_it->second.bounding_box_.length,
                  cache_it->second.bounding_box_.width,
                  cache_it->second.bounding_box_.height);
        }

        // PacketId::SCALE_MODE
        int scale_mode = static_cast<int>(obj->scaleMode_);
        if (cache_it->second.scale_mode_ != scale_mode)
        {
            cache_it->second.scale_mode_ = scale_mode;
            Write(PacketId::SCALE_MODE, cache_it->second.scale_mode_);
        }

        // PacketId::VISIBILITY_MASK
        if (cache_it->second.visibility_mask_ != obj->visibilityMask_)
        {
            cache_it->second.visibility_mask_ = obj->visibilityMask_;
            Write(PacketId::VISIBILITY_MASK, cache_it->second.visibility_mask_);
        }
        // PacketId::NAME
        if (cache_it->second.name_.c_str() != obj->GetName())
        {
            cache_it->second.name_ = obj->GetName();

            PacketString p_str = {static_cast<unsigned int>(cache_it->second.name_.size()), cache_it->second.name_};
            Write(PacketId::NAME, p_str);
        }
        // PacketId::ROAD_ID
        if (cache_it->second.road_id_ != obj->pos_.GetTrackId())
        {
            cache_it->second.road_id_ = obj->pos_.GetTrackId();
            Write(PacketId::ROAD_ID, cache_it->second.road_id_);
        }

        // PacketId::LANE_ID
        if (cache_it->second.lane_id_ != obj->pos_.GetLaneId())
        {
            cache_it->second.lane_id_ = obj->pos_.GetLaneId();
            Write(PacketId::LANE_ID, cache_it->second.lane_id_);
        }

        // PacketId::POS_OFFSET
        if (!NEAR_NUMBERSF(cache_it->second.pos_offset_, static_cast<float>(obj->pos_.GetOffset())))
        {
            cache_it->second.pos_offset_ = static_cast<float>(obj->pos_.GetOffset());
            Write(PacketId::POS_OFFSET, cache_it->second.pos_offset_);
        }

        // PacketId::POS_T
        if (!NEAR_NUMBERSF(cache_it->second.pos_t_, static_cast<float>(obj->pos_.GetT())))
        {
            cache_it->second.pos_t_ = static_cast<float>(obj->pos_.GetT());
            Write(PacketId::POS_T, cache_it->second.pos_t_);
        }

        // PacketId::POS_S
        if (!NEAR_NUMBERSF(cache_it->second.pos_s_, static_cast<float>(obj->pos_.GetS())))
        {
            cache_it->second.pos_s_ = static_cast<float>(obj->pos_.GetS());
            Write(PacketId::POS_S, cache_it->second.pos_s_);
        }

        // PacketId::REFPOINT_X_OFFSET
        if (!NEAR_NUMBERSF(cache_it->second.refpoint_x_offset_, static_cast<float>(obj->refpoint_x_offset_)))
        {
            cache_it->second.refpoint_x_offset_ = static_cast<float>(obj->refpoint_x_offset_);
            Write(PacketId::REFPOINT_X_OFFSET, cache_it->second.refpoint_x_offset_);
        }

        // PacketId::MODEL_X_OFFSET
        if (!NEAR_NUMBERSF(cache_it->second.model_x_offset_, static_cast<float>(obj->model3d_x_offset_)))
        {
            cache_it->second.model_x_offset_ = static_cast<float>(obj->model3d_x_offset_);
            Write(PacketId::MODEL_X_OFFSET, cache_it->second.model_x_offset_);
        }

        // PacketId::OBJ_MODEL3D
        if (cache_it->second.model3d_ != obj->GetModel3DFullPath())
        {
            cache_it->second.model3d_ = obj->GetModel3DFullPath();

            PacketString p_str = {static_cast<unsigned int>(cache_it->second.model3d_.size()), cache_it->second.model3d_};
            Write(PacketId::OBJ_MODEL3D, p_str);
        }

        // PacketId::SHAPE_2D_OUTLINE
        if (cache_it->second.outline_2d.size() == 0 && obj->outline_2d_.size() > 0)
        {
            cache_it->second.outline_2d = obj->outline_2d_;

            PacketShape2DOutline packet_shape = {cache_it->second.outline_2d};
            Write(PacketId::SHAPE_2D_OUTLINE, packet_shape);
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
    if (!file_.read(reinterpret_cast<char*>(&header), sizeof(header)))
    {
        if (!file_.eof())
        {
            LOG_ERROR("Failed to read packet header.");
        }
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

std::vector<SE_Point2D> Dat::DatReader::ReadOutlinePacket(const Dat::PacketGeneric& pkt)
{
    const char* ptr = pkt.data.data();

    std::vector<SE_Point2D> points(pkt.header.data_size / sizeof(SE_Point2D));
    memcpy(points.data(), ptr, pkt.header.data_size);

    return points;
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