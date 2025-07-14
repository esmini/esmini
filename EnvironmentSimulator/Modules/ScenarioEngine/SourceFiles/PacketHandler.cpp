#define DAT_VERSION_MAJOR 1
#define DAT_VERSION_MINOR 0

#include "ScenarioGateway.hpp"
#include "CommonMini.cpp"
#include "PacketHandler.hpp"

int Dat::DatLogger::Init(const std::string& file_name, const std::string& odr_name, const std::string& model_name)
{
    data_file_.open(file_name, std::ios::binary);
    if (data_file_.fail())
    {
        LOG_ERROR("Cannot open file: {}", file_name);
        return -1;
    }

    DatHeader d_header;
    d_header.version_major = DAT_VERSION_MAJOR;
    d_header.version_minor = DAT_VERSION_MINOR;

    d_header.odr_filename.size   = static_cast<unsigned int>(odr_name.size());
    d_header.odr_filename.string = odr_name;

    d_header.model_filename.size   = static_cast<unsigned int>(model_name.size());
    d_header.model_filename.string = model_name;

    // Write content
    Write(PacketId::DAT_HEADER,
          d_header.version_major,
          d_header.version_minor,
          d_header.odr_filename.size,
          d_header.odr_filename.string,
          d_header.model_filename.size,
          d_header.model_filename.string);

    return 0;
}

int Dat::DatLogger::WriteToDat(const scenarioengine::ObjectStateStruct& object_state)
{
    // We want to write a state to datfile, so first we set the timestamp
    object_state_cache_.timestamp_ = object_state.info.timeStamp;

    // PacketId::OBJ_ID
    auto cache_it = object_state_cache_.state_.find(object_state.info.id);
    if (cache_it == object_state_cache_.state_.end())
    {
        // New object state, add it to the cache
        ObjState obj_state;
        obj_state.obj_id_                                = object_state.info.id;
        object_state_cache_.state_[object_state.info.id] = obj_state;
        cache_it                                         = object_state_cache_.state_.find(object_state.info.id);
    }

    // PacketId::SPEED
    if (!IsDoubleEqual(cache_it->second.speed_, object_state.info.speed))
    {
        cache_it->second.speed_ = object_state.info.speed;
        Write(PacketId::SPEED, cache_it->second.speed_);
    }

    // PacketId::POSE
    if (!IsPoseEqual(cache_it->second.pose_, object_state.pos))
    {
        cache_it->second.pose_.x = object_state.pos.GetX();
        cache_it->second.pose_.y = object_state.pos.GetY();
        cache_it->second.pose_.z = object_state.pos.GetZ();
        cache_it->second.pose_.h = object_state.pos.GetH();
        cache_it->second.pose_.p = object_state.pos.GetP();
        cache_it->second.pose_.r = object_state.pos.GetR();

        Write(PacketId::POSE,
              cache_it->second.pose_.x,
              cache_it->second.pose_.y,
              cache_it->second.pose_.z,
              cache_it->second.pose_.h,
              cache_it->second.pose_.p,
              cache_it->second.pose_.r);
    }

    // PacketId::MODEL_ID
    if (cache_it->second.model_id_ != object_state.info.model_id)
    {
        cache_it->second.model_id_ = object_state.info.model_id;
        Write(PacketId::MODEL_ID, cache_it->second.model_id_);
    }

    // PacketId::OBJ_TYPE
    if (cache_it->second.obj_type_ != object_state.info.obj_type)
    {
        cache_it->second.obj_type_ = object_state.info.obj_type;
        Write(PacketId::OBJ_TYPE, cache_it->second.obj_type_);
    }

    // PacketId::OBJ_CATEGORY
    if (cache_it->second.obj_category_ != object_state.info.obj_category)
    {
        cache_it->second.obj_category_ = object_state.info.obj_category;
        Write(PacketId::OBJ_CATEGORY, cache_it->second.obj_category_);
    }

    // PacketId::CTRL_TYPE
    if (cache_it->second.ctrl_type_ != object_state.info.ctrl_type)
    {
        cache_it->second.ctrl_type_ = object_state.info.ctrl_type;
        Write(PacketId::CTRL_TYPE, cache_it->second.ctrl_type_);
    }

    // PacketId::WHEEL_ANGLE
    if (!IsDoubleEqual(cache_it->second.wheel_angle_, object_state.info.wheel_data[0].h))
    {
        cache_it->second.wheel_angle_ = object_state.info.wheel_data[0].h;
        Write(PacketId::WHEEL_ANGLE, cache_it->second.wheel_angle_);
    }

    // PacketId::WHEEL_ROT
    if (!IsDoubleEqual(cache_it->second.wheel_rot_, object_state.info.wheel_data[0].p))
    {
        cache_it->second.wheel_rot_ = object_state.info.wheel_data[0].p;
        Write(PacketId::WHEEL_ROT, cache_it->second.wheel_rot_);
    }

    // PacketId::BOUNDING_BOX
    if (!IsBoundingBoxEqual(cache_it->second.bounding_box_, object_state.info.boundingbox))
    {
        cache_it->second.bounding_box_.x      = object_state.info.boundingbox.center_.x_;
        cache_it->second.bounding_box_.y      = object_state.info.boundingbox.center_.y_;
        cache_it->second.bounding_box_.z      = object_state.info.boundingbox.center_.z_;
        cache_it->second.bounding_box_.length = object_state.info.boundingbox.dimensions_.length_;
        cache_it->second.bounding_box_.width  = object_state.info.boundingbox.dimensions_.width_;
        cache_it->second.bounding_box_.height = object_state.info.boundingbox.dimensions_.height_;

        Write(PacketId::BOUNDING_BOX,
              cache_it->second.bounding_box_.x,
              cache_it->second.bounding_box_.y,
              cache_it->second.bounding_box_.z,
              cache_it->second.bounding_box_.length,
              cache_it->second.bounding_box_.width,
              cache_it->second.bounding_box_.height);
    }

    // PacketId::SCALE_MODE
    if (cache_it->second.scale_mode_ != object_state.info.scaleMode)
    {
        cache_it->second.scale_mode_ = object_state.info.scaleMode;
        Write(PacketId::SCALE_MODE, cache_it->second.scale_mode_);
    }

    // PacketId::VISIBILITY_MASK
    if (cache_it->second.visibility_mask_ != object_state.info.visibilityMask)
    {
        cache_it->second.visibility_mask_ = object_state.info.visibilityMask;
        Write(PacketId::VISIBILITY_MASK, cache_it->second.visibility_mask_);
    }

    // PacketId::NAME
    if (std::strcmp(cache_it->second.name_.c_str(), object_state.info.name) != 0)
    {
        cache_it->second.name_ = std::string(object_state.info.name);

        auto name_size = static_cast<unsigned int>(cache_it->second.name_.size());
        Write(PacketId::NAME, name_size, cache_it->second.name_);
    }

    // PacketId::ROAD_ID
    if (cache_it->second.road_id_ != object_state.pos.GetTrackId())
    {
        cache_it->second.road_id_ = object_state.pos.GetTrackId();
        Write(PacketId::ROAD_ID, cache_it->second.road_id_);
    }

    // PacketId::LANE_ID
    if (cache_it->second.lane_id_ != object_state.pos.GetLaneId())
    {
        cache_it->second.lane_id_ = object_state.pos.GetLaneId();
        Write(PacketId::LANE_ID, cache_it->second.lane_id_);
    }

    // PacketId::POS_OFFSET
    if (!IsDoubleEqual(cache_it->second.pos_offset_, object_state.pos.GetOffset()))
    {
        cache_it->second.pos_offset_ = object_state.pos.GetOffset();
        Write(PacketId::POS_OFFSET, cache_it->second.pos_offset_);
    }

    // PacketId::POS_T
    if (!IsDoubleEqual(cache_it->second.pos_t_, object_state.pos.GetT()))
    {
        cache_it->second.pos_t_ = object_state.pos.GetT();
        Write(PacketId::POS_T, cache_it->second.pos_t_);
    }

    // PacketId::POS_S
    if (!IsDoubleEqual(cache_it->second.pos_s_, object_state.pos.GetS()))
    {
        cache_it->second.pos_s_ = object_state.pos.GetS();
        Write(PacketId::POS_S, cache_it->second.pos_s_);
    }

    return 0;
}

template <typename... Data>
int Dat::DatLogger::Write(PacketId p_id, const Data&... data)
{
    // Write Time packet, but only once
    if (!timestamp_written_ && p_id != PacketId::DAT_HEADER)
    {
        WriteTimestamp();
    }

    size_t total_size = (SerializedSize(data) + ...);

    // Create the packet
    PacketGeneric packet;
    packet.header.id        = static_cast<id_t>(p_id);
    packet.header.data_size = static_cast<unsigned int>(total_size);
    packet.data.resize(packet.header.data_size);

    char* write_ptr = packet.data.data();
    (WriteToBuffer(write_ptr, data), ...);

    // Write to file
    if (p_id != PacketId::OBJ_ADDED && p_id != PacketId::OBJ_DELETED && p_id != PacketId::END_OF_SCENARIO)
    {
        WritePacket(packet);
    }

    return 0;
}

void Dat::DatLogger::WritePacket(PacketGeneric& packet)
{
    data_file_.write(reinterpret_cast<char*>(&packet.header), sizeof(PacketHeader));
    data_file_.write(packet.data.data(), static_cast<std::streamsize>(packet.data.size()));
}

void Dat::DatLogger::WriteTimestamp()
{
    PacketGeneric packet;
    packet.header.id        = static_cast<id_t>(PacketId::TIMESTAMP);
    packet.header.data_size = static_cast<unsigned int>(sizeof(object_state_cache_.timestamp_));
    packet.data.resize(packet.header.data_size);

    char* write_ptr = packet.data.data();
    WriteToBuffer(write_ptr, object_state_cache_.timestamp_);
    WritePacket(packet);

    timestamp_written_ = true;
}

bool Dat::DatLogger::IsFileOpen() const
{
    return data_file_.is_open();
}

void Dat::DatLogger::SetTimestampWritten(bool written)
{
    timestamp_written_ = written;
}

bool Dat::DatLogger::IsPoseEqual(const Pose& pose, const roadmanager::Position& pos) const
{
    return (pose.x == pos.GetX() && pose.y == pos.GetY() && pose.z == pos.GetZ() && pose.h == pos.GetH() && pose.p == pos.GetP() &&
            pose.r == pos.GetR());
}

bool Dat::DatLogger::IsBoundingBoxEqual(const BoundingBox& bb, const scenarioengine::OSCBoundingBox& osc_bb) const
{
    return (bb.x == static_cast<double>(osc_bb.center_.x_) && bb.y == static_cast<double>(osc_bb.center_.y_) &&
            bb.z == static_cast<double>(osc_bb.center_.z_) && bb.length == static_cast<double>(osc_bb.dimensions_.length_) &&
            bb.width == static_cast<double>(osc_bb.dimensions_.width_) && bb.height == static_cast<double>(osc_bb.dimensions_.height_));
}

size_t Dat::DatLogger::SerializedSize(const std::string& str)
{
    return str.size();
}

void Dat::DatLogger::WriteToBuffer(char*& write_ptr, const std::string& str)
{
    memcpy(write_ptr, str.data(), str.size());
    write_ptr += str.size();
}

template <typename T>
size_t Dat::DatLogger::SerializedSize(const T& val)
{
    return sizeof(val);
}

template <typename T>
void Dat::DatLogger::WriteToBuffer(char*& write_ptr, const T& val)
{
    memcpy(write_ptr, &val, sizeof(T));
    write_ptr += sizeof(T);
}