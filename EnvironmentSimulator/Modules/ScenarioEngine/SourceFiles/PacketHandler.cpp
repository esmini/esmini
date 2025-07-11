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

    auto cache_it = object_state_cache_.state_.find(object_state.info.id);
    if (cache_it == object_state_cache_.state_.end())
    {
        // New object state, add it to the cache
        ObjState obj_state;
        obj_state.objId_                                 = object_state.info.id;
        object_state_cache_.state_[object_state.info.id] = obj_state;
        cache_it                                         = object_state_cache_.state_.find(object_state.info.id);
    }

    if (!IsDoubleEqual(cache_it->second.speed_, object_state.info.speed))
    {
        cache_it->second.speed_ = object_state.info.speed;
        Write(PacketId::SPEED, cache_it->second.speed_);
    }

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