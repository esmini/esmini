#define DAT_VERSION_MAJOR 1
#define DAT_VERSION_MINOR 0

#include "ScenarioGateway.hpp"
#include "CommonMini.cpp"
#include "PacketHandler.hpp"

int Dat::DatLogger::Init(const std::string& fileName, const std::string& odrName, const std::string& modelName)
{
    data_file_.open(fileName, std::ios::binary);
    if (data_file_.fail())
    {
        LOG_ERROR("Cannot open file: {}", fileName);
        return -1;
    }

    DatHeader d_header;
    d_header.version_major = DAT_VERSION_MAJOR;
    d_header.version_minor = DAT_VERSION_MINOR;

    d_header.odr_filename.size   = static_cast<unsigned int>(odrName.size());
    d_header.odr_filename.string = odrName;

    d_header.model_filename.size   = static_cast<unsigned int>(modelName.size());
    d_header.model_filename.string = modelName;

    PacketHeader p_header;
    p_header.id        = static_cast<id_t>(PacketId::DAT_HEADER);
    p_header.data_size = static_cast<unsigned int>(sizeof(d_header.version_major) + sizeof(d_header.version_minor)) +
                         static_cast<unsigned int>(sizeof(d_header.odr_filename.size)) + d_header.odr_filename.size +
                         static_cast<unsigned int>(sizeof(d_header.model_filename.size)) + d_header.model_filename.size;

    // Write content
    data_file_.write(reinterpret_cast<char*>(&p_header), sizeof(PacketHeader));

    // version
    data_file_.write(reinterpret_cast<char*>(&d_header.version_major), sizeof(d_header.version_major));
    data_file_.write(reinterpret_cast<char*>(&d_header.version_minor), sizeof(d_header.version_minor));

    // odr filename
    data_file_.write(reinterpret_cast<char*>(&d_header.odr_filename.size), sizeof(d_header.odr_filename.size));
    data_file_.write(d_header.odr_filename.string.data(), d_header.odr_filename.size);

    // model filename
    data_file_.write(reinterpret_cast<char*>(&d_header.model_filename.size), sizeof(d_header.model_filename.size));
    data_file_.write(d_header.model_filename.string.data(), d_header.model_filename.size);

    return 0;
}

template <typename... Args>
int Dat::DatLogger::Write(PacketId p_id, const Args&... data)
{
    // Write Time packet, but only once
    if (!timestamp_written_)
    {
        WriteTimestamp();
    }

    // Create the packet
    PacketGeneric packet;
    packet.header.id        = static_cast<id_t>(p_id);
    packet.header.data_size = static_cast<unsigned int>((sizeof(data) + ...));
    packet.data.resize(packet.header.data_size);

    char* write_ptr = packet.data.data();
    (..., (memcpy(write_ptr, &data, sizeof(data)), write_ptr += sizeof(data)));

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
    memcpy(packet.data.data(), &object_state_cache_.timestamp_, sizeof(object_state_cache_.timestamp_));

    WritePacket(packet);

    timestamp_written_ = true;
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

    return 0;
}