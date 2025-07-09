#define DAT_VERSION_MAJOR 1
#define DAT_VERSION_MINOR 0

#include "PacketHandler.hpp"

int Dat::DatLogger::Init(const std::string& fileName, const std::string& odrName, const std::string& modelName)
{
    data_file_.open(fileName, std::ios::binary);
    if (data_file_.fail())
    {
        // LOG_WARN("Cannot open file: {}", fileName);
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
    p_header.id        = static_cast<int>(PacketId::HEADER);
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