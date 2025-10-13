#pragma once

#include <fstream>
#include "CommonMini.hpp"

namespace scenarioengine
{
    struct ObjectState;
}

namespace Dat
{
    enum class PacketId : id_t
    {
        OBJ_ID          = 0,
        MODEL_ID        = 1,
        OBJ_TYPE        = 2,
        OBJ_CATEGORY    = 3,
        CTRL_TYPE       = 4,
        TIMESTAMP       = 5,
        NAME            = 6,
        SPEED           = 7,
        WHEEL_ANGLE     = 8,
        WHEEL_ROT       = 9,
        BOUNDING_BOX    = 10,
        SCALE_MODE      = 11,
        VISIBILITY_MASK = 12,
        POSE            = 13,
        ROAD_ID         = 14,
        LANE_ID         = 15,
        POS_OFFSET      = 16,
        POS_T           = 17,
        POS_S           = 18,
        OBJ_DELETED     = 19,
        OBJ_ADDED       = 20,
        DT              = 21,
        END_OF_SCENARIO = 22,
        PACKET_ID_SIZE  = 23,  // Keep this last
    };

    struct PacketString
    {
        unsigned int size;
        std::string  string;
    };

    struct DatHeader
    {
        unsigned int version_major;
        unsigned int version_minor;
        PacketString odr_filename;
        PacketString model_filename;
    };

    struct PacketHeader
    {
        id_t         id;
        unsigned int data_size;
    };

    struct Pose
    {
        float x;
        float y;
        float z;
        float h;
        float p;
        float r;
    };

    struct BoundingBox
    {
        float x;
        float y;
        float z;
        float length;
        float width;
        float height;
    };

    struct PacketGeneric
    {
        PacketHeader      header;
        std::vector<char> data;
    };

    struct ObjState  // Could this be ObjectStateStruct with some additional fields?
    {
        int         obj_id_      = -1;
        bool        active_      = false;
        bool        obj_written_ = false;  // denotes object added pkg written or not
        float       speed_       = LARGE_NUMBERF;
        Pose        pose_;
        int         model_id_        = -1;
        int         obj_type_        = -1;
        int         obj_category_    = -1;
        int         ctrl_type_       = -1;
        float       wheel_angle_     = LARGE_NUMBERF;
        float       wheel_rot_       = LARGE_NUMBERF;
        BoundingBox bounding_box_    = {};
        int         scale_mode_      = -1;
        int         visibility_mask_ = -1;
        std::string name_            = {};
        id_t        road_id_         = ID_UNDEFINED;
        int         lane_id_         = -LARGE_NUMBER_INT;
        float       pos_offset_      = LARGE_NUMBERF;
        float       pos_t_           = LARGE_NUMBERF;
        float       pos_s_           = LARGE_NUMBERF;
        float       dt_              = LARGE_NUMBERF;
    };

    struct ObjectStateCache  // Maybe rename to e.g. SimulationStateCache?
    {
        float                             dt_;
        float                             timestamp_;
        std::unordered_map<int, ObjState> state_;
    };

    class DatWriter
    {
    public:
        void           WritePacket(PacketGeneric& packet);
        int            WriteGenericDataToDat();
        int            WriteObjectStatesToDat(const std::vector<std::unique_ptr<scenarioengine::ObjectState>>& object_states);
        constexpr bool ShouldWriteObjId(PacketId p_id) const noexcept;

        size_t SerializedSize(const std::string& str);

        void WriteToBuffer(char*& write_ptr, const std::string& str);

        DatWriter();
        ~DatWriter();

        int Init(const std::string& file_name, const std::string& odr_name, const std::string& model_name);

        bool IsWriteFileOpen() const;
        void SetTimestampWritten(bool state);
        void SetObjectIdWritten(bool state);
        void SetSimulationTime(const double simulation_time);
        bool IsPoseEqual(const Pose& pose, const roadmanager::Position& pos) const;
        bool IsBoundingBoxEqual(const BoundingBox& bb, const scenarioengine::OSCBoundingBox& osc_bb) const;
        void ResetCurrentIds();
        void CheckDeletedObjects();
        void SetFixedTimestep(float fixed_timestep)  // Needed to set when merging datfiles
        {
            fixed_timestep_ = fixed_timestep;
        }

        /* Template definition kept in the header, otherwise symbols might not be resolved properly.
        Maybe it can be resolved during the build process somehow, but for now they are here. */

        template <typename... Data>
        int Write(PacketId p_id, const Data&... data)
        {
            /*
                Some recursion below, we'll try to write OBJ_ID, at which point we'll call this function again.
                Once inside the second call we'll try to Write timestamp and call function again.
                In the end, TIMESTAMP is written, then OBJ_ID, then data
            */

            // PacketId::OBJ_ID (we want to always write the object ID, with some exceptions)
            if (!object_id_written_ && ShouldWriteObjId(p_id))
            {
                object_id_written_ = true;
                Write(PacketId::OBJ_ID, current_object_id_);
            }

            // Write Time packet, but only once
            if (!timestamp_written_ && p_id != PacketId::END_OF_SCENARIO)
            {
                // Write with simulation time, object_states might be empty, then we have no time-reference
                timestamp_written_ = true;
                Write(PacketId::TIMESTAMP, static_cast<float>(simulation_time_));
            }

            size_t total_size = (SerializedSize(data) + ... + 0);  // +0 incase we want to write without data

            // Create the packet
            PacketGeneric packet;
            packet.header.id        = static_cast<id_t>(p_id);
            packet.header.data_size = static_cast<unsigned int>(total_size);
            packet.data.resize(packet.header.data_size);

            [[maybe_unused]] char* write_ptr = packet.data.data();
            (WriteToBuffer(write_ptr, data), ...);

            WritePacket(packet);

            return 0;
        }

        template <typename T>
        size_t SerializedSize(const T& val)
        {
            return sizeof(val);
        }

        template <typename T>
        void WriteToBuffer(char*& write_ptr, const T& val)
        {
            memcpy(write_ptr, &val, sizeof(T));
            write_ptr += sizeof(T);
        }

    private:
        std::ofstream           write_file_;
        ObjectStateCache        object_state_cache_;
        int                     current_object_id_ = -1;  // Current object ID being processed
        bool                    timestamp_written_ = false;
        bool                    object_id_written_ = false;
        double                  simulation_time_   = 0.0;
        std::unordered_set<int> previous_ids_;  // Keep track of object IDs
        std::unordered_set<int> current_ids_;   // Keep track of object IDs for the current state
        float                   fixed_timestep_ = -1.0f;
    };

    class DatReader
    {
    public:
        DatReader(const std::string& filename);
        ~DatReader() = default;

        int ReadStringPacket(std::string& str);

        int            FillDatHeader();
        Dat::DatHeader GetDatHeader() const
        {
            return header_;
        }

        void SetFileSize();
        bool ReadFile(Dat::PacketHeader& header);
        void UnknownPacket(const Dat::PacketHeader& header);
        void CloseFile();

        /* Template definition kept in the header, otherwise symbols might not be resolved properly.
        Maybe it can be resolved during the build process somehow, but for now they are here. */
        template <typename... Data>
        int ReadPacket(const Dat::PacketHeader& header, Data&... data)
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

    private:
        std::ifstream  file_;
        std::streampos file_size_;
        Dat::DatHeader header_;
    };

}  // namespace Dat