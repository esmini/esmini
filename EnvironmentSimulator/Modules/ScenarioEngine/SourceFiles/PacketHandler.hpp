#pragma once

#include <fstream>
#include "CommonMini.hpp"
#include "RoadManager.hpp"
#include "ScenarioEngine.hpp"

#define DAT_FILE_FORMAT_VERSION_MAJOR 4
#define DAT_FILE_FORMAT_VERSION_MINOR 3

namespace scenarioengine
{
    class ObjectState;
}

namespace Dat
{
    enum class PacketId : id_t
    {
        OBJ_ID            = 0,
        MODEL_ID          = 1,
        OBJ_TYPE          = 2,
        OBJ_CATEGORY      = 3,
        CTRL_TYPE         = 4,
        TIMESTAMP         = 5,
        NAME              = 6,
        SPEED             = 7,
        WHEEL_ANGLE       = 8,
        WHEEL_ROT         = 9,
        BOUNDING_BOX      = 10,
        SCALE_MODE        = 11,
        VISIBILITY_MASK   = 12,
        POSE              = 13,
        ROAD_ID           = 14,
        LANE_ID           = 15,
        POS_OFFSET        = 16,
        POS_T             = 17,
        POS_S             = 18,
        OBJ_DELETED       = 19,
        OBJ_ADDED         = 20,
        DT                = 21,
        END_OF_SCENARIO   = 22,
        TRAFFIC_LIGHT     = 23,
        REFPOINT_X_OFFSET = 24,
        MODEL_X_OFFSET    = 25,
        OBJ_MODEL3D       = 26,
        ELEM_STATE_CHANGE = 27,
        SHAPE_2D_OUTLINE  = 28,
        PACKET_ID_SIZE    = 29  // Keep this last
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
        PacketString git_rev;
    };

    struct PacketHeader
    {
        id_t         id;
        unsigned int data_size;
    };

    struct Pose
    {
        float x = std::nanf("");
        float y = std::nanf("");
        float z = std::nanf("");
        float h = std::nanf("");
        float p = std::nanf("");
        float r = std::nanf("");
    };

    struct BoundingBox
    {
        float x      = std::nanf("");
        float y      = std::nanf("");
        float z      = std::nanf("");
        float length = std::nanf("");
        float width  = std::nanf("");
        float height = std::nanf("");
    };

    struct TrafficLightLamp
    {
        int          traffic_light_id = -1;
        unsigned int lamp_id          = ID_UNDEFINED;
        unsigned int lamp_idx         = IDX_UNDEFINED;
        int          lamp_mode        = static_cast<int>(roadmanager::Signal::LampMode::MODE_UNDEFINED);
    };

    struct PacketGeneric
    {
        PacketHeader      header;
        std::vector<char> data;
    };

    struct PacketShape2DOutline
    {
        std::vector<SE_Point2D> points;
    };

    struct ObjState  // Could this be ObjectStateStruct with some additional fields?
    {
        int                     obj_id_            = -1;
        bool                    active_            = false;
        float                   speed_             = std::nanf("");
        Pose                    pose_              = {};
        int                     model_id_          = -1;
        int                     obj_type_          = -1;
        int                     obj_category_      = -1;
        int                     ctrl_type_         = -1;
        float                   wheel_angle_       = std::nanf("");
        float                   wheel_rot_         = std::nanf("");
        BoundingBox             bounding_box_      = {};
        int                     scale_mode_        = -1;
        int                     visibility_mask_   = -1;
        std::string             name_              = {};
        id_t                    road_id_           = ID_UNDEFINED;
        int                     lane_id_           = -LARGE_NUMBER_INT;
        float                   pos_offset_        = std::nanf("");
        float                   pos_t_             = std::nanf("");
        float                   pos_s_             = std::nanf("");
        float                   refpoint_x_offset_ = std::nanf("");
        float                   model_x_offset_    = std::nanf("");
        std::string             model3d_           = {};
        std::vector<SE_Point2D> outline_2d         = {};
    };

    struct ObjectStateCache  // Maybe rename to e.g. SimulationStateCache?
    {
        double                                     dt_ = LARGE_NUMBER;
        double                                     timestamp_;
        std::unordered_map<id_t, TrafficLightLamp> traffic_lights_lamps_;
        std::unordered_map<int, ObjState>          state_;
    };

    class DatWriter
    {
    public:
        void           WritePacket(PacketGeneric& packet);
        int            WriteDtToDat();
        int            WriteTrafficLightsToDat(const std::vector<roadmanager::Signal*>& dynamic_signals);
        int            WriteStoryBoardStateChangesToDat(const std::vector<std::string>& state_changes);
        int            WriteObjectStatesToDat(const std::vector<scenarioengine::Object*>& objects);
        constexpr bool ShouldWriteObjId(PacketId p_id) const noexcept;

        size_t SerializedSize(const std::string& str);

        void WriteToBuffer(char*& write_ptr, const std::string& str);

        DatWriter() = default;
        ~DatWriter();

        int Init(const std::string& file_name, const std::string& odr_name, const std::string& model_name, const std::string& git_rev);

        bool IsWriteFileOpen() const;
        void SetTimestampWritten(bool state);
        void SetObjectIdWritten(bool state);
        void SetSimulationTime(const double simulation_time, const double dt);
        bool IsPoseEqual(const Pose& pose, const roadmanager::Position& pos) const;
        bool IsBoundingBoxEqual(const BoundingBox& bb, const scenarioengine::OSCBoundingBox& osc_bb) const;
        void ResetCurrentIds();
        void CheckDeletedObjects();

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
                Write(PacketId::TIMESTAMP, simulation_time_);
            }

            size_t total_size = (SerializedSize(data) + ... + 0);  // +0 incase we want to write without data

            LOG_DEBUG("Writing packet id {} size {}", p_id, total_size);

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
        void ReWriteToBuffer(PacketGeneric& pkt, const T& value)
        {
            pkt.header.data_size = sizeof(T);
            pkt.data.resize(sizeof(T));
            std::memcpy(pkt.data.data(), &value, sizeof(T));
        }

        size_t SerializedSize(const PacketString& p)
        {
            return sizeof(p.size) + p.string.size();
        }

        size_t SerializedSize(const PacketShape2DOutline& p)
        {
            return p.points.size() * sizeof(SE_Point2D);
        }

        template <typename T>
        size_t SerializedSize(const T& val)
        {
            return sizeof(val);
        }

        void WriteToBuffer(char*& write_ptr, const PacketString& p)
        {
            memcpy(write_ptr, &p.size, sizeof(p.size));
            write_ptr += sizeof(p.size);

            memcpy(write_ptr, p.string.data(), p.string.size());
            write_ptr += p.string.size();
        }

        void WriteToBuffer(char*& write_ptr, const PacketShape2DOutline& p)
        {
            memcpy(write_ptr, p.points.data(), p.points.size() * sizeof(SE_Point2D));
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
        double                  dt_ = -1.0;
    };

    class DatReader
    {
    public:
        DatReader(const std::string& filename);
        DatReader() = default;
        ~DatReader();

        std::string ReadStringPacket(const Dat::PacketGeneric& pkt);
        int         ReadStringPacket(std::string& str);

        std::vector<SE_Point2D> ReadOutlinePacket(const Dat::PacketGeneric& pkt);
        int                     ReadOutlinePacket(std::vector<SE_Point2D>& points);

        int            FillDatHeader(bool quiet = false);
        Dat::DatHeader GetDatHeader() const
        {
            return header_;
        }

        void               SetFileSize();
        bool               ReadFile(Dat::PacketHeader& header);
        void               SkipPacket(const Dat::PacketHeader& header);
        void               CloseFile();
        Dat::PacketGeneric CreateGenericPacket(const Dat::PacketHeader& header)
        {
            Dat::PacketGeneric packet;
            packet.header = header;
            packet.data.resize(header.data_size);

            file_.read(packet.data.data(), packet.header.data_size);
            if (file_.gcount() != packet.header.data_size && !file_.eof())
            {
                LOG_ERROR("Failed to create generic packet");
            }

            return packet;
        }

        /* Template definition kept in the header, otherwise symbols might not be resolved properly.
        Maybe it can be resolved during the build process somehow, but for now they are here. */
        template <typename... Data>
        int ReadPacket(const Dat::PacketGeneric& gp, Data&... data)
        {
            const char* read_ptr        = gp.data.data();
            const char* end_ptr         = read_ptr + gp.header.data_size;
            bool        exceeded_bounds = false;

            // A lambda that safely copies data from the read_ptr to the provided field,
            // ensuring we don't read out of bounds.
            // If we send in too little data (data < header.size), we just silently ignore the rest of the data.
            // If we exceed bounds (header.size > data), we set the input field to default value.
            auto safe_copy = [&](auto& field)
            {
                if (read_ptr + sizeof(field) <= end_ptr)
                {
                    memcpy(&field, read_ptr, sizeof(field));
                    read_ptr += sizeof(field);
                }
                else
                {
                    field           = {};  // Initialized to default value
                    exceeded_bounds = true;
                }
            };

            (safe_copy(data), ...);

            size_t remaining_data = static_cast<size_t>(end_ptr - read_ptr);

            // Give an error for the packet if we have remaining data or exceeded bounds
            if (remaining_data > 0)
            {
                LOG_DEBUG("Unused data remaining in packet with id: {}", gp.header.id);
            }
            else if (exceeded_bounds)
            {
                LOG_DEBUG("Reading previous partial version of packet id: {}", gp.header.id);
            }

            return 0;
        }

        bool ReadHeader(PacketHeader& hdr)
        {
            file_.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
            return file_.good();
        }

    private:
        std::string    file_name_;
        std::ifstream  file_;
        std::streampos file_size_;
        Dat::DatHeader header_;
    };

}  // namespace Dat
