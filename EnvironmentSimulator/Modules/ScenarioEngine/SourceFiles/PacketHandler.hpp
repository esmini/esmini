#pragma once

#include <fstream>

namespace scenarioengine
{
    struct ObjectStateStruct;
}

namespace Dat
{
    enum class PacketId : id_t
    {
        DAT_HEADER      = 0,
        OBJ_ID          = 1,
        MODEL_ID        = 2,
        OBJ_TYPE        = 3,
        OBJ_CATEGORY    = 4,
        CTRL_TYPE       = 5,
        TIMESTAMP       = 6,
        NAME            = 7,
        SPEED           = 8,
        WHEEL_ANGLE     = 9,
        WHEEL_ROT       = 10,
        BOUNDING_BOX    = 11,
        SCALE_MODE      = 12,
        VISIBILITY_MASK = 13,
        POSE            = 14,
        ROAD_ID         = 15,
        LANE_ID         = 16,
        POS_OFFSET      = 17,
        POS_T           = 18,
        POS_S           = 19,
        OBJ_DELETED     = 20,
        OBJ_ADDED       = 21,
        END_OF_SCENARIO = 22,
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
        double x;
        double y;
        double z;
        double h;
        double p;
        double r;
    };

    struct BoundingBox
    {
        double x;
        double y;
        double z;
        double length;
        double width;
        double height;
    };

    struct PacketGeneric
    {
        PacketHeader      header;
        std::vector<char> data;
    };

    struct ObjState  // Could this be ObjectStateStruct with some additional fields?
    {
        int         obj_id_          = -1;
        bool        active_          = false;
        bool        obj_written_     = false;  // denotes object added pkg written or not
        double      speed_           = SMALL_NUMBER;
        Pose        pose_            = {};
        int         model_id_        = -1;
        int         obj_type_        = -1;
        int         obj_category_    = -1;
        int         ctrl_type_       = -1;
        double      wheel_angle_     = SMALL_NUMBER;
        double      wheel_rot_       = SMALL_NUMBER;
        BoundingBox bounding_box_    = {};
        int         scale_mode_      = -1;
        int         visibility_mask_ = -1;
        std::string name_            = {};
        id_t        road_id_         = ID_UNDEFINED;
        int         lane_id_         = -LARGE_NUMBER_INT;
        double      pos_offset_      = SMALL_NUMBER;
        double      pos_t_           = SMALL_NUMBER;
        double      pos_s_           = SMALL_NUMBER;
    };

    struct ObjectStateCache
    {
        double                            timestamp_;
        std::unordered_map<int, ObjState> state_;
    };

    class DatLogger
    {
    public:
        template <typename... Data>
        int  Write(PacketId p_id, const Data&... data);
        void WritePacket(PacketGeneric& packet);
        int  WriteToDat(const scenarioengine::ObjectStateStruct& object_state);

        size_t SerializedSize(const std::string& str);

        template <typename T>
        size_t SerializedSize(const T& val);

        template <typename T>
        void WriteToBuffer(char*& write_ptr, const T& val);

        void WriteToBuffer(char*& write_ptr, const std::string& str);

        template <typename... Data>
        int ReadPacket(const Dat::PacketHeader& header, Data&... data);

        DatLogger()  = default;
        ~DatLogger() = default;

        int Init(const std::string& file_name, const std::string& odr_name, const std::string& model_name);

        bool IsWriteFileOpen() const;
        void SetTimestampWritten(bool state);
        void SetObjectIdWritten(bool state);
        bool IsPoseEqual(const Pose& pose, const roadmanager::Position& pos) const;
        bool IsBoundingBoxEqual(const BoundingBox& bb, const scenarioengine::OSCBoundingBox& osc_bb) const;

    private:
        std::ofstream    write_file_;
        ObjectStateCache object_state_cache_;
        int              current_object_id_ = -1;  // Current object ID being processed
        bool             timestamp_written_ = false;
        bool             object_id_written_ = false;
    };

}  // namespace Dat