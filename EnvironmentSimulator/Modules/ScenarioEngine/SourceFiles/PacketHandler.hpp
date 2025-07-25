#pragma once

#include <fstream>

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
        END_OF_SCENARIO = 21,
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
        float       speed_       = SMALL_NUMBERF;
        Pose        pose_;
        int         model_id_        = -1;
        int         obj_type_        = -1;
        int         obj_category_    = -1;
        int         ctrl_type_       = -1;
        float       wheel_angle_     = SMALL_NUMBERF;
        float       wheel_rot_       = SMALL_NUMBERF;
        BoundingBox bounding_box_    = {};
        int         scale_mode_      = -1;
        int         visibility_mask_ = -1;
        std::string name_            = {};
        id_t        road_id_         = ID_UNDEFINED;
        int         lane_id_         = -LARGE_NUMBER_INT;
        float       pos_offset_      = SMALL_NUMBERF;
        float       pos_t_           = SMALL_NUMBERF;
        float       pos_s_           = SMALL_NUMBERF;
    };

    struct ObjectStateCache
    {
        float                             timestamp_;
        std::unordered_map<int, ObjState> state_;
    };

    class DatLogger
    {
    public:
        template <typename... Data>
        int  Write(PacketId p_id, const Data&... data);
        void WritePacket(PacketGeneric& packet);
        int  WriteToDat(const std::vector<std::unique_ptr<scenarioengine::ObjectState>>& object_states);

        size_t SerializedSize(const std::string& str);

        template <typename T>
        size_t SerializedSize(const T& val);

        template <typename T>
        void WriteToBuffer(char*& write_ptr, const T& val);

        void WriteToBuffer(char*& write_ptr, const std::string& str);

        template <typename... Data>
        int ReadPacket(const Dat::PacketHeader& header, Data&... data);

        DatLogger() = default;
        ~DatLogger();

        int Init(const std::string& file_name, const std::string& odr_name, const std::string& model_name);

        bool IsWriteFileOpen() const;
        void SetTimestampWritten(bool state);
        void SetObjectIdWritten(bool state);
        void SetSimulationTime(const double simulation_time);
        bool IsPoseEqual(const Pose& pose, const roadmanager::Position& pos) const;
        bool IsBoundingBoxEqual(const BoundingBox& bb, const scenarioengine::OSCBoundingBox& osc_bb) const;
        void ResetCurrentIds();
        void CheckDeletedObjects();

    private:
        std::ofstream           write_file_;
        ObjectStateCache        object_state_cache_;
        int                     current_object_id_ = -1;  // Current object ID being processed
        bool                    timestamp_written_ = false;
        bool                    object_id_written_ = false;
        double                  simulation_time_   = 0.0;
        std::unordered_set<int> previous_ids_;  // Keep track of object IDs
        std::unordered_set<int> current_ids_;   // Keep track of object IDs for the current state
    };

}  // namespace Dat