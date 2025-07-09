#pragma once

#include <fstream>
#include "CommonMini.hpp"

namespace Dat
{
    enum class PacketId
    {
        HEADER          = 0,
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

    struct PacketPose
    {
        double x;
        double y;
        double z;
        double h;
        double p;
        double r;
    };

    struct PacketBoundingBox
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

    struct ObjState
    {
        int               objId_          = -1;
        bool              active_         = false;
        bool              objWritten_     = false;  // denotes object added pkg written or not
        double            speed_          = SMALL_NUMBER;
        PacketPose        pos_            = {};
        int               modelId_        = -1;
        int               objType_        = -1;
        int               objCategory_    = -1;
        int               ctrlType_       = -1;
        double            wheelAngle_     = SMALL_NUMBER;
        double            wheelRot_       = SMALL_NUMBER;
        PacketBoundingBox boundingBox_    = {};
        int               scaleMode_      = -1;
        int               visibilityMask_ = -1;
        std::string       name_           = {};
        id_t              roadId_         = ID_UNDEFINED;
        int               laneId_         = -LARGE_NUMBER_INT;
        double            posOffset_      = SMALL_NUMBER;
        double            posT_           = SMALL_NUMBER;
        double            posS_           = SMALL_NUMBER;
    };

    struct ObjectStateCache
    {
        int                               timestamp;
        std::unordered_map<int, ObjState> obj_state_cache_;
    };

    class DatLogger
    {
    public:
        DatLogger()  = default;
        ~DatLogger() = default;

        int Init(const std::string& fileName, const std::string& odrName, const std::string& modelName);

    private:
        std::ofstream    data_file_;
        ObjectStateCache object_state_cache_;
    };

}  // namespace Dat