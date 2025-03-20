/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once

#include <fstream>
#include <vector>

#include "CommonMini.hpp"
#include "Entities.hpp"

namespace dat
{
    enum class PackageId
    {
        HEADER          = 11,
        TIME_SERIES     = 12,
        OBJ_ID          = 13,
        MODEL_ID        = 14,
        POSITIONS       = 15,
        SPEED           = 16,
        OBJ_TYPE        = 17,
        OBJ_CATEGORY    = 18,
        CTRL_TYPE       = 19,
        WHEEL_ANGLE     = 20,
        WHEEL_ROT       = 21,
        BOUNDING_BOX    = 22,
        SCALE_MODE      = 23,
        VISIBILITY_MASK = 24,
        NAME            = 25,
        ROAD_ID         = 26,
        LANE_ID         = 27,
        POS_OFFSET      = 28,
        POS_T           = 29,
        POS_S           = 30,
        LIGHT_STATES    = 31,
        OBJ_DELETED     = 32,
        OBJ_ADDED       = 33,
        END_OF_SCENARIO = 34
    };

    // mandatory packages
    struct CommonPkgHdr
    {
        id_t         id;
        unsigned int content_size;
    };

    // common package types
    struct CommonString
    {
        unsigned int      size;  // size of the string
        std::vector<char> string;
    };

    // specific packages
    struct DatHdr
    {
        int          version;
        CommonString odrFilename;
        CommonString modelFilename;
    };

    struct Pos
    {
        double x = SMALL_NUMBER;
        double y = SMALL_NUMBER;
        double z = SMALL_NUMBER;
        double h = SMALL_NUMBER;
        double p = SMALL_NUMBER;
        double r = SMALL_NUMBER;
    };

    struct BoundingBox
    {
        float x      = 0.0;
        float y      = 0.0;
        float z      = 0.0;
        float width  = 0.0;
        float length = 0.0;
        float height = 0.0;
    };

    struct Rgb_value
    {
        unsigned char rgb[13 * 4];
    };

    struct Rgb_old
    {
        unsigned char* rgb[13 * 4];
    };

    struct LightRGB
    {
        unsigned char red       = 255;
        unsigned char green     = 255;
        unsigned char blue      = 255;
        unsigned char intensity = 255;
    };

    struct LightState
    {
        LightRGB day_time_running_lights;
        LightRGB low_beam;
        LightRGB high_beam;
        LightRGB fog_lights_front;
        LightRGB fog_lights_rear;
        LightRGB brake_lights;
        LightRGB indicator_left;
        LightRGB indicator_right;
        LightRGB reversing_lights;
        LightRGB license_plater_illumination;
        LightRGB special_purpose_lights;
        LightRGB fog_lights;
        LightRGB warning_lights;
    };

    struct CommonPkg
    {
        CommonPkgHdr      hdr;
        std::vector<char> content;
    };

    struct ObjState
    {
        int         objId_  = -1;
        bool        active_ = false;
        double      speed_  = SMALL_NUMBER;
        Pos         pos_;
        int         modelId_     = -1;
        int         objType_     = -1;
        int         objCategory_ = -1;
        int         ctrlType_    = -1;
        double      wheelAngle_  = SMALL_NUMBER;
        double      wheelRot_    = SMALL_NUMBER;
        BoundingBox boundingBox_;
        int         scaleMode_      = -1;
        int         visibilityMask_ = -1;
        std::string name_;
        id_t        roadId_    = ID_UNDEFINED;
        int         laneId_    = -LARGE_NUMBER_INT;
        double      posOffset_ = SMALL_NUMBER;
        double      posT_      = SMALL_NUMBER;
        double      posS_      = SMALL_NUMBER;
        LightState  lightStates_;
    };

    struct CompleteObjectState
    {
        double                            time_ = SMALL_NUMBER;  // keeps track of time of last update of the object states
        std::unordered_map<int, ObjState> obj_states_;
    };

    struct ObjIdAdded
    {
        int  id;
        bool status = false;
    };

    class DatLogger
    {
    private:
        std::ofstream data_file_;

    public:
        DatLogger() = default;
        ~DatLogger();

        bool   TimePkgAdded_  = false;
        bool   ObjIdPkgAdded_ = false;
        double simTimeTemp_   = SMALL_NUMBER;  // keeps track of time of simulation

        CompleteObjectState     completeObjectState_;
        std::vector<ObjIdAdded> objIdAdded_;

        int  Init(const std::string& fileName, const std::string& odrName, const std::string& modelName);
        void DeleteObjState(int objId);

        void writePackage(CommonPkg package);  // will just write package
        void WriteStringPkg(std::string name, PackageId pkg_id);
        void WriteMandatoryPkg(int obj_id);
        void AddObject(int obj_id);
        void DeleteObject();
        bool IsObjIdAddPkgWritten(int id);
        void SetObjIdAddPkgWritten(int id, bool status);
        bool IsFileOpen();

        void WriteObjSpeed(int obj_id, double speed);
        void WriteTime(double t);
        void WriteObjPos(int obj_id, double x, double y, double z, double h, double p, double r);
        void WriteObjId(int obj_id);
        void WriteModelId(int obj_id, int model_id);
        void WriteObjType(int obj_id, int obj_type);
        void WriteObjCategory(int obj_id, int obj_category);
        void WriteCtrlType(int obj_id, int ctrl_type);

        void WriteWheelAngle(int obj_id, double angle);
        void WriteWheelRot(int obj_id, double rot);
        void WriteBB(int obj_id, float x, float y, float z, float length, float width, float height);
        void WriteScaleMode(int obj_id, int mode);
        void WriteVisiblityMask(int obj_id, int mask);
        void WriteName(int obj_id, std::string name);
        void WriteRoadId(int obj_id, id_t road_id);
        void WriteLaneId(int obj_id, int lane_id);
        void WritePosOffset(int obj_id, double pos_offset);
        void WritePosT(int obj_id, double t);
        void WritePosS(int obj_id, double s);
        void WriteLightState(int obj_id, LightState rgb_data);
        template <typename T>
        void WriteObjectData(int obj_id, T value, PackageId package_id);

        std::string pkgIdTostring(PackageId id);

        void CountPkg(PackageId id);
        void PrintPkgCount();
        int  speedPkgCount          = 0;
        int  posPkgCount            = 0;
        int  timePkgCount           = 0;
        int  objIdPkgCount          = 0;
        int  modelIdPkgCount        = 0;
        int  objTypePkgCount        = 0;
        int  objCategoryPkgCount    = 0;
        int  ctrlTypePkgCount       = 0;
        int  wheelAnglePkgCount     = 0;
        int  wheelRotPkgCount       = 0;
        int  bbPkgCount             = 0;
        int  scaleModePkgCount      = 0;
        int  visibilityMaskPkgCount = 0;
        int  namePkgCount           = 0;
        int  roadIdPkgCount         = 0;
        int  laneIdPkgCount         = 0;
        int  posOffsetPkgCount      = 0;
        int  posTPkgCount           = 0;
        int  posSPkgCount           = 0;
        int  lightStatePkgCount     = 0;
        int  objDeletedPkgCount     = 0;
        int  objAddedPkgCount       = 0;
        int  endOfScenarioPkgCount  = 0;
        int  headerPkgCount         = 0;
    };

}  // namespace dat