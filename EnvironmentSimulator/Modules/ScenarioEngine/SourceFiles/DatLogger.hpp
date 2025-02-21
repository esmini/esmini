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

namespace datLogger
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

    struct Time
    {
        double time = SMALL_NUMBER;
    };

    struct ObjId
    {
        int obj_id = -1;
    };
    struct ModelId
    {
        int model_id = -1;
    };

    struct ObjType
    {
        int obj_type = -1;
    };

    struct ObjCategory
    {
        int obj_category = -1;
    };

    struct CtrlType
    {
        int ctrl_type = -1;
    };

    struct WheelAngle
    {
        double wheel_angle = SMALL_NUMBER;
    };

    struct WheelRot
    {
        double wheel_rot = SMALL_NUMBER;
    };

    struct ScaleMode
    {
        int scale_mode = -1;
    };
    struct VisibilityMask
    {
        int visibility_mask = -1;
    };

    struct RoadId
    {
        id_t road_id = ID_UNDEFINED;
    };

    struct LaneId
    {
        int lane_id = -LARGE_NUMBER_INT;
    };

    struct PosOffset
    {
        double offset = SMALL_NUMBER;
    };

    struct PosT
    {
        double t = SMALL_NUMBER;
    };

    struct PosS
    {
        double s = SMALL_NUMBER;
    };

    struct Name
    {
        std::vector<char> string;
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

    struct Speed
    {
        double speed_ = SMALL_NUMBER;
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
        ObjId          obj_id_;
        bool           active = false;
        Speed          speed_;
        Pos            pos_;
        ModelId        modelId_;
        ObjType        objType_;
        ObjCategory    objCategory_;
        CtrlType       ctrlType_;
        WheelAngle     wheelAngle_;
        WheelRot       wheelRot_;
        BoundingBox    boundingBox_;
        ScaleMode      scaleMode_;
        VisibilityMask visibilityMask_;
        std::string    name_;
        RoadId         roadId_;
        LaneId         laneId_;
        PosOffset      posOffset_;
        PosT           posT;
        PosS           posS;
        LightState     lightStates_;
    };

    struct CompleteObjectState
    {
        Time                  time;  // keeps track of time of last update of the object states
        std::vector<ObjState> obj_states;
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
        ~DatLogger()
        {
            if (IsFileOpen())
            {
                WriteTime(simTimeTemp);
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::END_OF_SCENARIO);
                pkg.hdr.content_size = 0;
                pkg.content.resize(pkg.hdr.content_size);
                writePackage(pkg);

                data_file_.flush();
                data_file_.close();
            }
        }

        bool   isFirstEntry  = true;
        bool   notFirstEnd   = false;
        bool   TimePkgAdded  = false;
        bool   ObjIdPkgAdded = false;
        double simTimeTemp   = SMALL_NUMBER;  // keeps track of time of simulation

        CompleteObjectState     completeObjectState;
        std::vector<ObjIdAdded> objIdAdded_;

        int  init(std::string fileName, int ver, std::string odrName, std::string modelName);
        void DeleteObjState(int objId);

        void writePackage(CommonPkg package);  // will just write package
        void WriteStringPkg(std::string name, PackageId pkg_id);
        void WriteManPkg(int obj_id);
        int  AddObject(int obj_id);
        int  deleteObject();
        bool IsObjIdAddPkgWritten(int id);
        void SetObjIdAddPkgWritten(int id, bool status);
        bool IsFileOpen()
        {
            return data_file_.is_open();
        }

        int WriteObjSpeed(int obj_id, double speed);
        int WriteTime(double t);
        int WriteObjPos(int obj_id, double x, double y, double z, double h, double p, double r);
        int WriteObjId(int obj_id);
        int WriteModelId(int obj_id, int model_id);
        int WriteObjType(int obj_id, int obj_type);
        int WriteObjCategory(int obj_id, int obj_category);
        int WriteCtrlType(int obj_id, int ctrl_type);

        int  WriteWheelAngle(int obj_id, double angle);
        int  WriteWheelRot(int obj_id, double rot);
        int  WriteBB(int obj_id, float x, float y, float z, float length, float width, float height);
        int  WriteScaleMode(int obj_id, int mode);
        int  WriteVisiblityMask(int obj_id, int mask);
        int  WriteName(int obj_id, std::string name);
        int  WriteRoadId(int obj_id, id_t road_id);
        int  WriteLaneId(int obj_id, int lane_id);
        int  WritePosOffset(int obj_id, double pos_offset);
        int  WritePosT(int obj_id, double t);
        int  WritePosS(int obj_id, double s);
        void WriteLightState(int obj_id, LightState rgb_data);

        std::string pkgIdTostring(PackageId id);
    };

}  // namespace datLogger