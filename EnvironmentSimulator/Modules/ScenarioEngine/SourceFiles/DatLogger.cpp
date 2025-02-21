// logDat

#include <fstream>
#include <iostream>
#include <cstring>
#include <chrono>
#include <cmath>

#include "DatLogger.hpp"
#include "CommonMini.hpp"

#define NUMBER_OF_VEHICLE_LIGHTS (13)

using namespace datLogger;

void DatLogger::WriteManPkg(int obj_id)
{
    if (!TimePkgAdded)
    {
        WriteTime(simTimeTemp);
        TimePkgAdded = true;
    }
    if (!ObjIdPkgAdded)
    {
        WriteObjId(obj_id);
        ObjIdPkgAdded = true;
    }
}

bool DatLogger::IsObjIdAddPkgWritten(int id)
{
    bool status = false;
    for (size_t i = 0; i < objIdAdded_.size(); i++)
    {
        if (objIdAdded_[i].id == id)
        {
            if (objIdAdded_[i].status == true)
            {
                status = true;
                break;
            }
        }
    }
    return status;
}

void DatLogger::SetObjIdAddPkgWritten(int id, bool status)
{
    for (size_t i = 0; i < objIdAdded_.size(); i++)
    {
        if (objIdAdded_[i].id == id)
        {
            objIdAdded_[i].status = status;
            break;
        }
    }
}

int DatLogger::WriteObjSpeed(int obj_id, double speed)
{
    for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
    {
        if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
        {
            continue;
        }
        completeObjectState.obj_states[i].active = true;
        if (!isEqualDouble(completeObjectState.obj_states[i].speed_.speed_, speed))
        {
            WriteManPkg(obj_id);

            // create pkg
            CommonPkg pkg;
            pkg.hdr.id           = static_cast<int>(PackageId::SPEED);
            pkg.hdr.content_size = sizeof(speed);
            pkg.content.resize(pkg.hdr.content_size);
            std::memcpy(pkg.content.data(), &speed, sizeof(speed));
            writePackage(pkg);
            completeObjectState.obj_states[i].speed_.speed_ = speed;
            break;
        }
    }
    return 0;
}

int DatLogger::WriteModelId(int obj_id, int model_id)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].modelId_.model_id != model_id)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::MODEL_ID);
                pkg.hdr.content_size = sizeof(model_id);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &model_id, sizeof(model_id));
                writePackage(pkg);
                completeObjectState.obj_states[i].modelId_.model_id = model_id;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteObjType(int obj_id, int obj_type)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].objType_.obj_type != obj_type)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::OBJ_TYPE);
                pkg.hdr.content_size = sizeof(obj_type);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &obj_type, sizeof(obj_type));
                writePackage(pkg);
                completeObjectState.obj_states[i].objType_.obj_type = obj_type;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteObjCategory(int obj_id, int obj_category)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].objCategory_.obj_category != obj_category)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::OBJ_CATEGORY);
                pkg.hdr.content_size = sizeof(obj_category);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &obj_category, sizeof(obj_category));
                writePackage(pkg);
                completeObjectState.obj_states[i].objCategory_.obj_category = obj_category;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteCtrlType(int obj_id, int ctrl_type)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].ctrlType_.ctrl_type != ctrl_type)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::CTRL_TYPE);
                pkg.hdr.content_size = sizeof(ctrl_type);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &ctrl_type, sizeof(ctrl_type));
                writePackage(pkg);
                completeObjectState.obj_states[i].ctrlType_.ctrl_type = ctrl_type;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteWheelAngle(int obj_id, double angle)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (!isEqualDouble(completeObjectState.obj_states[i].wheelAngle_.wheel_angle, angle))
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::WHEEL_ANGLE);
                pkg.hdr.content_size = sizeof(angle);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &angle, sizeof(angle));
                writePackage(pkg);
                completeObjectState.obj_states[i].wheelAngle_.wheel_angle = angle;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteWheelRot(int obj_id, double rot)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (!isEqualDouble(completeObjectState.obj_states[i].wheelRot_.wheel_rot, rot))
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::WHEEL_ROT);
                pkg.hdr.content_size = sizeof(rot);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &rot, sizeof(rot));
                writePackage(pkg);
                completeObjectState.obj_states[i].wheelRot_.wheel_rot = rot;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteBB(int obj_id, float x, float y, float z, float length, float width, float height)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].boundingBox_.x != x || completeObjectState.obj_states[i].boundingBox_.y != y ||
                completeObjectState.obj_states[i].boundingBox_.z != z || completeObjectState.obj_states[i].boundingBox_.height != height ||
                completeObjectState.obj_states[i].boundingBox_.length != length || completeObjectState.obj_states[i].boundingBox_.width != width)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg   pkg;
                BoundingBox bb;
                bb.height            = height;
                bb.length            = length;
                bb.width             = width;
                bb.x                 = x;
                bb.y                 = y;
                bb.z                 = z;
                pkg.hdr.id           = static_cast<int>(PackageId::BOUNDING_BOX);
                pkg.hdr.content_size = sizeof(bb);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &bb, sizeof(bb));
                writePackage(pkg);
                completeObjectState.obj_states[i].boundingBox_ = bb;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteScaleMode(int obj_id, int mode)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].scaleMode_.scale_mode != mode)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::SCALE_MODE);
                pkg.hdr.content_size = sizeof(mode);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &mode, sizeof(mode));
                writePackage(pkg);
                completeObjectState.obj_states[i].scaleMode_.scale_mode = mode;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteVisiblityMask(int obj_id, int mask)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].visibilityMask_.visibility_mask != mask)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::VISIBILITY_MASK);
                pkg.hdr.content_size = sizeof(mask);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &mask, sizeof(mask));
                writePackage(pkg);
                completeObjectState.obj_states[i].visibilityMask_.visibility_mask = mask;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteTime(double t)
{
    if (data_file_.is_open())
    {
        if (!isEqualDouble(completeObjectState.time.time, t))
        {
            // create pkg
            CommonPkg pkg;
            pkg.hdr.id           = static_cast<int>(PackageId::TIME_SERIES);
            pkg.hdr.content_size = sizeof(t);
            pkg.content.resize(pkg.hdr.content_size);
            std::memcpy(pkg.content.data(), &t, sizeof(t));
            writePackage(pkg);
            completeObjectState.time.time = t;
        }
    }

    return 0;
}

void DatLogger::WriteStringPkg(std::string name, PackageId pkg_id)
{
    // create pkg
    CommonPkg pkg;
    pkg.hdr.id           = static_cast<id_t>(pkg_id);
    pkg.hdr.content_size = static_cast<unsigned int>(name.size() + 1);
    pkg.content.resize(pkg.hdr.content_size);
    StrCopy(pkg.content.data(), name.c_str(), pkg.hdr.content_size);
    data_file_.write(reinterpret_cast<char*>(&pkg.hdr), sizeof(CommonPkgHdr));
    data_file_.write(pkg.content.data(), pkg.hdr.content_size);
}

int DatLogger::WriteName(int obj_id, std::string name)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].name_.compare(name) != 0)
            {
                WriteManPkg(obj_id);
                // write name
                WriteStringPkg(name, PackageId::NAME);

                completeObjectState.obj_states[i].name_ = name;
                break;
            }
        }
    }

    return 0;
}

int DatLogger::WriteObjId(int obj_id)
{
    if (data_file_.is_open())
    {
        // create pkg
        CommonPkg pkg;
        pkg.hdr.id           = static_cast<int>(PackageId::OBJ_ID);
        pkg.hdr.content_size = sizeof(obj_id);
        pkg.content.resize(pkg.hdr.content_size);
        std::memcpy(pkg.content.data(), &obj_id, sizeof(obj_id));
        writePackage(pkg);
        if (!IsObjIdAddPkgWritten(obj_id))
        {
            CommonPkg pkg1;
            pkg1.hdr.id           = static_cast<int>(PackageId::OBJ_ADDED);
            pkg1.hdr.content_size = 0;
            pkg1.content.resize(static_cast<size_t>(pkg1.hdr.content_size));
            writePackage(pkg1);
            SetObjIdAddPkgWritten(obj_id, true);
        }
    }
    return 0;
}

int DatLogger::AddObject(int obj_id)
{
    if (completeObjectState.obj_states.size() == 0)  // first time
    {
        ObjState objState_;
        objState_.obj_id_.obj_id = obj_id;
        completeObjectState.obj_states.push_back(objState_);
        ObjIdAdded objIdAdded;
        objIdAdded.id = obj_id;
        objIdAdded_.push_back(objIdAdded);
    }
    else
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id == obj_id)
            {
                break;  // object already available in cache
            }
            if (i == completeObjectState.obj_states.size() - 1)  // reached last iteration so add object in cache
            {
                ObjState objState_;
                objState_.obj_id_.obj_id = obj_id;
                completeObjectState.obj_states.push_back(objState_);
                ObjIdAdded objIdAdded;
                objIdAdded.id = obj_id;
                objIdAdded_.push_back(objIdAdded);
                break;
            }
        }
    }
    return 0;
}

int DatLogger::deleteObject()
{
    for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
    {
        if (completeObjectState.obj_states[i].active == false)
        {
            if (!TimePkgAdded)
            {
                WriteTime(simTimeTemp);
            }
            WriteObjId(completeObjectState.obj_states[i].obj_id_.obj_id);
            CommonPkg pkg;
            pkg.hdr.id           = static_cast<int>(PackageId::OBJ_DELETED);
            pkg.hdr.content_size = 0;
            pkg.content.resize(pkg.hdr.content_size);
            writePackage(pkg);
            SetObjIdAddPkgWritten(completeObjectState.obj_states[i].obj_id_.obj_id, false);
            completeObjectState.obj_states.erase(completeObjectState.obj_states.begin() + static_cast<int>(i));
        }
        else
        {
            completeObjectState.obj_states[i].active = false;  // reset for next time frame
        }
    }
    return 0;
}

int DatLogger::WriteObjPos(int obj_id, double x, double y, double z, double h, double p, double r)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].pos_.h != h || completeObjectState.obj_states[i].pos_.p != p ||
                completeObjectState.obj_states[i].pos_.r != r || completeObjectState.obj_states[i].pos_.x != x ||
                completeObjectState.obj_states[i].pos_.y != y || completeObjectState.obj_states[i].pos_.z != z)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                Pos       pos_;
                pos_.x = x;
                pos_.y = y;
                pos_.z = z;
                pos_.h = h;
                pos_.p = p;
                pos_.r = r;

                pkg.hdr.id           = static_cast<int>(PackageId::POSITIONS);
                pkg.hdr.content_size = sizeof(pos_);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &pos_, sizeof(pos_));
                writePackage(pkg);
                completeObjectState.obj_states[i].pos_ = pos_;
                break;
            }
            else
            {
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteRoadId(int obj_id, id_t road_id)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].roadId_.road_id != road_id)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<id_t>(PackageId::ROAD_ID);
                pkg.hdr.content_size = sizeof(road_id);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &road_id, sizeof(road_id));
                writePackage(pkg);
                completeObjectState.obj_states[i].roadId_.road_id = road_id;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteLaneId(int obj_id, int lane_id)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (completeObjectState.obj_states[i].laneId_.lane_id != lane_id)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::LANE_ID);
                pkg.hdr.content_size = sizeof(lane_id);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &lane_id, sizeof(lane_id));
                writePackage(pkg);
                completeObjectState.obj_states[i].laneId_.lane_id = lane_id;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WritePosOffset(int obj_id, double offset)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (!isEqualDouble(completeObjectState.obj_states[i].posOffset_.offset, offset))
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::POS_OFFSET);
                pkg.hdr.content_size = sizeof(offset);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &offset, sizeof(offset));
                writePackage(pkg);
                completeObjectState.obj_states[i].posOffset_.offset = offset;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WritePosT(int obj_id, double t)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (!isEqualDouble(completeObjectState.obj_states[i].posT.t, t))
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::POS_T);
                pkg.hdr.content_size = sizeof(t);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &t, sizeof(t));
                writePackage(pkg);
                completeObjectState.obj_states[i].posT.t = t;
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WritePosS(int obj_id, double s)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            if (!isEqualDouble(completeObjectState.obj_states[i].posS.s, s))
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::POS_S);
                pkg.hdr.content_size = sizeof(s);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &s, sizeof(s));
                writePackage(pkg);
                completeObjectState.obj_states[i].posS.s = s;
                break;
            }
        }
    }
    return 0;
}

bool IsEqualRgb(const LightRGB& rgb1, const LightRGB& rgb2)
{
    if (rgb1.red == rgb2.red && rgb1.green == rgb2.green && rgb1.blue == rgb2.blue && rgb1.intensity == rgb2.intensity)
    {
        return true;
    }

    return false;
}

void DatLogger::WriteLightState(int obj_id, LightState rgb_data)
{
    if (data_file_.is_open())
    {
        for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)
        {
            if (completeObjectState.obj_states[i].obj_id_.obj_id != obj_id)
            {
                continue;
            }
            completeObjectState.obj_states[i].active = true;
            const size_t numLights                   = sizeof(rgb_data) / sizeof(datLogger::LightRGB);
            for (size_t j = 0; j < numLights; ++j)
            {
                LightRGB* lightNew = reinterpret_cast<datLogger::LightRGB*>(&rgb_data) + j;
                LightRGB* lightOld = reinterpret_cast<datLogger::LightRGB*>(&completeObjectState.obj_states[i].lightStates_) + j;
                if (lightNew->red != lightOld->red || lightNew->green != lightOld->green || lightNew->blue != lightOld->blue ||
                    lightNew->intensity != lightOld->intensity)
                {
                    WriteManPkg(obj_id);
                    // create pkg
                    CommonPkg pkg;
                    pkg.hdr.id           = static_cast<int>(PackageId::LIGHT_STATES);
                    pkg.hdr.content_size = sizeof(rgb_data);
                    pkg.content.resize(pkg.hdr.content_size);
                    std::memcpy(pkg.content.data(), &rgb_data, sizeof(rgb_data));
                    writePackage(pkg);
                    completeObjectState.obj_states[i].lightStates_ = rgb_data;
                    break;
                }
            }
        }
    }
}

void DatLogger::writePackage(CommonPkg package)
{
    if (data_file_.is_open())
    {
        data_file_.write(reinterpret_cast<char*>(&package.hdr), sizeof(CommonPkgHdr));
        if (!(package.hdr.id == static_cast<int>(PackageId::OBJ_ADDED) || package.hdr.id == static_cast<int>(PackageId::OBJ_DELETED) ||
              package.hdr.id == static_cast<int>(PackageId::END_OF_SCENARIO)))
        {
            data_file_.write(package.content.data(), package.hdr.content_size);
        }
    }
    else
    {
        LOG_ERROR_AND_QUIT("Failed to open file\n");
    }
}

void DatLogger::DeleteObjState(int objId)
{
    for (size_t i = 0; i < completeObjectState.obj_states.size(); i++)  // loop current state object id to find the object id
    {
        if (completeObjectState.obj_states[i].obj_id_.obj_id == objId)  // found object id
        {                                                               // delete now
            completeObjectState.obj_states.erase(completeObjectState.obj_states.begin() + static_cast<int>(i));
        }
    }
}

int DatLogger::init(std::string fileName, int ver, std::string odrName, std::string modelName)
{
    data_file_.open(fileName, std::ios::binary);
    if (data_file_.fail())
    {
        std::printf("Cannot open file: %s", fileName.c_str());
        return -1;
    }

    DatHdr datHdr;
    datHdr.version          = ver;
    datHdr.odrFilename.size = static_cast<unsigned int>(odrName.size() + 1);
    datHdr.odrFilename.string.resize(datHdr.odrFilename.size);
    datHdr.modelFilename.size = static_cast<unsigned int>(modelName.size() + 1);
    datHdr.modelFilename.string.resize(datHdr.modelFilename.size);
    StrCopy(datHdr.odrFilename.string.data(), odrName.c_str(), datHdr.odrFilename.size);
    StrCopy(datHdr.modelFilename.string.data(), modelName.c_str(), datHdr.modelFilename.size);

    CommonPkgHdr cmnHdr;
    cmnHdr.id           = static_cast<int>(PackageId::HEADER);
    cmnHdr.content_size = static_cast<int>(sizeof(datHdr.version)) + datHdr.odrFilename.size + datHdr.modelFilename.size;
    CommonPkg hdrPkg;
    hdrPkg.hdr = cmnHdr;
    hdrPkg.content.resize(hdrPkg.hdr.content_size);

    data_file_.write(reinterpret_cast<char*>(&hdrPkg.hdr), sizeof(CommonPkgHdr));

    // write content -> version
    data_file_.write(reinterpret_cast<char*>(&datHdr.version), sizeof(datHdr.version));

    // write content -> odr filename size
    data_file_.write(reinterpret_cast<char*>(&datHdr.odrFilename.size), sizeof(datHdr.odrFilename.size));

    // write actual odr filename string
    data_file_.write(datHdr.odrFilename.string.data(), datHdr.odrFilename.size);

    // write content -> model filename size
    data_file_.write(reinterpret_cast<char*>(&datHdr.modelFilename.size), sizeof(datHdr.modelFilename.size));

    // write actual model filename string
    data_file_.write(datHdr.modelFilename.string.data(), datHdr.modelFilename.size);

    return 0;
}

std::string DatLogger::pkgIdTostring(PackageId id)
{
    switch (id)
    {
        case PackageId::HEADER:
        {
            return "File_Header";
            break;
        }
        case PackageId::TIME_SERIES:
        {
            return "Time";
            break;
        }

        case PackageId::OBJ_ID:
        {
            return "Object_Id";
            break;
        }

        case PackageId::POSITIONS:
        {
            return "Position";
            break;
        }

        case PackageId::SPEED:
        {
            return "Speed";
            break;
        }

        case PackageId::BOUNDING_BOX:
        {
            return "Bounding_box";
            break;
        }

        case PackageId::CTRL_TYPE:
        {
            return "Ctrl_type";
            break;
        }

        case PackageId::MODEL_ID:
        {
            return "Model_Id";
            break;
        }

        case PackageId::NAME:
        {
            return "Name";
            break;
        }

        case PackageId::OBJ_CATEGORY:
        {
            return "Obj_Category";
            break;
        }

        case PackageId::OBJ_TYPE:
        {
            return "Obj_Type";
            break;
        }

        case PackageId::SCALE_MODE:
        {
            return "Scale_Mode";
            break;
        }

        case PackageId::VISIBILITY_MASK:
        {
            return "Visiblity_Mask";
            break;
        }

        case PackageId::WHEEL_ANGLE:
        {
            return "Wheel_angle";
            break;
        }

        case PackageId::WHEEL_ROT:
        {
            return "Wheel_Rot";
            break;
        }
        case PackageId::OBJ_ADDED:
        {
            return "obj_added";
            break;
        }
        case PackageId::OBJ_DELETED:
        {
            return "obj_deleted";
            break;
        }
        case PackageId::END_OF_SCENARIO:
        {
            return "end_of_scenario";
            break;
        }
        case PackageId::ROAD_ID:
        {
            return "road_id";
            break;
        }
        case PackageId::LANE_ID:
        {
            return "lane_id";
            break;
        }
        case PackageId::POS_OFFSET:
        {
            return "pos_offset";
            break;
        }
        case PackageId::POS_T:
        {
            return "pos_t";
            break;
        }
        case PackageId::POS_S:
        {
            return "pos_s";
            break;
        }
        default:
        {
            std::cout << "Unknown package read->package id :" << std::endl;
            return "none";
            break;
        }
    }
}
