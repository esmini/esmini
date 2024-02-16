// logDat

#include <iostream>
#include <fstream>
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
            pkg.content          = reinterpret_cast<char*>(&speed);
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
                pkg.content          = reinterpret_cast<char*>(&model_id);
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
                pkg.content          = reinterpret_cast<char*>(&obj_type);
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
                pkg.content          = reinterpret_cast<char*>(&obj_category);
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
                pkg.content          = reinterpret_cast<char*>(&ctrl_type);
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
                pkg.content          = reinterpret_cast<char*>(&angle);
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
                pkg.content          = reinterpret_cast<char*>(&rot);
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
                pkg.content          = reinterpret_cast<char*>(&bb);
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
                pkg.content          = reinterpret_cast<char*>(&mode);
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
                pkg.content          = reinterpret_cast<char*>(&mask);
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
            pkg.content          = reinterpret_cast<char*>(&t);
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
    pkg.hdr.id = static_cast<int>(pkg_id);
    Name nameStr;
    pkg.hdr.content_size = static_cast<int>(name.size() + 1);
    nameStr.string       = new char[pkg.hdr.content_size];
    StrCopy(nameStr.string, name.c_str(), static_cast<size_t>(pkg.hdr.content_size));

    data_file_.write(reinterpret_cast<char*>(&pkg.hdr), sizeof(CommonPkgHdr));

    data_file_.write(nameStr.string, pkg.hdr.content_size);
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

int DatLogger::WriteObjId(int obj_id)
{
    if (data_file_.is_open())
    {
        // create pkg
        CommonPkg pkg;
        pkg.hdr.id           = static_cast<int>(PackageId::OBJ_ID);
        pkg.hdr.content_size = sizeof(obj_id);
        pkg.content          = reinterpret_cast<char*>(&obj_id);
        writePackage(pkg);
        if (!IsObjIdAddPkgWritten(obj_id))
        {
            CommonPkg pkg1;
            pkg1.hdr.id           = static_cast<int>(PackageId::OBJ_ADDED);
            pkg1.hdr.content_size = 0;
            pkg1.content          = nullptr;
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
                WriteTime(completeObjectState.time.time);
            }
            WriteObjId(completeObjectState.obj_states[i].obj_id_.obj_id);
            CommonPkg pkg;
            pkg.hdr.id           = static_cast<int>(PackageId::OBJ_DELETED);
            pkg.hdr.content_size = 0;
            pkg.content          = nullptr;
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
                pkg.content          = reinterpret_cast<char*>(&pos_);
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

int DatLogger::WriteRoadId(int obj_id, int road_id)
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
                pkg.hdr.id           = static_cast<int>(PackageId::ROAD_ID);
                pkg.hdr.content_size = sizeof(road_id);
                pkg.content          = reinterpret_cast<char*>(&road_id);
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
                pkg.content          = reinterpret_cast<char*>(&lane_id);
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
                pkg.content          = reinterpret_cast<char*>(&offset);
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
                pkg.content          = reinterpret_cast<char*>(&t);
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
                pkg.content          = reinterpret_cast<char*>(&s);
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

            if ((!IsEqualRgb(rgb_data.day_time_running_lights, completeObjectState.obj_states[i].lightStates_.day_time_running_lights) ||
                 !IsEqualRgb(rgb_data.low_beam, completeObjectState.obj_states[i].lightStates_.low_beam) ||
                 !IsEqualRgb(rgb_data.high_beam, completeObjectState.obj_states[i].lightStates_.high_beam) ||
                 !IsEqualRgb(rgb_data.fog_lights_front, completeObjectState.obj_states[i].lightStates_.fog_lights_front) ||
                 !IsEqualRgb(rgb_data.fog_lights_rear, completeObjectState.obj_states[i].lightStates_.fog_lights_rear) ||
                 !IsEqualRgb(rgb_data.brake_lights, completeObjectState.obj_states[i].lightStates_.brake_lights) ||
                 !IsEqualRgb(rgb_data.indicator_left, completeObjectState.obj_states[i].lightStates_.indicator_left) ||
                 !IsEqualRgb(rgb_data.indicator_right, completeObjectState.obj_states[i].lightStates_.indicator_right) ||
                 !IsEqualRgb(rgb_data.reversing_lights, completeObjectState.obj_states[i].lightStates_.reversing_lights) ||
                 !IsEqualRgb(rgb_data.license_plater_illumination, completeObjectState.obj_states[i].lightStates_.license_plater_illumination) ||
                 !IsEqualRgb(rgb_data.special_purpose_lights, completeObjectState.obj_states[i].lightStates_.special_purpose_lights) ||
                 !IsEqualRgb(rgb_data.fog_lights, completeObjectState.obj_states[i].lightStates_.fog_lights) ||
                 !IsEqualRgb(rgb_data.warning_lights, completeObjectState.obj_states[i].lightStates_.warning_lights)) ||
                isFirstLightPkg)
            {
                WriteManPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::LIGHT_STATES);
                pkg.hdr.content_size = sizeof(rgb_data);
                pkg.content          = reinterpret_cast<char*>(&rgb_data);
                writePackage(pkg);
                completeObjectState.obj_states[i].lightStates_ = rgb_data;
                isFirstLightPkg                                = false;
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
            data_file_.write(package.content, package.hdr.content_size);
        }
    }
    else
    {
        LOG_AND_QUIT("Failed to open file\n");
    }
}

void DatLogger::deleteObjState(int objId)
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
    std::ofstream data_file(fileName, std::ios::binary);
    data_file_.open(fileName, std::ios::in | std::ios::out | std::ios::binary);
    if (data_file_.fail())
    {
        std::printf("Cannot open file: %s", fileName.c_str());
        return -1;
    }

    CommonPkgHdr cmnHdr;
    cmnHdr.id = static_cast<int>(PackageId::HEADER);

    CommonString odrStr;
    odrStr.size   = static_cast<int>(odrName.size() + 1);
    odrStr.string = new char[odrStr.size];
    StrCopy(odrStr.string, odrName.c_str(), static_cast<size_t>(odrStr.size));
    CommonString mdlStr;
    mdlStr.size   = static_cast<int>(modelName.size() + 1);
    mdlStr.string = new char[mdlStr.size];
    StrCopy(mdlStr.string, modelName.c_str(), static_cast<size_t>(mdlStr.size));

    cmnHdr.content_size = static_cast<int>(sizeof(odrStr.size)) + odrStr.size + mdlStr.size + static_cast<int>(sizeof(mdlStr.size));

    DatHdr datHdr;
    datHdr.version       = ver;
    datHdr.odrFilename   = odrStr;
    datHdr.modelFilename = mdlStr;

    CommonPkg hdrPkg;
    hdrPkg.hdr     = cmnHdr;
    hdrPkg.content = reinterpret_cast<char*>(&datHdr);

    // write header package

    // write common header
    data_file_.write(reinterpret_cast<char*>(&hdrPkg.hdr), sizeof(CommonPkgHdr));

    // write content -> version
    data_file_.write(reinterpret_cast<char*>(&datHdr.version), sizeof(datHdr.version));

    // write content -> odr filename size
    data_file_.write(reinterpret_cast<char*>(&datHdr.odrFilename.size), sizeof(datHdr.odrFilename.size));

    // write actual odr filename string
    data_file_.write(odrStr.string, datHdr.odrFilename.size);

    // write content -> model filename size
    data_file_.write(reinterpret_cast<char*>(&datHdr.modelFilename.size), sizeof(datHdr.modelFilename.size));

    // write actual model filename string
    data_file_.write(mdlStr.string, datHdr.modelFilename.size);

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
