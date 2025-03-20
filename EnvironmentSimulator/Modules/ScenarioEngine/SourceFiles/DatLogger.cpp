// logDat

#include <fstream>
#include <iostream>
#include <cstring>
#include <chrono>
#include <cmath>
#include <functional>  // For std::function

#include "DatLogger.hpp"
#include "CommonMini.hpp"

#define NUMBER_OF_VEHICLE_LIGHTS (13)

using namespace dat;

template <typename T>
void DatLogger::WriteObjectData(int obj_id, T value, PackageId package_id)
{
    WriteMandatoryPkg(obj_id);
    // create pkg
    CommonPkg pkg;
    pkg.hdr.id           = static_cast<id_t>(package_id);
    pkg.hdr.content_size = sizeof(value);
    pkg.content.resize(pkg.hdr.content_size);
    std::memcpy(pkg.content.data(), &value, sizeof(value));
    writePackage(pkg);
}

void DatLogger::WriteMandatoryPkg(int obj_id)
{
    if (!TimePkgAdded_)
    {
        WriteTime(simTimeTemp_);
        TimePkgAdded_ = true;
    }
    if (!ObjIdPkgAdded_)
    {
        WriteObjId(obj_id);
        ObjIdPkgAdded_ = true;
    }
}

bool DatLogger::IsObjIdAddPkgWritten(int id)
{
    for (const auto& objId : objIdAdded_)
    {
        if (objId.id == id)
        {
            if (objId.status == true)
            {
                return true;
                break;
            }
        }
    }
    return false;
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

bool dat::DatLogger::IsFileOpen()
{
    return data_file_.is_open();
}

void DatLogger::WriteObjSpeed(int obj_id, double speed)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (!IsEqualDouble(objState.speed_, speed))
        {
            objState.speed_ = speed;
            WriteObjectData<double>(obj_id, speed, PackageId::SPEED);
        }
    }
}

void DatLogger::WriteModelId(int obj_id, int model_id)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.modelId_ != model_id)
        {
            objState.modelId_ = model_id;
            WriteObjectData<int>(obj_id, model_id, PackageId::MODEL_ID);
        }
    }
}

void DatLogger::WriteObjType(int obj_id, int obj_type)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.objType_ != obj_type)
        {
            objState.objType_ = obj_type;
            WriteObjectData<int>(obj_id, obj_type, PackageId::OBJ_TYPE);
        }
    }
}

void DatLogger::WriteObjCategory(int obj_id, int objCategory)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.objCategory_ != objCategory)
        {
            objState.objCategory_ = objCategory;
            WriteObjectData<int>(obj_id, objCategory, PackageId::OBJ_CATEGORY);
        }
    }
}

void DatLogger::WriteCtrlType(int obj_id, int ctrl_type)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.ctrlType_ != ctrl_type)
        {
            objState.ctrlType_ = ctrl_type;
            WriteObjectData<int>(obj_id, ctrl_type, PackageId::CTRL_TYPE);
        }
    }
}

void DatLogger::WriteWheelAngle(int obj_id, double angle)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (!IsEqualDouble(objState.wheelAngle_, angle))
        {
            objState.wheelAngle_ = angle;
            WriteObjectData<double>(obj_id, angle, PackageId::WHEEL_ANGLE);
        }
    }
}

void DatLogger::WriteWheelRot(int obj_id, double rot)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (!IsEqualDouble(objState.wheelRot_, rot))
        {
            objState.wheelRot_ = rot;
            WriteObjectData<double>(obj_id, rot, PackageId::WHEEL_ROT);
        }
    }
}

void DatLogger::WriteBB(int obj_id, float x, float y, float z, float length, float width, float height)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.boundingBox_.x != x || objState.boundingBox_.y != y || objState.boundingBox_.z != z || objState.boundingBox_.height != height ||
            objState.boundingBox_.length != length || objState.boundingBox_.width != width)
        {
            WriteMandatoryPkg(obj_id);
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
            objState.boundingBox_ = bb;
        }
    }
}

void DatLogger::WriteScaleMode(int obj_id, int mode)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.scaleMode_ != mode)
        {
            objState.scaleMode_ = mode;
            WriteObjectData<int>(obj_id, mode, PackageId::SCALE_MODE);
        }
    }
}

void DatLogger::WriteVisiblityMask(int obj_id, int mask)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.visibilityMask_ != mask)
        {
            objState.visibilityMask_ = mask;
            WriteObjectData<int>(obj_id, mask, PackageId::VISIBILITY_MASK);
        }
    }
}

void DatLogger::WriteTime(double t)
{
    if (!IsEqualDouble(completeObjectState_.time_, t))
    {
        // create pkg
        CommonPkg pkg;
        pkg.hdr.id           = static_cast<int>(PackageId::TIME_SERIES);
        pkg.hdr.content_size = sizeof(t);
        pkg.content.resize(pkg.hdr.content_size);
        std::memcpy(pkg.content.data(), &t, sizeof(t));
        writePackage(pkg);
        completeObjectState_.time_ = t;
    }
}

void DatLogger::WriteStringPkg(std::string name, PackageId pkg_id)
{
    // create pkg
    CommonPkg pkg;
    pkg.hdr.id           = static_cast<id_t>(pkg_id);
    pkg.hdr.content_size = static_cast<uint16_t>(name.size() + 1);
    pkg.content.resize(pkg.hdr.content_size);
    StrCopy(pkg.content.data(), name.c_str(), pkg.hdr.content_size);
    data_file_.write(reinterpret_cast<char*>(&pkg.hdr), sizeof(CommonPkgHdr));
    data_file_.write(pkg.content.data(), pkg.hdr.content_size);
    CountPkg(pkg_id);
}

void DatLogger::WriteName(int obj_id, std::string name)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.name_.compare(name) != 0)
        {
            objState.name_ = name;
            WriteMandatoryPkg(obj_id);
            // write name
            WriteStringPkg(name, PackageId::NAME);
        }
    }
}

void DatLogger::WriteObjId(int obj_id)
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

void DatLogger::AddObject(int objId)
{
    if (completeObjectState_.obj_states_.find(objId) == completeObjectState_.obj_states_.end())
    {
        ObjState objState_;
        objState_.objId_                        = objId;
        completeObjectState_.obj_states_[objId] = objState_;
        ObjIdAdded objIdAdded;
        objIdAdded.id = objId;
        objIdAdded_.push_back(objIdAdded);
    }
}

void DatLogger::DeleteObject()
{
    for (auto it = completeObjectState_.obj_states_.begin(); it != completeObjectState_.obj_states_.end();)
    {
        if (it->second.active_ == false)
        {
            if (!TimePkgAdded_)
            {
                WriteTime(simTimeTemp_);
            }
            WriteObjId(it->second.objId_);
            CommonPkg pkg;
            pkg.hdr.id           = static_cast<int>(PackageId::OBJ_DELETED);
            pkg.hdr.content_size = 0;
            pkg.content.resize(pkg.hdr.content_size);
            writePackage(pkg);
            SetObjIdAddPkgWritten(it->second.objId_, false);
            it = completeObjectState_.obj_states_.erase(it);
        }
        else
        {
            it->second.active_ = false;  // reset for next time frame
            ++it;
        }
    }
}

void DatLogger::WriteObjPos(int obj_id, double x, double y, double z, double h, double p, double r)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (!IsEqualDouble(objState.pos_.x, x) || !IsEqualDouble(objState.pos_.y, y) || !IsEqualDouble(objState.pos_.z, z) ||
            !IsEqualDouble(objState.pos_.h, h) || !IsEqualDouble(objState.pos_.p, p) || !IsEqualDouble(objState.pos_.r, r))
        {
            WriteMandatoryPkg(obj_id);
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
            objState.pos_ = pos_;
        }
    }
}

void DatLogger::WriteRoadId(int obj_id, id_t road_id)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.roadId_ != road_id)
        {
            objState.roadId_ = road_id;
            WriteObjectData<unsigned int>(obj_id, road_id, PackageId::ROAD_ID);
        }
    }
}

void DatLogger::WriteLaneId(int obj_id, int lane_id)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.laneId_ != lane_id)
        {
            objState.laneId_ = lane_id;
            WriteObjectData<int>(obj_id, lane_id, PackageId::LANE_ID);
        }
    }
}

void DatLogger::WritePosOffset(int obj_id, double offset)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.posOffset_ != offset)
        {
            objState.posOffset_ = offset;
            WriteObjectData<double>(obj_id, offset, PackageId::POS_OFFSET);
        }
    }
}

void DatLogger::WritePosT(int obj_id, double posT)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.posT_ != posT)
        {
            objState.posT_ = posT;
            WriteObjectData<double>(obj_id, posT, PackageId::POS_T);
        }
    }
}

void DatLogger::WritePosS(int obj_id, double posS)
{
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState   = it->second;
        objState.active_ = true;
        if (objState.posS_ != posS)
        {
            objState.posS_ = posS;
            WriteObjectData<double>(obj_id, posS, PackageId::POS_S);
        }
    }
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
    if (auto it = completeObjectState_.obj_states_.find(obj_id); it != completeObjectState_.obj_states_.end())
    {
        auto& objState         = it->second;
        objState.active_       = true;
        const size_t numLights = sizeof(rgb_data) / sizeof(dat::LightRGB);
        for (size_t i = 0; i < numLights; ++i)
        {
            LightRGB* lightNew = reinterpret_cast<dat::LightRGB*>(&rgb_data) + i;
            LightRGB* lightOld = reinterpret_cast<dat::LightRGB*>(&objState.lightStates_) + i;
            if (lightNew->red != lightOld->red || lightNew->green != lightOld->green || lightNew->blue != lightOld->blue ||
                lightNew->intensity != lightOld->intensity)
            {
                WriteMandatoryPkg(obj_id);
                // create pkg
                CommonPkg pkg;
                pkg.hdr.id           = static_cast<int>(PackageId::LIGHT_STATES);
                pkg.hdr.content_size = sizeof(rgb_data);
                pkg.content.resize(pkg.hdr.content_size);
                std::memcpy(pkg.content.data(), &rgb_data, sizeof(rgb_data));
                writePackage(pkg);
                objState.lightStates_ = rgb_data;
                break;
            }
        }
    }
}

void DatLogger::writePackage(CommonPkg package)
{
    data_file_.write(reinterpret_cast<char*>(&package.hdr), sizeof(CommonPkgHdr));
    if (!(package.hdr.id == static_cast<int>(PackageId::OBJ_ADDED) || package.hdr.id == static_cast<int>(PackageId::OBJ_DELETED) ||
          package.hdr.id == static_cast<int>(PackageId::END_OF_SCENARIO)))
    {
        data_file_.write(package.content.data(), package.hdr.content_size);
    }
    CountPkg(static_cast<PackageId>(package.hdr.id));
}

void DatLogger::DeleteObjState(int objId)
{
    if (auto it = completeObjectState_.obj_states_.find(objId); it != completeObjectState_.obj_states_.end())
    {
        completeObjectState_.obj_states_.erase(objId);
    }
}

dat::DatLogger::~DatLogger()
{
    if (IsFileOpen())
    {
        WriteTime(simTimeTemp_);
        CommonPkg pkg;
        pkg.hdr.id           = static_cast<int>(PackageId::END_OF_SCENARIO);
        pkg.hdr.content_size = 0;
        pkg.content.resize(pkg.hdr.content_size);
        writePackage(pkg);

        data_file_.flush();
        data_file_.close();
    }
    PrintPkgCount();
}

int DatLogger::Init(const std::string& fileName, const std::string& odrName, const std::string& modelName)
{
    data_file_.open(fileName, std::ios::binary);
    if (data_file_.fail())
    {
        LOG_WARN("Cannot open file: {}", fileName);
        return -1;
    }

    DatHdr datHdr;
    datHdr.version          = DAT_FILE_FORMAT_VERSION;
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
    CountPkg(PackageId::HEADER);

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

void dat::DatLogger::CountPkg(PackageId id)
{
    switch (id)
    {
        case PackageId::HEADER:
        {
            headerPkgCount++;
            break;
        }
        case PackageId::TIME_SERIES:
        {
            timePkgCount++;
            break;
        }
        case PackageId::OBJ_ID:
        {
            objIdPkgCount++;
            break;
        }
        case PackageId::MODEL_ID:
        {
            modelIdPkgCount++;
            break;
        }
        case PackageId::POSITIONS:
        {
            posPkgCount++;
            break;
        }
        case PackageId::SPEED:
        {
            speedPkgCount++;
            break;
        }
        case PackageId::OBJ_TYPE:
        {
            objTypePkgCount++;
            break;
        }
        case PackageId::OBJ_CATEGORY:
        {
            objCategoryPkgCount++;
            break;
        }
        case PackageId::CTRL_TYPE:
        {
            ctrlTypePkgCount++;
            break;
        }
        case PackageId::WHEEL_ANGLE:
        {
            wheelAnglePkgCount++;
            break;
        }
        case PackageId::WHEEL_ROT:
        {
            wheelRotPkgCount++;
            break;
        }
        case PackageId::BOUNDING_BOX:
        {
            bbPkgCount++;
            break;
        }
        case PackageId::SCALE_MODE:
        {
            scaleModePkgCount++;
            break;
        }
        case PackageId::VISIBILITY_MASK:
        {
            visibilityMaskPkgCount++;
            break;
        }
        case PackageId::NAME:
        {
            namePkgCount++;
            break;
        }
        case PackageId::ROAD_ID:
        {
            roadIdPkgCount++;
            break;
        }
        case PackageId::LANE_ID:
        {
            laneIdPkgCount++;
            break;
        }
        case PackageId::POS_OFFSET:
        {
            posOffsetPkgCount++;
            break;
        }
        case PackageId::POS_T:
        {
            posTPkgCount++;
            break;
        }
        case PackageId::POS_S:
        {
            posSPkgCount++;
            break;
        }
        case PackageId::LIGHT_STATES:
        {
            lightStatePkgCount++;
            break;
        }
        case PackageId::OBJ_DELETED:
        {
            objDeletedPkgCount++;
            break;
        }
        case PackageId::OBJ_ADDED:
        {
            objAddedPkgCount++;
            break;
        }
        case PackageId::END_OF_SCENARIO:
        {
            endOfScenarioPkgCount++;
            break;
        }
        default:
        {
            std::cout << "Unknown package read->package id :" << std::endl;
            break;
        }
    }
}

void dat::DatLogger::PrintPkgCount()
{
    std::cout << "Header Pkg Count: " << headerPkgCount << std::endl;
    std::cout << "Time Pkg Count: " << timePkgCount << std::endl;
    std::cout << "Obj Id Pkg Count: " << objIdPkgCount << std::endl;
    std::cout << "Model Id Pkg Count: " << modelIdPkgCount << std::endl;
    std::cout << "Pos Pkg Count: " << posPkgCount << std::endl;
    std::cout << "Speed Pkg Count: " << speedPkgCount << std::endl;
    std::cout << "Obj Type Pkg Count: " << objTypePkgCount << std::endl;
    std::cout << "Obj Category Pkg Count: " << objCategoryPkgCount << std::endl;
    std::cout << "Ctrl Type Pkg Count: " << ctrlTypePkgCount << std::endl;
    std::cout << "Wheel Angle Pkg Count: " << wheelAnglePkgCount << std::endl;
    std::cout << "Wheel Rot Pkg Count: " << wheelRotPkgCount << std::endl;
    std::cout << "BB Pkg Count: " << bbPkgCount << std::endl;
    std::cout << "Scale Mode Pkg Count: " << scaleModePkgCount << std::endl;
    std::cout << "Visibility Mask Pkg Count: " << visibilityMaskPkgCount << std::endl;
    std::cout << "Name Pkg Count: " << namePkgCount << std::endl;
    std::cout << "Road Id Pkg Count: " << roadIdPkgCount << std::endl;
    std::cout << "Lane Id Pkg Count: " << laneIdPkgCount << std::endl;
    std::cout << "Pos Offset Pkg Count: " << posOffsetPkgCount << std::endl;
    std::cout << "Pos T Pkg Count: " << posTPkgCount << std::endl;
    std::cout << "Pos S Pkg Count: " << posSPkgCount << std::endl;
    std::cout << "Light State Pkg Count: " << lightStatePkgCount << std::endl;
    std::cout << "Obj Deleted Pkg Count: " << objDeletedPkgCount << std::endl;
    std::cout << "Obj Added Pkg Count: " << objAddedPkgCount << std::endl;
    std::cout << "End Of Scenario Pkg Count: " << endOfScenarioPkgCount << std::endl;
}
