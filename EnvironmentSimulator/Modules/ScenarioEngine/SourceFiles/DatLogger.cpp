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
int DatLogger::WriteObjectData(int obj_id, T value, PackageId package_id, std::function<bool(ObjState&, T)> updateState)
{
    if (IsFileOpen())
    {
        for (auto& objState : completeObjectState_.obj_states)
        {
            if (objState.obj_id_.obj_id == obj_id)
            {
                objState.active = true;
                if (!updateState(objState, value))  // Let the lambda handle the state comparison and update
                {
                    WriteManPkg(obj_id);

                    // create pkg
                    CommonPkg pkg;
                    pkg.hdr.id           = static_cast<id_t>(package_id);
                    pkg.hdr.content_size = sizeof(value);
                    pkg.content.resize(pkg.hdr.content_size);
                    std::memcpy(pkg.content.data(), &value, sizeof(value));
                    writePackage(pkg);
                }
                break;
            }
        }
    }
    return 0;
}

void DatLogger::WriteManPkg(int obj_id)
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

int DatLogger::WriteObjSpeed(int obj_id, double speed)
{
    WriteObjectData<double>(obj_id,
                            speed,
                            PackageId::SPEED,
                            [](ObjState& objState, double value) -> bool
                            {
                                if (!IsEqualDouble(objState.speed_.speed_, value))
                                {
                                    objState.speed_.speed_ = value;
                                    return false;  // Indicates that the state was updated
                                }
                                return true;  // Indicates that the state was unchanged
                            });
    return 0;
}

int DatLogger::WriteModelId(int obj_id, int model_id)
{
    WriteObjectData<int>(obj_id,
                         model_id,
                         PackageId::MODEL_ID,
                         [](ObjState& objState, int value) -> bool
                         {
                             if (objState.modelId_.model_id != value)
                             {
                                 objState.modelId_.model_id = value;  // Update the state
                                 return false;                        // Indicates that the state was updated
                             }
                             return true;  // Indicates that the state was unchanged
                         });
    return 0;
}

int DatLogger::WriteObjType(int obj_id, int obj_type)
{
    WriteObjectData<int>(obj_id,
                         obj_type,
                         PackageId::OBJ_TYPE,
                         [](ObjState& objState, int value) -> bool
                         {
                             if (objState.objType_.obj_type != value)
                             {
                                 objState.objType_.obj_type = value;  // Update the state
                                 return false;                        // Indicates that the state was updated
                             }
                             return true;  // Indicates that the state was unchanged
                         });
    return 0;
}

int DatLogger::WriteObjCategory(int obj_id, int obj_category)
{
    WriteObjectData<int>(obj_id,
                         obj_category,
                         PackageId::OBJ_CATEGORY,
                         [](ObjState& objState, int value) -> bool
                         {
                             if (objState.objCategory_.obj_category != value)
                             {
                                 objState.objCategory_.obj_category = value;  // Update the state
                                 return false;                                // Indicates that the state was updated
                             }
                             return true;  // Indicates that the state was unchanged
                         });
    return 0;
}

int DatLogger::WriteCtrlType(int obj_id, int ctrl_type)
{
    WriteObjectData<int>(obj_id,
                         ctrl_type,
                         PackageId::CTRL_TYPE,
                         [](ObjState& objState, int value) -> bool
                         {
                             if (objState.ctrlType_.ctrl_type != value)
                             {
                                 objState.ctrlType_.ctrl_type = value;  // Update the state
                                 return false;                          // Indicates that the state was updated
                             }
                             return true;  // Indicates that the state was unchanged
                         });
    return 0;
}

int DatLogger::WriteWheelAngle(int obj_id, double angle)
{
    WriteObjectData<double>(obj_id,
                            angle,
                            PackageId::WHEEL_ANGLE,
                            [](ObjState& objState, double value) -> bool
                            {
                                if (!IsEqualDouble(objState.wheelAngle_.wheel_angle, value))
                                {
                                    objState.wheelAngle_.wheel_angle = value;
                                    return false;  // Indicates that the state was updated
                                }
                                return true;  // Indicates that the state was unchanged
                            });
    return 0;
}

int DatLogger::WriteWheelRot(int obj_id, double rot)
{
    WriteObjectData<double>(obj_id,
                            rot,
                            PackageId::WHEEL_ROT,
                            [](ObjState& objState, double value) -> bool
                            {
                                if (!IsEqualDouble(objState.wheelRot_.wheel_rot, value))
                                {
                                    objState.wheelRot_.wheel_rot = value;
                                    return false;  // Indicates that the state was updated
                                }
                                return true;  // Indicates that the state was unchanged
                            });
    return 0;
}

int DatLogger::WriteBB(int obj_id, float x, float y, float z, float length, float width, float height)
{
    if (IsFileOpen())
    {
        for (auto& objState : completeObjectState_.obj_states)
        {
            if (objState.obj_id_.obj_id == obj_id)
            {
                objState.active = true;
                if (objState.boundingBox_.x != x || objState.boundingBox_.y != y || objState.boundingBox_.z != z ||
                    objState.boundingBox_.height != height || objState.boundingBox_.length != length || objState.boundingBox_.width != width)
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
                    objState.boundingBox_ = bb;
                }
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteScaleMode(int obj_id, int mode)
{
    WriteObjectData<int>(obj_id,
                         mode,
                         PackageId::SCALE_MODE,
                         [](ObjState& objState, int value) -> bool
                         {
                             if (objState.scaleMode_.scale_mode != value)
                             {
                                 objState.scaleMode_.scale_mode = value;  // Update the state
                                 return false;                            // Indicates that the state was updated
                             }
                             return true;  // Indicates that the state was unchanged
                         });
    return 0;
}

int DatLogger::WriteVisiblityMask(int obj_id, int mask)
{
    WriteObjectData<int>(obj_id,
                         mask,
                         PackageId::VISIBILITY_MASK,
                         [](ObjState& objState, int value) -> bool
                         {
                             if (objState.visibilityMask_.visibility_mask != value)
                             {
                                 objState.visibilityMask_.visibility_mask = value;  // Update the state
                                 return false;                                      // Indicates that the state was updated
                             }
                             return true;  // Indicates that the state was unchanged
                         });
    return 0;
}

int DatLogger::WriteTime(double t)
{
    if (data_file_.is_open())
    {
        if (!IsEqualDouble(completeObjectState_.time.time, t))
        {
            // create pkg
            CommonPkg pkg;
            pkg.hdr.id           = static_cast<int>(PackageId::TIME_SERIES);
            pkg.hdr.content_size = sizeof(t);
            pkg.content.resize(pkg.hdr.content_size);
            std::memcpy(pkg.content.data(), &t, sizeof(t));
            writePackage(pkg);
            completeObjectState_.time.time = t;
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
    if (IsFileOpen())
    {
        for (auto& objState : completeObjectState_.obj_states)
        {
            if (objState.obj_id_.obj_id == obj_id)
            {
                objState.active = true;
                if (objState.name_.compare(name) != 0)
                {
                    WriteManPkg(obj_id);
                    // write name
                    WriteStringPkg(name, PackageId::NAME);
                    objState.name_ = name;
                }
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
    if (completeObjectState_.obj_states.size() == 0)  // first time
    {
        ObjState objState_;
        objState_.obj_id_.obj_id = obj_id;
        completeObjectState_.obj_states.push_back(objState_);
        ObjIdAdded objIdAdded;
        objIdAdded.id = obj_id;
        objIdAdded_.push_back(objIdAdded);
    }
    else
    {
        for (size_t i = 0; i < completeObjectState_.obj_states.size(); i++)
        {
            if (completeObjectState_.obj_states[i].obj_id_.obj_id == obj_id)
            {
                break;  // object already available in cache
            }
            if (i == completeObjectState_.obj_states.size() - 1)  // reached last iteration so add object in cache
            {
                ObjState objState_;
                objState_.obj_id_.obj_id = obj_id;
                completeObjectState_.obj_states.push_back(objState_);
                ObjIdAdded objIdAdded;
                objIdAdded.id = obj_id;
                objIdAdded_.push_back(objIdAdded);
                break;
            }
        }
    }
    return 0;
}

int DatLogger::DeleteObject()
{
    for (size_t i = 0; i < completeObjectState_.obj_states.size(); i++)
    {
        if (completeObjectState_.obj_states[i].active == false)
        {
            if (!TimePkgAdded_)
            {
                WriteTime(simTimeTemp_);
            }
            WriteObjId(completeObjectState_.obj_states[i].obj_id_.obj_id);
            CommonPkg pkg;
            pkg.hdr.id           = static_cast<int>(PackageId::OBJ_DELETED);
            pkg.hdr.content_size = 0;
            pkg.content.resize(pkg.hdr.content_size);
            writePackage(pkg);
            SetObjIdAddPkgWritten(completeObjectState_.obj_states[i].obj_id_.obj_id, false);
            completeObjectState_.obj_states.erase(completeObjectState_.obj_states.begin() + static_cast<int>(i));
        }
        else
        {
            completeObjectState_.obj_states[i].active = false;  // reset for next time frame
        }
    }
    return 0;
}

int DatLogger::WriteObjPos(int obj_id, double x, double y, double z, double h, double p, double r)
{
    if (IsFileOpen())
    {
        for (auto& objState : completeObjectState_.obj_states)
        {
            if (objState.obj_id_.obj_id == obj_id)
            {
                objState.active = true;
                if (!IsEqualDouble(objState.pos_.x, x) || !IsEqualDouble(objState.pos_.y, y) || !IsEqualDouble(objState.pos_.z, z) ||
                    !IsEqualDouble(objState.pos_.h, h) || !IsEqualDouble(objState.pos_.p, p) || !IsEqualDouble(objState.pos_.r, r))
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
                    objState.pos_ = pos_;
                }
                break;
            }
        }
    }
    return 0;
}

int DatLogger::WriteRoadId(int obj_id, id_t road_id)
{
    WriteObjectData<id_t>(obj_id,
                          road_id,
                          PackageId::ROAD_ID,
                          [](ObjState& objState, id_t value) -> bool
                          {
                              if (objState.roadId_.road_id != value)
                              {
                                  objState.roadId_.road_id = value;  // Update the state
                                  return false;                      // Indicates that the state was updated
                              }
                              return true;  // Indicates that the state was unchanged
                          });
    return 0;
}

int DatLogger::WriteLaneId(int obj_id, int lane_id)
{
    WriteObjectData<int>(obj_id,
                         lane_id,
                         PackageId::LANE_ID,
                         [](ObjState& objState, int value) -> bool
                         {
                             if (objState.laneId_.lane_id != value)
                             {
                                 objState.laneId_.lane_id = value;  // Update the state
                                 return false;                      // Indicates that the state was updated
                             }
                             return true;  // Indicates that the state was unchanged
                         });
    return 0;
}

int DatLogger::WritePosOffset(int obj_id, double offset)
{
    WriteObjectData<double>(obj_id,
                            offset,
                            PackageId::POS_OFFSET,
                            [](ObjState& objState, double value) -> bool
                            {
                                if (!IsEqualDouble(objState.posOffset_.offset, value))
                                {
                                    objState.posOffset_.offset = value;
                                    return false;  // Indicates that the state was updated
                                }
                                return true;  // Indicates that the state was unchanged
                            });
    return 0;
}

int DatLogger::WritePosT(int obj_id, double t)
{
    WriteObjectData<double>(obj_id,
                            t,
                            PackageId::POS_T,
                            [](ObjState& objState, double value) -> bool
                            {
                                if (!IsEqualDouble(objState.posT.t, value))
                                {
                                    objState.posT.t = value;
                                    return false;  // Indicates that the state was updated
                                }
                                return true;  // Indicates that the state was unchanged
                            });
    return 0;
}

int DatLogger::WritePosS(int obj_id, double s)
{
    WriteObjectData<double>(obj_id,
                            s,
                            PackageId::POS_S,
                            [](ObjState& objState, double value) -> bool
                            {
                                if (!IsEqualDouble(objState.posS.s, value))
                                {
                                    objState.posS.s = value;
                                    return false;  // Indicates that the state was updated
                                }
                                return true;  // Indicates that the state was unchanged
                            });
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
    if (IsFileOpen())
    {
        for (auto& objState : completeObjectState_.obj_states)
        {
            if (objState.obj_id_.obj_id == obj_id)
            {
                objState.active        = true;
                const size_t numLights = sizeof(rgb_data) / sizeof(dat::LightRGB);
                for (size_t i = 0; i < numLights; ++i)
                {
                    LightRGB* lightNew = reinterpret_cast<dat::LightRGB*>(&rgb_data) + i;
                    LightRGB* lightOld = reinterpret_cast<dat::LightRGB*>(&objState.lightStates_) + i;
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
                        objState.lightStates_ = rgb_data;
                        break;
                    }
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
    for (size_t i = 0; i < completeObjectState_.obj_states.size(); i++)  // loop current state object id to find the object id
    {
        if (completeObjectState_.obj_states[i].obj_id_.obj_id == objId)  // found object id
        {                                                                // delete now
            completeObjectState_.obj_states.erase(completeObjectState_.obj_states.begin() + static_cast<int>(i));
        }
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
