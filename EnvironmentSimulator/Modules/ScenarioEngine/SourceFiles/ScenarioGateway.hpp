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
#include "RoadManager.hpp"
#include "OSCBoundingBox.hpp"
#include "Entities.hpp"

#define DAT_FILE_FORMAT_VERSION 2
#define DAT_FILENAME_SIZE       512

namespace scenarioengine
{

#define NAME_LEN 32

    struct ObjectInfoStruct
    {
        int            id;
        int            model_id;
        std::string    model3d;
        int            obj_type;      // 0=None, 1=Vehicle, 2=Pedestrian, 3=MiscObj (see Object::Type enum)
        int            obj_category;  // sub type for vehicle, pedestrian and miscobj
        int            obj_role;      // role for vehicle and pedestrian
        int            ctrl_type;     // See Controller::Type enum
        double         timeStamp;
        char           name[NAME_LEN];
        double         speed;
        double         wheel_angle;       // Only used for vehicle
        double         wheel_rot;         // Only used for vehicle
        double         rear_axle_z_pos;   // z coordinate of the middle of rear axle under neutral load conditions
        double         front_axle_x_pos;  // x coordinate of the middle of front axle under neutral load conditions
        double         front_axle_z_pos;  // z coordinate of the middle of front axle under neutral load conditions
        OSCBoundingBox boundingbox;
        int            scaleMode;       // 0=None, 1=BoundingBoxToModel, 2=ModelToBoundingBox (see enum EntityScaleMode)
        int            visibilityMask;  // bitmask according to Object::Visibility (1 = Graphics, 2 = Traffic, 4 = Sensors)
        double         friction[4];     // friction coefficient for wheels front_left, rear_left, rear_right, front_right
    };

    struct ObjectStateStruct
    {
        struct ObjectInfoStruct info;
        roadmanager::Position   pos;
    };

    struct ObjectInfoStructDat
    {
        int            id;
        int            model_id;
        int            obj_type;      // 0=None, 1=Vehicle, 2=Pedestrian, 3=MiscObj (see Object::Type enum)
        int            obj_category;  // sub type for vehicle, pedestrian and miscobj
        int            ctrl_type;     // See Controller::Type enum
        float          timeStamp;
        char           name[NAME_LEN];
        float          speed;
        float          wheel_angle;  // Only used for vehicle
        float          wheel_rot;    // Only used for vehicle
        OSCBoundingBox boundingbox;
        int            scaleMode;       // 0=None, 1=BoundingBoxToModel, 2=ModelToBoundingBox (see enum EntityScaleMode)
        int            visibilityMask;  // bitmask according to Object::Visibility (1 = Graphics, 2 = Traffic, 4 = Sensors)
    };

    struct ObjectPositionStructDat
    {
        float x;
        float y;
        float z;
        float h;
        float p;
        float r;
        int   roadId;
        int   laneId;
        float offset;
        float t;
        float s;
    };

    struct ObjectStateStructDat
    {
        struct ObjectInfoStructDat     info;
        struct ObjectPositionStructDat pos;
    };

    typedef struct
    {
        int  version;
        char odr_filename[DAT_FILENAME_SIZE];
        char model_filename[DAT_FILENAME_SIZE];
    } DatHeader;

    class ObjectState
    {
    public:
        ObjectState();
        ObjectState(int                    id,
                    std::string            name,
                    int                    obj_type,
                    int                    obj_category,
                    int                    obj_role,
                    int                    model_id,
                    std::string            model3d,
                    int                    ctrl_type,
                    OSCBoundingBox         boundingbox,
                    int                    scaleMode,
                    int                    visibilityMask,
                    double                 timestamp,
                    double                 speed,
                    double                 wheel_angle,
                    double                 wheel_rot,
                    double                 rear_axle_z_pos,
                    double                 front_axle_x_pos,
                    double                 front_axle_z_pos,
                    roadmanager::Position *pos);
        ObjectState(int            id,
                    std::string    name,
                    int            obj_type,
                    int            obj_category,
                    int            obj_role,
                    int            model_id,
                    int            ctrl_type,
                    OSCBoundingBox boundingbox,
                    int            scaleMode,
                    int            visibilityMask,
                    double         timestamp,
                    double         speed,
                    double         wheel_angle,
                    double         wheel_rot,
                    double         rear_axle_z_pos,
                    double         x,
                    double         y,
                    double         z,
                    double         h,
                    double         p,
                    double         r);
        ObjectState(int            id,
                    std::string    name,
                    int            obj_type,
                    int            obj_category,
                    int            obj_role,
                    int            model_id,
                    int            ctrl_type,
                    OSCBoundingBox boundingbox,
                    int            scaleMode,
                    int            visibilityMask,
                    double         timestamp,
                    double         speed,
                    double         wheel_angle,
                    double         wheel_rot,
                    double         rear_axle_z_pos,
                    int            roadId,
                    int            laneId,
                    double         laneOffset,
                    double         s);
        ObjectState(int            id,
                    std::string    name,
                    int            obj_type,
                    int            obj_category,
                    int            obj_role,
                    int            model_id,
                    int            ctrl_type,
                    OSCBoundingBox boundingbox,
                    int            scaleMode,
                    int            visibilityMask,
                    double         timestamp,
                    double         speed,
                    double         wheel_angle,
                    double         wheel_rot,
                    double         rear_axle_z_pos,
                    int            roadId,
                    double         lateralOffset,
                    double         s);

        ObjectState(const ObjectState &)            = default;
        ObjectState &operator=(const ObjectState &) = default;

        ObjectStateStruct getStruct()
        {
            return state_;
        }

        void Print();
        void clearDirtyBits()
        {
            dirty_ = 0;
        }

        ObjectStateStruct state_;
        unsigned int      dirty_;

    private:
        friend class ScenarioGateway;
    };

    class ScenarioGateway
    {
    public:
        ScenarioGateway();
        ~ScenarioGateway();

        int reportObject(int                    id,
                         std::string            name,
                         int                    obj_type,
                         int                    obj_category,
                         int                    obj_role,
                         int                    model_id,
                         std::string            model3d,
                         int                    ctrl_type,
                         OSCBoundingBox         boundingbox,
                         int                    scaleMode,
                         int                    visibilityMask,
                         double                 timestamp,
                         double                 speed,
                         double                 wheel_angle,
                         double                 wheel_rot,
                         double                 rear_axle_z_pos,
                         double                 front_axle_x_pos,
                         double                 front_axle_z_pos,
                         roadmanager::Position *pos);

        int reportObject(int            id,
                         std::string    name,
                         int            obj_type,
                         int            obj_category,
                         int            obj_role,
                         int            model_id,
                         int            ctrl_type,
                         OSCBoundingBox boundingbox,
                         int            scaleMode,
                         int            visibilityMask,
                         double         timestamp,
                         double         speed,
                         double         wheel_angle,
                         double         wheel_rot,
                         double         rear_axle_z_pos,
                         double         x,
                         double         y,
                         double         z,
                         double         h,
                         double         p,
                         double         r);

        int reportObject(int            id,
                         std::string    name,
                         int            obj_type,
                         int            obj_category,
                         int            obj_role,
                         int            model_id,
                         int            ctrl_type,
                         OSCBoundingBox boundingbox,
                         int            scaleMode,
                         int            visibilityMask,
                         double         timestamp,
                         double         speed,
                         double         wheel_angle,
                         double         wheel_rot,
                         double         rear_axle_z_pos,
                         double         x,
                         double         y,
                         double         h);

        int reportObject(int            id,
                         std::string    name,
                         int            obj_type,
                         int            obj_category,
                         int            obj_role,
                         int            model_id,
                         int            ctrl_type,
                         OSCBoundingBox boundingbox,
                         int            scaleMode,
                         int            visibilityMask,
                         double         timestamp,
                         double         speed,
                         double         wheel_angle,
                         double         wheel_rot,
                         double         rear_axle_z_pos,
                         int            roadId,
                         int            laneId,
                         double         laneOffset,
                         double         s);

        int reportObject(int            id,
                         std::string    name,
                         int            obj_type,
                         int            obj_category,
                         int            obj_role,
                         int            model_id,
                         int            ctrl_type,
                         OSCBoundingBox boundingbox,
                         int            scaleMode,
                         int            visibilityMask,
                         double         timestamp,
                         double         speed,
                         double         wheel_angle,
                         double         wheel_rot,
                         double         rear_axle_z_pos,
                         int            roadId,
                         double         lateralOffset,
                         double         s);

        int updateObjectPos(int id, double timestamp, roadmanager::Position *pos);
        int updateObjectRoadPos(int id, double timestamp, int roadId, double lateralOffset, double s);
        int updateObjectLanePos(int id, double timestamp, int roadId, int laneId, double offset, double s);
        int updateObjectWorldPos(int id, double timestamp, double x, double y, double z, double h, double p, double r);
        int updateObjectWorldPosMode(int id, double timestamp, double x, double y, double z, double h, double p, double r, int mode);
        int updateObjectWorldPosXYH(int id, double timestamp, double x, double y, double h);
        int updateObjectWorldPosXYHMode(int id, double timestamp, double x, double y, double h, int mode);
        int updateObjectSpeed(int id, double timestamp, double speed);
        int updateObjectVel(int id, double timestamp, double x_vel, double y_vel, double z_vel);
        int updateObjectAcc(int id, double timestamp, double x_acc, double y_acc, double z_acc);
        int updateObjectAngularVel(int id, double timestamp, double h_rate, double p_rate, double r_rate);
        int updateObjectAngularAcc(int id, double timestamp, double h_acc, double p_acc, double r_acc);
        int updateObjectWheelAngle(int id, double timestamp, double wheelAngle);
        int updateObjectWheelRotation(int id, double timestamp, double wheelRotation);
        int updateObjectVisibilityMask(int id, int visibilityMask);
        int updateObjectControllerType(int id, int controllerType);
        int updateObjectFrictionCoefficients(int id, double friction[4]);

        /**
        Specify if and how position object will align to the road. The setting is done for individual components:
        Z (elevation), Heading, Pitch, Roll and separately for set- and update operation. Set operations represents
        when position is affected by API calls, e.g. updateObjectWorldPos(). Update operations represents when the
        position is updated implicitly by the scenarioengine, e.g. default controller moving a vehicle along the lane.
        @param id Id of the object
        @param mode Bitmask combining values from roadmanager::PosMode enum
        example: To set relative z and absolute roll: (Z_REL | R_ABS) or (7 | 12288) = (7 + 12288) = 12295
        @param type Type of operations the setting applies to. SET (explicit set-functions) or UPDATE (updates by controllers),
        according to roadmanager::PosModeType
        */
        int setObjectPositionMode(int id, int type, int mode);

        /**
        Set default alignment mode for SET or UPDATE operations. See roadmanager::Position::GetModeDefault() to find out
        what are the default modes.
        @param id Id of the object
        @param type Type of operations the setting applies to. SET (explicit set-functions) or UPDATE (updates by controllers),
        according to roadmanager::PosModeType
        */
        int setObjectPositionModeDefault(int id, int type);

        bool isObjectReported(int id);
        void clearDirtyBits();

        void removeObject(int id);
        void removeObject(std::string name);
        int  getNumberOfObjects()
        {
            return static_cast<int>(objectState_.size());
        }
        ObjectState getObjectStateByIdx(int idx)
        {
            return *objectState_[static_cast<unsigned int>(idx)];
        }
        ObjectState *getObjectStatePtrByIdx(int idx)
        {
            return objectState_[static_cast<unsigned int>(idx)].get();
        }
        ObjectState *getObjectStatePtrById(int id);
        int          getObjectStateById(int idx, ObjectState &objState);
        void         WriteStatesToFile();
        int          RecordToFile(std::string filename, std::string odr_filename, std::string model_filename);

        std::vector<std::unique_ptr<ObjectState>> objectState_;

    private:
        int updateObjectInfo(ObjectState *obj_state, double timestamp, int visibilityMask, double speed, double wheel_angle, double wheel_rot);
        std::ofstream data_file_;
    };

}  // namespace scenarioengine
