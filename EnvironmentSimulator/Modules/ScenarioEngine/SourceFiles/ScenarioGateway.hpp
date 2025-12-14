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
#include "PacketHandler.hpp"

namespace scenarioengine
{

#define NAME_LEN 32

    struct ObjectInfoStruct
    {
        int                      id       = -1;
        id_t                     g_id     = ID_UNDEFINED;
        int                      model_id = -1;
        std::string              model3d;
        int                      obj_type       = 0;  // 0=None, 1=Vehicle, 2=Pedestrian, 3=MiscObj (see Object::Type enum)
        int                      obj_category   = 0;  // sub type for vehicle, pedestrian and miscobj
        int                      obj_role       = 0;  // role for vehicle and pedestrian
        int                      ctrl_type      = 0;  // See Controller::Type enum
        double                   timeStamp      = 0.0;
        char                     name[NAME_LEN] = "";
        double                   speed          = 0.0;
        double                   rear_axle_z_pos;   // z coordinate of the middle of rear axle under neutral load conditions
        double                   front_axle_x_pos;  // x coordinate of the middle of front axle under neutral load conditions
        double                   front_axle_z_pos;  // z coordinate of the middle of front axle under neutral load conditions
        OSCBoundingBox           boundingbox    = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
        int                      scaleMode      = 0;     // 0=None, 1=BoundingBoxToModel, 2=ModelToBoundingBox (see enum EntityScaleMode)
        int                      visibilityMask = 0xff;  // bitmask according to Object::Visibility (1 = Graphics, 2 = Traffic, 4 = Sensors)
        std::vector<WheelData>   wheel_data;             // make room for maximum number of wheels
        std::vector<std::string> source_reference;       // object property with same name mapping to OSI "source_reference"
        double                   refpoint_x_offset;      // x offset of the reference point
        double                   model_x_offset;         // x offset of the 3D model relative to the object reference point
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
        std::string    name;
        float          speed;
        float          wheel_angle;  // Only used for vehicle
        float          wheel_rot;    // Only used for vehicle
        OSCBoundingBox boundingbox;
        int            scaleMode;       // 0=None, 1=BoundingBoxToModel, 2=ModelToBoundingBox (see enum EntityScaleMode)
        int            visibilityMask;  // bitmask according to Object::Visibility (1 = Graphics, 2 = Traffic, 4 = Sensors)
        bool           active;
    };

    struct ObjectPositionStructDat
    {
        float x;
        float y;
        float z;
        float h;
        float p;
        float r;
        id_t  roadId;
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

    class ObjectState
    {
    public:
        ObjectState();
        ObjectState(int                          id,
                    id_t                         g_id,
                    std::string                  name,
                    int                          obj_type,
                    int                          obj_category,
                    int                          obj_role,
                    int                          model_id,
                    std::string                  model3d,
                    int                          ctrl_type,
                    OSCBoundingBox               boundingbox,
                    int                          scaleMode,
                    int                          visibilityMask,
                    double                       timestamp,
                    double                       speed,
                    double                       wheel_angle,
                    double                       wheel_rot,
                    double                       rear_axle_z_pos,
                    double                       front_axle_x_pos,
                    double                       front_axle_z_pos,
                    const roadmanager::Position *pos,
                    std::vector<std::string>     source_reference,
                    double                       refpoint_x_offset,
                    double                       model_x_offset);

        ObjectState(int            id,
                    id_t           g_id,
                    std::string    name,
                    int            obj_type,
                    int            obj_category,
                    int            obj_role,
                    int            model_id,
                    std::string    model3d,
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
                    id_t           g_id,
                    std::string    name,
                    int            obj_type,
                    int            obj_category,
                    int            obj_role,
                    int            model_id,
                    std::string    model3d,
                    int            ctrl_type,
                    OSCBoundingBox boundingbox,
                    int            scaleMode,
                    int            visibilityMask,
                    double         timestamp,
                    double         speed,
                    double         wheel_angle,
                    double         wheel_rot,
                    double         rear_axle_z_pos,
                    id_t           roadId,
                    int            laneId,
                    double         laneOffset,
                    double         s);

        ObjectState(int            id,
                    id_t           g_id,
                    std::string    name,
                    int            obj_type,
                    int            obj_category,
                    int            obj_role,
                    int            model_id,
                    std::string    model3d,
                    int            ctrl_type,
                    OSCBoundingBox boundingbox,
                    int            scaleMode,
                    int            visibilityMask,
                    double         timestamp,
                    double         speed,
                    double         wheel_angle,
                    double         wheel_rot,
                    double         rear_axle_z_pos,
                    id_t           roadId,
                    double         lateralOffset,
                    double         s);

        ObjectState(const ObjectState &)            = default;
        ObjectState &operator=(const ObjectState &) = default;

        ObjectStateStruct getStruct() const
        {
            return state_;
        }

        void Print();
        void clearDirtyBits()
        {
            dirty_ = 0;
        }

        unsigned int ReadDirtyBits() const
        {
            return dirty_;
        }

        int GetOSIIndex() const
        {
            return osi_index_;
        }

        void SetOSIIndex(int osi_index)
        {
            osi_index_ = osi_index;
        }

        ObjectStateStruct state_;

    private:
        friend class ScenarioGateway;
        unsigned int dirty_;
        bool         osi_update_flag_ = true;
        int          osi_index_       = -1;
    };

    class ScenarioGateway
    {
    public:
        ScenarioGateway();
        ~ScenarioGateway();

        int reportObjectPos(int                      id,
                            id_t                     g_id,
                            std::string              name,
                            int                      obj_type,
                            int                      obj_category,
                            int                      obj_role,
                            int                      model_id,
                            std::string              model3d,
                            int                      ctrl_type,
                            OSCBoundingBox           boundingbox,
                            int                      scaleMode,
                            int                      visibilityMask,
                            double                   timestamp,
                            double                   speed,
                            double                   wheel_angle,
                            double                   wheel_rot,
                            double                   rear_axle_z_pos,
                            double                   front_axle_x_pos,
                            double                   front_axle_z_pos,
                            roadmanager::Position   *pos,
                            std::vector<std::string> source_reference,
                            double                   refpoint_x_offset,
                            double                   model_x_offset);

        int reportObjectXYZHPR(int            id,
                               id_t           g_id,
                               std::string    name,
                               int            obj_type,
                               int            obj_category,
                               int            obj_role,
                               int            model_id,
                               std::string    model3d,
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

        int reportObjectXYH(int            id,
                            id_t           g_id,
                            std::string    name,
                            int            obj_type,
                            int            obj_category,
                            int            obj_role,
                            int            model_id,
                            std::string    model3d,
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

        int reportObjectLanePos(int            id,
                                id_t           g_id,
                                std::string    name,
                                int            obj_type,
                                int            obj_category,
                                int            obj_role,
                                int            model_id,
                                std::string    model3d,
                                int            ctrl_type,
                                OSCBoundingBox boundingbox,
                                int            scaleMode,
                                int            visibilityMask,
                                double         timestamp,
                                double         speed,
                                double         wheel_angle,
                                double         wheel_rot,
                                double         rear_axle_z_pos,
                                id_t           roadId,
                                int            laneId,
                                double         laneOffset,
                                double         s);

        int reportObjectRoadPos(int            id,
                                id_t           g_id,
                                std::string    name,
                                int            obj_type,
                                int            obj_category,
                                int            obj_role,
                                int            model_id,
                                std::string    model3d,
                                int            ctrl_type,
                                OSCBoundingBox boundingbox,
                                int            scaleMode,
                                int            visibilityMask,
                                double         timestamp,
                                double         speed,
                                double         wheel_angle,
                                double         wheel_rot,
                                double         rear_axle_z_pos,
                                id_t           roadId,
                                double         lateralOffset,
                                double         s);

        int updateObjectPos(int id, double timestamp, const roadmanager::Position *pos);
        int updateObjectRoadPos(int id, double timestamp, id_t roadId, double lateralOffset, double s);
        int updateObjectLanePos(int id, double timestamp, id_t roadId, int laneId, double offset, double s);
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
        int updateObjectLaneTypeSnapMask(int id, double timestamp, int laneTypeMask);
        int updateObjectWheelRotation(int id, double timestamp, double wheelRotation);
        int updateObjectVisibilityMask(int id, int visibilityMask);
        int updateObjectControllerType(int id, int controllerType);
        int updateObjectBoundingBox(int id, OSCBoundingBox bb);
        int updateObjectWheelData(int id, std::vector<WheelData> wheel_data);

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
        int  getNumberOfObjects() const
        {
            return static_cast<int>(objectState_.size());
        }
        ObjectState getObjectStateByIdx(int idx) const
        {
            return *objectState_[static_cast<unsigned int>(idx)];
        }
        ObjectState *getObjectStatePtrByIdx(int idx)
        {
            return objectState_[static_cast<unsigned int>(idx)].get();
        }
        ObjectState *getObjectStatePtrById(int id);
        int          getObjectStateById(int id, ObjectState &objectState) const;
        void         WriteStatesToFile(const double simulation_time, const double dt);
        int          RecordToFile(std::string filename, std::string odr_filename, std::string model_filename, std::string git_rev);

        std::vector<std::unique_ptr<ObjectState>> objectState_;

        void SetDynamicSignals(std::vector<roadmanager::Signal *> dynamic_signals)
        {
            dynamic_signals_ = dynamic_signals;
        }
        void UpdateStoryBoardStateChanges(std::vector<std::string> storyboard_state_changes)
        {
            storyboard_state_changes_ = storyboard_state_changes;
        }

    private:
        int updateObjectInfo(ObjectState *obj_state, double timestamp, int visibilityMask, double speed, double wheel_angle, double wheel_rot);
        std::ofstream                      data_file_;
        Dat::DatWriter                     dat_writer_;
        std::vector<roadmanager::Signal *> dynamic_signals_;
        std::vector<std::string>           storyboard_state_changes_;
    };

}  // namespace scenarioengine
