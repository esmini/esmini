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

#ifndef VIEWER_HPP_
#define VIEWER_HPP_

#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/NodeTrackerManipulator>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osgText/Text>
#include <osgAnimation/EaseMotion>
#include <osg/BlendColor>
#include <osg/ShapeDrawable>
#include <string>

#include "RubberbandManipulator.hpp"
#include "IdealSensor.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "roadgeom.hpp"
#include "Entities.hpp"
#include "OSIReporter.hpp"
#include "OSCParameterDistribution.hpp"

extern float color_green[3];
extern float color_gray[3];
extern float color_dark_gray[3];
extern float color_red[3];
extern float color_blue[3];
extern float color_yellow[3];
extern float color_white[3];
extern float color_black[3];

using namespace scenarioengine;

namespace viewer
{
    typedef enum
    {
        NODE_MASK_NONE             = (0),
        NODE_MASK_OBJECT_SENSORS   = (1 << 0),
        NODE_MASK_TRAIL_LINES      = (1 << 1),
        NODE_MASK_TRAIL_DOTS       = (1 << 2),
        NODE_MASK_ODR_FEATURES     = (1 << 3),
        NODE_MASK_OSI_POINTS       = (1 << 4),
        NODE_MASK_OSI_LINES        = (1 << 5),
        NODE_MASK_ENV_MODEL        = (1 << 6),
        NODE_MASK_ENTITY_MODEL     = (1 << 7),
        NODE_MASK_ENTITY_BB        = (1 << 8),
        NODE_MASK_INFO             = (1 << 9),
        NODE_MASK_INFO_PER_OBJ     = (1 << 10),
        NODE_MASK_ROAD_SENSORS     = (1 << 11),
        NODE_MASK_TRAJECTORY_LINES = (1 << 12),
        NODE_MASK_ROUTE_WAYPOINTS  = (1 << 13),
        NODE_MASK_LIGHTS_STATE     = (1 << 14),
    } NodeMask;

    osg::Vec4 ODR2OSGColor(roadmanager::RoadMarkColor color);

    class PolyLine
    {
    public:
        osg::ref_ptr<osg::Vec3Array>     pline_vertex_data_;
        osg::ref_ptr<osg::Vec4Array>     color_;
        osg::ref_ptr<osg::Geometry>      pline_geom_;
        osg::ref_ptr<osg::Geometry>      dots_geom_;
        osg::ref_ptr<osg::ShapeDrawable> dot3D_shape_;
        osg::ref_ptr<osg::Geode>         dot3D_geode_;
        osg::ref_ptr<osg::Group>         dots3D_group_;

        /**
         * Create and visualize a set of connected line segments defined by an array of points.
         * @param parent osg group which to add the line geometries
         * @param points Points defining the polyline vertices
         * @param color Red, green, blue and alpha (transparency) of the line segments and optional vertex dots range [0.0:1.0]
         * @param widh Width of the polyline
         * @param dotsize Size of the dots. Set to 0.0 to disable dots. Size unit is meter for 3D dots and pixels for default GL points
         * @param dots3D If true the dots are represented by 3D shape, otherwise just a OpenGL point
         */
        PolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize, bool dots3D);

        /**
         * Create and visualize a set of connected line segments defined by an array of points.
         * Vertex points are represented by GL points with uniform screen size (same size regardless of distance).
         * @param parent osg group which to add the line geometries
         * @param points Points defining the polyline vertices
         * @param color Red, green, blue and alpha (transparency) of the line segments and optional vertex dots range [0.0:1.0]
         * @param widh Width of the polyline
         * @param dotsize Size of the dots. Set to 0.0 to disable dots. Size unit is meter for 3D dots and pixel for default GL points
         */
        PolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize)
            : PolyLine(parent, points, color, width, dotsize, false)
        {
        }

        /**
         * Create and visualize a set of connected line segments defined by an array of points.
         * Points are not vrepresented by GL points with uniform screen size (same size regardless of distance).
         * Vertices are not visualized.
         * @param parent osg group which to add the line geometries
         * @param points Points defining the polyline vertices
         * @param color Red, green, blue and alpha (transparency) of the line segments and optional vertex dots range [0.0:1.0]
         * @param widh Width of the polyline
         */
        PolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width)
            : PolyLine(parent, points, color, width, 0.0, false)
        {
        }

        void SetPoints(osg::ref_ptr<osg::Vec3Array> points);
        void AddPoint(osg::Vec3 point);
        void Reset();
        void Update();
        void Redraw();
        void SetNodeMaskLines(unsigned int nodemask);
        void SetNodeMaskDots(unsigned int nodemask);

    private:
        bool                          dots3D_;
        void                          Add3DDot(osg::Vec3 pos);
        osg::ref_ptr<osg::DrawArrays> pline_array_;
        osg::ref_ptr<osg::DrawArrays> dots_array_;
    };

    class SensorViewFrustum
    {
    public:
        osg::ref_ptr<osg::PositionAttitudeTransform> txNode_;
        osg::ref_ptr<osg::Group>                     line_group_;
        std::vector<PolyLine*>                       plines_;
        ObjectSensor*                                sensor_;

        SensorViewFrustum(ObjectSensor* sensor, osg::Group* parent);
        ~SensorViewFrustum();
        void Update();
    };

#ifdef _USE_OSI

    class OSIDetectedPoint
    {
    public:
        osg::ref_ptr<osg::Group>     parent_;
        osg::ref_ptr<osg::Geometry>  osi_detection_geom_;
        osg::ref_ptr<osg::Vec3Array> osi_detection_points_;
        osg::ref_ptr<osg::Vec4Array> osi_detection_color_;
        bool                         showing_;

        OSIDetectedPoint(const osg::Vec3 point, osg::ref_ptr<osg::Group> parent);
        ~OSIDetectedPoint();
        void Show()
        {
            osi_detection_geom_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
            showing_ = true;
        };
        void Hide()
        {
            osi_detection_geom_->setNodeMask(0x0);
            showing_ = false;
        };
        void Update(const osg::Vec3 point);
    };

    class OSIDetectedCar
    {
    public:
        osg::ref_ptr<osg::Group>                     parent_;
        osg::ref_ptr<osg::Group>                     car_;
        osg::ref_ptr<osg::Geode>                     osi_detection_geode_box_;
        osg::ref_ptr<osg::Geode>                     osi_detection_geode_center_;
        osg::ref_ptr<osg::PositionAttitudeTransform> osi_detection_tx_;
        osg::Vec3                                    bb_dimensions_;
        bool                                         showing_;

        OSIDetectedCar(const osg::Vec3 point, double h, double w, double l, osg::ref_ptr<osg::Group> parent);
        ~OSIDetectedCar();
        void Show();
        void Hide();
        void Update(const osg::Vec3 point);
    };

    class OSISensorDetection
    {
    public:
        osg::ref_ptr<osg::Group> parent_;
        osg::ref_ptr<osg::Group> detected_points_group_;
        osg::ref_ptr<osg::Group> detected_bb_group_;

        std::map<uint64_t, OSIDetectedPoint*> detected_points_;
        std::map<uint64_t, OSIDetectedCar*>   detected_cars_;

        OSISensorDetection(osg::ref_ptr<osg::Group> parent);
        ~OSISensorDetection();
        void Update(osi3::SensorView* sv);
    };

#endif  // _USE_OSI

    class Trajectory
    {
    public:
        typedef struct
        {
            double x;
            double y;
            double z;
            double h;
        } TrajVertex;

        std::vector<TrajVertex>    vertices_;
        osg::Group*                parent_;
        osg::Node*                 node_;
        roadmanager::RMTrajectory* activeRMTrajectory_;

        Trajectory(osg::Group* parent, osgViewer::Viewer* viewer);
        ~Trajectory()
        {
        }

        void SetActiveRMTrajectory(roadmanager::RMTrajectory* RMTrajectory);
        void Disable();

    private:
        osgViewer::Viewer*        viewer_;
        std::unique_ptr<PolyLine> pline_;
    };

    class RouteWayPoints
    {
    public:
        osg::ref_ptr<osg::Group> parent_;
        osg::ref_ptr<osg::Group> group_;

        RouteWayPoints(osg::ref_ptr<osg::Group> parent, osg::Vec4 color);
        ~RouteWayPoints();

        osg::ref_ptr<osg::Geode> CreateWayPointGeometry(double x, double y, double z, double h, double scale);
        void                     SetWayPoints(roadmanager::Route* route);
    };

    class PointSensor
    {
    public:
        osg::ref_ptr<osg::Group>                     group_;
        osg::ref_ptr<osg::PositionAttitudeTransform> ball_;
        double                                       ball_radius_;
        osg::ref_ptr<osg::Geometry>                  line_;
        osg::ref_ptr<osg::Vec3Array>                 line_vertex_data_;

        osg::Vec3 pivot_pos;
        osg::Vec3 target_pos;

        PointSensor() : ball_(0), line_(0), line_vertex_data_(0){};
        void Show()
        {
            group_->setNodeMask(NODE_MASK_ROAD_SENSORS);
        }
        void Hide()
        {
            group_->setNodeMask(0x0);
        };
        bool IsVisible()
        {
            return group_->getNodeMask() & NODE_MASK_ROAD_SENSORS;
        }
    };

    class OnScreenText
    {
    public:
        OnScreenText()
        {
            string_[0] = 0;
        }
        char                        string_[256];
        osg::ref_ptr<osg::Geode>    geode_;
        osg::ref_ptr<osgText::Text> osg_text_;
    };

    class EntityModel
    {
    public:
        enum class EntityType
        {
            ENTITY  = 1 << 0,
            MOVING  = ENTITY | 1 << 1,
            VEHICLE = MOVING | 1 << 2,
        };

        osg::ref_ptr<osg::Group>                     group_;
        osg::ref_ptr<osg::Group>                     model_;
        osg::ref_ptr<osg::LOD>                       lod_;
        osg::ref_ptr<osg::PositionAttitudeTransform> txNode_;
        osg::PositionAttitudeTransform*              txShadow_;
        osg::Quat                                    quat_;
        osg::ref_ptr<osg::Group>                     parent_;
        osg::BoundingBox                             modelBB_;
        osg::ref_ptr<osg::Group>                     bbGroup_;

        std::unique_ptr<Trajectory> trajectory_;
        static const EntityType     entity_type_ = EntityType::ENTITY;
        virtual EntityType          GetType()
        {
            return entity_type_;
        }

        /* Returns true if type is or inherits from MOVING */
        bool IsMoving()
        {
            return static_cast<int>(GetType()) & 1 << 1;
        }

        /* Returns true if type is or inherits from VEHICLE */
        bool IsVehicle()
        {
            return static_cast<int>(GetType()) & 1 << 2;
        }

        std::string                   name_;
        std::string                   filename_;
        osg::ref_ptr<osg::BlendColor> blend_color_;
        osg::ref_ptr<osg::StateSet>   state_set_;

        EntityModel(osgViewer::Viewer*       viewer,
                    osg::ref_ptr<osg::Group> group,
                    osg::ref_ptr<osg::Group> parent,
                    osg::ref_ptr<osg::Group> trail_parent,
                    osg::ref_ptr<osg::Group> traj_parent,
                    osg::ref_ptr<osg::Node>  dot_node,
                    osg::ref_ptr<osg::Group> route_waypoint_parent,
                    osg::Vec4                trail_color,
                    std::string              name);
        virtual ~EntityModel();
        void SetPosition(double x, double y, double z);
        void SetRotation(double hRoad, double pRoad, double hRelative, double r);
        void SetRotation(double h, double p, double r);

        void SetTransparency(double factor);

        std::unique_ptr<PolyLine>       trail_;
        std::unique_ptr<RouteWayPoints> routewaypoints_;
        osgViewer::Viewer*              viewer_;
        OnScreenText                    on_screen_info_;
    };

    class MovingModel : public EntityModel
    {
    public:
        PointSensor*            road_sensor_;
        PointSensor*            lane_sensor_;
        PointSensor*            route_sensor_;
        PointSensor*            trail_sensor_;
        PointSensor*            steering_sensor_;
        static const EntityType entity_type_ = EntityType::MOVING;
        virtual EntityType      GetType()
        {
            return entity_type_;
        }

        MovingModel(osgViewer::Viewer*       viewer,
                    osg::ref_ptr<osg::Group> group,
                    osg::ref_ptr<osg::Group> parent,
                    osg::ref_ptr<osg::Group> trail_parent,
                    osg::ref_ptr<osg::Group> traj_parent,
                    osg::ref_ptr<osg::Node>  dot_node,
                    osg::ref_ptr<osg::Group> route_waypoint_parent,
                    osg::Vec4                trail_color,
                    std::string              name);
        ~MovingModel()
        {
        }
        void ShowRouteSensor(bool mode);
    };

    class CarModel : public MovingModel
    {
    public:
        std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> front_wheel_;
        std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> rear_wheel_;
        std::vector<osg::ref_ptr<osg::Geode>>                     light_material_;
        double                                                    wheel_angle_;
        double                                                    wheel_rot_;
        static const EntityType                                   entity_type_ = EntityType::VEHICLE;
        virtual EntityType                                        GetType()
        {
            return entity_type_;
        }

        CarModel(osgViewer::Viewer*       viewer,
                 osg::ref_ptr<osg::Group> group,
                 osg::ref_ptr<osg::Group> parent,
                 osg::ref_ptr<osg::Group> trail_parent,
                 osg::ref_ptr<osg::Group> traj_parent,
                 osg::ref_ptr<osg::Node>  dot_node,
                 osg::ref_ptr<osg::Group> route_waypoint_parent,
                 osg::Vec4                trail_color,
                 std::string              name,
                 bool                     light_action_state);
        ~CarModel();
        osg::ref_ptr<osg::PositionAttitudeTransform> AddWheel(osg::ref_ptr<osg::Node> carNode, const char* wheelName);
        void                                         UpdateWheels(double wheel_angle, double wheel_rotation);
        void                                         UpdateWheelsDelta(double wheel_angle, double wheel_rotation_delta);
        void                                         AddLights(osg::ref_ptr<osg::Group> group, bool light_action_status);
        void                                         UpdateLight(Object::VehicleLightActionStatus* list);
    };

    class VisibilityCallback : public osg::NodeCallback
    {
    public:
        VisibilityCallback(scenarioengine::Object* object, EntityModel* entity)
        {
            object_ = object;
            entity_ = entity;
        }
        virtual void operator()(osg::Node*, osg::NodeVisitor*);

    protected:
        osg::ref_ptr<osgAnimation::InCubicMotion> _motion;

    private:
        scenarioengine::Object* object_;
        EntityModel*            entity_;
    };

    // Callback for fetching key strokes
    typedef struct
    {
        int  key_;
        int  modKeyMask_;
        bool down_;
    } KeyEvent;

    typedef void (*KeyEventCallbackFunc)(KeyEvent*, void*);

    typedef struct
    {
        KeyEventCallbackFunc func;
        void*                data;
    } KeyEventCallback;

    // Callback for fetching rendered image
    typedef void (*ImageCallbackFunc)(OffScreenImage*, void*);

    typedef struct
    {
        ImageCallbackFunc func;
        void*             data;
    } ImageCallback;

    class Viewer
    {
    public:
        int                      currentCarInFocus_;
        int                      camMode_;
        osg::ref_ptr<osg::Group> line_node_;

        // Vehicle position debug visualization
        osg::ref_ptr<osg::Node> shadow_node_;

        // Trail dot model
        osg::ref_ptr<osg::Node> dot_node_;

        // Road debug visualization
        osg::ref_ptr<osg::Group>                     odrLines_;
        osg::ref_ptr<osg::Group>                     osiFeatures_;
        osg::ref_ptr<osg::Group>                     trajectoryLines_;
        osg::ref_ptr<osg::Group>                     routewaypoints_;
        osg::ref_ptr<osg::PositionAttitudeTransform> envTx_;
        osg::ref_ptr<osg::Node>                      environment_;
        osg::ref_ptr<osgGA::RubberbandManipulator>   rubberbandManipulator_;
        osg::ref_ptr<osgGA::NodeTrackerManipulator>  nodeTrackerManipulator_;
        std::vector<EntityModel*>                    entities_;
        float                                        lodScale_;
        osg::ref_ptr<osgViewer::Viewer>              osgViewer_;
        osg::MatrixTransform*                        rootnode_;
        osg::ref_ptr<osg::Group>                     roadSensors_;
        osg::ref_ptr<osg::Group>                     trails_;
        roadmanager::OpenDrive*                      odrManager_;
        std::unique_ptr<RoadGeom>                    roadGeom;

        std::string                   exe_path_;
        std::vector<KeyEventCallback> callback_;
        ImageCallback                 imgCallback_;

        osg::ref_ptr<osg::Camera>   infoTextCamera;
        osg::ref_ptr<osgText::Text> infoText;
        osg::ref_ptr<osg::Camera>   onScreenTextCamera;

        std::vector<PolyLine*> polyLine_;
        OffScreenImage         capturedImage_;
        int                    captureCounter_;
        int                    frameCounter_;
        int                    lightCounter_;

        SE_Semaphore renderSemaphore;
        SE_Mutex     imageMutex;

        Viewer(roadmanager::OpenDrive* odrManager,
               const char*             modelFilename,
               const char*             scenarioFilename,
               const char*             exe_path,
               osg::ArgumentParser     arguments,
               SE_Options*             opt = 0);
        ~Viewer();
        static void PrintUsage();
        void        AddCustomCamera(double x, double y, double z, double h, double p, bool fixed_pos);
        void        AddCustomCamera(double x, double y, double z, bool fixed_pos);
        void        AddCustomFixedTopCamera(double x, double y, double z, double rot);
        int         GetCameraPosAndRot(osg::Vec3& pos, osg::Vec3& rot);
        int         AddCustomLightSource(double x, double y, double z, double intensity);

        /**
         * Set mode of the esmini camera model
         * @param mode According to the RubberbandManipulator::CAMERA_MODE enum, plus any number of custom cameras. Set -1 to select the last.
         */
        void SetCameraMode(int mode);
        int  GetNumberOfCameraModes();
        void UpdateCameraFOV();
        void SetVehicleInFocus(int idx);
        int  GetEntityInFocus()
        {
            return currentCarInFocus_;
        }
        EntityModel*             CreateEntityModel(std::string             modelFilepath,
                                                   osg::Vec4               trail_color,
                                                   EntityModel::EntityType type,
                                                   bool                    road_sensor,
                                                   std::string             name,
                                                   OSCBoundingBox*         boundingBox,
                                                   EntityScaleMode         scaleMode = EntityScaleMode::NONE);
        int                      AddEntityModel(EntityModel* model);
        void                     RemoveCar(int index);
        void                     RemoveCar(std::string name);
        void                     ReplaceCar(int index, EntityModel* model);
        int                      LoadShadowfile(std::string vehicleModelFilename);
        int                      AddEnvironment(const char* filename);
        osg::ref_ptr<osg::Group> LoadEntityModel(const char* filename, osg::BoundingBox& bb);
        void                     UpdateSensor(PointSensor* sensor);
        void                     SensorSetPivotPos(PointSensor* sensor, double x, double y, double z);
        void                     SensorSetTargetPos(PointSensor* sensor, double x, double y, double z);
        void UpdateRoadSensors(PointSensor* road_sensor, PointSensor* route_sensor, PointSensor* lane_sensor, roadmanager::Position* pos);
        void setKeyUp(bool pressed)
        {
            keyUp_ = pressed;
        }
        void setKeyDown(bool pressed)
        {
            keyDown_ = pressed;
        }
        void setKeyLeft(bool pressed)
        {
            keyLeft_ = pressed;
        }
        void setKeyRight(bool pressed)
        {
            keyRight_ = pressed;
        }
        bool getKeyUp()
        {
            return keyUp_;
        }
        bool getKeyDown()
        {
            return keyDown_;
        }
        bool getKeyLeft()
        {
            return keyLeft_;
        }
        bool getKeyRight()
        {
            return keyRight_;
        }
        void SetQuitRequest(bool value)
        {
            quit_request_ = value;
        }
        bool GetQuitRequest()
        {
            return quit_request_;
        }
        void         SetInfoText(const char* text);
        void         SetNodeMaskBits(int bits);
        void         SetNodeMaskBits(int mask, int bits);
        void         ClearNodeMaskBits(int bits);
        void         ToggleNodeMaskBits(int bits);
        int          GetNodeMaskBit(int mask);
        void         SetCameraTrackNode(osg::ref_ptr<osg::Node> xform, bool calcDistance = false);
        PointSensor* CreateSensor(float color[], bool create_ball, bool create_line, double ball_radius, double line_width);
        bool         CreateRoadSensors(MovingModel* moving_model);
        void         SetWindowTitle(std::string title);
        void         SetWindowTitleFromArgs(std::vector<std::string>& arg);
        void         SetWindowTitleFromArgs(int argc, char* argv[]);
        void         RegisterKeyEventCallback(KeyEventCallbackFunc func, void* data);
        void         RegisterImageCallback(ImageCallbackFunc func, void* data);
        PolyLine*    AddPolyLine(osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize = 0);
        PolyLine*    AddPolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize = 0);

        void SaveImagesToFile(int nrOfFrames);
        int  GetSaveImagesToFile()
        {
            return saveImagesToFile_;
        }

        void SaveImagesToRAM(bool state)
        {
            saveImagesToRAM_ = state;
        };
        bool GetSaveImagesToRAM()
        {
            return saveImagesToRAM_;
        }
        void Frame();

        bool isLightStateAction;

    private:
        bool                                         CreateRoadLines(roadmanager::OpenDrive* od);
        bool                                         CreateRoadMarkLines(roadmanager::OpenDrive* od);
        int                                          CreateOutlineObject(roadmanager::Outline* outline, osg::Vec4 color);
        osg::ref_ptr<osg::PositionAttitudeTransform> LoadRoadFeature(roadmanager::Road* road, std::string filename);
        int                                          CreateRoadSignsAndObjects(roadmanager::OpenDrive* od);
        int                                          InitTraits(osg::ref_ptr<osg::GraphicsContext::Traits> traits,
                                                                int                                        x,
                                                                int                                        y,
                                                                int                                        w,
                                                                int                                        h,
                                                                int                                        samples,
                                                                bool                                       decoration,
                                                                int                                        screenNum,
                                                                bool                                       headless);
        int                                          AddGroundSurface();
        bool                                         keyUp_;
        bool                                         keyDown_;
        bool                                         keyLeft_;
        bool                                         keyRight_;
        bool                                         quit_request_;
        bool                                         saveImagesToRAM_;
        int                                          saveImagesToFile_;
        bool                                         disable_off_screen_;
        osgViewer::ViewerBase::ThreadingModel        initialThreadingModel_;

        struct
        {
            int x;
            int y;
            int w;
            int h;
        } winDim_;
    };

    class ViewerEventHandler : public osgGA::GUIEventHandler
    {
    public:
        ViewerEventHandler(Viewer* viewer) : viewer_(viewer)
        {
        }

        using osgGA::GUIEventHandler::handle;
        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

    private:
        Viewer* viewer_;
    };
}  // namespace viewer

#endif  // VIEWER_HPP_
