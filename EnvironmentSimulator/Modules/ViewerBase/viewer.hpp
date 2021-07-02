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

extern double color_green[3];
extern double color_gray[3];
extern double color_dark_gray[3];
extern double color_red[3];
extern double color_blue[3];
extern double color_yellow[3];
extern double color_white[3];
extern double color_black[3];

using namespace scenarioengine;

namespace viewer
{
	typedef enum
	{
		NODE_MASK_NONE =             (0),
		NODE_MASK_OBJECT_SENSORS =   (1 << 0),
		NODE_MASK_TRAIL_LINES =      (1 << 1),
		NODE_MASK_TRAIL_DOTS =       (1 << 2),
		NODE_MASK_ODR_FEATURES =     (1 << 3),
		NODE_MASK_OSI_POINTS =       (1 << 4),
		NODE_MASK_OSI_LINES =        (1 << 5),
		NODE_MASK_ENV_MODEL =        (1 << 6),
		NODE_MASK_ENTITY_MODEL =     (1 << 7),
		NODE_MASK_ENTITY_BB =        (1 << 8),
		NODE_MASK_INFO =             (1 << 9),
		NODE_MASK_ROAD_SENSORS =     (1 << 10),
		NODE_MASK_TRAJECTORY_LINES = (1 << 11),
	} NodeMask;

	class PolyLine
	{
	public:
		osg::ref_ptr<osg::Vec3Array> pline_vertex_data_;
		osg::ref_ptr<osg::Vec4Array> color_;
		osg::ref_ptr<osg::Geometry> pline_geom_;
		osg::ref_ptr<osg::Geometry> dots_geom_;
		osg::ref_ptr<osg::ShapeDrawable> dot3D_shape_;
		osg::ref_ptr<osg::Geode> dot3D_geode_;
		osg::ref_ptr<osg::Group> dots3D_group_;

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
		PolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize) :
			PolyLine(parent, points, color, width, dotsize, false) {}

		/**
		* Create and visualize a set of connected line segments defined by an array of points.
		* Points are not vrepresented by GL points with uniform screen size (same size regardless of distance).
		* Vertices are not visualized.
		* @param parent osg group which to add the line geometries
		* @param points Points defining the polyline vertices
		* @param color Red, green, blue and alpha (transparency) of the line segments and optional vertex dots range [0.0:1.0]
		* @param widh Width of the polyline
		*/
		PolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width) :
			PolyLine(parent, points, color, width, 0.0, false) {}

		void SetPoints(osg::ref_ptr<osg::Vec3Array> points);
		void AddPoint(osg::Vec3 point);
		void Update();
		void Redraw();
		void SetNodeMaskLines(int nodemask);
		void SetNodeMaskDots(int nodemask);
	private:
		bool dots3D_;
		void Add3DDot(osg::Vec3 pos);
		osg::ref_ptr<osg::DrawArrays> pline_array_;
		osg::ref_ptr<osg::DrawArrays> dots_array_;
	};

	class SensorViewFrustum
	{
	public:
		osg::ref_ptr<osg::PositionAttitudeTransform> txNode_;
		osg::ref_ptr<osg::Group> line_group_;
		std::vector<PolyLine*> plines_;
		ObjectSensor *sensor_;

		SensorViewFrustum(ObjectSensor *sensor, osg::Group *parent);
		~SensorViewFrustum();
		void Update();
	};

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

		std::vector<TrajVertex> vertices_;
		osg::Group* parent_;
		osg::Node* node_;
		roadmanager::RMTrajectory* activeRMTrajectory_;

		Trajectory(osg::Group* parent, osgViewer::Viewer* viewer);
		~Trajectory() {}

		void SetActiveRMTrajectory(roadmanager::RMTrajectory* RMTrajectory);
		void Disable();

	private:
		osgViewer::Viewer* viewer_;
		PolyLine* pline_;
	};

	class PointSensor
	{
	public:
		osg::ref_ptr<osg::Group> group_;
		osg::ref_ptr<osg::PositionAttitudeTransform> ball_;
		double ball_radius_;
		osg::ref_ptr<osg::Geometry> line_;
		osg::ref_ptr<osg::Vec3Array> line_vertex_data_;

		osg::Vec3 pivot_pos;
		osg::Vec3 target_pos;

		PointSensor() : line_(0), line_vertex_data_(0), ball_(0) {};
		void Show() { group_->setNodeMask(NODE_MASK_ROAD_SENSORS); }
		void Hide() { group_->setNodeMask(0x0); };
	};

	class EntityModel
	{
	public:

		typedef enum {
			ENTITY_TYPE_VEHICLE,
			ENTITY_TYPE_OTHER
		} EntityType;

		osg::ref_ptr<osg::Group> group_;
		osg::ref_ptr<osg::LOD> lod_;
		osg::ref_ptr<osg::PositionAttitudeTransform> txNode_;
		osg::ref_ptr<osg::Group> bb_;
		osg::Quat quat_;
		osg::ref_ptr<osg::Group> parent_;

		double size_x;
		double size_y;
		double center_x;
		double center_y;
		Trajectory* trajectory_;
		static const int entity_type_ = ENTITY_TYPE_OTHER;
		virtual int GetType() { return entity_type_; }

		std::string name_;
		std::string filename_;
		osg::ref_ptr<osg::BlendColor> blend_color_;
		osg::ref_ptr<osg::StateSet> state_set_;

		EntityModel(osgViewer::Viewer* viewer, osg::ref_ptr<osg::Group> group, osg::ref_ptr<osg::Group> parent, osg::ref_ptr<osg::Group>
			trail_parent, osg::ref_ptr<osg::Group>traj_parent, osg::ref_ptr<osg::Node> dot_node, osg::Vec4 trail_color, std::string name);
		~EntityModel();
		void SetPosition(double x, double y, double z);
		void SetRotation(double hRoad, double pRoad, double hRelative, double r);
		void SetRotation(double h, double p, double r);

		void SetTransparency(double factor);


		PolyLine* trail_;
		osgViewer::Viewer* viewer_;
	};

	class CarModel : public EntityModel
	{
	public:
		std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> wheel_;
		double wheel_angle_;
		double wheel_rot_;
		PointSensor* road_sensor_;
		PointSensor* lane_sensor_;
		PointSensor* trail_sensor_;
		PointSensor* steering_sensor_;
		static const int entity_type_ = ENTITY_TYPE_VEHICLE;
		virtual int GetType() { return entity_type_; }

		CarModel(osgViewer::Viewer* viewer, osg::ref_ptr<osg::Group> group, osg::ref_ptr<osg::Group> parent, osg::ref_ptr<osg::Group>
			trail_parent, osg::ref_ptr<osg::Group>traj_parent, osg::ref_ptr<osg::Node> dot_node, osg::Vec4 trail_color, std::string name);
		~CarModel();
		osg::ref_ptr<osg::PositionAttitudeTransform>  AddWheel(osg::ref_ptr<osg::Node> carNode, const char* wheelName);
		void UpdateWheels(double wheel_angle, double wheel_rotation);
		void UpdateWheelsDelta(double wheel_angle, double wheel_rotation_delta);
	};

	class VisibilityCallback : public osg::NodeCallback
	{
	public:
		VisibilityCallback(osg::Node* node, scenarioengine::Object* object, EntityModel* entity)
		{
			node_ = (osg::LOD*)node;
			object_ = object;
			entity_ = entity;
		}
		virtual void operator()(osg::Node*, osg::NodeVisitor*);

	protected:
		osg::ref_ptr<osgAnimation::InCubicMotion> _motion;

	private:
		scenarioengine::Object* object_;
		EntityModel* entity_;
		osg::LOD* node_;
	};

	typedef struct
	{
		int key_;
		int modKeyMask_;
		bool down_;
	} KeyEvent;

	typedef enum
	{
		ENTITY_HIDE,
		ENTITY_3D_MODEL,
		ENTITY_BOUNDINGBOX,
		ENTITY_BOTH
	} ENTITY_SHOW_MODE;

	typedef void (*KeyEventCallbackFunc)(KeyEvent*, void*);

	typedef struct
	{
		KeyEventCallbackFunc func;
		void* data;
	} KeyEventCallback;

	class Viewer
	{
	public:
		int currentCarInFocus_;
		int camMode_;
		osg::ref_ptr<osg::Group> line_node_;

		// Vehicle position debug visualization
		osg::ref_ptr<osg::Node> shadow_node_;

		// Trail dot model
		osg::ref_ptr<osg::Node> dot_node_;

		// Road debug visualization
		osg::ref_ptr<osg::Group> odrLines_;
		osg::ref_ptr<osg::Group> osiFeatures_;
		osg::ref_ptr<osg::Group> trajectoryLines_;
		osg::ref_ptr<osg::PositionAttitudeTransform> envTx_;
		osg::ref_ptr<osg::Node> environment_;
		osg::ref_ptr<osgGA::RubberbandManipulator> rubberbandManipulator_;
		osg::ref_ptr<osgGA::NodeTrackerManipulator> nodeTrackerManipulator_;
		std::vector<EntityModel*> entities_;
		float lodScale_;
		osgViewer::Viewer *osgViewer_;
		osg::MatrixTransform* rootnode_;
		osg::ref_ptr<osg::Group> roadSensors_;
		osg::ref_ptr<osg::Group> trails_;
		roadmanager::OpenDrive *odrManager_;
		bool showInfoText;
		RoadGeom* roadGeom;

		std::string exe_path_;
		std::vector<KeyEventCallback> callback_;

		osg::ref_ptr<osg::Camera> infoTextCamera;
		osg::ref_ptr<osgText::Text> infoText;

		std::vector<PolyLine> polyLine_;

		Viewer(roadmanager::OpenDrive *odrManager, const char* modelFilename, const char* scenarioFilename, const char* exe_path, osg::ArgumentParser arguments, SE_Options* opt = 0);
		~Viewer();
		static void PrintUsage();
		void SetCameraMode(int mode);
		void UpdateCameraFOV();
		void SetVehicleInFocus(int idx);
		int GetEntityInFocus() { return currentCarInFocus_; }
		EntityModel* AddEntityModel(std::string modelFilepath, osg::Vec4 trail_color, EntityModel::EntityType type,
			bool road_sensor, std::string name, OSCBoundingBox *boundingBox);
		void RemoveCar(std::string name);
		int LoadShadowfile(std::string vehicleModelFilename);
		int AddEnvironment(const char* filename);
		osg::ref_ptr<osg::Group> LoadEntityModel(const char *filename);
		void UpdateSensor(PointSensor *sensor);
		void SensorSetPivotPos(PointSensor *sensor, double x, double y, double z);
		void SensorSetTargetPos(PointSensor *sensor, double x, double y, double z);
		void UpdateRoadSensors(PointSensor *road_sensor, PointSensor *lane_sensor, roadmanager::Position *pos);
		void setKeyUp(bool pressed) { keyUp_ = pressed; }
		void setKeyDown(bool pressed) { keyDown_ = pressed; }
		void setKeyLeft(bool pressed) { keyLeft_ = pressed; }
		void setKeyRight(bool pressed) { keyRight_ = pressed; }
		bool getKeyUp() { return keyUp_; }
		bool getKeyDown() { return keyDown_; }
		bool getKeyLeft() { return keyLeft_; }
		bool getKeyRight() { return keyRight_; }
		void SetQuitRequest(bool value) { quit_request_ = value; }
		bool GetQuitRequest() { return quit_request_;  }
		void SetInfoTextProjection(int width, int height);
		void SetInfoText(const char* text);
		void SetNodeMaskBits(int bits);
		void SetNodeMaskBits(int mask, int bits);
		void ClearNodeMaskBits(int bits);
		void ToggleNodeMaskBits(int bits);
		int GetNodeMaskBit(int mask);
		PointSensor* CreateSensor(double color[], bool create_ball, bool create_line, double ball_radius, double line_width);
		bool CreateRoadSensors(CarModel *vehicle_model);
		void SetWindowTitle(std::string title);
		void SetWindowTitleFromArgs(std::vector<std::string> &arg);
		void SetWindowTitleFromArgs(int argc, char* argv[]);
		void RegisterKeyEventCallback(KeyEventCallbackFunc func, void* data);
		PolyLine* AddPolyLine(osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize=0);
		PolyLine* AddPolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize=0);

	private:

		bool CreateRoadLines(roadmanager::OpenDrive* od);
		bool CreateRoadMarkLines(roadmanager::OpenDrive* od);
		int CreateOutlineObject(roadmanager::Outline* outline);
		osg::ref_ptr<osg::PositionAttitudeTransform> LoadRoadFeature(roadmanager::Road* road, std::string filename);
		int CreateRoadSignsAndObjects(roadmanager::OpenDrive* od);
		bool keyUp_;
		bool keyDown_;
		bool keyLeft_;
		bool keyRight_;
		bool quit_request_;
	};

	class ViewerEventHandler : public osgGA::GUIEventHandler
	{
	public:
		ViewerEventHandler(Viewer *viewer) : viewer_(viewer) {}
		bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&);

	private:
		Viewer* viewer_;
	};
}



#endif  // VIEWER_HPP_

