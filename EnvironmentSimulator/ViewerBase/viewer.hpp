#ifndef VIEWER_HPP_
#define VIEWER_HPP_

#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osgGA/NodeTrackerManipulator>
#include <osg/MatrixTransform>
#include <string>

#include "RubberbandManipulator.h"
#include "roadmanager.hpp"

static const char* carModelsFiles_[] =
{
	"p1800.osgb",
	"car1.osgb",
	"car2.osgb",
	"car3.osgb",
	"car4.osgb",
	"car5.osgb",
	"car6.osgb",
	"bus1.osgb",
	"bus2.osgb",
	"bus3.osgb",
	"truck1.osgb",
	"truck2.osgb",
	"truck3.osgb"
};

namespace viewer
{
	class CarModel
	{
	public:
		osg::ref_ptr<osg::LOD> node_;
		osg::ref_ptr<osg::PositionAttitudeTransform> txNode_;
		std::vector<osg::ref_ptr<osg::PositionAttitudeTransform>> wheel_;
		osg::ref_ptr<osg::LOD> model;
		osg::Quat quat_;
		double size_x;
		double size_y;
		double center_x;
		double center_y;

		CarModel(osg::ref_ptr<osg::LOD> lod);
		~CarModel();
		void SetPosition(double x, double y, double z);
		void SetRotation(double h, double p, double r);
		void UpdateWheels(double wheel_angle, double wheel_rotation);

		osg::ref_ptr<osg::PositionAttitudeTransform>  AddWheel(osg::ref_ptr<osg::Node> carNode, const char *wheelName);
	};

	class Viewer
	{
	public:
		int currentCarInFocus_;
		int camMode_;
		osg::ref_ptr<osg::Group> line_node_;
		
		// Driver model steering debug visualization
		osg::ref_ptr<osg::Geometry> dm_steering_target_line_;
		osg::ref_ptr<osg::Vec3Array> dm_steering_target_line_vertexData_;
		osg::ref_ptr<osg::Geometry> dm_steering_target_point_;
		osg::ref_ptr<osg::Vec3Array> dm_steering_target_point_data_;

		// Vehicle position debug visualization
		osg::ref_ptr<osg::Node> shadow_node_;
		osg::ref_ptr<osg::Vec3Array> vertexData;
		osg::ref_ptr<osg::Geometry> vehicleLine_;
		osg::ref_ptr<osg::Vec3Array> pointData;
		osg::ref_ptr<osg::Geometry> vehiclePoint_;

		// Road debug visualization
		osg::ref_ptr<osg::Group> odrLines_;
		osg::ref_ptr<osg::PositionAttitudeTransform> envTx_;
		osg::ref_ptr<osg::Node> environment_;
		osg::ref_ptr<osgGA::RubberbandManipulator> rubberbandManipulator_;
		osg::ref_ptr<osgGA::NodeTrackerManipulator> nodeTrackerManipulator_;
		std::vector<CarModel*> cars_;
		float lodScale_;
		osgViewer::Viewer *osgViewer_;
		osg::MatrixTransform* rootnode_;
		roadmanager::OpenDrive *odrManager_;
		std::vector<osg::ref_ptr<osg::LOD>> carModels_;

		Viewer(roadmanager::OpenDrive *odrManager, const char *modelFilename, osg::ArgumentParser arguments);
		~Viewer();
		CarModel* AddCar(int modelId);
		int AddEnvironment(const char* filename);
		osg::ref_ptr<osg::LOD> LoadCarModel(const char *filename);
		void UpdateDriverModelPoint(double x, double y, double z);
		void UpdateVLine(double x, double y, double z);
		void UpdateVPoints(double xt, double yt, double xl, double yl, double z);
		void setKeyUp(bool pressed) { keyUp_ = pressed; }
		void setKeyDown(bool pressed) { keyDown_ = pressed; }
		void setKeyLeft(bool pressed) { keyLeft_ = pressed; }
		void setKeyRight(bool pressed) { keyRight_ = pressed; }
		bool getKeyUp() { return keyUp_; }
		bool getKeyDown() { return keyDown_; }
		bool getKeyLeft() { return keyLeft_; }
		bool getKeyRight() { return keyRight_; }

	private:

		std::string modelFilename_;

		bool ReadCarModels();
		bool CreateRoadLines(roadmanager::OpenDrive* od, osg::Group* parent);
		bool CreateVLineAndPoint(osg::Group* parent);
		bool CreateDriverModelLineAndPoint(osg::Group* parent);
		bool keyUp_;
		bool keyDown_;
		bool keyLeft_;
		bool keyRight_;
	};

	class KeyboardEventHandler : public osgGA::GUIEventHandler
	{
	public:
		KeyboardEventHandler(Viewer *viewer) : viewer_(viewer) {}
		bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&);

	private:
		Viewer * viewer_;
	};
}



#endif  // VIEWER_HPP_

