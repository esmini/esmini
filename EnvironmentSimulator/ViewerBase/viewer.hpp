#ifndef VIEWER_HPP_
#define VIEWER_HPP_

#include <osg/PositionAttitudeTransform>
#include <osgViewer/Viewer>
#include <osgGA/NodeTrackerManipulator>
#include <osg/MatrixTransform>
#include <string>

#include "RubberbandManipulator.h"
#include "roadmanager.hpp"

namespace viewer
{
	class CarModel
	{
	public:
		CarModel(osg::LOD *model);
		~CarModel();
		osg::ref_ptr<osg::LOD> node_;
		osg::ref_ptr<osg::PositionAttitudeTransform> txNode_;
		osg::Quat quat_;
	};

	class Viewer
	{
	public:
		int currentCarInFocus_;
		int camMode_;
		osg::ref_ptr<osg::Group> line_node_;
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

	private:
		std::vector<std::string> carModelsFiles_ =
		{
			"../../resources/models/car1.osgb",
			"../../resources/models/car2.osgb",
			"../../resources/models/car3.osgb",
			"../../resources/models/car4.osgb",
			"../../resources/models/car5.osgb",
			"../../resources/models/car6.osgb",
			"../../resources/models/bus1.osgb",
			"../../resources/models/bus2.osgb",
			"../../resources/models/bus3.osgb",
			"../../resources/models/truck1.osgb",
			"../../resources/models/truck2.osgb",
			"../../resources/models/truck3.osgb",
			"../../resources/models/p1800.osgb"
		};
		
		bool ReadCarModels();
		bool CreateRoadLines(roadmanager::OpenDrive* od, osg::Group* parent);

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

