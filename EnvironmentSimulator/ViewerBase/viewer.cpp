
#include "viewer.hpp"

#include <osgDB/ReadFile>
#include <osg/ComputeBoundsVisitor>
#include <osg/LineWidth>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osgViewer/ViewerEventHandlers>

#define SHADOW_SCALE 1.20
#define SHADOW_MODEL_FILENAME "../../resources/models/shadow_face.osgb"
#define LOD_DIST 3000
#define LOD_SCALE_DEFAULT 1.2

using namespace viewer;


CarModel::CarModel(osg::LOD *model)
{
	node_ = model;
	txNode_ = new osg::PositionAttitudeTransform();
	txNode_->addChild(node_);
}

CarModel::~CarModel()
{

}


Viewer::Viewer(roadmanager::OpenDrive *odrManager, const char *modelFilename, osg::ArgumentParser arguments)
{
	line_node_ = new osg::Group;
	if (!CreateRoadLines(odrManager, line_node_))
	{
		printf("Viewer::Viewer Failed to create road lines!\n");
	}
	if (!ReadCarModels())
	{
		printf("Viewer::Viewer Failed to read car models!\n");
	}
	
	lodScale_ = LOD_SCALE_DEFAULT;
	currentCarInFocus_ = 0;
	camMode_ = 0;
	osgViewer_ = new osgViewer::Viewer(arguments);
		
	arguments.getApplicationUsage()->addCommandLineOption("--lodScale <number>", "LOD Scale");
	arguments.read("--lodScale", lodScale_);

	// Set up 3D vehicle models
	ReadCarModels();

	// add road network lines
	odrManager_ = odrManager;
	line_node_ = new osg::Group;
	CreateRoadLines(odrManager_, line_node_);

	// set the scene to render
	rootnode_ = new osg::MatrixTransform;
	envTx_ = new osg::PositionAttitudeTransform;
	AddEnvironment(modelFilename);	// add environment
	rootnode_->addChild(envTx_);
	rootnode_->addChild(line_node_);
	osgViewer_->setSceneData(rootnode_);

	// Setup the camera models
	nodeTrackerManipulator_ = new osgGA::NodeTrackerManipulator;
	nodeTrackerManipulator_->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
	nodeTrackerManipulator_->setRotationMode(osgGA::NodeTrackerManipulator::ELEVATION_AZIM);
	nodeTrackerManipulator_->setVerticalAxisFixed(true);
#if 0 // generates unwanted debug output on stdout, plus wrong axis
	nodeTrackerManipulator->setTrackNode(cars[currentCarInFocus].node);
#endif

	osg::ref_ptr<osgGA::TrackballManipulator> trackBallManipulator;
	trackBallManipulator = new osgGA::TrackballManipulator;
	trackBallManipulator->setVerticalAxisFixed(true);

	osg::ref_ptr<osgGA::OrbitManipulator> orbitManipulator;
	orbitManipulator = new osgGA::OrbitManipulator;
	orbitManipulator->setVerticalAxisFixed(true);

	rubberbandManipulator_ = new osgGA::RubberbandManipulator;
	rubberbandManipulator_->setTrackNode(envTx_);
	rubberbandManipulator_->calculateCameraDistance();

	{
		osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

		keyswitchManipulator->addMatrixManipulator('1', "Rubberband", rubberbandManipulator_.get());
		keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
		keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
		keyswitchManipulator->addMatrixManipulator('4', "Terrain", new osgGA::TerrainManipulator());
		keyswitchManipulator->addMatrixManipulator('5', "Orbit", new osgGA::OrbitManipulator());
		keyswitchManipulator->addMatrixManipulator('6', "FirstPerson", new osgGA::FirstPersonManipulator());
		keyswitchManipulator->addMatrixManipulator('7', "Spherical", new osgGA::SphericalManipulator());
		keyswitchManipulator->addMatrixManipulator('8', "NodeTracker", nodeTrackerManipulator_.get());
		keyswitchManipulator->addMatrixManipulator('9', "Trackball", trackBallManipulator.get());

		osgViewer_->setCameraManipulator(keyswitchManipulator.get());
	}

	osgViewer_->addEventHandler(new KeyboardEventHandler(this));

	osgViewer_->getCamera()->setLODScale(lodScale_);
	osgViewer_->getCamera()->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	osgViewer_->getCamera()->setClearColor(osg::Vec4(0.5f, 0.75f, 1.0f, 0.0f));
	//	viewer.getCamera()->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f));

	// add the window size toggle handler
	osgViewer_->addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler
	osgViewer_->addEventHandler(new osgViewer::StatsHandler);

	// Light
	osgViewer_->setLightingMode(osg::View::SKY_LIGHT);
	osg::Light *light = osgViewer_->getLight();
	light->setPosition(osg::Vec4(50000, -50000, 70000, 1));
	light->setDirection(osg::Vec3(-1, 1, -1));
	float ambient = 0.6;
	light->setAmbient(osg::Vec4(ambient, ambient, 0.9*ambient, 1));
	light->setDiffuse(osg::Vec4(1, 1, 0.85, 1));

	osgViewer_->realize();
}

Viewer::~Viewer()
{
	for (auto &car : cars_)
	{
		delete(car);
	}
	delete(osgViewer_);
}

CarModel* Viewer::AddCar(int modelId)
{
	cars_.push_back(new CarModel(carModels_[modelId]));
	rootnode_->addChild(cars_.back()->txNode_);
	// Focus on first added car
	if (cars_.size() == 1)
	{
		currentCarInFocus_ = 1;
		rubberbandManipulator_->setTrackNode(cars_.back()->txNode_);
		rubberbandManipulator_->calculateCameraDistance();
		nodeTrackerManipulator_->setTrackNode(cars_.back()->node_);
	}
	return cars_.back();
}

bool Viewer::ReadCarModels()
{
	osg::ref_ptr<osg::Node> node;
	osg::ref_ptr<osg::Node> shadow;
	osg::ref_ptr<osg::PositionAttitudeTransform> txShadow;

	// Load shadow face
	shadow = osgDB::readNodeFile(SHADOW_MODEL_FILENAME);
	if (!shadow)
	{
		printf("Failed to shadow model %s\n", SHADOW_MODEL_FILENAME);
		return false;
	}

	for (int i = 0; i < carModelsFiles_.size(); i++)
	{
		node = osgDB::readNodeFile(carModelsFiles_[i]);
		if (!node)
		{
			printf("Failed to load car model %s\n", carModelsFiles_[i].c_str());
			return false;
		}

		const osg::MatrixList& m = node->getWorldMatrices();
		osg::ComputeBoundsVisitor cbv;
		node->accept(cbv);
		osg::BoundingBox bb = cbv.getBoundingBox();
		osg::Vec3 minV = bb._min * m.front();
		osg::Vec3 maxV = bb._max * m.front();

		double xc, yc, dx, dy;
		dx = maxV.x() - minV.x();
		dy = maxV.y() - minV.y();
		xc = (maxV.x() + minV.x())/2;
		yc = (maxV.y() + minV.y()) / 2;

		txShadow = new osg::PositionAttitudeTransform;
		txShadow->setPosition(osg::Vec3d(xc, yc, 0.0));
		txShadow->setScale(osg::Vec3d(SHADOW_SCALE*(dx / 2), SHADOW_SCALE*(dy / 2), 1.0));
		txShadow->addChild(shadow);

		osg::Group *group = new osg::Group;
		group->addChild(node);
		group->addChild(txShadow);

		osg::LOD *lod = new osg::LOD();
		lod->addChild(group);
		lod->setRange(0, 0, LOD_DIST);

		carModels_.push_back(lod);
	}
	
	return true;
}

bool Viewer::CreateRoadLines(roadmanager::OpenDrive* od, osg::Group* parent)
{
	double step_length_target = 1;
	double z_offset = 0.10;
	roadmanager::Position* pos = new roadmanager::Position();
	osg::Vec3 point(0, 0, 0);

	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		roadmanager::Road *road = od->GetRoadByIdx(r);

		for (int i = 0; i < road->GetNumberOfLaneSections(); i++)
		{
			roadmanager::LaneSection *lane_section = road->GetLaneSectionByIdx(i);
			double s_start = lane_section->GetS();
			double s_end = s_start + lane_section->GetLength();
			int steps = (int)((s_end - s_start) / step_length_target);
			double step_length = (s_end - s_start) / steps;

			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				roadmanager::Lane *lane = lane_section->GetLaneByIdx(j);
				if (!lane->IsDriving() && lane->GetId() != 0)
				{
					continue;
				}

				osg::ref_ptr<osg::Geometry> beam(new osg::Geometry);
				osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
				for (int k = 0; k < steps + 1; k++)
				{
					pos->Set(road->GetId(), lane->GetId(), fmin(s_end, s_start + k * step_length), 0, i);

					point.set(pos->GetX(), pos->GetY(), pos->GetZ() + z_offset);
					points->push_back(point);
				}
				osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;

				osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth();
				if (lane->GetId() == 0)
				{
					lineWidth->setWidth(5.0f);
					color->push_back(osg::Vec4(0xBB / (float)0xFF, 0x44 / (float)0xFF, 0x44 / (float)0xFF, 1.0));
				}
				else
				{
					lineWidth->setWidth(2.0f);
					color->push_back(osg::Vec4(0x33 / (float)0xFF, 0x33 / (float)0xFF, 0xBB / (float)0xFF, 1.0));
				}
				beam->setVertexArray(points.get());
				beam->setColorArray(color.get());
				beam->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
				beam->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, points->size()));
				beam->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
				beam->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
				parent->addChild(beam);
			}
		}
	}
	return true;
}

int Viewer::AddEnvironment(const char* filename)
{
	// remove current model, if any
	if (environment_ != 0)
	{
		printf("Removing current env\n");
		envTx_->removeChild(environment_);
		//free(envTx);
	}

	// load and apply new model
	environment_ = osgDB::readNodeFile(filename);
	if (environment_ == 0)
	{
		std::cout << "Failed to read environment model " << filename << "!\n";
		return -1;
	}

	envTx_->addChild(environment_);
	envTx_->setPosition(osg::Vec3(0, 0, 0));
	envTx_->setScale(osg::Vec3(1, 1, 1));
	envTx_->setAttitude(osg::Quat(0, 0, 0, 1));

	return 0;
}

bool KeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
	switch (ea.getKey())
	{
	case('c'):
		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->camMode_ += 1;
			if (viewer_->camMode_ >= osgGA::RubberbandManipulator::RB_NUM_MODES)
			{
				viewer_->camMode_ = 0;
			}
			viewer_->rubberbandManipulator_->setMode(viewer_->camMode_);
		}
		break;
	case('o'):
	{
		static bool visible = true;

		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			visible = !visible;
			viewer_->line_node_->setNodeMask(visible ? 0xffffffff : 0x0);
		}
	}
	break;
	case('m'):
	{
		static bool visible = true;

		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			visible = !visible;
			viewer_->envTx_->setNodeMask(visible ? 0xffffffff : 0x0);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Tab):
		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			int step = (ea.getModKeyMask() & osgGA::GUIEventAdapter::KEY_Shift_L) ? -1 : 1;
			int nextIndex = (viewer_->currentCarInFocus_ + step) % (viewer_->cars_.size() + 1);

			bool zoom;  // when going to or from environment model, recalc distance
			if (viewer_->currentCarInFocus_ == 0 || nextIndex == 0)
			{
				zoom = true;
			}
			else
			{
				zoom = false;
			}

			viewer_->currentCarInFocus_ = nextIndex;
			if (viewer_->currentCarInFocus_ == 0)
			{
				// Use an additional index for total environment
				viewer_->rubberbandManipulator_->setTrackNode(viewer_->envTx_, zoom);
				viewer_->nodeTrackerManipulator_->setTrackNode(viewer_->envTx_);
			}
			else
			{
				viewer_->rubberbandManipulator_->setTrackNode(viewer_->cars_[viewer_->currentCarInFocus_ - 1]->txNode_, zoom);
				viewer_->nodeTrackerManipulator_->setTrackNode(viewer_->cars_[viewer_->currentCarInFocus_ - 1]->node_);
			}
		}
		break;
	}
	return false;
}

