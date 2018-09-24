
#include "viewer.hpp"

#include <osgDB/ReadFile>
#include <osg/ComputeBoundsVisitor>
#include <osg/LineWidth>
#include <osg/Point>
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

// Derive a class from NodeVisitor to find a node with a  specific name.
class FindNamedNode : public osg::NodeVisitor
{
public:
	FindNamedNode(const std::string& name)
		: osg::NodeVisitor( // Traverse all children.
			osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
		_name(name) {}

	// This method gets called for every node in the scene graph. Check each node 
	// to see if its name matches out target. If so, save the node's address.
	virtual void apply(osg::Node& node)
	{
		if (node.getName() == _name)
			_node = &node;

		// Keep traversing the rest of the scene graph.
		traverse(node);
	}

	osg::Node* getNode() { return _node.get(); }

protected:
	std::string _name;
	osg::ref_ptr<osg::Node> _node;
};


osg::ref_ptr<osg::PositionAttitudeTransform> CarModel::AddWheel(osg::ref_ptr<osg::Node> carNode, const char *wheelName)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> tx_node = 0;

	// Find wheel node
	FindNamedNode fnn(wheelName);
	carNode->accept(fnn);	
	
	// Assume wheel is a tranformation node
	osg::MatrixTransform *node = dynamic_cast<osg::MatrixTransform*>(fnn.getNode());

	if (node != NULL)
	{
		tx_node = new osg::PositionAttitudeTransform;
		// We found the wheel. Put it under a useful transform node
		tx_node->addChild(node);

		// reset pivot point
		osg::Vec3d pos = node->getMatrix().getTrans();
		tx_node->setPivotPoint(pos);
		tx_node->setPosition(pos);

		osg::ref_ptr<osg::Group> parent = node->getParent(0);  // assume the wheel node only belongs to one vehicle
		parent->removeChild(node);
		parent->addChild(tx_node);

		wheel_.push_back(tx_node);
	}
	return tx_node;
}

CarModel::CarModel(osg::ref_ptr<osg::LOD> lod)
{
	node_ = lod;
	
	// Locate wheel nodes if available
	osg::ref_ptr<osg::Node> car_node = lod->getChild(0);
	AddWheel(car_node, "wheel_fl");
	AddWheel(car_node, "wheel_fr");
	AddWheel(car_node, "wheel_rr");
	AddWheel(car_node, "wheel_rl");

	// Extract boundingbox of car to calculate size and center
	osg::ComputeBoundsVisitor cbv;
	car_node->accept(cbv);
	osg::BoundingBox boundingBox = cbv.getBoundingBox();
	const osg::MatrixList& m = car_node->getWorldMatrices();
	osg::Vec3 minV = boundingBox._min * m.front();
	osg::Vec3 maxV = boundingBox._max * m.front();

	size_x = maxV.x() - minV.x();
	size_y = maxV.y() - minV.y();
	center_x = (maxV.x() + minV.x()) / 2;
	center_y = (maxV.y() + minV.y()) / 2;

	// Add car model to a tranform node
	txNode_ = new osg::PositionAttitudeTransform();
	txNode_->addChild(node_);
}

CarModel::~CarModel()
{

}

void CarModel::SetPosition(double x, double y, double z)
{
	txNode_->setPosition(osg::Vec3(x, y, z));
}

void CarModel::SetRotation(double h, double p, double r)
{
	quat_.makeRotate(
		r, osg::Vec3(1, 0, 0), // Roll
		p, osg::Vec3(0, 1, 0), // Pitch
		h, osg::Vec3(0, 0, 1)); // Heading
	txNode_->setAttitude(quat_);
}

void CarModel::UpdateWheels(double wheel_angle, double wheel_rotation)
{
	// Update wheel angles and rotation for front wheels
	osg::Quat quat;
	quat.makeRotate(
		0, osg::Vec3(1, 0, 0), // Roll
		wheel_rotation, osg::Vec3(0, 1, 0), // Pitch
		wheel_angle, osg::Vec3(0, 0, 1)); // Heading
	wheel_[0]->setAttitude(quat);
	wheel_[1]->setAttitude(quat);
	
	// Update rotation for rear wheels
	quat.makeRotate(
		0, osg::Vec3(1, 0, 0), // Roll
		wheel_rotation, osg::Vec3(0, 1, 0), // Pitch
		0, osg::Vec3(0, 0, 1)); // Heading
	wheel_[2]->setAttitude(quat);
	wheel_[3]->setAttitude(quat);
}

Viewer::Viewer(roadmanager::OpenDrive *odrManager, const char *modelFilename, osg::ArgumentParser arguments)
{
	odrManager_ = odrManager;

	if (!ReadCarModels())
	{
		printf("Viewer::Viewer Failed to read car models!\n");
	}
	lodScale_ = LOD_SCALE_DEFAULT;
	currentCarInFocus_ = 0;
	camMode_ = osgGA::RubberbandManipulator::RB_MODE_ORBIT;
	osgViewer_ = new osgViewer::Viewer(arguments);
		
	arguments.getApplicationUsage()->addCommandLineOption("--lodScale <number>", "LOD Scale");
	arguments.read("--lodScale", lodScale_);

	// Set up 3D vehicle models
	ReadCarModels();

	// set the scene to render
	rootnode_ = new osg::MatrixTransform;
	envTx_ = new osg::PositionAttitudeTransform;
	AddEnvironment(modelFilename);	// add environment
	rootnode_->addChild(envTx_);

	if (!CreateRoadLines(odrManager, rootnode_))
	{
		printf("Viewer::Viewer Failed to create road lines!\n");
	}
	if (!CreateVLineAndPoint(rootnode_))
	{
		printf("Viewer::Viewer Failed to create vehicle line!\n");
	}

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

	rubberbandManipulator_ = new osgGA::RubberbandManipulator(camMode_);
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
		currentCarInFocus_ = 0;
		rubberbandManipulator_->setTrackNode(cars_.back()->txNode_, true);
		rubberbandManipulator_->calculateCameraDistance();
		nodeTrackerManipulator_->setTrackNode(cars_.back()->node_);
	}
	return cars_.back();
}

osg::ref_ptr<osg::LOD> Viewer::LoadCarModel(const char *filename)
{
	static osg::ref_ptr<osg::Node> shadow_node = 0;
	osg::ref_ptr<osg::PositionAttitudeTransform> shadow_tx = 0;
	osg::ref_ptr<osg::Node> node;
	osg::ref_ptr<osg::LOD> lod = 0;

	if (shadow_node == 0)
	{
		shadow_node = osgDB::readNodeFile(SHADOW_MODEL_FILENAME);
		if (!shadow_node)
		{
			printf("Failed to shadow model %s\n", SHADOW_MODEL_FILENAME);
		}
	}

	node = osgDB::readNodeFile(filename);
	if (!node)
	{
		printf("Failed to load car model %s\n", filename);
		return 0;
	}

	osg::ComputeBoundsVisitor cbv;
	node->accept(cbv);
	osg::BoundingBox boundingBox = cbv.getBoundingBox();
	const osg::MatrixList& m = node->getWorldMatrices();
	osg::Vec3 minV = boundingBox._min * m.front();
	osg::Vec3 maxV = boundingBox._max * m.front();

	double xc, yc, dx, dy;
	dx = maxV.x() - minV.x();
	dy = maxV.y() - minV.y();
	xc = (maxV.x() + minV.x()) / 2;
	yc = (maxV.y() + minV.y()) / 2;

	shadow_tx = new osg::PositionAttitudeTransform;
	shadow_tx->setPosition(osg::Vec3d(xc, yc, 0.0));
	shadow_tx->setScale(osg::Vec3d(SHADOW_SCALE*(dx / 2), SHADOW_SCALE*(dy / 2), 1.0));
	shadow_tx->addChild(shadow_node);

	osg::ref_ptr<osg::Group> group = new osg::Group;
	group->addChild(node);
	group->addChild(shadow_tx);

	lod = new osg::LOD();
	lod->addChild(group);
	lod->setRange(0, 0, LOD_DIST);

	return lod;
}

bool Viewer::ReadCarModels()
{
	osg::ref_ptr<osg::LOD> lod;

	for (int i = 0; i < carModelsFiles_.size(); i++)
	{
		lod = LoadCarModel(carModelsFiles_[i].c_str());

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
	odrLines_ = new osg::Group;


	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		roadmanager::Road *road = od->GetRoadByIdx(r);

		// Road key points
		osg::ref_ptr<osg::Geometry> kp_geom = new osg::Geometry;
		osg::ref_ptr<osg::Vec3Array> kp_points = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> kp_color = new osg::Vec4Array;
		osg::ref_ptr<osg::Point> kp_point = new osg::Point();

		roadmanager::Geometry *geom;
		for (int i = 0; i < road->GetNumberOfGeometries()+1; i++)
		{
			if (i < road->GetNumberOfGeometries())
			{
				geom = road->GetGeometry(i);
				pos->SetTrackPos(road->GetId(), geom->GetS(), 0);
			}
			else
			{
				pos->SetTrackPos(road->GetId(), geom->GetS()+geom->GetLength(), 0);
			}

			point.set(pos->GetX(), pos->GetY(), pos->GetZ() + z_offset);
			kp_points->push_back(point);

			if (i == 0)
			{
				kp_color->push_back(osg::Vec4(0xEE / (float)0xFF, 0xAA / (float)0xFF, 0x44 / (float)0xFF, 1.0));
			}
			else
			{
				kp_color->push_back(osg::Vec4(0xBB / (float)0xFF, 0x44 / (float)0xFF, 0x44 / (float)0xFF, 1.0));
			}
		}

		kp_point->setSize(12.0f);
		kp_geom->setVertexArray(kp_points.get());
		kp_geom->setColorArray(kp_color.get());
		kp_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		kp_geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, kp_points->size()));
		kp_geom->getOrCreateStateSet()->setAttributeAndModes(kp_point, osg::StateAttribute::ON);
		kp_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

		odrLines_->addChild(kp_geom);

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
				osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
				osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
				osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
				osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth();

				if (!lane->IsDriving() && lane->GetId() != 0)
				{
					continue;
				}

				for (int k = 0; k < steps + 1; k++)
				{
					pos->SetLanePos(road->GetId(), lane->GetId(), fmin(s_end, s_start + k * step_length), 0, i);
					point.set(pos->GetX(), pos->GetY(), pos->GetZ() + z_offset);
					points->push_back(point);
				}

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

				geom->setVertexArray(points.get());
				geom->setColorArray(color.get());
				geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
				geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, points->size()));
				geom->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
				geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

				odrLines_->addChild(geom);
			}
		}
	}

	parent->addChild(odrLines_);
	return true;
}


bool Viewer::CreateVLineAndPoint(osg::Group* parent)
{
	// vehicle points
	pointData = new osg::Vec3Array;
	pointData->push_back(osg::Vec3d(0, 0, 0));
	pointData->push_back(osg::Vec3d(0, 0, 0));

	vehiclePoint_ = new osg::Geometry;
	vehiclePoint_->setCullingActive(false);
	vehiclePoint_->setVertexArray(pointData.get());
	vehiclePoint_->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, 2));

	osg::ref_ptr<osg::Point> point_point = new osg::Point;
	point_point->setSize(20.0f);
	vehiclePoint_->getOrCreateStateSet()->setAttributeAndModes(point_point, osg::StateAttribute::ON);
	vehiclePoint_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::Vec4Array> point_color = new osg::Vec4Array;
	point_color->push_back(osg::Vec4(0x44 / (float)0xFF, 0x44 / (float)0xFF, 0x44 / (float)0xFF, 1.0));
	point_color->push_back(osg::Vec4(0xCC / (float)0xFF, 0xCC / (float)0xFF, 0x33 / (float)0xFF, 1.0));
	vehiclePoint_->setDataVariance(osg::Object::DYNAMIC);
	vehiclePoint_->setColorArray(point_color.get());
	vehiclePoint_->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	parent->addChild(vehiclePoint_);

	vertexData = new osg::Vec3Array;
	vertexData->push_back(osg::Vec3d(0, 0, 0));
	vertexData->push_back(osg::Vec3d(0, 0, 0));

	vehicleLine_ = new osg::Geometry();
	vehicleLine_->setCullingActive(false);
	vehicleLine_->setVertexArray(vertexData.get());
	vehicleLine_->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, 2));

	osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth();
	lineWidth->setWidth(4.0f);
	vehicleLine_->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
	vehicleLine_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(0xCC / (float)0xFF, 0xCC / (float)0xFF, 0x33 / (float)0xFF, 1.0));
	vehicleLine_->setColorArray(color.get());
	vehicleLine_->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
	vehicleLine_->setDataVariance(osg::Object::DYNAMIC);
	parent->addChild(vehicleLine_);

	return true;
}

void Viewer::UpdateVPoints(double xt, double yt, double xl, double yl, double z)
{
	double z_offset = 0.1;

	pointData->clear();
	pointData->push_back(osg::Vec3d(xt, yt, z + z_offset));
	pointData->push_back(osg::Vec3d(xl, yl, z + z_offset));
	vehiclePoint_->dirtyGLObjects();
	pointData->dirty();
}

void Viewer::UpdateVLine(double x, double y, double z)
{
	double z_offset = 0.1;
	osg::ref_ptr<osg::PositionAttitudeTransform> tx = cars_[0]->txNode_;

	vertexData->clear();
	vertexData->push_back(osg::Vec3d(tx->getPosition().x(), tx->getPosition().y(), tx->getPosition().z() + z_offset));
	vertexData->push_back(osg::Vec3d(x, y, z + z_offset));
	vehicleLine_->dirtyGLObjects();
	vertexData->dirty();
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
	if (strcmp(filename, ""))
	{
		environment_ = osgDB::readNodeFile(filename);
		if (environment_ == 0)
		{
			std::cout << "Failed to read environment model " << filename << "! If file is missing, check SharePoint/SharedDocuments/models" << std::endl;
			return -1;
		}

		envTx_->addChild(environment_);
	} 
	else
	{
		printf("AddEnvironment: No environment 3D model specified - go ahead without\n");
	}

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
			viewer_->odrLines_->setNodeMask(visible ? 0xffffffff : 0x0);
			viewer_->vehicleLine_->setNodeMask(visible ? 0xffffffff : 0x0);
			viewer_->vehiclePoint_->setNodeMask(visible ? 0xffffffff : 0x0);
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
			viewer_->currentCarInFocus_ = (viewer_->currentCarInFocus_ + step) % (int)viewer_->cars_.size();
			if (viewer_->currentCarInFocus_ < 0)
			{
				viewer_->currentCarInFocus_ += viewer_->cars_.size();
			}

			viewer_->rubberbandManipulator_->setTrackNode(viewer_->cars_[viewer_->currentCarInFocus_]->txNode_, false);
			viewer_->nodeTrackerManipulator_->setTrackNode(viewer_->cars_[viewer_->currentCarInFocus_]->node_);
		}
		break;
	}
	
	return false;
}

