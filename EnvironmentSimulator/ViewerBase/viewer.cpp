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

#include "viewer.hpp"

#include <osgDB/ReadFile>
#include <osg/ComputeBoundsVisitor>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/Geode> 
#include <osg/Group> 
#include <osg/ShapeDrawable>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/Registry>
#include "CommonMini.hpp"

#define SHADOW_SCALE 1.20
#define SHADOW_MODEL_FILEPATH "shadow_face.osgb"  
#define ARROW_MODEL_FILEPATH "arrow.osgb"  
#define LOD_DIST 3000
#define LOD_SCALE_DEFAULT 1.0

static int COLOR_RED[3] = { 0xBB, 0x44, 0x44 };
static int COLOR_BLUE[3] = { 0x40, 0x60, 0xB0 };
static int COLOR_YELLOW[3] = { 0xCC, 0x99, 0x44 };
static int COLOR_WHITE[3] = { 0xCC, 0xCC, 0xCA };

//USE_OSGPLUGIN(fbx)
//USE_OSGPLUGIN(obj)
USE_OSGPLUGIN(osg2)
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
USE_SERIALIZER_WRAPPER_LIBRARY(osgSim)
USE_COMPRESSOR_WRAPPER(ZLibCompressor)
USE_GRAPHICSWINDOW()

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
	virtual void apply(osg::Group& node)
	{
		if (node.getName() == _name )
		{
			LOG("Found %s", _name.c_str());
			_node = &node;
		}
		else
		{

			// Keep traversing the rest of the scene graph.
			traverse(node);
		}
	}

	osg::Node* getNode() { return _node.get(); }

protected:
	std::string _name;
	osg::ref_ptr<osg::Group> _node;
};

void AlphaFadingCallback::operator()(osg::StateAttribute* sa, osg::NodeVisitor* nv)
{
	osg::Material* material = static_cast<osg::Material*>(sa);
	if (material)
	{
		internal_time_ += 0.034;
		if (internal_time_ > TRAIL_DOT_LIFE_SPAN)
		{
			_motion->update(0.034);  // assume 30 fps. Todo: replace with actual time
		}
		color_[3] = 1 - _motion->getValue();
		material->setDiffuse(osg::Material::FRONT_AND_BACK, color_);
		material->setAmbient(osg::Material::FRONT_AND_BACK, color_);
	}
}

TrailDot::TrailDot(float time, double x, double y, double z, double heading, osg::Group *parent, osg::ref_ptr<osg::Node> dot_node, osg::Vec4 trail_color)
{
	double dot_radius = 0.4;
	osg::ref_ptr<osg::Node> new_node;

	time_born = time;
	dot_ = new osg::PositionAttitudeTransform;
	dot_->setPosition(osg::Vec3(x, y, z));
	dot_->setScale(osg::Vec3(dot_radius, dot_radius, dot_radius));
	dot_->setAttitude(osg::Quat(heading, osg::Vec3(0, 0, 1)));

	if (dot_node == 0)
	{
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(new osg::ShapeDrawable(new osg::Box()));
		new_node = geode;

	}
	else
	{
		// Clone into a unique object for unique material and alpha fading
		new_node = dynamic_cast<osg::Node*>(dot_node->clone(osg::CopyOp()));
	}

	dot_->addChild(new_node);

	material_ = new osg::Material;
	material_->setDiffuse(osg::Material::FRONT_AND_BACK, trail_color);
	material_->setAmbient(osg::Material::FRONT_AND_BACK, trail_color);
	fade_callback_ = new AlphaFadingCallback(trail_color);
	material_->setUpdateCallback(fade_callback_);

	new_node->getOrCreateStateSet()->setAttributeAndModes(material_.get());
	osg::ref_ptr<osg::StateSet> stateset = new_node->getOrCreateStateSet(); // Get the StateSet of the group
	stateset->setAttribute(material_.get()); // Set Material 
	stateset->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
	stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	parent->addChild(dot_);
}

void TrailDot::Reset(float time, double x, double y, double z)
{
	dot_->setPosition(osg::Vec3(x, y, z));
	fade_callback_->Reset();
	time_born = time;
}

void Trail::AddDot(float time, double x, double y, double z, double heading)
{
	if (n_dots_ < TRAIL_MAX_DOTS)
	{
		dot_[current_] = new TrailDot(time, x, y, z, heading, parent_, dot_node_, color_);
		n_dots_++;
	}
	else
	{
		dot_[current_]->Reset(time, x, y, z);
	}

	if (++current_ >= TRAIL_MAX_DOTS)
	{
		current_ = 0;
	}
}

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
	else
	{
		LOG("Found no wheel node %s in vehicle model %s, ignoring", wheelName, carNode->getName().c_str());
	}
	return tx_node;
}

CarModel::CarModel(osg::ref_ptr<osg::LOD> lod, osg::ref_ptr<osg::Group> parent, osg::ref_ptr<osg::Group> trail_parent, osg::ref_ptr<osg::Node> dot_node, osg::Vec3 trail_color)
{
	if (!lod)
	{
		return;
	}
	node_ = lod;
	speed_sensor_ = 0;
	steering_sensor_ = 0;
	road_sensor_ = 0;
	lane_sensor_ = 0;
	trail_sensor_ = 0;

	wheel_angle_ = 0;
	wheel_rot_ = 0;

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
	txNode_->setName(car_node->getName());
	parent->addChild(txNode_);
	
	// Prepare trail of dots
	trail_ = new Trail(trail_parent, dot_node, trail_color);
}

CarModel::~CarModel()
{
	wheel_.clear();
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
	wheel_angle_ = wheel_angle;
	wheel_rot_ = wheel_rotation;
	
	osg::Quat quat;
	quat.makeRotate(
		0, osg::Vec3(1, 0, 0), // Roll
		wheel_rotation, osg::Vec3(0, 1, 0), // Pitch
		wheel_angle, osg::Vec3(0, 0, 1)); // Heading

	if (wheel_.size() < 4)
	{
		// Wheels not available
		return;
	}

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

void CarModel::UpdateWheelsDelta(double wheel_angle, double wheel_rotation_delta)
{
	UpdateWheels(wheel_angle, wheel_rot_ + wheel_rotation_delta);
}

Viewer::Viewer(roadmanager::OpenDrive *odrManager, const char *modelFilename, const char *scenarioFilename, osg::ArgumentParser arguments, bool create_ego_debug_lines)
{
	odrManager_ = odrManager;

	if(scenarioFilename != NULL)
	{ 
		scenarioDir_ = DirNameOf(scenarioFilename);
		LOG("Scenario file directory, %s, is used as pivot for model relative file paths", getScenarioDir().c_str());
	}
	else
	{
		// if scenario missing consider folder hosting the OpenDRIVE file to be the scenario directory
		scenarioDir_ = DirNameOf(odrManager->GetOpenDriveFilename());
		LOG("Scenario file missing, using OpenDRIVE directory, %s, as pivot for model relative file paths", getScenarioDir().c_str());
	}

	lodScale_ = LOD_SCALE_DEFAULT;
	currentCarInFocus_ = 0;
	keyUp_ = false;
	keyDown_ = false;
	keyLeft_ = false;
	keyRight_ = false;
	quit_request_ = false;
	showInfoText = true;  // show info text HUD per default
	camMode_ = osgGA::RubberbandManipulator::RB_MODE_ORBIT;

	arguments.getApplicationUsage()->addCommandLineOption("--lodScale <number>", "LOD Scale");
	arguments.read("--lodScale", lodScale_);

#ifdef _WIN32
	// When running on Linux in VirtualBox on Windows host - the application crashes when trying to apply AntiAlias as below
	// If someone know how to query AA capabilites, please replace this #ifdef with dynamic check so that AA can work on Linux as well
	osg::DisplaySettings::instance()->setNumMultiSamples(4);  // Set some AntiAliasing
#endif

	osgViewer_ = new osgViewer::Viewer(arguments); 


	// Load shadow geometry - assume it resides in the same resource folder as the environment model
	std::string shadowFilename = DirNameOf(modelFilename).append("/" + std::string(SHADOW_MODEL_FILEPATH));
	shadow_node_ = osgDB::readNodeFile(shadowFilename);
	if (!shadow_node_)
	{
		LOG("Failed to load shadow model %s\n", shadowFilename.c_str());
	}

	// Load shadow geometry - assume it resides in the same resource folder as the environment model
	std::string dotFilename = DirNameOf(modelFilename).append("/" + std::string(ARROW_MODEL_FILEPATH));
	dot_node_ = osgDB::readNodeFile(dotFilename);
	if (!dot_node_)
	{
		LOG("Failed to load trail dot model %s\n", dotFilename.c_str());
	}

	// set the scene to render
	rootnode_ = new osg::MatrixTransform;
	envTx_ = new osg::PositionAttitudeTransform;
	sensors_ = new osg::Group;
	rootnode_->addChild(sensors_);
	trails_ = new osg::Group;
	rootnode_->addChild(trails_);

	// add environment
	if (AddEnvironment(modelFilename) == -1)
	{
		LOG("Failed to load environment model: %s", modelFilename);
	}
	rootnode_->addChild(envTx_);

	if (odrManager->GetNumOfRoads() > 0 && !CreateRoadLines(odrManager, rootnode_))
	{
		LOG("Viewer::Viewer Failed to create road lines!\n");
	}

	osgViewer_->setSceneData(rootnode_);

	// Setup the camera models
	nodeTrackerManipulator_ = new osgGA::NodeTrackerManipulator;
	nodeTrackerManipulator_->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
	nodeTrackerManipulator_->setRotationMode(osgGA::NodeTrackerManipulator::ELEVATION_AZIM);
	nodeTrackerManipulator_->setVerticalAxisFixed(true);

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

	osgViewer_->addEventHandler(new ViewerEventHandler(this));

	osgViewer_->getCamera()->setLODScale(lodScale_);
	osgViewer_->getCamera()->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	osgViewer_->getCamera()->setClearColor(osg::Vec4(0.5f, 0.75f, 1.0f, 0.0f));

	// add the window size toggle handler
	osgViewer_->addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler
	osgViewer_->addEventHandler(new osgViewer::StatsHandler);

	osgViewer_->setReleaseContextAtEndOfFrameHint(false);

	// Light
	osgViewer_->setLightingMode(osg::View::SKY_LIGHT);
	osg::Light *light = osgViewer_->getLight();
	light->setPosition(osg::Vec4(50000, -50000, 70000, 1));
	light->setDirection(osg::Vec3(-1, 1, -1));
	float ambient = 0.4;
	light->setAmbient(osg::Vec4(ambient, ambient, 0.9*ambient, 1));
	light->setDiffuse(osg::Vec4(0.8, 0.8, 0.7, 1));

	osgViewer_->realize();

	// Overlay text
	osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
	osg::Vec4 layoutColor(0.9f, 0.9f, 0.9f, 1.0f);
	float layoutCharacterSize = 15.0f;

	infoText = new osgText::Text;
	infoText->setColor(layoutColor);
	infoText->setCharacterSize(layoutCharacterSize);
	infoText->setAxisAlignment(osgText::Text::SCREEN);
	infoText->setPosition(osg::Vec3(10, 10, 0));
	infoText->setDataVariance(osg::Object::DYNAMIC);

	textGeode->addDrawable(infoText);

	infoTextCamera = new osg::Camera;
	infoTextCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	infoTextCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	infoTextCamera->setRenderOrder(osg::Camera::POST_RENDER);
	infoTextCamera->setAllowEventFocus(false);
	infoTextCamera->addChild(textGeode.get());
	infoTextCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	osg::GraphicsContext* context = dynamic_cast<osgViewer::GraphicsWindow*>(osgViewer_->getCamera()->getGraphicsContext());
	SetInfoTextProjection(context->getTraits()->width, context->getTraits()->height);

	rootnode_->addChild(infoTextCamera);
}

Viewer::~Viewer()
{
	for (size_t i=0; i<cars_.size(); i++)
	{
		delete(cars_[i]);
	}
	cars_.clear();
}

CarModel* Viewer::AddCar(std::string modelFilepath, bool transparent, osg::Vec3 trail_color)
{
	if (modelFilepath == "")
	{
		LOG("No filename specified for car model!");
		return 0;
	}

	// Load 3D model
	std::string path = CombineDirectoryPathAndFilepath(getScenarioDir(), modelFilepath);

	osg::ref_ptr<osg::LOD> lod = LoadCarModel(path.c_str());

	if (lod == 0)
	{
		LOG("Failed to load car model: %s", path.c_str());
		return 0;
	}

	if (transparent)
	{
		osg::StateSet* state = lod->getOrCreateStateSet(); //Creating material

		osg::BlendColor* bc = new osg::BlendColor(osg::Vec4(1, 1, 1, transparent ? 0.4 : 1));
		state->setAttributeAndModes(bc);

		osg::BlendFunc* bf = new   //Blending
			osg::BlendFunc(osg::BlendFunc::CONSTANT_ALPHA, osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
	
		state->setAttributeAndModes(bf);
		state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
		state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}

	cars_.push_back(new CarModel(lod, rootnode_, trails_, dot_node_, trail_color));
	// Focus on first added car
	if (cars_.size() == 1)
	{
		currentCarInFocus_ = 0;
		rubberbandManipulator_->setTrackNode(cars_.back()->txNode_, true);
		nodeTrackerManipulator_->setTrackNode(cars_.back()->node_);
	}
	
	return cars_.back();
}

osg::ref_ptr<osg::LOD> Viewer::LoadCarModel(const char *filename)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> shadow_tx = 0;
	osg::ref_ptr<osg::Node> node;
	osg::ref_ptr<osg::LOD> lod = 0;

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
	shadow_tx->addChild(shadow_node_);

	osg::ref_ptr<osg::Group> group = new osg::Group;
	group->addChild(node);
	group->addChild(shadow_tx);
	group->setName(FileNameOf(filename));

	lod = new osg::LOD();
	lod->addChild(group);
	lod->setRange(0, 0, LOD_DIST);
	lod->setName(group->getName());

	return lod;
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
				kp_color->push_back(osg::Vec4(COLOR_YELLOW[0] / (float)0xFF, COLOR_YELLOW[1] / (float)0xFF, COLOR_YELLOW[2] / (float)0xFF, 1.0));
			}
			else
			{
				kp_color->push_back(osg::Vec4(COLOR_RED[0] / (float)0xFF, COLOR_RED[1] / (float)0xFF, COLOR_RED[2] / (float)0xFF, 1.0));
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
					pos->SetLanePos(road->GetId(), lane->GetId(), MIN(s_end, s_start + k * step_length), 0, i);
					point.set(pos->GetX(), pos->GetY(), pos->GetZ() + z_offset);
					points->push_back(point);
				}

				if (lane->GetId() == 0)
				{
					lineWidth->setWidth(5.0f);
					color->push_back(osg::Vec4(COLOR_RED[0] / (float)0xFF, COLOR_RED[1] / (float)0xFF, COLOR_RED[2] / (float)0xFF, 1.0));
				}
				else
				{
					lineWidth->setWidth(2.0f);
					color->push_back(osg::Vec4(COLOR_BLUE[0] / (float)0xFF, COLOR_BLUE[1] / (float)0xFF, COLOR_BLUE[2] / (float)0xFF, 1.0));
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


bool Viewer::CreateRoadSensors(CarModel *vehicle_model)
{
	vehicle_model->road_sensor_ = CreateSensor(COLOR_YELLOW, true, false, 0.25, 2.5);
	vehicle_model->lane_sensor_ = CreateSensor(COLOR_YELLOW, true, true, 0.25, 2.5);

	return true;
}

PointSensor* Viewer::CreateSensor(int color[], bool create_ball, bool create_line, double ball_radius, double line_width)
{
	PointSensor *sensor = new PointSensor();

	// Point
	if (create_ball)
	{
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere()));
		sensor->ball_ = new osg::PositionAttitudeTransform;
		sensor->ball_->setScale(osg::Vec3(ball_radius, ball_radius, ball_radius));
		sensor->ball_->addChild(geode);
		sensors_->addChild(sensor->ball_);

		osg::Material *material = new osg::Material();
		material->setDiffuse(osg::Material::FRONT, osg::Vec4(color[0] / (float)0xFF, color[1] / (float)0xFF, color[2] / (float)0xFF, 1.0));
		material->setAmbient(osg::Material::FRONT, osg::Vec4(color[0] / (float)0xFF, color[1] / (float)0xFF, color[2] / (float)0xFF, 1.0));
		sensor->ball_->getOrCreateStateSet()->setAttribute(material);
		sensor->ball_radius_ = ball_radius;
	}

	// line
	if (create_line)
	{
		sensor->line_vertex_data_ = new osg::Vec3Array;
		sensor->line_vertex_data_->push_back(osg::Vec3d(0, 0, 0));
		sensor->line_vertex_data_->push_back(osg::Vec3d(0, 0, 0));

		osg::ref_ptr<osg::Group> group = new osg::Group;
		sensor->line_ = new osg::Geometry();
		group->addChild(sensor->line_);
		//sensor->line_->setCullingActive(false);
		sensor->line_->setVertexArray(sensor->line_vertex_data_.get());
		sensor->line_->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, 2));

		osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth();
		lineWidth->setWidth(line_width);
		sensor->line_->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
		sensor->line_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

		osg::ref_ptr<osg::Vec4Array> color_ = new osg::Vec4Array;
		color_->push_back(osg::Vec4(color[0] / (float)0xFF, color[1] / (float)0xFF, color[2] / (float)0xFF, 1.0));
		sensor->line_->setColorArray(color_.get());
		sensor->line_->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		//sensor->line_->setDataVariance(osg::Object::DYNAMIC);
		sensors_->addChild(group);
	}




	return sensor;
}

void Viewer::UpdateRoadSensors(PointSensor *road_sensor, PointSensor *lane_sensor, roadmanager::Position *pos) 
{
	if (road_sensor == 0 || lane_sensor == 0)
	{
		return;
	}

	roadmanager::Position lane_pos(*pos);
	roadmanager::Position track_pos(*pos);

	// Points
	track_pos.SetTrackPos(pos->GetTrackId(), pos->GetS(), 0);	
	lane_pos.SetLanePos(pos->GetTrackId(), pos->GetLaneId(), pos->GetS(), 0);

	double road_target_pos[3] = { track_pos.GetX(), track_pos.GetY(),  track_pos.GetZ() };
	UpdateSensor(road_sensor, pos, road_target_pos);

	double lane_target_pos[3] = { lane_pos.GetX(), lane_pos.GetY(),  lane_pos.GetZ() };
	UpdateSensor(lane_sensor, pos, lane_target_pos);
}

void Viewer::UpdateSensor(PointSensor *sensor, roadmanager::Position *pivot_pos, double target_pos[3])
{
	double z_offset = 0.2;

	if (sensor == 0)
	{
		return;
	}

	// Line
	if (sensor->line_)
	{
		sensor->line_vertex_data_->clear();
		sensor->line_vertex_data_->push_back(osg::Vec3d(pivot_pos->GetX(), pivot_pos->GetY(), pivot_pos->GetZ() + MAX(sensor->ball_radius_ / 3.0, z_offset)));
		sensor->line_vertex_data_->push_back(osg::Vec3d(target_pos[0], target_pos[1], target_pos[2] + MAX(sensor->ball_radius_ / 3.0, z_offset)));
		sensor->line_->dirtyGLObjects();
		sensor->line_->dirtyBound();
		sensor->line_vertex_data_->dirty();
	}

	// Point/ball
	if (sensor->ball_)
	{
		sensor->ball_->setPosition(osg::Vec3(target_pos[0], target_pos[1], target_pos[2] + sensor->ball_radius_ / 3.0));
	}
}

int Viewer::AddEnvironment(const char* filename)
{
	// remove current model, if any
	if (environment_ != 0)
	{
		printf("Removing current env\n");
		envTx_->removeChild(environment_);
	}

	// load and apply new model
	if (strcmp(FileNameOf(filename).c_str(), ""))
	{
		environment_ = osgDB::readNodeFile(filename);
		if (environment_ == 0)
		{
			LOG("Failed to read environment model %s! If file is missing, check SharePoint/SharedDocuments/models", filename);
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

void Viewer::ShowInfoText(bool show)
{
	showInfoText = show;
}

void Viewer::SetInfoText(const char* text)
{
	if (showInfoText)
	{
		infoText->setText(text);
	}
	else
	{
		infoText->setText("");
	}
}

void Viewer::SetInfoTextProjection(int width, int height)
{
	infoTextCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, width, 0, height));
}

bool ViewerEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
	switch (ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::RESIZE):
		viewer_->SetInfoTextProjection(ea.getWindowWidth(), ea.getWindowHeight());
		break;
	}

	switch (ea.getKey())
	{
	case(osgGA::GUIEventAdapter::KEY_C):
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
	case(osgGA::GUIEventAdapter::KEY_O):
	{
		static bool visible = true;

		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			visible = !visible;

			if (viewer_->odrLines_)
			{
				viewer_->odrLines_->setNodeMask(visible ? 0xffffffff : 0x0);
			}

			viewer_->sensors_->setNodeMask(visible ? 0xffffffff : 0x0);
			viewer_->trails_->setNodeMask(visible ? 0xffffffff : 0x0);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_M):
	{
		static bool visible = true;

		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			visible = !visible;
			viewer_->envTx_->setNodeMask(visible ? 0xffffffff : 0x0);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_D):
	case(osgGA::GUIEventAdapter::KEY_Right):
	{
		viewer_->setKeyRight(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_A):
	case(osgGA::GUIEventAdapter::KEY_Left):
	{
		viewer_->setKeyLeft(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_W):
	case(osgGA::GUIEventAdapter::KEY_Up):
	{
		viewer_->setKeyUp(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_X):
	case(osgGA::GUIEventAdapter::KEY_Down):
	{
		viewer_->setKeyDown(ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN);
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Tab):
	{
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
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_T):
	{
		if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->showInfoText = !viewer_->showInfoText;
			viewer_->infoTextCamera->setNodeMask(viewer_->showInfoText ? 0xffffffff : 0x0);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Escape):
	{
		viewer_->osgViewer_->setDone(true);
		viewer_->SetQuitRequest(true);
	}
	break;
	}

	return false;
}
