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
#include <osg/CullFace>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/Registry>
#include <osgShadow/StandardShadowMap>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgUtil/SmoothingVisitor>
#include "CommonMini.hpp"
//#include "ScenarioEngine.hpp"

#define SHADOW_SCALE 1.20
#define SHADOW_MODEL_FILEPATH "shadow_face.osgb"  
#define ARROW_MODEL_FILEPATH "arrow.osgb"
#define LOD_DIST 3000
#define LOD_SCALE_DEFAULT 1.0
#define DEFAULT_AA_MULTISAMPLES 4

double color_green[3] = { 0.25, 0.6, 0.3 };
double color_gray[3] = { 0.7, 0.7, 0.7 };
double color_dark_gray[3] = { 0.5, 0.5, 0.5 };
double color_red[3] = { 0.73, 0.26, 0.26 };
double color_blue[3] = { 0.25, 0.38, 0.7 };
double color_yellow[3] = { 0.75, 0.7, 0.4 };
double color_white[3] = { 0.80, 0.80, 0.79 };

//USE_OSGPLUGIN(fbx)
//USE_OSGPLUGIN(obj)
USE_OSGPLUGIN(osg2)
USE_OSGPLUGIN(jpeg)
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
USE_SERIALIZER_WRAPPER_LIBRARY(osgSim)
USE_COMPRESSOR_WRAPPER(ZLibCompressor)
USE_GRAPHICSWINDOW()

using namespace viewer;

static osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene;

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

Line::Line(double x0, double y0, double z0, double x1, double y1, double z1, double r, double g, double b)
{
	line_vertex_data_ = new osg::Vec3Array;
	line_vertex_data_->push_back(osg::Vec3d(x0, y0, z0));
	line_vertex_data_->push_back(osg::Vec3d(x1, y1, z1));

	osg::ref_ptr<osg::Group> group = new osg::Group;
	line_ = new osg::Geometry();

	line_->setVertexArray(line_vertex_data_.get());
	line_->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, 2));

	line_->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(2), osg::StateAttribute::ON);
	line_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	osg::ref_ptr<osg::Vec4Array> color_ = new osg::Vec4Array;
	color_->push_back(osg::Vec4(r, g, b, 1.0));
	line_->setColorArray(color_.get());
	line_->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
}

void Line::SetPoints(double x0, double y0, double z0, double x1, double y1, double z1)
{
	line_vertex_data_->clear();
	line_vertex_data_->push_back(osg::Vec3d(x0, y0, z0));
	line_vertex_data_->push_back(osg::Vec3d(x1, y1, z1));
	line_->dirtyGLObjects();
	line_->dirtyBound();
	line_vertex_data_->dirty();
}

SensorViewFrustum::SensorViewFrustum(ObjectSensor *sensor, osg::Group *parent)
{
	sensor_ = sensor;
	txNode_ = new osg::PositionAttitudeTransform;
	txNode_->setNodeMask(SENSOR_NODE_MASK);
	parent->addChild(txNode_);

	// Create geometry
	int numSegments = 16 * sensor_->fovH_ / M_PI;
	double angleDelta = sensor_->fovH_ / numSegments;
	double angle = -sensor_->fovH_ / 2.0;
	double fovV_rate = 0.2;

	line_group_ = new osg::Group;
	for (size_t i = 0; i < sensor_->maxObj_; i++)
	{
		Line *line = new Line(0, 0, 0, 1, 0, 0, 0.8, 0.8, 0.8);
		line_group_->addChild(line->line_);
		lines_.push_back(line);
	}

	txNode_->addChild(line_group_);

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4 * (numSegments+1));
	osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_QUADS, 2 * 4 + 4 * 4 * numSegments);

	osg::ref_ptr<osg::DrawElementsUInt> indicesC0 = new osg::DrawElementsUInt(GL_LINE_STRIP, numSegments+1);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC1 = new osg::DrawElementsUInt(GL_LINE_STRIP, numSegments+1);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC2 = new osg::DrawElementsUInt(GL_LINE_STRIP, numSegments+1);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC3 = new osg::DrawElementsUInt(GL_LINE_STRIP, numSegments+1);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC4 = new osg::DrawElementsUInt(GL_LINE_LOOP, 4);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC5 = new osg::DrawElementsUInt(GL_LINE_LOOP, 4);

	size_t i;
	unsigned int idx = 0;
	unsigned int idx2 = 0;
	unsigned int idxC = 0;

	for (i = 0; i < numSegments+1; ++i, angle += angleDelta)
	{
		float x = cosf(angle);
		float y = sinf(angle);

		(*vertices)[i * 4 + 0].set(sensor_->near_ * x, sensor_->near_ * y, -sensor_->near_ * fovV_rate);
		(*vertices)[i * 4 + 3].set(sensor_->far_ * x, sensor_->far_ * y, -sensor_->far_ * fovV_rate);
		(*vertices)[i * 4 + 2].set(sensor_->far_ * x, sensor_->far_ * y, sensor_->far_ * fovV_rate);
		(*vertices)[i * 4 + 1].set(sensor_->near_ * x, sensor_->near_ * y, sensor_->near_ * fovV_rate);

		if (i > 0)
		{
			// Bottom face
			(*indices)[idx++] = (i - 1) * 4 + 0;
			(*indices)[idx++] = (i - 1) * 4 + 1;
			(*indices)[idx++] = (i - 0) * 4 + 1;
			(*indices)[idx++] = (i - 0) * 4 + 0;

			// Top face
			(*indices)[idx++] = (i - 1) * 4 + 3;
			(*indices)[idx++] = (i - 0) * 4 + 3;
			(*indices)[idx++] = (i - 0) * 4 + 2;
			(*indices)[idx++] = (i - 1) * 4 + 2;

			// Side facing host entity
			(*indices)[idx++] = (i - 1) * 4 + 0;
			(*indices)[idx++] = (i - 0) * 4 + 0;
			(*indices)[idx++] = (i - 0) * 4 + 3;
			(*indices)[idx++] = (i - 1) * 4 + 3;

			// Side facing away from host entity
			(*indices)[idx++] = (i - 1) * 4 + 1;
			(*indices)[idx++] = (i - 1) * 4 + 2;
			(*indices)[idx++] = (i - 0) * 4 + 2;
			(*indices)[idx++] = (i - 0) * 4 + 1;
		}
		// Countour
		(*indicesC0)[idxC] = i * 4 + 0;
		(*indicesC1)[idxC] = i * 4 + 1;
		(*indicesC2)[idxC] = i * 4 + 2;
		(*indicesC3)[idxC++] = i * 4 + 3;
	}

	(*indicesC4)[0] = (*indices)[idx++] = 0;
	(*indicesC4)[1] = (*indices)[idx++] = 3;
	(*indicesC4)[2] = (*indices)[idx++] = 2;
	(*indicesC4)[3] = (*indices)[idx++] = 1;
			 	   
	(*indicesC5)[0] = (*indices)[idx++] = (i - 1) * 4 + 0;
	(*indicesC5)[1] = (*indices)[idx++] = (i - 1) * 4 + 1;
	(*indicesC5)[2] = (*indices)[idx++] = (i - 1) * 4 + 2;
	(*indicesC5)[3] = (*indices)[idx++] = (i - 1) * 4 + 3;

	const osg::Vec4 normalColor(0.8f, 0.7f, 0.6f, 1.0f);
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(1);
	(*colors)[0] = normalColor;

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	osg::ref_ptr<osg::Geometry> geom2 = new osg::Geometry;
	geom->setDataVariance(osg::Object::STATIC);
	geom->setUseDisplayList(true);
	geom->setUseVertexBufferObjects(true);
	geom->setVertexArray(vertices.get());

	geom2->setUseDisplayList(true);
	geom2->setUseVertexBufferObjects(true);
	geom2->setVertexArray(vertices.get());

	geom->addPrimitiveSet(indices.get());
	geom2->addPrimitiveSet(indicesC0.get());
	geom2->addPrimitiveSet(indicesC1.get());
	geom2->addPrimitiveSet(indicesC2.get());
	geom2->addPrimitiveSet(indicesC3.get());
	geom2->addPrimitiveSet(indicesC4.get());
	geom2->addPrimitiveSet(indicesC5.get());
	geom2->setColorArray(colors.get());
	geom2->setColorBinding(osg::Geometry::BIND_OVERALL);

	osgUtil::SmoothingVisitor::smooth(*geom, 0.5);
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.release());
	osg::ref_ptr<osg::Material> material = new osg::Material;
	material->setDiffuse(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 0.2));
	material->setAmbient(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 0.2));

	osg::ref_ptr<osg::StateSet> stateset = geode->getOrCreateStateSet(); // Get the StateSet of the group
	stateset->setAttribute(material.get()); // Set Material 
	
	stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
	stateset->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
	stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::ref_ptr<osg::CullFace> cull = new osg::CullFace();
	cull->setMode(osg::CullFace::BACK);
	stateset->setAttributeAndModes(cull, osg::StateAttribute::ON);

	// Draw only wireframe to 
	osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
	polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	stateset->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);

	osg::ref_ptr<osg::Geode> geode2 = new osg::Geode;
	geom2->getOrCreateStateSet()->setAttribute(new osg::LineWidth(1.0f));
	geom2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geode2->addDrawable(geom2.release());


	txNode_->addChild(geode);
	txNode_->addChild(geode2);
	txNode_->setPosition(osg::Vec3(sensor_->pos_.x, sensor_->pos_.y, sensor_->pos_.z));
	txNode_->setAttitude(osg::Quat(sensor_->pos_.h, osg::Vec3(0, 0, 1)));
}

SensorViewFrustum::~SensorViewFrustum()
{
	for (size_t i = 0; i < lines_.size(); i++)
	{
		delete lines_[i];
	}
	lines_.clear();
}

void SensorViewFrustum::Update()
{
	// Visualize hits by a "line of sight"
	for (size_t i = 0; i < sensor_->nObj_; i++)
	{
		lines_[i]->SetPoints(0, 0, 0, sensor_->hitList_[i].x_, sensor_->hitList_[i].y_, sensor_->hitList_[i].z_);
	}

	// Reset additional lines possibly previously in use
	for (size_t i = sensor_->nObj_; i < sensor_->maxObj_; i++)
	{
		lines_[i]->SetPoints(0, 0, 0, 0, 0, 0);
	}
}

void VisibilityCallback::operator()(osg::Node* sa, osg::NodeVisitor* nv)
{
	if (object_->CheckDirtyBits(scenarioengine::Object::DirtyBit::VISIBILITY))
	{
		if (object_->visibilityMask_ & scenarioengine::Object::Visibility::GRAPHICS)
		{
			car_->txNode_->getChild(0)->setNodeMask(0xffffffff);
			if (object_->visibilityMask_ & scenarioengine::Object::Visibility::SENSORS)
			{
				car_->SetTransparency(0.0);
			}
			else
			{
				car_->SetTransparency(0.6);
			}
		}
		else
		{
			// Must set 0-mask on child node, otherwise it will not be traversed...
			car_->txNode_->getChild(0)->setNodeMask(0x0);
		}
	}
	object_->ClearDirtyBits(scenarioengine::Object::DirtyBit::VISIBILITY);

}

void AlphaFadingCallback::operator()(osg::StateAttribute* sa, osg::NodeVisitor* nv)
{
	osg::Material* material = static_cast<osg::Material*>(sa);
	if (material)
	{
		double age = viewer_->elapsedTime() - born_time_stamp_;
		double dt = viewer_->elapsedTime() - time_stamp_;
		time_stamp_ = viewer_->elapsedTime();
		if (age > TRAIL_DOT_LIFE_SPAN)
		{
			_motion->update(dt);  
		}
		color_[3] = 1 - _motion->getValue();
		material->setDiffuse(osg::Material::FRONT_AND_BACK, color_);
		material->setAmbient(osg::Material::FRONT_AND_BACK, color_);
	}
}

TrailDot::TrailDot(float time, double x, double y, double z, double heading, 
	osgViewer::Viewer *viewer, osg::Group *parent, osg::ref_ptr<osg::Node> dot_node, osg::Vec4 trail_color)
{
	double dot_radius = 0.8;
	osg::ref_ptr<osg::Node> new_node;

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
	fade_callback_ = new AlphaFadingCallback(viewer, trail_color);
	material_->setUpdateCallback(fade_callback_);

	new_node->getOrCreateStateSet()->setAttributeAndModes(material_.get());
	osg::ref_ptr<osg::StateSet> stateset = new_node->getOrCreateStateSet(); // Get the StateSet of the group
	stateset->setAttribute(material_.get()); // Set Material 
	stateset->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
	stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	parent->addChild(dot_);
}

static osg::ref_ptr<osg::Node> CreateDotGeometry()
{
	double height = 0.17;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(6);
	osg::ref_ptr<osg::DrawElementsUInt> indices1 = new osg::DrawElementsUInt(GL_QUADS, 3 * 4);
	osg::ref_ptr<osg::DrawElementsUInt> indices2 = new osg::DrawElementsUInt(GL_TRIANGLES, 3);
	int idx = 0;

	(*vertices)[idx++].set(0.0, -0.5, 0.0);
	(*vertices)[idx++].set(0.87, 0.0, 0.0);
	(*vertices)[idx++].set(0.0, 0.5, 0.0);
	(*vertices)[idx++].set(0.0, -0.5, height);
	(*vertices)[idx++].set(0.87, 0.0, height);
	(*vertices)[idx++].set(0.0, 0.5, height);

	// sides
	idx = 0;
	(*indices1)[idx++] = 0;
	(*indices1)[idx++] = 1;
	(*indices1)[idx++] = 4;
	(*indices1)[idx++] = 3;

	(*indices1)[idx++] = 2;
	(*indices1)[idx++] = 5;
	(*indices1)[idx++] = 4;
	(*indices1)[idx++] = 1;

	(*indices1)[idx++] = 0;
	(*indices1)[idx++] = 3;
	(*indices1)[idx++] = 5;
	(*indices1)[idx++] = 2;

	// Top face
	idx = 0;
	(*indices2)[idx++] = 3;
	(*indices2)[idx++] = 4;
	(*indices2)[idx++] = 5;

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setDataVariance(osg::Object::DYNAMIC);
	geom->setUseDisplayList(false);
	geom->setUseVertexBufferObjects(true);
	geom->setVertexArray(vertices.get());
	geom->addPrimitiveSet(indices1.get());
	geom->addPrimitiveSet(indices2.get());
	osgUtil::SmoothingVisitor::smooth(*geom, 0.5);
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.release());

	return geode;
}

void TrailDot::Reset(float time, double x, double y, double z, double heading)
{
	dot_->setPosition(osg::Vec3(x, y, z));

	dot_->setAttitude(osg::Quat(
		0, osg::Vec3(1, 0, 0),       // Roll
		0, osg::Vec3(0, 1, 0),       // Pitch
		heading, osg::Vec3(0, 0, 1)) // Heading
	); 

	fade_callback_->Reset();
}

void Trail::AddDot(float time, double x, double y, double z, double heading)
{
	if (n_dots_ < TRAIL_MAX_DOTS)
	{
		dot_[current_] = new TrailDot(time, x, y, z, heading, viewer_, parent_, dot_node_, color_);
		n_dots_++;
	}
	else
	{
		dot_[current_]->Reset(time, x, y, z, heading);
	}

	if (++current_ >= TRAIL_MAX_DOTS)
	{
		current_ = 0;
	}
}

Trail::~Trail()
{
	for (size_t i = 0; i < n_dots_; i++)
	{
		delete (dot_[i]);
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

	return tx_node;
}

CarModel::CarModel(osgViewer::Viewer *viewer, osg::ref_ptr<osg::LOD> lod, osg::ref_ptr<osg::Group> parent, osg::ref_ptr<osg::Group> trail_parent, osg::ref_ptr<osg::Node> dot_node, osg::Vec3 trail_color,std::string name)
{
	if (!lod)
	{
		return;
	}
	name_ = name;
	node_ = lod;
	speed_sensor_ = 0;
	steering_sensor_ = 0;
	road_sensor_ = 0;
	lane_sensor_ = 0;
	trail_sensor_ = 0;
	viewer_ = viewer;
	state_set_ = 0;
	blend_color_ = 0;

	wheel_angle_ = 0;
	wheel_rot_ = 0;

	// Locate wheel nodes if available
	osg::ref_ptr<osg::Node> car_node = lod->getChild(0);
	osg::ref_ptr<osg::Group> retval[4];
	retval[0] = AddWheel(car_node, "wheel_fl");
	retval[1] = AddWheel(car_node, "wheel_fr");
	retval[2] = AddWheel(car_node, "wheel_rr");
	retval[3] = AddWheel(car_node, "wheel_rl");
	if (!(retval[0] || retval[1] || retval[2] || retval[3]))
	{
		LOG("No wheel nodes in model %s. No problem, wheels will just not appear to roll or steer", car_node->getName().c_str());
	}
	else
	{
		if (!retval[0])
		{
			LOG("Missing wheel node %s in vehicle model %s - ignoring", "wheel_fl", car_node->getName().c_str());
		}
		if (!retval[1])
		{
			LOG("Missing wheel node %s in vehicle model %s - ignoring", "wheel_fr", car_node->getName().c_str());
		}
		if (!retval[2])
		{
			LOG("Missing wheel node %s in vehicle model %s - ignoring", "wheel_rr", car_node->getName().c_str());
		}
		if (!retval[3])
		{
			LOG("Missing wheel node %s in vehicle model %s - ignoring", "wheel_rl", car_node->getName().c_str());
		}
	}

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
	trail_ = new Trail(trail_parent, viewer, dot_node, trail_color);
}

CarModel::~CarModel()
{
	wheel_.clear();
	delete trail_;
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

void CarModel::SetTransparency(double factor)
{
	if (factor < 0 || factor > 1)
	{
		LOG("Clamping transparency factor %.2f to [0:1]", factor);
		factor = CLAMP(factor, 0, 1); 
	}
	blend_color_->setConstantColor(osg::Vec4(1, 1, 1, 1-factor));
}

Viewer::Viewer(roadmanager::OpenDrive* odrManager, const char* modelFilename, const char* scenarioFilename, const char* exe_path, osg::ArgumentParser arguments, SE_Options* opt)
{
	odrManager_ = odrManager;
	bool clear_color;
	std::string arg_str;

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
	shadow_node_ = NULL;
	environment_ = NULL;

	int aa_mode = DEFAULT_AA_MULTISAMPLES;  
	if (opt && (arg_str = opt->GetOptionArg("aa_mode")) != "")
	{
		aa_mode = atoi(arg_str.c_str());
	}
	LOG("Anti-Aliasing number of subsamples: %d", aa_mode);
	osg::DisplaySettings::instance()->setNumMultiSamples(aa_mode);

	arguments.getApplicationUsage()->addCommandLineOption("--lodScale <number>", "LOD Scale");
	arguments.read("--lodScale", lodScale_);

	clear_color = (arguments.find("--clear-color") != -1);

	// Store arguments in case we need to create a second viewer if the first fails
	int argc = arguments.argc();
	char **argv = (char**)malloc(argc * sizeof(char*));
	for (int i = 0; i < argc; i++)
	{
		argv[i] = (char*)malloc(strlen(arguments.argv()[i]) + 1);  // +1 to include null termination
		strncpy(argv[i], arguments.argv()[i], strlen(arguments.argv()[i]) + 1);
	}

	osgViewer_ = new osgViewer::Viewer(arguments);
	
	// Check if the viewer has been created correctly - window created is a indication
	osgViewer::ViewerBase::Windows wins;
	osgViewer_->getWindows(wins);
	if (wins.size() == 0)
	{
		// Viewer failed to create window. Probably Anti Aliasing is not supported on executing platform.
		// Make another attempt without AA
		LOG("Viewer failure. Probably requested level of Anti Aliasing (%d multisamples) is not supported - try a lower number. Making another attempt without Anti-Alias.", aa_mode);
		osg::DisplaySettings::instance()->setNumMultiSamples(0);
		delete osgViewer_;
		osg::ArgumentParser args2(&argc, argv);
		osgViewer_ = new osgViewer::Viewer(args2);
	}
	else
	{
		for (int i = 0; i < argc; i++)
		{
			free(argv[i]);
		}
		free(argv);
	}

	// Create 3D geometry for trail dots
	dot_node_ = CreateDotGeometry();

	// set the scene to render
	rootnode_ = new osg::MatrixTransform;
	

#if 0
	// Setup shadows
	const int CastsShadowTraversalMask = 0x2;
	LOG("1");
	shadowedScene = new osgShadow::ShadowedScene;
	osgShadow::ShadowSettings* settings = shadowedScene->getShadowSettings();
	LOG("2");
	settings->setReceivesShadowTraversalMask(CastsShadowTraversalMask);
//	shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
	osg::ref_ptr<osgShadow::ShadowMap> st = new osgShadow::ShadowMap;
//	osg::ref_ptr<osgShadow::StandardShadowMap> st = new osgShadow::StandardShadowMap;
	int mapres = 1024;
	LOG("3");
	st->setTextureSize(osg::Vec2s(mapres, mapres));
	shadowedScene->setShadowTechnique(st.get());
	shadowedScene->addChild(rootnode_);
	LOG("4");
#endif

	envTx_ = new osg::PositionAttitudeTransform;
	rootnode_->addChild(envTx_);

	roadSensors_ = new osg::Group;
	rootnode_->addChild(roadSensors_);
	roadSensors_ = new osg::Group;
	rootnode_->addChild(roadSensors_);
	trails_ = new osg::Group;
	rootnode_->addChild(trails_);
	odrLines_ = new osg::Group;
	rootnode_->addChild(odrLines_);
	osiLines_ = new osg::Group;
	rootnode_->addChild(osiLines_);
	exe_path_ = exe_path;

	ShowTrail(false);  // hide trails per default
	ShowRoadFeatures(true);
	ShowOSIFeatures(false); // hide OSI features by default
	ShowObjectSensors(false); // hide sensor frustums by default
	ShowRoadFeatures(false); // hide road features by default

	// add environment
	if (modelFilename != 0 && (!FileExists(modelFilename) || (AddEnvironment(modelFilename) == -1)))
	{
		// Look relative scenario directory
		const char* filename = 0;
		if (scenarioFilename != 0)
		{
			filename = scenarioFilename;
		}
		else if (!odrManager->GetOpenDriveFilename().empty())
		{
			filename = odrManager->GetOpenDriveFilename().c_str();
		}

		if (filename != 0)
		{
			std::string modelFilename2 = CombineDirectoryPathAndFilepath(DirNameOf(filename), modelFilename).c_str();
			if (!FileExists(modelFilename2.c_str()) || (AddEnvironment(modelFilename2.c_str()) == -1))
			{
				LOG("Failed to read environment model %s! Also tried %s. If file is missing, check SharePoint/SharedDocuments/models",
					modelFilename, CombineDirectoryPathAndFilepath(getScenarioDir(), modelFilename).c_str());
			}
		}
		else
		{
			LOG("Failed to locate environment model based on xosc or xodr locations - continue without");
		}
	}
	if (environment_ == 0)
	{
		// No environment model, i.e. visual model of the road network, loaded - 
		// enforce visualization of OpenDRIVE features
		ShowRoadFeatures(true);
	}

	if (odrManager->GetNumOfRoads() > 0 && !CreateRoadLines(odrManager))
	{
		LOG("Viewer::Viewer Failed to create road lines!\n");
	}

	if (odrManager->GetNumOfRoads() > 0 && !CreateRoadMarkLines(odrManager))
	{
		LOG("Viewer::Viewer Failed to create road mark lines!\n");
	}

#if 0
	osgViewer_->setSceneData(shadowedScene);
#else
	osgViewer_->setSceneData(rootnode_);
#endif

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
	if (!clear_color)
	{
		// Default background color
		osgViewer_->getCamera()->setClearColor(osg::Vec4(0.5f, 0.75f, 1.0f, 0.0f));
	}

	// add the window size toggle handler
	osgViewer_->addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler
	osgViewer_->addEventHandler(new osgViewer::StatsHandler);

	// add the state manipulator
	osgViewer_->addEventHandler(new osgGA::StateSetManipulator(osgViewer_->getCamera()->getOrCreateStateSet()));

#if 0
	// add the thread model handler
	osgViewer_->addEventHandler(new osgViewer::ThreadingHandler);
#else
	// If we see problem with chrashes when manipulating graphic nodes or states, in spite of 
	// trying to use callback mechanisms, then locking to single thread might be a solution. 

	// Hard code single thread model. Can't get setDataVariance(DYNAMIC)
	// to work with some state changes. And callbacks for all possible
	// nodes would be too much overhead. Solve when needed.
	osgViewer_->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
#endif

	// add the help handler
	osgViewer_->addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

	// add the record camera path handler
	osgViewer_->addEventHandler(new osgViewer::RecordCameraPathHandler);

	// add the LOD Scale handler
	osgViewer_->addEventHandler(new osgViewer::LODScaleHandler);

	// add the screen capture handler
	osgViewer_->addEventHandler(new osgViewer::ScreenCaptureHandler);

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
	float layoutCharacterSize = 12.0f;

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
	delete osgViewer_;
	osgViewer_ = 0;
}

void Viewer::SetCameraMode(int mode)
{
	if (mode < 0 || mode >= osgGA::RubberbandManipulator::RB_NUM_MODES)
	{
		return;
	}

	camMode_ = mode;
	rubberbandManipulator_->setMode(camMode_);
}

CarModel* Viewer::AddCar(std::string modelFilepath, osg::Vec3 trail_color, bool road_sensor, std::string name)
{
	// Load 3D model
	std::string path = modelFilepath;
	osg::ref_ptr<osg::LOD> lod;

	std::vector<std::string> file_name_candidates;
	file_name_candidates.push_back(path.c_str());
	file_name_candidates.push_back(CombineDirectoryPathAndFilepath(getScenarioDir(), path).c_str());
	file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(roadmanager::Position::GetOpenDrive()->GetOpenDriveFilename()), path).c_str());
	file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(exe_path_) + "/../resources/models", path).c_str());
	size_t i;
	for (i = 0; i < file_name_candidates.size(); i++)
	{
		if (FileExists(file_name_candidates[i].c_str()))
		{
			if (lod = LoadCarModel(file_name_candidates[i].c_str()))
			{
				break;
			}
		}
	}

	if (lod == 0)
	{
		// Failed to load model for some reason - maybe no filename. Create a dummy stand-in geometry
		if (path == "")
		{
			LOG("No filename specified for model! - creating a dummy model");
		}
		else
		{
			LOG("Failed to load car model %s. %s", path.c_str(), file_name_candidates.size() > 1 ? "Also tried the following paths:" : "");
			for (size_t i = 1; i < file_name_candidates.size(); i++)
			{
				LOG("    %s", file_name_candidates[i].c_str());
			}
			LOG("Creating a dummy model instead");
		}

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(new osg::ShapeDrawable(new osg::Box()));
		lod = new osg::LOD;
		lod->setRange(0, 0, LOD_DIST);
		osg::ref_ptr<osg::PositionAttitudeTransform> tx = new osg::PositionAttitudeTransform;
		lod->addChild(tx);

		// Set dimensions of the vehicle "box"
		tx->setScale(osg::Vec3(4.0, 2.0, 1.2));
		tx->setPosition(osg::Vec3(1.5, 0.0, 0.6));
		tx->addChild(geode);

		osg::Material* material = new osg::Material();

		// Set color of vehicle based on its index
		double* color;
		double b = 1.5;  // brighness
		int index = cars_.size() % 4;

		if (index == 0) color = color_white;
		else if (index == 1) color = color_red;
		else if (index == 2) color = color_blue;
		else color = color_yellow;

		material->setDiffuse(osg::Material::FRONT, osg::Vec4(b * color[0], b * color[1], b * color[2], 1.0));
		material->setAmbient(osg::Material::FRONT, osg::Vec4(b * color[0], b * color[1], b * color[2], 1.0));
		tx->getOrCreateStateSet()->setAttribute(material);
	}

	CarModel* car = new CarModel(osgViewer_, lod, rootnode_, trails_, dot_node_, trail_color, name);
	car->state_set_ = lod->getOrCreateStateSet(); //Creating material
	car->blend_color_ = new osg::BlendColor(osg::Vec4(1, 1, 1, 1));
	car->state_set_->setAttributeAndModes(car->blend_color_);
	car->blend_color_->setDataVariance(osg::Object::DYNAMIC);

	osg::BlendFunc* bf = new osg::BlendFunc(osg::BlendFunc::CONSTANT_ALPHA, osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
	car->state_set_->setAttributeAndModes(bf);
	car->state_set_->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	car->state_set_->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	cars_.push_back(car);

	// Focus on first added car
	if (cars_.size() == 1)
	{
		currentCarInFocus_ = 0;
		rubberbandManipulator_->setTrackNode(cars_.back()->txNode_, 
			rubberbandManipulator_->getMode() == osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_TOP ? false : true);
		nodeTrackerManipulator_->setTrackNode(cars_.back()->node_);
	}
	
	if (road_sensor)
	{
		CreateRoadSensors(cars_.back());
	}

	return cars_.back();
}

void Viewer::RemoveCar(std::string name)
{
	for (size_t i = 0; i < cars_.size(); i++) 
	{
		if (cars_[i]->name_ == name) 
		{
			cars_.erase(cars_.begin() + i);
		}
	}
}

osg::ref_ptr<osg::LOD> Viewer::LoadCarModel(const char *filename)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> shadow_tx = 0;
	osg::ref_ptr<osg::Node> node;
	osg::ref_ptr<osg::LOD> lod = 0;

	node = osgDB::readNodeFile(filename);
	if (!node)
	{
		return 0;
	}

	osg::ComputeBoundsVisitor cbv;
	node->accept(cbv);
	osg::BoundingBox boundingBox = cbv.getBoundingBox();

	double xc, yc, dx, dy;
	dx = boundingBox._max.x() - boundingBox._min.x();
	dy = boundingBox._max.y() - boundingBox._min.y();
	xc = (boundingBox._max.x() + boundingBox._min.x()) / 2;
	yc = (boundingBox._max.y() + boundingBox._min.y()) / 2;

	if (!shadow_node_)
	{
		LoadShadowfile(filename);
	}
	
	osg::ref_ptr<osg::Group> group = new osg::Group;

	group->addChild(node);

	if (shadow_node_)
	{
		shadow_tx = new osg::PositionAttitudeTransform;
		shadow_tx->setPosition(osg::Vec3d(xc, yc, 0.0));
		shadow_tx->setScale(osg::Vec3d(SHADOW_SCALE*(dx / 2), SHADOW_SCALE*(dy / 2), 1.0));
		shadow_tx->addChild(shadow_node_);

		group->addChild(shadow_tx);
	}

	group->setName(FileNameOf(filename));

	lod = new osg::LOD();
	lod->addChild(group);
	lod->setRange(0, 0, LOD_DIST);
	lod->setName(group->getName());

	return lod;
}

bool Viewer::CreateRoadMarkLines(roadmanager::OpenDrive* od)
{
	double z_offset = 0.10;
	osg::Vec3 point(0, 0, 0);

	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		roadmanager::Road *road = od->GetRoadByIdx(r);
		for (int i = 0; i < road->GetNumberOfLaneSections(); i++)
		{
			roadmanager::LaneSection *lane_section = road->GetLaneSectionByIdx(i);
			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				roadmanager::Lane *lane = lane_section->GetLaneByIdx(j);
				for (int k = 0; k < lane->GetNumberOfRoadMarks(); k++)
				{
					roadmanager::LaneRoadMark *lane_roadmark = lane->GetLaneRoadMarkByIdx(k);
					for (int m = 0; m < lane_roadmark->GetNumberOfRoadMarkTypes(); m++)
					{
						roadmanager::LaneRoadMarkType * lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(m);
						for (int n = 0; n < lane_roadmarktype->GetNumberOfRoadMarkTypeLines(); n++)
						{
							roadmanager::LaneRoadMarkTypeLine * lane_roadmarktypeline = lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(n);
							roadmanager::OSIPoints curr_osi_rm;
							curr_osi_rm = lane_roadmarktypeline->GetOSIPoints();

							if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN)
							{
								for (int q = 0; q < curr_osi_rm.GetPoints().size(); q+=2)
								{
									roadmanager::OSIPoints::OSIPointStruct osi_point1 = curr_osi_rm.GetPoint(q);
									roadmanager::OSIPoints::OSIPointStruct osi_point2 = curr_osi_rm.GetPoint(q+1);

									// osg references for road mark osi points
									osg::ref_ptr<osg::Geometry> osi_rm_geom = new osg::Geometry;
									osg::ref_ptr<osg::Vec3Array> osi_rm_points = new osg::Vec3Array;
									osg::ref_ptr<osg::Vec4Array> osi_rm_color = new osg::Vec4Array;
									osg::ref_ptr<osg::Point> osi_rm_point = new osg::Point();
									
									// osg references for drawing lines between each road mark osi points
									osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
									osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
									osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
									osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth();

									// start point of each road mark
									point.set(osi_point1.x, osi_point1.y, osi_point1.z + z_offset);
									osi_rm_points->push_back(point);

									// end point of each road mark
									point.set(osi_point2.x, osi_point2.y, osi_point2.z + z_offset);
									osi_rm_points->push_back(point);

									osi_rm_color->push_back(osg::Vec4(color_white[0], color_white[1], color_white[2], 1.0));

									// Put points at the start and end of the roadmark
									osi_rm_point->setSize(6.0f);
									osi_rm_geom->setVertexArray(osi_rm_points.get());
									osi_rm_geom->setColorArray(osi_rm_color.get());
									osi_rm_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
									osi_rm_geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, osi_rm_points->size()));
									osi_rm_geom->getOrCreateStateSet()->setAttributeAndModes(osi_rm_point, osg::StateAttribute::ON);
									osi_rm_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

									osiLines_->addChild(osi_rm_geom);

									// Draw lines from the start of the roadmark to the end of the roadmark
									lineWidth->setWidth(1.5f);
									geom->setVertexArray(osi_rm_points.get());
									geom->setColorArray(osi_rm_color.get());
									geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
									geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, osi_rm_points->size()));
									geom->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
									geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

									osiLines_->addChild(geom);
								}
							}
							else if(lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID)
							{
								// osg references for road mark osi points
								osg::ref_ptr<osg::Geometry> osi_rm_geom = new osg::Geometry;
								osg::ref_ptr<osg::Vec3Array> osi_rm_points = new osg::Vec3Array;
								osg::ref_ptr<osg::Vec4Array> osi_rm_color = new osg::Vec4Array;
								osg::ref_ptr<osg::Point> osi_rm_point = new osg::Point();
									
								// osg references for drawing lines between each road mark osi points
								osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
								osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
								osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
								osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth();

								// Creating points for the given roadmark
								for (int m = 0; m < curr_osi_rm.GetPoints().size(); m++)
								{
									point.set(curr_osi_rm.GetPoint(m).x, curr_osi_rm.GetPoint(m).y, curr_osi_rm.GetPoint(m).z + z_offset);
									osi_rm_points->push_back(point);
									osi_rm_color->push_back(osg::Vec4(color_white[0], color_white[1], color_white[2], 1.0));
								}

								// Put points on selected locations
								osi_rm_point->setSize(6.0f);
								osi_rm_geom->setVertexArray(osi_rm_points.get());
								osi_rm_geom->setColorArray(osi_rm_color.get());
								osi_rm_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
								osi_rm_geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, osi_rm_points->size()));
								osi_rm_geom->getOrCreateStateSet()->setAttributeAndModes(osi_rm_point, osg::StateAttribute::ON);
								osi_rm_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
									
								osiLines_->addChild(osi_rm_geom);

								// Draw lines between each selected points
								lineWidth->setWidth(1.5f);
								color->push_back(osg::Vec4(color_white[0], color_white[1], color_white[2], 1.0));

								geom->setVertexArray(osi_rm_points.get());
								geom->setColorArray(color.get());
								geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
								geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, osi_rm_points->size()));
								geom->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
								geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

								osiLines_->addChild(geom);

							}
						}
					}
				}
			}
		}
	}

	return true;
}

bool Viewer::CreateRoadLines(roadmanager::OpenDrive* od)
{
	double z_offset = 0.10;
	roadmanager::Position* pos = new roadmanager::Position();
	osg::Vec3 point(0, 0, 0);

	roadmanager::OSIPoints* curr_osi;

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
				kp_color->push_back(osg::Vec4(color_yellow[0], color_yellow[1], color_yellow[2], 1.0));
			}
			else
			{
				kp_color->push_back(osg::Vec4(color_red[0], color_red[1], color_red[2], 1.0));
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
			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				roadmanager::Lane *lane = lane_section->GetLaneByIdx(j);

				// osg references for osi points on the lane center
				osg::ref_ptr<osg::Geometry> osi_geom = new osg::Geometry;
				osg::ref_ptr<osg::Vec3Array> osi_points = new osg::Vec3Array;
				osg::ref_ptr<osg::Vec4Array> osi_color = new osg::Vec4Array;
				osg::ref_ptr<osg::Point> osi_point = new osg::Point();
				
				// osg references for drawing lines on the lane center using osi points
				osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
				osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
				osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
				osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth();

				if (!lane->IsDriving() && lane->GetId() != 0)
				{
					continue;
				}

				curr_osi = lane->GetOSIPoints();

				for (int m = 0; m < curr_osi->GetPoints().size(); m++)
				{
					roadmanager::OSIPoints::OSIPointStruct osi_point = curr_osi->GetPoint(m);
					point.set(osi_point.x, osi_point.y, osi_point.z + z_offset);
					osi_points->push_back(point);
					osi_color->push_back(osg::Vec4(color_blue[0], color_blue[1], color_blue[2], 1.0));
				}
				osi_point->setSize(6.0f);
				osi_geom->setVertexArray(osi_points.get());
				osi_geom->setColorArray(osi_color.get());
				osi_geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
				osi_geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, osi_points->size()));
				osi_geom->getOrCreateStateSet()->setAttributeAndModes(osi_point, osg::StateAttribute::ON);
				osi_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
					
				osiLines_->addChild(osi_geom);

				if (lane->GetId() == 0)
				{
					lineWidth->setWidth(4.0f);
					color->push_back(osg::Vec4(color_red[0], color_red[1], color_red[2], 1.0));
				}
				else
				{
					lineWidth->setWidth(1.5f);
					color->push_back(osg::Vec4(color_blue[0], color_blue[1], color_blue[2], 1.0));
				}

				geom->setVertexArray(osi_points.get());
				geom->setColorArray(color.get());
				geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
				geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, osi_points->size()));
				geom->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
				geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

				odrLines_->addChild(geom);
			}
		}
	}

	delete pos;

	return true;
}


bool Viewer::CreateRoadSensors(CarModel *vehicle_model)
{
	vehicle_model->road_sensor_ = CreateSensor(color_yellow, true, false, 0.25, 2.5);
	vehicle_model->lane_sensor_ = CreateSensor(color_yellow, true, true, 0.25, 2.5);

	return true;
}

PointSensor* Viewer::CreateSensor(double color[], bool create_ball, bool create_line, double ball_radius, double line_width)
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
		roadSensors_->addChild(sensor->ball_);

		osg::ref_ptr<osg::Material> material = new osg::Material();
		material->setDiffuse(osg::Material::FRONT, osg::Vec4(color[0], color[1], color[2], 1.0));
		material->setAmbient(osg::Material::FRONT, osg::Vec4(color[0], color[1], color[2], 1.0));
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
		color_->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
		sensor->line_->setColorArray(color_.get());
		sensor->line_->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
		//sensor->line_->setDataVariance(osg::Object::DYNAMIC);
		roadSensors_->addChild(group);
	}

	return sensor;
}

void Viewer::UpdateRoadSensors(PointSensor *road_sensor, PointSensor *lane_sensor, roadmanager::Position *pos) 
{
	if (road_sensor == 0 || lane_sensor == 0)
	{
		return;
	}

	roadmanager::Position track_pos(*pos);
	track_pos.SetTrackPos(pos->GetTrackId(), pos->GetS(), 0);

	SensorSetPivotPos(road_sensor, pos->GetX(), pos->GetY(), pos->GetZ());
	SensorSetTargetPos(road_sensor, track_pos.GetX(), track_pos.GetY(), track_pos.GetZ());
	UpdateSensor(road_sensor);

	roadmanager::Position lane_pos(*pos);
	lane_pos.SetLanePos(pos->GetTrackId(), pos->GetLaneId(), pos->GetS(), 0);

	SensorSetPivotPos(lane_sensor, pos->GetX(), pos->GetY(), pos->GetZ());
	SensorSetTargetPos(lane_sensor, lane_pos.GetX(), lane_pos.GetY(), lane_pos.GetZ());
	UpdateSensor(lane_sensor);
}

void Viewer::SensorSetPivotPos(PointSensor *sensor, double x, double y, double z)
{
	double z_offset = 0.2;
	sensor->pivot_pos = osg::Vec3(x, y, z + MAX(sensor->ball_radius_ / 3.0, z_offset));
}

void Viewer::SensorSetTargetPos(PointSensor *sensor, double x, double y, double z)
{
	double z_offset = 0.2;
	sensor->target_pos = osg::Vec3(x, y, z + MAX(sensor->ball_radius_ / 3.0, z_offset));
}

void Viewer::UpdateSensor(PointSensor *sensor)
{
	if (sensor == 0)
	{
		return;
	}

	// Line
	if (sensor->line_)
	{
		sensor->line_vertex_data_->clear();
		sensor->line_vertex_data_->push_back(sensor->pivot_pos);
		sensor->line_vertex_data_->push_back(sensor->target_pos);
		sensor->line_->dirtyGLObjects();
		sensor->line_->dirtyBound();
		sensor->line_vertex_data_->dirty();
	}

	// Point/ball
	if (sensor->ball_)
	{
		sensor->ball_->setPosition(sensor->target_pos);
	}
}

int Viewer::LoadShadowfile(std::string vehicleModelFilename)
{
	// Load shadow geometry - assume it resides in the same resource folder as the vehicle model
	std::string shadowFilename = DirNameOf(vehicleModelFilename).append("/" + std::string(SHADOW_MODEL_FILEPATH));
	if (FileExists(shadowFilename.c_str()))
	{
		shadow_node_ = osgDB::readNodeFile(shadowFilename);
	}

	if (!shadow_node_)
	{
		LOG("Failed to locate shadow model %s based on vehicle model filename %s - continue without", 
			SHADOW_MODEL_FILEPATH, vehicleModelFilename.c_str());
		return -1;
	}

	return 0;
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
	// First, assume absolute path or relative current directory
	if (strcmp(FileNameOf(filename).c_str(), ""))
	{
		if ((environment_ = osgDB::readNodeFile(filename)) == 0)
		{
			return -1;
		}

		envTx_->addChild(environment_);
	} 
	else
	{
		LOG("AddEnvironment: No environment 3D model specified (%s) - go ahead without\n", filename);
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

void Viewer::ShowTrail(bool show)
{
	showTrail = show;
	trails_->setNodeMask(showTrail ? 0xffffffff : 0x0);
}

void Viewer::ShowRoadFeatures(bool show)
{
	showRoadFeatures = show;
	odrLines_->setNodeMask(showRoadFeatures ? 0xffffffff : 0x0);
	roadSensors_->setNodeMask(showRoadFeatures ? 0xffffffff : 0x0);
}

void Viewer::ShowOSIFeatures(bool show)
{
	showOSIFeatures = show;
	osiLines_->setNodeMask(showOSIFeatures ? 0xffffffff : 0x0);
}

void Viewer::ShowObjectSensors(bool show)
{
	showObjectSensors = show;

	if (show)
	{
		osgViewer_->getCamera()->setCullMask(osgViewer_->getCamera()->getCullMask() | SENSOR_NODE_MASK);
	}
	else
	{
		osgViewer_->getCamera()->setCullMask(osgViewer_->getCamera()->getCullMask() & ~SENSOR_NODE_MASK);
	}
}

void Viewer::SetInfoTextProjection(int width, int height)
{
	infoTextCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, width, 0, height));
}

void Viewer::SetVehicleInFocus(int idx)
{
	currentCarInFocus_ = idx;
	if (cars_.size() > idx)
	{
		rubberbandManipulator_->setTrackNode(cars_[currentCarInFocus_]->txNode_, false);
		nodeTrackerManipulator_->setTrackNode(cars_[currentCarInFocus_]->node_);
	}
}

void Viewer::SetWindowTitle(std::string title)
{
	// Decorate window border with application name
	osgViewer::ViewerBase::Windows wins;
	osgViewer_->getWindows(wins);
	if (wins.size() > 0)
	{
		wins[0]->setWindowName(title);
	}
}

void Viewer::SetWindowTitleFromArgs(std::vector<std::string> &args)
{
	std::string titleString;
	for (int i = 0; i < args.size(); i++)
	{
		std::string arg = args[i];
		if (i == 0)
		{
			arg = FileNameWithoutExtOf(arg);
		}
		else if (arg == "--osc" || arg == "--odr" || arg == "--model")
		{
			titleString += std::string(arg) + " ";
			i++;
			arg = FileNameOf(std::string(args[i]));
		}
		else if (arg == "--window")
		{
			i += 4;
			continue;
		}

		titleString += std::string(arg) + " ";
	}
	
	SetWindowTitle(titleString);
}

void Viewer::SetWindowTitleFromArgs(int argc, char* argv[])
{
	std::vector<std::string> args;

	for (int i = 0; i < argc; i++)
	{
		args.push_back(argv[i]);
	}

	SetWindowTitleFromArgs(args);
}

void Viewer::RegisterKeyEventCallback(KeyEventCallbackFunc func, void* data)
{
	KeyEventCallback cb;
	cb.func = func;
	cb.data = data;
	callback_.push_back(cb);
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
	case(osgGA::GUIEventAdapter::KEY_K):
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
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
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ShowRoadFeatures(!viewer_->showRoadFeatures);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_U):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ShowOSIFeatures(!viewer_->showOSIFeatures);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_P):
	{
		static bool visible = true;

		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			visible = !visible;
			viewer_->envTx_->setNodeMask(visible ? 0xffffffff : 0x0);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Right):
	{
		viewer_->setKeyRight(ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN);
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Left):
	{
		viewer_->setKeyLeft(ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN);
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Up):
	{
		viewer_->setKeyUp(ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN);
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Down):
	{
		viewer_->setKeyDown(ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN);
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Tab):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			int idx = viewer_->currentCarInFocus_ + ((ea.getModKeyMask() & osgGA::GUIEventAdapter::KEY_Shift_L) ? -1 : 1);

			if (idx >= (int)viewer_->cars_.size())
			{
				idx = 0;
			}
			else if (idx < 0)
			{
				idx = viewer_->cars_.size() - 1;
			}

			viewer_->SetVehicleInFocus(idx);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_I):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->showInfoText = !viewer_->showInfoText;
			viewer_->infoTextCamera->setNodeMask(viewer_->showInfoText ? 0xffffffff : 0x0);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_J):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->showTrail = !viewer_->showTrail;
			viewer_->trails_->setNodeMask(viewer_->showTrail ? 0xffffffff : 0x0);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_R):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ShowObjectSensors(!viewer_->showObjectSensors);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Escape):
	{
		viewer_->SetQuitRequest(true);
		viewer_->osgViewer_->setDone(true);
	}
	break;
	}

	// Send key event to registered callback subscribers
	if (ea.getKey() > 0)
	{
		for (size_t i = 0; i < viewer_->callback_.size(); i++)
		{
			KeyEvent ke = { ea.getKey(), ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN ? true : false };
			viewer_->callback_[i].func(&ke, viewer_->callback_[i].data);
		}
	}

	return false;
}
