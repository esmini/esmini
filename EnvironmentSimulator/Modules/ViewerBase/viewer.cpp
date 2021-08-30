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
#include <osgDB/WriteFile>
#include <osgShadow/StandardShadowMap>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/Tessellator> // to tessellate multiple contours
#include <osgUtil/Optimizer>   // to flatten transform nodes

#include "CommonMini.hpp"

#define SHADOW_SCALE 1.20
#define SHADOW_MODEL_FILEPATH "shadow_face.osgb"
#define ARROW_MODEL_FILEPATH "arrow.osgb"
#define LOD_DIST 3000
#define LOD_DIST_ROAD_FEATURES 500
#define LOD_SCALE_DEFAULT 1.0
#define DEFAULT_AA_MULTISAMPLES 4
#define OSI_LINE_WIDTH 2.0f
#define TRAIL_WIDTH 2
#define TRAIL_DOT_SIZE 10
#define TRAIL_DOT3D_SIZE 0.2
#define TRAILDOT3D 1
#define PERSP_FOV 30.0
#define ORTHO_FOV 1.0

double color_green[3] = { 0.25, 0.6, 0.3 };
double color_gray[3] = { 0.7, 0.7, 0.7 };
double color_dark_gray[3] = { 0.5, 0.5, 0.5 };
double color_red[3] = { 0.73, 0.26, 0.26 };
double color_black[3] = { 0.2, 0.2, 0.2 };
double color_blue[3] = { 0.25, 0.38, 0.7 };
double color_yellow[3] = { 0.75, 0.7, 0.4 };
double color_white[3] = { 0.90, 0.90, 0.85 };

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

osg::ref_ptr<osg::Geode> CreateDotGeometry(double size, osg::Vec4 color, int nrPoints)
{
	nrPoints = MAX(nrPoints, 3);

	osg::ref_ptr<osg::Vec4Array> color_outline = new osg::Vec4Array;
	color_outline->push_back(color);

	osg::ref_ptr<osg::Vec3Array> vertices_sides = new osg::Vec3Array((nrPoints + 1) * 2);  // one set at bottom and one at top
	osg::ref_ptr<osg::Vec3Array> vertices_top = new osg::Vec3Array(nrPoints);  // one set for roof

	// Set vertices
	double height = 0.5 * size;
	for (size_t i = 0; i < nrPoints; i++)
	{
		double a = (double)i * 2.0 * M_PI / nrPoints;
		double x = size * cos(a);
		double y = size * sin(a);
		(*vertices_sides)[i * 2 + 0].set(x, y, 0);
		(*vertices_sides)[i * 2 + 1].set(x, y, height);
		(*vertices_top)[i].set(x, y, height);
	}

	// Close geometry
	(*vertices_sides)[2 * (nrPoints + 1) - 2].set((*vertices_sides)[0]);
	(*vertices_sides)[2 * (nrPoints + 1) - 1].set((*vertices_sides)[1]);

	// Finally create and add geometry
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geom[] = { new osg::Geometry, new osg::Geometry };

	geom[0]->setVertexArray(vertices_sides.get());
	geom[0]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, 2 * (nrPoints+1)));

	// Add roof
	geom[1]->setVertexArray(vertices_top.get());
	geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, nrPoints));

	for (int i = 0; i < 2; i++)
	{
		osgUtil::SmoothingVisitor::smooth(*geom[i], 10);
		geom[i]->setColorArray(color_outline);
		geom[i]->setColorBinding(osg::Geometry::BIND_OVERALL);
		geom[i]->setDataVariance(osg::Object::STATIC);
		geom[i]->setUseDisplayList(true);
		geode->addDrawable(geom[i]);
	}

	return geode;
}

PolyLine::PolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize, bool dots3D) :
	dots3D_(dots3D), dots_array_(nullptr), dots_geom_(nullptr), dot3D_geode_(nullptr)
{
	color_ = new osg::Vec4Array;
	color_->push_back(color);

	if (points == 0)
	{
		pline_vertex_data_ = new osg::Vec3Array;
	}
	else
	{
		pline_vertex_data_ = points;
	}

	pline_geom_ = new osg::Geometry;
	pline_geom_->setVertexArray(pline_vertex_data_.get());
	pline_geom_->setColorArray(color_.get());
	pline_geom_->setColorBinding(osg::Geometry::BIND_OVERALL);
	pline_geom_->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(width), osg::StateAttribute::ON);
	pline_geom_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	pline_array_ = new osg::DrawArrays(GL_LINE_STRIP, 0, pline_vertex_data_->size());
	pline_geom_->addPrimitiveSet(pline_array_);

	if (dotsize > SMALL_NUMBER)
	{
		if (dots3D)
		{
#if 0
			dot3D_shape_ = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0, 0.0, 0.25 * dotsize), dotsize, 0.5 * dotsize));
			dot3D_shape_->setColor(color);
			dot3D_geode_ = new osg::Geode;
			dot3D_geode_->addDrawable(dot3D_shape_.get());
#else
			dot3D_geode_ = CreateDotGeometry(dotsize, color, 12);
#endif
			dots3D_group_ = new osg::Group;
			for (size_t i = 0; i < pline_vertex_data_->size(); i++)
			{
				Add3DDot((*pline_vertex_data_)[i]);
			}
		}
		else
		{
			dots_geom_ = new osg::Geometry;
			dots_geom_->setVertexArray(pline_vertex_data_.get());
			dots_geom_->setColorArray(color_.get());
			dots_geom_->setColorBinding(osg::Geometry::BIND_OVERALL);
			dots_geom_->getOrCreateStateSet()->setAttribute(new osg::Point(dotsize));
			dots_geom_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
			dots_array_ = new osg::DrawArrays(GL_POINTS, 0, pline_vertex_data_->size());
			dots_geom_->addPrimitiveSet(dots_array_);
		}
	}

	parent->addChild(pline_geom_);

	if (dots3D)
	{
		parent->addChild(dots3D_group_);
	}
	else
	{
		parent->addChild(dots_geom_);
	}
}

void PolyLine::Redraw()
{
	pline_geom_->dirtyGLObjects();
	pline_geom_->dirtyBound();

	if (!dots3D_ && dots_geom_ != nullptr)
	{
		dots_geom_->dirtyGLObjects();
		dots_geom_->dirtyBound();
	}

	pline_vertex_data_->dirty();
}

void PolyLine::Update()
{
	pline_array_->setCount(pline_vertex_data_->size());

	if (!dots3D_ && dots_array_ != nullptr)
	{
		dots_array_->setCount(pline_vertex_data_->size());
	}

	Redraw();
}

void PolyLine::SetNodeMaskLines(int nodemask)
{
	pline_geom_->setNodeMask(nodemask);
}

void PolyLine::SetNodeMaskDots(int nodemask)
{
	if (dot3D_geode_) dot3D_geode_->setNodeMask(nodemask);
	if (dots_geom_) dots_geom_->setNodeMask(nodemask);
}

void PolyLine::SetPoints(osg::ref_ptr<osg::Vec3Array> points)
{
	pline_vertex_data_->clear();
	pline_vertex_data_ = points;

	if (dots3D_)
	{
		for (size_t i = 0; i < pline_vertex_data_->size(); i++)
		{
			Add3DDot((*pline_vertex_data_)[i]);
		}
	}

	Update();
}

void PolyLine::AddPoint(osg::Vec3 point)
{
	pline_vertex_data_->push_back(point);

	if (dots3D_)
	{
		Add3DDot(point);
	}

	Update();
}

void PolyLine::Add3DDot(osg::Vec3 pos)
{
	osg::ref_ptr<osg::Geode> geode2 = dynamic_cast<osg::Geode*>(dot3D_geode_->clone(osg::CopyOp::SHALLOW_COPY));
	osg::ref_ptr<osg::MatrixTransform> tx = new osg::MatrixTransform;
	tx->setMatrix(osg::Matrix::translate(pline_vertex_data_->back()));
	tx->addChild(geode2);
	dots3D_group_->addChild(tx);
}

SensorViewFrustum::SensorViewFrustum(ObjectSensor *sensor, osg::Group *parent)
{
	sensor_ = sensor;
	txNode_ = new osg::PositionAttitudeTransform;
	txNode_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
	parent->addChild(txNode_);

	// Create geometry
	int numSegments = 16 * sensor_->fovH_ / M_PI;
	double angleDelta = sensor_->fovH_ / numSegments;
	double angle = -sensor_->fovH_ / 2.0;
	double fovV_rate = 0.2;

	line_group_ = new osg::Group;
	for (size_t i = 0; i < sensor_->maxObj_; i++)
	{
		osg::ref_ptr<osg::Vec3Array> varray = new osg::Vec3Array;
		varray->push_back(osg::Vec3(0, 0, 0));
		varray->push_back(osg::Vec3(1, 0, 0));
		PolyLine* pline = new PolyLine(line_group_, varray, osg::Vec4(0.8, 0.8, 0.8, 1.0), 2.0);
		plines_.push_back(pline);
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
	osgUtil::SmoothingVisitor::smooth(*geom2, 0.0);
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
	for (size_t i = 0; i < plines_.size(); i++)
	{
		delete plines_[i];
	}
	plines_.clear();
}

void SensorViewFrustum::Update()
{
	// Visualize hits by a "line of sight"
	for (size_t i = 0; i < sensor_->nObj_; i++)
	{
		(*plines_[i]->pline_vertex_data_)[1][0] = sensor_->hitList_[i].x_;
		(*plines_[i]->pline_vertex_data_)[1][1] = sensor_->hitList_[i].y_;
		(*plines_[i]->pline_vertex_data_)[1][2] = sensor_->hitList_[i].z_;

		plines_[i]->Redraw();
	}

	// Reset additional lines possibly previously in use
	for (size_t i = sensor_->nObj_; i < sensor_->maxObj_; i++)
	{
		(*plines_[i]->pline_vertex_data_)[1][0] = 0;
		(*plines_[i]->pline_vertex_data_)[1][1] = 0;
		(*plines_[i]->pline_vertex_data_)[1][2] = 0;

		plines_[i]->Redraw();
	}
}

void VisibilityCallback::operator()(osg::Node* sa, osg::NodeVisitor* nv)
{
	if (object_->CheckDirtyBits(scenarioengine::Object::DirtyBit::VISIBILITY))
	{
		if (object_->visibilityMask_ & scenarioengine::Object::Visibility::GRAPHICS)
		{
			entity_->txNode_->getChild(0)->setNodeMask(NodeMask::NODE_MASK_ENTITY_MODEL);
			if (object_->visibilityMask_ & scenarioengine::Object::Visibility::SENSORS)
			{
				entity_->SetTransparency(0.0);
			}
			else
			{
				entity_->SetTransparency(0.6);
			}
		}
		else
		{
			// Must set 0-mask on child node, otherwise it will not be traversed...
			entity_->txNode_->getChild(0)->setNodeMask(NodeMask::NODE_MASK_NONE);
		}
	}
	object_->ClearDirtyBits(scenarioengine::Object::DirtyBit::VISIBILITY);

}

Trajectory::Trajectory(osg::Group* parent, osgViewer::Viewer* viewer) :
	parent_(parent), viewer_(viewer), activeRMTrajectory_(0)
{
	pline_ = new PolyLine(parent_, new osg::Vec3Array, osg::Vec4(0.9, 0.7, 0.3, 1.0), 3.0);
}

void Trajectory::SetActiveRMTrajectory(roadmanager::RMTrajectory* RMTrajectory)
{
	// Trajectory has been activated on the entity, visualize it
	double z_offset = 0.15;

	vertices_.clear();
	pline_->pline_vertex_data_->clear();

	if (RMTrajectory->shape_->pline_.GetNumberOfVertices() < 1)
	{
		return;
	}

	for (int i = 0; i < RMTrajectory->shape_->pline_.GetNumberOfVertices(); i++)
	{
		roadmanager::TrajVertex& v = RMTrajectory->shape_->pline_.vertex_[i];

		vertices_.push_back({ v.x, v.y, v.z, v.h });
		pline_->pline_vertex_data_->push_back(osg::Vec3(v.x, v.y, v.z + z_offset));
	}

	pline_->Update();
	activeRMTrajectory_ = RMTrajectory;
}

void Trajectory::Disable()
{
	activeRMTrajectory_ = 0;
	pline_->pline_vertex_data_->clear();
	vertices_.clear();

	pline_->Update();
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
		osg::Vec3 pos = node->getMatrix().getTrans();
		tx_node->setPivotPoint(pos);
		tx_node->setPosition(pos);

		osg::ref_ptr<osg::Group> parent = node->getParent(0);  // assume the wheel node only belongs to one vehicle
		parent->removeChild(node);
		parent->addChild(tx_node);

		wheel_.push_back(tx_node);
	}

	return tx_node;
}

EntityModel::EntityModel(osgViewer::Viewer *viewer, osg::ref_ptr<osg::Group> group, osg::ref_ptr<osg::Group> parent,
	osg::ref_ptr<osg::Group> trail_parent, osg::ref_ptr<osg::Group> traj_parent, osg::ref_ptr<osg::Node> dot_node,
	osg::Vec4 trail_color,std::string name)
{
	if (!group)
	{
		return;
	}
	name_ = name;
	group_ = group;
	parent_ = parent;

	// Add LevelOfDetail node
	lod_ = new osg::LOD();
	lod_->addChild(group_);
	lod_->setRange(0, 0, LOD_DIST);
	lod_->setName(name);

	// Add transform node
	txNode_ = new osg::PositionAttitudeTransform();
	txNode_->addChild(lod_);
	txNode_->setName(name);
	parent_->addChild(txNode_);

	// Find boundingbox
	bb_ = 0;
	for (int i = 0; i < (int)group->getNumChildren(); i++)
	{
		if (group->getChild(i)->getName() == "BoundingBox")
		{
			bb_ = (osg::Group*)group->getChild(i);
			break;
		}
	}

	// Add trajectory placeholder
	trajectory_ = new Trajectory(traj_parent, viewer);

	viewer_ = viewer;
	state_set_ = 0;
	blend_color_ = 0;

	// Extract boundingbox of car to calculate size and center
	osg::ComputeBoundsVisitor cbv;
	txNode_->accept(cbv);
	osg::BoundingBox boundingBox = cbv.getBoundingBox();
	const osg::MatrixList& m = txNode_->getWorldMatrices();
	osg::Vec3 minV = boundingBox._min * m.front();
	osg::Vec3 maxV = boundingBox._max * m.front();

	size_x = maxV.x() - minV.x();
	size_y = maxV.y() - minV.y();
	center_x = (maxV.x() + minV.x()) / 2;
	center_y = (maxV.y() + minV.y()) / 2;

	// Prepare trail of dots
	trail_ = new PolyLine(trail_parent, 0, trail_color, TRAIL_WIDTH, TRAIL_DOT3D_SIZE, true);
	trail_->SetNodeMaskLines(NodeMask::NODE_MASK_TRAIL_LINES);
	trail_->SetNodeMaskDots(NodeMask::NODE_MASK_TRAIL_DOTS);
}

EntityModel::~EntityModel()
{
	parent_->removeChild(txNode_);
}


CarModel::CarModel(osgViewer::Viewer* viewer, osg::ref_ptr<osg::Group> group, osg::ref_ptr<osg::Group> parent,
	osg::ref_ptr<osg::Group> trail_parent, osg::ref_ptr<osg::Group> traj_parent, osg::ref_ptr<osg::Node> dot_node, osg::Vec4 trail_color, std::string name):
	EntityModel(viewer, group, parent, trail_parent, traj_parent, dot_node, trail_color, name)
{
	steering_sensor_ = 0;
	road_sensor_ = 0;
	lane_sensor_ = 0;
	trail_sensor_ = 0;

	wheel_angle_ = 0;
	wheel_rot_ = 0;

	osg::ref_ptr<osg::Group> retval[4];
	osg::ref_ptr<osg::Node> car_node = txNode_->getChild(0);
	retval[0] = AddWheel(car_node, "wheel_fl");
	retval[1] = AddWheel(car_node, "wheel_fr");
	retval[2] = AddWheel(car_node, "wheel_rr");
	retval[3] = AddWheel(car_node, "wheel_rl");

	// Print message only if some wheel nodes are missing
	if (retval[0] || retval[1] || retval[2] || retval[3])
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
}

CarModel ::~CarModel()
{
	wheel_.clear();
	delete trail_;
}

void EntityModel::SetPosition(double x, double y, double z)
{
	txNode_->setPosition(osg::Vec3(x, y, z));
}

void EntityModel::SetRotation(double hRoad, double pRoad, double hRelative, double r)
{
	// First align to road orientation
	osg::Quat quatTmp(
			0, osg::Vec3(osg::X_AXIS),      // Roll
			pRoad, osg::Vec3(osg::Y_AXIS),  // Pitch
			hRoad, osg::Vec3(osg::Z_AXIS)   // Heading
		);

	// Rotation relative road
	quat_.makeRotate(
		r, osg::Vec3(osg::X_AXIS),         // Roll
		0, osg::Vec3(osg::Y_AXIS),         // Pitch
		hRelative, osg::Vec3(osg::Z_AXIS)  // Heading
	);

	// Combine
	txNode_->setAttitude(quat_* quatTmp);
}

void EntityModel::SetRotation(double h, double p, double r)
{
	quat_.makeRotate(
		r, osg::Vec3(osg::X_AXIS),  // Roll
		p, osg::Vec3(osg::Y_AXIS),  // Pitch
		h, osg::Vec3(osg::Z_AXIS)   // Heading
	);

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

void EntityModel::SetTransparency(double factor)
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
	osgViewer_ = 0;

	if(scenarioFilename != NULL && strcmp(scenarioFilename, ""))
	{
		SE_Env::Inst().AddPath(DirNameOf(scenarioFilename));
	}

	if (odrManager != NULL)
	{
		SE_Env::Inst().AddPath(DirNameOf(odrManager->GetOpenDriveFilename()));
	}

	if (modelFilename != NULL && strcmp(modelFilename, ""))
	{
		SE_Env::Inst().AddPath(DirNameOf(modelFilename));
	}

	// suppress OSG info messages
	osg::setNotifyLevel(osg::NotifySeverity::WARN);

	lodScale_ = LOD_SCALE_DEFAULT;
	currentCarInFocus_ = 0;
	keyUp_ = false;
	keyDown_ = false;
	keyLeft_ = false;
	keyRight_ = false;
	quit_request_ = false;
	camMode_ = osgGA::RubberbandManipulator::RB_MODE_ORBIT;
	shadow_node_ = NULL;
	environment_ = NULL;
	roadGeom = NULL;

	int aa_mode = DEFAULT_AA_MULTISAMPLES;
	if (opt && (arg_str = opt->GetOptionArg("aa_mode")) != "")
	{
		aa_mode = atoi(arg_str.c_str());
	}
	osg::DisplaySettings::instance()->setNumMultiSamples(aa_mode);

	arguments.getApplicationUsage()->addCommandLineOption("--lodScale <number>", "LOD Scale");
	arguments.read("--lodScale", lodScale_);

	clear_color = (arguments.find("--clear-color") != -1);

	// Store arguments in case we need to create a second viewer if the first fails
	int argc = arguments.argc() + 2;  // make room for screen argument
	char **argv = (char**)malloc(argc * sizeof(char*));
	for (int i = 0; i < argc; i++)
	{
		argv[i] = (char*)malloc(strlen(arguments.argv()[i]) + 1);  // +1 to include null termination
		strncpy(argv[i], arguments.argv()[i], strlen(arguments.argv()[i]) + 1);
	}

	osgViewer_ = new osgViewer::Viewer(arguments);
	if (osgViewer_ == nullptr)
	{
		LOG("Failed to initialize OSG viewer");
		return;
	}

	// Check if the viewer has been created correctly - window created is a indication
	osgViewer::ViewerBase::Windows wins;
	osgViewer_->getWindows(wins);
	if (wins.size() == 0)
	{
		// Viewer failed to create window. Probably Anti Aliasing is not supported on executing platform.
		// Make another attempt without AA
		LOG("Viewer failure. Probably requested level of Anti Aliasing (%d multisamples) is not supported - try a lower number. Making another attempt without Anti-Alias and single screen.", aa_mode);
		osg::DisplaySettings::instance()->setNumMultiSamples(0);
		delete osgViewer_;
		osg::ArgumentParser args2(&argc, argv);
		// force single screen
		strncpy(argv[argc - 2], "--screen", strlen("--screen") + 1);
		strncpy(argv[argc - 1], "0", strlen("0") + 1);

		osgViewer_ = new osgViewer::Viewer(args2);
		osgViewer_->getWindows(wins);
		if (wins.size() == 0)
		{
			LOG("Failed second attempt opening a viewer window. Give up.");
			delete osgViewer_;
			for (int i = 0; i < argc; i++)
			{
				free(argv[i]);
			}
			return;
		}
	}
	else
	{
		for (int i = 0; i < argc; i++)
		{
			free(argv[i]);
		}
		free(argv);
	}

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
	envTx_->setPosition(osg::Vec3(0, 0, 0));
	envTx_->setScale(osg::Vec3(1, 1, 1));
	envTx_->setAttitude(osg::Quat(0, 0, 0, 1));
	envTx_->setNodeMask(NodeMask::NODE_MASK_ENV_MODEL);
	rootnode_->addChild(envTx_);

	ClearNodeMaskBits(NodeMask::NODE_MASK_TRAIL_LINES); // hide trails per default
	ClearNodeMaskBits(NodeMask::NODE_MASK_TRAIL_DOTS);
	ClearNodeMaskBits(NodeMask::NODE_MASK_OSI_LINES);
	ClearNodeMaskBits(NodeMask::NODE_MASK_OSI_POINTS);
	ClearNodeMaskBits(NodeMask::NODE_MASK_OBJECT_SENSORS);
	ClearNodeMaskBits(NodeMask::NODE_MASK_ODR_FEATURES);
	ClearNodeMaskBits(NodeMask::NODE_MASK_ENTITY_BB);
	SetNodeMaskBits(NodeMask::NODE_MASK_ENTITY_MODEL);
	SetNodeMaskBits(NodeMask::NODE_MASK_INFO);
	SetNodeMaskBits(NodeMask::NODE_MASK_TRAJECTORY_LINES);

	roadSensors_ = new osg::Group;
	roadSensors_->setNodeMask(NodeMask::NODE_MASK_ODR_FEATURES);
	rootnode_->addChild(roadSensors_);
	trails_ = new osg::Group;
	rootnode_->addChild(trails_);
	odrLines_ = new osg::Group;
	odrLines_->setNodeMask(NodeMask::NODE_MASK_ODR_FEATURES);
	rootnode_->addChild(odrLines_);
	osiFeatures_ = new osg::Group;
	rootnode_->addChild(osiFeatures_);
	trajectoryLines_ = new osg::Group;
	trajectoryLines_->setNodeMask(NodeMask::NODE_MASK_TRAJECTORY_LINES);
	rootnode_->addChild(trajectoryLines_);
	exe_path_ = exe_path;

	// add environment
	if (modelFilename != 0 && strcmp(modelFilename, ""))
	{
		std::vector<std::string> file_name_candidates;

		// absolute path or relative to current directory
		file_name_candidates.push_back(modelFilename);

		// Remove all directories from path and look in current directory
		file_name_candidates.push_back(FileNameOf(modelFilename));

		// Finally check registered paths
		for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
		{
			// Including file path
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], modelFilename));

			// Excluding file path
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(modelFilename)));
		}

		size_t i;
		for (i = 0; i < file_name_candidates.size(); i++)
		{
			if (FileExists(file_name_candidates[i].c_str()))
			{
				if (AddEnvironment(file_name_candidates[i].c_str()) == 0)
				{
					break;
				}
			}
		}

		if (i == file_name_candidates.size())
		{
			LOG("Failed to read environment model %s!", modelFilename);
		}
	}

	if (environment_ == 0)
	{
		if (odrManager->GetNumOfRoads() > 0)
		{
			// No visual model of the road network loaded
			// Generate a simplistic 3D model based on OpenDRIVE content
			LOG("No scenegraph 3D model loaded. Generating a simplistic one...");

			roadGeom = new RoadGeom(odrManager);
			environment_ = roadGeom->root_;
			envTx_->addChild(environment_);

			// Since the generated 3D model is based on OSI features, let's hide those
			ClearNodeMaskBits(NodeMask::NODE_MASK_ODR_FEATURES);
			ClearNodeMaskBits(NodeMask::NODE_MASK_OSI_LINES);
		}
	}

	//const osg::BoundingSphere bs = environment_->getBound();
	//printf("bs radius %.2f\n", bs.radius());
	if (odrManager->GetNumOfRoads() > 0 && !CreateRoadLines(odrManager))
	{
		LOG("Viewer::Viewer Failed to create road lines!\n");
	}

	if (odrManager->GetNumOfRoads() > 0 && !CreateRoadMarkLines(odrManager))
	{
		LOG("Viewer::Viewer Failed to create road mark lines!\n");
	}

	if (!(opt && opt->GetOptionSet("generate_no_road_objects")))
	{
		if (odrManager->GetNumOfRoads() > 0 && CreateRoadSignsAndObjects(odrManager) != 0)
		{
			LOG("Viewer::Viewer Failed to create road signs and objects!\n");
		}
	}

	if (roadGeom && opt && (opt->GetOptionSet("save_generated_model")))
	{
		// If road model was generated AND user want to save it
		if (osgDB::writeNodeFile(*envTx_, "generated_road.osgb"))
		{
			LOG("Saved generated 3D model in \"generated_road.osgb\"");
		}
		else
		{
			LOG("Failed to save generated 3D model");
		}
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
	nodeTrackerManipulator_->setWheelZoomFactor(-1 * nodeTrackerManipulator_->getWheelZoomFactor());  // inverse scroll wheel

	osg::ref_ptr<osgGA::TrackballManipulator> trackBallManipulator;
	trackBallManipulator = new osgGA::TrackballManipulator;
	trackBallManipulator->setVerticalAxisFixed(true);
	trackBallManipulator->setWheelZoomFactor(-1 * trackBallManipulator->getWheelZoomFactor());  // inverse scroll wheel

	osg::ref_ptr<osgGA::OrbitManipulator> orbitManipulator;
	orbitManipulator = new osgGA::OrbitManipulator;
	orbitManipulator->setVerticalAxisFixed(true);
	orbitManipulator->setWheelZoomFactor(-1 * orbitManipulator->getWheelZoomFactor());  // inverse scroll wheel

	osg::ref_ptr<osgGA::TerrainManipulator> terrainManipulator;
	terrainManipulator = new osgGA::TerrainManipulator();
	terrainManipulator->setWheelZoomFactor(-1 * terrainManipulator->getWheelZoomFactor());  // inverse scroll wheel

	rubberbandManipulator_ = new osgGA::RubberbandManipulator(camMode_);
	rubberbandManipulator_->setTrackNode(envTx_);
	rubberbandManipulator_->calculateCameraDistance();

	{
		osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

		keyswitchManipulator->addMatrixManipulator('1', "Rubberband", rubberbandManipulator_.get());
		keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
		keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
		keyswitchManipulator->addMatrixManipulator('4', "Terrain", terrainManipulator.get());
		keyswitchManipulator->addMatrixManipulator('5', "Orbit", orbitManipulator.get());
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

#if 1
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
	osgViewer_->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
	osg::Light *light = osgViewer_->getLight();
	light->setPosition(osg::Vec4(-7500., 5000., 10000., 1.0));
	light->setDirection(osg::Vec3(7.5, -5., -10.));
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
	infoText->setNodeMask(NodeMask::NODE_MASK_INFO);

	textGeode->addDrawable(infoText);

	infoTextCamera = new osg::Camera;
	infoTextCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	infoTextCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	infoTextCamera->setRenderOrder(osg::Camera::POST_RENDER, 10);
	infoTextCamera->setAllowEventFocus(false);
	infoTextCamera->addChild(textGeode.get());
	infoTextCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	osg::GraphicsContext* context = dynamic_cast<osgViewer::GraphicsWindow*>(osgViewer_->getCamera()->getGraphicsContext());
	SetInfoTextProjection(context->getTraits()->width, context->getTraits()->height);

	rootnode_->addChild(infoTextCamera);

}

Viewer::~Viewer()
{
	osgViewer_->setDone(true);
	for (size_t i=0; i< entities_.size(); i++)
	{
		delete(entities_[i]);
	}
	entities_.clear();
	delete osgViewer_;
	osgViewer_ = 0;
}

void Viewer::PrintUsage()
{
	// Inform about a few OSG options
	printf("Additional OSG graphics options:\n");
	printf("  --clear-color <color>         Set the background color of the viewer in the form \"r,g,b[,a]\"\n");
	printf("  --screen <num>                Set the screen to use when multiple screens are present\n");
	printf("  --window <x y w h>            Set the position (x,y) and size (w,h) of the viewer window\n");
	printf("  --borderless-window <x y w h>	Set the position(x, y) and size(w, h) of a borderless viewer window\n");
	printf("\n");
}

void Viewer::SetCameraMode(int mode)
{
	if (mode < 0 || mode >= osgGA::RubberbandManipulator::RB_NUM_MODES)
	{
		return;
	}

	camMode_ = mode;
	rubberbandManipulator_->setMode(camMode_);
	UpdateCameraFOV();
}

void Viewer::UpdateCameraFOV()
{
	double fov;

	if (camMode_ == osgGA::RubberbandManipulator::RB_MODE_TOP)
	{
		fov = ORTHO_FOV;
	}
	else
	{
		fov = PERSP_FOV;
	}

	osg::ref_ptr<osg::GraphicsContext> gc = osgViewer_->getCamera()->getGraphicsContext();
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = const_cast<osg::GraphicsContext::Traits*>(osgViewer_->getCamera()->getGraphicsContext()->getTraits());

	osgViewer_->getCamera()->setProjectionMatrixAsPerspective(fov, (double)traits->width / traits->height, 1.0*PERSP_FOV/fov,  1E5 * PERSP_FOV / fov);

	osgViewer_->getCamera()->setLODScale(fov/PERSP_FOV);
}

EntityModel* Viewer::AddEntityModel(std::string modelFilepath, osg::Vec4 trail_color, EntityModel::EntityType type,
	bool road_sensor, std::string name, OSCBoundingBox* boundingBox)
{
	// Load 3D model
	std::string path = modelFilepath;
	osg::ref_ptr<osg::Group> group = 0;

	std::vector<std::string> file_name_candidates;
	file_name_candidates.push_back(path);
	file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(exe_path_) + "/../resources/models", path));
	// Finally check registered paths
	for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
	{
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], path));
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], std::string("../models/" + path)));
	}
	for (size_t i = 0; i < file_name_candidates.size(); i++)
	{
		if (FileExists(file_name_candidates[i].c_str()))
		{
			if (group = LoadEntityModel(file_name_candidates[i].c_str()))
			{
				break;
			}
		}
	}

	if (boundingBox || group == 0)
	{
		// Create a bounding box visual representation

		if (path == "")
		{
			LOG("No filename specified for model! - creating a dummy model");
		}
		else if (group == 0)
		{
			LOG("Failed to load visual model %s. %s", path.c_str(), file_name_candidates.size() > 1 ? "Also tried the following paths:" : "");
			for (size_t i = 1; i < file_name_candidates.size(); i++)
			{
				LOG("    %s", file_name_candidates[i].c_str());
			}
			LOG("Creating a dummy model instead");
		}

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(new osg::ShapeDrawable(new osg::Box()));

		osg::ref_ptr<osg::Group> bbGroup = new osg::Group;
		osg::ref_ptr<osg::PositionAttitudeTransform> tx = new osg::PositionAttitudeTransform;

		osg::Material* material = new osg::Material();

		// Set color of vehicle based on its index
		double* color;
		double b = 1.5;  // brighness
		int index = entities_.size() % 4;

		if (index == 0) color = color_white;
		else if (index == 1) color = color_red;
		else if (index == 2) color = color_blue;
		else color = color_yellow;

		material->setDiffuse(osg::Material::FRONT, osg::Vec4(b * color[0], b * color[1], b * color[2], 1.0));
		material->setAmbient(osg::Material::FRONT, osg::Vec4(b * color[0], b * color[1], b * color[2], 1.0));

		if (group == 0)
		{
			// If no model loaded, make a shaded copy of bounding box as model
			osg::ref_ptr<osg::Geode> geodeCopy = dynamic_cast<osg::Geode*>(geode->clone(osg::CopyOp::DEEP_COPY_ALL));
			group = new osg::Group;
			tx->addChild(geodeCopy);
			geodeCopy->setNodeMask(NodeMask::NODE_MASK_ENTITY_MODEL);
		}

		// Set dimensions of the entity "box"
		if (boundingBox)
		{
			tx->setScale(osg::Vec3(boundingBox->dimensions_.length_, boundingBox->dimensions_.width_, boundingBox->dimensions_.height_));
			tx->setPosition(osg::Vec3(boundingBox->center_.x_, boundingBox->center_.y_, boundingBox->center_.z_));

			// Draw only wireframe
			osg::PolygonMode* polygonMode = new osg::PolygonMode;
			polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
			osg::ref_ptr<osg::StateSet> stateset = geode->getOrCreateStateSet(); // Get the StateSet of the group
			stateset->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
			stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
			geode->setNodeMask(NodeMask::NODE_MASK_ENTITY_BB);

			osg::ref_ptr<osg::Geode> center = new osg::Geode;
			center->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), 0.2)));

			center->setNodeMask(NodeMask::NODE_MASK_ENTITY_BB);
			bbGroup->addChild(center);
		}
		else
		{
			// Scale to something car-ish
			tx->setScale(osg::Vec3(4.5, 1.8, 1.5));
			tx->setPosition(osg::Vec3(1.5, 0, 0.75));
		}
		tx->addChild(geode);
		tx->getOrCreateStateSet()->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
		tx->getOrCreateStateSet()->setAttribute(material);
		bbGroup->setName("BoundingBox");
		bbGroup->addChild(tx);

		group->addChild(bbGroup);
		group->setName(name);
	}

	EntityModel* model;
	if (type == EntityModel::EntityType::ENTITY_TYPE_VEHICLE)
	{
		model = new CarModel(osgViewer_, group, rootnode_, trails_, trajectoryLines_, dot_node_, trail_color, name);
	}
	else
	{
		model = new EntityModel(osgViewer_, group, rootnode_, trails_, trajectoryLines_, dot_node_, trail_color, name);
	}

	model->state_set_ = model->lod_->getOrCreateStateSet(); // Creating material
	model->blend_color_ = new osg::BlendColor(osg::Vec4(1, 1, 1, 1));
	model->state_set_->setAttributeAndModes(model->blend_color_);
	model->blend_color_->setDataVariance(osg::Object::DYNAMIC);

	osg::BlendFunc* bf = new osg::BlendFunc(osg::BlendFunc::CONSTANT_ALPHA, osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
	model->state_set_->setAttributeAndModes(bf);
	model->state_set_->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	model->state_set_->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	entities_.push_back(model);

	// Focus on first added car
	if (entities_.size() == 1)
	{
		currentCarInFocus_ = 0;
		rubberbandManipulator_->setTrackNode(entities_.back()->txNode_,
			rubberbandManipulator_->getMode() == osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_TOP ? false : true);
		nodeTrackerManipulator_->setTrackNode(entities_.back()->txNode_);
	}

	if (type == EntityModel::EntityType::ENTITY_TYPE_VEHICLE)
	{
		CarModel* vehicle = (CarModel*)entities_.back();
		CreateRoadSensors(vehicle);

		if (road_sensor)
		{
			vehicle->road_sensor_->Show();
		}
		else
		{
			vehicle->road_sensor_->Hide();
		}
	}

	return entities_.back();
}

void Viewer::RemoveCar(std::string name)
{
	for (size_t i = 0; i < entities_.size(); i++)
	{
		if (entities_[i]->name_ == name)
		{
			if (entities_[i] != nullptr)
			{
				delete (entities_[i]);
			}
			entities_.erase(entities_.begin() + i);

			if (currentCarInFocus_ > i)
			{
				// Shift with reduces list
				currentCarInFocus_--;
			}
			else if (currentCarInFocus_ == i)
			{
				if (entities_.size() > 0)
				{
					SetVehicleInFocus((currentCarInFocus_) % entities_.size());
				}
				else
				{
					// No more objects to follow, switch camera model
					currentCarInFocus_ = -1;
					((osgGA::KeySwitchMatrixManipulator*)osgViewer_->getCameraManipulator())->selectMatrixManipulator(5);
				}
			}

			break;
		}
	}
}

osg::ref_ptr<osg::Group> Viewer::LoadEntityModel(const char *filename)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> shadow_tx = 0;
	osg::ref_ptr<osg::Node> node;
	osg::ref_ptr<osg::Group> group = new osg::Group;

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

	node->setNodeMask(NodeMask::NODE_MASK_ENTITY_MODEL);
	group->addChild(node);

	if (shadow_node_)
	{
		shadow_tx = new osg::PositionAttitudeTransform;
		shadow_tx->setPosition(osg::Vec3d(xc, yc, 0.0));
		shadow_tx->setScale(osg::Vec3d(SHADOW_SCALE*(dx / 2), SHADOW_SCALE*(dy / 2), 1.0));
		shadow_tx->addChild(shadow_node_);

		shadow_tx->setNodeMask(NodeMask::NODE_MASK_ENTITY_MODEL);
		group->addChild(shadow_tx);
	}


	return group;
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
						int inner_index = -1;
						if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN_SOLID ||
							lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID_BROKEN)
						{
							if (lane_roadmarktype->GetNumberOfRoadMarkTypeLines() < 2)
							{
								break;
								std::runtime_error("You need to specify at least 2 line for broken solid or solid broken roadmark type");
							}
							std::vector<double> sort_solidbroken_brokensolid;
							for (int q=0; q<lane_roadmarktype->GetNumberOfRoadMarkTypeLines(); q++)
							{
								sort_solidbroken_brokensolid.push_back(lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(q)->GetTOffset());
							}

							if (lane->GetId() < 0 || lane->GetId() == 0)
							{
								inner_index = std::max_element(sort_solidbroken_brokensolid.begin(), sort_solidbroken_brokensolid.end()) - sort_solidbroken_brokensolid.begin();
							}
							else
							{
								inner_index = std::min_element(sort_solidbroken_brokensolid.begin(), sort_solidbroken_brokensolid.end()) - sort_solidbroken_brokensolid.begin();
							}
						}

						for (int n = 0; n < lane_roadmarktype->GetNumberOfRoadMarkTypeLines(); n++)
						{
							roadmanager::LaneRoadMarkTypeLine * lane_roadmarktypeline = lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(n);
							roadmanager::OSIPoints* curr_osi_rm = lane_roadmarktypeline->GetOSIPoints();

							bool broken = false;
							if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN_SOLID)
							{
								if (inner_index == n)
								{
									broken = true;
								}
							}

							if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID_BROKEN)
							{
								broken = true;
								if (inner_index == n)
								{
									broken = false;
								}
							}

							if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN ||
								lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN_BROKEN ||
								broken)
							{
								for (int q = 0; q < curr_osi_rm->GetPoints().size(); q+=2)
								{
									roadmanager::PointStruct osi_point1 = curr_osi_rm->GetPoint(q);
									roadmanager::PointStruct osi_point2 = curr_osi_rm->GetPoint(q+1);

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

									osi_rm_geom->setNodeMask(NodeMask::NODE_MASK_OSI_POINTS);
									osiFeatures_->addChild(osi_rm_geom);

									// Draw lines from the start of the roadmark to the end of the roadmark
									lineWidth->setWidth(OSI_LINE_WIDTH);
									geom->setVertexArray(osi_rm_points.get());
									geom->setColorArray(osi_rm_color.get());
									geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
									geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, osi_rm_points->size()));
									geom->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
									geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

									geom->setNodeMask(NodeMask::NODE_MASK_OSI_LINES);
									osiFeatures_->addChild(geom);
								}
							}
							else if(lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID ||
							lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID_SOLID ||
							!broken)
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
								for (int s = 0; s < curr_osi_rm->GetPoints().size(); s++)
								{
									point.set(curr_osi_rm->GetPoint(s).x, curr_osi_rm->GetPoint(s).y, curr_osi_rm->GetPoint(s).z + z_offset);
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

								osi_rm_geom->setNodeMask(NodeMask::NODE_MASK_OSI_POINTS);
								osiFeatures_->addChild(osi_rm_geom);

								// Draw lines between each selected points
								lineWidth->setWidth(OSI_LINE_WIDTH);
								color->push_back(osg::Vec4(color_white[0], color_white[1], color_white[2], 1.0));

								geom->setVertexArray(osi_rm_points.get());
								geom->setColorArray(color.get());
								geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
								geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, osi_rm_points->size()));
								geom->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);
								geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

								geom->setNodeMask(NodeMask::NODE_MASK_OSI_LINES);
								osiFeatures_->addChild(geom);
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

		roadmanager::Geometry *geom = nullptr;
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

				// visualize both lane center and lane boundary
				for (int k = 0; k < 2; k++)
				{
					// skip lane center for all non driving lanes except center lane
					if ((k == 0 && lane->GetId() != 0 && !lane->IsDriving()) ||
						// skip lane boundary for center lane
						(k == 1 && lane->GetId() == 0))
					{
						continue;
					}

					// osg references for drawing lines on the lane center using osi points
					osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

					if (k == 0)
					{
						curr_osi = lane->GetOSIPoints();
					}
					else
					{
						curr_osi = lane->GetLaneBoundary()->GetOSIPoints();
					}

					if (curr_osi == 0)
					{
						continue;
					}

					for (int m = 0; m < curr_osi->GetPoints().size(); m++)
					{
						roadmanager::PointStruct osi_point_s = curr_osi->GetPoint(m);
						vertices->push_back(osg::Vec3(osi_point_s.x, osi_point_s.y, osi_point_s.z + z_offset));
					}

					PolyLine* pline = nullptr;
					if (lane->GetId() == 0)
					{
						pline = AddPolyLine(odrLines_, vertices, osg::Vec4(color_red[0], color_red[1], color_red[2], 1.0), 4.0, 3.0);
					}
					else if (k==0)
					{
						pline = AddPolyLine(odrLines_, vertices, osg::Vec4(color_blue[0], color_blue[1], color_blue[2], 1.0), 1.5, 3.0);
					}
					else
					{
						pline = AddPolyLine(odrLines_, vertices, osg::Vec4(color_gray[0], color_gray[1], color_gray[2], 1.0), 1.5, 3.0);
					}
					if (pline != nullptr)
					{
						pline->SetNodeMaskDots(NodeMask::NODE_MASK_OSI_POINTS);
					}
				}
			}
		}
	}

	delete pos;

	return true;
}

int Viewer::CreateOutlineObject(roadmanager::Outline *outline)
{
	if (outline == 0) return -1;
	bool roof = outline->closed_ ? true : false;

	// nrPoints will be corners + 1 if the outline should be closed, reusing first corner as last
	int nrPoints = outline->closed_ ? outline->corner_.size() + 1 : outline->corner_.size();

	osg::ref_ptr<osg::Group> group = new osg::Group();
	osg::Vec4 color = osg::Vec4(0.5f, 0.5f, 0.5f, 1.0f);  // outline objects will be gray
	osg::ref_ptr<osg::Vec4Array> color_outline = new osg::Vec4Array;
	color_outline->push_back(color);

	osg::ref_ptr<osg::Vec3Array> vertices_sides = new osg::Vec3Array(nrPoints * 2);  // one set at bottom and one at top
	osg::ref_ptr<osg::Vec3Array> vertices_top = new osg::Vec3Array(nrPoints);  // one set at bottom and one at top

	// Set vertices
	for (size_t i = 0; i < outline->corner_.size(); i++)
	{
		double x, y, z;
		roadmanager::OutlineCorner* corner = outline->corner_[i];
		corner->GetPos(x, y, z);
		(*vertices_sides)[i * 2 + 0].set(x, y, z + corner->GetHeight());
		(*vertices_sides)[i * 2 + 1].set(x, y, z);
		(*vertices_top)[i].set(x, y, z + corner->GetHeight());
	}

	// Close geometry
	if (outline->closed_)
	{
		(*vertices_sides)[2 * nrPoints - 2].set((*vertices_sides)[0]);
		(*vertices_sides)[2 * nrPoints - 1].set((*vertices_sides)[1]);
		(*vertices_top)[nrPoints-1].set((*vertices_top)[0]);
	}

	// Finally create and add geometry
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geom[] = { new osg::Geometry, new osg::Geometry };

	geom[0]->setVertexArray(vertices_sides.get());
	geom[0]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, 2 * nrPoints));

	if (roof)
	{
		geom[1]->setVertexArray(vertices_top.get());
		geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, nrPoints));
		osgUtil::Tessellator tessellator;
		tessellator.retessellatePolygons(*geom[1]);
	}

	int nrGeoms = roof ? 2 : 1;
	for (int i = 0; i < nrGeoms; i++)
	{
		osgUtil::SmoothingVisitor::smooth(*geom[i], 0.5);
		geom[i]->setColorArray(color_outline);
		geom[i]->setColorBinding(osg::Geometry::BIND_OVERALL);
		geom[i]->setDataVariance(osg::Object::STATIC);
		geom[i]->setUseDisplayList(true);
		geode->addDrawable(geom[i]);
	}

	osg::ref_ptr<osg::Material> material_ = new osg::Material;
	material_->setDiffuse(osg::Material::FRONT_AND_BACK, color);
	material_->setAmbient(osg::Material::FRONT_AND_BACK, color);
	geode->getOrCreateStateSet()->setAttributeAndModes(material_.get());

	group->addChild(geode);
	envTx_->addChild(group);

	return 0;
}

osg::ref_ptr<osg::PositionAttitudeTransform> Viewer::LoadRoadFeature(roadmanager::Road *road, std::string filename)
{
	osg::ref_ptr<osg::Node> node;
	osg::ref_ptr<osg::PositionAttitudeTransform> xform = 0;

	// Load file, try multiple paths
	std::vector<std::string> file_name_candidates;
	file_name_candidates.push_back(filename);
	file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(exe_path_) + "/../resources/models", filename));
	// Finally check registered paths
	for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
	{
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], filename));
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], std::string("../models/" + filename)));
	}
	for (size_t i = 0; i < file_name_candidates.size(); i++)
	{
		if (FileExists(file_name_candidates[i].c_str()))
		{
			node = osgDB::readNodeFile(file_name_candidates[i]);
			if (!node)
			{
				return 0;
			}

			xform = new osg::PositionAttitudeTransform;
			xform->addChild(node);
		}
	}

	return xform;
}

int Viewer::CreateRoadSignsAndObjects(roadmanager::OpenDrive* od)
{
	osg::ref_ptr<osg::Group> objGroup = new osg::Group;
	osg::ref_ptr<osg::PositionAttitudeTransform> tx = 0;

	roadmanager::Position pos;

	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		roadmanager::Road* road = od->GetRoadByIdx(r);
		for (size_t s = 0; s < road->GetNumberOfSignals(); s++)
		{
			roadmanager::Signal* signal = road->GetSignal(s);
			double orientation = signal->GetOrientation() == roadmanager::Signal::Orientation::NEGATIVE ? M_PI : 0.0;

			tx = LoadRoadFeature(road, signal->GetName() + ".osgb");

			if (tx == nullptr)
			{
				return -1;
			}
			else
			{
				pos.SetTrackPos(road->GetId(), signal->GetS(), signal->GetT());
				tx->setPosition(osg::Vec3(pos.GetX(), pos.GetY(), signal->GetZOffset() + pos.GetZ()));
				tx->setAttitude(osg::Quat(pos.GetH() + orientation + signal->GetHOffset(), osg::Vec3(0, 0, 1)));

				objGroup->addChild(tx);
			}
		}

		for (size_t o = 0; o < road->GetNumberOfObjects(); o++)
		{
			roadmanager::RMObject* object = road->GetObject(o);

			if (object->GetNumberOfOutlines() > 0)
			{
				for (size_t j = 0; j < object->GetNumberOfOutlines(); j++)
				{
					roadmanager::Outline* outline = object->GetOutline(j);
					CreateOutlineObject(outline);
				}
				LOG("Created outline geometry for object %s.", object->GetName().c_str());
				LOG("  if it looks strange, e.g.faces too dark or light color, ");
				LOG("  check that corners are defined counter-clockwise (as OpenGL default).");
			}
			else
			{
				// Assume name is representing a 3D model filename
				double orientation = object->GetOrientation() == roadmanager::Signal::Orientation::NEGATIVE ? M_PI : 0.0;

				std::vector<std::string> file_name_candidates;

				// absolute path or relative to current directory
				std::string filename = object->GetName();
				if (FileNameExtOf(filename) == "")
				{
					filename += ".osgb";  // add missing extension
				}

				tx = LoadRoadFeature(road, filename);

				if (tx == 0)
				{
					LOG("Failed to load road object model file: %s (%s)", filename.c_str(), object->GetName().c_str());
					return -1;
				}

				roadmanager::Repeat* rep = object->GetRepeat();
				int nCopies = 1;
				if (rep && rep->GetDistance() > SMALL_NUMBER)
				{
					nCopies = rep->length_ / rep->distance_;
				}

				osg::ComputeBoundsVisitor cbv;
				tx->accept(cbv);
				osg::BoundingBox boundingBox = cbv.getBoundingBox();

				double dim_x = boundingBox._max.x() - boundingBox._min.x();
				double dim_y = boundingBox._max.y() - boundingBox._min.y();
				double dim_z = boundingBox._max.z() - boundingBox._min.z();

				double lastLODs = 0.0;  // used for putting object copies in LOD groups
				osg::ref_ptr<osg::Group> LODGroup = 0;
				osg::ref_ptr<osg::PositionAttitudeTransform> clone = 0;

				for (size_t i = 0; i < nCopies; i++)
				{
					double factor, t, s, zOffset;
					double scale_x = 1.0;
					double scale_y = 1.0;
					double scale_z = 1.0;

					clone = dynamic_cast<osg::PositionAttitudeTransform*>(tx->clone(osg::CopyOp::SHALLOW_COPY));

					if (rep == nullptr)
					{
						factor = 1.0;
						t = object->GetT();
						s = object->GetS();
						zOffset = object->GetZOffset();

						if (object->GetLength() > SMALL_NUMBER)
						{
							scale_x = object->GetLength() / dim_x;
						}
						if (object->GetWidth() > SMALL_NUMBER)
						{
							scale_y = object->GetWidth() / dim_y;
						}
						if (object->GetHeight() > SMALL_NUMBER)
						{
							scale_z = object->GetHeight() / dim_z;
						}

						pos.SetTrackPos(road->GetId(), object->GetS(), object->GetT());

						clone->setScale(osg::Vec3(scale_x, scale_y, scale_z));
						clone->setPosition(osg::Vec3(pos.GetX(), pos.GetY(), object->GetZOffset() + pos.GetZ()));

						// First align to road orientation
						osg::Quat quatRoad(osg::Quat(pos.GetR(), osg::X_AXIS, pos.GetP(), osg::Y_AXIS, pos.GetH(), osg::Z_AXIS));
						// Specified local rotation
						osg::Quat quatLocal(orientation + object->GetHOffset(), osg::Vec3(osg::Z_AXIS));  // Heading
						// Combine
						clone->setAttitude(quatLocal* quatRoad);
					}
					else
					{
						factor = (double)i / nCopies;
						t = rep->GetTStart() + factor * (rep->GetTEnd() - rep->GetTStart());
						s = rep->GetS() + i * rep->GetDistance();
						zOffset = rep->GetZOffsetStart() + factor * (rep->GetZOffsetEnd() - rep->GetZOffsetStart());

						if (rep->GetLengthStart() > SMALL_NUMBER || rep->GetLengthEnd() > SMALL_NUMBER)
						{
							scale_x = (rep->GetLengthStart() + factor * (rep->GetLengthEnd() - rep->GetLengthStart())) / dim_x;
						}
						if (rep->GetWidthStart() > SMALL_NUMBER || rep->GetWidthEnd() > SMALL_NUMBER)
						{
							scale_y = (rep->GetWidthStart() + factor * (rep->GetWidthEnd() - rep->GetWidthStart())) / dim_y;
						}
						if (rep->GetHeightStart() > SMALL_NUMBER || rep->GetHeightEnd() > SMALL_NUMBER)
						{
							scale_z = (rep->GetHeightStart() + factor * (rep->GetHeightEnd() - rep->GetHeightStart())) / dim_z;
						}

						pos.SetTrackPos(road->GetId(), s, t);

						clone->setScale(osg::Vec3(scale_x, scale_y, scale_z));
						clone->setPosition(osg::Vec3(pos.GetX(), pos.GetY(), pos.GetZ() + zOffset));

						// First align to road orientation
						osg::Quat quatRoad(osg::Quat(pos.GetR(), osg::X_AXIS, pos.GetP(), osg::Y_AXIS, pos.GetH(), osg::Z_AXIS));
						// Specified local rotation
						osg::Quat quatLocal(object->GetHOffset(), osg::Vec3(osg::Z_AXIS));  // Heading
						// Combine
						clone->setAttitude(quatLocal * quatRoad);
					}

					clone->setDataVariance(osg::Object::STATIC);

					if (LODGroup == 0 || s - lastLODs > 0.5 * LOD_DIST_ROAD_FEATURES)
					{
						// add current LOD and create a new one
						osg::ref_ptr<osg::LOD> lod = new osg::LOD();
						LODGroup = new osg::Group();
						lod->addChild(LODGroup);
						lod->setRange(0, 0, LOD_DIST_ROAD_FEATURES);
						objGroup->addChild(lod);
						lastLODs = s;
					}

					LODGroup->addChild(clone);
				}
			}
		}
	}

	osgUtil::Optimizer optimizer;
	optimizer.optimize(objGroup, osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS);

	envTx_->addChild(objGroup);

	return 0;
}

bool Viewer::CreateRoadSensors(CarModel *vehicle_model)
{
	vehicle_model->road_sensor_ = CreateSensor(color_gray, true, false, 0.25, 2.5);
	vehicle_model->lane_sensor_ = CreateSensor(color_gray, true, true, 0.25, 2.5);

	return true;
}

PointSensor* Viewer::CreateSensor(double color[], bool create_ball, bool create_line, double ball_radius, double line_width)
{
	PointSensor *sensor = new PointSensor();
	sensor->group_ = new osg::Group();

	// Point
	if (create_ball)
	{
		osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,0), ball_radius));
		shape->setColor(osg::Vec4(color[0], color[1], color[2], 1.0));

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(shape);

		sensor->ball_ = new osg::PositionAttitudeTransform;
		sensor->ball_->addChild(geode);

		sensor->ball_radius_ = ball_radius;
		sensor->group_->addChild(sensor->ball_);
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
		sensor->group_->addChild(group);
	}

	// Make sensor visible as default
	roadSensors_->addChild(sensor->group_);
	sensor->Show();

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

	return 0;
}

void Viewer::SetInfoText(const char* text)
{
	if (GetNodeMaskBit(NodeMask::NODE_MASK_INFO))
	{
		infoText->setText(text);
	}
	else
	{
		infoText->setText("");
	}
}

void Viewer::SetNodeMaskBits(int bits)
{
	osgViewer_->getCamera()->setCullMask(osgViewer_->getCamera()->getCullMask() | bits);
}

void Viewer::SetNodeMaskBits(int mask, int bits)
{
	osgViewer_->getCamera()->setCullMask((osgViewer_->getCamera()->getCullMask() & ~mask) | bits);
}

void Viewer::ClearNodeMaskBits(int bits)
{
	osgViewer_->getCamera()->setCullMask(osgViewer_->getCamera()->getCullMask() & ~bits);
}

void Viewer::ToggleNodeMaskBits(int bits)
{
	osgViewer_->getCamera()->setCullMask(osgViewer_->getCamera()->getCullMask() ^ bits);
}

int Viewer::GetNodeMaskBit(int mask)
{
	return osgViewer_->getCamera()->getCullMask() & mask;
}

void Viewer::SetInfoTextProjection(int width, int height)
{
	infoTextCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, width, 0, height));
}

void Viewer::SetVehicleInFocus(int idx)
{
	currentCarInFocus_ = idx;
	if (entities_.size() > idx)
	{
		rubberbandManipulator_->setTrackNode(entities_[currentCarInFocus_]->txNode_, false);
		nodeTrackerManipulator_->setTrackNode(entities_[currentCarInFocus_]->txNode_);
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
			if (arg != "esmini")
			{
				arg = "esmini " + arg;
			}
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

PolyLine* Viewer::AddPolyLine(osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize)
{
	polyLine_.push_back(PolyLine(rootnode_, points, color, width, dotsize));
	return &polyLine_.back();
}

PolyLine* Viewer::AddPolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize)
{
	polyLine_.push_back(PolyLine(parent, points, color, width, dotsize));
	return &polyLine_.back();
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
			viewer_->SetCameraMode((viewer_->camMode_ + 1) % osgGA::RubberbandManipulator::RB_NUM_MODES);
		}
		break;
	case(osgGA::GUIEventAdapter::KEY_O):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_N):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAJECTORY_LINES);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_U):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(viewer::NodeMask::NODE_MASK_OSI_LINES);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Y):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(viewer::NodeMask::NODE_MASK_OSI_POINTS);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_P):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(NodeMask::NODE_MASK_ENV_MODEL);
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

			if (idx >= (int)viewer_->entities_.size())
			{
				idx = 0;
			}
			else if (idx < 0)
			{
				idx = viewer_->entities_.size() - 1;
			}

			viewer_->SetVehicleInFocus(idx);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Comma):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			int mask = viewer_->GetNodeMaskBit(
				viewer::NodeMask::NODE_MASK_ENTITY_MODEL |
				viewer::NodeMask::NODE_MASK_ENTITY_BB) / viewer::NodeMask::NODE_MASK_ENTITY_MODEL;

			// Toggle between modes: 0: none, 1: model only, 2: bounding box, 3. model + Bounding box
			mask = ((mask + 1) % 4) * viewer::NodeMask::NODE_MASK_ENTITY_MODEL;

			viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ENTITY_MODEL |
				viewer::NodeMask::NODE_MASK_ENTITY_BB, mask);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_I):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(viewer::NodeMask::NODE_MASK_INFO);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_J):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			int mask = viewer_->GetNodeMaskBit(
				viewer::NodeMask::NODE_MASK_TRAIL_LINES |
				viewer::NodeMask::NODE_MASK_TRAIL_DOTS) / viewer::NodeMask::NODE_MASK_TRAIL_LINES;

			// Toggle between modes: 0: none, 1: lines only, 2: dots only, 3. lines and dots
			mask = ((mask + 1) % 4) * viewer::NodeMask::NODE_MASK_TRAIL_LINES;

			viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAIL_LINES |
				viewer::NodeMask::NODE_MASK_TRAIL_DOTS, mask);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_R):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(viewer::NodeMask::NODE_MASK_OBJECT_SENSORS);
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
			KeyEvent ke = { ea.getKey(), ea.getModKeyMask(),  ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN ? true : false };
			viewer_->callback_[i].func(&ke, viewer_->callback_[i].data);
		}
	}

	if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Space)
	{
		// prevent OSG "view reset" action on space key
		return true;
	}
	else
	{
		// forward all other key events to OSG
		return false;
	}
}
