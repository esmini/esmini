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
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/Tessellator> // to tessellate multiple contours
#include <osgUtil/Optimizer>   // to flatten transform nodes

#define SHADOW_SCALE 1.20
#define SHADOW_MODEL_FILEPATH "shadow_face.osgb"
#define ARROW_MODEL_FILEPATH "arrow.osgb"
#define LOD_DIST 3000
#define LOD_DIST_ROAD_FEATURES 500
#define LOD_SCALE_DEFAULT 1.0
#define DEFAULT_AA_MULTISAMPLES 4
#define OSI_LINE_WIDTH 2.0f
#define OSI_LINE_WIDTH_BOLD 4.0f
#define TRAIL_WIDTH 2
#define TRAIL_DOT_SIZE 10
#define TRAIL_DOT3D_SIZE 0.2
#define WAYPOINT_HEIGHT 0.2
#define TRAILDOT3D 1
#define PERSP_FOV 30.0
#define ORTHO_FOV 1.0

double color_green[3] = { 0.25, 0.6, 0.3 };
double color_gray[3] = { 0.7, 0.7, 0.7 };
double color_dark_gray[3] = { 0.5, 0.5, 0.5 };
double color_light_gray[3] = { 0.7, 0.7, 0.7 };
double color_red[3] = { 0.73, 0.26, 0.26 };
double color_black[3] = { 0.2, 0.2, 0.2 };
double color_blue[3] = { 0.25, 0.38, 0.7 };
double color_yellow[3] = { 0.75, 0.7, 0.4 };
double color_white[3] = { 0.90, 0.90, 0.85 };

USE_OSGPLUGIN(osg2)
USE_OSGPLUGIN(jpeg)
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
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
		if (node.getName().find(_name) != std::string::npos)
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

class FindNamedNodes : public osg::NodeVisitor
{
public:
	FindNamedNodes(const std::string& name, std::vector<osg::Node*>& nodes)
		: osg::NodeVisitor( // Traverse all children.
			osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
		_name(name), _nodes(nodes) {}

	// This method gets called for every node in the scene graph. Check each node
	// to see if its name matches out target. If so, save the node's address.
	virtual void apply(osg::Group& node)
	{
		if (node.getName().find(_name) != std::string::npos)
		{
			_nodes.push_back(&node);
		}

		// Keep traversing the rest of the scene graph.
		traverse(node);
	}

	osg::Node* getNode() { return _node.get(); }

protected:
	std::string _name;
	std::vector<osg::Node*>& _nodes;
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
	geom[0]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, 2 * (nrPoints + 1)));

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

	if (dots3D && dots3D_group_ != nullptr)
	{
		parent->addChild(dots3D_group_);
	}
	else if (dots_geom_ != nullptr)
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

void PolyLine::Reset()
{
	pline_vertex_data_->clear();
	dots3D_group_->removeChildren(0, dots3D_group_->getNumChildren());

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

SensorViewFrustum::SensorViewFrustum(ObjectSensor* sensor, osg::Group* parent)
{
	sensor_ = sensor;
	txNode_ = new osg::PositionAttitudeTransform;
	txNode_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
	txNode_->setPosition(osg::Vec3(sensor_->pos_.x, sensor_->pos_.y, sensor_->pos_.z));
	txNode_->setAttitude(osg::Quat(sensor_->pos_.h, osg::Vec3(0, 0, 1)));
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

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4 * (numSegments + 1) + 1);
	osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_QUADS, 2 * 4 + 4 * 4 * numSegments);

	osg::ref_ptr<osg::DrawElementsUInt> indicesC0 = new osg::DrawElementsUInt(GL_LINE_STRIP, numSegments + 1);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC1 = new osg::DrawElementsUInt(GL_LINE_STRIP, numSegments + 1);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC2 = new osg::DrawElementsUInt(GL_LINE_STRIP, numSegments + 1);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC3 = new osg::DrawElementsUInt(GL_LINE_STRIP, numSegments + 1);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC4 = new osg::DrawElementsUInt(GL_LINE_LOOP, 4);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC5 = new osg::DrawElementsUInt(GL_LINE_LOOP, 4);
	osg::ref_ptr<osg::DrawElementsUInt> indicesC6 = new osg::DrawElementsUInt(GL_LINES, 8);

	size_t i;
	unsigned int idx = 0;
	unsigned int idxC = 0;

	for (i = 0; i < numSegments + 1; ++i, angle += angleDelta)
	{
		float x = cosf(angle);
		float y = sinf(angle);

		(*vertices)[i * 4 + 0].set(sensor_->near_ * x, sensor_->near_ * y, -sensor_->near_ * fovV_rate); // near bottom
		(*vertices)[i * 4 + 3].set(sensor_->far_ * x, sensor_->far_ * y, -sensor_->far_ * fovV_rate); // far bottom
		(*vertices)[i * 4 + 2].set(sensor_->far_ * x, sensor_->far_ * y, sensor_->far_ * fovV_rate); // far upper
		(*vertices)[i * 4 + 1].set(sensor_->near_ * x, sensor_->near_ * y, sensor_->near_ * fovV_rate); // near upper

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
	(*vertices)[4 * (numSegments + 1)].set(0, 0, 0);

	(*indicesC4)[0] = (*indices)[idx++] = 0;
	(*indicesC4)[1] = (*indices)[idx++] = 3;
	(*indicesC4)[2] = (*indices)[idx++] = 2;
	(*indicesC4)[3] = (*indices)[idx++] = 1;

	(*indicesC5)[0] = (*indices)[idx++] = (i - 1) * 4 + 0;
	(*indicesC5)[1] = (*indices)[idx++] = (i - 1) * 4 + 1;
	(*indicesC5)[2] = (*indices)[idx++] = (i - 1) * 4 + 2;
	(*indicesC5)[3] = (*indices)[idx++] = (i - 1) * 4 + 3;

	// Drawing the lines from sensor mounting position to the near plane to indicate the near blind spot of the sensor
	(*indicesC6)[0] = 4 * (numSegments + 1);
	(*indicesC6)[1] = 0;
	(*indicesC6)[2] = 4 * (numSegments + 1);
	(*indicesC6)[3] = 1;
	(*indicesC6)[4] = 4 * (numSegments + 1);
	(*indicesC6)[5] = 4 * numSegments;
	(*indicesC6)[6] = 4 * (numSegments + 1);
	(*indicesC6)[7] = 4 * numSegments + 1;

	// Geometry -> Drawing transparent segments in FOV (between near plane and far plane)
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setDataVariance(osg::Object::STATIC);
	geom->setUseDisplayList(true);
	geom->setUseVertexBufferObjects(true);
	geom->setVertexArray(vertices.get());
	geom->addPrimitiveSet(indices.get());
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
	osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
	polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	stateset->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
	txNode_->addChild(geode);

	// Geometry2 -> Drawing solid lines representing the boundary of each volume of FOV
	osg::ref_ptr<osg::Geometry> geom2 = new osg::Geometry;
	osg::ref_ptr<osg::Vec4Array> FOV_color = new osg::Vec4Array(1);
	(*FOV_color)[0].set(color_green[0], color_green[1], color_green[2], 1.0);
	geom2->setUseDisplayList(true);
	geom2->setUseVertexBufferObjects(true);
	geom2->setVertexArray(vertices.get());
	geom2->addPrimitiveSet(indicesC0.get());
	geom2->addPrimitiveSet(indicesC1.get());
	geom2->addPrimitiveSet(indicesC2.get());
	geom2->addPrimitiveSet(indicesC3.get());
	geom2->addPrimitiveSet(indicesC4.get());
	geom2->addPrimitiveSet(indicesC5.get());
	geom2->setColorArray(FOV_color.get());
	geom2->setColorBinding(osg::Geometry::BIND_OVERALL);
	osgUtil::SmoothingVisitor::smooth(*geom2, 0.0);
	osg::ref_ptr<osg::Geode> geode2 = new osg::Geode;
	geom2->getOrCreateStateSet()->setAttribute(new osg::LineWidth(2.0f));
	geom2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geode2->addDrawable(geom2.release());
	txNode_->addChild(geode2);

	// Geometry3 -> Drawing solid lines representing the boundary of blind spot area between sensor mounting position and near FOV plane
	osg::ref_ptr<osg::Geometry> geom3 = new osg::Geometry;
	osg::ref_ptr<osg::Vec4Array> blind_spot_color = new osg::Vec4Array(1);
	(*blind_spot_color)[0].set(color_red[0], color_red[1], color_red[2], 1.0);
	geom3->setUseDisplayList(true);
	geom3->setUseVertexBufferObjects(true);
	geom3->setVertexArray(vertices.get());
	geom3->addPrimitiveSet(indicesC6.get());
	geom3->setColorArray(blind_spot_color.get());
	geom3->setColorBinding(osg::Geometry::BIND_OVERALL);
	osgUtil::SmoothingVisitor::smooth(*geom3, 0.0);
	osg::ref_ptr<osg::Geode> geode3 = new osg::Geode;
	geom3->getOrCreateStateSet()->setAttribute(new osg::LineWidth(2.0f));
	geom3->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	geode3->addDrawable(geom3.release());
	txNode_->addChild(geode3);

	// Geometry4 -> Drawing a point on desired sensor mounting position
	osg::ref_ptr<osg::Geometry> geom4 = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> mounting_vertice = new osg::Vec3Array(1);
	osg::ref_ptr<osg::Vec4Array> mounting_color = new osg::Vec4Array(1);
	osg::ref_ptr<osg::Point> mount_point = new osg::Point();
	mount_point->setSize(8.0f);
	(*mounting_color)[0].set(color_red[0], color_red[1], color_red[2], 1.0);
	(*mounting_vertice)[0].set(0, 0, 0);
	geom4->setUseDisplayList(true);
	geom4->setUseVertexBufferObjects(true);
	geom4->setVertexArray(mounting_vertice.get());
	osg::ref_ptr<osg::DrawElementsUInt> point_element = new osg::DrawElementsUInt(GL_POINTS, 1);
	geom4->addPrimitiveSet(point_element.get());
	geom4->setColorArray(mounting_color.get());
	geom4->setColorBinding(osg::Geometry::BIND_OVERALL);
	osgUtil::SmoothingVisitor::smooth(*geom4, 0.0);
	osg::ref_ptr<osg::Geode> geode4 = new osg::Geode;
	geom4->getOrCreateStateSet()->setAttribute(mount_point);
	geom4->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	txNode_->addChild(geom4);
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

OSISensorDetection::OSISensorDetection(osg::ref_ptr<osg::Group> parent)
{
	parent_ = parent;

	detected_points_group_ = new osg::Group;
	detected_points_group_->setDataVariance(osg::Object::DYNAMIC);
	parent->addChild(detected_points_group_);
	detected_bb_group_ = new osg::Group;
	detected_bb_group_->setDataVariance(osg::Object::DYNAMIC);
	parent->addChild(detected_bb_group_);
}

OSISensorDetection::~OSISensorDetection()
{
	for (auto point : detected_points_)
	{
		delete point.second;
	}
	detected_points_.clear();

	for (auto car : detected_cars_)
	{
		delete car.second;
	}
	detected_cars_.clear();

	parent_->removeChild(detected_points_group_);
	parent_->removeChild(detected_bb_group_);
}

void OSISensorDetection::Update(osi3::SensorView* sv)
{
	//lets find the detected objects that are still under the radar FOV
	if (sv)
	{
		bool found = false;
		for (auto point : detected_points_)
		{
			for (size_t i = 0; i < sv->global_ground_truth().lane_boundary_size(); ++i)
			{
				for (size_t j = 0; j < sv->global_ground_truth().lane_boundary()[i].boundary_line_size(); ++j)
				{
					//lets see the lane boundary ID and check if detected points map are in the sensor view
					std::string str_id = std::to_string(sv->global_ground_truth().lane_boundary()[i].id().value()) + std::to_string(j);
					uint64_t id = std::stoul(str_id);
					if (point.first == id)
					{
						found = true;
						break;
					}
					if (found)
					{
						break;
					}
				}
			}

			//If the point isn't in the sensor view then we hide it
			if (!found)
			{
				point.second->Hide();
			}
			// If the point was detected before, we show it again
			else if (found && !point.second->showing_)
			{
				point.second->Show();
			}
			found = false;
		}

		found = false;
		for (auto car : detected_cars_)
		{
			for (size_t i = 0; i < sv->global_ground_truth().moving_object_size(); i++)
			{
				//lets see the moving object ID and check if detected cars map are in the sensor view
				uint64_t id = sv->global_ground_truth().moving_object()[i].id().value();
				if (car.first == id)
				{
					found = true;
					break;
				}
			}

			//If the moving object isn't in the sensor view then we hide it
			if (!found)
			{
				car.second->Hide();
			}
			// If the moving object was detected before, we show it again
			else if (found && !car.second->showing_)
			{
				car.second->Show();
			}
			found = false;
		}

		// Lets check if it is needed to create new points and moving objects visuals
		double z_offset = 0.10;
		if (sv->has_global_ground_truth())
		{
			for (size_t i = 0; i < sv->global_ground_truth().moving_object_size(); i++)
			{
				//Get moving object position and dimension
				const osi3::Vector3d moving_object_position = sv->global_ground_truth().moving_object()[i].base().position();
				const osi3::Dimension3d moving_object_dimension = sv->global_ground_truth().moving_object()[i].base().dimension();

				//Get moving object id
				uint64_t id = sv->global_ground_truth().moving_object()[i].id().value();

				// If the moving object ID isn't in the cars map then we create one and added to the map
				if (detected_cars_.count(id) == 0)
				{
					detected_cars_.emplace(id, new OSIDetectedCar(osg::Vec3(moving_object_position.x(), moving_object_position.y(), moving_object_position.z() + z_offset), moving_object_dimension.height() + 1.0, moving_object_dimension.width() + 1.0, moving_object_dimension.length() + 1.0, detected_bb_group_));
				}
				else
				{
					//Otherwise update the visual object
					osg::Vec3 bb_dimension = detected_cars_.find(id)->second->bb_dimensions_;
					detected_cars_.find(id)->second->Update(osg::Vec3(moving_object_position.x(), moving_object_position.y(), moving_object_position.z() + bb_dimension.z() + z_offset));
				}
			}

			for (size_t i = 0; i < sv->global_ground_truth().lane_boundary_size(); ++i)
			{
				for (size_t j = 0; j < sv->global_ground_truth().lane_boundary()[i].boundary_line_size(); ++j)
				{
					//Get line boundary id
					std::string str_id = std::to_string(sv->global_ground_truth().lane_boundary()[i].id().value()) + std::to_string(j);
					uint64_t id = std::stoul(str_id);

					//Get line boundary position
					const osi3::Vector3d boundary_line_position = sv->global_ground_truth().lane_boundary()[i].boundary_line()[j].position();

					// If the lane boundary ID isn't in the points map then we create one and added to the map
					if (detected_points_.count(id) == 0)
					{
						detected_points_.emplace(id, new OSIDetectedPoint(osg::Vec3(boundary_line_position.x(), boundary_line_position.y(), boundary_line_position.z() + z_offset), detected_points_group_));
					}
					else
					{
						//Otherwise update the visual object
						detected_points_.find(id)->second->Update(osg::Vec3(boundary_line_position.x(), boundary_line_position.y(), boundary_line_position.z() + z_offset));
					}
				}
			}
		}
	}
}

OSIDetectedPoint::OSIDetectedPoint(const osg::Vec3 point, osg::ref_ptr<osg::Group> parent)
{
	parent_ = parent;
	osi_detection_geom_ = new osg::Geometry;
	osi_detection_points_ = new osg::Vec3Array;
	osi_detection_color_ = new osg::Vec4Array;
	osg::ref_ptr<osg::Point> osi_detection_point = new osg::Point();

	// start point of each road mark
	osi_detection_points_->push_back(point);
	osi_detection_color_->push_back(osg::Vec4(color_green[0], color_green[1], color_green[2], 1.0));

	osi_detection_point->setSize(8.0f);
	osi_detection_geom_->getOrCreateStateSet()->setAttributeAndModes(osi_detection_point, osg::StateAttribute::ON);
	osi_detection_geom_->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	osi_detection_geom_->getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);

	osi_detection_geom_->setVertexArray(osi_detection_points_.get());
	osi_detection_geom_->setColorArray(osi_detection_color_.get());
	osi_detection_geom_->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	osi_detection_geom_->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, osi_detection_points_->size()));
	osi_detection_geom_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
	showing_ = true;

	parent_->addChild(osi_detection_geom_);
}

void OSIDetectedPoint::Update(const osg::Vec3 point)
{
	osi_detection_points_->clear();
	osi_detection_points_->push_back(point);
	osi_detection_points_->dirty();
	osi_detection_geom_->dirtyGLObjects();
	osi_detection_geom_->dirtyBound();
}

OSIDetectedPoint::~OSIDetectedPoint()
{
	parent_->removeChild(osi_detection_geom_);
}

OSIDetectedCar::OSIDetectedCar(const osg::Vec3 point, double h, double w, double l, osg::ref_ptr<osg::Group> parent)
{
	parent_ = parent;
	bb_dimensions_.set(l, w, h);

	car_ = new osg::Group;
	osi_detection_geode_box_ = new osg::Geode;
	osi_detection_geode_box_->addDrawable(new osg::ShapeDrawable(new osg::Box()));
	osi_detection_geode_box_->getDrawable(0)->setDataVariance(osg::Object::DataVariance::DYNAMIC);

	osi_detection_tx_ = new osg::PositionAttitudeTransform;

	osg::Material* material = new osg::Material();

	// Set color of vehicle based on its index
	double* color = color_green;
	double b = 1.5;  // brighness

	material->setDiffuse(osg::Material::FRONT, osg::Vec4(b * color[0], b * color[1], b * color[2], 1.0));
	material->setAmbient(osg::Material::FRONT, osg::Vec4(b * color[0], b * color[1], b * color[2], 1.0));

	// Set dimensions of the entity "box"
	osi_detection_tx_->setScale(bb_dimensions_);
	osi_detection_tx_->setPosition(osg::Vec3d(point.x(), point.y(), point.z()));

	// Draw only wireframe
	osg::PolygonMode* polygonMode = new osg::PolygonMode;
	polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	osg::ref_ptr<osg::StateSet> stateset = osi_detection_geode_box_->getOrCreateStateSet(); // Get the StateSet of the group
	stateset->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
	stateset->setMode(GL_COLOR_MATERIAL, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	stateset->setDataVariance(osg::Object::DYNAMIC);
	osi_detection_geode_box_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);

	osi_detection_geode_center_ = new osg::Geode;
	osi_detection_geode_center_->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), 0.2)));
	osi_detection_geode_center_->getDrawable(0)->setDataVariance(osg::Object::DYNAMIC);

	osi_detection_geode_center_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
	car_->addChild(osi_detection_geode_center_);

	osi_detection_tx_->addChild(osi_detection_geode_box_);
	osi_detection_tx_->getOrCreateStateSet()->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
	osi_detection_tx_->getOrCreateStateSet()->setAttribute(material);
	osi_detection_tx_->getOrCreateStateSet()->setDataVariance(osg::Object::DYNAMIC);
	car_->setName("BoundingBox");
	car_->addChild(osi_detection_tx_);

	parent_->addChild(car_);
}

OSIDetectedCar::~OSIDetectedCar()
{
	parent_->removeChild(car_);
}

void OSIDetectedCar::Update(const osg::Vec3 point)
{
	osi_detection_tx_->setPosition(point);
	osi_detection_tx_->dirtyBound();
	osi_detection_geode_center_->dirtyBound();
	osi_detection_geode_box_->dirtyBound();
}

void OSIDetectedCar::Show()
{
	car_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
	osi_detection_geode_box_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
	osi_detection_geode_center_->setNodeMask(NodeMask::NODE_MASK_OBJECT_SENSORS);
	showing_ = true;
};

void OSIDetectedCar::Hide()
{
	car_->setNodeMask(0x0);
	osi_detection_geode_box_->setNodeMask(0x0);
	osi_detection_geode_center_->setNodeMask(0x0);
	showing_ = false;
};

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


RouteWayPoints::RouteWayPoints(osg::ref_ptr<osg::Group> parent, osg::Vec4 color) : parent_(parent)
{
	group_ = new osg::Group;
	group_->setNodeMask(NodeMask::NODE_MASK_ROUTE_WAYPOINTS);

	// Finally attach a simple material for shading properties
	osg::ref_ptr<osg::Material> material_ = new osg::Material;
	material_->setDiffuse(osg::Material::FRONT_AND_BACK, color);
	material_->setAmbient(osg::Material::FRONT_AND_BACK, color);
	group_->getOrCreateStateSet()->setAttributeAndModes(material_.get());

	parent->addChild(group_);
}

RouteWayPoints::~RouteWayPoints()
{
	group_->removeChildren(0, group_->getNumChildren());
}

osg::ref_ptr<osg::Geode> RouteWayPoints::CreateWayPointGeometry(double x, double y, double z, double h, double scale)
{
	osg::ref_ptr<osg::Node> node = new osg::Node;

	// Create geometry for route waypoint triangle
	double size = 1.8 * scale;

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(8);
	osg::ref_ptr<osg::DrawElementsUInt> indices_side = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, 10);
	osg::ref_ptr<osg::DrawElementsUInt> indices_roof = new osg::DrawElementsUInt(GL_TRIANGLE_FAN, 4);

	// Calculate vertices of a rotated arrow
	double p[2] = { 0.0, 0.0 };
	// Bottom contour
	(*vertices)[0].set(x, y, z);
	RotateVec2D(size * -0.25, size * -0.5, h, p[0], p[1]);
	(*vertices)[1].set(x + p[0], y + p[1], z);
	RotateVec2D(size * 1.0, 0, h, p[0], p[1]);
	(*vertices)[2].set(x + p[0], y + p[1], z);
	RotateVec2D(size * -0.25, size * 0.5, h, p[0], p[1]);
	(*vertices)[3].set(x + p[0], y + p[1], z);

	// Top contour (copy bottom to z offset)
	(*vertices)[4].set((*vertices)[0][0], (*vertices)[0][1], z + WAYPOINT_HEIGHT);
	(*vertices)[5].set((*vertices)[1][0], (*vertices)[1][1], z + WAYPOINT_HEIGHT);
	(*vertices)[6].set((*vertices)[2][0], (*vertices)[2][1], z + WAYPOINT_HEIGHT);
	(*vertices)[7].set((*vertices)[3][0], (*vertices)[3][1], z + WAYPOINT_HEIGHT);

	// Indices, zigzag between top and bottom
	for (int i = 0; i < 4; i++)
	{
		(*indices_side)[i * 2] = 4 + i;
		(*indices_side)[i * 2 + 1] = i;
	}
	// close to first vertices
	(*indices_side)[8] = 4;
	(*indices_side)[9] = 0;

	// Indices of the "roof" of the arrow
	(*indices_roof)[0] = 4;
	(*indices_roof)[1] = 5;
	(*indices_roof)[2] = 6;
	(*indices_roof)[3] = 7;

	// Finally create and add geometry
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geom[] = { new osg::Geometry, new osg::Geometry };
	geom[0]->setVertexArray(vertices.get());
	geom[0]->addPrimitiveSet(indices_side.get());
	geom[1]->setVertexArray(vertices.get());  // reuse vertex list from sides
	geom[1]->addPrimitiveSet(indices_roof.get());

	osg::ref_ptr<osg::Vec4Array> colorarray = new osg::Vec4Array;

	for (int i = 0; i < 2; i++)
	{
		geom[i]->setDataVariance(osg::Object::STATIC);
		geom[i]->setUseDisplayList(true);
		osgUtil::SmoothingVisitor::smooth(*geom[i], 0.0);  // calculate normals
		geom[i]->setDataVariance(osg::Object::STATIC);
		geode->addDrawable(geom[i]);
	}

	return geode;
}

void RouteWayPoints::SetWayPoints(roadmanager::Route* route)
{
	if (route == nullptr)
	{
		return;
	}

	group_->removeChildren(0, group_->getNumChildren());

	// First put all waypoints on ground
	for (int i = 0; i < (int)route->all_waypoints_.size(); i++)
	{
		group_->addChild(CreateWayPointGeometry(
			route->all_waypoints_[i].GetX(),
			route->all_waypoints_[i].GetY(),
			route->all_waypoints_[i].GetZ(),
			route->all_waypoints_[i].GetH(), 1.0));
	}

#if 0  // Keeping code below in case need arise to visualize minimal wp set on top
	// Then indicate minimal set with a smaller waypoint symbol on top
	for (int i = 0; i < (int)route->minimal_waypoints_.size(); i++)
	{
		group_->addChild(CreateWayPointGeometry(
			route->minimal_waypoints_[i].GetX(),
			route->minimal_waypoints_[i].GetY(),
			route->minimal_waypoints_[i].GetZ() + WAYPOINT_HEIGHT,
			route->minimal_waypoints_[i].GetH(), 0.5));
	}
#endif
}

osg::ref_ptr<osg::PositionAttitudeTransform> CarModel::AddWheel(osg::ref_ptr<osg::Node> carNode, const char* wheelName)
{
	osg::ref_ptr<osg::PositionAttitudeTransform> tx_node = 0;

	// Find wheel node
	std::vector<osg::Node*> nodes;
	FindNamedNodes fnn(wheelName, nodes);
	carNode->accept(fnn);

	for (size_t i = 0; i < nodes.size(); i++)
	{
		// Assume wheel is a tranformation node
		osg::MatrixTransform* node = dynamic_cast<osg::MatrixTransform*>(nodes[i]);
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

			if (std::string(wheelName).find("wheel_r") != std::string::npos)
			{
				rear_wheel_.push_back(tx_node);
			}
			else if (std::string(wheelName).find("wheel_f") != std::string::npos)
			{
				front_wheel_.push_back(tx_node);
			}
		}
	}

	return tx_node;
}

EntityModel::EntityModel(osgViewer::Viewer* viewer, osg::ref_ptr<osg::Group> group, osg::ref_ptr<osg::Group> parent,
	osg::ref_ptr<osg::Group> trail_parent, osg::ref_ptr<osg::Group> traj_parent, osg::ref_ptr<osg::Node> dot_node,
	osg::ref_ptr<osg::Group> route_waypoint_parent, osg::Vec4 trail_color, std::string name)
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

	// Add trajectory placeholder
	trajectory_ = new Trajectory(traj_parent, viewer);

	viewer_ = viewer;
	state_set_ = 0;
	blend_color_ = 0;

	// Prepare trail of dots
	trail_ = new PolyLine(trail_parent, 0, trail_color, TRAIL_WIDTH, TRAIL_DOT3D_SIZE, true);
	trail_->SetNodeMaskLines(NodeMask::NODE_MASK_TRAIL_LINES);
	trail_->SetNodeMaskDots(NodeMask::NODE_MASK_TRAIL_DOTS);

	routewaypoints_ = new RouteWayPoints(route_waypoint_parent, trail_color);
}

EntityModel::~EntityModel()
{
	parent_->removeChild(txNode_);
}


CarModel::CarModel(osgViewer::Viewer* viewer, osg::ref_ptr<osg::Group> group, osg::ref_ptr<osg::Group> parent,
	osg::ref_ptr<osg::Group> trail_parent, osg::ref_ptr<osg::Group> traj_parent, osg::ref_ptr<osg::Node> dot_node,
	osg::ref_ptr<osg::Group> route_waypoint_parent, osg::Vec4 trail_color, std::string name) :
	EntityModel(viewer, group, parent, trail_parent, traj_parent, dot_node, route_waypoint_parent, trail_color, name)
{
	steering_sensor_ = 0;
	road_sensor_ = 0;
	route_sensor_ = 0;
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
			LOG_ONCE("Missing wheel node %s in vehicle model %s - ignoring", "wheel_fl", car_node->getName().c_str());
		}
		if (!retval[1])
		{
			LOG_ONCE("Missing wheel node %s in vehicle model %s - ignoring", "wheel_fr", car_node->getName().c_str());
		}
		if (!retval[2])
		{
			LOG_ONCE("Missing wheel node %s in vehicle model %s - ignoring", "wheel_rr", car_node->getName().c_str());
		}
		if (!retval[3])
		{
			LOG_ONCE("Missing wheel node %s in vehicle model %s - ignoring", "wheel_rl", car_node->getName().c_str());
		}
	}
}

CarModel ::~CarModel()
{
	front_wheel_.clear();
	rear_wheel_.clear();
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
	txNode_->setAttitude(quat_ * quatTmp);
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

	for (size_t i = 0; i < front_wheel_.size(); i++)
	{
		quat.makeRotate(
			0, osg::Vec3(1, 0, 0), // Roll
			wheel_rotation, osg::Vec3(0, 1, 0), // Pitch
			wheel_angle, osg::Vec3(0, 0, 1)); // Heading
		front_wheel_[i]->setAttitude(quat);
	}

	for (size_t i = 0; i < rear_wheel_.size(); i++)
	{
		// Update rotation for rear wheels
		quat.makeRotate(
			0, osg::Vec3(1, 0, 0), // Roll
			wheel_rotation, osg::Vec3(0, 1, 0), // Pitch
			0, osg::Vec3(0, 0, 1)); // Heading
		rear_wheel_[i]->setAttitude(quat);
	}
}

void CarModel::UpdateWheelsDelta(double wheel_angle, double wheel_rotation_delta)
{
	UpdateWheels(wheel_angle, wheel_rot_ + wheel_rotation_delta);
}

void CarModel::ShowRouteSensor(bool mode)
{
	if (mode == true)
	{
		route_sensor_->group_->setNodeMask(lane_sensor_->group_->getNodeMask());
	}
	else
	{
		route_sensor_->group_->setNodeMask(0x0);
	}
}

void EntityModel::SetTransparency(double factor)
{
	if (factor < 0 || factor > 1)
	{
		LOG("Clamping transparency factor %.2f to [0:1]", factor);
		factor = CLAMP(factor, 0, 1);
	}
	blend_color_->setConstantColor(osg::Vec4(1, 1, 1, 1 - factor));
}

struct FetchImage : public osg::Camera::DrawCallback
{
	FetchImage(Viewer* viewer)
	{
		image_ = new osg::Image;
		viewer_ = viewer;
		viewer_->capturedImage_ = { 0, 0, 0, 0, 0 };
	}

	virtual void operator() (osg::RenderInfo& renderInfo) const
	{
		if (viewer_ != nullptr &&
			SE_Env::Inst().GetOffScreenRendering() &&
			!viewer_->GetQuitRequest() &&
			(viewer_->GetSaveImagesToRAM() ||
				viewer_->frameCounter_ == 0 ||
				viewer_->GetSaveImagesToFile() != 0 ||
				viewer_->imgCallback_.func != nullptr))
		{
			osg::Camera* camera = renderInfo.getCurrentCamera();
			osg::Viewport* viewport = camera ? camera->getViewport() : 0;

			if (viewport && image_.valid())
			{
				viewer_->imageMutex.Lock();

				image_->readPixels(int(viewport->x()), int(viewport->y()), int(viewport->width()), int(viewport->height()),
					GL_BGR,  // only GL_RGB and GL_BGR supported for now
					GL_UNSIGNED_BYTE);

				if (image_->getPixelFormat() == GL_RGB || image_->getPixelFormat() == GL_BGR)
				{
					viewer_->capturedImage_.width = image_->s();
					viewer_->capturedImage_.height = image_->t();
					viewer_->capturedImage_.pixelSize = 3;
					viewer_->capturedImage_.pixelFormat = static_cast<int>(image_->getPixelFormat());
					viewer_->capturedImage_.data = image_->data();

					if (viewer_->GetSaveImagesToFile() != 0)
					{
						static char filename[64];
						snprintf(filename, 64, "screen_shot_%05d.tga", viewer_->captureCounter_);
						SE_WriteTGA(filename, viewer_->capturedImage_.width, viewer_->capturedImage_.height, viewer_->capturedImage_.data,
							viewer_->capturedImage_.pixelSize, viewer_->capturedImage_.pixelFormat, true);
						viewer_->captureCounter_++;

						// If not continuous (-1), decrement frame counter
						if (viewer_->GetSaveImagesToFile() > 0)
						{
							viewer_->SaveImagesToFile(viewer_->GetSaveImagesToFile() - 1);
						}
					}
				}
				else
				{
					printf("Unsupported pixel format 0x%x\n", image_->getPixelFormat());
					viewer_->capturedImage_ = { 0, 0, 0, 0, 0 };  // Reset image data
				}

				if (viewer_->imgCallback_.func != nullptr)
				{
					viewer_->imgCallback_.func(&viewer_->capturedImage_, viewer_->imgCallback_.data);
				}

				viewer_->imageMutex.Unlock();
			}
		}
		else
		{
			viewer_->capturedImage_.data = nullptr;
		}

		viewer_->frameCounter_++;

		viewer_->renderSemaphore.Release(); // Lower flag to indicate rendering done
	}

	mutable osg::ref_ptr<osg::Image> image_;
	viewer::Viewer* viewer_;
};

int Viewer::InitTraits(osg::ref_ptr<osg::GraphicsContext::Traits> traits,
	int x, int y, int w, int h, int samples, bool decoration, int screenNum, bool headless)
{
	traits->x = x;
	traits->y = y;
	traits->width = w;
	traits->height = h;
	traits->samples = samples;
	traits->sampleBuffers = (traits->samples > 0) ? 1 : 0;
	traits->sharedContext = 0;
	traits->readDISPLAY();
	traits->screenNum = screenNum;
	traits->setUndefinedScreenDetailsToDefaultScreen();

	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	unsigned int width, height;
	wsi->getScreenResolution(traits->displayNum, width, height);
	if (traits->width <= 0) traits->width = width;
	if (traits->height <= 0) traits->height = height;

	if (headless)
	{
		// Setup off-screen rendering only, no window on screen
		traits->windowDecoration = false;
		traits->doubleBuffer = false;
		traits->pbuffer = true;
	}
	else
	{
		traits->windowDecoration = decoration;
		traits->doubleBuffer = true;
	}

	return 0;
}

int Viewer::AddGroundSurface()
{
	const double margin = 1E4;
	const double z_offset = -1.0;
	const osg::BoundingSphere bs = environment_->getBound();

	osg::ComputeBoundsVisitor cbv;
	environment_->accept(cbv);
	osg::BoundingBox bb = cbv.getBoundingBox();

	osg::ref_ptr<osg::Geode> ground = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geom = osg::createTexturedQuadGeometry(
		osg::Vec3(bb.xMin() - margin, bb.yMin() - margin, bb.zMin() + z_offset),
		osg::Vec3(0, 2 * margin + (bb.yMax() - bb.yMin()), bb.zMin() + z_offset),
		osg::Vec3(2 * margin + (bb.xMax() - bb.xMin()), 0, bb.zMin() + z_offset));
	osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
	color->push_back(osg::Vec4(0.8, 0.8, 0.8, 1.0));
	geom->setColorArray(color.get());
	geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
	ground->addDrawable(geom);

	envTx_->addChild(ground.get());

	return 0;
}

Viewer::Viewer(roadmanager::OpenDrive* odrManager, const char* modelFilename, const char* scenarioFilename, const char* exe_path, osg::ArgumentParser arguments, SE_Options* opt)
{
	odrManager_ = odrManager;
	bool clear_color = false;
	std::string arg_str;

	if (scenarioFilename != NULL && strcmp(scenarioFilename, ""))
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
	captureCounter_ = 0;
	frameCounter_ = 0;
	saveImagesToRAM_ = false;  // Default is to read back rendered image for possible fetch via API
	saveImagesToFile_ = 0;
	imgCallback_ = { nullptr, nullptr };
	winDim_ = { -1, -1, -1, -1 };
	bool decoration = true;
	int screenNum = -1;

	int aa_mode = DEFAULT_AA_MULTISAMPLES;
	if (opt && (arg_str = opt->GetOptionArg("aa_mode")) != "")
	{
		aa_mode = atoi(arg_str.c_str());
	}

	while (arguments.read("--window", winDim_.x, winDim_.y, winDim_.w, winDim_.h)) {}

	while (arguments.read("--borderless-window", winDim_.x, winDim_.y, winDim_.w, winDim_.h))
	{
		decoration = false;
	}

	while (arguments.read("--screen", screenNum)) {}

	arguments.getApplicationUsage()->addCommandLineOption("--lodScale <number>", "LOD Scale");
	arguments.read("--lodScale", lodScale_);

	// set the scene to render
	rootnode_ = new osg::MatrixTransform;

	// Instantiate the window
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	InitTraits(traits, winDim_.x, winDim_.y, winDim_.w, winDim_.h, aa_mode, decoration, screenNum, opt->GetOptionSet("headless"));

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	if (!gc.valid())
	{
		// Viewer failed to create graphics context. Probably Anti Aliasing is not supported on executing platform.
		// Make another attempt without AA
		LOG("Viewer failure. Probably requested level of Anti Aliasing (%d multisamples) is not supported. Making another attempt without Anti-Alias and on first screen.", aa_mode);

		InitTraits(traits, winDim_.x, winDim_.y, winDim_.w, winDim_.h, 0, decoration, 0, opt->GetOptionSet("headless"));

		gc = osg::GraphicsContext::createGraphicsContext(traits.get());

		if (!gc.valid())
		{
			LOG("Failed 2nd attempt to create viewer, giving up. Try --headless option to run without viewer");
			return;
		}
	}

	osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
	if (!opt->GetOptionSet("headless") && gw == nullptr)
	{
		LOG("Failed to create viewer window. Try --headless option to run without window");
		return;
	}

	osgViewer_ = new osgViewer::Viewer;
	if (osgViewer_ == nullptr)
	{
		LOG("Failed to initialize OSG viewer");
		return;
	}

	osg::ref_ptr<osg::Camera> camera = osgViewer_->getCamera();

	camera->setGraphicsContext(gc);
	camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	if (!clear_color)
	{
		// Default background color
		camera->setClearColor(osg::Vec4(0.5f, 0.75f, 1.0f, 1.0f));
	}
	else
	{
		camera->setClearColor(osg::Vec4(0.5f, 0.75f, 1.0f, 1.0f));
	}
	camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	camera->setProjectionMatrixAsPerspective(30.0f, (double)traits->width / (double)traits->height, 0.5, 1E5);
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	camera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	camera->setLODScale(lodScale_);

	osgViewer_->setCamera(camera.get());

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
	ClearNodeMaskBits(NodeMask::NODE_MASK_INFO_PER_OBJ);
	SetNodeMaskBits(NodeMask::NODE_MASK_ENTITY_MODEL);
	SetNodeMaskBits(NodeMask::NODE_MASK_INFO);
	SetNodeMaskBits(NodeMask::NODE_MASK_TRAJECTORY_LINES);
	SetNodeMaskBits(NodeMask::NODE_MASK_ROUTE_WAYPOINTS);

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
	routewaypoints_ = new osg::Group;
	routewaypoints_->setNodeMask(NodeMask::NODE_MASK_ROUTE_WAYPOINTS);
	rootnode_->addChild(routewaypoints_);

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
					LOG("Loaded scenegraph: %s", file_name_candidates[i].c_str());
					break;
				}
			}
		}

		if (i == file_name_candidates.size())
		{
			LOG("Failed to read environment model %s!", modelFilename);
		}
	}

	if (environment_ == 0 || opt->GetOptionSet("enforce_generate_model"))
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

	// Add an optional large ground surface
	if (opt->GetOptionSet("ground_plane"))
	{
		AddGroundSurface();
	}

	//const osg::BoundingSphere bs = environment_->getBound();
	//printf("bs radius %.2f\n", bs.radius());
	if (odrManager->GetNumOfRoads() > 0 && !CreateRoadLines(odrManager))
	{
		LOG("Viewer::Viewer Failed to create road lines!");
	}

	if (odrManager->GetNumOfRoads() > 0 && !CreateRoadMarkLines(odrManager))
	{
		LOG("Viewer::Viewer Failed to create road mark lines!");
	}

	if (!(opt && opt->GetOptionSet("generate_no_road_objects")))
	{
		if (odrManager->GetNumOfRoads() > 0 && CreateRoadSignsAndObjects(odrManager) != 0)
		{
			LOG("Viewer::Viewer Failed to create road signs and objects!");
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

	osgViewer_->setSceneData(rootnode_);

	osgViewer_->addEventHandler(new ViewerEventHandler(this));

	// add the window size toggle handler
	osgViewer_->addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler
	osgViewer_->addEventHandler(new osgViewer::StatsHandler);

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

	osgViewer_->setReleaseContextAtEndOfFrameHint(false);

	// Handle arguments
	// Copy argument handling from osgViewer. Unfortunately we can't use osgViewer(arguments) constructor
	// since it will create the window without checking compatibility, e.g. antialias support
	while (arguments.read("--SingleThreaded")) osgViewer_->setThreadingModel(osgViewer::ViewerBase::ThreadingModel::SingleThreaded);
	while (arguments.read("--CullDrawThreadPerContext")) osgViewer_->setThreadingModel(osgViewer::ViewerBase::ThreadingModel::CullDrawThreadPerContext);
	while (arguments.read("--DrawThreadPerContext")) osgViewer_->setThreadingModel(osgViewer::ViewerBase::ThreadingModel::DrawThreadPerContext);
	while (arguments.read("--CullThreadPerCameraDrawThreadPerContext")) osgViewer_->setThreadingModel(osgViewer::ViewerBase::ThreadingModel::CullThreadPerCameraDrawThreadPerContext);

	std::string colorStr;
	bool clearColorSet = false;
	if (arguments.read("--clear-color", colorStr))
	{
		float r, g, b;
		float a = 1.0f;
		int cnt = sscanf(colorStr.c_str(), "%f,%f,%f,%f", &r, &g, &b, &a);
		if (cnt == 3 || cnt == 4)
		{
			camera->setClearColor(osg::Vec4(r, g, b, a));
			clearColorSet = true;
		}
		else
		{
			LOG("Invalid clear color \"%s\" - setting some default", colorStr.c_str());
		}
	}

	if (!clearColorSet)
	{
		camera->setClearColor(osg::Vec4(0.5f, 0.75f, 1.0f, 1.0f));
		clearColorSet = true;
	}

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

	// add the state manipulator
	osgViewer_->addEventHandler(new osgGA::StateSetManipulator(camera->getOrCreateStateSet()));

	// Light
	osgViewer_->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
	osg::Light* light = osgViewer_->getLight();
	light->setPosition(osg::Vec4(-7500., 5000., 10000., 1.0));
	light->setDirection(osg::Vec3(7.5, -5., -10.));
	float ambient = 0.4;
	light->setAmbient(osg::Vec4(ambient, ambient, 0.9 * ambient, 1));
	light->setDiffuse(osg::Vec4(0.8, 0.8, 0.7, 1));

	// Overlay text
	osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
	infoText = new osgText::Text;
	infoText->setColor(osg::Vec4(0.9f, 0.9f, 0.9f, 1.0f));
	infoText->setCharacterSize(12.0f);
	infoText->setAxisAlignment(osgText::Text::SCREEN);
	infoText->setPosition(osg::Vec3(10, 10, 0));
	infoText->setDataVariance(osg::Object::DYNAMIC);
	infoText->setNodeMask(NodeMask::NODE_MASK_INFO);

	textGeode->addDrawable(infoText);

	// Some tricks from https://osg-users.openscenegraph.narkive.com/XFhfB4ug/problem-in-osgtext-text
	osg::StateSet* ptrTextState = textGeode->getOrCreateStateSet();
	ptrTextState->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED | osg::StateAttribute::OVERRIDE);
	ptrTextState->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED | osg::StateAttribute::OVERRIDE);
	ptrTextState->setRenderBinDetails(INT_MAX, "RenderBin", osg::StateSet::RenderBinMode::OVERRIDE_RENDERBIN_DETAILS);

	infoTextCamera = new osg::Camera;
	infoTextCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	infoTextCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	infoTextCamera->setRenderOrder(osg::Camera::POST_RENDER, 10);
	infoTextCamera->setAllowEventFocus(false);
	infoTextCamera->addChild(textGeode.get());
	infoTextCamera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	infoTextCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, traits->width, 0, traits->height));
	rootnode_->addChild(infoTextCamera);

	// Register callback for fetch rendered image into RAM buffer
	if (SE_Env::Inst().GetOffScreenRendering())
	{
		osgViewer_->getCamera()->setFinalDrawCallback(new FetchImage(this));
	}
	else
	{
		// add OSG default screen capture handler
		osgViewer_->addEventHandler(new osgViewer::ScreenCaptureHandler);
	}

	initialThreadingModel_ = osgViewer_->getThreadingModel();

	osgViewer_->realize();
}

Viewer::~Viewer()
{
	renderSemaphore.Wait();  //  wait for any ongoing rendering
	osgViewer_->setDone(true);  // flag OSG to tear down

	while (!osgViewer_->done() || osgViewer_->areThreadsRunning())
	{
		osgViewer_->stopThreading();
		SE_sleep(100);  // In case viewer still not closed
	}

	for (size_t i = 0; i < entities_.size(); i++)
	{
		delete(entities_[i]);
	}

	for (size_t i = 0; i < polyLine_.size(); i++)
	{
		delete(polyLine_[i]);
	}

	entities_.clear();
}

void Viewer::PrintUsage()
{
	// Inform about a few OSG options
	printf("Additional OSG graphics options:\n");
	printf("  --clear-color <color>                      Set the background color of the viewer in the form \"r,g,b[,a]\"\n");
	printf("  --screen <num>                             Set the screen to use when multiple screens are present\n");
	printf("  --window <x y w h>                         Set the position x, y and size w, h of the viewer window. -1 -1 -1 -1 for fullscreen.\n");
	printf("  --borderless-window <x y w h>	             Set the position x, y and size w, h of a borderless viewer window. -1 -1 -1 -1 for fullscreen.\n");
	printf("  --SingleThreaded                           Run application and all graphics tasks in one single thread.\n");
	printf("\n");
}

void Viewer::AddCustomCamera(double x, double y, double z, double h, double p, bool fixed_pos)
{
	osgGA::RubberbandManipulator::CustomCamera cam(osg::Vec3(x, y, z), osg::Vec3(h, p, 0.0), fixed_pos);
	rubberbandManipulator_->AddCustomCamera(cam);

	UpdateCameraFOV();
}

void Viewer::AddCustomCamera(double x, double y, double z, bool fixed_pos)
{
	osgGA::RubberbandManipulator::CustomCamera cam(osg::Vec3(x, y, z), fixed_pos);
	rubberbandManipulator_->AddCustomCamera(cam);

	UpdateCameraFOV();
}

void Viewer::AddCustomFixedTopCamera(double x, double y, double z, double rot)
{
	osgGA::RubberbandManipulator::CustomCamera cam(osg::Vec3(x, y, z), rot);
	rubberbandManipulator_->AddCustomCamera(cam);

	UpdateCameraFOV();
}

int Viewer::GetCameraPosAndRot(osg::Vec3 &pos, osg::Vec3 &rot)
{
	osg::Matrix m = osgViewer_->getCamera()->getInverseViewMatrix();
	osg::Quat quat = m.getRotate();
	pos = m.getTrans();

	double qx = quat.x();
	double qy = quat.y();
	double qz = quat.z();
	double qw = quat.w();

	double sqx = qx * qx;
	double sqy = qy * qy;
	double sqz = qz * qz;
	double sqw = qw * qw;

	double term1 = 2 * (qx * qy + qw * qz);
	double term2 = sqw + sqx - sqy - sqz;
	double term3 = -2 * (qx * qz - qw * qy);
	double term4 = 2 * (qw * qx + qy * qz);
	double term5 = sqw - sqx - sqy + sqz;

	rot[0] = GetAngleInInterval2PI(M_PI_2 + atan2(term1, term2));
	rot[1] = GetAngleInInterval2PI(M_PI_2 - atan2(term4, term5));
	rot[2] = asin(term3);

	return 0;
}

void Viewer::SetCameraMode(int mode)
{
	if (mode < 0 || mode >= GetNumberOfCameraModes())
	{
		// set to last camera mode
		mode = (int)(rubberbandManipulator_->GetNumberOfCameraModes() - 1);
	}

	camMode_ = mode;
	rubberbandManipulator_->setMode(camMode_);
	UpdateCameraFOV();
}

int Viewer::GetNumberOfCameraModes()
{
	return (int)rubberbandManipulator_->GetNumberOfCameraModes();
}

void Viewer::UpdateCameraFOV()
{
	double fov;
	osgGA::RubberbandManipulator::CustomCamera* custom_cam = rubberbandManipulator_->GetCurrentCustomCamera();

	if (camMode_ == osgGA::RubberbandManipulator::RB_MODE_TOP || custom_cam && custom_cam->GetOrtho())
	{
		fov = ORTHO_FOV;
	}
	else
	{
		fov = PERSP_FOV;
	}

	osg::ref_ptr<osg::GraphicsContext> gc = osgViewer_->getCamera()->getGraphicsContext();
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = const_cast<osg::GraphicsContext::Traits*>(osgViewer_->getCamera()->getGraphicsContext()->getTraits());

	osgViewer_->getCamera()->setProjectionMatrixAsPerspective(fov, (double)traits->width / traits->height, 1.0 * PERSP_FOV / fov, 1E5 * PERSP_FOV / fov);

	osgViewer_->getCamera()->setLODScale(fov / PERSP_FOV);
}

EntityModel* Viewer::CreateEntityModel(std::string modelFilepath, osg::Vec4 trail_color, EntityModel::EntityType type,
	bool road_sensor, std::string name, OSCBoundingBox* boundingBox, EntityScaleMode scaleMode)
{
	// Load 3D model
	osg::ref_ptr<osg::Group> group = new osg::Group;
	osg::ref_ptr<osg::Group> modelgroup = nullptr;
	osg::ref_ptr<osg::Group> bbGroup = nullptr;
	osg::BoundingBox modelBB;
	std::vector<std::string> file_name_candidates;
	double carStdDim[] = { 4.5, 1.8, 1.5 };
	double carStdOrig[] = { 1.5, 0.0, 0.75 };

	// Check if model already loaded
	for (size_t i = 0; i < entities_.size(); i++)
	{
		if (entities_[i]->filename_ == modelFilepath)
		{
			modelgroup = dynamic_cast<osg::Group*>(entities_[i]->group_->getChild(0)->asGroup()->getChild(0)->clone(osg::CopyOp::DEEP_COPY_NODES));
			modelBB = entities_[i]->modelBB_;
			break;
		}
	}

	// First try to load 3d model
	if (modelgroup == nullptr && !modelFilepath.empty())
	{
		file_name_candidates.push_back(modelFilepath);
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(exe_path_) + "/../resources/models", modelFilepath));
		// Finally check registered paths
		for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
		{
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], modelFilepath));
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], std::string("../models/" + modelFilepath)));
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(modelFilepath)));
		}
		for (size_t i = 0; i < file_name_candidates.size(); i++)
		{
			if (FileExists(file_name_candidates[i].c_str()))
			{
				if (modelgroup = LoadEntityModel(file_name_candidates[i].c_str(), modelBB))
				{
					break;
				}
			}
		}
	}

	// Make sure we have a 3D model
	// Set color of vehicle based on its index
	double* color;
	double b = 1.5;  // brighness
	int index = entities_.size() % 4;

	if (index == 0) color = color_light_gray;
	else if (index == 1) color = color_red;
	else if (index == 2) color = color_blue;
	else color = color_yellow;

	osg::Material* material = new osg::Material();
	material->setDiffuse(osg::Material::FRONT, osg::Vec4(b * color[0], b * color[1], b * color[2], 1.0));
	material->setAmbient(osg::Material::FRONT, osg::Vec4(b * color[0], b * color[1], b * color[2], 1.0));
	osg::ref_ptr<osg::PositionAttitudeTransform> modeltx = new osg::PositionAttitudeTransform;
	if (modelgroup == nullptr)
	{
		if (modelFilepath.empty())
		{
			LOG("No filename specified for model! - creating a dummy model");
		}
		else
		{
			LOG("Failed to load visual model %s. %s", modelFilepath.c_str(),
				file_name_candidates.size() > 1 ? "Also tried the following paths:" : "");
			for (size_t i = 1; i < file_name_candidates.size(); i++)
			{
				LOG("    %s", file_name_candidates[i].c_str());
			}
			LOG("Creating a dummy model instead");
		}

		// Create a dummy cuboid
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		if (boundingBox != nullptr &&
			!(boundingBox->dimensions_.length_ < SMALL_NUMBER &&
				boundingBox->dimensions_.width_ < SMALL_NUMBER &&
				boundingBox->dimensions_.height_ < SMALL_NUMBER))
		{
			geode->addDrawable(new osg::ShapeDrawable(new osg::Box(
				osg::Vec3(boundingBox->center_.x_, boundingBox->center_.y_, boundingBox->center_.z_),
				boundingBox->dimensions_.length_,
				boundingBox->dimensions_.width_,
				boundingBox->dimensions_.height_)));
		}
		else
		{
			geode->addDrawable(new osg::ShapeDrawable(new osg::Box(
				osg::Vec3(carStdOrig[0], carStdOrig[1], carStdOrig[2]),
				carStdDim[0], carStdDim[1], carStdDim[2])));
		}
		geode->setNodeMask(NodeMask::NODE_MASK_ENTITY_MODEL);
		modeltx->addChild(geode);
		modeltx->getOrCreateStateSet()->setAttribute(material);

		// and extract the OSG bounding box
		osg::ComputeBoundsVisitor cbv;
		geode->accept(cbv);
		modelBB = cbv.getBoundingBox();
	}

	// Then create a bounding box visual representation
	bbGroup = new osg::Group;
	osg::ref_ptr<osg::Geode> bbGeode = new osg::Geode;

	if (scaleMode == EntityScaleMode::NONE || scaleMode == EntityScaleMode::MODEL_TO_BB)
	{
		// Bounding box should not scale, create one from OSC description if available
		if (boundingBox != nullptr &&
			!(boundingBox->dimensions_.length_ < SMALL_NUMBER &&
				boundingBox->dimensions_.width_ < SMALL_NUMBER &&
				boundingBox->dimensions_.height_ < SMALL_NUMBER))
		{
			bbGeode->addDrawable(new osg::ShapeDrawable(new osg::Box(
				osg::Vec3(boundingBox->center_.x_, boundingBox->center_.y_, boundingBox->center_.z_),
				boundingBox->dimensions_.length_,
				boundingBox->dimensions_.width_,
				boundingBox->dimensions_.height_)));
		}
		else
		{
			if (scaleMode == EntityScaleMode::MODEL_TO_BB)
			{
				LOG("Request to scale model (%s / %s) to non existing or 0 size bounding box. Created a dummy BB of typical car dimension.",
					name.c_str(), modelFilepath.c_str());
			}
			else if (scaleMode == EntityScaleMode::NONE)
			{
				LOG("Non existing or 0 size bounding box. Created a dummy BB of typical car dimension.");
			}

			// No bounding box specified. Create a bounding box of typical car dimension.
			bbGeode->addDrawable(new osg::ShapeDrawable(new osg::Box(
				osg::Vec3(carStdOrig[0], carStdOrig[1], carStdOrig[2]),
				carStdDim[0], carStdDim[1], carStdDim[2])));

			if (boundingBox != nullptr)
			{
				// Update OSC boundingbox
				boundingBox->center_.x_ = carStdOrig[0];
				boundingBox->center_.y_ = carStdOrig[1];
				boundingBox->center_.z_ = carStdOrig[2];
				boundingBox->dimensions_.length_ = carStdDim[0];
				boundingBox->dimensions_.width_ = carStdDim[1];
				boundingBox->dimensions_.height_ = carStdDim[2];
			}
		}
	}

	if (scaleMode == EntityScaleMode::BB_TO_MODEL)
	{
		// Create visual model of object bounding box, copy values from model bounding box
		bbGeode->addDrawable(new osg::ShapeDrawable(new osg::Box(modelBB.center(),
			modelBB._max.x() - modelBB._min.x(),
			modelBB._max.y() - modelBB._min.y(),
			modelBB._max.z() - modelBB._min.z())));

		// Also update OSC boundingbox
		if (boundingBox != nullptr)
		{
			boundingBox->center_.x_ = modelBB.center().x();
			boundingBox->center_.y_ = modelBB.center().y();
			boundingBox->center_.z_ = modelBB.center().z();
			boundingBox->dimensions_.length_ = modelBB._max.x() - modelBB._min.x();
			boundingBox->dimensions_.width_ = modelBB._max.y() - modelBB._min.y();
			boundingBox->dimensions_.height_ = modelBB._max.z() - modelBB._min.z();
		}
	}
	else if (scaleMode == EntityScaleMode::MODEL_TO_BB)
	{
		// Scale loaded 3d model
		modeltx->getOrCreateStateSet()->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

		double sx = boundingBox->dimensions_.length_ / (modelBB._max.x() - modelBB._min.x());
		double sy = boundingBox->dimensions_.width_ / (modelBB._max.y() - modelBB._min.y());
		double sz = boundingBox->dimensions_.height_ / (modelBB._max.z() - modelBB._min.z());

		modeltx->setPosition(osg::Vec3(
			boundingBox->center_.x_ - sx * modelBB.center().x(),
			boundingBox->center_.y_ - sy * modelBB.center().y(),
			boundingBox->center_.z_ - sz * modelBB.center().z()));
		modeltx->setScale(osg::Vec3(sx, sy, sz));
	}

	// Put transform node under modelgroup
	modeltx->addChild(modelgroup);

	// Draw only wireframe
	osg::PolygonMode* polygonMode = new osg::PolygonMode;
	polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	osg::ref_ptr<osg::StateSet> stateset = bbGeode->getOrCreateStateSet(); // Get the StateSet of the group
	stateset->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	bbGeode->setNodeMask(NodeMask::NODE_MASK_ENTITY_BB);

	osg::ref_ptr<osg::Geode> center = new osg::Geode;
	center->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), 0.2)));
	center->setNodeMask(NodeMask::NODE_MASK_ENTITY_BB);

	bbGroup->addChild(bbGeode);
	bbGroup->addChild(center);
	bbGroup->getOrCreateStateSet()->setAttribute(material);
	bbGroup->setName("BoundingBox");

	group->addChild(modeltx);
	group->addChild(bbGroup);
	group->setName(name);

	EntityModel* emodel;
	if (type == EntityModel::EntityType::VEHICLE)
	{
		emodel = new CarModel(osgViewer_, group, rootnode_, trails_, trajectoryLines_,
			dot_node_, routewaypoints_, trail_color, name);
	}
	else
	{
		emodel = new EntityModel(osgViewer_, group, rootnode_, trails_, trajectoryLines_,
			dot_node_, routewaypoints_, trail_color, name);
	}
	emodel->filename_ = modelFilepath;

	emodel->blend_color_ = new osg::BlendColor(osg::Vec4(1, 1, 1, 1));
	emodel->blend_color_->setDataVariance(osg::Object::DYNAMIC);
	emodel->state_set_ = emodel->lod_->getOrCreateStateSet(); // Creating material
	osg::BlendFunc* bf = new osg::BlendFunc(osg::BlendFunc::CONSTANT_ALPHA, osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
	emodel->state_set_->setAttributeAndModes(emodel->blend_color_);
	emodel->state_set_->setAttributeAndModes(bf);
	emodel->state_set_->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	emodel->state_set_->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	emodel->modelBB_ = modelBB;

	if (type == EntityModel::EntityType::VEHICLE)
	{
		CarModel* vehicle = (CarModel*)emodel;
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

	// on-screen text
	emodel->on_screen_info_.geode_ = new osg::Geode;
	emodel->on_screen_info_.osg_text_ = new osgText::Text;
	emodel->on_screen_info_.osg_text_->setColor(osg::Vec4(1.0f, 1.0f, 0.1f, 1.0f));
	emodel->on_screen_info_.osg_text_->setCharacterSize(10.0);
	emodel->on_screen_info_.osg_text_->setCharacterSizeMode(osgText::TextBase::CharacterSizeMode::SCREEN_COORDS);
	emodel->on_screen_info_.osg_text_->setAxisAlignment(osgText::Text::SCREEN);
	emodel->on_screen_info_.osg_text_->setDrawMode(osgText::Text::TEXT);
	emodel->on_screen_info_.osg_text_->setPosition(osg::Vec3(0.0, 0.0, (boundingBox ? boundingBox->dimensions_.height_ : modelBB.zMax()) + 0.5));
	emodel->on_screen_info_.osg_text_->setDataVariance(osg::Object::DYNAMIC);
	emodel->on_screen_info_.osg_text_->setNodeMask(NodeMask::NODE_MASK_INFO_PER_OBJ);
	emodel->on_screen_info_.osg_text_->setAlignment(osgText::Text::LEFT_BOTTOM);

	osg::StateSet* text_state = emodel->on_screen_info_.osg_text_->getOrCreateStateSet();
	text_state->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED | osg::StateAttribute::OVERRIDE);
	osg::ref_ptr<osg::BlendFunc> bf2 = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	text_state->setAttributeAndModes(bf2);
	text_state->setAttributeAndModes(emodel->blend_color_);
	text_state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED | osg::StateAttribute::OVERRIDE);
	text_state->setRenderBinDetails(INT_MAX-1, "RenderBin", osg::StateSet::RenderBinMode::OVERRIDE_RENDERBIN_DETAILS);

	emodel->on_screen_info_.geode_->addDrawable(emodel->on_screen_info_.osg_text_);
	group->addChild(emodel->on_screen_info_.geode_.get());

	return emodel;
}

int Viewer::AddEntityModel(EntityModel* model)
{
	entities_.push_back(model);

	// Focus on first added car
	if (entities_.size() == 1)
	{
		currentCarInFocus_ = 0;
		((osgGA::KeySwitchMatrixManipulator*)osgViewer_->getCameraManipulator())->selectMatrixManipulator(0);
		rubberbandManipulator_->setTrackNode(entities_.back()->txNode_,
			rubberbandManipulator_->getMode() == osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_TOP ? false : true);
		nodeTrackerManipulator_->setTrackNode(entities_.back()->txNode_);
	}

	return 0;
}

void Viewer::RemoveCar(int index)
{
	if (entities_[index] != nullptr)
	{
		entities_[index]->parent_->removeChild(entities_[index]->group_);
		delete (entities_[index]);
	}
	entities_.erase(entities_.begin() + index);

	if (currentCarInFocus_ > index && currentCarInFocus_ > 0)
	{
		// Shift with reduces list
		currentCarInFocus_--;
		SetVehicleInFocus((currentCarInFocus_) % entities_.size());
	}
	else if (currentCarInFocus_ == index)
	{
		if (entities_.size() > 0)
		{
			// Reset camera focus
			SetVehicleInFocus((currentCarInFocus_) % entities_.size());
		}
	}

	if (entities_.size() == 0)
	{
		// No more objects to follow, switch camera model
		currentCarInFocus_ = -1;
		((osgGA::KeySwitchMatrixManipulator*)osgViewer_->getCameraManipulator())->selectMatrixManipulator(5);
	}
}

void Viewer::ReplaceCar(int index, EntityModel* model)
{
	if (entities_[index] != nullptr)
	{
		entities_[index]->parent_->removeChild(entities_[index]->txNode_);
		delete (entities_[index]);
		entities_[index] = model;
	}

	if (currentCarInFocus_ == index)
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
}

void Viewer::RemoveCar(std::string name)
{
	for (size_t i = 0; i < entities_.size(); i++)
	{
		if (entities_[i]->name_ == name)
		{
			if (entities_[i] != nullptr)
			{
				RemoveCar(i);
				return;
			}
		}
	}
}

osg::ref_ptr<osg::Group> Viewer::LoadEntityModel(const char* filename, osg::BoundingBox& bb)
{
	static int elev = 0;  // Avoid shadow node to flicker, put every second on slightly different Z
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
	bb = cbv.getBoundingBox();

	double xc, yc, dx, dy;
	dx = bb._max.x() - bb._min.x();
	dy = bb._max.y() - bb._min.y();
	xc = (bb._max.x() + bb._min.x()) / 2;
	yc = (bb._max.y() + bb._min.y()) / 2;

	if (!shadow_node_)
	{
		LoadShadowfile(filename);
	}

	node->setNodeMask(NodeMask::NODE_MASK_ENTITY_MODEL);
	group->addChild(node);

	if (shadow_node_)
	{
		shadow_tx = new osg::PositionAttitudeTransform;
		shadow_tx->setName("shadow_tx");
		shadow_tx->setPosition(osg::Vec3d(xc, yc, 0.05 * elev));
		shadow_tx->setScale(osg::Vec3d(SHADOW_SCALE * (dx / 2), SHADOW_SCALE * (dy / 2), 1.0));
		shadow_tx->addChild(shadow_node_);

		shadow_tx->setNodeMask(NodeMask::NODE_MASK_ENTITY_MODEL);
		group->addChild(shadow_tx);
	}

	elev = (elev + 1) % 3;
	return group;
}

bool Viewer::CreateRoadMarkLines(roadmanager::OpenDrive* od)
{
	double z_offset = 0.10;
	osg::Vec3 point(0, 0, 0);

	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		roadmanager::Road* road = od->GetRoadByIdx(r);
		for (int i = 0; i < road->GetNumberOfLaneSections(); i++)
		{
			roadmanager::LaneSection* lane_section = road->GetLaneSectionByIdx(i);
			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				roadmanager::Lane* lane = lane_section->GetLaneByIdx(j);
				for (int k = 0; k < lane->GetNumberOfRoadMarks(); k++)
				{
					roadmanager::LaneRoadMark* lane_roadmark = lane->GetLaneRoadMarkByIdx(k);
					for (int m = 0; m < lane_roadmark->GetNumberOfRoadMarkTypes(); m++)
					{
						roadmanager::LaneRoadMarkType* lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(m);
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
							for (int q = 0; q < lane_roadmarktype->GetNumberOfRoadMarkTypeLines(); q++)
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
							roadmanager::LaneRoadMarkTypeLine* lane_roadmarktypeline = lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(n);
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
								for (int q = 0; q < curr_osi_rm->GetPoints().size(); q += 2)
								{
									roadmanager::PointStruct osi_point1 = curr_osi_rm->GetPoint(q);
									roadmanager::PointStruct osi_point2 = curr_osi_rm->GetPoint(q + 1);

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
									if (lane_roadmark->GetWeight() == roadmanager::LaneRoadMark::BOLD)
									{
										lineWidth->setWidth(OSI_LINE_WIDTH_BOLD);
									}
									else
									{
										lineWidth->setWidth(OSI_LINE_WIDTH);
									}
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
							else if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID ||
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
								if (lane_roadmark->GetWeight() == roadmanager::LaneRoadMark::BOLD)
								{
									lineWidth->setWidth(OSI_LINE_WIDTH_BOLD);
								}
								else
								{
									lineWidth->setWidth(OSI_LINE_WIDTH);
								}
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
		roadmanager::Road* road = od->GetRoadByIdx(r);

		// Road key points
		osg::ref_ptr<osg::Geometry> kp_geom = new osg::Geometry;
		osg::ref_ptr<osg::Vec3Array> kp_points = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> kp_color = new osg::Vec4Array;
		osg::ref_ptr<osg::Point> kp_point = new osg::Point();

		roadmanager::Geometry* geom = nullptr;
		for (int i = 0; i < road->GetNumberOfGeometries() + 1; i++)
		{
			if (i < road->GetNumberOfGeometries())
			{
				geom = road->GetGeometry(i);
				pos->SetTrackPos(road->GetId(), geom->GetS(), 0);
			}
			else
			{
				pos->SetTrackPos(road->GetId(), geom->GetS() + geom->GetLength(), 0);
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
			roadmanager::LaneSection* lane_section = road->GetLaneSectionByIdx(i);
			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				roadmanager::Lane* lane = lane_section->GetLaneByIdx(j);

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
					else if (k == 0)
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

int Viewer::CreateOutlineObject(roadmanager::Outline* outline, osg::Vec4 color)
{
	if (outline == 0) return -1;
	bool roof = outline->closed_ ? true : false;

	// nrPoints will be corners + 1 if the outline should be closed, reusing first corner as last
	int nrPoints = outline->closed_ ? outline->corner_.size() + 1 : outline->corner_.size();

	osg::ref_ptr<osg::Group> group = new osg::Group();

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
		(*vertices_top)[nrPoints - 1].set((*vertices_top)[0]);
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

osg::ref_ptr<osg::PositionAttitudeTransform> Viewer::LoadRoadFeature(roadmanager::Road* road, std::string filename)
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
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(filename)));
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
	osg::ref_ptr<osg::PositionAttitudeTransform> tx = nullptr;

	roadmanager::Position pos;

	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		roadmanager::Road* road = od->GetRoadByIdx(r);
		for (size_t s = 0; s < road->GetNumberOfSignals(); s++)
		{
			tx = nullptr;
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
			osg::Vec4 color;
			tx = nullptr;

			// Set color based on object type
			if (object->GetType() == roadmanager::RMObject::ObjectType::BUILDING ||
				object->GetType() == roadmanager::RMObject::ObjectType::BARRIER)
			{
				color = osg::Vec4(0.6, 0.6, 0.6, 1.0);
			}
			else if (object->GetType() == roadmanager::RMObject::ObjectType::OBSTACLE)
			{
				color = osg::Vec4(0.5, 0.3, 0.3, 1.0);
			}
			else if (object->GetType() == roadmanager::RMObject::ObjectType::TREE ||
				object->GetType() == roadmanager::RMObject::ObjectType::VEGETATION)
			{
				color = osg::Vec4(0.22, 0.32, 0.22, 1.0);
			}
			else
			{
				color = osg::Vec4(0.4, 0.4, 0.4, 1.0);
			}

			if (object->GetNumberOfOutlines() > 0)
			{
				for (size_t j = 0; j < object->GetNumberOfOutlines(); j++)
				{
					roadmanager::Outline* outline = object->GetOutline(j);
					CreateOutlineObject(outline, color);
				}
				LOG("Created outline geometry for object %s.", object->GetName().c_str());
				LOG("  if it looks strange, e.g.faces too dark or light color, ");
				LOG("  check that corners are defined counter-clockwise (as OpenGL default).");
			}
			else
			{
				double orientation = object->GetOrientation() == roadmanager::Signal::Orientation::NEGATIVE ? M_PI : 0.0;

				// absolute path or relative to current directory
				std::string filename = object->GetName();

				// Assume name is representing a 3D model filename
				if (!filename.empty())
				{
					std::vector<std::string> file_name_candidates;

					if (FileNameExtOf(filename) == "")
					{
						filename += ".osgb";  // add missing extension
					}

					tx = LoadRoadFeature(road, filename);

					if (tx == nullptr)
					{
						LOG("Failed to load road object model file: %s (%s). Creating a bounding box as stand in.", filename.c_str(), object->GetName().c_str());
					}
				}

				if (tx == nullptr)
				{
					// create a bounding box to represent the object
					osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, 0, 0.5 * object->GetHeight()),
						object->GetLength(),
						object->GetWidth(),
						object->GetHeight()));

					shape->setColor(color);

					tx = new osg::PositionAttitudeTransform;
					tx->addChild(shape);
				}

				roadmanager::Repeat* rep = object->GetRepeat();
				int nCopies = 0;
				double cur_s = 0.0;

				osg::ComputeBoundsVisitor cbv;
				tx->accept(cbv);
				osg::BoundingBox boundingBox = cbv.getBoundingBox();

				double dim_x = boundingBox._max.x() - boundingBox._min.x();
				double dim_y = boundingBox._max.y() - boundingBox._min.y();
				double dim_z = boundingBox._max.z() - boundingBox._min.z();
				if (object->GetLength() < SMALL_NUMBER && dim_x > SMALL_NUMBER)
				{
					LOG("Object %s missing length, set to bounding box length %.2f", object->GetName().c_str(), dim_x);
					object->SetLength(dim_x);
				}
				if (object->GetWidth() < SMALL_NUMBER && dim_y > SMALL_NUMBER)
				{
					LOG("Object %s missing width, set to bounding box width %.2f", object->GetName().c_str(), dim_y);
					object->SetWidth(dim_y);
				}
				if (object->GetHeight() < SMALL_NUMBER && dim_z > SMALL_NUMBER)
				{
					LOG("Object %s missing height, set to bounding box height %.2f", object->GetName().c_str(), dim_z);
					object->SetHeight(dim_z);
				}

				double lastLODs = 0.0;  // used for putting object copies in LOD groups
				osg::ref_ptr<osg::Group> LODGroup = 0;
				osg::ref_ptr<osg::PositionAttitudeTransform> clone = 0;

				for (; nCopies < 1 || rep && rep->length_ > SMALL_NUMBER && cur_s < rep->GetLength() &&
					cur_s + rep->GetS() < road->GetLength(); nCopies++)
				{
					double factor, t, s, zOffset;
					double scale_x = 1.0;
					double scale_y = 1.0;
					double scale_z = 1.0;

					if (rep && cur_s + rep->GetS() + (object->GetLength() * scale_x) * cos(object->GetHOffset()) > road->GetLength())
					{
						break;  // object would reach outside specified total length
					}

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
						clone->setAttitude(quatLocal * quatRoad);
					}
					else
					{
						factor = (double)cur_s / rep->GetLength();
						t = rep->GetTStart() + factor * (rep->GetTEnd() - rep->GetTStart());
						s = rep->GetS() + cur_s;
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
						clone->setAttitude(quatLocal* quatRoad);

						// increase current s distance according to road curvature
						cur_s += pos.DistanceToDS(rep->distance_);
					}

					clone->setDataVariance(osg::Object::STATIC);

					if (LODGroup == 0 || s - lastLODs > 0.5 * LOD_DIST_ROAD_FEATURES)
					{
						// add current LOD and create a new one
						osg::ref_ptr<osg::LOD> lod = new osg::LOD();
						LODGroup = new osg::Group();
						lod->addChild(LODGroup);
						lod->setRange(0, 0, LOD_DIST_ROAD_FEATURES + MAX(boundingBox.xMax() - boundingBox.xMin(), boundingBox.yMax() - boundingBox.yMin()));
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

bool Viewer::CreateRoadSensors(CarModel* vehicle_model)
{
	vehicle_model->road_sensor_ = CreateSensor(color_gray, true, false, 0.35, 2.5);
	vehicle_model->route_sensor_ = CreateSensor(color_blue, true, false, 0.30, 2.5);
	vehicle_model->lane_sensor_ = CreateSensor(color_gray, true, true, 0.25, 2.5);

	return true;
}

PointSensor* Viewer::CreateSensor(double color[], bool create_ball, bool create_line, double ball_radius, double line_width)
{
	PointSensor* sensor = new PointSensor();
	sensor->group_ = new osg::Group();

	// Point
	if (create_ball)
	{
		osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), ball_radius));
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

void Viewer::UpdateRoadSensors(PointSensor *road_sensor, PointSensor* route_sensor, PointSensor *lane_sensor, roadmanager::Position *pos)
{
	if (road_sensor == 0 || route_sensor == 0 || lane_sensor == 0)
	{
		return;
	}

	roadmanager::Position track_pos(*pos);
	track_pos.SetTrackPos(pos->GetTrackId(), pos->GetS(), 0);

	SensorSetPivotPos(road_sensor, pos->GetX(), pos->GetY(), pos->GetZ());
	SensorSetTargetPos(road_sensor, track_pos.GetX(), track_pos.GetY(), track_pos.GetZ());
	UpdateSensor(road_sensor);

	roadmanager::Position route_pos(track_pos);
	roadmanager::Route* r = pos->GetRoute();
	if (r && r->IsValid())
	{
		route_pos.SetLanePos(r->GetTrackId(), r->GetLaneId(), r->GetTrackS(), 0.0);
	}

	SensorSetPivotPos(route_sensor, pos->GetX(), pos->GetY(), pos->GetZ());
	SensorSetTargetPos(route_sensor, route_pos.GetX(), route_pos.GetY(), route_pos.GetZ());
	UpdateSensor(route_sensor);

	roadmanager::Position lane_pos(*pos);
	lane_pos.SetLanePos(pos->GetTrackId(), pos->GetLaneId(), pos->GetS(), 0);

	SensorSetPivotPos(lane_sensor, pos->GetX(), pos->GetY(), pos->GetZ());
	SensorSetTargetPos(lane_sensor, lane_pos.GetX(), lane_pos.GetY(), lane_pos.GetZ());
	UpdateSensor(lane_sensor);
}

void Viewer::SensorSetPivotPos(PointSensor* sensor, double x, double y, double z)
{
	double z_offset = 0.2;
	sensor->pivot_pos = osg::Vec3(x, y, z + MAX(sensor->ball_radius_ / 3.0, z_offset));
}

void Viewer::SensorSetTargetPos(PointSensor* sensor, double x, double y, double z)
{
	double z_offset = 0.2;
	sensor->target_pos = osg::Vec3(x, y, z + MAX(sensor->ball_radius_ / 3.0, z_offset));
}

void Viewer::UpdateSensor(PointSensor* sensor)
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

void Viewer::SetVehicleInFocus(int idx)
{
	if (idx >= 0 && idx < entities_.size())
	{
		currentCarInFocus_ = idx;
		rubberbandManipulator_->setTrackNode(entities_[currentCarInFocus_]->txNode_, false);
		nodeTrackerManipulator_->setTrackNode(entities_[currentCarInFocus_]->txNode_);
	}
}

void SetFixCameraFlag(bool fixed)
{

}

void SetFixCameraPosition(osg::Vec3 pos, osg::Vec3 rot)
{

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

void Viewer::SetWindowTitleFromArgs(std::vector<std::string>& args)
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
	PolyLine* pline = new PolyLine(rootnode_, points, color, width, dotsize);
	polyLine_.push_back(pline);
	return polyLine_.back();
}

PolyLine* Viewer::AddPolyLine(osg::Group* parent, osg::ref_ptr<osg::Vec3Array> points, osg::Vec4 color, double width, double dotsize)
{
	PolyLine* pline = new PolyLine(parent, points, color, width, dotsize);
	polyLine_.push_back(pline);
	return pline;
}

void Viewer::RegisterKeyEventCallback(KeyEventCallbackFunc func, void* data)
{
	KeyEventCallback cb;
	cb.func = func;
	cb.data = data;
	callback_.push_back(cb);
}

void Viewer::RegisterImageCallback(ImageCallbackFunc func, void* data)
{
	imgCallback_.func = func;
	imgCallback_.data = data;
}

void Viewer::SaveImagesToFile(int nrOfFrames)
{
	saveImagesToFile_ = nrOfFrames;
}

void Viewer::Frame()
{
	if (SE_Env::Inst().GetOffScreenRendering())
	{
		renderSemaphore.Set();  // Raise semaphore to flag rendering ongoing
	}

	osgViewer_->frame();

	if (!SE_Env::Inst().GetOffScreenRendering())
	{
		frameCounter_++;
	}
}

bool ViewerEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
	switch (ea.getEventType())
	{
	case(osgGA::GUIEventAdapter::RESIZE):
		viewer_->infoTextCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, ea.getWindowWidth(), 0, ea.getWindowHeight()));
		break;
	case(osgGA::GUIEventAdapter::CLOSE_WINDOW):
	case(osgGA::GUIEventAdapter::QUIT_APPLICATION):
		viewer_->SetQuitRequest(true);
		break;
	}

	switch (ea.getKey())
	{
	case('C'):
	case('c'):
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			if (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)
			{
				if (viewer_->GetSaveImagesToFile() == 0)
				{
					viewer_->SaveImagesToFile(-1);
				}
				else
				{
					viewer_->SaveImagesToFile(0);
				}
			}
			else
			{
				viewer_->SaveImagesToFile(1);  // single frame
			}
		}
		break;
	case('k'):
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->SetCameraMode((viewer_->camMode_ + 1) % viewer_->rubberbandManipulator_->GetNumberOfCameraModes());
		}
		break;
	case('K'):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			// Print current camera position
			osg::Vec3 pos, rot;
			viewer_->GetCameraPosAndRot(pos, rot);
			printf("Camera pos: %.5f, %.5f, %.5f rot: %.5f, %.5f, %.5f\n", pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]);
		}
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
	case('R'):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(viewer::NodeMask::NODE_MASK_ROUTE_WAYPOINTS);
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
	case(osgGA::GUIEventAdapter::KEY_BackSpace):  // Also map backspace
	case(0xFE20):  // Cover left_shift+Tab on Linux
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			int step = 1;
			if (ea.getKey() == 0xFF08 || ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_LEFT_SHIFT)
			{
				step = -1;
			}
			int idx = viewer_->currentCarInFocus_ + step;

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
			int mask = viewer_->GetNodeMaskBit(
				viewer::NodeMask::NODE_MASK_INFO |
				viewer::NodeMask::NODE_MASK_INFO_PER_OBJ) / viewer::NodeMask::NODE_MASK_INFO;

			// Toggle between modes: 0: none, 1: global info, 2: per object only, 3. both
			mask = ((mask + 1) % 4) * viewer::NodeMask::NODE_MASK_INFO;

			viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_INFO |
				viewer::NodeMask::NODE_MASK_INFO_PER_OBJ, mask);

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
	case('r'):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->ToggleNodeMaskBits(viewer::NodeMask::NODE_MASK_OBJECT_SENSORS);
		}
	}
	break;
	case(osgGA::GUIEventAdapter::KEY_Escape):
	{
		if (ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN)
		{
			viewer_->SetQuitRequest(true);
		}
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

	if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Space)  // prevent OSG "view reset" action on space key
	{
		return true;
	}
	else
	{
		// forward all other key events to OSG
		return false;
	}
}
