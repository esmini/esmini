#include "roadgeom.hpp"
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

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Switch>
#include <osg/BlendFunc>
#include <osg/ArgumentParser>
#include <osg/ShapeDrawable>
#include <osgGA/TrackballManipulator>
#include <osgDB/WriteFile>
#include <osg/Quat>
#include <osg/Texture2D>
#include <osg/Material>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Texture2D>
#include <osg/Image>
#include <osgDB/ReadFile>  // For osgDB::readImage
#include <osgViewer/Viewer>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osgUtil/SmoothingVisitor>
#include "trafficlightmodel.hpp"

osg::ref_ptr<osg::Geode> TrafficLightModel::CreateOpenBox()
{
    osg::ref_ptr<osg::Vec4Array> color_outline = new osg::Vec4Array();
    color_outline->push_back(osg::Vec4(0.1, 0.1, 0.1, 1.0));

    osg::ref_ptr<osg::Vec3Array> vertices_sides = new osg::Vec3Array(10);  // one closed set at bottom and one at top
    osg::ref_ptr<osg::Vec3Array> vertices_top   = new osg::Vec3Array(4);   // one set for roof

    // Set vertices
    float y       = box_width_ / 2.0f;
    float v[4][2] = {{-y, 0}, {-y, box_height_ * n_lights_}, {y, box_height_ * n_lights_}, {y, 0}};
    for (size_t i = 0; i < 5; i++)
    {
        (*vertices_sides)[i * 2].set(-box_depth_, v[i % 4][0], v[i % 4][1]);  // closed quad strip bottom vertices
        (*vertices_sides)[i * 2 + 1].set(0.0, v[i % 4][0], v[i % 4][1]);      // closed quad strip top vertices
        if (i < 4)
            (*vertices_top)[i].set(0.0, v[i][0], v[i][1]);  // back face
    }

    // Finally create and add geometry
    osg::ref_ptr<osg::Geode>    geode  = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom[] = {new osg::Geometry, new osg::Geometry};

    geom[0]->setVertexArray(vertices_sides.get());
    geom[0]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, 10));

    // Add back face
    geom[1]->setVertexArray(vertices_top.get());
    geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    for (int i = 0; i < 2; i++)
    {
        osgUtil::SmoothingVisitor::smooth(*geom[i], 0.1);
        geom[i]->setColorArray(color_outline);
        geom[i]->setColorBinding(osg::Geometry::BIND_OVERALL);
        geom[i]->setDataVariance(osg::Object::STATIC);
        geom[i]->setUseDisplayList(true);
        geode->addDrawable(geom[i]);
    }

    return geode;
}

osg::ref_ptr<osg::Switch> TrafficLightModel::CreateFrontSwitchNode(int index, osg::ref_ptr<osg::Texture2D> texture)
{
    osg::ref_ptr<osg::Switch> switch_node = new osg::Switch;

    osg::ref_ptr<osg::Vec4Array> color_white = new osg::Vec4Array();
    color_white->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    osg::ref_ptr<osg::Vec4Array> color_black = new osg::Vec4Array();
    color_black->push_back(osg::Vec4(0.0, 0.0, 0.0, 1.0));

    osg::ref_ptr<osg::Vec3Array> vertices_top = new osg::Vec3Array(4);  // one set for the face

    // Set vertices
    float l       = box_width_ / 2.0f;
    float z0      = index * box_height_;
    float v[4][2] = {{-l, z0}, {l, z0}, {l, z0 + box_height_}, {-l, z0 + box_height_}};
    for (size_t i = 0; i < 4; i++)
    {
        (*vertices_top)[i].set(-box_depth_, v[i][0], v[i][1]);
    }

    // Finally create and add geometry
    osg::ref_ptr<osg::Geode>    geode[2] = {new osg::Geode, new osg::Geode};
    osg::ref_ptr<osg::Geometry> geom     = new osg::Geometry;

    geom->setVertexArray(vertices_top.get());
    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    // create a pure black variant of the textured face
    osg::ref_ptr<osg::Geometry> geom_black = dynamic_cast<osg::Geometry*>(geom->clone(osg::CopyOp::DEEP_COPY_ALL));

    geom->setColorArray(color_white);
    geom_black->setColorArray(color_black);

    for (auto& g : {geom, geom_black})
    {
        osgUtil::SmoothingVisitor::smooth(*g, 0.1);
        g->setColorBinding(osg::Geometry::BIND_OVERALL);
        g->setDataVariance(osg::Object::STATIC);
        g->setUseDisplayList(true);
    }

    // Texture Coordinates (passed as arguments)
    osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array;
    texcoords->push_back(osg::Vec2(1, static_cast<float>(index) / n_lights_));
    texcoords->push_back(osg::Vec2(0, static_cast<float>(index) / n_lights_));
    texcoords->push_back(osg::Vec2(0, static_cast<float>(index + 1) / n_lights_));
    texcoords->push_back(osg::Vec2(1, static_cast<float>(index + 1) / n_lights_));
    geom->setTexCoordArray(0, texcoords);  // Channel 0

    // Create a StateSet for the textured face and make if unaffected by lighting
    osg::ref_ptr<osg::StateSet> texturedStateSet = geode[1]->getOrCreateStateSet();
    texturedStateSet->setTextureAttributeAndModes(0, texture.get());
    texturedStateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    geode[0]->addDrawable(geom_black);
    geode[1]->addDrawable(geom);

    switch_node->addChild(geode[0].get(), true);
    switch_node->addChild(geode[1].get(), false);
    switch_node->setName(std::string("lamp_") + std::to_string(index));

    return switch_node.get();
}

void TrafficLightModel::SetState(unsigned int light_index, bool state)
{
    if (light_index < switches_.size() && switches_[light_index] != nullptr)
    {
        switches_[light_index]->setAllChildrenOff();
        switches_[light_index]->setValueList({!state, state});
    }
}

bool TrafficLightModel::GetState(unsigned int light_index) const
{
    return switches_[light_index]->getValue(1);
}

TrafficLightModel::TrafficLightModel(unsigned int n_lights, std::string texture_filename) : n_lights_(n_lights)
{
    tx_ = new osg::PositionAttitudeTransform;

    // texture
    osg::ref_ptr<osg::Image> image = osgDB::readImageFile(texture_filename);  // Replace with your image
    if (image == nullptr)
    {
        std::cout << "Error: Could not load texture image!" << std::endl;
    }

    osg::ref_ptr<osg::Geode> geode_box = CreateOpenBox();
    tx_->addChild(geode_box);

    osg::ref_ptr<osg::Texture2D> texture;
    if (image != nullptr)
    {
        // Create a Texture2D object for the light objects
        texture = new osg::Texture2D;
        texture->setImage(image);
        texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
        texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    }

    // create the light objects
    for (unsigned int i = 0; i < n_lights; i++)
    {
        // add lights in reversed order, from top to bottom
        osg::ref_ptr<osg::Switch> switch_node = CreateFrontSwitchNode(n_lights - i - 1, texture);
        switches_.push_back(switch_node);
        tx_->addChild(switch_node.get());
    }
}
