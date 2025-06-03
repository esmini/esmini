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

#ifndef ROADGEOM_HPP_
#define ROADGEOM_HPP_

#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Material>
#include "RoadManager.hpp"

class RoadGeom
{
public:
    enum class MaterialType : uint8_t
    {
        NONE = 0,
        ASPHALT,
        GRASS,
        ROADMARK,
        CONCRETE,
        BORDER
    };

    osg::ref_ptr<osg::Group>                                  root_;
    osg::ref_ptr<osg::Group>                                  rm_group_;
    osg::ref_ptr<osg::Vec4Array>                              color_asphalt_ = new osg::Vec4Array;
    std::unordered_map<uint64_t, osg::ref_ptr<osg::Material>> std_materials_ = {};

    RoadGeom(roadmanager::OpenDrive* odr, osg::Vec3d origin);

    int                          AddRoadMarks(roadmanager::Lane* lane, osg::Group* group);
    void                         AddRoadMarkGeom(osg::ref_ptr<osg::Vec3Array>        vertices,
                                                 osg::ref_ptr<osg::DrawElementsUInt> indices,
                                                 roadmanager::RoadMarkColor          color,
                                                 double                              fade);
    osg::ref_ptr<osg::Material>  GetOrCreateMaterial(const std::string& basename, osg::Vec4 color, uint8_t texture_type, uint8_t has_friction = 0);
    osg::ref_ptr<osg::Texture2D> ReadTexture(std::string filename);
    const osg::Vec4              GetFrictionColor(const double friction);

private:
    unsigned int                                                   number_of_materials = 0;
    std::unordered_map<MaterialType, osg::ref_ptr<osg::Texture2D>> texture_map_        = {};
    double                                                         lane_friction_      = 1.0;
};

#endif  // ROADGEOM_HPP_
