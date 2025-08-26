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

#include "roadgeom.hpp"
#include "RoadManager.hpp"
#include "logger.hpp"

#include <osg/StateSet>
#include <osg/Group>
#include <osg/TexEnv>
#include <osg/LOD>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osgGA/StateSetManipulator>
#include <osg/PolygonOffset>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>
#include <osg/ShapeDrawable>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/Optimizer>    // to flatten transform nodes
#include <osgUtil/Tessellator>  // to tessellate multiple contours
#include <osgDB/WriteFile>

#include "CommonMini.hpp"

// cppcheck-suppress [unknownMacro]
USE_OSGPLUGIN(osg2)
USE_OSGPLUGIN(jpeg)
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
USE_COMPRESSOR_WRAPPER(ZLibCompressor)

#define GEOM_TOLERANCE         (0.2 - SMALL_NUMBER)  // Minimum distance between two vertices along road s-axis
#define TEXTURE_SCALE          2.0                   // Scale factor for asphalt and grass textures 2.0 means whole texture fits in 2 x 2 m square
#define MAX_GEOM_ERROR         0.25                  // maximum distance from the 3D geometry to the OSI lines
#define MAX_GEOM_LENGTH        50                    // maximum length of a road geometry mesh segment
#define MIN_GEOM_LENGTH        0.1                   // minimum length of a road geometry mesh segment, adjust if possible
#define ROADMARK_TEXTURE_SCALE 3.0                   // scale factor for roadmark textures, 3.0 means whole texture fits in 3 x 3 m square

#define POLYGON_OFFSET_SIDEWALK  2.0
#define POLYGON_OFFSET_ROADMARKS 1.0
#define POLYGON_OFFSET_BORDER    -1.0
#define POLYGON_OFFSET_GRASS     -2.0

#define ROADMARK_Z_OFFSET 0.02

#define DEFAULT_LENGTH_FOR_CONTINUOUS_OBJS 10.0
#define LOD_DIST_ROAD_FEATURES             500

const static double      friction_max       = 5.0;
const static double      friction_default   = 1.0;
const static std::string prefix_road        = "road_";
const static std::string prefix_road_object = "road_object_";
const static std::string prefix_road_signal = "road_signal_";
const static std::string prefix_roadmark    = "roadmark_";

namespace roadgeom
{

    bool compare_s_values(double s0, double s1)
    {
        return (fabs(s1 - s0) < 0.1);
    }

    uint64_t GenerateMaterialKey(double r, double g, double b, double a, uint8_t t, uint8_t f)
    {
        uint8_t r8 = static_cast<uint8_t>(std::max(0.0, std::min(255.0, r * 255.0)));
        uint8_t g8 = static_cast<uint8_t>(std::max(0.0, std::min(255.0, g * 255.0)));
        uint8_t b8 = static_cast<uint8_t>(std::max(0.0, std::min(255.0, b * 255.0)));
        uint8_t a8 = static_cast<uint8_t>(std::max(0.0, std::min(255.0, a * 255.0)));

        // code the color as a 64-bit integer in the format 0xRRGGBBAATTFF (T = texture_type, F = friction)
        return (static_cast<uint64_t>(r8) << 40) | (static_cast<uint64_t>(g8) << 32) | (static_cast<uint64_t>(b8) << 24) |
               (static_cast<uint64_t>(a8) << 16) | (static_cast<uint64_t>(t) << 8) | static_cast<uint64_t>(f);
    }

    osg::Vec4 ODR2OSGColor(roadmanager::RoadMarkColor color)
    {
        const float(&rgb)[3] = SE_Color::Color2RBG(roadmanager::ODRColor2SEColor(color));
        return osg::Vec4(rgb[0], rgb[1], rgb[2], 1.0f);
    }

    osg::ref_ptr<osg::Material> RoadGeom::GetOrCreateMaterial(const std::string& basename,
                                                              osg::Vec4          color,
                                                              uint8_t            texture_type,
                                                              uint8_t            has_friction)
    {
        uint64_t key = GenerateMaterialKey(color[0], color[1], color[2], color[3], texture_type, has_friction);

        if (std_materials_.find(key) != std_materials_.end())
        {
            return std_materials_[key];
        }
        else
        {
            // create and store new materiak
            osg::ref_ptr<osg::Material> material = new osg::Material;
            material->setName(fmt::format("Material_{}_{}_0x{:02x}{:02x}{:02x}{:02x}_{:02x}_{:02x}",
                                          number_of_materials,
                                          basename,
                                          static_cast<unsigned int>(color[0] * 255),
                                          static_cast<unsigned int>(color[1] * 255),
                                          static_cast<unsigned int>(color[2] * 255),
                                          static_cast<unsigned int>(color[3] * 255),
                                          static_cast<unsigned int>(texture_type),
                                          static_cast<unsigned int>(has_friction)));

            LOG_DEBUG("Creating material {}", material->getName());
            material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
            material->setAmbient(osg::Material::FRONT_AND_BACK, color);
            material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
            material->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
            material->setShininess(osg::Material::FRONT_AND_BACK, 1.0f);
            if (static_cast<uint8_t>(MaterialType::ASPHALT) == texture_type)
            {
                material->setUserValue("friction", lane_friction_);  // default friction value
            }

            // store material for reuse
            std_materials_[key] = material;

            // keep track of the number of created materials
            number_of_materials++;

            return material;
        }
    }

    osg::ref_ptr<osg::Texture2D> RoadGeom::ReadTexture(std::string filename)
    {
        osg::ref_ptr<osg::Texture2D> tex = 0;
        osg::ref_ptr<osg::Image>     img = 0;

        std::vector<std::string> file_name_candidates;
        file_name_candidates.push_back(filename);

        // Check registered paths
        for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
        {
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], filename));
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], "../models/" + filename));
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(filename)));
        }
        for (size_t i = 0; i < file_name_candidates.size(); i++)
        {
            if (FileExists(file_name_candidates[i].c_str()))
            {
                if (img = osgDB::readImageFile(file_name_candidates[i].c_str()))
                {
                    break;
                }
            }
        }

        if (img)
        {
            tex = new osg::Texture2D(img.get());
            tex->setUnRefImageDataAfterApply(true);
            tex->setWrap(osg::Texture2D::WrapParameter::WRAP_S, osg::Texture2D::WrapMode::REPEAT);
            tex->setWrap(osg::Texture2D::WrapParameter::WRAP_T, osg::Texture2D::WrapMode::REPEAT);
        }

        return tex;
    }

    const osg::Vec4 RoadGeom::GetFrictionColor(const double friction)
    {
        osg::Vec4 new_color = color_asphalt_->at(0);
        if (friction < friction_default - SMALL_NUMBER)  // low friction, make it blueish
        {
            double factor = (1.0 - friction) / friction_default;
            new_color[0] -= 0.75 * factor;
            new_color[1] -= 0.75 * factor;
            new_color[2] += factor;
        }
        else if (friction > friction_default + SMALL_NUMBER)  // high friction, make it redish
        {
            double factor = (MIN(friction, friction_max) - friction_default) / (friction_max - friction_default);
            new_color[0] += factor;
            new_color[1] -= 0.75 * factor;
            new_color[2] -= 0.75 * factor;
        }

        new_color[0] = CLAMP(0.0, 1.0, new_color[0]);
        new_color[1] = CLAMP(0.0, 1.0, new_color[1]);
        new_color[2] = CLAMP(0.0, 1.0, new_color[2]);

        return new_color;
    }

    void RoadGeom::AddRoadMarkGeom(osg::ref_ptr<osg::Vec3Array>        vertices,
                                   osg::ref_ptr<osg::DrawElementsUInt> indices,
                                   osg::Group*                         rm_group,
                                   roadmanager::RoadMarkColor          color,
                                   double                              fade)
    {
        osg::ref_ptr<osg::Vec4Array> color_array = new osg::Vec4Array;
        color_array->push_back(ODR2OSGColor(color));
        color_array->back()[3] = 1.0 - fade;  // Set alpha value

        // Finally create and add geometry
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setUseDisplayList(true);
        geom->setVertexArray(vertices.get());
        geom->addPrimitiveSet(indices.get());

        // Use PolygonOffset feature to avoid z-fighting with road surface
        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(-POLYGON_OFFSET_ROADMARKS, -SIGN(POLYGON_OFFSET_ROADMARKS)));

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;

        // create material with unique name
        osg::ref_ptr<osg::Material> materialRoadmark_ = GetOrCreateMaterial("RoadMark_" + roadmanager::LaneRoadMark::RoadMarkColor2Str(color),
                                                                            color_array->back(),
                                                                            static_cast<uint8_t>(RoadGeom::MaterialType::ROADMARK));

        // also embed color in geometry, e.g. for post processing in full stack simulations
        geom->setColorArray(color_array.get());
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);

        if (fade > SMALL_NUMBER)
        {
            geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        }

        geode->getOrCreateStateSet()->setAttributeAndModes(materialRoadmark_.get());

        osg::ref_ptr<osg::Texture2D> tex_roadmark;
        if (!SE_Env::Inst().GetOptions().GetOptionSet("generate_without_textures"))
        {
            // set texture coordinates anyway, for potential post processing
            osg::ref_ptr<osg::Vec2Array> texcoords   = new osg::Vec2Array(indices.get()->getNumIndices());
            double                       rm_texscale = 1.0 / ROADMARK_TEXTURE_SCALE;
            for (unsigned int i = 0; i < indices.get()->getNumIndices(); i++)
            {
                (*texcoords)[i].set(osg::Vec2(static_cast<float>(rm_texscale * ((*vertices)[(*indices)[i]][0])),
                                              static_cast<float>(rm_texscale * ((*vertices)[(*indices)[i]][1]))));
            }
            geom->setTexCoordArray(0, texcoords.get());

            tex_roadmark = ReadTexture("roadmark.jpg");
            if (tex_roadmark)
            {
                // set color to white for texture mapping. but keep alpha
                color_array->back()[0] = 1.0f;
                color_array->back()[1] = 1.0f;
                color_array->back()[2] = 1.0f;
                geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex_roadmark.get());
            }
        }

        osgUtil::SmoothingVisitor::smooth(*geom, 0.0);
        geode->addDrawable(geom.get());
        geode->setName(prefix_roadmark + std::to_string(rm_group->getNumChildren()));
        rm_group->addChild(geode);
    }

    int RoadGeom::AddRoadMarks(roadmanager::Lane* lane, osg::Group* rm_group, const osg::Vec3d& origin)
    {
        for (unsigned int i = 0; i < lane->GetNumberOfRoadMarks(); i++)
        {
            roadmanager::LaneRoadMark* lane_roadmark = lane->GetLaneRoadMarkByIdx(static_cast<int>(i));

            if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::NONE_TYPE)
            {
                continue;
            }

            for (unsigned int m = 0; m < lane_roadmark->GetNumberOfRoadMarkTypes(); m++)
            {
                roadmanager::LaneRoadMarkType* lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(m);

                for (unsigned int n = 0; n < lane_roadmarktype->GetNumberOfRoadMarkTypeLines(); n++)
                {
                    roadmanager::LaneRoadMarkTypeLine* lane_roadmarktypeline = lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(n);
                    roadmanager::OSIPoints*            curr_osi_rm           = lane_roadmarktypeline->GetOSIPoints();

                    if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BOTTS_DOTS)
                    {
                        for (unsigned int q = 0; q < curr_osi_rm->GetPoints().size(); q++)
                        {
                            const double                    botts_dot_size = 0.15;
                            static osg::ref_ptr<osg::Geode> dot            = 0;

                            if (dot == 0)
                            {
                                osg::ref_ptr<osg::TessellationHints> th = new osg::TessellationHints();
                                th->setDetailRatio(0.3f);
                                osg::ref_ptr<osg::ShapeDrawable> shape =
                                    new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0, 0.0, 0.0),
                                                                             static_cast<float>(botts_dot_size),
                                                                             0.3f * static_cast<float>(botts_dot_size)),
                                                           th);
                                shape->setColor(ODR2OSGColor(lane_roadmark->GetColor()));
                                dot = new osg::Geode;
                                dot->addDrawable(shape);
                            }

                            roadmanager::PointStruct osi_point0 = curr_osi_rm->GetPoint(q);

                            osg::ref_ptr<osg::PositionAttitudeTransform> tx = new osg::PositionAttitudeTransform;
                            tx->setPosition(osg::Vec3(static_cast<float>(osi_point0.x - origin[0]),
                                                      static_cast<float>(osi_point0.y - origin[1]),
                                                      static_cast<float>(osi_point0.z + ROADMARK_Z_OFFSET)));
                            tx->addChild(dot);
                            tx->setName(prefix_roadmark + std::to_string(rm_group->getNumChildren()));
                            rm_group->addChild(tx);
                        }
                    }
                    else
                    {
                        std::vector<roadmanager::PointStruct> osi_points = curr_osi_rm->GetPoints();

                        if (osi_points.size() < 2)
                        {
                            // No line - skip
                            continue;
                        }

                        double l0p0l[2] = {0.0, 0.0};  // previous line, startpoint, left side
                        double l0p0r[2] = {0.0, 0.0};  // previous line, startpoint, right side
                        double l0p1l[2] = {0.0, 0.0};  // previous line, endpoint, left side
                        double l0p1r[2] = {0.0, 0.0};  // previous line, endpoint, right side
                        double l1p0l[2] = {0.0, 0.0};  // current line, startpoint, left side
                        double l1p0r[2] = {0.0, 0.0};  // current line, startpoint, right side
                        double l1p1l[2] = {0.0, 0.0};  // current line, endpoint, left side
                        double l1p1r[2] = {0.0, 0.0};  // current line, endpoint, right side

                        osg::ref_ptr<osg::Vec3Array>        vertices;
                        osg::ref_ptr<osg::DrawElementsUInt> indices;

                        unsigned int startpoint = 0;

                        for (unsigned int q = 0; q < static_cast<unsigned int>(osi_points.size()); q++)
                        {
                            // Find offset points of solid roadmark at each OSI point
                            // each line has two points, beginning and end
                            // from each point one left and one right point will be calculated based on width of marking
                            // l1 is current line, l0 is previous

                            if (q == startpoint)
                            {
                                vertices = new osg::Vec3Array();
                                indices  = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP);
                            }

                            if (q < osi_points.size() - 1)
                            {
                                OffsetVec2D(osi_points[q].x - origin[0],
                                            osi_points[q].y - origin[1],
                                            osi_points[q + 1].x - origin[0],
                                            osi_points[q + 1].y - origin[1],
                                            -lane_roadmarktypeline->GetWidth() / 2,
                                            l1p0l[0],
                                            l1p0l[1],
                                            l1p1l[0],
                                            l1p1l[1]);
                                OffsetVec2D(osi_points[q].x - origin[0],
                                            osi_points[q].y - origin[1],
                                            osi_points[q + 1].x - origin[0],
                                            osi_points[q + 1].y - origin[1],
                                            lane_roadmarktypeline->GetWidth() / 2,
                                            l1p0r[0],
                                            l1p0r[1],
                                            l1p1r[0],
                                            l1p1r[1]);
                            }
                            else if (!osi_points[q].endpoint)
                            {
                                LOG_ERROR("Unexpected last point without endpoint q {}", q);
                            }

                            if (q == startpoint)
                            {
                                // First point in a line sequence, no adjustment needed
                                (*vertices).push_back(osg::Vec3(static_cast<float>(l1p0l[0]),
                                                                static_cast<float>(l1p0l[1]),
                                                                static_cast<float>(osi_points[q].z + ROADMARK_Z_OFFSET)));
                                (*vertices).push_back(osg::Vec3(static_cast<float>(l1p0r[0]),
                                                                static_cast<float>(l1p0r[1]),
                                                                static_cast<float>(osi_points[q].z + ROADMARK_Z_OFFSET)));
                            }
                            else if (osi_points[q].endpoint)
                            {
                                // Last point of a line sequence, no adjustment needed
                                double* left  = (q < osi_points.size() - 1) ? l1p0l : l1p1l;
                                double* right = (q < osi_points.size() - 1) ? l1p0r : l1p1r;
                                (*vertices).push_back(osg::Vec3(static_cast<float>(left[0]),
                                                                static_cast<float>(left[1]),
                                                                static_cast<float>(osi_points[q].z + ROADMARK_Z_OFFSET)));
                                (*vertices).push_back(osg::Vec3(static_cast<float>(right[0]),
                                                                static_cast<float>(right[1]),
                                                                static_cast<float>(osi_points[q].z + ROADMARK_Z_OFFSET)));
                            }
                            else
                            {
                                // Find intersection of non parallel lines
                                double isect[2];

                                if (GetIntersectionOfTwoLineSegments(l0p0l[0],
                                                                     l0p0l[1],
                                                                     l0p1l[0],
                                                                     l0p1l[1],
                                                                     l1p0l[0],
                                                                     l1p0l[1],
                                                                     l1p1l[0],
                                                                     l1p1l[1],
                                                                     isect[0],
                                                                     isect[1]) == 0)
                                {
                                    (*vertices).push_back(osg::Vec3(static_cast<float>(isect[0]),
                                                                    static_cast<float>(isect[1]),
                                                                    static_cast<float>(osi_points[q].z + ROADMARK_Z_OFFSET)));
                                }
                                else
                                {
                                    // lines parallel, no adjustment needed
                                    (*vertices).push_back(osg::Vec3(static_cast<float>(l1p0l[0]),
                                                                    static_cast<float>(l1p0l[1]),
                                                                    static_cast<float>(osi_points[q].z + ROADMARK_Z_OFFSET)));
                                }

                                if (GetIntersectionOfTwoLineSegments(l0p0r[0],
                                                                     l0p0r[1],
                                                                     l0p1r[0],
                                                                     l0p1r[1],
                                                                     l1p0r[0],
                                                                     l1p0r[1],
                                                                     l1p1r[0],
                                                                     l1p1r[1],
                                                                     isect[0],
                                                                     isect[1]) == 0)
                                {
                                    (*vertices).push_back(osg::Vec3(static_cast<float>(isect[0]),
                                                                    static_cast<float>(isect[1]),
                                                                    static_cast<float>(osi_points[q].z + ROADMARK_Z_OFFSET)));
                                }
                                else
                                {
                                    // lines parallel, no adjustment needed
                                    (*vertices).push_back(osg::Vec3(static_cast<float>(l1p0r[0]),
                                                                    static_cast<float>(l1p0r[1]),
                                                                    static_cast<float>(osi_points[q].z + ROADMARK_Z_OFFSET)));
                                }
                            }

                            if (q < osi_points.size() - 1)
                            {
                                // Shift points one step forward
                                memcpy(l0p0l, l1p0l, sizeof(l0p0l));
                                memcpy(l0p0r, l1p0r, sizeof(l0p0r));
                                memcpy(l0p1l, l1p1l, sizeof(l0p0l));
                                memcpy(l0p1r, l1p1r, sizeof(l0p0r));
                            }

                            // Set indices
                            (*indices).push_back(static_cast<unsigned int>(2 * (q - startpoint)));
                            (*indices).push_back(static_cast<unsigned int>(2 * (q - startpoint) + 1));

                            if (osi_points[q].endpoint)
                            {
                                // create and add OSG geometry for the line sequence
                                AddRoadMarkGeom(vertices, indices, rm_group, lane_roadmarktypeline->GetColor(), lane_roadmark->GetFade());
                                startpoint = q + 1;
                            }
                        }
                    }
                }
            }
        }

        return 0;
    }

    RoadGeom::RoadGeom(roadmanager::OpenDrive* odr,
                       osg::Vec3d              origin,
                       bool                    generate_road_surface,
                       bool                    generate_road_objects,
                       bool                    add_ground_plane,
                       std::string             exe_path,
                       bool                    optimize)
        : optimize_(optimize)
    {
        if (!generate_road_surface && !generate_road_objects)
        {
            return;
        }

        root_                             = new osg::Group;
        osg::ref_ptr<osg::Group> r_group_ = new osg::Group;
        r_group_->setName("roads");
        osg::ref_ptr<osg::Group> rm_group_ = new osg::Group;
        rm_group_->setName("roadmarks");
        root_->setName("esmini_generated_road_model");
        root_->addChild(rm_group_);
        root_->addChild(r_group_);

        origin      = origin;
        odrManager_ = odr;

        if (generate_road_surface)
        {
            LOG_INFO("Generating a simplistic 3D model of the road network");

            if (!SE_Env::Inst().GetOptions().GetOptionSet("generate_without_textures"))
            {
                texture_map_[MaterialType::ASPHALT] = ReadTexture("asphalt.jpg");
                texture_map_[MaterialType::GRASS]   = ReadTexture("grass.jpg");
            }

            osg::ref_ptr<osg::Vec4Array> color_concrete     = new osg::Vec4Array;
            osg::ref_ptr<osg::Vec4Array> color_border_inner = new osg::Vec4Array;
            osg::ref_ptr<osg::Vec4Array> color_grass        = new osg::Vec4Array;

            if (auto it = texture_map_.find(MaterialType::ASPHALT); it != texture_map_.end() && it->second != nullptr)  // We have a texture
            {
                color_asphalt_->push_back(osg::Vec4(1.f, 1.f, 1.f, 1.0f));
            }
            else  // No texture, use default color
            {
                color_asphalt_->push_back(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
            }

            if (auto it = texture_map_.find(MaterialType::GRASS); it != texture_map_.end() && it->second != nullptr)  // We have a texture
            {
                color_grass->push_back(osg::Vec4(1.f, 1.f, 1.f, 1.0f));
            }
            else  // No texture, use default color
            {
                color_grass->push_back(osg::Vec4(0.25f, 0.5f, 0.35f, 1.0f));
            }

            color_concrete->push_back(osg::Vec4(0.61f, 0.61f, 0.61f, 1.0f));
            color_border_inner->push_back(osg::Vec4(0.45f, 0.45f, 0.45f, 1.0f));

            // algorithm:
            // for each road and lane section:
            // - establish first point of each lane at s value = 0, set to current
            // - loop until reaching end of lane section:
            //   - for each lane:
            //     - find next OSI point along the lane, from the current section s value
            //       - register s value as lane current and as candidate section current
            //   - sort the list of section s value candidates
            //   - for each candidate:
            //     - for each lane:
            //       - calculate point at candidate s value
            //       - measure error from tangent of current section s-value point
            //       - if error is too large:
            //         - break
            //       - else, if error is OK:
            //         - register as new current section s value
            //     - if no OK point was found, pick the first candiate (lowest s-value)
            //   - establish points for all lanes at this s-value

            for (size_t i = 0; i < static_cast<unsigned int>(odr->GetNumOfRoads()); i++)
            {
                roadmanager::Road* road = odr->GetRoadByIdx(static_cast<int>(i));

                for (size_t j = 0; j < static_cast<unsigned int>(road->GetNumberOfLaneSections()); j++)
                {
                    roadmanager::LaneSection* lsec = road->GetLaneSectionByIdx(static_cast<int>(j));
                    if (lsec->GetNumberOfLanes() < 2)
                    {
                        // need at least reference lane plus another lane to form a road geometry
                        continue;
                    }

                    // First make sure there are OSI points of the center lane
                    roadmanager::Lane* lane = lsec->GetLaneById(0);
                    if (lane->GetOSIPoints() == 0)
                    {
                        LOG_ERROR("Missing OSI points of centerlane road {} section {}", road->GetId(), j);
                        throw std::runtime_error("Missing OSI points");
                    }

                    // create a 2d list of positions for vertices, nr_of_s-values x nr_of_lanes
                    typedef struct
                    {
                        double x;
                        double y;
                        double z;
                        double h;
                        double slope;
                        double s;
                    } GeomPoint;

                    typedef struct
                    {
                        int    geom_point_index;
                        double friction;
                    } GeomStrip;  // could be multiple of these per lane

                    struct GeomCacheEntry
                    {
                        GeomPoint point;
                        double    friction = 1.0;
                    };

                    struct CandidatePos
                    {
                        double x;
                        double y;
                    };

                    std::vector<std::vector<GeomPoint>> geom_points_list;                              // one list of points per lane
                    std::vector<std::vector<GeomStrip>> geom_strips_list;                              // one list of strips info per lane
                    std::vector<GeomCacheEntry>         geom_cache(lsec->GetNumberOfLanes());          // one cache entry per lane
                    std::vector<int>                    lane_osi_index(lsec->GetNumberOfLanes());      // current osi point per lane
                    std::vector<double>                 s_value_candidates(lsec->GetNumberOfLanes());  // candidates for next current s-value
                    std::vector<CandidatePos>           candidates_pos(lsec->GetNumberOfLanes());      // candidates for next current s-value
                    double                              section_current_s = lsec->GetS();

                    roadmanager::Position pos;  // used for calculating points along the road

                    // First populate s values of the material elements
                    //   - for each material a new friction segment is to be added
                    //   - loop over material friction segments, insert new vertices if needed
                    std::vector<double> friction_s_list;
                    for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
                    {
                        lane = lsec->GetLaneByIdx(k);
                        for (size_t l = 0; l < lane->GetNumberOfMaterials(); l++)
                        {
                            friction_s_list.push_back(lsec->GetS() + lane->GetMaterialByIdx(l)->s_offset);
                        }
                    }

                    // sort friction s-values and remove duplicates
                    std::sort(friction_s_list.begin(), friction_s_list.end());
                    friction_s_list.erase(std::unique(friction_s_list.begin(), friction_s_list.end(), compare_s_values), friction_s_list.end());

                    // collect a list of s values where vertices are needed, considering all lanes
                    int                   friction_s_list_index = friction_s_list.size() > 0 ? 1 : -1;
                    bool                  done_section          = false;
                    roadmanager::Position pos2;

                    for (int counter = 0; !done_section; counter++)
                    {
                        if (counter == 0)
                        {
                            // First add s = start of lane section, to set start of mesh
                            done_section = false;
                            for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
                            {
                                lane_osi_index[k]     = 0;
                                s_value_candidates[k] = lsec->GetS();
                            }
                        }
                        else
                        {
                            // for each lane, find next s-value in and register it as candidate section current s-value
                            for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
                            {
                                lane                                            = lsec->GetLaneByIdx(static_cast<int>(k));
                                std::vector<roadmanager::PointStruct> osiPoints = lane->GetOSIPoints()->GetPoints();

                                for (size_t l = lane_osi_index[k]; l < osiPoints.size(); l++)
                                {
                                    if (l == osiPoints.size() - 1 || osiPoints[l].s > section_current_s + SMALL_NUMBER)
                                    {
                                        lane_osi_index[k]     = l;
                                        s_value_candidates[k] = osiPoints[l].s;

                                        // generate point at osi index s-value
                                        lane = lsec->GetLaneByIdx(static_cast<int>(k));
                                        pos2.SetTrackPos(road->GetId(),
                                                         s_value_candidates[k],
                                                         SIGN(lane->GetId()) * lsec->GetOuterOffset(s_value_candidates[k], lane->GetId()),
                                                         true);
                                        candidates_pos[k].x = pos2.GetX();
                                        candidates_pos[k].y = pos2.GetY();

                                        break;
                                    }
                                }
                            }

                            // sort candidates
                            std::sort(s_value_candidates.begin(), s_value_candidates.end());

                            // find highest s-value not exceeding the tolerated error, over all lanes
                            size_t k = 0;
                            for (; k < s_value_candidates.size(); k++)
                            {
                                size_t l = 0;
                                for (; l < static_cast<unsigned int>(lsec->GetNumberOfLanes()); l++)
                                {
                                    lane = lsec->GetLaneByIdx(static_cast<int>(l));

                                    // generate point at pivot s-value
                                    double t = road->GetLaneOffset(s_value_candidates[k]) +
                                               SIGN(lane->GetId()) * lsec->GetOuterOffset(s_value_candidates[k], lane->GetId());
                                    pos.SetTrackPos(road->GetId(), s_value_candidates[k], t, true);

                                    // calculate horizontal error at this s value
                                    // find out heading of the previous calculated vertex point
                                    double h = lane_osi_index[l] > 0 ? GetAngleOfVector(candidates_pos[l].x - geom_cache[l].point.x,
                                                                                        candidates_pos[l].y - geom_cache[l].point.y)
                                                                     : geom_cache[l].point.h;
                                    double error_horizontal =
                                        DistanceFromPointToLine2DWithAngle(pos.GetX(), pos.GetY(), geom_cache[l].point.x, geom_cache[l].point.y, h);

                                    // calculate vertical error at this s value
                                    double error_vertical =
                                        abs((pos.GetZ() - geom_cache[l].point.z) - geom_cache[l].point.slope * (pos.GetS() - geom_cache[l].point.s));

                                    if (error_horizontal > MAX_GEOM_ERROR || error_vertical > MAX_GEOM_ERROR)
                                    {
                                        break;
                                    }
                                }

                                if (l == static_cast<unsigned int>(lsec->GetNumberOfLanes()))
                                {
                                    // no error, register preliminary section current s value
                                    section_current_s = s_value_candidates[k];
                                }
                                else
                                {
                                    // error too large, stop searching
                                    if (k == 0)
                                    {
                                        // no candidate was OK, pick the first one
                                        section_current_s = s_value_candidates[k];
                                    }
                                    break;
                                }

                                // we have s-value of a OSI point, check if there is a new friction value before that
                                // also check for maximum length
                                double s_next_friction     = (friction_s_list_index > -1 && friction_s_list_index < friction_s_list.size())
                                                                 ? friction_s_list[friction_s_list_index]
                                                                 : lsec->GetS() + lsec->GetLength();
                                double s_next_geom_max_len = geom_cache[k].point.s + MAX_GEOM_LENGTH;

                                if (s_next_friction < section_current_s &&
                                    s_next_friction < s_next_geom_max_len + MIN_GEOM_LENGTH)  // add min geom len to avoid mini patches
                                {
                                    section_current_s = s_next_friction;
                                    friction_s_list_index++;
                                    break;
                                }
                                else if (s_next_geom_max_len < section_current_s - SMALL_NUMBER &&
                                         s_next_geom_max_len + MIN_GEOM_LENGTH < s_next_friction)  // add min geom len to avoid mini patches
                                {
                                    section_current_s = s_next_geom_max_len;
                                    break;
                                }
                            }
                        }

                        if (section_current_s > lsec->GetS() + lsec->GetLength() - SMALL_NUMBER)
                        {
                            done_section = true;
                        }

                        // s-value for next point established, create vertices for each lane
                        for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
                        {
                            roadmanager::Lane::Material* mat            = nullptr;
                            int                          lane_id        = lsec->GetLaneIdByIdx(static_cast<int>(k));
                            int                          friction_index = k;

                            if (k > 0)  // skip friction for first vertex strip (leftmost outer lane boundary)
                            {
                                // For friction we need to work from left to right. For left lanes, it means shifting friction one lane right
                                if (lane_id >= 0)
                                {
                                    friction_index = k - 1;
                                }
                            }
                            else
                            {
                                friction_index = lsec->GetLaneIdxById(0);
                            }

                            roadmanager::Lane* lane_for_friction;
                            lane_for_friction = lsec->GetLaneByIdx(static_cast<int>(friction_index));
                            mat               = lane_for_friction->GetMaterialByS(section_current_s - lsec->GetS());
                            double friction   = mat != nullptr ? mat->friction : FRICTION_DEFAULT;

                            // retrieve position at s-value
                            double t = road->GetLaneOffset(section_current_s) + SIGN(lane_id) * lsec->GetOuterOffset(section_current_s, lane_id);
                            pos.SetTrackPos(road->GetId(), section_current_s, t, true);
                            GeomPoint gp = {pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), pos.GetZRoadPrim(), pos.GetS()};

                            if (counter == 0)
                            {
                                // add geometry and strip list for the lane to
                                std::vector<GeomPoint> geom_points;
                                geom_points_list.push_back(geom_points);

                                std::vector<GeomStrip> geom_strips;
                                geom_strips_list.push_back(geom_strips);
                            }

                            if (counter == 0 || !NEAR_NUMBERS(friction, geom_cache[k].friction))
                            {
                                // create initial strip or strip with new friction value
                                geom_strips_list[k].push_back({static_cast<int>(geom_points_list[k].size()), friction});
                            }

                            geom_points_list[k].push_back(gp);

                            if (geom_cache.size() <= k)
                            {
                                geom_cache.push_back({{pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), pos.GetZRoadPrim(), pos.GetS()}, friction});
                            }
                            else
                            {
                                geom_cache[k].point    = {pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), pos.GetZRoadPrim(), pos.GetS()};
                                geom_cache[k].friction = friction;
                            }
                        }
                    }

                    // Then create actual vertices and triangle strips for the lane section
                    // Each strip is made of two lanes, so we need to create a separate geometry for each pair of lanes
                    // Also within each lane, we need to create a separate geometry for each material segment
                    unsigned int nr_vertices =
                        static_cast<unsigned int>(geom_points_list[0].size() * geom_strips_list.size());  // same nr vertices in all lanes
                    osg::ref_ptr<osg::Vec3Array> verticesAll  = new osg::Vec3Array(nr_vertices);
                    osg::ref_ptr<osg::Vec2Array> texcoordsAll = new osg::Vec2Array(nr_vertices);

                    // Potential optimization: Swap loops, creating all vertices for same s-value for each step
                    int vertex_index_left_local_next = 0;
                    int vertex_idx_all               = 0;

                    for (size_t k = 0; k < geom_strips_list.size(); k++)  // loop over lanes
                    {
                        osg::ref_ptr<osg::Vec3Array>        verticesLocal;
                        osg::ref_ptr<osg::Vec2Array>        texcoordsLocal;
                        osg::ref_ptr<osg::Vec4Array>        colorLocal;
                        osg::ref_ptr<osg::DrawElementsUInt> indices;
                        lane                               = lsec->GetLaneByIdx(static_cast<int>(k));
                        roadmanager::Lane* laneForMaterial = nullptr;

                        int vertex_index_left_local  = vertex_index_left_local_next;
                        int vertex_index_right_local = vertex_idx_all;
                        vertex_index_left_local_next = vertex_index_right_local;

                        for (size_t m = 0; m < geom_strips_list[k].size(); m++)  // loop over lane patches with constant friction
                        {
                            lane_friction_                      = geom_strips_list[k][m].friction;
                            unsigned int            gpi         = geom_strips_list[k][m].geom_point_index;
                            std::vector<GeomPoint>& geom_points = geom_points_list[k];
                            unsigned int            n_points    = 0;

                            if (m < geom_strips_list[k].size() - 1)
                            {
                                n_points = geom_strips_list[k][m + 1].geom_point_index - gpi + 1;  // +
                            }
                            else
                            {
                                n_points = geom_points.size() - gpi;
                            }

                            if (k > 0)
                            {
                                verticesLocal   = new osg::Vec3Array(static_cast<unsigned int>(n_points * 2));
                                indices         = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, static_cast<unsigned int>(n_points * 2));
                                texcoordsLocal  = new osg::Vec2Array(static_cast<unsigned int>(n_points * 2));
                                laneForMaterial = lsec->GetLaneByIdx(lane->GetId() < 0 ? static_cast<int>(k) : static_cast<int>(k) - 1);
                            }

                            int index_counter = 0;

                            for (size_t l = 0; l < n_points; l++)
                            {
                                if (m == 0 || l > 0)
                                {
                                    GeomPoint& gp = geom_points[gpi + l];
                                    (*verticesAll)[static_cast<unsigned int>(vertex_idx_all)].set(static_cast<float>(gp.x - origin[0]),
                                                                                                  static_cast<float>(gp.y - origin[1]),
                                                                                                  static_cast<float>(gp.z));
                                    double texscale = 1.0 / TEXTURE_SCALE;
                                    (*texcoordsAll)[static_cast<unsigned int>(vertex_idx_all)].set(
                                        osg::Vec2(static_cast<float>(texscale * (gp.x - origin[0])),
                                                  static_cast<float>(texscale * (gp.y - origin[1]))));
                                    vertex_idx_all++;
                                }
                                else
                                {
                                    vertex_index_right_local--;  // reuse previous vertex
                                    vertex_index_left_local--;   // reuse previous vertex
                                }

                                // Create indices for the lane strip, referring to the vertex list
                                if (k > 0)
                                {
                                    // vertex of left lane border
                                    (*verticesLocal)[static_cast<unsigned int>(index_counter)]  = (*verticesAll)[vertex_index_left_local];
                                    (*texcoordsLocal)[static_cast<unsigned int>(index_counter)] = (*texcoordsAll)[vertex_index_left_local];
                                    (*indices)[index_counter]                                   = static_cast<unsigned int>(index_counter);
                                    if (l < geom_points.size() - 1)
                                    {
                                        vertex_index_left_local++;
                                    }
                                    index_counter++;

                                    // vertex of right
                                    (*verticesLocal)[static_cast<unsigned int>(index_counter)]  = (*verticesAll)[vertex_index_right_local];
                                    (*texcoordsLocal)[static_cast<unsigned int>(index_counter)] = (*texcoordsAll)[vertex_index_right_local];
                                    (*indices)[index_counter]                                   = static_cast<unsigned int>(index_counter);
                                    if (l < geom_points.size() - 1)
                                    {
                                        vertex_index_right_local++;
                                    }
                                    index_counter++;
                                }
                            }

                            if (k != 0)
                            {
                                // Create geometry for the strip made of this and previous lane
                                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                                geom->setUseDisplayList(true);
                                geom->setVertexArray(verticesLocal.get());
                                geom->addPrimitiveSet(indices.get());
                                geom->setTexCoordArray(0, texcoordsLocal.get());
                                osgUtil::SmoothingVisitor::smooth(*geom, 0.5);

                                MaterialType material_t;

                                if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_ANY_ROAD))
                                {
                                    material_t = MaterialType::ASPHALT;
                                    osg::ref_ptr<osg::Material> materialAsphalt_ =
                                        GetOrCreateMaterial("Asphalt", GetFrictionColor(lane_friction_), static_cast<uint8_t>(material_t), 1);

                                    geom->getOrCreateStateSet()->setAttributeAndModes(materialAsphalt_.get());
                                }
                                else if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BIKING) ||
                                         laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_SIDEWALK))
                                {
                                    material_t = MaterialType::CONCRETE;
                                    osg::ref_ptr<osg::Material> materialConcrete_ =
                                        GetOrCreateMaterial("Concrete", color_concrete->at(0), static_cast<uint8_t>(material_t));
                                    geom->getOrCreateStateSet()->setAttributeAndModes(materialConcrete_.get());

                                    // Use PolygonOffset feature to avoid z-fighting with road surface
                                    geom->getOrCreateStateSet()->setAttributeAndModes(
                                        new osg::PolygonOffset(-POLYGON_OFFSET_SIDEWALK, -SIGN(POLYGON_OFFSET_SIDEWALK)));
                                }
                                else if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BORDER) && k != 1 &&
                                         k != static_cast<unsigned int>(lsec->GetNumberOfLanes()) - 1)
                                {
                                    material_t = MaterialType::BORDER;
                                    osg::ref_ptr<osg::Material> materialBorderInner_ =
                                        GetOrCreateMaterial("Border", color_border_inner->at(0), static_cast<uint8_t>(material_t));
                                    geom->getOrCreateStateSet()->setAttributeAndModes(materialBorderInner_.get());

                                    // Use PolygonOffset feature to avoid z-fighting with road surface
                                    geom->getOrCreateStateSet()->setAttributeAndModes(
                                        new osg::PolygonOffset(-POLYGON_OFFSET_BORDER, -SIGN(POLYGON_OFFSET_BORDER)));
                                }
                                else
                                {
                                    material_t = MaterialType::GRASS;
                                    osg::ref_ptr<osg::Material> materialGrass_ =
                                        GetOrCreateMaterial("Grass", color_grass->at(0), static_cast<uint8_t>(material_t));

                                    geom->getOrCreateStateSet()->setAttributeAndModes(materialGrass_.get());

                                    // Use PolygonOffset feature to avoid z-fighting with road surface
                                    geom->getOrCreateStateSet()->setAttributeAndModes(
                                        new osg::PolygonOffset(-POLYGON_OFFSET_GRASS, -SIGN(POLYGON_OFFSET_GRASS)));
                                }
                                geom->setColorBinding(osg::Geometry::BIND_OVERALL);

                                // See if the material type has a texture associated with it, if so, apply it
                                auto texture_it = texture_map_.find(material_t);
                                if (texture_it != texture_map_.end())
                                {
                                    geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture_it->second.get());
                                }

                                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                                geode->addDrawable(geom.get());

                                // osgUtil::Optimizer optimizer;
                                // optimizer.optimize(geode);

                                geode->setName(prefix_road + std::to_string(road->GetId()) + "_" + std::to_string(k));
                                r_group_->addChild(geode);
                            }
                        }
                        AddRoadMarks(lane, rm_group_, origin);
                    }
                }
            }
        }

        if (generate_road_objects)
        {
            if (odrManager_->GetNumOfRoads() > 0 && CreateRoadSignsAndObjects(odrManager_, origin, generate_road_surface, root_, exe_path) != 0)
            {
                LOG_ERROR("Viewer::Viewer Failed to create road signs and objects!");
            }
        }
    }

    int RoadGeom::CreateRoadSignsAndObjects(roadmanager::OpenDrive*  od,
                                            const osg::Vec3d&        origin,
                                            bool                     stand_in_model,
                                            osg::ref_ptr<osg::Group> parent,
                                            std::string              exe_path)
    {
        osg::ref_ptr<osg::Group>                     objGroup = new osg::Group;
        osg::ref_ptr<osg::PositionAttitudeTransform> tx       = nullptr;

        objGroup->setName("road_objects");

        roadmanager::Position pos;

        for (unsigned int r = 0; r < od->GetNumOfRoads(); r++)
        {
            roadmanager::Road* road = od->GetRoadByIdx(r);

            for (unsigned int s = 0; s < road->GetNumberOfSignals(); s++)
            {
                osg::ref_ptr<osg::Group> signGroup = new osg::Group;
                signGroup->setName(prefix_road_signal + std::to_string(s));
                tx                          = nullptr;
                roadmanager::Signal* signal = road->GetSignal(s);

                // create a bounding for the sign
                osg::ref_ptr<osg::PositionAttitudeTransform> tx_bb = new osg::PositionAttitudeTransform;
                signGroup->addChild(tx_bb);

                // avoid zero width, length and width - set to a minimum value of 0.05m
                osg::ref_ptr<osg::ShapeDrawable> shape =
                    new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.5f * MAX(0.05f, static_cast<float>(signal->GetHeight()))),
                                                        MAX(0.05f, static_cast<float>(signal->GetDepth())),
                                                        MAX(0.05f, static_cast<float>(signal->GetWidth())),
                                                        MAX(0.05f, static_cast<float>(signal->GetHeight()))));

                shape->setColor(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
                tx_bb->addChild(shape);
                tx_bb->setPosition(osg::Vec3(static_cast<float>(signal->GetX() - origin[0]),
                                             static_cast<float>(signal->GetY() - origin[1]),
                                             static_cast<float>(signal->GetZ() + signal->GetZOffset())));
                tx_bb->setAttitude(osg::Quat(signal->GetH() + signal->GetHOffset(), osg::Vec3(0, 0, 1)));

                if (stand_in_model == true || !SE_Env::Inst().GetOptions().GetOptionSet("use_signs_in_external_model"))
                {
                    // Road sign filename is the combination of type_subtype_value
                    std::string filename = signal->GetCountry() + "_" + signal->GetType();
                    if (!(signal->GetSubType().empty() || signal->GetSubType() == "none" || signal->GetSubType() == "-1"))
                    {
                        filename += "_" + signal->GetSubType();
                    }

                    if (!NEAR_NUMBERS(signal->GetValue(), -1.0))
                    {
                        filename += "-" + signal->GetValueStr();
                    }
                    tx = LoadRoadFeature(road, filename + ".osgb", exe_path);

                    if (tx == nullptr)
                    {
                        // if file according to type, subtype and value could not be resolved, try from name
                        tx = LoadRoadFeature(road, signal->GetName() + ".osgb", exe_path);
                    }

                    if (tx != nullptr)
                    {
                        tx->setPosition(osg::Vec3(static_cast<float>(signal->GetX() - origin[0]),
                                                  static_cast<float>(signal->GetY() - origin[1]),
                                                  static_cast<float>(signal->GetZ() + signal->GetZOffset())));
                        tx->setAttitude(osg::Quat(signal->GetH() + signal->GetHOffset(), osg::Vec3(0, 0, 1)));
                        tx->setNodeMask(NODE_MASK_SIGN);
                        signGroup->addChild(tx);
                    }
                    else
                    {
                        LOG_DEBUG("Failed to load signal {}.osgb / {}.osgb - use simple bounding box", filename, signal->GetName());
                        osg::ref_ptr<osg::PositionAttitudeTransform> obj_standin =
                            dynamic_cast<osg::PositionAttitudeTransform*>(tx_bb->clone(osg::CopyOp::DEEP_COPY_ALL));
                        obj_standin->setNodeMask(NODE_MASK_SIGN);
                        signGroup->addChild(obj_standin);
                    }
                }

                // set bounding box to wireframe mode
                osg::PolygonMode* polygonMode = new osg::PolygonMode;
                polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
                shape->getOrCreateStateSet()->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
                tx_bb->setNodeMask(NODE_MASK_ODR_FEATURES);

                objGroup->addChild(signGroup);
            }

            for (unsigned int o = 0; o < road->GetNumberOfObjects(); o++)
            {
                roadmanager::RMObject* object = road->GetRoadObject(o);
                osg::Vec4              color;
                tx = nullptr;

                for (unsigned int c = 0; c < 4; c++)
                {
                    color[c] = object->GetColor()[c];
                }

                if (object->GetNumberOfOutlines() > 0 &&
                    object->GetNumberOfRepeats() == 0)  // if repeats are defined, wait and see if outline should replace failed 3D model or not
                {
                    for (size_t j = 0; j < static_cast<unsigned int>(object->GetNumberOfOutlines()); j++)
                    {
                        roadmanager::Outline* outline = object->GetOutline(j);
                        CreateOutlineObject(outline, color, origin, objGroup, object->GetId());
                    }
                    LOG_INFO("Created outline geometry for object {}.", object->GetName());
                    LOG_DEBUG("  if it looks strange, e.g.faces too dark or light color, ");
                    LOG_DEBUG("  check that corners are defined counter-clockwise (as OpenGL default).");
                }
                else
                {
                    double orientation = object->GetOrientation() == roadmanager::Signal::Orientation::NEGATIVE ? M_PI : 0.0;

                    // absolute path or relative to current directory
                    std::string filename = object->GetName();

                    // Assume name is representing a 3D model filename
                    if (!filename.empty())
                    {
                        if (FileNameExtOf(filename) == "")
                        {
                            filename += ".osgb";  // add missing extension
                        }

                        tx = LoadRoadFeature(road, filename, exe_path);

                        if (tx == nullptr)
                        {
                            LOG_WARN("Failed to load road object model file: {} ({}). Creating a bounding box as stand in.",
                                     filename,
                                     object->GetName());
                        }
                    }

                    roadmanager::Repeat*         rep     = object->GetRepeat();
                    int                          nCopies = 0;
                    double                       cur_s   = 0.0;
                    osg::ref_ptr<osg::Vec3Array> vertices_right_side;
                    osg::ref_ptr<osg::Vec3Array> vertices_left_side;
                    osg::ref_ptr<osg::Vec3Array> vertices_top;
                    osg::ref_ptr<osg::Vec3Array> vertices_bottom;
                    osg::ref_ptr<osg::Group>     group;

                    if (tx == nullptr)  // No model loaded
                    {
                        if (rep && rep->GetDistance() < SMALL_NUMBER)  //  non continuous objects
                        {
                            // use outline, if exists
                            if (object->GetNumberOfOutlines() > 0)
                            {
                                for (unsigned int j = 0; j < object->GetNumberOfOutlines(); j++)
                                {
                                    roadmanager::Outline* outline = object->GetOutline(j);
                                    CreateOutlineObject(outline, color, origin, objGroup, object->GetId());
                                }
                                continue;
                            }
                            else
                            {
                                // create stand in object
                                vertices_left_side  = new osg::Vec3Array;
                                vertices_right_side = new osg::Vec3Array;
                                vertices_top        = new osg::Vec3Array;
                                group               = new osg::Group();
                            }
                        }
                        else
                        {
                            // create a bounding box to represent the object
                            tx = new osg::PositionAttitudeTransform;

                            // avoid zero width, length and width - set to a minimum value of 0.05m
                            osg::ref_ptr<osg::ShapeDrawable> shape =
                                new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.5f * MAX(0.05f, static_cast<float>(object->GetHeight()))),
                                                                    MAX(0.05f, static_cast<float>(object->GetLength())),
                                                                    MAX(0.05f, static_cast<float>(object->GetWidth())),
                                                                    MAX(0.05f, static_cast<float>(object->GetHeight()))));

                            shape->setColor(color);
                            tx->addChild(shape);
                        }
                    }

                    double dim_x = 0.0;
                    double dim_y = 0.0;
                    double dim_z = 0.0;

                    osg::BoundingBox boundingBox;
                    if (tx != nullptr)
                    {
                        osg::ComputeBoundsVisitor cbv;
                        tx->accept(cbv);
                        boundingBox = cbv.getBoundingBox();

                        dim_x = boundingBox._max.x() - boundingBox._min.x();
                        dim_y = boundingBox._max.y() - boundingBox._min.y();
                        dim_z = boundingBox._max.z() - boundingBox._min.z();
                        if (object->GetLength() < SMALL_NUMBER && dim_x > SMALL_NUMBER)
                        {
                            LOG_WARN("Object {} missing length, set to bounding box length {:.2f}", object->GetName(), dim_x);
                            object->SetLength(dim_x);
                        }
                        if (object->GetWidth() < SMALL_NUMBER && dim_y > SMALL_NUMBER)
                        {
                            LOG_WARN("Object {} missing width, set to bounding box width {:.2f}", object->GetName(), dim_y);
                            object->SetWidth(dim_y);
                        }
                        if (object->GetHeight() < SMALL_NUMBER && dim_z > SMALL_NUMBER)
                        {
                            LOG_WARN("Object {} missing height, set to bounding box height {:.2f}", object->GetName(), dim_z);
                            object->SetHeight(dim_z);
                        }
                    }

                    double                   lastLODs = 0.0;  // used for putting object copies in LOD groups
                    osg::ref_ptr<osg::Group> LODGroup = 0;

                    osg::ref_ptr<osg::PositionAttitudeTransform> clone = 0;

                    for (; nCopies < 1 ||
                           (rep && rep->length_ > SMALL_NUMBER && cur_s < rep->GetLength() + SMALL_NUMBER && cur_s + rep->GetS() < road->GetLength());
                         nCopies++)
                    {
                        double s;
                        double scale_x = 1.0;
                        double scale_y = 1.0;
                        double scale_z = 1.0;

                        if (rep && cur_s + rep->GetS() + (object->GetLength() * scale_x) * cos(object->GetHOffset()) > road->GetLength())
                        {
                            break;  // object would reach outside specified total length
                        }

                        if (nCopies == 0)
                        {
                            // first object
                            clone = tx;
                        }
                        else
                        {
                            clone = tx != nullptr ? dynamic_cast<osg::PositionAttitudeTransform*>(tx->clone(osg::CopyOp::DEEP_COPY_ALL)) : nullptr;
                            clone->setDataVariance(osg::Object::STATIC);
                        }

                        if (rep == nullptr)
                        {
                            s = object->GetS();

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

                            // position mode relative for aligning to road heading
                            pos.SetTrackPosMode(road->GetId(),
                                                object->GetS(),
                                                object->GetT(),
                                                roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::Z_REL |
                                                    roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);

                            clone->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                            clone->setScale(osg::Vec3(static_cast<float>(scale_x), static_cast<float>(scale_y), static_cast<float>(scale_z)));

                            clone->setPosition(osg::Vec3(static_cast<float>(pos.GetX() - origin[0]),
                                                         static_cast<float>(pos.GetY() - origin[1]),
                                                         static_cast<float>(object->GetZOffset() + pos.GetZ())));

                            // First align to road orientation
                            osg::Quat quatRoad(osg::Quat(pos.GetR(), osg::X_AXIS, pos.GetP(), osg::Y_AXIS, pos.GetH(), osg::Z_AXIS));
                            // Specified local rotation
                            osg::Quat quatLocal(orientation + object->GetHOffset(), osg::Vec3(osg::Z_AXIS));  // Heading
                            // Combine
                            clone->setAttitude(quatLocal * quatRoad);
                        }
                        else  // repeated objects (separate or continuous)
                        {
                            double factor  = cur_s / rep->GetLength();
                            double t       = rep->GetTStart() + factor * (rep->GetTEnd() - rep->GetTStart());
                            s              = rep->GetS() + cur_s;
                            double zOffset = rep->GetZOffsetStart() + factor * (rep->GetZOffsetEnd() - rep->GetZOffsetStart());

                            // position mode relative for aligning to road heading
                            pos.SetTrackPosMode(road->GetId(),
                                                s,
                                                t,
                                                roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::Z_REL |
                                                    roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);

                            // Find angle based on delta t
                            double h_offset = atan2(rep->GetTEnd() - rep->GetTStart(), rep->GetLength());
                            pos.SetHeadingRelative(h_offset);

                            if (tx == nullptr && rep->GetDistance() < SMALL_NUMBER)  // one single continuous object to be created
                            {
                                // add two vertices at this s-value
                                double x   = 0.0;
                                double y   = rep->GetWidthStart() + factor * (rep->GetWidthEnd() - rep->GetWidthStart());
                                double z   = rep->GetHeightStart() + factor * (rep->GetHeightEnd() - rep->GetHeightStart());
                                double p0x = 0.0;
                                double p0y = 0.0;
                                double p1x = 0.0;
                                double p1y = 0.0;
                                RotateVec2D(x, y, pos.GetH(), p0x, p0y);
                                RotateVec2D(x, -y, pos.GetH(), p1x, p1y);

                                vertices_right_side->push_back(osg::Vec3d(pos.GetX() + p1x - origin[0], pos.GetY() + p1y - origin[1], pos.GetZ()));
                                vertices_right_side->push_back(
                                    osg::Vec3d(pos.GetX() + p1x - origin[0], pos.GetY() + p1y - origin[1], pos.GetZ() + z));
                                // add left vertices in reversed order, since they will be concatenated later in reversed order
                                vertices_left_side->push_back(osg::Vec3d(pos.GetX() + p0x - origin[0], pos.GetY() + p0y - origin[1], pos.GetZ() + z));
                                vertices_left_side->push_back(osg::Vec3d(pos.GetX() + p0x - origin[0], pos.GetY() + p0y - origin[1], pos.GetZ()));
                                vertices_top->push_back(osg::Vec3d(pos.GetX() + p0x - origin[0], pos.GetY() + p0y - origin[1], pos.GetZ() + z));
                                vertices_top->push_back(osg::Vec3d(pos.GetX() + p1x - origin[0], pos.GetY() + p1y - origin[1], pos.GetZ() + z));
                                vertices_bottom->push_back(osg::Vec3d(pos.GetX() + p0x - origin[0], pos.GetY() + p0y - origin[1], pos.GetZ()));
                                vertices_bottom->push_back(osg::Vec3d(pos.GetX() + p1x - origin[0], pos.GetY() + p1y - origin[1], pos.GetZ()));
                            }
                            else  // separate objects
                            {
                                if (rep->GetLengthStart() > SMALL_NUMBER || rep->GetLengthEnd() > SMALL_NUMBER)
                                {
                                    scale_x =
                                        ((rep->GetLengthStart() + factor * (rep->GetLengthEnd() - rep->GetLengthStart())) / cos(h_offset)) / dim_x;
                                }
                                else
                                {
                                    scale_x = (abs(h_offset) < M_PI_2 - SMALL_NUMBER) ? scale_x / cos(h_offset) : LARGE_NUMBER;
                                }
                                if (rep->GetWidthStart() > SMALL_NUMBER || rep->GetWidthEnd() > SMALL_NUMBER)
                                {
                                    scale_y = (rep->GetWidthStart() + factor * (rep->GetWidthEnd() - rep->GetWidthStart())) / dim_y;
                                }
                                if (rep->GetHeightStart() > SMALL_NUMBER || rep->GetHeightEnd() > SMALL_NUMBER)
                                {
                                    scale_z = (rep->GetHeightStart() + factor * (rep->GetHeightEnd() - rep->GetHeightStart())) / dim_z;
                                }

                                clone->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                                clone->setScale(osg::Vec3(static_cast<float>(scale_x), static_cast<float>(scale_y), static_cast<float>(scale_z)));
                                clone->setPosition(osg::Vec3(static_cast<float>(pos.GetX() - origin[0]),
                                                             static_cast<float>(pos.GetY() - origin[1]),
                                                             static_cast<float>(pos.GetZ() + zOffset)));

                                // First align to road orientation
                                osg::Quat quatRoad(osg::Quat(pos.GetR(), osg::X_AXIS, pos.GetP(), osg::Y_AXIS, pos.GetH(), osg::Z_AXIS));

                                // Specified local rotation
                                osg::Quat quatLocal(object->GetHOffset(), osg::Vec3(osg::Z_AXIS));  // Heading

                                // Combine
                                clone->setAttitude(quatLocal * quatRoad);
                            }

                            // increase current s according to distance
                            if (rep->distance_ > SMALL_NUMBER)
                            {
                                cur_s += rep->distance_;
                            }
                            else
                            {
                                // for continuous objects, move along s wrt to road curvature
                                cur_s += pos.DistanceToDS(object->GetLength() < SMALL_NUMBER
                                                              ? MIN(rep->GetLength(), DEFAULT_LENGTH_FOR_CONTINUOUS_OBJS)
                                                              : MIN(object->GetLength(), DEFAULT_LENGTH_FOR_CONTINUOUS_OBJS));
                            }
                        }

                        if (tx != nullptr)  // wait with continuous object
                        {
                            clone->setDataVariance(osg::Object::STATIC);
                            if (LODGroup == 0 || s - lastLODs > 0.5 * LOD_DIST_ROAD_FEATURES)
                            {
                                // add current LOD and create a new one
                                osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                                LODGroup                   = new osg::Group();
                                lod->addChild(LODGroup);
                                lod->setRange(
                                    0,
                                    0,
                                    LOD_DIST_ROAD_FEATURES + MAX(boundingBox.xMax() - boundingBox.xMin(), boundingBox.yMax() - boundingBox.yMin()));
                                objGroup->addChild(lod);
                                lastLODs = s;
                            }

                            LODGroup->addChild(clone);
                        }
                    }
                    if (tx == nullptr)
                    {
                        // Create geometry for continuous object
                        osg::ref_ptr<osg::Geode>    geode  = new osg::Geode;
                        osg::ref_ptr<osg::Geometry> geom[] = {new osg::Geometry, new osg::Geometry, new osg::Geometry};

                        // Concatenate vertices for right and left side into one single array going around the object counter clockwise
                        osg::ref_ptr<osg::Vec3Array> vertices = vertices_right_side;
                        vertices->insert(vertices->end(), vertices_left_side->rbegin(), vertices_left_side->rend());

                        // Finally, duplicate first set of vertices to the end in order to close the geometry
                        vertices->insert(vertices->end(), vertices_right_side->begin(), vertices_right_side->begin() + 2);

                        geom[0]->setVertexArray(vertices.get());
                        geom[0]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, static_cast<int>(vertices->size())));

                        // Add roof
                        geom[1]->setVertexArray(vertices_top.get());
                        geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, static_cast<int>(vertices_top->size())));

                        // Add floor
                        geom[2]->setVertexArray(vertices_bottom.get());
                        geom[2]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, static_cast<int>(vertices_top->size())));

                        osgUtil::Tessellator tessellator;
                        tessellator.retessellatePolygons(*geom[1]);
                        tessellator.retessellatePolygons(*geom[2]);

                        osg::ref_ptr<osg::Vec4Array> color_obj = new osg::Vec4Array();
                        color_obj->push_back(color);
                        for (auto& g : geom)
                        {
                            osgUtil::SmoothingVisitor::smooth(*g, 0.5);
                            g->setDataVariance(osg::Object::STATIC);
                            g->setUseDisplayList(true);
                            g->setColorArray(color_obj);
                            g->setColorBinding(osg::Geometry::BIND_OVERALL);
                            geode->addDrawable(g);
                            group->addChild(g);
                        }

                        osg::ComputeBoundsVisitor cbv;
                        group->accept(cbv);
                        boundingBox = cbv.getBoundingBox();

                        osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                        lod->addChild(group);
                        lod->setRange(0,
                                      0,
                                      LOD_DIST_ROAD_FEATURES + MAX(boundingBox.xMax() - boundingBox.xMin(), boundingBox.yMax() - boundingBox.yMin()));
                        objGroup->addChild(lod);
                    }
                    if (tx != nullptr)
                    {
                        tx->setName(prefix_road_object + std::to_string(object->GetId()));
                    }
                    else
                    {
                        LOG_WARN("No tx");
                    }
                }
            }
        }

        if (optimize_)
        {
            // For some reason this operation ruins the positioning of road objects in exported model
            osgUtil::Optimizer optimizer;
            optimizer.optimize(objGroup, osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS);
        }

        parent->addChild(objGroup);

        return 0;
    }

    osg::ref_ptr<osg::PositionAttitudeTransform> RoadGeom::LoadRoadFeature(roadmanager::Road* road, std::string filename, std::string exe_path)
    {
        (void)road;
        osg::ref_ptr<osg::Node>                      node;
        osg::ref_ptr<osg::PositionAttitudeTransform> xform = 0;

        // Load file, try multiple paths
        std::vector<std::string> file_name_candidates;
        file_name_candidates.push_back(filename);
        file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(exe_path) + "/../resources/models", filename));
        // Finally check registered paths
        for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
        {
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], filename));
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], "../models/" + filename));
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

    int RoadGeom::CreateOutlineObject(roadmanager::Outline*    outline,
                                      osg::Vec4                color,
                                      const osg::Vec3d&        origin,
                                      osg::ref_ptr<osg::Group> parent,
                                      id_t                     id)
    {
        if (outline == 0)
            return -1;
        bool roof = outline->roof_ ? true : false;

        // nrPoints will be corners + 1 if the outline should be closed, reusing first corner as last
        int nrPoints = outline->closed_ ? static_cast<int>(outline->corner_.size()) + 1 : static_cast<int>(outline->corner_.size());

        osg::ref_ptr<osg::Group> group = new osg::Group();
        group->setName(prefix_road_object + std::to_string(id));

        osg::ref_ptr<osg::Vec3Array> vertices_sides =
            new osg::Vec3Array(static_cast<unsigned int>(nrPoints) * 2);                                         // one set at bottom and one at top
        osg::ref_ptr<osg::Vec3Array> vertices_top    = new osg::Vec3Array(static_cast<unsigned int>(nrPoints));  // top
        osg::ref_ptr<osg::Vec3Array> vertices_bottom = new osg::Vec3Array(static_cast<unsigned int>(nrPoints));  // bottom

        // Set vertices
        for (size_t i = 0; i < outline->corner_.size(); i++)
        {
            double                      x, y, z;
            roadmanager::OutlineCorner* corner = outline->corner_[i];
            corner->GetPos(x, y, z);
            (*vertices_sides)[i * 2 + 0].set(static_cast<float>(x - origin[0]),
                                             static_cast<float>(y - origin[1]),
                                             static_cast<float>(z + corner->GetHeight()));
            (*vertices_sides)[i * 2 + 1].set(static_cast<float>(x - origin[0]), static_cast<float>(y - origin[1]), static_cast<float>(z));

            // top and bottom shapes
            if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_POLYGON)
            {
                (*vertices_top)[i].set(static_cast<float>(x - origin[0]),
                                       static_cast<float>(y - origin[1]),
                                       static_cast<float>(z + corner->GetHeight()));
                (*vertices_bottom)[i].set(static_cast<float>(x - origin[0]), static_cast<float>(y - origin[1]), static_cast<float>(z));
            }
        }

        if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_QUAD_STRIP)
        {
            // rearrange vertices for quad strip
            for (size_t i = 0; i < outline->corner_.size(); i++)
            {
                // points are starting at right side
                double                      x, y, z;
                unsigned                    index  = ((i % 2) == 0) ? outline->corner_.size() - 1 - (i / 2) : i / 2;
                roadmanager::OutlineCorner* corner = outline->corner_[index];
                corner->GetPos(x, y, z);

                (*vertices_top)[i].set(static_cast<float>(x - origin[0]),
                                       static_cast<float>(y - origin[1]),
                                       static_cast<float>(z + corner->GetHeight()));
                (*vertices_bottom)[i].set(static_cast<float>(x - origin[0]), static_cast<float>(y - origin[1]), static_cast<float>(z));
            }
        }

        // Close geometry
        if (outline->closed_)
        {
            (*vertices_sides)[2 * static_cast<unsigned int>(nrPoints) - 2].set((*vertices_sides)[0]);
            (*vertices_sides)[2 * static_cast<unsigned int>(nrPoints) - 1].set((*vertices_sides)[1]);
            (*vertices_top)[static_cast<unsigned int>(nrPoints) - 1].set((*vertices_top)[0]);
            (*vertices_bottom)[static_cast<unsigned int>(nrPoints) - 1].set((*vertices_bottom)[0]);
        }

        // Finally create and add geometry
        osg::ref_ptr<osg::Geode>    geode  = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geom[] = {new osg::Geometry, new osg::Geometry, new osg::Geometry};

        geom[0]->setVertexArray(vertices_sides.get());
        geom[0]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, 2 * nrPoints));

        if (roof)
        {
            if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_POLYGON)
            {
                geom[1]->setVertexArray(vertices_top.get());
                geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, nrPoints));
                osgUtil::Tessellator tessellator;
                tessellator.retessellatePolygons(*geom[1]);

                // then also add bottom
                geom[2]->setVertexArray(vertices_bottom.get());
                geom[2]->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, nrPoints));
                tessellator.retessellatePolygons(*geom[2]);
            }
            else
            {
                geom[1]->setVertexArray(vertices_top.get());
                geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, nrPoints - 1));

                geom[2]->setVertexArray(vertices_bottom.get());
                geom[2]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, nrPoints - 1));
            }
        }

        int nrGeoms = roof ? 3 : 1;
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
        geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        geode->getOrCreateStateSet()->setRenderBinDetails(20, "DepthSortedBin", osg::StateSet::RenderBinMode::OVERRIDE_RENDERBIN_DETAILS);
        geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
        geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

        group->addChild(geode);
        parent->addChild(group);

        return 0;
    }

    int RoadGeom::SaveToFile(std::string filename)
    {
        osgDB::writeNodeFile(*root_, filename);
        return 0;
    }

}  // namespace roadgeom