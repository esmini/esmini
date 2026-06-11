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
#include <osg/Switch>
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
#include <osg/TexMat>
#include <osgUtil/MeshOptimizers>
#include <osgUtil/Optimizer>    // to flatten transform nodes
#include <osgUtil/Tessellator>  // to tessellate multiple contours
#include <osgDB/WriteFile>

#include "CommonMini.hpp"

// cppcheck-suppress [unknownMacro]
USE_OSGPLUGIN(osg2)
USE_OSGPLUGIN(jpeg)
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
USE_COMPRESSOR_WRAPPER(ZLibCompressor)

#define GEOM_TOLERANCE            (0.2 - SMALL_NUMBER)  // Minimum distance between two vertices along road s-axis
#define TEXTURE_SCALE             2.0                   // Scale factor for asphalt and grass textures 2.0 means whole texture fits in 2 x 2 m square
#define MAX_GEOM_ERROR_HORIZONTAL 0.25                  // maximum distance from the 3D geometry to the OSI lines, on road surface plane
#define MAX_GEOM_ERROR_VERTICAL   0.1                   // maximum distance from the 3D geometry to the OSI lines, vertical to road surface
#define MAX_GEOM_LENGTH           50                    // maximum length of a road geometry mesh segment
#define MIN_GEOM_LENGTH           0.1                   // minimum length of a road geometry mesh segment, adjust if possible
#define ROADMARK_TEXTURE_SCALE    3.0                   // scale factor for roadmark textures, 3.0 means whole texture fits in 3 x 3 m square

#define POLYGON_OFFSET_SIDEWALK  2.0
#define POLYGON_OFFSET_ROADMARKS 1.0
#define POLYGON_OFFSET_BORDER    -1.0
#define POLYGON_OFFSET_GRASS     -2.0

#define ROADMARK_Z_OFFSET 0.01

#define DEFAULT_LENGTH_FOR_CONTINUOUS_OBJS 10.0
#define LOD_DIST_ROAD_FEATURES             500

const static double      friction_max       = 5.0;
const static double      friction_default   = 1.0;
const static std::string prefix_road        = "road_";
const static std::string prefix_road_object = "road_object_";
const static std::string prefix_tunnel_wall = "tunnel_wall_";
const static std::string prefix_tunnel_roof = "tunnel_roof_";
const static std::string prefix_road_signal = "road_signal_";
const static std::string prefix_roadmark    = "roadmark_";

namespace roadgeom
{
    class FindNamedNode : public osg::NodeVisitor
    {
    public:
        FindNamedNode(const std::string& name) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), _name(name)
        {
        }

        // This method gets called for every node in the scene graph. Check each node
        // to see if its name matches out target. If so, save the node's address.
        using osg::NodeVisitor::apply;
        void apply(osg::Group& node) override
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

        osg::Node* getNode()
        {
            return _node.get();
        }

    protected:
        std::string              _name;
        osg::ref_ptr<osg::Group> _node;
    };

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
        return osg::Vec4(rgb[0], rgb[1], rgb[2], 1.0);
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

    osg::ref_ptr<osg::Texture2D> RoadGeom::ReadTexture(std::string filename, bool log_missing_file)
    {
        osg::ref_ptr<osg::Texture2D> tex = 0;
        osg::ref_ptr<osg::Image>     img = 0;
        bool                         found;
        std::string                  file_path = LocateFile(filename,
                                                            {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models", exe_dir_ + "/../resources/models"},
                                           "Texture file",
                                           found,
                                           log_missing_file);

        if (found)
        {
            img = osgDB::readImageFile(file_path);

            if (img)
            {
                tex = new osg::Texture2D(img.get());
                tex->setUnRefImageDataAfterApply(true);
                tex->setWrap(osg::Texture2D::WrapParameter::WRAP_S, osg::Texture2D::WrapMode::REPEAT);
                tex->setWrap(osg::Texture2D::WrapParameter::WRAP_T, osg::Texture2D::WrapMode::REPEAT);
            }
            else
            {
                LOG_WARN("Failed to load texture file: {}", filename);
            }
        }

        return tex;
    }

    osg::Vec4 RoadGeom::GetFrictionColor(const double friction)
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
                                   const roadmanager::LaneRoadMark&    road_mark,
                                   double                              fade)
    {
        osg::ref_ptr<osg::Vec4Array> color_array = new osg::Vec4Array;
        color_array->push_back(ODR2OSGColor(road_mark.GetColor()));
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
        osg::ref_ptr<osg::Material> materialRoadmark_ =
            GetOrCreateMaterial("RoadMark_" + roadmanager::LaneRoadMark::RoadMarkColor2Str(road_mark.GetColor()),
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

            if (texture_map_[MaterialType::ROADMARK])
            {
                // set color to white for texture mapping. but keep alpha
                color_array->back()[0] = 1.0f;
                color_array->back()[1] = 1.0f;
                color_array->back()[2] = 1.0f;
                geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture_map_[MaterialType::ROADMARK].get());
            }
        }

        osgUtil::SmoothingVisitor::smooth(*geom, 0.0);
        geode->addDrawable(geom.get());
        SetNodeName(*geode, prefix_roadmark, rm_group->getNumChildren(), road_mark.Type2Str());
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
                                osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(
                                    new osg::Cylinder(osg::Vec3(0.0, 0.0, ROADMARK_Z_OFFSET), botts_dot_size, 0.3 * botts_dot_size),
                                    th);
                                shape->setColor(ODR2OSGColor(lane_roadmark->GetColor()));
                                dot = new osg::Geode;
                                dot->addDrawable(shape);
                            }

                            roadmanager::PointStruct osi_point0 = curr_osi_rm->GetPoint(q);

                            osg::ref_ptr<osg::PositionAttitudeTransform> tx = new osg::PositionAttitudeTransform;
                            tx->setPosition(osg::Vec3(static_cast<float>(osi_point0.x - origin[0]),
                                                      static_cast<float>(osi_point0.y - origin[1]),
                                                      static_cast<float>(osi_point0.z)));
                            tx->addChild(dot);
                            SetNodeName(*tx, prefix_roadmark, rm_group->getNumChildren(), lane_roadmark->Type2Str());
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

                        double l0p0l[3] = {0.0, 0.0, 0.0};  // previous line, startpoint, left side
                        double l0p0r[3] = {0.0, 0.0, 0.0};  // previous line, startpoint, right side
                        double l0p1l[3] = {0.0, 0.0, 0.0};  // previous line, endpoint, left side
                        double l0p1r[3] = {0.0, 0.0, 0.0};  // previous line, endpoint, right side
                        double l1p0l[3] = {0.0, 0.0, 0.0};  // current line, startpoint, left side
                        double l1p0r[3] = {0.0, 0.0, 0.0};  // current line, startpoint, right side
                        double l1p1l[3] = {0.0, 0.0, 0.0};  // current line, endpoint, left side
                        double l1p1r[3] = {0.0, 0.0, 0.0};  // current line, endpoint, right side

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
                                // calculate roadmark vertices, y offset based on width, z offset based on ROADMARK_Z_OFFSET, and
                                // rotation based on heading, pitch and roll of the OSI point

                                const double w    = lane_roadmarktypeline->GetWidth() / 2;
                                double       v[3] = {};

                                // right starting point
                                RotateVec3d(osi_points[q].h, osi_points[q].p, osi_points[q].r, 0.0, w, ROADMARK_Z_OFFSET, v[0], v[1], v[2]);
                                l1p0l[0] = osi_points[q].x + v[0] - origin[0];
                                l1p0l[1] = osi_points[q].y + v[1] - origin[1];
                                l1p0l[2] = osi_points[q].z + v[2];

                                // left end point
                                RotateVec3d(osi_points[q + 1].h,
                                            osi_points[q + 1].p,
                                            osi_points[q + 1].r,
                                            0.0,
                                            w,
                                            ROADMARK_Z_OFFSET,
                                            v[0],
                                            v[1],
                                            v[2]);
                                l1p1l[0] = osi_points[q + 1].x + v[0] - origin[0];
                                l1p1l[1] = osi_points[q + 1].y + v[1] - origin[1];
                                l1p1l[2] = osi_points[q + 1].z + v[2];

                                // left starting point
                                RotateVec3d(osi_points[q].h, osi_points[q].p, osi_points[q].r, 0.0, -w, ROADMARK_Z_OFFSET, v[0], v[1], v[2]);
                                l1p0r[0] = osi_points[q].x + v[0] - origin[0];
                                l1p0r[1] = osi_points[q].y + v[1] - origin[1];
                                l1p0r[2] = osi_points[q].z + v[2];

                                // right end point
                                RotateVec3d(osi_points[q + 1].h,
                                            osi_points[q + 1].p,
                                            osi_points[q + 1].r,
                                            0.0,
                                            -w,
                                            ROADMARK_Z_OFFSET,
                                            v[0],
                                            v[1],
                                            v[2]);
                                l1p1r[0] = osi_points[q + 1].x + v[0] - origin[0];
                                l1p1r[1] = osi_points[q + 1].y + v[1] - origin[1];
                                l1p1r[2] = osi_points[q + 1].z + v[2];
                            }
                            else if (!osi_points[q].endpoint)
                            {
                                LOG_ERROR("Unexpected last point without endpoint q {}", q);
                            }

                            if (q == startpoint)
                            {
                                // First point in a line sequence, no adjustment needed
                                (*vertices).push_back(
                                    osg::Vec3(static_cast<float>(l1p0l[0]), static_cast<float>(l1p0l[1]), static_cast<float>(l1p0l[2])));
                                (*vertices).push_back(
                                    osg::Vec3(static_cast<float>(l1p0r[0]), static_cast<float>(l1p0r[1]), static_cast<float>(l1p0r[2])));
                            }
                            else if (osi_points[q].endpoint)
                            {
                                // Last point of a line sequence, no adjustment needed
                                double* left  = (q < osi_points.size() - 1) ? l1p0l : l1p1l;
                                double* right = (q < osi_points.size() - 1) ? l1p0r : l1p1r;
                                (*vertices).push_back(
                                    osg::Vec3(static_cast<float>(left[0]), static_cast<float>(left[1]), static_cast<float>(left[2])));
                                (*vertices).push_back(
                                    osg::Vec3(static_cast<float>(right[0]), static_cast<float>(right[1]), static_cast<float>(right[2])));
                            }
                            else
                            {
                                // Find intersection of non parallel lines
                                double isect[2];

                                // left side
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
                                    (*vertices).push_back(
                                        osg::Vec3(static_cast<float>(isect[0]), static_cast<float>(isect[1]), static_cast<float>(l0p1l[2])));
                                }
                                else
                                {
                                    // lines parallel, no adjustment needed
                                    (*vertices).push_back(
                                        osg::Vec3(static_cast<float>(l1p0l[0]), static_cast<float>(l1p0l[1]), static_cast<float>(l1p0l[2])));
                                }

                                // right side
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
                                    (*vertices).push_back(
                                        osg::Vec3(static_cast<float>(isect[0]), static_cast<float>(isect[1]), static_cast<float>(l0p1r[2])));
                                }
                                else
                                {
                                    // lines parallel, no adjustment needed
                                    (*vertices).push_back(
                                        osg::Vec3(static_cast<float>(l1p0r[0]), static_cast<float>(l1p0r[1]), static_cast<float>(l1p0r[2])));
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
                                AddRoadMarkGeom(vertices, indices, rm_group, *lane_roadmark, lane_roadmark->GetFade());
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
                       osg::Node*              environment,
                       osg::Vec3d              origin,
                       bool                    generate_road_surface,
                       bool                    generate_road_objects,
                       std::string             exe_path,
                       bool                    optimize)
        : environment_(environment),
          optimize_(optimize)
    {
        if (!generate_road_surface && !generate_road_objects)
        {
            return;
        }

        exe_dir_    = DirNameOf(exe_path);
        odrManager_ = odr;
        root_       = new osg::Group;
        root_->setName("esmini_generated_road_model");

        if (generate_road_surface)
        {
            LOG_INFO("Generating a simplistic 3D model of the road network");

            osg::ref_ptr<osg::Group> r_group_ = new osg::Group;
            r_group_->setName("roads");
            osg::ref_ptr<osg::Group> rm_group_ = new osg::Group;
            rm_group_->setName("roadmarks");
            root_->addChild(rm_group_);
            root_->addChild(r_group_);

            if (!SE_Env::Inst().GetOptions().GetOptionSet("generate_without_textures"))
            {
                texture_map_[MaterialType::ASPHALT]  = ReadTexture("asphalt.jpg");
                texture_map_[MaterialType::GRASS]    = ReadTexture("grass.jpg");
                texture_map_[MaterialType::ROADMARK] = ReadTexture("roadmark.jpg", false);  // optional texture, not part of esmini
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
            // for each road and lane section, consider all lanes except center lane (id 0):
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

                    std::vector<int> all_lane_ids;
                    all_lane_ids.reserve(lsec->GetNumberOfLanes());

                    std::vector<int> lane_ids;  // all physical lane ids, except center lane which has no area
                    lane_ids.reserve(lsec->GetNumberOfLanes() - 1);
                    std::unordered_map<int, int> lane_idxs;  // store indexes for all lanes, except center lane (id 0)

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
                        double p;
                        double r;
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
                        double s;
                        double x;
                        double y;
                        double z;
                    };

                    std::vector<std::vector<std::vector<GeomPoint>>> geom_points_list;                          // two lists of points per lane
                    std::vector<std::vector<GeomStrip>>              geom_strips_list;                          // one list of strips info per lane
                    std::vector<int>                                 lane_osi_index(lsec->GetNumberOfLanes());  // current osi point per lane
                    std::vector<GeomCacheEntry>                      geom_cache(lsec->GetNumberOfLanes());      // one cache entry per lane
                    std::vector<CandidatePos>                        candidates_pos(lsec->GetNumberOfLanes());  // candidates for next current s-value
                    double                                           section_current_s = lsec->GetS();

                    roadmanager::Position pos;  // used for calculating points along the road
                    pos.SetSnapLaneTypes(-1);

                    // First populate s values of the material elements
                    //   - for each material a new friction segment is to be added
                    //   - loop over material friction segments, insert new vertices if needed
                    std::vector<double> friction_s_list;
                    for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
                    {
                        lane = lsec->GetLaneByIdx(k);

                        all_lane_ids.push_back(lane->GetId());

                        if (lane->GetId() == 0)
                        {
                            // skip center lane
                            continue;
                        }

                        lane_idxs[lane->GetId()] = lane_idxs.size();
                        lane_ids.push_back(lane->GetId());

                        for (size_t l = 0; l < lane->GetNumberOfMaterials(); l++)
                        {
                            friction_s_list.push_back(lsec->GetS() + lane->GetMaterialByIdx(l)->s_offset);
                        }

                        // add lane height entries to the s-value list
                        for (size_t l = 0; l < lane->GetNumberOfHeights(); l++)
                        {
                            friction_s_list.push_back(lsec->GetS() + lane->GetHeightByIdx(l)->s_offset);
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
                            for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes() - 1); k++)
                            {
                                lane_osi_index[k]   = 0;
                                candidates_pos[k].s = lsec->GetS();
                            }
                        }
                        else
                        {
                            // for each lane, find next s-value in and register it as candidate section current s-value
                            std::vector<double> s_list_sorted(all_lane_ids.size());
                            for (unsigned int k = 0; k < all_lane_ids.size(); k++)
                            {
                                lane                                            = lsec->GetLaneById(all_lane_ids[k]);
                                std::vector<roadmanager::PointStruct> osiPoints = lane->GetOSIPoints()->GetPoints();

                                for (size_t l = lane_osi_index[k]; l < osiPoints.size(); l++)
                                {
                                    if (l == osiPoints.size() - 1 || osiPoints[l].s > section_current_s + SMALL_NUMBER)
                                    {
                                        lane_osi_index[k]   = l;
                                        candidates_pos[k].s = osiPoints[l].s;
                                        s_list_sorted[k]    = osiPoints[l].s;

                                        // generate point at osi index s-value
                                        pos2.SetLaneBoundaryPos(road->GetId(), lane->GetId(), candidates_pos[k].s);
                                        candidates_pos[k].x = pos2.GetX();
                                        candidates_pos[k].y = pos2.GetY();
                                        candidates_pos[k].z = pos2.GetZ();

                                        break;
                                    }
                                }
                            }

                            // sort candidates
                            std::sort(s_list_sorted.begin(), s_list_sorted.end());
                            s_list_sorted.erase(std::unique(s_list_sorted.begin(), s_list_sorted.end(), compare_s_values), s_list_sorted.end());

                            // find highest s-value not exceeding the tolerated error, over all lanes
                            size_t k = 0;
                            for (; k < s_list_sorted.size(); k++)
                            {
                                size_t l = 0;
                                for (; l < all_lane_ids.size(); l++)
                                {
                                    lane = lsec->GetLaneById(all_lane_ids[l]);

                                    // generate point at pivot s-value
                                    pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), s_list_sorted[k]);

                                    // create a delta vector from real pos to cache/pivot point
                                    double diff[3], diff_tx[3];
                                    diff[0] = pos.GetX() - geom_cache[l].point.x;
                                    diff[1] = pos.GetY() - geom_cache[l].point.y;
                                    diff[2] = pos.GetZ() - geom_cache[l].point.z;

                                    // transform delta vector to road local coordinates, to get longitudinal and lateral error
                                    InverseRotateVec3d(geom_cache[l].point.h,
                                                       geom_cache[l].point.p,
                                                       geom_cache[l].point.r,
                                                       diff[0],
                                                       diff[1],
                                                       diff[2],
                                                       diff_tx[0],
                                                       diff_tx[1],
                                                       diff_tx[2]);

                                    double error_horizontal = abs(diff_tx[1]);
                                    double error_vertical   = abs(diff_tx[2]);

                                    if (error_horizontal > MAX_GEOM_ERROR_HORIZONTAL || error_vertical > MAX_GEOM_ERROR_VERTICAL)
                                    {
                                        break;
                                    }
                                }

                                if (l == all_lane_ids.size())
                                {
                                    // no error, register preliminary s value - if larger than current
                                    if (s_list_sorted[k] > section_current_s)
                                    {
                                        section_current_s = s_list_sorted[k];
                                    }
                                }
                                else
                                {
                                    // break occured, error too large, stop searching
                                    if (k == 0)
                                    {
                                        // can't skip first point
                                        section_current_s = s_list_sorted[k];
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
                        unsigned int geom_idx = 0;
                        for (size_t k = 0; k < all_lane_ids.size(); k++)
                        {
                            int lane_id  = all_lane_ids[k];
                            int lane_idx = lane_idxs[lane_id];

                            if (counter == 0 && lane_id != 0)
                            {
                                // add geometry and strip list for the lane
                                geom_points_list.push_back({{}, {}});
                                geom_strips_list.push_back({});
                            }

                            roadmanager::Lane::Material* mat = nullptr;
                            lane                             = lsec->GetLaneById(lane_id);
                            mat                              = lane->GetMaterialByS(section_current_s - lsec->GetS());
                            double friction                  = mat != nullptr ? mat->friction : FRICTION_DEFAULT;

                            if (lane_id != 0 && counter == 0 || !NEAR_NUMBERS(friction, geom_cache[k].friction))
                            {
                                // create initial strip or strip with new friction value
                                geom_strips_list[lane_idx].push_back({static_cast<int>(geom_points_list[lane_idx][0].size()), friction});
                            }

                            for (auto side : {0, 1})  // left, right
                            {
                                // Determine if we are on the "outer" edge relative to the road center
                                bool is_boundary = (lane_id >= 0) ? (side == 0) : (side == 1);

                                if (!is_boundary)
                                {
                                    double lane_width = lsec->GetWidth(section_current_s, lane_id);
                                    pos.SetLanePos(road->GetId(),
                                                   lane_id,
                                                   section_current_s,
                                                   -1 * SIGN(lane_id) * lane_width / 2);  // offset from lane center (half lane) to inner boundary
                                }
                                else
                                {
                                    pos.SetLaneBoundaryPos(road->GetId(), lane_id, section_current_s);
                                }

                                GeomPoint gp =
                                    {pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), pos.GetP(), pos.GetR(), pos.GetZRoadPrim(), pos.GetS()};

                                if (lane_id != 0)
                                {
                                    geom_points_list[geom_idx][side].push_back(gp);
                                }

                                if (is_boundary)
                                {
                                    geom_cache[k] = {gp, friction};
                                }

                                if (lane_id == 0)
                                {
                                    break;
                                }
                            }
                            if (lane_id != 0)
                            {
                                geom_idx++;
                            }
                        }
                    }

                    // Then create actual vertices and triangle strips for the lane section
                    // Each strip is made of two lanes, so we need to create a separate geometry for each pair of lanes
                    // Also within each lane, we need to create a separate geometry for each material segment
                    unsigned int nr_vertices =
                        static_cast<unsigned int>(2 * geom_points_list[0][0].size() * geom_strips_list.size());  // same nr vertices in all lanes
                    osg::ref_ptr<osg::Vec3Array> verticesAll  = new osg::Vec3Array(nr_vertices);
                    osg::ref_ptr<osg::Vec2Array> texcoordsAll = new osg::Vec2Array(nr_vertices);

                    // Potential optimization: Swap loops, creating all vertices for same s-value for each step
                    unsigned int vertex_counter = 0;
                    unsigned int vertex_idx     = 0;
                    double       texscale       = 1.0 / TEXTURE_SCALE;

                    for (size_t k = 0; k < geom_strips_list.size(); k++)  // loop over lanes
                    {
                        osg::ref_ptr<osg::Vec3Array>        verticesLocal;
                        osg::ref_ptr<osg::Vec2Array>        texcoordsLocal;
                        osg::ref_ptr<osg::Vec4Array>        colorLocal;
                        osg::ref_ptr<osg::DrawElementsUInt> indices;
                        lane = lsec->GetLaneById(lane_ids[k]);

                        for (size_t m = 0; m < geom_strips_list[k].size(); m++)  // loop over lane patches with constant friction
                        {
                            lane_friction_                                   = geom_strips_list[k][m].friction;
                            unsigned int                         gpi         = geom_strips_list[k][m].geom_point_index;
                            std::vector<std::vector<GeomPoint>>& geom_points = geom_points_list[k];
                            unsigned int                         n_points    = 0;

                            if (m < geom_strips_list[k].size() - 1)
                            {
                                n_points = geom_strips_list[k][m + 1].geom_point_index - gpi + 1;
                            }
                            else
                            {
                                n_points = geom_points[0].size() - gpi;
                            }

                            verticesLocal  = new osg::Vec3Array(n_points * 2);
                            indices        = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, n_points * 2);
                            texcoordsLocal = new osg::Vec2Array(n_points * 2);

                            unsigned int index_counter = 0;

                            int z_gap_found = 0;  // -1: left pointing wall, 1: right pointing wall, 0: no z difference => no wall
                            for (size_t l = 0; l < n_points; l++)
                            {
                                if (m == 0 || l > 0)
                                {
                                    // only create vertex when needed, reuse vertex of previous patch for first vertex of any additional patch
                                    for (auto side : {0, 1})  // left, right side of the lane - comply with GL_TRIANGLE_STRIP order
                                    {
                                        GeomPoint& gp = geom_points[side][gpi + l];

                                        // vertex
                                        (*verticesAll)[vertex_counter].set(static_cast<float>(gp.x - origin[0]),
                                                                           static_cast<float>(gp.y - origin[1]),
                                                                           static_cast<float>(gp.z));
                                        (*texcoordsAll)[vertex_counter].set(osg::Vec2(static_cast<float>(texscale * (gp.x - origin[0])),
                                                                                      static_cast<float>(texscale * (gp.y - origin[1]))));
                                        vertex_idx = vertex_counter++;
                                    }

                                    if (!z_gap_found && k > 0)  // look for z gap to the left neighbor lane (skip first)
                                    {
                                        double z_diff = geom_points[0][gpi + l].z - geom_points_list[k - 1][1][gpi + l].z;
                                        if (fabs(z_diff) > 1e-3)
                                        {
                                            z_gap_found = z_diff < 0 ? -1 : 1;
                                        }
                                    }
                                }

                                for (auto side : {0, 1})  // left, right side of the lane
                                {
                                    // Create indices for the lane strip, referring to the vertex list
                                    (*verticesLocal)[index_counter]  = (*verticesAll)[vertex_idx + side - 1];
                                    (*texcoordsLocal)[index_counter] = (*texcoordsAll)[vertex_idx + side - 1];
                                    (*indices)[index_counter++]      = index_counter;
                                }
                            }

                            // Create geometry for the strip made of this and previous lane
                            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                            geom->setUseDisplayList(true);
                            geom->setVertexArray(verticesLocal.get());
                            geom->addPrimitiveSet(indices.get());
                            geom->setTexCoordArray(0, texcoordsLocal.get());

                            MaterialType material_t;

                            if (lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_ANY_ROAD))
                            {
                                material_t = MaterialType::ASPHALT;
                                osg::ref_ptr<osg::Material> materialAsphalt_ =
                                    GetOrCreateMaterial("Asphalt", GetFrictionColor(lane_friction_), static_cast<uint8_t>(material_t), 1);

                                geom->getOrCreateStateSet()->setAttributeAndModes(materialAsphalt_.get());
                            }
                            else if (lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BIKING) ||
                                     lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_SIDEWALK))
                            {
                                material_t = MaterialType::CONCRETE;
                                osg::ref_ptr<osg::Material> materialConcrete_ =
                                    GetOrCreateMaterial("Concrete", color_concrete->at(0), static_cast<uint8_t>(material_t));
                                geom->getOrCreateStateSet()->setAttributeAndModes(materialConcrete_.get());

                                // Use PolygonOffset feature to avoid z-fighting with road surface
                                geom->getOrCreateStateSet()->setAttributeAndModes(
                                    new osg::PolygonOffset(-POLYGON_OFFSET_SIDEWALK, -SIGN(POLYGON_OFFSET_SIDEWALK)));
                            }
                            // for border and grass materials, consider also lane position and neighbor lanes
                            // If lane is inner; either has neighbor lanes on both sides or is the first lane next to center:
                            // "none" type will get border material
                            // "border" type will get grass material
                            else if (((k > 0 && k + 1 < lane_ids.size()) || abs(lane->GetId()) == 1) &&
                                     (lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BORDER) ||
                                      lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_NONE)))
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
                            SetNodeName(*geode, prefix_road, road->GetId(), std::to_string(k) + "_" + std::to_string(m));
                            r_group_->addChild(geode);

                            if (z_gap_found != 0)
                            {
                                // create a patch to fill the verical gap using material of outer patch

                                osg::ref_ptr<osg::Geode> geode_to_clone;
                                if (z_gap_found == 1)
                                {
                                    // vertical patch is facing left, use material from current lane
                                    geode_to_clone = static_cast<osg::Geode*>(r_group_->getChild(r_group_->getNumChildren() - 1));
                                }
                                else
                                {
                                    // vertical patch is facing right, use material from previous (left) patch
                                    geode_to_clone = static_cast<osg::Geode*>(r_group_->getChild(r_group_->getNumChildren() - 2));
                                }

                                osg::ref_ptr<osg::Geode> geode_gap = static_cast<osg::Geode*>(geode_to_clone->clone(osg::CopyOp::DEEP_COPY_ALL));
                                if (geode_gap)
                                {
                                    osg::ref_ptr<osg::Vec3Array> vertices_gap = new osg::Vec3Array(n_points * 2);
                                    for (size_t l = 0; l < n_points; l++)
                                    {
                                        // vertex from previous patch
                                        (*vertices_gap)[l * 2 + 0].set(geom_points_list[k - 1][1][gpi + l].x - origin[0],
                                                                       geom_points_list[k - 1][1][gpi + l].y - origin[1],
                                                                       geom_points_list[k - 1][1][gpi + l].z);
                                        // vertex from current patch
                                        (*vertices_gap)[l * 2 + 1].set(osg::Vec3(geom_points_list[k][0][gpi + l].x - origin[0],
                                                                                 geom_points_list[k][0][gpi + l].y - origin[1],
                                                                                 geom_points_list[k][0][gpi + l].z));
                                    }

                                    osg::Geometry* geom_gap = geode_gap->getDrawable(0)->asGeometry();
                                    if (geom_gap)
                                    {
                                        geom_gap->setVertexArray(vertices_gap.get());
                                        r_group_->addChild(geode_gap);
                                        LOG_DEBUG("Added vertical gap patch for road {} section {} between lanes {} and {}",
                                                  road->GetId(),
                                                  j,
                                                  lane_ids[k],
                                                  lane_ids[k - 1]);
                                    }
                                    else
                                    {
                                        LOG_WARN("Failed to create geom for vertical gap patch at road {} section {} between lanes {} and {}",
                                                 road->GetId(),
                                                 j,
                                                 lane_ids[k],
                                                 lane_ids[k - 1]);
                                    }
                                }
                                else
                                {
                                    LOG_WARN("Failed to create geode for vertical gap patch for road {} section {} between lanes {} and {}",
                                             road->GetId(),
                                             j,
                                             lane_ids[k],
                                             lane_ids[k - 1]);
                                }
                            }
                        }
                    }

                    for (unsigned int l = 0; l < lsec->GetNumberOfLanes(); l++)
                    {
                        AddRoadMarks(lsec->GetLaneByIdx(l), rm_group_, origin);
                    }
                }
            }
            for (unsigned int i = 0; i < r_group_->getNumChildren(); i++)
            {
                // calculate normals for all lane strips
                osg::ref_ptr<osg::Geode> lane_patch_geode = static_cast<osg::Geode*>(r_group_->getChild(i));
                if (lane_patch_geode)
                {
                    osgUtil::SmoothingVisitor::smooth(*lane_patch_geode->getDrawable(0)->asGeometry(), 0.5);
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

        std::string opt_groundplane = SE_Env::Inst().GetOptions().GetOptionValueByEnum(esmini_options::GROUND_PLANE);
        if ((opt_groundplane == "auto" && (!generate_road_surface && environment == nullptr)) || opt_groundplane == "on")
        {
            AddGroundSurface();
        }
    }

    osg::ref_ptr<osg::Geometry> createTiledFace(const osg::Vec3&                    v0,
                                                const osg::Vec3&                    v1,
                                                const osg::Vec3&                    v2,
                                                const osg::Vec3&                    v3,
                                                const osg::Vec3&                    normal,
                                                float                               faceWidth,
                                                float                               faceHeight,
                                                float                               texture_scale,
                                                const osg::ref_ptr<osg::Vec4Array>& color_array)
    {
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

        // 1. Set Vertices
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        vertices->push_back(v0);
        vertices->push_back(v1);
        vertices->push_back(v2);
        vertices->push_back(v3);
        geom->setVertexArray(vertices.get());

        // 2. Set scaled UVs
        osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array();
        texcoords->push_back(osg::Vec2(0.0f, 0.0f));
        texcoords->push_back(osg::Vec2(faceWidth / texture_scale, 0.0f));
        texcoords->push_back(osg::Vec2(faceWidth / texture_scale, faceHeight / texture_scale));
        texcoords->push_back(osg::Vec2(0.0f, faceHeight / texture_scale));
        geom->setTexCoordArray(0, texcoords.get());

        // 3. Set Normal
        osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
        normals->push_back(normal);
        geom->setNormalArray(normals.get(), osg::Array::BIND_OVERALL);

        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
        geom->setColorArray(color_array.get(), osg::Array::BIND_OVERALL);

        return geom;
    }

    osg::ref_ptr<osg::Geode> createTiledBox(float length, float width, float height, float texSize, const osg::Vec4& color)
    {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();

        float dx = length / 2.0f;
        float dy = width / 2.0f;
        float dz = height;

        osg::ref_ptr<osg::Vec4Array> color_array = new osg::Vec4Array();
        color_array->push_back(color);

        // Define 3D corner points
        osg::Vec3 p0(-dx, -dy, 0), p1(dx, -dy, 0), p2(dx, dy, 0), p3(-dx, dy, 0);
        osg::Vec3 p4(-dx, -dy, dz), p5(dx, -dy, dz), p6(dx, dy, dz), p7(-dx, dy, dz);
        osg::Vec3 n0(0, 0, -1), n1(0, 0, 1), n2(0, 1, 0), n3(0, -1, 0), n4(1, 0, 0), n5(-1, 0, 0);

        // Front Face (Length x Height)
        geode->addDrawable(createTiledFace(p0, p1, p5, p4, n0, length, height, texSize, color_array));
        // Back Face (Length x Height)
        geode->addDrawable(createTiledFace(p2, p3, p7, p6, n1, length, height, texSize, color_array));
        // Top Face (Length x Width)
        geode->addDrawable(createTiledFace(p4, p5, p6, p7, n2, length, width, texSize, color_array));
        // Bottom Face (Length x Width)
        geode->addDrawable(createTiledFace(p3, p2, p1, p0, n3, length, width, texSize, color_array));
        // Right Face (Width x Height)
        geode->addDrawable(createTiledFace(p1, p2, p6, p5, n4, width, height, texSize, color_array));
        // Left Face (Width x Height)
        geode->addDrawable(createTiledFace(p3, p0, p4, p7, n5, width, height, texSize, color_array));

        // osg::ref_ptr<osg::Material> material_ = new osg::Material;
        // material_->setDiffuse(osg::Material::FRONT_AND_BACK, color);
        // material_->setAmbient(osg::Material::FRONT_AND_BACK, color);
        // geode->getOrCreateStateSet()->setAttributeAndModes(material_.get());

        return geode;
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
                tx                                 = nullptr;
                roadmanager::Signal* signal        = road->GetSignal(s);
                SetNodeName(*signGroup, prefix_road_signal, s, signal->GetName());

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

                    if (!(NEAR_NUMBERS(signal->GetValue(), -1.0) || signal->GetValueStr().empty()))
                    {
                        filename += "-" + signal->GetValueStr();
                    }

                    bool        found = false;
                    std::string located_file_path =
                        LocateFile(filename + ".osgb",
                                   {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models", DirNameOf(exe_path) + "/../resources/models"},
                                   "Road signal 3D model",
                                   found);

                    if (found)
                    {
                        tx = LoadRoadFeature(road, located_file_path);
                    }

                    if (tx == nullptr)
                    {
                        // if file according to type, subtype and value could not be resolved, try from name
                        located_file_path =
                            LocateFile(signal->GetName() + ".osgb",
                                       {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models", DirNameOf(exe_path) + "/../resources/models"},
                                       "Road signal 3D model",
                                       found);
                        if (found)
                        {
                            tx = LoadRoadFeature(road, signal->GetName() + ".osgb");
                        }
                    }

                    if (found)
                    {
                        signal->SetModel3DFullPath(located_file_path);
                    }

                    if (tx == nullptr && typeid(*signal) == typeid(roadmanager::TrafficLight))
                    {
                        std::string texture_file_name = "opendrive_" + signal->GetCombinedType() + ".png";

                        auto it = roadmanager::traffic_light_type_map.find(signal->GetCombinedType());

                        if (it != roadmanager::traffic_light_type_map.end())
                        {
                            roadmanager::TrafficLightInfo tl_info = it->second;

                            traffic_light_.emplace(
                                signal->GetId(),
                                TrafficLightModel(tl_info.nr_lamps,
                                                  LocateFile(texture_file_name,
                                                             {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models/roads_signal_textures",
                                                              DirNameOf(exe_path) + "/../resources/models/roads_signal_textures"},
                                                             "Traffic light texture",
                                                             found)));
                            if (found)
                            {
                                tx = traffic_light_.at(signal->GetId()).GetTx();
                            }
                        }
                        else
                        {
                            LOG_ERROR("Unsupport traffic signal type {}, can't resolve corresponding texture file", signal->GetType());
                        }
                    }

                    if (tx == nullptr)
                    {
                        LOG_DEBUG("Failed to load signal {}.osgb / {}.osgb or create 3D model - using simple bounding box",
                                  FileNameOf(filename),
                                  signal->GetName());
                        osg::ref_ptr<osg::PositionAttitudeTransform> obj_standin =
                            dynamic_cast<osg::PositionAttitudeTransform*>(tx_bb->clone(osg::CopyOp::DEEP_COPY_ALL));
                        obj_standin->setNodeMask(NODE_MASK_SIGN);
                        signGroup->addChild(obj_standin);
                    }
                    else
                    {
                        tx->setPosition(osg::Vec3(static_cast<float>(signal->GetX() - origin[0]),
                                                  static_cast<float>(signal->GetY() - origin[1]),
                                                  static_cast<float>(signal->GetZ() + signal->GetZOffset())));
                        tx->setAttitude(osg::Quat(signal->GetH() + signal->GetHOffset(), osg::Vec3(0, 0, 1)));
                        tx->setNodeMask(NODE_MASK_SIGN);
                        signGroup->addChild(tx);
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
                roadmanager::RMObject*       object  = road->GetRoadObject(o);
                osg::ref_ptr<osg::Texture2D> texture = nullptr;
                osg::Vec4                    color;
                tx                   = nullptr;
                std::string obj_type = prefix_road_object;  // road object is default
                switch (object->GetTunnelComponentType())
                {
                    case roadmanager::RMObject::TunnelComponentType::TUNNEL_WALL:
                        obj_type = prefix_tunnel_wall;
                        break;
                    case roadmanager::RMObject::TunnelComponentType::TUNNEL_ROOF:
                        obj_type = prefix_tunnel_roof;
                        break;
                }

                if (!object->GetTextureFilename().empty())
                {
                    texture = ReadTexture(object->GetTextureFilename());
                }

                for (unsigned int c = 0; c < 4; c++)
                {
                    color[c] = object->GetColor()[c];
                }

                // View-mode handling (same as entities, toggled with the ',' key / --view_mode):
                // The object's "volume" (3D model, stand-in box, outline or continuous geometry) is
                // tagged NODE_MASK_ENTITY_MODEL. When the object carries markings, the volume is hidden
                // by default (markings represent it instead); otherwise it is shown by default.
                const bool         has_markings    = object->GetNumberOfMarkings() > 0;
                const int          vol_mask        = has_markings ? NODE_MASK_NONE : NODE_MASK_ENTITY_MODEL;
                const unsigned int vol_child_begin = objGroup->getNumChildren();

                if (object->GetNumberOfOutlines() > 0 &&
                    object->GetNumberOfRepeats() == 0)  // if repeats are defined, wait and see if outline should replace failed 3D model or not
                {
                    for (size_t j = 0; j < static_cast<unsigned int>(object->GetNumberOfOutlines()); j++)
                    {
                        roadmanager::Outline*    outline = object->GetOutline(j);
                        osg::ref_ptr<osg::Group> olgroup = CreateOutlineObject(outline, color, origin, {}, 1.0, texture, object->GetTextureScale());
                        if (olgroup != nullptr)
                        {
                            SetNodeName(*olgroup,
                                        obj_type,
                                        object->GetId(),
                                        object->GetName().empty() ? object->GetTypeStr() : object->GetName() + "_" + std::to_string(j));
                            objGroup->addChild(olgroup);
                            if (texture != nullptr)
                            {
                                olgroup->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
                            }
                        }
                    }
                    LOG_INFO("Created outline geometry for object {}.", object->GetName());
                    LOG_DEBUG("  if it looks strange, e.g.faces too dark or light color, ");
                    LOG_DEBUG("  check that corners are defined counter-clockwise (as OpenGL default).");
                }
                else if (roadmanager::Repeat* outl_rep = object->GetRepeat(); object->GetNumberOfOutlines() > 0 && outl_rep != nullptr &&
                                                                              outl_rep->GetDistance() > SMALL_NUMBER &&
                                                                              outl_rep->GetLength() > SMALL_NUMBER)
                {
                    // Repeated object (separate copies) with outline(s): create curvature-aware geometry
                    // per instance so cornerRoad outlines follow the road (e.g. distorted in curves).
                    CreateRepeatedOutlineObjects(object, road, color, origin, objGroup.get(), obj_type, texture);
                    LOG_INFO("Created repeated outline geometry for object {}.", object->GetName());
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

                        bool        found = false;
                        std::string located_file_path =
                            LocateFile(filename,
                                       {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models", DirNameOf(exe_path) + "/../resources/models"},
                                       "Road object 3D model",
                                       found);

                        if (found)
                        {
                            tx = LoadRoadFeature(road, located_file_path);
                            object->SetModel3DFullPath(located_file_path);
                        }

                        if (tx == nullptr)
                        {
                            LOG_WARN("Failed to load road object model file: {} ({}). Creating a bounding box as stand in.",
                                     FileNameOf(filename),
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
                                    roadmanager::Outline*    outline = object->GetOutline(j);
                                    osg::ref_ptr<osg::Group> olgroup =
                                        CreateOutlineObject(outline, color, origin, {}, 1.0, texture, object->GetTextureScale());
                                    if (olgroup != nullptr)
                                    {
                                        SetNodeName(*olgroup,
                                                    obj_type,
                                                    object->GetId(),
                                                    object->GetName().empty() ? object->GetTypeStr() : object->GetName() + "_" + std::to_string(j));
                                        olgroup->setNodeMask(static_cast<unsigned int>(vol_mask));
                                        objGroup->addChild(olgroup);
                                    }
                                }
                                CreateObjectBoundingBoxes(object, road, color, origin, objGroup.get());

                                if (has_markings)
                                {
                                    osg::ref_ptr<osg::Group> markings_group = CreateObjectMarkings(object, road, origin);
                                    if (markings_group != nullptr && markings_group->getNumChildren() > 0)
                                    {
                                        SetNodeName(*markings_group,
                                                    prefix_road_object,
                                                    object->GetId(),
                                                    object->GetName().empty() ? object->GetTypeStr() : object->GetName() + "_markings");
                                        objGroup->addChild(markings_group);
                                    }
                                }
                                continue;
                            }
                            else
                            {
                                // create stand in object
                                vertices_left_side  = new osg::Vec3Array;
                                vertices_right_side = new osg::Vec3Array;
                                vertices_top        = new osg::Vec3Array;
                                vertices_bottom     = new osg::Vec3Array;
                                group               = new osg::Group();
                            }
                        }
                        else
                        {
                            // create a bounding box to represent the object
                            tx = new osg::PositionAttitudeTransform;

                            // avoid zero width, length and width - set to a minimum value of 0.05m
                            const float obj_hgt = MAX(0.05f, static_cast<float>(object->GetHeight()));
                            const float length  = MAX(0.05f, static_cast<float>(object->GetLength()));
                            const float width   = MAX(0.05f, static_cast<float>(object->GetWidth()));

                            if (texture != nullptr)
                            {
                                color[0] = color[1] = color[2] = 1.0f;  // set color to white to show texture colors without tint
                            }

                            if (object->GetRadius() > SMALL_NUMBER)
                            {
                                // Circular object (ASAM OpenDRIVE 13.8): represent as a cylinder, base resting at z = 0
                                osg::ref_ptr<osg::ShapeDrawable>     shape;
                                osg::ref_ptr<osg::TessellationHints> th = new osg::TessellationHints();
                                th->setDetailRatio(0.4f);
                                shape = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 0.5f * obj_hgt),
                                                                                 MAX(0.05f, static_cast<float>(object->GetRadius())),
                                                                                 obj_hgt),
                                                               th.get());
                                shape->setColor(color);
                                if (texture != nullptr)
                                {
                                    shape->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
                                }
                                tx->addChild(shape.get());
                            }
                            else
                            {
                                osg::ref_ptr<osg::Geode> geode;

                                if (texture != nullptr)
                                {
                                    color[0] = color[1] = color[2] = 1.0f;  // set color to white to show texture colors without tint
                                    geode                          = createTiledBox(length, width, obj_hgt, object->GetTextureScale(), color);
                                    geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
                                }
                                else
                                {
                                    osg::ref_ptr<osg::Box> box = new osg::Box(osg::Vec3(0.0f, 0.0f, 0.5f * obj_hgt), length, width, obj_hgt);
                                    osg::ref_ptr<osg::ShapeDrawable> box_shape = new osg::ShapeDrawable(box.get());
                                    box_shape->setColor(color);
                                    geode = new osg::Geode();
                                    geode->addDrawable(box_shape.get());
                                }
                                tx->addChild(geode);
                            }
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

                    for (;
                         nCopies < 1 || (rep && rep->length_ > SMALL_NUMBER && cur_s < rep->GetLength() + SMALL_NUMBER && cur_s < road->GetLength());
                         nCopies++)
                    {
                        double s;
                        double scale_x = 1.0;
                        double scale_y = 1.0;
                        double scale_z = 1.0;

                        // Count instances by accumulated length only; the repeat start s offset
                        // compensates for the bounding box center, not the object border, so it must
                        // not reduce the number of copies that fit along the road.
                        if (rep && cur_s + (object->GetLength() * scale_x) * cos(object->GetHOffset()) > road->GetLength())
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

                        if (clone != nullptr)
                        {
                            SetNodeName(*clone,
                                        obj_type,
                                        object->GetId(),
                                        object->GetName().empty() ? object->GetTypeStr() : object->GetName() + "_" + std::to_string(nCopies));
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
                                if (rep->GetRadiusStart() > SMALL_NUMBER || rep->GetRadiusEnd() > SMALL_NUMBER)
                                {
                                    // Circular object: scale both x and y by the interpolated diameter
                                    double diameter = 2.0 * (rep->GetRadiusStart() + factor * (rep->GetRadiusEnd() - rep->GetRadiusStart()));
                                    scale_x         = diameter / dim_x;
                                    scale_y         = diameter / dim_y;
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
                        geom[2]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, static_cast<int>(vertices_bottom->size())));

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

                    if (tx == nullptr)
                    {
                        LOG_WARN("No tx");
                    }
                }

                // Tag this object's volume geometry for the entity view-mode toggle (',' / --view_mode).
                // Hidden by default when the object carries markings, shown otherwise.
                for (unsigned int ci = vol_child_begin; ci < objGroup->getNumChildren(); ci++)
                {
                    objGroup->getChild(ci)->setNodeMask(static_cast<unsigned int>(vol_mask));
                }

                // Wireframe and solid bounding box representations (toggle with the same view modes).
                CreateObjectBoundingBoxes(object, road, color, origin, objGroup.get());

                // Object markings (e.g. parking space lines, crosswalk stripes)
                if (object->GetNumberOfMarkings() > 0)
                {
                    osg::ref_ptr<osg::Group> markings_group = CreateObjectMarkings(object, road, origin);
                    if (markings_group != nullptr && markings_group->getNumChildren() > 0)
                    {
                        SetNodeName(*markings_group,
                                    prefix_road_object,
                                    object->GetId(),
                                    object->GetName().empty() ? object->GetTypeStr() : object->GetName() + "_markings");
                        objGroup->addChild(markings_group);
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

    osg::ref_ptr<osg::PositionAttitudeTransform> RoadGeom::LoadRoadFeature(roadmanager::Road* road, std::string file_path)
    {
        (void)road;
        osg::ref_ptr<osg::Node>                      node;
        osg::ref_ptr<osg::PositionAttitudeTransform> xform = 0;

        node = osgDB::readNodeFile(file_path);
        if (!node)
        {
            return 0;
        }

        xform = new osg::PositionAttitudeTransform;
        xform->addChild(node);

        return xform;
    }

    // Build a per-instance corner position provider for a repeated outline object:
    // - cornerRoad corners are re-evaluated on the road at the instance position (curvature aware)
    // - cornerLocal corners keep their fixed local shape, rigidly placed in the instance frame
    // The scale factors (along the object length/width and the height) are applied about the corner's
    // own reference so the outline shape is scaled by the repeat parameters, like the bounding box.
    // Build a corner placement function for one repeat instance. The corner positions were resolved once
    // in RMObject::GetRepeatInstances and stored on the instance in its local frame (shared with the OSI
    // reporter); here they are rotated and translated by the instance pose to obtain world coordinates.
    // The corner is matched by its index within the outline (outline corner ids are not guaranteed to be
    // unique - e.g. tree crown outlines omit ids, so they all default to 0 - hence an id based lookup
    // would collapse the whole outline onto a single corner).
    static std::function<void(roadmanager::OutlineCorner*, double&, double&, double&)>
    MakeInstanceOutlineCornerPosFn(const roadmanager::RepeatInstance& ri, roadmanager::Outline* outline, unsigned int outline_index)
    {
        const double ch = cos(ri.h);
        const double sh = sin(ri.h);
        return [ri, outline, outline_index, ch, sh](roadmanager::OutlineCorner* corner, double& x, double& y, double& z)
        {
            const roadmanager::ResolvedOutlineCorner* rc = nullptr;
            if (outline != nullptr && outline_index < ri.outline_corners.size())
            {
                const std::vector<roadmanager::ResolvedOutlineCorner>& group = ri.outline_corners[outline_index];
                for (size_t k = 0; k < outline->corner_.size() && k < group.size(); k++)
                {
                    if (outline->corner_[k] == corner)
                    {
                        rc = &group[k];
                        break;
                    }
                }
            }
            if (rc != nullptr)
            {
                x = ri.x + rc->x * ch - rc->y * sh;
                y = ri.y + rc->x * sh + rc->y * ch;
                z = rc->z;
            }
            else
            {
                corner->GetPos(x, y, z);
            }
        };
    }

    void RoadGeom::CreateRepeatedOutlineObjects(roadmanager::RMObject* object,
                                                roadmanager::Road*     road,
                                                osg::Vec4              color,
                                                const osg::Vec3d&      origin,
                                                osg::Group*            objGroup,
                                                const std::string&     obj_type,
                                                osg::Texture2D*        texture,
                                                double                 texture_scale)
    {
        roadmanager::Repeat* rep = object->GetRepeat();
        if (rep == nullptr || objGroup == nullptr)
        {
            return;
        }

        // Placement of every repeat copy is computed once in RoadManager (shared with the OSI reporter).
        int nCopies = 0;
        for (const roadmanager::RepeatInstance& ri : object->GetRepeatInstances(road))
        {
            for (unsigned int j = 0; j < object->GetNumberOfOutlines(); j++)
            {
                roadmanager::Outline*    outline       = object->GetOutline(j);
                auto                     corner_pos_fn = MakeInstanceOutlineCornerPosFn(ri, outline, j);
                osg::ref_ptr<osg::Group> olgroup = CreateOutlineObject(outline, color, origin, corner_pos_fn, ri.scale_hgt, texture, texture_scale);
                if (olgroup != nullptr)
                {
                    SetNodeName(*olgroup,
                                obj_type,
                                object->GetId(),
                                object->GetName().empty() ? object->GetTypeStr()
                                                          : object->GetName() + "_" + std::to_string(nCopies) + "_" + std::to_string(j));
                    objGroup->addChild(olgroup);
                }
            }
            nCopies++;
        }
    }

    osg::ref_ptr<osg::Group> RoadGeom::CreateOutlineObject(
        roadmanager::Outline*                                                              outline,
        osg::Vec4                                                                          color,
        const osg::Vec3d&                                                                  origin,
        const std::function<void(roadmanager::OutlineCorner*, double&, double&, double&)>& corner_pos_fn,
        double                                                                             height_scale,
        osg::Texture2D*                                                                    texture,
        double                                                                             texture_scale)
    {
        if (outline == 0)
        {
            return nullptr;
        }

        // Per-corner extrusion height, scaled by the repeat height factor (1.0 when not repeated/scaled).
        auto cornerHeight = [&](roadmanager::OutlineCorner* corner) { return corner->GetHeight() * height_scale; };

        // Resolve a corner world position, optionally via a caller provided function (e.g. per repeat
        // instance placement). Falls back to the corner's own road based position.
        auto getPos = [&](roadmanager::OutlineCorner* corner, double& x, double& y, double& z)
        {
            if (corner_pos_fn)
            {
                corner_pos_fn(corner, x, y, z);
            }
            else
            {
                corner->GetPos(x, y, z);
            }
        };

        bool roof = outline->roof_ ? true : false;

        // nrPoints will be corners + 1 if the outline should be closed, reusing first corner as last
        int nrPoints = outline->closed_ ? static_cast<int>(outline->corner_.size()) + 1 : static_cast<int>(outline->corner_.size());

        osg::ref_ptr<osg::Group> group = new osg::Group();

        osg::ref_ptr<osg::Vec3Array> vertices_sides =
            new osg::Vec3Array(static_cast<unsigned int>(nrPoints) * 2);                                         // one set at bottom and one at top
        osg::ref_ptr<osg::Vec3Array> vertices_top    = new osg::Vec3Array(static_cast<unsigned int>(nrPoints));  // top
        osg::ref_ptr<osg::Vec3Array> vertices_bottom = new osg::Vec3Array(static_cast<unsigned int>(nrPoints));  // bottom

        osg::ref_ptr<osg::Vec2Array> tex_coords_sides  = new osg::Vec2Array(static_cast<unsigned int>(nrPoints) * 2);
        osg::ref_ptr<osg::Vec2Array> tex_coords_top    = new osg::Vec2Array(static_cast<unsigned int>(nrPoints));
        osg::ref_ptr<osg::Vec2Array> tex_coords_bottom = new osg::Vec2Array(static_cast<unsigned int>(nrPoints));

        // Set vertices
        float cumulative_side_dist = 0.0f;
        for (size_t i = 0; i < outline->corner_.size(); i++)
        {
            double                      x, y, z;
            roadmanager::OutlineCorner* corner = outline->corner_[i];
            getPos(corner, x, y, z);
            (*vertices_sides)[i * 2 + 0].set(static_cast<float>(x - origin[0]),
                                             static_cast<float>(y - origin[1]),
                                             static_cast<float>(z + cornerHeight(corner)));
            (*vertices_sides)[i * 2 + 1].set(static_cast<float>(x - origin[0]), static_cast<float>(y - origin[1]), static_cast<float>(z));

            if (i > 0)
            {
                double x1, y1, z1;
                getPos(outline->corner_[i - 1], x1, y1, z1);
                float dx = x1 - x;
                float dy = y1 - y;
                cumulative_side_dist += std::sqrt(dx * dx + dy * dy);
            }

            (*tex_coords_sides)[i * 2 + 0].set(cumulative_side_dist / texture_scale, cornerHeight(corner) / texture_scale);
            (*tex_coords_sides)[i * 2 + 1].set(cumulative_side_dist / texture_scale, 0.0f);

            // top and bottom shapes
            if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_POLYGON)
            {
                (*vertices_top)[i].set(static_cast<float>(x - origin[0]),
                                       static_cast<float>(y - origin[1]),
                                       static_cast<float>(z + cornerHeight(corner)));
                (*vertices_bottom)[outline->corner_.size() - 1 - i].set(static_cast<float>(x - origin[0]),
                                                                        static_cast<float>(y - origin[1]),
                                                                        static_cast<float>(z));

                double x2, y2, z2;
                getPos(outline->corner_[outline->corner_.size() - 1 - i], x2, y2, z2);
                float dx    = x2 - x;
                float dy    = y2 - y;
                float width = std::sqrt(dx * dx + dy * dy);

                (*tex_coords_top)[i].set(0.0, cumulative_side_dist / texture_scale);
                (*tex_coords_top)[outline->corner_.size() - 1 - i].set(width / texture_scale, cumulative_side_dist / texture_scale);
                (*tex_coords_bottom)[i].set(0.0, cumulative_side_dist / texture_scale);
                (*tex_coords_bottom)[outline->corner_.size() - 1 - i].set(width / texture_scale, cumulative_side_dist / texture_scale);
            }
        }

        if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_QUAD_STRIP)
        {
            float cumulative_roof_dist = 0.0f;
            // rearrange vertices for quad strip
            for (size_t i = 0; i < outline->corner_.size(); i += 2)
            {
                unsigned right_index = outline->corner_.size() - 1 - (i / 2);
                unsigned left_index  = i / 2;

                double xl, yl, zl, xr, yr, zr;
                getPos(outline->corner_[left_index], xl, yl, zl);
                getPos(outline->corner_[right_index], xr, yr, zr);

                (*vertices_top)[i].set(static_cast<float>(xr - origin[0]),
                                       static_cast<float>(yr - origin[1]),
                                       static_cast<float>(zr + cornerHeight(outline->corner_[right_index])));
                (*vertices_top)[i + 1].set(static_cast<float>(xl - origin[0]),
                                           static_cast<float>(yl - origin[1]),
                                           static_cast<float>(zl + cornerHeight(outline->corner_[right_index])));
                (*vertices_bottom)[i].set(static_cast<float>(xl - origin[0]), static_cast<float>(yl - origin[1]), static_cast<float>(zl));
                (*vertices_bottom)[i + 1].set(static_cast<float>(xr - origin[0]), static_cast<float>(yr - origin[1]), static_cast<float>(zr));

                osg::Vec3f left(xl, yl, zl);
                osg::Vec3f right(xr, yr, zr);
                float      width = (right - left).length();

                if (i >= 2)
                {
                    unsigned left_index_prev  = (i - 1) / 2;
                    unsigned right_index_prev = outline->corner_.size() - 1 - ((i - 2) / 2);

                    double xl_prev, yl_prev, zl_prev, xr_prev, yr_prev, zr_prev;
                    getPos(outline->corner_[left_index_prev], xl_prev, yl_prev, zl_prev);
                    getPos(outline->corner_[right_index_prev], xr_prev, yr_prev, zr_prev);

                    osg::Vec3f left_prev(xl_prev, yl_prev, zl_prev);
                    osg::Vec3f right_prev(xr_prev, yr_prev, zr_prev);

                    osg::Vec3f center_prev = (left_prev + right_prev) * 0.5f;
                    osg::Vec3f center      = (left + right) * 0.5f;

                    cumulative_roof_dist += (center - center_prev).length();
                }

                (*tex_coords_top)[i].set(width / texture_scale, cumulative_roof_dist / texture_scale);
                (*tex_coords_top)[i + 1].set(0.0f, cumulative_roof_dist / texture_scale);
                (*tex_coords_bottom)[i].set(width / texture_scale, cumulative_roof_dist / texture_scale);
                (*tex_coords_bottom)[i + 1].set(0.0f, cumulative_roof_dist / texture_scale);
            }
        }

        // Close geometry
        if (outline->closed_)
        {
            cumulative_side_dist += ((*vertices_sides)[0] - (*vertices_sides)[2 * (nrPoints - 2)]).length();

            (*vertices_sides)[2 * static_cast<unsigned int>(nrPoints) - 2].set((*vertices_sides)[0]);
            (*vertices_sides)[2 * static_cast<unsigned int>(nrPoints) - 1].set((*vertices_sides)[1]);
            (*tex_coords_sides)[2 * static_cast<unsigned int>(nrPoints) - 2].set(cumulative_side_dist / texture_scale, (*tex_coords_sides)[0].y());
            (*tex_coords_sides)[2 * static_cast<unsigned int>(nrPoints) - 1].set(cumulative_side_dist / texture_scale, (*tex_coords_sides)[1].y());

            (*vertices_top)[static_cast<unsigned int>(nrPoints) - 1].set((*vertices_top)[0]);
            (*vertices_bottom)[static_cast<unsigned int>(nrPoints) - 1].set((*vertices_bottom)[0]);
            (*tex_coords_top)[static_cast<unsigned int>(nrPoints) - 1].set((*tex_coords_top)[0]);
            (*tex_coords_bottom)[static_cast<unsigned int>(nrPoints) - 1].set((*tex_coords_bottom)[0]);
        }

        // Finally create and add geometry
        osg::ref_ptr<osg::Geode>    geode  = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geom[] = {new osg::Geometry, new osg::Geometry, new osg::Geometry};

        geom[0]->setVertexArray(vertices_sides.get());
        geom[0]->setTexCoordArray(0, tex_coords_sides.get());
        geom[0]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, 2 * nrPoints));

        if (roof)
        {
            geom[1]->setVertexArray(vertices_top.get());
            geom[1]->setTexCoordArray(0, tex_coords_top.get());
            geom[2]->setVertexArray(vertices_bottom.get());
            geom[2]->setTexCoordArray(0, tex_coords_bottom.get());
            if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_POLYGON)
            {
                geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, nrPoints));
                osgUtil::Tessellator tessellator;
                tessellator.retessellatePolygons(*geom[1]);

                geom[2]->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, nrPoints));
                tessellator.retessellatePolygons(*geom[2]);
            }
            else
            {
                geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, nrPoints - 1));
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

        if (texture != nullptr)
        {
            geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
            // set color to white for texture mapping, but keep alpha
            color[0] = 1.0f;
            color[1] = 1.0f;
            color[2] = 1.0f;
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

        return group;
    }

    void RoadGeom::CreateObjectBoundingBoxes(roadmanager::RMObject* object,
                                             roadmanager::Road*     road,
                                             osg::Vec4              color,
                                             const osg::Vec3d&      origin,
                                             osg::Group*            objGroup,
                                             osg::Texture2D*        texture,
                                             double                 texture_scale)
    {
        if (object == nullptr || road == nullptr || objGroup == nullptr)
        {
            return;
        }

        osg::ref_ptr<osg::Group> bb_wire = new osg::Group;  // wireframe boxes
        osg::ref_ptr<osg::Group> bb_fill = new osg::Group;  // solid boxes

        // Add one box (wireframe + solid) for a single object instance. The box is oriented by
        // inst_h, positioned so its base rests at inst_z, and offset within the local frame by
        // (loc_cx, loc_cy) so it can be centered on geometry that is not symmetric about the
        // object reference (e.g. an outline whose corners are given by unique s/t coordinates).
        auto emit_box = [&](double inst_x,
                            double inst_y,
                            double inst_z,
                            double inst_h,
                            double loc_cx,
                            double loc_cy,
                            double inst_len,
                            double inst_wid,
                            double inst_height)
        {
            const float len = MAX(0.05f, static_cast<float>(inst_len));
            const float wid = MAX(0.05f, static_cast<float>(inst_wid));
            const float hgt = MAX(0.05f, static_cast<float>(inst_height));

            osg::ref_ptr<osg::Box> box = new osg::Box(osg::Vec3(static_cast<float>(loc_cx), static_cast<float>(loc_cy), 0.5f * hgt), len, wid, hgt);
            osg::ref_ptr<osg::ShapeDrawable> wire_shape = new osg::ShapeDrawable(box.get());
            osg::ref_ptr<osg::ShapeDrawable> fill_shape = new osg::ShapeDrawable(box.get());
            wire_shape->setColor(color);
            fill_shape->setColor(color);

            osg::ref_ptr<osg::Geode> wire_geode = new osg::Geode;
            osg::ref_ptr<osg::Geode> fill_geode = new osg::Geode;
            wire_geode->addDrawable(wire_shape.get());
            fill_geode->addDrawable(fill_shape.get());

            osg::ref_ptr<osg::PositionAttitudeTransform> wire_tx = new osg::PositionAttitudeTransform;
            osg::ref_ptr<osg::PositionAttitudeTransform> fill_tx = new osg::PositionAttitudeTransform;
            const osg::Vec3 p(static_cast<float>(inst_x - origin[0]), static_cast<float>(inst_y - origin[1]), static_cast<float>(inst_z - origin[2]));
            const osg::Quat q(inst_h, osg::Vec3(osg::Z_AXIS));
            wire_tx->setPosition(p);
            fill_tx->setPosition(p);
            wire_tx->setAttitude(q);
            fill_tx->setAttitude(q);
            wire_tx->addChild(wire_geode.get());
            fill_tx->addChild(fill_geode.get());

            bb_wire->addChild(wire_tx.get());
            bb_fill->addChild(fill_tx.get());
        };

        // Emit the actual outline based shape (not a simplified box) for a single instance. The same
        // geometry is added to both the wireframe and the solid groups so the bounding box views show
        // the true object shape. When a repeat instance is given, each corner is resolved from the
        // pre-computed instance corners (matched by index within the outline); otherwise the corner's
        // own road position is used. cornerRoad corners keep their unique s/t (and follow the road).
        auto emit_outline_shape = [&](const roadmanager::RepeatInstance* ri_ptr)
        {
            const double height_scale = (ri_ptr != nullptr) ? ri_ptr->scale_hgt : 1.0;
            for (unsigned int j = 0; j < object->GetNumberOfOutlines(); j++)
            {
                roadmanager::Outline* outline = object->GetOutline(j);
                if (outline == nullptr)
                {
                    continue;
                }
                std::function<void(roadmanager::OutlineCorner*, double&, double&, double&)> corner_pos_fn;
                if (ri_ptr != nullptr)
                {
                    corner_pos_fn = MakeInstanceOutlineCornerPosFn(*ri_ptr, outline, j);
                }
                else
                {
                    corner_pos_fn = [](roadmanager::OutlineCorner* corner, double& x, double& y, double& z) { corner->GetPos(x, y, z); };
                }
                osg::ref_ptr<osg::Group> fill = CreateOutlineObject(outline, color, origin, corner_pos_fn, height_scale, texture, texture_scale);
                osg::ref_ptr<osg::Group> wire = CreateOutlineObject(outline, color, origin, corner_pos_fn, height_scale, texture, texture_scale);
                if (fill != nullptr)
                {
                    bb_fill->addChild(fill.get());
                }
                if (wire != nullptr)
                {
                    bb_wire->addChild(wire.get());
                }
            }
        };

        roadmanager::Repeat*  rep             = object->GetRepeat();
        const bool            separate_copies = (rep != nullptr && rep->GetLength() > SMALL_NUMBER && rep->GetDistance() > SMALL_NUMBER);
        const bool            has_outlines    = object->GetNumberOfOutlines() > 0;
        roadmanager::Position pos;

        if (!separate_copies)
        {
            pos.SetTrackPosMode(road->GetId(),
                                object->GetS(),
                                object->GetT(),
                                roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::Z_REL |
                                    roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);

            const double inst_h = pos.GetH() + object->GetHOffset();
            const double inst_z = pos.GetZ() + object->GetZOffset();

            if (has_outlines)
            {
                // Actual outline shape (unique s/t per corner), placed via each corner's own road position.
                emit_outline_shape(nullptr);
            }
            else
            {
                emit_box(pos.GetX(), pos.GetY(), inst_z, inst_h, 0.0, 0.0, object->GetLength(), object->GetWidth(), object->GetHeight());
            }
        }
        else
        {
            for (const roadmanager::RepeatInstance& ri : object->GetRepeatInstances(road))
            {
                if (has_outlines)
                {
                    // Per-instance corner placement, matching CreateRepeatedOutlineObjects:
                    // the corner positions resolved once in RoadManager are reused (cornerRoad follows
                    // the road curvature, cornerLocal keeps a fixed shape).
                    emit_outline_shape(&ri);
                }
                else
                {
                    emit_box(ri.x, ri.y, ri.z, ri.h, 0.0, 0.0, ri.inst_len, ri.inst_wid, ri.inst_hgt);
                }
            }
        }

        if (bb_wire->getNumChildren() == 0)
        {
            return;
        }

        // Wireframe rendering for the outline boxes
        osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
        polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
        osg::StateSet* wire_ss = bb_wire->getOrCreateStateSet();
        wire_ss->setAttributeAndModes(polygonMode.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
        wire_ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

        bb_wire->setNodeMask(NODE_MASK_ENTITY_BB);
        bb_fill->setNodeMask(NODE_MASK_ENTITY_BB_FILLED);

        SetNodeName(*bb_wire, prefix_road_object, object->GetId(), object->GetName().empty() ? object->GetTypeStr() : object->GetName() + "_bb");
        SetNodeName(*bb_fill,
                    prefix_road_object,
                    object->GetId(),
                    object->GetName().empty() ? object->GetTypeStr() : object->GetName() + "_bb_filled");

        objGroup->addChild(bb_wire.get());
        objGroup->addChild(bb_fill.get());
    }

    osg::ref_ptr<osg::Group> RoadGeom::CreateObjectMarkings(roadmanager::RMObject* object, roadmanager::Road* road, const osg::Vec3d& origin)
    {
        if (object == nullptr || object->GetNumberOfMarkings() == 0)
        {
            return nullptr;
        }

        osg::ref_ptr<osg::Group> group = new osg::Group();

        // For each object instance, build marking geometry. Outline (cornerReference) markings are
        // emitted directly and their interior corners are mitered (stitched) so consecutive edges
        // meet cleanly. Bounding box "side" markings are collected per instance and chained together
        // so that adjacent sides sharing a box corner are stitched, gated by their start/stop offsets.
        // cornerRoad outline corners are evaluated on the road per instance (so they follow the road
        // curvature); cornerLocal corners and side markings use the object local (u, v) frame.
        struct SideEdge
        {
            osg::Vec3d                 p0, p1;
            double                     half_w         = 0.0;
            double                     line_length    = 0.0;
            double                     space_length   = 0.0;
            double                     start_offset   = 0.0;
            double                     stop_offset    = 0.0;
            double                     lateral_offset = 0.0;
            roadmanager::RoadMarkColor color          = roadmanager::RoadMarkColor::WHITE;
        };

        auto emit_instance = [&](const roadmanager::RepeatInstance& ri)
        {
            const double inst_x   = ri.x;
            const double inst_y   = ri.y;
            const double inst_z   = ri.z;
            const double inst_h   = ri.h;
            const double inst_len = ri.inst_len;
            const double inst_wid = ri.inst_wid;
            const double ch       = cos(inst_h);
            const double sh       = sin(inst_h);

            std::vector<SideEdge> side_edges;

            // Shift a centerline polyline sideways (custom "lateralOffset" userData feature). Positive
            // moves to the left of the edge direction, negative to the right. seg_off holds the offset
            // per edge (seg_off[i] applies to the segment verts[i]..verts[i+1]). Interior vertices are
            // offset along their miter (bisector) so the line keeps a constant lateral distance, which
            // preserves the corner stitching.
            auto offset_polyline = [](std::vector<osg::Vec3d>& pl, const std::vector<double>& seg_off)
            {
                if (pl.size() < 2 || seg_off.size() + 1 != pl.size())
                {
                    return;
                }
                bool any = false;
                for (double o : seg_off)
                {
                    if (fabs(o) > SMALL_NUMBER)
                    {
                        any = true;
                        break;
                    }
                }
                if (!any)
                {
                    return;
                }
                auto leftPerp = [](const osg::Vec3d& a, const osg::Vec3d& b) -> osg::Vec3d
                {
                    osg::Vec3d d     = b - a;
                    d.z()            = 0.0;
                    const double len = d.length();
                    if (len < 1e-9)
                    {
                        return osg::Vec3d(0.0, 0.0, 0.0);
                    }
                    d /= len;
                    return osg::Vec3d(-d.y(), d.x(), 0.0);
                };
                const size_t            n = pl.size();
                std::vector<osg::Vec3d> result(n);
                for (size_t i = 0; i < n; i++)
                {
                    if (i == 0)
                    {
                        result[i] = pl[i] + leftPerp(pl[0], pl[1]) * seg_off[0];
                    }
                    else if (i == n - 1)
                    {
                        result[i] = pl[i] + leftPerp(pl[n - 2], pl[n - 1]) * seg_off[n - 2];
                    }
                    else
                    {
                        const osg::Vec3d lp0 = leftPerp(pl[i - 1], pl[i]);
                        const osg::Vec3d lp1 = leftPerp(pl[i], pl[i + 1]);
                        const double     off = 0.5 * (seg_off[i - 1] + seg_off[i]);
                        osg::Vec3d       m   = lp0 + lp1;
                        const double     ml  = m.length();
                        if (ml < 1e-6)
                        {
                            result[i] = pl[i] + lp0 * off;  // ~180 deg turn, fall back to edge perpendicular
                        }
                        else
                        {
                            m /= ml;
                            const double cosHalf = m * lp0;
                            const double scale   = (fabs(cosHalf) > 1e-3) ? off / cosHalf : off;
                            result[i]            = pl[i] + m * scale;
                        }
                    }
                }
                pl = result;
            };

            for (unsigned int m = 0; m < object->GetNumberOfMarkings(); m++)
            {
                const roadmanager::ObjectMarking* marking = object->GetMarking(m);
                const double                      z       = inst_z + marking->z_offset_;

                if (marking->UsesOutline())
                {
                    std::vector<osg::Vec3d> polyline;  // world coordinates (origin subtracted later)

                    for (id_t corner_id : marking->corner_references_)
                    {
                        const roadmanager::ResolvedOutlineCorner* rc = ri.FindCorner(corner_id);
                        if (rc == nullptr)
                        {
                            LOG_WARN("Object {} marking references unknown outline corner id {}, skipping point", object->GetName(), corner_id);
                            continue;
                        }
                        // Reuse the resolved corner (scaled edge, object floor) and transform to world.
                        const double wx = inst_x + rc->x * ch - rc->y * sh;
                        const double wy = inst_y + rc->x * sh + rc->y * ch;
                        polyline.emplace_back(wx, wy, rc->marking_z + marking->z_offset_);
                    }

                    // Apply optional lateral shift (left +, right -) to the marking centerline.
                    if (polyline.size() >= 2)
                    {
                        offset_polyline(polyline, std::vector<double>(polyline.size() - 1, marking->lateral_offset_));
                    }

                    if (polyline.size() >= 2)
                    {
                        // Within a single outline marking the interior corners are always stitched; the
                        // start/stop offsets only trim the two open ends of the polyline.
                        const size_t        edge_count = polyline.size() - 1;
                        std::vector<double> half_w(edge_count, 0.5 * marking->width_);
                        std::vector<double> line_len(edge_count, marking->line_length_);
                        std::vector<double> space_len(edge_count, marking->space_length_);
                        std::vector<bool>   miter(polyline.size(), false);
                        for (size_t v = 1; v + 1 < polyline.size(); v++)
                        {
                            miter[v] = true;
                        }
                        CreateObjectMarkingChainGeom(polyline,
                                                     half_w,
                                                     line_len,
                                                     space_len,
                                                     miter,
                                                     marking->start_offset_,
                                                     marking->stop_offset_,
                                                     marking->color_,
                                                     origin,
                                                     group.get());
                    }
                }
                else if (marking->side_ != roadmanager::ObjectMarking::Side::NONE)
                {
                    // Marking attached to one side of the object bounding box (local u/v frame).
                    // u points along object heading (length), v to the left (width).
                    double half_l = 0.5 * inst_len;
                    double half_w = 0.5 * inst_wid;
                    double u0 = 0.0, v0 = 0.0, u1 = 0.0, v1 = 0.0;
                    switch (marking->side_)
                    {
                        case roadmanager::ObjectMarking::Side::FRONT:
                            u0 = half_l;
                            v0 = -half_w;
                            u1 = half_l;
                            v1 = half_w;
                            break;
                        case roadmanager::ObjectMarking::Side::REAR:
                            u0 = -half_l;
                            v0 = -half_w;
                            u1 = -half_l;
                            v1 = half_w;
                            break;
                        case roadmanager::ObjectMarking::Side::LEFT:
                            u0 = -half_l;
                            v0 = half_w;
                            u1 = half_l;
                            v1 = half_w;
                            break;
                        case roadmanager::ObjectMarking::Side::RIGHT:
                            u0 = -half_l;
                            v0 = -half_w;
                            u1 = half_l;
                            v1 = -half_w;
                            break;
                        default:
                            break;
                    }
                    SideEdge se;
                    se.p0.set(inst_x + u0 * ch - v0 * sh, inst_y + u0 * sh + v0 * ch, z);
                    se.p1.set(inst_x + u1 * ch - v1 * sh, inst_y + u1 * sh + v1 * ch, z);

                    // Keep the un-offset edge endpoints so adjacent sides still share corners for
                    // stitching; the lateral shift is applied to the assembled chain polyline below.
                    se.half_w         = 0.5 * marking->width_;
                    se.line_length    = marking->line_length_;
                    se.space_length   = marking->space_length_;
                    se.start_offset   = marking->start_offset_;
                    se.stop_offset    = marking->stop_offset_;
                    se.lateral_offset = marking->lateral_offset_;
                    se.color          = marking->color_;
                    side_edges.push_back(se);
                }
            }

            // Chain the side markings: join edges that share a box corner into ordered polylines and
            // stitch (miter) a shared corner only when the incoming edge has no stop offset and the
            // outgoing edge has no start offset at that corner.
            if (!side_edges.empty())
            {
                struct OEdge
                {
                    osg::Vec3d                 s, e;
                    double                     half_w, line_length, space_length, start_off, end_off, lateral_off;
                    roadmanager::RoadMarkColor color;
                };
                const double PT_EPS = 1e-3;
                auto         samePt = [PT_EPS](const osg::Vec3d& a, const osg::Vec3d& b) { return (a - b).length() < PT_EPS; };
                auto         toOE   = [](const SideEdge& se, bool rev)
                {
                    OEdge o;
                    if (!rev)
                    {
                        o.s         = se.p0;
                        o.e         = se.p1;
                        o.start_off = se.start_offset;
                        o.end_off   = se.stop_offset;
                    }
                    else
                    {
                        o.s         = se.p1;
                        o.e         = se.p0;
                        o.start_off = se.stop_offset;
                        o.end_off   = se.start_offset;
                    }
                    o.half_w       = se.half_w;
                    o.line_length  = se.line_length;
                    o.space_length = se.space_length;
                    o.lateral_off  = se.lateral_offset;
                    o.color        = se.color;
                    return o;
                };

                std::vector<bool> used(side_edges.size(), false);
                for (size_t i = 0; i < side_edges.size(); i++)
                {
                    if (used[i])
                    {
                        continue;
                    }
                    std::vector<OEdge> chain;
                    chain.push_back(toOE(side_edges[i], false));
                    used[i] = true;

                    // Extend the chain forward (matching the current end point) ...
                    bool extended = true;
                    while (extended)
                    {
                        extended = false;
                        for (size_t j = 0; j < side_edges.size(); j++)
                        {
                            if (used[j])
                            {
                                continue;
                            }
                            if (samePt(side_edges[j].p0, chain.back().e))
                            {
                                chain.push_back(toOE(side_edges[j], false));
                                used[j]  = true;
                                extended = true;
                                break;
                            }
                            if (samePt(side_edges[j].p1, chain.back().e))
                            {
                                chain.push_back(toOE(side_edges[j], true));
                                used[j]  = true;
                                extended = true;
                                break;
                            }
                        }
                    }
                    // ... and backward (matching the current start point).
                    extended = true;
                    while (extended)
                    {
                        extended = false;
                        for (size_t j = 0; j < side_edges.size(); j++)
                        {
                            if (used[j])
                            {
                                continue;
                            }
                            if (samePt(side_edges[j].p1, chain.front().s))
                            {
                                chain.insert(chain.begin(), toOE(side_edges[j], false));
                                used[j]  = true;
                                extended = true;
                                break;
                            }
                            if (samePt(side_edges[j].p0, chain.front().s))
                            {
                                chain.insert(chain.begin(), toOE(side_edges[j], true));
                                used[j]  = true;
                                extended = true;
                                break;
                            }
                        }
                    }

                    // Enforce a consistent winding (counter-clockwise) on the assembled chain. The
                    // greedy stitching above seeds from an arbitrary edge, so two otherwise identical
                    // boxes (e.g. mirrored on opposite sides of the road) can come out traversed in
                    // opposite directions. Since the lateral offset is applied along the left
                    // perpendicular of the travel direction, an inconsistent winding would make the
                    // same offset sign shift opposite physical ways. Forcing CCW ties the sign to a
                    // consistent side (positive = toward the box interior) regardless of seed/mirroring.
                    {
                        double     area2 = 0.0;  // twice the signed area (shoelace), >0 => CCW
                        osg::Vec3d prev  = chain.front().s;
                        for (const OEdge& oe : chain)
                        {
                            area2 += prev.x() * oe.e.y() - oe.e.x() * prev.y();
                            prev = oe.e;
                        }
                        area2 += prev.x() * chain.front().s.y() - chain.front().s.x() * prev.y();
                        if (area2 < 0.0)  // clockwise -> reverse to make it CCW
                        {
                            std::reverse(chain.begin(), chain.end());
                            for (OEdge& oe : chain)
                            {
                                std::swap(oe.s, oe.e);
                                std::swap(oe.start_off, oe.end_off);
                            }
                        }
                    }

                    std::vector<osg::Vec3d> verts;
                    std::vector<double>     chain_half_w, chain_line_len, chain_space_len, chain_lateral;
                    verts.push_back(chain.front().s);
                    for (const OEdge& oe : chain)
                    {
                        verts.push_back(oe.e);
                        chain_half_w.push_back(oe.half_w);
                        chain_line_len.push_back(oe.line_length);
                        chain_space_len.push_back(oe.space_length);
                        chain_lateral.push_back(oe.lateral_off);
                    }
                    // Apply the lateral shift on the assembled (still corner-sharing) chain so the
                    // stitched corners move together and stay joined.
                    offset_polyline(verts, chain_lateral);
                    std::vector<bool> miter(verts.size(), false);
                    for (size_t v = 1; v + 1 < verts.size(); v++)
                    {
                        miter[v] = (chain[v - 1].end_off < SMALL_NUMBER && chain[v].start_off < SMALL_NUMBER);
                    }
                    CreateObjectMarkingChainGeom(verts,
                                                 chain_half_w,
                                                 chain_line_len,
                                                 chain_space_len,
                                                 miter,
                                                 chain.front().start_off,
                                                 chain.back().end_off,
                                                 chain.front().color,
                                                 origin,
                                                 group.get());
                }
            }
        };

        // Replicate markings for each resolved instance (single object, or one per repeat copy). The
        // instance placement and corner resolution are computed once in RoadManager so the viewer and
        // OSI stay in sync.
        for (const roadmanager::RepeatInstance& ri : object->GetRepeatInstances(road))
        {
            emit_instance(ri);
        }

        return group;
    }

    void RoadGeom::CreateObjectMarkingChainGeom(const std::vector<osg::Vec3d>& verts,
                                                const std::vector<double>&     half_w,
                                                const std::vector<double>&     line_length,
                                                const std::vector<double>&     space_length,
                                                const std::vector<bool>&       miter,
                                                double                         start_offset,
                                                double                         stop_offset,
                                                roadmanager::RoadMarkColor     color,
                                                const osg::Vec3d&              origin,
                                                osg::Group*                    group)
    {
        const size_t N = verts.size();
        if (N < 2 || group == nullptr)
        {
            return;
        }
        const size_t E = N - 1;

        // Per-edge unit direction and left-hand perpendicular (the marking lies flat, so the
        // perpendicular is horizontal).
        std::vector<osg::Vec3d> dir(E), perp(E);
        for (size_t e = 0; e < E; e++)
        {
            osg::Vec3d d = verts[e + 1] - verts[e];
            double     l = d.length();
            if (l < SMALL_NUMBER)
            {
                dir[e].set(0.0, 0.0, 0.0);
                perp[e].set(0.0, 0.0, 0.0);
            }
            else
            {
                dir[e] = d / l;
                perp[e].set(-dir[e].y(), dir[e].x(), 0.0);
            }
        }

        // 2D line/line intersection (x/y plane). Returns false if the two lines are near-parallel.
        auto intersect2D = [](const osg::Vec3d& p, const osg::Vec3d& d, const osg::Vec3d& q, const osg::Vec3d& e, osg::Vec3d& out) -> bool
        {
            double denom = d.x() * e.y() - d.y() * e.x();
            if (fabs(denom) < SMALL_NUMBER)
            {
                return false;
            }
            double t = ((q.x() - p.x()) * e.y() - (q.y() - p.y()) * e.x()) / denom;
            out.set(p.x() + t * d.x(), p.y() + t * d.y(), p.z());
            return true;
        };

        // Miter points (left/right ribbon boundary) at the interior vertices that should be stitched:
        // the outer offset lines are extrapolated and the inner ones trimmed to their intersection.
        std::vector<osg::Vec3d> miter_l(N), miter_r(N);
        std::vector<bool>       miter_ok(N, false);
        const double            MITER_LIMIT = 6.0;  // cap miter length / half-width to avoid spikes
        for (size_t v = 1; v + 1 < N; v++)
        {
            if (!miter[v] || dir[v - 1].length() < SMALL_NUMBER || dir[v].length() < SMALL_NUMBER)
            {
                continue;
            }
            const osg::Vec3d& d_in   = dir[v - 1];
            const osg::Vec3d& p_in   = perp[v - 1];
            const osg::Vec3d& d_out  = dir[v];
            const osg::Vec3d& p_out  = perp[v];
            const double      hw_in  = half_w[v - 1];
            const double      hw_out = half_w[v];
            osg::Vec3d        ml, mr;
            const bool        ok_l = intersect2D(verts[v] + p_in * hw_in, d_in, verts[v] + p_out * hw_out, d_out, ml);
            const bool        ok_r = intersect2D(verts[v] - p_in * hw_in, d_in, verts[v] - p_out * hw_out, d_out, mr);
            const double      lim  = MITER_LIMIT * MAX(hw_in, hw_out);
            if (ok_l && ok_r && (ml - verts[v]).length() < lim && (mr - verts[v]).length() < lim)
            {
                miter_l[v]  = ml;
                miter_r[v]  = mr;
                miter_ok[v] = true;
            }
        }

        osg::ref_ptr<osg::Vec3Array>        vertices = new osg::Vec3Array;
        osg::ref_ptr<osg::DrawElementsUInt> indices  = new osg::DrawElementsUInt(GL_TRIANGLES);

        for (size_t e = 0; e < E; e++)
        {
            if (dir[e].length() < SMALL_NUMBER)
            {
                continue;
            }
            const osg::Vec3d& d  = dir[e];
            const osg::Vec3d& p  = perp[e];
            const double      hw = half_w[e];
            const osg::Vec3d& P0 = verts[e];
            const osg::Vec3d& P1 = verts[e + 1];

            // Ribbon boundary (left/right) and centerline point at the two ends of this edge.
            osg::Vec3d start_l, start_r, end_l, end_r, cstart, cend;

            if (e > 0 && miter_ok[e])
            {
                start_l = miter_l[e];
                start_r = miter_r[e];
                cstart  = P0;
            }
            else if (e == 0)
            {
                cstart  = P0 + d * MAX(0.0, start_offset);
                start_l = cstart + p * hw;
                start_r = cstart - p * hw;
            }
            else
            {
                cstart  = P0;
                start_l = P0 + p * hw;
                start_r = P0 - p * hw;
            }

            if (e + 1 < N - 1 && miter_ok[e + 1])
            {
                end_l = miter_l[e + 1];
                end_r = miter_r[e + 1];
                cend  = P1;
            }
            else if (e == E - 1)
            {
                cend  = P1 - d * MAX(0.0, stop_offset);
                end_l = cend + p * hw;
                end_r = cend - p * hw;
            }
            else
            {
                cend  = P1;
                end_l = P1 + p * hw;
                end_r = P1 - p * hw;
            }

            const double seg_len = (cend - cstart).length();
            if (seg_len < SMALL_NUMBER)
            {
                continue;
            }

            // Dash intervals along the edge centerline [0, seg_len].
            std::vector<std::pair<double, double>> dashes;
            if (space_length[e] < SMALL_NUMBER)
            {
                dashes.emplace_back(0.0, seg_len);  // solid
            }
            else if (line_length[e] >= SMALL_NUMBER)
            {
                const double period = line_length[e] + space_length[e];
                for (double d0 = 0.0; d0 < seg_len - SMALL_NUMBER; d0 += period)
                {
                    double d1 = MIN(d0 + line_length[e], seg_len);
                    if (d1 > d0 + SMALL_NUMBER)
                    {
                        dashes.emplace_back(d0, d1);
                    }
                }
            }

            for (const std::pair<double, double>& dash : dashes)
            {
                const double a_lo = dash.first / seg_len;
                const double a_hi = dash.second / seg_len;
                osg::Vec3d   qa_l = start_l + (end_l - start_l) * a_lo - origin;
                osg::Vec3d   qa_r = start_r + (end_r - start_r) * a_lo - origin;
                osg::Vec3d   qb_l = start_l + (end_l - start_l) * a_hi - origin;
                osg::Vec3d   qb_r = start_r + (end_r - start_r) * a_hi - origin;

                unsigned int base = static_cast<unsigned int>(vertices->size());
                vertices->push_back(osg::Vec3(qa_l));
                vertices->push_back(osg::Vec3(qa_r));
                vertices->push_back(osg::Vec3(qb_r));
                vertices->push_back(osg::Vec3(qb_l));

                indices->push_back(base + 0);
                indices->push_back(base + 1);
                indices->push_back(base + 2);
                indices->push_back(base + 0);
                indices->push_back(base + 2);
                indices->push_back(base + 3);
            }
        }

        if (vertices->empty())
        {
            return;
        }

        osg::Vec4 osg_color = ODR2OSGColor(color);

        osg::ref_ptr<osg::Vec4Array> color_array = new osg::Vec4Array;
        color_array->push_back(osg_color);

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setUseDisplayList(true);
        geom->setVertexArray(vertices.get());
        geom->addPrimitiveSet(indices.get());
        geom->setColorArray(color_array.get());
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);

        // Use PolygonOffset to avoid z-fighting with the road / object surface
        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(-POLYGON_OFFSET_ROADMARKS, -SIGN(POLYGON_OFFSET_ROADMARKS)));

        osgUtil::SmoothingVisitor::smooth(*geom, 0.0);

        osg::ref_ptr<osg::Material> material = GetOrCreateMaterial("ObjectMarking_" + roadmanager::LaneRoadMark::RoadMarkColor2Str(color),
                                                                   osg_color,
                                                                   static_cast<uint8_t>(RoadGeom::MaterialType::ROADMARK));

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->getOrCreateStateSet()->setAttributeAndModes(material.get());
        geode->addDrawable(geom.get());
        group->addChild(geode);
    }

    int RoadGeom::AddGroundSurface()
    {
        const double margin   = 1E4;
        const double z_offset = -1.0;
        // const osg::BoundingSphere bs = environment_->getBound();

        osg::ComputeBoundsVisitor cbv;
        osg::BoundingBox          bb;

        osg::Node* bb_node = environment_ != nullptr ? environment_ : root_->getNumChildren() > 0 ? root_->asNode() : nullptr;
        if (bb_node != nullptr)
        {
            bb_node->accept(cbv);
            bb = cbv.getBoundingBox();
        }
        else
        {
            bb.set(osg::Vec3d(0.0, 0.0, 0.0), osg::Vec3d(1e4, 1e4, 1e4));
        }

        osg::ref_ptr<osg::Geode>    ground = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geom   = osg::createTexturedQuadGeometry(
            osg::Vec3(bb.xMin() - static_cast<float>(margin), bb.yMin() - static_cast<float>(margin), bb.zMin() + static_cast<float>(z_offset)),
            osg::Vec3(0.0f, 2.0f * static_cast<float>(margin) + (bb.yMax() - bb.yMin()), bb.zMin() + static_cast<float>(z_offset)),
            osg::Vec3(2.0f * static_cast<float>(margin) + (bb.xMax() - bb.xMin()), 0.0f, bb.zMin() + static_cast<float>(z_offset)));
        osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
        color->push_back(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
        geom->setColorArray(color.get());
        geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
        ground->addDrawable(geom);

        root_->addChild(ground.get());

        return 0;
    }

    void RoadGeom::SetNodeName(osg::Node& node, const std::string& prefix, id_t id, const std::string& label)
    {
        node.setName(prefix + std::to_string(id) + "_" + label);
    }

    int RoadGeom::SaveToFile(const std::string& filename)
    {
        osgDB::writeNodeFile(*root_, filename);
        return 0;
    }

    TrafficLightModel* RoadGeom::GetTrafficLightModel(int id)
    {
        auto it = traffic_light_.find(id);
        if (it != traffic_light_.end())
        {
            return &it->second;
        }
        return nullptr;
    }

}  // namespace roadgeom
