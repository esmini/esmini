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
#include <osg/Material>
#include <osgGA/StateSetManipulator>
#include <osg/PolygonOffset>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>
#include <osg/ShapeDrawable>

#include "CommonMini.hpp"
#include "viewer.hpp"

#define GEOM_TOLERANCE  (0.2 - SMALL_NUMBER)  // Minimum distance between two vertices along road s-axis
#define TEXTURE_SCALE   0.5                   // Scale factor for asphalt and grass textures 1.0 means whole texture fits in 1 x 1 m square
#define MAX_GEOM_ERROR  0.25                  // maximum distance from the 3D geometry to the OSI lines
#define MAX_GEOM_LENGTH 50                    // maximum length of a road geometry mesh segment
#define MIN_GEOM_LENGTH 0.1                   // minimum length of a road geometry mesh segment, adjust if possible

#define POLYGON_OFFSET_SIDEWALK  2.0
#define POLYGON_OFFSET_ROADMARKS 1.0
#define POLYGON_OFFSET_BORDER    -1.0
#define POLYGON_OFFSET_GRASS     -2.0

#define ROADMARK_Z_OFFSET 0.02

const static double friction_max     = 5.0;
const static double friction_default = 1.0;
static osg::Vec3d   origin_          = {0.0, 0.0, 0.0};

bool compare_s_values(double s0, double s1)
{
    return (fabs(s1 - s0) < 0.1);
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

void RoadGeom::AddRoadMarkGeom(osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::DrawElementsUInt> indices, roadmanager::RoadMarkColor color)
{
    osg::ref_ptr<osg::Material>  materialRoadmark_ = new osg::Material;
    osg::ref_ptr<osg::Vec4Array> color_array       = new osg::Vec4Array;
    color_array->push_back(viewer::ODR2OSGColor(color));

    materialRoadmark_->setDiffuse(osg::Material::FRONT_AND_BACK, color_array->at(0));
    materialRoadmark_->setAmbient(osg::Material::FRONT_AND_BACK, color_array->at(0));
    //	materialRoadmark_->setShininess(osg::Material::FRONT_AND_BACK, 0);

    // Finally create and add geometry
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setUseDisplayList(true);
    geom->setVertexArray(vertices.get());
    geom->addPrimitiveSet(indices.get());
    geom->setColorArray(color_array.get());
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    // Use PolygonOffset feature to avoid z-fighting with road surface
    geom->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(-POLYGON_OFFSET_ROADMARKS, -SIGN(POLYGON_OFFSET_ROADMARKS)));
    osgUtil::SmoothingVisitor::smooth(*geom, 0.0);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);
    geode->getOrCreateStateSet()->setAttributeAndModes(materialRoadmark_.get());
    rm_group_->addChild(geode);
}

int RoadGeom::AddRoadMarks(roadmanager::Lane* lane, osg::Group* parent)
{
    (void)parent;
    for (size_t i = 0; i < static_cast<unsigned int>(lane->GetNumberOfRoadMarks()); i++)
    {
        roadmanager::LaneRoadMark* lane_roadmark = lane->GetLaneRoadMarkByIdx(static_cast<int>(i));

        if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::NONE_TYPE)
        {
            continue;
        }

        for (int m = 0; m < lane_roadmark->GetNumberOfRoadMarkTypes(); m++)
        {
            roadmanager::LaneRoadMarkType* lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(m);

            for (int n = 0; n < lane_roadmarktype->GetNumberOfRoadMarkTypeLines(); n++)
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
                            shape->setColor(viewer::ODR2OSGColor(lane_roadmark->GetColor()));
                            dot = new osg::Geode;
                            dot->addDrawable(shape);
                        }

                        roadmanager::PointStruct osi_point0 = curr_osi_rm->GetPoint(static_cast<int>(q));

                        osg::ref_ptr<osg::PositionAttitudeTransform> tx = new osg::PositionAttitudeTransform;
                        tx->setPosition(osg::Vec3(static_cast<float>(osi_point0.x - origin_[0]),
                                                  static_cast<float>(osi_point0.y - origin_[1]),
                                                  static_cast<float>(osi_point0.z)));
                        tx->addChild(dot);
                        rm_group_->addChild(tx);
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

                    int startpoint = 0;

                    for (size_t q = 0; q < osi_points.size(); q++)
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
                            OffsetVec2D(osi_points[q].x - origin_[0],
                                        osi_points[q].y - origin_[1],
                                        osi_points[q + 1].x - origin_[0],
                                        osi_points[q + 1].y - origin_[1],
                                        -lane_roadmarktypeline->GetWidth() / 2,
                                        l1p0l[0],
                                        l1p0l[1],
                                        l1p1l[0],
                                        l1p1l[1]);
                            OffsetVec2D(osi_points[q].x - origin_[0],
                                        osi_points[q].y - origin_[1],
                                        osi_points[q + 1].x - origin_[0],
                                        osi_points[q + 1].y - origin_[1],
                                        lane_roadmarktypeline->GetWidth() / 2,
                                        l1p0r[0],
                                        l1p0r[1],
                                        l1p1r[0],
                                        l1p1r[1]);
                        }
                        else if (!osi_points[q].endpoint)
                        {
                            printf("Unexpected last point without endpoint q %zd\n", q);
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
                            AddRoadMarkGeom(vertices, indices, lane_roadmarktypeline->GetColor());
                            startpoint = q + 1;
                        }
                    }
                }
            }
        }
    }

    return 0;
}

RoadGeom::RoadGeom(roadmanager::OpenDrive* odr, osg::Vec3d origin)
{
    root_     = new osg::Group;
    rm_group_ = new osg::Group;
    origin_   = origin;
    root_->addChild(rm_group_);

    osg::ref_ptr<osg::Texture2D> tex_asphalt;
    osg::ref_ptr<osg::Texture2D> tex_grass;
    if (!SE_Env::Inst().GetOptions().GetOptionSet("generate_without_textures"))
    {
        tex_asphalt = ReadTexture("asphalt.jpg");
        tex_grass   = ReadTexture("grass.jpg");
    }

    osg::ref_ptr<osg::Vec4Array> color_asphalt      = new osg::Vec4Array;
    osg::ref_ptr<osg::Vec4Array> color_concrete     = new osg::Vec4Array;
    osg::ref_ptr<osg::Vec4Array> color_border_inner = new osg::Vec4Array;
    osg::ref_ptr<osg::Vec4Array> color_grass        = new osg::Vec4Array;

    if (tex_asphalt)
    {
        color_asphalt->push_back(osg::Vec4(1.f, 1.f, 1.f, 1.0f));
    }
    else
    {
        color_asphalt->push_back(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
    }

    if (tex_grass)
    {
        color_grass->push_back(osg::Vec4(1.f, 1.f, 1.f, 1.0f));
    }
    else
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
                            if (osiPoints[l].s > section_current_s + SMALL_NUMBER)
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
                            lane                                            = lsec->GetLaneByIdx(static_cast<int>(l));
                            std::vector<roadmanager::PointStruct> osiPoints = lane->GetOSIPoints()->GetPoints();

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
            int vertex_index_left_local      = 0;
            int vertex_index_right_local     = 0;
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

                vertex_index_left_local      = vertex_index_left_local_next;
                vertex_index_right_local     = vertex_idx_all;
                vertex_index_left_local_next = vertex_index_right_local;

                for (size_t m = 0; m < geom_strips_list[k].size(); m++)  // loop over lane patches with constant friction
                {
                    double                  friction    = geom_strips_list[k][m].friction;
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
                        GeomPoint& gp = geom_points[gpi + l];
                        if (m == 0 || l > 0)
                        {
                            (*verticesAll)[static_cast<unsigned int>(vertex_idx_all)].set(static_cast<float>(gp.x - origin_[0]),
                                                                                          static_cast<float>(gp.y - origin_[1]),
                                                                                          static_cast<float>(gp.z));
                            double texscale = TEXTURE_SCALE;
                            (*texcoordsAll)[static_cast<unsigned int>(vertex_idx_all)].set(
                                osg::Vec2(static_cast<float>(texscale * (gp.x - origin_[0])), static_cast<float>(texscale * (gp.y - origin_[1]))));
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

                        osg::ref_ptr<osg::Texture2D> tex = nullptr;

                        if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_ANY_ROAD))
                        {
                            osg::ref_ptr<osg::Material> materialAsphalt_ = new osg::Material;
                            osg::Vec4                   new_color        = color_asphalt->at(0);

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

                            materialAsphalt_->setDiffuse(osg::Material::FRONT_AND_BACK, new_color);
                            materialAsphalt_->setAmbient(osg::Material::FRONT_AND_BACK, new_color);

                            if (tex_asphalt)
                            {
                                tex = tex_asphalt.get();
                            }
                            geom->getOrCreateStateSet()->setAttributeAndModes(materialAsphalt_.get());
                        }
                        else if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BIKING) ||
                                 laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_SIDEWALK))
                        {
                            osg::ref_ptr<osg::Material> materialConcrete_ = new osg::Material;
                            materialConcrete_->setDiffuse(osg::Material::FRONT_AND_BACK, color_concrete->at(0));
                            materialConcrete_->setAmbient(osg::Material::FRONT_AND_BACK, color_concrete->at(0));

                            geom->getOrCreateStateSet()->setAttributeAndModes(materialConcrete_.get());

                            // Use PolygonOffset feature to avoid z-fighting with road surface
                            geom->getOrCreateStateSet()->setAttributeAndModes(
                                new osg::PolygonOffset(-POLYGON_OFFSET_SIDEWALK, -SIGN(POLYGON_OFFSET_SIDEWALK)));
                        }
                        else if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BORDER) && k != 1 &&
                                 k != static_cast<unsigned int>(lsec->GetNumberOfLanes()) - 1)
                        {
                            osg::ref_ptr<osg::Material> materialBorderInner_ = new osg::Material;
                            materialBorderInner_->setDiffuse(osg::Material::FRONT_AND_BACK, color_border_inner->at(0));
                            materialBorderInner_->setAmbient(osg::Material::FRONT_AND_BACK, color_border_inner->at(0));

                            geom->getOrCreateStateSet()->setAttributeAndModes(materialBorderInner_.get());

                            // Use PolygonOffset feature to avoid z-fighting with road surface
                            geom->getOrCreateStateSet()->setAttributeAndModes(
                                new osg::PolygonOffset(-POLYGON_OFFSET_BORDER, -SIGN(POLYGON_OFFSET_BORDER)));
                        }
                        else
                        {
                            osg::ref_ptr<osg::Material> materialGrass_ = new osg::Material;

                            materialGrass_->setDiffuse(osg::Material::FRONT_AND_BACK, color_grass->at(0));
                            materialGrass_->setAmbient(osg::Material::FRONT_AND_BACK, color_grass->at(0));

                            if (tex_grass)
                            {
                                tex = tex_grass.get();
                            }
                            geom->getOrCreateStateSet()->setAttributeAndModes(materialGrass_.get());

                            // Use PolygonOffset feature to avoid z-fighting with road surface
                            geom->getOrCreateStateSet()->setAttributeAndModes(
                                new osg::PolygonOffset(-POLYGON_OFFSET_GRASS, -SIGN(POLYGON_OFFSET_GRASS)));
                        }
                        geom->setColorBinding(osg::Geometry::BIND_OVERALL);

                        if (tex != nullptr)
                        {
                            geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get());
                        }

                        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                        geode->addDrawable(geom.get());

                        // osgUtil::Optimizer optimizer;
                        // optimizer.optimize(geode);

                        root_->addChild(geode);
                    }
                }
                AddRoadMarks(lane, root_);
            }
        }
    }
}