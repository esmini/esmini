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

#define GEOM_TOLERANCE (0.2 - SMALL_NUMBER) // Minimum distance between two vertices along road s-axis
#define TEXTURE_SCALE 0.5   // Scale factor for asphalt and grass textures 1.0 means whole texture fits in 1 x 1 m square
#define MAX_GEOM_ERROR 0.1  // maximum distance from the 3D geometry to the OSI lines
#define MAX_GEOM_LENGTH 50  // maximum length of a road geometry mesh segment


osg::ref_ptr<osg::Texture2D> RoadGeom::ReadTexture(std::string filename)
{
	osg::ref_ptr<osg::Texture2D> tex = 0;
	osg::ref_ptr<osg::Image> img = 0;

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

void RoadGeom::AddRoadMarkGeom(osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::DrawElementsUInt> indices,
	roadmanager::RoadMarkColor color)
{
	osg::ref_ptr<osg::Material> materialRoadmark_ = new osg::Material;
	osg::ref_ptr<osg::Vec4Array> color_array = new osg::Vec4Array;
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
	geom->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(-2.0, -1.0));
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
					inner_index = static_cast<int>(std::max_element(sort_solidbroken_brokensolid.begin(), sort_solidbroken_brokensolid.end()) - sort_solidbroken_brokensolid.begin());
				}
				else
				{
					inner_index = static_cast<int>(std::min_element(sort_solidbroken_brokensolid.begin(), sort_solidbroken_brokensolid.end()) - sort_solidbroken_brokensolid.begin());
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

				if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BOTTS_DOTS)
				{
					for (unsigned int q = 0; q < curr_osi_rm->GetPoints().size(); q++)
					{
						const double botts_dot_size = 0.15;
						static osg::ref_ptr<osg::Geode> dot = 0;

						if (dot == 0)
						{
							osg::ref_ptr<osg::TessellationHints> th = new osg::TessellationHints();
							th->setDetailRatio(0.3f);
							osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(
								new osg::Cylinder(osg::Vec3(0.0, 0.0, 0.0), static_cast<float>(botts_dot_size), 0.3f * static_cast<float>(botts_dot_size)), th
							);
							shape->setColor(viewer::ODR2OSGColor(lane_roadmark->GetColor()));
							dot = new osg::Geode;
							dot->addDrawable(shape);
						}

						roadmanager::PointStruct osi_point0 = curr_osi_rm->GetPoint(static_cast<int>(q));

						osg::ref_ptr<osg::PositionAttitudeTransform> tx = new osg::PositionAttitudeTransform;
						tx->setPosition(osg::Vec3(static_cast<float>(osi_point0.x), static_cast<float>(osi_point0.y), static_cast<float>(osi_point0.z)));
						tx->addChild(dot);
						rm_group_->addChild(tx);
					}
				}
				else if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN ||
					lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN_BROKEN ||
					broken)
				{
					for (unsigned int q = 0; q < curr_osi_rm->GetPoints().size(); q += 2)
					{
						roadmanager::PointStruct osi_point0 = curr_osi_rm->GetPoint(static_cast<int>(q));
						roadmanager::PointStruct osi_point1 = curr_osi_rm->GetPoint(static_cast<int>(q) + 1);

						osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4);
						osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, 4);

						// Find left points of roadmark
						double x0l, x1l, y0l, y1l;
						OffsetVec2D(osi_point0.x, osi_point0.y, osi_point1.x, osi_point1.y, -lane_roadmarktypeline->GetWidth()/2, x0l, y0l, x1l, y1l);

						// Find right points of roadmark
						double x0r, x1r, y0r, y1r;
						OffsetVec2D(osi_point0.x, osi_point0.y, osi_point1.x, osi_point1.y, lane_roadmarktypeline->GetWidth()/2, x0r, y0r, x1r, y1r);

						// Set vertices and indices
						(*vertices)[0].set(static_cast<float>(x0l), static_cast<float>(y0l), static_cast<float>(osi_point0.z));
						(*vertices)[1].set(static_cast<float>(x0r), static_cast<float>(y0r), static_cast<float>(osi_point0.z));
						(*vertices)[2].set(static_cast<float>(x1l), static_cast<float>(y1l), static_cast<float>(osi_point1.z));
						(*vertices)[3].set(static_cast<float>(x1r), static_cast<float>(y1r), static_cast<float>(osi_point1.z));

						(*indices)[0] = 0;
						(*indices)[1] = 1;
						(*indices)[2] = 2;
						(*indices)[3] = 3;

						// Finally create and add OSG geometries
						AddRoadMarkGeom(vertices, indices, lane_roadmarktypeline->GetColor());
					}
				}
				else if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID ||
				lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID_SOLID ||
				!broken)
				{
					std::vector<roadmanager::PointStruct> osi_points = curr_osi_rm->GetPoints();

					if (osi_points.size() < 2)
					{
						// No line - skip
						continue;
					}

					osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(static_cast<unsigned int>(osi_points.size() * 2));
					osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, static_cast<unsigned int>(osi_points.size()) * 2);

					for (size_t q = 0; q < osi_points.size(); q++)
					{
						// Find offset points of solid roadmark at each OSI point
						double l0p0l[2], l0p0r[2], l0p1l[2], l0p1r[2], l1p0l[2], l1p0r[2], l1p1l[2], l1p1r[2];

						if (q < osi_points.size() - 1)
						{
							OffsetVec2D(osi_points[q].x, osi_points[q].y, osi_points[q + 1].x, osi_points[q + 1].y, -lane_roadmarktypeline->GetWidth() / 2, l1p0l[0], l1p0l[1], l1p1l[0], l1p1l[1]);
							OffsetVec2D(osi_points[q].x, osi_points[q].y, osi_points[q + 1].x, osi_points[q + 1].y, lane_roadmarktypeline->GetWidth() / 2, l1p0r[0], l1p0r[1], l1p1r[0], l1p1r[1]);
						}

						if (q == 0)
						{
							// First point, no adjustment needed
							(*vertices)[q * 2 + 0].set(static_cast<float>(l1p0l[0]), static_cast<float>(l1p0l[1]), static_cast<float>(osi_points[q].z));
							(*vertices)[q * 2 + 1].set(static_cast<float>(l1p0r[0]), static_cast<float>(l1p0r[1]), static_cast<float>(osi_points[q].z));
						}
						else if (q == osi_points.size() - 1)
						{
							// Last point, no adjustment needed
							(*vertices)[q * 2 + 0].set(static_cast<float>(l1p1l[0]), static_cast<float>(l1p1l[1]), static_cast<float>(osi_points[q].z));
							(*vertices)[q * 2 + 1].set(static_cast<float>(l1p1r[0]), static_cast<float>(l1p1r[1]), static_cast<float>(osi_points[q].z));
						}
						else
						{
							// Find intersection of non parallel lines
							double isect[2];

							if (GetIntersectionOfTwoLineSegments(l0p0l[0], l0p0l[1], l0p1l[0], l0p1l[1], l1p0l[0], l1p0l[1], l1p1l[0], l1p1l[1], isect[0], isect[1]) == 0)
							{
								(*vertices)[q * 2 + 0].set(static_cast<float>(isect[0]), static_cast<float>(isect[1]), static_cast<float>(osi_points[q].z));
							}
							else
							{
								// lines parallel, no adjustment needed
								(*vertices)[q * 2 + 0].set(static_cast<float>(l1p0l[0]), static_cast<float>(l1p0l[1]), static_cast<float>(osi_points[q].z));
							}

							if (GetIntersectionOfTwoLineSegments(l0p0r[0], l0p0r[1], l0p1r[0], l0p1r[1], l1p0r[0], l1p0r[1], l1p1r[0], l1p1r[1], isect[0], isect[1]) == 0)
							{
								(*vertices)[q * 2 + 1].set(static_cast<float>(isect[0]), static_cast<float>(isect[1]), static_cast<float>(osi_points[q].z));
							}
							else
							{
								// lines parallel, no adjustment needed
								(*vertices)[q * 2 + 1].set(static_cast<float>(l1p0r[0]), static_cast<float>(l1p0r[1]), static_cast<float>(osi_points[q].z));
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
						(*indices)[q * 2 + 0] = static_cast<unsigned int>(q) * 2 + 0;
						(*indices)[q * 2 + 1] = static_cast<unsigned int>(q) * 2 + 1;
					}

					// Finally create and add OSG geometries
					AddRoadMarkGeom(vertices, indices, lane_roadmarktypeline->GetColor());
				}
			}
		}
	}

	return 0;
}

RoadGeom::RoadGeom(roadmanager::OpenDrive *odr)
{
	root_ = new osg::Group;
	rm_group_ = new osg::Group;

	root_->addChild(rm_group_);

	osg::ref_ptr<osg::Texture2D> tex_asphalt = ReadTexture("asphalt.jpg");
	osg::ref_ptr<osg::Texture2D> tex_grass = ReadTexture("grass.jpg");

	osg::ref_ptr<osg::Vec4Array> color_asphalt = new osg::Vec4Array;
	osg::ref_ptr<osg::Vec4Array> color_concrete = new osg::Vec4Array;
	osg::ref_ptr<osg::Vec4Array> color_border_inner = new osg::Vec4Array;
	osg::ref_ptr<osg::Vec4Array> color_grass = new osg::Vec4Array;

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

	osg::ref_ptr<osg::Material> materialAsphalt_ = new osg::Material;
	osg::ref_ptr<osg::Material> materialGrass_ = new osg::Material;
	osg::ref_ptr<osg::Material> materialConcrete_ = new osg::Material;
	osg::ref_ptr<osg::Material> materialBorderInner_ = new osg::Material;
	materialAsphalt_->setDiffuse(osg::Material::FRONT_AND_BACK, color_asphalt->at(0));
	materialAsphalt_->setAmbient(osg::Material::FRONT_AND_BACK, color_asphalt->at(0));
	materialGrass_->setDiffuse(osg::Material::FRONT_AND_BACK, color_grass->at(0));
	materialGrass_->setAmbient(osg::Material::FRONT_AND_BACK, color_grass->at(0));
	materialConcrete_->setDiffuse(osg::Material::FRONT_AND_BACK, color_concrete->at(0));
	materialConcrete_->setAmbient(osg::Material::FRONT_AND_BACK, color_concrete->at(0));
	materialBorderInner_->setDiffuse(osg::Material::FRONT_AND_BACK, color_border_inner->at(0));
	materialBorderInner_->setAmbient(osg::Material::FRONT_AND_BACK, color_border_inner->at(0));

	for (size_t i = 0; i < static_cast<unsigned int>(odr->GetNumOfRoads()); i++)
	{
		roadmanager::Road* road = odr->GetRoadByIdx(static_cast<int>(i));
		std::vector<double> s_list;

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
				LOG("Missing OSI points of centerlane road %d section %d", road->GetId(), j);
				throw std::runtime_error("Missing OSI points");
			}

			// create a list to keep track of current s-value for each lane
			std::vector<int> s_index(static_cast<unsigned int>(lsec->GetNumberOfLanes()), 0);

			// create a 2d list of positions for vertices, nr_of_lanes x nr_of_s-values
			typedef struct
			{
				double x;
				double y;
				double z;
				double h;
				double slope;
				double s;
			} GeomPoint;
			std::vector<std::vector<GeomPoint>> geom_point_list;
			std::vector<GeomPoint> geom_point;

			roadmanager::Position pos;
			s_list.clear();

			bool done = false;
			for(int counter = 1; !done && counter > 0; counter++)
			{
				double s_min = lsec->GetS() + lsec->GetLength();
				done = true;

				if (counter == 1)
				{
					// First add s = start of lane section, to set start of mesh
					s_list.push_back(lsec->GetS());
					done = false;
				}
				else
				{
					// find next s-value based on accumulated error of each lane
					for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
					{
						lane = lsec->GetLaneByIdx(static_cast<int>(k));
						std::vector<roadmanager::PointStruct> osiPoints = lane->GetOSIPoints()->GetPoints();
						unsigned int l = static_cast<unsigned int>(s_index[k]) + 1;

						// Find next s-value for this lane - go forward until error becomes too large
						for (; l < osiPoints.size(); l++)
						{
							// generate point at this s-value
							pos.SetTrackPos(road->GetId(), osiPoints[l].s, SIGN(lane->GetId())* lsec->GetOuterOffset(osiPoints[l].s, lane->GetId()), true);

							// calculate horizontal error at this s value
							double error_horizontal = DistanceFromPointToLine2DWithAngle(
								pos.GetX(), pos.GetY(),
								geom_point_list.back()[k].x, geom_point_list.back()[k].y, geom_point_list.back()[k].h);

							// calculate vertical error at this s value
							double error_vertical = abs((pos.GetZ() - geom_point_list.back()[k].z) -
								geom_point_list.back()[k].slope * (pos.GetS() - geom_point_list.back()[k].s));

							if (error_horizontal > MAX_GEOM_ERROR || error_vertical > MAX_GEOM_ERROR)
							{
								done = false;
								break;
							}
						}

						if (l < osiPoints.size())
						{
							if (s_index[k] != static_cast<int>(l) - 1)
							{
								// make sure previous s-value before error exceeded threshold is included
								s_index[k] = static_cast<int>(l) - 1;
							}
							else
							{
								// add s-value exceeding the threshold
								s_index[k] = static_cast<int>(l);
							}
						}
						else
						{
							s_index[k] = static_cast<int>(osiPoints.size()) - 1;  // add last OSI point for end of mesh
						}

						if (osiPoints[static_cast<unsigned int>(s_index[k])].s < s_min)
						{
							s_min = osiPoints[static_cast<unsigned int>(s_index[k])].s;  // register pivot s-value
						}
					}

					s_list.push_back(s_min);
				}

				double total_segment_length = s_list.back();
				if (s_list.size() > 1)
				{
					total_segment_length -= s_list.rbegin()[1];
				}
				else
				{
					total_segment_length -= lsec->GetS();
				}

				// limit segment length - split if needed
				int n = static_cast<int>((total_segment_length - SMALL_NUMBER) / MAX_GEOM_LENGTH) + 1;
				double segment_length = total_segment_length / n;
				for (int m = 0; m < n; m++)
				{
					geom_point.clear();
					double s = 0.0;

					if (m == n - 1)
					{
						s = s_list.back();
					}
					else
					{
						s = s_list.rbegin()[1] + (m + 1) * segment_length;
					}

					for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
					{
						lane = lsec->GetLaneByIdx(static_cast<int>(k));
						pos.SetTrackPos(road->GetId(), s, SIGN(lane->GetId())* lsec->GetOuterOffset(s, lane->GetId()), true);
						geom_point.push_back({ pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), pos.GetZRoadPrim(), pos.GetS() });
					}

					geom_point_list.push_back(geom_point);
				}
			}

			// Then create actual vertices and triangle strips for the lane section
			pos.SetAlignMode(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
			int nrOfVerticesTotal = static_cast<int>(geom_point_list.size()) * lsec->GetNumberOfLanes();
			osg::ref_ptr<osg::Vec3Array> verticesAll = new osg::Vec3Array(static_cast<unsigned int>(nrOfVerticesTotal));
			osg::ref_ptr<osg::Vec2Array> texcoordsAll = new osg::Vec2Array;
			int vidxAll = 0;

			// Potential optimization: Swap loops, creating all vertices for same s-value for each step
			for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
			{
				osg::ref_ptr<osg::Vec3Array> verticesLocal;
				osg::ref_ptr<osg::Vec2Array> texcoordsLocal;
				osg::ref_ptr<osg::DrawElementsUInt> indices;
				int vidxLocal = 0;
				lane = lsec->GetLaneByIdx(static_cast<int>(k));

				if (k > 0)
				{
					verticesLocal = new osg::Vec3Array(static_cast<unsigned int>(geom_point_list.size()) * 2);
					indices = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, static_cast<unsigned int>(geom_point_list.size()) * 2);
					texcoordsLocal = new osg::Vec2Array(static_cast<unsigned int>(geom_point_list.size()) * 2);
				}

				for (size_t l = 0; l < geom_point_list.size(); l++)
				{
					GeomPoint& gp = geom_point_list[l][k];
					(*verticesAll)[static_cast<unsigned int>(vidxAll++)].set(static_cast<float>(gp.x), static_cast<float>(gp.y), static_cast<float>(gp.z));
					double texscale = TEXTURE_SCALE;
					texcoordsAll->push_back(osg::Vec2(static_cast<float>(texscale * gp.x), static_cast<float>(texscale * gp.y)));

					// Create indices for the lane strip, referring to the vertex list
					if (k > 0)
					{
						// vertex of left lane border
						(*verticesLocal)[static_cast<unsigned int>(vidxLocal)] = (*verticesAll)[(k - 1) * geom_point_list.size() + l];
						(*texcoordsLocal)[static_cast<unsigned int>(vidxLocal)] = (*texcoordsAll)[(k - 1) * geom_point_list.size() + l];
						(*indices)[static_cast<unsigned int>(vidxLocal)] = static_cast<unsigned int>(vidxLocal);
						vidxLocal++;
						// vertex of right
						(*verticesLocal)[static_cast<unsigned int>(vidxLocal)] = (*verticesAll)[k * geom_point_list.size() + l];
						(*texcoordsLocal)[static_cast<unsigned int>(vidxLocal)] = (*texcoordsAll)[k * geom_point_list.size() + l];
						(*indices)[static_cast<unsigned int>(vidxLocal)] = static_cast<unsigned int>(vidxLocal);
						vidxLocal++;
					}
				}

				if (k > 0)
				{
					// Create geometry for the strip made of this and previous lane
					osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
					geom->setUseDisplayList(true);
					geom->setVertexArray(verticesLocal.get());
					geom->addPrimitiveSet(indices.get());
					geom->setTexCoordArray(0, texcoordsLocal.get());
					osgUtil::SmoothingVisitor::smooth(*geom, 0.5);

					osg::ref_ptr<osg::Texture2D> tex = nullptr;

					roadmanager::Lane* laneForMaterial = lsec->GetLaneByIdx(lane->GetId() < 0 ? static_cast<int>(k) : static_cast<int>(k) - 1);

					if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_ANY_ROAD))
					{
						geom->setColorArray(color_asphalt.get());
						if (tex_asphalt)
						{
							tex = tex_asphalt.get();
						}
						geom->getOrCreateStateSet()->setAttributeAndModes(materialAsphalt_.get());
					}
					else if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BIKING) ||
						laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_SIDEWALK))
					{
						geom->setColorArray(color_concrete.get());
						geom->getOrCreateStateSet()->setAttributeAndModes(materialConcrete_.get());
					}
					else if (laneForMaterial->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BORDER) &&
						k != 1 && k != static_cast<unsigned int>(lsec->GetNumberOfLanes()) - 1)
					{
						geom->setColorArray(color_border_inner.get());
						geom->getOrCreateStateSet()->setAttributeAndModes(materialBorderInner_.get());
					}
					else
					{
						geom->setColorArray(color_grass.get());
						if (tex_grass)
						{
							tex = tex_grass.get();
						}
						geom->getOrCreateStateSet()->setAttributeAndModes(materialGrass_.get());
					}
					geom->setColorBinding(osg::Geometry::BIND_OVERALL);

					if (tex != nullptr)
					{
						geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get());
						osg::ref_ptr<osg::TexEnv> texEnv = new osg::TexEnv;
						texEnv->setMode(osg::TexEnv::MODULATE);
						geom->getOrCreateStateSet()->setTextureAttribute(0, texEnv.get(), osg::StateAttribute::ON);
					}

					osg::ref_ptr<osg::Geode> geode = new osg::Geode;
					geode->addDrawable(geom.get());

					//osgUtil::Optimizer optimizer;
					//optimizer.optimize(geode);

					root_->addChild(geode);
				}
				AddRoadMarks(lane, root_);
			}
		}
	}
}