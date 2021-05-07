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

#include "CommonMini.hpp"

#define GEOM_TOLERANCE 0.2  // Minimum distance between two vertices along road s-axis
#define TEXTURE_SCALE 0.5   // Scale factor for asphalt and grass textures 1.0 means whole texture fits in 1 x 1 m square


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
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], std::string("../models/" + filename)));
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

void RoadGeom::AddRoadMarkGeom(osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::DrawElementsUInt> indices)
{
	osg::ref_ptr<osg::Material> materialRoadmark_ = new osg::Material;
	osg::ref_ptr<osg::Vec4Array> color_roadmark = new osg::Vec4Array;
	color_roadmark->push_back(osg::Vec4(0.95f, 0.95f, 0.9f, 1.0f));

	materialRoadmark_->setDiffuse(osg::Material::FRONT_AND_BACK, color_roadmark->at(0));
	materialRoadmark_->setAmbient(osg::Material::FRONT_AND_BACK, color_roadmark->at(0));

	// Finally create and add geometry
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setUseDisplayList(true);
	geom->setVertexArray(vertices.get());
	geom->addPrimitiveSet(indices.get());
	geom->setColorArray(color_roadmark.get());
	geom->setColorBinding(osg::Geometry::BIND_OVERALL);

	// Use PolygonOffset feature to avoid z-fighting with road surface
	geom->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(-1, -1));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom);
	geode->getOrCreateStateSet()->setAttributeAndModes(materialRoadmark_.get());
	rm_group_->addChild(geode);
}

int RoadGeom::AddRoadMarks(roadmanager::Lane* lane, osg::Group* parent)
{
	for (size_t i = 0; i < lane->GetNumberOfRoadMarks(); i++)
	{
		roadmanager::LaneRoadMark* lane_roadmark = lane->GetLaneRoadMarkByIdx(i);
		for (int m = 0; m < lane_roadmark->GetNumberOfRoadMarkTypes(); m++)
		{
			roadmanager::LaneRoadMarkType* lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(m);
			for (int n = 0; n < lane_roadmarktype->GetNumberOfRoadMarkTypeLines(); n++)
			{
				roadmanager::LaneRoadMarkTypeLine* lane_roadmarktypeline = lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(n);
				roadmanager::OSIPoints* curr_osi_rm = lane_roadmarktypeline->GetOSIPoints();

				if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BROKEN)
				{
					for (int q = 0; q < curr_osi_rm->GetPoints().size(); q += 2)
					{
						roadmanager::PointStruct osi_point0 = curr_osi_rm->GetPoint(q);
						roadmanager::PointStruct osi_point1 = curr_osi_rm->GetPoint(q + 1);

						osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4);
						osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, 4);

						// Find left points of roadmark
						double x0l, x1l, y0l, y1l;
						OffsetVec2D(osi_point0.x, osi_point0.y, osi_point1.x, osi_point1.y, -lane_roadmarktypeline->GetWidth()/2, x0l, y0l, x1l, y1l);

						// Find right points of roadmark
						double x0r, x1r, y0r, y1r;
						OffsetVec2D(osi_point0.x, osi_point0.y, osi_point1.x, osi_point1.y, lane_roadmarktypeline->GetWidth()/2, x0r, y0r, x1r, y1r);

						// Set vertices and indices
						(*vertices)[0].set(x0l, y0l, osi_point0.z);
						(*vertices)[1].set(x0r, y0r, osi_point0.z);
						(*vertices)[2].set(x1l, y1l, osi_point1.z);
						(*vertices)[3].set(x1r, y1r, osi_point1.z);

						(*indices)[0] = 0;
						(*indices)[1] = 1;
						(*indices)[2] = 2;
						(*indices)[3] = 3;

						// Finally create and add OSG geometries
						AddRoadMarkGeom(vertices, indices);
					}
				}
				else if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::SOLID)
				{
					std::vector<roadmanager::PointStruct> osi_points = curr_osi_rm->GetPoints();

					if (osi_points.size() < 2)
					{
						// No line - skip
						continue;
					}

					osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(osi_points.size() * 2);
					osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, osi_points.size() * 2);

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
							(*vertices)[q * 2 + 0].set(l1p0l[0], l1p0l[1], osi_points[q].z);
							(*vertices)[q * 2 + 1].set(l1p0r[0], l1p0r[1], osi_points[q].z);
						}
						else if (q == osi_points.size() - 1)
						{
							// Last point, no adjustment needed
							(*vertices)[q * 2 + 0].set(l1p1l[0], l1p1l[1], osi_points[q].z);
							(*vertices)[q * 2 + 1].set(l1p1r[0], l1p1r[1], osi_points[q].z);
						}
						else
						{
							// Find intersection of non parallel lines
							double isect[2];

							if (GetIntersectionOfTwoLineSegments(l0p0l[0], l0p0l[1], l0p1l[0], l0p1l[1], l1p0l[0], l1p0l[1], l1p1l[0], l1p1l[1], isect[0], isect[1]) == 0)
							{ 
								(*vertices)[q * 2 + 0].set(isect[0], isect[1], osi_points[q].z);
							}
							else
							{
								// lines parallel, no adjustment needed
								(*vertices)[q * 2 + 0].set(l1p0l[0], l1p0l[1], osi_points[q].z);
							}
							
							if (GetIntersectionOfTwoLineSegments(l0p0r[0], l0p0r[1], l0p1r[0], l0p1r[1], l1p0r[0], l1p0r[1], l1p1r[0], l1p1r[1], isect[0], isect[1]) == 0)
							{
								(*vertices)[q * 2 + 1].set(isect[0], isect[1], osi_points[q].z);
							}
							else
							{
								// lines parallel, no adjustment needed
								(*vertices)[q * 2 + 1].set(l1p0r[0], l1p0r[1], osi_points[q].z);
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
						(*indices)[q * 2 + 0] = q * 2 + 0;
						(*indices)[q * 2 + 1] = q * 2 + 1;
					}
					
					// Finally create and add OSG geometries
					AddRoadMarkGeom(vertices, indices);
				}
			}
		}
	}

	return 0;
}

RoadGeom::RoadGeom(roadmanager::OpenDrive *odr)
{
	std::vector<double> s_list;
	root_ = new osg::Group;
	rm_group_ = new osg::Group;
	root_->addChild(rm_group_);

	osg::ref_ptr<osg::Texture2D> tex_asphalt = ReadTexture("asphalt.jpg");
	osg::ref_ptr<osg::Texture2D> tex_grass = ReadTexture("grass.jpg");

	osg::ref_ptr<osg::Vec4Array> color_asphalt = new osg::Vec4Array;
	osg::ref_ptr<osg::Vec4Array> color_grass = new osg::Vec4Array;

	color_asphalt->push_back(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
	color_grass->push_back(osg::Vec4(0.20f, 0.55f, 0.35f, 1.0f));

	osg::ref_ptr<osg::Material> materialAsphalt_ = new osg::Material;
	osg::ref_ptr<osg::Material> materialGrass_ = new osg::Material;
	materialAsphalt_->setDiffuse(osg::Material::FRONT_AND_BACK, color_asphalt->at(0));
	materialAsphalt_->setAmbient(osg::Material::FRONT_AND_BACK, color_asphalt->at(0));
	materialGrass_->setDiffuse(osg::Material::FRONT_AND_BACK, color_grass->at(0));
	materialGrass_->setAmbient(osg::Material::FRONT_AND_BACK, color_grass->at(0));


	for (size_t i = 0; i < odr->GetNumOfRoads(); i++)
	{
		roadmanager::Road* road = odr->GetRoadByIdx(i);

		for (size_t j = 0; j < road->GetNumberOfLaneSections(); j++)
		{
			roadmanager::LaneSection* lsec = road->GetLaneSectionByIdx(j);
			if (lsec->GetNumberOfLanes() < 2)
			{
				// need at least reference lane plus another lane to form a road geometry
				continue;
			}

			// First find s-values of OSI points of the center lane
			roadmanager::Lane* lane = lsec->GetLaneById(0);
			if (lane->GetOSIPoints() == 0)
			{
				LOG("Missing OSI points of centerlane road %d section %d", road->GetId(), j);
				throw std::runtime_error("Missing OSI points");
			}

			std::vector<roadmanager::PointStruct> osiPoints = lane->GetOSIPoints()->GetPoints();
			s_list.clear();
			for (size_t l = 0; l < osiPoints.size(); l++)
			{
				// Fetch s-value of all osi points
				s_list.push_back(osiPoints[l].s);
			}
			
			// If last point on intermediate lanesection, 
			// add a point sligthly behind in case number of lanes changes
			if (j < road->GetNumberOfLaneSections() - 1)
			{
				s_list.insert(s_list.begin() + s_list.size() - 1, s_list.back() - SMALL_NUMBER);
			}

			// Then fill in from other lanes
			for (size_t k = 0; k < lsec->GetNumberOfLanes(); k++)
			{
				lane = lsec->GetLaneByIdx(k);
				if (lane->GetId() == 0)
				{
					continue;  // Center lane already done
				}

				if (lane->GetOSIPoints() == 0)
				{
					LOG("Missing OSI points %d, %d, %d", i, j, k);
					throw std::runtime_error("Missing OSI points");
				}

				int s_index = 0;
				osiPoints = lane->GetOSIPoints()->GetPoints();

				for (size_t l=0; l<osiPoints.size(); l++)
				{
					// Locate position in list of s-values
					while (s_index < s_list.size() - 1 && osiPoints[l].s > s_list[s_index+1])
					{
						if (s_index < s_list.size() - 1)
						{
							s_index++;
						}
						else
						{
							break;
						}
					}

					if (s_index == s_list.size() - 1)
					{
						// At end of s-list, append
						if (osiPoints[l].s - s_list[s_index] > GEOM_TOLERANCE)
						{
							s_list.push_back(osiPoints[l].s);
						}
					}
					else if (s_index < s_list.size() - 1)
					{
						// Insert 
						if (osiPoints[l].s - s_list[s_index] > GEOM_TOLERANCE &&
							s_list[s_index+1] - osiPoints[l].s > GEOM_TOLERANCE)
						{
							s_list.insert(s_list.begin() + s_index + 1, osiPoints[l].s);
						}
					}
					else
					{
						LOG("Unexpected s_list index: %d (of %d)", s_index, s_list.size());
						throw std::runtime_error("Unexpected s_list index");
					}
				}
			}

			// Then create actual vertices and triangle strips for the lane section
			roadmanager::Position pos;
			pos.SetAlignMode(roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
			int nrOfVertices = s_list.size() * lsec->GetNumberOfLanes();
			osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(nrOfVertices);
			osg::ref_ptr<osg::DrawElementsUInt> indices;
			int vidx = 0;
			int iidx = 0;

			osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array;

			// Potential optimization: Swap loops, creating all vertices for same s-value for each step
			for (size_t k = 0; k < lsec->GetNumberOfLanes(); k++)
			{
				lane = lsec->GetLaneByIdx(k);

				if (k > 0)
				{
					indices = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, s_list.size() * 2);
					iidx = 0;
				}

				for (size_t l = 0; l < s_list.size(); l++)
				{
					pos.SetTrackPos(road->GetId(), s_list[l], SIGN(lane->GetId()) * lsec->GetOuterOffset(s_list[l], lane->GetId()), true);
					(*vertices)[vidx++].set(pos.GetX(), pos.GetY(), pos.GetZ());
					double texscale = TEXTURE_SCALE;
					texcoords->push_back(osg::Vec2(texscale * pos.GetX(), texscale* pos.GetY()));

					// Create indices for the lane strip, referring to the vertex list
					if (k > 0)
					{
						(*indices)[iidx++] = s_list.size() * (k-1) + l;  // vertex of left lane border
						(*indices)[iidx++] = s_list.size() * k + l;  // vertex of right lane border
					}
				}

				if (k > 0)
				{
					// Create geometry for the strip made of this and previous lane
					osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
					geom->setUseDisplayList(true);
					geom->setVertexArray(vertices.get());
					geom->addPrimitiveSet(indices.get());
					geom->setTexCoordArray(0, texcoords.get());
					osg::ref_ptr<osg::Texture2D> tex = 0;

					if (lsec->GetLaneByIdx(lane->GetId() < 0 ? k : k-1)->IsType(roadmanager::Lane::LaneType::LANE_TYPE_ANY_ROAD))
					{
						geom->setColorArray(color_asphalt.get());
						if (tex_asphalt)
						{
							tex = tex_asphalt.get();
						}
						geom->getOrCreateStateSet()->setAttributeAndModes(materialAsphalt_.get());
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
					if (tex)
					{
						geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get());
						osg::ref_ptr<osg::TexEnv> texEnv = new osg::TexEnv;
						texEnv->setMode(osg::TexEnv::REPLACE);
						geom->getOrCreateStateSet()->setTextureAttribute(0, texEnv.get(), osg::StateAttribute::ON);
					}

					osg::ref_ptr<osg::Geode> geode = new osg::Geode;
					geode->addDrawable(geom.release());
					root_->addChild(geode);
				}
				AddRoadMarks(lane, root_);
			}
		}
	}
}