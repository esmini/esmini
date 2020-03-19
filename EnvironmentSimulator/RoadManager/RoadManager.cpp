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

 /*
  * This module provides an limited interface to OpenDRIVE files.
  * It supports all geometry types (as of ODR 1.4), junctions and some properties such as lane offset
  * but lacks many features as road markings, signals, road signs, speed and road banking
  *
  * It converts between world (cartesian) and road coordinates (both Track and Lane)
  *
  * When used standalone (outside ScenarioEngine) the road manager is initialized via the Position class like this:
  *   roadmanager::Position::LoadOpenDrive("example.xodr");
  *
  * Simplest use case is to put a vehicle on the road and simply move it forward along the road, e.g:
  *
  *   car->pos = new roadmanager::Position(3, -2, 10, 0);
  *   while(true)
  *   {
  *	    car->pos->MoveAlongS(0.1);
  *   }
  *
  * The first line will create a Position object initialized at road with ID = 3, in lane = -2 and at lane offset = 0
  * Then the position is updated along that road and lane, moving 10 cm at a time.
  *
  * A bit more realistic example: 
  *
  *   car->pos = new roadmanager::Position(odrManager->GetRoadByIdx(0)->GetId(), -2, 10, 0);
  *   while(true)
  *   {
  *	    car->pos->MoveAlongS(speed * dt);
  *   }
  * 
  * Here we refer to the ID of the first road in the network. And instead of static delta movement, the distance 
  * is a function of speed and delta time since last update.
  * 
  */

#include <iostream>
#include <cstring>
#include <random>
#include <time.h>
#include <limits>


#include "RoadManager.hpp"
#include "odrSpiral.h"
#include "pugixml.hpp"
#include "CommonMini.hpp"

static std::mt19937 mt_rand;

using namespace std;
using namespace roadmanager;

#define CURV_ZERO 0.00001
#define MAX(x, y) (y > x ? y : x)
#define MIN(x, y) (y < x ? y : x)
#define CLAMP(x, a, b) (MIN(MAX(x, a), b))
#define MAX_TRACK_DIST 10



double Polynomial::Evaluate(double s)
{
	double p = s / (s_max_);

	return (a_ + p * b_ + p * p*c_ + p * p*p*d_);
}

double Polynomial::EvaluatePrim(double s)
{
	double p = s / (s_max_);

	return (b_ + 2 * p*c_ + 3 * p*p*d_);
}

double Polynomial::EvaluatePrimPrim(double s)
{
	double p = s / (s_max_);

	return (2 * c_ + 6 * p*d_);
}

void Polynomial::Set(double a, double b, double c, double d, double s_max)
{
	a_ = a;
	b_ = b;
	c_ = c;
	d_ = d;
	s_max_ = s_max;
}

void Geometry::Print()
{
	LOG("Geometry virtual Print\n");
}

void Geometry::EvaluateDS(double ds, double *x, double *y, double *h)
{
	(void)ds; (void)x; (void)y; (void)h;
	LOG("Geometry virtual Evaluate\n");
}

void Line::Print()
{
	LOG("Line x: %.2f, y: %.2f, h: %.2f length: %.2f\n", GetX(), GetY(), GetHdg(), GetLength());
}

void Line::EvaluateDS(double ds, double *x, double *y, double *h)
{
	*h = GetHdg();
	*x = GetX() + ds * cos(*h);
	*y = GetY() + ds * sin(*h);
}

void Arc::Print()
{
	LOG("Arc x: %.2f, y: %.2f, h: %.2f curvature: %.2f length: %.2f\n", GetX(), GetY(), GetHdg(), curvature_, GetLength());
}

void Arc::EvaluateDS(double ds, double *x, double *y, double *h)
{
	double x_local = 0;
	double y_local = 0;

	// arc_length = angle * radius -> angle = arc_length / radius = arc_length * curvature
	double angle = ds * curvature_;

	// Now calculate x, y in a local unit circle coordinate system
	if (curvature_ < 0)
	{
		// starting from 90 degrees going clockwise
		x_local = cos(angle + M_PI / 2.0);
		y_local = sin(angle + M_PI / 2.0) - 1;  // -1 transform to y = 0
	}
	else
	{
		// starting from -90 degrees going counter clockwise
		x_local = cos(angle + 3.0 * M_PI / 2.0);
		y_local = sin(angle + 3.0 * M_PI / 2.0) + 1;  // +1 transform to y = 0
	}

	// Rotate according to heading and scale according to radius
	*x = GetX() + GetRadius() * (x_local * cos(GetHdg()) - y_local * sin(GetHdg()));
	*y = GetY() + GetRadius() * (x_local * sin(GetHdg()) + y_local * cos(GetHdg()));
	*h = GetHdg() + angle;
}

void Spiral::Print()
{
	LOG("Spiral x: %.2f, y: %.2f, h: %.2f start curvature: %.4f end curvature: %.4f length: %.2f\n",
		GetX(), GetY(), GetHdg(), GetCurvStart(), GetCurvEnd(), GetLength());
}

void Spiral::EvaluateDS(double ds, double *x, double *y, double *h)
{
	double xTmp, yTmp, t, curv_a, curv_b, h_start;

	curv_a = GetCurvStart();
	curv_b = GetCurvEnd();
	h_start = GetHdg();

	if (abs(curv_b) > abs(curv_a))
	{
		odrSpiral(ds + GetS0(), GetCDot(), &xTmp, &yTmp, &t);
		*h = t;
	}
	else  // backwards, starting from sharper curve - ending with lower curvature
	{
		double x0, y0, t0, x1, y1, t1;

		odrSpiral(GetS0() + GetLength(), GetCDot(), &x0, &y0, &t0);
		odrSpiral(GetS0() + GetLength() - ds, GetCDot(), &x1, &y1, &t1);

		xTmp = x0 - x1;
		yTmp = y0 - y1;

		// rotate point according to heading, and translate to start position
		h_start -= t0;
		*h = t1 - t0;
	}

	*h += GetHdg() - GetH0();

	double x1, x2, y1, y2;

	// transform spline segment to origo and start angle = 0
	x1 = xTmp - GetX0();
	y1 = yTmp - GetY0();
	x2 = x1 * cos(-GetH0()) - y1 * sin(-GetH0());
	y2 = x1 * sin(-GetH0()) + y1 * cos(-GetH0());

	// Then transform according to segment start position and heading
	*x = GetX() + x2 * cos(h_start) - y2 * sin(h_start);
	*y = GetY() + x2 * sin(h_start) + y2 * cos(h_start);
}

double Spiral::EvaluateCurvatureDS(double ds)
{
	return (curv_start_ + (ds / GetLength())* (curv_end_ - curv_start_));
}

void Poly3::Print()
{
	LOG("Poly3 x: %.2f, y: %.2f, h: %.2f length: %.2f a: %.2f b: %.2f c: %.2f d: %.2f\n",
		GetX(), GetY(), GetHdg(), GetLength(), poly3_.GetA(), poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

void Poly3::EvaluateDS(double ds, double *x, double *y, double *h)
{
	double p = (ds / GetLength()) * GetUMax();

	double u_local = p;
	double v_local = poly3_.Evaluate(p);

	*x = GetX() + u_local * cos(GetHdg()) - v_local * sin(GetHdg());
	*y = GetY() + u_local * sin(GetHdg()) + v_local * cos(GetHdg());
	*h = GetHdg() + poly3_.EvaluatePrim(p);
}

double Poly3::EvaluateCurvatureDS(double ds)
{
	return poly3_.EvaluatePrimPrim(ds);
}

void ParamPoly3::Print()
{
	LOG("ParamPoly3 x: %.2f, y: %.2f, h: %.2f length: %.2f U: %.8f, %.8f, %.8f, %.8f V: %.8f, %.8f, %.8f, %.8f\n",
		GetX(), GetY(), GetHdg(), GetLength(), 
		poly3U_.GetA(), poly3U_.GetB(), poly3U_.GetC(), poly3U_.GetD(),
		poly3V_.GetA(), poly3V_.GetB(), poly3V_.GetC(), poly3V_.GetD()
	);
}

void ParamPoly3::EvaluateDS(double ds, double *x, double *y, double *h)
{
	double p = ds;

	if (GetPRange() == ParamPoly3::P_RANGE_NORMALIZED)
	{
		p /= GetLength();
	}

	double u_local = poly3U_.Evaluate(p);
	double v_local = poly3V_.Evaluate(p);

	*x = GetX() + u_local * cos(GetHdg()) - v_local * sin(GetHdg());
	*y = GetY() + u_local * sin(GetHdg()) + v_local * cos(GetHdg());
	*h = GetHdg() + poly3V_.EvaluatePrim(p) / poly3U_.EvaluatePrim(p);
}

double ParamPoly3::EvaluateCurvatureDS(double ds)
{
	return poly3V_.EvaluatePrimPrim(ds) / poly3U_.EvaluatePrim(ds);;
}

void Elevation::Print()
{
	LOG("Elevation: s: %.2f A: %.4f B: %.4f C: %.4f D: %.4f\n",
		GetS(), poly3_.GetA(), poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

void LaneLink::Print()
{
	LOG("LaneLink type: %d id: %d\n", type_, id_);
}

LaneWidth *Lane::GetWidthByS(double s)
{
	if (lane_width_.size() == 0)
	{
		return 0;  // No lanewidth defined
	}
	for (int i=0; i+1<(int)lane_width_.size(); i++)
	{
		if (s < lane_width_[i + 1]->GetSOffset())
		{
			return lane_width_[i];
		}
	}
	return lane_width_.back();
}

LaneLink *Lane::GetLink(LinkType type)
{
	for (int i=0; i<(int)link_.size(); i++)
	{
		LaneLink *l = link_[i];
		if (l->GetType() == type)
		{
			return l;
		}
	}
	return 0; // No link of requested type exists
}

void LaneWidth::Print()
{
	LOG("LaneWidth: sOffset: %.2f, a: %.2f, b: %.2f, c: %.2f, d: %.2f\n",
		s_offset_, poly3_.GetA(), poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

void LaneOffset::Print()
{
	LOG("LaneOffset s %.2f a %.4f b %.2f c %.2f d %.2f s_max %.2f length %.2f\n",
		s_, polynomial_.GetA(), polynomial_.GetB(), polynomial_.GetC(), polynomial_.GetD(), polynomial_.GetSMax(), length_);
}

double LaneOffset::GetLaneOffset(double s)
{
	return (polynomial_.Evaluate(s - s_));
}

double LaneOffset::GetLaneOffsetPrim(double s)
{
	return (polynomial_.EvaluatePrim(s - s_));
}

void Lane::Print()
{
	LOG("Lane: %d, type: %d, level: %d\n", id_, type_, level_);
	
	for (size_t i = 0; i < link_.size(); i++)
	{
		link_[i]->Print();
	}

	for (size_t i=0; i<lane_width_.size(); i++)
	{
		lane_width_[i]->Print();
	}
}

int Lane::IsDriving()
{
	if (GetId() == 0)
	{
		return 0;  // Ref lane no width -> no driving
	}

	switch (type_)
	{
	case Lane::LANE_TYPE_DRIVING:
	case Lane::LANE_TYPE_ENTRY:
	case Lane::LANE_TYPE_EXIT:
	case Lane::LANE_TYPE_OFF_RAMP:
	case Lane::LANE_TYPE_ON_RAMP:
	case Lane::LANE_TYPE_PARKING:
		return 1;
		break;
	default:
		// Avoid code analysis warning
	break;
	}

	return 0;
}

LaneSection* Road::GetLaneSectionByIdx(int idx)
{
	if (idx >= 0 && idx < lane_section_.size())
	{
		return lane_section_[idx];
	}
	else
	{
		return 0;
	}
}

int Road::GetLaneSectionIdxByS(double s, int start_at)
{
	if (start_at < 0 || start_at > lane_section_.size() - 1)
	{
		return -1;
	}

	LaneSection *lane_section = lane_section_[start_at];
	size_t i = start_at;

	if (s < lane_section->GetS() && start_at > 0)  
	{
		// Look backwards
		for (i = start_at - 1; i > 0; i--)  // No need to check the first one
		{
			lane_section = GetLaneSectionByIdx((int)i);
			if (s > lane_section->GetS())
			{
				break;
			}
		}
	}
	else  
	{
		// look forward
		for (i = start_at; i < GetNumberOfLaneSections()-1; i++) // No need to check the last one 
		{
			lane_section = GetLaneSectionByIdx((int)i);
			if (s < lane_section->GetS() + lane_section->GetLength())
			{
				break;
			}
		}
	}

	return (int)i;
}

LaneInfo Road::GetLaneInfoByS(double s, int start_lane_section_idx, int start_lane_id)
{
	LaneInfo lane_info;
	
	lane_info.lane_section_idx_ = start_lane_section_idx;
	lane_info.lane_id_ = start_lane_id;

	if (lane_info.lane_section_idx_ >= (int)lane_section_.size())
	{
		LOG("Error idx %d > n_lane_sections %d\n", lane_info.lane_section_idx_, (int)lane_section_.size());
	}
	else
	{
		LaneSection *lane_section = lane_section_[lane_info.lane_section_idx_];

		// check if we passed current section
		if (s > lane_section->GetS() + lane_section->GetLength() || s < lane_section->GetS())
		{
			if (s > lane_section->GetS() + lane_section->GetLength())
			{
				while (s > lane_section->GetS() + lane_section->GetLength() && lane_info.lane_section_idx_ + 1 < GetNumberOfLaneSections())
				{
					// Find out connecting lane, then move to next lane section
					lane_info.lane_id_ = lane_section->GetConnectingLaneId(lane_info.lane_id_, SUCCESSOR);
					lane_section = GetLaneSectionByIdx(++lane_info.lane_section_idx_);
				}
			}
			else if (s < lane_section->GetS())
			{
				while (s < lane_section->GetS() && lane_info.lane_section_idx_ > 0)
				{
					// Move to previous lane section
					lane_info.lane_id_ = lane_section->GetConnectingLaneId(lane_info.lane_id_, PREDECESSOR);
					lane_section = GetLaneSectionByIdx(--lane_info.lane_section_idx_);
				}
			}

			// If new lane is not driving, try to move into a close driving lane
			if (!lane_section->GetLaneById(lane_info.lane_id_)->IsDriving())
			{
				lane_info.lane_id_ = lane_section->FindClosestDrivingLane(lane_info.lane_id_);
				if (lane_info.lane_id_ == 0)
				{
					LOG("Failed to find a closest driving lane");
				}
			}
		}
	}

	return lane_info;
}

double Road::GetLaneWidthByS(double s, int lane_id)
{
	LaneSection *lsec;

	if (GetNumberOfLaneSections() < 1)
	{
		return 0.0;
	}

	for (size_t i = 0; i < GetNumberOfLaneSections(); i++)
	{
		lsec = GetLaneSectionByIdx((int)i);
		if (s < lsec->GetS() + lsec->GetLength())
		{
			break;
		}
	}

	return lsec->GetWidth(s, lane_id);
}

double Road::GetSpeedByS(double s)
{
	if (type_.size() > 0)
	{
		size_t i;
		for (i = 0; i < type_.size() - 1 && s > type_[i + 1]->s_; i++);

		return type_[i]->speed_;
	}

	// No type entries, fall back to a speed based on nr of lanes
	return 0;
}

Geometry* Road::GetGeometry(int idx)
{
	if (idx < 0 || idx + 1 > (int)geometry_.size())
	{
		LOG("Road::GetGeometry index %d out of range [0:%d]\n", idx, (int)geometry_.size());
		return 0;
	}
	return geometry_[idx]; 
}

void LaneSection::Print()
{
	LOG("LaneSection: %.2f, %d lanes:\n", s_, (int)lane_.size());
	
	for (size_t i=0; i<lane_.size(); i++)
	{
		lane_[i]->Print();
	}
}

Lane* LaneSection::GetLaneByIdx(int idx)
{
	if (idx < (int)lane_.size())
	{
		return lane_[idx];
	}

	return 0;
}

Lane* LaneSection::GetLaneById(int id)
{
	for (size_t i=0; i<lane_.size(); i++)
	{
		if (lane_[i]->GetId() == id)
		{
			return lane_[i];
		}
	}
	return 0;
}

int LaneSection::FindClosestDrivingLane(int id)
{
	int id_best = id;
	size_t delta_best = lane_.size() + 1;

	for (size_t i = 0; i < lane_.size(); i++)
	{
		if (lane_[i]->IsDriving())
		{
			if ((abs(lane_[i]->GetId() - id) < delta_best) || 
				(abs(lane_[i]->GetId() - id) == delta_best && abs(lane_[i]->GetId()) < abs(id_best)))
			{
				delta_best = abs(lane_[i]->GetId() - id);
				id_best = lane_[i]->GetId();
			}
		}
	}
	return id_best;
}

int LaneSection::GetLaneIdByIdx(int idx)
{
	if (idx > (int)lane_.size() - 1)
	{
		LOG("LaneSection::GetLaneIdByIdx Error: index %d, only %d lanes\n", idx, (int)lane_.size());
		return 0;
	}
	else
	{
		return (lane_[idx]->GetId());
	}
}

int LaneSection::GetLaneIdxById(int id)
{
	for (int i = 0; i<(int)lane_.size(); i++)
	{
		if (lane_[i]->GetId() == id)
		{
			return i;
		}
	}
	return -1;
}

int LaneSection::GetNumberOfDrivingLanes()
{
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++)
	{
		if (lane_[i]->IsDriving())
		{
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNumberOfDrivingLanesSide(int side)
{
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++)
	{
		if (SIGN(lane_[i]->GetId()) == SIGN(side) && lane_[i]->IsDriving())
		{
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNUmberOfLanesRight()
{
	int counter = 0;

	for (size_t i=0; i<lane_.size(); i++)
	{
		if (lane_[i]->GetId() < 0)
		{
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNUmberOfLanesLeft()
{
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++)
	{
		if (lane_[i]->GetId() > 0)
		{
			counter++;
		}
	}
	return counter;
}

double LaneSection::GetWidth(double s, int lane_id)
{
	if (lane_id == 0)
	{
		return 0.0;  // reference lane has no width
	}

	Lane *lane = GetLaneById(lane_id);
	if (lane == 0)
	{
		LOG("Error (lane id %d)\n", lane_id);
		return 0.0;
	}

	LaneWidth *lane_width = lane->GetWidthByS(s - s_);
	if (lane_width == 0) // No lane width registered
	{
		return 0.0;
	}

	// Calculate local s-parameter in width segment
	double ds = s - (s_ + lane_width->GetSOffset());

	// Calculate width at local s
	return lane_width->poly3_.Evaluate(ds);
}

double LaneSection::GetOuterOffset(double s, int lane_id)
{
	double width = GetWidth(s, lane_id);

	if (abs(lane_id) == 1)
	{
		// this is the last lane, next to reference lane of width = 0. Stop here.
		return width;
	}
	else
	{
		int step = lane_id < 0 ? +1 : -1;
		return (width + GetOuterOffset(s, lane_id + step));
	}
}

double LaneSection::GetCenterOffset(double s, int lane_id)
{
	if (lane_id == 0)
	{
		// Reference lane (0) has no width
		return 0.0;
	}
	double inner_offset = GetOuterOffset(s, lane_id);
	double width = GetWidth(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return inner_offset - width / 2;
}

double LaneSection::GetOuterOffsetHeading(double s, int lane_id)
{
	if (lane_id == 0)
	{
		return 0.0;  // reference lane has no width
	}

	Lane *lane = GetLaneById(lane_id);
	if (lane == 0)
	{
		LOG("LaneSection::GetOuterOffsetHeading Error (lane id %d)\n", lane_id);
		return 0.0;
	}

	LaneWidth *lane_width = lane->GetWidthByS(s - s_);
	if (lane_width == 0) // No lane width registered
	{
		return 0.0;
	}

	// Calculate local s-parameter in width segment
	double ds = s - (s_ + lane_width->GetSOffset());

	// Calculate heading at local s
	double heading = lane_width->poly3_.EvaluatePrim(ds);

	if (abs(lane_id) == 1)
	{
		// this is the last lane, next to reference lane of width = 0. Stop here.
		return heading;
	}
	else
	{
		int step = lane_id < 0 ? +1 : -1;
		return (heading + GetOuterOffsetHeading(s, lane_id + step));
	}
}

double LaneSection::GetCenterOffsetHeading(double s, int lane_id)
{
	int step = lane_id < 0 ? +1 : -1;

	if (lane_id == 0)
	{
		// Reference lane (0) has no width
		return 0.0;
	}
	double inner_offset_heading = GetOuterOffsetHeading(s, lane_id + step);
	double outer_offset_heading = GetOuterOffsetHeading(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return (inner_offset_heading + outer_offset_heading) / 2;
}

void LaneSection::AddLane(Lane *lane)
{
	lane_.push_back(lane);
}

int LaneSection::GetConnectingLaneId(int incoming_lane_id, LinkType link_type)
{
	int id = incoming_lane_id;

	if (GetLaneById(id) == 0)
	{
		LOG("Lane id %d not available in lane section!", id);
		return 0;
	}

	if (GetLaneById(id)->GetLink(link_type))
	{
		id = GetLaneById(id)->GetLink(link_type)->GetId();
	}
	else
	{
		// if no driving lane found - stay on same index
		id = incoming_lane_id;
	}
	
	return id;
}

double LaneSection::GetWidthBetweenLanes(int lane_id1, int lane_id2, double s)
{
	double lanewidth = (std::fabs(GetCenterOffset(s, lane_id1)) - std::fabs(GetCenterOffset(s, lane_id2)));

	return lanewidth;
}

// Offset from lane1 to lane2 in direction of reference line
double LaneSection::GetOffsetBetweenLanes(int lane_id1, int lane_id2, double s)
{
	double laneCenter1 = GetCenterOffset(s, lane_id1) * SIGN(lane_id1);
	double laneCenter2 = GetCenterOffset(s, lane_id2) * SIGN(lane_id2);
	return (laneCenter2 - laneCenter1);
}

RoadLink::RoadLink(LinkType type, pugi::xml_node node)
{
	string element_type = node.attribute("elementType").value();
	string contact_point_type = "";
	type_ = type;
	element_id_ = atoi(node.attribute("elementId").value());
	
	if (node.attribute("contactPoint") != NULL)
	{
		contact_point_type = node.attribute("contactPoint").value();
	}

	if (element_type == "road")
	{
		element_type_ = ELEMENT_TYPE_ROAD;
		if (contact_point_type == "start")
		{
			contact_point_type_ = CONTACT_POINT_START;
		}
		else if (contact_point_type == "end")
		{
			contact_point_type_ = CONTACT_POINT_END;
		}
		else
		{
			LOG("Unsupported element type: %s\n", contact_point_type.c_str());
			contact_point_type_ = CONTACT_POINT_UNKNOWN;
		}
	}
	else if (element_type == "junction")
	{
		element_type_ = ELEMENT_TYPE_JUNCTION;
		contact_point_type_ = CONTACT_POINT_NONE;
	}
	else
	{
		LOG("Unsupported element type: %s\n", element_type.c_str());
		element_type_ = ELEMENT_TYPE_UNKNOWN;
	}
}

void RoadLink::Print()
{
	cout << "RoadLink type: " << type_ << " id: " << element_id_ << " element type: " << element_type_ << " contact point type: " << contact_point_type_ << endl;
}

Road::~Road()
{
	for (size_t i=0; i<geometry_.size(); i++)
	{
		delete(geometry_[i]);
	}
	for (size_t i=0; i<elevation_profile_.size(); i++)
	{
		delete(elevation_profile_[i]);
	}
	for (size_t i=0; i<link_.size(); i++)
	{
		delete(link_[i]);
	}
}

void Road::Print()
{
	LOG("Road id: %d length: %.2f\n", id_, GetLength());
	cout << "Geometries:" << endl;

	for (size_t i = 0; i < geometry_.size(); i++)
	{
		cout << "Geometry type: " << geometry_[i]->GetType() << endl;
	}

	for (size_t i=0; i<link_.size(); i++)
	{
		link_[i]->Print();
	}

	for (size_t i=0; i<lane_section_.size(); i++)
	{
		lane_section_[i]->Print();
	}

	for (size_t i=0; i<lane_offset_.size(); i++)
	{
		lane_offset_[i]->Print();
	}
}

void Road::AddLine(Line *line)
{
	geometry_.push_back((Geometry*)line);
}

void Road::AddArc(Arc *arc)
{
	geometry_.push_back((Geometry*)arc);
}

void Road::AddSpiral(Spiral *spiral)
{
	if (abs(spiral->GetCurvEnd()) > CURV_ZERO && abs(spiral->GetCurvStart()) > CURV_ZERO)
	{
		// not starting from zero curvature (straight line)
		// need to calculate S starting value, and rotation 

		// First identify end with lowest curvature
		double curvature_min;
		if (abs(spiral->GetCurvStart()) < abs(spiral->GetCurvEnd()))
		{
			curvature_min = spiral->GetCurvStart();
		}
		else
		{
			curvature_min = spiral->GetCurvEnd();
		}

		// How long do we need to follow the spiral to reach min curve value?
		double c_dot = (spiral->GetCurvEnd() - spiral->GetCurvStart()) / spiral->GetLength();
		double ds = curvature_min / c_dot;

		// Find out x, y, heading of start position
		double x, y, heading;
		odrSpiral(ds, c_dot, &x, &y, &heading);

		spiral->SetX0(x);
		spiral->SetY0(y);
		spiral->SetH0(heading);
		spiral->SetS0(ds);
		spiral->SetCDot(c_dot);
	}
	else
	{
		spiral->SetCDot((spiral->GetCurvEnd() - spiral->GetCurvStart()) / spiral->GetLength());
	}
	geometry_.push_back((Geometry*)spiral);
}

void Road::AddPoly3(Poly3 *poly3)
{
	geometry_.push_back((Geometry*)poly3);
	Poly3 *p3 = (Poly3*)geometry_.back();
	
	// Calculate umax (valid interval)
	int step_len = 1;
	double s = 0;
	double x0 = 0;
	double y0 = 0;
	double x1 = 0;
	double y1 = 0;

	while(s < p3->GetLength() - 0.5*step_len)
	{
		x1 = x0 + step_len;
		y1 = p3->poly3_.Evaluate(s);
		s += sqrt((x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0));
		x0 = x1;
		y0 = y1;
	}
	p3->SetUMax(x0);
}

void Road::AddParamPoly3(ParamPoly3 *param_poly3)
{
	geometry_.push_back((Geometry*)param_poly3);
}

void Road::AddElevation(Elevation *elevation)
{
	// Adjust last elevation length
	if (elevation_profile_.size() > 0)
	{
		Elevation *e_previous = elevation_profile_.back();
		e_previous->SetLength(elevation->GetS() - e_previous->GetS());
	}
	elevation->SetLength(length_ - elevation->GetS());

	elevation_profile_.push_back((Elevation*)elevation);
}

Elevation* Road::GetElevation(int idx)
{ 
	if (idx < 0 || idx >= elevation_profile_.size())
	{
		return 0;
	}
	
	return elevation_profile_[idx];
}

double Road::GetLaneOffset(double s)
{
	int i = 0;

	if (lane_offset_.size() == 0)
	{
		return 0;
	}

	for (; i + 1 < (int)lane_offset_.size(); i++)
	{
		if (s < lane_offset_[i + 1]->GetS())
		{
			break;
		}
	}
	return (lane_offset_[i]->GetLaneOffset(s));
}

double Road::GetLaneOffsetPrim(double s)
{
	int i = 0;

	if (lane_offset_.size() == 0)
	{
		return 0;
	}

	for (; i + 1 < (int)lane_offset_.size(); i++)
	{
		if (s < lane_offset_[i + 1]->GetS())
		{
			break;
		}
	}
	return (lane_offset_[i]->GetLaneOffsetPrim(s));
}

int Road::GetNumberOfLanes(double s)
{
	LaneSection *lsec = GetLaneSectionByS(s);

	if (lsec)
	{
		return (lsec->GetNumberOfLanes());
	}

	return 0;
}

int Road::GetNumberOfDrivingLanes(double s)
{
	LaneSection *lsec = GetLaneSectionByS(s);

	if (lsec)
	{
		return (lsec->GetNumberOfDrivingLanes());
	}
	
	return 0;
}

Lane* Road::GetDrivingLaneByIdx(double s, int idx)
{
	int count = 0;

	LaneSection *ls = GetLaneSectionByS(s);

	for (int i = 0; i < ls->GetNumberOfLanes(); i++)
	{
		if (ls->GetLaneByIdx(i)->IsDriving())
		{
			if (count++ == idx)
			{
				return ls->GetLaneByIdx(i);
			}
		}
	}

	return 0;
}

int Road::GetNumberOfDrivingLanesSide(double s, int side)
{
	int i;

	for (i = 0; i < GetNumberOfLaneSections() - 1; i++)
	{
		if (s < lane_section_[i]->GetS())
		{
			break;
		}
	}

	return (lane_section_[i]->GetNumberOfDrivingLanesSide(side));
}

void Road::AddLaneOffset(LaneOffset *lane_offset)
{
	// Adjust lane offset length
	if (lane_offset_.size() > 0)
	{
		LaneOffset *lo_previous = lane_offset_.back();
		lo_previous->SetLength(lane_offset->GetS() - lo_previous->GetS());
	}
	lane_offset->SetLength(length_ - lane_offset->GetS());
	
	lane_offset_.push_back((LaneOffset*)lane_offset);
}

double Road::GetCenterOffset(double s, int lane_id)
{
	// First find out what lane section 
	LaneSection *lane_section = GetLaneSectionByS(s);
	if (lane_section)
	{
		return lane_section->GetCenterOffset(s, lane_id);
	}

	return 0.0;
}

RoadLink* Road::GetLink(LinkType type)
{
	for (size_t i=0; i<link_.size(); i++)
	{
		if (link_[i]->GetType() == type)
		{
			return link_[i];
		}
	}
	return 0;  // Link of requested type is missing
}

void Road::AddLaneSection(LaneSection *lane_section)
{
	// Adjust last elevation section length
	if (lane_section_.size() > 0)
	{
		LaneSection *ls_previous = lane_section_.back();
		ls_previous->SetLength(lane_section->GetS() - ls_previous->GetS());
	}
	lane_section->SetLength(length_ - lane_section->GetS());

	lane_section_.push_back((LaneSection*)lane_section);
}

bool Road::GetZAndPitchByS(double s, double *z, double *pitch, int *index)
{
	if (GetNumberOfElevations() > 0)
	{
		if (*index < 0 || *index >= GetNumberOfElevations())
		{
			*index = 0;
		}
		Elevation *elevation = GetElevation(*index);
		if (elevation == NULL)
		{
			LOG("Elevation error NULL, nelev: %d elev_idx: %d\n", GetNumberOfElevations(), *index);
			return false;
		}

		if (elevation && s > elevation->GetS() + elevation->GetLength())
		{
			while (s > elevation->GetS() + elevation->GetLength() && *index < GetNumberOfElevations() - 1)
			{
				// Move to next elevation section
				elevation = GetElevation(++*index);
			}
		}
		else if (elevation && s < elevation->GetS())
		{
			while (s < elevation->GetS() && *index > 0)
			{
				// Move to previous elevation section
				elevation = GetElevation(--*index);
			}
		}

		if (elevation)
		{
			double p = s - elevation->GetS();
			*z = elevation->poly3_.Evaluate(p);
			*pitch = -elevation->poly3_.EvaluatePrim(p);

			return true;
		}
	}

	return false;
}

Road* OpenDrive::GetRoadById(int id)
{
	for (size_t i=0; i<road_.size(); i++)
	{
		if (road_[i]->GetId() == id)
		{
			return road_[i];
		}
	}
	return 0;
}

Road *OpenDrive::GetRoadByIdx(int idx)
{
	if (idx >= 0 && idx < (int)road_.size())
	{
		return road_[idx];
	}
	else
	{
		return 0;
	}
}

Geometry *OpenDrive::GetGeometryByIdx(int road_idx, int geom_idx)
{
	if (road_idx >= 0 && road_idx < (int)road_.size())
	{
		return road_[road_idx]->GetGeometry(geom_idx);
	}
	else
	{
		return 0;
	}
}

Junction* OpenDrive::GetJunctionById(int id)
{
	for (size_t i=0; i<junction_.size(); i++)
	{
		if (junction_[i]->GetId() == id)
		{
			return junction_[i];
		}
	}
	return 0;
}

Junction *OpenDrive::GetJunctionByIdx(int idx)
{	
	if (idx >= 0 && idx < (int)junction_.size())
	{
		return junction_[idx];
	}
	else
	{
		LOG("GetJunctionByIdx error (idx %d, njunctions %d)\n", idx, (int)junction_.size());
		return 0;
	}
}

OpenDrive::OpenDrive(const char *filename)
{
	if (!LoadOpenDriveFile(filename))
	{
		LOG("Error loading OpenDrive %s\n", filename);
		throw std::invalid_argument("Failed to load OpenDrive file");
	}
}

bool OpenDrive::LoadOpenDriveFile(const char *filename, bool replace)
{
	mt_rand.seed((unsigned int)time(0));

	if (replace)
	{
		for (size_t i=0; i<road_.size(); i++)
		{
			delete road_[i];
		}
		road_.clear();

		for (size_t i=0; i<junction_.size(); i++)
		{
			delete junction_[i];
		}
		junction_.clear();
	}

	odr_filename_ = filename;

	if (odr_filename_ == "")
	{
		return false;
	}

	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(filename);
	if (!result)
	{
		// Try current folder
		std::string path = std::string(filename);
		std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
		LOG("Failed to load %s - looking for file %s in current folder", filename, base_filename.c_str());
		pugi::xml_parse_result result = doc.load_file(base_filename.c_str());

		if (!result)
		{
			throw std::invalid_argument(std::string("Failed to load OpenDRIVE file ") + std::string(filename));

			return false;
		}
	}

	pugi::xml_node node = doc.child("OpenDRIVE");
	if (node == NULL)
	{
		cout << "Root null" << endl;
		throw std::invalid_argument("The file does not seem to be an OpenDRIVE");
	}

	for (pugi::xml_node road_node = node.child("road"); road_node; road_node = road_node.next_sibling("road"))
	{
		Road *r = new Road(atoi(road_node.attribute("id").value()), road_node.attribute("name").value());
		r->SetLength(atof(road_node.attribute("length").value()));
		r->SetJunction(atoi(road_node.attribute("junction").value()));

		for (pugi::xml_node type_node = road_node.child("type"); type_node; type_node = type_node.next_sibling("type"))
		{
			RoadTypeEntry *r_type = new RoadTypeEntry();
			
			std::string type = type_node.attribute("type").value();
			if (type == "unknown")
			{
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_UNKNOWN;
			}
			else if (type == "rural")
			{
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_RURAL;
			}
			else if (type == "motorway")
			{
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_MOTORWAY;
			}
			else if (type == "town")
			{
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_TOWN;
			}
			else if (type == "lowSpeed")
			{
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_LOWSPEED;
			}
			else if (type == "pedestrian")
			{
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_PEDESTRIAN;
			}
			else if (type == "bicycle")
			{
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_BICYCLE;
			}
			else if (type == "")
			{
				LOG("Missing road type - setting default (rural)");
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_RURAL;
			}
			else
			{
				LOG("Unsupported road type: %s - assuming rural", type.c_str());
				r_type->road_type_ = roadmanager::RoadType::ROADTYPE_RURAL;
			}

			r_type->s_ = atof(type_node.attribute("s").value());

			// Check for optional speed record
			pugi::xml_node speed = type_node.child("speed");
			if (speed != NULL)
			{
				r_type->speed_ = atof(speed.attribute("max").value());
				std::string unit = speed.attribute("unit").value();
				if (unit == "km/h")
				{
					r_type->speed_ /= 3.6;  // Convert to m/s
				}
				else if (unit == "mph")
				{
					r_type->speed_ *= 0.44704; // Convert to m/s
				}
				else if (unit == "m/s")
				{
					// SE unit - do nothing
				}
				else 
				{
					LOG("Unsupported speed unit: %s - assuming SE unit m/s", unit.c_str());
				}
			}

			r->AddRoadType(r_type);
		}

		pugi::xml_node link = road_node.child("link");
		if (link != NULL)
		{
			pugi::xml_node successor = link.child("successor");
			if (successor != NULL)
			{
				r->AddLink(new RoadLink(SUCCESSOR, successor));
			}

			pugi::xml_node predecessor = link.child("predecessor");
			if (predecessor != NULL)
			{
				r->AddLink(new RoadLink(PREDECESSOR, predecessor));
			}
		}

		pugi::xml_node plan_view = road_node.child("planView");
		if (plan_view != NULL)
		{
			for (pugi::xml_node geometry = plan_view.child("geometry"); geometry; geometry = geometry.next_sibling())
			{
				double s = atof(geometry.attribute("s").value());
				double x = atof(geometry.attribute("x").value());
				double y = atof(geometry.attribute("y").value());
				double hdg = atof(geometry.attribute("hdg").value());
				double length = atof(geometry.attribute("length").value());

				pugi::xml_node type = geometry.last_child();
				if (type != NULL)
				{
					// Find out the type of geometry
					if (!strcmp(type.name(), "line"))
					{
						r->AddLine(new Line(s, x, y, hdg, length));
					}
					else if (!strcmp(type.name(), "arc"))
					{
						double curvature = atof(type.attribute("curvature").value());
						r->AddArc(new Arc(s, x, y, hdg, length, curvature));
					}
					else if (!strcmp(type.name(), "spiral"))
					{
						double curv_start = atof(type.attribute("curvStart").value());
						double curv_end = atof(type.attribute("curvEnd").value());
						r->AddSpiral(new Spiral(s, x, y, hdg, length, curv_start, curv_end));
					}
					else if (!strcmp(type.name(), "poly3"))
					{
						double a = atof(type.attribute("a").value());
						double b = atof(type.attribute("b").value());
						double c = atof(type.attribute("c").value());
						double d = atof(type.attribute("d").value());
						r->AddPoly3(new Poly3(s, x, y, hdg, length, a, b, c, d));
					}
					else if (!strcmp(type.name(), "paramPoly3"))
					{
						double aU = atof(type.attribute("aU").value());
						double bU = atof(type.attribute("bU").value());
						double cU = atof(type.attribute("cU").value());
						double dU = atof(type.attribute("dU").value());
						double aV = atof(type.attribute("aV").value());
						double bV = atof(type.attribute("bV").value());
						double cV = atof(type.attribute("cV").value());
						double dV = atof(type.attribute("dV").value());
						ParamPoly3::PRangeType p_range = ParamPoly3::P_RANGE_NORMALIZED;
						
						pugi::xml_attribute attr = type.attribute("pRange");
						if (attr && !strcmp(attr.value(), "arcLength"))
						{
							p_range = ParamPoly3::P_RANGE_ARC_LENGTH;
						}

						ParamPoly3 *pp3 = new ParamPoly3(s, x, y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV, p_range);
						if (pp3 != NULL)
						{
							r->AddParamPoly3(pp3);
						}
						else
						{
							LOG("ParamPoly3: Major error\n");
						}
					}
					else
					{
						cout << "Unknown geometry type: " << type.name() << endl;
					}
				}
				else
				{
					cout << "Type == NULL" << endl;
				}
			}
		}
		
		pugi::xml_node elevation_profile = road_node.child("elevationProfile");
		if (elevation_profile != NULL)
		{
			for (pugi::xml_node elevation = elevation_profile.child("elevation"); elevation; elevation = elevation.next_sibling())
			{
				double s = atof(elevation.attribute("s").value());
				double a = atof(elevation.attribute("a").value());
				double b = atof(elevation.attribute("b").value());
				double c = atof(elevation.attribute("c").value());
				double d = atof(elevation.attribute("d").value());

				Elevation *ep = new Elevation(s, a, b, c, d);
				if (ep != NULL)
				{
					r->AddElevation(ep);
				}
				else
				{
					LOG("Elevation: Major error\n");
				}
			}
		}
		
		pugi::xml_node lanes = road_node.child("lanes");
		if (lanes != NULL)
		{
			for (pugi::xml_node_iterator child = lanes.children().begin(); child != lanes.children().end(); child++)
			{
				if (!strcmp(child->name(), "laneOffset"))
				{
					double s = atof(child->attribute("s").value());
					double a = atof(child->attribute("a").value());
					double b = atof(child->attribute("b").value());
					double c = atof(child->attribute("c").value());
					double d = atof(child->attribute("d").value());
					r->AddLaneOffset(new LaneOffset(s, a, b, c, d));
				}
				else if (!strcmp(child->name(), "laneSection"))
				{
					double s = atof(child->attribute("s").value());
					LaneSection *lane_section = new LaneSection(s);
					r->AddLaneSection(lane_section);

					for (pugi::xml_node_iterator child2 = child->children().begin(); child2 != child->children().end(); child2++)
					{
						if (!strcmp(child2->name(), "left"))
						{
							//LOG("Lane left\n");
						}
						else if (!strcmp(child2->name(), "right"))
						{
							//LOG("Lane right\n");
						}
						else if (!strcmp(child2->name(), "center"))
						{
							//LOG("Lane center\n");
						}
						else
						{
							LOG("Unsupported lane side: %s\n", child2->name());
							continue;
						}
						for (pugi::xml_node_iterator lane_node = child2->children().begin(); lane_node != child2->children().end(); lane_node++)
						{
							if (strcmp(lane_node->name(), "lane"))
							{
								LOG("Unexpected element: %s, expected \"lane\"\n", lane_node->name());
								continue;
							}

							Lane::LaneType lane_type = Lane::LANE_TYPE_NONE;
							if (lane_node->attribute("type") == 0 || !strcmp(lane_node->attribute("type").value(), ""))
							{
								LOG("Lane type error");
							}
							if (!strcmp(lane_node->attribute("type").value(), "none"))
							{
								lane_type = Lane::LANE_TYPE_NONE;
							}
							else  if (!strcmp(lane_node->attribute("type").value(), "driving"))
							{
								lane_type = Lane::LANE_TYPE_DRIVING;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "stop"))
							{
								lane_type = Lane::LANE_TYPE_STOP;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "shoulder"))
							{
								lane_type = Lane::LANE_TYPE_SHOULDER;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "biking"))
							{
								lane_type = Lane::LANE_TYPE_BIKING;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "sidewalk"))
							{
								lane_type = Lane::LANE_TYPE_SIDEWALK;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "border"))
							{
								lane_type = Lane::LANE_TYPE_BORDER;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "restricted"))
							{
								lane_type = Lane::LANE_TYPE_RESTRICTED;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "parking"))
							{
								lane_type = Lane::LANE_TYPE_PARKING;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "bidirectional"))
							{
								lane_type = Lane::LANE_TYPE_BIDIRECTIONAL;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "medcian"))
							{
								lane_type = Lane::LANE_TYPE_MEDIAN;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "special1"))
							{
								lane_type = Lane::LANE_TYPE_SPECIAL1;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "special2"))
							{
								lane_type = Lane::LANE_TYPE_SPECIAL2;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "special3"))
							{
								lane_type = Lane::LANE_TYPE_SPECIAL3;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "roadmarks"))
							{
								lane_type = Lane::LANE_TYPE_ROADMARKS;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "tram"))
							{
								lane_type = Lane::LANE_TYPE_TRAM;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "rail"))
							{
								lane_type = Lane::LANE_TYPE_RAIL;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "entry") ||
								!strcmp(lane_node->attribute("type").value(), "mwyEntry"))
							{
								lane_type = Lane::LANE_TYPE_ENTRY;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "exit") ||
								!strcmp(lane_node->attribute("type").value(), "mwyExit"))
							{
								lane_type = Lane::LANE_TYPE_EXIT;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "offRamp"))
							{
								lane_type = Lane::LANE_TYPE_OFF_RAMP;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "onRamp"))
							{
								lane_type = Lane::LANE_TYPE_ON_RAMP;
							}
							else
							{
								LOG("unknown lane type: %s (road id=%d)\n", lane_node->attribute("type").value(), r->GetId());
							}

							int lane_id = atoi(lane_node->attribute("id").value());
							Lane *lane = new Lane(lane_id, lane_type);
							if (lane == NULL)
							{
								LOG("Error: creating lane\n");
								return false;
							}
							lane_section->AddLane(lane);

							// Link
							pugi::xml_node link = lane_node->child("link");
							if (link != NULL)
							{
								pugi::xml_node successor = link.child("successor");
								if (successor != NULL)
								{
									lane->AddLink(new LaneLink(SUCCESSOR, atoi(successor.attribute("id").value())));
								}
								pugi::xml_node predecessor = link.child("predecessor");
								if (predecessor != NULL)
								{
									lane->AddLink(new LaneLink(PREDECESSOR, atoi(predecessor.attribute("id").value())));
								}
							}

							// Width
							for (pugi::xml_node width = lane_node->child("width"); width; width = width.next_sibling("width"))
							{
								double s_offset = atof(width.attribute("sOffset").value());
								double a = atof(width.attribute("a").value());
								double b = atof(width.attribute("b").value());
								double c = atof(width.attribute("c").value());
								double d = atof(width.attribute("d").value());
								lane->AddLaneWIdth(new LaneWidth(s_offset, a, b, c, d));
							}
						}
					}
				}
				else
				{
					LOG("Unsupported lane type: %s\n", child->name());
				}
			}
		}

		if (r->GetNumberOfLaneSections() == 0)
		{
			// Add empty center reference lane
			LaneSection *lane_section = new LaneSection(0.0);
			lane_section->AddLane(new Lane(0, Lane::LANE_TYPE_NONE));
			r->AddLaneSection(lane_section);
		}
			
		road_.push_back(r);
	}

	for (pugi::xml_node junction_node = node.child("junction"); junction_node; junction_node = junction_node.next_sibling("junction"))
	{
		int id = atoi(junction_node.attribute("id").value());
		std::string name = junction_node.attribute("name").value();

		Junction *j = new Junction(id, name);

		for (pugi::xml_node connection_node = junction_node.child("connection"); connection_node; connection_node = connection_node.next_sibling("connection"))
		{
			if (connection_node != NULL)
			{
				int id = atoi(connection_node.attribute("id").value());
				(void)id;
				int incoming_road_id = atoi(connection_node.attribute("incomingRoad").value());
				int connecting_road_id = atoi(connection_node.attribute("connectingRoad").value());
				Road *incoming_road = GetRoadById(incoming_road_id);
				Road *connecting_road = GetRoadById(connecting_road_id);
				ContactPointType contact_point = CONTACT_POINT_UNKNOWN;
				std::string contact_point_str = connection_node.attribute("contactPoint").value();
				if (contact_point_str == "start")
				{
					contact_point = CONTACT_POINT_START;
				}
				else if (contact_point_str == "end")
				{
					contact_point = CONTACT_POINT_END;
				}
				else
				{
					LOG("Unsupported contact point: %s\n", contact_point_str.c_str());
				}

				Connection *connection = new Connection(incoming_road, connecting_road, contact_point);

				for (pugi::xml_node lane_link_node = connection_node.child("laneLink"); lane_link_node; lane_link_node = lane_link_node.next_sibling("laneLink"))
				{
					int from_id = atoi(lane_link_node.attribute("from").value());
					int to_id = atoi(lane_link_node.attribute("to").value());
					connection->AddJunctionLaneLink(from_id, to_id);
				}
				j->AddConnection(connection);
			}
		}
		junction_.push_back(j);
	}

	// CheckConnections();

	return true;
}

Connection::Connection(Road* incoming_road, Road *connecting_road, ContactPointType contact_point)
{
	// Find corresponding road objects
	incoming_road_ = incoming_road;
	connecting_road_ = connecting_road;
	contact_point_ = contact_point;
}

Connection::~Connection()
{ 
	for (size_t i=0; i<lane_link_.size(); i++) 
	{
		delete lane_link_[i];
	}
}

void Connection::AddJunctionLaneLink(int from, int to)
{
	lane_link_.push_back(new JunctionLaneLink(from, to));
}

int Connection::GetConnectingLaneId(int incoming_lane_id)
{
	for (size_t i = 0; i < lane_link_.size(); i++)
	{
		if (lane_link_[i]->from_ == incoming_lane_id)
		{
			return lane_link_[i]->to_;
		}
	}
	return 0;
}

void Connection::Print()
{
	LOG("Connection: incoming %d connecting %d\n", incoming_road_->GetId(), connecting_road_->GetId());
	for (size_t i = 0; i < lane_link_.size(); i++)
	{
		lane_link_[i]->Print();
	}
}

Junction::~Junction()
{ 
	for (size_t i=0; i<connection_.size(); i++)
	{
		delete connection_[i];
	}
}

int Junction::GetNumberOfRoadConnections(int roadId, int laneId)
{
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++)
	{
		Connection * connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad() && roadId == connection->GetIncomingRoad()->GetId())
		{
			for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++)
			{
				JunctionLaneLink *lane_link = connection->GetLaneLink(j);
				if (laneId == lane_link->from_)
				{
					counter++;
				}
			}
		}
	}
	return counter;
}

LaneRoadLaneConnection Junction::GetRoadConnectionByIdx(int roadId, int laneId, int idx)
{
	int counter = 0;
	LaneRoadLaneConnection lane_road_lane_connection;

	for (int i = 0; i < GetNumberOfConnections(); i++)
	{
		Connection *connection = GetConnectionByIdx(i);

		if (connection && connection->GetIncomingRoad() && roadId == connection->GetIncomingRoad()->GetId())
		{
			for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++)
			{
				JunctionLaneLink *lane_link = connection->GetLaneLink(j);
				if (laneId == lane_link->from_)
				{
					if (counter == idx)
					{
						lane_road_lane_connection.SetLane(laneId);
						lane_road_lane_connection.contact_point_ = connection->GetContactPoint();
						lane_road_lane_connection.SetConnectingRoad(connection->GetConnectingRoad()->GetId());
						lane_road_lane_connection.SetConnectingLane(lane_link->to_);
						// find out driving direction
						int laneSectionId;
						if (lane_link->to_ < 0)
						{
							laneSectionId = 0;
						}
						else
						{
							laneSectionId = connection->GetConnectingRoad()->GetNumberOfLaneSections() - 1;
						}
						if (!connection->GetConnectingRoad()->GetLaneSectionByIdx(laneSectionId)->GetLaneById(lane_link->to_)->IsDriving())
						{
							LOG("OpenDrive::GetJunctionConnection target lane not driving! from %d, %d to %d, %d\n",
								roadId, laneId, connection->GetConnectingRoad()->GetId(), lane_link->to_);
						}

						return lane_road_lane_connection;
					}
					counter++;
				}
			}
		}
	}

//	LOG("RoadConnection not found!");
	return lane_road_lane_connection;
}

int Junction::GetNoConnectionsFromRoadId(int incomingRoadId)
{
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++)
	{
		Connection * connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad()->GetId() == incomingRoadId)
		{
			counter++;
		}
	}
	
	return counter;
}

int Junction::GetConnectingRoadIdFromIncomingRoadId(int incomingRoadId, int index)
{
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++)
	{
		Connection * connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad()->GetId() == incomingRoadId)
		{
			if (counter == index)
			{
				return GetConnectionByIdx(i)->GetConnectingRoad()->GetId();
			}
			else
			{
				counter++;
			}
		}
	}
	return -1;
}

void Junction::Print()
{
	LOG("Junction %d %s: \n", id_, name_.c_str());

	for (size_t i=0; i<connection_.size(); i++)
	{
		connection_[i]->Print();
	}
}

OpenDrive::~OpenDrive()
{
	for (size_t i = 0; i < road_.size(); i++)
	{
		delete(road_[i]);
	}
	for (size_t i = 0; i < junction_.size(); i++)
	{
		delete(junction_[i]);
	}
}

int OpenDrive::GetTrackIdxById(int id)
{
	for (int i = 0; i<(int)road_.size(); i++)
	{
		if (road_[i]->GetId() == id)
		{
			return i;
		}
	}
	LOG("OpenDrive::GetTrackIdxById Error: Road id %d not found\n", id);
	return -1;
}

int OpenDrive::GetTrackIdByIdx(int idx)
{
	if (idx >= 0 && idx < (int)road_.size())
	{
		return (road_[idx]->GetId());
	}
	LOG("OpenDrive::GetTrackIdByIdx: idx %d out of range [0:%d]\n", idx, (int)road_.size());
	return 0;
}

int OpenDrive::IsDirectlyConnected(int road1_id, int road2_id, double &angle)
{
	Road *road1 = GetRoadById(road1_id);
	Road *road2 = GetRoadById(road2_id);
	RoadLink *link;

	// Look from road 1, both ends, for road 2 
		
	for (int i = 0; i < 2; i++)
	{
		if ((link = road1->GetLink(i == 0 ? LinkType::SUCCESSOR : LinkType::PREDECESSOR)))
		{
			if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
			{
				if (link->GetElementId() == road2->GetId())
				{
					angle = 0;
					return i == 0 ? 1 : -1;
				}
			}
			else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
			{
				Junction *junction = GetJunctionById(link->GetElementId());

				for (int j = 0; j < junction->GetNumberOfConnections(); j++)
				{
					Connection *connection = junction->GetConnectionByIdx(j);
					Position test_pos1;
					Position test_pos2;

					double heading1, heading2;

					// check case where road1 is incoming road
					if (connection->GetIncomingRoad()->GetId() == road1_id && connection->GetConnectingRoad()->GetId() == road2_id)
					{
						test_pos1.SetLanePos(road2_id, 0, 0, 0);
						test_pos2.SetLanePos(road2_id, 0, road2->GetLength(), 0);

						if (connection->GetContactPoint() == CONTACT_POINT_END)
						{
							heading1 = test_pos2.GetH() + M_PI;
							heading2 = test_pos1.GetH() + M_PI;
						}
						else if(connection->GetContactPoint() == CONTACT_POINT_START)
						{
							heading1 = test_pos1.GetH();
							heading2 = test_pos2.GetH();
						}
						else
						{
							LOG("Unexpected contact point %d", connection->GetContactPoint());
							return 0;
						}

						angle = GetAbsAngleDifference(heading1, heading2);

						return i == 0 ? 1 : -1;
					}
					// then check other case where road1 is outgoing from connecting road (connecting road is a road within junction)
					else if (connection->GetConnectingRoad()->GetId() == road2_id && 
						((connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START && connection->GetConnectingRoad()->GetLink(LinkType::SUCCESSOR)->GetElementId() == road1_id) ||
						 (connection->GetContactPoint() == ContactPointType::CONTACT_POINT_END && connection->GetConnectingRoad()->GetLink(LinkType::PREDECESSOR)->GetElementId() == road1_id)) )

					{
						if (connection->GetContactPoint() == CONTACT_POINT_START) // connecting road ends up connecting to road_1
						{
							test_pos1.SetLanePos(road2_id, 0, road2->GetLength(), 0);
							test_pos2.SetLanePos(road2_id, 0, 0, 0);

							if (connection->GetConnectingRoad()->GetLink(LinkType::SUCCESSOR)->GetContactPointType() == CONTACT_POINT_START)  // connecting to start of road_1
							{
								heading1 = test_pos2.GetH();
								heading2 = test_pos1.GetH();
							}
							else if (connection->GetConnectingRoad()->GetLink(LinkType::SUCCESSOR)->GetContactPointType() == CONTACT_POINT_END)  // connecting to end of road_1
							{
								heading1 = test_pos2.GetH() + M_PI;
								heading2 = test_pos1.GetH() + M_PI;
							}
							else
							{
								LOG("Unexpected contact point %d", connection->GetConnectingRoad()->GetLink(LinkType::PREDECESSOR)->GetContactPointType());
								return 0;
							}
						}
						else if (connection->GetContactPoint() == CONTACT_POINT_END) // connecting road start point connecting to road_1 
						{
							test_pos1.SetLanePos(road2_id, 0, 0, 0);
							test_pos2.SetLanePos(road2_id, 0, road2->GetLength(), 0);

							if (connection->GetConnectingRoad()->GetLink(LinkType::PREDECESSOR)->GetContactPointType() == CONTACT_POINT_START)  // connecting to start of road_1
							{
								heading1 = test_pos2.GetH() + M_PI;
								heading2 = test_pos1.GetH() + M_PI;
							}
							else if (connection->GetConnectingRoad()->GetLink(LinkType::PREDECESSOR)->GetContactPointType() == CONTACT_POINT_END)  // connecting to end of road_1
							{
								heading1 = test_pos2.GetH();
								heading2 = test_pos1.GetH();
							}
							else
							{
								LOG("Unexpected contact point %d", connection->GetConnectingRoad()->GetLink(LinkType::PREDECESSOR)->GetContactPointType());
								return 0;
							}
						}
						else
						{
							LOG("Unexpected contact point %d", connection->GetContactPoint());
							return 0;
						}

						angle = GetAbsAngleDifference(heading1, heading2);

						return i == 0 ? 1 : -1;
					}
				}
			}
		}
	}

	return 0;
}

bool OpenDrive::IsIndirectlyConnected(int road1_id, int road2_id, int* &connecting_road_id, int* &connecting_lane_id, int lane1_id, int lane2_id)
{
	Road *road1 = GetRoadById(road1_id);
	Road *road2 = GetRoadById(road2_id);
	RoadLink *link = 0;

	LinkType link_type[2] = { SUCCESSOR , PREDECESSOR };

	// Try both ends
	for (int k = 0; k < 2; k++)
	{
		link = road1->GetLink(link_type[k]);
		if (link == 0)
		{
			continue;
		}

		LaneSection *lane_section = 0;

		if (link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
		{
			if (link->GetElementId() == road2->GetId())
			{
				if (lane1_id != 0 && lane2_id != 0)
				{
					// Check lane connected
					if (link_type[k] == SUCCESSOR)
					{
						lane_section = road1->GetLaneSectionByIdx(road1->GetNumberOfLaneSections() - 1);
					}
					else if (link_type[k] == PREDECESSOR)
					{
						lane_section = road1->GetLaneSectionByIdx(0);
					}
					else
					{
						LOG("Error LinkType %d not suppoered\n", link_type[k]);
						return false;
					}
					if (lane_section == 0)
					{
						LOG("Error lane section == 0\n");
						return false;
					}
					Lane *lane = lane_section->GetLaneById(lane1_id);
					(void)lane;
					if (!(lane_section->GetConnectingLaneId(lane1_id, link_type[k]) == lane2_id))
					{
						return false;
					}
					// Now, check other end lane connectivitiy
				}
				return true;
			}
		}
		// check whether the roads are connected via a junction connecting road and specified lane
		else if (link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
		{
			Junction *junction = GetJunctionById(link->GetElementId());

			for (int i = 0; i < junction->GetNumberOfConnections(); i++)
			{
				Connection *connection = junction->GetConnectionByIdx(i);

				if (connection->GetIncomingRoad()->GetId() == road1_id)
				{
					// Found a connecting road - now check if it connects to second road
					Road *connecting_road = connection->GetConnectingRoad();
					RoadLink *exit_link = 0;

					if (connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START)
					{
						exit_link = connecting_road->GetLink(SUCCESSOR);
					}
					else
					{
						exit_link = connecting_road->GetLink(PREDECESSOR);
					}

					if (exit_link->GetElementId() == road2_id)
					{
						// Finally check that lanes are connected through the junction
						// Look at lane section and locate lane connecting both roads
						// Assume connecting road has only one lane section 
						lane_section = connecting_road->GetLaneSectionByIdx(0);
						if (lane_section == 0)
						{
							LOG("Error lane section == 0\n");
							return false;
						}
						for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
						{
							Lane *lane = lane_section->GetLaneByIdx(j);
							LaneLink *lane_link_predecessor = lane->GetLink(PREDECESSOR);
							LaneLink *lane_link_successor = lane->GetLink(SUCCESSOR);
							if (lane_link_predecessor == 0 || lane_link_successor == 0)
							{
								continue;
							}
							if ((connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START &&
								 lane_link_predecessor->GetId() == lane1_id && 
								 lane_link_successor->GetId() == lane2_id) ||
								(connection->GetContactPoint() == ContactPointType::CONTACT_POINT_END &&
								 lane_link_predecessor->GetId() == lane2_id &&
								 lane_link_successor->GetId() == lane1_id))
							{
								// Found link
								if (connecting_road_id != 0)
								{
									*connecting_road_id = connection->GetConnectingRoad()->GetId();
								}
								if (connecting_lane_id != 0)
								{
									*connecting_lane_id = lane->GetId();
								}
								return true;
							}
						}
					}
				}
			}
		}
		else
		{
			LOG("Error: LinkElementType %d unsupported\n", link->GetElementType());
		}
	}
	
	LOG("Link not found");
	
	return false;
}

int OpenDrive::CheckConnectedRoad(Road *road, RoadLink *link, ContactPointType expected_contact_point_type, Road *road2, RoadLink *link2)
{
	if (link2 == 0)
	{
		return -1;
	}

	if (link2->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		if (link->GetElementId() == road->GetId())
		{
			if (link->GetContactPointType() != expected_contact_point_type)
			{
				LOG("Found connecting road from other end, but contact point is wrong (expected START, got %s)",
					ContactPointType2Str(link->GetContactPointType()).c_str());
			}
		}
	}

	return 0;
}

int OpenDrive::CheckJunctionConnection(Junction *junction, Connection *connection)
{
	if (junction == 0)
	{
		return -1;
	}
	
	// Check if junction is referred to from the connected road
	Road *road = connection->GetConnectingRoad();
	if (road == 0)
	{
		LOG("Error no connecting road");
		return -1;
	}

	RoadLink *link[2];
	link[0] = road->GetLink(LinkType::PREDECESSOR);
	link[1] = road->GetLink(LinkType::SUCCESSOR);
	for (int i = 0; i < 2; i++)
	{
		if (link[i] != 0)
		{
			if (link[i]->GetElementType() != RoadLink::ElementType::ELEMENT_TYPE_ROAD)
			{
				LOG("Expected element type ROAD, found %s", link[i]->GetElementType());
				return -1;
			}
			
			if (link[i]->GetElementId() != connection->GetIncomingRoad()->GetId())
			{
				// Check connection from this outgoing road
				Road *road = GetRoadById(link[i]->GetElementId());
				RoadLink *link2[2];
				link2[0] = road->GetLink(LinkType::PREDECESSOR);
				link2[1] = road->GetLink(LinkType::SUCCESSOR);
				for (int j = 0; j < 2; j++)
				{
					if (link2[j] != 0)
					{
						if (link2[j]->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION &&
							link2[j]->GetElementId() == junction->GetId())
						{
							// Now finally find the reverse link 
							for (int k = 0; k < junction->GetNumberOfConnections(); k++)
							{
								if (junction->GetConnectionByIdx(k)->GetIncomingRoad() == road)
								{
									// Sharing same connecting road?
									if (junction->GetConnectionByIdx(k)->GetConnectingRoad() == connection->GetConnectingRoad())
									{
										return 0;
									}
								}
							}
							LOG("Warning: Missing reverse connection from road %d to %d via junction %d connecting road %d. Potential issue in the OpenDRIVE file.", 
								connection->GetIncomingRoad()->GetId(), road->GetId(), junction->GetId(), connection->GetConnectingRoad()->GetId());
						}
					}
				}
			}
		}
	}

	return -1;
}

int OpenDrive::CheckLink(Road *road, RoadLink *link, ContactPointType expected_contact_point_type)
{
	// does this connection exist in the other direction?
	if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		Road *connecting_road = GetRoadById(link->GetElementId());
		if (connecting_road != 0)
		{
			if (CheckConnectedRoad(road, link, expected_contact_point_type, connecting_road, connecting_road->GetLink(LinkType::PREDECESSOR)) == 0)
			{
				return 0;
			}
			else if (CheckConnectedRoad(road, link, expected_contact_point_type, connecting_road, connecting_road->GetLink(LinkType::SUCCESSOR)) == 0)
			{
				return 0;
			}
			else
			{
				LOG("Warning: Reversed road link %d->%d not found. Might be a flaw in the OpenDRIVE description.", road->GetId(), connecting_road->GetId());
			}
		}
	}
	else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		Junction *junction = GetJunctionById(link->GetElementId());

		// Check all outgoing connections
		for (size_t i = 0; i < junction->GetNumberOfConnections(); i++)
		{
			Connection *connection = junction->GetConnectionByIdx((int)i);

			if (connection->GetIncomingRoad() == road)
			{
				CheckJunctionConnection(junction, connection);
			}
		}
	}

	return 0;
}

int OpenDrive::CheckConnections()
{
	int counter = 0;
	RoadLink *link;

	for (size_t i = 0; i < road_.size(); i++)
	{
		// Check for connections
		if ((link = road_[i]->GetLink(LinkType::PREDECESSOR)) != 0)
		{
			CheckLink(road_[i], link, ContactPointType::CONTACT_POINT_START);
		}
		if ((link = road_[i]->GetLink(LinkType::SUCCESSOR)) != 0)
		{
			CheckLink(road_[i], link, ContactPointType::CONTACT_POINT_END);
		}
	}
	
	return counter;
}

void OpenDrive::Print()
{
	LOG("Roads:\n");
	for (size_t i=0; i<road_.size(); i++)
	{
		road_[i]->Print();
	}

	LOG("junctions\n");
	for (size_t i=0; i<junction_.size(); i++)
	{
		junction_[i]->Print();
	}
}

void Position::Init()
{
	track_id_ = -1;
	lane_id_ = 0;
	s_ = 0.0;
	s_route_ = 0.0;
	t_ = 0.0;
	offset_ = 0.0;
	x_ = 0.0;
	y_ = 0.0;
	z_ = 0.0;
	h_ = 0.0;
	p_ = 0.0;
	r_ = 0.0;
	h_offset_ = 0.0;
	h_road_ = 0.0;
	h_relative_ = 0.0;
	curvature_ = 0.0;
	p_road_ = 0.0;

	z_road_ = 0.0;
	track_idx_ = -1;
	geometry_idx_ = -1;
	lane_section_idx_ = -1;
	lane_idx_ = -1;
	elevation_idx_ = -1;
	route_ = 0;
}

Position::Position()
{
	Init();
}

Position::Position(int track_id, double s, double t)
{
	Init();
	SetTrackPos(track_id, s, t);
}

Position::Position(int track_id, int lane_id, double s, double offset)
{
	Init();
	SetLanePos(track_id, lane_id, s, offset);
}

Position::Position(double x, double y, double z, double h, double p, double r)
{
	Init();
	SetInertiaPos(x, y, z, h, p, r);
}

Position::Position(double x, double y, double z, double h, double p, double r, bool calculateTrackPosition)
{
	Init();
	SetInertiaPos(x, y, z, h, p, r, calculateTrackPosition);
}

Position::~Position()
{
	
}

bool Position::LoadOpenDrive(const char *filename)
{
	return(GetOpenDrive()->LoadOpenDriveFile(filename));
}

OpenDrive* Position::GetOpenDrive()
{
	static OpenDrive od;
	return &od; 
}

int LaneSection::GetClosestLaneIdx(double s, double t, double &offset)
{
	double min_offset = t;  // Initial offset relates to reference line
	int candidate_lane_idx = -1;

	for (int i = 0; i < GetNumberOfLanes(); i++)  // Search through all lanes
	{
		int lane_id = GetLaneIdByIdx(i);
		double laneCenterOffset = SIGN(lane_id) * GetCenterOffset(s, lane_id);

		if (GetLaneById(lane_id)->IsDriving() && (candidate_lane_idx == -1 || fabs(t - laneCenterOffset) < fabs(min_offset)))
		{
			min_offset = t - laneCenterOffset;
 			candidate_lane_idx = i;
		}
	}

	offset = min_offset;

	return candidate_lane_idx;
}

int Position::GotoClosestDrivingLaneAtCurrentPosition()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0)
	{
		LOG("No road %d", track_idx_);
		return -1;
	}

	LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

	if (lane_section == 0)
	{
		LOG("No lane section for idx %d - keeping current lane setting\n", lane_section_idx_);
		return -1;
	}

	double offset;
	int lane_idx = lane_section->GetClosestLaneIdx(s_, t_, offset);

	if (lane_idx == -1)
	{
		LOG("Failed to find a valid drivable lane");
		return -1;
	}

	lane_id_ = lane_section->GetLaneIdByIdx(lane_idx);
	offset_ = offset;

	return 0;
}

void Position::Track2Lane()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0)
	{
		LOG("Position::Track2Lane Error: No road %d\n", track_idx_);
		return;
	}

	Geometry *geometry = road->GetGeometry(geometry_idx_);
	if (geometry == 0)
	{
		LOG("Position::Track2Lane Error: No geometry %d\n", geometry_idx_);
		return;
	}

	// Find LaneSection according to s, starting from current
	int lane_section_idx = road->GetLaneSectionIdxByS(s_, lane_section_idx_);
	LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx);
	if (lane_section == 0)
	{
		LOG("No lane section for idx %d - keeping current lane setting", lane_section_idx_);
		return;
	}

	// Find the closest driving lane within the lane section
	double offset;
	int lane_idx = lane_section->GetClosestLaneIdx(s_, t_, offset);

	if (lane_idx == -1)
	{
		LOG("Failed find closest lane");
		return;
	}

	offset_ = offset;
	// Update cache indices
	lane_idx_ = lane_idx;
	lane_id_ = lane_section->GetLaneIdByIdx(lane_idx_);
	lane_section_idx_ = lane_section_idx;
}

double Position::GetDistToTrackGeom(double x3, double y3, double z3, double h, Road *road, Geometry *geom, bool &inside, double &sNorm)
{
	// Step 1: Approximate geometry with a line, and check distance roughly

	double x1, y1, h1;
	double x2, y2, h2;
	int side;
	double dist = 0;
	double dsMin = 0;
	double z = 0;
	double min_lane_dist;

	// Evaluate line endpoints to get a straight line in between
	geom->EvaluateDS(0, &x1, &y1, &h1);
	geom->EvaluateDS(geom->GetLength(), &x2, &y2, &h2);

	// Apply lane offset
	x1 += road->GetLaneOffset(0) * cos(h1 + M_PI_2);
	y1 += road->GetLaneOffset(geom->GetLength()) * sin(h1 + M_PI_2);
	x2 += road->GetLaneOffset(0) * cos(h2 + M_PI_2);
	y2 += road->GetLaneOffset(geom->GetLength()) * sin(h2 + M_PI_2);

	// Find vector from point perpendicular to line segment
	double x4, y4;
	ProjectPointOnVector2D(x3, y3, x1, y1, x2, y2, x4, y4);

	// Check whether the projected point is inside or outside line segment
	inside = PointInBetweenVectorEndpoints(x4, y4, x1, y1, x2, y2, sNorm);

	if (inside)
	{
		// Distance between given point and that point projected on the straight line
		dist = PointDistance2D(x3, y3, x4, y4);
	}
	else
	{
		// Distance is measured between point to closest endpoint of line
		double d1, d2;

		d1 = PointDistance2D(x3, y3, x1, y1);
		d2 = PointDistance2D(x3, y3, x2, y2);
		if (d1 < d2)
		{
			dist = d1;
			dsMin = 0;
			sNorm = 0;
		}
		else
		{
			dist = d2;
			dsMin = geom->GetLength();
			sNorm = 1;
		}
	}

	// Find out what side of the straight line the gigen point is located
	side = PointSideOfVec(x3, y3, x1, y1, x2, y2);

	// Now, calculate accurate distance from given point to road geometry - not to a straight line
	// But do this only for relevant geometries within reasonable short distance - 
	// else just use approximate distance value as calculated above

	if (dist < 50 + 0.5 * geom->GetLength())  // Extend search area for long geometries since they might bend from the straight line
	{
		// Step 2: Find s value within given geometry segment
		// If heading at start and end points of the geometry are practically equal let's approximate with a straight line
		if (GetAbsAngleDifference(h1, h2) < 0.01)
		{
			// If small heading difference, treat segment as a straight line 
			// Just use the normalized s value from calculation above, but clip at 0 and 1 to keep it inside geometry boundries
			dsMin = CLAMP(sNorm, 0.0, 1.0) * geom->GetLength();
		}
		else
		{
			// Strategy: 
			// 1. Find intersection of the two geometry endpoint tangents 
			// 2. Define a line segment from intersection to the point of query
			// 3. Find out angles between this line segment l1 and extended tangents t1 and t2
			// 4. s value corresponds to the angle between l1 and t1 divided by angle between t2 and t1

			double t1x1, t1y1, t1x2, t1y2, t2x1, t2y1, t2x2, t2y2, px, py, l1x, l1y;

			// Define tangent vectors from geometry start- and end point respectivelly 
			t1x1 = x1;
			t1y1 = y1;
			t1x2 = t1x1 + cos(h1 + M_PI_2);
			t1y2 = t1y1 + sin(h1 + M_PI_2);

			t2x1 = x2;
			t2y1 = y2;
			t2x2 = t2x1 + cos(h2 + M_PI_2);
			t2y2 = t2y1 + sin(h2 + M_PI_2);

			if (GetIntersectionOfTwoLineSegments(t1x1, t1y1, t1x2, t1y2, t2x1, t2y1, t2x2, t2y2, px, py) == 0)
			{
				l1x = px - x3;
				l1y = py - y3;

				// First normalize l1
				double l1_len = sqrt(l1x * l1x + l1y * l1y);
				double l1_norm_x = l1x / l1_len;
				double l1_norm_y = l1y / l1_len;

				// Redefine tangents in direction towards intersection point
				double t1x = px - x1;
				double t1y = py - y1;
				double t2x = px - x2;
				double t2y = py - y2;
				double t1_len = sqrt(t1x * t1x + t1y * t1y);
				double t1_norm_x = t1x / t1_len;
				double t1_norm_y = t1y / t1_len;
				double t2_len = sqrt(t2x * t2x + t2y * t2y);
				double t2_norm_x = t2x / t2_len;
				double t2_norm_y = t2y / t2_len;



				// Then run acos on dot products to find angles
				double angle1 = acos(GetDotProduct2D(l1_norm_x, l1_norm_y, t1_norm_x, t1_norm_y));
				double angle2 = acos(GetDotProduct2D(t2_norm_x, t2_norm_y, t1_norm_x, t1_norm_y));

				// Check whether point is inside fan shape that that extends perpendicular from endpoints along tangents
				double d1 = GetDotProduct2D(cos(h1), sin(h1), x3 - x1, y3 - y1);
				double d2 = GetDotProduct2D(cos(h2), sin(h2), x3 - x2, y3 - y2);

				if (d1 > 0 && d2 < 0)  // if angle is not within {-90:90} 
				{
					inside = true;
					sNorm = angle1 / angle2;
					dsMin = geom->GetLength() * sNorm;
				}
				else
				{
					inside = false;
				}
			}
			else
			{
				// Lines are close to parallell, meaning geometry is or close to a straight line - keep calculations from the approx method above
			}
		}
	
		double sMin = geom->GetS() + dsMin;
		double x, y;
		double pitch = 0;


		// Find out Z level
		road->GetZAndPitchByS(sMin, &z, &pitch, &elevation_idx_);

		// Step 3: Find exact position along road geometry at calculated s-value
		// and calculated distance from this point on road to given point 

		geom->EvaluateDS(dsMin, &x, &y, &h);
		// Apply lane offset
		x += road->GetLaneOffset(sMin) * cos(h + M_PI_2);
		y += road->GetLaneOffset(sMin) * sin(h + M_PI_2);
		dist = PointDistance2D(x3, y3, x, y);

		// Check whether the point is left or right side of road
		// x3, y3 is the point checked against a vector aligned with heading
		side = PointSideOfVec(x3, y3, x, y, x + cos(h), y + sin(h));

		// dist is now actually the lateral distance from reference lane, e.g. track coordinate t-value

		// Finally calculate exakt distance, but only for inside points
		LaneSection *lane_section = road->GetLaneSectionByS(sMin);
		min_lane_dist = std::numeric_limits<double>::infinity();
		if (lane_section != 0)
		{
			for (int i = 0; i < lane_section->GetNumberOfLanes(); i++)
			{
				if (lane_section->GetLaneByIdx(i)->IsDriving())
				{
					double lane_dist;
					{
						double signed_offset = dist * SIGN(side);
						double signed_lane_center_offset = SIGN(lane_section->GetLaneIdByIdx(i)) * lane_section->GetCenterOffset(sMin, lane_section->GetLaneIdByIdx(i));
						lane_dist = signed_offset - signed_lane_center_offset;
						if (fabs(lane_dist) < fabs(min_lane_dist))
						{
							min_lane_dist = lane_dist;
						}
					}
				}
			}
		}
	} 
	else
	{
		min_lane_dist = dist;
	}

	return fabs(min_lane_dist) + fabs(GetZ() - z);
}

void Position::XYZH2TrackPos(double x3, double y3, double z3, double h3, bool alignZPitchRoll)
{
	double dist;
	double distMin = std::numeric_limits<double>::infinity();
	double sNorm;
	double sNormMin;
	Geometry *geom, *current_geom;
	Geometry *geomMin = 0;
	Road *road, *current_road = 0;
	Road *roadMin = 0;
	bool inside = false;
	bool directlyConnected = false;
	bool directlyConnectedMin = false;
	double weight = 0; // Add some resistance to switch from current road, applying a stronger bound to current road
	double angle = 0;

	if (GetOpenDrive()->GetNumOfRoads() == 0)
	{
		return;
	}

	if ((current_road = GetOpenDrive()->GetRoadByIdx(track_idx_)) != 0)
	{
		if ((current_geom = current_road->GetGeometry(geometry_idx_)) == 0)
		{
			LOG("Invalid geometry index %d\n", geometry_idx_);
			return;
		}
	}

	x_ = x3;
	y_ = y3;

	for (int i = 0; i < GetOpenDrive()->GetNumOfRoads(); i++)
	{
		road = GetOpenDrive()->GetRoadByIdx(i);
		weight = 0;
		angle = 0;

		// Add resistance to leave current road or directly connected ones 
		// actual weights are totally unscientific... up to tuning
		if (current_road && (road == current_road || GetOpenDrive()->IsDirectlyConnected(current_road->GetId(), road->GetId(), angle)))
		{
			weight = angle;
			directlyConnected = true;
		}
		else
		{
			if (directlyConnectedMin) // if already found a directly connected position - add offset distance
			{
				weight = 3; 
			}

			weight += 5;  // For non connected roads add additional "penalty" threshold  
			directlyConnected = false;
		}
		
		for (int j = 0; j < road->GetNumberOfGeometries(); j++)
		{
			geom = road->GetGeometry(j);
			dist = GetDistToTrackGeom(x3, y3, z3, h3, road, geom, inside, sNorm);
			
			dist += weight + (inside ? 0 : 2);  // penalty for roads outside projection area

			if (dist < distMin)
			{
				geomMin = geom;
				directlyConnectedMin = directlyConnected;
				roadMin = road;
				sNormMin = CLAMP(sNorm, 0.0, 1.0);
				distMin = dist;
			}
		}
	}

	if (roadMin == 0)
	{
		LOG("Error finding minimum distance\n");
		return;
	}

	double dsMin = sNormMin * geomMin->GetLength();
	double sMin = geomMin->GetS() + dsMin;
	double x, y;

	// Found closest geometry. Now calculate exact distance to geometry. First find point perpendicular on geometry.
	geomMin->EvaluateDS(dsMin, &x, &y, &h_road_);
	
	// Apply lane offset
	x += roadMin->GetLaneOffset(dsMin) * cos(h_road_ + M_PI_2);
	y += roadMin->GetLaneOffset(dsMin) * sin(h_road_ + M_PI_2);
	distMin = PointDistance2D(x3, y3, x, y);

	// Check whether the point is left or right side of road
	// x3, y3 is the point checked against closest point on geometry
	int side = PointSideOfVec(x3, y3, x, y, x + cos(h_road_), y + sin(h_road_));

	// Set specified heading
	SetHeading(h3);

	// Find out what lane and set position
	SetTrackPos(roadMin->GetId(), sMin, distMin * side, false);

	EvaluateRoadZPitchRoll(alignZPitchRoll);
}
	

bool Position::EvaluateRoadZPitchRoll(bool alignZPitchRoll)
{
	bool ret_value = GetRoadById(track_id_)->GetZAndPitchByS(s_, &z_road_, &p_road_, &elevation_idx_);

	if (alignZPitchRoll)
	{
		z_ = z_road_;
		p_ = p_road_;
		r_ = 0;  // Road roll not implementade yet

		// Find out pitch of road in driving direction
		if (GetHRelative() > M_PI / 2 && GetHRelative() < 3 * M_PI / 2)
		{
			p_ *= -1;
			// r_ *= -1;
		}
	}

	return ret_value;
}

void Position::Track2XYZ()
{
	if (GetOpenDrive()->GetNumOfRoads() == 0)
	{
		return;
	}

	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0)
	{
		LOG("Position::Track2XYZ Error: No road %d\n", track_idx_);
		return;
	}

	Geometry *geometry = road->GetGeometry(geometry_idx_);
	if (geometry == 0)
	{
		LOG("Position::Track2XYZ Error: No geometry %d\n", geometry_idx_);
		return;
	}

	geometry->EvaluateDS(s_ - geometry->GetS(), &x_, &y_, &h_road_);
	
	// Consider lateral t position, perpendicular to track heading
	double x_local = (t_ + road->GetLaneOffset(s_)) * cos(h_road_ + M_PI_2);
	double y_local = (t_ + road->GetLaneOffset(s_)) * sin(h_road_ + M_PI_2);
	h_road_ += atan(road->GetLaneOffsetPrim(s_)) + h_offset_;
	h_ = h_road_ + h_relative_;  // Update heading, taking relative heading into account
	x_ += x_local;
	y_ += y_local;

	// z = Elevation 
	EvaluateRoadZPitchRoll(true);
}

void Position::Lane2Track()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	t_ = 0;

	if (road != 0 && road->GetNumberOfLaneSections() > 0)
	{
		LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		if (lane_section != 0)
		{
			t_ = offset_ + lane_section->GetCenterOffset(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
			h_offset_ = lane_section->GetCenterOffsetHeading(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
		}
	}
}

void Position::XYZ2Track(bool alignZAndPitch)
{
	XYZH2TrackPos(GetX(), GetY(), GetZ(), GetH(), alignZAndPitch);
}

int Position::SetLongitudinalTrackPos(int track_id, double s)
{
	Road *road;

	if (GetOpenDrive()->GetNumOfRoads() == 0)
	{
		return -1;
	}
	
	if ((road = GetOpenDrive()->GetRoadById(track_id)) == 0)
	{
		LOG("Position::Set Error: track %d not found\n", track_id);
		return -1;
	}
	if (track_id != track_id_)
	{
		// update internal track and geometry indices
		track_id_ = track_id;
		track_idx_ = GetOpenDrive()->GetTrackIdxById(track_id);
		geometry_idx_ = 0;
		elevation_idx_ = 0;
		lane_section_idx_ = 0;
		lane_id_ = 0;
		lane_idx_ = 0;
	}

	if (s > road->GetLength())
	{
		if (s > road->GetLength() + SMALL_NUMBER)
		{
			LOG("Position::Set Warning: s (%.2f) too large, track %d only %.2f m long\n", s, track_id_, road->GetLength());
		}
		s_ = road->GetLength();
	}
	else
	{
		s_ = s;
	}

	Geometry *geometry = road->GetGeometry(geometry_idx_);
	// check if still on same geometry
	if (s_ > geometry->GetS() + geometry->GetLength())
	{
		while (s_ > geometry->GetS() + geometry->GetLength() && geometry_idx_ < road->GetNumberOfGeometries() - 1)
		{
			// Move to next geometry
			geometry = road->GetGeometry(++geometry_idx_);
		}
	}
	else if (s_ < geometry->GetS())
	{
		while (s_ < geometry->GetS() && geometry_idx_ > 0)
		{
			// Move to previous geometry
			geometry = road->GetGeometry(--geometry_idx_);
		}
	}

	return 0;
}

void Position::SetTrackPos(int track_id, double s, double t, bool calculateXYZ)
{
	if (SetLongitudinalTrackPos(track_id, s) != 0)
	{
		return;
	}

	t_ = t;
	Track2Lane();
	if (calculateXYZ)
	{
		Track2XYZ();
	}
}

void Position::ForceLaneId(int lane_id)
{
	if (lane_idx_ < 0 || lane_section_idx_ < 0)
	{
		return;
	}
	// find out lateral distance between current and target lane
	Road *road = GetRoadById(GetTrackId());

	double lat_dist = road->GetLaneSectionByIdx(lane_section_idx_)->GetOffsetBetweenLanes(lane_id_, lane_id, GetS());

	lane_id_ = lane_id;
	offset_ -= lat_dist;
}

static std::string LinkType2Str(LinkType type)
{
	if (type == LinkType::PREDECESSOR)
	{
		return "PREDECESSOR";
	}
	else if (type == LinkType::SUCCESSOR)
	{
		return "SUCCESSOR";
	}
	else if (type == LinkType::NONE)
	{
		return "NONE";
	}
	else if (type == LinkType::UNKNOWN)
	{
		return "UNKNOWN";
	}
	else
	{
		return "UNDEFINED";
	}
}

std::string OpenDrive::ContactPointType2Str(ContactPointType type)
{
	if (type == ContactPointType::CONTACT_POINT_START)
	{
		return "PREDECESSOR";
	}
	else if (type == ContactPointType::CONTACT_POINT_END)
	{
		return "SUCCESSOR";
	}
	else if (type == ContactPointType::CONTACT_POINT_NONE)
	{
		return "NONE";
	}
	else if (type == ContactPointType::CONTACT_POINT_UNKNOWN)
	{
		return "UNKNOWN";
	}
	else
	{
		return "UNDEFINED";
	}
}

std::string OpenDrive::ElementType2Str(RoadLink::ElementType type)
{
	if (type == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		return "JUNCTION";
	}
	else if (type == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		return "ROAD";
	}
	else if (type == RoadLink::ElementType::ELEMENT_TYPE_UNKNOWN)
	{
		return "UNKNOWN";
	}
	else
	{
		return "UNDEFINED";
	}
}

int Position::MoveToConnectingRoad(RoadLink *road_link, ContactPointType &contact_point_type, Junction::JunctionStrategyType strategy)
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	Road *next_road = 0;
	LaneSection *lane_section;
	Lane *lane;
	int new_lane_id = 0;

	if (road == 0)
	{
		LOG("Invalid road id %d\n", road->GetId());
		return -1;
	}

	if (road_link->GetElementId() == -1)
	{
		LOG("No connecting road or junction at rid %d link_type %s", road->GetId(), LinkType2Str(road_link->GetType()).c_str());
		return -1;
	}
	
	// Get lane info from current road
	lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	if (lane_section == 0)
	{
		LOG("No lane section rid %d ls_idx %d link_type  %s", road->GetId(), lane_section_idx_, LinkType2Str(road_link->GetType()).c_str());
		return -1;
	}

	lane = lane_section->GetLaneByIdx(lane_idx_);
	if (lane == 0)
	{
		LOG("No lane rid %d lidx %d nlanes %d link_type %s lsecidx %d\n", 
			road->GetId(), lane_idx_, lane_section->GetNumberOfLanes(), LinkType2Str(road_link->GetType()).c_str(), lane_section_idx_);
		return -1;
	}
	
	if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
	{
		LaneLink *lane_link = lane->GetLink(road_link->GetType());
		if (lane_link != 0)
		{
			new_lane_id = lane->GetLink(road_link->GetType())->GetId();
			if (new_lane_id == 0)
			{
				LOG("Road+ new lane id %d\n", new_lane_id);
			}
		}
		else
		{
			//LOG("No lane link from rid %d lid %d to rid %d", GetTrackId(), GetLaneId(), road_link->GetElementId());
		}
		contact_point_type = road_link->GetContactPointType();
		next_road = GetOpenDrive()->GetRoadById(road_link->GetElementId());
	}
	else if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
	{
		Junction *junction = GetOpenDrive()->GetJunctionById(road_link->GetElementId());

		if (junction == 0)
		{
			LOG("Error: junction %d not existing\n", road_link->GetElementType());
			return -1;
		}
	
		int connection_idx = 0;
		int n_connections = junction->GetNumberOfRoadConnections(road->GetId(), lane->GetId());

		if (n_connections == 0)
		{
//			LOG("No connections from road id %d lane id %d in junction %d", road->GetId(), lane->GetId(), junction->GetId());
			return -1;
		}
		else if (n_connections == 1)
		{
			connection_idx = 0;
		}
		else
		{
			// find valid connecting road, if multiple choices choose either most straight one OR by random
			if (strategy == Junction::JunctionStrategyType::STRAIGHT)
			{
				// Find the straighest link
				int best_road_index = 0;
				double min_heading_diff = 1E10; // set huge number
				for (int i = 0; i < n_connections; i++)
				{
					LaneRoadLaneConnection lane_road_lane_connection = 
						junction->GetRoadConnectionByIdx(road->GetId(), lane->GetId(), i);
					next_road = GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId()); 

					Position test_pos;
					if (lane_road_lane_connection.contact_point_ == CONTACT_POINT_START)
					{
						// Inspect heading at the connecting road
						test_pos.SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), 0);
					}
					else if (lane_road_lane_connection.contact_point_ == CONTACT_POINT_END)
					{
						test_pos.SetLanePos(next_road->GetId(), new_lane_id, 0, 0);
					}
					else
					{
						LOG("Unexpected contact point type: %d", road_link->GetContactPointType());
					}

					// Transform angle into a comparable format

					double heading_diff = GetAbsAngleDifference(test_pos.GetHRoad(), GetHRoad());
					if (heading_diff > M_PI / 2)
					{
						heading_diff = fabs(heading_diff - M_PI);  // don't care of driving direction here
					}

					if (heading_diff < min_heading_diff)
					{
						min_heading_diff = heading_diff;
						best_road_index = i;
					}
				}
				connection_idx = best_road_index;
			}
			else if (strategy == Junction::JunctionStrategyType::RANDOM)
			{
				connection_idx = (int)(n_connections * (double)mt_rand() / mt_rand.max());
			}
		}

		LaneRoadLaneConnection lane_road_lane_connection = junction->GetRoadConnectionByIdx(road->GetId(), lane->GetId(), connection_idx);
		contact_point_type = lane_road_lane_connection.contact_point_;

		new_lane_id = lane_road_lane_connection.GetConnectinglaneId();
		next_road = GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId());
	}

	if (next_road == 0)
	{
		LOG("No next road\n");
		return -1;
	}

	if (new_lane_id == 0)
	{
		LOG("No connection from rid %d lid %d -> rid %d eltype %d - try moving to closest lane\n", 
			road->GetId(), lane->GetId(), road_link->GetElementId(), road_link->GetElementType());

		// Find closest lane on new road - by convert to track pos and then set lane offset = 0
		if (road_link->GetContactPointType() == CONTACT_POINT_START)
		{
			SetTrackPos(next_road->GetId(), 0, GetT(), false);
		}
		else if (road_link->GetContactPointType() == CONTACT_POINT_END)
		{
			SetTrackPos(next_road->GetId(), next_road->GetLength(), GetT(), false);
		}
		offset_ = 0;

		return 0;
	}

	double new_offset = offset_;
	// EG: Suggestion, positiv offset_ should always move towards the reference line
	// it simplifies lane change action since we dont have to consider if we change track during the action
	// but then we need to consider this when we convert between offset_ and t_ etc.
	// now I do this to keep same world position after we go to different track, I think... :)
	if (SIGN(lane_id_) != SIGN(new_lane_id))
	{
		new_offset *= -1;
	}

	// Find out if connecting to start or end of new road
	if (road_link->GetContactPointType() == CONTACT_POINT_START)
	{
		// Specify first (0) lane section
		SetLanePos(next_road->GetId(), new_lane_id, 0, new_offset, 0);
	}
	else if (road_link->GetContactPointType() == CONTACT_POINT_END)
	{
		// Find out and specify last lane section
		SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), new_offset, GetRoadById(road_link->GetElementId())->GetNumberOfLaneSections()-1);
	}
	else if (road_link->GetContactPointType() == CONTACT_POINT_NONE && road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
	{
		if (contact_point_type == CONTACT_POINT_START)
		{
			SetLanePos(next_road->GetId(), new_lane_id, 0, new_offset);
		}
		else if (contact_point_type == CONTACT_POINT_END)
		{
			SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), new_offset);
		}
		else
		{
			LOG("Unexpected contact point: %d\n", contact_point_type);
		}
	}
	else
	{
		LOG("Unsupported contact point type %d\n", road_link->GetContactPointType());
		return -1;
	}

	return 0;
}

int Position::MoveAlongS(double ds, double dLaneOffset, Junction::JunctionStrategyType strategy)
{
	RoadLink *link;
	double ds_signed = ds;
	int max_links = 8;  // limit lookahead through junctions/links 
	ContactPointType contact_point_type;

	if (GetOpenDrive()->GetNumOfRoads() == 0 || track_idx_ < 0)
	{
		// No roads available or current track undefined
		return 0;
	}

	// EG: If offset_ is not along reference line, but instead along lane direction then we dont need
	// the SIGN() adjustment. But for now this adjustment means that a positive dLaneOffset always moves left?
	offset_ += dLaneOffset * -SIGN(GetLaneId());
	double s_stop = 0;
	
	for (int i = 0; i < max_links; i++)
	{
		ds_signed = -SIGN(GetLaneId()) * ds; // adjust sign of ds according to lane direction - right lane is < 0 in road dir

		if (s_ + ds_signed > GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength())
		{
			ds_signed = s_ + ds_signed - GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength();
			link = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLink(SUCCESSOR);
			s_stop = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength();
		}
		else if (s_ + ds_signed < 0)
		{
			ds_signed = s_ + ds_signed;
			link = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLink(PREDECESSOR);
			s_stop = 0;
		}
		else  // New position is within current track
		{
			break;
		}

		if (!link || link->GetElementId() == -1 || MoveToConnectingRoad(link, contact_point_type, strategy) != 0)
		{
			// Failed to find a connection, stay at end of current road
			SetLanePos(track_id_, lane_id_, s_stop, offset_);

			// Return special code
			return 1;
		}

		ds = SIGN(ds) * fabs(ds_signed);
	}

	SetLanePos(track_id_, lane_id_, s_ + ds_signed, offset_);
	return 0;
}

void Position::SetLanePos(int track_id, int lane_id, double s, double offset, int lane_section_idx)
{
	offset_ = offset;
	int old_lane_id = lane_id_;
	int old_track_id = track_id_;

	if (SetLongitudinalTrackPos(track_id, s) != 0)
	{
		return;
	}

	Road *road = GetOpenDrive()->GetRoadById(track_id);
	if (road == 0)
	{
		LOG("Position::Set Error: track %d not available\n", track_id);
		return;
	}

	if (lane_id != lane_id_ && lane_section_idx == -1)
	{
		// New lane ID might indicate a discreet jump to a new, distant position, reset lane section, if not specified in func parameter)
		lane_section_idx = road->GetLaneSectionIdxByS(s);
	}

	LaneSection *lane_section = 0;
	if (lane_section_idx > -1)  // If lane section was specified or reset
	{
		lane_section_idx_ = lane_section_idx;
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		lane_id_ = lane_id;
	}
	else  // Find LaneSection and info according to s
	{
		LaneInfo lane_info = road->GetLaneInfoByS(s_, lane_section_idx_, lane_id_);
		lane_section_idx_ = lane_info.lane_section_idx_;
		lane_id_ = lane_info.lane_id_;
		
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	}

	if (lane_section != 0)
	{
		lane_idx_ = lane_section->GetLaneIdxById(lane_id_);
		if (lane_idx_ == -1)
		{
			LOG("lane_idx %d fail for lane id %d\n", lane_idx_, lane_id_);
			lane_idx_ = 0;
		}
	}
	else
	{
		LOG("Position::Set (lanepos) Error - lanesection NULL lsidx %d rid %d lid %d\n",
			lane_section_idx_, road->GetId(), lane_id_);
	}

	// Check road direction when on new track 
	if (old_lane_id != 0 && lane_id_ != 0 && track_id_ != old_track_id && SIGN(lane_id_) != SIGN(old_lane_id))
	{
		h_relative_ = GetAngleSum(h_relative_, M_PI);
	}

	// If moved over to opposite driving direction, then turn relative heading 180 degrees
	//if (old_lane_id != 0 && lane_id_ != 0 && SIGN(lane_id_) != SIGN(old_lane_id))
	//{
	//	h_relative_ = GetAngleSum(h_relative_, M_PI);
	//}

	Lane2Track();
	Track2XYZ();
}

void Position::SetInertiaPos(double x, double y, double z, double h, double p, double r, bool updateTrackPos)
{
	x_ = x;
	y_ = y;
	z_ = z;
	SetHeading(h);
	p_ = p;
	r_ = r;

	if (updateTrackPos)
	{
		XYZ2Track();
	}
}

void Position::SetHeading(double heading)
{
	h_ = heading;
	h_relative_ = GetAngleDifference(h_, h_road_);
}

void Position::SetHeadingRelative(double heading)
{
	h_relative_ = fmod(heading, 2 * M_PI);
	if (h_relative_ < 0)
	{
		h_relative_ += 2 * M_PI;
	}
	h_ = GetAngleSum(h_road_, h_relative_);
}

void Position::SetHeadingRelativeRoadDirection(double heading)
{
	if (h_relative_ > M_PI/2 && h_relative_ < 3 * M_PI / 2)
	{
		// Driving towards road direction
		h_relative_ = fmod(-heading + M_PI, 2 * M_PI);
		//LOG("Driving towards road direction h_ %.2f h_relative_ %.2f heading %.2f ", h_, h_relative_, heading);
	}
	else
	{
		h_relative_ = fmod(heading, 2 * M_PI);
		//LOG("Driving along road direction h_ %.2f h_relative_ %.2f heading %.2f ", h_, h_relative_, heading);
	}
	h_ = GetAngleSum(h_road_, h_relative_);
}

double Position::GetCurvature()
{
	Geometry *geom = GetOpenDrive()->GetGeometryByIdx(track_idx_, geometry_idx_);
	
	if (geom)
	{
		return geom->EvaluateCurvatureDS(GetS() - geom->GetS());
	}
	else
	{
		return 0;
	}
}

double Position::GetHRoadInDrivingDirection()
{
	return h_road_ + (lane_id_ > 0 ? M_PI : 0);
}

double Position::GetSpeedLimit()
{
	double speed_limit = 70 / 3.6;  // some default speed
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	
	if (road)
	{
		speed_limit = road->GetSpeedByS(s_);

		if (speed_limit < SMALL_NUMBER)
		{
			// No speed limit defined, set a value depending on number of lanes
			speed_limit = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetNumberOfDrivingLanesSide(GetS(), SIGN(GetLaneId())) > 1 ? 120 / 3.6 : 60 / 3.6;
		}
	}

	return speed_limit;
}

double Position::GetDrivingDirection()
{
	double x, y, h;
	Geometry *geom = GetOpenDrive()->GetGeometryByIdx(track_idx_, geometry_idx_);

	if (!geom)
	{
		return h_;
	}

	geom->EvaluateDS(GetS() - geom->GetS(), &x, &y, &h);

	// adjust 180 degree according to side of road
	if (GetLaneId() > 0)  // Left side of road reference line
	{
		h = GetAngleSum(h, M_PI);
	}

	return(h);
}

void Position::CopyRMPos(Position *from)
{
	// Preserve route field
	Route *tmp = route_;
	
	*this = *from;
	route_ = tmp;
}


void Position::PrintTrackPos()
{
	LOG("	Track pos: (road_id %d, s %.2f, t %.2f, h %.2f)", track_id_, s_, t_, h_);
}

void Position::PrintLanePos()
{
	LOG("	Lane pos: (road_id %d, lane_id %d, s %.2f, offset %.2f, h %.2f)", track_id_, lane_id_, s_, offset_, h_);
}

void Position::PrintInertialPos()
{
	LOG("	Inertial pos: (x %.2f, y %.2f, z %.2f, h %.2f, p %.2f, r %.2f)", x_, y_, z_, h_, p_, r_);
}

void Position::Print()
{
	LOG("Position:");
	PrintTrackPos();
	PrintLanePos();
	PrintInertialPos();
}

void Position::PrintXY()
{
	LOG("%.2f, %.2f\n", x_, y_);
}

double Position::getRelativeDistance(Position target_position, double &x, double &y)
{
	// Calculate diff vector from current to target
	double diff_x, diff_y;

	diff_x = target_position.GetX() - GetX();
	diff_y = target_position.GetY() - GetY();

	// Compensate for current heading (rotate so that current heading = 0)
	x = diff_x * cos(-GetH()) - diff_y * sin(-GetH());
	y = diff_x * sin(-GetH()) + diff_y * cos(-GetH());

	// Now just check whether diff vector X-component is less than 0 (behind current)
	int sign = x > 0 ? 1 : -1;

	// Return length of dist vector
	return sign * sqrt((x * x) + (y * y));
}

double Position::FindDistToPos(Position *pos, RoadLink *link, int &call_count, int level_count, bool &found)
{
	double tmp_dist = 0;
	Road *road = GetOpenDrive()->GetRoadById(GetTrackId());

	if (!link)
	{
		return 0;
	}

	if (++call_count > 20 || level_count > 3)
	{
		return 0;
	}

	if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		Road *next_road = GetOpenDrive()->GetRoadById(link->GetElementId());
		if (link->GetElementId() == pos->GetTrackId())
		{
			if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_START)
			{
				found = true;
				return pos->GetS();
			}
			else if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_END)
			{
				found = true;
				return next_road->GetLength() - pos->GetS();
			}
			else
			{
				LOG("Unexpected link contact point type: %d", link->GetContactPointType());
				return 0;
			}
		}
		else
		{
			RoadLink *next_link = 0;

			if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_START)
			{
				next_link = next_road->GetLink(LinkType::SUCCESSOR);
			}
			else if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_END)
			{
				next_link = next_road->GetLink(LinkType::PREDECESSOR);
			}

			if(next_link == 0)
			{
				// no link 
				return 0;
			}

			tmp_dist = FindDistToPos(pos, next_link, call_count, level_count, found);
			if (found)
			{
				return next_road->GetLength() + tmp_dist;
			}
			else
			{
				return 0;
			}
		}
	}
	else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		Junction *junction = GetOpenDrive()->GetJunctionById(link->GetElementId());
		
		for (size_t i = 0; i < junction->GetNoConnectionsFromRoadId(road->GetId()); i++)
		{
			Road *next_road = GetOpenDrive()->GetRoadById(junction->GetConnectingRoadIdFromIncomingRoadId(road->GetId(), (int)i));
			if (next_road == 0)
			{
				LOG("Failed look up connecting road");
				return 0;
			}

			if (next_road->GetId() == pos->GetTrackId())
			{
				if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_START)
				{
					found = true;
					return pos->GetS();
				}
				else if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_END)
				{
					found = true;
					return junction->GetConnectionByIdx((int)i)->GetConnectingRoad()->GetLength() - pos->GetS();;
				}
				else
				{
					LOG("Unexpected contact point type: %d", link->GetContactPointType());
				}
			}
			else
			{
				if (next_road->GetLink(LinkType::SUCCESSOR)->GetElementId() == road->GetId())
				{
					link = next_road->GetLink(LinkType::PREDECESSOR);
				}
				else if (next_road->GetLink(LinkType::PREDECESSOR)->GetElementId() == road->GetId())
				{
					link = next_road->GetLink(LinkType::SUCCESSOR);
				}
				else
				{
					// connecting road no no connection to road
					continue;
				}

				tmp_dist = FindDistToPos(pos, link, call_count, level_count+1, found);
				if(found)
				{
					return next_road->GetLength() + tmp_dist;
				}
			}
		}
	}
	else
	{
		LOG("Unexpected link element type: %d", link->GetElementType());
		return 0;
	}

	return 0;
}

void Position::CalcRoutePosition()
{
	if (route_ == 0)
	{
		return;
	}

	// Loop over waypoints - look for current track ID and sum the distance (route s) up to current position
	double dist = 0;
	for (size_t i = 0; i < route_->waypoint_.size(); i++)
	{
		int direction = route_->GetWayPointDirection((int)i);
		if (direction == 0)
		{
			LOG("Unexpected lack of connection in route at waypoint %d", i);
			return;
		}

		if (i == 0)
		{
			// Subtract initial s-value for the first waypoint
			
			if (direction > 0)  // route in waypoint road direction
			{
				dist = GetRoadById(route_->waypoint_[i]->GetTrackId())->GetLength() - route_->waypoint_[i]->s_;
			}
			else
			{
				// going towards road direction - remaining distance equals route s-value
				dist = route_->waypoint_[i]->s_;
			}
		}
		else
		{
			// Add length of intermediate waypoint road
			dist += GetRoadById(route_->waypoint_[i]->GetTrackId())->GetLength();
		}

		if (GetTrackId() == route_->waypoint_[i]->GetTrackId())
		{
			// current position is at the road of this waypoint - i.e. along the route
			// remove remaming s from road
			if (direction > 0)
			{
				dist -= GetRoadById(route_->waypoint_[i]->GetTrackId())->GetLength() - GetS();
			}
			else
			{
				dist -= GetS();
			}
			s_route_ = dist;
			break;
		}
	}
}

void Position::SetRoute(Route *route)
{
	route_ = route; 

	// Also find out current position in terms of route position
	CalcRoutePosition();
}

bool Position::Delta(Position pos_b, double &ds, double &dt, int &dLaneId)
{
	Road *road = GetOpenDrive()->GetRoadById(GetTrackId());
	RoadLink *link = 0;
	double dist = 0;
	bool found = false;

	if (pos_b.GetTrackId() != GetTrackId())
	{
		// Look along road segments and through junctions
		int call_count = 0;
		int level_count = 0;

		// First look forward
		link = road->GetLink(roadmanager::LinkType::SUCCESSOR);
		ds = road->GetLength() - GetS();
		dist = FindDistToPos(&pos_b, link, call_count, level_count, found);

		if (found)
		{
			ds += dist;
		}
		else
		{
			// Search backwards
			link = road->GetLink(roadmanager::LinkType::PREDECESSOR);
			ds = -GetS();
			dist = FindDistToPos(&pos_b, link, call_count, level_count, found);

			if (found)
			{
				ds -= dist;
			}
		}		
	}
	else
	{
		found = true;

		ds = pos_b.GetS() - GetS();

		link = road->GetLink(roadmanager::LinkType::SUCCESSOR);

		// Check whether road is looping
		if (link && link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
		{
			if (link->GetElementId() == GetTrackId())  // Loop
			{
				if (ds > road->GetLength() / 2)
				{
					ds -= road->GetLength();
				}
				else if (ds < -road->GetLength() / 2)
				{
					ds += road->GetLength();
				}
			}
		}
	}

	if (GetHRelative() > M_PI_2 && GetHRelative() < 3 * M_PI / 2)
	{
		ds *= -1;
	}

	dLaneId = pos_b.GetLaneId() - GetLaneId();
	dt = pos_b.GetT() - GetT();

	return found;
}

bool Position::IsAheadOf(Position target_position)
{
	// Calculate diff vector from current to target
	double diff_x, diff_y;
	double diff_x0;

	diff_x = target_position.GetX() - GetX();
	diff_y = target_position.GetY() - GetY();

	// Compensate for current heading (rotate so that current heading = 0) 
	// Only x component needed
	diff_x0 = diff_x * cos(-GetH()) - diff_y * sin(-GetH());

	// Now just check whether diff vector X-component is less than 0 (behind current)
	return(diff_x0 < 0);
}

int Position::GetRoadLaneInfo(double lookahead_distance, RoadLaneInfo *data, LookAheadMode lookAheadMode)
{
	Position target(*this);  // Make a copy of current position

	if (lookAheadMode == LOOKAHEADMODE_AT_ROAD_CENTER)
	{
		// Look along reference lane requested, move pivot position to t=0 plus a small number in order to 
		// fall into the right direction
		target.SetTrackPos(target.GetTrackId(), target.GetS(), SMALL_NUMBER * SIGN(GetLaneId()), true);
	}
	else if (lookAheadMode == LOOKAHEADMODE_AT_LANE_CENTER)
	{
		// Look along current lane center requested, move pivot position accordingly 
		target.SetLanePos(target.GetTrackId(), target.GetLaneId(), target.GetS(), 0);
	}

	if (fabs(lookahead_distance) > SMALL_NUMBER && target.MoveAlongS(lookahead_distance, 0, Junction::STRAIGHT) != 0)
	{
		return -1;
	}
	
	if (fabs(target.GetCurvature()) > SMALL_NUMBER)
	{
		double radius = 1.0 / target.GetCurvature();
		radius -= target.GetT(); // curvature positive in left curves, lat_offset positive left of reference lane
		data->curvature = (1.0 / radius);
	}
	else
	{
		// curvature close to zero (straight segment), radius infitite - curvature the same in all lanes
		data->curvature = target.GetCurvature();
	}

	data->pos[0] = target.GetX();
	data->pos[1] = target.GetY();
	data->pos[2] = target.GetZ();
	data->heading = target.GetHRoad();
	data->pitch = target.GetP();
	data->roll = target.GetR();

	// Then find out the width of the lane at current s-value
	Road *road = target.GetRoadById(target.GetTrackId());
	data->width = road->GetLaneWidthByS(target.GetS(), target.GetLaneId());
	data->speed_limit = road->GetSpeedByS(target.GetS());

	return 0;
}

void Position::CalcSteeringTarget(Position *target, SteeringTargetInfo *data)
{
	data->global_pos[0] = target->GetX();
	data->global_pos[1] = target->GetY();
	data->global_pos[2] = target->GetZRoad();

	// find out local x, y, z
	double diff_x = target->GetX() - GetX();
	double diff_y = target->GetY() - GetY();
	double diff_z = target->GetZRoad() - GetZRoad();

	data->local_pos[0] = diff_x * cos(-GetH()) - diff_y * sin(-GetH());
	data->local_pos[1] = diff_x * sin(-GetH()) + diff_y * cos(-GetH());
	data->local_pos[2] = diff_z;

#if 0
	// for validation
	data->global_pos[0] = GetX() + data->local_pos[0] * cos(GetH()) - data->local_pos[1] * sin(GetH());
	data->global_pos[1] = GetY() + data->local_pos[0] * sin(GetH()) + data->local_pos[1] * cos(GetH());
	data->global_pos[2] = GetZ() + data->local_pos[2];
#endif

	// Calculate angle - by dot product
	if (fabs(data->local_pos[0]) < SMALL_NUMBER && fabs(data->local_pos[1]) < SMALL_NUMBER && fabs(data->local_pos[2]) < SMALL_NUMBER)
	{
		data->angle = GetH();
	}
	else
	{
		double dot_prod =
			(data->local_pos[0] * 1.0 + data->local_pos[1] * 0.0) /
			sqrt(data->local_pos[0] * data->local_pos[0] + data->local_pos[1] * data->local_pos[1]);
		data->angle = SIGN(data->local_pos[1]) * acos(dot_prod);
	}

	if (fabs(target->GetCurvature()) > SMALL_NUMBER)
	{
		double radius = 1.0 / target->GetCurvature();
		radius -= target->GetT(); // curvature positive in left curves, lat_offset positive left of reference lane
		data->curvature = (float)(1.0 / radius);
	}
	else
	{
		// curvature close to zero (straight segment), radius infitite - curvature the same in all lanes
		data->curvature = (float)target->GetCurvature();
	}

	data->road_heading = target->GetHRoad();
	data->road_pitch = target->GetPRoad();
	data->road_roll = target->GetR();

	Road *road = target->GetRoadById(target->GetTrackId());
	if (road)
	{
		data->speed_limit = road->GetSpeedByS(target->GetS());
	}
}

int Position::GetSteeringTargetInfo(double lookahead_distance, SteeringTargetInfo *data, LookAheadMode lookAheadMode)
{
	if (GetOpenDrive()->GetNumOfRoads() == 0)
	{
		return -1;
	}
	Position target(*this);  // Make a copy of current position

	if (lookAheadMode == LOOKAHEADMODE_AT_ROAD_CENTER)
	{
		// Look along reference lane requested, move pivot position to t=0 plus a small number in order to 
		// fall into the right direction
		target.SetTrackPos(target.GetTrackId(), target.GetS(), SMALL_NUMBER * SIGN(GetLaneId()), true);
	}
	else if (lookAheadMode == LOOKAHEADMODE_AT_LANE_CENTER)
	{
		// Look along current lane center requested, move pivot position accordingly 
		target.SetLanePos(target.GetTrackId(), target.GetLaneId(), target.GetS(), 0);
	}

	if (fabs(lookahead_distance) > SMALL_NUMBER && target.MoveAlongS(lookahead_distance, 0, Junction::STRAIGHT) != 0)
	{
		return -1;
	}

	CalcSteeringTarget(&target, data);

	return 0;
}

int Position::GetSteeringTargetInfo(Position *target_pos, SteeringTargetInfo *data)
{
	CalcSteeringTarget(target_pos, data);

	return 0;
}

int Position::SetRoutePosition(Position *position)
{
	if(!route_)
	{
		return -1;
	}

	// Is it a valid position, i.e. is it along the route
	for (size_t i=0; i<route_->waypoint_.size(); i++)
	{
		if (route_->waypoint_[i]->GetTrackId() == position->GetTrackId()) // Same road
		{
			// Update current position 
			Route *tmp = route_;  // save route pointer, copy the 
			*this = *position;
			route_ = tmp;
			return 0;
		}
	}

	return -1;
}

int Position::MoveRouteDS(double ds)
{
	if (!route_)
	{
		return -1;
	}

	if (route_->waypoint_.size() == 0)
	{
		return -1;
	}
	SetRouteS(route_, s_route_ + ds);

	return 0;
}

int Position::SetRouteLanePosition(Route *route, double route_s, int laneId, double  laneOffset)
{
	SetRouteS(route, route_s);

	// Override lane data
	SetLanePos(track_id_, laneId, s_, laneOffset);

	return 0;
}

int Position::SetRouteS(Route *route, double route_s)
{
	if (route->waypoint_.size() == 0)
	{
		LOG("SetOffset No waypoints!");
		return -1;
	}

	OpenDrive *od = route->waypoint_[0]->GetOpenDrive();

	double initial_s_offset = 0;

	if (route->GetWayPointDirection(0) > 0)
	{
		initial_s_offset = route->waypoint_[0]->GetS();
	}
	else
	{
		initial_s_offset = od->GetRoadById(route->waypoint_[0]->GetTrackId())->GetLength() - route->waypoint_[0]->GetS();
	}

	double route_length = 0;
	s_route_ = route_s;

	// Find out what road and local s value
	for (size_t i = 0; i < route->waypoint_.size(); i++)
	{
		int track_id = route->waypoint_[i]->GetTrackId();
		double road_length = route->waypoint_[i]->GetOpenDrive()->GetRoadById(track_id)->GetLength();
		if (s_route_ < route_length + road_length - initial_s_offset)
		{
			// Found road segment
			double local_s = 0;

			int route_direction = route->GetWayPointDirection((int)i);

			if (route_direction == 0)
			{
				LOG("Unexpected lack of connection within route at waypoint %d", i);
				return -1;
			}

			local_s = s_route_ - route_length + initial_s_offset;
			if (route_direction < 0)  // along waypoint road direction
			{
				local_s = road_length - local_s;
			}

			SetLanePos(route->waypoint_[i]->GetTrackId(), route->waypoint_[i]->GetLaneId(), local_s, GetOffset());
			return 0;
		}
		route_length += road_length - initial_s_offset;
		initial_s_offset = 0;  // For all following road segments, calculate length from beginning
	}

	return -1;
}

int Route::AddWaypoint(Position *position)
{
	// Validate waypoint: Is it on the previous road or a connecting one?
	if (waypoint_.size() > 0)
	{
		bool connected = false;
		Position *prev_pos = waypoint_.back();

		int connecting_road_id = 0;
		int *connecting_road_id_ptr = &connecting_road_id;
		int connecting_lane_id = 0;
		int *connecting_lane_id_ptr = &connecting_lane_id;

		if (prev_pos->GetTrackId() != position->GetTrackId())
		{
			if (position->GetOpenDrive()->IsIndirectlyConnected(prev_pos->GetTrackId(), position->GetTrackId(), 
				connecting_road_id_ptr, connecting_lane_id_ptr, prev_pos->GetLaneId(), position->GetLaneId()))
			{
				connected = true;
				if (connecting_road_id != 0)
				{
					// Adding waypoint for junction connecting road
					Position *connected_pos = new Position(connecting_road_id, connecting_lane_id, 0, 0);
					waypoint_.push_back(connected_pos);
					LOG("Route::AddWaypoint Added connecting waypoint %d: %d, %d, %.2f\n",
						(int)waypoint_.size() - 1, connecting_road_id, connecting_lane_id, 0.0);
				}
			}
		}

		if (!connected)
		{
			LOG("Error: waypoint (%d, %d) is not connected to the previous one (%d, %d)\n",
				position->GetTrackId(), position->GetLaneId(), prev_pos->GetTrackId(), prev_pos->GetLaneId());

			return -1;
		}
	}

	waypoint_.push_back(position);
	LOG("Route::AddWaypoint Added waypoint %d: %d, %d, %.2f\n", (int)waypoint_.size()-1, position->GetTrackId(), position->GetLaneId(), position->GetS());

	return 0;
}

int Route::GetWayPointDirection(int index)
{
	if (waypoint_.size() == 0 || index < 0 || index >= waypoint_.size())
	{
		LOG("Waypoint index %d out of range (%d)", index, waypoint_.size());
		return 0;
	}

	if (waypoint_.size() == 1)
	{
		LOG("Only one waypoint, no direction");
		return 0;
	}

	OpenDrive *od = waypoint_[index]->GetOpenDrive();
	Road *road = od->GetRoadById(waypoint_[index]->GetTrackId());
	int connected = 0;
	double angle;

	if (index == waypoint_.size() - 1)
	{
		// Find connection point to previous waypoint road
		Road *prev_road = od->GetRoadById(waypoint_[index-1]->GetTrackId());

		connected = -od->IsDirectlyConnected(road->GetId(), prev_road->GetId(), angle);
	} 
	else
	{
		// Find connection point to next waypoint road
		Road *next_road = od->GetRoadById(waypoint_[index+1]->GetTrackId());
		
		connected = od->IsDirectlyConnected(road->GetId(), next_road->GetId(), angle);
	}
	
	return connected;
}

double Route::GetLength()
{

	return 0.0;
}

void Route::setName(std::string name)
{
	this->name = name;
}

std::string Route::getName()
{
	return name;
}
