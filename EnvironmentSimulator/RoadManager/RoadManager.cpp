	
#include <iostream>
#include <cstring>
#include <random>
#include <time.h>
#include <limits>

#define _USE_MATH_DEFINES
#include <math.h>

#include "RoadManager.hpp"
#include "odrSpiral.h"
#include "pugixml.hpp"
#include "CommonMini.hpp"

static std::mt19937 mt_rand;

using namespace std;
using namespace roadmanager;

#define CURV_ZERO 0.00001
#define SIGN(x) (x < 0 ? -1 : 1)
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

	*h += GetHdg() + GetH0();

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
	if (lane_section_.size() > 0)
	{
		return lane_section_[idx];
	}
	else
	{
		return 0;
	}
}

LaneSection* Road::GetLaneSectionByS(double s)
{
	double length = 0;

	if (lane_section_.size() == 0)
	{
		return 0;
	}

	for (size_t i = 0; i < lane_section_.size(); i++)
	{
		length = length + lane_section_[i]->GetLength();

		if (s < length)
		{
			return lane_section_[i];
		}
	}

	// s outside segment - return last lane section
	return (lane_section_.back());
}

LaneInfo Road::GetLaneInfoByS(double s, int start_lane_section_idx, int start_lane_id)
{
	LaneInfo lane_info;
	
	lane_info.lane_section_idx_ = start_lane_section_idx;
	lane_info.lane_id_ = start_lane_id;

	if (lane_info.lane_section_idx_ >= (int)lane_section_.size())
	{
		LOG("Road::GetLaneSectionByS: Error idx %d > n_lane_sections %d\n", lane_info.lane_section_idx_, (int)lane_section_.size());
	}
	else
	{
		LaneSection *lane_section = lane_section_[lane_info.lane_section_idx_];

		// check if we passed current section
		if (s > lane_section->GetS() + lane_section->GetLength())
		{
			//LOG("look forward for lane section at %d / %.2f lid %d (sec end s: %.2f)\n", 
			//	lane_info.lane_section_idx_, s, lane_info.lane_id_, lane_section->GetS() + lane_section->GetLength());
			while (s > lane_section->GetS() + lane_section->GetLength() && lane_info.lane_section_idx_ + 1 < GetNumberOfLaneSections())
			{
				// Find out connecting lane, then move to next lane section
				lane_info.lane_id_ = lane_section->GetConnectingLaneId(lane_info.lane_id_, SUCCESSOR);
				lane_section = GetLaneSectionByIdx(++lane_info.lane_section_idx_);
				//LOG("moved forward to lane section id %d @s %.2f laneid: %d\n", lane_info.lane_section_idx_, s, lane_info.lane_id_);
			}
		}
		else if (s < lane_section->GetS())
		{
			//LOG("look backward for lane section at %d / %.2f lid %d (sec end s: %.2f)\n",
			//	lane_info.lane_section_idx_, s, lane_info.lane_id_, lane_section->GetS() + lane_section->GetLength());
			while (s < lane_section->GetS() && lane_info.lane_section_idx_ > 0)
			{
				// Move to previous lane section
				lane_info.lane_id_ = lane_section->GetConnectingLaneId(lane_info.lane_id_, PREDECESSOR);
				lane_section = GetLaneSectionByIdx(--lane_info.lane_section_idx_);
				//LOG("moved backward to lane section id %d @s %.2f laneid: %d\n", lane_info.lane_section_idx_, s, lane_info.lane_id_);
			}
		}
	}

	return lane_info;
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

double LaneSection::GetOuterOffset(double s, int lane_id)
{
	if (lane_id == 0)
	{
		return 0.0;  // reference lane has no width
	}
	
	Lane *lane = GetLaneById(lane_id);
	if (lane == 0)
	{
		LOG("LaneSection::GetOuterOffset Error (lane id %d)\n", lane_id);
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
	double width = lane_width->poly3_.Evaluate(ds);

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
	int step = lane_id < 0 ? +1 : -1;

	if (lane_id == 0)
	{
		// Reference lane (0) has no width
		return 0.0;
	}
	double inner_offset = GetOuterOffset(s, lane_id + step);
	double outer_offset = GetOuterOffset(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return (inner_offset + outer_offset) / 2;
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

	if (GetLaneById(id)->GetLink(link_type))
	{
		id = GetLaneById(id)->GetLink(link_type)->GetId();
	}
	else
	{
		// Move inward, step by step until a driving lane is found
		while (id != 0)
		{
			id -= SIGN(id) * 1;
			if (GetLaneById(id))
			{
				if (GetLaneById(id)->IsDriving())
				{
					break;
				}
			}
		}
		if (id == 0)
		{
			// if no driving lane found - stay on same index
			id = incoming_lane_id;
		}
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
	int i;

	for (i = 0; i < GetNumberOfLaneSections() - 1; i++)
	{
		if (s < lane_section_[i]->GetS())
		{
			break;
		}
	}

	return (lane_section_[i]->GetNumberOfLanes());
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
		LOG("GetRoadByIdx error (idx %d, nroads %d)\n", idx, (int)road_.size());
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

	pugi::xml_document doc;

	pugi::xml_parse_result result = doc.load_file(filename);
	if (!result)
	{
		throw std::invalid_argument(std::string("Failed to load OpenDRIVE file ") + std::string(filename));

		return false;
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
		if (roadId == connection->GetIncomingRoad()->GetId())
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
		Connection * connection = GetConnectionByIdx(i);
		if (roadId == connection->GetIncomingRoad()->GetId())
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

	return lane_road_lane_connection;
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
	for (size_t i=0; i<road_.size(); i++)
	{
		delete(road_[i]);
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

bool OpenDrive::IsConnected(int road1_id, int road2_id, int* &connecting_road_id, int* &connecting_lane_id, int lane1_id, int lane2_id)
{
	Road *road1 = GetRoadById(road1_id);
	Road *road2 = GetRoadById(road2_id);

	LinkType link_type;
	RoadLink *link = 0;

	// Look from road 1, both ends, for road 2
	if (lane1_id < 0)
	{
		link_type = SUCCESSOR;
		link = road1->GetLink(link_type);
	}
	else if (lane1_id > 0)
	{
		link_type = PREDECESSOR;
		link = road1->GetLink(link_type);
	}
	else
	{
		// Try both ends
		link_type = SUCCESSOR;
		link = road1->GetLink(link_type);
		if (link == 0)
		{
			link_type = PREDECESSOR;
			link = road1->GetLink(link_type);
			if (link == 0)
			{
				LOG("OpenDrive::IsConnected Link not found!\n");
				return false;
			}
		}
	}
	if (link == 0)
	{
		return false;
	}

	LaneSection *lane_section = 0;

	if (link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
	{
		if (link->GetElementId() == road2->GetId())
		{
			if (lane1_id != 0 && lane2_id != 0)
			{
				// Check lane connected
				if (link_type == SUCCESSOR)
				{
					lane_section = road1->GetLaneSectionByIdx(road1->GetNumberOfLaneSections() - 1);
				}
				else if (link_type == PREDECESSOR)
				{
					lane_section = road1->GetLaneSectionByIdx(0);
				}
				else
				{
					LOG("OpenDrive::IsConnected Error LinkType %d not suppoered\n", link_type);
					return false;
				}
				if (lane_section == 0)
				{
					LOG("OpenDrive::IsConnected Error lane section == 0\n");
					return false;
				}
				Lane *lane = lane_section->GetLaneById(lane1_id);
				(void)lane;
				if (!(lane_section->GetConnectingLaneId(lane1_id, link_type) == lane2_id))
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
				RoadLink *exit_link = connecting_road->GetLink(SUCCESSOR);

				if (exit_link->GetElementId() == road2_id)
				{
					// Finally check that lanes are connected through the junction
					// Look at lane section and locate lane connecting both roads
					// Assume connecting road has only one lane section 
					lane_section = connecting_road->GetLaneSectionByIdx(0);
					if (lane_section == 0)
					{
						LOG("OpenDrive::IsConnected Error lane section == 0\n");
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
						if (lane_link_predecessor->GetId() == lane1_id && lane_link_successor->GetId() == lane2_id)
						{
							// Found link
							if (connecting_road_id != 0)
							{
								*connecting_road_id = connection->GetConnectingRoad()->GetId();
							}
							if (connecting_lane_id != 0)
							{
								*connecting_lane_id = connection->GetConnectingLaneId(lane1_id);
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
		LOG("OpenDrive::IsConnected Error: LinkElementType %d unsupported\n", link->GetElementType());
	}

	return false;
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
	track_id_ = 0;
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
	h_relative_ = 0.0;

	track_idx_ = 0;
	geometry_idx_ = 0;
	lane_section_idx_ = 0;
	lane_idx_ = 0;
	elevation_idx_ = 0;
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
	LaneInfo lane_info = road->GetLaneInfoByS(s_, lane_section_idx_, lane_id_);
	LaneSection *lane_section = road->GetLaneSectionByIdx(lane_info.lane_section_idx_);

	if (lane_section == 0)
	{
		LOG("Position::Track2Lane: No lane section for idx %d - keeping current lane setting\n", lane_section_idx_);
		return;
	}

	// Find the closest driving lane within the lane section
	int n_lanes = lane_section->GetNumberOfLanes();
	double min_offset = t_;  // Initial offset relates to reference line
	int candidate_lane_id = 0;

	if (n_lanes > 0)
	{
		for (int i = 0; i < n_lanes; i++)  // Search through all lanes
		{
			int lane_id = lane_section->GetLaneIdByIdx(i);
			double laneCenterOffset = SIGN(lane_id) * lane_section->GetCenterOffset(s_, lane_id);
						
			if (lane_section->GetLaneById(lane_id)->IsDriving() && (candidate_lane_id == 0 || fabs(t_ - laneCenterOffset) < fabs(min_offset)))
			{
				min_offset = t_ - laneCenterOffset;
				candidate_lane_id = lane_id;
			}
		}
	}

	offset_ = min_offset;

	if (candidate_lane_id != lane_id_)
	{
		//LOG("change lane: %d -> %d\n", lane_id_, candidate_lane_id);
		// Update cache indices
		lane_id_ = candidate_lane_id;
		lane_idx_ = lane_section->GetLaneIdxById(lane_id_);
	}
}

static double PointDistance(double x0, double y0, double x1, double y1)
{
	// https://en.wikipedia.org/wiki/Distance

	return sqrt((x1 - x0)*(x1 - x0) + (y1 - y0) * (y1 - y0));
}

static bool PointInBetween(double x3, double y3, double x1, double y1, double x2, double y2, double &sNorm)
{
	bool inside;

	if (fabs(x2 - x1) < fabs(y2 - y1))  // Line is steep (more vertical than horizontal
	{
		sNorm = (y3 - y1) / (y2 - y1);
		if (y2 > y1)  // ascending
		{
			inside = !(y3 < y1 || y3 > y2);
		}
		else
		{
			inside = !(y3 > y1 || y3 < y2);
		}
	}
	else
	{
		sNorm = (x3 - x1) / (x2 - x1);
		if (x2 > x1)  // forward
		{
			inside = !(x3 < x1 || x3 > x2);
		}
		else
		{
			inside = !(x3 > x1 || x3 < x2);
		}
	}
	return inside;
}


static int PointSideOfVec(double px, double py, double vx1, double vy1, double vx2, double vy2)
{
	// Use dot product
	return SIGN((vx2 - vx1)*(py - vy1) - (vy2 - vy1)*(px - vx1));
}

double Position::GetDistToTrackGeom(double x3, double y3, double h, Road *road, Geometry *geom, bool &inside, double &sNorm)
{
	// Find vector from point perpendicular to line segment
	// https://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point

	x_ = x3;
	y_ = y3;
	h_ = h;
	r_ = 0;
	double x1, y1, h1;
	double x2, y2, h2;
	int side;

//	double line_dist = 0;
	double dist = 0;
	geom->EvaluateDS(0, &x1, &y1, &h1);
	geom->EvaluateDS(geom->GetLength(), &x2, &y2, &h2);

	// Apply lane offset
	x1 += road->GetLaneOffset(0) * cos(h1 + M_PI_2);
	y1 += road->GetLaneOffset(geom->GetLength()) * sin(h1 + M_PI_2);
	x2 += road->GetLaneOffset(0) * cos(h2 + M_PI_2);
	y2 += road->GetLaneOffset(geom->GetLength()) * sin(h2 + M_PI_2);


	double x4, y4, k;
	k = ((y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1)) / ((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1));
	x4 = x3 - k * (y2 - y1);
	y4 = y3 + k * (x2 - x1);

	// Check whether the projected point is inside or outside line segment
	inside = PointInBetween(x4, y4, x1, y1, x2, y2, sNorm);
	if (inside)
	{
		dist = PointDistance(x3, y3, x4, y4);
	}
	else
	{
		// Distance is mesared between point to closest endpoint of line
		double d1, d2;
		d1 = PointDistance(x3, y3, x1, y1);
		d2 = PointDistance(x3, y3, x2, y2);
		dist = MIN(d1, d2);
	}
	side = PointSideOfVec(x3, y3, x1, y1, x2, y2);

#if 1
	// Now, calculate actual distance to road geometry - not to a straight line

	double dsMin = sNorm * geom->GetLength();
	double sMin = geom->GetS() + dsMin;
	double x, y;

	if (inside)  // else stick with line approximation
	{
		geom->EvaluateDS(dsMin, &x, &y, &h);
		// Apply lane offset
		x += road->GetLaneOffset(dsMin) * cos(h + M_PI_2);
		y += road->GetLaneOffset(dsMin) * sin(h + M_PI_2);
		dist = PointDistance(x3, y3, x, y);

		// Check whether the point is left or right side of road
		// x3, y3 is the point checked against a vector aligned with heading
		side = PointSideOfVec(x3, y3, x, y, x + cos(h), y + sin(h));
	}
	// dist is now actually the lateral distance from reference lane, e.g. track coordinate t-value
	// Finally find closest lane

	// Finally calculate exakt distance
	double min_lane_dist = dist;

	LaneSection *lane_section = road->GetLaneSectionByS(sMin);
	if (lane_section != 0)
	{
		for (int i = 0; i < lane_section->GetNumberOfLanes(); i++)
		{
			if (lane_section->GetLaneByIdx(i)->IsDriving())
			{
				double signed_offset = dist * SIGN(side);
				double signed_lane_center_offset = SIGN(lane_section->GetLaneIdByIdx(i)) * lane_section->GetCenterOffset(sMin, lane_section->GetLaneIdByIdx(i));
				double lane_dist = signed_offset - signed_lane_center_offset;

				if (fabs(lane_dist) < fabs(min_lane_dist))
				{
					min_lane_dist = lane_dist;
				}
			}
		}
	}
	return fabs(min_lane_dist);
#else
	return dist;
#endif
}

void Position::XYH2TrackPos(double x3, double y3, double h3, bool evaluateZAndPitch)
{
	double dist;
	double distMin = std::numeric_limits<double>::infinity();
	double sNorm;
	double sNormMin;
	Geometry *geom;
	Geometry *geomMin = 0;
	Road *road;
	Road *roadMin;
	bool inside = false;
	bool insideMin = false;

	(void)insideMin;

	if ((road = GetOpenDrive()->GetRoadByIdx(track_idx_)) == 0)
	{
		LOG("Invalid road index %d\n", track_idx_);
		return;
	}

	if ((geom = road->GetGeometry(geometry_idx_)) == 0)
	{
		LOG("Invalid geometry index %d\n", geometry_idx_);
		return;
	}

	// Search all road and lanes. Inefficient - but simple. Todo: Optimize
	bool found = false;
	(void)found;

	for (int i = 0; i < GetOpenDrive()->GetNumOfRoads(); i++)
	{
		road = GetOpenDrive()->GetRoadByIdx(i);
		for (int j = 0; j < road->GetNumberOfGeometries(); j++)
		{
			geom = road->GetGeometry(j);
			dist = GetDistToTrackGeom(x3, y3, h3, road, geom, inside, sNorm);
	
			if (dist < distMin)  // First value (min_dist = -1) is always the closest so far
			{
				geomMin = geom;
				roadMin = road;
				sNormMin = CLAMP(sNorm, 0.0, 1.0);
				distMin = dist;
				insideMin = inside;
				found = true;
			}
		}
	}

	if (!found)
	{
		LOG("Error finding minimum distance\n");
		return;
	}

	double dsMin = sNormMin * geomMin->GetLength();
	double sMin = geomMin->GetS() + dsMin;
	double x, y, h;

	// Found closest geometry. Now calculate exact distance to geometry. First find point perpendicular on geometry.
	geomMin->EvaluateDS(dsMin, &x, &y, &h);
	// Apply lane offset
	x += roadMin->GetLaneOffset(dsMin) * cos(h + M_PI_2);
	y += roadMin->GetLaneOffset(dsMin) * sin(h + M_PI_2);
	distMin = PointDistance(x3, y3, x, y);

	// Check whether the point is left or right side of road
	// x3, y3 is the point checked against a vector aligned with heading
	int side = PointSideOfVec(x3, y3, x, y, x + cos(h), y + sin(h));

	// Find out what lane 
	SetTrackPos(roadMin->GetId(), sMin, distMin * side, false);		

	//LOG("Closest point: dist %.2f side %d track_id %d lane_id %d s %.2f h %.2f\n", distMin, side, roadMin->GetId(), GetLaneId(), s_, h);
	if (evaluateZAndPitch)
	{
		EvaluateZAndPitch();
	}
}
	
bool Position::EvaluateZAndPitch()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road && road->GetNumberOfElevations() > 0)
	{
		Elevation *elevation = road->GetElevation(elevation_idx_);
		if (elevation == NULL)
		{
			LOG("Elevation error NULL, nelev: %d elev_idx: %d\n", road->GetNumberOfElevations(), elevation_idx_);
		}
		if (elevation && s_ > elevation->GetS() + elevation->GetLength())
		{
			while (s_ > elevation->GetS() + elevation->GetLength() && elevation_idx_ < road->GetNumberOfElevations() - 1)
			{
				// Move to next elevation section
				elevation = road->GetElevation(++elevation_idx_);
			}
		}
		else if (elevation && s_ < elevation->GetS())
		{
			while (s_ < elevation->GetS() && elevation_idx_ > 0)
			{
				// Move to previous elevation section
				elevation = road->GetElevation(--elevation_idx_);
			}
		}

		if (elevation)
		{
			double p = s_ - elevation->GetS();
			z_ = elevation->poly3_.Evaluate(p);
			//LOG("s: %.2f elevation_idx %d z: %.2f\n", s_, elevation_idx_, z_);
			p_ = -elevation->poly3_.EvaluatePrim(p);
		}
		return true;
	}
	else
	{
		return false;
	}
}

void Position::Track2XYZ()
{
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

	geometry->EvaluateDS(s_ - geometry->GetS(), &x_, &y_, &h_);
	
	// Consider lateral t position, perpendicular to track heading
	double x_local = (t_ + road->GetLaneOffset(s_)) * cos(h_ + M_PI_2);
	double y_local = (t_ + road->GetLaneOffset(s_)) * sin(h_ + M_PI_2);
	h_ += atan(road->GetLaneOffsetPrim(s_)) + h_offset_ + h_relative_;
	x_ += x_local;
	y_ += y_local;

	// z = Elevation 
	EvaluateZAndPitch();
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

void Position::XYZ2Track()
{
	XYH2TrackPos(GetX(), GetY(), GetH(), false);
}

void Position::SetLongitudinalTrackPos(int track_id, double s)
{
	Road *road;

	if ((road = GetOpenDrive()->GetRoadById(track_id)) == 0)
	{
		LOG("Position::Set Error: track %d not found\n", track_id);
		return;
	}
	if (track_id != track_id_)
	{
		// update internal track and geometry indices
		track_id_ = track_id;
		track_idx_ = GetOpenDrive()->GetTrackIdxById(track_id);
		geometry_idx_ = 0;
		elevation_idx_ = 0;
		lane_section_idx_ = 0;
	}

	if (s > road->GetLength())
	{
		LOG("Position::Set Warning: s (%.2f) too large, track %d only %.2f m long\n", s, track_id_, road->GetLength());
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
}

void Position::SetTrackPos(int track_id, double s, double t, bool calculateXYZ)
{
	SetLongitudinalTrackPos(track_id, s);

	t_ = t;
	Track2Lane();
	if (calculateXYZ)
	{
		Track2XYZ();
	}
}

int Position::MoveToConnectingRoad(RoadLink *road_link, double ds, double &s_remains, Junction::JunctionStrategyType strategy)
{
	double s_new;
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	Road *next_road = 0;
	LaneSection *lane_section;
	Lane *lane;
	int new_lane_id = 0;

	if (road == 0)
	{
		LOG("Position::MoveToConnectingRoad invalid road id %d\n", road->GetId());
		return -1;
	}

	if (road_link->GetElementId() == -1)
	{
		LOG("Position::MoveToConnectingRoad No connecting road or junction at rid %d s_ %.2f (ds %.2f s_ %.2f)\n", road->GetId(), s_ + ds, ds, s_);
		return -1;
	}
	
	lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	if (lane_section == 0)
	{
		LOG("Position::MoveToConnectingRoad No lane section rid %d ls_idx %d s %.2f\n", road->GetId(), lane_section_idx_, s_ + ds);
		return -1;
	}

	lane = lane_section->GetLaneByIdx(lane_idx_);
	if (lane == 0)
	{
		LOG("Position::MoveToConnectingRoad No lane rid %d lidx %d nlanes %d s_ %.2f s %.2f lsecidx %d\n", 
			road->GetId(), lane_idx_, lane_section->GetNumberOfLanes(), s_, s_ + ds, lane_section_idx_);
		return -1;
	}

	LinkType link_type;
	if (s_ + ds > road->GetLength())
	{
		s_new = s_ + ds - road->GetLength();
		link_type = SUCCESSOR;
	}
	else if (s_ + ds < 0)
	{
		s_new = -(s_ + ds);
		link_type = PREDECESSOR;
	}
	else
	{
		LOG("Position::MoveToConnectingRoad s_ + ds %.2f within range [0:%.2f]\n", s_ + ds, road->GetLength());
		return -1;
	}

	int contact_point = 0;

	if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
	{
		LaneLink *lane_link = lane->GetLink(link_type);
		if (lane_link != 0)
		{
			new_lane_id = lane->GetLink(link_type)->GetId();
			if (new_lane_id == 0)
			{
				LOG("Position::MoveToConnectingRoad Road+ new lane id %d\n", new_lane_id);
			}
		}
		contact_point = road_link->GetContactPointType();
		next_road = GetOpenDrive()->GetRoadById(road_link->GetElementId());
	}
	else if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
	{
		Junction *junction = GetOpenDrive()->GetJunctionById(road_link->GetElementId());

		if (junction == 0)
		{
			LOG("Position::MoveToConnectingRoad Error: junction %d not existing\n", road_link->GetElementType());
			return -1;
		}
	
		int connection_idx;
		int n_connections = junction->GetNumberOfRoadConnections(road->GetId(), lane->GetId());

		if (n_connections == 0)
		{
			LOG("No connections!");
			return -1;
		}
		else if (n_connections == 1)
		{
			LOG("Strange: Only one connection!");
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
					double heading_diff = fmod(test_pos.GetH() - GetH(), M_PI);
					if (heading_diff > 180)
					{
						heading_diff = 360 - heading_diff;
					}
					else if (heading_diff < -180)
					{
						heading_diff = 360 + heading_diff;
					}

					if (abs(heading_diff) < abs(min_heading_diff))
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
		contact_point = lane_road_lane_connection.contact_point_;

		new_lane_id = lane_road_lane_connection.GetConnectinglaneId();
		next_road = GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId());
	}

	if (new_lane_id == 0)
	{
		LOG("Position::MoveToConnectingRoad: No connection from rid %d lid %d -> rid %d eltype %d\n", 
			road->GetId(), lane->GetId(), road_link->GetElementId(), road_link->GetElementType());
		return -1;
	}

	if (next_road == 0)
	{
		LOG("Position::MoveToConnectingRoad: No next road\n");
		return -1;
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
	s_remains = 0;
	if (road_link->GetContactPointType() == CONTACT_POINT_START)
	{
		if (s_new <= next_road->GetLength())
		{
			SetLanePos(road_link->GetElementId(), new_lane_id, s_new, new_offset);
		}
		else
		{
			SetLanePos(road_link->GetElementId(), new_lane_id, next_road->GetLength(), new_offset);
			s_remains = s_new - next_road->GetLength();
		}
	}
	else if (road_link->GetContactPointType() == CONTACT_POINT_END)
	{
		if (s_new <= next_road->GetLength())
		{
			SetLanePos(road_link->GetElementId(), new_lane_id, next_road->GetLength() - s_new, new_offset);
		}
		else
		{
			SetLanePos(road_link->GetElementId(), new_lane_id, 0, new_offset);
			s_remains = s_new - next_road->GetLength();
		}
	}
	else if (road_link->GetContactPointType() == CONTACT_POINT_NONE && road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
	{
		if (contact_point == CONTACT_POINT_START)
		{
			if (s_new <= next_road->GetLength())
			{
				SetLanePos(next_road->GetId(), new_lane_id, s_new, new_offset);
			}
			else
			{
				SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), new_offset);
				s_remains = s_new - next_road->GetLength();
			}
		}
		else if (contact_point == CONTACT_POINT_END)
		{
			if (s_new <= next_road->GetLength())
			{
				SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength() - s_new, new_offset);
			}
			else
			{
				SetLanePos(next_road->GetId(), new_lane_id, 0, new_offset);
				s_remains = s_new - next_road->GetLength();
			}
		}
		else
		{
			LOG("Position::MoveToConnectingRoad unexpected contact point: %d\n", contact_point);
		}
	}
	else
	{
		LOG("Position::MoveToConnectingRoad Unsupported contact point type %d\n", road_link->GetContactPointType());
		return -1;
	}

	if(s_remains > 1E-10)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int Position::MoveAlongS(double ds, double dLaneOffset, Junction::JunctionStrategyType strategy)
{
	RoadLink *link;
	int max_links = 4;  // limit lookahead through junctions/links 
	
	// EG: If offset_ is not along reference line, but instead along lane direction then we dont need
	// the SIGN() adjustment. But for now this adjustment means that a positive dLaneOffset always moves left?
	offset_ += dLaneOffset * -SIGN(GetLaneId());

	for (int i = 0; i < max_links; i++)
	{
		ds *= -SIGN(GetLaneId()); // adjust sign of ds according to lane direction - right lane is < 0 in road dir

		if (s_ + ds > GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength())
		{
			link = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLink(SUCCESSOR);
		}
		else if (s_ + ds < 0)
		{
			link = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLink(PREDECESSOR);
		}
		else  // New position is within current track
		{
			SetLanePos(track_id_, lane_id_, s_ + ds, offset_);
			return 0;
		}

		// Move to connected road
		if (!link || link->GetElementId() == -1)
		{
			return -1;
		}

		double s_remains = 0;
		int returnvalue = MoveToConnectingRoad(link, ds, s_remains, strategy);

		if(returnvalue != 1)
		{
			return returnvalue;
		}
		else
		{
			ds = s_remains;  // not done, we need to go further across a linked road
		}
	}
	return(-1);
}

void Position::SetLanePos(int track_id, int lane_id, double s, double offset, int lane_section_idx)
{
	offset_ = offset;

	SetLongitudinalTrackPos(track_id, s);

	Road *road = GetOpenDrive()->GetRoadById(track_id);
	if (road == 0)
	{
		LOG("Position::Set Error: track %d not available\n", track_id);
		return;
	}

	if (lane_id != lane_id_ && lane_section_idx == -1)
	{
		// New lane ID might indicate a discreet jump to a new, distant position, reset lane section, if not specified)
		//LOG("Explicit lane change %d -> %d and reseting lane section\n", lane_id_, lane_id);
		lane_section_idx = 0;
	}

	LaneSection *lane_section = 0;

	if (lane_section_idx > -1)  // If lane section was specified or reset
	{
		lane_section_idx_ = lane_section_idx;
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
		//LOG("New lane section, new lane: %d -> %d\n", lane_id_, lane_id);
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

	Lane2Track();
	Track2XYZ();

	// Adjust heading to lane direction
	if (lane_id > 0)
	{
		h_ += M_PI;
		p_ *= -1;
	}
}

void Position::SetInertiaPos(double x, double y, double z, double h, double p, double r, bool updateTrackPos)
{
	x_ = x;
	y_ = y;
	z_ = z;
	h_ = h;
	p_ = p;
	r_ = r;

	if (updateTrackPos)
	{
		XYZ2Track();
	}
}

double Position::GetCurvature()
{
	Geometry *geom = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetGeometry(geometry_idx_);

	return(geom->EvaluateCurvatureDS(GetS() - geom->GetS()));
}

void Position::PrintTrackPos()
{
	LOG("	Track pos: (%d, %.2f, %.2f)\n", track_id_, s_, t_);
}

void Position::PrintLanePos()
{
	LOG("	Lane pos: (%d, %d, %.2f, %.2f)\n", track_id_, lane_id_, s_, offset_);
}

void Position::PrintInertialPos()
{
	LOG("	Inertial pos: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)\n", x_, y_, z_, h_, p_, r_);
}

void Position::Print()
{
	LOG("Position: \n");
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

int Position::GetSteeringTargetPos(double lookahead_distance, double *target_pos_local, double *target_pos_global, double *angle, double *curvature)
{
	Position target(*this);  // Make a copy of current position
	target.offset_ = 0.0;  // Fix to lane center

	target.MoveAlongS(lookahead_distance, 0, Junction::STRAIGHT);
	
	target_pos_global[0] = target.GetX();
	target_pos_global[1] = target.GetY();
	target_pos_global[2] = target.GetZ();

	// find out local x, y, z
	double diff_x = target.GetX() - GetX();
	double diff_y = target.GetY() - GetY();
	double diff_z = target.GetZ() - GetZ();

	target_pos_local[0] = diff_x * cos(-GetH()) - diff_y * sin(-GetH());
	target_pos_local[1] = diff_x * sin(-GetH()) + diff_y * cos(-GetH());
	target_pos_local[2] = diff_z;

#if 0
	// remove when validated
	target_pos_global[0] = GetX() + target_pos_local[0] * cos(GetH()) - target_pos_local[1] * sin(GetH());
	target_pos_global[1] = GetY() + target_pos_local[0] * sin(GetH()) + target_pos_local[1] * cos(GetH());
	target_pos_global[2] = GetZ() + target_pos_local[2];
#endif
	
	// Calculate angle - by dot product
	double dot_prod = 
		(target_pos_local[0] * 1.0 + target_pos_local[1] * 0.0) / 
		sqrt(target_pos_local[0] * target_pos_local[0] + target_pos_local[1] * target_pos_local[1]);
	*angle = SIGN(target_pos_local[1]) * acos(dot_prod);

	*curvature = target.GetCurvature();

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

int Position::MoveRouteDS(double ds, int dLane, double  dLaneOffset)
{
	if (!route_)
	{
		return -1;
	}

	if (route_->waypoint_.size() == 0)
	{
		return -1;
	}
	SetRouteLaneOffset(s_route_ + ds, dLane, dLaneOffset);

	return 0;
}

int Position::SetRouteLanePosition(double route_s, int laneId, double  laneOffset)
{
	SetRouteLaneOffset(route_s);

	// Override lane data
	SetLanePos(track_id_, laneId, s_, laneOffset);

	return 0;
}

int Position::SetRouteLaneOffset(double route_s, int dLane, double  dLaneOffset)
{
	if (route_->waypoint_.size() == 0)
	{
		LOG("SetOffset No waypoints!");
		return -1;
	}
	double s_start = route_->waypoint_[0]->GetS();
	double route_length = 0;
	s_route_ = route_s;

	// Find out what road and local s value
	for (size_t i = 0; i < route_->waypoint_.size(); i++)
	{
		double road_length = route_->waypoint_[i]->GetOpenDrive()->GetRoadById(route_->waypoint_[i]->GetTrackId())->GetLength();
		if (s_route_ < route_length + road_length - s_start)
		{
			// Found road segment
			double local_s = s_route_ + s_start - route_length;

			// Determine driving direction by lane id
			if (route_->waypoint_[i]->GetLaneId() > 0)
			{
				// going towards road direction (left side of reference line), adjust s accordingly
				local_s = road_length - local_s;
			}

			SetLanePos(route_->waypoint_[i]->GetTrackId(), route_->waypoint_[i]->GetLaneId() + dLane, local_s, dLaneOffset);
			return 0;
		}
		route_length += road_length - s_start;
		s_start = 0;  // For all following road segments, calculate length from beginning
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
			if (position->GetOpenDrive()->IsConnected(prev_pos->GetTrackId(), position->GetTrackId(), 
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
			LOG("Route::AddWaypoint Error: waypoint (%d, %d) is not connected to the previous one (%d, %d)\n",
				position->GetTrackId(), position->GetLaneId(), prev_pos->GetTrackId(), prev_pos->GetLaneId());

			return -1;
		}
	}

	waypoint_.push_back(position);
	LOG("Route::AddWaypoint Added waypoint %d: %d, %d, %.2f\n", (int)waypoint_.size()-1, position->GetTrackId(), position->GetLaneId(), position->GetS());

	return 0; 
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
