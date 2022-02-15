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
 * When used standalone (outside ScenarioEngine) the road manager is initialized via the Position class like
 *this: roadmanager::Position::LoadOpenDrive("example.xodr");
 *
 * Simplest use case is to put a vehicle on the road and simply move it forward along the road, e.g:
 *
 *   car->pos = new roadmanager::Position(3, -2, 10, 0);
 *   while(true)
 *   {
 *	    car->pos->MoveAlongS(0.1);
 *   }
 *
 * The first line will create a Position object initialized at road with ID = 3, in lane = -2 and at lane
 *offset = 0 Then the position is updated along that road and lane, moving 10 cm at a time.
 *
 * A bit more realistic example:
 *
 *   car->pos = new roadmanager::Position(odrManager->GetRoadByIdx(0)->GetId(), -2, 10, 0);
 *   while(true)
 *   {
 *	    car->pos->MoveAlongS(speed * dt);
 *   }
 *
 * Here we refer to the ID of the first road in the network. And instead of static delta movement, the
 *distance is a function of speed and delta time since last update.
 *
 */

#include <assert.h>
#include <time.h>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>

#include "CommonMini.hpp"
#include "RoadManager.hpp"
#include "odrSpiral.h"
#include "pugixml.hpp"

static unsigned int global_lane_counter;

using namespace std;
using namespace roadmanager;

#define CURV_ZERO 0.00001
#define MAX_TRACK_DIST 10
#define OSI_POINT_CALC_STEPSIZE 1		 // [m]
#define OSI_TANGENT_LINE_TOLERANCE 0.01	 // [m]
#define OSI_POINT_DIST_SCALE 0.025
#define ROADMARK_WIDTH_STANDARD 0.15
#define ROADMARK_WIDTH_BOLD 0.20

static int g_Lane_id = 0;
static int g_Laneb_id = 0;

static std::string LinkType2Str(LinkType type) {
	if (type == LinkType::PREDECESSOR) {
		return "PREDECESSOR";
	} else if (type == LinkType::SUCCESSOR) {
		return "SUCCESSOR";
	} else if (type == LinkType::NONE) {
		return "NONE";
	} else {
		return std::string("Unknown link type: " + std::to_string(type));
	}
}

int roadmanager::GetNewGlobalLaneId() {
	int returnvalue = g_Lane_id;
	g_Lane_id++;
	return returnvalue;
}

int roadmanager::GetNewGlobalLaneBoundaryId() {
	int returnvalue = g_Laneb_id;
	g_Laneb_id++;
	return returnvalue;
}

std::string ReadAttribute(pugi::xml_node node, std::string attribute_name, bool required) {
	if (!strcmp(attribute_name.c_str(), "")) {
		if (required) {
			LOG("Warning: Required but empty attribute");
		}
		return "";
	}

	pugi::xml_attribute attr;

	if ((attr = node.attribute(attribute_name.c_str()))) {
		return attr.value();
	} else {
		if (required) {
			LOG("Warning: missing required attribute: %s -> %s", node.name(), attribute_name.c_str());
		}
	}

	return "";
}

void ValidityRecord::Save(pugi::xml_node& object) {
	auto validity = object.append_child("validity");
	validity.append_attribute("fromLane").set_value(fromLane_);
	validity.append_attribute("toLane").set_value(toLane_);

	for (auto userData : user_data_) {
		userData->Save(validity);
	}
}

bool RoadPath::CheckRoad(Road* checkRoad, RoadPath::PathNode* srcNode, Road* fromRoad, int fromLaneId) {
	// Register length of this road and find node in other end of the road (link)

	RoadLink* nextLink = 0;

	if (srcNode->link->GetElementType() == RoadLink::RoadLink::ELEMENT_TYPE_ROAD) {
		// node link is a road, find link in the other end of it
		if (srcNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END) {
			nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
		} else {
			nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
		}
	} else if (srcNode->link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
		Junction* junction = Position::GetOpenDrive()->GetJunctionById(srcNode->link->GetElementId());
		if (junction && junction->GetType() == Junction::JunctionType::DIRECT) {
			if (checkRoad->GetLink(LinkType::SUCCESSOR)
				&& checkRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == junction->GetId()) {
				// Node link is a direct junction, and it is the successor to the road being checked
				// hence next link is the predecessor of that road
				nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
			} else if (checkRoad->GetLink(LinkType::PREDECESSOR)
					   && checkRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == junction->GetId()) {
				// Node link is a direct junction, and it is the predecessor to the road being checked
				// hence next link is the successor of that road
				nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
			}
		} else {
			if (checkRoad->GetLink(LinkType::SUCCESSOR)
				&& checkRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == fromRoad->GetId()) {
				// Node link is a non direct junction, and it is the successor to the connecting road being
				// checked hence next link is the predecessor of that connecting road
				nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
			} else if (checkRoad->GetLink(LinkType::PREDECESSOR)
					   && checkRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == fromRoad->GetId()) {
				// Node link is a non direct junction, and it is the predecessor to the connecting road being
				// checked hence next link is the successor of that connecting road
				nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
			}
		}
	}

	if (nextLink == 0) {
		// end of road
		return false;
	}

	int nextLaneId = fromRoad->GetConnectingLaneId(srcNode->link, fromLaneId, checkRoad->GetId());
	if (nextLaneId == 0) {
		return false;
	}

	// Check if next node is already visited
	for (size_t i = 0; i < visited_.size(); i++) {
		if (visited_[i]->link == nextLink) {
			// Already visited, ignore and return
			return false;
		}
	}

	// Check if next node is already among unvisited
	size_t i;
	for (i = 0; i < unvisited_.size(); i++) {
		if (unvisited_[i]->link == nextLink) {
			// Consider it, i.e. calc distance and potentially store it (if less than old)
			if (srcNode->dist + checkRoad->GetLength() < unvisited_[i]->dist) {
				unvisited_[i]->dist = srcNode->dist + checkRoad->GetLength();
			}
		}
	}

	if (i == unvisited_.size()) {
		// link not visited before, add it
		PathNode* pNode = new PathNode;
		pNode->dist = srcNode->dist + checkRoad->GetLength();
		pNode->link = nextLink;
		pNode->fromRoad = checkRoad;
		pNode->fromLaneId = nextLaneId;
		pNode->previous = srcNode;
		unvisited_.push_back(pNode);
	}

	return true;
}

int RoadPath::Calculate(double& dist, bool bothDirections, double maxDist) {
	OpenDrive* odr = startPos_->GetOpenDrive();
	RoadLink* link = 0;
	Junction* junction = 0;
	Road* startRoad = odr->GetRoadById(startPos_->GetTrackId());
	Road* targetRoad = odr->GetRoadById(targetPos_->GetTrackId());
	Road* pivotRoad = startRoad;
	int pivotLaneId = startPos_->GetLaneId();
	Road* nextRoad = startRoad;
	bool found = false;
	double tmpDist = 0;
	size_t i;

	// This method will find and measure the length of the shortest path
	// between a start position and a target position
	// The implementation is based on Dijkstra's algorithm
	// https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

	if (pivotRoad == 0) {
		LOG("Invalid startpos road ID: %d", startPos_->GetTrackId());
		return -1;
	}

	for (i = 0; i < (bothDirections ? 2 : 1); i++) {
		ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
		if (bothDirections) {
			if (i == 0) {
				contact_point = ContactPointType::CONTACT_POINT_START;
				link = pivotRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
			} else {
				contact_point = ContactPointType::CONTACT_POINT_END;
				link = pivotRoad->GetLink(LinkType::SUCCESSOR);	 // Find link to previous road or junction
			}
		} else {
			// Look only in forward direction, w.r.t. entity heading
			if (startPos_->GetHRelative() < M_PI_2 || startPos_->GetHRelative() > 3 * M_PI_2) {
				// Along road direction
				contact_point = ContactPointType::CONTACT_POINT_END;
				link = pivotRoad->GetLink(LinkType::SUCCESSOR);	 // Find link to next road or junction
			} else {
				// Opposite road direction
				contact_point = ContactPointType::CONTACT_POINT_START;
				link = pivotRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
			}
		}

		if (link) {
			PathNode* pNode = new PathNode;
			pNode->link = link;
			pNode->fromRoad = pivotRoad;
			pNode->fromLaneId = pivotLaneId;
			pNode->previous = 0;
			pNode->contactPoint = contact_point;
			if (contact_point == ContactPointType::CONTACT_POINT_START) {
				pNode->dist = startPos_->GetS();  // distance to first road link is distance to start of road
			} else if (contact_point == ContactPointType::CONTACT_POINT_END) {
				pNode->dist = pivotRoad->GetLength() - startPos_->GetS();  // distance to end of road
			}

			unvisited_.push_back(pNode);
		}
	}

	if (startRoad == targetRoad) {
		dist = targetPos_->GetS() - startPos_->GetS();

		// Special case: On same road, distance is equal to delta s
		if (startPos_->GetLaneId() < 0) {
			if (startPos_->GetHRelative() > M_PI_2 && startPos_->GetHRelative() < 3 * M_PI_2) {
				// facing opposite road direction
				dist *= -1;
			}
		} else {
			// decreasing in lanes with positive IDs
			dist *= -1;

			if (startPos_->GetHRelative() < M_PI_2 || startPos_->GetHRelative() > 3 * M_PI_2) {
				// facing along road direction
				dist *= -1;
			}
		}

		return 0;
	}

	if (unvisited_.size() == 0) {
		// No links
		dist = 0;
		return -1;
	}

	for (i = 0; i < 100 && !found && unvisited_.size() > 0 && tmpDist < maxDist; i++) {
		found = false;

		// Find unvisited PathNode with shortest distance
		double minDist = LARGE_NUMBER;
		int minIndex = 0;
		for (size_t j = 0; j < unvisited_.size(); j++) {
			if (unvisited_[j]->dist < minDist) {
				minIndex = (int)j;
				minDist = unvisited_[j]->dist;
			}
		}

		link = unvisited_[minIndex]->link;
		tmpDist = unvisited_[minIndex]->dist;
		pivotRoad = unvisited_[minIndex]->fromRoad;
		pivotLaneId = unvisited_[minIndex]->fromLaneId;

		// - Inspect all unvisited neighbor nodes (links), measure edge (road) distance to that link
		// - Note the total distance
		// - If not already in invisited list, put it there.
		// - Update distance to this link if shorter than previously registered value
		if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
			// only one edge (road)
			nextRoad = odr->GetRoadById(link->GetElementId());

			if (nextRoad == targetRoad) {
				// Special case: On same road, distance is equal to delta s, direction considered
				if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_START) {
					tmpDist += targetPos_->GetS();
				} else {
					tmpDist += nextRoad->GetLength() - targetPos_->GetS();
				}

				found = true;
			} else {
				CheckRoad(nextRoad, unvisited_[minIndex], pivotRoad, pivotLaneId);
			}
		} else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
			// check all junction links (connecting roads) that has pivot road as incoming road
			junction = odr->GetJunctionById(link->GetElementId());
			for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(pivotRoad->GetId()); j++) {
				nextRoad = odr->GetRoadById(
					junction->GetConnectingRoadIdFromIncomingRoadId(pivotRoad->GetId(), (int)j));
				if (nextRoad == 0) {
					return 0;
				}

				if (nextRoad == targetRoad)	 // target road reached
				{
					ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
					if (nextRoad->IsSuccessor(pivotRoad, &contact_point)
						|| nextRoad->IsPredecessor(pivotRoad, &contact_point)) {
						if (contact_point == ContactPointType::CONTACT_POINT_START) {
							tmpDist += targetPos_->GetS();
						} else if (contact_point == ContactPointType::CONTACT_POINT_END) {
							tmpDist += nextRoad->GetLength() - targetPos_->GetS();
						} else {
							LOG("Unexpected contact point %s",
								OpenDrive::ContactPointType2Str(contact_point).c_str());
							return -1;
						}
					} else {
						LOG("Failed to check link in junction");
						return -1;
					}
					found = true;
				} else {
					CheckRoad(nextRoad, unvisited_[minIndex], pivotRoad, pivotLaneId);
				}
			}
		}

		// Mark pivot link as visited (move it from unvisited to visited)
		visited_.push_back(unvisited_[minIndex]);
		unvisited_.erase(unvisited_.begin() + minIndex);
	}

	if (found) {
		// Find out whether the path goes forward or backwards from starting position
		if (visited_.size() > 0) {
			RoadPath::PathNode* node = visited_.back();

			while (node) {
				if (node->previous == 0) {
					// This is the first node - inspect whether it is in front or behind start position
					if ((node->link == startRoad->GetLink(LinkType::PREDECESSOR)
						 && abs(startPos_->GetHRelative()) > M_PI_2
						 && abs(startPos_->GetHRelative()) < 3 * M_PI / 2)
						|| ((node->link == startRoad->GetLink(LinkType::SUCCESSOR)
								 && abs(startPos_->GetHRelative()) < M_PI_2
							 || abs(startPos_->GetHRelative()) > 3 * M_PI / 2))) {
						direction_ = 1;
					} else {
						direction_ = -1;
					}
				}
				node = node->previous;
			}
		}
	}

	// Compensate for heading of the start position
	if (startPos_->GetHRelativeDrivingDirection() > M_PI_2
		&& startPos_->GetHRelativeDrivingDirection() < 3 * M_PI_2) {
		direction_ *= -1;
	}
	dist = direction_ * tmpDist;

	return found ? 0 : -1;
}

RoadPath::~RoadPath() {
	for (size_t i = 0; i < visited_.size(); i++) {
		delete (visited_[i]);
	}
	visited_.clear();

	for (size_t i = 0; i < unvisited_.size(); i++) {
		delete (unvisited_[i]);
	}
	unvisited_.clear();
}

int PolyLineBase::EvaluateSegmentByLocalS(int i, double local_s, double cornerRadius, TrajVertex& pos) {
	TrajVertex* vp0 = &vertex_[i];

	if (i >= GetNumberOfVertices() - 1) {
		pos.x = vp0->x;
		pos.y = vp0->y;
		pos.z = vp0->z;
		pos.h = vp0->h;
		pos.s = vp0->s;
		pos.p = vp0->p;
		pos.time = vp0->time;
		pos.speed = vp0->speed;
	} else if (i >= 0) {
		TrajVertex* vp1 = &vertex_[i + 1];

		double length = MAX(vertex_[i + 1].s - vertex_[i].s, SMALL_NUMBER);

		local_s = CLAMP(local_s, 0, length);

		double a = local_s / length;  // a = interpolation factor

		pos.x = (1 - a) * vp0->x + a * vp1->x;
		pos.y = (1 - a) * vp0->y + a * vp1->y;
		pos.z = (1 - a) * vp0->z + a * vp1->z;
		pos.time = (1 - a) * vp0->time + a * vp1->time;
		pos.speed = (1 - a) * vp0->speed + a * vp1->speed;
		pos.s = (1 - a) * vp0->s + a * vp1->s;
		pos.p = (1 - a) * vp0->p + a * vp1->p;

		if (vertex_[i + 1].calcHeading && !interpolateHeading_) {
			// Strategy: Align to line, but interpolate at corners
			double radius = MIN(4.0, length);
			if (local_s < radius) {
				// passed a corner
				a = (radius + local_s) / (2 * radius);
				if (i > 0) {
					pos.h = GetAngleInInterval2PI(vertex_[i - 1].h
												  + a * GetAngleDifference(vertex_[i].h, vertex_[i - 1].h));
				} else {
					// No previous value to interpolate
					pos.h = vertex_[i].h;
				}
			} else if (local_s > length - radius) {
				a = (radius + (length - local_s)) / (2 * radius);
				if (i > GetNumberOfVertices() - 2) {
					// Last segment, no next point to interpolate
					pos.h = a * vertex_[i].h;
				} else {
					pos.h = GetAngleInInterval2PI(
						vertex_[i].h + (1 - a) * GetAngleDifference(vertex_[i + 1].h, vertex_[i].h));
				}
			} else {
				pos.h = vertex_[i].h;
			}
		} else {
			// Interpolate
			pos.h = GetAngleInInterval2PI(vp0->h + a * GetAngleDifference(vp1->h, vp0->h));
		}
	} else {
		return -1;
	}

	return 0;
}

TrajVertex* PolyLineBase::AddVertex(double x, double y, double z, double h) {
	TrajVertex v;

	v.calcHeading = false;
	vertex_.push_back(v);

	return UpdateVertex(GetNumberOfVertices() - 1, x, y, z, GetAngleInInterval2PI(h));
}

TrajVertex* PolyLineBase::AddVertex(double x, double y, double z) {
	TrajVertex v;

	v.calcHeading = true;
	vertex_.push_back(v);

	return UpdateVertex(GetNumberOfVertices() - 1, x, y, z);
}

TrajVertex* PolyLineBase::AddVertex(TrajVertex p) {
	vertex_.push_back(p);

	if (p.calcHeading) {
		return UpdateVertex(GetNumberOfVertices() - 1, p.x, p.y, p.z);
	} else {
		return UpdateVertex(GetNumberOfVertices() - 1, p.x, p.y, p.z, p.h);
	}
}

TrajVertex* PolyLineBase::UpdateVertex(int i, double x, double y, double z) {
	TrajVertex* v = &vertex_[i];

	v->x = x;
	v->y = y;
	v->z = z;

	if (i > 0) {
		TrajVertex* vp = &vertex_[i - 1];

		if (v->calcHeading) {
			// Calulate heading from line segment between this and previous vertices
			if (PointDistance2D(v->x, v->y, vp->x, v->y) < SMALL_NUMBER) {
				// If points conside, use heading of previous vertex
				v->h = vp->h;
			} else {
				v->h = GetAngleInInterval2PI(atan2(v->y - vp->y, v->x - vp->x));
			}
		}

		if (vp->calcHeading) {
			// Update heading of previous vertex now that outgoing line segment is known
			vp->h = v->h;
		}

		// Update polyline length
		double dist = PointDistance2D(x, y, vp->x, vp->y);
		length_ += dist;
	} else if (i == 0) {
		length_ = 0;
	}

	v->s = length_;

	return &vertex_[i];
}

TrajVertex* PolyLineBase::UpdateVertex(int i, double x, double y, double z, double h) {
	TrajVertex* v = &vertex_[i];

	v->h = h;

	UpdateVertex(i, x, y, z);

	return &vertex_[i];
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, double cornerRadius, int startAtIndex) {
	double s_local = 0;
	int i = startAtIndex;

	if (GetNumberOfVertices() < 1) {
		return -1;
	}

	if (s > GetVertex(-1)->s) {
		// end of trajectory
		s = length_;
		s_local = 0;
		i = GetNumberOfVertices() - 1;
	} else {
		for (; i < GetNumberOfVertices() - 1 && vertex_[i + 1].s <= s; i++)
			;

		double s0 = vertex_[i].s;
		s_local = s - s0;
	}

	EvaluateSegmentByLocalS(i, s_local, cornerRadius, pos);
	pos.s = s;

	return i;
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, double cornerRadius) {
	return Evaluate(s, pos, cornerRadius, 0);
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, int startAtIndex) {
	return Evaluate(s, pos, 0.0, startAtIndex);
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos) {
	return Evaluate(s, pos, 0.0, 0);
}

int PolyLineBase::Time2S(double time, double& s) {
	if (GetNumberOfVertices() < 1) {
		return -1;
	}

	// start looking from current index
	int i = vIndex_;

	for (size_t j = 0; j < GetNumberOfVertices(); j++) {
		if (vertex_[i].time <= time && vertex_[i + 1].time > time) {
			double w = (time - vertex_[i].time) / (vertex_[i + 1].time - vertex_[i].time);
			s = vertex_[i].s + w * (vertex_[i + 1].s - vertex_[i].s);
			vIndex_ = i;
			return 0;
		}

		if (++i >= GetNumberOfVertices() - 1) {
			// Reached end of buffer, continue from start
			i = 0;
		}
	}

	// s seems out of range, grab last element
	s = GetVertex(-1)->s;

	return 0;
}

int PolyLineBase::FindClosestPoint(double xin, double yin, TrajVertex& pos, int& index, int startAtIndex) {
	// look along the line segments
	TrajVertex tmpPos;
	double sLocal = 0.0;
	double sLocalMin = 0.0;
	int iMin = startAtIndex;
	double distMin = LARGE_NUMBER;

	// If a teleportation is made by the Ghost, a reset of trajectory has benn made. Hence, we can't look from
	// the usual point ad has to set startAtIndex = 0

	if (startAtIndex > GetNumberOfVertices() - 1) {
		startAtIndex = 0;
		index = 0;
	}

	// Find closest line segment
	for (int i = startAtIndex; i < GetNumberOfVertices() - 1; i++) {
		ProjectPointOnVector2D(xin, yin, vertex_[i].x, vertex_[i].y, vertex_[i + 1].x, vertex_[i + 1].y,
							   tmpPos.x, tmpPos.y);
		double distTmp = PointDistance2D(xin, yin, tmpPos.x, tmpPos.y);

		bool inside = PointInBetweenVectorEndpoints(tmpPos.x, tmpPos.y, vertex_[i].x, vertex_[i].y,
													vertex_[i + 1].x, vertex_[i + 1].y, sLocal);
		if (!inside) {
			// Find combined longitudinal and lateral distance to line endpoint
			// sLocal represent now (outside line segment) distance to closest line segment end point
			distTmp = sqrt(distTmp * distTmp + sLocal * sLocal);
			if (sLocal < 0) {
				sLocal = 0;
			} else {
				sLocal = vertex_[i + 1].s - vertex_[i].s;
			}
		} else {
			// rescale normalized s
			sLocal *= (vertex_[i + 1].s - vertex_[i].s);
		}

		if (distTmp < distMin) {
			iMin = (int)i;
			sLocalMin = sLocal;
			distMin = distTmp;
		}
	}

	if (distMin < LARGE_NUMBER) {
		EvaluateSegmentByLocalS(iMin, sLocalMin, 0.0, pos);
		index = iMin;
		return 0;
	} else {
		return -1;
	}
}

int PolyLineBase::FindPointAhead(double s_start,
								 double distance,
								 TrajVertex& pos,
								 int& index,
								 int startAtIndex) {
	index = Evaluate(s_start + distance, pos, startAtIndex);

	return 0;
}

TrajVertex* PolyLineBase::GetVertex(int index) {
	if (GetNumberOfVertices() < 1) {
		return nullptr;
	}

	if (index == -1) {
		return &vertex_.back();
	} else {
		return &vertex_[index];
	}
}

void PolyLineBase::Reset() {
	vertex_.clear();
	vIndex_ = 0;
	length_ = 0;
}

void PolyLineShape::AddVertex(Position pos, double time, bool calculateHeading) {
	Vertex* v = new Vertex();
	v->pos_ = pos;
	vertex_.push_back(v);
	pline_.AddVertex({pos.GetTrajectoryS(), pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), time, 0.0, 0.0,
					  calculateHeading});
}

int PolyLineShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	double s = 0;
	int i = 0;

	if (pline_.GetNumberOfVertices() < 1) {
		return -1;
	}

	if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_S && p > pline_.GetVertex(-1)->s
		|| ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME && p > pline_.GetVertex(-1)->time) {
		// end of trajectory
		s = GetLength();
		i = (int)vertex_.size() - 1;
	} else {
		for (; i < vertex_.size() - 1
			   && (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_S && pline_.vertex_[i + 1].s < p
				   || ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME && pline_.vertex_[i + 1].time < p);
			 i++)
			;

		if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME) {
			double a = (p - pline_.vertex_[i].time)
					   / (pline_.vertex_[i + 1].time - pline_.vertex_[i].time);	 // a = interpolation factor
			s = pline_.vertex_[i].s + a * (pline_.vertex_[i + 1].s - pline_.vertex_[i].s);
		} else {
			s = p;
		}
	}

	pline_.Evaluate(s, pos, i);

	return 0;
}

double PolyLineShape::GetStartTime() {
	if (vertex_.size() == 0) {
		return 0.0;
	}

	return pline_.vertex_[0].time;
}

double PolyLineShape::GetDuration() {
	if (vertex_.size() == 0) {
		return 0.0;
	}

	return pline_.vertex_.back().time - pline_.vertex_[0].time;
}

double NurbsShape::CoxDeBoor(double x, int i, int k, const std::vector<double>& t) {
	// Inspiration: Nurbs Curve Example @
	// https://nccastaff.bournemouth.ac.uk/jmacey/OldWeb/RobTheBloke/www/opengl_programming.html

	if (k == 1) {
		if (t[i] <= x && x < t[i + 1]) {
			return 1.0;
		}
		return 0.0;
	}

	double den1 = t[i + k - 1] - t[i];
	double den2 = t[i + k] - t[i + 1];
	double eq1 = 0.0;
	double eq2 = 0.0;

	if (den1 > 0) {
		eq1 = ((x - t[i]) / den1) * CoxDeBoor(x, i, k - 1, t);
	}

	if (den2 > 0) {
		eq2 = (t[i + k] - x) / den2 * CoxDeBoor(x, i + 1, k - 1, t);
	}

	return eq1 + eq2;
}

void NurbsShape::CalculatePolyLine() {
	if (ctrlPoint_.size() < 1) {
		return;
	}
	Position tmpRoadPos;

	// Calculate approximate length - to find a reasonable step length

	length_ = 0;
	double steplen = 1.0;  // steplen in meters
	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		ctrlPoint_[i].pos_.ReleaseRelation();
		ctrlPoint_[i].t_ = knot_[i + order_ - 1];
		if (i > 0) {
			length_ += PointDistance2D(ctrlPoint_[i - 1].pos_.GetX(), ctrlPoint_[i - 1].pos_.GetY(),
									   ctrlPoint_[i].pos_.GetX(), ctrlPoint_[i].pos_.GetY());
		}
	}

	if (length_ == 0) {
		throw std::runtime_error("Nurbs zero length - check controlpoints");
	}

	// Calculate arc length
	double newLength = 0.0;
	int nSteps = (int)(1 + length_ / steplen);
	double p_steplen = knot_.back() / nSteps;
	TrajVertex pos = {0, 0, 0, 0, 0, 0, 0, 0, false};
	TrajVertex oldpos = {0, 0, 0, 0, 0, 0, 0, 0, false};
	TrajVertex tmppos = {0, 0, 0, 0, 0, 0, 0, 0, false};

	pline_.Reset();
	for (int i = 0; i < nSteps + 1; i++) {
		double t = i * p_steplen;
		EvaluateInternal(t, pos);

		// Calulate heading from line segment between this and previous vertices
		if (i < nSteps) {
			EvaluateInternal(t + 0.01 * p_steplen, tmppos);
		} else {
			EvaluateInternal(t - 0.01 * p_steplen, tmppos);
		}

		if (PointDistance2D(tmppos.x, tmppos.y, pos.x, pos.y) < SMALL_NUMBER) {
			// If points conside, use heading from polyline
			pos.calcHeading = false;
		} else {
			if (i < nSteps) {
				pos.h = GetAngleInInterval2PI(atan2(tmppos.y - pos.y, tmppos.x - pos.x));
			} else {
				pos.h = GetAngleInInterval2PI(atan2(pos.y - tmppos.y, pos.x - tmppos.x));
			}
		}

		if (i > 0) {
			newLength += PointDistance2D(pos.x, pos.y, oldpos.x, oldpos.y);
		}
		pos.s = newLength;

		// Find max contributing controlpoint for time interpolation
		for (int j = 0; j < ctrlPoint_.size(); j++) {
			if (d_[j] > dPeakValue_[j]) {
				dPeakValue_[j] = d_[j];
				dPeakT_[j] = t;
			}
		}

		pline_.AddVertex(pos);
		pline_.vertex_[i].p = i * p_steplen;
		oldpos = pos;
		// Resolve Z value - from road elevation
		tmpRoadPos.SetInertiaPos(pos.x, pos.y, pos.h);
		pos.z = tmpRoadPos.GetZ();
		pline_.vertex_[i].z = pos.z;
	}

	// Calculate time interpolations
	int currentCtrlPoint = 0;
	for (int i = 0; i < pline_.vertex_.size(); i++) {
		if (pline_.vertex_[i].p >= dPeakT_[currentCtrlPoint + 1]) {
			currentCtrlPoint = MIN(currentCtrlPoint + 1, (int)(ctrlPoint_.size()) - 2);
		}
		double w = (pline_.vertex_[i].p - dPeakT_[currentCtrlPoint])
				   / (dPeakT_[currentCtrlPoint + 1] - dPeakT_[currentCtrlPoint]);
		pline_.vertex_[i].time
			= ctrlPoint_[currentCtrlPoint].time_
			  + w * (ctrlPoint_[currentCtrlPoint + 1].time_ - ctrlPoint_[currentCtrlPoint].time_);
	}

	length_ = newLength;
}

int NurbsShape::EvaluateInternal(double t, TrajVertex& pos) {
	pos.x = pos.y = 0.0;

	// Find knot span
	t = CLAMP(t, knot_[0], knot_.back() - SMALL_NUMBER);

	double rationalWeight = 0.0;

	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		// calculate the effect of this point on the curve
		d_[i] = CoxDeBoor(t, (int)i, order_, knot_);
		rationalWeight += d_[i] * ctrlPoint_[i].weight_;
	}

	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		if (d_[i] > SMALL_NUMBER) {
			// sum effect of CV on this part of the curve
			pos.x += d_[i] * ctrlPoint_[i].pos_.GetX() * ctrlPoint_[i].weight_ / rationalWeight;
			pos.y += d_[i] * ctrlPoint_[i].pos_.GetY() * ctrlPoint_[i].weight_ / rationalWeight;
		}
	}

	return 0;
}

void NurbsShape::AddControlPoint(Position pos, double time, double weight, bool calcHeading) {
	if (calcHeading == false) {
		LOG_ONCE("Info: Explicit orientation in Nurbs trajectory control points not supported yet");
	}
	ctrlPoint_.push_back(ControlPoint(pos, time, weight, true));
	d_.push_back(0);
	dPeakT_.push_back(0);
	dPeakValue_.push_back(0);
}

void NurbsShape::AddKnots(std::vector<double> knots) {
	knot_ = knots;

	if (knot_.back() < SMALL_NUMBER) {
		return;
	}
}

int NurbsShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	if (order_ < 1 || ctrlPoint_.size() < order_ || GetLength() < SMALL_NUMBER) {
		return -1;
	}

	double s = p;

	if (ptype == TRAJ_PARAM_TYPE_TIME) {
		pline_.Time2S(p, s);
	}

	pline_.Evaluate(s, pos, pline_.vIndex_);

	EvaluateInternal(pos.p, pos);

	return 0;
}

double NurbsShape::GetStartTime() {
	if (ctrlPoint_.size() == 0) {
		return 0.0;
	}

	return ctrlPoint_[0].time_;
}

double NurbsShape::GetDuration() {
	if (ctrlPoint_.size() == 0) {
		return 0.0;
	}

	return ctrlPoint_.back().time_ - ctrlPoint_[0].time_;
}

ClothoidShape::ClothoidShape(roadmanager::Position pos,
							 double curv,
							 double curvPrime,
							 double len,
							 double tStart,
							 double tEnd)
	: Shape(ShapeType::CLOTHOID) {
	pos_ = pos;
	spiral_ = new roadmanager::Spiral(0, pos_.GetX(), pos_.GetY(), pos_.GetH(), len, curv,
									  curv + curvPrime * len);
	t_start_ = tStart;
	t_end_ = tEnd;
	pline_.interpolateHeading_ = true;
}

void ClothoidShape::CalculatePolyLine() {
	// Create polyline placeholder representation
	double stepLen = 1.0;
	int steps = (int)(spiral_->GetLength() / stepLen);
	pline_.Reset();
	TrajVertex v;

	for (size_t i = 0; i < steps + 1; i++) {
		if (i < steps) {
			EvaluateInternal((double)i, v);
		} else {
			// Add endpoint of spiral
			EvaluateInternal(spiral_->GetLength(), v);
		}

		// resolve road coordinates to get elevation at point
		pos_.SetInertiaPos(v.x, v.y, v.h, true);
		v.z = pos_.GetZ();

		v.p = v.s = (double)i;
		v.time = t_start_ + (i * stepLen / spiral_->GetLength()) * t_end_;

		pline_.AddVertex(v);
	}
}

int ClothoidShape::EvaluateInternal(double s, TrajVertex& pos) {
	spiral_->EvaluateDS(s, &pos.x, &pos.y, &pos.h);

	return 0;
}

int ClothoidShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME) {
		if (p >= t_start_ && p <= t_end_) {
			double t = p - t_start_;
			// Transform time parameter value into a s value
			p = GetLength() * (t - t_start_) / (t_end_ - t_start_);
		} else {
			LOG("Requested time %.2f outside range [%.2f, %.2f]", p, t_start_, t_end_);
			p = GetLength();
		}
	} else if (p > GetLength()) {
		p = GetLength();
	}

	pline_.Evaluate(p, pos);

	spiral_->EvaluateDS(p, &pos.x, &pos.y, &pos.h);

	pos.s = p;

	return 0;
}

double ClothoidShape::GetStartTime() {
	return t_start_;
}

double ClothoidShape::GetDuration() {
	return t_end_ - t_start_;
}

int Route::AddWaypoint(Position* position) {
	if (minimal_waypoints_.size() > 0) {
		// Keep only one consecutive waypoint per road
		// Keep first specified waypoint for first road
		// then, for following roads, keep the last waypoint.

		if (position->GetTrackId() == minimal_waypoints_.back().GetTrackId()) {
			if (minimal_waypoints_.size() == 1) {
				// Ignore
				LOG("Ignoring additional waypoint for road %d (s %.2f)", position->GetTrackId(),
					position->GetS());
				all_waypoints_.push_back(*position);
				return -1;
			} else	// at least two road-unique waypoints
			{
				// Keep this, remove previous
				LOG("Removing previous waypoint for same road %d (at s %.2f)",
					minimal_waypoints_.back().GetTrackId(), minimal_waypoints_.back().GetS());
				minimal_waypoints_.pop_back();
			}
		}

		// Check that there is a valid path from previous waypoint
		RoadPath* path = new RoadPath(&minimal_waypoints_.back(), position);
		double dist = 0;

		if (path->Calculate(dist, false) == 0) {
			// Path is found by tracing previous nodes
			RoadPath::PathNode* previous = 0;
			std::vector<RoadPath::PathNode*> nodes;

			if (path->visited_.size() > 0) {
				previous = path->visited_.back()->previous;
				nodes.push_back(path->visited_.back());
				while (previous != nullptr) {
					nodes.push_back(previous);
					previous = previous->previous;
				}
			}

			if (nodes.size() > 1) {
				// Add internal waypoints, one for each road along the path
				for (int i = (int)nodes.size() - 1; i >= 1; i--) {
					// Find out lane ID of the connecting road
					Position connected_pos
						= Position(nodes[i - 1]->fromRoad->GetId(), nodes[i - 1]->fromLaneId, 0, 0);
					all_waypoints_.push_back(*position);
					minimal_waypoints_.push_back(connected_pos);
					LOG("Route::AddWaypoint Added intermediate waypoint %d roadId %d laneId %d",
						(int)minimal_waypoints_.size() - 1, connected_pos.GetTrackId(),
						nodes[i - 1]->fromLaneId);
				}
			}

			length_ += dist;
		} else {
			invalid_route_ = true;
		}
	} else {
		// First waypoint, make it the current position
		currentPos_ = *position;
	}
	all_waypoints_.push_back(*position);
	minimal_waypoints_.push_back(*position);
	LOG("Route::AddWaypoint Added waypoint %d: %d, %d, %.2f", (int)minimal_waypoints_.size() - 1,
		position->GetTrackId(), position->GetLaneId(), position->GetS());

	return 0;
}

void Route::CheckValid() {
	if (invalid_route_) {
		LOG("Warning: Route %s is not valid, will be ignored for the default controller.", getName().c_str());
		minimal_waypoints_.clear();
	}
}

Road* Route::GetRoadAtOtherEndOfConnectingRoad(Road* incoming_road) {
	Road* connecting_road = Position::GetOpenDrive()->GetRoadById(GetTrackId());
	Junction* junction = Position::GetOpenDrive()->GetJunctionById(connecting_road->GetJunction());

	if (junction == 0) {
		LOG("Unexpected: Road %d not a connecting road", connecting_road->GetId());
		return 0;
	}

	return junction->GetRoadAtOtherEndOfConnectingRoad(connecting_road, incoming_road);
}

int Route::GetDirectionRelativeRoad() {
	return GetWayPointDirection(waypoint_idx_);
}

int Route::GetWayPointDirection(int index) {
	if (minimal_waypoints_.size() == 0 || index < 0 || index >= minimal_waypoints_.size()) {
		LOG("Waypoint index %d out of range (%d)", index, minimal_waypoints_.size());
		return 0;
	}

	if (minimal_waypoints_.size() == 1) {
		LOG("Only one waypoint, no direction");
		return 0;
	}

	OpenDrive* od = minimal_waypoints_[index].GetOpenDrive();
	Road* road = od->GetRoadById(minimal_waypoints_[index].GetTrackId());
	if (road == nullptr) {
		LOG("Waypoint %d invalid road id %d!", index, minimal_waypoints_[index].GetTrackId());
		return 0;
	}

	int direction = 0;
	Position* pos2 = nullptr;

	// Looking in the direction of heading
	direction
		= minimal_waypoints_[index].GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0
			  ? -1
			  : 1;

	if (index < minimal_waypoints_.size() - 1) {
		// Looking in the direction of heading
		direction
			= currentPos_.GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0 ? -1 : 1;

		// Look at next waypoint
		pos2 = GetWaypoint(index + 1);
	} else if (index > 0) {
		// Looking in the opposite direction of heading
		direction
			= currentPos_.GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0 ? 1 : -1;

		// Look at previous waypoint
		pos2 = GetWaypoint(index - 1);
	}

	if (direction == 1 && road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))
		|| direction == -1
			   && road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		// Expected case, route direction aligned with waypoint headings
		return 1;
	} else if (road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))
			   && road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		LOG("Road %d connects to both ends of road %d using relative heading of waypoint", pos2->GetTrackId(),
			road->GetId());
		return direction;
	} else if (road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		return 1 * direction;
	} else if (road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		return -1 * direction;
	}

	LOG("Unexpected case, failed to find out direction of route (from road id %d)", road->GetId());

	return direction;
}

void Route::setName(std::string name) {
	this->name_ = name;
}

std::string Route::getName() {
	return name_;
}

void RMTrajectory::Freeze() {
	if (shape_->type_ == Shape::ShapeType::POLYLINE) {
		PolyLineShape* pline = (PolyLineShape*)shape_;

		for (size_t i = 0; i < pline->vertex_.size(); i++) {
			Position* pos = &pline->vertex_[i]->pos_;
			pos->ReleaseRelation();

			if (pline->pline_.vertex_[i].calcHeading) {
				pline->pline_.UpdateVertex((int)i, pos->GetX(), pos->GetY(), pos->GetZ());
			} else {
				pline->pline_.UpdateVertex((int)i, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetH());
			}
		}
	} else if (shape_->type_ == Shape::ShapeType::CLOTHOID) {
		ClothoidShape* clothoid = (ClothoidShape*)shape_;

		clothoid->pos_.ReleaseRelation();

		clothoid->spiral_->SetX(clothoid->pos_.GetX());
		clothoid->spiral_->SetY(clothoid->pos_.GetY());
		clothoid->spiral_->SetHdg(clothoid->pos_.GetH());

		clothoid->CalculatePolyLine();
	} else {
		NurbsShape* nurbs = (NurbsShape*)shape_;

		nurbs->CalculatePolyLine();
	}
}

double RMTrajectory::GetTimeAtS(double s) {
	// Find out corresponding time-value using polyline representation
	TrajVertex v;
	shape_->pline_.Evaluate(s, v);

	return v.time;
}

double RMTrajectory::GetStartTime() {
	return shape_->GetStartTime();
}

double RMTrajectory::GetDuration() {
	return shape_->GetDuration();
}
