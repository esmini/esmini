#include "Position.hpp"

void Position::Init() {
	track_id_ = -1;
	lane_id_ = 0;
	s_ = 0.0;
	s_trajectory_ = 0.0;
	t_trajectory_ = 0.0;
	t_ = 0.0;
	offset_ = 0.0;
	x_ = 0.0;
	y_ = 0.0;
	z_ = 0.0;
	h_ = 0.0;
	p_ = 0.0;
	r_ = 0.0;
	velX_ = 0.0;
	velY_ = 0.0;
	velZ_ = 0.0;
	accX_ = 0.0;
	accY_ = 0.0;
	accZ_ = 0.0;
	h_rate_ = 0.0;
	p_rate_ = 0.0;
	r_rate_ = 0.0;
	h_acc_ = 0.0;
	p_acc_ = 0.0;
	r_acc_ = 0.0;
	h_offset_ = 0.0;
	h_road_ = 0.0;
	h_relative_ = 0.0;
	z_relative_ = 0.0;
	curvature_ = 0.0;
	p_road_ = 0.0;
	p_relative_ = 0.0;
	r_road_ = 0.0;
	r_relative_ = 0.0;
	roadSuperElevationPrim_ = 0.0;
	z_roadPrimPrim_ = 0.0;
	z_roadPrim_ = 0.0;
	rel_pos_ = 0;
	align_h_ = ALIGN_MODE::ALIGN_SOFT;
	align_p_ = ALIGN_MODE::ALIGN_SOFT;
	align_r_ = ALIGN_MODE::ALIGN_SOFT;
	align_z_ = ALIGN_MODE::ALIGN_SOFT;
	type_ = PositionType::NORMAL;
	orientation_type_ = OrientationType::ORIENTATION_ABSOLUTE;
	snapToLaneTypes_ = Lane::LaneType::LANE_TYPE_ANY_DRIVING;
	status_ = 0;
	lockOnLane_ = false;

	z_road_ = 0.0;
	track_idx_ = -1;
	geometry_idx_ = -1;
	lane_section_idx_ = -1;
	lane_idx_ = -1;
	elevation_idx_ = -1;
	super_elevation_idx_ = -1;
	osi_point_idx_ = -1;
	route_ = 0;
	trajectory_ = 0;
}

Position::Position() {
	Init();
}

Position::Position(int track_id, double s, double t) {
	Init();
	SetTrackPos(track_id, s, t);
}

Position::Position(int track_id, int lane_id, double s, double offset) {
	Init();
	SetLanePos(track_id, lane_id, s, offset);
}

Position::Position(double x, double y, double z, double h, double p, double r) {
	Init();
	SetInertiaPos(x, y, z, h, p, r);
}

Position::Position(double x, double y, double z, double h, double p, double r, bool calculateTrackPosition) {
	Init();
	SetInertiaPos(x, y, z, h, p, r, calculateTrackPosition);
}

Position::~Position() {}

bool Position::LoadOpenDrive(const char* filename) {
	return (GetOpenDrive()->LoadOpenDriveFile(filename));
}

OpenDrive* Position::GetOpenDrive() {
	static OpenDrive od;
	return &od;
}

int Position::GotoClosestDrivingLaneAtCurrentPosition() {
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0) {
		LOG("No road %d", track_idx_);
		return -1;
	}

	LaneSection* lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

	if (lane_section == 0) {
		LOG("No lane section for idx %d - keeping current lane setting\n", lane_section_idx_);
		return -1;
	}

	double offset;
	int lane_idx = lane_section->GetClosestLaneIdx(s_, t_, offset, true);

	if (lane_idx == -1) {
		LOG("Failed to find a valid drivable lane");
		return -1;
	}

	lane_id_ = lane_section->GetLaneIdByIdx(lane_idx);

	offset_ = offset;

	return 0;
}

void Position::Track2Lane() {
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0) {
		LOG("Position::Track2Lane Error: No road %d\n", track_idx_);
		return;
	}

	Geometry* geometry = road->GetGeometry(geometry_idx_);
	if (geometry == 0) {
		LOG("Position::Track2Lane Error: No geometry %d\n", geometry_idx_);
		return;
	}

	// Find LaneSection according to s, starting from current
	int lane_section_idx = road->GetLaneSectionIdxByS(s_, lane_section_idx_);
	LaneSection* lane_section = road->GetLaneSectionByIdx(lane_section_idx);
	if (lane_section == 0) {
		LOG("No lane section for idx %d - keeping current lane setting", lane_section_idx_);
		return;
	}

	// Find the closest driving lane within the lane section
	double offset;
	int lane_idx = lane_section->GetClosestLaneIdx(s_, t_, offset, true, snapToLaneTypes_);

	if (lane_idx == -1) {
		LOG("Failed find closest lane");
		return;
	}

	offset_ = offset;
	// Update cache indices
	lane_idx_ = lane_idx;
	lane_id_ = lane_section->GetLaneIdByIdx(lane_idx_);
	lane_section_idx_ = lane_section_idx;
}

Position::ErrorCode Position::XYZH2TrackPos(double x3,
											double y3,
											double z3,
											double h3,
											bool connectedOnly,
											int roadId) {
	// Overall method:
	//   1. Iterate over all roads, looking at OSI points of each lane sections center line (lane 0)
	//   2. Identify line segment (between two OSI points) closest to xyz point
	//   3. Identify which vertex of the line is closest
	//   4. Given the normals of lines on each side of the vertex, identify which line the points projects
	//   onto
	//   5. The s value for projected xyz point on the line segment corresponds to the rate
	//      between angle from xyz point to projected point and the difference of angle normals

	Road *road, *current_road = 0;
	Road* roadMin = 0;
	bool directlyConnected = false;
	double weight
		= 0;  // Add some resistance to switch from current road, applying a stronger bound to current road
	double angle = 0;
	bool search_done = false;
	double closestS = 0;
	int j2, k2, jMin = -1, kMin = -1, jMinLocal, kMinLocal;
	double closestPointDist = INFINITY;
	bool closestPointInside = false;
	bool insideCurrentRoad = false;	 // current postion projects on current road
	double headingDiffMin = INFINITY;
	bool closestPointDirectlyConnected = false;

	if (GetOpenDrive()->GetNumOfRoads() == 0) {
		return ErrorCode::ERROR_GENERIC;
	}

	// First step is to identify closest road and OSI line segment

	size_t nrOfRoads;
	if (route_ && route_->IsValid()) {
		// Route assigned. Iterate over all roads in the route. I.e. check all waypoints road ID.
		nrOfRoads = route_->minimal_waypoints_.size();
	} else {
		// Iterate over all roads in the road network
		nrOfRoads = GetOpenDrive()->GetNumOfRoads();
	}

	if (roadId == -1) {
		current_road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	} else {
		// Look only at specified road
		current_road = GetOpenDrive()->GetRoadByIdx(roadId);
		nrOfRoads = 0;
	}

	for (int i = -1; !search_done && i < (int)nrOfRoads; i++) {
		if (i == -1) {
			// First check current road (from last known position).
			if (current_road) {
				road = current_road;
			} else {
				continue;  // Skip, no current road
			}
		} else {
			if (current_road && i == track_idx_) {
				continue;  // Skip, already checked this one
			} else {
				if (route_ && route_->IsValid()) {
					road = GetOpenDrive()->GetRoadById(route_->minimal_waypoints_[i].GetTrackId());
				} else {
					road = GetOpenDrive()->GetRoadByIdx(i);
				}
				if (connectedOnly) {
					// Check whether the road is reachble from current position
					Position tmpPos(road->GetId(), 0.0, 0.0);
					PositionDiff posDiff;
					if (Delta(&tmpPos, posDiff) == false) {
						continue;  // skip unreachable road
					}
				}
			}
		}

		// Check whether complete road is too far away - then skip to next
		const double potentialWidthOfRoad = 25;
		if (PointDistance2D(x3, y3, road->GetGeometry(0)->GetX(), road->GetGeometry(0)->GetY())
				- (road->GetLength() + potentialWidthOfRoad)
			> closestPointDist)	 // add potential width of the road
		{
			continue;
		}

		weight = 0;

		// Add resistance to leave current road or directly connected ones
		// actual weights are totally unscientific... up to tuning
		if (road != current_road) {
			if (current_road && current_road->IsDirectlyConnected(road)) {
				directlyConnected = true;
			} else {
				weight += 3;  // For non connected roads add additional "penalty" threshold
				directlyConnected = false;
			}
		}

		// First find distance from current position
		double distFromCurrentPos = GetLengthOfLine2D(x3, y3, GetX(), GetY());
		int startLaneSecIdx = 0;
		if (road == current_road && distFromCurrentPos < 5) {
			// If new point is close to current/old, then start look from current OSI points
			startLaneSecIdx = -1;
		}

		for (int j = startLaneSecIdx; j < road->GetNumberOfLaneSections() && !search_done; j++) {
			int lsec_idx;
			OSIPoints* osiPoints;
			if (j == -1) {
				// new point is close, start look at current/old lane section
				lsec_idx = MAX(0, lane_section_idx_);
			} else {
				lsec_idx = j;
			}
			osiPoints = road->GetLaneSectionByIdx(lsec_idx)->GetLaneById(0)->GetOSIPoints();
			double sLocal = -1;

			// skip last point on last lane section
			int numPoints = osiPoints->GetNumOfOSIPoints();
			if (lsec_idx == road->GetNumberOfLaneSections() - 1) {
				numPoints--;
			}

			// Find closest line or point
			int pointIdxStart, pointIdxEnd;
			if (j == -1) {
				// Start looking in neigborhood of current pos
				pointIdxStart = MAX(0, osi_point_idx_ - 10);
				pointIdxEnd = MIN(numPoints, osi_point_idx_ + 10);
			} else {
				// Then look all over
				pointIdxStart = 0;
				pointIdxEnd = numPoints;
			}

			for (int k = pointIdxStart; k < pointIdxEnd; k++) {
				double distTmp = 0;
				PointStruct& osi_point = osiPoints->GetPoint(k);
				double z = osi_point.z;
				bool inside = false;

				// in case of multiple roads with the same reference line, also look at width of the road of
				// relevant side side of road is determined by cross product of position (relative OSI point)
				// and road heading
				double cp = GetCrossProduct2D(cos(osi_point.h), sin(osi_point.h), x3 - osi_point.x,
											  y3 - osi_point.y);
				double width = road->GetWidth(osi_point.s, SIGN(cp), ~Lane::LaneType::LANE_TYPE_NONE);

				double x2, y2, z2, sLocalTmp;

				jMinLocal = lsec_idx;
				kMinLocal = k;

				double px, py;

				if (k == osiPoints->GetNumOfOSIPoints() - 1) {
					// End of lane section, look into next one
					j2 = MIN(lsec_idx + 1, road->GetNumberOfLaneSections() - 1);
					k2 = MIN(
						1, road->GetLaneSectionByIdx(j2)->GetLaneById(0)->GetOSIPoints()->GetNumOfOSIPoints()
							   - 1);
				} else {
					k2 = k + 1;
					j2 = lsec_idx;
				}
				x2 = road->GetLaneSectionByIdx(j2)->GetLaneById(0)->GetOSIPoints()->GetPoint(k2).x;
				y2 = road->GetLaneSectionByIdx(j2)->GetLaneById(0)->GetOSIPoints()->GetPoint(k2).y;
				z2 = road->GetLaneSectionByIdx(j2)->GetLaneById(0)->GetOSIPoints()->GetPoint(k2).z;

				// OSI points is an approximation of actual geometry
				// Check potential additional area formed by actual normal and OSI points normal
				// at start and end
				if ((lsec_idx == 0 && k == 0)
					|| ((lsec_idx > 1 || k > 1)
						&& (lsec_idx == road->GetNumberOfLaneSections() - 1
							&& k == osiPoints->GetNumOfOSIPoints() - 2))) {
					double x, y, h;
					Position pos;

					if (lsec_idx == 0 && k == 0) {
						// road startpoint
						pos.SetLanePos(road->GetId(), 0, 0, 0);
					} else {
						// road endpoint
						pos.SetLanePos(road->GetId(), 0, road->GetLength(), 0);
					}
					x = pos.GetX();
					y = pos.GetY();
					h = pos.GetH();

					// Calculate actual normal
					double n_actual_angle = GetAngleSum(h, M_PI_2);
					double n_actual_x, n_actual_y;
					RotateVec2D(1, 0, n_actual_angle, n_actual_x, n_actual_y);

					// Calculate normal of OSI line
					double h_osi = GetAngleOfVector(x2 - osi_point.x, y2 - osi_point.y);
					double n_osi_angle = GetAngleSum(h_osi, M_PI_2);
					double n_osi_x, n_osi_y;
					RotateVec2D(1, 0, n_osi_angle, n_osi_x, n_osi_y);

					// Calculate vector from road endpoint (first or last) to obj pos
					double vx = x3 - x;
					double vy = y3 - y;

					// make sure normals points same side as point
					double forward_x = 1.0;
					double forward_y = 0.0;
					RotateVec2D(1.0, 0.0, h, forward_x, forward_y);
					if (GetCrossProduct2D(forward_x, forward_y, vx, vy) < 0) {
						n_actual_x = -n_actual_x;
						n_actual_y = -n_actual_y;
						n_osi_x = -n_osi_x;
						n_osi_y = -n_osi_y;
					}

					double cp_actual = GetCrossProduct2D(vx, vy, n_actual_x, n_actual_y);
					double cp_osi = GetCrossProduct2D(vx, vy, n_osi_x, n_osi_y);

					if (SIGN(cp_actual) != SIGN(cp_osi)) {
						inside = true;
						distTmp = GetLengthOfLine2D(vx, vy, 0, 0);
						jMinLocal = lsec_idx;
						kMinLocal = k;
					}
				}

				if (!inside) {
					// Ok, now look along the OSI lines, between the OSI points along the road centerline

					ProjectPointOnVector2D(x3, y3, osi_point.x, osi_point.y, x2, y2, px, py);
					distTmp = PointDistance2D(x3, y3, px, py);

					inside
						= PointInBetweenVectorEndpoints(px, py, osi_point.x, osi_point.y, x2, y2, sLocalTmp);
					if (!inside && k > pointIdxStart && (SIGN(sLocalTmp) != SIGN(sLocal))) {
						// In between two line segments, or more precisely in the triangle area outside a
						// convex vertex corner between two line segments. Consider beeing inside road
						// segment.
						inside = true;
					}
					sLocal = sLocalTmp;

					// Find closest point of the two
					if (PointSquareDistance2D(x3, y3, osi_point.x, osi_point.y)
						< PointSquareDistance2D(x3, y3, x2, y2)) {
						jMinLocal = lsec_idx;
						kMinLocal = k;
					} else {
						jMinLocal = j2;
						kMinLocal = k2;
					}
				}

				// subtract width of the road
				distTmp = distTmp - width;
				if (distTmp < 0) {
					// On road - distance is zero, but continue search because
					// we could be in a junction where roads are overlapping
					distTmp = 0;
				}

				if (inside) {
					z = (1 - sLocal) * osi_point.z + sLocal * z2;
				} else {
					// Find combined longitudinal and lateral distance to line endpoint
					// sLocal represent now (outside line segment) distance to closest line segment end point
					distTmp = sqrt(distTmp * distTmp + sLocal * sLocal);
				}

				if (fabs(z3 - z) > 2.0) {
					// Add threshold for considering z - to avoid noise in co-planar distance calculations
					distTmp += fabs(z3 - z);
				}

				if (!insideCurrentRoad && road == current_road) {
					// Register whether current position is on current road
					// Allow for 2 meter lateral slack outside road edges
					insideCurrentRoad = inside && distTmp < 2;
				} else if (insideCurrentRoad) {
					// Only add weight if position inside current road
					// longitudinal (end points) and lateral (road width)
					distTmp += weight;
				}
				if (distTmp < closestPointDist + SMALL_NUMBER) {
					bool directlyConnectedCandidate = false;

					if (directlyConnected && closestPointDirectlyConnected) {
						// For directly connected roads (junction), we might have options
						// among equally close ones, find the one which goes the most straight forward
						if (fabs(distTmp - closestPointDist) < SMALL_NUMBER) {
							if (angle < headingDiffMin) {
								directlyConnectedCandidate = true;
							}
						}
					}

					if (directlyConnectedCandidate || distTmp < closestPointDist) {
						closestPointDist = distTmp;
						roadMin = road;
						jMin = jMinLocal;
						kMin = kMinLocal;
						closestPointInside = inside;
						closestPointDirectlyConnected = directlyConnected;
						osi_point_idx_ = kMinLocal;

						if (directlyConnected) {
							headingDiffMin = angle;
						}
					}
				} else if (startLaneSecIdx == -1) {
					// distance is now increasing, indicating that we already passed the closest point
					if (closestPointInside && closestPointDist < SMALL_NUMBER) {
						search_done = true;
						break;
					}
				}
			}
		}
	}

	if (closestPointInside) {
		status_ &= ~static_cast<int>(Position::PositionStatusMode::POS_STATUS_END_OF_ROAD);
	} else {
		status_ |= static_cast<int>(Position::PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}

	// The closest OSI vertex has been identified
	// Now, find out exact road s-value based on interpolation of normal angles
	// for the two lines having the vertex in common

	if (roadMin == 0) {
		LOG("Error finding minimum distance\n");
		return ErrorCode::ERROR_GENERIC;
	}

	if (jMin != -1 && kMin != -1) {
		// Find out what line the points projects to, starting or ending with closest point?
		// Do this by comparing the angle to the position with the road normal at found point

		PointStruct osip_closest, osip_first, osip_second;
		osip_closest = roadMin->GetLaneSectionByIdx(jMin)->GetLaneById(0)->GetOSIPoints()->GetPoint(kMin);

		double xTangent = cos(osip_closest.h);
		double yTangent = sin(osip_closest.h);
		double dotP = GetDotProduct2D(xTangent, yTangent, x3 - osip_closest.x, y3 - osip_closest.y);

		int jFirst, jSecond, kFirst, kSecond;

		if (dotP > 0) {
			// Positive dot product means closest OSI point is behind
			osip_first = osip_closest;
			jFirst = jMin;
			kFirst = kMin;

			if (kMin < roadMin->GetLaneSectionByIdx(jMin)->GetLaneById(0)->GetOSIPoints()->GetNumOfOSIPoints()
						   - 1) {
				jSecond = jMin;
				kSecond = kMin + 1;
			} else {
				if (jMin < roadMin->GetNumberOfLaneSections() - 1) {
					jSecond = jMin + 1;
					if (roadMin->GetLaneSectionByIdx(jSecond)
							->GetLaneById(0)
							->GetOSIPoints()
							->GetNumOfOSIPoints()
						> 1) {
						kSecond = 1;  // Skip first point, it's the same as last in last lane section
					} else {
						kSecond = 0;  // Only one point available in lane section - don't go further
					}
				} else {
					// Last point
					jSecond = jMin;
					kSecond = kMin;
				}
			}
			osip_second
				= roadMin->GetLaneSectionByIdx(jSecond)->GetLaneById(0)->GetOSIPoints()->GetPoint(kSecond);
		} else {
			// Negative dot product means closest OSI point is ahead
			osip_second = osip_closest;
			jSecond = jMin;
			kSecond = kMin;

			if (kMin > 0) {
				jFirst = jMin;
				kFirst = kMin - 1;
			} else {
				if (jMin > 0) {
					jFirst = jMin - 1;
					if (roadMin->GetLaneSectionByIdx(jFirst)
							->GetLaneById(0)
							->GetOSIPoints()
							->GetNumOfOSIPoints()
						> 1) {
						// Skip last point, it's the same as first in successor lane section
						kFirst = roadMin->GetLaneSectionByIdx(jFirst)
									 ->GetLaneById(0)
									 ->GetOSIPoints()
									 ->GetNumOfOSIPoints()
								 - 2;
					} else {
						// Only one point available in lane section - don't go further
						kFirst = roadMin->GetLaneSectionByIdx(jFirst)
									 ->GetLaneById(0)
									 ->GetOSIPoints()
									 ->GetNumOfOSIPoints()
								 - 1;
					}
				} else {
					// First point
					jFirst = jMin;
					kFirst = kMin;
				}
			}
			osip_first
				= roadMin->GetLaneSectionByIdx(jFirst)->GetLaneById(0)->GetOSIPoints()->GetPoint(kFirst);
		}

		if (jFirst == jSecond && kFirst == kSecond) {
			// Same point
			closestS = osip_first.s;
		} else {
			// Different points
			double angleBetweenNormals, angleToPosition;
			double normalIntersectionX, normalIntersectionY;
			double sNorm = 0;

			// Check for straight line
			if (fabs(osip_first.h - osip_second.h)
				< 1e-5)	 // Select threshold to avoid precision issues in calculations
			{
				double px, py;
				ProjectPointOnVector2D(x3, y3, osip_first.x, osip_first.y, osip_second.x, osip_second.y, px,
									   py);

				// Find relative position of projected point on line segment
				double l1 = GetLengthOfLine2D(osip_first.x, osip_first.y, px, py);
				double l2 = GetLengthOfLine2D(osip_first.x, osip_first.y, osip_second.x, osip_second.y);
				sNorm = l1 / l2;
			} else {
				// Find normals at end points of line segment
				double xn0, yn0, xn1, yn1;
				RotateVec2D(cos(osip_first.h), sin(osip_first.h), M_PI_2, xn0, yn0);
				RotateVec2D(cos(osip_second.h), sin(osip_second.h), M_PI_2, xn1, yn1);

				// Find intersection of extended normals
				GetIntersectionOfTwoLineSegments(osip_first.x, osip_first.y, osip_first.x + xn0,
												 osip_first.y + yn0, osip_second.x, osip_second.y,
												 osip_second.x + xn1, osip_second.y + yn1,
												 normalIntersectionX, normalIntersectionY);

				// Align normal vectors to direction from intersection towards line segment
				NormalizeVec2D(osip_first.x - normalIntersectionX, osip_first.y - normalIntersectionY, xn0,
							   yn0);
				NormalizeVec2D(osip_second.x - normalIntersectionX, osip_second.y - normalIntersectionY, xn1,
							   yn1);

				// Find angle between normals
				angleBetweenNormals = acos(GetDotProduct2D(-xn0, -yn0, -xn1, -yn1));

				// Find angle between the two vectors:
				// 1. line between normals intersection and the point of query
				// 2. Normal in the first point of closest line segment (turned around to match direction of
				// first line)
				double lx = normalIntersectionX - x3;
				double ly = normalIntersectionY - y3;
				double lLength = sqrt(lx * lx + ly * ly);
				angleToPosition
					= acos(CLAMP(GetDotProduct2D(-xn0, -yn0, lx / lLength, ly / lLength), -1.0, 1.0));

				// Finally calculate interpolation factor
				if (fabs(angleBetweenNormals) < SMALL_NUMBER) {
					sNorm = 0.0;
				} else {
					sNorm = angleToPosition / angleBetweenNormals;
				}

				// printf("road_id %d jMin %d kMin %d lx %.2f ly %.2f angle0 %.2f angle1 %.2f
				// normalIntersectionX %.2f normalIntersectionY %.2f sNorm %.2f\n", 	roadMin->GetId(),
				// jMin,
				// kMin, lx, ly, angleToPosition, angleBetweenNormals, normalIntersectionX,
				// normalIntersectionY, sNorm);
			}

			closestS = (1 - sNorm) * osip_first.s + sNorm * osip_second.s;
			closestS = CLAMP(closestS, 0, roadMin->GetLength());
		}
	} else {
		LOG("Unexpected: No closest OSI point found!");
	}

	double fixedLaneOffset = 0;
	int fixedLaneId = 0;
	if (lockOnLane_) {
		// Register lateral position of previous lane
		LaneSection* lsec = current_road->GetLaneSectionByIdx(lane_section_idx_);
		if (lsec) {
			fixedLaneOffset = SIGN(lane_id_) * lsec->GetCenterOffset(s_, lane_id_);

			// Now find cloest lane at that lateral position, at updated s value
			double laneOffset;
			int lane_idx = lsec->GetClosestLaneIdx(closestS, fixedLaneOffset, laneOffset, true,
												   Lane::LaneType::LANE_TYPE_ANY_DRIVING);
			fixedLaneId = lsec->GetLaneIdByIdx(lane_idx);
		}
	}

	// Set position exact on center line
	ErrorCode retvalue = SetTrackPos(roadMin->GetId(), closestS, 0, true);

	double xCenterLine = x_;
	double yCenterLine = y_;

	// Find out actual lateral position
	double latOffset = PointToLineDistance2DSigned(
		x3, y3, xCenterLine, yCenterLine, xCenterLine + cos(GetHRoad()), yCenterLine + sin(GetHRoad()));

	// Update lateral offsets
	if (lockOnLane_) {
		SetLanePos(roadMin->GetId(), fixedLaneId, closestS, latOffset - fixedLaneOffset);
	} else {
		SetTrackPos(roadMin->GetId(), closestS, latOffset, false);
	}

	static int rid = 0;
	if (roadMin->GetId() != rid) {
		rid = roadMin->GetId();
	}

	// Set specified position and heading
	SetX(x3);
	SetY(y3);
	SetHeading(h3);

	EvaluateRoadZPitchRoll();

	if (!closestPointInside) {
		// if outside road endpoint boundries, ignore road pitch
		p_road_ = 0.0;
		SetPitch(0.0);
	}

	// If on a route, calculate corresponding route position
	if (route_ && route_->IsValid()) {
		CalcRoutePosition();
	}

	return retvalue;
}

bool Position::EvaluateRoadZPitchRoll() {
	if (track_id_ < 0) {
		return false;
	}

	bool ret_value = false;

	Road* road = GetRoadById(track_id_);
	if (road != nullptr) {
		ret_value
			= road->GetZAndPitchByS(s_, &z_road_, &z_roadPrim_, &z_roadPrimPrim_, &p_road_, &elevation_idx_);
		ret_value &= road->UpdateZAndRollBySAndT(s_, t_, &z_road_, &roadSuperElevationPrim_, &r_road_,
												 &super_elevation_idx_);
	} else {
		LOG("Failed to lookup road id %d", track_id_);
	}

	if (align_z_ == ALIGN_MODE::ALIGN_SOFT) {
		z_ = z_road_ + z_relative_;
	} else if (align_z_ == ALIGN_MODE::ALIGN_HARD) {
		z_ = z_road_;
	} else {
		z_ = z_relative_;
	}

	return ret_value;
}

Position::ErrorCode Position::Track2XYZ() {
	if (GetOpenDrive()->GetNumOfRoads() == 0) {
		return ErrorCode::ERROR_GENERIC;
	}

	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0) {
		LOG("Position::Track2XYZ Error: No road %d\n", track_idx_);
		return ErrorCode::ERROR_GENERIC;
	}

	Geometry* geometry = road->GetGeometry(geometry_idx_);
	if (geometry == 0) {
		LOG("Position::Track2XYZ Error: No geometry %d\n", geometry_idx_);
		return ErrorCode::ERROR_GENERIC;
	}

	geometry->EvaluateDS(s_ - geometry->GetS(), &x_, &y_, &h_road_);

	// Consider lateral t position, perpendicular to track heading
	double x_local = (t_ + road->GetLaneOffset(s_)) * cos(h_road_ + M_PI_2);
	double y_local = (t_ + road->GetLaneOffset(s_)) * sin(h_road_ + M_PI_2);

	h_road_ += atan(road->GetLaneOffsetPrim(s_)) + h_offset_;
	h_road_ = GetAngleInInterval2PI(h_road_);

	x_ += x_local;
	y_ += y_local;

	// z = Elevation
	EvaluateRoadZPitchRoll();

	EvaluateOrientation();

	return ErrorCode::ERROR_NO_ERROR;
}

void Position::LaneBoundary2Track() {
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	t_ = 0;

	if (road != 0 && road->GetNumberOfLaneSections() > 0) {
		LaneSection* lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		if (lane_section != 0 && lane_id_ != 0) {
			t_ = offset_ + lane_section->GetOuterOffset(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
			h_offset_ = lane_section->GetOuterOffsetHeading(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
		}
	}
}

void Position::Lane2Track() {
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	t_ = 0;

	if (road != 0 && road->GetNumberOfLaneSections() > 0) {
		LaneSection* lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		if (lane_section != 0) {
			t_ = offset_ + lane_section->GetCenterOffset(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
			h_offset_ = lane_section->GetCenterOffsetHeading(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
		}
	}
}

void Position::RoadMark2Track() {
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	t_ = 0;

	if (road != 0 && road->GetNumberOfLaneSections() > 0) {
		LaneSection* lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		if (lane_section != 0 && lane_id_ != 0) {
			t_ = offset_ + lane_section->GetOuterOffset(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
			h_offset_ = lane_section->GetOuterOffsetHeading(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
		}

		Lane* lane = lane_section->GetLaneByIdx(lane_idx_);
		LaneRoadMark* lane_roadmark = lane->GetLaneRoadMarkByIdx(roadmark_idx_);
		LaneRoadMarkType* lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(roadmarktype_idx_);
		LaneRoadMarkTypeLine* lane_roadmarktypeline
			= lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(roadmarkline_idx_);

		if (lane_roadmarktypeline != 0) {
			t_ = t_ + lane_roadmarktypeline->GetTOffset();
		}
	}
}

void Position::XYZ2Track() {
	XYZH2TrackPos(x_, y_, z_, h_);
}

Position::ErrorCode Position::SetLongitudinalTrackPos(int track_id, double s) {
	Road* road;

	if (GetOpenDrive()->GetNumOfRoads() == 0) {
		return ErrorCode::ERROR_GENERIC;
	}

	if ((road = GetOpenDrive()->GetRoadById(track_id)) == 0) {
		LOG("Position::Set Error: track %d not found", track_id);

		// Just hard code values and return
		track_id_ = track_id;
		s_ = s;

		return ErrorCode::ERROR_GENERIC;
	}
	if (track_id != track_id_) {
		// update internal track and geometry indices
		track_id_ = track_id;
		track_idx_ = GetOpenDrive()->GetTrackIdxById(track_id);
		geometry_idx_ = 0;
		elevation_idx_ = 0;
		super_elevation_idx_ = 0;
		lane_section_idx_ = 0;
		lane_id_ = 0;
		lane_idx_ = 0;
		osi_point_idx_ = 0;
	}

	Geometry* geometry = road->GetGeometry(geometry_idx_);
	// check if still on same geometry
	if (s > geometry->GetS() + geometry->GetLength()) {
		while (s > geometry->GetS() + geometry->GetLength()
			   && geometry_idx_ < road->GetNumberOfGeometries() - 1) {
			// Move to next geometry
			geometry = road->GetGeometry(++geometry_idx_);
		}
	} else if (s < geometry->GetS()) {
		while (s < geometry->GetS() && geometry_idx_ > 0) {
			// Move to previous geometry
			geometry = road->GetGeometry(--geometry_idx_);
		}
	}

	if (s > road->GetLength()) {
		if (s > road->GetLength() + SMALL_NUMBER) {
			LOG("Position::Set Warning: s (%.2f) too large, track %d only %.2f m long\n", s, track_id_,
				road->GetLength());
		}
		s_ = road->GetLength();
		status_ |= static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROAD);
		return ErrorCode::ERROR_END_OF_ROAD;
	} else {
		s_ = s;
	}

	if (s < SMALL_NUMBER || s > road->GetLength() - SMALL_NUMBER) {
		status_ |= static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROAD);
	} else {
		status_ &= ~static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}

	return ErrorCode::ERROR_NO_ERROR;
}

Position::ErrorCode Position::SetTrackPos(int track_id, double s, double t, bool UpdateXY) {
	ErrorCode retval_long = SetLongitudinalTrackPos(track_id, s);

	if (retval_long != ErrorCode::ERROR_GENERIC) {
		t_ = t;
		Track2Lane();
		if (UpdateXY) {
			ErrorCode retval_lat = Track2XYZ();
			if (retval_lat != ErrorCode::ERROR_NO_ERROR) {
				return retval_lat;
			}
		}
	}
	return retval_long;
}

void Position::ForceLaneId(int lane_id) {
	if (lane_idx_ < 0 || lane_section_idx_ < 0) {
		return;
	}
	// find out lateral distance between current and target lane
	Road* road = GetRoadById(GetTrackId());

	double lat_dist
		= road->GetLaneSectionByIdx(lane_section_idx_)->GetOffsetBetweenLanes(lane_id_, lane_id, GetS());

	lane_id_ = lane_id;
	offset_ -= lat_dist;

	// Update track position (t) as well
	Lane2Track();
}

int Position::TeleportTo(Position* position) {
	roadmanager::Position tmpPos;

	if (position->GetRelativePosition() == this) {
		// Special case: Relation short circuit - need to make a copy before reseting
		tmpPos.CopyRMPos(this);

		position->SetRelativePosition(&tmpPos, position->GetType());
	}

	CopyRMPos(position);

	// Resolve any relative positions
	ReleaseRelation();

	if (GetRoute())	 // on a route
	{
		CalcRoutePosition();
	}

	return 0;
}

int Position::MoveToConnectingRoad(RoadLink* road_link,
								   ContactPointType& contact_point_type,
								   double junctionSelectorAngle) {
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	Road* next_road = 0;
	LaneSection* lane_section;
	Lane* lane;
	int new_lane_id = 0;

	if (road == 0) {
		LOG("Invalid road id %d\n", road->GetId());
		return -1;
	}

	if (road_link->GetElementId() == -1) {
		LOG("No connecting road or junction at rid %d link_type %s", road->GetId(),
			LinkType2Str(road_link->GetType()).c_str());
		return -1;
	}

	// Get lane info from current road
	lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	if (lane_section == 0) {
		LOG("No lane section rid %d ls_idx %d link_type  %s", road->GetId(), lane_section_idx_,
			LinkType2Str(road_link->GetType()).c_str());
		return -1;
	}

	lane = lane_section->GetLaneByIdx(lane_idx_);
	if (lane == 0) {
		LOG("No lane rid %d lidx %d nlanes %d link_type %s lsecidx %d\n", road->GetId(), lane_idx_,
			lane_section->GetNumberOfLanes(), LinkType2Str(road_link->GetType()).c_str(), lane_section_idx_);
		return -1;
	}

	if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD) {
		LaneLink* lane_link = lane->GetLink(road_link->GetType());
		if (lane_link != 0) {
			new_lane_id = lane->GetLink(road_link->GetType())->GetId();
			if (new_lane_id == 0) {
				LOG("Road+ new lane id %d\n", new_lane_id);
			}
		} else {
			// LOG("No lane link from rid %d lid %d to rid %d", GetTrackId(), GetLaneId(),
			// road_link->GetElementId());
		}
		contact_point_type = road_link->GetContactPointType();
		next_road = GetOpenDrive()->GetRoadById(road_link->GetElementId());
	} else if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION) {
		Junction* junction = GetOpenDrive()->GetJunctionById(road_link->GetElementId());

		if (junction == 0) {
			LOG("Error: junction %d not existing\n", road_link->GetElementType());
			return -1;
		}

		int connection_idx = 0;
		int n_connections = junction->GetNumberOfRoadConnections(road->GetId(), lane->GetId());

		if (n_connections == 0) {
			//			LOG("No connections from road id %d lane id %d in junction %d", road->GetId(),
			// lane->GetId(), junction->GetId());
			return -1;
		} else if (n_connections == 1) {
			connection_idx = 0;
		} else {
			// find valid connecting road, if multiple choices choose either most straight one OR by random
			if (GetRoute()) {
				// Choose direction of the route
				Route* r = GetRoute();

				// Find next road in route
				Road* outgoing_road_target = r->GetRoadAtOtherEndOfConnectingRoad(road);
				for (int i = 0; i < n_connections; i++) {
					LaneRoadLaneConnection lane_road_lane_connection
						= junction->GetRoadConnectionByIdx(road->GetId(), lane->GetId(), i, snapToLaneTypes_);
					Road* connecting_road
						= GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId());
					if (connecting_road) {
						Road* outgoing_road
							= junction->GetRoadAtOtherEndOfConnectingRoad(connecting_road, road);
						if (outgoing_road == outgoing_road_target) {
							connection_idx = i;
						}
					}
				}
			} else if (junctionSelectorAngle >= 0.0) {
				// Find the straighest link
				int best_road_index = 0;
				double min_heading_diff = 1E10;	 // set huge number
				for (int i = 0; i < n_connections; i++) {
					LaneRoadLaneConnection lane_road_lane_connection
						= junction->GetRoadConnectionByIdx(road->GetId(), lane->GetId(), i, snapToLaneTypes_);
					next_road = GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId());

					// Get a position at end of the connecting road
					Position test_pos;
					double outHeading = 0.0;
					if (lane_road_lane_connection.contact_point_ == CONTACT_POINT_START) {
						test_pos.SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), 0);
						outHeading = test_pos.GetHRoad();
					} else if (lane_road_lane_connection.contact_point_ == CONTACT_POINT_END) {
						test_pos.SetLanePos(next_road->GetId(), new_lane_id, 0, 0);
						outHeading = GetAngleSum(test_pos.GetHRoad(), M_PI);
					} else {
						LOG("Unexpected contact point type: %d", road_link->GetContactPointType());
					}

					// Compare heading angle difference, find smallest
					double deltaHeading
						= GetAngleInInterval2PI(GetAngleDifference(outHeading, GetHRoadInDrivingDirection()));
					double heading_diff = GetAbsAngleDifference(deltaHeading, junctionSelectorAngle);
					if (heading_diff < min_heading_diff) {
						min_heading_diff = heading_diff;
						best_road_index = i;
					}
				}
				connection_idx = best_road_index;
			} else	// randomize
			{
				connection_idx = (int)(n_connections * (double)(SE_Env::Inst().GetGenerator())()
									   / (SE_Env::Inst().GetGenerator()).max());
			}
		}

		LaneRoadLaneConnection lane_road_lane_connection = junction->GetRoadConnectionByIdx(
			road->GetId(), lane->GetId(), connection_idx, snapToLaneTypes_);
		contact_point_type = lane_road_lane_connection.contact_point_;

		new_lane_id = lane_road_lane_connection.GetConnectinglaneId();
		next_road = GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId());
	}

	if (next_road == 0) {
		LOG("No next road\n");
		return -1;
	}

	if (new_lane_id == 0) {
		LOG("No connection from rid %d lid %d -> rid %d eltype %d - try moving to closest lane\n",
			road->GetId(), lane->GetId(), road_link->GetElementId(), road_link->GetElementType());

		// Find closest lane on new road - by convert to track pos and then set lane offset = 0
		if (road_link->GetContactPointType() == CONTACT_POINT_START) {
			SetTrackPos(next_road->GetId(), 0, GetT(), false);
		} else if (road_link->GetContactPointType() == CONTACT_POINT_END) {
			SetTrackPos(next_road->GetId(), next_road->GetLength(), GetT(), false);
		}
		offset_ = 0;

		return 0;
	}

	double new_offset = offset_;
	if ((road_link->GetType() == LinkType::PREDECESSOR
		 && contact_point_type == ContactPointType::CONTACT_POINT_START)
		|| (road_link->GetType() == LinkType::SUCCESSOR
			&& contact_point_type == ContactPointType::CONTACT_POINT_END)) {
		h_relative_ = GetAngleSum(h_relative_, M_PI);
		new_offset = -offset_;
	}

	// Find out if connecting to start or end of new road
	if (road_link->GetContactPointType() == CONTACT_POINT_START) {
		// Specify first (0) lane section
		SetLanePos(next_road->GetId(), new_lane_id, 0, new_offset, 0);
	} else if (road_link->GetContactPointType() == CONTACT_POINT_END) {
		// Find out and specify last lane section
		SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), new_offset,
				   GetRoadById(road_link->GetElementId())->GetNumberOfLaneSections() - 1);
	} else if (road_link->GetContactPointType() == CONTACT_POINT_JUNCTION
			   && road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION) {
		if (contact_point_type == CONTACT_POINT_START) {
			SetLanePos(next_road->GetId(), new_lane_id, 0, new_offset);
		} else if (contact_point_type == CONTACT_POINT_END) {
			SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), new_offset);
		} else {
			LOG("Unexpected contact point: %d\n", contact_point_type);
		}
	} else {
		LOG("Unsupported contact point type %d\n", road_link->GetContactPointType());
		return -1;
	}

	return 0;
}

double Position::DistanceToDS(double ds) {
	// Add or subtract stepsize according to curvature and offset, in order to keep constant speed
	double curvature = GetCurvature();
	double offset = GetT();

	// Also compensate for any lane offset at current road position (if available)
	if (GetOpenDrive()) {
		roadmanager::Road* road = GetOpenDrive()->GetRoadById(GetTrackId());
		if (road != nullptr) {
			offset += road->GetLaneOffset(GetS());
		}
	}

	if (abs(curvature) > SMALL_NUMBER) {
		// Approximate delta length by sampling curvature in current position
		if (curvature * offset > 1.0 - SMALL_NUMBER) {
			// Radius not large enough for offset, probably being closer to another road segment
			XYZH2TrackPos(GetX(), GetY(), GetY(), GetH(), true);
			SetHeadingRelative(GetHRelative());
			curvature = GetCurvature();
			offset = GetT();
		}
		double stepScaleFactor = 1 / (1 - curvature * offset);
		ds *= stepScaleFactor;
	}

	return ds;
}

Position::ErrorCode Position::MoveAlongS(double ds,
										 double dLaneOffset,
										 double junctionSelectorAngle,
										 bool actualDistance) {
	RoadLink* link;
	int max_links = 8;	// limit lookahead through junctions/links
	ContactPointType contact_point_type;

	if (actualDistance) {
		ds = DistanceToDS(ds);
	}

	if (type_ == PositionType::RELATIVE_LANE) {
		// Create a temporary position to evaluate in relative lane coordinates
		Position pos = *this->rel_pos_;

		// First move position along s
		pos.MoveAlongS(ds);

		// Then move laterally
		pos.SetLanePos(pos.track_id_, pos.lane_id_ + this->lane_id_, pos.s_, pos.offset_ + this->offset_);

		this->x_ = pos.x_;
		this->y_ = pos.y_;
		this->z_ = pos.z_;
		this->h_ = pos.h_;
		this->p_ = pos.p_;
		this->r_ = pos.r_;

		return Position::ErrorCode::ERROR_NO_ERROR;
	}

	if (GetOpenDrive()->GetNumOfRoads() == 0 || track_idx_ < 0) {
		// No roads available or current track undefined
		return Position::ErrorCode::ERROR_NO_ERROR;
	}

	double s_stop = 0;
	double ds_signed = ds * (IsAngleForward(GetHRelative()) ? 1 : -1);
	double signed_dLaneOffset = dLaneOffset;

	// move from road to road until ds-value is within road length or maximum of connections has been crossed
	for (int i = 0; i < max_links; i++) {
		if (s_ + ds_signed > GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength()) {
			// Calculate remaining s-value once we moved to the connected road
			ds_signed = s_ + ds_signed - GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength();
			link = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLink(SUCCESSOR);

			// register s-value at end of the road, to be used in case of bad connection
			s_stop = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength();
		} else if (s_ + ds_signed < 0) {
			// Calculate remaining s-value once we moved to the connected road
			ds_signed = s_ + ds_signed;
			link = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLink(PREDECESSOR);

			// register s-value at end of the road, to be used in case of bad connection
			s_stop = 0;
		} else	// New position is within current track (road)
		{
			break;
		}

		// If link is OK then move to the start- or endpoint of the connected road, depending on contact point
		if (!link || link->GetElementId() == -1
			|| MoveToConnectingRoad(link, contact_point_type, junctionSelectorAngle) != 0) {
			// Failed to find a connection, stay at end of current road
			SetLanePos(track_id_, lane_id_, s_stop, offset_);

			status_ |= static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROAD);
			return ErrorCode::ERROR_END_OF_ROAD;
		}

		// Adjust sign of ds based on connection point
		if (contact_point_type == ContactPointType::CONTACT_POINT_END) {
			ds_signed = -fabs(ds_signed);
			signed_dLaneOffset = -dLaneOffset;
		} else {
			ds_signed = fabs(ds_signed);
			signed_dLaneOffset = dLaneOffset;
		}
	}

	// Finally, update the position with the adjusted s and offset values
	SetLanePos(track_id_, lane_id_, s_ + ds_signed, offset_ + signed_dLaneOffset);

	// Check if lane has narrowed down to zero width
	Road* road = GetOpenDrive()->GetRoadById(track_id_);
	LaneInfo li = road->GetLaneInfoByS(GetS(), lane_section_idx_, lane_id_, snapToLaneTypes_);
	if (road->GetLaneWidthByS(GetS(), li.lane_id_) < SMALL_NUMBER) {
		double offset = 0;
		int old_lane_id = lane_id_;
		int new_lane_idx = road->GetLaneSectionByIdx(li.lane_section_idx_)
							   ->GetClosestLaneIdx(GetS(), GetT(), offset, true, snapToLaneTypes_);
		int new_lane_id
			= road->GetLaneSectionByIdx(li.lane_section_idx_)->GetLaneByIdx(new_lane_idx)->GetId();
		SetLanePos(track_id_, new_lane_id, GetS(), 0);
		LOG("Lane %d on road %d is or became zero width, moved to closest available lane: %d", road->GetId(),
			old_lane_id, GetLaneId());
	}

	if (s_ < SMALL_NUMBER || s_ > road->GetLength() - SMALL_NUMBER) {
		status_ |= static_cast<int>(Position::PositionStatusMode::POS_STATUS_END_OF_ROAD);
	} else {
		status_ &= ~static_cast<int>(Position::PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}

	return Position::ErrorCode::ERROR_NO_ERROR;
}

Position::ErrorCode Position::SetLanePos(int track_id,
										 int lane_id,
										 double s,
										 double offset,
										 int lane_section_idx) {
	offset_ = offset;
	ErrorCode retvalue = ErrorCode::ERROR_NO_ERROR;

	if ((retvalue = SetLongitudinalTrackPos(track_id, s)) == ErrorCode::ERROR_GENERIC) {
		lane_id_ = lane_id;
		offset_ = offset;
		return retvalue;
	}

	Road* road = GetOpenDrive()->GetRoadById(track_id);
	if (road == 0) {
		LOG("Position::Set Error: track %d not available", track_id);
		lane_id_ = lane_id;
		offset_ = offset;
		return ErrorCode::ERROR_GENERIC;
	}

	if (lane_id != lane_id_ && lane_section_idx == -1) {
		// New lane ID might indicate a discreet jump to a new, distant position, reset lane section, if not
		// specified in func parameter)
		lane_section_idx = road->GetLaneSectionIdxByS(s);
	}

	LaneSection* lane_section = 0;
	if (lane_section_idx > -1)	// If lane section was specified or reset
	{
		lane_section_idx_ = lane_section_idx;
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		lane_id_ = lane_id;
	} else	// Find LaneSection and info according to s
	{
		LaneInfo lane_info = road->GetLaneInfoByS(s_, lane_section_idx_, lane_id_, snapToLaneTypes_);
		lane_section_idx_ = lane_info.lane_section_idx_;
		lane_id_ = lane_info.lane_id_;

		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	}

	if (lane_section != 0) {
		lane_idx_ = lane_section->GetLaneIdxById(lane_id_);
		if (lane_idx_ == -1) {
			LOG("lane_idx %d fail for lane id %d\n", lane_idx_, lane_id_);
			lane_idx_ = 0;
		}
	} else {
		LOG("Position::Set (lanepos) Error - lanesection NULL lsidx %d rid %d lid %d\n", lane_section_idx_,
			road->GetId(), lane_id_);
	}

	Lane2Track();
	Track2XYZ();

	return retvalue;
}

void Position::SetLaneBoundaryPos(int track_id, int lane_id, double s, double offset, int lane_section_idx) {
	offset_ = offset;
	int old_lane_id = lane_id_;
	int old_track_id = track_id_;
	ErrorCode retval;

	if ((retval = SetLongitudinalTrackPos(track_id, s)) != Position::ErrorCode::ERROR_NO_ERROR) {
		lane_id_ = lane_id;
		offset_ = offset;
		return;
	}

	Road* road = GetOpenDrive()->GetRoadById(track_id);
	if (road == 0) {
		LOG("Position::Set Error: track %d not available", track_id);
		lane_id_ = lane_id;
		offset_ = offset;
		return;
	}

	if (lane_id != lane_id_ && lane_section_idx == -1) {
		// New lane ID might indicate a discreet jump to a new, distant position, reset lane section, if not
		// specified in func parameter)
		lane_section_idx = road->GetLaneSectionIdxByS(s);
	}

	LaneSection* lane_section = 0;
	if (lane_section_idx > -1)	// If lane section was specified or reset
	{
		lane_section_idx_ = lane_section_idx;
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		lane_id_ = lane_id;
	} else	// Find LaneSection and info according to s
	{
		LaneInfo lane_info = road->GetLaneInfoByS(s_, lane_section_idx_, lane_id_, snapToLaneTypes_);
		lane_section_idx_ = lane_info.lane_section_idx_;
		lane_id_ = lane_info.lane_id_;

		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	}

	if (lane_section != 0) {
		lane_idx_ = lane_section->GetLaneIdxById(lane_id_);
		if (lane_idx_ == -1) {
			LOG("lane_idx %d fail for lane id %d\n", lane_idx_, lane_id_);
			lane_idx_ = 0;
		}
	} else {
		LOG("Position::Set (lanepos) Error - lanesection NULL lsidx %d rid %d lid %d\n", lane_section_idx_,
			road->GetId(), lane_id_);
	}

	// Check road direction when on new track
	if (old_lane_id != 0 && lane_id_ != 0 && track_id_ != old_track_id
		&& SIGN(lane_id_) != SIGN(old_lane_id)) {
		h_relative_ = GetAngleSum(h_relative_, M_PI);
	}

	// If moved over to opposite driving direction, then turn relative heading 180 degrees
	// if (old_lane_id != 0 && lane_id_ != 0 && SIGN(lane_id_) != SIGN(old_lane_id))
	//{
	//	h_relative_ = GetAngleSum(h_relative_, M_PI);
	//}

	// Lane2Track();
	LaneBoundary2Track();
	Track2XYZ();

	return;
}

void Position::SetRoadMarkPos(int track_id,
							  int lane_id,
							  int roadmark_idx,
							  int roadmarktype_idx,
							  int roadmarkline_idx,
							  double s,
							  double offset,
							  int lane_section_idx) {
	offset_ = offset;
	int old_lane_id = lane_id_;
	int old_track_id = track_id_;

	Road* road = GetOpenDrive()->GetRoadById(track_id);
	if (road == 0) {
		LOG("Position::Set Error: track %d not available", track_id);
		lane_id_ = lane_id;
		offset_ = offset;
		return;
	}

	if (s > road->GetLength()) {
		// Truncate road mark point to road length
		s = road->GetLength();
	}

	if (SetLongitudinalTrackPos(track_id, s) != Position::ErrorCode::ERROR_NO_ERROR) {
		lane_id_ = lane_id;
		offset_ = offset;
		roadmark_idx_ = roadmark_idx;
		roadmarktype_idx_ = roadmarktype_idx;
		roadmarkline_idx_ = roadmarkline_idx;
		return;
	}

	if (lane_id != lane_id_ && lane_section_idx == -1) {
		// New lane ID might indicate a discreet jump to a new, distant position, reset lane section, if not
		// specified in func parameter)
		lane_section_idx = road->GetLaneSectionIdxByS(s);
	}

	LaneSection* lane_section = 0;
	if (lane_section_idx > -1)	// If lane section was specified or reset
	{
		lane_section_idx_ = lane_section_idx;
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		lane_id_ = lane_id;
	} else	// Find LaneSection and info according to s
	{
		LaneInfo lane_info = road->GetLaneInfoByS(s_, lane_section_idx_, lane_id_, snapToLaneTypes_);
		lane_section_idx_ = lane_info.lane_section_idx_;
		lane_id_ = lane_info.lane_id_;

		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	}

	if (lane_section != 0) {
		lane_idx_ = lane_section->GetLaneIdxById(lane_id_);
		if (lane_idx_ == -1) {
			LOG("lane_idx %d fail for lane id %d\n", lane_idx_, lane_id_);
			lane_idx_ = 0;
		}
	} else {
		LOG("Position::Set (lanepos) Error - lanesection NULL lsidx %d rid %d lid %d\n", lane_section_idx_,
			road->GetId(), lane_id_);
	}

	// Check road direction when on new track
	if (old_lane_id != 0 && lane_id_ != 0 && track_id_ != old_track_id
		&& SIGN(lane_id_) != SIGN(old_lane_id)) {
		h_relative_ = GetAngleSum(h_relative_, M_PI);
	}

	Lane* lane = lane_section->GetLaneByIdx(lane_idx_);
	if (lane != 0) {
		roadmark_idx_ = roadmark_idx;
	}

	LaneRoadMark* lane_roadmark = lane->GetLaneRoadMarkByIdx(roadmark_idx_);
	if (lane_roadmark != 0) {
		s_ = MIN(s_, road->GetLength());
	} else {
		LOG("roadmark_idx_ %d fail for lane id %d\n", roadmark_idx_, lane_idx_);
		roadmark_idx_ = 0;
	}

	if (lane_roadmark->GetNumberOfRoadMarkTypes() != 0) {
		roadmarktype_idx_ = roadmarktype_idx;
	} else {
		roadmarktype_idx_ = 0;
	}

	LaneRoadMarkType* lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(roadmarktype_idx_);
	if (lane_roadmarktype != 0) {
		roadmarkline_idx_ = roadmarkline_idx;
		LaneRoadMarkTypeLine* lane_roadmarktypeline
			= lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(roadmarkline_idx_);
		if (lane_roadmarktypeline != 0) {
			s_ = MIN(s_, road->GetLength());
		} else {
			LOG("roadmarktypeline_idx_ %d fail for roadmarktype_idx %d\n", roadmarkline_idx_,
				roadmarktype_idx_);
			roadmarkline_idx_ = 0;
		}
	} else {
		LOG("roadmarktype_idx_ %d fail for roadmark_idx %d\n", roadmarktype_idx_, roadmark_idx_);
		roadmarkline_idx_ = 0;
	}

	RoadMark2Track();
	Track2XYZ();
}

int Position::SetInertiaPos(double x, double y, double z, double h, double p, double r, bool updateTrackPos) {
	x_ = x;
	y_ = y;
	z_ = z;

	if (updateTrackPos) {
		XYZ2Track();
	}

	// Now when road orientation is known, call functions for
	// updating angles both absolute and relative the road
	SetHeading(h);
	SetPitch(p);
	SetRoll(r);

	EvaluateOrientation();

	return 0;
}

int Position::SetInertiaPos(double x, double y, double h, bool updateTrackPos) {
	x_ = x;
	y_ = y;

	if (updateTrackPos) {
		XYZ2Track();
	}

	// Now when road orientation is known, call functions for
	// updating angles both absolute and relative the road
	SetHeading(h);

	EvaluateOrientation();

	if (align_z_ == ALIGN_MODE::ALIGN_SOFT) {
		SetZRelative(z_relative_);
	} else if (align_z_ == ALIGN_MODE::ALIGN_HARD) {
		SetZ(z_road_);
	}

	return 0;
}

void Position::SetHeading(double heading) {
	h_ = heading;
	h_relative_ = GetAngleInInterval2PI(GetAngleDifference(h_, h_road_));  // Something wrong with -angles
}

void Position::SetHeadingRelative(double heading) {
	h_relative_ = GetAngleInInterval2PI(heading);
	h_ = GetAngleSum(h_road_, h_relative_);
}

void Position::SetHeadingRelativeRoadDirection(double heading) {
	if (h_relative_ > M_PI_2 && h_relative_ < 3 * M_PI_2) {
		// Driving towards road direction
		h_relative_ = GetAngleInInterval2PI(heading + M_PI);
	} else {
		h_relative_ = GetAngleInInterval2PI(heading);
	}
	h_ = GetAngleSum(h_road_, h_relative_);
}

void Position::SetRoll(double roll) {
	r_ = roll;
	r_relative_ = GetAngleInInterval2PI(GetAngleDifference(r_, r_road_));
}

void Position::SetRollRelative(double roll) {
	r_relative_ = GetAngleInInterval2PI(roll);
	r_ = GetAngleSum(r_road_, r_relative_);
}

void Position::SetPitch(double pitch) {
	p_ = pitch;
	p_relative_ = GetAngleInInterval2PI(GetAngleDifference(p_, p_road_));
}

void Position::SetPitchRelative(double pitch) {
	p_relative_ = GetAngleInInterval2PI(pitch);
	p_ = GetAngleSum(p_road_, p_relative_);
}

void Position::SetZ(double z) {
	z_relative_ = z - z_road_;
	z_ = z;
}

void Position::SetZRelative(double z) {
	z_relative_ = z;
	z_ = z_road_ + z_relative_;
}

void Position::EvaluateOrientation() {
	if (align_h_ != ALIGN_MODE::ALIGN_NONE || align_p_ != ALIGN_MODE::ALIGN_NONE
		|| align_r_ != ALIGN_MODE::ALIGN_NONE) {
		R0R12EulerAngles(align_h_ != ALIGN_MODE::ALIGN_NONE ? GetHRoad() : 0.0,
						 align_p_ != ALIGN_MODE::ALIGN_NONE ? GetPRoad() : 0.0,
						 align_r_ != ALIGN_MODE::ALIGN_NONE ? GetRRoad() : 0.0,
						 align_h_ != ALIGN_MODE::ALIGN_HARD ? GetHRelative() : 0.0,
						 align_p_ != ALIGN_MODE::ALIGN_HARD ? GetPRelative() : 0.0,
						 align_r_ != ALIGN_MODE::ALIGN_HARD ? GetRRelative() : 0.0, h_, p_, r_);
		h_ = GetAngleInInterval2PI(h_);
		p_ = GetAngleInInterval2PI(p_);
		r_ = GetAngleInInterval2PI(r_);
	} else {
		h_ = GetHRelative();
		p_ = GetPRelative();
		r_ = GetRRelative();
	}
}

double Position::GetCurvature() {
	Geometry* geom = GetOpenDrive()->GetGeometryByIdx(track_idx_, geometry_idx_);

	if (geom) {
		return geom->EvaluateCurvatureDS(GetS() - geom->GetS());
	} else {
		return 0;
	}
}

int Position::GetDrivingDirectionRelativeRoad() const {
	if (GetTrackId() >= 0 && GetRoadById(GetTrackId()) != nullptr) {
		// Consider road rule (left hand or right hand traffic)
		if (GetLaneId() < 0 && GetRoadById(GetTrackId())->GetRule() == Road::RoadRule::LEFT_HAND_TRAFFIC
			|| GetLaneId() > 0
				   && GetRoadById(GetTrackId())->GetRule() == Road::RoadRule::RIGHT_HAND_TRAFFIC) {
			return -1;
		} else {
			return 1;
		}
	} else {
		return GetLaneId() > 0 ? -1 : 1;
	}
}

double Position::GetHRoadInDrivingDirection() const {
	return GetAngleSum(GetHRoad(), GetDrivingDirectionRelativeRoad() < 0 ? M_PI : 0.0);
}

double Position::GetPRoadInDrivingDirection() {
	return GetPRoad() * GetDrivingDirectionRelativeRoad();
}

double Position::GetHRelativeDrivingDirection() const {
	return GetAngleDifference(h_, GetDrivingDirection());
}

double Position::GetSpeedLimit() {
	double speed_limit = 70 / 3.6;	// some default speed
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);

	if (road) {
		speed_limit = road->GetSpeedByS(s_);

		if (speed_limit < SMALL_NUMBER) {
			// No speed limit defined, set a value depending on number of lanes
			speed_limit = GetOpenDrive()
									  ->GetRoadByIdx(track_idx_)
									  ->GetNumberOfDrivingLanesSide(GetS(), SIGN(GetLaneId()))
								  > 1
							  ? 120 / 3.6
							  : 60 / 3.6;
		}
	}

	return speed_limit;
}

double Position::GetDrivingDirection() const {
	double x, y, h;
	Geometry* geom = GetOpenDrive()->GetGeometryByIdx(track_idx_, geometry_idx_);

	if (!geom) {
		return h_;
	}

	geom->EvaluateDS(GetS() - geom->GetS(), &x, &y, &h);

	// adjust 180 degree according to side of road
	if (GetLaneId() > 0)  // Left side of road reference line
	{
		h = GetAngleSum(h, M_PI);
	}

	return (h);
}

double Position::GetVelLat() {
	double vx = GetVelX();
	double vy = GetVelY();
	double vlat = 0.0;
	double vlong = 0.0;
	RotateVec2D(vx, vy, -GetH(), vlong, vlat);

	return vlat;
}

double Position::GetVelLong() {
	double vx = GetVelX();
	double vy = GetVelY();
	double vlat = 0.0;
	double vlong = 0.0;
	RotateVec2D(vx, vy, -GetH(), vlong, vlat);

	return vlong;
}

void Position::GetVelLatLong(double& vlat, double& vlong) {
	double vx = GetVelX();
	double vy = GetVelY();
	RotateVec2D(vx, vy, -GetH(), vlong, vlat);
}

double Position::GetAccLat() {
	double ax = GetAccX();
	double ay = GetAccY();
	double alat = 0.0;
	double along = 0.0;
	RotateVec2D(ax, ay, -GetH(), along, alat);

	return alat;
}

double Position::GetAccLong() {
	double ax = GetAccX();
	double ay = GetAccY();
	double alat = 0.0;
	double along = 0.0;
	RotateVec2D(ax, ay, -GetH(), along, alat);

	return along;
}

void Position::GetAccLatLong(double& alat, double& along) {
	double ax = GetVelX();
	double ay = GetVelY();
	RotateVec2D(ax, ay, -GetH(), along, alat);
}

double Position::GetVelT() {
	double vx = GetVelX();
	double vy = GetVelY();
	double vt = 0.0;
	double vs = 0.0;
	RotateVec2D(vx, vy, -GetHRoad(), vs, vt);

	return vt;
}

double Position::GetVelS() {
	double vx = GetVelX();
	double vy = GetVelY();
	double vt = 0.0;
	double vs = 0.0;
	RotateVec2D(vx, vy, -GetHRoad(), vs, vt);

	return vs;
}

void Position::GetVelTS(double& vt, double& vs) {
	double vx = GetVelX();
	double vy = GetVelY();
	RotateVec2D(vx, vy, -GetHRoad(), vs, vt);
}

double Position::GetAccT() {
	double ax = GetAccX();
	double ay = GetAccY();
	double at = 0.0;
	double as = 0.0;
	RotateVec2D(ax, ay, -GetHRoad(), as, at);

	return at;
}

double Position::GetAccS() {
	double ax = GetAccX();
	double ay = GetAccY();
	double at = 0.0;
	double as = 0.0;
	RotateVec2D(ax, ay, -GetHRoad(), as, at);

	return as;
}

void Position::GetAccTS(double& at, double& as) {
	double ax = GetAccX();
	double ay = GetAccY();
	RotateVec2D(ax, ay, -GetHRoad(), as, at);
}

void Position::CopyRMPos(Position* from) {
	// Preserve route field
	Route* route_tmp = route_;

	*this = *from;
	route_ = route_tmp;
}

void Position::PrintTrackPos() {
	LOG("	Track pos: (road_id %d, s %.2f, t %.2f, h %.2f)", track_id_, s_, t_, h_);
}

void Position::PrintLanePos() {
	LOG("	Lane pos: (road_id %d, lane_id %d, s %.2f, offset %.2f, h %.2f)", track_id_, lane_id_, s_,
		offset_, h_);
}

void Position::PrintInertialPos() {
	LOG("	Inertial pos: (x %.2f, y %.2f, z %.2f, h %.2f, p %.2f, r %.2f)", x_, y_, z_, h_, p_, r_);
}

void Position::Print() {
	LOG("Pos(%.2f, %.2f, %.2f) Rot(%.2f, %.2f, %.2f) roadId %d laneId %d offset %.2f t %.2f", GetX(), GetY(),
		GetZ(), GetH(), GetP(), GetR(), GetTrackId(), GetLaneId(), GetOffset(), GetT());
}

void Position::PrintXY() {
	LOG("%.2f, %.2f\n", x_, y_);
}

bool Position::IsOffRoad() {
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road) {
		// Check whether outside road width
		if (fabs(t_) > road->GetWidth(GetS(), SIGN(t_), ~Lane::LaneType::LANE_TYPE_NONE)) {
			return true;
		}
	}

	return false;
}

bool Position::IsInJunction() {
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road) {
		return road->GetJunction() != -1;
	}

	return false;
}

double Position::getRelativeDistance(double targetX, double targetY, double& x, double& y) const {
	// Calculate diff vector from current to target
	double diff_x, diff_y;

	diff_x = targetX - GetX();
	diff_y = targetY - GetY();

	// Align with closest road driving direction
	double hAlign = 0.0;
	if (GetHRelative() > M_PI_2 && GetHRelative() < 3 * M_PI_2) {
		hAlign = M_PI;
	}
	hAlign = -GetAngleSum(GetHRoad(), hAlign);
	x = diff_x * cos(hAlign) - diff_y * sin(hAlign);
	y = diff_x * sin(hAlign) + diff_y * cos(hAlign);

	// Now just check whether diff vector X-component is less than 0 (behind current)
	int sign = x > 0 ? 1 : -1;

	// Return length of dist vector
	return sign * sqrt((x * x) + (y * y));
}

int Position::CalcRoutePosition() {
	if (route_ == 0 || !route_->IsValid()) {
		return -1;
	}

	if (route_->SetTrackS(GetTrackId(), GetS()) == ErrorCode::ERROR_NO_ERROR) {
		return 0;
	} else {
		return -1;
	}
}

int Position::SetRoute(Route* route) {
	route_ = route;

	// Also find out current position in terms of route position
	return CalcRoutePosition();
}

void Position::SetTrajectory(RMTrajectory* trajectory) {
	trajectory_ = trajectory;

	// Reset trajectory S value
	s_trajectory_ = 0;
}

bool Position::Delta(Position* pos_b, PositionDiff& diff, bool bothDirections, double maxDist) const {
	double dist = 0;
	bool found;

	RoadPath* path = new RoadPath(this, pos_b);
	found = (path->Calculate(dist, bothDirections, maxDist) == 0);
	if (found) {
		int laneIdB = pos_b->GetLaneId();
		double tB = pos_b->GetT();

		// If start and end roads are oppotite directed, inverse one side for delta calculations
		if (path->visited_.size() > 0
			&& ((path->visited_[0]->link->GetType() == LinkType::SUCCESSOR
				 && path->visited_.back()->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END)
				|| (path->visited_[0]->link->GetType() == LinkType::PREDECESSOR
					&& path->visited_.back()->link->GetContactPointType()
						   == ContactPointType::CONTACT_POINT_START))) {
			laneIdB = -laneIdB;
			tB = -tB;
		}

		// calculate delta lane id and lateral position
		diff.dLaneId = -SIGN(GetLaneId()) * (laneIdB - GetLaneId());
		diff.dt = -SIGN(GetLaneId()) * (tB - GetT());

		diff.ds = dist;

#if 0  // Change to 1 to print some info on stdout - e.g. for debugging
		printf("Dist %.2f Path (reversed): %d", dist, pos_b.GetTrackId());
		if (path->visited_.size() > 0)
		{
			RoadPath::PathNode* node = path->visited_.back();

			while (node)
			{
				if (node->fromRoad != 0)
				{
					printf(" <- %d", node->fromRoad->GetId());
				}
				node = node->previous;
			}
		}
		printf("\n");
#endif
	} else	// no valid route found
	{
		diff.dLaneId = 0;
		diff.ds = LARGE_NUMBER;
		diff.dt = LARGE_NUMBER;
		getRelativeDistance(pos_b->GetX(), pos_b->GetY(), diff.dx, diff.dy);
	}

	delete path;

	return found;
}

int Position::Distance(Position* pos_b,
					   CoordinateSystem cs,
					   RelativeDistanceType relDistType,
					   double& dist,
					   double maxDist) {
	// Handle/convert depricated value
	if (relDistType == RelativeDistanceType::REL_DIST_CARTESIAN) {
		relDistType = RelativeDistanceType::REL_DIST_EUCLIDIAN;
	}

	if (relDistType == RelativeDistanceType::REL_DIST_EUCLIDIAN) {
		double dx, dy;
		dist = getRelativeDistance(pos_b->GetX(), pos_b->GetY(), dx, dy);
	} else if (relDistType == RelativeDistanceType::REL_DIST_LATERAL
			   || relDistType == RelativeDistanceType::REL_DIST_LONGITUDINAL) {
		if (cs == CoordinateSystem::CS_LANE) {
			LOG_ONCE("Lane coordinateSystem not supported yet. Falling back to Road coordinate system.");
			cs = CoordinateSystem::CS_ROAD;
		}

		if (cs == CoordinateSystem::CS_ROAD) {
			PositionDiff diff;
			bool routeFound = Delta(pos_b, diff, true, maxDist);
			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? diff.dt : diff.ds;
			if (routeFound == false) {
				return -1;
			}
		} else if (cs == CoordinateSystem::CS_ENTITY) {
			double dx, dy;

			getRelativeDistance(pos_b->GetX(), pos_b->GetY(), dx, dy);

			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? dy : dx;
		} else if (cs == CoordinateSystem::CS_TRAJECTORY) {
			dist
				= relDistType == RelativeDistanceType::REL_DIST_LATERAL ? GetTrajectoryT() : GetTrajectoryS();
		}
	} else {
		LOG("Unhandled case: cs %d reDistType %d freeSpace false\n", cs, relDistType);
		return -1;
	}

	return 0;
}

int Position::Distance(double x,
					   double y,
					   CoordinateSystem cs,
					   RelativeDistanceType relDistType,
					   double& dist,
					   double maxDist) {
	// Handle/convert depricated value
	if (relDistType == RelativeDistanceType::REL_DIST_CARTESIAN) {
		relDistType = RelativeDistanceType::REL_DIST_EUCLIDIAN;
	}

	if (relDistType == RelativeDistanceType::REL_DIST_EUCLIDIAN) {
		double dx, dy;
		dist = getRelativeDistance(x, y, dx, dy);
	} else if (relDistType == RelativeDistanceType::REL_DIST_LATERAL
			   || relDistType == RelativeDistanceType::REL_DIST_LONGITUDINAL) {
		if (cs == CoordinateSystem::CS_LANE) {
			LOG_ONCE("Lane coordinateSystem not supported yet. Falling back to Road coordinate system.");
			cs = CoordinateSystem::CS_ROAD;
		}

		if (cs == CoordinateSystem::CS_ROAD) {
			Position pos_b(x, y, 0, 0, 0, 0);
			PositionDiff diff;
			bool routeFound = Delta(&pos_b, diff, true, maxDist);
			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? diff.dt : diff.ds;
			if (routeFound == false) {
				return -1;
			}
		} else if (cs == CoordinateSystem::CS_ENTITY) {
			double dx, dy;

			getRelativeDistance(x, y, dx, dy);

			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? dy : dx;
		} else if (cs == CoordinateSystem::CS_TRAJECTORY) {
			dist
				= relDistType == RelativeDistanceType::REL_DIST_LATERAL ? GetTrajectoryT() : GetTrajectoryS();
		}
	} else {
		LOG("Unhandled case: cs %d reDistType %d freeSpace false\n", cs, relDistType);
		return -1;
	}

	return 0;
}

bool Position::IsAheadOf(Position target_position) {
	// Calculate diff vector from current to target
	double diff_x, diff_y;
	double diff_x0;

	diff_x = target_position.GetX() - GetX();
	diff_y = target_position.GetY() - GetY();

	// Compensate for current heading (rotate so that current heading = 0)
	// Only x component needed
	diff_x0 = diff_x * cos(-GetH()) - diff_y * sin(-GetH());

	// Now just check whether diff vector X-component is less than 0 (behind current)
	return (diff_x0 < 0);
}

int Position::GetRoadLaneInfo(RoadLaneInfo* data) {
	if (fabs(GetCurvature()) > SMALL_NUMBER) {
		double radius = 1.0 / GetCurvature();
		radius -= GetT();  // curvature positive in left curves, lat_offset positive left of reference lane
		data->curvature = (1.0 / radius);
	} else {
		// curvature close to zero (straight segment), radius infitite - curvature the same in all lanes
		data->curvature = GetCurvature();
	}

	data->pos[0] = GetX();
	data->pos[1] = GetY();
	data->pos[2] = GetZRoad();
	data->heading = GetHRoad();
	data->pitch = GetPRoad();
	data->roll = GetRRoad();
	data->laneId = GetLaneId();
	data->laneOffset = GetOffset();
	data->roadId = GetTrackId();
	data->junctionId = GetJunctionId();
	data->t = GetT();
	data->s = GetS();

	// Then find out the width of the lane at current s-value
	Road* road = GetRoadById(GetTrackId());
	if (road) {
		data->width = road->GetLaneWidthByS(GetS(), GetLaneId());
		data->speed_limit = road->GetSpeedByS(GetS());
	}

	return 0;
}

int Position::GetRoadLaneInfo(double lookahead_distance, RoadLaneInfo* data, LookAheadMode lookAheadMode) {
	Position target(*this);	 // Make a copy of current position

	if (lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_ROAD_CENTER) {
		// Look along reference lane requested, move pivot position to t=0 plus a small number in order to
		// fall into the right direction
		target.SetTrackPos(target.GetTrackId(), target.GetS(), SMALL_NUMBER * SIGN(GetLaneId()));
	} else if (lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER) {
		// Look along current lane center requested, move pivot position accordingly
		target.SetLanePos(target.GetTrackId(), target.GetLaneId(), target.GetS(), 0);
	}

	if (fabs(lookahead_distance) > SMALL_NUMBER) {
		if (target.MoveAlongS(lookahead_distance, 0.0, 0.0) != Position::ErrorCode::ERROR_NO_ERROR) {
			return -1;
		}
	}

	target.GetRoadLaneInfo(data);

	return 0;
}

int Position::CalcProbeTarget(Position* target, RoadProbeInfo* data) {
	int retval = target->GetRoadLaneInfo(&data->road_lane_info);

	if (retval == 0) {
		// find out local x, y, z
		double diff_x = target->GetX() - GetX();
		double diff_y = target->GetY() - GetY();
		double diff_z = target->GetZRoad() - GetZRoad();

		data->relative_pos[0] = diff_x * cos(-GetH()) - diff_y * sin(-GetH());
		data->relative_pos[1] = diff_x * sin(-GetH()) + diff_y * cos(-GetH());
		data->relative_pos[2] = diff_z;

#if 0
		// for validation
		data->global_pos[0] = GetX() + data->local_pos[0] * cos(GetH()) - data->local_pos[1] * sin(GetH());
		data->global_pos[1] = GetY() + data->local_pos[0] * sin(GetH()) + data->local_pos[1] * cos(GetH());
		data->global_pos[2] = GetZ() + data->local_pos[2];
#endif

		// Calculate angle - by dot product
		if (fabs(data->relative_pos[0]) < SMALL_NUMBER && fabs(data->relative_pos[1]) < SMALL_NUMBER
			&& fabs(data->relative_pos[2]) < SMALL_NUMBER) {
			data->relative_h = GetH();
		} else {
			double dot_prod = (data->relative_pos[0] * 1.0 + data->relative_pos[1] * 0.0)
							  / sqrt(data->relative_pos[0] * data->relative_pos[0]
									 + data->relative_pos[1] * data->relative_pos[1]);
			data->relative_h = SIGN(data->relative_pos[1]) * acos(dot_prod);
		}
	}

	return retval;
}

Position::ErrorCode Position::GetProbeInfo(double lookahead_distance,
										   RoadProbeInfo* data,
										   LookAheadMode lookAheadMode) {
	ErrorCode retval = ErrorCode::ERROR_NO_ERROR;

	if (GetOpenDrive()->GetNumOfRoads() == 0) {
		return ErrorCode::ERROR_GENERIC;
	}
	Position target(*this);	 // Make a copy of current position

	if (lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_ROAD_CENTER) {
		// Look along reference lane requested, move pivot position to t=0 plus a small number in order to
		// fall into the right direction
		retval = target.SetTrackPos(target.GetTrackId(), target.GetS(), SMALL_NUMBER * SIGN(GetLaneId()));
	} else if (lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER) {
		// Look along current lane center requested, move pivot position accordingly
		retval = target.SetLanePos(target.GetTrackId(), target.GetLaneId(), target.GetS(), 0);
	}

	if (fabs(lookahead_distance) > SMALL_NUMBER) {
		if (target.route_ && target.route_->IsValid()) {
			retval = target.MoveRouteDS(lookahead_distance,
										lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
		} else {
			retval = target.MoveAlongS(lookahead_distance, 0.0, 0.0,
									   lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
		}
	}

	if (retval != ErrorCode::ERROR_GENERIC) {
		CalcProbeTarget(&target, data);
	}

	return retval;
}

Position::ErrorCode Position::GetProbeInfo(Position* target_pos, RoadProbeInfo* data) {
	if (CalcProbeTarget(target_pos, data) != 0) {
		return ErrorCode::ERROR_GENERIC;
	}

	return ErrorCode::ERROR_NO_ERROR;
}

int Position::GetTrackId() const {
	if (rel_pos_ && rel_pos_ != this
		&& (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)) {
		return rel_pos_->GetTrackId();
	}

	return track_id_;
}

int Position::GetJunctionId() const {
	if (rel_pos_ && rel_pos_ != this
		&& (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)) {
		return rel_pos_->GetJunctionId();
	}

	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road) {
		return road->GetJunction();
	}

	return -1;
}

int Position::GetLaneId() const {
	if (rel_pos_ && rel_pos_ != this && type_ == PositionType::RELATIVE_LANE) {
		return rel_pos_->GetLaneId() + lane_id_;
	}

	return lane_id_;
}

int Position::GetLaneGlobalId() {
	Road* road = GetRoadById(GetTrackId());
	if (road == 0) {
		// No road
		return -1;
	}

	if (road->GetJunction() != -1) {
		return GetOpenDrive()->GetJunctionById(road->GetJunction())->GetGlobalId();
	}

	LaneSection* lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

	if (lane_section == 0) {
		LOG("No lane section for idx %d - keeping current lane setting\n", lane_section_idx_);
		return -2;
	}

	double offset;
	int lane_idx = lane_section->GetClosestLaneIdx(s_, t_, offset, false, Lane::LaneType::LANE_TYPE_ANY);

	if (lane_idx == -1) {
		LOG("Failed to find a valid drivable lane");
		return -3;
	}

	// Check if it is not a center lane
	int lane_id = lane_section->GetLaneIdByIdx(lane_idx);
	if (!lane_section->IsOSILaneById(lane_id)) {
		if (offset >= 0) {
			lane_id = 1;
		} else {
			lane_id = -1;
		}
		lane_idx = lane_section->GetLaneIdxById(lane_id);
	}

	return lane_section->GetLaneGlobalIdByIdx(lane_idx);
}

double Position::GetS() const {
	if (rel_pos_ && rel_pos_ != this
		&& (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)) {
		return rel_pos_->GetS() + s_;
	}

	return s_;
}

double Position::GetT() const {
	if (rel_pos_ && rel_pos_ != this
		&& (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)) {
		return rel_pos_->GetT() + t_;
	}

	return t_;
}

double Position::GetOffset() {
	if (rel_pos_ && rel_pos_ != this
		&& (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)) {
		return rel_pos_->GetOffset() + offset_;
	}

	return offset_;
}

double Position::GetX() const {
	if (!rel_pos_ || rel_pos_ == this) {
		return x_;
	} else if (type_ == PositionType::RELATIVE_OBJECT) {
		return rel_pos_->GetX() + x_ * cos(rel_pos_->GetH()) - y_ * sin(rel_pos_->GetH());
	} else if (type_ == PositionType::RELATIVE_WORLD) {
		return x_ + rel_pos_->GetX();
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		// Create a temporary position to evaluate in relative lane coordinates
		Position pos = *this->rel_pos_;

		// If valid road ID, then move laterally
		if (pos.GetTrackId() != -1) {
			pos.SetLanePos(pos.GetTrackId(), pos.GetLaneId() + lane_id_, pos.GetS() + s_,
						   pos.GetOffset() + offset_);
		}

		return pos.GetX();
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return x_;
}

double Position::GetY() const {
	if (!rel_pos_ || rel_pos_ == this) {
		return y_;
	} else if (type_ == PositionType::RELATIVE_OBJECT) {
		return rel_pos_->GetY() + y_ * cos(rel_pos_->GetH()) + x_ * sin(rel_pos_->GetH());
	} else if (type_ == PositionType::RELATIVE_WORLD) {
		return y_ + rel_pos_->GetY();
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		// Create a temporary position to evaluate in relative lane coordinates
		Position pos = *this->rel_pos_;

		// If valid road ID, then move laterally
		if (pos.GetTrackId() != -1) {
			pos.SetLanePos(pos.GetTrackId(), pos.GetLaneId() + lane_id_, pos.GetS() + s_,
						   pos.GetOffset() + offset_);
		}

		return pos.GetY();
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return y_;
}

double Position::GetZ() const {
	if (!rel_pos_ || rel_pos_ == this) {
		return z_;
	} else if (type_ == PositionType::RELATIVE_OBJECT || type_ == PositionType::RELATIVE_WORLD) {
		return z_ + rel_pos_->GetZ();
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		// Create a temporary position to evaluate in relative lane coordinates
		Position pos = *this->rel_pos_;

		// If valid road ID, then move laterally
		if (pos.GetTrackId() != -1) {
			pos.SetLanePos(pos.GetTrackId(), pos.GetLaneId() + lane_id_, pos.GetS() + s_,
						   pos.GetOffset() + offset_);
		}

		return pos.GetZ();
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return z_;
}

double Position::GetH() const {
	if (!rel_pos_ || rel_pos_ == this) {
		return h_;
	} else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return h_;
		} else {
			return h_ + rel_pos_->GetH();
		}
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return h_;
		} else {
			return h_ + GetHRoadInDrivingDirection();
		}
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return h_;
}

double Position::GetHRelative() const {
	if (!rel_pos_ || rel_pos_ == this) {
		return h_relative_;
	} else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return h_relative_;
		} else {
			return GetAngleInInterval2PI(h_relative_ + rel_pos_->GetHRelative());
		}
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return h_relative_;
		} else {
			return GetAngleInInterval2PI(h_relative_ + GetHRoadInDrivingDirection());
		}
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return h_relative_;
}

double Position::GetP() {
	if (!rel_pos_ || rel_pos_ == this) {
		return p_;
	} else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return p_;
		} else {
			return GetAngleSum(p_, rel_pos_->GetP());
		}
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return p_;
		} else {
			return GetAngleSum(p_, GetPRoadInDrivingDirection());
		}
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return p_;
}

double Position::GetPRelative() {
	if (!rel_pos_ || rel_pos_ == this) {
		return p_relative_;
	} else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return p_relative_;
		} else {
			return GetAngleInInterval2PI(p_relative_ + rel_pos_->GetPRelative());
		}
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return p_relative_;
		} else {
			return GetAngleInInterval2PI(p_relative_ + GetPRoadInDrivingDirection());
		}
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return p_relative_;
}

double Position::GetR() {
	if (!rel_pos_ || rel_pos_ == this) {
		return r_;
	} else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return r_;
		} else {
			return GetAngleSum(r_, rel_pos_->GetR());
		}
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return r_;
		} else {
			return r_;	// road R not implemented yet
		}
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return r_;
}

double Position::GetRRelative() {
	if (!rel_pos_ || rel_pos_ == this) {
		return r_relative_;
	} else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return r_relative_;
		} else {
			return GetAngleInInterval2PI(r_relative_ + rel_pos_->GetRRelative());
		}
	} else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD) {
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE) {
			return r_relative_;
		} else {
			return GetAngleInInterval2PI(r_relative_ + r_road_);
		}
	} else {
		LOG("Unexpected PositionType: %d", type_);
	}

	return r_relative_;
}

int Position::SetRoutePosition(Position* position) {
	if (!route_ || !route_->IsValid()) {
		return -1;
	}

	// Is it a valid position, i.e. is it along the route
	for (size_t i = 0; i < route_->minimal_waypoints_.size(); i++) {
		if (route_->minimal_waypoints_[i].GetTrackId() == position->GetTrackId())  // Same road
		{
			// Update current position
			Route* tmp = route_;  // save route pointer, copy the
			*this = *position;
			route_ = tmp;
			return 0;
		}
	}

	return -1;
}

double Position::GetRouteS() {
	if (!route_ || !route_->IsValid()) {
		return 0.0;
	}

	return route_->GetPathS();
}

Position::ErrorCode Position::MoveRouteDS(double ds, bool actualDistance) {
	Position::ErrorCode retval = ErrorCode::ERROR_NO_ERROR;

	if (!route_ || !route_->IsValid()) {
		return ErrorCode::ERROR_GENERIC;
	}

	// Idea:
	// Calculate adjusted ds for entity actual distance
	// if already in junction:
	//   Move entity first
	//   Move route pos to same % along connecting road as entity
	// else:
	//   Move route position object first
	//   Move entity position object
	//   if both entered junction:
	// 	   Move route pos to same % along connecting road as entity
	//   else if only one entered junction:
	// 	   Sync both to end of incoming road

	if (actualDistance) {
		ds = DistanceToDS(ds);
	}

	Road* entity_road = Position::GetOpenDrive()->GetRoadById(GetTrackId());
	double s_route = route_->GetTrackS();
	double s_entity = GetS();
	double t_entity = GetT();

	if (entity_road->GetJunction() > -1) {
		MoveAlongS(ds, false);	// actual distance = false since ds is already adjusted

		if (Position::GetOpenDrive()->GetRoadById(GetTrackId())->GetJunction() > -1) {
			if (route_->CopySFractionOfLength(this) != ErrorCode::ERROR_NO_ERROR) {
				route_->SetTrackS(GetTrackId(), GetS());
			}
		} else {
			// If out of junction, sync positions again
			route_->SetTrackS(GetTrackId(), GetS());
		}
	} else {
		retval = route_->MovePathDS(ds * (IsAngleForward(GetHRelative()) ? 1 : -1));
		MoveAlongS(ds, false);	// actual distance = false since ds is already adjusted

		Road* route_road2 = Position::GetOpenDrive()->GetRoadById(route_->GetTrackId());
		Road* entity_road2 = Position::GetOpenDrive()->GetRoadById(GetTrackId());

		if (entity_road2->GetJunction() > -1 && route_road2->GetJunction() > -1) {
			if (route_->CopySFractionOfLength(this) != ErrorCode::ERROR_NO_ERROR) {
				route_->SetTrackS(GetTrackId(), GetS());
			}
		} else if (entity_road2->GetJunction() > -1 || route_road2->GetJunction() > -1) {
			if (entity_road2->GetJunction() > -1) {
				SetTrackPos(route_->GetTrackId(), s_route + ds, t_entity);
			} else {
				retval = route_->SetTrackS(route_->GetTrackId(), s_entity + ds);
			}
		}
	}

	if (retval == ErrorCode::ERROR_END_OF_ROUTE) {
		SetRoute(nullptr);
	}

	return retval;
}

int Position::SetRouteLanePosition(Route* route, double path_s, int lane_id, double lane_offset) {
	route->SetPathS(path_s);

	SetLanePos(route->GetTrackId(), lane_id, route->GetTrackS(), lane_offset);

	return 0;
}
int Position::MoveTrajectoryDS(double ds) {
	if (!trajectory_) {
		return -1;
	}

	TrajVertex pos;
	trajectory_->shape_->Evaluate(s_trajectory_ + ds, Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, pos);

	SetInertiaPos(pos.x, pos.y, pos.h);

	s_trajectory_ = pos.s;

	return 0;
}

int Position::SetTrajectoryPosByTime(double time) {
	if (!trajectory_) {
		return -1;
	}

	TrajVertex pos;
	trajectory_->shape_->Evaluate(time, Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_TIME, pos);

	SetInertiaPos(pos.x, pos.y, pos.h);

	s_trajectory_ = pos.s;

	return 0;
}

int Position::SetTrajectoryS(double s) {
	if (!trajectory_) {
		return -1;
	}

	TrajVertex pos;
	trajectory_->shape_->Evaluate(s, Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, pos);
	SetInertiaPos(pos.x, pos.y, pos.z, pos.h, 0.0, 0.0, true);

	s_trajectory_ = pos.s;

	return 0;
}

Position::ErrorCode Position::SetRouteS(double route_s) {
	Position::ErrorCode retval = Position::ErrorCode::ERROR_NO_ERROR;

	if (!GetRoute()) {
		return ErrorCode::ERROR_GENERIC;
	}

	retval = route_->SetPathS(route_s);
	if (retval != ErrorCode::ERROR_NO_ERROR && retval != ErrorCode::ERROR_END_OF_ROUTE) {
		return retval;
	}

	// Register current driving direction
	int driving_direction = 1;
	if (GetAbsAngleDifference(GetH(), GetDrivingDirection()) > M_PI_2) {
		driving_direction = -1;
	}

	double offset_dir_neutral = offset_ * SIGN(GetLaneId());
	SetLanePos(route_->GetTrackId(), route_->GetLaneId(), route_->GetTrackS(), offset_dir_neutral);

	if (retval == ErrorCode::ERROR_END_OF_ROUTE) {
		LOG("Reached end of route, reset and continue");
		SetRoute(nullptr);
		status_ |= static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROUTE);
		return ErrorCode::ERROR_END_OF_ROUTE;
	}

	return ErrorCode::ERROR_NO_ERROR;
}

void Position::ReleaseRelation() {
	// Fetch values and then disconnect
	double x = GetX();
	double y = GetY();
	double z = GetZ();
	int roadId = GetTrackId();
	int laneId = GetLaneId();
	double s = GetS();
	double t = GetT();
	double offset = GetOffset();
	double p = GetP();
	double r = GetR();
	double h = GetH();
	double hAbs = h_;
	double hRel = h_relative_;
	double pAbs = p_;
	double pRel = p_relative_;
	double rAbs = r_;
	double rRel = r_relative_;
	PositionType type = type_;

	SetRelativePosition(0, PositionType::NORMAL);

	if (type == Position::PositionType::RELATIVE_LANE) {
		if (orientation_type_ == OrientationType::ORIENTATION_RELATIVE) {
			SetLanePos(roadId, laneId, s, offset);

			hRel = GetAngleSum(hRel, GetDrivingDirectionRelativeRoad() < 0 ? M_PI : 0.0);

			SetHeadingRelative(hRel);
			SetPitchRelative(pRel);
			SetRollRelative(rRel);
		} else {
			SetLanePos(roadId, laneId, s, offset);
			SetHeading(hAbs);
			SetPitch(pAbs);
			SetRoll(rAbs);
		}
	}
	if (type == Position::PositionType::RELATIVE_ROAD) {
		if (orientation_type_ == OrientationType::ORIENTATION_RELATIVE) {
			// Resolve requested position
			SetTrackPos(roadId, s, t);

			// Finally set requested heading considering lane ID and traffic rule
			hRel = GetAngleSum(hRel, GetDrivingDirectionRelativeRoad() < 0 ? M_PI : 0.0);

			SetHeadingRelative(hRel);
			SetPitchRelative(pRel);
			SetRollRelative(rRel);
		} else {
			SetTrackPos(roadId, s, t);
			SetHeading(hAbs);
			SetPitch(pAbs);
			SetRoll(rAbs);
		}
	} else if (type == PositionType::RELATIVE_OBJECT || type == PositionType::RELATIVE_WORLD) {
		SetInertiaPos(x, y, z, h, p, r, true);
	}
}

Position::ErrorCode Route::CopySFractionOfLength(Position* pos) {
	Road* road0 = Position::GetOpenDrive()->GetRoadById(pos->GetTrackId());
	Road* road1 = Position::GetOpenDrive()->GetRoadById(GetTrackId());

	Position::ErrorCode retval = Position::ErrorCode::ERROR_NO_ERROR;

	if (road0 == nullptr || road1 == nullptr) {
		return Position::ErrorCode::ERROR_GENERIC;
	}

	// Check direction, same of not
	int direction = 0;
	if (road0 == road1) {
		direction = 1;
	} else if (road0->GetJunction() == road1->GetJunction()) {
		// connecting roads in same junction
		RoadLink* link0_pre = road0->GetLink(LinkType::PREDECESSOR);
		RoadLink* link1_pre = road1->GetLink(LinkType::PREDECESSOR);
		RoadLink* link0_suc = road0->GetLink(LinkType::SUCCESSOR);
		RoadLink* link1_suc = road1->GetLink(LinkType::SUCCESSOR);

		if (link0_pre && link1_pre && *link0_pre == *link1_pre) {
			direction = 1;
		} else if (link0_suc && link1_suc && *link0_suc == *link1_suc) {
			direction = 1;
		} else if (link0_pre && link1_suc) {
			if ((link0_pre->GetType() == LinkType::PREDECESSOR && link1_suc->GetType() == LinkType::SUCCESSOR)
				|| (link0_pre->GetType() == LinkType::SUCCESSOR
					&& link1_suc->GetType() == LinkType::PREDECESSOR)
					   && link0_pre->GetElementType() == link1_suc->GetElementType()
					   && link0_pre->GetElementId() == link1_suc->GetElementId()
					   && link0_pre->GetContactPointType() == link1_suc->GetContactPointType())
				direction = -1;
		} else if (link0_suc && link1_pre) {
			if ((link0_suc->GetType() == LinkType::PREDECESSOR && link1_pre->GetType() == LinkType::SUCCESSOR)
				|| (link0_suc->GetType() == LinkType::SUCCESSOR
					&& link1_pre->GetType() == LinkType::PREDECESSOR)
					   && link0_suc->GetElementType() == link1_pre->GetElementType()
					   && link0_suc->GetElementId() == link1_pre->GetElementId()
					   && link0_suc->GetContactPointType() == link1_pre->GetContactPointType())
				direction = -1;
		} else {
			retval = Position::ErrorCode::ERROR_GENERIC;
		}
	}

	if (direction != 0) {
		double fraction = CLAMP(pos->GetS() / MAX(SMALL_NUMBER, road0->GetLength()), 0.0, 1.0);
		if (direction < 0) {
			fraction = 1 - fraction;
		}
		currentPos_.SetLanePos(GetTrackId(), GetLaneId(), fraction * road1->GetLength(), 0.0);
	}

	return retval;
}

Position::ErrorCode Route::SetTrackS(int trackId, double s) {
	// Loop over waypoints - look for current track ID and sum the distance (route s) up to current position
	double dist = 0;
	double local_s;

	if (s < 0 || minimal_waypoints_.size() == 0) {
		path_s_ = 0.0;
		waypoint_idx_ = 0;
		if (minimal_waypoints_.size() > 0) {
			currentPos_.SetTrackPos(GetWaypoint(waypoint_idx_)->GetTrackId(), 0.0, 0.0);
		} else {
			currentPos_.SetTrackPos(-1, 0.0, 0.0);
		}
		return Position::ErrorCode::ERROR_NO_ERROR;
	}

	for (size_t i = 0; i < minimal_waypoints_.size(); i++) {
		int route_direction = GetWayPointDirection((int)i);

		if (route_direction == 0) {
			LOG("Unexpected lack of connection in route at waypoint %d", i);
			return Position::ErrorCode::ERROR_GENERIC;
		}

		// Add length of intermediate waypoint road
		dist += Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength();

		if (i == 0) {
			// Subtract initial s-value for the first waypoint
			if (route_direction > 0)  // route in waypoint road direction
			{
				dist -= minimal_waypoints_[0].GetS();
				dist = MAX(dist, 0.0);
			} else {
				// route in opposite road direction - remaining distance equals waypoint s-value
				dist = minimal_waypoints_[0].GetS();
			}
		}

		if (trackId == minimal_waypoints_[i].GetTrackId()) {
			// current position is at the road of this waypoint - i.e. along the route
			// remove remaming s from road
			if (route_direction > 0) {
				dist
					-= (Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength()
						- s);
			} else {
				dist -= s;
			}

			path_s_ = MAX(dist, 0.0);
			local_s = s;
			waypoint_idx_ = (int)i;
			currentPos_.SetLanePos(GetWaypoint(waypoint_idx_)->GetTrackId(),
								   GetWaypoint(waypoint_idx_)->GetLaneId(), local_s, 0.0);

			return Position::ErrorCode::ERROR_NO_ERROR;
		}
	}

	// Failed to map current position to the current route
	return Position::ErrorCode::ERROR_GENERIC;
}

Position::ErrorCode Route::MovePathDS(double ds) {
	if (minimal_waypoints_.size() == 0) {
		return Position::ErrorCode::ERROR_GENERIC;
	}

	// Consider route direction
	ds *= GetDirectionRelativeRoad();

	return SetPathS(GetPathS() + ds);
}

Position::ErrorCode Route::SetPathS(double s) {
	// Loop over waypoints - until reaching s meters
	double dist = 0;
	double local_s = 0.0;

	if (s < 0 || minimal_waypoints_.size() == 0) {
		path_s_ = 0.0;
		waypoint_idx_ = 0;
		currentPos_.SetTrackPos(GetWaypoint(waypoint_idx_)->GetTrackId(), 0.0, 0.0);
		return Position::ErrorCode::ERROR_NO_ERROR;
	}

	for (size_t i = 0; i < minimal_waypoints_.size(); i++) {
		int route_direction = GetWayPointDirection((int)i);

		if (route_direction == 0) {
			LOG("Unexpected lack of connection in route at waypoint %d", i);
			return Position::ErrorCode::ERROR_GENERIC;
		}

		// Add length of intermediate waypoint road
		dist += Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength();

		if (i == 0) {
			// Subtract initial s-value for the first waypoint
			if (route_direction > 0)  // route in waypoint road direction
			{
				dist -= minimal_waypoints_[0].GetS();
				dist = MAX(dist, 0.0);
			} else {
				// route in opposite road direction - remaining distance equals waypoint s-value
				dist = minimal_waypoints_[0].GetS();
			}
		}

		if (s < dist) {
			// current position is at the road of this waypoint - i.e. along the route
			// remove remaming s from road
			if (route_direction > 0) {
				local_s = s
						  - (dist
							 - Position::GetOpenDrive()
								   ->GetRoadById(minimal_waypoints_[i].GetTrackId())
								   ->GetLength());
			} else {
				local_s = dist - s;
			}

			path_s_ = s;
			waypoint_idx_ = (int)i;
			currentPos_.SetLanePos(GetWaypoint(waypoint_idx_)->GetTrackId(),
								   GetWaypoint(waypoint_idx_)->GetLaneId(), local_s, 0.0);

			return Position::ErrorCode::ERROR_NO_ERROR;
		} else if (i == minimal_waypoints_.size() - 1) {
			// Past end of route
			if (route_direction > 0) {
				local_s
					= Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength();
			} else {
				local_s = 0.0;
			}
			waypoint_idx_ = (int)i;
			currentPos_.SetLanePos(GetWaypoint(waypoint_idx_)->GetTrackId(),
								   GetWaypoint(waypoint_idx_)->GetLaneId(), local_s, 0.0);
		}
	}

	// Failed to map current position, past the end of route
	path_s_ = GetLength();
	return Position::ErrorCode::ERROR_END_OF_ROUTE;
}

Position* Route::GetWaypoint(int index) {
	if (index == -1)  // Get current
	{
		index = waypoint_idx_;
	}

	if (minimal_waypoints_.size() == 0 || index < 0 || index >= minimal_waypoints_.size()) {
		LOG("Waypoint index %d out of range (%d)", index, minimal_waypoints_.size());
		return 0;
	}

	return &minimal_waypoints_[index];
}
