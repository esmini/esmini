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

#include <random>
#include "Entities.hpp"
#include "Controller.hpp"
#include "OSCPrivateAction.hpp"
#include "OSCManeuver.hpp"

using namespace scenarioengine;
using namespace roadmanager;

#define ELEVATION_DIFF_THRESHOLD 2.5

Object::Object(Type type) : type_(type), id_(0), speed_(0), wheel_angle_(0), wheel_rot_(0),
route_(0), model3d_(""), ghost_trail_s_(0), trail_follow_index_(0), odometer_(0), end_of_road_timestamp_(0.0),
off_road_timestamp_(0.0), stand_still_timestamp_(0), dirty_(0), reset_(0), controller_(0), headstart_time_(0), ghost_(0), ghost_Ego_(0),
visibilityMask_(0xff), isGhost_(false), junctionSelectorStrategy_(Junction::JunctionStrategyType::RANDOM),
nextJunctionSelectorAngle_(0.0), scaleMode_(EntityScaleMode::NONE)
{
	sensor_pos_[0] = 0;
	sensor_pos_[1] = 0;
	sensor_pos_[2] = 0;

	state_old.pos_x = 0;
	state_old.pos_y = 0;
	state_old.vel_x = 0;
	state_old.vel_y = 0;
	state_old.h = 0;
	state_old.h_rate = 0;

	trail_closest_pos_ = { 0, 0, 0, 0, 0, 0, false };

	// initialize override vector
	for (int i = 0; i < OVERRIDE_NR_TYPES; i++)
	{
		overrideActionList[i].type = (OverrideType)i;
		overrideActionList[i].value = 0.0;
		overrideActionList[i].active = false;
	}

	boundingbox_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	if (junctionSelectorStrategy_ == Junction::JunctionStrategyType::RANDOM)
	{
		SetJunctionSelectorAngleRandom();
	}
}

void Object::SetEndOfRoad(bool state, double time)
{
	if (state == true)
	{
		end_of_road_timestamp_ = time;
	}
	else
	{
		end_of_road_timestamp_ = 0.0;
	}
}

void Object::SetOffRoad(bool state, double time)
{
	if (state == true)
	{
		off_road_timestamp_ = time;
	}
	else
	{
		off_road_timestamp_ = 0.0;
	}
}

void  Object::SetStandStill(bool state, double time)
{
	if (state == true)
	{
		stand_still_timestamp_ = time;
	}
	else
	{
		stand_still_timestamp_ = 0.0;
	}
}

int Object::MoveAlongS(double ds, bool actualDistance)
{
	int retval = 0;

	if (GetAbsAngleDifference(pos_.GetH(), pos_.GetDrivingDirection()) > M_PI_2)
	{
		// If pointing in other direction
		ds *= -1;
	}

	if (pos_.GetRoute() && !pos_.GetRoute()->invalid_route_)
	{
		retval = static_cast<int>(pos_.MoveRouteDS(ds, actualDistance));
	}
	else
	{
		retval = static_cast<int>(pos_.MoveAlongS(ds, 0.0, GetJunctionSelectorAngle(), actualDistance));
	}

	return retval;
}

int Object::GetAssignedControllerType()
{
	if (controller_)
	{
		return controller_->GetType();
	}
	else
	{
		// Report 0 if not assigned or not activated on any domain
		return 0;
	}
}

int Object::GetActivatedControllerType()
{
	if (controller_ && (controller_->GetDomain() != ControlDomains::DOMAIN_NONE))
	{
		return controller_->GetType();
	}
	else
	{
		// Report 0 if not assigned or not activated on any domain
		if (IsGhost())
		{
			return Controller::Type::GHOST_RESERVED_TYPE;
		}
		else
		{
			return 0;
		}
	}
}

bool Object::IsControllerActiveOnDomains(ControlDomains domainMask)
{
	if (controller_)
	{
		return controller_->IsActiveOnDomains(domainMask);
	}
	else
	{
		return false;
	}
}

bool Object::IsControllerActiveOnAnyOfDomains(ControlDomains domainMask)
{
	if (controller_)
	{
		return controller_->IsActiveOnAnyOfDomains(domainMask);
	}
	else
	{
		return false;
	}
}

bool Object::IsControllerActive()
{
	if (controller_)
	{
		return controller_->IsActive();
	}
	else
	{
		return false;
	}
}

int Object::GetControllerMode()
{
	if (controller_)
	{
		return controller_->GetMode();
	}
	else
	{
		return 0;  // default
	}
}

void Object::SetVisibilityMask(int mask)
{
	visibilityMask_ = mask;
	SetDirtyBits(dirty_ | DirtyBit::VISIBILITY);
}

void Object::SetVel(double x_vel, double y_vel, double z_vel)
{
	pos_.SetVel(x_vel, y_vel, z_vel);
	SetDirtyBits(dirty_ | DirtyBit::VELOCITY);
}

void Object::SetAcc(double x_acc, double y_acc, double z_acc)
{
	pos_.SetAcc(x_acc, y_acc, z_acc);
	SetDirtyBits(dirty_ | DirtyBit::ACCELERATION);
}

void Object::SetAngularVel(double h_vel, double p_vel, double r_vel)
{
	pos_.SetAngularVel(h_vel, p_vel, r_vel);
	SetDirtyBits(dirty_ | DirtyBit::ANGULAR_RATE);
}

void Object::SetAngularAcc(double h_acc, double p_acc, double r_acc)
{
	pos_.SetAngularAcc(h_acc, p_acc, r_acc);
	SetDirtyBits(dirty_ | DirtyBit::ANGULAR_ACC);
}

void Object::SetJunctionSelectorAngle(double angle)
{
	if (std::isnan(angle))
	{
		nextJunctionSelectorAngle_ = angle;
	}
	else
	{
		nextJunctionSelectorAngle_ = GetAngleInInterval2PI(angle);
	}
}

void Object::SetJunctionSelectorAngleRandom()
{
	nextJunctionSelectorAngle_ = 2 * M_PI * ((double)(SE_Env::Inst().GetGenerator())()) / (SE_Env::Inst().GetGenerator()).max();
}

bool Object::CollisionAndRelativeDistLatLong(Object* target, double *distLat, double *distLong)
{
	// Apply method Separating Axis Theorem (SAT)
	// http://www.euclideanspace.com/threed/games/examples/cars/collisions/
	// https://www.sevenson.com.au/actionscript/sat/

	// Idea:
	// For each side of the bounding boxes:
	//   The normal of that edge will be the projection axis
	//   Project all points of the two bounding boxes onto that axis
	//   If we find ONE side with a gap between point clusters from BB1 and BB2,
	//   it's enough to conclude they are not overlapping/colliding
	//
	// Optimization: Since the bounding boxes are boxes with parallel
	// sides, we only need to check half of the sides

	if (target == 0)
	{
		return false;
	}

	Object* obj0 = this;
	Object* obj1 = target;
	bool gap = false;
	if (distLong) *distLong = 0.0;
	if (distLat) *distLat = 0.0;

	// First do a rough check to rule out potential overlap/collision
	// Compare radial/euclidean distance with sum of the diagonal dimension of the bounding boxes
	double x = 0, y = 0;
	double dist = fabs(this->pos_.getRelativeDistance(target->pos_.GetX(), target->pos_.GetY(), x, y));
	double max_length = this->boundingbox_.dimensions_.length_ + target->boundingbox_.dimensions_.length_;
	double max_width = this->boundingbox_.dimensions_.width_ + target->boundingbox_.dimensions_.width_;
	double dist_threshold = sqrt(max_length * max_length + max_width * max_width);

	if (dist > dist_threshold && distLong == nullptr && distLat == nullptr)
	{
		return false;
	}

	// Also do a Z sanity check, to rule out on different road elevations
	if (fabs(obj0->pos_.GetZ() - obj1->pos_.GetZ()) > ELEVATION_DIFF_THRESHOLD && distLong == nullptr && distLat == nullptr)
	{
		return false;
	}

	for (int i = 0; i < 2; i++)  // for each of the two BBs
	{
		if (i == 1)
		{
			// swap order, now check all edges of target object
			obj0 = target;
			obj1 = this;
		}

		double n0[2] = { 0.0, 0.0 };
		for (int j = 0; j < 2; j++)  // for longitudinal and lateral sides
		{
			if (j == 0)
			{
				// Normal for longitudinal side (sides) points along lateral direction
				n0[0] = 0.0;
				n0[1] = 1.0;
			}
			else
			{
				// Normal for lateral side (front/rear) points along lateral direction
				n0[0] = 1.0;
				n0[1] = 0.0;
			}

			// Rotate the normal/projection axis to align with bounding box/vehicle
			double n1[2] = { 0.0, 0.0 };
			RotateVec2D(n0[0], n0[1], obj0->pos_.GetH(), n1[0], n1[1]);

			// Now, project each point of each BB onto the rotated normal
			// And register min and max for point cluster of each BB
			double min[2] = { 0.0, 0.0 }, max[2] = { 0.0, 0.0 };
			for (int k = 0; k < 2; k++)
			{
				Object* obj = (k == 0 ? obj0 : obj1);

				// Specify bounding box corner vertices, starting at first quadrant
				double vertices[4][2] =
				{
					{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
					{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
					{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 },
					{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 }
				};

				for (int l = 0; l < 4; l++)
				{
					double point_to_project[2];

					// Align projection points to object heading
					RotateVec2D(vertices[l][0], vertices[l][1], obj->pos_.GetH(), point_to_project[0], point_to_project[1]);

					double dot_p = GetDotProduct2D(
						obj->pos_.GetX() + point_to_project[0],
						obj->pos_.GetY() + point_to_project[1],
						n1[0], n1[1]);

					if (l == 0)
					{
						min[k] = max[k] = dot_p;
					}
					else
					{
						min[k] = MIN(dot_p, min[k]);
						max[k] = MAX(dot_p, max[k]);
					}
				}
			}

			if (min[0] < min[1] - SMALL_NUMBER && max[0] < min[1] - SMALL_NUMBER ||
				max[0] > max[1] + SMALL_NUMBER && min[0] > max[1] + SMALL_NUMBER)
			{
				// gap found
				gap = true;
				if (distLong == nullptr && distLat == nullptr)
				{
					return !gap;
				}
				else
				{
					// measure gap relative pivot vehicle
					if (i == 0)
					{
						if (min[0] < min[1] - SMALL_NUMBER && max[0] < min[1] - SMALL_NUMBER)
						{
							if (j == 0)
							{
								if (distLat) *distLat = min[1] - max[0];
							}
							else
							{
								if (distLong) *distLong = min[1] - max[0];
							}
						}
						else
						{
							if (j == 0)
							{
								if (distLat) *distLat = -(min[0] - max[1]);
							}
							else
							{
								if (distLong) *distLong = -(min[0] - max[1]);
							}
						}
					}
				}
			}
		}
	}

	return !gap;
}

double Object::PointCollision(double x, double y)
{
	// Apply method Separating Axis Theorem (SAT)
	// http://www.euclideanspace.com/threed/games/examples/cars/collisions/
	// https://www.sevenson.com.au/actionscript/sat/

	// Idea:
	// For each side of the bounding box:
	//   The normal of that edge will be the projection axis
	//   Project the point and all axis of the bounding boxe onto that axis
	//   If we find ONE side with a gap between point clusters from BB1 and BB2,
	//   it's enough to conclude they are not overlapping/colliding
	//
	// Optimization: Since the bounding boxes are boxes with parallel
	// sides, we only need to check half of the sides

	Object* obj0 = this;

	double n0[2] = { 0.0, 0.0 };
	for (int j = 0; j < 2; j++)  // for longitudinal and lateral sides
	{
		if (j == 0)
		{
			// Normal for longitudinal side (sides) points along lateral side
			n0[0] = 0.0;
			n0[1] = 1.0;
		}
		else
		{
			// Normal for lateral side (front/rear) points along lateral side
			n0[0] = 1.0;
			n0[1] = 0.0;
		}

		// Rotate the normal/projection axis to align with bounding box/vehicle
		double n1[2] = { 0.0, 0.0 };
		RotateVec2D(n0[0], n0[1], obj0->pos_.GetH(), n1[0], n1[1]);

		// Now, project each point of each BB onto the rotated normal
		// And register min and max for point cluster of each BB
		double min[2] = { 0.0, 0.0 }, max[2] = { 0.0, 0.0 };

		// Specify bounding box corner vertices, starting at first quadrant
		double vertices[4][2] =
		{
			{ obj0->boundingbox_.center_.x_ + obj0->boundingbox_.dimensions_.length_ / 2.0, obj0->boundingbox_.center_.y_ + obj0->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj0->boundingbox_.center_.x_ - obj0->boundingbox_.dimensions_.length_ / 2.0, obj0->boundingbox_.center_.y_ + obj0->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj0->boundingbox_.center_.x_ - obj0->boundingbox_.dimensions_.length_ / 2.0, obj0->boundingbox_.center_.y_ - obj0->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj0->boundingbox_.center_.x_ + obj0->boundingbox_.dimensions_.length_ / 2.0, obj0->boundingbox_.center_.y_ - obj0->boundingbox_.dimensions_.width_ / 2.0 }
		};

		for (int l = 0; l < 4; l++)
		{
			double point_to_project[2];

			// Align projection points to object heading
			RotateVec2D(vertices[l][0], vertices[l][1], obj0->pos_.GetH(), point_to_project[0], point_to_project[1]);

			double dot_p = GetDotProduct2D(
				obj0->pos_.GetX() + point_to_project[0],
				obj0->pos_.GetY() + point_to_project[1],
				n1[0], n1[1]);

			if (l == 0)
			{
				min[0] = max[0] = dot_p;
			}
			else
			{
				min[0] = MIN(dot_p, min[0]);
				max[0] = MAX(dot_p, max[0]);
			}
		}

		double dot_p = GetDotProduct2D(x, y, n1[0], n1[1]);

		if (min[0] < dot_p - SMALL_NUMBER && max[0] < dot_p - SMALL_NUMBER ||
			max[0] > dot_p + SMALL_NUMBER && min[0] > dot_p + SMALL_NUMBER)
		{
			// gap found - no collision
			return false;
		}
	}

	return true;
}

double Object::FreeSpaceDistance(Object* target, double* latDist, double* longDist)
{
	double minDist = LARGE_NUMBER;
	*latDist = LARGE_NUMBER;
	*longDist = LARGE_NUMBER;

	if (target == 0)
	{
		return minDist;
	}

	if (CollisionAndRelativeDistLatLong(target, latDist, longDist))
	{
		return 0.0;
	}

	// OK, they are not overlapping. Now find the distance.
	// Strategy: Brute force check all vertices of one bounding box
	// against all sides of the other bounding box - then switch to
	// check vertices of the other bounding box against the sides
	// of the first bounding box.

	double vertices[2][4][2];


	for (int i = 0; i < 2; i++)  // for each of the two BBs
	{
		Object* obj = (i == 0 ? this : target);

		// Specify bounding box corner vertices, starting at first quadrant
		double vtmp[4][2] =
		{
			{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 }
		};

		for (int j = 0; j < 4; j++)  // for all vertices
		{
			// Align points to object heading and position
			RotateVec2D(vtmp[j][0], vtmp[j][1], obj->pos_.GetH(), vertices[i][j][0], vertices[i][j][1]);
			vertices[i][j][0] += obj->pos_.GetX();
			vertices[i][j][1] += obj->pos_.GetY();
		}
	}

	for (int i = 0; i < 2; i++)  // for each of the two BBs
	{
		int vindex = (i == 0 ? 0 : 1);

		for (int j = 0; j < 4; j++)  // for all vertices
		{
			double point[2] = { vertices[vindex][j][0], vertices[vindex][j][1] };

			for (int k = 0; k < 4; k++)  // for all sides/edges in the other bounding box
			{
				double edge[2][2];
				edge[0][0] = vertices[(vindex + 1) % 2][k][0];
				edge[0][1] = vertices[(vindex + 1) % 2][k][1];
				edge[1][0] = vertices[(vindex + 1) % 2][(k + 1) % 4][0];
				edge[1][1] = vertices[(vindex + 1) % 2][(k + 1) % 4][1];

				double xProj = 0;
				double yProj = 0;
				double tmpDist = DistanceFromPointToEdge2D(point[0], point[1], edge[0][0], edge[0][1], edge[1][0], edge[1][1], &xProj, &yProj);

				if (tmpDist < minDist)
				{
					minDist = tmpDist;
				}
			}
		}
	}

	return minDist;
}

double Object::FreeSpaceDistancePoint(double x, double y, double* latDist, double* longDist)
{
	double minDist = LARGE_NUMBER;
	*latDist = LARGE_NUMBER;
	*longDist = LARGE_NUMBER;

	if (PointCollision(x, y))
	{
		*latDist = 0.0;
		*longDist = 0.0;
		return 0.0;
	}

	// OK, they are not overlapping. Now find the distance.
	// Strategy: Brute force check point against all sides
	// of the bounding box

	Object* obj = this;

	double vertices[4][2];

	// Specify bounding box corner vertices, starting at first quadrant
	double vtmp[4][2] =
	{
		{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
		{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
		{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 },
		{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 }
	};

	for (int j = 0; j < 4; j++)  // for all vertices
	{
		// Align points to object heading and position
		RotateVec2D(vtmp[j][0], vtmp[j][1], obj->pos_.GetH(), vertices[j][0], vertices[j][1]);
		vertices[j][0] += obj->pos_.GetX();
		vertices[j][1] += obj->pos_.GetY();
	}


	double point[2] = { x, y };

	for (int k = 0; k < 4; k++)  // for all sides/edges of the bounding box
	{
		double edge[2][2];
		edge[0][0] = vertices[k][0];
		edge[0][1] = vertices[k][1];
		edge[1][0] = vertices[(k + 1) % 4][0];
		edge[1][1] = vertices[(k + 1) % 4][1];

		double xProj = 0;
		double yProj = 0;
		double tmpDist = DistanceFromPointToLine2D(point[0], point[1], edge[0][0], edge[0][1], edge[1][0], edge[1][1],
			&xProj, &yProj);

		if (tmpDist < minDist)
		{
			minDist = tmpDist;

			if (latDist && longDist)
			{
				// Calculate x, y components of the distance in vehicle reference system
				// y points left in vehicle ref system, x forward
				RotateVec2D(point[0] - xProj, point[1] - yProj, -this->pos_.GetH(), *longDist, *latDist);
			}
		}
	}

	return minDist;
}

int Object::FreeSpaceDistancePointRoadLane(double x, double y, double* latDist, double* longDist, CoordinateSystem cs)
{
	*latDist = LARGE_NUMBER;
	*longDist = LARGE_NUMBER;

	if (cs != CoordinateSystem::CS_LANE && cs != CoordinateSystem::CS_ROAD)
	{
		LOG("Unexpected coordinateSystem (%d). %d or %d expected.", CoordinateSystem::CS_LANE, CoordinateSystem::CS_ROAD);
		return -1;
	}

	if (cs == CoordinateSystem::CS_LANE)
	{
		LOG("freespace LANE coordinateSystem not supported yet, falling back to freespace ROAD");
		cs = CoordinateSystem::CS_ROAD;
	}

	// Specify bounding box corner vertices, starting at first quadrant
	double vtmp[4][2] =
	{
		{ boundingbox_.center_.x_ + boundingbox_.dimensions_.length_ / 2.0, boundingbox_.center_.y_ + boundingbox_.dimensions_.width_ / 2.0 },
		{ boundingbox_.center_.x_ - boundingbox_.dimensions_.length_ / 2.0, boundingbox_.center_.y_ + boundingbox_.dimensions_.width_ / 2.0 },
		{ boundingbox_.center_.x_ - boundingbox_.dimensions_.length_ / 2.0, boundingbox_.center_.y_ - boundingbox_.dimensions_.width_ / 2.0 },
		{ boundingbox_.center_.x_ + boundingbox_.dimensions_.length_ / 2.0, boundingbox_.center_.y_ - boundingbox_.dimensions_.width_ / 2.0 }
	};

	// Align points to object heading and position
	double vertices[4][3];
	for (int i = 0; i < 4; i++)
	{
		RotateVec2D(vtmp[i][0], vtmp[i][1], pos_.GetH(), vertices[i][0], vertices[i][1]);
		vertices[i][0] += pos_.GetX();
		vertices[i][1] += pos_.GetY();
		vertices[i][2] = pos_.GetH();
	}

	// Map XY point to road coordinates, but consider only roads reachable from point
	Position pointPos = pos_;
	if (pointPos.XYZH2TrackPos(x, y, 0, 0, true) != Position::ErrorCode::ERROR_NO_ERROR)
	{
		return -1;
	}

	// Find long and lat max values
	Position pos[4];
	double maxDS = 0.0;
	double minDS = LARGE_NUMBER;
	double maxDT = 0.0;
	double minDT = LARGE_NUMBER;
	PositionDiff posDiff;
	for (int j = 0; j < 4; j++)
	{
		pos[j] = pos_;
		// Map bounding box points to road coordinates, consider only roads reachable from current position
		if (pos[j].XYZH2TrackPos(vertices[j][0], vertices[j][1], 0, vertices[j][2], true) != Position::ErrorCode::ERROR_NO_ERROR)
		{
			return -1;
		}

		if (pos[j].Delta(&pointPos, posDiff) == false)
		{
			return -1;
		}

		if (j == 0 || fabs(posDiff.ds) < fabs(minDS))
		{
			minDS = posDiff.ds;
		}

		if (j == 0 || fabs(posDiff.dt) < fabs(minDT))
		{
			minDT = posDiff.dt;
		}

		if (j == 0 || fabs(posDiff.ds) > fabs(maxDS))
		{
			maxDS = posDiff.ds;
		}

		if (j == 0 || fabs(posDiff.dt) > fabs(maxDT))
		{
			maxDT = posDiff.dt;
		}
	}

	*longDist = minDS;
	*latDist = minDT;

	// Check for overlap
	if (SIGN(minDS) != SIGN(maxDS))
	{
		// Overlap
		*longDist = 0.0;
	}
	if (SIGN(minDT) != SIGN(maxDT))
	{
		// Overlap
		*latDist = 0.0;
	}

	return 0;
}

int Object::FreeSpaceDistanceObjectRoadLane(Object* target, double* latDist, double* longDist, CoordinateSystem cs)
{
	*latDist = LARGE_NUMBER;
	*longDist = LARGE_NUMBER;

	// First some checks
	if (target == 0)
	{
		return -1;
	}

	if (cs != CoordinateSystem::CS_LANE && cs != CoordinateSystem::CS_ROAD)
	{
		LOG("Unexpected coordinateSystem (%d). %d or %d expected.", CoordinateSystem::CS_LANE, CoordinateSystem::CS_ROAD);
		return -1;
	}

	if (cs == CoordinateSystem::CS_LANE)
	{
		LOG("freespace LANE coordinateSystem not supported yet, falling back to freespace ROAD");
		cs = CoordinateSystem::CS_ROAD;
	}

	if (Collision(target))
	{
		*longDist = 0.0;
		*latDist = 0.0;
		return 0;
	}


	// OK, they are not overlapping (colliding). Now find the distance.
	// Strategy: Brute force check all vertices of one bounding box
	// against all sides of the other bounding box - then switch to
	// check vertices of the other bounding box against the sides
	// of the first bounding box.

	double vertices[2][4][3];
	Position pos[2][4];

	for (int i = 0; i < 2; i++)  // for each of the two BBs
	{
		Object* obj = (i == 0 ? this : target);

		// Specify bounding box corner vertices, starting at first quadrant
		double vtmp[4][2] =
		{
			{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 },
			{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 }
		};

		for (int j = 0; j < 4; j++)  // for all vertices
		{
			// Align points to object heading and position
			RotateVec2D(vtmp[j][0], vtmp[j][1], obj->pos_.GetH(), vertices[i][j][0], vertices[i][j][1]);
			vertices[i][j][0] += obj->pos_.GetX();
			vertices[i][j][1] += obj->pos_.GetY();
			vertices[i][j][2] = obj->pos_.GetH();

			// Map XY points to road coordinates, but consider only roads reachable from point
			pos[i][j] = pos_;
			if (pos[i][j].XYZH2TrackPos(vertices[i][j][0], vertices[i][j][1], 0, vertices[i][j][2], true) != Position::ErrorCode::ERROR_NO_ERROR)
			{
				return -1;
			}
		}
	}

	// Find long and lat max values
	double maxDS = 0.0;
	double minDS = LARGE_NUMBER;
	double maxDT = 0.0;
	double minDT = LARGE_NUMBER;
	PositionDiff posDiff;

	double ds[4][4];  // delta s between every vertex on first bb to every vertex on second bb
	double dt[4][4];  // delta t between every vertex on first bb to every vertex on second bb

	for (int i = 0; i < 4; i++)  // for each vertex of first BBs
	{
		for (int j = 0; j < 4; j++)  // for each vertex of second BBs
		{
			if (pos[0][i].Delta(&pos[1][j], posDiff) == false)
			{
				return -1;
			}
			ds[i][j] = posDiff.ds;
			dt[i][j] = posDiff.dt;

			if (i == 0 && j == 0 || fabs(posDiff.ds) < fabs(minDS))
			{
				minDS = posDiff.ds;
			}

			if (i == 0 && j == 0 || fabs(posDiff.dt) < fabs(minDT))
			{
				minDT = posDiff.dt;
			}

			if (i == 0 && j == 0 || fabs(posDiff.ds) > fabs(maxDS))
			{
				maxDS = posDiff.ds;
			}

			if (i == 0 && j == 0 || fabs(posDiff.dt) > fabs(maxDT))
			{
				maxDT = posDiff.dt;
			}
		}
	}

	*longDist = minDS;
	*latDist = minDT;

	// Check for overlap
	for (int i = 0; i < 4; i++)  // for each ds
	{
		if (!((SIGN(ds[i][0]) == SIGN(ds[i][1])) && (SIGN(ds[i][0]) == SIGN(ds[i][2])) && (SIGN(ds[i][0]) == SIGN(ds[i][3]))))
		{
			// Overlap
			*longDist = 0.0;
		}
		if (!((SIGN(dt[i][0]) == SIGN(dt[i][1])) && (SIGN(dt[i][0]) == SIGN(dt[i][2])) && (SIGN(dt[i][0]) == SIGN(dt[i][3]))))
		{
			// Overlap
			*latDist = 0.0;
		}
	}

	return 0;
}

int Object::Distance(Object* target, roadmanager::CoordinateSystem cs, roadmanager::RelativeDistanceType relDistType, bool freeSpace,
	double& dist, double maxDist)
{
	if (freeSpace)
	{
		double latDist, longDist;

		if (cs == roadmanager::CoordinateSystem::CS_ENTITY ||
			relDistType == RelativeDistanceType::REL_DIST_EUCLIDIAN ||
			relDistType == RelativeDistanceType::REL_DIST_CARTESIAN)
		{
			// Get Cartesian/Euclidian distance
			dist = FreeSpaceDistance(target, &latDist, &longDist);

			// sign indicates target being in front (+) or behind (-)
			dist *= SIGN(longDist);

			if (cs == roadmanager::CoordinateSystem::CS_ENTITY)
			{
				if (relDistType == roadmanager::RelativeDistanceType::REL_DIST_LATERAL)
				{
					dist = latDist;
				}
				else if (relDistType == roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL)
				{
					dist = longDist;
				}
			}
		}
		else if (cs == CoordinateSystem::CS_ROAD || cs == CoordinateSystem::CS_LANE)
		{
			if (FreeSpaceDistanceObjectRoadLane(target, &latDist, &longDist, cs) != 0)
			{
				return -1;
			}
			else
			{
				if (relDistType == RelativeDistanceType::REL_DIST_LATERAL)
				{
					dist = latDist;
				}
				else if (relDistType == RelativeDistanceType::REL_DIST_LONGITUDINAL)
				{
					dist = longDist;
				}
				else
				{
					LOG("Unexpected relativeDistanceType: %d", relDistType);
					return -1;
				}
			}
		}
		else
		{
			LOG("Unhandled case: cs %d reDistType %d freeSpace %d\n", cs, relDistType, freeSpace);
			return -1;
		}
	}
	else  // not freeSpace
	{
		return pos_.Distance(&target->pos_, cs, relDistType, dist);
	}

	return 0;
}

int Object::Distance(double x, double y, roadmanager::CoordinateSystem cs, roadmanager::RelativeDistanceType relDistType, bool freeSpace,
	double& dist, double maxDist)
{
	if (freeSpace)
	{
		double latDist, longDist;

		if (cs == roadmanager::CoordinateSystem::CS_ENTITY ||
			relDistType == RelativeDistanceType::REL_DIST_EUCLIDIAN ||
			relDistType == RelativeDistanceType::REL_DIST_CARTESIAN)
		{
			// Get Cartesian/Euclidian distance
			dist = FreeSpaceDistancePoint(x, y, &latDist, &longDist);

			// sign indicates target being in front (+) or behind (-)
			dist *= SIGN(longDist);

			if (cs == roadmanager::CoordinateSystem::CS_ENTITY)
			{
				if (relDistType == roadmanager::RelativeDistanceType::REL_DIST_LATERAL)
				{
					dist = latDist;
				}
				else if (relDistType == roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL)
				{
					dist = longDist;
				}
			}
		}
		else if (cs == CoordinateSystem::CS_ROAD || cs == CoordinateSystem::CS_LANE)
		{
			if (FreeSpaceDistancePointRoadLane(x, y, &latDist, &longDist, cs) != 0)
			{
				return -1;
			}
			else
			{
				if (relDistType == RelativeDistanceType::REL_DIST_LATERAL)
				{
					dist = latDist;
				}
				else if (relDistType == RelativeDistanceType::REL_DIST_LONGITUDINAL)
				{
					dist = longDist;
				}
				else
				{
					LOG("Unexpected relativeDistanceType: %d", relDistType);
					return -1;
				}
			}
		}
		else
		{
			LOG("Unhandled case: cs %d reDistType %d freeSpace %d\n", cs, relDistType, freeSpace);
			return -1;
		}
	}
	else  // not freeSpace
	{
		return pos_.Distance(x, y, cs, relDistType, dist, maxDist);
	}

	return 0;
}

int Entities::addObject(Object* obj)
{
	obj->id_ = getNewId();
	object_.push_back(obj);
	return obj->id_;
}

void Entities::removeObject(int id)
{
	for (size_t i = 0; i < object_.size(); i++)
	{
		if (object_[i]->id_ == id)
		{
			object_.erase(object_.begin() + i);
		}
	}
}

void Entities::removeObject(std::string name)
{
	for (size_t i = 0; i < object_.size(); i++)
	{
		if (object_[i]->name_ == name)
		{
			object_.erase(object_.begin() + i);
		}
	}
}

bool Entities::nameExists(std::string name)
{
	for (size_t i = 0; i < object_.size(); i++)
	{
		if (object_[i]->name_ == name)
		{
			return true;
		}
	}
	return false;
}

bool Entities::indexExists(int id)
{
	for (size_t i = 0; i < object_.size(); i++)
	{
		if (object_[i]->id_ == id)
		{
			return false;
		}
	}
	return true;
}

int Entities::getNewId()
{
	return nextId_++;
}

Object* Entities::GetObjectByName(std::string name)
{
	for (size_t i = 0; i < object_.size(); i++)
	{
		if (name == object_[i]->name_)
		{
			return object_[i];
		}
	}

	LOG("Failed to find object %s", name.c_str());

	return 0;
}

Object* Entities::GetObjectById(int id)
{
	for (size_t i = 0; i < object_.size(); i++)
	{
		if (id == object_[i]->id_)
		{
			return object_[i];
		}
	}

	LOG("Failed to find object with id %d", id);

	return 0;
}

void Object::removeEvent(Event* event)
{
	auto it = std::find(objectEvents_.begin(), objectEvents_.end(), event);
		if (it != objectEvents_.end())
			objectEvents_.erase(it);
}

std::vector<OSCPrivateAction*> Object::getPrivateActions() {
	std::vector<OSCPrivateAction*> actions;
	for (int i = 0; i < objectEvents_.size(); i++)
	{
		Event* event = objectEvents_[i];
		for (size_t n = 0; n < event->action_.size(); n++)
		{
			OSCAction* action = event->action_[n];
			if (action->base_type_ == OSCAction::BaseType::PRIVATE)
			{
				OSCPrivateAction* pa = (OSCPrivateAction*)action;
				if (pa->IsActive() || pa->IsTriggable()) {
					actions.push_back(pa);
				}
			}
		}
	}
	return actions;
}