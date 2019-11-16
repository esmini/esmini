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

#include "CommonMini.hpp"
#include "trail.hpp"
#include "RoadManager.hpp"

#define TRAIL_DT 0.2

using namespace scenarioengine;


void ObjectTrail::AddState(float timestamp, float x, float y, float z, float speed)
{
	state_[current_].timeStamp_ = timestamp;
	state_[current_].x_ = x;
	state_[current_].y_ = y;
	state_[current_].z_ = z;
	state_[current_].speed_ = speed;

	current_ = (current_ + 1) % TRAIL_MAX_STATES;

	if (n_states_ == TRAIL_MAX_STATES - 1)
	{
		LOG("Trace array now full (%d entries) - for next entry buffer will wrap around", n_states_ + 1);
	}

	n_states_ = MIN(TRAIL_MAX_STATES, n_states_ + 1);
}

ObjectTrailState* ObjectTrail::GetStateByTime(float timestamp)
{
	ObjectTrailState *state = 0;
	int tmp_index = 0;

	if (n_states_ <= 0)
	{
		return state;
	}

	float time = state_[0].timeStamp_;

	while (time < timestamp && tmp_index < n_states_ - 1)
	{
		tmp_index++;
		time = state_[tmp_index].timeStamp_;
	}

	return &state_[tmp_index];
}

ObjectTrailState* ObjectTrail::GetStateByIndex(int idx)
{
	if (n_states_ <= 0 || idx < 0 || idx > n_states_ - 1)
	{
		return 0;
	}

	return &state_[idx];
}

ObjectTrailState* ObjectTrail::GetStateLast()
{
	if (n_states_ <= 0)
	{
		return 0;
	}

	return &state_[n_states_ - 1];
}

int ObjectTrail::GetNextSegmentIndex(int index)
{
	if (n_states_ == 0)
	{
		// No elements
		return 0;
	}

	int next_index_candidate = index + 1;

	if (index == n_states_ - 1)
	{
		if (n_states_ == TRAIL_MAX_STATES)
		{
			// wrap around
			next_index_candidate = 0;
		}
		else
		{
			// At last index - no next
			return index;
		}
	}

	if (next_index_candidate == current_)
	{
		// Reached end of ring buffer - stay at last entry
		next_index_candidate = index;
	}

	return next_index_candidate;
}

int ObjectTrail::GetPreviousSegmentIndex(int index)
{
	int previous_index_candidate = index - 1;

	if (index == current_)
	{
		// at first entry, no previous available
		return index;
	}
	else if (n_states_ == 0)
	{
		// No elements
		return 0;
	}
	else if (index == 0)
	{
		if (n_states_ == TRAIL_MAX_STATES)
		{
			// All buckets in use, previous is last index
			previous_index_candidate = n_states_ - 1;
		}
		else
		{
			// at first entry, no previous available
			return 0;
		}
	}

	// Default case, return previous index
	return previous_index_candidate;
}

double ObjectTrail::QuadDistToPoint(double x, double y, int idx)
{
	return (x - state_[idx].x_ ) * (x - state_[idx].x_) + (y - state_[idx].y_) * (y - state_[idx].y_);
}

double ObjectTrail::GetSegmentlength(int index)
{
	if (index >= n_states_)
	{
		return 0;
	}

	int next_index = GetNextSegmentIndex(index);
	return sqrt(pow(state_[next_index].x_ - state_[index].x_, 2) + pow(state_[next_index].y_ - state_[index].y_, 2));
}

void ObjectTrail::GetPointOnSegmentByS(int index, double s, double &x, double &y, double &z)
{
	int next_index = GetNextSegmentIndex(index);

	x = state_[index].x_ + s * (state_[next_index].x_ - state_[index].x_);
	y = state_[index].y_ + s * (state_[next_index].y_ - state_[index].y_);
}

void ObjectTrail::GetPointOnSegmentByDist(int index, double dist, double &x, double &y, double &z)
{
	if (index >= n_states_)
	{
		index = n_states_ - 1;
	}

	double segment_length = MAX(GetSegmentlength(index), SMALL_NUMBER);
	
	double s = MIN(1.0, dist / segment_length);

	GetPointOnSegmentByS(index, s, x, y, z);
}

int ObjectTrail::FindPointAhead(int index_start, double s_start, double distance, double &x, double &y, double &z, double &speed, int &index_out, double &s_out)
{
	int i = index_start;
	double dist_tmp = distance;

	if (n_states_ == 0)
	{
		x = y = z = speed = 0;
		return 0;
	}

	// Find segment containing the point of interest
	for (int count = 0; count < n_states_; count++)
	{
		double segment_length = GetSegmentlength(i);

		if (dist_tmp < segment_length)
		{
			// point is at this segment
			GetPointOnSegmentByDist(i, dist_tmp, x, y, z);
			speed = state_[i].speed_;
			index_out = i;
			s_out = dist_tmp;
			return 0;
		}
		
		int next_index = GetNextSegmentIndex(i);
		if (next_index == i)
		{
			// at end of trail - just use end point
			x = state_[i].x_;
			y = state_[i].y_;
			z = state_[i].z_;

			return 0;
		}

		dist_tmp -= segment_length;
		i = next_index;
	}

	LOG("Point not found!");
	
	return -1;
}

int ObjectTrail::FindClosestPoint(double x0, double y0, double &x, double &y, double &s, int &idx, int start_search_index)
{

	if (n_states_ <= 0)
	{
		x = y = 0;
		return -1;
	}
	else if (n_states_ == 1)
	{
		x = state_[0].x_;
		y = state_[0].y_;
		s = 0;
		return -1;
	}
	
	// Strategy: Find closest line segment
	// by projecting current position on line segments
	// if outside segment, then measure to endpoints
	// starting with last known segment,
	// then look one segment forward and backward from there

	// Evaluate line endpoints to get a straight line in between
	double x4, y4;
	double dist, sNorm;
	double distMin = std::numeric_limits<double>::infinity();
	bool inside;
	int i = GetPreviousSegmentIndex(start_search_index);

	for (int count = 0; count<10; count++)
	{
		int next_index = GetNextSegmentIndex(i);
		double x1 = state_[i].x_;
		double x2 = state_[next_index].x_;
		double y1 = state_[i].y_;
		double y2 = state_[next_index].y_;

		// Find vector from point perpendicular to line segment
		ProjectPointOnVector2D(x0, y0, x1, y1, x2, y2, x4, y4);

		// Check whether the projected point is inside or outside line segment
		inside = PointInBetweenVectorEndpoints(x4, y4, x1, y1, x2, y2, sNorm);

		if (inside)
		{
			// Distance between given point and that point projected on the straight line
			dist = PointDistance2D(x0, y0, x4, y4);
		}
		else
		{
			// Distance is measured between point to closest endpoint of line
			double d1, d2;

			d1 = PointDistance2D(x0, y0, x1, y1);
			d2 = PointDistance2D(x0, y0, x2, y2);
			if (d1 < d2)
			{
				dist = d1;
				sNorm = 0;
			}
			else
			{
				dist = d2;
				sNorm = 1;
			}
		}
		if (dist < distMin)
		{
			distMin = dist;
			idx = i;
			s = sNorm;
			x = x1 + s * (x2 - x1);
			y = y1 + s * (y2 - y1);
		}

		i = next_index;
	}

	return 0;
}

