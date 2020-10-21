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
#include "Trail.hpp"
#include "RoadManager.hpp"

#define TRAIL_DT 0.5

using namespace scenarioengine;


void ObjectTrail::AddState(float timestamp, float x, float y, float z, float speed)
{
	ObjectTrailState *previous_state = 0;
	
	if (n_states_ > 0)
	{
		previous_state = GetStateLast();

		// Check timestamp of previous state - add only if delta time has passed
		if (previous_state && timestamp < previous_state->timeStamp_ + TRAIL_DT)
		{
			return;
		}
	}

	state_[current_].timeStamp_ = timestamp;
	state_[current_].x_ = x;
	state_[current_].y_ = y;
	state_[current_].z_ = z;
	state_[current_].speed_ = speed;

	if (previous_state)
	{
		if (PointSquareDistance2D(state_[current_].x_, state_[current_].y_, previous_state->x_, previous_state->y_) > SMALL_NUMBER)
		{
			// Now when direction is defined by new point, add heading to previous segment
			previous_state->h_ = (float)GetAngleOfVector(state_[current_].x_ - previous_state->x_, state_[current_].y_ - previous_state->y_);
		}

		state_[current_].h_ = previous_state->h_;  // set heading of last point to same as previous segment
	}
	else
	{
		state_[current_].h_ = 0;  // First point, direction not defined yet
	}

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
	int lastIndex = current_ - 1;
	if (lastIndex < 0)
	{
		lastIndex = n_states_ - 1;
	}
	return &state_[lastIndex];
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
	return (x - state_[idx].x_) * (x - state_[idx].x_) + (y - state_[idx].y_) * (y - state_[idx].y_);
}

double ObjectTrail::GetSegmentlength(int index)
{
	if (n_states_ < 2)
	{
		return 0;
	}

	int next_index = GetNextSegmentIndex(index);
	return sqrt(pow(state_[next_index].x_ - state_[index].x_, 2) + pow(state_[next_index].y_ - state_[index].y_, 2));
}

void ObjectTrail::GetPointOnSegmentBySNorm(int index, double s, double &x, double &y, double &z)
{
	int next_index = GetNextSegmentIndex(index);

	x = state_[index].x_ + s * (state_[next_index].x_ - state_[index].x_);
	y = state_[index].y_ + s * (state_[next_index].y_ - state_[index].y_);
	z = state_[index].z_ + s * (state_[next_index].z_ - state_[index].z_);
}

void ObjectTrail::GetPointOnSegmentByDist(int index, double dist, double &x, double &y, double &z)
{
	if (n_states_ < 1)
	{
		return;
	}
	if (index >= n_states_)
	{
		index = n_states_ - 1;
	}

	double segment_length = MAX(GetSegmentlength(index), SMALL_NUMBER);

	double s = MIN(1.0, dist / segment_length);

	GetPointOnSegmentBySNorm(index, s, x, y, z);
}


void ObjectTrail::GetSpeedOnSegmentBySNorm(int index, double s, double &speed)
{
	int next_index = GetNextSegmentIndex(index);

	speed = state_[index].speed_ + s * (state_[next_index].speed_ - state_[index].speed_);
}

void ObjectTrail::GetSpeedOnSegmentByDist(int index, double dist, double &speed)
{
	if (n_states_ < 1)
	{
		return;
	}
	if (index >= n_states_)
	{
		index = n_states_ - 1;
	}

	double segment_length = MAX(GetSegmentlength(index), SMALL_NUMBER);

	double s = MIN(1.0, dist / segment_length);

	GetSpeedOnSegmentBySNorm(index, s, speed);
}

void ObjectTrail::GetHeadingOnSegmentBySNorm(int index, double s, double &heading)
{
	int next_index = GetNextSegmentIndex(index);

	heading = state_[index].h_ + s * (state_[next_index].h_ - state_[index].h_);
}

void ObjectTrail::GetHeadingOnSegmentByDist(int index, double dist, double &heading)
{
	if (n_states_ < 1)
	{
		return;
	}
	if (index >= n_states_)
	{
		index = n_states_ - 1;
	}

	double segment_length = GetSegmentlength(index);

	if (abs(segment_length) < SMALL_NUMBER)
	{
		// Use heading from previous state
		heading = state_[GetPreviousSegmentIndex(index)].h_;
	}
	else
	{
		double s = MIN(1.0, dist / segment_length);
		GetHeadingOnSegmentBySNorm(index, s, heading);
	}
}

int ObjectTrail::FindPointAhead(int index_start, double s_start, double distance, ObjectTrailState &state, int &index_out, double &s_out)
{
	int i = index_start;
	double dist_tmp = distance + s_start;

	if (n_states_ == 0)
	{
		state.x_ = state.y_ = state.z_ = state.speed_ = 0;
		return 0;
	}

	// Find segment containing the point of interest
	for (int count = 0; count < n_states_; count++)
	{
		double segment_length = GetSegmentlength(i);
		int next_index = GetNextSegmentIndex(i);

		if (dist_tmp < segment_length || next_index == i)
		{
			if (next_index == i)
			{
				// at end of trail - just use end point
				state.x_ = state_[i].x_;
				state.y_ = state_[i].y_;
				state.z_ = state_[i].z_;
				state.speed_ = state_[i].speed_;
				if (n_states_ > 1)
				{
					state.h_ = state_[i].h_;
				}
				s_out = segment_length;
			}
			else
			{
				// point is at this segment - find interpolated value
				double x;
				double y;
				double z;
				double speed;
				double h;
				GetPointOnSegmentByDist(i, dist_tmp, x, y, z);
				GetSpeedOnSegmentByDist(i, dist_tmp, speed);
				GetHeadingOnSegmentByDist(i, dist_tmp, h);

				state.x_ = (float)x;
				state.y_ = (float)y;
				state.z_ = (float)z;
				state.speed_ = (float)speed;
				state.h_ = (float)h;

				s_out = dist_tmp;
			}
			index_out = i;
			return 0;
		}

		dist_tmp -= segment_length;
		i = next_index;
	}
	
	return -1;
}

int ObjectTrail::FindClosestPoint(double x0, double y0, double &x, double &y, double &s, int &idx, int start_search_index)
{

	if (n_states_ <= 0)
	{
		x = y = 0;
		idx = 0;
		return -1;
	}
	else if (n_states_ == 1)
	{
		x = state_[0].x_;
		y = state_[0].y_;
		s = 0;
		idx = 0;
		return 0;
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
		double segmentLength = GetSegmentlength(i);
		int next_index = GetNextSegmentIndex(i);

		//LOG("i %d next %d of %d", i, next_index, n_states_);
		if (next_index == i)
		{
			// last segment
			break;
		}

		double x1 = state_[i].x_;
		double x2 = state_[next_index].x_;
		double y1 = state_[i].y_;
		double y2 = state_[next_index].y_;

		if (segmentLength < SMALL_NUMBER)
		{
			i = next_index;
			// Segment too small, skip and go forward along trail
			continue;
		}

		// Find vector from point perpendicular to line segment
		ProjectPointOnVector2D(x0, y0, x1, y1, x2, y2, x4, y4);

		// Check whether the projected point is inside or outside line segment
		inside = PointInBetweenVectorEndpoints(x4, y4, x1, y1, x2, y2, sNorm);

		if (inside)
		{
			// Distance between given poin<t and that point projected on the straight line
			dist = PointDistance2D(x4, y4, x0, y0);
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
			x = x1 + sNorm * (x2 - x1);
			y = y1 + sNorm * (y2 - y1);
			s = sNorm * segmentLength;
		}
		else if (dist > distMin)
		{
			// Distance growing, no need to search further
			break;
		}

		i = next_index;
	}

	return 0;
}

