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

#include "RoadManager.hpp"

#define TRAIL_MAX_STATES 4096

namespace scenarioengine
{
	typedef struct
	{
		float timeStamp_;
		float x_;
		float y_;
		float z_;
		float h_;
		float speed_;
	} ObjectTrailState;
	
	class ObjectTrail
	{
	public:

		ObjectTrailState state_[TRAIL_MAX_STATES];
		int n_states_;
		int current_;

		ObjectTrail() : n_states_(0), current_(0) {}
		void AddState(float timestamp, float x, float y, float z, float speed);
		ObjectTrailState* GetStateByTime(float timestamp);
		ObjectTrailState* GetStateLast();
		int GetNextSegmentIndex(int index);
		int GetPreviousSegmentIndex(int index);
		double GetSegmentlength(int index);
		ObjectTrailState* GetStateByIndex(int index);
		void GetPointOnSegmentByDist(int index, double dist, double &x, double &y, double &z);
		void GetPointOnSegmentBySNorm(int index, double s, double &x, double &y, double &z);
		void GetSpeedOnSegmentByDist(int index, double dist, double &speed);
		void GetSpeedOnSegmentBySNorm(int index, double s, double &speed);
		void GetHeadingOnSegmentByDist(int index, double dist, double &heading);
		void GetHeadingOnSegmentBySNorm(int index, double s, double &heading);
		double QuadDistToPoint(double x, double y, int idx);
		/**
		Get information suitable for driver modeling of a point along specified trail from a specified start point
		@param trail The trail
		@param from_segment The segment index of the start point
		@param from_s Distance along the start segment from which to measure from
		@param distance The distance along the trail to the target position
		@param data Struct to fill in calculated values, see typdef for details
		@return 0 if successful, -1 if not
		*/
		int FindPointAhead(int index_start, double s_start, double distance, ObjectTrailState &state, int &index_out, double &s_out);


		int FindClosestPoint(double x0, double y0, double &x, double &y, double &s, int &idx, int start_search_index);
	};

}
