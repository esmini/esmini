#include "LaneSection.hpp"

void LaneSection::Print() {
	LOG("LaneSection: %.2f, %d lanes:\n", s_, (int)lane_.size());

	for (size_t i = 0; i < lane_.size(); i++) {
		lane_[i]->Print();
	}
}

void LaneSection::Save(pugi::xml_node& lanes) {
	auto laneSection = lanes.append_child("laneSection");
	laneSection.append_attribute("s").set_value(s_);
	if (singleSide_)  // Value initialized
	{
		if (singleSide_.value())  // Check contained value
		{
			laneSection.append_attribute("singleSide").set_value("true");
		} else {
			laneSection.append_attribute("singleSide").set_value("false");
		}
	}
	assert(!lane_.empty());
	for (auto lane : lane_) {
		lane->Save(laneSection);
	}

	for (auto userData : user_data_) {
		userData->Save(laneSection);
	}
}

Lane* LaneSection::GetLaneByIdx(int idx) {
	if (idx < (int)lane_.size()) {
		return lane_[idx];
	}

	return 0;
}

bool LaneSection::IsOSILaneById(int id) {
	Lane* lane = GetLaneById(id);
	if (lane == 0) {
		return false;
	} else {
		return !lane->IsCenter();
	}
}

Lane* LaneSection::GetLaneById(int id) {
	for (size_t i = 0; i < lane_.size(); i++) {
		if (lane_[i]->GetId() == id) {
			return lane_[i];
		}
	}
	return 0;
}

int LaneSection::GetLaneIdByIdx(int idx) {
	if (idx > (int)lane_.size() - 1) {
		LOG("LaneSection::GetLaneIdByIdx Error: index %d, only %d lanes\n", idx, (int)lane_.size());
		return 0;
	} else {
		return (lane_[idx]->GetId());
	}
}

int LaneSection::GetLaneIdxById(int id) {
	for (int i = 0; i < (int)lane_.size(); i++) {
		if (lane_[i]->GetId() == id) {
			return i;
		}
	}
	return -1;
}

int LaneSection::GetLaneGlobalIdByIdx(int idx) {
	if (idx < 0 || idx > (int)lane_.size() - 1) {
		LOG("LaneSection::GetLaneIdByIdx Error: index %d, only %d lanes\n", idx, (int)lane_.size());
		return 0;
	} else {
		return (lane_[idx]->GetGlobalId());
	}
}
int LaneSection::GetLaneGlobalIdById(int id) {
	for (size_t i = 0; i < (int)lane_.size(); i++) {
		if (lane_[i]->GetId() == id) {
			return lane_[i]->GetGlobalId();
		}
	}
	return -1;
}

int LaneSection::GetNumberOfDrivingLanes() {
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++) {
		if (lane_[i]->IsDriving()) {
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNumberOfDrivingLanesSide(int side) {
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++) {
		if (SIGN(lane_[i]->GetId()) == SIGN(side) && lane_[i]->IsDriving()) {
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNUmberOfLanesRight() {
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++) {
		if (lane_[i]->GetId() < 0) {
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNUmberOfLanesLeft() {
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++) {
		if (lane_[i]->GetId() > 0) {
			counter++;
		}
	}
	return counter;
}

double LaneSection::GetWidth(double s, int lane_id) {
	if (lane_id == 0) {
		return 0.0;	 // reference lane has no width
	}

	// Enforce s within range of section
	s = CLAMP(s, s_, s_ + GetLength());

	Lane* lane = GetLaneById(lane_id);
	if (lane == 0) {
		return 0.0;
	}

	LaneWidth* lane_width = lane->GetWidthByS(s - s_);
	if (lane_width == 0)  // No lane width registered
	{
		return 0.0;
	}

	// Calculate local s-parameter in width segment
	double ds = s - (s_ + lane_width->GetSOffset());

	// Calculate width at local s
	return lane_width->poly3_.Evaluate(ds);
}

double LaneSection::GetOuterOffset(double s, int lane_id) {
	if (lane_id == 0) {
		return 0;
	}

	double width = GetWidth(s, lane_id);

	if (abs(lane_id) == 1) {
		// this is the last lane, next to reference lane of width = 0. Stop here.
		return width;
	} else {
		int step = lane_id < 0 ? +1 : -1;
		return (width + GetOuterOffset(s, lane_id + step));
	}
}

double LaneSection::GetCenterOffset(double s, int lane_id) {
	if (lane_id == 0) {
		// Reference lane (0) has no width
		return 0.0;
	}
	double outer_offset = GetOuterOffset(s, lane_id);
	double width = GetWidth(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return outer_offset - width / 2;
}

double LaneSection::GetOuterOffsetHeading(double s, int lane_id) {
	if (lane_id == 0) {
		return 0.0;	 // reference lane has no width
	}

	Lane* lane = GetLaneById(lane_id);
	if (lane == 0) {
		return 0.0;
	}

	LaneWidth* lane_width = lane->GetWidthByS(s - s_);
	if (lane_width == 0)  // No lane width registered
	{
		return 0.0;
	}

	// Calculate local s-parameter in width segment
	double ds = s - (s_ + lane_width->GetSOffset());

	// Calculate heading at local s
	double heading = lane_width->poly3_.EvaluatePrim(ds);

	if (abs(lane_id) == 1) {
		// this is the last lane, next to reference lane of width = 0. Stop here.
		return heading;
	} else {
		int step = lane_id < 0 ? +1 : -1;
		return (heading + GetOuterOffsetHeading(s, lane_id + step));
	}
}

double LaneSection::GetCenterOffsetHeading(double s, int lane_id) {
	int step = lane_id < 0 ? +1 : -1;

	if (lane_id == 0) {
		// Reference lane (0) has no width
		return 0.0;
	}
	double inner_offset_heading = GetOuterOffsetHeading(s, lane_id + step);
	double outer_offset_heading = GetOuterOffsetHeading(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return (inner_offset_heading + outer_offset_heading) / 2;
}

void LaneSection::AddLane(Lane* lane) {
	lane->SetGlobalId();
	global_lane_counter++;

	// Keep list sorted on lane ID, from + to -
	if (lane_.size() > 0 && lane->GetId() > lane_.back()->GetId()) {
		for (size_t i = 0; i < lane_.size(); i++) {
			if (lane->GetId() > lane_[i]->GetId()) {
				lane_.insert(lane_.begin() + i, lane);
				break;
			}
		}
	} else {
		lane_.push_back(lane);
	}
}

int LaneSection::GetConnectingLaneId(int incoming_lane_id, LinkType link_type) {
	int id = incoming_lane_id;

	if (GetLaneById(id) == 0) {
		LOG("Lane id %d not available in lane section!", id);
		return 0;
	}

	if (GetLaneById(id)->GetLink(link_type)) {
		id = GetLaneById(id)->GetLink(link_type)->GetId();
	} else {
		// if no driving lane found - stay on same index
		id = incoming_lane_id;
	}

	return id;
}

double LaneSection::GetWidthBetweenLanes(int lane_id1, int lane_id2, double s) {
	double lanewidth = (std::fabs(GetCenterOffset(s, lane_id1)) - std::fabs(GetCenterOffset(s, lane_id2)));

	return lanewidth;
}

// Offset from lane1 to lane2 in direction of reference line
double LaneSection::GetOffsetBetweenLanes(int lane_id1, int lane_id2, double s) {
	double laneCenter1 = GetCenterOffset(s, lane_id1) * SIGN(lane_id1);
	double laneCenter2 = GetCenterOffset(s, lane_id2) * SIGN(lane_id2);
	return (laneCenter2 - laneCenter1);
}

int LaneSection::GetClosestLaneIdx(double s, double t, double& offset, bool noZeroWidth, int laneTypeMask) {
	double min_offset = t;	// Initial offset relates to reference line
	int candidate_lane_idx = -1;

	for (int i = 0; i < GetNumberOfLanes(); i++)  // Search through all lanes
	{
		int lane_id = GetLaneIdByIdx(i);
		double laneCenterOffset = SIGN(lane_id) * GetCenterOffset(s, lane_id);

		// Only consider lanes with matching lane type
		if (laneTypeMask & GetLaneById(lane_id)->GetLaneType()
			&& (!noZeroWidth || GetWidth(s, lane_id) > SMALL_NUMBER)) {
			// If position is within a lane, we can return it without further checks
			if (fabs(t - laneCenterOffset) < (GetWidth(s, lane_id) / 2.)) {
				min_offset = t - laneCenterOffset;
				candidate_lane_idx = i;
				break;
			}
			if (candidate_lane_idx == -1 || fabs(t - laneCenterOffset) < fabs(min_offset)) {
				min_offset = t - laneCenterOffset;
				candidate_lane_idx = i;
			}
		}
	}

	offset = min_offset;

	if (candidate_lane_idx == -1) {
		// Fall back to reference lane
		candidate_lane_idx = GetLaneIdxById(0);
	}

	return candidate_lane_idx;
}
