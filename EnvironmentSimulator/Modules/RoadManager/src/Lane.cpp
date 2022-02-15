#include "Lane.hpp"

void Lane::SetGlobalId() {
	global_id_ = GetNewGlobalLaneId();
}

LaneWidth* Lane::GetWidthByIndex(int index) {
	if (lane_width_.size() <= index || lane_width_.size() == 0) {
		throw std::runtime_error("Lane::GetWidthByIndex(int index) -> exceeds index");
	} else if (lane_width_.size() < 0) {
		throw std::runtime_error("Lane::GetWidthByIndex(int index) -> index must be larger than 0");
	} else {
		return lane_width_[index];
	}
}

LaneWidth* Lane::GetWidthByS(double s) {
	if (lane_width_.size() == 0) {
		return 0;  // No lanewidth defined
	}
	for (int i = 0; i + 1 < (int)lane_width_.size(); i++) {
		if (s < lane_width_[i + 1]->GetSOffset()) {
			return lane_width_[i];
		}
	}
	return lane_width_.back();
}

LaneLink* Lane::GetLink(LinkType type) {
	for (int i = 0; i < (int)link_.size(); i++) {
		LaneLink* l = link_[i];
		if (l->GetType() == type) {
			return l;
		}
	}
	return 0;  // No link of requested type exists
}

LaneRoadMark* Lane::GetLaneRoadMarkByIdx(int idx) {
	if (lane_roadMark_.size() <= idx || lane_roadMark_.size() == 0) {
		throw std::runtime_error("Lane::GetLaneRoadMarkByIdx(int idx) -> exceeds index");
	} else if (lane_roadMark_.size() < 0) {
		throw std::runtime_error("Lane::GetLaneRoadMarkByIdx(int idx) -> index must be larger than 0");
	} else {
		return lane_roadMark_[idx];
	}
}

std::vector<int> Lane::GetLineGlobalIds() {
	std::vector<int> line_ids;
	for (int i = 0; i < GetNumberOfRoadMarks(); i++) {
		LaneRoadMark* laneroadmark = GetLaneRoadMarkByIdx(i);
		for (int j = 0; j < laneroadmark->GetNumberOfRoadMarkTypes(); j++) {
			LaneRoadMarkType* laneroadmarktype = laneroadmark->GetLaneRoadMarkTypeByIdx(j);

			for (int h = 0; h < laneroadmarktype->GetNumberOfRoadMarkTypeLines(); h++) {
				LaneRoadMarkTypeLine* laneroadmarktypeline
					= laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(h);
				line_ids.push_back(laneroadmarktypeline->GetGlobalId());
			}
		}
	}

	return line_ids;
}

int Lane::GetLaneBoundaryGlobalId() {
	if (lane_boundary_) {
		return lane_boundary_->GetGlobalId();
	} else {
		return -1;
	}
}

void Lane::SetLaneBoundary(LaneBoundaryOSI* lane_boundary) {
	lane_boundary->SetGlobalId();
	lane_boundary_ = lane_boundary;
}

void Lane::Print() {
	LOG("Lane: %d, type: %d, level: %d\n", id_, type_, level_);

	for (size_t i = 0; i < link_.size(); i++) {
		link_[i]->Print();
	}

	for (size_t i = 0; i < lane_width_.size(); i++) {
		lane_width_[i]->Print();
	}
}

void Lane::Save(pugi::xml_node& laneSection) {
	pugi::xml_node lcr;
	if (GetId() > 0)  // Left lane
	{
		lcr = laneSection.child("left");
		if (lcr.empty()) {
			lcr = laneSection.append_child("left");
		}
	} else if (GetId() == 0)  // Center lane
	{
		lcr = laneSection.child("center");
		if (lcr.empty()) {
			lcr = laneSection.append_child("center");
		}
	} else if (GetId() < 0)	 // Right lane
	{
		lcr = laneSection.child("right");
		if (lcr.empty()) {
			lcr = laneSection.append_child("right");
		}
	}

	auto lane = lcr.append_child("lane");
	lane.append_attribute("id").set_value(id_);

	if (level_ == 1) {
		lane.append_attribute("level").set_value("true");
	} else if (level_ == 0) {
		lane.append_attribute("level").set_value("false");
	}

	switch (type_) {
	case Lane::LaneType::LANE_TYPE_NONE:
		lane.append_attribute("type").set_value("none");
		break;
	case Lane::LaneType::LANE_TYPE_DRIVING:
		lane.append_attribute("type").set_value("driving");
		break;
	case Lane::LaneType::LANE_TYPE_STOP:
		lane.append_attribute("type").set_value("stop");
		break;
	case Lane::LaneType::LANE_TYPE_SHOULDER:
		lane.append_attribute("type").set_value("shoulder");
		break;
	case Lane::LaneType::LANE_TYPE_BIKING:
		lane.append_attribute("type").set_value("biking");
		break;
	case Lane::LaneType::LANE_TYPE_SIDEWALK:
		lane.append_attribute("type").set_value("sidewalk");
		break;
	case Lane::LaneType::LANE_TYPE_BORDER:
		lane.append_attribute("type").set_value("border");
		break;
	case Lane::LaneType::LANE_TYPE_RESTRICTED:
		lane.append_attribute("type").set_value("restricted");
		break;
	case Lane::LaneType::LANE_TYPE_PARKING:
		lane.append_attribute("type").set_value("parking");
		break;
	case Lane::LaneType::LANE_TYPE_MEDIAN:
		lane.append_attribute("type").set_value("median");
		break;
	case Lane::LaneType::LANE_TYPE_SPECIAL1:
		lane.append_attribute("type").set_value("special1");
		break;
	case Lane::LaneType::LANE_TYPE_SPECIAL2:
		lane.append_attribute("type").set_value("special2");
		break;
	case Lane::LaneType::LANE_TYPE_SPECIAL3:
		lane.append_attribute("type").set_value("special3");
		break;
	case Lane::LaneType::LANE_TYPE_ROADWORKS:
		lane.append_attribute("type").set_value("roadWorks");
		break;
	case Lane::LaneType::LANE_TYPE_TRAM:
		lane.append_attribute("type").set_value("tram");
		break;
	case Lane::LaneType::LANE_TYPE_RAIL:
		lane.append_attribute("type").set_value("rail");
		break;
	case Lane::LaneType::LANE_TYPE_ENTRY:
		lane.append_attribute("type").set_value("entry");
		break;
	case Lane::LaneType::LANE_TYPE_EXIT:
		lane.append_attribute("type").set_value("exit");
		break;
	case Lane::LaneType::LANE_TYPE_OFF_RAMP:
		lane.append_attribute("type").set_value("offRamp");
		break;
	case Lane::LaneType::LANE_TYPE_ON_RAMP:
		lane.append_attribute("type").set_value("onRamp");
		break;
	default:
		assert(false && "Default reached in lane type switch");
		break;
	}

	for (auto link : link_) {
		link->Save(lane);
	}

	for (auto width : lane_width_) {
		width->Save(lane);
	}

	for (auto roadMark : lane_roadMark_) {
		roadMark->Save(lane);
	}

	for (auto material : lane_material_) {
		material->Save(lane);
	}

	for (auto speed : lane_speed_) {
		speed->Save(lane);
	}

	for (auto userData : user_data_) {
		userData->Save(lane);
	}
}

bool Lane::IsCenter() {
	if (GetId() == 0) {
		return true;  // Ref lane no width -> no driving
	} else {
		return false;
	}
}
bool Lane::IsType(Lane::LaneType type) {
	if (GetId() == 0) {
		return false;  // Ref lane no width -> no driving
	}

	return bool(type_ & type);
}

bool Lane::IsDriving() {
	if (GetId() == 0) {
		return false;  // Ref lane no width -> no driving
	}

	return bool(type_ & Lane::LaneType::LANE_TYPE_ANY_DRIVING);
}

// Offset from closest left road mark to current position
RoadMarkInfo Lane::GetRoadMarkInfoByS(int track_id, int lane_id, double s) {
	Position* pos = new roadmanager::Position();
	Road* road = pos->GetRoadById(track_id);
	LaneSection* lsec;
	Lane* lane;
	LaneRoadMark* lane_roadMark;
	LaneRoadMarkType* lane_roadMarkType;
	LaneRoadMarkTypeLine* lane_roadMarkTypeLine;
	RoadMarkInfo rm_info = {-1, -1};
	int lsec_idx, number_of_lsec, number_of_roadmarks, number_of_roadmarktypes, number_of_roadmarklines;
	double s_roadmark, s_roadmarkline, s_end_roadmark, s_end_roadmarkline = 0, lsec_end = 0;
	if (road == 0) {
		LOG("Position::Set Error: track %d not available", track_id);
		lsec_idx = -1;
	} else {
		lsec_idx = road->GetLaneSectionIdxByS(s);
	}

	lsec = road->GetLaneSectionByIdx(lsec_idx);

	if (lsec == 0) {
		LOG("Position::Set Error: lane section %d not available", lsec_idx);
	} else {
		number_of_lsec = road->GetNumberOfLaneSections();
		if (lsec_idx == number_of_lsec - 1) {
			lsec_end = road->GetLength();
		} else {
			lsec_end = road->GetLaneSectionByIdx(lsec_idx + 1)->GetS();
		}
	}

	lane = lsec->GetLaneById(lane_id);
	if (lane == 0) {
		LOG("Position::Set Error: lane section %d not available", lane_id);
	}

	number_of_roadmarks = lane->GetNumberOfRoadMarks();

	if (number_of_roadmarks > 0) {
		for (int m = 0; m < number_of_roadmarks; m++) {
			lane_roadMark = lane->GetLaneRoadMarkByIdx(m);
			s_roadmark = lsec->GetS() + lane_roadMark->GetSOffset();
			if (m == number_of_roadmarks - 1) {
				s_end_roadmark = lsec_end;
			} else {
				s_end_roadmark = lane->GetLaneRoadMarkByIdx(m + 1)->GetSOffset();
			}

			// Check the existence of "type" keyword under roadmark
			number_of_roadmarktypes = lane_roadMark->GetNumberOfRoadMarkTypes();
			if (number_of_roadmarktypes != 0) {
				lane_roadMarkType = lane_roadMark->GetLaneRoadMarkTypeByIdx(0);
				number_of_roadmarklines = lane_roadMarkType->GetNumberOfRoadMarkTypeLines();

				// Looping through each roadmarkline under roadmark
				for (int n = 0; n < number_of_roadmarklines; n++) {
					lane_roadMarkTypeLine = lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(n);
					s_roadmarkline = s_roadmark + lane_roadMarkTypeLine->GetSOffset();
					if (lane_roadMarkTypeLine != 0) {
						if (n == number_of_roadmarklines - 1) {
							s_end_roadmarkline = s_end_roadmark;
						} else {
							s_end_roadmarkline
								= lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(n + 1)->GetSOffset();
						}
					}

					if (s >= s_roadmarkline && s < s_end_roadmarkline) {
						rm_info.roadmark_idx_ = m;
						rm_info.roadmarkline_idx_ = n;
					} else {
						continue;
					}
				}
			} else {
				rm_info.roadmarkline_idx_ = 0;
				if (s >= s_roadmark && s < s_end_roadmark) {
					rm_info.roadmark_idx_ = m;
				} else {
					continue;
				}
			}
		}
	}
	delete pos;
	return rm_info;
}
