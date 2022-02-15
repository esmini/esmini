#include "LaneRoadMarkType.hpp"

void LaneRoadMarkType::AddLine(LaneRoadMarkTypeLine* lane_roadMarkTypeLine) {
	lane_roadMarkTypeLine->SetGlobalId();
	lane_roadMarkTypeLine_.push_back(lane_roadMarkTypeLine);
}

void LaneRoadMarkType::Save(pugi::xml_node& roadMark) {
	auto type = roadMark.child("type");
	if (type.empty()) {
		type = roadMark.append_child("type");
	}
	type.append_attribute("name").set_value(name_.c_str());
	type.append_attribute("width").set_value(width_);

	assert(!lane_roadMarkTypeLine_.empty());
	for (auto line : lane_roadMarkTypeLine_) {
		line->Save(type);
	}

	for (auto userData : user_data_) {
		userData->Save(type);
	}
}

LaneRoadMarkType* LaneRoadMark::GetLaneRoadMarkTypeByIdx(int idx) {
	if (idx < (int)lane_roadMarkType_.size()) {
		return lane_roadMarkType_[idx];
	}

	return 0;
}
