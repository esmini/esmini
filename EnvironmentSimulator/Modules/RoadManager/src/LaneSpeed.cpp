#include "LaneSpeed.hpp"

void LaneSpeed::Save(pugi::xml_node& lane) {
	auto laneSpeed = lane.append_child("speed");
	laneSpeed.append_attribute("sOffset").set_value(s_offset_);
	laneSpeed.append_attribute("max").set_value(max_);
	laneSpeed.append_attribute("unit").set_value(unit_.c_str());
	for (auto userData : user_data_) {
		userData->Save(laneSpeed);
	}
}
