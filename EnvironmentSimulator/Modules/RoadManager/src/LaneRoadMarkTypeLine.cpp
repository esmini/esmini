#include "LaneRoadMarkTypeLine.hpp"

// void LaneRoadMarkTypeLine::SetGlobalId() {
	// global_id_ = GetNewGlobalLaneBoundaryId();
// }

void LaneRoadMarkTypeLine::Save(pugi::xml_node& type) {
	auto line = type.append_child("line");
	line.append_attribute("length").set_value(length_);
	line.append_attribute("space").set_value(space_);
	line.append_attribute("tOffset").set_value(t_offset_);
	line.append_attribute("sOffset").set_value(s_offset_);

	switch (rule_) {
	case RoadMarkTypeLineRule::CAUTION:
		line.append_attribute("rule").set_value("caution");
		break;
	case RoadMarkTypeLineRule::NO_PASSING:
		line.append_attribute("rule").set_value("no passing");
		break;
	case RoadMarkTypeLineRule::NONE:
		line.append_attribute("rule").set_value("none");
		break;
	default:  // rule is optional
		break;
	}

	line.append_attribute("width").set_value(width_);

	switch (color_) {
	case RoadMarkColor::STANDARD_COLOR:
		line.append_attribute("color").set_value("standard");
		break;
	case RoadMarkColor::BLUE:
		line.append_attribute("color").set_value("blue");
		break;
	case RoadMarkColor::GREEN:
		line.append_attribute("color").set_value("green");
		break;
	case RoadMarkColor::RED:
		line.append_attribute("color").set_value("red");
		break;
	case RoadMarkColor::WHITE:
		line.append_attribute("color").set_value("white");
		break;
	case RoadMarkColor::YELLOW:
		line.append_attribute("color").set_value("yellow");
		break;
	default:  // color is optional
		break;
	}
}


