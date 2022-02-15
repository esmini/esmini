#include "LaneRoadMark.hpp"

RoadMarkColor LaneRoadMark::ParseColor(pugi::xml_node node) {
	RoadMarkColor color = RoadMarkColor::UNDEFINED;

	if (node.attribute("color") != 0 && strcmp(node.attribute("color").value(), "")) {
		if (!strcmp(node.attribute("color").value(), "standard")) {
			color = RoadMarkColor::STANDARD_COLOR;
		} else if (!strcmp(node.attribute("color").value(), "blue")) {
			color = RoadMarkColor::BLUE;
		} else if (!strcmp(node.attribute("color").value(), "green")) {
			color = RoadMarkColor::GREEN;
		} else if (!strcmp(node.attribute("color").value(), "red")) {
			color = RoadMarkColor::RED;
		} else if (!strcmp(node.attribute("color").value(), "white")) {
			color = RoadMarkColor::WHITE;
		} else if (!strcmp(node.attribute("color").value(), "yellow")) {
			color = RoadMarkColor::YELLOW;
		}
	}

	return color;
}

void LaneRoadMark::Save(pugi::xml_node& lane) {
	auto roadmark = lane.append_child("roadMark");
	roadmark.append_attribute("sOffset").set_value(s_offset_);
	switch (type_) {
	case LaneRoadMark::RoadMarkType::NONE_TYPE:
		roadmark.append_attribute("type").set_value("none");
		break;
	case LaneRoadMark::RoadMarkType::SOLID:
		roadmark.append_attribute("type").set_value("solid");
		break;
	case LaneRoadMark::RoadMarkType::BROKEN:
		roadmark.append_attribute("type").set_value("broken");
		break;
	case LaneRoadMark::RoadMarkType::SOLID_SOLID:
		roadmark.append_attribute("type").set_value("solid solid");
		break;
	case LaneRoadMark::RoadMarkType::SOLID_BROKEN:
		roadmark.append_attribute("type").set_value("solid broken");
		break;
	case LaneRoadMark::RoadMarkType::BROKEN_SOLID:
		roadmark.append_attribute("type").set_value("broken solid");
		break;
	case LaneRoadMark::RoadMarkType::BROKEN_BROKEN:
		roadmark.append_attribute("type").set_value("broken broken");
		break;
	case LaneRoadMark::RoadMarkType::BOTTS_DOTS:
		roadmark.append_attribute("type").set_value("botts dots");
		break;
	case LaneRoadMark::RoadMarkType::GRASS:
		roadmark.append_attribute("type").set_value("grass");
		break;
	case LaneRoadMark::RoadMarkType::CURB:
		roadmark.append_attribute("type").set_value("curb");
		break;
	default:
		assert(false && "Default in roadmark type switch reached");
		break;
	}

	switch (weight_) {
	case LaneRoadMark::RoadMarkWeight::BOLD:
		roadmark.append_attribute("weight").set_value("bold");
		break;
	case LaneRoadMark::RoadMarkWeight::STANDARD:
		roadmark.append_attribute("weight").set_value("standard");
		break;
	default:  // weight is optional
		break;
	}

	switch (color_) {
	case RoadMarkColor::STANDARD_COLOR:
		roadmark.append_attribute("color").set_value("standard");
		break;
	case RoadMarkColor::BLUE:
		roadmark.append_attribute("color").set_value("blue");
		break;
	case RoadMarkColor::GREEN:
		roadmark.append_attribute("color").set_value("green");
		break;
	case RoadMarkColor::RED:
		roadmark.append_attribute("color").set_value("red");
		break;
	case RoadMarkColor::WHITE:
		roadmark.append_attribute("color").set_value("white");
		break;
	case RoadMarkColor::YELLOW:
		roadmark.append_attribute("color").set_value("yellow");
		break;
	default:
		roadmark.append_attribute("color").set_value("standard");
		break;
	}

	switch (material_) {
	case LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL:
		roadmark.append_attribute("material").set_value("standard");
		break;
	default:  // material is optional
		break;
	}

	roadmark.append_attribute("width").set_value(width_);

	switch (lane_change_) {
	case LaneRoadMark::RoadMarkLaneChange::BOTH:
		roadmark.append_attribute("laneChange").set_value("both");
		break;
	case LaneRoadMark::RoadMarkLaneChange::INCREASE:
		roadmark.append_attribute("laneChange").set_value("increase");
		break;
	case LaneRoadMark::RoadMarkLaneChange::DECREASE:
		roadmark.append_attribute("laneChange").set_value("decrease");
		break;
	case LaneRoadMark::RoadMarkLaneChange::NONE_LANECHANGE:
		roadmark.append_attribute("laneChange").set_value("none");
		break;
	default:  // lanechange is optional
		break;
	}

	roadmark.append_attribute("height").set_value(height_);

	for (auto roadMarkType : lane_roadMarkType_) {
		if (roadMarkType->GetName().compare("stand-in"))  // only used internally, don't export
			roadMarkType->Save(roadmark);
	}

	for (auto userData : user_data_) {
		userData->Save(roadmark);
	}
}
