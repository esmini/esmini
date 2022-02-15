#include "LaneLink.hpp"

void LaneLink::Print() {
	LOG("LaneLink type: %d id: %d\n", type_, id_);
}

void LaneLink::Save(pugi::xml_node& lane) {
	auto link = lane.child("link");
	if (link.empty()) {
		link = lane.append_child("link");
	}

	if (GetType() == LinkType::PREDECESSOR) {
		auto predecessor = link.append_child("predecessor");
		predecessor.append_attribute("id").set_value(GetId());
	} else if (GetType() == LinkType::SUCCESSOR) {
		auto successor = link.append_child("successor");
		successor.append_attribute("id").set_value(GetId());
	}
}
