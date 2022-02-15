#include "LaneWidth.hpp"

void LaneWidth::Print() {
	LOG("LaneWidth: sOffset: %.2f, a: %.2f, b: %.2f, c: %.2f, d: %.2f\n", s_offset_, poly3_.GetA(),
		poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

void LaneWidth::Save(pugi::xml_node& lane) {
	auto width = lane.append_child("width");
	width.append_attribute("sOffset").set_value(s_offset_);
	width.append_attribute("a").set_value(poly3_.GetA());
	width.append_attribute("b").set_value(poly3_.GetB());
	width.append_attribute("c").set_value(poly3_.GetC());
	width.append_attribute("d").set_value(poly3_.GetD());
}
