#include "LaneOffset.hpp"

void LaneOffset::Print() {
	LOG("LaneOffset s %.2f a %.4f b %.2f c %.2f d %.2f length %.2f\n", s_, polynomial_.GetA(),
		polynomial_.GetB(), polynomial_.GetC(), polynomial_.GetD(), length_);
}

void LaneOffset::Save(pugi::xml_node& lanes) {
	auto offset = lanes.append_child("laneOffset");
	offset.append_attribute("s").set_value(s_);
	offset.append_attribute("a").set_value(polynomial_.GetA());
	offset.append_attribute("b").set_value(polynomial_.GetB());
	offset.append_attribute("c").set_value(polynomial_.GetC());
	offset.append_attribute("d").set_value(polynomial_.GetD());

	for (auto userData : user_data_) {
		userData->Save(offset);
	}
}

double LaneOffset::GetLaneOffset(double s) {
	return (polynomial_.Evaluate(s - s_));
}

double LaneOffset::GetLaneOffsetPrim(double s) {
	return (polynomial_.EvaluatePrim(s - s_));
}
