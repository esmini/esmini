#include "Repeat.hpp"

void Repeat::Save(pugi::xml_node& object) {
	auto repeat = object.append_child("repeat");
	repeat.append_attribute("s").set_value(s_);
	repeat.append_attribute("length").set_value(length_);
	repeat.append_attribute("distance").set_value(distance_);
	repeat.append_attribute("tStart").set_value(tStart_);
	repeat.append_attribute("tEnd").set_value(tEnd_);
	repeat.append_attribute("widthStart").set_value(widthStart_);
	repeat.append_attribute("widthEnd").set_value(widthEnd_);
	repeat.append_attribute("heightStart").set_value(heightStart_);
	repeat.append_attribute("heightEnd").set_value(heightEnd_);
	repeat.append_attribute("zOffsetStart").set_value(zOffsetStart_);
	repeat.append_attribute("zOffsetEnd").set_value(zOffsetEnd_);
	if (lengthStart_)
		repeat.append_attribute("lengthStart").set_value(lengthStart_);
	if (lengthEnd_)
		repeat.append_attribute("lengthEnd").set_value(lengthEnd_);
	if (radiusStart_)
		repeat.append_attribute("radiusStart").set_value(radiusStart_);
	if (radiusEnd_)
		repeat.append_attribute("radiusEnd").set_value(radiusEnd_);
}

// OpenDRIVE 1.5 standard uses only 1 repeat tag.
Repeat* RMObject::GetRepeat() {
	if (repeat_.empty())
		return nullptr;
	else
		return repeat_[0];
}
