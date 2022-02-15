#include "RMObject.hpp"

void RMObject::SetRepeat(Repeat* repeat) {
	if (repeat_.empty())
		repeat_.push_back(repeat);
	else
		repeat_.at(0) = repeat;
}

void RMObject::Save(pugi::xml_node& objects) {
	auto object = objects.append_child("object");
	if (!type_.empty())	 // type is optional
		object.append_attribute("type").set_value(type_.c_str());
	// object.append_attribute("subtype").set_value() TODO:
	// object.append_attribute("dynamic").set_value() TODO:
	if (!name_.empty())	 // name is optional
		object.append_attribute("name").set_value(name_.c_str());
	object.append_attribute("id").set_value(id_);
	object.append_attribute("s").set_value(s_);
	object.append_attribute("t").set_value(t_);
	object.append_attribute("zOffset").set_value(z_offset_);
	// object.append_attribute("validLength").set_value(); TODO:
	switch (orientation_) {
	case RoadObject::Orientation::NEGATIVE:
		object.append_attribute("orientation").set_value("-");
		break;
	case RoadObject::Orientation::POSITIVE:
		object.append_attribute("orientation").set_value("+");
		break;
	case RoadObject::Orientation::NONE:
		object.append_attribute("orientation").set_value("none");
		break;
	default:
		assert(false && "Default reached in road object orientation switch");
		break;
	}
	object.append_attribute("hdg").set_value(heading_);
	object.append_attribute("pitch").set_value(pitch_);
	object.append_attribute("roll").set_value(roll_);
	if (length_)
		object.append_attribute("length").set_value(length_);
	if (width_)
		object.append_attribute("width").set_value(width_);
	if (height_)
		object.append_attribute("height").set_value(height_);

	for (auto repeat : repeat_) {
		repeat->Save(object);
	}

	for (auto outline : outlines_) {
		outline->Save(object);
	}

	for (auto validity : validity_) {
		validity.Save(object);
	}

	for (auto userData : user_data_) {
		userData->Save(object);
	}
}
