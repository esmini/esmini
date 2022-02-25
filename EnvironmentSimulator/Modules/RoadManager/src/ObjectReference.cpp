#include "ObjectReference.hpp"

void ObjectReference::Save(pugi::xml_node& objects) {
	auto objectRef = objects.append_child("objectReference");
	objectRef.append_attribute("s").set_value(s_);
	objectRef.append_attribute("t").set_value(t_);
	objectRef.append_attribute("id").set_value(id_);
	objectRef.append_attribute("zOffset").set_value(z_offset_);
	objectRef.append_attribute("validLength").set_value(valid_length_);
	switch (orientation_) {
	case RoadObject::Orientation::NEGATIVE:
		objectRef.append_attribute("orientation").set_value("-");
		break;
	case RoadObject::Orientation::POSITIVE:
		objectRef.append_attribute("orientation").set_value("+");
		break;
	case RoadObject::Orientation::NONE:
		objectRef.append_attribute("orientation").set_value("none");
		break;
	default:
		assert(false && "Default reached in road object reference orientation switch");
		break;
	}

	for (auto validity : validity_) {
		validity.Save(objectRef);
	}

	for (auto userData : user_data_) {
		userData->Save(objectRef);
	}
}
