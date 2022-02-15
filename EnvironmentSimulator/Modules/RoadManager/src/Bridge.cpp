#include "Bridge.hpp"

void Bridge::Save(pugi::xml_node& objects) {
	auto bridge = objects.append_child("bridge");
	bridge.append_attribute("s").set_value(s_);
	bridge.append_attribute("length").set_value(length_);
	if (!name_.empty())
		bridge.append_attribute("name").set_value(name_.c_str());
	bridge.append_attribute("id").set_value(id_);
	switch (type_) {
	case CONCRETE:
		bridge.append_attribute("type").set_value("concrete");
		break;
	case STEEL:
		bridge.append_attribute("type").set_value("steel");
		break;
	case BRICK:
		bridge.append_attribute("type").set_value("brick");
		break;
	case WOOD:
		bridge.append_attribute("type").set_value("wood");
		break;
	default:
		assert(false && "Default reached in bridge switch");
		break;
	}

	for (auto validity : validity_) {
		validity.Save(bridge);
	}

	for (auto userData : user_data_) {
		userData->Save(bridge);
	}
}
