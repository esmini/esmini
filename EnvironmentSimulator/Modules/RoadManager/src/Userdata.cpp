#include "Userdata.hpp"

void UserData::Save(pugi::xml_node& parent) {
	for (auto userData : origin_node_) {
		userData.attribute("code").set_value(code_.c_str());
		userData.attribute("value").set_value(value_.c_str());
		parent.append_copy(userData);
	}
}
