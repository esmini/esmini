#pragma once

#include "RoadObject.hpp"

void ValidityRecord::Save(pugi::xml_node& object) {
	auto validity = object.append_child("validity");
	validity.append_attribute("fromLane").set_value(fromLane_);
	validity.append_attribute("toLane").set_value(toLane_);

	for (auto userData : user_data_) {
		userData->Save(validity);
	}
}