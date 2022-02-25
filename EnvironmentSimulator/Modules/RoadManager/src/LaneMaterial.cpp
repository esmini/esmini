#include "LaneMaterial.hpp"

void LaneMaterial::Save(pugi::xml_node& lane) {
	auto laneMaterial = lane.append_child("material");
	laneMaterial.append_attribute("sOffset").set_value(s_offset_);
	laneMaterial.append_attribute("surface").set_value(surface_.c_str());
	laneMaterial.append_attribute("friction").set_value(friction_);
	laneMaterial.append_attribute("roughness").set_value(roughness_);
	for (auto userData : user_data_) {
		userData->Save(laneMaterial);
	}
}
