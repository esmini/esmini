#include "Elevation.hpp"

void Elevation::Print() {
	LOG("Elevation: s: %.2f A: %.4f B: %.4f C: %.4f D: %.4f\n", GetS(), poly3_.GetA(), poly3_.GetB(),
		poly3_.GetC(), poly3_.GetD());
}

void Elevation::Save(pugi::xml_node& elevationProfile, const std::string name) {
	auto elevation = elevationProfile.append_child(name.c_str());
	elevation.append_attribute("s").set_value(s_);
	elevation.append_attribute("a").set_value(poly3_.GetA());
	elevation.append_attribute("b").set_value(poly3_.GetB());
	elevation.append_attribute("c").set_value(poly3_.GetC());
	elevation.append_attribute("d").set_value(poly3_.GetD());

	for (auto userData : user_data_) {
		userData->Save(elevationProfile);
	}
}
