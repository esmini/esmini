#include "GeoReference.hpp"

void GeoReference::Save(pugi::xml_node& header) const {
	auto georeference = header.append_child("geoReference");
	// TODO: Fix proper CDATA formating for this
	for (auto userData : user_data_) {
		userData->Save(georeference);
	}
}
