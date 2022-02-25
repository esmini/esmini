#include "RoadLink.hpp"

RoadLink::RoadLink(LinkType type, pugi::xml_node node)
	: contact_point_type_(ContactPointType::CONTACT_POINT_UNDEFINED) {
	std::string element_type = node.attribute("elementType").value();
	std::string contact_point_type = "";
	type_ = type;
	element_id_ = atoi(node.attribute("elementId").value());

	if (node.attribute("contactPoint") != NULL) {
		contact_point_type = node.attribute("contactPoint").value();
	}

	if (element_type == "road") {
		element_type_ = ELEMENT_TYPE_ROAD;
		if (contact_point_type == "start") {
			contact_point_type_ = CONTACT_POINT_START;
		} else if (contact_point_type == "end") {
			contact_point_type_ = CONTACT_POINT_END;
		} else if (contact_point_type.empty()) {
			LOG("Missing contact point type\n");
		} else {
			LOG("Unsupported contact point type: %s\n", contact_point_type.c_str());
			contact_point_type_ = CONTACT_POINT_UNDEFINED;
		}
	} else if (element_type == "junction") {
		element_type_ = ELEMENT_TYPE_JUNCTION;
		// contact_point_type_ = CONTACT_POINT_JUNCTION;
		if (contact_point_type == "start") {
			contact_point_type_ = CONTACT_POINT_START;
		} else if (contact_point_type == "end") {
			contact_point_type_ = CONTACT_POINT_END;
		}
	} else if (element_type.empty()) {
		LOG("Missing element type\n");
	} else {
		LOG("Unsupported element type: %s\n", element_type.c_str());
		element_type_ = ELEMENT_TYPE_UNKNOWN;
	}
}

bool RoadLink::operator==(RoadLink& rhs) {
	return (rhs.type_ == type_ && rhs.element_type_ == element_id_ && rhs.element_id_ == element_id_
			&& rhs.contact_point_type_ == contact_point_type_);
}

void RoadLink::Print() {
	std::cout << "RoadLink type: " << type_ << " id: " << element_id_ << " element type: " << element_type_
		 << " contact point type: " << contact_point_type_ << std::endl;
}

void RoadLink::Save(pugi::xml_node& link) {
	auto linkType = GetType();
	pugi::xml_node type;

	switch (GetType()) {
	case LinkType::PREDECESSOR:
		type = link.append_child("predecessor");
		break;
	case LinkType::SUCCESSOR:
		type = link.append_child("successor");
		break;
	default:
		assert(false && "The default case of LinkType switch was reached.");
		return;
	}

	switch (GetElementType()) {
	case RoadLink::ElementType::ELEMENT_TYPE_JUNCTION:
		type.append_attribute("elementType").set_value("junction");
		break;
	case RoadLink::ElementType::ELEMENT_TYPE_ROAD:
		type.append_attribute("elementType").set_value("road");
		break;
	default:
		assert(false && "The default case of elementType switch was reached.");
		break;
	}

	switch (GetContactPointType()) {
	case ContactPointType::CONTACT_POINT_START:
		type.append_attribute("contactPoint").set_value("start");
		break;
	case ContactPointType::CONTACT_POINT_END:
		type.append_attribute("contactPoint").set_value("end");
		break;
	case ContactPointType::CONTACT_POINT_JUNCTION:
		std::cerr << "Unsupported contact point: Junction" << std::endl;
		break;
	case ContactPointType::CONTACT_POINT_UNDEFINED:
		// Suppress output atm. since a lot of tags miss this entry..
		// std::cerr << "Unsupported contact point: Undefined" << std::endl;
		break;
	default:
		assert(false && "The default case of road link contactPoint switch was reached.");
		break;
	}

	assert(GetElementId() >= 0 && "Element ID of road link cannot be negative");
	type.append_attribute("elementId").set_value(GetElementId());

	for (auto userData : user_data_) {
		userData->Save(type);
	}
}
