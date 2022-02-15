#include "Connection.hpp"

Connection::Connection(Road* incoming_road, Road* connecting_road, ContactPointType contact_point) {
	// Find corresponding road objects
	incoming_road_ = incoming_road;
	connecting_road_ = connecting_road;
	contact_point_ = contact_point;
}

Connection::Connection(int id, Road* incoming_road, Road* connecting_road, ContactPointType contact_point) {
	// Find corresponding road objects
	incoming_road_ = incoming_road;
	connecting_road_ = connecting_road;
	contact_point_ = contact_point;
	id_ = id;
}

Connection::~Connection() {
	for (size_t i = 0; i < lane_link_.size(); i++) {
		delete lane_link_[i];
	}
}

void Connection::AddJunctionLaneLink(int from, int to) {
	lane_link_.push_back(new JunctionLaneLink(from, to));
}

int Connection::GetConnectingLaneId(int incoming_lane_id) {
	for (size_t i = 0; i < lane_link_.size(); i++) {
		if (lane_link_[i]->from_ == incoming_lane_id) {
			return lane_link_[i]->to_;
		}
	}
	return 0;
}

void Connection::Print() {
	LOG("Connection: incoming %d connecting %d\n", incoming_road_->GetId(), connecting_road_->GetId());
	for (size_t i = 0; i < lane_link_.size(); i++) {
		lane_link_[i]->Print();
	}
}

void Connection::Save(pugi::xml_node& junction) {
	auto connection = junction.append_child("connection");
	connection.append_attribute("id").set_value(id_);
	connection.append_attribute("incomingRoad").set_value(incoming_road_->GetId());
	connection.append_attribute("connectingRoad").set_value(connecting_road_->GetId());
	switch (contact_point_) {
	case ContactPointType::CONTACT_POINT_END:
		connection.append_attribute("contactPoint").set_value("end");
		break;
	case ContactPointType::CONTACT_POINT_START:
		connection.append_attribute("contactPoint").set_value("start");
		break;

	default:
		break;
	}

	for (auto laneLink : lane_link_) {
		laneLink->Save(connection);
	}

	for (auto userData : user_data_) {
		userData->Save(connection);
	}
}

void JunctionLaneLink::Save(pugi::xml_node& connection) {
	auto lanelink = connection.append_child("laneLink");
	lanelink.append_attribute("from").set_value(from_);
	lanelink.append_attribute("to").set_value(to_);
}
