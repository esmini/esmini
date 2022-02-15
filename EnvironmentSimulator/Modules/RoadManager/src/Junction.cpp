#include "Junction.hpp"

Junction::~Junction() {
	for (size_t i = 0; i < connection_.size(); i++) {
		delete connection_[i];
	}
}

int Junction::GetNumberOfRoadConnections(int roadId, int laneId) {
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++) {
		Connection* connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad() && roadId == connection->GetIncomingRoad()->GetId()) {
			for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++) {
				JunctionLaneLink* lane_link = connection->GetLaneLink(j);
				if (laneId == lane_link->from_) {
					counter++;
				}
			}
		}
	}
	return counter;
}

LaneRoadLaneConnection Junction::GetRoadConnectionByIdx(int roadId, int laneId, int idx, int laneTypeMask) {
	int counter = 0;
	LaneRoadLaneConnection lane_road_lane_connection;

	for (int i = 0; i < GetNumberOfConnections(); i++) {
		Connection* connection = GetConnectionByIdx(i);

		if (connection && connection->GetIncomingRoad() && roadId == connection->GetIncomingRoad()->GetId()) {
			for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++) {
				JunctionLaneLink* lane_link = connection->GetLaneLink(j);
				if (laneId == lane_link->from_) {
					if (counter == idx) {
						lane_road_lane_connection.SetLane(laneId);
						lane_road_lane_connection.contact_point_ = connection->GetContactPoint();
						lane_road_lane_connection.SetConnectingRoad(connection->GetConnectingRoad()->GetId());
						lane_road_lane_connection.SetConnectingLane(lane_link->to_);
						// find out driving direction
						int laneSectionId;
						if (lane_link->to_ < 0) {
							laneSectionId = 0;
						} else {
							laneSectionId = connection->GetConnectingRoad()->GetNumberOfLaneSections() - 1;
						}
						if (!(connection->GetConnectingRoad()
								  ->GetLaneSectionByIdx(laneSectionId)
								  ->GetLaneById(lane_link->to_)
								  ->GetLaneType()
							  & laneTypeMask)) {
							LOG("OpenDrive::GetJunctionConnection target lane not driving! from %d, %d to "
								"%d, %d\n",
								roadId, laneId, connection->GetConnectingRoad()->GetId(), lane_link->to_);
						}

						return lane_road_lane_connection;
					}
					counter++;
				}
			}
		}
	}

	return lane_road_lane_connection;
}
void Junction::SetGlobalId() {
	global_id_ = GetNewGlobalLaneId();
}

bool Junction::IsOsiIntersection() {
	if (connection_[0]->GetIncomingRoad()->GetRoadType(0) != 0) {
		if (connection_[0]->GetIncomingRoad()->GetRoadType(0)->road_type_
			== Road::RoadType::ROADTYPE_MOTORWAY) {
			return false;
		} else {
			return true;
		}
	} else {
		LOG_ONCE(
			"Type of roads are missing, cannot determine for OSI intersection or not, assuming that it is an "
			"intersection.");
		return true;
	}
}

int Junction::GetNoConnectionsFromRoadId(int incomingRoadId) {
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++) {
		Connection* connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad()->GetId() == incomingRoadId) {
			counter++;
		}
	}

	return counter;
}

int Junction::GetConnectingRoadIdFromIncomingRoadId(int incomingRoadId, int index) {
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++) {
		Connection* connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad()->GetId() == incomingRoadId) {
			if (counter == index) {
				return GetConnectionByIdx(i)->GetConnectingRoad()->GetId();
			} else {
				counter++;
			}
		}
	}
	return -1;
}

void Junction::Print() {
	LOG("Junction %d %s: \n", id_, name_.c_str());

	for (size_t i = 0; i < connection_.size(); i++) {
		connection_[i]->Print();
	}
}

void Junction::Save(pugi::xml_node& root) {
	auto junction = root.append_child("junction");
	junction.append_attribute("name").set_value(name_.c_str());
	junction.append_attribute("id").set_value(id_);
	switch (type_) {
	case Junction::JunctionType::DEFAULT:
		junction.append_attribute("type").set_value("default");
		break;
	case Junction::JunctionType::VIRTUAL:
		junction.append_attribute("type").set_value("virtual");
		break;
	default:
		break;
	}

	for (auto connection : connection_) {
		connection->Save(junction);
		std::cout << "saved connection for junction " << id_ << std::endl;
	}

	for (auto controller : controller_) {
		controller.Save(junction);
	}

	for (auto userData : user_data_) {
		userData->Save(junction);
	}
}

JunctionController* Junction::GetJunctionControllerByIdx(int index) {
	if (index >= 0 && index < controller_.size()) {
		return &controller_[index];
	}

	return 0;
}

Road* Junction::GetRoadAtOtherEndOfConnectingRoad(Road* connecting_road, Road* incoming_road) {
	if (connecting_road->GetJunction() == 0) {
		LOG("Unexpected: Road %d not a connecting road", connecting_road->GetId());
		return 0;
	}

	// Check both ends
	LinkType link_type[2] = {LinkType::PREDECESSOR, LinkType::SUCCESSOR};
	for (int i = 0; i < 2; i++) {
		RoadLink* link = connecting_road->GetLink(link_type[i]);
		if (link && link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
			if (link->GetElementId() == incoming_road->GetId()) {
				// Get road at other end
				RoadLink* link2 = connecting_road->GetLink(link_type[(i + 1) % 2]);
				if (link2 && link2->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
					return Position::GetOpenDrive()->GetRoadById(link2->GetElementId());
				}
			}
		}
	}

	LOG("Failed to find road at other end of the connecting road %d from road %d", connecting_road->GetId(),
		incoming_road->GetId());
	return nullptr;
}

void JunctionController::Save(pugi::xml_node& junction) {
	auto controller = junction.append_child("controller");
	controller.append_attribute("id").set_value(id_);
	controller.append_attribute("type").set_value(type_.c_str());
	controller.append_attribute("sequence").set_value(sequence_);
	for (auto userData : user_data_) {
		userData->Save(controller);
	}
}

