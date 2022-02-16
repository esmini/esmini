
#include "Road.hpp"

std::shared_ptr<LaneSection> Road::GetLaneSectionByIdx(int idx) {
	if (idx >= 0 && idx < lane_section_.size()) {
		return lane_section_[idx];
	} else {
		return 0;
	}
}

int Road::GetLaneSectionIdxByS(double s, int start_at) {
	if (start_at < 0 || start_at > lane_section_.size() - 1) {
		return -1;
	}

	std::shared_ptr<LaneSection> lane_section = lane_section_[start_at];
	size_t i = start_at;

	if (s < lane_section->GetS() && start_at > 0) {
		// Look backwards
		for (i = start_at - 1; i > 0; i--)	// No need to check the first one
		{
			lane_section = GetLaneSectionByIdx((int)i);
			if (s > lane_section->GetS()) {
				break;
			}
		}
	} else {
		// look forward
		for (i = start_at; i < GetNumberOfLaneSections() - 1; i++)	// No need to check the last one
		{
			lane_section = GetLaneSectionByIdx((int)i);
			if (s < lane_section->GetS() + lane_section->GetLength()) {
				break;
			}
		}
	}

	return (int)i;
}

LaneInfo Road::GetLaneInfoByS(double s, int start_lane_section_idx, int start_lane_id, int laneTypeMask) {
	LaneInfo lane_info;

	lane_info.lane_section_idx_ = start_lane_section_idx;
	lane_info.lane_id_ = start_lane_id;

	if (lane_info.lane_section_idx_ >= (int)lane_section_.size()) {
		LOG("Error idx %d > n_lane_sections %d\n", lane_info.lane_section_idx_, (int)lane_section_.size());
	} else {
		std::shared_ptr<LaneSection> lane_section = lane_section_[lane_info.lane_section_idx_];

		// check if we passed current section
		if (s > lane_section->GetS() + lane_section->GetLength() || s < lane_section->GetS()) {
			if (s > lane_section->GetS() + lane_section->GetLength()) {
				while (s > lane_section->GetS() + lane_section->GetLength()
					   && lane_info.lane_section_idx_ + 1 < GetNumberOfLaneSections()) {
					// Find out connecting lane, then move to next lane section
					lane_info.lane_id_ = lane_section->GetConnectingLaneId(lane_info.lane_id_, SUCCESSOR);
					lane_section = GetLaneSectionByIdx(++lane_info.lane_section_idx_);
				}
			} else if (s < lane_section->GetS()) {
				while (s < lane_section->GetS() && lane_info.lane_section_idx_ > 0) {
					// Move to previous lane section
					lane_info.lane_id_ = lane_section->GetConnectingLaneId(lane_info.lane_id_, PREDECESSOR);
					lane_section = GetLaneSectionByIdx(--lane_info.lane_section_idx_);
				}
			}

			// If new lane is not of snapping type, try to move into a close valid lane
			std::shared_ptr<Lane> lane = lane_section->GetLaneById(lane_info.lane_id_);
			if (lane == 0 || !(laneTypeMask & lane_section->GetLaneById(lane_info.lane_id_)->GetLaneType())) {
				double offset = 0;
				double t = 0;

				if (lane == 0) {
					LOG("No valid connecting lane (s: %.2f lane_id %d) - looking for a valid lane from "
						"center outwards",
						s, lane_info.lane_id_);
				} else {
					t = lane->GetOffsetFromRef() + GetLaneWidthByS(s, lane->GetId());
				}
				lane_info.lane_id_
					= lane_section
						  ->GetLaneByIdx(lane_section->GetClosestLaneIdx(s, t, offset, true, laneTypeMask))
						  ->GetId();
				if (lane_info.lane_id_ == 0) {
					LOG("Failed to find a closest snapping lane");
				}
			}
		}
	}

	return lane_info;
}

// int Road::GetConnectingLaneId(std::shared_ptr<RoadLink> road_link, int fromLaneId, int connectingRoadId) {
// std::shared_ptr<Lane> lane;
//
// if (road_link->GetElementId() == -1) {
// LOG("No connecting road or junction at rid %d link_type %s", GetId(),
// LinkType2Str(road_link->GetType()).c_str());
// return -1;
// }
//
// if (road_link->GetType() == LinkType::SUCCESSOR) {
// lane = lane_section_.back()->GetLaneById(fromLaneId);
// } else {
// lane = lane_section_[0]->GetLaneById(fromLaneId);
// }
//
// if (lane == nullptr) {
// LOG("Failed to get connecting lane %d %d %d", GetId(), fromLaneId, connectingRoadId);
// return 0;
// }
//
// if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD) {
// if (road_link->GetElementId() != connectingRoadId) {
// LOG("Wrong connectingRoadId %d (expected %d)", road_link->GetElementId(), connectingRoadId);
// return 0;
// }
//
// LaneLink* lane_link = lane->GetLink(road_link->GetType());
// if (lane_link != 0) {
// return lane->GetLink(road_link->GetType())->GetId();
// } else {
// return 0;
// }
// } else if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION) {
// Junction* junction = Position::GetOpenDrive()->GetJunctionById(road_link->GetElementId());
//
// if (junction == 0) {
// LOG("Error: junction %d not existing\n", road_link->GetElementType());
// return -1;
// }
//
// int n_connections = junction->GetNumberOfRoadConnections(GetId(), lane->GetId());
//
// for (int i = 0; i < n_connections; i++) {
// LaneRoadLaneConnection lane_road_lane_connection
// = junction->GetRoadConnectionByIdx(GetId(), lane->GetId(), i);
//
// if (lane_road_lane_connection.GetConnectingRoadId() == connectingRoadId) {
// return lane_road_lane_connection.GetConnectinglaneId();
// }
// }
// }
//
// return 0;
// }

double Road::GetLaneWidthByS(double s, int lane_id) {
	std::shared_ptr<LaneSection> lsec;

	if (GetNumberOfLaneSections() < 1) {
		return 0.0;
	}

	for (size_t i = 0; i < GetNumberOfLaneSections(); i++) {
		lsec = GetLaneSectionByIdx((int)i);
		if (s < lsec->GetS() + lsec->GetLength()) {
			return lsec->GetWidth(s, lane_id);
		} else if (i == GetNumberOfLaneSections() - 1) {
			// Passed end of road - pick width at road endpoint
			return lsec->GetWidth(s, lane_id);
		}
	}

	return 0.0;
}

double Road::GetSpeedByS(double s) {
	if (type_.size() > 0) {
		size_t i;
		for (i = 0; i < type_.size() - 1 && s > type_[i + 1]->s_; i++)
			;

		return type_[i]->speed_;
	}

	// No type entries, fall back to a speed based on nr of lanes
	return 0;
}

std::shared_ptr<Geometry> Road::GetGeometry(int idx) {
	if (idx < 0 || idx + 1 > (int)geometry_.size()) {
		LOG("Road::GetGeometry index %d out of range [0:%d]\n", idx, (int)geometry_.size());
		return 0;
	}
	return geometry_[idx];
}

Road::~Road() {
	geometry_.clear();
	elevation_profile_.clear();
	super_elevation_profile_.clear();
	link_.clear();
}

void Road::Print() {
	LOG("Road id: %d length: %.2f\n", id_, GetLength());
	std::cout << "Geometries:" << std::endl;

	for (size_t i = 0; i < geometry_.size(); i++) {
		std::cout << "Geometry type: " << geometry_[i]->GetType() << std::endl;
	}

	for (size_t i = 0; i < link_.size(); i++) {
		link_[i]->Print();
	}

	for (size_t i = 0; i < lane_section_.size(); i++) {
		lane_section_[i]->Print();
	}

	for (size_t i = 0; i < lane_offset_.size(); i++) {
		lane_offset_[i]->Print();
	}
}

void Road::Save(pugi::xml_node& root) {
	auto road = root.append_child("road");
	if (GetName().compare(""))	// name Attribute is optional
		road.append_attribute("name").set_value(GetName().c_str());
	road.append_attribute("length").set_value(GetLength());
	road.append_attribute("id").set_value(GetId());
	road.append_attribute("junction").set_value(GetJunction());

	switch (GetRule()) {
	case RoadRule::RIGHT_HAND_TRAFFIC:
		road.append_attribute("rule").set_value("RHT");
		break;
	case RoadRule::LEFT_HAND_TRAFFIC:
		road.append_attribute("rule").set_value("LHT");
		break;
	default:  // rule attribute is optional
		break;
	}

	if (!link_.empty()) {
		auto link = road.append_child("link");
		for (auto roadLink : link_) {
			roadLink->Save(link);
		}
	}

	if (!type_.empty()) {
		for (auto t : type_) {
			auto type = road.append_child("type");
			type.append_attribute("s").set_value(t->s_);
			auto typeNode = type.append_attribute("type");
			switch (t->road_type_) {
			case RoadType::ROADTYPE_BICYCLE:
				typeNode.set_value("bicycle");
				break;
			case RoadType::ROADTYPE_LOWSPEED:
				typeNode.set_value("lowSpeed");
				break;
			case RoadType::ROADTYPE_MOTORWAY:
				typeNode.set_value("motorway");
				break;
			case RoadType::ROADTYPE_PEDESTRIAN:
				typeNode.set_value("pedestrian");
				break;
			case RoadType::ROADTYPE_RURAL:
				typeNode.set_value("rural");
				break;
			case RoadType::ROADTYPE_TOWN:
				typeNode.set_value("town");
				break;
			case RoadType::ROADTYPE_UNKNOWN:
				typeNode.set_value("unknown");
				break;
			default:
				assert(false && "default raeched in roadType switch");
				break;
			}

			if (t->speed_)	// Optional speed record
			{
				auto speed = type.append_child("speed");
				speed.append_attribute("max").set_value(t->speed_);
				speed.append_attribute("unit").set_value("m/s");
			}

			for (auto userData : t->user_data_) {
				userData->Save(type);
			}
		}
	}

	auto planView = road.append_child("planView");
	assert(!geometry_.empty());
	for (auto geom : geometry_) {
		auto geometry = planView.append_child("geometry");
		geom->Save(geometry);
	}

	if (!elevation_profile_.empty()) {
		auto elevationProfile = road.append_child("elevationProfile");
		for (auto elevation : elevation_profile_) {
			elevation->Save(elevationProfile, "elevation");
		}
	}

	if (!super_elevation_profile_.empty()) {
		auto lateralProfile = road.append_child("lateralProfile");
		for (auto superelevation : super_elevation_profile_) {
			superelevation->Save(lateralProfile, "superelevation");
		}
	}

	auto lanes = road.append_child("lanes");
	if (!lane_offset_.empty()) {
		for (auto laneOffset : lane_offset_) {
			laneOffset->Save(lanes);
		}
	}

	assert(!lane_section_.empty());
	for (auto laneSection : lane_section_) {
		laneSection->Save(lanes);
	}

	if (!object_.empty() || !bridge_.empty() || !object_reference_.empty()) {
		auto objects = road.append_child("objects");
		for (auto object : object_) {
			object->Save(objects);
		}

		for (auto bridge : bridge_) {
			bridge->Save(objects);
		}

		for (auto objectRef : object_reference_) {
			objectRef->Save(objects);
		}
	}

	if (!signal_.empty()) {
		auto signals = road.append_child("signals");
		for (auto signal : signal_) {
			signal->Save(signals);
		}
	}

	for (auto userData : user_data_) {
		userData->Save(road);
	}
}

void Road::AddElevation(std::shared_ptr<Elevation> elevation) {
	// Adjust last elevation length
	if (elevation_profile_.size() > 0) {
		std::shared_ptr<Elevation> e_previous = elevation_profile_.back();
		e_previous->SetLength(elevation->GetS() - e_previous->GetS());
	}
	elevation->SetLength(length_ - elevation->GetS());

	elevation_profile_.push_back(elevation);
}

void Road::AddSuperElevation(std::shared_ptr<Elevation> super_elevation) {
	// Adjust last super elevation length
	if (super_elevation_profile_.size() > 0) {
		std::shared_ptr<Elevation> e_previous = super_elevation_profile_.back();
		e_previous->SetLength(super_elevation->GetS() - e_previous->GetS());
	}
	super_elevation->SetLength(length_ - super_elevation->GetS());

	super_elevation_profile_.push_back(super_elevation);
}

std::shared_ptr<Elevation> Road::GetElevation(int idx) {
	if (idx < 0 || idx >= elevation_profile_.size()) {
		return 0;
	}

	return elevation_profile_[idx];
}

std::shared_ptr<Elevation> Road::GetSuperElevation(int idx) {
	if (idx < 0 || idx >= super_elevation_profile_.size()) {
		return 0;
	}

	return super_elevation_profile_[idx];
}

void Road::AddSignal(std::shared_ptr<Signal> signal) {
	// Adjust signal length
	if (signal_.size() > 0) {
		std::shared_ptr<Signal> sig_previous = signal_.back();
		sig_previous->SetLength(signal->GetS() - sig_previous->GetS());
	}
	signal->SetLength(length_ - signal->GetS());

	// LOG("Add signal[%d]: \"%s\" type %d subtype %d to road %d", (int)signal_.size(),
	// signal->GetName().c_str(), 	signal->GetType(), signal->GetSubType(), GetId());
	signal_.push_back(signal);
}

int Road::GetNumberOfSignals() {
	return (int)signal_.size();
}

std::shared_ptr<Signal> Road::GetSignal(int idx) {
	if (idx < 0 || idx >= signal_.size()) {
		return 0;
	}

	return signal_[idx];
}

void Road::AddObject(std::shared_ptr<RMObject> object) {
	/*LOG("Add object[%d]: %s", (int)object_.size(), object->GetName().c_str());*/
	object_.push_back(object);
}

void Road::AddBridge(std::shared_ptr<Bridge> bridge) {
	/*LOG("Add bridge[%d]: %s", (int)bridge_.size(), bridge->GetName().c_str());*/
	bridge_.push_back(bridge);
}

void Road::AddObjectReference(std::shared_ptr<ObjectReference> object_reference) {
	/*LOG("Add object reference[%d]: %s", (int)object_reference_.size(),
	 * object_reference->GetName().c_str());*/
	object_reference_.push_back(object_reference);
}

std::shared_ptr<RMObject> Road::GetObject(int idx) {
	if (idx < 0 || idx >= object_.size()) {
		return 0;
	}

	return object_[idx];
}

std::shared_ptr<Bridge> Road::GetBridge(int idx) {
	if (idx < 0 || idx >= bridge_.size()) {
		return 0;
	}

	return bridge_[idx];
}

std::shared_ptr<ObjectReference> Road::GetObjectReference(int idx) {
	if (idx < 0 || idx >= object_reference_.size()) {
		return 0;
	}

	return object_reference_[idx];
}

double Road::GetLaneOffset(double s) {
	int i = 0;

	if (lane_offset_.size() == 0) {
		return 0;
	}

	for (; i + 1 < (int)lane_offset_.size(); i++) {
		if (s < lane_offset_[i + 1]->GetS()) {
			break;
		}
	}
	return (lane_offset_[i]->GetLaneOffset(s));
}

double Road::GetLaneOffsetPrim(double s) {
	int i = 0;

	if (lane_offset_.size() == 0) {
		return 0;
	}

	for (; i + 1 < (int)lane_offset_.size(); i++) {
		if (s < lane_offset_[i + 1]->GetS()) {
			break;
		}
	}
	return (lane_offset_[i]->GetLaneOffsetPrim(s));
}

int Road::GetNumberOfLanes(double s) {
	std::shared_ptr<LaneSection> lsec = GetLaneSectionByS(s);

	if (lsec) {
		return (lsec->GetNumberOfLanes());
	}

	return 0;
}

int Road::GetNumberOfDrivingLanes(double s) {
	std::shared_ptr<LaneSection> lsec = GetLaneSectionByS(s);

	if (lsec) {
		return (lsec->GetNumberOfDrivingLanes());
	}

	return 0;
}

std::shared_ptr<Lane> Road::GetDrivingLaneByIdx(double s, int idx) {
	int count = 0;

	std::shared_ptr<LaneSection> ls = GetLaneSectionByS(s);

	for (int i = 0; i < ls->GetNumberOfLanes(); i++) {
		if (ls->GetLaneByIdx(i)->IsDriving()) {
			if (count++ == idx) {
				return ls->GetLaneByIdx(i);
			}
		}
	}

	return 0;
}

std::shared_ptr<Lane> Road::GetDrivingLaneSideByIdx(double s, int side, int idx) {
	int count = 0;

	std::shared_ptr<LaneSection> ls = GetLaneSectionByS(s);

	for (int i = 0; i < ls->GetNumberOfLanes(); i++) {
		std::shared_ptr<Lane> lane = ls->GetLaneByIdx(i);
		if (lane->IsDriving() && SIGN(lane->GetId()) == side) {
			if (count++ == idx) {
				return lane;
			}
		}
	}

	return 0;
}

std::shared_ptr<Lane> Road::GetDrivingLaneById(double s, int id) {
	std::shared_ptr<LaneSection> ls = GetLaneSectionByS(s);

	if (ls->GetLaneById(id)->IsDriving()) {
		return ls->GetLaneById(id);
	}

	return 0;
}

int Road::GetNumberOfDrivingLanesSide(double s, int side) {
	int i;

	for (i = 0; i < GetNumberOfLaneSections() - 1; i++) {
		if (s < lane_section_[i + 1]->GetS()) {
			break;
		}
	}

	return (lane_section_[i]->GetNumberOfDrivingLanesSide(side));
}

/*
bool Road::IsDirectlyConnected(std::shared_ptr<Road> road,
							   LinkType link_type,
							   ContactPointType* contact_point) {
	if (road == nullptr) {
		return false;
	}

	std::shared_ptr<RoadLink> link = GetLink(link_type);
	if (link == nullptr) {
		return false;  // lacking successor
	}

	if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
		if (link->GetElementId() == road->GetId()) {
			if (contact_point != nullptr) {
				*contact_point = link->GetContactPointType();
			}
			return true;
		}
	} else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
		// Check all connections
		Junction* junction = Position::GetOpenDrive()->GetJunctionById(link->GetElementId());
		for (int i = 0; junction != nullptr && i < (int)junction->GetNumberOfConnections(); i++) {
			Connection* connection = junction->GetConnectionByIdx(i);
			if (connection->GetIncomingRoad() == this && connection->GetConnectingRoad() == road) {
				if (contact_point != nullptr) {
					*contact_point = connection->GetContactPoint();
				}
				// need to calculate contact point for non direct junctions?
				return true;
			}
		}
	}

	return false;
}*/

/*bool Road::IsSuccessor(std::shared_ptr<Road> road, ContactPointType* contact_point) {
	return IsDirectlyConnected(road, LinkType::SUCCESSOR, contact_point) != 0;
}

bool Road::IsPredecessor(std::shared_ptr<Road> road, ContactPointType* contact_point) {
	return IsDirectlyConnected(road, LinkType::PREDECESSOR, contact_point) != 0;
}

bool Road::IsDirectlyConnected(std::shared_ptr<Road> road) {
	// Unspecified link, check both ends
	return IsSuccessor(road) || IsPredecessor(road);
}*/

double Road::GetWidth(double s, int side, int laneTypeMask) {
	double offset0 = 0;
	double offset1 = 0;
	size_t i = 0;
	int index = 0;

	for (; i < GetNumberOfLaneSections() - 1; i++) {
		if (s < lane_section_[i + 1]->GetS()) {
			break;
		}
	}

	if (i < GetNumberOfLaneSections()) {
		std::shared_ptr<LaneSection> lsec = lane_section_[i];
		// Since the lanes are sorted from left to right,
		// side == +1 means first lane and side == -1 means last lane

		if (side > 0) {
			index = 0;
		} else {
			index = lsec->GetNumberOfLanes() - 1;
		}

		int lane_id = lsec->GetLaneIdByIdx(index);
		int step = side > 0 ? +1 : -1;

		// Find outmost lane matching requested lane type
		while (lane_id != 0 && !(lsec->GetLaneByIdx(index)->GetLaneType() & laneTypeMask)) {
			lane_id = lsec->GetLaneIdByIdx(index += step);
		}
		offset0 = fabs(lsec->GetOuterOffset(s, lane_id));

		if (side == 0) {
			// offset0 holds rightmost offset, now find outmost lane on left side of centerlane
			index = 0;
			lane_id = lsec->GetLaneIdByIdx(index);

			while (lane_id != 0 && !(lsec->GetLaneByIdx(index)->GetLaneType() & laneTypeMask)) {
				lane_id = lsec->GetLaneIdByIdx(index += 1);
			}
			offset1 = fabs(lsec->GetOuterOffset(s, lane_id));
		}
	}

	return offset0 + offset1;
}

void Road::AddLaneOffset(std::shared_ptr<LaneOffset> lane_offset) {
	// Adjust lane offset length
	if (lane_offset_.size() > 0) {
		std::shared_ptr<LaneOffset> lo_previous = lane_offset_.back();
		lo_previous->SetLength(lane_offset->GetS() - lo_previous->GetS());
	}
	lane_offset->SetLength(length_ - lane_offset->GetS());

	lane_offset_.push_back(lane_offset);
}

double Road::GetCenterOffset(double s, int lane_id) {
	// First find out what lane section
	std::shared_ptr<LaneSection> lane_section = GetLaneSectionByS(s);
	if (lane_section) {
		return lane_section->GetCenterOffset(s, lane_id);
	}

	return 0.0;
}

std::shared_ptr<Road::RoadTypeEntry> Road::GetRoadType(int idx) {
	if (type_.size() > 0) {
		return type_[idx];
	} else {
		return 0;
	}
}

std::shared_ptr<RoadLink> Road::GetLink(LinkType type) {
	for (size_t i = 0; i < link_.size(); i++) {
		if (link_[i]->GetType() == type) {
			return link_[i];
		}
	}
	return 0;  // Link of requested type is missing
}

void Road::AddLaneSection(std::shared_ptr<LaneSection> lane_section) {
	// Adjust last elevation section length
	if (lane_section_.size() > 0) {
		std::shared_ptr<LaneSection> ls_previous = lane_section_.back();
		ls_previous->SetLength(lane_section->GetS() - ls_previous->GetS());
	}
	lane_section->SetLength(length_ - lane_section->GetS());

	lane_section_.push_back(lane_section);
}

bool Road::GetZAndPitchByS(double s,
						   double* z,
						   double* z_prim,
						   double* z_primPrim,
						   double* pitch,
						   int* index) {
	if (GetNumberOfElevations() > 0) {
		if (*index < 0 || *index >= GetNumberOfElevations()) {
			*index = 0;
		}
		std::shared_ptr<Elevation> elevation = GetElevation(*index);
		if (elevation == NULL) {
			LOG("Elevation error NULL, nelev: %d elev_idx: %d\n", GetNumberOfElevations(), *index);
			return false;
		}

		if (elevation && s > elevation->GetS() + elevation->GetLength()) {
			while (s > elevation->GetS() + elevation->GetLength() && *index < GetNumberOfElevations() - 1) {
				// Move to next elevation section
				elevation = GetElevation(++*index);
			}
		} else if (elevation && s < elevation->GetS()) {
			while (s < elevation->GetS() && *index > 0) {
				// Move to previous elevation section
				elevation = GetElevation(--*index);
			}
		}

		if (elevation) {
			double p = s - elevation->GetS();
			*z = elevation->poly3_.Evaluate(p);
			*z_prim = elevation->poly3_.EvaluatePrim(p);
			*z_primPrim = elevation->poly3_.EvaluatePrimPrim(p);
			*pitch = -atan(elevation->poly3_.EvaluatePrim(p));
			return true;
		}
	}

	*z = 0.0;
	*pitch = 0.0;

	return false;
}

bool Road::UpdateZAndRollBySAndT(double s,
								 double t,
								 double* z,
								 double* roadSuperElevationPrim,
								 double* roll,
								 int* index) {
	if (GetNumberOfSuperElevations() > 0) {
		if (*index < 0 || *index >= GetNumberOfSuperElevations()) {
			*index = 0;
		}
		std::shared_ptr<Elevation> super_elevation = GetSuperElevation(*index);
		if (super_elevation == NULL) {
			LOG("Superelevation error NULL, nelev: %d elev_idx: %d\n", GetNumberOfSuperElevations(), *index);
			return false;
		}

		if (super_elevation && s > super_elevation->GetS() + super_elevation->GetLength()) {
			while (s > super_elevation->GetS() + super_elevation->GetLength()
				   && *index < GetNumberOfSuperElevations() - 1) {
				// Move to next elevation section
				super_elevation = GetSuperElevation(++*index);
			}
		} else if (super_elevation && s < super_elevation->GetS()) {
			while (s < super_elevation->GetS() && *index > 0) {
				// Move to previous elevation section
				super_elevation = GetSuperElevation(--*index);
			}
		}

		if (super_elevation) {
			double ds = s - super_elevation->GetS();
			*roll = super_elevation->poly3_.Evaluate(ds);
			*z += sin(*roll) * (t + GetLaneOffset(s));
			*roadSuperElevationPrim = super_elevation->poly3_.EvaluatePrim(ds);
			return true;
		}
	}
	return false;
}
