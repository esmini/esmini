#pragma once

#include "StructsandDefines.hpp"

class LaneRoadLaneConnection {
   public:
	LaneRoadLaneConnection()
		: lane_id_(0),
		  connecting_road_id_(-1),
		  connecting_lane_id_(0),
		  contact_point_(ContactPointType::CONTACT_POINT_UNDEFINED) {}
	LaneRoadLaneConnection(int lane_id, int connecting_road_id, int connecting_lane_id)
		: lane_id_(lane_id),
		  connecting_road_id_(connecting_road_id),
		  connecting_lane_id_(connecting_lane_id),
		  contact_point_(ContactPointType::CONTACT_POINT_UNDEFINED) {}
	void SetLane(int id) { lane_id_ = id; }
	void SetConnectingRoad(int id) { connecting_road_id_ = id; }
	void SetConnectingLane(int id) { connecting_lane_id_ = id; }
	int GetLaneId() { return lane_id_; }
	int GetConnectingRoadId() { return connecting_road_id_; }
	int GetConnectinglaneId() { return connecting_lane_id_; }

	ContactPointType contact_point_;

   private:
	int lane_id_;
	int connecting_road_id_;
	int connecting_lane_id_;
};