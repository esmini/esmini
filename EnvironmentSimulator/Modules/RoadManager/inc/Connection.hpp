#ifndef CONNECTION_HPP
#define CONNECTION_HPP

#include <memory>
#include <vector>
#include "Road.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

class Connection {
   public:
	Connection(Road* incoming_road, Road* connecting_road, ContactPointType contact_point);
	Connection(int id, Road* incoming_road, Road* connecting_road, ContactPointType contact_point);
	~Connection();
	int GetNumberOfLaneLinks() { return (int)lane_link_.size(); }
	JunctionLaneLink* GetLaneLink(int idx) { return lane_link_[idx]; }
	int GetConnectingLaneId(int incoming_lane_id);
	Road* GetIncomingRoad() { return incoming_road_; }
	Road* GetConnectingRoad() { return connecting_road_; }
	ContactPointType GetContactPoint() { return contact_point_; }
	void AddJunctionLaneLink(int from, int to);
	void Print();
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& junction);

   protected:
	std::vector<std::shared_ptr<JunctionLaneLink>> lane_link_;
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	Road* incoming_road_;
	Road* connecting_road_;
	ContactPointType contact_point_;
	int id_;
};

class JunctionLaneLink {
   public:
	JunctionLaneLink(int from, int to) : from_(from), to_(to) {}
	int from_;
	int to_;
	void Print() { printf("JunctionLaneLink: from %d to %d\n", from_, to_); }
	void AddUserData(std::shared_ptr < UserData userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& connection);

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;
};   
#endif