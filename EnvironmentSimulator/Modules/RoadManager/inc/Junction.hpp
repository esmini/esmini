#ifndef JUNCTION_HPP
#define JUNCTION_HPP

#include <memory>
#include <string>
#include <vector>
#include "Connection.hpp"
#include "Road.hpp"
#include "StructsandDefines.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

class Junction {
   public:
	typedef enum {
		DEFAULT,
		DIRECT,
		VIRTUAL	 // not supported yet
	} JunctionType;

	typedef enum {
		RANDOM,
		SELECTOR_ANGLE,	 // choose road which heading (relative incoming road) is closest to specified angle
	} JunctionStrategyType;

	Junction(int id, std::string name, JunctionType type) : id_(id), name_(name), type_(type) {
		SetGlobalId();
	}
	~Junction();
	int GetId() { return id_; }
	std::string GetName() { return name_; }
	int GetNumberOfConnections() { return (int)connection_.size(); }
	int GetNumberOfRoadConnections(int roadId, int laneId);
	LaneRoadLaneConnection GetRoadConnectionByIdx(int roadId,
												  int laneId,
												  int idx,
												  int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING);
	void AddConnection(std::shared_ptr<Connection> connection) { connection_.push_back(connection); }
	int GetNoConnectionsFromRoadId(int incomingRoadId);
	Connection* GetConnectionByIdx(int idx) { return connection_[idx]; }
	int GetConnectingRoadIdFromIncomingRoadId(int incomingRoadId, int index);
	void Print();
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& root);
	bool IsOsiIntersection();
	int GetGlobalId() { return global_id_; }
	void SetGlobalId();
	int GetNumberOfControllers() { return (int)controller_.size(); }
	JunctionController* GetJunctionControllerByIdx(int index);
	void AddController(JunctionController controller) { controller_.push_back(controller); }
	Road* GetRoadAtOtherEndOfConnectingRoad(Road* connecting_road, Road* incoming_road);
	JunctionType GetType() { return type_; }

   protected:
	std::vector<std::shared_ptr<Connection>> connection_;
	std::vector<JunctionController> controller_;
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	int id_;
	int global_id_;
	std::string name_;
	JunctionType type_;
};

class JunctionLaneLink {
   public:
	JunctionLaneLink(int from, int to) : from_(from), to_(to) {}
	int from_;
	int to_;
	void Print() { printf("JunctionLaneLink: from %d to %d\n", from_, to_); }
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& connection);

   private:
	std::vector<std::shared_ptr<UserData>> user_data_;
};
#endif