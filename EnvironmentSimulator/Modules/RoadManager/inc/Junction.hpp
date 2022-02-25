#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "Connection.hpp"
#include "Lane.hpp"
#include "LaneRoadLaneConnection.hpp"
#include "Road.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

typedef struct {
	std::vector<std::shared_ptr<UserData>> user_data_;
	int id_;
	std::string type_;
	int sequence_;
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& junction);
} JunctionController;
class Connection;
class Road;
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
		// SetGlobalId();
	}
	~Junction();
	int GetId() { return id_; }
	std::string GetName() { return name_; }
	int GetNumberOfConnections() { return (int)connection_.size(); }
	int GetNumberOfRoadConnections(int roadId, int laneId);
	/*
	LaneRoadLaneConnection GetRoadConnectionByIdx(int roadId,
												  int laneId,
												  int idx,
												  int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING);
	*/
	void AddConnection(std::shared_ptr<Connection> connection) { connection_.push_back(connection); }
	int GetNoConnectionsFromRoadId(int incomingRoadId);
	std::shared_ptr<Connection> GetConnectionByIdx(int idx) { return connection_[idx]; }
	int GetConnectingRoadIdFromIncomingRoadId(int incomingRoadId, int index);
	void Print();
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& root);
	bool IsOsiIntersection();
	// int GetGlobalId() { return global_id_; }
	// void SetGlobalId();
	int GetNumberOfControllers() { return (int)controller_.size(); }
	JunctionController* GetJunctionControllerByIdx(int index);
	void AddController(JunctionController controller) { controller_.push_back(controller); }
	std::shared_ptr<Road> GetRoadAtOtherEndOfConnectingRoad(std::shared_ptr<Road> connecting_road,
															std::shared_ptr<Road> incoming_road);
	JunctionType GetType() { return type_; }

   protected:
	std::vector<std::shared_ptr<Connection>> connection_;
	std::vector<JunctionController> controller_;
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	int id_;
	// int global_id_;
	std::string name_;
	JunctionType type_;
};