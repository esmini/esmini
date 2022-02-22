#pragma once

#include <cassert>
#include <memory>
#include <string>
#include <vector>
#include "LaneRoadMarkTypeLine.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

class LaneRoadMarkType {
   public:
	LaneRoadMarkType(std::string name, double width) : name_(name), width_(width) {}

	void AddLine(std::shared_ptr<LaneRoadMarkTypeLine> lane_roadMarkTypeLine);
	std::string GetName() { return name_; }
	double GetWidth() { return width_; }
	std::shared_ptr<LaneRoadMarkTypeLine> GetLaneRoadMarkTypeLineByIdx(int idx); //TODO
	int GetNumberOfRoadMarkTypeLines() { return (int)lane_roadMarkTypeLine_.size(); }
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);
	std::vector<std::shared_ptr<UserData>> getUserDataVector() { return user_data_; }
	std::vector<std::shared_ptr<LaneRoadMarkTypeLine>> getLaneRoadMarkTypeLineVector() { return lane_roadMarkTypeLine_; }


   protected:
	std::vector<std::shared_ptr<LaneRoadMarkTypeLine>> lane_roadMarkTypeLine_;
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	std::string name_;
	double width_;
};
