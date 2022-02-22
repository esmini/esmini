#pragma once

#include <string.h>
#include <cassert>
#include <memory>
#include <vector>
#include "LaneRoadMarkType.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

class LaneRoadMark {
   public:
	enum RoadMarkType {
		NONE_TYPE = 1,
		SOLID = 2,
		BROKEN = 3,
		SOLID_SOLID = 4,
		SOLID_BROKEN = 5,
		BROKEN_SOLID = 6,
		BROKEN_BROKEN = 7,
		BOTTS_DOTS = 8,
		GRASS = 9,
		CURB = 10
	};

	enum RoadMarkWeight { STANDARD, BOLD };

	enum RoadMarkMaterial {
		STANDARD_MATERIAL  // only "standard" is available for now
	};

	enum RoadMarkLaneChange { INCREASE, DECREASE, BOTH, NONE_LANECHANGE };

	LaneRoadMark(double s_offset,
				 RoadMarkType type,
				 RoadMarkWeight weight,
				 RoadMarkColor color,
				 RoadMarkMaterial material,
				 RoadMarkLaneChange lane_change,
				 double width,
				 double height)
		: s_offset_(s_offset),
		  type_(type),
		  weight_(weight),
		  color_(color),
		  material_(material),
		  lane_change_(lane_change),
		  width_(width),
		  height_(height) {}

	void AddType(std::shared_ptr<LaneRoadMarkType> lane_roadMarkType) {
		lane_roadMarkType_.push_back(lane_roadMarkType);
	}

	double GetSOffset() { return s_offset_; }
	double GetWidth() { return width_; }
	double GetHeight() { return height_; }
	RoadMarkType GetType() { return type_; }
	RoadMarkWeight GetWeight() { return weight_; }
	RoadMarkColor GetColor() { return color_; }
	RoadMarkMaterial GetMaterial() { return material_; }
	RoadMarkLaneChange GetLaneChange() { return lane_change_; }

	int GetNumberOfRoadMarkTypes() { return (int)lane_roadMarkType_.size(); }
	std::shared_ptr<LaneRoadMarkType> GetLaneRoadMarkTypeByIdx(int idx);

	static RoadMarkColor ParseColor(pugi::xml_node node);

	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);

	std::vector<std::shared_ptr<UserData>> getUserDataVector() { return user_data_; }
	std::vector<std::shared_ptr<LaneRoadMarkType>> getLaneRoadMarkTypeVector() { return lane_roadMarkType_; }

   protected:
	std::vector<std::shared_ptr<LaneRoadMarkType>> lane_roadMarkType_;
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	double s_offset_;
	RoadMarkType type_;
	RoadMarkWeight weight_;
	RoadMarkColor color_;
	RoadMarkMaterial material_;
	RoadMarkLaneChange lane_change_;
	double width_;
	double height_;
};