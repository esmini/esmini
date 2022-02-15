#ifndef LANE_HPP
#define LANE_HPP

#include <memory>
#include <vector>
#include "CommonMini.hpp"
#include "LaneLink.hpp"
#include "LaneMaterial.hpp"
#include "LaneRoadMark.hpp"
#include "LaneSpeed.hpp"
#include "LaneWidth.hpp"
#include "OSI.hpp"
#include "Userdata.hpp"

class Lane {
   public:
	enum LanePosition { LANE_POS_CENTER, LANE_POS_LEFT, LANE_POS_RIGHT };

	typedef enum {
		LANE_TYPE_NONE = (1 << 0),
		LANE_TYPE_DRIVING = (1 << 1),
		LANE_TYPE_STOP = (1 << 2),
		LANE_TYPE_SHOULDER = (1 << 3),
		LANE_TYPE_BIKING = (1 << 4),
		LANE_TYPE_SIDEWALK = (1 << 5),
		LANE_TYPE_BORDER = (1 << 6),
		LANE_TYPE_RESTRICTED = (1 << 7),
		LANE_TYPE_PARKING = (1 << 8),
		LANE_TYPE_BIDIRECTIONAL = (1 << 9),
		LANE_TYPE_MEDIAN = (1 << 10),
		LANE_TYPE_SPECIAL1 = (1 << 11),
		LANE_TYPE_SPECIAL2 = (1 << 12),
		LANE_TYPE_SPECIAL3 = (1 << 13),
		LANE_TYPE_ROADWORKS = (1 << 14),
		LANE_TYPE_TRAM = (1 << 15),
		LANE_TYPE_RAIL = (1 << 16),
		LANE_TYPE_ENTRY = (1 << 17),
		LANE_TYPE_EXIT = (1 << 18),
		LANE_TYPE_OFF_RAMP = (1 << 19),
		LANE_TYPE_ON_RAMP = (1 << 20),
		LANE_TYPE_ANY_DRIVING = LANE_TYPE_DRIVING | LANE_TYPE_ENTRY | LANE_TYPE_EXIT | LANE_TYPE_OFF_RAMP
								| LANE_TYPE_ON_RAMP | LANE_TYPE_PARKING,
		LANE_TYPE_ANY_ROAD = LANE_TYPE_ANY_DRIVING | LANE_TYPE_RESTRICTED | LANE_TYPE_STOP,
		LANE_TYPE_ANY = (0xFFFFFFFF)
	} LaneType;

	// Construct & Destruct
	Lane()
		: id_(0),
		  type_(LaneType::LANE_TYPE_NONE),
		  level_(-1),
		  offset_from_ref_(0.0),
		  global_id_(0),
		  lane_boundary_(0) {}
	Lane(int id, Lane::LaneType type)
		: id_(id), type_(type), level_(-1), offset_from_ref_(0), global_id_(0), lane_boundary_(0) {}
	~Lane() {}

	// Base Get Functions
	int GetId() { return id_; }
	double GetOffsetFromRef() { return offset_from_ref_; }
	LaneType GetLaneType() { return type_; }
	int GetGlobalId() { return global_id_; }

	// Add Functions
	void AddLink(std::shared_ptr<LaneLink> lane_link) { link_.push_back(lane_link); }
	void AddLaneWidth(std::shared_ptr<LaneWidth> lane_width) { lane_width_.push_back(lane_width); }
	void AddLaneRoadMark(std::shared_ptr<LaneRoadMark> lane_roadMark) {
		lane_roadMark_.push_back(lane_roadMark);
	}
	void AddLaneMaterial(std::shared_ptr<LaneMaterial> lane_material) {
		lane_material_.push_back(lane_material);
	}
	void AddLaneSpeed(std::shared_ptr<LaneSpeed> lane_speed) { lane_speed_.push_back(lane_speed); }
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	// Get Functions
	int GetNumberOfRoadMarks() { return (int)lane_roadMark_.size(); }
	int GetNumberOfLinks() { return (int)link_.size(); }
	int GetNumberOfLaneWidths() { return (int)lane_width_.size(); }

	std::shared_ptr<LaneLink> GetLink(LinkType type);
	std::shared_ptr<LaneWidth> GetWidthByIndex(int index);
	std::shared_ptr<LaneWidth> GetWidthByS(double s);
	std::shared_ptr<LaneRoadMark> GetLaneRoadMarkByIdx(int idx);

	RoadMarkInfo GetRoadMarkInfoByS(int track_id, int lane_id, double s);
	OSIPoints* GetOSIPoints() { return &osi_points_; }
	std::vector<int> GetLineGlobalIds();
	LaneBoundaryOSI* GetLaneBoundary() { return lane_boundary_; }
	int GetLaneBoundaryGlobalId();

	// Set Functions
	void SetGlobalId();
	void SetLaneBoundary(LaneBoundaryOSI* lane_boundary);
	void SetOffsetFromRef(double offset) { offset_from_ref_ = offset; }
	void SetLevel(int level) { level_ = level; }

	// Others
	bool IsType(Lane::LaneType type);
	bool IsCenter();
	bool IsDriving();
	bool IsOSIIntersection() { return osiintersection_ > 0; }
	void SetOSIIntersection(int is_osi_intersection) { osiintersection_ = is_osi_intersection; }
	int GetOSIIntersectionId() { return osiintersection_; }
	void Print();
	void Save(pugi::xml_node&);
	OSIPoints osi_points_;

   protected:
	std::vector<std::shared_ptr<LaneLink>> link_;
	std::vector<std::shared_ptr<LaneWidth>> lane_width_;
	std::vector<std::shared_ptr<LaneRoadMark>> lane_roadMark_;
	std::vector<std::shared_ptr<LaneMaterial>> lane_material_;
	std::vector<std::shared_ptr<LaneSpeed>> lane_speed_;
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	int id_;			   // center = 0, left > 0, right < 0
	int global_id_;		   // Unique ID for OSI
	int osiintersection_;  // flag to see if the lane is part of an osi-lane section or not
	LaneType type_;
	int level_;	 // boolean, true = keep lane on level
	double offset_from_ref_;

	LaneBoundaryOSI* lane_boundary_;
};

#endif