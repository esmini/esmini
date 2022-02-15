#ifndef LANESECTION_HPP
#define LANESECTION_HPP
#include <experimental/optional>
#include <memory>
#include <vector>
#include "Lane.hpp"
#include "StructsandDefines.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

class LaneSection {
   public:
	LaneSection(double s) : s_(s), length_(0) {}
	void AddLane(std::shared_ptr<Lane> lane);
	double GetS() { return s_; }
	Lane* GetLaneByIdx(int idx);
	Lane* GetLaneById(int id);
	int GetLaneIdByIdx(int idx);
	int GetLaneIdxById(int id);
	bool IsOSILaneById(int id);
	int GetLaneGlobalIdByIdx(int idx);
	int GetLaneGlobalIdById(int id);
	double GetOuterOffset(double s, int lane_id);
	double GetWidth(double s, int lane_id);
	int GetClosestLaneIdx(double s,
						  double t,
						  double& offset,
						  bool noZeroWidth,
						  int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING);

	/**
	Get lateral position of lane center, from road reference lane (lane id=0)
	Example: If lane id 1 is 5 m wide and lane id 2 is 4 m wide, then
			lane 1 center offset is 5/2 = 2.5 and lane 2 center offset is 5 + 4/2 = 7
	@param s distance along the road segment
	@param lane_id lane specifier, starting from center -1, -2, ... is on the right side, 1, 2... on the left
	*/
	double GetCenterOffset(double s, int lane_id);
	double GetOuterOffsetHeading(double s, int lane_id);
	double GetCenterOffsetHeading(double s, int lane_id);
	double GetLength() { return length_; }
	int GetNumberOfLanes() { return (int)lane_.size(); }
	int GetNumberOfDrivingLanes();
	int GetNumberOfDrivingLanesSide(int side);
	int GetNUmberOfLanesRight();
	int GetNUmberOfLanesLeft();
	void SetLength(double length) { length_ = length; }
	void SetSingleSide(bool single) { singleSide_ = single; }
	int GetConnectingLaneId(int incoming_lane_id, LinkType link_type);
	double GetWidthBetweenLanes(int lane_id1, int lane_id2, double s);
	double GetOffsetBetweenLanes(int lane_id1, int lane_id2, double s);
	void Print();
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);

   private:
	double s_;
	std::experimental::optional<bool> singleSide_;
	double length_;
	std::vector<std::shared_ptr<Lane>> lane_;
	std::vector<std::shared_ptr<UserData>> user_data_;
};
#endif