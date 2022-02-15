#ifndef ROAD_HPP
#define ROAD_HPP

#include <memory>
#include <string>
#include <vector>
#include "Bridge.hpp"
#include "Elevation.hpp"
#include "Geometry.hpp"
#include "LaneOffset.hpp"
#include "LaneSection.hpp"
#include "ObjectReference.hpp"
#include "RMObject.hpp"
#include "RoadLink.hpp"
#include "RoadManager.hpp"
#include "Signal.hpp"

class Road {
   public:
	enum class RoadType {
		ROADTYPE_UNKNOWN,
		ROADTYPE_RURAL,
		ROADTYPE_MOTORWAY,
		ROADTYPE_TOWN,
		ROADTYPE_LOWSPEED,
		ROADTYPE_PEDESTRIAN,
		ROADTYPE_BICYCLE
	};

	typedef struct {
		double s_;
		RoadType road_type_;
		double speed_ = 0.0;  // m/s
		std::vector<UserData*> user_data_;
		void AddUserData(UserData* userData) { user_data_.push_back(userData); }
	} RoadTypeEntry;

	enum class RoadRule { RIGHT_HAND_TRAFFIC, LEFT_HAND_TRAFFIC, ROAD_RULE_UNDEFINED };

	Road(int id, std::string name, RoadRule rule = RoadRule::RIGHT_HAND_TRAFFIC)
		: id_(id), name_(name), length_(0), junction_(-1), rule_(rule) {}
	~Road();

	void Print();
	void SetId(int id) { id_ = id; }
	int GetId() { return id_; }
	RoadRule GetRule() { return rule_; }
	void SetName(std::string name) { name_ = name; }
	Geometry* GetGeometry(int idx);
	int GetNumberOfGeometries() { return (int)geometry_.size(); }

	/**
	Retrieve the lanesection specified by vector element index (idx)
	useful for iterating over all available lane sections, e.g:
	for (int i = 0; i < road->GetNumberOfLaneSections(); i++)
	{
		int n_lanes = road->GetLaneSectionByIdx(i)->GetNumberOfLanes();
	...
	@param idx index into the vector of lane sections
	*/
	LaneSection* GetLaneSectionByIdx(int idx);

	/**
	Retrieve the lanesection index at specified s-value
	@param s distance along the road segment
	*/
	int GetLaneSectionIdxByS(double s, int start_at = 0);

	/**
	Retrieve the lanesection at specified s-value
	@param s distance along the road segment
	*/
	LaneSection* GetLaneSectionByS(double s, int start_at = 0) {
		return GetLaneSectionByIdx(GetLaneSectionIdxByS(s, start_at));
	}

	/**
	Get lateral position of lane center, from road reference lane (lane id=0)
	Example: If lane id 1 is 5 m wide and lane id 2 is 4 m wide, then
	lane 1 center offset is 5/2 = 2.5 and lane 2 center offset is 5 + 4/2 = 7
	@param s distance along the road segment
	@param lane_id lane specifier, starting from center -1, -2, ... is on the right side, 1, 2... on the left
	*/
	double GetCenterOffset(double s, int lane_id);

	LaneInfo GetLaneInfoByS(double s,
							int start_lane_link_idx,
							int start_lane_id,
							int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING);
	int GetConnectingLaneId(RoadLink* road_link, int fromLaneId, int connectingRoadId);
	double GetLaneWidthByS(double s, int lane_id);
	double GetSpeedByS(double s);
	bool GetZAndPitchByS(double s, double* z, double* z_prim, double* z_primPrim, double* pitch, int* index);
	bool UpdateZAndRollBySAndT(double s,
							   double t,
							   double* z,
							   double* roadSuperElevationPrim,
							   double* roll,
							   int* index);
	int GetNumberOfLaneSections() { return (int)lane_section_.size(); }
	std::string GetName() { return name_; }
	void SetLength(double length) { length_ = length; }
	double GetLength() const { return length_; }
	void SetJunction(int junction) { junction_ = junction; }
	int GetJunction() const { return junction_; }
	void AddLink(RoadLink* link) { link_.push_back(link); }
	void AddRoadType(RoadTypeEntry* type) { type_.push_back(type); }
	int GetNumberOfRoadTypes() const { return (int)type_.size(); }
	RoadTypeEntry* GetRoadType(int idx);
	RoadLink* GetLink(LinkType type);
	void AddLine(Line* line);
	void AddArc(Arc* arc);
	void AddSpiral(Spiral* spiral);
	void AddPoly3(Poly3* poly3);
	void AddParamPoly3(ParamPoly3* param_poly3);
	void AddElevation(Elevation* elevation);
	void AddSuperElevation(Elevation* super_elevation);
	void AddLaneSection(LaneSection* lane_section);
	void AddLaneOffset(LaneOffset* lane_offset);
	void AddSignal(Signal* signal);
	void AddObject(RMObject* object);
	void AddBridge(Bridge* bridge);
	void AddObjectReference(ObjectReference* object_reference);
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }
	Elevation* GetElevation(int idx);
	Elevation* GetSuperElevation(int idx);
	int GetNumberOfSignals();
	Signal* GetSignal(int idx);
	int GetNumberOfObjects() { return (int)object_.size(); }
	RMObject* GetObject(int idx);
	int GetNumberOfBridges() { return (int)bridge_.size(); }
	Bridge* GetBridge(int idx);
	int GetNumberOfObjectReference() { return (int)object_reference_.size(); }
	ObjectReference* GetObjectReference(int idx);
	int GetNumberOfElevations() { return (int)elevation_profile_.size(); }
	int GetNumberOfSuperElevations() { return (int)super_elevation_profile_.size(); }
	double GetLaneOffset(double s);
	double GetLaneOffsetPrim(double s);
	int GetNumberOfLanes(double s);
	int GetNumberOfLaneOffsets() { return (int)lane_offset_.size(); }
	int GetNumberOfDrivingLanes(double s);
	Lane* GetDrivingLaneByIdx(double s, int idx);
	Lane* GetDrivingLaneSideByIdx(double s, int side, int idx);
	Lane* GetDrivingLaneById(double s, int idx);
	LaneOffset* GetLaneOffsetByIdx(unsigned int idx) {
		return idx > lane_offset_.size() ? nullptr : lane_offset_[idx];
	}
	int GetNumberOfDrivingLanesSide(double s, int side);  // side = -1 right, 1 left

	/**
		Check if specified road is directly connected to at specified end of current one (this)
		@param road Road to check if connected with current one
		@param contact_point If not null it will contain the contact point of specified road
		@return true if connection exist, else false
	*/
	bool IsDirectlyConnected(Road* road, LinkType link_type, ContactPointType* contact_point = 0);

	/**
		Check if specified road is directly connected, at least in one end of current one (this)
		@param road Road to check if connected with current one
		@return true if connection exist, else false
	*/
	bool IsDirectlyConnected(Road* road);

	/**
		Check if specified road is directly connected as successor to current one (this)
		@param road Road to check if connected with current one
		@param contact_point If not null it will contain the contact point of the successor road
		@return true if connection exist, else false
	*/
	bool IsSuccessor(Road* road, ContactPointType* contact_point = 0);

	/**
		Check if specified road is directly connected as predecessor to current one (this)
		@param road Road to check if connected with current one
		@param contact_point If not null it will contain the contact point of the predecessor road
		@return true if connection exist, else false
	*/
	bool IsPredecessor(Road* road, ContactPointType* contact_point = 0);

	/**
		Get width of road
		@param s Longitudinal position/distance along the road
		@param side Side of the road: -1=right, 1=left, 0=both
		@param laneTypeMask Bitmask specifying what lane types to consider - see Lane::LaneType
		@return Width (m)
	*/
	double GetWidth(double s,
					int side,
					int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY);	// side: -1=right, 1=left, 0=both

	void Save(pugi::xml_node&);

   protected:
	int id_;
	std::string name_;
	double length_;
	int junction_;
	RoadRule rule_;

	std::vector<std::shared_ptr<RoadTypeEntry>> type_;
	std::vector<std::shared_ptr<RoadLink>> link_;
	std::vector<std::shared_ptr<Geometry>> geometry_;
	std::vector<std::shared_ptr<Elevation>> elevation_profile_;
	std::vector<std::shared_ptr<Elevation>> super_elevation_profile_;
	std::vector<std::shared_ptr<LaneSection>> lane_section_;
	std::vector<std::shared_ptr<LaneOffset>> lane_offset_;
	std::vector<std::shared_ptr<Signal>> signal_;
	std::vector<std::shared_ptr<RMObject>> object_;
	std::vector<std::shared_ptr<Bridge>> bridge_;
	std::vector<std::shared_ptr<ObjectReference>> object_reference_;
	std::vector<std::shared_ptr<UserData>> user_data_;
};
#endif