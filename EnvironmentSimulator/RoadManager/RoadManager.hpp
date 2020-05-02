/* 
 * esmini - Environment Simulator Minimalistic 
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * 
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#ifndef OPENDRIVE_HH_
#define OPENDRIVE_HH_

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include "pugixml.hpp"

namespace roadmanager
{

	class Polynomial
	{
	public:
		Polynomial() : a_(0), b_(0), c_(0), d_(0), s_max_(1.0) {}
		Polynomial(double a, double b, double c, double d, double s_max = 1) : a_(a), b_(b), c_(c), d_(d), s_max_(s_max) {}
		void Set(double a, double b, double c, double d, double s_max = 1);
		void SetA(double a) { a_ = a; }
		void SetB(double b) { b_ = b; }
		void SetC(double c) { c_ = c; }
		void SetD(double d) { d_ = d; }
		void SetSMax(double s_max) { s_max_ = s_max; }
		double GetA() { return a_; }
		double GetB() { return b_; }
		double GetC() { return c_; }
		double GetD() { return d_; }
		double GetSMax() { return s_max_; }
		double Evaluate(double s);
		double EvaluatePrim(double s);
		double EvaluatePrimPrim(double s);

	private:
		double a_;
		double b_;
		double c_;
		double d_;
		double s_max_;
	};


	class Geometry
	{
	public:
		enum GeometryType
		{
			GEOMETRY_TYPE_UNKNOWN,
			GEOMETRY_TYPE_LINE,
			GEOMETRY_TYPE_ARC,
			GEOMETRY_TYPE_SPIRAL,
			GEOMETRY_TYPE_POLY3,
			GEOMETRY_TYPE_PARAM_POLY3,
		};

		Geometry() : s_(0.0), x_(0.0), y_(0), hdg_(0), length_(0), type_(GEOMETRY_TYPE_UNKNOWN) {}
		Geometry(double s, double x, double y, double hdg, double length, GeometryType type) :
			s_(s), x_(x), y_(y), hdg_(hdg), length_(length), type_(type) {}
		virtual ~Geometry() {}

		GeometryType GetType() { return type_; }
		double GetLength() { return length_; }
		double GetX() { return x_; }
		double GetY() { return y_; }
		double GetHdg() { return hdg_; }
		double GetS() { return s_; }
		virtual double EvaluateCurvatureDS(double ds) = 0;
		virtual void Print();
		virtual void EvaluateDS(double ds, double *x, double *y, double *h);

	private:
		double s_;
		double x_;
		double y_;
		double hdg_;
		double length_;
		GeometryType type_;
	};


	class Line : public Geometry
	{
	public:
		Line(double s, double x, double y, double hdg, double length) : Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_LINE) {}
		~Line() {};

		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);
		double EvaluateCurvatureDS(double ds) { (void)ds; return 0; }
	};


	class Arc : public Geometry
	{
	public:
		Arc(double s, double x, double y, double hdg, double length, double curvature) :
			Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_ARC), curvature_(curvature) {}
		~Arc() {}

		double EvaluateCurvatureDS(double ds) { (void)ds; return curvature_; }
		double GetRadius() { return std::fabs(1.0 / curvature_); }
		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);

	private:
		double curvature_;
	};


	class Spiral : public Geometry
	{
	public:
		Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end) :
			Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_SPIRAL),
			curv_start_(curv_start), curv_end_(curv_end), c_dot_(0.0), x0_(0.0), y0_(0.0), h0_(0.0), s0_(0.0) {}
		~Spiral() {};

		double GetCurvStart() { return curv_start_; }
		double GetCurvEnd() { return curv_end_; }
		double GetX0() { return x0_; }
		double GetY0() { return y0_; }
		double GetH0() { return h0_; }
		double GetS0() { return s0_; }
		double GetCDot() { return c_dot_; }
		void SetX0(double x0) { x0_ = x0; }
		void SetY0(double y0) { y0_ = y0; }
		void SetH0(double h0) { h0_ = h0; }
		void SetS0(double s0) { s0_ = s0; }
		void SetCDot(double c_dot) { c_dot_ = c_dot; }
		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);
		double EvaluateCurvatureDS(double ds);

	private:
		double curv_start_;
		double curv_end_;
		double c_dot_;
		double x0_; // 0 if spiral starts with curvature = 0
		double y0_; // 0 if spiral starts with curvature = 0
		double h0_; // 0 if spiral starts with curvature = 0
		double s0_; // 0 if spiral starts with curvature = 0
	};


	class Poly3 : public Geometry
	{
	public:
		Poly3(double s, double x, double y, double hdg, double length, double a, double b, double c, double d) :
			Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_POLY3), umax_(0.0)
		{
			poly3_.Set(a, b, c, d);
			
		}
		~Poly3() {};
		
		void SetUMax(double umax) { umax_ = umax; }
		double GetUMax() { return umax_; }
		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);
		double EvaluateCurvatureDS(double ds);

		Polynomial poly3_;

	private:
		double umax_;
	};


	class ParamPoly3 : public Geometry
	{
	public:
		enum PRangeType
		{
			P_RANGE_UNKNOWN,
			P_RANGE_NORMALIZED,
			P_RANGE_ARC_LENGTH
		};

		ParamPoly3(
			double s, double x, double y, double hdg, double length,
			double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV, PRangeType p_range) :
			Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_PARAM_POLY3), p_range_(p_range)
		{
			poly3U_.Set(aU, bU, cU, dU);
			poly3V_.Set(aV, bV, cV, dV);
		}
		~ParamPoly3() {};

		double GetPRange() { return p_range_; }
		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);
		double EvaluateCurvatureDS(double ds);

		Polynomial poly3U_;
		Polynomial poly3V_;

	private:
		int p_range_;
	};


	class Elevation
	{
	public:
		Elevation(double s, double a, double b, double c, double d) : s_(s), length_(0)
		{
			poly3_.Set(a, b, c, d);
		}

		double GetS() { return s_; }
		void SetLength(double length) { length_ = length; }
		double GetLength() { return length_; }
		void Print();

		Polynomial poly3_;

	private:
		double s_;
		double length_;
	};

	typedef enum 
	{
		UNKNOWN,
		SUCCESSOR,
		PREDECESSOR,
		NONE
	} LinkType;


	class LaneLink
	{
	public:
		LaneLink(LinkType type, int id) : type_(type), id_(id) {}

		LinkType GetType() { return type_; }
		int GetId() { return id_; }
		void Print();

	private:
		LinkType type_;
		int id_;
	};

	class LaneWidth
	{
	public:
		LaneWidth(double s_offset, double a, double b, double c, double d) : s_offset_(s_offset)
		{
			poly3_.Set(a, b, c, d);
		}

		double GetSOffset() { return s_offset_; }
		void Print();

		Polynomial poly3_;

	private:
		double s_offset_;
	};

	class LaneOffset
	{
	public:
		LaneOffset() {}
		LaneOffset(double s, double a, double b, double c, double d) : s_(s), length_(0) 
		{ 
			polynomial_.Set(a, b, c, d); 
		}

		void Set(double s, double a, double b, double c, double d) 
		{
			s_ = s;
			polynomial_.Set(a, b, c, d); 
		}
		void SetLength(double length) { length_ = length; }
		double GetS() { return s_; }
		double GetLength() { return length_; }
		double GetLaneOffset(double s);
		double GetLaneOffsetPrim(double s);
		void Print();

	private:
		Polynomial polynomial_;
		double s_;
		double length_;
	};

	class Lane
	{
	public:
		enum LanePosition
		{
			LANE_POS_CENTER,
			LANE_POS_LEFT,
			LANE_POS_RIGHT
		};

		enum LaneType
		{
			LANE_TYPE_NONE,
			LANE_TYPE_DRIVING,
			LANE_TYPE_STOP,
			LANE_TYPE_SHOULDER,
			LANE_TYPE_BIKING,
			LANE_TYPE_SIDEWALK,
			LANE_TYPE_BORDER,
			LANE_TYPE_RESTRICTED,
			LANE_TYPE_PARKING,
			LANE_TYPE_BIDIRECTIONAL,
			LANE_TYPE_MEDIAN,
			LANE_TYPE_SPECIAL1,
			LANE_TYPE_SPECIAL2,
			LANE_TYPE_SPECIAL3,
			LANE_TYPE_ROADMARKS,
			LANE_TYPE_TRAM,
			LANE_TYPE_RAIL,
			LANE_TYPE_ENTRY,
			LANE_TYPE_EXIT,
			LANE_TYPE_OFF_RAMP,
			LANE_TYPE_ON_RAMP,
		};

		Lane(int id, Lane::LaneType type) : id_(id), type_(type), level_(1), offset_from_ref_(0) {}
		void AddLink(LaneLink *lane_link) { link_.push_back(lane_link); }
		int GetId() { return id_; }
		LaneWidth *GetWidthByIndex(int index) { return lane_width_[index]; }
		LaneWidth *GetWidthByS(double s);
		LaneLink *GetLink(LinkType type);
		void SetOffsetFromRef(double offset) { offset_from_ref_ = offset; }
		double GetOffsetFromRef() { return offset_from_ref_; }
		void AddLaneWIdth(LaneWidth *lane_width) { lane_width_.push_back(lane_width); }
		int IsDriving();
		void Print();

	private:
		int id_;		// center = 0, left > 0, right < 0
		LaneType type_;
		int level_;	// boolean, true = keep lane on level
		double offset_from_ref_;
		std::vector<LaneLink*> link_;
		std::vector<LaneWidth*> lane_width_;
	};

	class LaneSection
	{
	public:
		LaneSection(double s) : s_(s), length_(0) {}
		void AddLane(Lane *lane);
		double GetS() { return s_; }
		Lane* GetLaneByIdx(int idx);
		Lane* GetLaneById(int id);
		int FindClosestDrivingLane(int id);
		int GetLaneIdByIdx(int idx);
		int GetLaneIdxById(int id);
		double GetOuterOffset(double s, int lane_id);
		double GetWidth(double s, int lane_id);
		int GetClosestLaneIdx(double s, double t, double &offset);

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
		int GetConnectingLaneId(int incoming_lane_id, LinkType link_type);
		double GetWidthBetweenLanes(int lane_id1, int lane_id2, double s);
		double GetOffsetBetweenLanes(int lane_id1, int lane_id2, double s);
		void Print();

	private:
		double s_;
		double length_;
		std::vector<Lane*> lane_;
	};

	enum ContactPointType
	{
		CONTACT_POINT_UNKNOWN,
		CONTACT_POINT_START,
		CONTACT_POINT_END,
		CONTACT_POINT_NONE,  // No contact point for element type junction
	};

	class RoadLink
	{
	public:
		typedef enum 
		{
			ELEMENT_TYPE_UNKNOWN,
			ELEMENT_TYPE_ROAD,
			ELEMENT_TYPE_JUNCTION,
		} ElementType;

		RoadLink() : type_(NONE), element_id_(-1), element_type_(ELEMENT_TYPE_UNKNOWN), contact_point_type_(CONTACT_POINT_UNKNOWN) {}
		RoadLink(LinkType type, ElementType element_type, int element_id, ContactPointType contact_point_type) :
			type_(type), element_id_(element_id), element_type_(element_type),  contact_point_type_(contact_point_type) {}
		RoadLink(LinkType type, pugi::xml_node node);

		int GetElementId() { return element_id_; }
		LinkType GetType() { return type_; }
		RoadLink::ElementType GetElementType() { return element_type_; }
		ContactPointType GetContactPointType() { return contact_point_type_; }

		void Print();

	private:
		LinkType type_;
		int element_id_;
		ElementType element_type_;
		ContactPointType contact_point_type_;
	};

	struct LaneInfo
	{
		int lane_section_idx_;
		int lane_id_;
	};

	enum RoadType
	{
		ROADTYPE_UNKNOWN,
		ROADTYPE_RURAL,
		ROADTYPE_MOTORWAY,
		ROADTYPE_TOWN,
		ROADTYPE_LOWSPEED,
		ROADTYPE_PEDESTRIAN,
		ROADTYPE_BICYCLE
	};

	typedef struct
	{
		double s_;
		RoadType road_type_;
		double speed_;  // m/s
	} RoadTypeEntry;

	class Road
	{
	public:

		Road(int id, std::string name) : id_(id), name_(name), length_(0), junction_(0) {}
		~Road();

		void Print();
		void SetId(int id) { id_ = id; }
		int GetId() { return id_; }
		void SetName(std::string name) { name_ = name; }
		Geometry *GetGeometry(int idx);
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
		LaneSection *GetLaneSectionByIdx(int idx);
		
		/**
		Retrieve the lanesection index at specified s-value
		@param s distance along the road segment
		*/
		int GetLaneSectionIdxByS(double s, int start_at = 0);

		/**
		Retrieve the lanesection at specified s-value
		@param s distance along the road segment
		*/
		LaneSection* GetLaneSectionByS(double s, int start_at = 0) { return GetLaneSectionByIdx(GetLaneSectionIdxByS(s, start_at)); }

		/**
		Get lateral position of lane center, from road reference lane (lane id=0)
		Example: If lane id 1 is 5 m wide and lane id 2 is 4 m wide, then
		lane 1 center offset is 5/2 = 2.5 and lane 2 center offset is 5 + 4/2 = 7
		@param s distance along the road segment
		@param lane_id lane specifier, starting from center -1, -2, ... is on the right side, 1, 2... on the left
		*/
		double GetCenterOffset(double s, int lane_id);

		LaneInfo GetLaneInfoByS(double s, int start_lane_link_idx, int start_lane_id);
		double GetLaneWidthByS(double s, int lane_id);
		double GetSpeedByS(double s);
		bool GetZAndPitchByS(double s, double *z, double *pitch, int *index);
		int GetNumberOfLaneSections() { return (int)lane_section_.size(); }
		std::string GetName() { return name_; }
		void SetLength(double length) { length_ = length; }
		double GetLength() { return length_; }
		void SetJunction(int junction) { junction_ = junction; }
		int GetJunction() { return junction_; }
		void AddLink(RoadLink *link) { link_.push_back(link); }
		void AddRoadType(RoadTypeEntry *type) { type_.push_back(type); }
		RoadLink *GetLink(LinkType type);
		void AddLine(Line *line);
		void AddArc(Arc *arc);
		void AddSpiral(Spiral *spiral);
		void AddPoly3(Poly3 *poly3);
		void AddParamPoly3(ParamPoly3 *param_poly3);
		void AddElevation(Elevation *elevation);
		void AddLaneSection(LaneSection *lane_section);
		void AddLaneOffset(LaneOffset *lane_offset);
		Elevation *GetElevation(int idx);
		int GetNumberOfElevations() { return (int)elevation_profile_.size(); }
		double GetLaneOffset(double s);
		double GetLaneOffsetPrim(double s);
		int GetNumberOfLanes(double s);
		int GetNumberOfDrivingLanes(double s);
		Lane* GetDrivingLaneByIdx(double s, int idx);
		int GetNumberOfDrivingLanesSide(double s, int side);  // side = -1 right, 1 left
		double GetDrivableWidth(double s, int side=0);   // side: -1=right, 1=left, 0=both

	protected:
		int id_;
		std::string name_;
		double length_;
		int junction_;
		std::vector<RoadTypeEntry*> type_;
		std::vector<RoadLink*> link_;
		std::vector<Geometry*> geometry_;
		std::vector<Elevation*> elevation_profile_;
		std::vector<LaneSection*> lane_section_;
		std::vector<LaneOffset*> lane_offset_;
	};

	class LaneRoadLaneConnection
	{
	public:
		LaneRoadLaneConnection() :
			lane_id_(0), connecting_road_id_(-1), connecting_lane_id_(0), contact_point_(ContactPointType::CONTACT_POINT_NONE) {}
		LaneRoadLaneConnection(int lane_id, int connecting_road_id, int connecting_lane_id) :
			lane_id_(lane_id), connecting_road_id_(connecting_road_id), connecting_lane_id_(connecting_lane_id), contact_point_(ContactPointType::CONTACT_POINT_NONE) {}
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

	class JunctionLaneLink
	{
	public:
		JunctionLaneLink(int from, int to) : from_(from), to_(to) {}
		int from_;
		int to_;
		void Print() { printf("JunctionLaneLink: from %d to %d\n", from_, to_); }
	};

	class Connection
	{
	public:
		Connection(Road *incoming_road, Road *connecting_road, ContactPointType contact_point);
		~Connection();
		int GetNumberOfLaneLinks() { return (int)lane_link_.size(); }
		JunctionLaneLink *GetLaneLink(int idx) { return lane_link_[idx]; }
		int GetConnectingLaneId(int incoming_lane_id);
		Road *GetIncomingRoad() { return incoming_road_; }
		Road *GetConnectingRoad() { return connecting_road_; }
		ContactPointType GetContactPoint() { return contact_point_; }
		void AddJunctionLaneLink(int from, int to);
		void Print();

	private:
		Road *incoming_road_;
		Road *connecting_road_;
		ContactPointType contact_point_;
		std::vector<JunctionLaneLink*> lane_link_;
	};

	class Junction
	{
	public:
		typedef enum
		{
			RANDOM,
			STRAIGHT,
		} JunctionStrategyType;

		Junction(int id, std::string name) : id_(id), name_(name) {}
		~Junction();
		int GetId() { return id_; }
		std::string GetName() { return name_; }
		int GetNumberOfConnections() { return (int)connection_.size(); }
		int GetNumberOfRoadConnections(int roadId, int laneId);
		LaneRoadLaneConnection GetRoadConnectionByIdx(int roadId, int laneId, int idx);
		void AddConnection(Connection *connection) { connection_.push_back(connection); }
		int GetNoConnectionsFromRoadId(int incomingRoadId);
		Connection *GetConnectionByIdx(int idx) { return connection_[idx]; }
		int GetConnectingRoadIdFromIncomingRoadId(int incomingRoadId, int index);
		void Print();

	private:
		std::vector<Connection*> connection_;
		int id_;
		std::string name_;
	};

	class OpenDrive
	{
	public:
		OpenDrive() {};
		OpenDrive(const char *filename);
		~OpenDrive();

		/**
		Load a road network, specified in the OpenDRIVE file format
		@param filename OpenDRIVE file
		@param replace If true any old road data will be erased, else new will be added to the old
		*/
		bool LoadOpenDriveFile(const char *filename, bool replace = true);

		/**
		Get the filename of currently loaded OpenDRIVE file
		*/
		std::string GetOpenDriveFilename() { return odr_filename_; }

		/**
		Retrieve a road segment specified by road ID 
		@param id road ID as specified in the OpenDRIVE file
		*/
		Road* GetRoadById(int id);

		/**
		Retrieve a road segment specified by road vector element index
		useful for iterating over all available road segments, e.g:
		for (int i = 0; i < GetNumOfRoads(); i++)
		{
			int n_lanes = GetRoadyIdx(i)->GetNumberOfLanes();
		...
		@param idx index into the vector of roads
		*/
		Road* GetRoadByIdx(int idx);
		Geometry* GetGeometryByIdx(int road_idx, int geom_idx);
		int GetTrackIdxById(int id);
		int GetTrackIdByIdx(int idx);
		int GetNumOfRoads() { return (int)road_.size(); }
		Junction* GetJunctionById(int id);
		Junction* GetJunctionByIdx(int idx);
		int GetNumOfJunctions() { return (int)junction_.size(); }
		/**
		Check if two roads are connected directly
		@param road1_id Id of the first road
		@param road2_id Id of the second road
		@param angle if connected, the angle between road 2 and road 1 is returned here
		@return 0 if not connected, -1 if road 2 is the predecessor of road 1, +1 if road 2 is the successor of road 1
		*/
		int IsDirectlyConnected(int road1_id, int road2_id, double &angle);
		bool IsIndirectlyConnected(int road1_id, int road2_id, int* &connecting_road_id, int* &connecting_lane_id, int lane1_id = 0, int lane2_id = 0);

		/**
		Add any missing connections so that road connectivity is two-ways
		Look at all road connections, and make sure they are defined both ways
		@param idx index into the vector of roads
		@return number of added connections
		*/
		int CheckConnections();
		int CheckLink(Road *road, RoadLink *link, ContactPointType expected_contact_point_type);
		int CheckConnectedRoad(Road *road, RoadLink *link, ContactPointType expected_contact_point_type, Road *road2, RoadLink *link2);
		int CheckJunctionConnection(Junction *junction, Connection *connection);
		std::string ContactPointType2Str(ContactPointType type);
		std::string ElementType2Str(RoadLink::ElementType type);


		void Print();
	
	private:
		pugi::xml_node root_node_;
		std::vector<Road*> road_;
		std::vector<Junction*> junction_;
		std::string odr_filename_;
	};

	typedef struct
	{
		double pos[3];		// position, in global coordinate system
		double heading;		// road heading at steering target point
		double pitch;		// road pitch (inclination) at steering target point
		double roll;		// road roll (camber) at steering target point
		double width;		// lane width
		double curvature;	// road curvature at steering target point
		double speed_limit; // speed limit given by OpenDRIVE type entry
	} RoadLaneInfo;
	
	typedef struct
	{
		RoadLaneInfo road_lane_info;  // Road info at probe target position
		double relative_pos[3];       // probe target position relative vehicle (pivot position object) coordinate system
		double relative_h;			  // heading angle to probe target from and relatove to vehicle (pivot position)
	} RoadProbeInfo;

	typedef struct
	{
		double ds;				// delta s (longitudinal distance)
		double dt;				// delta t (lateral distance)
		int dLaneId;			// delta laneId (increasing left and decreasing to the right)
	} PositionDiff;

	// Forward declaration of Route
	class Route;

	class Position
	{
	public:

		enum LookAheadMode
		{
			LOOKAHEADMODE_AT_LANE_CENTER,
			LOOKAHEADMODE_AT_ROAD_CENTER,
			LOOKAHEADMODE_AT_CURRENT_LATERAL_OFFSET,
		};

		explicit Position();
		explicit Position(int track_id, double s, double t);
		explicit Position(int track_id, int lane_id, double s, double offset);
		explicit Position(double x, double y, double z, double h, double p, double r);
		explicit Position(double x, double y, double z, double h, double p, double r, bool calculateTrackPosition);
		~Position();
		
		void Init();
		static bool LoadOpenDrive(const char *filename);
		static OpenDrive* GetOpenDrive();
		int GotoClosestDrivingLaneAtCurrentPosition();
		void SetTrackPos(int track_id, double s, double t, bool calculateXYZ = true);
		void ForceLaneId(int lane_id);
		void SetLanePos(int track_id, int lane_id, double s, double offset, int lane_section_idx = -1);
		void SetInertiaPos(double x, double y, double z, double h, double p, double r, bool updateTrackPos = true);
		void SetHeading(double heading);
		void SetHeadingRelative(double heading);
		void SetHeadingRelativeRoadDirection(double heading);
		void XYZH2TrackPos(double x, double y, double z, double h, bool copyZAndPitch = true);
		int MoveToConnectingRoad(RoadLink *road_link, ContactPointType &contact_point_type, Junction::JunctionStrategyType strategy = Junction::RANDOM);
		double FindDistToPos(Position *pos, RoadLink *link, Road *road, int &call_count, int level_count, bool &found);

		void SetRoute(Route *route);
		void CalcRoutePosition();
		const roadmanager::Route* GetRoute() const { return route_; }
		roadmanager::Route* GetRoute() { return route_; }

		/**
		Set the current position along the route.
		@param position A regular position created with road, lane or world coordinates
		@return Non zero return value indicates error of some kind
		*/
		int SetRoutePosition(Position *position);

		/**
		Retrieve the S-value of the current route position. Note: This is the S along the
		complete route, not the actual individual roads.
		*/
		double GetRouteS() { return s_; }

		/**
		Move current position forward, or backwards, ds meters along the route
		@param ds Distance to move, negative will move backwards
		@return Non zero return value indicates error of some kind
		*/
		int MoveRouteDS(double ds);

		/**
		Move current position to specified S-value along the route
		@param route_s Distance to move, negative will move backwards
		@param laneId Explicit (not delta/offset) lane ID
		@param laneOffset Explicit (not delta/offset) lane offset value
		@return Non zero return value indicates error of some kind
		*/
		int SetRouteLanePosition(Route* route, double route_s, int laneId, double  laneOffset);

		/**
		Move current position to specified S-value along the route
		@param route_s Distance to move, negative will move backwards
		@return Non zero return value indicates error of some kind
		*/
		int SetRouteS(Route* route, double route_s);

		/**
		Straight (not route) distance between the current position and the one specified in argument
		@param target_position The position to measure distance from current position. 
		@param x (meter). X component of the relative distance.
		@param y (meter). Y component of the relative distance.
		@return distance (meter). Negative if the specified position is behind the current one.
		*/
		double getRelativeDistance(Position target_position, double &x, double &y);

		/**
		Find out the difference between two position objects, in effect subtracting the values 
		@param positionB The position which will be subtracted from the current position object
		@return true if position found and parameter values are valid, else false
		*/
		bool Delta(Position pos_b, PositionDiff &diff);

		/**
		Is the current position ahead of the one specified in argument
		This method is more efficient than getRelativeDistance
		@param target_position The position to compare the current to.
		@return true of false
		*/
		bool IsAheadOf(Position target_position);

		/**
		Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
		@param lookahead_distance The distance, along the road, to the point
		@param data Struct to fill in calculated values, see typdef for details
		@param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum
		@return 0 if successful, -1 if not
		*/
		int GetProbeInfo(double lookahead_distance, RoadProbeInfo *data, LookAheadMode lookAheadMode);

		/**
		Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
		@param target_pos The target position
		@param data Struct to fill in calculated values, see typdef for details
		@return 0 if successful, -1 if not
		*/
		int GetProbeInfo(Position *target_pos, RoadProbeInfo *data);

		/**
		Get information of current lane at a specified distance from object along the road ahead
		@param lookahead_distance The distance, along the road, to the point
		@param data Struct to fill in calculated values, see typdef for details
		@param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode enum
		@return 0 if successful, -1 if not
		*/
		int GetRoadLaneInfo(double lookahead_distance, RoadLaneInfo *data, LookAheadMode lookAheadMode);

		/**
		Get information of current lane at a specified distance from object along the road ahead
		@param lookahead_distance The distance, along the road, to the point
		@param data Struct to fill in calculated values, see typdef for details
		@return 0 if successful, -1 if not
		*/
//		int GetTrailInfo(double lookahead_distance, RoadLaneInfo *data);

		void CalcProbeTarget(Position *target, RoadProbeInfo *data);

		/**
		Move position along the road network, forward or backward, from the current position
		It will automatically follow connecting lanes between connected roads 
		If multiple options (only possible in junctions) it will choose randomly 
		@param ds distance to move from current position
		*/
		int MoveAlongS(double ds, double dLaneOffset = 0, Junction::JunctionStrategyType strategy = Junction::JunctionStrategyType::RANDOM);

		/**
		Retrieve the track/road ID from the position object
		@return track/road ID
		*/
		int GetTrackId() const { return track_id_; }

		/**
		Retrieve the lane ID from the position object
		@return lane ID
		*/
		int GetLaneId() const { return lane_id_; }

		/**
		Retrieve a road segment specified by road ID
		@param id road ID as specified in the OpenDRIVE file
		*/
		Road *GetRoadById(int id) { return GetOpenDrive()->GetRoadById(id);	}

		/**
		Retrieve the s value (distance along the road segment)
		*/
		double GetS() const { return s_; }

		/**
		Retrieve the t value (lateral distance from reference lanem (id=0))
		*/
		double GetT() const { return t_; }

		/**
		Retrieve the offset from current lane
		*/
		double GetOffset() const { return offset_; }

		/**
		Retrieve the world coordinate X-value
		*/
		double GetX() const { return x_; }

		/**
		Retrieve the world coordinate Y-value
		*/
		double GetY() const { return y_; }

		/**
		Retrieve the world coordinate Z-value
		*/
		double GetZ() const { return z_; }

		/**
		Retrieve the road Z-value 
		*/
		double GetZRoad() const { return z_road_; }

		/**
		Retrieve the world coordinate heading angle (radians)
		*/
		double GetH() const { return h_; }

		/**
		Retrieve the road heading angle (radians)
		*/
		double GetHRoad() const { return h_road_; }

		/**
		Retrieve the road heading angle (radians) relative driving direction (lane sign considered)
		*/
		double GetHRoadInDrivingDirection();

		/**
		Retrieve the heading angle (radians) relative driving direction (lane sign considered)
		*/
		double GetHRelativeDrivingDirection();

		/**
		Retrieve the relative heading angle (radians)
		*/
		double GetHRelative() const { return h_relative_; }

		/**
		Retrieve the world coordinate pitch angle (radians)
		*/
		double GetP() const { return p_; }

		/**
		Retrieve the road pitch value
		*/
		double GetPRoad() const { return p_road_; }

		/**
		Retrieve the world coordinate roll angle (radians)
		*/
		double GetR() const { return r_; }

		/**
		Retrieve the road curvature at current position
		*/
		double GetCurvature();

		/**
		Retrieve the speed limit at current position
		*/
		double GetSpeedLimit();

		/**
		Retrieve the road heading/direction at current position, and in the direction given by current lane
		*/
		double GetDrivingDirection();

		void CopyRMPos(Position *from);

		void PrintTrackPos();
		void PrintLanePos();
		void PrintInertialPos();

		void Print();
		void PrintXY();
	
	protected:
		void Track2Lane();
		void Track2XYZ();
		void Lane2Track();
		void XYZ2Track(bool alignZAndPitch = false);
		int SetLongitudinalTrackPos(int track_id, double s);
		bool EvaluateRoadZPitchRoll(bool alignZPitchRoll);
		double GetDistToTrackGeom(double x3, double y3, double z3, double h, Road *road, Geometry *geom, bool &inside, double &sNorm);

		// route reference
		Route  *route_;			// if pointer set, the position corresponds to a point along (s) the route

		// track reference
		int     track_id_;
		double  s_;				// longitudinal point/distance along the track
		double  t_;				// lateral position relative reference line (geometry)
		int     lane_id_;		// lane reference
		double  offset_;		// lateral position relative lane given by lane_id
		double  h_road_;		// heading of the road
		double  h_offset_;		// local heading offset given by lane width and offset 
		double  h_relative_;	// heading relative to the road (h_ = h_road_ + h_relative_)
		double  s_route_;		// longitudinal point/distance along the route
		double  curvature_;

		// inertial reference
		double	x_;
		double	y_;
		double	z_;
		double	h_;
		double	p_;
		double	r_;
		double	z_road_;
		double	p_road_;

		// keep track for fast incremental updates of the position
		int		track_idx_;		// road index 
		int		lane_idx_;		// road index 
		int		lane_section_idx_;	// lane section
		int		geometry_idx_;	// index of the segment within the track given by track_idx
		int		elevation_idx_;	// index of the current elevation entry 
	};


	// A route is a sequence of positions, at least one per road along the route
	class Route
	{
	public:
		explicit Route() {}

		/**
		Adds a waypoint to the route. One waypoint per road. At most one junction between waypoints.
		@param position A regular position created with road, lane or world coordinates
		@return Non zero return value indicates error of some kind
		*/
		int AddWaypoint(Position *position);
		int GetWayPointDirection(int index);

		void setName(std::string name);
		std::string getName();
		double GetLength();

		std::vector<Position*> waypoint_;
		std::string name;
	};

	// A Road Path is a linked list of road links (road connections or junctions)
	// between a starting position and a target position
	// The path can be calculated automatically 
	class RoadPath
	{
	public:

		typedef struct PathNode
		{
			RoadLink *link;
			double dist;
			Road* fromRoad;
			PathNode* previous;
		} PathNode;

		std::vector<PathNode*> visited_;
		std::vector<PathNode*> unvisited_;
		Position *startPos_;
		Position *targetPos_;

		RoadPath(Position* startPos, Position* targetPos) : startPos_(startPos), targetPos_(targetPos) {};
		~RoadPath();

		int Calculate(double &dist);
	
	private:
		bool CheckRoad(Road* checkRoad, RoadPath::PathNode* srcNode, Road* fromRoad);
	};


} // namespace

#endif // OPENDRIVE_HH_
