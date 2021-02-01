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
#include "CommonMini.hpp"


namespace roadmanager
{
	class Polynomial
	{
	public:
		Polynomial() : a_(0), b_(0), c_(0), d_(0), p_scale_(1.0) {}
		Polynomial(double a, double b, double c, double d, double p_scale = 1) : a_(a), b_(b), c_(c), d_(d), p_scale_(p_scale) {}
		void Set(double a, double b, double c, double d, double p_scale = 1);
		void SetA(double a) { a_ = a; }
		void SetB(double b) { b_ = b; }
		void SetC(double c) { c_ = c; }
		void SetD(double d) { d_ = d; }
		double GetA() { return a_; }
		double GetB() { return b_; }
		double GetC() { return c_; }
		double GetD() { return d_; }
		double GetPscale() { return p_scale_; }
		double Evaluate(double s);
		double EvaluatePrim(double s);
		double EvaluatePrimPrim(double s);

	private:
		double a_;
		double b_;
		double c_;
		double d_;
		double p_scale_;
	};

	class OSIPoints
	{
		public:
			typedef struct
			{
				double s;
				double x;
				double y;
				double z;
				double h;
			} OSIPointStruct;

			OSIPoints() {}
			OSIPoints(std::vector<OSIPointStruct> points) : point_(points) {}
			void Set(std::vector<OSIPointStruct> points) { point_ = points; }
			std::vector<OSIPointStruct>& GetPoints() {return point_;}
			OSIPointStruct& GetPoint(int i);
			double GetXfromIdx(int i);
			double GetYfromIdx(int i);
			double GetZfromIdx(int i);
			int GetNumOfOSIPoints();

		private:
			std::vector<OSIPointStruct> point_;
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

		Geometry() : s_(0.0), x_(0.0), y_(0), hdg_(0), length_(0), type_(GeometryType::GEOMETRY_TYPE_UNKNOWN) {}
		Geometry(double s, double x, double y, double hdg, double length, GeometryType type) :
			s_(s), x_(x), y_(y), hdg_(hdg), length_(length), type_(type) {}
		virtual ~Geometry() {}

		GeometryType GetType() { return type_; }
		double GetLength() { return length_; }
		virtual double GetX() { return x_; }
		void SetX(double x) { x_ = x; }
		virtual double GetY() { return y_; }
		void SetY(double y) { y_ = y; }
		virtual double GetHdg() { return GetAngleInInterval2PI(hdg_); }
		void SetHdg(double hdg) { hdg_ = hdg; }
		double GetS() { return s_; }
		virtual double EvaluateCurvatureDS(double ds) = 0;
		virtual void Print();
		virtual void EvaluateDS(double ds, double *x, double *y, double *h);

	protected:
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
		Line() {}
		Line(double s, double x, double y, double hdg, double length) : Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_LINE) {}
		~Line() {};

		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);
		double EvaluateCurvatureDS(double ds) { (void)ds; return 0; }

	};


	class Arc : public Geometry
	{
	public:
		Arc(): curvature_(0.0) {}
		Arc(double s, double x, double y, double hdg, double length, double curvature) :
			Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_ARC), curvature_(curvature) {}
		~Arc() {}

		double EvaluateCurvatureDS(double ds) { (void)ds; return curvature_; }
		double GetRadius() { return std::fabs(1.0 / curvature_); }
		double GetCurvature() { return curvature_; }
		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);

	private:
		double curvature_;
	};


	class Spiral : public Geometry
	{
	public:
		Spiral(): curv_start_(0.0), curv_end_(0.0), c_dot_(0.0), x0_(0.0), y0_(0.0), h0_(0.0), s0_(0.0), arc_(0), line_(0) {}
		Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end);
			
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
		void SetX(double x);
		void SetY(double y);
		void SetHdg(double h);

		Arc* arc_;
		Line* line_;

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
		Poly3(): umax_(0.0) {}
		Poly3(double s, double x, double y, double hdg, double length, double a, double b, double c, double d);
		~Poly3() {};
		
		void SetUMax(double umax) { umax_ = umax; }
		double GetUMax() { return umax_; }
		void Print();
		Polynomial GetPoly3() {return poly3_;}
		void EvaluateDS(double ds, double *x, double *y, double *h);
		double EvaluateCurvatureDS(double ds);

		Polynomial poly3_;

	private:
		double umax_;
		void EvaluateDSLocal(double ds, double& u, double& v);
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

		ParamPoly3() {}
		ParamPoly3(
			double s, double x, double y, double hdg, double length,
			double aU, double bU, double cU, double dU, double aV, double bV, double cV, double dV, PRangeType p_range) :
			Geometry(s, x, y, hdg, length, GeometryType::GEOMETRY_TYPE_PARAM_POLY3)
		{
			poly3U_.Set(aU, bU, cU, dU, p_range == PRangeType::P_RANGE_NORMALIZED ? 1.0/length : 1.0);
			poly3V_.Set(aV, bV, cV, dV, p_range == PRangeType::P_RANGE_NORMALIZED ? 1.0/length : 1.0);
		}
		~ParamPoly3() {};

		void Print();
		Polynomial GetPoly3U() {return poly3U_;}
		Polynomial GetPoly3V() {return poly3V_;}
		void EvaluateDS(double ds, double *x, double *y, double *h);
		double EvaluateCurvatureDS(double ds);
		
		Polynomial poly3U_;
		Polynomial poly3V_;
	};


	class Elevation
	{
	public:
		Elevation(): s_(0.0), length_(0.0) {}
		Elevation(double s, double a, double b, double c, double d) : s_(s), length_(0)
		{
			poly3_.Set(a, b, c, d);
		}
		~Elevation() {};

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

	class LaneBoundaryOSI
	{
	public:
		LaneBoundaryOSI(int gbid): global_id_(gbid) {}
		~LaneBoundaryOSI() {};
		void SetGlobalId();
		int GetGlobalId() { return global_id_; }
		OSIPoints* GetOSIPoints() {return &osi_points_;}
		OSIPoints osi_points_;
	private:
		int global_id_;  // Unique ID for OSI 
	};

	struct RoadMarkInfo
	{
		int roadmark_idx_;
		int roadmarkline_idx_;
	};

	class LaneRoadMarkTypeLine
	{
	public:
		enum RoadMarkTypeLineRule
		{
			NO_PASSING,
			CAUTION,
			NONE
		};

		LaneRoadMarkTypeLine(double length, double space, double t_offset, double s_offset, RoadMarkTypeLineRule rule, double width): 
		length_(length), space_(space), t_offset_(t_offset), s_offset_(s_offset), rule_(rule), width_(width) {}
		~LaneRoadMarkTypeLine() {}; 
		double GetSOffset() { return s_offset_; }
		double GetTOffset() { return t_offset_; }
		double GetLength() {return length_;}
		double GetSpace() {return space_;}
		double GetWidth() {return width_;}
		OSIPoints GetOSIPoints() {return osi_points_;}
		OSIPoints osi_points_;
		void SetGlobalId();
		int GetGlobalId() { return global_id_; }

	private:
		double length_;
		double space_;
		double t_offset_;
		double s_offset_;
		RoadMarkTypeLineRule rule_;
		double width_;
		int global_id_;  // Unique ID for OSI
	};

	class LaneRoadMarkType
	{
	public:
		LaneRoadMarkType(std::string name, double width) : name_(name), width_(width) {}
		
		void AddLine(LaneRoadMarkTypeLine *lane_roadMarkTypeLine); 
		std::string GetName() { return name_; }
		double GetWidth() { return width_; }
		LaneRoadMarkTypeLine* GetLaneRoadMarkTypeLineByIdx(int idx);
		int GetNumberOfRoadMarkTypeLines() { return (int)lane_roadMarkTypeLine_.size(); }
		void Print();

	private:
		std::string name_;
		double width_;
		std::vector<LaneRoadMarkTypeLine*> lane_roadMarkTypeLine_;
	};

	class LaneRoadMark
	{
	public:
		enum RoadMarkType
		{
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

		enum RoadMarkWeight
		{
			STANDARD,
			BOLD
		};

		enum RoadMarkColor
		{
			STANDARD_COLOR, // equivalent to white
			BLUE,
			GREEN,
			RED,
			WHITE,
			YELLOW
		};

		enum RoadMarkMaterial
		{
			STANDARD_MATERIAL // only "standard" is available for now
		};

		enum RoadMarkLaneChange
		{
			INCREASE,
			DECREASE,
			BOTH,
			NONE_LANECHANGE
		};

		LaneRoadMark(double s_offset, RoadMarkType type, RoadMarkWeight weight, RoadMarkColor color, 
		RoadMarkMaterial material, RoadMarkLaneChange lane_change, double width, double height): 
		s_offset_(s_offset), type_(type), weight_(weight), color_(color), material_(material), lane_change_(lane_change), 
		width_(width), height_(height) {}

		void AddType(LaneRoadMarkType *lane_roadMarkType) { lane_roadMarkType_.push_back(lane_roadMarkType); }

		double GetSOffset() { return s_offset_; }
		double GetWidth() { return width_; }
		double GetHeight() { return height_; }
		RoadMarkType GetType() { return type_; }
		RoadMarkWeight GetWeight() { return weight_; }
		RoadMarkColor GetColor() { return color_; }
		RoadMarkMaterial GetMaterial() { return material_; }
		RoadMarkLaneChange GetLaneChange() { return lane_change_; }

		int GetNumberOfRoadMarkTypes() { return (int)lane_roadMarkType_.size(); }
		LaneRoadMarkType* GetLaneRoadMarkTypeByIdx(int idx);

	private:
		double s_offset_;
		RoadMarkType type_;
		RoadMarkWeight weight_;
		RoadMarkColor color_;
		RoadMarkMaterial material_;
		RoadMarkLaneChange lane_change_;
		double width_;
		double height_;
		std::vector<LaneRoadMarkType*> lane_roadMarkType_;
	};

	class LaneOffset
	{
	public:
		LaneOffset(): s_(0.0), length_(0.0) {}
		LaneOffset(double s, double a, double b, double c, double d) : s_(s), length_(0.0) 
		{ 
			polynomial_.Set(a, b, c, d); 
		}
		~LaneOffset() {}

		void Set(double s, double a, double b, double c, double d) 
		{
			s_ = s;
			polynomial_.Set(a, b, c, d); 
		}
		void SetLength(double length) { length_ = length; }
		double GetS() { return s_; }
		Polynomial GetPolynomial() { return polynomial_; }
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

		typedef enum 
		{
			LANE_TYPE_NONE =          (1 << 0),
			LANE_TYPE_DRIVING =       (1 << 1),
			LANE_TYPE_STOP =          (1 << 2),
			LANE_TYPE_SHOULDER =      (1 << 3),
			LANE_TYPE_BIKING =        (1 << 4),
			LANE_TYPE_SIDEWALK =      (1 << 5),
			LANE_TYPE_BORDER =        (1 << 6),
			LANE_TYPE_RESTRICTED =    (1 << 7),
			LANE_TYPE_PARKING =       (1 << 8),
			LANE_TYPE_BIDIRECTIONAL = (1 << 9),
			LANE_TYPE_MEDIAN =        (1 << 10),
			LANE_TYPE_SPECIAL1 =      (1 << 11),
			LANE_TYPE_SPECIAL2 =      (1 << 12),
			LANE_TYPE_SPECIAL3 =      (1 << 13),
			LANE_TYPE_ROADMARKS =     (1 << 14),
			LANE_TYPE_TRAM =          (1 << 15),
			LANE_TYPE_RAIL =          (1 << 16),
			LANE_TYPE_ENTRY =         (1 << 17),
			LANE_TYPE_EXIT =          (1 << 18),
			LANE_TYPE_OFF_RAMP =      (1 << 19),
			LANE_TYPE_ON_RAMP =       (1 << 20),
			LANE_TYPE_ANY_DRIVING =   LANE_TYPE_DRIVING |
			                          LANE_TYPE_ENTRY | 
			                          LANE_TYPE_EXIT | 
			                          LANE_TYPE_OFF_RAMP | 
			                          LANE_TYPE_ON_RAMP | 
			                          LANE_TYPE_PARKING,
			LANE_TYPE_ANY_ROAD =      LANE_TYPE_ANY_DRIVING |
			                          LANE_TYPE_RESTRICTED |
			                          LANE_TYPE_STOP,
			LANE_TYPE_ANY =           (0xFFFFFFFF)
		} LaneType;

		// Construct & Destruct
		Lane() : id_(0), type_(LaneType::LANE_TYPE_NONE), level_(0), offset_from_ref_(0.0), global_id_(0), lane_boundary_(0) {}
		Lane(int id, Lane::LaneType type) : id_(id), type_(type), level_(1), offset_from_ref_(0), global_id_(0), lane_boundary_(0) {}
		~Lane() {}

		// Base Get Functions
		int GetId() { return id_; }
		double GetOffsetFromRef() { return offset_from_ref_; }
		LaneType GetLaneType() { return type_; }
		int GetGlobalId() { return global_id_; }

		// Add Functions
		void AddLink(LaneLink *lane_link) { link_.push_back(lane_link); }
		void AddLaneWidth(LaneWidth *lane_width) { lane_width_.push_back(lane_width); }
		void AddLaneRoadMark(LaneRoadMark *lane_roadMark) { lane_roadMark_.push_back(lane_roadMark); }
		
		// Get Functions
		int GetNumberOfRoadMarks() { return (int)lane_roadMark_.size(); }
		int GetNumberOfLinks() { return (int)link_.size(); }
		int GetNumberOfLaneWidths() { return (int)lane_width_.size(); }

		LaneLink *GetLink(LinkType type);
		LaneWidth *GetWidthByIndex(int index);
		LaneWidth *GetWidthByS(double s);
		LaneRoadMark* GetLaneRoadMarkByIdx(int idx);

		RoadMarkInfo GetRoadMarkInfoByS(int track_id, int lane_id, double s);
		OSIPoints* GetOSIPoints() { return &osi_points_;}
		std::vector<int> GetLineGlobalIds(); 
		LaneBoundaryOSI* GetLaneBoundary() {return lane_boundary_; }
		int GetLaneBoundaryGlobalId();

		// Set Functions
		void SetGlobalId();
		void SetLaneBoundary(LaneBoundaryOSI *lane_boundary);
		void SetOffsetFromRef(double offset) { offset_from_ref_ = offset; }

		// Others
		bool IsType(Lane::LaneType type);
		bool IsCenter();
		bool IsDriving();
		void Print();
		OSIPoints osi_points_;

	private:
		int id_;		// center = 0, left > 0, right < 0
		int global_id_;  // Unique ID for OSI 
		LaneType type_;
		int level_;	// boolean, true = keep lane on level
		double offset_from_ref_;
		std::vector<LaneLink*> link_;
		std::vector<LaneWidth*> lane_width_;
		std::vector<LaneRoadMark*> lane_roadMark_;
		LaneBoundaryOSI* lane_boundary_;
	};

	class LaneSection
	{
	public:
		LaneSection(double s) : s_(s), length_(0) {}
		void AddLane(Lane *lane);
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
		int GetClosestLaneIdx(double s, double t, double &offset, bool noZeroWidth, int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING);
		
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

	class Signal
	{
	public:
		enum Orientation
		{
			POSITIVE,
			NEGATIVE,
			NONE,
		};

		enum Type
		{
			NONETYPE,
			T1000001, // traditional red-yellow-green light
			T1000002, // 2 subtypes: pedestrian light (red or green & only red)
			T1000007, // 4 subtypes: pedestrian + cyclists light (red or green & only red & only yellow & only green)
			T1000008, // 3 subtypes: yellow light (no arrow & left arrow & right arrow)
			T1000009, // 2 subtypes: red-yellow light & solid yellow or green light
			T1000010, // 2 subtypes: yellow-green light (only left arrows & only right arrows)
			T1000011, // 5 subtypes: red-yellow-green light (only left arrows & only right arrows & only straight arrows & straight+left arrows & straight+right arrows)
			T1000012, // 2 subtypes: green light (left arrow & right arrow)
			T1000013, // red-green cyclist light
			T1000014, // yellow tram light
			T1000015  // yellow pedestrian light
		};

		enum SubType
		{
			NONESUBTYPE,
			SUBT10,
			SUBT20,
			SUBT30,
			SUBT40,
			SUBT50
		};

		Signal(double s, double t, int id, std::string name, bool dynamic, Orientation orientation, double z_offset, std::string country,
		Type type, SubType sub_type, double value, std::string unit, double height, double width, std::string text, double h_offset,
		double pitch, double roll) : s_(s), t_(t), id_(id), name_(name), dynamic_(dynamic), orientation_(orientation), z_offset_(z_offset), 
		country_(country), type_(type), sub_type_(sub_type), value_(value), unit_(unit), height_(height), width_(width), text_(text),
		h_offset_(h_offset), pitch_(pitch), roll_(roll), length_(0) {}
				
		std::string GetName() { return name_; }
		double GetS() { return s_; }
		double GetT() { return t_; }
		void SetLength(double length) { length_ = length; }
		double GetLength() { return length_; }

	private:
		double s_;
		double t_;
		int id_;
		std::string name_;
		bool dynamic_;
		Orientation orientation_;
		double z_offset_;
		std::string country_;
		Type type_;
		SubType sub_type_;
		double value_;
		std::string unit_;
		double height_;
		double width_;
		std::string text_;
		double h_offset_;
		double pitch_;
		double roll_;
		double length_;
	};

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

		LaneInfo GetLaneInfoByS(double s, int start_lane_link_idx, int start_lane_id, int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING);
		double GetLaneWidthByS(double s, int lane_id);
		double GetSpeedByS(double s);
		bool GetZAndPitchByS(double s, double *z, double *pitch, int *index);
		bool UpdateZAndRollBySAndT(double s, double t, double *z, double *roll, int *index);
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
		void AddSuperElevation(Elevation *super_elevation);
		void AddLaneSection(LaneSection *lane_section);
		void AddLaneOffset(LaneOffset *lane_offset);
		void AddSignal(Signal *signal);
		Elevation *GetElevation(int idx);
		Elevation *GetSuperElevation(int idx);
		Signal *GetSignal(int idx);
		int GetNumberOfElevations() { return (int)elevation_profile_.size(); }
		int GetNumberOfSuperElevations() { return (int)super_elevation_profile_.size(); }
		double GetLaneOffset(double s);
		double GetLaneOffsetPrim(double s);
		int GetNumberOfLanes(double s);
		int GetNumberOfDrivingLanes(double s);
		Lane* GetDrivingLaneByIdx(double s, int idx);
		int GetNumberOfDrivingLanesSide(double s, int side);  // side = -1 right, 1 left

		/// <summary>Get width of road</summary>
		/// <param name="s">Longitudinal position/distance along the road</param>
		/// <param name="side">Side of the road: -1=right, 1=left, 0=both</param>
		/// <param name="laneTypeMask">Bitmask specifying what lane types to consider - see Lane::LaneType</param>
		/// <returns>Width (m)</returns>
		double GetWidth(double s, int side, int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY);   // side: -1=right, 1=left, 0=both

	protected:
		int id_;
		std::string name_;
		double length_;
		int junction_;
		std::vector<RoadTypeEntry*> type_;
		std::vector<RoadLink*> link_;
		std::vector<Geometry*> geometry_;
		std::vector<Elevation*> elevation_profile_;
		std::vector<Elevation*> super_elevation_profile_;
		std::vector<LaneSection*> lane_section_;
		std::vector<LaneOffset*> lane_offset_;
		std::vector<Signal*> signal_;
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
		LaneRoadLaneConnection GetRoadConnectionByIdx(int roadId, int laneId, int idx, 
			int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING);
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
		Initialize the global ids for lanes
		*/ 
		void InitGlobalLaneIds();

		/**
		Get the filename of currently loaded OpenDRIVE file
		*/
		std::string GetOpenDriveFilename() { return odr_filename_; }

		/**
		Setting information based on the OSI standards for OpenDrive elements
		*/
		bool SetRoadOSI();
		bool CheckLaneOSIRequirement(std::vector<double> x0, std::vector<double> y0, std::vector<double> x1, std::vector<double> y1);
		void SetLaneOSIPoints();
		void SetRoadMarkOSIPoints();
		/**
		Checks all lanes - if a lane has RoadMarks it does nothing. If a lane does not have roadmarks 
		then it creates a LaneBoundary following the lane border (left border for left lanes, right border for right lanes)
		*/
		void SetLaneBoundaryPoints();
		
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
		int CheckConnectedRoad(Road *road, RoadLink *link, ContactPointType expected_contact_point_type, RoadLink *link2);
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

	// Forward declarations
	class Route;
	class Trajectory;

	class Position
	{
	public:

		enum PositionType
		{
			NORMAL,
			ROUTE,
			RELATIVE_OBJECT,
			RELATIVE_WORLD,
			RELATIVE_LANE,
			RELATIVE_ROAD
		};

		enum OrientationType
		{
			ORIENTATION_RELATIVE,
			ORIENTATION_ABSOLUTE
		};

		enum LookAheadMode
		{
			LOOKAHEADMODE_AT_LANE_CENTER,
			LOOKAHEADMODE_AT_ROAD_CENTER,
			LOOKAHEADMODE_AT_CURRENT_LATERAL_OFFSET,
		};

		enum ErrorCode
		{
			ERROR_NO_ERROR = 0,
			ERROR_GENERIC = -1,
			ERROR_END_OF_ROAD = -2,
			ERROR_END_OF_ROUTE = -3,
			ERROR_OFF_ROAD = -4,
		};

		enum UpdateTrackPosMode
		{
			UPDATE_NOT_XYZH,
			UPDATE_XYZ,
			UPDATE_XYZH
		};

		typedef enum
		{
			POS_STATUS_END_OF_ROAD = (1 << 0),
			POS_STATUS_END_OF_ROUTE = (1 << 1)
		} POSITION_STATUS_MODES;

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

		/**
		Specify position by track coordinate (road_id, s, t)
		@param track_id Id of the road (track)
		@param s Distance to the position along and from the start of the road (track)
		@param updateMode UPDATE_NOT_XYZH: no update of x, y, z, h UPDATE_XYZ: recalculate x, y, z UPDATE_XYZH relaculate x, y, z and align h
		@return Non zero return value indicates error of some kind
		*/
		int SetTrackPos(int track_id, double s, double t, UpdateTrackPosMode updateMode = UpdateTrackPosMode::UPDATE_XYZH);
		void ForceLaneId(int lane_id);
		int SetLanePos(int track_id, int lane_id, double s, double offset, int lane_section_idx = -1);
		void SetLaneBoundaryPos(int track_id, int lane_id, double s, double offset, int lane_section_idx = -1);
		void SetRoadMarkPos(int track_id, int lane_id, int roadmark_idx, int roadmarktype_idx, int roadmarkline_idx, double s, double offset, int lane_section_idx = -1);

		/**
		Specify position by cartesian x, y, z and heading, pitch, roll
		@param x x
		@param y y
		@param z z
		@param h heading
		@param p pitch
		@param r roll
		@param updateTrackPos True: road position will be calculated False: don't update road position
		@return Non zero return value indicates error of some kind
		*/
		int SetInertiaPos(double x, double y, double z, double h, double p, double r, bool updateTrackPos = true);

		/**
		Specify position by cartesian x, y and heading. z, pitch and roll will be aligned to road.
		@param x x
		@param y y
		@param h heading
		@param updateTrackPos True: road position will be calculated False: don't update road position
		@return Non zero return value indicates error of some kind
		*/
		int SetInertiaPos(double x, double y, double h, bool updateTrackPos = true);
		void SetHeading(double heading);
		void SetHeadingRelative(double heading);
		void SetHeadingRelativeRoadDirection(double heading);
		void SetRoll(double roll);
		void SetRollRelative(double roll);
		void SetPitch(double roll);
		void SetPitchRelative(double pitch);

		/**
		Specify position by cartesian coordinate (x, y, z, h)
		@param x X-coordinate
		@param y Y-coordinate
		@param z Z-coordinate
		@param alignZAndPitch Align Z-coordinate to road elevation and Heading to tangent of the road 
		@return Non zero return value indicates error of some kind
		*/
		int XYZH2TrackPos(double x, double y, double z, double h, bool alignZAndPitch = true);
		
		int MoveToConnectingRoad(RoadLink *road_link, ContactPointType &contact_point_type, Junction::JunctionStrategyType strategy = Junction::RANDOM);
		
		void SetRelativePosition(Position* rel_pos, PositionType type)
		{
			rel_pos_ = rel_pos;
			type_ = type;
		}

		Position* GetRelativePosition() { return rel_pos_; }

		void ReleaseRelation();

		void SetRoute(Route *route);
		void CalcRoutePosition();
		const roadmanager::Route* GetRoute() const { return route_; }
		Route* GetRoute() { return route_; }
		Trajectory* GetTrajectory() { return trajectory_; }

		void SetTrajectory(Trajectory* trajectory);

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
		@return Non zero return value indicates error of some kind, most likely End Of Route
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
		@return Non zero return value indicates error of some kind, most likely End Of Route
		*/
		int SetRouteS(Route* route, double route_s);

		/**
		Move current position forward, or backwards, ds meters along the trajectory
		@param ds Distance to move, negative will move backwards
		@return Non zero return value indicates error of some kind
		*/
		int MoveTrajectoryDS(double ds);

		/**
		Move current position to specified S-value along the trajectory
		@param trajectory_s Distance to move, negative will move backwards
		@return Non zero return value indicates error of some kind
		*/
		int SetTrajectoryS(Trajectory* trajectory, double trajectory_s);

		int SetTrajectoryPosByTime(Trajectory* trajectory, double time);

		/**
		Retrieve the S-value of the current route position. Note: This is the S along the
		complete route, not the actual individual roads.
		*/
		double GetTrajectoryS() { return s_trajectory_; }

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
		It can be used to calculate the distance from current position to another one (pos_b)
		@param pos_b The position from which to subtract the current position (this position object)
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
		int GetRoadLaneInfo(RoadLaneInfo *data);

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
		int GetTrackId();

		/**
		Retrieve the lane ID from the position object
		@return lane ID
		*/
		int GetLaneId();
		/**
		Retrieve the global lane ID from the position object
		@return lane ID
		*/
		int GetLaneGlobalId();

		/**
		Retrieve a road segment specified by road ID
		@param id road ID as specified in the OpenDRIVE file
		*/
		Road *GetRoadById(int id) { return GetOpenDrive()->GetRoadById(id);	}

		/**
		Retrieve the s value (distance along the road segment)
		*/
		double GetS();

		/**
		Retrieve the t value (lateral distance from reference lanem (id=0))
		*/
		double GetT();

		/**
		Retrieve the offset from current lane
		*/
		double GetOffset();

		/**
		Retrieve the world coordinate X-value
		*/
		double GetX();

		/**
		Retrieve the world coordinate Y-value
		*/
		double GetY();

		/**
		Retrieve the world coordinate Z-value
		*/
		double GetZ();

		/**
		Retrieve the road Z-value 
		*/
		double GetZRoad() const { return z_road_; }

		/**
		Retrieve the world coordinate heading angle (radians)
		*/
		double GetH();

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
		double GetHRelative();

		/**
		Retrieve the world coordinate pitch angle (radians)
		*/
		double GetP();

		/**
		Retrieve the road pitch value
		*/
		double GetPRoad() const { return p_road_; }

		/**
		Retrieve the relative pitch angle (radians)
		*/
		double GetPRelative();

		/**
		Retrieve the world coordinate roll angle (radians)
		*/
		double GetR();

		/**
		Retrieve the road roll value
		*/
		double GetRRoad() const { return r_road_; }

		/**
		Retrieve the relative roll angle (radians)
		*/
		double GetRRelative();

		/**
		Retrieve the road pitch value, driving direction considered
		*/
		double GetPRoadInDrivingDirection();


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

		PositionType GetType() { return type_; }

		void SetTrackId(int trackId) { track_id_ = trackId; }
		void SetLaneId(int laneId) { lane_id_ = laneId; }
		void SetS(double s) { s_ = s; }
		void SetOffset(double offset) { offset_ = offset; }
		void SetT(double t) { t_ = t; }
		void SetX(double x) { x_ = x; }
		void SetY(double y) { y_ = y; }
		void SetZ(double z) { z_ = z; }
		void SetH(double h) { h_ = h; }
		void SetHRate(double h_rate) { h_rate_ = h_rate; }
		void SetHAcc(double h_acc) { h_acc_ = h_acc; }
		void SetP(double p) { p_ = p; }
		void SetR(double r) { r_ = r; }
		void SetVelX(double x) { velX_ = x; }
		void SetVelY(double y) { velY_ = y; }
		void SetVelZ(double z) { velZ_ = z; }
		void SetAccX(double x) { accX_ = x; }
		void SetAccY(double y) { accY_ = y; }
		void SetAccZ(double z) { accZ_ = z; }
		double GetVelX() { return velX_; }
		double GetVelY() { return velY_; }
		double GetVelZ() { return velZ_; }
		double GetAccX() { return accX_; }
		double GetAccY() { return accY_; }
		double GetAccZ() { return accZ_; }
		double GetHRate() { return h_rate_; }
		double GetHAcc() { return h_acc_; }

		int GetStatusBitMask() { return status_; }

		void SetOrientationType(OrientationType type) { orientation_type_ = type; }

		/**
		Specify which lane types the position object snaps to (is aware of)
		@param laneTypes A combination (bitmask) of lane types
		@return -
		*/
		void SetSnapLaneTypes(int laneTypeMask) { snapToLaneTypes_ = laneTypeMask; }

		void CopyRMPos(Position *from);

		void PrintTrackPos();
		void PrintLanePos();
		void PrintInertialPos();

		void Print();
		void PrintXY();

		bool IsOffRoad();

		void ReplaceObjectRefs(Position* pos1, Position* pos2)
		{
			if (rel_pos_ == pos1)
			{
				rel_pos_ = pos2;
			}
		}

	protected:
		void Track2Lane();
		int Track2XYZ(bool alignH = true);
		void Lane2Track();
		void RoadMark2Track();
		/**
		Set position to the border of lane (right border for right lanes, left border for left lanes)
		*/
		void LaneBoundary2Track();
		void XYZ2Track(bool alignZAndPitch = false);
		int SetLongitudinalTrackPos(int track_id, double s);
		bool EvaluateRoadZPitchRoll(bool alignZPitchRoll);

		// route reference
		Route  *route_;			// if pointer set, the position corresponds to a point along (s) the route

		// route reference
		Trajectory* trajectory_; // if pointer set, the position corresponds to a point along (s) the trajectory

		// track reference
		int     track_id_;
		double  s_;					// longitudinal point/distance along the track
		double  t_;					// lateral position relative reference line (geometry)
		int     lane_id_;			// lane reference
		double  offset_;			// lateral position relative lane given by lane_id
		double  h_road_;			// heading of the road
		double  h_offset_;			// local heading offset given by lane width and offset 
		double  h_relative_;		// heading relative to the road (h_ = h_road_ + h_relative_)
		double  s_route_;			// longitudinal point/distance along the route
		double  s_trajectory_;		// longitudinal point/distance along the trajectory
		double  curvature_;
		double  p_relative_;		// pitch relative to the road (h_ = h_road_ + h_relative_)
		double  r_relative_;		// roll relative to the road (h_ = h_road_ + h_relative_)
		Position* rel_pos_;
		PositionType type_;
		OrientationType orientation_type_;  // Applicable for relative positions
		int snapToLaneTypes_;  // Bitmask of lane types that the position will snap to
		int status_;           // Bitmask of various states, e.g. off_road, end_of_road

		// inertial reference
		double	x_;
		double	y_;
		double	z_;
		double	h_;
		double  h_rate_;
		double  h_acc_;
		double	p_;
		double	r_;
		double	velX_;
		double	velY_;
		double	velZ_;
		double	accX_;
		double	accY_;
		double	accZ_;
		double	z_road_;
		double	p_road_;
		double	r_road_;

		// keep track for fast incremental updates of the position
		int		track_idx_;				// road index 
		int		lane_idx_;				// lane index 
		int 	roadmark_idx_;  		// laneroadmark index
		int 	roadmarktype_idx_;  		// laneroadmark index
		int 	roadmarkline_idx_;  	// laneroadmarkline index
		int		lane_section_idx_;		// lane section
		int		geometry_idx_;			// index of the segment within the track given by track_idx
		int		elevation_idx_;			// index of the current elevation entry 
		int		super_elevation_idx_;   // index of the current super elevation entry 
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
		std::string name_;
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
		int direction_;  // direction of path from starting pos. 0==not set, 1==forward, 2==backward

		RoadPath(Position* startPos, Position* targetPos) : startPos_(startPos), targetPos_(targetPos) {};
		~RoadPath();

		/**
		Calculate shortest path between starting position and target position, 
		using Dijkstra's algorithm https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
		it also calculates the length of the path, or distance between the positions
		positive distance means that the shortest path was found in forward direction
		negative distance means that the shortest path goes in opposite direction from the heading of the starting position
		@param dist A reference parameter into which the calculated path distance is stored
		@return 0 on success, -1 on failure e.g. path not found
		*/
		int Calculate(double &dist);
	
	private:
		bool CheckRoad(Road* checkRoad, RoadPath::PathNode* srcNode, Road* fromRoad);
	};


	// Trajectory stuff
	class Shape
	{
	public:
		typedef enum
		{
			POLYLINE,
			CLOTHOID,
			NURBS,
			SHAPE_TYPE_UNDEFINED
		} ShapeType;

		Shape(ShapeType type) : type_(type), length_(0) {}

		ShapeType type_;
		double length_;
	};

	class PolyLine : public Shape
	{
	public:

		class Vertex
		{
		public:
			Position pos_;
			double time_;
			bool calc_heading_;
		};

		PolyLine() : Shape(ShapeType::POLYLINE) {}
		void AddVertex(Position pos, double time, bool calculateHeading);

		std::vector<Vertex*> vertex_;
	};

	class Clothoid : public Shape
	{
	public:

		Clothoid(roadmanager::Position pos, double curv, double curvDot, double len, double tStart, double tEnd) : Shape(ShapeType::CLOTHOID)
		{
			pos_ = pos;
			spiral_ = new roadmanager::Spiral(0, pos_.GetX(), pos_.GetY(), pos_.GetH(), len, curv, curv + curvDot * len);
			length_ = len;
			t_start_ = tStart;
			t_end_ = tEnd;
		}

		Position pos_;
		roadmanager::Spiral* spiral_;  // make use of the OpenDRIVE clothoid definition
		double t_start_;
		double t_end_;
	};

	class Nurbs : public Shape
	{
	public:
		Nurbs() : Shape(ShapeType::NURBS) {}
	};

	class Trajectory
	{
	public:
		Shape* shape_;

		Trajectory(Shape* shape, std::string name, bool closed) : shape_(shape), name_(name), closed_(closed) {}
		Trajectory() : shape_(0), closed_(false) {}
		void Freeze();

		std::string name_;
		bool closed_;
	};


} // namespace

#endif // OPENDRIVE_HH_
