#ifndef OPENDRIVE_HH_
#define OPENDRIVE_HH_

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

		GeometryType GetType() { return type_; }
		double GetLength() { return length_; }
		double GetX() { return x_; }
		double GetY() { return y_; }
		double GetHdg() { return hdg_; }
		double GetS() { return s_; }
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

		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);
	};


	class Arc : public Geometry
	{
	public:
		Arc(double s, double x, double y, double hdg, double length, double curvature) :
			Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_ARC), curvature_(curvature) {}

		double GetCurvature() { return curvature_; }
		double GetRadius() { return fabs(1.0 / curvature_); }
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
			curv_start_(curv_start), curv_end_(curv_end), x0_(0.0), y0_(0.0), h0_(0.0), s0_(0.0), c_dot_(0.0) {}

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
		
		void SetUMax(double umax) { umax_ = umax; }
		double GetUMax() { return umax_; }
		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);

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

		double GetPRange() { return p_range_; }
		void Print();
		void EvaluateDS(double ds, double *x, double *y, double *h);

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

	enum LinkType
	{
		UNKNOWN,
		SUCCESSOR,
		PREDECESSOR,
		NONE
	};


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
		int GetLaneIdByIdx(int idx);
		int GetLaneIdxById(int id);
		double GetOuterOffset(double s, int lane_id);
		double GetCenterOffset(double s, int lane_id);
		double GetOuterOffsetHeading(double s, int lane_id);
		double GetCenterOffsetHeading(double s, int lane_id);
		double GetLength() { return length_; }
		int GetNumberOfLanes() { return (int)lane_.size(); }
		int GetNUmberOfLanesRight();
		int GetNUmberOfLanesLeft();
		void SetLength(double length) { length_ = length; }
		int GetConnectingLaneId(int incoming_lane_id, LinkType link_type);
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
		enum ElementType
		{
			ELEMENT_TYPE_UNKNOWN,
			ELEMENT_TYPE_ROAD,
			ELEMENT_TYPE_JUNCTION,
		};

		RoadLink() : type_(LinkType::NONE), element_id_(-1), element_type_(ELEMENT_TYPE_UNKNOWN), contact_point_type_(CONTACT_POINT_UNKNOWN) {}
		RoadLink(LinkType type, ElementType element_type, int element_id, ContactPointType contact_point_type) :
			element_type_(element_type), element_id_(element_id), contact_point_type_(contact_point_type), type_(type) {}
		RoadLink(LinkType type, pugi::xml_node node);

		int GetElementId() { return element_id_; }
		LinkType GetType() { return type_; }
		int GetElementType() { return element_type_; }
		int GetContactPointType() { return contact_point_type_; }

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

	class Road
	{
	public:
		Road(int id, std::string name) : id_(id), name_(name), length_(0) {}
		~Road();

		void Print();
		void SetI(int id) { id_ = id; }
		int GetId() { return id_; }
		void SetName(std::string name) { name_ = name; }
		Geometry *GetGeometry(int idx) { return geometry_[idx]; }
		int GetNumberOfGeometries() { return (int)geometry_.size(); }
		LaneSection *GetLaneSectionByIdx(int idx);
		LaneInfo GetLaneInfoByS(double s, int start_lane_link_idx, int start_lane_id);
		int GetNumberOfLaneSections() { return (int)lane_section_.size(); }
		std::string GetName() { return name_; }
		void SetLength(double length) { length_ = length; }
		double GetLength() { return length_; }
		void SetJunction(int junction) { junction_ = junction; }
		int GetJunction() { return junction_; }
		void AddLink(RoadLink *link) { link_.push_back(link); }
		RoadLink *GetLink(LinkType type);
		void AddLine(Line *line);
		void AddArc(Arc *arc);
		void AddSpiral(Spiral *spiral);
		void AddPoly3(Poly3 *poly3);
		void AddParamPoly3(ParamPoly3 *param_poly3);
		void AddElevation(Elevation *elevation);
		void AddLaneSection(LaneSection *lane_section);
		void AddLaneOffset(LaneOffset *lane_offset);
		Elevation *GetElevation(int idx) { return elevation_profile_[idx]; }
		int GetNumberOfElevations() { return (int)elevation_profile_.size(); }
		double GetLaneOffset(double s);
		double GetLaneOffsetPrim(double s);
		int GetNumberOfLanes(double s);

	protected:
		int id_;
		std::string name_;
		double length_;
		int junction_;
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
			lane_id_(0), connecting_road_id_(-1), connecting_lane_id_(0) {}
		LaneRoadLaneConnection(int lane_id, int connecting_road_id, int connecting_lane_id) :
			lane_id_(lane_id), connecting_road_id_(connecting_road_id), connecting_lane_id_(connecting_lane_id) {}
		void SetLane(int id) { lane_id_ = id; }
		void SetConnectingRoad(int id) { connecting_road_id_ = id; }
		void SetConnectingLane(int id) { connecting_lane_id_ = id; }
		int GetLaneId() { return lane_id_; }
		int GetConnectingRoadId() { return connecting_road_id_; }
		int GetConnectinglaneId() { return connecting_lane_id_; }
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
		~Connection() { for (auto &ll : lane_link_) delete ll; }
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
		Junction(int id, std::string name) : id_(id), name_(name) {}
		~Junction() { for (auto &c : connection_) delete c; }
		int GetId() { return id_; }
		std::string GetName() { return name_; }
		int GetNumberOfConnections() { return (int)connection_.size(); }
		void AddConnection(Connection *connection) { connection_.push_back(connection); }
		Connection *GetConnectionByIdx(int idx) { return connection_[idx]; }
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
		bool LoadOpenDriveFile(const char *filename);
		Road* GetRoadById(int id);
		Road* GetRoadByIdx(int idx);
		int GetTrackIdxById(int id);
		int GetTrackIdByIdx(int idx);
		int GetNumOfRoads() { return (int)road_.size(); }
		Junction* GetJunctionById(int id);
		Junction* GetJunctionByIdx(int idx);
		int GetNumOfJunctions() { return (int)junction_.size(); }
		int GetNumberOfJunctionConnections(Road *road, Lane *lane);
		LaneRoadLaneConnection GetJunctionConnection(Road *road, Lane *lane, int idx);

		void Print();
	
	private:
		pugi::xml_node root_node_;
		std::vector<Road*> road_;
		std::vector<Junction*> junction_;
	};

	class Position
	{
	public:
		explicit Position();
		explicit Position(int track_id, double s, double t);
		explicit Position(int track_id, int lane_id, double s, double offset);
		explicit Position(double x, double y, double z, double h, double p, double r);
		~Position();
		
		static bool LoadOpenDrive(const char *filename);
		static OpenDrive* GetOpenDrive();
		void Set(int track_id, double s, double t);
		void Set(int track_id, int lane_id, double s, double offset, int lane_section_idx = -1);
		void Set(double x, double y, double z, double h, double p, double r);
		void Set(double x, double y, double h);
		int MoveToConnectingRoad(RoadLink *road_link, double ds);
		int MoveAlongS(double ds);

		int GetTrackId() { return track_id_; }
		int GetLaneId() { return lane_id_; }
		double GetS() { return s_; }
		double GetT() { return t_; }
		double GetOffset() { return offset_; }
		double GetX() { return x_; }
		double GetY() { return y_; }
		double GetZ() { return z_; }
		double GetH() { return h_; }
		double GetP() { return p_; }
		double GetR() { return r_; }

		void PrintTrackPos();
		void PrintLanePos();
		void PrintInertialPos();

		void Print();
		void PrintXY();
	
	protected:
		void Track2Lane();
		void Track2XYZ();
		void Lane2Track();
		void XYZ2Track();
		void SetLongitudinalTrackPos(int track_id, double s);
		bool Position::EvaluateZAndPitch();


		// track reference
		int     track_id_;
		double  s_;				// longitudinal point/distance along the track
		double  t_;				// lateral position relative reference line (geometry)
		int     lane_id_;		// lane reference
		double  offset_;		// lateral position relative lane given by lane_id
		double  h_offset_;		// local heading offset given by lane width and offset

		// inertial reference
		double	x_;
		double	y_;
		double	z_;
		double	h_;
		double	p_;
		double	r_;

		// keep track for fast incremental updates of the position
		int		track_idx_;		// road index 
		int		lane_idx_;		// road index 
		int		lane_section_idx_;	// lane section
		int		geometry_idx_;	// index of the segment within the track given by track_idx
		int		elevation_idx_;	// index of the current elevation entry 
	};

} // namespace

#endif // OPENDRIVE_HH_
