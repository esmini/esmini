#ifndef POSITION_HPP
#define POSITION_HPP
class Position {
   public:
	enum class PositionType { NORMAL, ROUTE, RELATIVE_OBJECT, RELATIVE_WORLD, RELATIVE_LANE, RELATIVE_ROAD };

	enum class OrientationType { ORIENTATION_RELATIVE, ORIENTATION_ABSOLUTE };

	enum class LookAheadMode {
		LOOKAHEADMODE_AT_LANE_CENTER,
		LOOKAHEADMODE_AT_ROAD_CENTER,
		LOOKAHEADMODE_AT_CURRENT_LATERAL_OFFSET,
	};

	enum class ErrorCode {
		ERROR_NO_ERROR = 0,
		ERROR_GENERIC = -1,
		ERROR_END_OF_ROAD = -2,
		ERROR_END_OF_ROUTE = -3,
		ERROR_OFF_ROAD = -4,
	};

	enum class UpdateTrackPosMode { UPDATE_NOT_XYZH, UPDATE_XYZ, UPDATE_XYZH };

	enum class PositionStatusMode { POS_STATUS_END_OF_ROAD = (1 << 0), POS_STATUS_END_OF_ROUTE = (1 << 1) };

	typedef enum {
		ALIGN_NONE = 0,	 // No alignment to road
		ALIGN_SOFT = 1,	 // Align to road but add relative orientation
		ALIGN_HARD = 2	 // Completely align to road, disregard relative orientation
	} ALIGN_MODE;

	explicit Position();
	explicit Position(int track_id, double s, double t);
	explicit Position(int track_id, int lane_id, double s, double offset);
	explicit Position(double x, double y, double z, double h, double p, double r);
	explicit Position(double x,
					  double y,
					  double z,
					  double h,
					  double p,
					  double r,
					  bool calculateTrackPosition);
	~Position();

	void Init();
	static bool LoadOpenDrive(const char* filename);
	static OpenDrive* GetOpenDrive();
	int GotoClosestDrivingLaneAtCurrentPosition();

	/**
	Specify position by track coordinate (road_id, s, t)
	@param track_id Id of the road (track)
	@param s Distance to the position along and from the start of the road (track)
	@param updateXY update world coordinates x, y... as well - or not
	@return Non zero return value indicates error of some kind
	*/
	ErrorCode SetTrackPos(int track_id, double s, double t, bool UpdateXY = true);
	void ForceLaneId(int lane_id);
	ErrorCode SetLanePos(int track_id, int lane_id, double s, double offset, int lane_section_idx = -1);
	void SetLaneBoundaryPos(int track_id, int lane_id, double s, double offset, int lane_section_idx = -1);
	void SetRoadMarkPos(int track_id,
						int lane_id,
						int roadmark_idx,
						int roadmarktype_idx,
						int roadmarkline_idx,
						double s,
						double offset,
						int lane_section_idx = -1);

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
	void SetZ(double z);
	void SetZRelative(double z);

	/**

	*/
	void EvaluateOrientation();

	/**
	Specify position by cartesian coordinate (x, y, z, h)
	@param x X-coordinate
	@param y Y-coordinate
	@param z Z-coordinate
	@param h Heading
	@param conenctedOnly If true only roads that can be reached from current position will be considered, if
	false all roads will be considered
	@param roadId If != -1 only this road will be considered else all roads will be searched
	@return Non zero return value indicates error of some kind
	*/
	ErrorCode XYZH2TrackPos(double x,
							double y,
							double z,
							double h,
							bool connectedOnly = false,
							int roadId = -1);

	int TeleportTo(Position* pos);

	int MoveToConnectingRoad(RoadLink* road_link,
							 ContactPointType& contact_point_type,
							 double junctionSelectorAngle = -1.0);

	void SetRelativePosition(Position* rel_pos, PositionType type) {
		rel_pos_ = rel_pos;
		type_ = type;
	}

	Position* GetRelativePosition() { return rel_pos_; }

	void ReleaseRelation();

	int SetRoute(Route* route);
	int CalcRoutePosition();
	const roadmanager::Route* GetRoute() const { return route_; }
	Route* GetRoute() { return route_; }
	RMTrajectory* GetTrajectory() { return trajectory_; }

	void SetTrajectory(RMTrajectory* trajectory);

	/**
	Set the current position along the route.
	@param position A regular position created with road, lane or world coordinates
	@return Non zero return value indicates error of some kind
	*/
	int SetRoutePosition(Position* position);

	/**
	Retrieve the S-value of the current route position. Note: This is the S along the
	complete route, not the actual individual roads.
	*/
	double GetRouteS();
	/**
	Move current position forward, or backwards, ds meters along the route
	@param ds Distance to move, negative will move backwards
	@param actualDistance Distance considering lateral offset and curvature (true/default) or along centerline
	(false)
	@return Non zero return value indicates error of some kind, most likely End Of Route
	*/
	ErrorCode MoveRouteDS(double ds, bool actualDistance = true);

	/**
	Move current position to specified S-value along the route
	@param route_s Distance to move, negative will move backwards
	@return Non zero return value indicates error of some kind, most likely End Of Route
	*/
	ErrorCode SetRouteS(double route_s);

	/**
	Move current position along the route
	@param ds Distance to move, negative will move backwards
	@return Non zero return value indicates error of some kind
	*/
	int SetRouteLanePosition(Route* route, double path_s, int lane_id, double lane_offset);

	/**
	Move current position forward, or backwards, ds meters along the trajectory
	@param ds Distance to move, negative will move backwards
	@return Non zero return value indicates error of some kind
	*/
	int MoveTrajectoryDS(double ds);

	/**
	Move current position to specified S-value along the trajectory
	@param trajectory_s Distance from start of the trajectory
	@return Non zero return value indicates error of some kind
	*/
	int SetTrajectoryS(double trajectory_s);

	int SetTrajectoryPosByTime(double time);

	/**
	Retrieve the S-value of the current trajectory position
	*/
	double GetTrajectoryS() { return s_trajectory_; }

	/**
	Move current position to specified T-value along the trajectory
	@param trajectory_t Lateral distance from trajectory at current s-value
	@return Non zero return value indicates error of some kind
	*/
	void SetTrajectoryT(double trajectory_t) { t_trajectory_ = trajectory_t; }

	/**
	Retrieve the T-value of the current trajectory position
	*/
	double GetTrajectoryT() { return t_trajectory_; }

	/**
	Straight (not route) distance between the current position and the one specified in argument
	@param target_position The position to measure distance from current position.
	@param x (meter). X component of the relative distance.
	@param y (meter). Y component of the relative distance.
	@return distance (meter). Negative if the specified position is behind the current one.
	*/
	double getRelativeDistance(double targetX, double targetY, double& x, double& y) const;

	/**
	Find out the difference between two position objects, in effect subtracting the values
	It can be used to calculate the distance from current position to another one (pos_b)
	@param pos_b The position from which to subtract the current position (this position object)
	@param bothDirections Set to true in order to search also backwards from object
	@param maxDist Don't look further than this
	@return true if position found and parameter values are valid, else false
	*/
	bool Delta(Position* pos_b,
			   PositionDiff& diff,
			   bool bothDirections = true,
			   double maxDist = LARGE_NUMBER) const;

	/**
	Find out the distance, on specified system and type, between two position objects
	@param pos_b The position from which to subtract the current position (this position object)
	@param dist Distance (output parameter)
	@return 0 if position found and parameter values are valid, else -1
	*/
	int Distance(Position* pos_b,
				 CoordinateSystem cs,
				 RelativeDistanceType relDistType,
				 double& dist,
				 double maxDist = LARGE_NUMBER);

	/**
	Find out the distance, on specified system and type, to a world x, y position
	@param x X coordinate of position from which to subtract the current position (this position object)
	@param y Y coordinate of position from which to subtract the current position (this position object)
	@param dist Distance (output parameter)
	@return 0 if position found and parameter values are valid, else -1
	*/
	int Distance(double x,
				 double y,
				 CoordinateSystem cs,
				 RelativeDistanceType relDistType,
				 double& dist,
				 double maxDist = LARGE_NUMBER);

	/**
	Is the current position ahead of the one specified in argument
	This method is more efficient than getRelativeDistance
	@param target_position The position to compare the current to.
	@return true of false
	*/
	bool IsAheadOf(Position target_position);

	/**
	Get information suitable for driver modeling of a point at a specified distance from object along the road
	ahead
	@param lookahead_distance The distance, along the road, to the point
	@param data Struct to fill in calculated values, see typdef for details
	@param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See
	roadmanager::Position::LookAheadMode enum
	@return 0 if successful, other codes see Position::ErrorCode
	*/
	ErrorCode GetProbeInfo(double lookahead_distance, RoadProbeInfo* data, LookAheadMode lookAheadMode);

	/**
	Get information suitable for driver modeling of a point at a specified distance from object along the road
	ahead
	@param target_pos The target position
	@param data Struct to fill in calculated values, see typdef for details
	@return 0 if successful, other codes see Position::ErrorCode
	*/
	ErrorCode GetProbeInfo(Position* target_pos, RoadProbeInfo* data);

	/**
	Get information of current lane at a specified distance from object along the road ahead
	@param lookahead_distance The distance, along the road, to the point
	@param data Struct to fill in calculated values, see typdef for details
	@param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See
	roadmanager::Position::LookAheadMode enum
	@return 0 if successful, -1 if not
	*/
	int GetRoadLaneInfo(double lookahead_distance, RoadLaneInfo* data, LookAheadMode lookAheadMode);
	int GetRoadLaneInfo(RoadLaneInfo* data);

	/**
	Get information of current lane at a specified distance from object along the road ahead
	@param lookahead_distance The distance, along the road, to the point
	@param data Struct to fill in calculated values, see typdef for details
	@return 0 if successful, -1 if not
	*/
	int CalcProbeTarget(Position* target, RoadProbeInfo* data);

	double DistanceToDS(double ds);

	/**
	Move position along the road network, forward or backward, from the current position
	It will automatically follow connecting lanes between connected roads
	If reaching a junction, choose way according to specified junctionSelectorAngle
	@param ds distance to move from current position
	@param dLaneOffset delta lane offset (adding to current position lane offset)
	@param junctionSelectorAngle Desired direction [0:2pi] from incoming road direction (angle = 0), set -1 to
	randomize
	@param actualDistance Distance considering lateral offset and curvature (true/default) or along centerline
	(false)
	@return 0 if successful, other codes see Position::ErrorCode
	*/
	ErrorCode MoveAlongS(double ds,
						 double dLaneOffset,
						 double junctionSelectorAngle,
						 bool actualDistance = true);

	/**
	Move position along the road network, forward or backward, from the current position
	It will automatically follow connecting lanes between connected roads
	If multiple options (only possible in junctions) it will choose randomly
	@param ds distance to move from current position
	@return 0 if successful, other codes see Position::ErrorCode
	*/
	ErrorCode MoveAlongS(double ds, bool actualDistance = true) {
		return MoveAlongS(ds, 0.0, -1.0, actualDistance);
	}

	/**
	Retrieve the track/road ID from the position object
	@return track/road ID
	*/
	int GetTrackId() const;

	/**
	Retrieve the junction ID from the position object
	@return junction ID, -1 if not in a junction
	*/
	int GetJunctionId() const;

	/**
	Retrieve the lane ID from the position object
	@return lane ID
	*/
	int GetLaneId() const;
	/**
	Retrieve the global lane ID from the position object
	@return lane ID
	*/
	int GetLaneGlobalId();

	/**
	Retrieve a road segment specified by road ID
	@param id road ID as specified in the OpenDRIVE file
	*/
	Road* GetRoadById(int id) const { return GetOpenDrive()->GetRoadById(id); }

	/**
	Retrieve the s value (distance along the road segment)
	*/
	double GetS() const;

	/**
	Retrieve the t value (lateral distance from reference lanem (id=0))
	*/
	double GetT() const;

	/**
	Retrieve the offset from current lane
	*/
	double GetOffset();

	/**
	Retrieve the world coordinate X-value
	*/
	double GetX() const;

	/**
	Retrieve the world coordinate Y-value
	*/
	double GetY() const;

	/**
	Retrieve the world coordinate Z-value
	*/
	double GetZ() const;

	/**
	Retrieve the road Z-value
	*/
	double GetZRoad() const { return z_road_; }

	/**
	Retrieve the road slope (vertical inclination)
	*/
	double GetZRoadPrim() const { return z_roadPrim_; }

	/**
	Retrieve the road slope rate of change (vertical bend)
	*/
	double GetZRoadPrimPrim() const { return z_roadPrimPrim_; }

	/**
	Retrieve the road rate of change of the road lateral inclination
	*/
	double GetRoadSuperElevationPrim() const { return roadSuperElevationPrim_; }

	/**
	Retrieve the world coordinate heading angle (radians)
	*/
	double GetH() const;

	/**
	Retrieve the road heading angle (radians)
	*/
	double GetHRoad() const { return h_road_; }

	/**
	Retrieve the driving direction considering lane ID and rult (lef or right hand traffic)
	Will be either 1 (road direction) or -1 (opposite road direction)
	*/
	int GetDrivingDirectionRelativeRoad() const;

	/**
	Retrieve the road heading angle (radians) relative driving direction (lane sign considered)
	*/
	double GetHRoadInDrivingDirection() const;

	/**
	Retrieve the heading angle (radians) relative driving direction (lane sign considered)
	*/
	double GetHRelativeDrivingDirection() const;

	/**
	Retrieve the relative heading angle (radians)
	*/
	double GetHRelative() const;

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
	double GetDrivingDirection() const;

	PositionType GetType() { return type_; }

	void SetTrackId(int trackId) { track_id_ = trackId; }
	void SetLaneId(int laneId) { lane_id_ = laneId; }
	void SetS(double s) { s_ = s; }
	void SetOffset(double offset) { offset_ = offset; }
	void SetT(double t) { t_ = t; }
	void SetX(double x) { x_ = x; }
	void SetY(double y) { y_ = y; }
	void SetH(double h) { h_ = h; }
	void SetP(double p) { p_ = p; }
	void SetR(double r) { r_ = r; }
	void SetVel(double x_vel, double y_vel, double z_vel) { velX_ = x_vel, velY_ = y_vel, velZ_ = z_vel; }
	void SetAcc(double x_acc, double y_acc, double z_acc) { accX_ = x_acc, accY_ = y_acc, accZ_ = z_acc; }
	void SetAngularVel(double h_vel, double p_vel, double r_vel) {
		h_rate_ = h_vel, p_rate_ = p_vel, r_rate_ = r_vel;
	}
	void SetAngularAcc(double h_acc, double p_acc, double r_acc) {
		h_acc_ = h_acc, p_acc_ = p_acc, r_acc_ = r_acc;
	}
	double GetVelX() { return velX_; }
	double GetVelY() { return velY_; }
	double GetVelZ() { return velZ_; }

	/**
	Get lateral component of velocity in vehicle local coordinate system
	*/
	double GetVelLat();

	/**
	Get longitudinal component of velocity in vehicle local coordinate system
	*/
	double GetVelLong();

	/**
	Get lateral and longitudinal component of velocity in vehicle local coordinate system
	This is slightly more effecient than calling GetVelLat and GetVelLong separately
	@param vlat reference parameter returning lateral velocity
	@param vlong reference parameter returning longitudinal velocity
	@return -
	*/
	void GetVelLatLong(double& vlat, double& vlong);

	/**
	Get lateral component of acceleration in vehicle local coordinate system
	*/
	double GetAccLat();

	/**
	Get longitudinal component of acceleration in vehicle local coordinate system
	*/
	double GetAccLong();

	/**
	Get lateral and longitudinal component of acceleration in vehicle local coordinate system
	This is slightly more effecient than calling GetAccLat and GetAccLong separately
	@param alat reference parameter returning lateral acceleration
	@param along reference parameter returning longitudinal acceleration
	@return -
	*/
	void GetAccLatLong(double& alat, double& along);

	/**
	Get lateral component of velocity in road coordinate system
	*/
	double GetVelT();

	/**
	Get longitudinal component of velocity in road coordinate system
	*/
	double GetVelS();

	/**
	Get lateral and longitudinal component of velocity in road coordinate system
	This is slightly more effecient than calling GetVelT and GetVelS separately
	@param vt reference parameter returning lateral velocity
	@param vs reference parameter returning longitudinal velocity
	@return -
	*/
	void GetVelTS(double& vt, double& vs);

	/**
	Get lateral component of acceleration in road coordinate system
	*/
	double GetAccT();

	/**
	Get longitudinal component of acceleration in road coordinate system
	*/
	double GetAccS();

	/**
	Get lateral and longitudinal component of acceleration in road coordinate system
	This is slightly more effecient than calling GetAccT and GetAccS separately
	@param at reference parameter returning lateral acceleration
	@param as reference parameter returning longitudinal acceleration
	@return -
	*/
	void GetAccTS(double& at, double& as);

	double GetAccX() { return accX_; }
	double GetAccY() { return accY_; }
	double GetAccZ() { return accZ_; }
	double GetHRate() { return h_rate_; }
	double GetPRate() { return p_rate_; }
	double GetRRate() { return r_rate_; }
	double GetHAcc() { return h_acc_; }
	double GetPAcc() { return p_acc_; }
	double GetRAcc() { return r_acc_; }

	int GetStatusBitMask() { return status_; }

	void SetOrientationType(OrientationType type) { orientation_type_ = type; }
	OrientationType GetOrientationType() { return orientation_type_; }
	void SetAlignModeH(ALIGN_MODE mode) { align_h_ = mode; }
	void SetAlignModeP(ALIGN_MODE mode) { align_p_ = mode; }
	void SetAlignModeR(ALIGN_MODE mode) { align_r_ = mode; }
	void SetAlignModeZ(ALIGN_MODE mode) { align_z_ = mode; }
	void SetAlignMode(ALIGN_MODE mode) { align_h_ = align_p_ = align_r_ = align_z_ = mode; }

	/**
	Specify which lane types the position object snaps to (is aware of)
	@param laneTypes A combination (bitmask) of lane types
	@return -
	*/
	void SetSnapLaneTypes(int laneTypeMask) { snapToLaneTypes_ = laneTypeMask; }

	void CopyRMPos(Position* from);

	void PrintTrackPos();
	void PrintLanePos();
	void PrintInertialPos();

	void Print();
	void PrintXY();

	bool IsOffRoad();
	bool IsInJunction();

	void ReplaceObjectRefs(Position* pos1, Position* pos2) {
		if (rel_pos_ == pos1) {
			rel_pos_ = pos2;
		}
	}

	/**
		Controls whether to keep lane ID regardless of lateral position or snap to closest lane (default)
		@parameter mode True=keep lane False=Snap to closest (default)
	*/
	void SetLockOnLane(bool mode) { lockOnLane_ = mode; }

   protected:
	void Track2Lane();
	ErrorCode Track2XYZ();
	void Lane2Track();
	void RoadMark2Track();
	/**
	Set position to the border of lane (right border for right lanes, left border for left lanes)
	*/
	void LaneBoundary2Track();
	void XYZ2Track();
	ErrorCode SetLongitudinalTrackPos(int track_id, double s);
	bool EvaluateRoadZPitchRoll();

	// Control lane belonging
	bool lockOnLane_;  // if true then keep logical lane regardless of lateral position, default false

	// route reference
	Route* route_;	// if pointer set, the position corresponds to a point along (s) the route

	// route reference
	RMTrajectory*
		trajectory_;  // if pointer set, the position corresponds to a point along (s) the trajectory

	// track reference
	int track_id_;
	double s_;			   // longitudinal point/distance along the track
	double t_;			   // lateral position relative reference line (geometry)
	int lane_id_;		   // lane reference
	double offset_;		   // lateral position relative lane given by lane_id
	double h_road_;		   // heading of the road
	double h_offset_;	   // local heading offset given by lane width and offset
	double h_relative_;	   // heading relative to the road (h_ = h_road_ + h_relative_)
	double z_relative_;	   // z relative to the road
	double s_trajectory_;  // longitudinal point/distance along the trajectory
	double t_trajectory_;  // longitudinal point/distance along the trajectory
	double curvature_;
	double p_relative_;	  // pitch relative to the road (h_ = h_road_ + h_relative_)
	double r_relative_;	  // roll relative to the road (h_ = h_road_ + h_relative_)
	ALIGN_MODE align_h_;  // Align to road: None, Soft or Hard
	ALIGN_MODE align_p_;  // Align to road: None, Soft or Hard
	ALIGN_MODE align_r_;  // Align to road: None, Soft or Hard
	ALIGN_MODE align_z_;  // Align elevation (Z) to road: None, Soft or Hard

	Position* rel_pos_;
	PositionType type_;
	OrientationType orientation_type_;	// Applicable for relative positions
	int snapToLaneTypes_;				// Bitmask of lane types that the position will snap to
	int status_;						// Bitmask of various states, e.g. off_road, end_of_road

	// inertial reference
	double x_;
	double y_;
	double z_;
	double h_;
	double p_;
	double r_;
	double h_rate_;
	double p_rate_;
	double r_rate_;
	double h_acc_;
	double p_acc_;
	double r_acc_;
	double velX_;
	double velY_;
	double velZ_;
	double accX_;
	double accY_;
	double accZ_;
	double z_road_;
	double p_road_;
	double r_road_;
	double z_roadPrim_;				 // the road vertical slope (dz/ds)
	double z_roadPrimPrim_;			 // rate of change of the road slope, like the vertical curvature
	double roadSuperElevationPrim_;	 // rate of change of the road superelevation/lateral inclination

	// keep track for fast incremental updates of the position
	int track_idx_;			   // road index
	int lane_idx_;			   // lane index
	int roadmark_idx_;		   // laneroadmark index
	int roadmarktype_idx_;	   // laneroadmark index
	int roadmarkline_idx_;	   // laneroadmarkline index
	int lane_section_idx_;	   // lane section
	int geometry_idx_;		   // index of the segment within the track given by track_idx
	int elevation_idx_;		   // index of the current elevation entry
	int super_elevation_idx_;  // index of the current super elevation entry
	int osi_point_idx_;		   // index of the current closest OSI road point
};

#endif 