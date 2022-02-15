#ifndef STRUCTSANDDEFINES_HPP
#define STRUCTSANDDEFINES_HPP

typedef enum { NONE = 0, SUCCESSOR = 1, PREDECESSOR = -1 } LinkType;

typedef struct {
	double s;
	double x;
	double y;
	double z;
	double h;
	double time;
	double speed;
	double p;
	bool calcHeading;
} TrajVertex;

typedef struct {
	double s;
	double x;
	double y;
	double z;
	double h;
} PointStruct;

struct RoadMarkInfo {
	int roadmark_idx_;
	int roadmarkline_idx_;
};

enum class RoadMarkColor {
	UNDEFINED,
	STANDARD_COLOR,	 // equivalent to white
	BLUE,
	GREEN,
	RED,
	WHITE,
	YELLOW
};

enum ContactPointType {
	CONTACT_POINT_UNDEFINED,
	CONTACT_POINT_START,
	CONTACT_POINT_END,
	CONTACT_POINT_JUNCTION,	 // No contact point for element type junction
};

struct LaneInfo {
	int lane_section_idx_;
	int lane_id_;
};

typedef struct {
	int fromLane_;
	int toLane_;
	std::vector<UserData*> user_data_;
	void Save(pugi::xml_node& object);
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }
} ValidityRecord;


typedef struct {
	std::vector<UserData*> user_data_;
	int signalId_;
	std::string type_;
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);
} Control;

typedef struct {
	std::vector<UserData*> user_data_;
	int id_;
	std::string type_;
	int sequence_;
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& junction);
} JunctionController;

struct OpenDriveOffset {
	OpenDriveOffset() : x_(0), y_(0), z_(0), hdg_(0){};
	double x_;
	double y_;
	double z_;
	double hdg_;

	std::vector<UserData*> user_data_;
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&) const;
	bool isValid() const { return x_ || y_ || z_ || hdg_; };
};

struct OpenDriveHeader {
	void Save(pugi::xml_node&) const;
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }

	uint revMajor_;
	uint revMinor_;
	std::string name_;
	float version_;
	std::string date_;
	double north_;
	double south_;
	double east_;
	double west_;
	std::string vendor_;
	OpenDriveOffset offset_;
	GeoReference georeference_;
	std::vector<UserData*> user_data_;
};

typedef struct {
	double pos[3];		 // position, in global coordinate system
	double heading;		 // road heading at steering target point
	double pitch;		 // road pitch (inclination) at steering target point
	double roll;		 // road roll (camber) at steering target point
	double width;		 // lane width
	double curvature;	 // road curvature at steering target point
	double speed_limit;	 // speed limit given by OpenDRIVE type entry
	int roadId;			 // road ID
	int junctionId;		 // junction ID (-1 if not in a junction)
	int laneId;			 // lane ID
	double laneOffset;	 // lane offset (lateral distance from lane center)
	double s;			 // s (longitudinal distance along reference line)
	double t;			 // t (lateral distance from reference line)
} RoadLaneInfo;

typedef struct {
	RoadLaneInfo road_lane_info;  // Road info at probe target position
	double
		relative_pos[3];  // probe target position relative vehicle (pivot position object) coordinate system
	double relative_h;	  // heading angle to probe target from and relatove to vehicle (pivot position)
} RoadProbeInfo;

typedef struct {
	double ds;		 // delta s (longitudinal distance)
	double dt;		 // delta t (lateral distance)
	int dLaneId;	 // delta laneId (increasing left and decreasing to the right)
	double dx;		 // delta x (world coordinate system)
	double dy;		 // delta y (world coordinate system)
	double dxLocal;	 // delta x (local coordinate system)
	double dyLocal;	 // delta y (local coordinate system)
} PositionDiff;

enum class CoordinateSystem { CS_UNDEFINED, CS_ENTITY, CS_LANE, CS_ROAD, CS_TRAJECTORY };

enum class RelativeDistanceType {
	REL_DIST_UNDEFINED,
	REL_DIST_LATERAL,
	REL_DIST_LONGITUDINAL,
	REL_DIST_CARTESIAN,
	REL_DIST_EUCLIDIAN
};

#endif