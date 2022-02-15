#ifndef LANEROADMARKTYPELINE_HPP
#define LANEROADMARKTYPELINE_HPP

class LaneRoadMarkTypeLine {
   public:
	enum RoadMarkTypeLineRule { NO_PASSING, CAUTION, NONE };

	LaneRoadMarkTypeLine(double length,
						 double space,
						 double t_offset,
						 double s_offset,
						 RoadMarkTypeLineRule rule,
						 double width,
						 RoadMarkColor color = RoadMarkColor::UNDEFINED)
		: length_(length),
		  space_(space),
		  t_offset_(t_offset),
		  s_offset_(s_offset),
		  rule_(rule),
		  width_(width),
		  color_(color) {}
	~LaneRoadMarkTypeLine(){};
	double GetSOffset() { return s_offset_; }
	double GetTOffset() { return t_offset_; }
	double GetLength() { return length_; }
	double GetSpace() { return space_; }
	double GetWidth() { return width_; }
	OSIPoints* GetOSIPoints() { return &osi_points_; }
	OSIPoints osi_points_;
	void SetGlobalId();
	int GetGlobalId() { return global_id_; }
	RoadMarkColor GetColor() { return color_; }
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);

   private:
	double length_;
	double space_;
	double t_offset_;
	double s_offset_;
	RoadMarkTypeLineRule rule_;
	double width_;
	int global_id_;		   // Unique ID for OSI
	RoadMarkColor color_;  // if set, supersedes setting in <RoadMark>
	std::vector<UserData*> user_data_;
};

