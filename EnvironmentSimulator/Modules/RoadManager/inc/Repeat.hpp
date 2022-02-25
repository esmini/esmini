#pragma once

#include <memory>
#include <vector>
#include "Userdata.hpp"

class Repeat {
   public:
	double s_;
	double length_;
	double distance_;
	double tStart_;
	double tEnd_;
	double heightStart_;
	double heightEnd_;
	double zOffsetStart_;
	double zOffsetEnd_;
	double widthStart_;
	double widthEnd_;
	double lengthStart_;
	double lengthEnd_;
	double radiusStart_;
	double radiusEnd_;

	Repeat(double s,
		   double length,
		   double distance,
		   double tStart,
		   double tEnd,
		   double heightStart,
		   double heightEnd,
		   double zOffsetStart,
		   double zOffsetEnd)
		: s_(s),
		  length_(length),
		  distance_(distance),
		  tStart_(tStart),
		  tEnd_(tEnd),
		  heightStart_(heightStart),
		  heightEnd_(heightEnd),
		  zOffsetStart_(zOffsetStart),
		  zOffsetEnd_(zOffsetEnd),
		  widthStart_(0.0),
		  widthEnd_(0.0),
		  lengthStart_(0.0),
		  lengthEnd_(0.0),
		  radiusStart_(0.0),
		  radiusEnd_(0.0) {}

	void SetWidthStart(double widthStart) { widthStart_ = widthStart; }
	void SetWidthEnd(double widthEnd) { widthEnd_ = widthEnd; }
	void SetLengthStart(double lengthStart) { lengthStart_ = lengthStart; }
	void SetLengthEnd(double lengthEnd) { lengthStart_ = lengthEnd; }
	void SetHeightStart(double heightStart) { heightStart_ = heightStart; }
	void SeHeightEnd(double heightStart) { heightStart_ = heightStart; }
	double GetS() { return s_; }
	double GetLength() { return length_; }
	double GetDistance() { return distance_; }
	double GetTStart() { return tStart_; }
	double GetTEnd() { return tEnd_; }
	double GetHeightStart() { return heightStart_; }
	double GetHeightEnd() { return heightEnd_; }
	double GetZOffsetStart() { return zOffsetStart_; }
	double GetZOffsetEnd() { return zOffsetEnd_; }
	double GetWidthStart() { return widthStart_; }
	double GetWidthEnd() { return widthEnd_; }
	double GetLengthStart() { return lengthStart_; }
	double GetLengthEnd() { return lengthEnd_; }
	double GetRadiusStart() { return radiusStart_; }
	double GetRadiusEnd() { return radiusEnd_; }

	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;
};