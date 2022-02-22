#pragma once

#include <cassert>
#include <vector>
#include "Outline.hpp"
#include "Repeat.hpp"
#include "RoadObject.hpp"
#include "pugixml.hpp"

class RMObject : public RoadObject {
   public:
	RMObject(double s,
			 double t,
			 int id,
			 std::string name,
			 Orientation orientation,
			 double z_offset,
			 std::string type,
			 double length,
			 double height,
			 double width,
			 double heading,
			 double pitch,
			 double roll)
		: s_(s),
		  t_(t),
		  id_(id),
		  name_(name),
		  orientation_(orientation),
		  z_offset_(z_offset),
		  type_(type),
		  length_(length),
		  height_(height),
		  width_(width),
		  heading_(heading),
		  pitch_(pitch),
		  roll_(roll),
		  repeat_(0) {}

	~RMObject() { outlines_.clear(); }

	std::string GetName() { return name_; }
	std::string GetType() { return type_; }
	int GetId() { return id_; }
	double GetS() { return s_; }
	double GetT() { return t_; }
	double GetHOffset() { return heading_; }
	double GetPitch() { return pitch_; }
	double GetRoll() { return roll_; }
	double GetZOffset() { return z_offset_; }
	double GetHeight() { return height_; }
	double GetLength() { return length_; }
	double GetWidth() { return width_; }
	Orientation GetOrientation() { return orientation_; }
	void AddOutline(std::shared_ptr<Outline> outline) { outlines_.push_back(outline); }
	void SetRepeat(std::shared_ptr<Repeat> repeat);								   // odr1.5
	void AddRepeat(std::shared_ptr<Repeat> repeat) { repeat_.push_back(repeat); }  // odr1.4
	std::shared_ptr<Repeat> GetRepeat();
	int GetNumberOfOutlines() { return (int)outlines_.size(); }
	std::shared_ptr<Outline> GetOutline(int i) { return (0 <= i && i < outlines_.size()) ? outlines_[i] : 0; }
	void Save(pugi::xml_node&);

	std::vector<std::shared_ptr<Outline>> getOutlineVector() { return outlines_; }
	std::vector<std::shared_ptr<Repeat>> getRepeatVector() { return repeat_; }


   protected:
	std::vector<std::shared_ptr<Outline>> outlines_;
	std::vector<std::shared_ptr<Repeat>> repeat_;  // OpenDRIVE 1.4 uses multiple repeat tags.

   private:
	std::string type_;
	std::string name_;
	int id_;
	double s_;
	double t_;
	double z_offset_;
	Orientation orientation_;
	double length_;
	double height_;
	double width_;
	double heading_;
	double pitch_;
	double roll_;
};