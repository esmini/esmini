#ifndef LANEWIDTH_HPP
#define LANEWIDTH_HPP

#include <memory
#include "Polynomial.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

class LaneWidth {
   public:
	LaneWidth(double s_offset, double a, double b, double c, double d) : s_offset_(s_offset) {
		poly3_.Set(a, b, c, d);
	}

	double GetSOffset() { return s_offset_; }
	void Print();
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);

	Polynomial poly3_;

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	double s_offset_;
};
#endif