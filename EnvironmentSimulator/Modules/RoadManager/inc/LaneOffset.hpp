#ifndef LANEOFFSET_HPP
#define LANEOFFSET_HPP

#include <memory>
#include <vector>
#include "CommonMini.hpp"
#include "Polynomial.hpp"
#include "pugixml.hpp"

class LaneOffset {
   public:
	LaneOffset() : s_(0.0), length_(0.0) {}
	LaneOffset(double s, double a, double b, double c, double d) : s_(s), length_(0.0) {
		polynomial_.Set(a, b, c, d);
	}
	~LaneOffset() {}

	void Set(double s, double a, double b, double c, double d) {
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
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	Polynomial polynomial_;
	double s_;
	double length_;
};

#endif