#pragma once

#include <memory>
#include <vector>
#include "CommonMini.hpp"
#include "Polynomial.hpp"
#include "Userdata.hpp"

class Elevation {
   public:
	Elevation() : s_(0.0), length_(0.0) {}
	Elevation(double s, double a, double b, double c, double d) : s_(s), length_(0) {
		poly3_.Set(a, b, c, d);
	}
	~Elevation(){};

	double GetS() { return s_; }
	void SetLength(double length) { length_ = length; }
	double GetLength() { return length_; }
	void Print();
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&, const std::string);
	Polynomial poly3_;

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;	// TODO check if it can be unique ptr

   private:
	double s_;
	double length_;
};