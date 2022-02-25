#pragma once
#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>
#include <memory>
#include <vector>
#include "CommonMini.hpp"
#include "Polynomial.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"
class Geometry {
   public:
	typedef enum {
		GEOMETRY_TYPE_UNKNOWN,
		GEOMETRY_TYPE_LINE,
		GEOMETRY_TYPE_ARC,
		GEOMETRY_TYPE_SPIRAL,
		GEOMETRY_TYPE_POLY3,
		GEOMETRY_TYPE_PARAM_POLY3,
	} GeometryType;

	Geometry() : s_(0.0), x_(0.0), y_(0), hdg_(0), length_(0), type_(GeometryType::GEOMETRY_TYPE_UNKNOWN) {}
	Geometry(double s, double x, double y, double hdg, double length, GeometryType type)
		: s_(s), x_(x), y_(y), hdg_(hdg), length_(length), type_(type) {}
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
	virtual void EvaluateDS(double ds, double* x, double* y, double* h);
	virtual void Save(pugi::xml_node& geometry);
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }

   protected:
	double s_;
	double x_;
	double y_;
	double hdg_;
	double length_;
	GeometryType type_;
	std::vector<std::shared_ptr<UserData>> user_data_;	// TODO check if it can be unique ptr
};

#endif
