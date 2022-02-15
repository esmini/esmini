#ifndef SPIRAL_HPP
#define SPIRAL_HPP

#include "Geometry.hpp"
#include "Arc.hpp"
#include "Line.hpp"

class Spiral : public Geometry {
   public:
	Spiral()
		: curv_start_(0.0),
		  curv_end_(0.0),
		  c_dot_(0.0),
		  x0_(0.0),
		  y0_(0.0),
		  h0_(0.0),
		  s0_(0.0),
		  arc_(0),
		  line_(0) {}
	Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end);

	~Spiral(){};

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
	void EvaluateDS(double ds, double* x, double* y, double* h);
	double EvaluateCurvatureDS(double ds);
	void SetX(double x);
	void SetY(double y);
	void SetHdg(double h);
	void Save(pugi::xml_node&) override;

	Arc* arc_;
	Line* line_;

   private:
	double curv_start_;
	double curv_end_;
	double c_dot_;
	double x0_;	 // 0 if spiral starts with curvature = 0
	double y0_;	 // 0 if spiral starts with curvature = 0
	double h0_;	 // 0 if spiral starts with curvature = 0
	double s0_;	 // 0 if spiral starts with curvature = 0
};

#endif
