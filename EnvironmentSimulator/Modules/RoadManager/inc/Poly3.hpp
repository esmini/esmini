#ifndef POLY3_HPP
#define POLY3_HPP

#include "Geometry.hpp"

class Poly3 : public Geometry {
   public:
	Poly3() : umax_(0.0) {}
	Poly3(double s, double x, double y, double hdg, double length, double a, double b, double c, double d);
	~Poly3(){};

	void SetUMax(double umax) { umax_ = umax; }
	double GetUMax() { return umax_; }
	void Print();
	Polynomial GetPoly3() { return poly3_; }
	void EvaluateDS(double ds, double* x, double* y, double* h);
	double EvaluateCurvatureDS(double ds);
	void Save(pugi::xml_node&) override;

	Polynomial poly3_;

   private:
	double umax_;
	void EvaluateDSLocal(double ds, double& u, double& v);
};

#endif