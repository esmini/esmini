#ifndef ARC_HPP
#define ARC_HPP

#include "Geometry.hpp"
class Arc : public Geometry {
   public:
	Arc() : curvature_(0.0) {}
	Arc(double s, double x, double y, double hdg, double length, double curvature)
		: Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_ARC), curvature_(curvature) {}
	~Arc() {}

	double EvaluateCurvatureDS(double ds) {
		(void)ds;
		return curvature_;
	}
	double GetRadius() { return std::fabs(1.0 / curvature_); }
	double GetCurvature() { return curvature_; }
	void Print();
	void EvaluateDS(double ds, double* x, double* y, double* h);
	void Save(pugi::xml_node&) override;

   private:
	double curvature_;
};
#endif