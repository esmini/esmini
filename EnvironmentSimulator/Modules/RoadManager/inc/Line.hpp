#ifndef LINE_HPP
#define LINE_HPP
#include "Geometry.hpp"
#include "pugixml.hpp"

class Line : public Geometry {
   public:
	Line() {}
	Line(double s, double x, double y, double hdg, double length)
		: Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_LINE) {}
	~Line(){};

	void Print();
	void EvaluateDS(double ds, double* x, double* y, double* h);
	double EvaluateCurvatureDS(double ds) {
		(void)ds;
		return 0;
	}
	void Save(pugi::xml_node&) override;
};
#endif