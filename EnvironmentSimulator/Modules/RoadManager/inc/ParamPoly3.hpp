#ifndef PARAMPOLY3_HPP
#define PARAMPOLY3_HPP
#include "Geometry.hpp"
class ParamPoly3 : public Geometry {
   public:
	enum PRangeType { P_RANGE_UNKNOWN, P_RANGE_NORMALIZED, P_RANGE_ARC_LENGTH };

	ParamPoly3() {}
	ParamPoly3(double s,
			   double x,
			   double y,
			   double hdg,
			   double length,
			   double aU,
			   double bU,
			   double cU,
			   double dU,
			   double aV,
			   double bV,
			   double cV,
			   double dV,
			   PRangeType p_range)
		: Geometry(s, x, y, hdg, length, GeometryType::GEOMETRY_TYPE_PARAM_POLY3), p_range_(p_range) {
		poly3U_.Set(aU, bU, cU, dU, p_range == PRangeType::P_RANGE_NORMALIZED ? 1.0 / length : 1.0);
		poly3V_.Set(aV, bV, cV, dV, p_range == PRangeType::P_RANGE_NORMALIZED ? 1.0 / length : 1.0);
		calcS2PMap(p_range);
	}
	~ParamPoly3(){};

	void Print();
	Polynomial GetPoly3U() { return poly3U_; }
	Polynomial GetPoly3V() { return poly3V_; }
	void EvaluateDS(double ds, double* x, double* y, double* h);
	double EvaluateCurvatureDS(double ds);
	void calcS2PMap(PRangeType p_range);
	double s2p_map_[PARAMPOLY3_STEPS + 1][2];
	double S2P(double s);
	void Save(pugi::xml_node&) override;

	PRangeType p_range_;
	Polynomial poly3U_;
	Polynomial poly3V_;
};
#endif