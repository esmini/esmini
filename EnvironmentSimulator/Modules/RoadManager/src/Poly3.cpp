#include "Poly3.hpp"

Poly3::Poly3(double s, double x, double y, double hdg, double length, double a, double b, double c, double d)
	: Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_POLY3), umax_(0.0) {
	poly3_.Set(a, b, c, d);

	double xTmp = 0;
	double yTmp = 0;

	EvaluateDSLocal(GetLength() - SMALL_NUMBER, xTmp, yTmp);
	SetUMax(xTmp);
}

void Poly3::Print() {
	LOG("Poly3 x: %.2f, y: %.2f, h: %.2f length: %.2f a: %.2f b: %.2f c: %.2f d: %.2f\n", GetX(), GetY(),
		GetHdg(), GetLength(), poly3_.GetA(), poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

void Poly3::Save(pugi::xml_node& geometry) {
	Geometry::Save(geometry);
	auto poly3 = geometry.append_child("poly3");
	poly3.append_attribute("a").set_value(poly3_.GetA());
	poly3.append_attribute("b").set_value(poly3_.GetB());
	poly3.append_attribute("c").set_value(poly3_.GetC());
	poly3.append_attribute("d").set_value(poly3_.GetD());
}

void Poly3::EvaluateDSLocal(double ds, double& u, double& v) {
	double distTmp = 0;
	double steplen = MIN(10, ds);  // along u axis - to be tuned

	u = v = 0;

	if (ds > length_ - SMALL_NUMBER) {
		u = umax_;
		v = poly3_.Evaluate(u);
	} else if (ds > SMALL_NUMBER) {
		for (double uTmp = 0; uTmp < length_; uTmp += steplen) {
			double vTmp = poly3_.Evaluate(uTmp);
			double delta = sqrt((uTmp - u) * (uTmp - u) + (vTmp - v) * (vTmp - v));

			if (distTmp + delta > ds) {
				// interpolate
				double w = (distTmp + delta - ds) / MAX(delta, SMALL_NUMBER);
				u = w * u + (1 - w) * uTmp;
				v = poly3_.Evaluate(u);
				break;
			}
			distTmp += delta;
			u = uTmp;
			v = vTmp;
		}
	}
}

void Poly3::EvaluateDS(double ds, double* x, double* y, double* h) {
	double u_local = 0;
	double v_local = 0;

	EvaluateDSLocal(ds, u_local, v_local);

	*x = GetX() + u_local * cos(GetHdg()) - v_local * sin(GetHdg());
	*y = GetY() + u_local * sin(GetHdg()) + v_local * cos(GetHdg());
	*h = GetHdg() + atan(poly3_.EvaluatePrim(u_local));
}

double Poly3::EvaluateCurvatureDS(double ds) {
	return poly3_.EvaluatePrimPrim(ds);
}
