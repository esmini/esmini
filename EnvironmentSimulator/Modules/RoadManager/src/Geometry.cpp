#include "Geometry.hpp"

void Geometry::Print() {
	LOG("Geometry virtual Print\n");
}

void Geometry::Save(pugi::xml_node& geometry) {
	geometry.append_attribute("s").set_value(s_);
	geometry.append_attribute("x").set_value(x_);
	geometry.append_attribute("y").set_value(y_);
	geometry.append_attribute("hdg").set_value(hdg_);
	geometry.append_attribute("length").set_value(length_);

	for (auto userData : user_data_) {
		userData->Save(geometry);
	}
}

void Geometry::EvaluateDS(double ds, double* x, double* y, double* h) {
	(void)ds;
	(void)x;
	(void)y;
	(void)h;
	LOG("Geometry virtual Evaluate\n");
}

void Line::Print() {
	LOG("Line x: %.2f, y: %.2f, h: %.2f length: %.2f\n", GetX(), GetY(), GetHdg(), GetLength());
}

void Line::Save(pugi::xml_node& geometry) {
	Geometry::Save(geometry);
	geometry.append_child("line");
}

void Line::EvaluateDS(double ds, double* x, double* y, double* h) {
	*h = GetHdg();
	*x = GetX() + ds * cos(*h);
	*y = GetY() + ds * sin(*h);
}

void Arc::Print() {
	LOG("Arc x: %.2f, y: %.2f, h: %.2f curvature: %.2f length: %.2f\n", GetX(), GetY(), GetHdg(), curvature_,
		GetLength());
}

void Arc::Save(pugi::xml_node& geometry) {
	Geometry::Save(geometry);
	auto arc = geometry.append_child("arc");
	arc.append_attribute("curvature").set_value(curvature_);
}

void Arc::EvaluateDS(double ds, double* x, double* y, double* h) {
	double x_local = 0;
	double y_local = 0;

	// arc_length = angle * radius -> angle = arc_length / radius = arc_length * curvature
	double angle = ds * curvature_;

	// Now calculate x, y in a local unit circle coordinate system
	if (curvature_ < 0) {
		// starting from 90 degrees going clockwise
		x_local = cos(angle + M_PI / 2.0);
		y_local = sin(angle + M_PI / 2.0) - 1;	// -1 transform to y = 0
	} else {
		// starting from -90 degrees going counter clockwise
		x_local = cos(angle + 3.0 * M_PI_2);
		y_local = sin(angle + 3.0 * M_PI_2) + 1;  // +1 transform to y = 0
	}

	// Rotate according to heading and scale according to radius
	*x = GetX() + GetRadius() * (x_local * cos(GetHdg()) - y_local * sin(GetHdg()));
	*y = GetY() + GetRadius() * (x_local * sin(GetHdg()) + y_local * cos(GetHdg()));
	*h = GetHdg() + angle;
}
Spiral::Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end)
	: Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_SPIRAL),
	  curv_start_(curv_start),
	  curv_end_(curv_end),
	  c_dot_(0.0),
	  x0_(0.0),
	  y0_(0.0),
	  h0_(0.0),
	  s0_(0.0),
	  arc_(0),
	  line_(0) {
	SetCDot((curv_end_ - curv_start_) / length_);

	if (fabs(GetCDot()) < SMALL_NUMBER) {
		// constant radius => clothoid is actually a line or an arc
		if (fabs(this->GetCurvStart()) < SMALL_NUMBER)	// Line
		{
			line_ = new Line(s, x, y, hdg, length);
		} else	// Arc
		{
			arc_ = new Arc(s, x, y, hdg, length, curv_start);
		}
	} else {
		if (fabs(curv_start_) > SMALL_NUMBER) {
			// not starting from zero curvature (straight line)
			// How long do we need to follow the spiral to reach start curve value?
			SetS0(curv_start_ / c_dot_);

			// Find out x, y, heading of start position
			double x0, y0, h0;
			odrSpiral(GetS0(), c_dot_, &x0, &y0, &h0);

			SetX0(x0);
			SetY0(y0);
			SetH0(h0);
		}
	}
}

void Spiral::Print() {
	LOG("Spiral x: %.2f, y: %.2f, h: %.2f start curvature: %.4f end curvature: %.4f length: %.2f %s\n",
		GetX(), GetY(), GetHdg(), GetCurvStart(), GetCurvEnd(), GetLength(),
		arc_ != 0	 ? " - actually an Arc"
		: line_ != 0 ? "- actually a Line"
					 : "");
}

void Spiral::Save(pugi::xml_node& geometry) {
	Geometry::Save(geometry);
	auto spiral = geometry.append_child("spiral");
	spiral.append_attribute("curvStart").set_value(curv_start_);
	spiral.append_attribute("curvEnd").set_value(curv_end_);
}

void Spiral::EvaluateDS(double ds, double* x, double* y, double* h) {
	double xTmp, yTmp, t;

	if (line_ != 0) {
		line_->EvaluateDS(ds, x, y, h);
	} else if (arc_ != 0) {
		arc_->EvaluateDS(ds, x, y, h);
	} else {
		odrSpiral(s0_ + ds, c_dot_, &xTmp, &yTmp, &t);

		*h = t - GetH0() + GetHdg();

		double x1, x2, y1, y2;

		// transform spline segment to origo and start angle = 0
		x1 = xTmp - GetX0();
		y1 = yTmp - GetY0();
		x2 = x1 * cos(-GetH0()) - y1 * sin(-GetH0());
		y2 = x1 * sin(-GetH0()) + y1 * cos(-GetH0());

		// Then transform according to segment start position and heading
		*x = GetX() + x2 * cos(GetHdg()) - y2 * sin(GetHdg());
		*y = GetY() + x2 * sin(GetHdg()) + y2 * cos(GetHdg());
	}
}

double Spiral::EvaluateCurvatureDS(double ds) {
	if (line_ != 0) {
		return LARGE_NUMBER;
	} else if (arc_ != 0) {
		return arc_->GetCurvature();
	} else {
		return (curv_start_ + (ds / GetLength()) * (curv_end_ - curv_start_));
	}
}

void Spiral::SetX(double x) {
	if (line_ != 0) {
		line_->SetX(x);
	} else if (arc_ != 0) {
		arc_->SetX(x);
	} else {
		x_ = x;
	}
}

void Spiral::SetY(double y) {
	if (line_ != 0) {
		line_->SetY(y);
	} else if (arc_ != 0) {
		arc_->SetY(y);
	} else {
		y_ = y;
	}
}

void Spiral::SetHdg(double h) {
	if (line_ != 0) {
		line_->SetHdg(h);
	} else if (arc_ != 0) {
		arc_->SetHdg(h);
	} else {
		hdg_ = h;
	}
}
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

void ParamPoly3::Print() {
	LOG("ParamPoly3 x: %.2f, y: %.2f, h: %.2f length: %.2f U: %.8f, %.8f, %.8f, %.8f V: %.8f, %.8f, %.8f, "
		"%.8f\n",
		GetX(), GetY(), GetHdg(), GetLength(), poly3U_.GetA(), poly3U_.GetB(), poly3U_.GetC(), poly3U_.GetD(),
		poly3V_.GetA(), poly3V_.GetB(), poly3V_.GetC(), poly3V_.GetD());
}

void ParamPoly3::Save(pugi::xml_node& geometry) {
	Geometry::Save(geometry);
	auto paramPoly3 = geometry.append_child("paramPoly3");
	paramPoly3.append_attribute("aU").set_value(poly3U_.GetA());
	paramPoly3.append_attribute("bU").set_value(poly3U_.GetB());
	paramPoly3.append_attribute("cU").set_value(poly3U_.GetC());
	paramPoly3.append_attribute("dU").set_value(poly3U_.GetD());
	paramPoly3.append_attribute("aV").set_value(poly3V_.GetA());
	paramPoly3.append_attribute("bV").set_value(poly3V_.GetB());
	paramPoly3.append_attribute("cV").set_value(poly3V_.GetC());
	paramPoly3.append_attribute("dV").set_value(poly3V_.GetD());

	if (p_range_ == PRangeType::P_RANGE_ARC_LENGTH) {
		paramPoly3.append_attribute("pRange").set_value("arcLength");
	} else if (p_range_ == PRangeType::P_RANGE_NORMALIZED) {
		paramPoly3.append_attribute("pRange").set_value("normalized");
	}
}

void ParamPoly3::EvaluateDS(double ds, double* x, double* y, double* h) {
	double p = S2P(ds);
	double hdg = GetHdg();

	double u_local = poly3U_.Evaluate(p);
	double v_local = poly3V_.Evaluate(p);

	*x = GetX() + u_local * cos(hdg) - v_local * sin(hdg);
	*y = GetY() + u_local * sin(hdg) + v_local * cos(hdg);
	*h = hdg + atan2(poly3V_.EvaluatePrim(p), poly3U_.EvaluatePrim(p));
}

double ParamPoly3::EvaluateCurvatureDS(double ds) {
	return poly3V_.EvaluatePrimPrim(ds) / poly3U_.EvaluatePrim(ds);
}

void ParamPoly3::calcS2PMap(PRangeType p_range) {
	double len = 0;
	double p_step_len = 1.0 / double(PARAMPOLY3_STEPS);
	double p = 0;

	if (p_range == PRangeType::P_RANGE_ARC_LENGTH) {
		p_step_len = length_ / (PARAMPOLY3_STEPS);
	}

	// Calculate actual arc length of the curve
	s2p_map_[0][0] = 0;
	for (size_t i = 1; i < PARAMPOLY3_STEPS + 1; i++) {
		p += p_step_len;

		double pm = p - 0.5 * p_step_len;  // midpoint method
		double integrator
			= sqrt(pow(3 * poly3U_.GetD() * pm * pm + 2 * poly3U_.GetC() * pm + poly3U_.GetB(), 2)
				   + pow(3 * poly3V_.GetD() * pm * pm + 2 * poly3V_.GetC() * pm + poly3V_.GetB(), 2));

		len += p_step_len * integrator;
		s2p_map_[i][0] = len;
	}

	// Map length (ds) to p for each sub-segment, adjust for incorrect length attribute
	double scale_factor;
	scale_factor = length_ / len;

	for (size_t i = 0; i < PARAMPOLY3_STEPS + 1; i++) {
		s2p_map_[i][0] *= scale_factor;
		s2p_map_[i][1] = i * length_ / PARAMPOLY3_STEPS;
	}
}

double ParamPoly3::S2P(double s) {
	for (size_t i = 0; i < PARAMPOLY3_STEPS; i++) {
		if (s2p_map_[i + 1][0] > s) {
			double w = (s - s2p_map_[i][0]) / (s2p_map_[i + 1][0] - s2p_map_[i][0]);
			return s2p_map_[i][1] + w * (s2p_map_[i + 1][1] - s2p_map_[i][1]);
		}
	}
	return s2p_map_[PARAMPOLY3_STEPS][1];
}
