#include "Spiral.hpp"

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
