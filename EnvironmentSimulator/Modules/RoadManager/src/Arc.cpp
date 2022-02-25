#include "Arc.hpp"

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