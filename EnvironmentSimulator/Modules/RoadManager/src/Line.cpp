#include "Line.hpp"

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

