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



