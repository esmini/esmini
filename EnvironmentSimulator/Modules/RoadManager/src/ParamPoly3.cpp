#include "ParamPoly3.hpp"

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
