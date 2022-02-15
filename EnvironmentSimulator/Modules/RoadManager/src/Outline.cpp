#include "Outline.hpp"
#include "Position.hpp"
void Outline::Save(pugi::xml_node& object) {
	// Sace according to OpenDRIVE 1.5M
	auto outlines = object.child("outlines");
	if (outlines.empty()) {
		outlines = object.append_child("outlines");
	}
	auto outline = outlines.append_child("outline");

	if (id_)
		outline.append_attribute("id").set_value(id_);

	switch (fillType_) {
	case Outline::FillType::FILL_TYPE_GRASS:
		outline.append_attribute("fillType").set_value("grass");
		break;
	case Outline::FillType::FILL_TYPE_CONCRETE:
		outline.append_attribute("fillType").set_value("concrete");
		break;
	case Outline::FillType::FILL_TYPE_COBBLE:
		outline.append_attribute("fillType").set_value("cobble");
		break;
	case Outline::FillType::FILL_TYPE_ASPHALT:
		outline.append_attribute("fillType").set_value("asphalt");
		break;
	case Outline::FillType::FILL_TYPE_PAVEMENT:
		outline.append_attribute("fillType").set_value("pavement");
		break;
	case Outline::FillType::FILL_TYPE_GRAVEL:
		outline.append_attribute("fillType").set_value("gravel");
		break;
	case Outline::FillType::FILL_TYPE_SOIL:
		outline.append_attribute("fillType").set_value("soil");
		break;
	default:  // Will not be defined for OpenDRIVE 1.4
		break;
	}
	// outline.append_attribute("outer").set_value(); // TODO:
	if (closed_)
		outline.append_attribute("closed").set_value("true");
	else if (!closed_)
		outline.append_attribute("closed").set_value("false");

	// outline.append_attribute("laneType").set_value(); // TODO:

	for (auto corner : corner_) {
		corner->Save(outline);
	}
}

OutlineCornerRoad::OutlineCornerRoad(int roadId, double s, double t, double dz, double height)
	: roadId_(roadId), s_(s), t_(t), dz_(dz), height_(height) {}

OutlineCornerRoad::OutlineCornerRoad(int id, int roadId, double s, double t, double dz, double height)
	: id_(id), roadId_(roadId), s_(s), t_(t), dz_(dz), height_(height) {}

void OutlineCornerRoad::GetPos(double& x, double& y, double& z) {
	Position pos;
	pos.SetTrackPos(roadId_, s_, t_);
	x = pos.GetX();
	y = pos.GetY();
	z = pos.GetZ() + dz_;
}

void OutlineCornerRoad::Save(pugi::xml_node& outline) {
	auto cornerRoad = outline.append_child("cornerRoad");
	cornerRoad.append_attribute("s").set_value(s_);
	cornerRoad.append_attribute("t").set_value(t_);
	cornerRoad.append_attribute("dz").set_value(dz_);
	cornerRoad.append_attribute("height").set_value(height_);
	if (id_)  // Introduced in OpenDRIVE 1.5
		cornerRoad.append_attribute("id").set_value(id_);
}

OutlineCornerLocal::OutlineCornerLocal(int roadId,
									   double s,
									   double t,
									   double u,
									   double v,
									   double zLocal,
									   double height,
									   double heading)
	: roadId_(roadId), s_(s), t_(t), u_(u), v_(v), zLocal_(zLocal), height_(height), heading_(heading) {}

OutlineCornerLocal::OutlineCornerLocal(int id,
									   int roadId,
									   double s,
									   double t,
									   double u,
									   double v,
									   double zLocal,
									   double height,
									   double heading)
	: id_(id),
	  roadId_(roadId),
	  s_(s),
	  t_(t),
	  u_(u),
	  v_(v),
	  zLocal_(zLocal),
	  height_(height),
	  heading_(heading) {}

void OutlineCornerLocal::GetPos(double& x, double& y, double& z) {
	Position pref;
	pref.SetTrackPos(roadId_, s_, t_);
	double total_heading = GetAngleSum(pref.GetH(), heading_);
	double u2, v2;
	RotateVec2D(u_, v_, total_heading, u2, v2);

	x = pref.GetX() + u2;
	y = pref.GetY() + v2;
	z = pref.GetZ() + zLocal_;
}

void OutlineCornerLocal::Save(pugi::xml_node& outline) {
	auto cornerLocal = outline.append_child("cornerLocal");
	cornerLocal.append_attribute("u").set_value(u_);
	cornerLocal.append_attribute("v").set_value(v_);
	cornerLocal.append_attribute("z").set_value(zLocal_);
	cornerLocal.append_attribute("height").set_value(height_);
	if (id_)  // Introduced in OpenDRIVE 1.5
		cornerLocal.append_attribute("id").set_value(id_);
}
