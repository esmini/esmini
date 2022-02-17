#include "OpenDrive.hpp"

std::string ReadAttribute(pugi::xml_node node, std::string attribute_name, bool required) {
	if (!strcmp(attribute_name.c_str(), "")) {
		if (required) {
			LOG("Warning: Required but empty attribute");
		}
		return "";
	}

	pugi::xml_attribute attr;

	if ((attr = node.attribute(attribute_name.c_str()))) {
		return attr.value();
	} else {
		if (required) {
			LOG("Warning: missing required attribute: %s -> %s", node.name(), attribute_name.c_str());
		}
	}

	return "";
}

std::shared_ptr<Road> OpenDrive::GetRoadById(int id) {
	for (size_t i = 0; i < road_.size(); i++) {
		if (road_[i]->GetId() == id) {
			return road_[i];
		}
	}
	return 0;
}

std::shared_ptr<Road> OpenDrive::GetRoadByIdx(int idx) {
	if (idx >= 0 && idx < (int)road_.size()) {
		return road_[idx];
	} else {
		return 0;
	}
}

std::shared_ptr<Junction> OpenDrive::GetJunctionById(int id) {
	for (size_t i = 0; i < junction_.size(); i++) {
		if (junction_[i]->GetId() == id) {
			return junction_[i];
		}
	}
	return 0;
}

std::shared_ptr<Junction> OpenDrive::GetJunctionByIdx(int idx) {
	if (idx >= 0 && idx < (int)junction_.size()) {
		return junction_[idx];
	} else {
		LOG("GetJunctionByIdx error (idx %d, njunctions %d)\n", idx, (int)junction_.size());
		return 0;
	}
}

OpenDrive::OpenDrive(const char* filename) {
	if (!LoadOpenDriveFile(filename)) {
		throw std::invalid_argument(std::string("Failed to load OpenDrive file: ") + filename);
	}
}
/*
void OpenDrive::InitGlobalLaneIds() {
	g_Lane_id = 0;
	g_Laneb_id = 0;
}*/

std::shared_ptr<Controller> OpenDrive::GetControllerByIdx(int index) {
	if (index >= 0 && index < controller_.size()) {
		return std::make_shared<Controller>(controller_[index]);
	}

	return 0;
}

std::shared_ptr<Controller> OpenDrive::GetControllerById(int id) {
	// look for this controller in global list
	for (int i = 0; i < GetNumberOfControllers(); i++) {
		if (id == GetControllerByIdx(i)->GetId()) {
			return GetControllerByIdx(i);
		}
	}

	return nullptr;
}

bool OpenDrive::LoadOpenDriveFile(const char* filename, bool replace) {
	if (replace) {
		// InitGlobalLaneIds();
		road_.clear();
		junction_.clear();
	}

	odr_filename_ = filename;

	if (odr_filename_ == "") {
		std::cerr << "Empty file name" << std::endl;
		return false;
	}

	pugi::xml_document doc;

	// First assume absolute path
	pugi::xml_parse_result result = doc.load_file(filename);
	if (!result) {
		LOG("%s at offset (character position): %d", result.description(), result.offset);
		return false;
	}

	pugi::xml_node node = doc.child("OpenDRIVE");
	if (node == NULL) {
		LOG("Invalid OpenDRIVE file, can't find OpenDRIVE element");
		return false;
	}

	// Initialize GeoRef structure
	geo_ref_ = {std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN(),
				"",
				std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN(),
				"",
				"",
				"",
				"",
				std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<double>::quiet_NaN(),
				"",
				"",
				std::numeric_limits<double>::quiet_NaN(),
				std::numeric_limits<int>::quiet_NaN()};

	for (auto child : node.children("userData")) {
		AddUserData(std::make_shared<UserData>(UserData(child)));
	}

	pugi::xml_node header_node = node.child("header");
	header_.revMajor_ = std::atoi(header_node.attribute("revMajor").value());
	header_.revMinor_ = std::atoi(header_node.attribute("revMinor").value());
	header_.name_ = header_node.attribute("name").value();
	header_.version_ = std::atof(header_node.attribute("version").value());
	header_.date_ = header_node.attribute("date").value();
	header_.north_ = std::atof(header_node.attribute("north").value());
	header_.south_ = std::atof(header_node.attribute("south").value());
	header_.east_ = std::atof(header_node.attribute("east").value());
	header_.west_ = std::atof(header_node.attribute("west").value());
	header_.vendor_ = header_node.attribute("vendor").value();

	if (header_node.child("geoReference") != NULL) {
		// Get the string to parse, geoReference tag is just a string with the data separated by spaces and
		// each attribute start with a + character
		std::string geo_ref_str = header_node.child_value("geoReference");
		ParseGeoLocalization(geo_ref_str);
		header_.georeference_ = geo_ref_;
	}

	// TODO: Remove GeoRef from OpenDrive class and use the header container instead.

	pugi::xml_node offset = header_node.child("offset");
	if (offset != NULL) {
		header_.offset_.x_ = std::atof(offset.attribute("x").value());
		header_.offset_.y_ = std::atof(offset.attribute("y").value());
		header_.offset_.z_ = std::atof(offset.attribute("z").value());
		header_.offset_.hdg_ = std::atof(offset.attribute("hdg").value());
	}

	for (auto child : header_node.children("userData")) {
		header_.AddUserData(std::make_shared<UserData>(UserData(child)));
	}

	for (pugi::xml_node road_node = node.child("road"); road_node;
		 road_node = road_node.next_sibling("road")) {
		int rid = atoi(road_node.attribute("id").value());
		std::string rname = road_node.attribute("name").value();
		double roadlength = atof(road_node.attribute("length").value());
		int junction_id = atoi(road_node.attribute("junction").value());
		Road::RoadRule rrule = Road::RoadRule::RIGHT_HAND_TRAFFIC;	// right hand traffic is default

		if (!road_node.attribute("rule").empty()) {
			std::string rule_str = road_node.attribute("rule").value();
			if (rule_str == "LHT" || rule_str == "lht") {
				rrule = Road::RoadRule::LEFT_HAND_TRAFFIC;
			}
		}

		std::shared_ptr<Road> r = std::make_shared<Road>(Road(rid, rname, rrule));
		r->SetLength(roadlength);
		r->SetJunction(junction_id);

		for (auto child : road_node.children("userData")) {
			r->AddUserData(std::make_shared<UserData>(UserData(child)));
		}

		for (pugi::xml_node type_node = road_node.child("type"); type_node;
			 type_node = type_node.next_sibling("type")) {
			std::shared_ptr<Road::RoadTypeEntry> r_type = std::make_shared<Road::RoadTypeEntry>();

			for (auto child : type_node.children("userData")) {
				r_type->AddUserData(std::make_shared<UserData>(UserData(
					child.attribute("code").as_string(), child.attribute("value").as_string(), child)));
			}

			std::string type = type_node.attribute("type").value();
			if (type == "unknown") {
				r_type->road_type_ = Road::RoadType::ROADTYPE_UNKNOWN;
			} else if (type == "rural") {
				r_type->road_type_ = Road::RoadType::ROADTYPE_RURAL;
			} else if (type == "motorway") {
				r_type->road_type_ = Road::RoadType::ROADTYPE_MOTORWAY;
			} else if (type == "town") {
				r_type->road_type_ = Road::RoadType::ROADTYPE_TOWN;
			} else if (type == "lowSpeed") {
				r_type->road_type_ = Road::RoadType::ROADTYPE_LOWSPEED;
			} else if (type == "pedestrian") {
				r_type->road_type_ = Road::RoadType::ROADTYPE_PEDESTRIAN;
			} else if (type == "bicycle") {
				r_type->road_type_ = Road::RoadType::ROADTYPE_BICYCLE;
			} else if (type == "") {
				LOG("Missing road type - setting default (rural)");
				r_type->road_type_ = Road::RoadType::ROADTYPE_RURAL;
			} else {
				LOG("Unsupported road type: %s - assuming rural", type.c_str());
				r_type->road_type_ = Road::RoadType::ROADTYPE_RURAL;
			}

			r_type->s_ = atof(type_node.attribute("s").value());

			// Check for optional speed record
			pugi::xml_node speed = type_node.child("speed");
			if (speed != NULL) {
				r_type->speed_ = atof(speed.attribute("max").value());
				std::string unit = speed.attribute("unit").value();
				if (unit == "km/h") {
					r_type->speed_ /= 3.6;	// Convert to m/s
				} else if (unit == "mph") {
					r_type->speed_ *= 0.44704;	// Convert to m/s
				} else if (unit == "m/s") {
					// SE unit - do nothing
				} else {
					LOG("Unsupported speed unit: %s - assuming SE unit m/s", unit.c_str());
				}
			}

			r->AddRoadType(r_type);
		}

		pugi::xml_node link = road_node.child("link");
		if (link != NULL) {
			pugi::xml_node successor = link.child("successor");
			if (successor != NULL) {
				r->AddLink(std::make_shared<RoadLink>(RoadLink(SUCCESSOR, successor)));

				for (auto child : successor.children("userData")) {
					r->GetLink(SUCCESSOR)->AddUserData(std::make_shared<UserData>(UserData(child)));
				}
			}

			pugi::xml_node predecessor = link.child("predecessor");
			if (predecessor != NULL) {
				r->AddLink(std::make_shared<RoadLink>(RoadLink(PREDECESSOR, predecessor)));

				for (auto child : predecessor.children("userData")) {
					r->GetLink(PREDECESSOR)->AddUserData(std::make_shared<UserData>(UserData(child)));
				}
			}

			if (r->GetJunction() != -1) {
				// As connecting road it is expected to have connections in both ends
				if (successor == NULL) {
					LOG("Warning: connecting road %d in junction %d lacks successor", r->GetId(),
						r->GetJunction());
				}
				if (predecessor == NULL) {
					LOG("Warning: connecting road %d in junction %d lacks predesessor", r->GetId(),
						r->GetJunction());
				}
			}
		}

		pugi::xml_node plan_view = road_node.child("planView");
		if (plan_view != NULL) {
			for (pugi::xml_node geometry = plan_view.child("geometry"); geometry;
				 geometry = geometry.next_sibling()) {
				double s = atof(geometry.attribute("s").value());
				double x = atof(geometry.attribute("x").value());
				double y = atof(geometry.attribute("y").value());
				double hdg = atof(geometry.attribute("hdg").value());
				double glength = atof(geometry.attribute("length").value());

				pugi::xml_node type = geometry.last_child();
				if (type != NULL) {
					// Find out the type of geometry
					if (!strcmp(type.name(), "line")) {
						r->AddLine(std::make_shared<Line>(Line(s, x, y, hdg, glength)));
					} else if (!strcmp(type.name(), "arc")) {
						double curvature = atof(type.attribute("curvature").value());
						r->AddArc(std::make_shared<Arc>(Arc(s, x, y, hdg, glength, curvature)));
					} else if (!strcmp(type.name(), "spiral")) {
						double curv_start = atof(type.attribute("curvStart").value());
						double curv_end = atof(type.attribute("curvEnd").value());
						r->AddSpiral(
							std::make_shared<Spiral>(Spiral(s, x, y, hdg, glength, curv_start, curv_end)));
					} else if (!strcmp(type.name(), "poly3")) {
						double a = atof(type.attribute("a").value());
						double b = atof(type.attribute("b").value());
						double c = atof(type.attribute("c").value());
						double d = atof(type.attribute("d").value());
						r->AddPoly3(std::make_shared<Poly3>(Poly3(s, x, y, hdg, glength, a, b, c, d)));
					} else if (!strcmp(type.name(), "paramPoly3")) {
						double aU = atof(type.attribute("aU").value());
						double bU = atof(type.attribute("bU").value());
						double cU = atof(type.attribute("cU").value());
						double dU = atof(type.attribute("dU").value());
						double aV = atof(type.attribute("aV").value());
						double bV = atof(type.attribute("bV").value());
						double cV = atof(type.attribute("cV").value());
						double dV = atof(type.attribute("dV").value());
						ParamPoly3::PRangeType p_range = ParamPoly3::P_RANGE_NORMALIZED;

						pugi::xml_attribute attr = type.attribute("pRange");
						if (attr && !strcmp(attr.value(), "arcLength")) {
							p_range = ParamPoly3::P_RANGE_ARC_LENGTH;
						}

						auto pp3 = std::make_shared<ParamPoly3>(
							ParamPoly3(s, x, y, hdg, glength, aU, bU, cU, dU, aV, bV, cV, dV, p_range));
						if (pp3 != NULL) {
							r->AddParamPoly3(pp3);
						} else {
							LOG("ParamPoly3: Major error\n");
						}
					} else {
						std::cout << "Unknown geometry type: " << type.name() << std::endl;
						continue;
					}

					for (auto child : type.children("userData")) {
						r->GetGeometry(r->GetNumberOfGeometries() - 1)
							->AddUserData(std::make_shared<UserData>(UserData(child)));
					}
				}
			}
		}

		pugi::xml_node elevation_profile = road_node.child("elevationProfile");
		if (elevation_profile != NULL) {
			for (pugi::xml_node elevation = elevation_profile.child("elevation"); elevation;
				 elevation = elevation.next_sibling()) {
				double s = atof(elevation.attribute("s").value());
				double a = atof(elevation.attribute("a").value());
				double b = atof(elevation.attribute("b").value());
				double c = atof(elevation.attribute("c").value());
				double d = atof(elevation.attribute("d").value());

				auto ep = std::make_shared<Elevation>(Elevation(s, a, b, c, d));
				if (ep != NULL) {
					for (auto child : elevation_profile.children("userData")) {
						ep->AddUserData(std::make_shared<UserData>(UserData(child)));
					}
					r->AddElevation(ep);
				} else {
					LOG("Elevation: Major error\n");
				}
			}
		}

		pugi::xml_node super_elevation_profile = road_node.child("lateralProfile");
		if (super_elevation_profile != NULL) {
			for (pugi::xml_node super_elevation = super_elevation_profile.child("superelevation");
				 super_elevation; super_elevation = super_elevation.next_sibling("superelevation")) {
				double s = atof(super_elevation.attribute("s").value());
				double a = atof(super_elevation.attribute("a").value());
				double b = atof(super_elevation.attribute("b").value());
				double c = atof(super_elevation.attribute("c").value());
				double d = atof(super_elevation.attribute("d").value());

				auto ep = std::make_shared<Elevation>(s, a, b, c, d);
				if (ep != NULL) {
					for (auto child : elevation_profile.children("userData")) {
						ep->AddUserData(std::make_shared<UserData>(UserData(child)));
					}
					r->AddSuperElevation(ep);
				} else {
					LOG("SuperElevation: Major error\n");
				}
			}
		}

		pugi::xml_node lanes = road_node.child("lanes");
		if (lanes != NULL) {
			for (pugi::xml_node_iterator child = lanes.children().begin(); child != lanes.children().end();
				 child++) {
				if (!strcmp(child->name(), "laneOffset")) {
					double s = atof(child->attribute("s").value());
					double a = atof(child->attribute("a").value());
					double b = atof(child->attribute("b").value());
					double c = atof(child->attribute("c").value());
					double d = atof(child->attribute("d").value());
					r->AddLaneOffset(std::make_shared<LaneOffset>(LaneOffset(s, a, b, c, d)));

					for (auto userData : child->children("userData")) {
						r->GetLaneOffsetByIdx(r->GetNumberOfLaneOffsets() - 1)
							->AddUserData(std::make_shared<UserData>(UserData(userData)));
					}
				} else if (!strcmp(child->name(), "laneSection")) {
					double s = atof(child->attribute("s").value());
					auto lane_section = std::make_shared<LaneSection>(LaneSection(s));

					for (auto userData : child->children("userData")) {
						lane_section->AddUserData(std::make_shared<UserData>(UserData(userData)));
					}

					if (child->attribute("singleSide").value() == "true") {
						lane_section->SetSingleSide(true);
					} else if (!strcmp(child->attribute("singleSide").value(), "false")) {
						lane_section->SetSingleSide(false);
					}

					r->AddLaneSection(lane_section);

					for (pugi::xml_node_iterator child2 = child->children().begin();
						 child2 != child->children().end(); child2++) {
						if (!strcmp(child2->name(), "left")) {
							// LOG("Lane left\n");
						} else if (!strcmp(child2->name(), "right")) {
							// LOG("Lane right\n");
						} else if (!strcmp(child2->name(), "center")) {
							// LOG("Lane center\n");
						} else if (!strcmp(child2->name(), "userData")) {
							// Not supported
							continue;
						} else {
							LOG("Unsupported lane side: %s\n", child2->name());
							continue;
						}
						for (pugi::xml_node_iterator lane_node = child2->children().begin();
							 lane_node != child2->children().end(); lane_node++) {
							if (strcmp(lane_node->name(), "lane")) {
								LOG("Unexpected element: %s, expected \"lane\"\n", lane_node->name());
								continue;
							}

							Lane::LaneType lane_type = Lane::LANE_TYPE_NONE;
							if (lane_node->attribute("type") == 0
								|| !strcmp(lane_node->attribute("type").value(), "")) {
								LOG("Lane type error");
							}
							if (!strcmp(lane_node->attribute("type").value(), "none")) {
								lane_type = Lane::LANE_TYPE_NONE;
							} else if (!strcmp(lane_node->attribute("type").value(), "driving")) {
								lane_type = Lane::LANE_TYPE_DRIVING;
							} else if (!strcmp(lane_node->attribute("type").value(), "stop")) {
								lane_type = Lane::LANE_TYPE_STOP;
							} else if (!strcmp(lane_node->attribute("type").value(), "shoulder")) {
								lane_type = Lane::LANE_TYPE_SHOULDER;
							} else if (!strcmp(lane_node->attribute("type").value(), "biking")) {
								lane_type = Lane::LANE_TYPE_BIKING;
							} else if (!strcmp(lane_node->attribute("type").value(), "sidewalk")) {
								lane_type = Lane::LANE_TYPE_SIDEWALK;
							} else if (!strcmp(lane_node->attribute("type").value(), "border")) {
								lane_type = Lane::LANE_TYPE_BORDER;
							} else if (!strcmp(lane_node->attribute("type").value(), "restricted")) {
								lane_type = Lane::LANE_TYPE_RESTRICTED;
							} else if (!strcmp(lane_node->attribute("type").value(), "parking")) {
								lane_type = Lane::LANE_TYPE_PARKING;
							} else if (!strcmp(lane_node->attribute("type").value(), "bidirectional")) {
								lane_type = Lane::LANE_TYPE_BIDIRECTIONAL;
							} else if (!strcmp(lane_node->attribute("type").value(), "median")) {
								lane_type = Lane::LANE_TYPE_MEDIAN;
							} else if (!strcmp(lane_node->attribute("type").value(), "special1")) {
								lane_type = Lane::LANE_TYPE_SPECIAL1;
							} else if (!strcmp(lane_node->attribute("type").value(), "special2")) {
								lane_type = Lane::LANE_TYPE_SPECIAL2;
							} else if (!strcmp(lane_node->attribute("type").value(), "special3")) {
								lane_type = Lane::LANE_TYPE_SPECIAL3;
							} else if (!strcmp(lane_node->attribute("type").value(), "roadWorks")) {
								lane_type = Lane::LANE_TYPE_ROADWORKS;
							} else if (!strcmp(lane_node->attribute("type").value(), "tram")) {
								lane_type = Lane::LANE_TYPE_TRAM;
							} else if (!strcmp(lane_node->attribute("type").value(), "rail")) {
								lane_type = Lane::LANE_TYPE_RAIL;
							} else if (!strcmp(lane_node->attribute("type").value(), "entry")
									   || !strcmp(lane_node->attribute("type").value(), "mwyEntry")) {
								lane_type = Lane::LANE_TYPE_ENTRY;
							} else if (!strcmp(lane_node->attribute("type").value(), "exit")
									   || !strcmp(lane_node->attribute("type").value(), "mwyExit")) {
								lane_type = Lane::LANE_TYPE_EXIT;
							} else if (!strcmp(lane_node->attribute("type").value(), "offRamp")) {
								lane_type = Lane::LANE_TYPE_OFF_RAMP;
							} else if (!strcmp(lane_node->attribute("type").value(), "onRamp")) {
								lane_type = Lane::LANE_TYPE_ON_RAMP;
							} else {
								LOG("unknown lane type: %s (road id=%d)\n",
									lane_node->attribute("type").value(), r->GetId());
							}

							int lane_id = atoi(lane_node->attribute("id").value());

							// If lane ID == 0, make sure it's not a driving lane
							if (lane_id == 0 && lane_type == Lane::LANE_TYPE_DRIVING) {
								lane_type = Lane::LANE_TYPE_NONE;
							}

							auto lane = std::make_shared<Lane>(Lane(lane_id, lane_type));
							if (lane == NULL) {
								LOG("Error: creating lane\n");
								return false;
							}

							for (auto child : lane_node->children("userData")) {
								lane->AddUserData(std::make_shared<UserData>(UserData(child)));
							}

							if (!strcmp(lane_node->attribute("level").value(), "true")) {
								lane->SetLevel(1);
							} else if (!strcmp(lane_node->attribute("level").value(), "false")) {
								lane->SetLevel(0);
							}

							lane_section->AddLane(lane);

							// Link
							pugi::xml_node lane_link = lane_node->child("link");
							if (lane_link != NULL) {
								pugi::xml_node predecessor = lane_link.child("predecessor");
								if (predecessor != NULL) {
									lane->AddLink(std::make_shared<LaneLink>(
										LaneLink(PREDECESSOR, atoi(predecessor.attribute("id").value()))));
									for (auto child : predecessor.children("userData")) {
										lane->GetLink(PREDECESSOR)
											->AddUserData(std::make_shared<UserData>(UserData(child)));
									}
								}

								pugi::xml_node successor = lane_link.child("successor");
								if (successor != NULL) {
									lane->AddLink(std::make_shared<LaneLink>(
										LaneLink(SUCCESSOR, atoi(successor.attribute("id").value()))));
									for (auto child : successor.children("userData")) {
										lane->GetLink(SUCCESSOR)->AddUserData(
											std::make_shared<UserData>(UserData(child)));
									}
								}
							}

							// Width
							for (pugi::xml_node width = lane_node->child("width"); width;
								 width = width.next_sibling("width")) {
								double s_offset = atof(width.attribute("sOffset").value());
								double a = atof(width.attribute("a").value());
								double b = atof(width.attribute("b").value());
								double c = atof(width.attribute("c").value());
								double d = atof(width.attribute("d").value());
								lane->AddLaneWidth(
									std::make_shared<LaneWidth>(LaneWidth(s_offset, a, b, c, d)));
								for (auto child : width.children("userData")) {
									lane->GetWidthByIndex(lane->GetNumberOfLaneWidths() - 1)
										->AddUserData(std::make_shared<UserData>(UserData(child)));
								}
							}

							// roadMark
							for (pugi::xml_node roadMark = lane_node->child("roadMark"); roadMark;
								 roadMark = roadMark.next_sibling("roadMark")) {
								// s_offset
								double s_offset = atof(roadMark.attribute("sOffset").value());

								// type
								LaneRoadMark::RoadMarkType roadMark_type = LaneRoadMark::NONE_TYPE;
								if (roadMark.attribute("type") == 0
									|| !strcmp(roadMark.attribute("type").value(), "")) {
									LOG("Lane road mark type error");
								}
								if (!strcmp(roadMark.attribute("type").value(), "none")) {
									roadMark_type = LaneRoadMark::NONE_TYPE;
								} else if (!strcmp(roadMark.attribute("type").value(), "solid")) {
									roadMark_type = LaneRoadMark::SOLID;
								} else if (!strcmp(roadMark.attribute("type").value(), "broken")) {
									roadMark_type = LaneRoadMark::BROKEN;
								} else if (!strcmp(roadMark.attribute("type").value(), "solid solid")) {
									roadMark_type = LaneRoadMark::SOLID_SOLID;
								} else if (!strcmp(roadMark.attribute("type").value(), "solid broken")) {
									roadMark_type = LaneRoadMark::SOLID_BROKEN;
								} else if (!strcmp(roadMark.attribute("type").value(), "broken solid")) {
									roadMark_type = LaneRoadMark::BROKEN_SOLID;
								} else if (!strcmp(roadMark.attribute("type").value(), "broken broken")) {
									roadMark_type = LaneRoadMark::BROKEN_BROKEN;
								} else if (!strcmp(roadMark.attribute("type").value(), "botts dots")) {
									roadMark_type = LaneRoadMark::BOTTS_DOTS;
								} else if (!strcmp(roadMark.attribute("type").value(), "grass")) {
									roadMark_type = LaneRoadMark::GRASS;
								} else if (!strcmp(roadMark.attribute("type").value(), "curb")) {
									roadMark_type = LaneRoadMark::CURB;
								} else {
									LOG("unknown lane road mark type: %s (road id=%d)\n",
										roadMark.attribute("type").value(), r->GetId());
								}

								// weight - consider it optional with default value = STANDARD
								LaneRoadMark::RoadMarkWeight roadMark_weight = LaneRoadMark::STANDARD;
								if (roadMark.attribute("weight") != 0
									&& strcmp(roadMark.attribute("weight").value(), "")) {
									if (!strcmp(roadMark.attribute("weight").value(), "standard")) {
										roadMark_weight = LaneRoadMark::STANDARD;
									} else if (!strcmp(roadMark.attribute("weight").value(), "bold")) {
										roadMark_weight = LaneRoadMark::BOLD;
									} else {
										LOG("unknown lane road mark weight: %s (road id=%d) setting to "
											"standard\n",
											roadMark.attribute("type").value(), r->GetId());
										roadMark_weight = LaneRoadMark::STANDARD;
									}
								}

								// color - consider it optional with default value = STANDARD_COLOR
								RoadMarkColor roadMark_color = LaneRoadMark::ParseColor(roadMark);
								if (roadMark_color == RoadMarkColor::UNDEFINED) {
									LOG("unknown lane road mark color: %s (road id=%d), set to standard "
										"(white)",
										roadMark_color, r->GetId());
									roadMark_color = RoadMarkColor::STANDARD_COLOR;
								}

								// material
								LaneRoadMark::RoadMarkMaterial roadMark_material
									= LaneRoadMark::STANDARD_MATERIAL;

								// optional laneChange. If it's missing Both should be valid
								LaneRoadMark::RoadMarkLaneChange roadMark_laneChange = LaneRoadMark::BOTH;
								if (!roadMark.attribute("laneChange").empty()) {
									if (!strcmp(roadMark.attribute("laneChange").value(), "")) {
										LOG("Lane roadmark lanechange error");
									} else {
										if (!strcmp(roadMark.attribute("laneChange").value(), "none")) {
											roadMark_laneChange = LaneRoadMark::NONE_LANECHANGE;
										} else if (!strcmp(roadMark.attribute("laneChange").value(),
														   "increase")) {
											roadMark_laneChange = LaneRoadMark::INCREASE;
										} else if (!strcmp(roadMark.attribute("laneChange").value(),
														   "decrease")) {
											roadMark_laneChange = LaneRoadMark::DECREASE;
										} else if (!strcmp(roadMark.attribute("laneChange").value(),
														   "both")) {
											roadMark_laneChange = LaneRoadMark::BOTH;
										} else {
											LOG("unknown lane road mark lane change: %s (road id=%d)\n",
												roadMark.attribute("laneChange").value(), r->GetId());
										}
									}
								}

								double roadMark_width;
								if (roadMark.attribute("width").empty()) {
									roadMark_width = (roadMark_weight == LaneRoadMark::BOLD)
														 ? ROADMARK_WIDTH_BOLD
														 : ROADMARK_WIDTH_STANDARD;
								} else {
									roadMark_width = atof(roadMark.attribute("width").value());
								}

								double roadMark_height = atof(roadMark.attribute("height").value());
								auto lane_roadMark = std::make_shared<LaneRoadMark>(LaneRoadMark(
									s_offset, roadMark_type, roadMark_weight, roadMark_color,
									roadMark_material, roadMark_laneChange, roadMark_width, roadMark_height));

								for (auto child : roadMark.children("userData")) {
									lane_roadMark->AddUserData(std::make_shared<UserData>(UserData(child)));
								}

								lane->AddLaneRoadMark(lane_roadMark);
								// sub_type
								std::shared_ptr<LaneRoadMarkType> lane_roadMarkType = 0;
								for (pugi::xml_node sub_type = roadMark.child("type"); sub_type;
									 sub_type = sub_type.next_sibling("type")) {
									if (sub_type != NULL) {
										std::string sub_type_name = sub_type.attribute("name").value();
										double sub_type_width = atof(sub_type.attribute("width").value());
										lane_roadMarkType = std::make_shared<LaneRoadMarkType>(
											LaneRoadMarkType(sub_type_name, sub_type_width));
										for (auto child : sub_type.children("userData")) {
											lane_roadMarkType->AddUserData(
												std::make_shared<UserData>(UserData(child)));
										}
										lane_roadMark->AddType(lane_roadMarkType);

										for (pugi::xml_node line = sub_type.child("line"); line;
											 line = line.next_sibling("line")) {
											double llength = atof(line.attribute("length").value());
											double space = atof(line.attribute("space").value());
											double t_offset = atof(line.attribute("tOffset").value());
											double s_offset_l = atof(line.attribute("sOffset").value());

											if (!line.attribute("color").empty()) {
												RoadMarkColor tmp_color = LaneRoadMark::ParseColor(line);
												if (tmp_color != RoadMarkColor::UNDEFINED) {
													roadMark_color
														= tmp_color;  // supersedes the setting in <RoadMark>
																	  // element (available from odr v1.5)
												}
											}

											// rule (optional)
											LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule
												= LaneRoadMarkTypeLine::NONE;
											if (line.attribute("rule") != 0
												&& strcmp(line.attribute("rule").value(), "")) {
												if (!strcmp(line.attribute("rule").value(), "none")) {
													rule = LaneRoadMarkTypeLine::NONE;
												} else if (!strcmp(line.attribute("rule").value(),
																   "caution")) {
													rule = LaneRoadMarkTypeLine::CAUTION;
												} else if (!strcmp(line.attribute("rule").value(),
																   "no passing")) {
													rule = LaneRoadMarkTypeLine::NO_PASSING;
												} else {
													LOG("unknown lane road mark type line rule: %s (road "
														"id=%d)\n",
														line.attribute("rule").value(), r->GetId());
												}
											}

											double width = atof(line.attribute("width").value());

											auto lane_roadMarkTypeLine
												= std::make_shared<LaneRoadMarkTypeLine>(
													LaneRoadMarkTypeLine(llength, space, t_offset, s_offset_l,
																		 rule, width, roadMark_color));

											for (auto child : line.children("userData")) {
												lane_roadMarkTypeLine->AddUserData(
													std::make_shared<UserData>(UserData(child)));
											}
											lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
										}
									}
								}
								if (roadMark_type != LaneRoadMark::NONE_TYPE && lane_roadMarkType == 0) {
									if (roadMark_type == LaneRoadMark::SOLID
										|| roadMark_type == LaneRoadMark::CURB) {
										lane_roadMarkType = std::make_shared<LaneRoadMarkType>(
											LaneRoadMarkType("stand-in", roadMark_width));
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule
											= LaneRoadMarkTypeLine::NONE;
										auto lane_roadMarkTypeLine
											= std::make_shared<LaneRoadMarkTypeLine>(LaneRoadMarkTypeLine(
												0, 0, 0, 0, rule, roadMark_width, roadMark_color));
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
									} else if (roadMark_type == LaneRoadMark::SOLID_SOLID) {
										lane_roadMarkType = std::make_shared<LaneRoadMarkType>(
											LaneRoadMarkType("stand-in", roadMark_width));
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule
											= LaneRoadMarkTypeLine::NONE;
										auto lane_roadMarkTypeLine = std::make_shared<LaneRoadMarkTypeLine>(
											LaneRoadMarkTypeLine(0, 0, -roadMark_width, 0, rule,
																 roadMark_width, roadMark_color));
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
										auto lane_roadMarkTypeLine2 = std::make_shared<LaneRoadMarkTypeLine>(
											LaneRoadMarkTypeLine(0, 0, roadMark_width, 0, rule,
																 roadMark_width, roadMark_color));
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine2);
									} else if (roadMark_type == LaneRoadMark::BROKEN) {
										lane_roadMarkType = std::make_shared<LaneRoadMarkType>(
											LaneRoadMarkType("stand-in", roadMark_width));
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule
											= LaneRoadMarkTypeLine::NONE;
										auto lane_roadMarkTypeLine
											= std::make_shared<LaneRoadMarkTypeLine>(LaneRoadMarkTypeLine(
												4, 8, 0, 0, rule, roadMark_width, roadMark_color));
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
									} else if (roadMark_type == LaneRoadMark::BROKEN_BROKEN) {
										lane_roadMarkType = std::make_shared<LaneRoadMarkType>(
											LaneRoadMarkType("stand-in", roadMark_width));
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule
											= LaneRoadMarkTypeLine::NONE;
										auto lane_roadMarkTypeLine = std::make_shared<LaneRoadMarkTypeLine>(
											LaneRoadMarkTypeLine(4, 8, -roadMark_width, 0, rule,
																 roadMark_width, roadMark_color));
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
										auto lane_roadMarkTypeLine2 = std::make_shared<LaneRoadMarkTypeLine>(
											LaneRoadMarkTypeLine(4, 8, roadMark_width, 0, rule,
																 roadMark_width, roadMark_color));
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine2);
									} else {
										LOG("No road mark created for road %d lane %d. Type %d not "
											"supported. Either switch type or add a roadMark <type> element.",
											r->GetId(), lane_id, roadMark_type);
									}
								}
							}

							// Lane Material
							for (pugi::xml_node laneMaterial = lane_node->child("material"); laneMaterial;
								 laneMaterial = laneMaterial.next_sibling("material")) {
								double s_offset = atof(laneMaterial.attribute("sOffset").value());
								std::string surface = laneMaterial.attribute("surface").as_string();
								double friction = atof(laneMaterial.attribute("friction").value());
								double roughness = atof(laneMaterial.attribute("roughness").value());
								auto material = std::make_shared<LaneMaterial>(
									LaneMaterial(s_offset, surface, friction, roughness));
								for (auto child : laneMaterial.children("userData")) {
									material->AddUserData(std::make_shared<UserData>(UserData(child)));
								}
								lane->AddLaneMaterial(material);
							}

							// Lane Speed
							for (pugi::xml_node laneSpeed = lane_node->child("speed"); laneSpeed;
								 laneSpeed = laneSpeed.next_sibling("speed")) {
								double s_offset = atof(laneSpeed.attribute("sOffset").value());
								std::string unit = laneSpeed.attribute("unit").as_string();
								double max = atof(laneSpeed.attribute("max").value());

								auto speed = std::make_shared<LaneSpeed>(LaneSpeed(s_offset, max, unit));
								for (auto child : laneSpeed.children("userData")) {
									speed->AddUserData(std::make_shared<UserData>(UserData(child)));
								}
								lane->AddLaneSpeed(speed);
							}
						}
					}
					// Check lane indices
					int lastLaneId = 0;
					for (int i = 0; i < lane_section->GetNumberOfLanes(); i++) {
						int laneId = lane_section->GetLaneByIdx(i)->GetId();
						if (i > 0 && laneId != lastLaneId - 1) {
							LOG("Warning: expected laneId %d missing of roadId %d. Found laneIds %d and %d",
								lastLaneId - 1, r->GetId(), lastLaneId, laneId);
						}
						lastLaneId = laneId;
					}
				} else {
					LOG("Unsupported lane type: %s\n", child->name());
				}
			}
		}

		pugi::xml_node signals = road_node.child("signals");
		if (signals != NULL) {
			// Variables to check if the country file is loaded
			bool country_file_loaded = false;
			std::string current_country = "";
			for (pugi::xml_node signal = signals.child("signal"); signal; signal = signal.next_sibling()) {
				if (!strcmp(signal.name(), "signal")) {
					double s = atof(signal.attribute("s").value());
					double t = atof(signal.attribute("t").value());
					int ids = atoi(signal.attribute("id").value());
					std::string name = signal.attribute("name").value();

					// dynamic
					bool dynamic = false;
					if (!strcmp(signal.attribute("dynamic").value(), "")) {
						LOG("Signal dynamic check error");
					}
					if (!strcmp(signal.attribute("dynamic").value(), "no")) {
						dynamic = false;
					} else if (!strcmp(signal.attribute("dynamic").value(), "yes")) {
						dynamic = true;
					} else {
						LOG("unknown dynamic signal identification: %s (road ids=%d)\n",
							signal.attribute("dynamic").value(), r->GetId());
					}

					// orientation
					Signal::Orientation orientation = Signal::NONE;
					if (signal.attribute("orientation") == 0
						|| !strcmp(signal.attribute("orientation").value(), "")) {
						LOG("Road signal orientation error");
					}
					if (!strcmp(signal.attribute("orientation").value(), "none")) {
						orientation = Signal::NONE;
					} else if (!strcmp(signal.attribute("orientation").value(), "+")) {
						orientation = Signal::POSITIVE;
					} else if (!strcmp(signal.attribute("orientation").value(), "-")) {
						orientation = Signal::NEGATIVE;
					} else {
						LOG("unknown road signal orientation: %s (road ids=%d)\n",
							signal.attribute("orientation").value(), r->GetId());
					}

					double z_offset = atof(signal.attribute("zOffset").value());
					std::string country = signal.attribute("country").value();
					// Load the country file for types
					if (!country_file_loaded || current_country != country) {
						current_country = country;
						country_file_loaded = LoadSignalsByCountry(country);
					}

					// type
					int type = Signal::TYPE_UNKNOWN;
					if (signal.attribute("type") == 0 || !strcmp(signal.attribute("type").value(), "")) {
						LOG("Road signal type error");
					}
					// sub_type
					if (signal.attribute("subtype") == 0
						|| !strcmp(signal.attribute("subtype").value(), "")) {
						LOG("Road signal sub-type error");
					}

					if (strcmp(signal.attribute("type").value(), "none")
						&& strcmp(signal.attribute("type").value(), "-1")) {
						std::string type_to_find = signal.attribute("type").value();
						if (strcmp(signal.attribute("subtype").value(), "none")
							&& strcmp(signal.attribute("subtype").value(), "-1")) {
							type_to_find = type_to_find + "-" + signal.attribute("subtype").value();
						}

						if (signals_types_.count(type_to_find) != 0) {
							std::string enum_string = signals_types_.find(type_to_find)->second;
							type = static_cast<int>(Signal::GetTypeFromString(enum_string));
						} else {
							LOG("Signal Type %s doesn't exists for this country", type_to_find.c_str());
						}
					}

					double value = atof(signal.attribute("value").value());
					std::string unit = signal.attribute("unit").value();
					double height = atof(signal.attribute("height").value());
					double width = atof(signal.attribute("width").value());
					std::string text = signal.attribute("text").value();
					double h_offset = atof(signal.attribute("hOffset").value());
					double pitch = atof(signal.attribute("pitch").value());
					double roll = atof(signal.attribute("roll").value());

					auto sig = std::make_shared<Signal>(Signal(s, t, ids, name, dynamic, orientation,
															   z_offset, country, type, value, unit, height,
															   width, text, h_offset, pitch, roll));
					if (sig != NULL) {
						for (auto child : signal.children("userData")) {
							sig->AddUserData(std::make_shared<UserData>(UserData(child)));
						}
						r->AddSignal(sig);
					} else {
						LOG("Signal: Major error\n");
					}

					for (pugi::xml_node validity_node = signal.child("validity"); validity_node;
						 validity_node = validity_node.next_sibling("validity")) {
						ValidityRecord validity;
						validity.fromLane_ = atoi(validity_node.attribute("fromLane").value());
						validity.toLane_ = atoi(validity_node.attribute("toLane").value());
						for (auto child : validity_node.children("userData")) {
							validity.AddUserData(std::make_shared<UserData>(UserData(child)));
						}
						sig->validity_.push_back(validity);
					}
				} else {
					LOG_ONCE("INFO: signal element \"%s\" not supported yet", signal.name());
				}
			}
		}

		pugi::xml_node objects = road_node.child("objects");
		if (objects != NULL) {
			for (pugi::xml_node object = objects.child("object"); object;
				 object = object.next_sibling("object")) {
				// Read any repeat element first, since its s-value overrides the one in the object element
				std::shared_ptr<Repeat> repeat = 0;
				std::vector<std::shared_ptr<Repeat>> repeats;
				for (pugi::xml_node repeat_node = object.child("repeat"); repeat_node;
					 repeat_node = repeat_node.next_sibling("repeat")) {
					std::string rattr;
					double rs
						= (rattr = ReadAttribute(repeat_node, "s", true)) == "" ? 0.0 : std::stod(rattr);
					double rlength
						= (rattr = ReadAttribute(repeat_node, "length", true)) == "" ? 0.0 : std::stod(rattr);
					double rdistance = (rattr = ReadAttribute(repeat_node, "distance", true)) == ""
										   ? 0.0
										   : std::stod(rattr);
					double rtStart
						= (rattr = ReadAttribute(repeat_node, "tStart", true)) == "" ? 0.0 : std::stod(rattr);
					double rtEnd
						= (rattr = ReadAttribute(repeat_node, "tEnd", true)) == "" ? 0.0 : std::stod(rattr);
					double rheightStart = (rattr = ReadAttribute(repeat_node, "heightStart", true)) == ""
											  ? 0.0
											  : std::stod(rattr);
					double rheightEnd = (rattr = ReadAttribute(repeat_node, "heightEnd", true)) == ""
											? 0.0
											: std::stod(rattr);
					double rzOffsetStart = (rattr = ReadAttribute(repeat_node, "zOffsetStart", true)) == ""
											   ? 0.0
											   : std::stod(rattr);
					double rzOffsetEnd = (rattr = ReadAttribute(repeat_node, "zOffsetEnd", true)) == ""
											 ? 0.0
											 : std::stod(rattr);

					double rwidthStart = (rattr = ReadAttribute(repeat_node, "widthStart", false)) == ""
											 ? 0.0
											 : std::stod(rattr);
					double rwidthEnd = (rattr = ReadAttribute(repeat_node, "widthEnd", false)) == ""
										   ? 0.0
										   : std::stod(rattr);
					double rlengthStart = (rattr = ReadAttribute(repeat_node, "lengthStart", false)) == ""
											  ? 0.0
											  : std::stod(rattr);
					double rlengthEnd = (rattr = ReadAttribute(repeat_node, "lengthEnd", false)) == ""
											? 0.0
											: std::stod(rattr);
					double rradiusStart = (rattr = ReadAttribute(repeat_node, "radiusStart", false)) == ""
											  ? 0.0
											  : std::stod(rattr);
					double rradiusEnd = (rattr = ReadAttribute(repeat_node, "radiusEnd", false)) == ""
											? 0.0
											: std::stod(rattr);

					repeat = std::make_shared<Repeat>(Repeat(rs, rlength, rdistance, rtStart, rtEnd,
															 rheightStart, rheightEnd, rzOffsetStart,
															 rzOffsetEnd));

					if (fabs(rwidthStart) > SMALL_NUMBER)
						repeat->SetWidthStart(rwidthStart);
					if (fabs(rwidthEnd) > SMALL_NUMBER)
						repeat->SetWidthEnd(rwidthEnd);
					if (fabs(rlengthStart) > SMALL_NUMBER)
						repeat->SetLengthStart(rlengthStart);
					if (fabs(rlengthEnd) > SMALL_NUMBER)
						repeat->SetLengthEnd(rlengthEnd);

					if (fabs(rradiusStart) > SMALL_NUMBER)
						printf("Attribute object/repeat/radiusStart not supported yet\n");
					if (fabs(rradiusEnd) > SMALL_NUMBER)
						printf("Attribute object/repeat/radiusEnd not supported yet\n");
					for (auto child : repeat_node.children("userData")) {
						repeat->AddUserData(std::make_shared<UserData>(UserData(child)));
					}
					repeats.push_back(repeat);
				}

				double s;
				if (repeat) {
					s = repeats[0]->GetS();
				} else {
					s = atof(object.attribute("s").value());
				}
				double t = atof(object.attribute("t").value());
				int ids = atoi(object.attribute("id").value());
				std::string name = object.attribute("name").value();

				// orientation
				RMObject::Orientation orientation = RMObject::Orientation::NONE;
				if (object.attribute("orientation") != 0
					&& strcmp(object.attribute("orientation").value(), "")) {
					if (!strcmp(object.attribute("orientation").value(), "none")) {
						orientation = RMObject::Orientation::NONE;
					} else if (!strcmp(object.attribute("orientation").value(), "+")) {
						orientation = RMObject::Orientation::POSITIVE;
					} else if (!strcmp(object.attribute("orientation").value(), "-")) {
						orientation = RMObject::Orientation::NEGATIVE;
					} else {
						LOG("unknown road object orientation: %s (road ids=%d)\n",
							object.attribute("orientation").value(), r->GetId());
					}
				}
				std::string type = object.attribute("type").value();
				double z_offset = atof(object.attribute("zOffset").value());
				double length = atof(object.attribute("length").value());
				double height = atof(object.attribute("height").value());
				double width = atof(object.attribute("width").value());
				double heading = atof(object.attribute("hdg").value());
				double pitch = atof(object.attribute("pitch").value());
				double roll = atof(object.attribute("roll").value());

				auto obj = std::make_shared<RMObject>(RMObject(s, t, ids, name, orientation, z_offset, type,
															   length, height, width, heading, pitch, roll));
				for (auto child : object.children("userData")) {
					obj->AddUserData(std::make_shared<UserData>(UserData(child)));
				}
				if (repeat) {
					for (auto rep : repeats) {
						obj->AddRepeat(rep);
					}
				}

				float version
					= stof(std::to_string(header_.revMajor_) + "." + std::to_string(header_.revMinor_));

				pugi::xml_node outlines_node;
				if (version > 1.4) {
					outlines_node = object.child("outlines");
				} else {
					outlines_node = object;
				}

				if (outlines_node != NULL) {
					for (pugi::xml_node outline_node = outlines_node.child("outline"); outline_node;
						 outline_node = outline_node.next_sibling()) {
						auto outlineFillType = outlines_node.attribute("fillType").value();
						int id = atoi(outline_node.attribute("id").value());
						bool closed
							= !strcmp(outline_node.attribute("closed").value(), "true") ? true : false;
						auto outline = std::make_shared<Outline>(
							Outline(id, Outline::FillType::FILL_TYPE_UNDEFINED, closed));

						for (auto child : outline_node.children("userData")) {
							outline->AddUserData(std::make_shared<UserData>(UserData(child)));
						}

						for (pugi::xml_node corner_node = outline_node.first_child(); corner_node;
							 corner_node = corner_node.next_sibling()) {
							std::shared_ptr<OutlineCorner> corner = 0;

							if (!strcmp(corner_node.name(), "cornerRoad")) {
								double sc = atof(corner_node.attribute("s").value());
								double tc = atof(corner_node.attribute("t").value());
								double dz = atof(corner_node.attribute("dz").value());
								double heightc = atof(corner_node.attribute("height").value());
								int id = atoi(corner_node.attribute("id").value());

								corner = std::make_shared<OutlineCornerRoad>(
									OutlineCornerRoad(id, r->GetId(), sc, tc, dz, heightc));
							} else if (!strcmp(corner_node.name(), "cornerLocal")) {
								double u = atof(corner_node.attribute("u").value());
								double v = atof(corner_node.attribute("v").value());
								double zLocal = atof(corner_node.attribute("z").value());
								double heightc = atof(corner_node.attribute("height").value());
								int id = atoi(corner_node.attribute("id").value());

								corner = std::make_shared<OutlineCornerLocal>(
									OutlineCornerLocal(id, r->GetId(), obj->GetS(), obj->GetT(), u, v, zLocal,
													   heightc, heading));
							}
							for (auto child : corner_node.children("userData")) {
								corner->AddUserData(std::make_shared<UserData>(UserData(child)));
							}
							outline->AddCorner(corner);
						}

						obj->AddOutline(outline);
					}
				}

				for (pugi::xml_node validity_node = object.child("validity"); validity_node;
					 validity_node = validity_node.next_sibling("validity")) {
					ValidityRecord validity;
					validity.fromLane_ = atoi(validity_node.attribute("fromLane").value());
					validity.toLane_ = atoi(validity_node.attribute("toLane").value());

					for (auto child : validity_node.children("userData")) {
						validity.AddUserData(std::make_shared<UserData>(UserData(child)));
					}

					obj->validity_.push_back(validity);
				}

				if (obj != NULL) {
					r->AddObject(obj);
				} else {
					LOG("RMObject: Major error\n");
				}
			}

			pugi::xml_node bridge = objects.child("bridge");
			if (bridge != NULL) {
				for (bridge; bridge; bridge = bridge.next_sibling("bridge")) {
					double s = atof(bridge.attribute("s").value());
					double length = atof(bridge.attribute("length").value());
					std::string name = bridge.attribute("name").as_string();
					int id = atoi(bridge.attribute("id").value());
					Bridge::Type type;

					if (!strcmp(bridge.attribute("type").value(), "concrete")) {
						type = Bridge::Type::CONCRETE;
					} else if (!strcmp(bridge.attribute("type").value(), "steel")) {
						type = Bridge::Type::STEEL;
					} else if (!strcmp(bridge.attribute("type").value(), "brick")) {
						type = Bridge::Type::BRICK;
					} else if (!strcmp(bridge.attribute("type").value(), "wood")) {
						type = Bridge::Type::WOOD;
					} else {
						type = Bridge::Type::UNKNOWN;
						LOG("unknown bridge type: %s (road ids=%d)\n", bridge.attribute("type").value(),
							r->GetId());
					}

					auto brdg = std::make_shared<Bridge>(Bridge(s, length, name, id, type));

					for (auto child : bridge.children("userData")) {
						brdg->AddUserData(std::make_shared<UserData>(UserData(child)));
					}

					for (pugi::xml_node validity_node = bridge.child("validity"); validity_node;
						 validity_node = validity_node.next_sibling("validity")) {
						ValidityRecord validity;
						validity.fromLane_ = atoi(validity_node.attribute("fromLane").value());
						validity.toLane_ = atoi(validity_node.attribute("toLane").value());
						for (auto child : validity_node.children("userData")) {
							validity.AddUserData(std::make_shared<UserData>(UserData(child)));
						}
						brdg->validity_.push_back(validity);
					}

					if (brdg != NULL) {
						r->AddBridge(brdg);
					} else {
						LOG("Bridge: Major error\n");
					}
				}
			}

			pugi::xml_node object_reference = objects.child("objectReference");
			if (object_reference != NULL) {
				for (object_reference; object_reference;
					 object_reference = object_reference.next_sibling("objectReference")) {
					double s = atof(object_reference.attribute("s").value());
					double t = atof(object_reference.attribute("t").value());
					int id = atoi(object_reference.attribute("id").value());
					double zOffset = atof(object_reference.attribute("zOffset").value());
					double validLength = atof(object_reference.attribute("validLength").value());
					RoadObject::Orientation orientation;

					if (!strcmp(object_reference.attribute("orientation").value(), "none")) {
						orientation = RoadObject::Orientation::NONE;
					} else if (!strcmp(object_reference.attribute("orientation").value(), "+")) {
						orientation = RoadObject::Orientation::POSITIVE;
					} else if (!strcmp(object_reference.attribute("orientation").value(), "-")) {
						orientation = RoadObject::Orientation::NEGATIVE;
					} else {
						LOG("unknown road object reference orientation: %s (road ids=%d)\n",
							object_reference.attribute("orientation").value(), r->GetId());
					}

					auto objectRef = std::make_shared<ObjectReference>(
						ObjectReference(s, t, id, zOffset, validLength, orientation));
					for (auto child : object_reference.children("userData")) {
						objectRef->AddUserData(std::make_shared<UserData>(UserData(child)));
					}
					for (pugi::xml_node validity_node = object_reference.child("validity"); validity_node;
						 validity_node = validity_node.next_sibling("validity")) {
						ValidityRecord validity;
						validity.fromLane_ = atoi(validity_node.attribute("fromLane").value());
						validity.toLane_ = atoi(validity_node.attribute("toLane").value());
						for (auto child : validity_node.children("userData")) {
							validity.AddUserData(std::make_shared<UserData>(UserData(child)));
						}
						objectRef->validity_.push_back(validity);
					}

					if (objectRef != NULL) {
						r->AddObjectReference(objectRef);
					} else {
						LOG("ObjectReference: Major error\n");
					}
				}
			}
		}

		if (r->GetNumberOfLaneSections() == 0) {
			// Add empty center reference lane
			auto lane_section = std::make_shared<LaneSection>(LaneSection(0.0));
			lane_section->AddLane(std::make_shared<Lane>(Lane(0, Lane::LANE_TYPE_NONE)));
			r->AddLaneSection(lane_section);
		}

		road_.push_back(r);
	}

	for (pugi::xml_node controller_node = node.child("controller"); controller_node;
		 controller_node = controller_node.next_sibling("controller")) {
		int id = atoi(controller_node.attribute("id").value());
		std::string name = controller_node.attribute("name").value();
		int sequence = atoi(controller_node.attribute("sequence").value());
		Controller controller(id, name, sequence);
		for (auto child : controller_node.children("userData")) {
			controller.AddUserData(std::make_shared<UserData>(UserData(child)));
		}
		for (pugi::xml_node control_node = controller_node.child("control"); control_node;
			 control_node = control_node.next_sibling("control")) {
			Control control;
			for (auto child : control_node.children("userData")) {
				control.AddUserData(std::make_shared<UserData>(UserData(child)));
			}
			control.signalId_ = atoi(control_node.attribute("signalId").value());
			control.type_ = control_node.attribute("type").value();
			controller.AddControl(control);
		}

		AddController(controller);
	}

	for (pugi::xml_node junction_node = node.child("junction"); junction_node;
		 junction_node = junction_node.next_sibling("junction")) {
		int idj = atoi(junction_node.attribute("id").value());
		std::string name = junction_node.attribute("name").value();
		std::string junction_type_str = junction_node.attribute("type").value();

		Junction::JunctionType junction_type = Junction::JunctionType::DEFAULT;
		if (junction_type_str == "direct") {
			junction_type = Junction::JunctionType::DIRECT;
		} else if (junction_type_str == "virtual") {
			LOG("Virtual junction type found. Not supported yet. Continue treating it as default type");
			junction_type = Junction::JunctionType::DEFAULT;
		}

		std::shared_ptr<Junction> j = std::make_shared<Junction>(Junction(idj, name, junction_type));

		for (auto child : junction_node.children("userData")) {
			j->AddUserData(std::make_shared<UserData>(UserData(child)));
		}

		for (pugi::xml_node connection_node = junction_node.child("connection"); connection_node;
			 connection_node = connection_node.next_sibling("connection")) {
			if (connection_node != NULL) {
				int idc = atoi(connection_node.attribute("id").value());

				int incoming_road_id = atoi(connection_node.attribute("incomingRoad").value());
				std::shared_ptr<Road> incoming_road = GetRoadById(incoming_road_id);

				int connecting_road_id = -1;
				if (junction_type == Junction::JunctionType::DIRECT) {
					connecting_road_id = atoi(connection_node.attribute("linkedRoad").value());
				} else {
					connecting_road_id = atoi(connection_node.attribute("connectingRoad").value());
				}
				std::shared_ptr<Road> connecting_road = GetRoadById(connecting_road_id);

				// Check that the connecting road is referring back to this junction
				if (j->GetType() != Junction::JunctionType::DIRECT
					&& connecting_road->GetJunction() != j->GetId()) {
					LOG("Warning: Connecting road (id %d) junction attribute (%d) is not referring back to "
						"junction %d which is making use of it",
						connecting_road->GetId(), connecting_road->GetJunction(), j->GetId());
				}

				ContactPointType contact_point = CONTACT_POINT_UNDEFINED;
				std::string contact_point_str = connection_node.attribute("contactPoint").value();
				if (contact_point_str == "start") {
					contact_point = CONTACT_POINT_START;
				} else if (contact_point_str == "end") {
					contact_point = CONTACT_POINT_END;
				} else {
					LOG("Unsupported contact point: %s\n", contact_point_str.c_str());
				}

				std::shared_ptr<Connection> connection = std::make_shared<Connection>(
					Connection(idc, incoming_road, connecting_road, contact_point));

				for (auto child : connection_node.children("userData")) {
					connection->AddUserData(std::make_shared<UserData>(UserData(child)));
				}

				for (pugi::xml_node lane_link_node = connection_node.child("laneLink"); lane_link_node;
					 lane_link_node = lane_link_node.next_sibling("laneLink")) {
					int from_id = atoi(lane_link_node.attribute("from").value());
					int to_id = atoi(lane_link_node.attribute("to").value());
					connection->AddJunctionLaneLink(from_id, to_id);
				}
				j->AddConnection(connection);
			}
		}

		for (pugi::xml_node controller_node = junction_node.child("controller"); controller_node;
			 controller_node = controller_node.next_sibling("controller")) {
			JunctionController controller;
			controller.id_ = atoi(controller_node.attribute("id").value());
			controller.type_ = controller_node.attribute("type").value();
			controller.sequence_ = atoi(controller_node.attribute("sequence").value());
			for (auto child : controller_node.children("userData")) {
				controller.AddUserData(std::make_shared<UserData>(UserData(child)));
			}
			j->AddController(controller);
		}

		junction_.push_back(j);
	}

	CheckConnections();

	// if (!SetRoadOSI()) {
	// LOG("Failed to create OSI points for OpenDrive road!");
	// }

	return true;
}
OpenDrive::~OpenDrive() {}

int OpenDrive::GetRoadIdxById(int id) {
	for (int i = 0; i < (int)road_.size(); i++) {
		if (road_[i]->GetId() == id) {
			return i;
		}
	}
	LOG("OpenDrive::GetTrackIdxById Error: Road id %d not found\n", id);
	return -1;
}

int OpenDrive::GetRoadIdByIdx(int idx) {
	if (idx >= 0 && idx < (int)road_.size()) {
		return (road_[idx]->GetId());
	}
	LOG("OpenDrive::GetRoadIdByIdx: idx %d out of range [0:%d]\n", idx, (int)road_.size());
	return 0;
}

int OpenDrive::CheckConnectedRoad(std::shared_ptr<Road> road,
								  std::shared_ptr<RoadLink> link,
								  ContactPointType expected_contact_point_type,
								  std::shared_ptr<RoadLink> link2) {
	if (link2 == 0) {
		return -1;
	}

	if (link2->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
		if (link->GetElementId() == road->GetId()) {
			if (link->GetContactPointType() != expected_contact_point_type) {
				LOG("Found connecting road from other end, but contact point is wrong (expected START, got "
					"%s)",
					ContactPointType2Str(link->GetContactPointType()).c_str());
				return -1;
			}
		}
	}

	return 0;
}

int OpenDrive::CheckJunctionConnection(std::shared_ptr<Junction> junction,
									   std::shared_ptr<Connection> connection) {
	if (junction == 0) {
		return -1;
	}

	// Check if junction is referred to from the connected road
	std::shared_ptr<Road> road = connection->GetConnectingRoad();
	if (road == 0) {
		LOG("Error no connecting road");
		return -1;
	}

	std::shared_ptr<RoadLink> link[2];
	link[0] = road->GetLink(LinkType::PREDECESSOR);
	link[1] = road->GetLink(LinkType::SUCCESSOR);
	for (int i = 0; i < 2; i++) {
		if (link[i] != 0) {
			if (junction->GetType() == Junction::JunctionType::DIRECT) {
				if ((i == 0 &&	// link points to predecessor
					 connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START
					 && link[i]->GetContactPointType() == ContactPointType::CONTACT_POINT_JUNCTION
					 && link[i]->GetElementId() != junction->GetId())
					|| (i == 1 &&  // link points to successor
						connection->GetContactPoint() == ContactPointType::CONTACT_POINT_END
						&& link[i]->GetContactPointType() == ContactPointType::CONTACT_POINT_JUNCTION
						&& link[i]->GetElementId() != junction->GetId())) {
					LOG("Expected direct junction linkedRoad to connect back to junction id %d, found id %d",
						link[i]->GetElementId());
					return -1;
				}

				// Create counter connections, treating linkedRoad as incomingRoad
				// Find out contact point for original incoming road
				ContactPointType new_contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
				if (connection->GetIncomingRoad()->GetLink(LinkType::SUCCESSOR)
					&& connection->GetIncomingRoad()->GetLink(LinkType::SUCCESSOR)->GetElementId()
						   == junction->GetId()) {
					new_contact_point = ContactPointType::CONTACT_POINT_END;
				} else if (connection->GetIncomingRoad()->GetLink(LinkType::PREDECESSOR)
						   && connection->GetIncomingRoad()->GetLink(LinkType::PREDECESSOR)->GetElementId()
								  == junction->GetId()) {
					new_contact_point = ContactPointType::CONTACT_POINT_START;
				} else {
					LOG("Failed to find out contactpoint of direct junction incoming road");
					return -1;
				}

				// Check that it does not already exist
				std::shared_ptr<Connection> new_connection = 0;
				for (size_t k = 0; k < junction->GetNumberOfConnections(); k++) {
					if (junction->GetConnectionByIdx((int)k)->GetIncomingRoad()
						== connection->GetConnectingRoad()) {
						new_connection = junction->GetConnectionByIdx((int)k);
						break;
					}
				}
				if (!new_connection) {
					new_connection = std::make_shared<Connection>(Connection(
						connection->GetConnectingRoad(), connection->GetIncomingRoad(), new_contact_point));
					for (size_t j = 0; j < connection->GetNumberOfLaneLinks(); j++) {
						std::shared_ptr<JunctionLaneLink> tmp_link = connection->GetLaneLink((int)j);
						new_connection->AddJunctionLaneLink(tmp_link->to_, tmp_link->from_);
					}
					junction->AddConnection(new_connection);
				}
			} else {
				if (link[i]->GetElementType() != RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
					LOG("Expected element type ROAD, found %s",
						ElementType2Str(link[i]->GetElementType()).c_str());
					return -1;
				}

				if (link[i]->GetElementId() != connection->GetIncomingRoad()->GetId()) {
					// Check connection from this outgoing road
					std::shared_ptr<Road> roadc = GetRoadById(link[i]->GetElementId());
					std::shared_ptr<RoadLink> link2[2];
					link2[0] = roadc->GetLink(LinkType::PREDECESSOR);
					link2[1] = roadc->GetLink(LinkType::SUCCESSOR);
					for (int j = 0; j < 2; j++) {
						if (link2[j] != 0) {
							if (link2[j]->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION
								&& link2[j]->GetElementId() == junction->GetId()) {
								// Now finally find the reverse link
								for (int k = 0; k < junction->GetNumberOfConnections(); k++) {
									if (junction->GetConnectionByIdx(k)->GetIncomingRoad() == roadc) {
										// Sharing same connecting road?
										if (junction->GetConnectionByIdx(k)->GetConnectingRoad()
											== connection->GetConnectingRoad()) {
											return 0;
										}
									}
								}

								// Create counter connections on other side of connecting road
								LinkType newLinkType = (i == 0 ? LinkType::PREDECESSOR : LinkType::SUCCESSOR);
								std::shared_ptr<RoadLink> newLink
									= connection->GetConnectingRoad()->GetLink(newLinkType);
								if (newLink
									&& newLink->GetElementType()
										   == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
									ContactPointType new_contact_point
										= connection->GetContactPoint() == ContactPointType::CONTACT_POINT_END
											  ? ContactPointType::CONTACT_POINT_START
											  : ContactPointType::CONTACT_POINT_END;

									// Create new connection to the connecting road from other side
									std::shared_ptr<Connection> newConnection = std::make_shared<Connection>(
										Connection(GetRoadById(newLink->GetElementId()),
												   connection->GetConnectingRoad(), new_contact_point));

									// Add lane links - assume only one lane section in the connecting road
									auto ls = newConnection->GetConnectingRoad()->GetLaneSectionByIdx(0);
									for (int l = 0; l < ls->GetNumberOfLanes(); l++) {
										std::shared_ptr<Lane> lane = ls->GetLaneByIdx(l);
										if (lane->GetLink(newLinkType)) {
											int from_id = lane->GetId();
											int to_id = lane->GetLink(newLinkType)->GetId();
											newConnection->AddJunctionLaneLink(to_id, from_id);
										}
									}
									if (newConnection->GetNumberOfLaneLinks() > 0) {
										junction->AddConnection(newConnection);
									}
								}
							}
						}
					}
				}
			}
		}
	}

	return -1;
}

int OpenDrive::CheckLink(std::shared_ptr<Road> road,
						 std::shared_ptr<RoadLink> link,
						 ContactPointType expected_contact_point_type) {
	// does this connection exist in the other direction?
	if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
		std::shared_ptr<Road> connecting_road = GetRoadById(link->GetElementId());
		if (connecting_road != 0) {
			if (CheckConnectedRoad(road, link, expected_contact_point_type,
								   connecting_road->GetLink(LinkType::PREDECESSOR))
				== 0) {
				return 0;
			} else if (CheckConnectedRoad(road, link, expected_contact_point_type,
										  connecting_road->GetLink(LinkType::SUCCESSOR))
					   == 0) {
				return 0;
			} else {
				LOG("Warning: Reversed road link %d->%d not found. Might be a flaw in the OpenDRIVE "
					"description.",
					road->GetId(), connecting_road->GetId());
			}
		}
	} else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
		std::shared_ptr<Junction> junction = GetJunctionById(link->GetElementId());

		// Check all outgoing connections
		if (junction == nullptr) {
			LOG("Info: Junction id %d, referred to by road %d, does not exist", link->GetElementId(),
				road->GetId());
			return -1;
		}

		int nrConnections = junction->GetNumberOfConnections();
		for (int i = 0; i < nrConnections; i++) {
			std::shared_ptr<Connection> connection = junction->GetConnectionByIdx((int)i);

			if (connection->GetIncomingRoad() == road) {
				CheckJunctionConnection(junction, connection);
			}
		}
	}

	return 0;
}

int OpenDrive::CheckConnections() {
	int counter = 0;
	std::shared_ptr<RoadLink> link;

	for (size_t i = 0; i < road_.size(); i++) {
		// Check for connections
		if ((link = road_[i]->GetLink(LinkType::PREDECESSOR)) != 0) {
			CheckLink(road_[i], link, ContactPointType::CONTACT_POINT_START);
		}
		if ((link = road_[i]->GetLink(LinkType::SUCCESSOR)) != 0) {
			CheckLink(road_[i], link, ContactPointType::CONTACT_POINT_END);
		}
	}

	return counter;
}

void OpenDrive::Print() {
	LOG("Roads:\n");
	for (size_t i = 0; i < road_.size(); i++) {
		road_[i]->Print();
	}

	LOG("junctions\n");
	for (size_t i = 0; i < junction_.size(); i++) {
		junction_[i]->Print();
	}
}

void OpenDrive::Save(const std::string fileName) const {
	pugi::xml_document doc;
	auto root = doc.append_child("OpenDRIVE");
	header_.Save(root);

	for (auto road : road_) {
		road->Save(root);
	}

	for (auto controller : controller_) {
		controller.Save(root);
	}

	for (auto junction : junction_) {
		junction->Save(root);
	}

	for (auto userData : user_data_) {
		userData->Save(root);
	}

	doc.save_file(fileName.c_str());
}

std::shared_ptr<GeoReference> OpenDrive::GetGeoReference() {
	return std::make_shared<GeoReference>(geo_ref_);
}

std::string OpenDrive::GetGeoReferenceAsString() {
	std::ostringstream out;
	if (!std::isnan(geo_ref_.lat_0_) && !std::isnan(geo_ref_.lon_0_)) {
		out.precision(13);
		out << "+proj=" << geo_ref_.proj_ << " +lat_0=" << std::fixed << geo_ref_.lat_0_
			<< " +lon_0=" << std::fixed << geo_ref_.lon_0_;
	}
	return out.str();
}

void OpenDrive::ParseGeoLocalization(const std::string& geoLocalization) {
	std::map<std::string, std::string> attributes;
	char space_char = ' ';
	char asignment_char = '=';

	std::stringstream sstream(geoLocalization);
	std::string attribute = "";
	// Get each attribute of geoReference
	while (std::getline(sstream, attribute, space_char)) {
		std::stringstream sstream_attrib(attribute);
		std::string key_value = "";
		std::string attribute_key = "";
		std::string attribute_value = "";
		// Get key and value of each attribute
		while (std::getline(sstream_attrib, key_value, asignment_char)) {
			// Keys starts with a + character
			if (key_value.rfind('+', 0) == 0) {
				attribute_key = key_value;
			} else {
				attribute_value = key_value;
			}
		}
		attributes.emplace(attribute_key, attribute_value);
	}

	for (const auto& attr : attributes) {
		if (attr.first == "+a") {
			geo_ref_.a_ = std::stod(attr.second);
		} else if (attr.first == "+axis") {
			geo_ref_.axis_ = std::stod(attr.second);
		} else if (attr.first == "+b") {
			geo_ref_.b_ = std::stod(attr.second);
		} else if (attr.first == "+ellps") {
			geo_ref_.ellps_ = attr.second;
		} else if (attr.first == "+k") {
			geo_ref_.k_ = std::stod(attr.second);
		} else if (attr.first == "+k_0") {
			geo_ref_.k_0_ = std::stod(attr.second);
		} else if (attr.first == "+lat_0") {
			geo_ref_.lat_0_ = std::stod(attr.second);
		} else if (attr.first == "+lon_0") {
			geo_ref_.lon_0_ = std::stod(attr.second);
		} else if (attr.first == "+lon_wrap") {
			geo_ref_.lon_wrap_ = std::stod(attr.second);
		} else if (attr.first == "+over") {
			geo_ref_.over_ = std::stod(attr.second);
		} else if (attr.first == "+pm") {
			geo_ref_.pm_ = attr.second;
		} else if (attr.first == "+proj") {
			geo_ref_.proj_ = attr.second;
		} else if (attr.first == "+units") {
			geo_ref_.units_ = attr.second;
		} else if (attr.first == "+vunits") {
			geo_ref_.vunits_ = attr.second;
		} else if (attr.first == "+x_0") {
			geo_ref_.x_0_ = std::stod(attr.second);
		} else if (attr.first == "+y_0") {
			geo_ref_.y_0_ = std::stod(attr.second);
		} else if (attr.first == "+datum") {
			geo_ref_.datum_ = attr.second;
		} else if (attr.first == "+geoidgrids") {
			geo_ref_.geo_id_grids_ = attr.second;
		} else if (attr.first == "+zone") {
			geo_ref_.zone_ = std::stod(attr.second);
		} else if (attr.first == "+towgs84") {
			geo_ref_.towgs84_ = std::stoi(attr.second);
		} else {
			LOG("Unsupported geo reference attr: %s", attr.first.c_str());
		}
	}

	if (std::isnan(geo_ref_.lat_0_) || std::isnan(geo_ref_.lon_0_)) {
		LOG("cannot parse georeference: '%s'. Using default values.", geoLocalization.c_str());
		geo_ref_.lat_0_ = 0.0;
		geo_ref_.lon_0_ = 0.0;
	}
}

bool OpenDrive::LoadSignalsByCountry(const std::string& country) {
	std::vector<std::string> file_name_candidates;
	// absolute path or relative to current directory
	file_name_candidates.push_back("../../../resources/traffic_signals/" + country + "_traffic_signals.txt");
	// relative path to scenario directory
	file_name_candidates.push_back("resources/traffic_signals/" + country + "_traffic_signals.txt");
	// Remove all directories from path and look in current directory
	file_name_candidates.push_back(country + "_traffic_signals.txt");
	// Finally check registered paths
	for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++) {
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(
			SE_Env::Inst().GetPaths()[i], "resources/traffic_signals/" + country + "_traffic_signals.txt"));
		file_name_candidates.push_back(
			CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], country + "_traffic_signals.txt"));
	}
	size_t i;
	bool located = false;
	for (i = 0; i < file_name_candidates.size(); i++) {
		if (FileExists(file_name_candidates[i].c_str())) {
			located = true;
			std::string line;
			// assuming the file is text
			std::ifstream fs;
			fs.open(file_name_candidates[i].c_str());

			if (fs.fail()) {
				LOG("Signal: Error to load traffic signals file - %s\n", file_name_candidates[i].c_str());
				if (i < file_name_candidates.size() - 1) {
					LOG("  -> trying: %s", file_name_candidates[i + 1].c_str());
				}
			} else {
				const char delimiter = '=';

				// process each line in turn
				while (std::getline(fs, line)) {
					std::stringstream sstream(line);
					std::string key = "";
					std::string value = "";

					std::getline(sstream, key, delimiter);
					std::getline(sstream, value, delimiter);

					signals_types_.emplace(key, value);
				}

				fs.close();

				break;
			}
		}
	}

	return true;
}

std::string OpenDrive::ContactPointType2Str(ContactPointType type) {
	if (type == ContactPointType::CONTACT_POINT_START) {
		return "PREDECESSOR";
	} else if (type == ContactPointType::CONTACT_POINT_END) {
		return "SUCCESSOR";
	} else if (type == ContactPointType::CONTACT_POINT_JUNCTION) {
		return "JUNCTION";
	} else if (type == ContactPointType::CONTACT_POINT_UNDEFINED) {
		return "UNDEFINED";
	} else {
		return "UNDEFINED";
	}
}

std::string OpenDrive::ElementType2Str(RoadLink::ElementType type) {
	if (type == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
		return "JUNCTION";
	} else if (type == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
		return "ROAD";
	} else if (type == RoadLink::ElementType::ELEMENT_TYPE_UNKNOWN) {
		return "UNKNOWN";
	} else {
		return "UNDEFINED";
	}
}

void OpenDriveOffset::Save(pugi::xml_node& header) const {
	if (!isValid())
		return;

	auto offset = header.append_child("offset");
	offset.append_attribute("x").set_value(x_);
	offset.append_attribute("y").set_value(y_);
	offset.append_attribute("z").set_value(z_);
	offset.append_attribute("hdg").set_value(hdg_);
	for (auto userData : user_data_) {
		userData->Save(offset);
	}
}

void OpenDriveHeader::Save(pugi::xml_node& root) const {
	auto header = root.append_child("header");
	header.append_attribute("revMajor").set_value(revMajor_);
	header.append_attribute("revMinor").set_value(revMinor_);
	if (!name_.empty())
		header.append_attribute("name").set_value(name_.c_str());
	if (version_)
		header.append_attribute("version").set_value(version_);
	if (!date_.empty())
		header.append_attribute("date").set_value(date_.c_str());
	if (north_)
		header.append_attribute("north").set_value(north_);
	if (south_)
		header.append_attribute("south").set_value(south_);
	if (east_)
		header.append_attribute("east").set_value(east_);
	if (west_)
		header.append_attribute("west").set_value(west_);
	if (!vendor_.empty())
		header.append_attribute("vendor").set_value(vendor_.c_str());

	georeference_.Save(header);
	offset_.Save(header);

	for (auto userData : user_data_) {
		userData->Save(header);
	}
}

// void OpenDrive::SetLaneBoundaryPoints() {
// 	// Initialization
// 	Position* pos = new roadmanager::Position();
// 	std::shared_ptr<Road> road;
// 	LaneSection* lsec;
// 	Lane* lane;
// 	int number_of_lane_sections, number_of_lanes, counter;
// 	double lsec_end;
// 	std::vector<double> x0, y0, x1, y1;
// 	std::vector<PointStruct> osi_point;
// 	double s0, s1, s1_prev;
// 	bool osi_requirement;
// 	double max_segment_length = SE_Env::Inst().GetOSIMaxLongitudinalDistance();

// 	// Looping through each road
// 	for (int i = 0; i < road_.size(); i++) {
// 		road = road_[i];

// 		// Looping through each lane section
// 		number_of_lane_sections = road_[i]->GetNumberOfLaneSections();
// 		for (int j = 0; j < number_of_lane_sections; j++) {
// 			// Get the ending position of the current lane section
// 			lsec = road->GetLaneSectionByIdx(j);
// 			if (j == number_of_lane_sections - 1) {
// 				lsec_end = road->GetLength();
// 			} else {
// 				lsec_end = road->GetLaneSectionByIdx(j + 1)->GetS();
// 			}

// 			// Starting points of the each lane section for OSI calculations
// 			s0 = lsec->GetS();
// 			s1 = s0 + OSI_POINT_CALC_STEPSIZE;
// 			s1_prev = s0;

// 			// Looping through each lane
// 			number_of_lanes = lsec->GetNumberOfLanes();
// 			for (int k = 0; k < number_of_lanes; k++) {
// 				lane = lsec->GetLaneByIdx(k);
// 				counter = 0;

// 				int n_roadmarks = lane->GetNumberOfRoadMarks();
// 				if (n_roadmarks == 0) {
// 					// Looping through sequential points along the track determined by
// 					// "OSI_POINT_CALC_STEPSIZE"
// 					while (true) {
// 						counter++;

// 						// Make sure we stay within lane section length
// 						s1 = MIN(s1, lsec_end - OSI_TANGENT_LINE_TOLERANCE);

// 						// [XO, YO] = closest position with given (-) tolerance
// 						pos->SetLaneBoundaryPos(road->GetId(), lane->GetId(),
// 												MAX(0, s0 - OSI_TANGENT_LINE_TOLERANCE), 0, j);
// 						x0.push_back(pos->GetX());
// 						y0.push_back(pos->GetY());

// 						// [XO, YO] = Real position with no tolerance
// 						pos->SetLaneBoundaryPos(road->GetId(), lane->GetId(), s0, 0, j);
// 						x0.push_back(pos->GetX());
// 						y0.push_back(pos->GetY());

// 						// Add the starting point of each lane as osi point
// 						if (counter == 1) {
// 							PointStruct p = {s0, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetHRoad()};
// 							osi_point.push_back(p);
// 						}

// 						// [XO, YO] = closest position with given (+) tolerance
// 						pos->SetLaneBoundaryPos(road->GetId(), lane->GetId(), s0 + OSI_TANGENT_LINE_TOLERANCE,
// 												0, j);
// 						x0.push_back(pos->GetX());
// 						y0.push_back(pos->GetY());

// 						// [X1, Y1] = closest position with given (-) tolerance
// 						pos->SetLaneBoundaryPos(road->GetId(), lane->GetId(), s1 - OSI_TANGENT_LINE_TOLERANCE,
// 												0, j);
// 						x1.push_back(pos->GetX());
// 						y1.push_back(pos->GetY());

// 						// [X1, Y1] = Real position with no tolerance
// 						pos->SetLaneBoundaryPos(road->GetId(), lane->GetId(), s1, 0, j);
// 						x1.push_back(pos->GetX());
// 						y1.push_back(pos->GetY());

// 						// [X1, Y1] = closest position with given (+) tolerance
// 						pos->SetLaneBoundaryPos(road->GetId(), lane->GetId(), s1 + OSI_TANGENT_LINE_TOLERANCE,
// 												0, j);
// 						x1.push_back(pos->GetX());
// 						y1.push_back(pos->GetY());

// 						// Check OSI Requirement between current given points
// 						if (x1[1] - x0[1] != 0 && y1[1] - y0[1] != 0) {
// 							osi_requirement = CheckLaneOSIRequirement(x0, y0, x1, y1);
// 						} else {
// 							osi_requirement = true;
// 						}

// 						// Make sure max segment length is longer than stepsize
// 						max_segment_length = GetMaxSegmentLen(pos, 1.1 * OSI_POINT_CALC_STEPSIZE,
// 															  SE_Env::Inst().GetOSIMaxLongitudinalDistance(),
// 															  OSI_POINT_DIST_SCALE, OSI_POINT_DIST_SCALE);

// 						// If requirement is satisfied -> look further points
// 						// If requirement is not satisfied:
// 						// Assign last satisfied point as OSI point
// 						// Continue searching from the last satisfied point
// 						if (osi_requirement && s1 - s0 < max_segment_length) {
// 							s1_prev = s1;
// 							s1 = s1 + OSI_POINT_CALC_STEPSIZE;

// 						} else {
// 							s0 = s1_prev;
// 							s1_prev = s1;
// 							s1 = s0 + OSI_POINT_CALC_STEPSIZE;

// 							if (counter != 1) {
// 								pos->SetLaneBoundaryPos(road->GetId(), lane->GetId(), s0, 0, j);
// 								PointStruct p = {s0, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetHRoad()};
// 								osi_point.push_back(p);
// 							}
// 						}

// 						// If the end of the lane reached, assign end of the lane as final OSI point for
// 						// current lane
// 						if (s1 + OSI_TANGENT_LINE_TOLERANCE >= lsec_end) {
// 							pos->SetLaneBoundaryPos(road->GetId(), lane->GetId(),
// 													MAX(0, lsec_end - SMALL_NUMBER), 0, j);
// 							PointStruct p
// 								= {lsec_end, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetHRoad()};
// 							osi_point.push_back(p);
// 							break;
// 						}

// 						// Clear x-y collectors for next iteration
// 						x0.clear();
// 						y0.clear();
// 						x1.clear();
// 						y1.clear();
// 					}
// 					// Initialization of LaneBoundary class
// 					LaneBoundaryOSI* lb = new LaneBoundaryOSI((int)0);
// 					// add the lane boundary class to the lane class and generating the global id
// 					lane->SetLaneBoundary(lb);
// 					// Fills up the osi points in the lane boundary class
// 					lb->osi_points_.Set(osi_point);
// 					// Clear osi collectors for next iteration
// 					osi_point.clear();

// 					// Re-assign the starting point of the next lane as the start point of the current lane
// 					// section for OSI calculations
// 					s0 = lsec->GetS();
// 					s1 = s0 + OSI_POINT_CALC_STEPSIZE;
// 					s1_prev = s0;
// 				}
// 			}
// 		}
// 	}
// }

// bool OpenDrive::SetRoadOSI() {
// SetLaneOSIPoints();
// SetRoadMarkOSIPoints();
// SetLaneBoundaryPoints();
// return true;
// }

// Geometry* OpenDrive::GetGeometryByIdx(int road_idx, int geom_idx) {
// if (road_idx >= 0 && road_idx < (int)road_.size()) {
// return road_[road_idx]->GetGeometry(geom_idx);
// } else {
// return 0;
// }
// }

//  MOve to OSI class
// void OpenDrive::SetRoadMarkOSIPoints() {
// 	// Initialization
// 	Position* pos = new roadmanager::Position();
// 	std::shared_ptr<Road> road;
// 	LaneSection* lsec;
// 	Lane* lane;
// 	LaneRoadMark* lane_roadMark;
// 	LaneRoadMarkType* lane_roadMarkType;
// 	LaneRoadMarkTypeLine* lane_roadMarkTypeLine;
// 	int number_of_lane_sections, number_of_lanes, number_of_roadmarks, number_of_roadmarktypes,
// 		number_of_roadmarklines, counter;
// 	double s0, s1, s1_prev, lsec_end, s_roadmark, s_end_roadmark, s_roadmarkline, s_end_roadmarkline;
// 	std::vector<double> x0, y0, x1, y1;
// 	std::vector<PointStruct> osi_point;
// 	bool osi_requirement;
// 	double max_segment_length = SE_Env::Inst().GetOSIMaxLongitudinalDistance();

// 	// Looping through each road
// 	for (int i = 0; i < road_.size(); i++) {
// 		road = road_[i];

// 		// Looping through each lane section
// 		number_of_lane_sections = road_[i]->GetNumberOfLaneSections();
// 		for (int j = 0; j < number_of_lane_sections; j++) {
// 			// Get the ending position of the current lane section
// 			lsec = road->GetLaneSectionByIdx(j);
// 			if (j == number_of_lane_sections - 1) {
// 				lsec_end = road->GetLength();
// 			} else {
// 				lsec_end = road->GetLaneSectionByIdx(j + 1)->GetS();
// 			}

// 			// Looping through each lane
// 			number_of_lanes = lsec->GetNumberOfLanes();
// 			for (int k = 0; k < number_of_lanes; k++) {
// 				lane = lsec->GetLaneByIdx(k);

// 				// Looping through each roadMark within the lane
// 				number_of_roadmarks = lane->GetNumberOfRoadMarks();
// 				if (number_of_roadmarks != 0) {
// 					for (int m = 0; m < number_of_roadmarks; m++) {
// 						lane_roadMark = lane->GetLaneRoadMarkByIdx(m);
// 						s_roadmark = lsec->GetS() + lane_roadMark->GetSOffset();
// 						if (m == number_of_roadmarks - 1) {
// 							s_end_roadmark = MAX(0, lsec_end - SMALL_NUMBER);
// 						} else {
// 							s_end_roadmark
// 								= MAX(0, lsec->GetS() + lane->GetLaneRoadMarkByIdx(m + 1)->GetSOffset()
// 											 - SMALL_NUMBER);
// 						}

// 						// Check the existence of "type" keyword under roadmark
// 						number_of_roadmarktypes = lane_roadMark->GetNumberOfRoadMarkTypes();
// 						if (number_of_roadmarktypes != 0) {
// 							lane_roadMarkType = lane_roadMark->GetLaneRoadMarkTypeByIdx(0);
// 							number_of_roadmarklines = lane_roadMarkType->GetNumberOfRoadMarkTypeLines();

// 							int inner_index = -1;
// 							if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::BROKEN_SOLID
// 								|| lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::SOLID_BROKEN) {
// 								if (number_of_roadmarklines < 2) {
// 									LOG_AND_QUIT(
// 										"You need to specify at least 2 line for broken solid or solid "
// 										"broken roadmark type");
// 								}
// 								std::vector<double> sort_solidbroken_brokensolid;
// 								for (int q = 0; q < number_of_roadmarklines; q++) {
// 									sort_solidbroken_brokensolid.push_back(
// 										lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(q)->GetTOffset());
// 								}

// 								if (lane->GetId() < 0 || lane->GetId() == 0) {
// 									inner_index = (int)(std::max_element(sort_solidbroken_brokensolid.begin(),
// 																		 sort_solidbroken_brokensolid.end())
// 														- sort_solidbroken_brokensolid.begin());
// 								} else {
// 									inner_index = (int)(std::min_element(sort_solidbroken_brokensolid.begin(),
// 																		 sort_solidbroken_brokensolid.end())
// 														- sort_solidbroken_brokensolid.begin());
// 								}
// 							}

// 							// Looping through each roadmarkline under roadmark
// 							for (int n = 0; n < number_of_roadmarklines; n++) {
// 								lane_roadMarkTypeLine = lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(n);
// 								s_roadmarkline = s_roadmark + lane_roadMarkTypeLine->GetSOffset();
// 								if (lane_roadMarkTypeLine != 0) {
// 									s_end_roadmarkline = s_end_roadmark;

// 									bool broken = false;
// 									if (lane_roadMark->GetType()
// 										== LaneRoadMark::RoadMarkType::BROKEN_SOLID) {
// 										if (inner_index == n) {
// 											broken = true;
// 										}
// 									}

// 									if (lane_roadMark->GetType()
// 										== LaneRoadMark::RoadMarkType::SOLID_BROKEN) {
// 										broken = true;
// 										if (inner_index == n) {
// 											broken = false;
// 										}
// 									}

// 									if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::BROKEN
// 										|| lane_roadMark->GetType()
// 											   == LaneRoadMark::RoadMarkType::BROKEN_BROKEN
// 										|| broken) {
// 										// Setting OSI points for each roadmarkline
// 										while (true) {
// 											pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n,
// 																s_roadmarkline, 0, j);
// 											PointStruct p = {s_roadmarkline, pos->GetX(), pos->GetY(),
// 															 pos->GetZ(), pos->GetHRoad()};
// 											osi_point.push_back(p);

// 											pos->SetRoadMarkPos(
// 												road->GetId(), lane->GetId(), m, 0, n,
// 												s_roadmarkline + lane_roadMarkTypeLine->GetLength(), 0, j);
// 											p = {s_roadmarkline + lane_roadMarkTypeLine->GetLength(),
// 												 pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetHRoad()};
// 											osi_point.push_back(p);

// 											s_roadmarkline += lane_roadMarkTypeLine->GetLength()
// 															  + lane_roadMarkTypeLine->GetSpace();
// 											if (s_roadmarkline < SMALL_NUMBER
// 												|| s_roadmarkline > s_end_roadmarkline - SMALL_NUMBER) {
// 												if (s_roadmarkline < SMALL_NUMBER) {
// 													LOG("Roadmark length + space = 0 - ignoring");
// 												}
// 												break;
// 											}
// 										}
// 									} else if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::SOLID
// 											   || lane_roadMark->GetType()
// 													  == LaneRoadMark::RoadMarkType::SOLID_SOLID
// 											   || !broken) {
// 										s0 = s_roadmarkline;
// 										s1 = s0 + OSI_POINT_CALC_STEPSIZE;
// 										s1_prev = s0;
// 										counter = 0;

// 										while (true) {
// 											counter++;

// 											// Make sure we stay within road length
// 											s1 = MIN(s1, road->GetLength() - OSI_TANGENT_LINE_TOLERANCE);

// 											// [XO, YO] = closest position with given (-) tolerance
// 											pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n,
// 																s0 - OSI_TANGENT_LINE_TOLERANCE, 0, j);
// 											x0.push_back(pos->GetX());
// 											y0.push_back(pos->GetY());

// 											// [XO, YO] = Real position with no tolerance
// 											pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, s0, 0,
// 																j);
// 											x0.push_back(pos->GetX());
// 											y0.push_back(pos->GetY());

// 											// Add the starting point of each lane as osi point
// 											if (counter == 1) {
// 												PointStruct p = {s0, pos->GetX(), pos->GetY(), pos->GetZ(),
// 																 pos->GetHRoad()};
// 												osi_point.push_back(p);
// 											}

// 											// [XO, YO] = closest position with given (+) tolerance
// 											pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n,
// 																s0 + OSI_TANGENT_LINE_TOLERANCE, 0, j);
// 											x0.push_back(pos->GetX());
// 											y0.push_back(pos->GetY());

// 											// [X1, Y1] = closest position with given (-) tolerance
// 											pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n,
// 																s1 - OSI_TANGENT_LINE_TOLERANCE, 0, j);
// 											x1.push_back(pos->GetX());
// 											y1.push_back(pos->GetY());

// 											// [X1, Y1] = Real position with no tolerance
// 											pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, s1, 0,
// 																j);
// 											x1.push_back(pos->GetX());
// 											y1.push_back(pos->GetY());

// 											// [X1, Y1] = closest position with given (+) tolerance
// 											pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n,
// 																s1 + OSI_TANGENT_LINE_TOLERANCE, 0, j);
// 											x1.push_back(pos->GetX());
// 											y1.push_back(pos->GetY());

// 											// Check OSI Requirement between current given points
// 											osi_requirement = CheckLaneOSIRequirement(x0, y0, x1, y1);

// 											// Make sure max segment length is longer than stepsize
// 											max_segment_length = GetMaxSegmentLen(
// 												pos, 1.1 * OSI_POINT_CALC_STEPSIZE,
// 												SE_Env::Inst().GetOSIMaxLongitudinalDistance(),
// 												OSI_POINT_DIST_SCALE, OSI_POINT_DIST_SCALE);

// 											// If requirement is satisfied -> look further points
// 											// If requirement is not satisfied:
// 											// Assign last satisfied point as OSI point
// 											// Continue searching from the last satisfied point
// 											if (osi_requirement && s1 - s0 < max_segment_length) {
// 												s1_prev = s1;
// 												s1 = s1 + OSI_POINT_CALC_STEPSIZE;

// 											} else {
// 												s0 = s1_prev;
// 												s1_prev = s1;
// 												s1 = s0 + OSI_POINT_CALC_STEPSIZE;

// 												if (counter != 1) {
// 													pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n,
// 																		s0, 0, j);
// 													PointStruct p = {s0, pos->GetX(), pos->GetY(),
// 																	 pos->GetZ(), pos->GetHRoad()};
// 													osi_point.push_back(p);
// 												}
// 											}

// 											// If the end of the road mark line reached, assign end of the
// 											// road mark line as final OSI point for current road mark line
// 											if (s1 > s_end_roadmarkline - SMALL_NUMBER) {
// 												pos->SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n,
// 																	s_end_roadmarkline, 0, j);
// 												PointStruct p = {s_end_roadmarkline, pos->GetX(), pos->GetY(),
// 																 pos->GetZ(), pos->GetHRoad()};
// 												osi_point.push_back(p);
// 												break;
// 											}

// 											// Clear x-y collectors for next iteration
// 											x0.clear();
// 											y0.clear();
// 											x1.clear();
// 											y1.clear();
// 										}
// 									}

// 									// Set all collected osi points for the current lane rpadmarkline
// 									lane_roadMarkTypeLine->osi_points_.Set(osi_point);

// 									// Clear osi collectors for roadmarks for next iteration
// 									osi_point.clear();
// 								} else {
// 									LOG("LaneRoadMarkTypeLine %d for LaneRoadMarkType for LaneRoadMark %d "
// 										"for lane %d is not defined",
// 										n, m, lane->GetId());
// 								}
// 							}
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
// }
// inherit OpenDrive class to OSI class where you init osi stuff

// move to OSI class
// void OpenDrive::SetLaneOSIPoints() {
// 	// Initialization
// 	Position* pos = new roadmanager::Position();
// 	std::shared_ptr<Road> road;
// 	LaneSection* lsec;
// 	Lane* lane;
// 	int number_of_lane_sections, number_of_lanes, counter;
// 	double lsec_end;
// 	std::vector<PointStruct> osi_point;
// 	std::vector<double> x0, y0, x1, y1;
// 	double s0, s1, s1_prev;
// 	bool osi_requirement;
// 	double max_segment_length = SE_Env::Inst().GetOSIMaxLongitudinalDistance();
// 	int osiintersection;

// 	// Looping through each road
// 	for (int i = 0; i < road_.size(); i++) {
// 		road = road_[i];

// 		if (road->GetJunction() == -1) {
// 			osiintersection = -1;
// 		} else {
// 			if (GetJunctionById(road->GetJunction())->IsOsiIntersection()) {
// 				osiintersection = GetJunctionById(road->GetJunction())->GetGlobalId();
// 			} else {
// 				osiintersection = -1;
// 			}
// 		}

// 		// Looping through each lane section
// 		number_of_lane_sections = road_[i]->GetNumberOfLaneSections();
// 		for (int j = 0; j < number_of_lane_sections; j++) {
// 			// Get the ending position of the current lane section
// 			lsec = road->GetLaneSectionByIdx(j);
// 			if (j == number_of_lane_sections - 1) {
// 				lsec_end = road->GetLength();
// 			} else {
// 				lsec_end = road->GetLaneSectionByIdx(j + 1)->GetS();
// 			}

// 			// Starting points of the each lane section for OSI calculations
// 			s0 = lsec->GetS();
// 			s1 = s0 + OSI_POINT_CALC_STEPSIZE;
// 			s1_prev = s0;

// 			// Looping through each lane
// 			number_of_lanes = lsec->GetNumberOfLanes();
// 			for (int k = 0; k < number_of_lanes; k++) {
// 				lane = lsec->GetLaneByIdx(k);
// 				counter = 0;

// 				// Looping through sequential points along the track determined by "OSI_POINT_CALC_STEPSIZE"
// 				while (true) {
// 					counter++;

// 					// Make sure we stay within lane section length
// 					s1 = MIN(s1, lsec_end - OSI_TANGENT_LINE_TOLERANCE);

// 					// [XO, YO] = closest position with given (-) tolerance
// 					pos->SetLanePos(road->GetId(), lane->GetId(), MAX(0, s0 - OSI_TANGENT_LINE_TOLERANCE), 0,
// 									j);
// 					x0.push_back(pos->GetX());
// 					y0.push_back(pos->GetY());

// 					// [XO, YO] = Real position with no tolerance
// 					pos->SetLanePos(road->GetId(), lane->GetId(), s0, 0, j);
// 					x0.push_back(pos->GetX());
// 					y0.push_back(pos->GetY());

// 					// Add the starting point of each lane as osi point
// 					if (counter == 1) {
// 						PointStruct p = {s0, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetHRoad()};
// 						osi_point.push_back(p);
// 					}

// 					// [XO, YO] = closest position with given (+) tolerance
// 					pos->SetLanePos(road->GetId(), lane->GetId(), s0 + OSI_TANGENT_LINE_TOLERANCE, 0, j);
// 					x0.push_back(pos->GetX());
// 					y0.push_back(pos->GetY());

// 					// [X1, Y1] = closest position with given (-) tolerance
// 					pos->SetLanePos(road->GetId(), lane->GetId(), s1 - OSI_TANGENT_LINE_TOLERANCE, 0, j);
// 					x1.push_back(pos->GetX());
// 					y1.push_back(pos->GetY());

// 					// [X1, Y1] = Real position with no tolerance
// 					pos->SetLanePos(road->GetId(), lane->GetId(), s1, 0, j);
// 					x1.push_back(pos->GetX());
// 					y1.push_back(pos->GetY());

// 					// [X1, Y1] = closest position with given (+) tolerance
// 					pos->SetLanePos(road->GetId(), lane->GetId(), s1 + OSI_TANGENT_LINE_TOLERANCE, 0, j);
// 					x1.push_back(pos->GetX());
// 					y1.push_back(pos->GetY());

// 					// Check OSI Requirement between current given points
// 					if (x1[1] - x0[1] != 0 && y1[1] - y0[1] != 0) {
// 						osi_requirement = CheckLaneOSIRequirement(x0, y0, x1, y1);
// 					} else {
// 						osi_requirement = true;
// 					}

// 					// If requirement is satisfied -> look further points
// 					// If requirement is not satisfied:
// 					//    Assign last unique satisfied point as OSI point
// 					//    Continue searching from the last satisfied point

// 					// Make sure max segment length is longer than stepsize
// 					max_segment_length = GetMaxSegmentLen(pos, 1.1 * OSI_POINT_CALC_STEPSIZE,
// 														  SE_Env::Inst().GetOSIMaxLongitudinalDistance(),
// 														  OSI_POINT_DIST_SCALE, OSI_POINT_DIST_SCALE);

// 					if ((osi_requirement && s1 - s0 < max_segment_length) || s1 - s0 < 0.1) {
// 						s1_prev = s1;
// 						s1 = s1 + OSI_POINT_CALC_STEPSIZE;

// 					} else {
// 						if (s1 - s0 < OSI_POINT_CALC_STEPSIZE + SMALL_NUMBER) {
// 							// Back to last point and try smaller step forward
// 							s1_prev = s1;
// 							s1 = MIN(s0 + (s1 - s0) * 0.5, lsec_end - OSI_TANGENT_LINE_TOLERANCE);
// 						} else {
// 							s0 = s1_prev;
// 							s1_prev = s1;
// 							s1 = MIN(s0 + OSI_POINT_CALC_STEPSIZE, lsec_end - OSI_TANGENT_LINE_TOLERANCE);

// 							if (counter != 1) {
// 								pos->SetLanePos(road->GetId(), lane->GetId(), s0, 0, j);
// 								PointStruct p = {s0, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetHRoad()};
// 								osi_point.push_back(p);
// 							}
// 						}
// 					}

// 					// If the end of the lane reached, assign end of the lane as final OSI point for current
// 					// lane
// 					if (s1 + OSI_TANGENT_LINE_TOLERANCE >= lsec_end) {
// 						pos->SetLanePos(road->GetId(), lane->GetId(), MAX(0, lsec_end - SMALL_NUMBER), 0, j);
// 						PointStruct p = {lsec_end, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetHRoad()};
// 						osi_point.push_back(p);
// 						break;
// 					}

// 					// Clear x-y collectors for next iteration
// 					x0.clear();
// 					y0.clear();
// 					x1.clear();
// 					y1.clear();
// 				}

// 				// Set all collected osi points for the current lane
// 				lane->osi_points_.Set(osi_point);
// 				lane->SetOSIIntersection(osiintersection);

// 				// Clear osi collectors for next iteration
// 				osi_point.clear();

// 				// Re-assign the starting point of the next lane as the start point of the current lane
// 				// section for OSI calculations
// 				s0 = lsec->GetS();
// 				s1 = s0 + OSI_POINT_CALC_STEPSIZE;
// 				s1_prev = s0;
// 			}
// 		}
// 	}
// }

// move to OSI class
// bool OpenDrive::CheckLaneOSIRequirement(std::vector<double> x0,
// 										std::vector<double> y0,
// 										std::vector<double> x1,
// 										std::vector<double> y1) {
// 	double x0_tan_diff, y0_tan_diff, x1_tan_diff, y1_tan_diff;
// 	x0_tan_diff = x0[2] - x0[0];
// 	y0_tan_diff = y0[2] - y0[0];
// 	x1_tan_diff = x1[2] - x1[0];
// 	y1_tan_diff = y1[2] - y1[0];

// 	// Avoiding Zero Denominator in OSI point calculations
// 	if (x0_tan_diff == 0) {
// 		x0_tan_diff += 0.001;
// 	}

// 	if (y0_tan_diff == 0) {
// 		y0_tan_diff += 0.001;
// 	}

// 	if (x1_tan_diff == 0) {
// 		x1_tan_diff += 0.001;
// 	}

// 	if (y1_tan_diff == 0) {
// 		y1_tan_diff += 0.001;
// 	}

// 	// Creating tangent line around the point (First Point) with given tolerance
// 	double k_0 = y0_tan_diff / x0_tan_diff;
// 	double m_0 = y0[1] - k_0 * x0[1];

// 	// Creating tangent line around the point (Second Point) with given tolerance
// 	double k_1 = y1_tan_diff / x1_tan_diff;
// 	double m_1 = y1[1] - k_1 * x1[1];

// 	// Intersection point of the tangent lines
// 	double intersect_tangent_x = (m_0 - m_1) / (k_1 - k_0);
// 	double intersect_tangent_y = k_0 * intersect_tangent_x + m_0;

// 	// Creating real line between the First Point and Second Point
// 	double k = (y1[1] - y0[1]) / (x1[1] - x0[1]);
// 	double m = y0[1] - k * x0[1];

// 	// The maximum distance can be found between the real line and a tangent line: passing through
// 	// [u_intersect, y_intersect] with slope "k" The perpendicular line to the tangent line can be formulated
// 	// as f(Q) = intersect_tangent_y + (intersect_tangent_x / k) - Q/k Then the point on the real line which
// 	// gives maximum distance -> f(Q) = k*Q + m
// 	double intersect_x = (intersect_tangent_y + (intersect_tangent_x / k) - m) / (k + 1 / k);
// 	double intersect_y = k * intersect_x + m;
// 	double max_distance
// 		= sqrt(pow(intersect_y - intersect_tangent_y, 2) + pow(intersect_x - intersect_tangent_x, 2));

// 	// Max distance can be "nan" when the lane is perfectly straigt and hence k = 0.
// 	// In this case, it satisfies OSI_LANE_CALC_REQUIREMENT since it is a perfect line
// 	if (max_distance < SE_Env::Inst().GetOSIMaxLateralDeviation() || std::isnan(max_distance)) {
// 		return true;
// 	} else {
// 		return false;
// 	}
// }

// bool OpenDrive::IsIndirectlyConnected(int road1_id,
// 									  int road2_id,
// 									  int*& connecting_road_id,
// 									  int*& connecting_lane_id,
// 									  int lane1_id,
// 									  int lane2_id) {
// 	std::shared_ptr<Road> road1 = GetRoadById(road1_id);
// 	std::shared_ptr<Road> road2 = GetRoadById(road2_id);
// 	std::shared_ptr<RoadLink> link = 0;

// 	LinkType link_type[2] = {SUCCESSOR, PREDECESSOR};

// 	// Try both ends
// 	for (int k = 0; k < 2; k++) {
// 		link = road1->GetLink(link_type[k]);
// 		if (link == 0) {
// 			continue;
// 		}

// 		LaneSection* lane_section = 0;

// 		if (link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD) {
// 			if (link->GetElementId() == road2->GetId()) {
// 				if (lane1_id != 0 && lane2_id != 0) {
// 					// Check lane connected
// 					if (link_type[k] == SUCCESSOR) {
// 						lane_section = road1->GetLaneSectionByIdx(road1->GetNumberOfLaneSections() - 1);
// 					} else if (link_type[k] == PREDECESSOR) {
// 						lane_section = road1->GetLaneSectionByIdx(0);
// 					} else {
// 						LOG("Error LinkType %d not suppoered\n", link_type[k]);
// 						return false;
// 					}
// 					if (lane_section == 0) {
// 						LOG("Error lane section == 0\n");
// 						return false;
// 					}
// 					Lane* lane = lane_section->GetLaneById(lane1_id);
// 					(void)lane;
// 					if (!(lane_section->GetConnectingLaneId(lane1_id, link_type[k]) == lane2_id)) {
// 						return false;
// 					}
// 					// Now, check other end lane connectivitiy
// 				}
// 				return true;
// 			}
// 		}
// 		// check whether the roads are connected via a junction connecting road and specified lane
// 		else if (link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION) {
// 			std::shared_ptr<Junction> junction = GetJunctionById(link->GetElementId());

// 			for (int i = 0; i < junction->GetNumberOfConnections(); i++) {
// 				std::shared_ptr<Connection> connection = junction->GetConnectionByIdx(i);

// 				if (connection->GetIncomingRoad()->GetId() == road1_id) {
// 					std::shared_ptr<Road> connecting_road = connection->GetConnectingRoad();
// 					std::shared_ptr<RoadLink> exit_link = 0;

// 					// Found a connecting road - first check if this is the second road
// 					if (connecting_road->GetId() == road2_id) {
// 						// Check lanes
// 						for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++) {
// 							if (connection->GetLaneLink(j)->from_ == lane1_id
// 								&& connection->GetLaneLink(j)->to_ == lane2_id) {
// 								return true;
// 							}
// 						}
// 					}

// 					// Then check if it connects to second road
// 					if (connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START) {
// 						exit_link = connecting_road->GetLink(SUCCESSOR);
// 					} else {
// 						exit_link = connecting_road->GetLink(PREDECESSOR);
// 					}

// 					if (exit_link->GetElementId() == road2_id) {
// 						// Finally check that lanes are connected through the junction
// 						// Look at lane section and locate lane connecting both roads
// 						// Assume connecting road has only one lane section
// 						lane_section = connecting_road->GetLaneSectionByIdx(0);
// 						if (lane_section == 0) {
// 							LOG("Error lane section == 0\n");
// 							return false;
// 						}
// 						for (int j = 0; j < lane_section->GetNumberOfLanes(); j++) {
// 							Lane* lane = lane_section->GetLaneByIdx(j);
// 							LaneLink* lane_link_predecessor = lane->GetLink(PREDECESSOR);
// 							LaneLink* lane_link_successor = lane->GetLink(SUCCESSOR);
// 							if (lane_link_predecessor == 0 || lane_link_successor == 0) {
// 								continue;
// 							}
// 							if ((connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START
// 								 && lane_link_predecessor->GetId() == lane1_id
// 								 && lane_link_successor->GetId() == lane2_id)
// 								|| (connection->GetContactPoint() == ContactPointType::CONTACT_POINT_END
// 									&& lane_link_predecessor->GetId() == lane2_id
// 									&& lane_link_successor->GetId() == lane1_id)) {
// 								// Found link
// 								if (connecting_road_id != 0) {
// 									*connecting_road_id = connection->GetConnectingRoad()->GetId();
// 								}
// 								if (connecting_lane_id != 0) {
// 									*connecting_lane_id = lane->GetId();
// 								}
// 								return true;
// 							}
// 						}
// 					}
// 				}
// 			}
// 		} else {
// 			LOG("Error: LinkElementType %d unsupported\n", link->GetElementType());
// 		}
// 	}

// 	LOG("Link not found");

// 	return false;
