/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

/*
 * This module provides an limited interface to OpenDRIVE files.
 * It supports all geometry types (as of ODR 1.4), junctions and some properties such as lane offset
 * but lacks many features as road markings, signals, road signs, speed and road banking
 *
 * It converts between world (cartesian) and road coordinates (both Track and Lane)
 *
 * When used standalone (outside ScenarioEngine) the road manager is initialized via the Position class like
 *this: roadmanager::Position::LoadOpenDrive("example.xodr");
 *
 * Simplest use case is to put a vehicle on the road and simply move it forward along the road, e.g:
 *
 *   car->pos = new roadmanager::Position(3, -2, 10, 0);
 *   while(true)
 *   {
 *	    car->pos->MoveAlongS(0.1);
 *   }
 *
 * The first line will create a Position object initialized at road with ID = 3, in lane = -2 and at lane
 *offset = 0 Then the position is updated along that road and lane, moving 10 cm at a time.
 *
 * A bit more realistic example:
 *
 *   car->pos = new roadmanager::Position(odrManager->GetRoadByIdx(0)->GetId(), -2, 10, 0);
 *   while(true)
 *   {
 *	    car->pos->MoveAlongS(speed * dt);
 *   }
 *
 * Here we refer to the ID of the first road in the network. And instead of static delta movement, the
 *distance is a function of speed and delta time since last update.
 *
 */

#include <assert.h>
#include <time.h>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>

#include "CommonMini.hpp"
#include "RoadManager.hpp"
#include "odrSpiral.h"
#include "pugixml.hpp"

static unsigned int global_lane_counter;

using namespace std;
using namespace roadmanager;

#define CURV_ZERO 0.00001
#define MAX_TRACK_DIST 10
#define OSI_POINT_CALC_STEPSIZE 1		 // [m]
#define OSI_TANGENT_LINE_TOLERANCE 0.01	 // [m]
#define OSI_POINT_DIST_SCALE 0.025
#define ROADMARK_WIDTH_STANDARD 0.15
#define ROADMARK_WIDTH_BOLD 0.20

static int g_Lane_id = 0;
static int g_Laneb_id = 0;

const std::map<std::string, Signal::Type> Signal::types_mapping_ = {
	{"TYPE_UNKNOWN", Signal::TYPE_UNKNOWN},
	{"TYPE_OTHER", Signal::TYPE_OTHER},
	{"TYPE_DANGER_SPOT", Signal::TYPE_DANGER_SPOT},
	{"TYPE_ZEBRA_CROSSING", Signal::TYPE_ZEBRA_CROSSING},
	{"TYPE_FLIGHT", Signal::TYPE_FLIGHT},
	{"TYPE_CATTLE", Signal::TYPE_CATTLE},
	{"TYPE_HORSE_RIDERS", Signal::TYPE_HORSE_RIDERS},
	{"TYPE_AMPHIBIANS", Signal::TYPE_AMPHIBIANS},
	{"TYPE_FALLING_ROCKS", Signal::TYPE_FALLING_ROCKS},
	{"TYPE_SNOW_OR_ICE", Signal::TYPE_SNOW_OR_ICE},
	{"TYPE_LOOSE_GRAVEL", Signal::TYPE_LOOSE_GRAVEL},
	{"TYPE_WATERSIDE", Signal::TYPE_WATERSIDE},
	{"TYPE_CLEARANCE", Signal::TYPE_CLEARANCE},
	{"TYPE_MOVABLE_BRIDGE", Signal::TYPE_MOVABLE_BRIDGE},
	{"TYPE_RIGHT_BEFORE_LEFT_NEXT_INTERSECTION", Signal::TYPE_RIGHT_BEFORE_LEFT_NEXT_INTERSECTION},
	{"TYPE_TURN_LEFT", Signal::TYPE_TURN_LEFT},
	{"TYPE_TURN_RIGHT", Signal::TYPE_TURN_RIGHT},
	{"TYPE_DOUBLE_TURN_LEFT", Signal::TYPE_DOUBLE_TURN_LEFT},
	{"TYPE_DOUBLE_TURN_RIGHT", Signal::TYPE_DOUBLE_TURN_RIGHT},
	{"TYPE_HILL_DOWNWARDS", Signal::TYPE_HILL_DOWNWARDS},
	{"TYPE_HILL_UPWARDS", Signal::TYPE_HILL_UPWARDS},
	{"TYPE_UNEVEN_ROAD", Signal::TYPE_UNEVEN_ROAD},
	{"TYPE_ROAD_SLIPPERY_WET_OR_DIRTY", Signal::TYPE_ROAD_SLIPPERY_WET_OR_DIRTY},
	{"TYPE_SIDE_WINDS", Signal::TYPE_SIDE_WINDS},
	{"TYPE_ROAD_NARROWING", Signal::TYPE_ROAD_NARROWING},
	{"TYPE_ROAD_NARROWING_RIGHT", Signal::TYPE_ROAD_NARROWING_RIGHT},
	{"TYPE_ROAD_NARROWING_LEFT", Signal::TYPE_ROAD_NARROWING_LEFT},
	{"TYPE_ROAD_WORKS", Signal::TYPE_ROAD_WORKS},
	{"TYPE_TRAFFIC_QUEUES", Signal::TYPE_TRAFFIC_QUEUES},
	{"TYPE_TWO_WAY_TRAFFIC", Signal::TYPE_TWO_WAY_TRAFFIC},
	{"TYPE_ATTENTION_TRAFFIC_LIGHT", Signal::TYPE_ATTENTION_TRAFFIC_LIGHT},
	{"TYPE_PEDESTRIANS", Signal::TYPE_PEDESTRIANS},
	{"TYPE_CHILDREN_CROSSING", Signal::TYPE_CHILDREN_CROSSING},
	{"TYPE_CYCLE_ROUTE", Signal::TYPE_CYCLE_ROUTE},
	{"TYPE_DEER_CROSSING", Signal::TYPE_DEER_CROSSING},
	{"TYPE_UNGATED_LEVEL_CROSSING", Signal::TYPE_UNGATED_LEVEL_CROSSING},
	{"TYPE_LEVEL_CROSSING_MARKER", Signal::TYPE_LEVEL_CROSSING_MARKER},
	{"TYPE_RAILWAY_TRAFFIC_PRIORITY", Signal::TYPE_RAILWAY_TRAFFIC_PRIORITY},
	{"TYPE_GIVE_WAY", Signal::TYPE_GIVE_WAY},
	{"TYPE_STOP", Signal::TYPE_STOP},
	{"TYPE_PRIORITY_TO_OPPOSITE_DIRECTION", Signal::TYPE_PRIORITY_TO_OPPOSITE_DIRECTION},
	{"TYPE_PRIORITY_TO_OPPOSITE_DIRECTION_UPSIDE_DOWN",
	 Signal::TYPE_PRIORITY_TO_OPPOSITE_DIRECTION_UPSIDE_DOWN},
	{"TYPE_PRESCRIBED_LEFT_TURN", Signal::TYPE_PRESCRIBED_LEFT_TURN},
	{"TYPE_PRESCRIBED_RIGHT_TURN", Signal::TYPE_PRESCRIBED_RIGHT_TURN},
	{"TYPE_PRESCRIBED_STRAIGHT", Signal::TYPE_PRESCRIBED_STRAIGHT},
	{"TYPE_PRESCRIBED_RIGHT_WAY", Signal::TYPE_PRESCRIBED_RIGHT_WAY},
	{"TYPE_PRESCRIBED_LEFT_WAY", Signal::TYPE_PRESCRIBED_LEFT_WAY},
	{"TYPE_PRESCRIBED_RIGHT_TURN_AND_STRAIGHT", Signal::TYPE_PRESCRIBED_RIGHT_TURN_AND_STRAIGHT},
	{"TYPE_PRESCRIBED_LEFT_TURN_AND_STRAIGHT", Signal::TYPE_PRESCRIBED_LEFT_TURN_AND_STRAIGHT},
	{"TYPE_PRESCRIBED_LEFT_TURN_AND_RIGHT_TURN", Signal::TYPE_PRESCRIBED_LEFT_TURN_AND_RIGHT_TURN},
	{"TYPE_PRESCRIBED_LEFT_TURN_RIGHT_TURN_AND_STRAIGHT",
	 Signal::TYPE_PRESCRIBED_LEFT_TURN_RIGHT_TURN_AND_STRAIGHT},
	{"TYPE_ROUNDABOUT", Signal::TYPE_ROUNDABOUT},
	{"TYPE_ONEWAY_LEFT", Signal::TYPE_ONEWAY_LEFT},
	{"TYPE_ONEWAY_RIGHT", Signal::TYPE_ONEWAY_RIGHT},
	{"TYPE_PASS_LEFT", Signal::TYPE_PASS_LEFT},
	{"TYPE_PASS_RIGHT", Signal::TYPE_PASS_RIGHT},
	{"TYPE_SIDE_LANE_OPEN_FOR_TRAFFIC", Signal::TYPE_SIDE_LANE_OPEN_FOR_TRAFFIC},
	{"TYPE_SIDE_LANE_CLOSED_FOR_TRAFFIC", Signal::TYPE_SIDE_LANE_CLOSED_FOR_TRAFFIC},
	{"TYPE_SIDE_LANE_CLOSING_FOR_TRAFFIC", Signal::TYPE_SIDE_LANE_CLOSING_FOR_TRAFFIC},
	{"TYPE_BUS_STOP", Signal::TYPE_BUS_STOP},
	{"TYPE_TAXI_STAND", Signal::TYPE_TAXI_STAND},
	{"TYPE_BICYCLES_ONLY", Signal::TYPE_BICYCLES_ONLY},
	{"TYPE_HORSE_RIDERS_ONLY", Signal::TYPE_HORSE_RIDERS_ONLY},
	{"TYPE_PEDESTRIANS_ONLY", Signal::TYPE_PEDESTRIANS_ONLY},
	{"TYPE_BICYCLES_PEDESTRIANS_SHARED_ONLY", Signal::TYPE_BICYCLES_PEDESTRIANS_SHARED_ONLY},
	{"TYPE_BICYCLES_PEDESTRIANS_SEPARATED_LEFT_ONLY", Signal::TYPE_BICYCLES_PEDESTRIANS_SEPARATED_LEFT_ONLY},
	{"TYPE_BICYCLES_PEDESTRIANS_SEPARATED_RIGHT_ONLY",
	 Signal::TYPE_BICYCLES_PEDESTRIANS_SEPARATED_RIGHT_ONLY},
	{"TYPE_PEDESTRIAN_ZONE_BEGIN", Signal::TYPE_PEDESTRIAN_ZONE_BEGIN},
	{"TYPE_PEDESTRIAN_ZONE_END", Signal::TYPE_PEDESTRIAN_ZONE_END},
	{"TYPE_BICYCLE_ROAD_BEGIN", Signal::TYPE_BICYCLE_ROAD_BEGIN},
	{"TYPE_BICYCLE_ROAD_END", Signal::TYPE_BICYCLE_ROAD_END},
	{"TYPE_BUS_LANE", Signal::TYPE_BUS_LANE},
	{"TYPE_BUS_LANE_BEGIN", Signal::TYPE_BUS_LANE_BEGIN},
	{"TYPE_BUS_LANE_END", Signal::TYPE_BUS_LANE_END},
	{"TYPE_ALL_PROHIBITED", Signal::TYPE_ALL_PROHIBITED},
	{"TYPE_MOTORIZED_MULTITRACK_PROHIBITED", Signal::TYPE_MOTORIZED_MULTITRACK_PROHIBITED},
	{"TYPE_TRUCKS_PROHIBITED", Signal::TYPE_TRUCKS_PROHIBITED},
	{"TYPE_BICYCLES_PROHIBITED", Signal::TYPE_BICYCLES_PROHIBITED},
	{"TYPE_MOTORCYCLES_PROHIBITED", Signal::TYPE_MOTORCYCLES_PROHIBITED},
	{"TYPE_MOPEDS_PROHIBITED", Signal::TYPE_MOPEDS_PROHIBITED},
	{"TYPE_HORSE_RIDERS_PROHIBITED", Signal::TYPE_HORSE_RIDERS_PROHIBITED},
	{"TYPE_HORSE_CARRIAGES_PROHIBITED", Signal::TYPE_HORSE_CARRIAGES_PROHIBITED},
	{"TYPE_CATTLE_PROHIBITED", Signal::TYPE_CATTLE_PROHIBITED},
	{"TYPE_BUSES_PROHIBITED", Signal::TYPE_BUSES_PROHIBITED},
	{"TYPE_CARS_PROHIBITED", Signal::TYPE_CARS_PROHIBITED},
	{"TYPE_CARS_TRAILERS_PROHIBITED", Signal::TYPE_CARS_TRAILERS_PROHIBITED},
	{"TYPE_TRUCKS_TRAILERS_PROHIBITED", Signal::TYPE_TRUCKS_TRAILERS_PROHIBITED},
	{"TYPE_TRACTORS_PROHIBITED", Signal::TYPE_TRACTORS_PROHIBITED},
	{"TYPE_PEDESTRIANS_PROHIBITED", Signal::TYPE_PEDESTRIANS_PROHIBITED},
	{"TYPE_MOTOR_VEHICLES_PROHIBITED", Signal::TYPE_MOTOR_VEHICLES_PROHIBITED},
	{"TYPE_HAZARDOUS_GOODS_VEHICLES_PROHIBITED", Signal::TYPE_HAZARDOUS_GOODS_VEHICLES_PROHIBITED},
	{"TYPE_OVER_WEIGHT_VEHICLES_PROHIBITED", Signal::TYPE_OVER_WEIGHT_VEHICLES_PROHIBITED},
	{"TYPE_VEHICLES_AXLE_OVER_WEIGHT_PROHIBITED", Signal::TYPE_VEHICLES_AXLE_OVER_WEIGHT_PROHIBITED},
	{"TYPE_VEHICLES_EXCESS_WIDTH_PROHIBITED", Signal::TYPE_VEHICLES_EXCESS_WIDTH_PROHIBITED},
	{"TYPE_VEHICLES_EXCESS_HEIGHT_PROHIBITED", Signal::TYPE_VEHICLES_EXCESS_HEIGHT_PROHIBITED},
	{"TYPE_VEHICLES_EXCESS_LENGTH_PROHIBITED", Signal::TYPE_VEHICLES_EXCESS_LENGTH_PROHIBITED},
	{"TYPE_DO_NOT_ENTER", Signal::TYPE_DO_NOT_ENTER},
	{"TYPE_SNOW_CHAINS_REQUIRED", Signal::TYPE_SNOW_CHAINS_REQUIRED},
	{"TYPE_WATER_POLLUTANT_VEHICLES_PROHIBITED", Signal::TYPE_WATER_POLLUTANT_VEHICLES_PROHIBITED},
	{"TYPE_ENVIRONMENTAL_ZONE_BEGIN", Signal::TYPE_ENVIRONMENTAL_ZONE_BEGIN},
	{"TYPE_ENVIRONMENTAL_ZONE_END", Signal::TYPE_ENVIRONMENTAL_ZONE_END},
	{"TYPE_NO_U_TURN_LEFT", Signal::TYPE_NO_U_TURN_LEFT},
	{"TYPE_NO_U_TURN_RIGHT", Signal::TYPE_NO_U_TURN_RIGHT},
	{"TYPE_PRESCRIBED_U_TURN_LEFT", Signal::TYPE_PRESCRIBED_U_TURN_LEFT},
	{"TYPE_PRESCRIBED_U_TURN_RIGHT", Signal::TYPE_PRESCRIBED_U_TURN_RIGHT},
	{"TYPE_MINIMUM_DISTANCE_FOR_TRUCKS", Signal::TYPE_MINIMUM_DISTANCE_FOR_TRUCKS},
	{"TYPE_SPEED_LIMIT_BEGIN", Signal::TYPE_SPEED_LIMIT_BEGIN},
	{"TYPE_SPEED_LIMIT_ZONE_BEGIN", Signal::TYPE_SPEED_LIMIT_ZONE_BEGIN},
	{"TYPE_SPEED_LIMIT_ZONE_END", Signal::TYPE_SPEED_LIMIT_ZONE_END},
	{"TYPE_MINIMUM_SPEED_BEGIN", Signal::TYPE_MINIMUM_SPEED_BEGIN},
	{"TYPE_OVERTAKING_BAN_BEGIN", Signal::TYPE_OVERTAKING_BAN_BEGIN},
	{"TYPE_OVERTAKING_BAN_FOR_TRUCKS_BEGIN", Signal::TYPE_OVERTAKING_BAN_FOR_TRUCKS_BEGIN},
	{"TYPE_SPEED_LIMIT_END", Signal::TYPE_SPEED_LIMIT_END},
	{"TYPE_MINIMUM_SPEED_END", Signal::TYPE_MINIMUM_SPEED_END},
	{"TYPE_OVERTAKING_BAN_END", Signal::TYPE_OVERTAKING_BAN_END},
	{"TYPE_OVERTAKING_BAN_FOR_TRUCKS_END", Signal::TYPE_OVERTAKING_BAN_FOR_TRUCKS_END},
	{"TYPE_ALL_RESTRICTIONS_END", Signal::TYPE_ALL_RESTRICTIONS_END},
	{"TYPE_NO_STOPPING", Signal::TYPE_NO_STOPPING},
	{"TYPE_NO_PARKING", Signal::TYPE_NO_PARKING},
	{"TYPE_NO_PARKING_ZONE_BEGIN", Signal::TYPE_NO_PARKING_ZONE_BEGIN},
	{"TYPE_NO_PARKING_ZONE_END", Signal::TYPE_NO_PARKING_ZONE_END},
	{"TYPE_RIGHT_OF_WAY_NEXT_INTERSECTION", Signal::TYPE_RIGHT_OF_WAY_NEXT_INTERSECTION},
	{"TYPE_RIGHT_OF_WAY_BEGIN", Signal::TYPE_RIGHT_OF_WAY_BEGIN},
	{"TYPE_RIGHT_OF_WAY_END", Signal::TYPE_RIGHT_OF_WAY_END},
	{"TYPE_PRIORITY_OVER_OPPOSITE_DIRECTION", Signal::TYPE_PRIORITY_OVER_OPPOSITE_DIRECTION},
	{"TYPE_PRIORITY_OVER_OPPOSITE_DIRECTION_UPSIDE_DOWN",
	 Signal::TYPE_PRIORITY_OVER_OPPOSITE_DIRECTION_UPSIDE_DOWN},
	{"TYPE_TOWN_BEGIN", Signal::TYPE_TOWN_BEGIN},
	{"TYPE_TOWN_END", Signal::TYPE_TOWN_END},
	{"TYPE_CAR_PARKING", Signal::TYPE_CAR_PARKING},
	{"TYPE_CAR_PARKING_ZONE_BEGIN", Signal::TYPE_CAR_PARKING_ZONE_BEGIN},
	{"TYPE_CAR_PARKING_ZONE_END", Signal::TYPE_CAR_PARKING_ZONE_END},
	{"TYPE_SIDEWALK_HALF_PARKING_LEFT", Signal::TYPE_SIDEWALK_HALF_PARKING_LEFT},
	{"TYPE_SIDEWALK_HALF_PARKING_RIGHT", Signal::TYPE_SIDEWALK_HALF_PARKING_RIGHT},
	{"TYPE_SIDEWALK_PARKING_LEFT", Signal::TYPE_SIDEWALK_PARKING_LEFT},
	{"TYPE_SIDEWALK_PARKING_RIGHT", Signal::TYPE_SIDEWALK_PARKING_RIGHT},
	{"TYPE_SIDEWALK_PERPENDICULAR_HALF_PARKING_LEFT", Signal::TYPE_SIDEWALK_PERPENDICULAR_HALF_PARKING_LEFT},
	{"TYPE_SIDEWALK_PERPENDICULAR_HALF_PARKING_RIGHT",
	 Signal::TYPE_SIDEWALK_PERPENDICULAR_HALF_PARKING_RIGHT},
	{"TYPE_SIDEWALK_PERPENDICULAR_PARKING_LEFT", Signal::TYPE_SIDEWALK_PERPENDICULAR_PARKING_LEFT},
	{"TYPE_SIDEWALK_PERPENDICULAR_PARKING_RIGHT", Signal::TYPE_SIDEWALK_PERPENDICULAR_PARKING_RIGHT},
	{"TYPE_LIVING_STREET_BEGIN", Signal::TYPE_LIVING_STREET_BEGIN},
	{"TYPE_LIVING_STREET_END", Signal::TYPE_LIVING_STREET_END},
	{"TYPE_TUNNEL", Signal::TYPE_TUNNEL},
	{"TYPE_EMERGENCY_STOPPING_LEFT", Signal::TYPE_EMERGENCY_STOPPING_LEFT},
	{"TYPE_EMERGENCY_STOPPING_RIGHT", Signal::TYPE_EMERGENCY_STOPPING_RIGHT},
	{"TYPE_HIGHWAY_BEGIN", Signal::TYPE_HIGHWAY_BEGIN},
	{"TYPE_HIGHWAY_END", Signal::TYPE_HIGHWAY_END},
	{"TYPE_EXPRESSWAY_BEGIN", Signal::TYPE_EXPRESSWAY_BEGIN},
	{"TYPE_EXPRESSWAY_END", Signal::TYPE_EXPRESSWAY_END},
	{"TYPE_NAMED_HIGHWAY_EXIT", Signal::TYPE_NAMED_HIGHWAY_EXIT},
	{"TYPE_NAMED_EXPRESSWAY_EXIT", Signal::TYPE_NAMED_EXPRESSWAY_EXIT},
	{"TYPE_NAMED_ROAD_EXIT", Signal::TYPE_NAMED_ROAD_EXIT},
	{"TYPE_HIGHWAY_EXIT", Signal::TYPE_HIGHWAY_EXIT},
	{"TYPE_EXPRESSWAY_EXIT", Signal::TYPE_EXPRESSWAY_EXIT},
	{"TYPE_ONEWAY_STREET", Signal::TYPE_ONEWAY_STREET},
	{"TYPE_CROSSING_GUARDS", Signal::TYPE_CROSSING_GUARDS},
	{"TYPE_DEADEND", Signal::TYPE_DEADEND},
	{"TYPE_DEADEND_EXCLUDING_DESIGNATED_ACTORS", Signal::TYPE_DEADEND_EXCLUDING_DESIGNATED_ACTORS},
	{"TYPE_FIRST_AID_STATION", Signal::TYPE_FIRST_AID_STATION},
	{"TYPE_POLICE_STATION", Signal::TYPE_POLICE_STATION},
	{"TYPE_TELEPHONE", Signal::TYPE_TELEPHONE},
	{"TYPE_FILLING_STATION", Signal::TYPE_FILLING_STATION},
	{"TYPE_HOTEL", Signal::TYPE_HOTEL},
	{"TYPE_INN", Signal::TYPE_INN},
	{"TYPE_KIOSK", Signal::TYPE_KIOSK},
	{"TYPE_TOILET", Signal::TYPE_TOILET},
	{"TYPE_CHAPEL", Signal::TYPE_CHAPEL},
	{"TYPE_TOURIST_INFO", Signal::TYPE_TOURIST_INFO},
	{"TYPE_REPAIR_SERVICE", Signal::TYPE_REPAIR_SERVICE},
	{"TYPE_PEDESTRIAN_UNDERPASS", Signal::TYPE_PEDESTRIAN_UNDERPASS},
	{"TYPE_PEDESTRIAN_BRIDGE", Signal::TYPE_PEDESTRIAN_BRIDGE},
	{"TYPE_CAMPER_PLACE", Signal::TYPE_CAMPER_PLACE},
	{"TYPE_ADVISORY_SPEED_LIMIT_BEGIN", Signal::TYPE_ADVISORY_SPEED_LIMIT_BEGIN},
	{"TYPE_ADVISORY_SPEED_LIMIT_END", Signal::TYPE_ADVISORY_SPEED_LIMIT_END},
	{"TYPE_PLACE_NAME", Signal::TYPE_PLACE_NAME},
	{"TYPE_TOURIST_ATTRACTION", Signal::TYPE_TOURIST_ATTRACTION},
	{"TYPE_TOURIST_ROUTE", Signal::TYPE_TOURIST_ROUTE},
	{"TYPE_TOURIST_AREA", Signal::TYPE_TOURIST_AREA},
	{"TYPE_SHOULDER_NOT_PASSABLE_MOTOR_VEHICLES", Signal::TYPE_SHOULDER_NOT_PASSABLE_MOTOR_VEHICLES},
	{"TYPE_SHOULDER_UNSAFE_TRUCKS_TRACTORS", Signal::TYPE_SHOULDER_UNSAFE_TRUCKS_TRACTORS},
	{"TYPE_TOLL_BEGIN", Signal::TYPE_TOLL_BEGIN},
	{"TYPE_TOLL_END", Signal::TYPE_TOLL_END},
	{"TYPE_TOLL_ROAD", Signal::TYPE_TOLL_ROAD},
	{"TYPE_CUSTOMS", Signal::TYPE_CUSTOMS},
	{"TYPE_INTERNATIONAL_BORDER_INFO", Signal::TYPE_INTERNATIONAL_BORDER_INFO},
	{"TYPE_STREETLIGHT_RED_BAND", Signal::TYPE_STREETLIGHT_RED_BAND},
	{"TYPE_FEDERAL_HIGHWAY_ROUTE_NUMBER", Signal::TYPE_FEDERAL_HIGHWAY_ROUTE_NUMBER},
	{"TYPE_HIGHWAY_ROUTE_NUMBER", Signal::TYPE_HIGHWAY_ROUTE_NUMBER},
	{"TYPE_HIGHWAY_INTERCHANGE_NUMBER", Signal::TYPE_HIGHWAY_INTERCHANGE_NUMBER},
	{"TYPE_EUROPEAN_ROUTE_NUMBER", Signal::TYPE_EUROPEAN_ROUTE_NUMBER},
	{"TYPE_FEDERAL_HIGHWAY_DIRECTION_LEFT", Signal::TYPE_FEDERAL_HIGHWAY_DIRECTION_LEFT},
	{"TYPE_FEDERAL_HIGHWAY_DIRECTION_RIGHT", Signal::TYPE_FEDERAL_HIGHWAY_DIRECTION_RIGHT},
	{"TYPE_PRIMARY_ROAD_DIRECTION_LEFT", Signal::TYPE_PRIMARY_ROAD_DIRECTION_LEFT},
	{"TYPE_PRIMARY_ROAD_DIRECTION_RIGHT", Signal::TYPE_PRIMARY_ROAD_DIRECTION_RIGHT},
	{"TYPE_SECONDARY_ROAD_DIRECTION_LEFT", Signal::TYPE_SECONDARY_ROAD_DIRECTION_LEFT},
	{"TYPE_SECONDARY_ROAD_DIRECTION_RIGHT", Signal::TYPE_SECONDARY_ROAD_DIRECTION_RIGHT},
	{"TYPE_DIRECTION_DESIGNATED_ACTORS_LEFT", Signal::TYPE_DIRECTION_DESIGNATED_ACTORS_LEFT},
	{"TYPE_DIRECTION_DESIGNATED_ACTORS_RIGHT", Signal::TYPE_DIRECTION_DESIGNATED_ACTORS_RIGHT},
	{"TYPE_ROUTING_DESIGNATED_ACTORS", Signal::TYPE_ROUTING_DESIGNATED_ACTORS},
	{"TYPE_DIRECTION_TO_HIGHWAY_LEFT", Signal::TYPE_DIRECTION_TO_HIGHWAY_LEFT},
	{"TYPE_DIRECTION_TO_HIGHWAY_RIGHT", Signal::TYPE_DIRECTION_TO_HIGHWAY_RIGHT},
	{"TYPE_DIRECTION_TO_LOCAL_DESTINATION_LEFT", Signal::TYPE_DIRECTION_TO_LOCAL_DESTINATION_LEFT},
	{"TYPE_DIRECTION_TO_LOCAL_DESTINATION_RIGHT", Signal::TYPE_DIRECTION_TO_LOCAL_DESTINATION_RIGHT},
	{"TYPE_CONSOLIDATED_DIRECTIONS", Signal::TYPE_CONSOLIDATED_DIRECTIONS},
	{"TYPE_STREET_NAME", Signal::TYPE_STREET_NAME},
	{"TYPE_DIRECTION_PREANNOUNCEMENT", Signal::TYPE_DIRECTION_PREANNOUNCEMENT},
	{"TYPE_DIRECTION_PREANNOUNCEMENT_LANE_CONFIG", Signal::TYPE_DIRECTION_PREANNOUNCEMENT_LANE_CONFIG},
	{"TYPE_DIRECTION_PREANNOUNCEMENT_HIGHWAY_ENTRIES",
	 Signal::TYPE_DIRECTION_PREANNOUNCEMENT_HIGHWAY_ENTRIES},
	{"TYPE_HIGHWAY_ANNOUNCEMENT", Signal::TYPE_HIGHWAY_ANNOUNCEMENT},
	{"TYPE_OTHER_ROAD_ANNOUNCEMENT", Signal::TYPE_OTHER_ROAD_ANNOUNCEMENT},
	{"TYPE_HIGHWAY_ANNOUNCEMENT_TRUCK_STOP", Signal::TYPE_HIGHWAY_ANNOUNCEMENT_TRUCK_STOP},
	{"TYPE_HIGHWAY_PREANNOUNCEMENT_DIRECTIONS", Signal::TYPE_HIGHWAY_PREANNOUNCEMENT_DIRECTIONS},
	{"TYPE_POLE_EXIT", Signal::TYPE_POLE_EXIT},
	{"TYPE_HIGHWAY_DISTANCE_BOARD", Signal::TYPE_HIGHWAY_DISTANCE_BOARD},
	{"TYPE_DETOUR_LEFT", Signal::TYPE_DETOUR_LEFT},
	{"TYPE_DETOUR_RIGHT", Signal::TYPE_DETOUR_RIGHT},
	{"TYPE_NUMBERED_DETOUR", Signal::TYPE_NUMBERED_DETOUR},
	{"TYPE_DETOUR_BEGIN", Signal::TYPE_DETOUR_BEGIN},
	{"TYPE_DETOUR_END", Signal::TYPE_DETOUR_END},
	{"TYPE_DETOUR_ROUTING_BOARD", Signal::TYPE_DETOUR_ROUTING_BOARD},
	{"TYPE_OPTIONAL_DETOUR", Signal::TYPE_OPTIONAL_DETOUR},
	{"TYPE_OPTIONAL_DETOUR_ROUTING", Signal::TYPE_OPTIONAL_DETOUR_ROUTING},
	{"TYPE_ROUTE_RECOMMENDATION", Signal::TYPE_ROUTE_RECOMMENDATION},
	{"TYPE_ROUTE_RECOMMENDATION_END", Signal::TYPE_ROUTE_RECOMMENDATION_END},
	{"TYPE_ANNOUNCE_LANE_TRANSITION_LEFT", Signal::TYPE_ANNOUNCE_LANE_TRANSITION_LEFT},
	{"TYPE_ANNOUNCE_LANE_TRANSITION_RIGHT", Signal::TYPE_ANNOUNCE_LANE_TRANSITION_RIGHT},
	{"TYPE_ANNOUNCE_RIGHT_LANE_END", Signal::TYPE_ANNOUNCE_RIGHT_LANE_END},
	{"TYPE_ANNOUNCE_LEFT_LANE_END", Signal::TYPE_ANNOUNCE_LEFT_LANE_END},
	{"TYPE_ANNOUNCE_RIGHT_LANE_BEGIN", Signal::TYPE_ANNOUNCE_RIGHT_LANE_BEGIN},
	{"TYPE_ANNOUNCE_LEFT_LANE_BEGIN", Signal::TYPE_ANNOUNCE_LEFT_LANE_BEGIN},
	{"TYPE_ANNOUNCE_LANE_CONSOLIDATION", Signal::TYPE_ANNOUNCE_LANE_CONSOLIDATION},
	{"TYPE_DETOUR_CITY_BLOCK", Signal::TYPE_DETOUR_CITY_BLOCK},
	{"TYPE_GATE", Signal::TYPE_GATE},
	{"TYPE_POLE_WARNING", Signal::TYPE_POLE_WARNING},
	{"TYPE_TRAFFIC_CONE", Signal::TYPE_TRAFFIC_CONE},
	{"TYPE_MOBILE_LANE_CLOSURE", Signal::TYPE_MOBILE_LANE_CLOSURE},
	{"TYPE_REFLECTOR_POST", Signal::TYPE_REFLECTOR_POST},
	{"TYPE_DIRECTIONAL_BOARD_WARNING", Signal::TYPE_DIRECTIONAL_BOARD_WARNING},
	{"TYPE_GUIDING_PLATE", Signal::TYPE_GUIDING_PLATE},
	{"TYPE_GUIDING_PLATE_WEDGES", Signal::TYPE_GUIDING_PLATE_WEDGES},
	{"TYPE_PARKING_HAZARD", Signal::TYPE_PARKING_HAZARD},
	{"TYPE_TRAFFIC_LIGHT_GREEN_ARROW", Signal::TYPE_TRAFFIC_LIGHT_GREEN_ARROW}};

void UserData::Save(pugi::xml_node& parent) {
	for (auto userData : origin_node_) {
		userData.attribute("code").set_value(code_.c_str());
		userData.attribute("value").set_value(value_.c_str());
		parent.append_copy(userData);
	}
}

Signal::Type Signal::GetTypeFromString(const std::string& type) {
	if (types_mapping_.count(type) != 0) {
		return types_mapping_.find(type)->second;
	}
	return Signal::TYPE_UNKNOWN;
}

void Signal::Save(pugi::xml_node& signals) {
	auto signal = signals.append_child("signal");
	signal.append_attribute("s").set_value(s_);
	signal.append_attribute("t").set_value(t_);
	signal.append_attribute("id").set_value(id_);
	if (!name_.empty())	 // Name is optional
		signal.append_attribute("name").set_value(name_.c_str());
	if (dynamic_)
		signal.append_attribute("dynamic").set_value("true");
	else if (!dynamic_)
		signal.append_attribute("dynamic").set_value("false");
	else
		assert(false && "Missing dynamic in road signal");

	switch (orientation_) {
	case RoadObject::Orientation::NEGATIVE:
		signal.append_attribute("orientation").set_value("-");
		break;
	case RoadObject::Orientation::POSITIVE:
		signal.append_attribute("orientation").set_value("+");
		break;
	case RoadObject::Orientation::NONE:
		signal.append_attribute("orientation").set_value("none");
		break;
	default:
		assert(false && "Default reached in road signal switch");
		break;
	}
	signal.append_attribute("zOffset").set_value(z_offset_);
	if (!country_.empty())	// country is optional
		signal.append_attribute("country").set_value(country_.c_str());
	// TODO: countryRevision
	signal.append_attribute("type").set_value(type_);
	// object.append_attribute("subtype").set_value() TODO:
	signal.append_attribute("value").set_value(value_);
	if (!unit_.empty())	 // unit is optional
		signal.append_attribute("unit").set_value(unit_.c_str());
	if (height_)
		signal.append_attribute("height").set_value(height_);
	if (width_)
		signal.append_attribute("width").set_value(width_);
	if (!text_.empty())	 // text is optional
		signal.append_attribute("text").set_value(text_.c_str());
	if (h_offset_)	// h_offset is optional
		signal.append_attribute("hOffset").set_value(h_offset_);
	if (pitch_)	 // pitch is optional
		signal.append_attribute("pitch").set_value(pitch_);
	if (roll_)	// roll is optional
		signal.append_attribute("roll").set_value(roll_);

	for (auto validity : validity_) {
		validity.Save(signal);
	}

	for (auto userData : user_data_) {
		userData->Save(signal);
	}
}

static std::string LinkType2Str(LinkType type) {
	if (type == LinkType::PREDECESSOR) {
		return "PREDECESSOR";
	} else if (type == LinkType::SUCCESSOR) {
		return "SUCCESSOR";
	} else if (type == LinkType::NONE) {
		return "NONE";
	} else {
		return std::string("Unknown link type: " + std::to_string(type));
	}
}

int roadmanager::GetNewGlobalLaneId() {
	int returnvalue = g_Lane_id;
	g_Lane_id++;
	return returnvalue;
}

int roadmanager::GetNewGlobalLaneBoundaryId() {
	int returnvalue = g_Laneb_id;
	g_Laneb_id++;
	return returnvalue;
}


void Elevation::Print() {
	LOG("Elevation: s: %.2f A: %.4f B: %.4f C: %.4f D: %.4f\n", GetS(), poly3_.GetA(), poly3_.GetB(),
		poly3_.GetC(), poly3_.GetD());
}

void Elevation::Save(pugi::xml_node& elevationProfile, const std::string name) {
	auto elevation = elevationProfile.append_child(name.c_str());
	elevation.append_attribute("s").set_value(s_);
	elevation.append_attribute("a").set_value(poly3_.GetA());
	elevation.append_attribute("b").set_value(poly3_.GetB());
	elevation.append_attribute("c").set_value(poly3_.GetC());
	elevation.append_attribute("d").set_value(poly3_.GetD());

	for (auto userData : user_data_) {
		userData->Save(elevationProfile);
	}
}

void LaneLink::Print() {
	LOG("LaneLink type: %d id: %d\n", type_, id_);
}

void LaneLink::Save(pugi::xml_node& lane) {
	auto link = lane.child("link");
	if (link.empty()) {
		link = lane.append_child("link");
	}

	if (GetType() == LinkType::PREDECESSOR) {
		auto predecessor = link.append_child("predecessor");
		predecessor.append_attribute("id").set_value(GetId());
	} else if (GetType() == LinkType::SUCCESSOR) {
		auto successor = link.append_child("successor");
		successor.append_attribute("id").set_value(GetId());
	}
}

void Lane::SetGlobalId() {
	global_id_ = GetNewGlobalLaneId();
}

void LaneRoadMarkTypeLine::SetGlobalId() {
	global_id_ = GetNewGlobalLaneBoundaryId();
}

void LaneRoadMarkTypeLine::Save(pugi::xml_node& type) {
	auto line = type.append_child("line");
	line.append_attribute("length").set_value(length_);
	line.append_attribute("space").set_value(space_);
	line.append_attribute("tOffset").set_value(t_offset_);
	line.append_attribute("sOffset").set_value(s_offset_);

	switch (rule_) {
	case RoadMarkTypeLineRule::CAUTION:
		line.append_attribute("rule").set_value("caution");
		break;
	case RoadMarkTypeLineRule::NO_PASSING:
		line.append_attribute("rule").set_value("no passing");
		break;
	case RoadMarkTypeLineRule::NONE:
		line.append_attribute("rule").set_value("none");
		break;
	default:  // rule is optional
		break;
	}

	line.append_attribute("width").set_value(width_);

	switch (color_) {
	case RoadMarkColor::STANDARD_COLOR:
		line.append_attribute("color").set_value("standard");
		break;
	case RoadMarkColor::BLUE:
		line.append_attribute("color").set_value("blue");
		break;
	case RoadMarkColor::GREEN:
		line.append_attribute("color").set_value("green");
		break;
	case RoadMarkColor::RED:
		line.append_attribute("color").set_value("red");
		break;
	case RoadMarkColor::WHITE:
		line.append_attribute("color").set_value("white");
		break;
	case RoadMarkColor::YELLOW:
		line.append_attribute("color").set_value("yellow");
		break;
	default:  // color is optional
		break;
	}
}

LaneWidth* Lane::GetWidthByIndex(int index) {
	if (lane_width_.size() <= index || lane_width_.size() == 0) {
		throw std::runtime_error("Lane::GetWidthByIndex(int index) -> exceeds index");
	} else if (lane_width_.size() < 0) {
		throw std::runtime_error("Lane::GetWidthByIndex(int index) -> index must be larger than 0");
	} else {
		return lane_width_[index];
	}
}

LaneWidth* Lane::GetWidthByS(double s) {
	if (lane_width_.size() == 0) {
		return 0;  // No lanewidth defined
	}
	for (int i = 0; i + 1 < (int)lane_width_.size(); i++) {
		if (s < lane_width_[i + 1]->GetSOffset()) {
			return lane_width_[i];
		}
	}
	return lane_width_.back();
}

LaneLink* Lane::GetLink(LinkType type) {
	for (int i = 0; i < (int)link_.size(); i++) {
		LaneLink* l = link_[i];
		if (l->GetType() == type) {
			return l;
		}
	}
	return 0;  // No link of requested type exists
}

void LaneWidth::Print() {
	LOG("LaneWidth: sOffset: %.2f, a: %.2f, b: %.2f, c: %.2f, d: %.2f\n", s_offset_, poly3_.GetA(),
		poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

void LaneWidth::Save(pugi::xml_node& lane) {
	auto width = lane.append_child("width");
	width.append_attribute("sOffset").set_value(s_offset_);
	width.append_attribute("a").set_value(poly3_.GetA());
	width.append_attribute("b").set_value(poly3_.GetB());
	width.append_attribute("c").set_value(poly3_.GetC());
	width.append_attribute("d").set_value(poly3_.GetD());
}

LaneRoadMark* Lane::GetLaneRoadMarkByIdx(int idx) {
	if (lane_roadMark_.size() <= idx || lane_roadMark_.size() == 0) {
		throw std::runtime_error("Lane::GetLaneRoadMarkByIdx(int idx) -> exceeds index");
	} else if (lane_roadMark_.size() < 0) {
		throw std::runtime_error("Lane::GetLaneRoadMarkByIdx(int idx) -> index must be larger than 0");
	} else {
		return lane_roadMark_[idx];
	}
}

std::vector<int> Lane::GetLineGlobalIds() {
	std::vector<int> line_ids;
	for (int i = 0; i < GetNumberOfRoadMarks(); i++) {
		LaneRoadMark* laneroadmark = GetLaneRoadMarkByIdx(i);
		for (int j = 0; j < laneroadmark->GetNumberOfRoadMarkTypes(); j++) {
			LaneRoadMarkType* laneroadmarktype = laneroadmark->GetLaneRoadMarkTypeByIdx(j);

			for (int h = 0; h < laneroadmarktype->GetNumberOfRoadMarkTypeLines(); h++) {
				LaneRoadMarkTypeLine* laneroadmarktypeline
					= laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(h);
				line_ids.push_back(laneroadmarktypeline->GetGlobalId());
			}
		}
	}

	return line_ids;
}

int Lane::GetLaneBoundaryGlobalId() {
	if (lane_boundary_) {
		return lane_boundary_->GetGlobalId();
	} else {
		return -1;
	}
}

RoadMarkColor LaneRoadMark::ParseColor(pugi::xml_node node) {
	RoadMarkColor color = RoadMarkColor::UNDEFINED;

	if (node.attribute("color") != 0 && strcmp(node.attribute("color").value(), "")) {
		if (!strcmp(node.attribute("color").value(), "standard")) {
			color = RoadMarkColor::STANDARD_COLOR;
		} else if (!strcmp(node.attribute("color").value(), "blue")) {
			color = RoadMarkColor::BLUE;
		} else if (!strcmp(node.attribute("color").value(), "green")) {
			color = RoadMarkColor::GREEN;
		} else if (!strcmp(node.attribute("color").value(), "red")) {
			color = RoadMarkColor::RED;
		} else if (!strcmp(node.attribute("color").value(), "white")) {
			color = RoadMarkColor::WHITE;
		} else if (!strcmp(node.attribute("color").value(), "yellow")) {
			color = RoadMarkColor::YELLOW;
		}
	}

	return color;
}

LaneRoadMarkType* LaneRoadMark::GetLaneRoadMarkTypeByIdx(int idx) {
	if (idx < (int)lane_roadMarkType_.size()) {
		return lane_roadMarkType_[idx];
	}

	return 0;
}

LaneRoadMarkTypeLine* LaneRoadMarkType::GetLaneRoadMarkTypeLineByIdx(int idx) {
	if (idx < (int)lane_roadMarkTypeLine_.size()) {
		return lane_roadMarkTypeLine_[idx];
	}

	return 0;
}

void LaneRoadMarkType::AddLine(LaneRoadMarkTypeLine* lane_roadMarkTypeLine) {
	lane_roadMarkTypeLine->SetGlobalId();
	lane_roadMarkTypeLine_.push_back(lane_roadMarkTypeLine);
}

void LaneRoadMarkType::Save(pugi::xml_node& roadMark) {
	auto type = roadMark.child("type");
	if (type.empty()) {
		type = roadMark.append_child("type");
	}
	type.append_attribute("name").set_value(name_.c_str());
	type.append_attribute("width").set_value(width_);

	assert(!lane_roadMarkTypeLine_.empty());
	for (auto line : lane_roadMarkTypeLine_) {
		line->Save(type);
	}

	for (auto userData : user_data_) {
		userData->Save(type);
	}
}

void LaneRoadMark::Save(pugi::xml_node& lane) {
	auto roadmark = lane.append_child("roadMark");
	roadmark.append_attribute("sOffset").set_value(s_offset_);
	switch (type_) {
	case LaneRoadMark::RoadMarkType::NONE_TYPE:
		roadmark.append_attribute("type").set_value("none");
		break;
	case LaneRoadMark::RoadMarkType::SOLID:
		roadmark.append_attribute("type").set_value("solid");
		break;
	case LaneRoadMark::RoadMarkType::BROKEN:
		roadmark.append_attribute("type").set_value("broken");
		break;
	case LaneRoadMark::RoadMarkType::SOLID_SOLID:
		roadmark.append_attribute("type").set_value("solid solid");
		break;
	case LaneRoadMark::RoadMarkType::SOLID_BROKEN:
		roadmark.append_attribute("type").set_value("solid broken");
		break;
	case LaneRoadMark::RoadMarkType::BROKEN_SOLID:
		roadmark.append_attribute("type").set_value("broken solid");
		break;
	case LaneRoadMark::RoadMarkType::BROKEN_BROKEN:
		roadmark.append_attribute("type").set_value("broken broken");
		break;
	case LaneRoadMark::RoadMarkType::BOTTS_DOTS:
		roadmark.append_attribute("type").set_value("botts dots");
		break;
	case LaneRoadMark::RoadMarkType::GRASS:
		roadmark.append_attribute("type").set_value("grass");
		break;
	case LaneRoadMark::RoadMarkType::CURB:
		roadmark.append_attribute("type").set_value("curb");
		break;
	default:
		assert(false && "Default in roadmark type switch reached");
		break;
	}

	switch (weight_) {
	case LaneRoadMark::RoadMarkWeight::BOLD:
		roadmark.append_attribute("weight").set_value("bold");
		break;
	case LaneRoadMark::RoadMarkWeight::STANDARD:
		roadmark.append_attribute("weight").set_value("standard");
		break;
	default:  // weight is optional
		break;
	}

	switch (color_) {
	case RoadMarkColor::STANDARD_COLOR:
		roadmark.append_attribute("color").set_value("standard");
		break;
	case RoadMarkColor::BLUE:
		roadmark.append_attribute("color").set_value("blue");
		break;
	case RoadMarkColor::GREEN:
		roadmark.append_attribute("color").set_value("green");
		break;
	case RoadMarkColor::RED:
		roadmark.append_attribute("color").set_value("red");
		break;
	case RoadMarkColor::WHITE:
		roadmark.append_attribute("color").set_value("white");
		break;
	case RoadMarkColor::YELLOW:
		roadmark.append_attribute("color").set_value("yellow");
		break;
	default:
		roadmark.append_attribute("color").set_value("standard");
		break;
	}

	switch (material_) {
	case LaneRoadMark::RoadMarkMaterial::STANDARD_MATERIAL:
		roadmark.append_attribute("material").set_value("standard");
		break;
	default:  // material is optional
		break;
	}

	roadmark.append_attribute("width").set_value(width_);

	switch (lane_change_) {
	case LaneRoadMark::RoadMarkLaneChange::BOTH:
		roadmark.append_attribute("laneChange").set_value("both");
		break;
	case LaneRoadMark::RoadMarkLaneChange::INCREASE:
		roadmark.append_attribute("laneChange").set_value("increase");
		break;
	case LaneRoadMark::RoadMarkLaneChange::DECREASE:
		roadmark.append_attribute("laneChange").set_value("decrease");
		break;
	case LaneRoadMark::RoadMarkLaneChange::NONE_LANECHANGE:
		roadmark.append_attribute("laneChange").set_value("none");
		break;
	default:  // lanechange is optional
		break;
	}

	roadmark.append_attribute("height").set_value(height_);

	for (auto roadMarkType : lane_roadMarkType_) {
		if (roadMarkType->GetName().compare("stand-in"))  // only used internally, don't export
			roadMarkType->Save(roadmark);
	}

	for (auto userData : user_data_) {
		userData->Save(roadmark);
	}
}

void Lane::SetLaneBoundary(LaneBoundaryOSI* lane_boundary) {
	lane_boundary->SetGlobalId();
	lane_boundary_ = lane_boundary;
}

void LaneMaterial::Save(pugi::xml_node& lane) {
	auto laneMaterial = lane.append_child("material");
	laneMaterial.append_attribute("sOffset").set_value(s_offset_);
	laneMaterial.append_attribute("surface").set_value(surface_.c_str());
	laneMaterial.append_attribute("friction").set_value(friction_);
	laneMaterial.append_attribute("roughness").set_value(roughness_);
	for (auto userData : user_data_) {
		userData->Save(laneMaterial);
	}
}

void LaneSpeed::Save(pugi::xml_node& lane) {
	auto laneSpeed = lane.append_child("speed");
	laneSpeed.append_attribute("sOffset").set_value(s_offset_);
	laneSpeed.append_attribute("max").set_value(max_);
	laneSpeed.append_attribute("unit").set_value(unit_.c_str());
	for (auto userData : user_data_) {
		userData->Save(laneSpeed);
	}
}

void LaneOffset::Print() {
	LOG("LaneOffset s %.2f a %.4f b %.2f c %.2f d %.2f length %.2f\n", s_, polynomial_.GetA(),
		polynomial_.GetB(), polynomial_.GetC(), polynomial_.GetD(), length_);
}

void LaneOffset::Save(pugi::xml_node& lanes) {
	auto offset = lanes.append_child("laneOffset");
	offset.append_attribute("s").set_value(s_);
	offset.append_attribute("a").set_value(polynomial_.GetA());
	offset.append_attribute("b").set_value(polynomial_.GetB());
	offset.append_attribute("c").set_value(polynomial_.GetC());
	offset.append_attribute("d").set_value(polynomial_.GetD());

	for (auto userData : user_data_) {
		userData->Save(offset);
	}
}

double LaneOffset::GetLaneOffset(double s) {
	return (polynomial_.Evaluate(s - s_));
}

double LaneOffset::GetLaneOffsetPrim(double s) {
	return (polynomial_.EvaluatePrim(s - s_));
}

void Lane::Print() {
	LOG("Lane: %d, type: %d, level: %d\n", id_, type_, level_);

	for (size_t i = 0; i < link_.size(); i++) {
		link_[i]->Print();
	}

	for (size_t i = 0; i < lane_width_.size(); i++) {
		lane_width_[i]->Print();
	}
}

void Lane::Save(pugi::xml_node& laneSection) {
	pugi::xml_node lcr;
	if (GetId() > 0)  // Left lane
	{
		lcr = laneSection.child("left");
		if (lcr.empty()) {
			lcr = laneSection.append_child("left");
		}
	} else if (GetId() == 0)  // Center lane
	{
		lcr = laneSection.child("center");
		if (lcr.empty()) {
			lcr = laneSection.append_child("center");
		}
	} else if (GetId() < 0)	 // Right lane
	{
		lcr = laneSection.child("right");
		if (lcr.empty()) {
			lcr = laneSection.append_child("right");
		}
	}

	auto lane = lcr.append_child("lane");
	lane.append_attribute("id").set_value(id_);

	if (level_ == 1) {
		lane.append_attribute("level").set_value("true");
	} else if (level_ == 0) {
		lane.append_attribute("level").set_value("false");
	}

	switch (type_) {
	case Lane::LaneType::LANE_TYPE_NONE:
		lane.append_attribute("type").set_value("none");
		break;
	case Lane::LaneType::LANE_TYPE_DRIVING:
		lane.append_attribute("type").set_value("driving");
		break;
	case Lane::LaneType::LANE_TYPE_STOP:
		lane.append_attribute("type").set_value("stop");
		break;
	case Lane::LaneType::LANE_TYPE_SHOULDER:
		lane.append_attribute("type").set_value("shoulder");
		break;
	case Lane::LaneType::LANE_TYPE_BIKING:
		lane.append_attribute("type").set_value("biking");
		break;
	case Lane::LaneType::LANE_TYPE_SIDEWALK:
		lane.append_attribute("type").set_value("sidewalk");
		break;
	case Lane::LaneType::LANE_TYPE_BORDER:
		lane.append_attribute("type").set_value("border");
		break;
	case Lane::LaneType::LANE_TYPE_RESTRICTED:
		lane.append_attribute("type").set_value("restricted");
		break;
	case Lane::LaneType::LANE_TYPE_PARKING:
		lane.append_attribute("type").set_value("parking");
		break;
	case Lane::LaneType::LANE_TYPE_MEDIAN:
		lane.append_attribute("type").set_value("median");
		break;
	case Lane::LaneType::LANE_TYPE_SPECIAL1:
		lane.append_attribute("type").set_value("special1");
		break;
	case Lane::LaneType::LANE_TYPE_SPECIAL2:
		lane.append_attribute("type").set_value("special2");
		break;
	case Lane::LaneType::LANE_TYPE_SPECIAL3:
		lane.append_attribute("type").set_value("special3");
		break;
	case Lane::LaneType::LANE_TYPE_ROADWORKS:
		lane.append_attribute("type").set_value("roadWorks");
		break;
	case Lane::LaneType::LANE_TYPE_TRAM:
		lane.append_attribute("type").set_value("tram");
		break;
	case Lane::LaneType::LANE_TYPE_RAIL:
		lane.append_attribute("type").set_value("rail");
		break;
	case Lane::LaneType::LANE_TYPE_ENTRY:
		lane.append_attribute("type").set_value("entry");
		break;
	case Lane::LaneType::LANE_TYPE_EXIT:
		lane.append_attribute("type").set_value("exit");
		break;
	case Lane::LaneType::LANE_TYPE_OFF_RAMP:
		lane.append_attribute("type").set_value("offRamp");
		break;
	case Lane::LaneType::LANE_TYPE_ON_RAMP:
		lane.append_attribute("type").set_value("onRamp");
		break;
	default:
		assert(false && "Default reached in lane type switch");
		break;
	}

	for (auto link : link_) {
		link->Save(lane);
	}

	for (auto width : lane_width_) {
		width->Save(lane);
	}

	for (auto roadMark : lane_roadMark_) {
		roadMark->Save(lane);
	}

	for (auto material : lane_material_) {
		material->Save(lane);
	}

	for (auto speed : lane_speed_) {
		speed->Save(lane);
	}

	for (auto userData : user_data_) {
		userData->Save(lane);
	}
}

bool Lane::IsCenter() {
	if (GetId() == 0) {
		return true;  // Ref lane no width -> no driving
	} else {
		return false;
	}
}
bool Lane::IsType(Lane::LaneType type) {
	if (GetId() == 0) {
		return false;  // Ref lane no width -> no driving
	}

	return bool(type_ & type);
}

bool Lane::IsDriving() {
	if (GetId() == 0) {
		return false;  // Ref lane no width -> no driving
	}

	return bool(type_ & Lane::LaneType::LANE_TYPE_ANY_DRIVING);
}

void LaneSection::Print() {
	LOG("LaneSection: %.2f, %d lanes:\n", s_, (int)lane_.size());

	for (size_t i = 0; i < lane_.size(); i++) {
		lane_[i]->Print();
	}
}

void LaneSection::Save(pugi::xml_node& lanes) {
	auto laneSection = lanes.append_child("laneSection");
	laneSection.append_attribute("s").set_value(s_);
	if (singleSide_)  // Value initialized
	{
		if (singleSide_.value())  // Check contained value
		{
			laneSection.append_attribute("singleSide").set_value("true");
		} else {
			laneSection.append_attribute("singleSide").set_value("false");
		}
	}
	assert(!lane_.empty());
	for (auto lane : lane_) {
		lane->Save(laneSection);
	}

	for (auto userData : user_data_) {
		userData->Save(laneSection);
	}
}

Lane* LaneSection::GetLaneByIdx(int idx) {
	if (idx < (int)lane_.size()) {
		return lane_[idx];
	}

	return 0;
}

bool LaneSection::IsOSILaneById(int id) {
	Lane* lane = GetLaneById(id);
	if (lane == 0) {
		return false;
	} else {
		return !lane->IsCenter();
	}
}

Lane* LaneSection::GetLaneById(int id) {
	for (size_t i = 0; i < lane_.size(); i++) {
		if (lane_[i]->GetId() == id) {
			return lane_[i];
		}
	}
	return 0;
}

int LaneSection::GetLaneIdByIdx(int idx) {
	if (idx > (int)lane_.size() - 1) {
		LOG("LaneSection::GetLaneIdByIdx Error: index %d, only %d lanes\n", idx, (int)lane_.size());
		return 0;
	} else {
		return (lane_[idx]->GetId());
	}
}

int LaneSection::GetLaneIdxById(int id) {
	for (int i = 0; i < (int)lane_.size(); i++) {
		if (lane_[i]->GetId() == id) {
			return i;
		}
	}
	return -1;
}

int LaneSection::GetLaneGlobalIdByIdx(int idx) {
	if (idx < 0 || idx > (int)lane_.size() - 1) {
		LOG("LaneSection::GetLaneIdByIdx Error: index %d, only %d lanes\n", idx, (int)lane_.size());
		return 0;
	} else {
		return (lane_[idx]->GetGlobalId());
	}
}
int LaneSection::GetLaneGlobalIdById(int id) {
	for (size_t i = 0; i < (int)lane_.size(); i++) {
		if (lane_[i]->GetId() == id) {
			return lane_[i]->GetGlobalId();
		}
	}
	return -1;
}

int LaneSection::GetNumberOfDrivingLanes() {
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++) {
		if (lane_[i]->IsDriving()) {
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNumberOfDrivingLanesSide(int side) {
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++) {
		if (SIGN(lane_[i]->GetId()) == SIGN(side) && lane_[i]->IsDriving()) {
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNUmberOfLanesRight() {
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++) {
		if (lane_[i]->GetId() < 0) {
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNUmberOfLanesLeft() {
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++) {
		if (lane_[i]->GetId() > 0) {
			counter++;
		}
	}
	return counter;
}

double LaneSection::GetWidth(double s, int lane_id) {
	if (lane_id == 0) {
		return 0.0;	 // reference lane has no width
	}

	// Enforce s within range of section
	s = CLAMP(s, s_, s_ + GetLength());

	Lane* lane = GetLaneById(lane_id);
	if (lane == 0) {
		return 0.0;
	}

	LaneWidth* lane_width = lane->GetWidthByS(s - s_);
	if (lane_width == 0)  // No lane width registered
	{
		return 0.0;
	}

	// Calculate local s-parameter in width segment
	double ds = s - (s_ + lane_width->GetSOffset());

	// Calculate width at local s
	return lane_width->poly3_.Evaluate(ds);
}

double LaneSection::GetOuterOffset(double s, int lane_id) {
	if (lane_id == 0) {
		return 0;
	}

	double width = GetWidth(s, lane_id);

	if (abs(lane_id) == 1) {
		// this is the last lane, next to reference lane of width = 0. Stop here.
		return width;
	} else {
		int step = lane_id < 0 ? +1 : -1;
		return (width + GetOuterOffset(s, lane_id + step));
	}
}

double LaneSection::GetCenterOffset(double s, int lane_id) {
	if (lane_id == 0) {
		// Reference lane (0) has no width
		return 0.0;
	}
	double outer_offset = GetOuterOffset(s, lane_id);
	double width = GetWidth(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return outer_offset - width / 2;
}

double LaneSection::GetOuterOffsetHeading(double s, int lane_id) {
	if (lane_id == 0) {
		return 0.0;	 // reference lane has no width
	}

	Lane* lane = GetLaneById(lane_id);
	if (lane == 0) {
		return 0.0;
	}

	LaneWidth* lane_width = lane->GetWidthByS(s - s_);
	if (lane_width == 0)  // No lane width registered
	{
		return 0.0;
	}

	// Calculate local s-parameter in width segment
	double ds = s - (s_ + lane_width->GetSOffset());

	// Calculate heading at local s
	double heading = lane_width->poly3_.EvaluatePrim(ds);

	if (abs(lane_id) == 1) {
		// this is the last lane, next to reference lane of width = 0. Stop here.
		return heading;
	} else {
		int step = lane_id < 0 ? +1 : -1;
		return (heading + GetOuterOffsetHeading(s, lane_id + step));
	}
}

double LaneSection::GetCenterOffsetHeading(double s, int lane_id) {
	int step = lane_id < 0 ? +1 : -1;

	if (lane_id == 0) {
		// Reference lane (0) has no width
		return 0.0;
	}
	double inner_offset_heading = GetOuterOffsetHeading(s, lane_id + step);
	double outer_offset_heading = GetOuterOffsetHeading(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return (inner_offset_heading + outer_offset_heading) / 2;
}

void LaneSection::AddLane(Lane* lane) {
	lane->SetGlobalId();
	global_lane_counter++;

	// Keep list sorted on lane ID, from + to -
	if (lane_.size() > 0 && lane->GetId() > lane_.back()->GetId()) {
		for (size_t i = 0; i < lane_.size(); i++) {
			if (lane->GetId() > lane_[i]->GetId()) {
				lane_.insert(lane_.begin() + i, lane);
				break;
			}
		}
	} else {
		lane_.push_back(lane);
	}
}

int LaneSection::GetConnectingLaneId(int incoming_lane_id, LinkType link_type) {
	int id = incoming_lane_id;

	if (GetLaneById(id) == 0) {
		LOG("Lane id %d not available in lane section!", id);
		return 0;
	}

	if (GetLaneById(id)->GetLink(link_type)) {
		id = GetLaneById(id)->GetLink(link_type)->GetId();
	} else {
		// if no driving lane found - stay on same index
		id = incoming_lane_id;
	}

	return id;
}

double LaneSection::GetWidthBetweenLanes(int lane_id1, int lane_id2, double s) {
	double lanewidth = (std::fabs(GetCenterOffset(s, lane_id1)) - std::fabs(GetCenterOffset(s, lane_id2)));

	return lanewidth;
}

// Offset from lane1 to lane2 in direction of reference line
double LaneSection::GetOffsetBetweenLanes(int lane_id1, int lane_id2, double s) {
	double laneCenter1 = GetCenterOffset(s, lane_id1) * SIGN(lane_id1);
	double laneCenter2 = GetCenterOffset(s, lane_id2) * SIGN(lane_id2);
	return (laneCenter2 - laneCenter1);
}

// Offset from closest left road mark to current position
RoadMarkInfo Lane::GetRoadMarkInfoByS(int track_id, int lane_id, double s) {
	Position* pos = new roadmanager::Position();
	Road* road = pos->GetRoadById(track_id);
	LaneSection* lsec;
	Lane* lane;
	LaneRoadMark* lane_roadMark;
	LaneRoadMarkType* lane_roadMarkType;
	LaneRoadMarkTypeLine* lane_roadMarkTypeLine;
	RoadMarkInfo rm_info = {-1, -1};
	int lsec_idx, number_of_lsec, number_of_roadmarks, number_of_roadmarktypes, number_of_roadmarklines;
	double s_roadmark, s_roadmarkline, s_end_roadmark, s_end_roadmarkline = 0, lsec_end = 0;
	if (road == 0) {
		LOG("Position::Set Error: track %d not available", track_id);
		lsec_idx = -1;
	} else {
		lsec_idx = road->GetLaneSectionIdxByS(s);
	}

	lsec = road->GetLaneSectionByIdx(lsec_idx);

	if (lsec == 0) {
		LOG("Position::Set Error: lane section %d not available", lsec_idx);
	} else {
		number_of_lsec = road->GetNumberOfLaneSections();
		if (lsec_idx == number_of_lsec - 1) {
			lsec_end = road->GetLength();
		} else {
			lsec_end = road->GetLaneSectionByIdx(lsec_idx + 1)->GetS();
		}
	}

	lane = lsec->GetLaneById(lane_id);
	if (lane == 0) {
		LOG("Position::Set Error: lane section %d not available", lane_id);
	}

	number_of_roadmarks = lane->GetNumberOfRoadMarks();

	if (number_of_roadmarks > 0) {
		for (int m = 0; m < number_of_roadmarks; m++) {
			lane_roadMark = lane->GetLaneRoadMarkByIdx(m);
			s_roadmark = lsec->GetS() + lane_roadMark->GetSOffset();
			if (m == number_of_roadmarks - 1) {
				s_end_roadmark = lsec_end;
			} else {
				s_end_roadmark = lane->GetLaneRoadMarkByIdx(m + 1)->GetSOffset();
			}

			// Check the existence of "type" keyword under roadmark
			number_of_roadmarktypes = lane_roadMark->GetNumberOfRoadMarkTypes();
			if (number_of_roadmarktypes != 0) {
				lane_roadMarkType = lane_roadMark->GetLaneRoadMarkTypeByIdx(0);
				number_of_roadmarklines = lane_roadMarkType->GetNumberOfRoadMarkTypeLines();

				// Looping through each roadmarkline under roadmark
				for (int n = 0; n < number_of_roadmarklines; n++) {
					lane_roadMarkTypeLine = lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(n);
					s_roadmarkline = s_roadmark + lane_roadMarkTypeLine->GetSOffset();
					if (lane_roadMarkTypeLine != 0) {
						if (n == number_of_roadmarklines - 1) {
							s_end_roadmarkline = s_end_roadmark;
						} else {
							s_end_roadmarkline
								= lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(n + 1)->GetSOffset();
						}
					}

					if (s >= s_roadmarkline && s < s_end_roadmarkline) {
						rm_info.roadmark_idx_ = m;
						rm_info.roadmarkline_idx_ = n;
					} else {
						continue;
					}
				}
			} else {
				rm_info.roadmarkline_idx_ = 0;
				if (s >= s_roadmark && s < s_end_roadmark) {
					rm_info.roadmark_idx_ = m;
				} else {
					continue;
				}
			}
		}
	}
	delete pos;
	return rm_info;
}

RoadLink::RoadLink(LinkType type, pugi::xml_node node)
	: contact_point_type_(ContactPointType::CONTACT_POINT_UNDEFINED) {
	string element_type = node.attribute("elementType").value();
	string contact_point_type = "";
	type_ = type;
	element_id_ = atoi(node.attribute("elementId").value());

	if (node.attribute("contactPoint") != NULL) {
		contact_point_type = node.attribute("contactPoint").value();
	}

	if (element_type == "road") {
		element_type_ = ELEMENT_TYPE_ROAD;
		if (contact_point_type == "start") {
			contact_point_type_ = CONTACT_POINT_START;
		} else if (contact_point_type == "end") {
			contact_point_type_ = CONTACT_POINT_END;
		} else if (contact_point_type.empty()) {
			LOG("Missing contact point type\n");
		} else {
			LOG("Unsupported contact point type: %s\n", contact_point_type.c_str());
			contact_point_type_ = CONTACT_POINT_UNDEFINED;
		}
	} else if (element_type == "junction") {
		element_type_ = ELEMENT_TYPE_JUNCTION;
		// contact_point_type_ = CONTACT_POINT_JUNCTION;
		if (contact_point_type == "start") {
			contact_point_type_ = CONTACT_POINT_START;
		} else if (contact_point_type == "end") {
			contact_point_type_ = CONTACT_POINT_END;
		}
	} else if (element_type.empty()) {
		LOG("Missing element type\n");
	} else {
		LOG("Unsupported element type: %s\n", element_type.c_str());
		element_type_ = ELEMENT_TYPE_UNKNOWN;
	}
}

bool RoadLink::operator==(RoadLink& rhs) {
	return (rhs.type_ == type_ && rhs.element_type_ == element_id_ && rhs.element_id_ == element_id_
			&& rhs.contact_point_type_ == contact_point_type_);
}

void RoadLink::Print() {
	cout << "RoadLink type: " << type_ << " id: " << element_id_ << " element type: " << element_type_
		 << " contact point type: " << contact_point_type_ << endl;
}

void RoadLink::Save(pugi::xml_node& link) {
	auto linkType = GetType();
	pugi::xml_node type;

	switch (GetType()) {
	case LinkType::PREDECESSOR:
		type = link.append_child("predecessor");
		break;
	case LinkType::SUCCESSOR:
		type = link.append_child("successor");
		break;
	default:
		assert(false && "The default case of LinkType switch was reached.");
		return;
	}

	switch (GetElementType()) {
	case RoadLink::ElementType::ELEMENT_TYPE_JUNCTION:
		type.append_attribute("elementType").set_value("junction");
		break;
	case RoadLink::ElementType::ELEMENT_TYPE_ROAD:
		type.append_attribute("elementType").set_value("road");
		break;
	default:
		assert(false && "The default case of elementType switch was reached.");
		break;
	}

	switch (GetContactPointType()) {
	case ContactPointType::CONTACT_POINT_START:
		type.append_attribute("contactPoint").set_value("start");
		break;
	case ContactPointType::CONTACT_POINT_END:
		type.append_attribute("contactPoint").set_value("end");
		break;
	case ContactPointType::CONTACT_POINT_JUNCTION:
		std::cerr << "Unsupported contact point: Junction" << std::endl;
		break;
	case ContactPointType::CONTACT_POINT_UNDEFINED:
		// Suppress output atm. since a lot of tags miss this entry..
		// std::cerr << "Unsupported contact point: Undefined" << std::endl;
		break;
	default:
		assert(false && "The default case of road link contactPoint switch was reached.");
		break;
	}

	assert(GetElementId() >= 0 && "Element ID of road link cannot be negative");
	type.append_attribute("elementId").set_value(GetElementId());

	for (auto userData : user_data_) {
		userData->Save(type);
	}
}

OutlineCornerRoad::OutlineCornerRoad(int roadId, double s, double t, double dz, double height)
	: roadId_(roadId), s_(s), t_(t), dz_(dz), height_(height) {}

OutlineCornerRoad::OutlineCornerRoad(int id, int roadId, double s, double t, double dz, double height)
	: id_(id), roadId_(roadId), s_(s), t_(t), dz_(dz), height_(height) {}

void OutlineCornerRoad::GetPos(double& x, double& y, double& z) {
	roadmanager::Position pos;
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
	roadmanager::Position pref;
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

void RMObject::SetRepeat(Repeat* repeat) {
	if (repeat_.empty())
		repeat_.push_back(repeat);
	else
		repeat_.at(0) = repeat;
}

// OpenDRIVE 1.5 standard uses only 1 repeat tag.
Repeat* RMObject::GetRepeat() {
	if (repeat_.empty())
		return nullptr;
	else
		return repeat_[0];
}

void RMObject::Save(pugi::xml_node& objects) {
	auto object = objects.append_child("object");
	if (!type_.empty())	 // type is optional
		object.append_attribute("type").set_value(type_.c_str());
	// object.append_attribute("subtype").set_value() TODO:
	// object.append_attribute("dynamic").set_value() TODO:
	if (!name_.empty())	 // name is optional
		object.append_attribute("name").set_value(name_.c_str());
	object.append_attribute("id").set_value(id_);
	object.append_attribute("s").set_value(s_);
	object.append_attribute("t").set_value(t_);
	object.append_attribute("zOffset").set_value(z_offset_);
	// object.append_attribute("validLength").set_value(); TODO:
	switch (orientation_) {
	case RoadObject::Orientation::NEGATIVE:
		object.append_attribute("orientation").set_value("-");
		break;
	case RoadObject::Orientation::POSITIVE:
		object.append_attribute("orientation").set_value("+");
		break;
	case RoadObject::Orientation::NONE:
		object.append_attribute("orientation").set_value("none");
		break;
	default:
		assert(false && "Default reached in road object orientation switch");
		break;
	}
	object.append_attribute("hdg").set_value(heading_);
	object.append_attribute("pitch").set_value(pitch_);
	object.append_attribute("roll").set_value(roll_);
	if (length_)
		object.append_attribute("length").set_value(length_);
	if (width_)
		object.append_attribute("width").set_value(width_);
	if (height_)
		object.append_attribute("height").set_value(height_);

	for (auto repeat : repeat_) {
		repeat->Save(object);
	}

	for (auto outline : outlines_) {
		outline->Save(object);
	}

	for (auto validity : validity_) {
		validity.Save(object);
	}

	for (auto userData : user_data_) {
		userData->Save(object);
	}
}

void ObjectReference::Save(pugi::xml_node& objects) {
	auto objectRef = objects.append_child("objectReference");
	objectRef.append_attribute("s").set_value(s_);
	objectRef.append_attribute("t").set_value(t_);
	objectRef.append_attribute("id").set_value(id_);
	objectRef.append_attribute("zOffset").set_value(z_offset_);
	objectRef.append_attribute("validLength").set_value(valid_length_);
	switch (orientation_) {
	case RoadObject::Orientation::NEGATIVE:
		objectRef.append_attribute("orientation").set_value("-");
		break;
	case RoadObject::Orientation::POSITIVE:
		objectRef.append_attribute("orientation").set_value("+");
		break;
	case RoadObject::Orientation::NONE:
		objectRef.append_attribute("orientation").set_value("none");
		break;
	default:
		assert(false && "Default reached in road object reference orientation switch");
		break;
	}

	for (auto validity : validity_) {
		validity.Save(objectRef);
	}

	for (auto userData : user_data_) {
		userData->Save(objectRef);
	}
}

void Bridge::Save(pugi::xml_node& objects) {
	auto bridge = objects.append_child("bridge");
	bridge.append_attribute("s").set_value(s_);
	bridge.append_attribute("length").set_value(length_);
	if (!name_.empty())
		bridge.append_attribute("name").set_value(name_.c_str());
	bridge.append_attribute("id").set_value(id_);
	switch (type_) {
	case CONCRETE:
		bridge.append_attribute("type").set_value("concrete");
		break;
	case STEEL:
		bridge.append_attribute("type").set_value("steel");
		break;
	case BRICK:
		bridge.append_attribute("type").set_value("brick");
		break;
	case WOOD:
		bridge.append_attribute("type").set_value("wood");
		break;
	default:
		assert(false && "Default reached in bridge switch");
		break;
	}

	for (auto validity : validity_) {
		validity.Save(bridge);
	}

	for (auto userData : user_data_) {
		userData->Save(bridge);
	}
}

void ValidityRecord::Save(pugi::xml_node& object) {
	auto validity = object.append_child("validity");
	validity.append_attribute("fromLane").set_value(fromLane_);
	validity.append_attribute("toLane").set_value(toLane_);

	for (auto userData : user_data_) {
		userData->Save(validity);
	}
}

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

void Repeat::Save(pugi::xml_node& object) {
	auto repeat = object.append_child("repeat");
	repeat.append_attribute("s").set_value(s_);
	repeat.append_attribute("length").set_value(length_);
	repeat.append_attribute("distance").set_value(distance_);
	repeat.append_attribute("tStart").set_value(tStart_);
	repeat.append_attribute("tEnd").set_value(tEnd_);
	repeat.append_attribute("widthStart").set_value(widthStart_);
	repeat.append_attribute("widthEnd").set_value(widthEnd_);
	repeat.append_attribute("heightStart").set_value(heightStart_);
	repeat.append_attribute("heightEnd").set_value(heightEnd_);
	repeat.append_attribute("zOffsetStart").set_value(zOffsetStart_);
	repeat.append_attribute("zOffsetEnd").set_value(zOffsetEnd_);
	if (lengthStart_)
		repeat.append_attribute("lengthStart").set_value(lengthStart_);
	if (lengthEnd_)
		repeat.append_attribute("lengthEnd").set_value(lengthEnd_);
	if (radiusStart_)
		repeat.append_attribute("radiusStart").set_value(radiusStart_);
	if (radiusEnd_)
		repeat.append_attribute("radiusEnd").set_value(radiusEnd_);
}

Connection::Connection(Road* incoming_road, Road* connecting_road, ContactPointType contact_point) {
	// Find corresponding road objects
	incoming_road_ = incoming_road;
	connecting_road_ = connecting_road;
	contact_point_ = contact_point;
}

Connection::Connection(int id, Road* incoming_road, Road* connecting_road, ContactPointType contact_point) {
	// Find corresponding road objects
	incoming_road_ = incoming_road;
	connecting_road_ = connecting_road;
	contact_point_ = contact_point;
	id_ = id;
}

Connection::~Connection() {
	for (size_t i = 0; i < lane_link_.size(); i++) {
		delete lane_link_[i];
	}
}

void Connection::AddJunctionLaneLink(int from, int to) {
	lane_link_.push_back(new JunctionLaneLink(from, to));
}

int Connection::GetConnectingLaneId(int incoming_lane_id) {
	for (size_t i = 0; i < lane_link_.size(); i++) {
		if (lane_link_[i]->from_ == incoming_lane_id) {
			return lane_link_[i]->to_;
		}
	}
	return 0;
}

void Connection::Print() {
	LOG("Connection: incoming %d connecting %d\n", incoming_road_->GetId(), connecting_road_->GetId());
	for (size_t i = 0; i < lane_link_.size(); i++) {
		lane_link_[i]->Print();
	}
}

void Connection::Save(pugi::xml_node& junction) {
	auto connection = junction.append_child("connection");
	connection.append_attribute("id").set_value(id_);
	connection.append_attribute("incomingRoad").set_value(incoming_road_->GetId());
	connection.append_attribute("connectingRoad").set_value(connecting_road_->GetId());
	switch (contact_point_) {
	case ContactPointType::CONTACT_POINT_END:
		connection.append_attribute("contactPoint").set_value("end");
		break;
	case ContactPointType::CONTACT_POINT_START:
		connection.append_attribute("contactPoint").set_value("start");
		break;

	default:
		break;
	}

	for (auto laneLink : lane_link_) {
		laneLink->Save(connection);
	}

	for (auto userData : user_data_) {
		userData->Save(connection);
	}
}

void JunctionController::Save(pugi::xml_node& junction) {
	auto controller = junction.append_child("controller");
	controller.append_attribute("id").set_value(id_);
	controller.append_attribute("type").set_value(type_.c_str());
	controller.append_attribute("sequence").set_value(sequence_);
	for (auto userData : user_data_) {
		userData->Save(controller);
	}
}

void JunctionLaneLink::Save(pugi::xml_node& connection) {
	auto lanelink = connection.append_child("laneLink");
	lanelink.append_attribute("from").set_value(from_);
	lanelink.append_attribute("to").set_value(to_);
}

Junction::~Junction() {
	for (size_t i = 0; i < connection_.size(); i++) {
		delete connection_[i];
	}
}

int Junction::GetNumberOfRoadConnections(int roadId, int laneId) {
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++) {
		Connection* connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad() && roadId == connection->GetIncomingRoad()->GetId()) {
			for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++) {
				JunctionLaneLink* lane_link = connection->GetLaneLink(j);
				if (laneId == lane_link->from_) {
					counter++;
				}
			}
		}
	}
	return counter;
}

LaneRoadLaneConnection Junction::GetRoadConnectionByIdx(int roadId, int laneId, int idx, int laneTypeMask) {
	int counter = 0;
	LaneRoadLaneConnection lane_road_lane_connection;

	for (int i = 0; i < GetNumberOfConnections(); i++) {
		Connection* connection = GetConnectionByIdx(i);

		if (connection && connection->GetIncomingRoad() && roadId == connection->GetIncomingRoad()->GetId()) {
			for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++) {
				JunctionLaneLink* lane_link = connection->GetLaneLink(j);
				if (laneId == lane_link->from_) {
					if (counter == idx) {
						lane_road_lane_connection.SetLane(laneId);
						lane_road_lane_connection.contact_point_ = connection->GetContactPoint();
						lane_road_lane_connection.SetConnectingRoad(connection->GetConnectingRoad()->GetId());
						lane_road_lane_connection.SetConnectingLane(lane_link->to_);
						// find out driving direction
						int laneSectionId;
						if (lane_link->to_ < 0) {
							laneSectionId = 0;
						} else {
							laneSectionId = connection->GetConnectingRoad()->GetNumberOfLaneSections() - 1;
						}
						if (!(connection->GetConnectingRoad()
								  ->GetLaneSectionByIdx(laneSectionId)
								  ->GetLaneById(lane_link->to_)
								  ->GetLaneType()
							  & laneTypeMask)) {
							LOG("OpenDrive::GetJunctionConnection target lane not driving! from %d, %d to "
								"%d, %d\n",
								roadId, laneId, connection->GetConnectingRoad()->GetId(), lane_link->to_);
						}

						return lane_road_lane_connection;
					}
					counter++;
				}
			}
		}
	}

	return lane_road_lane_connection;
}
void Junction::SetGlobalId() {
	global_id_ = GetNewGlobalLaneId();
}

bool Junction::IsOsiIntersection() {
	if (connection_[0]->GetIncomingRoad()->GetRoadType(0) != 0) {
		if (connection_[0]->GetIncomingRoad()->GetRoadType(0)->road_type_
			== Road::RoadType::ROADTYPE_MOTORWAY) {
			return false;
		} else {
			return true;
		}
	} else {
		LOG_ONCE(
			"Type of roads are missing, cannot determine for OSI intersection or not, assuming that it is an "
			"intersection.");
		return true;
	}
}

int Junction::GetNoConnectionsFromRoadId(int incomingRoadId) {
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++) {
		Connection* connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad()->GetId() == incomingRoadId) {
			counter++;
		}
	}

	return counter;
}

int Junction::GetConnectingRoadIdFromIncomingRoadId(int incomingRoadId, int index) {
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++) {
		Connection* connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad()->GetId() == incomingRoadId) {
			if (counter == index) {
				return GetConnectionByIdx(i)->GetConnectingRoad()->GetId();
			} else {
				counter++;
			}
		}
	}
	return -1;
}

void Junction::Print() {
	LOG("Junction %d %s: \n", id_, name_.c_str());

	for (size_t i = 0; i < connection_.size(); i++) {
		connection_[i]->Print();
	}
}

void Junction::Save(pugi::xml_node& root) {
	auto junction = root.append_child("junction");
	junction.append_attribute("name").set_value(name_.c_str());
	junction.append_attribute("id").set_value(id_);
	switch (type_) {
	case Junction::JunctionType::DEFAULT:
		junction.append_attribute("type").set_value("default");
		break;
	case Junction::JunctionType::VIRTUAL:
		junction.append_attribute("type").set_value("virtual");
		break;
	default:
		break;
	}

	for (auto connection : connection_) {
		connection->Save(junction);
		std::cout << "saved connection for junction " << id_ << std::endl;
	}

	for (auto controller : controller_) {
		controller.Save(junction);
	}

	for (auto userData : user_data_) {
		userData->Save(junction);
	}
}

JunctionController* Junction::GetJunctionControllerByIdx(int index) {
	if (index >= 0 && index < controller_.size()) {
		return &controller_[index];
	}

	return 0;
}

Road* Junction::GetRoadAtOtherEndOfConnectingRoad(Road* connecting_road, Road* incoming_road) {
	if (connecting_road->GetJunction() == 0) {
		LOG("Unexpected: Road %d not a connecting road", connecting_road->GetId());
		return 0;
	}

	// Check both ends
	LinkType link_type[2] = {LinkType::PREDECESSOR, LinkType::SUCCESSOR};
	for (int i = 0; i < 2; i++) {
		RoadLink* link = connecting_road->GetLink(link_type[i]);
		if (link && link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
			if (link->GetElementId() == incoming_road->GetId()) {
				// Get road at other end
				RoadLink* link2 = connecting_road->GetLink(link_type[(i + 1) % 2]);
				if (link2 && link2->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
					return Position::GetOpenDrive()->GetRoadById(link2->GetElementId());
				}
			}
		}
	}

	LOG("Failed to find road at other end of the connecting road %d from road %d", connecting_road->GetId(),
		incoming_road->GetId());
	return nullptr;
}

bool RoadPath::CheckRoad(Road* checkRoad, RoadPath::PathNode* srcNode, Road* fromRoad, int fromLaneId) {
	// Register length of this road and find node in other end of the road (link)

	RoadLink* nextLink = 0;

	if (srcNode->link->GetElementType() == RoadLink::RoadLink::ELEMENT_TYPE_ROAD) {
		// node link is a road, find link in the other end of it
		if (srcNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END) {
			nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
		} else {
			nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
		}
	} else if (srcNode->link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
		Junction* junction = Position::GetOpenDrive()->GetJunctionById(srcNode->link->GetElementId());
		if (junction && junction->GetType() == Junction::JunctionType::DIRECT) {
			if (checkRoad->GetLink(LinkType::SUCCESSOR)
				&& checkRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == junction->GetId()) {
				// Node link is a direct junction, and it is the successor to the road being checked
				// hence next link is the predecessor of that road
				nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
			} else if (checkRoad->GetLink(LinkType::PREDECESSOR)
					   && checkRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == junction->GetId()) {
				// Node link is a direct junction, and it is the predecessor to the road being checked
				// hence next link is the successor of that road
				nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
			}
		} else {
			if (checkRoad->GetLink(LinkType::SUCCESSOR)
				&& checkRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == fromRoad->GetId()) {
				// Node link is a non direct junction, and it is the successor to the connecting road being
				// checked hence next link is the predecessor of that connecting road
				nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
			} else if (checkRoad->GetLink(LinkType::PREDECESSOR)
					   && checkRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == fromRoad->GetId()) {
				// Node link is a non direct junction, and it is the predecessor to the connecting road being
				// checked hence next link is the successor of that connecting road
				nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
			}
		}
	}

	if (nextLink == 0) {
		// end of road
		return false;
	}

	int nextLaneId = fromRoad->GetConnectingLaneId(srcNode->link, fromLaneId, checkRoad->GetId());
	if (nextLaneId == 0) {
		return false;
	}

	// Check if next node is already visited
	for (size_t i = 0; i < visited_.size(); i++) {
		if (visited_[i]->link == nextLink) {
			// Already visited, ignore and return
			return false;
		}
	}

	// Check if next node is already among unvisited
	size_t i;
	for (i = 0; i < unvisited_.size(); i++) {
		if (unvisited_[i]->link == nextLink) {
			// Consider it, i.e. calc distance and potentially store it (if less than old)
			if (srcNode->dist + checkRoad->GetLength() < unvisited_[i]->dist) {
				unvisited_[i]->dist = srcNode->dist + checkRoad->GetLength();
			}
		}
	}

	if (i == unvisited_.size()) {
		// link not visited before, add it
		PathNode* pNode = new PathNode;
		pNode->dist = srcNode->dist + checkRoad->GetLength();
		pNode->link = nextLink;
		pNode->fromRoad = checkRoad;
		pNode->fromLaneId = nextLaneId;
		pNode->previous = srcNode;
		unvisited_.push_back(pNode);
	}

	return true;
}

int RoadPath::Calculate(double& dist, bool bothDirections, double maxDist) {
	OpenDrive* odr = startPos_->GetOpenDrive();
	RoadLink* link = 0;
	Junction* junction = 0;
	Road* startRoad = odr->GetRoadById(startPos_->GetTrackId());
	Road* targetRoad = odr->GetRoadById(targetPos_->GetTrackId());
	Road* pivotRoad = startRoad;
	int pivotLaneId = startPos_->GetLaneId();
	Road* nextRoad = startRoad;
	bool found = false;
	double tmpDist = 0;
	size_t i;

	// This method will find and measure the length of the shortest path
	// between a start position and a target position
	// The implementation is based on Dijkstra's algorithm
	// https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

	if (pivotRoad == 0) {
		LOG("Invalid startpos road ID: %d", startPos_->GetTrackId());
		return -1;
	}

	for (i = 0; i < (bothDirections ? 2 : 1); i++) {
		ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
		if (bothDirections) {
			if (i == 0) {
				contact_point = ContactPointType::CONTACT_POINT_START;
				link = pivotRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
			} else {
				contact_point = ContactPointType::CONTACT_POINT_END;
				link = pivotRoad->GetLink(LinkType::SUCCESSOR);	 // Find link to previous road or junction
			}
		} else {
			// Look only in forward direction, w.r.t. entity heading
			if (startPos_->GetHRelative() < M_PI_2 || startPos_->GetHRelative() > 3 * M_PI_2) {
				// Along road direction
				contact_point = ContactPointType::CONTACT_POINT_END;
				link = pivotRoad->GetLink(LinkType::SUCCESSOR);	 // Find link to next road or junction
			} else {
				// Opposite road direction
				contact_point = ContactPointType::CONTACT_POINT_START;
				link = pivotRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
			}
		}

		if (link) {
			PathNode* pNode = new PathNode;
			pNode->link = link;
			pNode->fromRoad = pivotRoad;
			pNode->fromLaneId = pivotLaneId;
			pNode->previous = 0;
			pNode->contactPoint = contact_point;
			if (contact_point == ContactPointType::CONTACT_POINT_START) {
				pNode->dist = startPos_->GetS();  // distance to first road link is distance to start of road
			} else if (contact_point == ContactPointType::CONTACT_POINT_END) {
				pNode->dist = pivotRoad->GetLength() - startPos_->GetS();  // distance to end of road
			}

			unvisited_.push_back(pNode);
		}
	}

	if (startRoad == targetRoad) {
		dist = targetPos_->GetS() - startPos_->GetS();

		// Special case: On same road, distance is equal to delta s
		if (startPos_->GetLaneId() < 0) {
			if (startPos_->GetHRelative() > M_PI_2 && startPos_->GetHRelative() < 3 * M_PI_2) {
				// facing opposite road direction
				dist *= -1;
			}
		} else {
			// decreasing in lanes with positive IDs
			dist *= -1;

			if (startPos_->GetHRelative() < M_PI_2 || startPos_->GetHRelative() > 3 * M_PI_2) {
				// facing along road direction
				dist *= -1;
			}
		}

		return 0;
	}

	if (unvisited_.size() == 0) {
		// No links
		dist = 0;
		return -1;
	}

	for (i = 0; i < 100 && !found && unvisited_.size() > 0 && tmpDist < maxDist; i++) {
		found = false;

		// Find unvisited PathNode with shortest distance
		double minDist = LARGE_NUMBER;
		int minIndex = 0;
		for (size_t j = 0; j < unvisited_.size(); j++) {
			if (unvisited_[j]->dist < minDist) {
				minIndex = (int)j;
				minDist = unvisited_[j]->dist;
			}
		}

		link = unvisited_[minIndex]->link;
		tmpDist = unvisited_[minIndex]->dist;
		pivotRoad = unvisited_[minIndex]->fromRoad;
		pivotLaneId = unvisited_[minIndex]->fromLaneId;

		// - Inspect all unvisited neighbor nodes (links), measure edge (road) distance to that link
		// - Note the total distance
		// - If not already in invisited list, put it there.
		// - Update distance to this link if shorter than previously registered value
		if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
			// only one edge (road)
			nextRoad = odr->GetRoadById(link->GetElementId());

			if (nextRoad == targetRoad) {
				// Special case: On same road, distance is equal to delta s, direction considered
				if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_START) {
					tmpDist += targetPos_->GetS();
				} else {
					tmpDist += nextRoad->GetLength() - targetPos_->GetS();
				}

				found = true;
			} else {
				CheckRoad(nextRoad, unvisited_[minIndex], pivotRoad, pivotLaneId);
			}
		} else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
			// check all junction links (connecting roads) that has pivot road as incoming road
			junction = odr->GetJunctionById(link->GetElementId());
			for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(pivotRoad->GetId()); j++) {
				nextRoad = odr->GetRoadById(
					junction->GetConnectingRoadIdFromIncomingRoadId(pivotRoad->GetId(), (int)j));
				if (nextRoad == 0) {
					return 0;
				}

				if (nextRoad == targetRoad)	 // target road reached
				{
					ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
					if (nextRoad->IsSuccessor(pivotRoad, &contact_point)
						|| nextRoad->IsPredecessor(pivotRoad, &contact_point)) {
						if (contact_point == ContactPointType::CONTACT_POINT_START) {
							tmpDist += targetPos_->GetS();
						} else if (contact_point == ContactPointType::CONTACT_POINT_END) {
							tmpDist += nextRoad->GetLength() - targetPos_->GetS();
						} else {
							LOG("Unexpected contact point %s",
								OpenDrive::ContactPointType2Str(contact_point).c_str());
							return -1;
						}
					} else {
						LOG("Failed to check link in junction");
						return -1;
					}
					found = true;
				} else {
					CheckRoad(nextRoad, unvisited_[minIndex], pivotRoad, pivotLaneId);
				}
			}
		}

		// Mark pivot link as visited (move it from unvisited to visited)
		visited_.push_back(unvisited_[minIndex]);
		unvisited_.erase(unvisited_.begin() + minIndex);
	}

	if (found) {
		// Find out whether the path goes forward or backwards from starting position
		if (visited_.size() > 0) {
			RoadPath::PathNode* node = visited_.back();

			while (node) {
				if (node->previous == 0) {
					// This is the first node - inspect whether it is in front or behind start position
					if ((node->link == startRoad->GetLink(LinkType::PREDECESSOR)
						 && abs(startPos_->GetHRelative()) > M_PI_2
						 && abs(startPos_->GetHRelative()) < 3 * M_PI / 2)
						|| ((node->link == startRoad->GetLink(LinkType::SUCCESSOR)
								 && abs(startPos_->GetHRelative()) < M_PI_2
							 || abs(startPos_->GetHRelative()) > 3 * M_PI / 2))) {
						direction_ = 1;
					} else {
						direction_ = -1;
					}
				}
				node = node->previous;
			}
		}
	}

	// Compensate for heading of the start position
	if (startPos_->GetHRelativeDrivingDirection() > M_PI_2
		&& startPos_->GetHRelativeDrivingDirection() < 3 * M_PI_2) {
		direction_ *= -1;
	}
	dist = direction_ * tmpDist;

	return found ? 0 : -1;
}

RoadPath::~RoadPath() {
	for (size_t i = 0; i < visited_.size(); i++) {
		delete (visited_[i]);
	}
	visited_.clear();

	for (size_t i = 0; i < unvisited_.size(); i++) {
		delete (unvisited_[i]);
	}
	unvisited_.clear();
}

void GeoReference::Save(pugi::xml_node& header) const {
	auto georeference = header.append_child("geoReference");
	// TODO: Fix proper CDATA formating for this
	for (auto userData : user_data_) {
		userData->Save(georeference);
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


void Control::Save(pugi::xml_node& controller) {
	auto control = controller.append_child("control");
	control.append_attribute("signalId").set_value(signalId_);
	control.append_attribute("type").set_value(type_.c_str());
	for (auto userData : user_data_) {
		userData->Save(control);
	}
}

void Controller::Save(pugi::xml_node& root) {
	auto controller = root.append_child("controller");
	controller.append_attribute("id").set_value(id_);
	controller.append_attribute("name").set_value(name_.c_str());
	controller.append_attribute("sequence").set_value(sequence_);

	for (auto control : control_) {
		control.Save(controller);
	}

	for (auto userData : user_data_) {
		userData->Save(controller);
	}
}

int LaneSection::GetClosestLaneIdx(double s, double t, double& offset, bool noZeroWidth, int laneTypeMask) {
	double min_offset = t;	// Initial offset relates to reference line
	int candidate_lane_idx = -1;

	for (int i = 0; i < GetNumberOfLanes(); i++)  // Search through all lanes
	{
		int lane_id = GetLaneIdByIdx(i);
		double laneCenterOffset = SIGN(lane_id) * GetCenterOffset(s, lane_id);

		// Only consider lanes with matching lane type
		if (laneTypeMask & GetLaneById(lane_id)->GetLaneType()
			&& (!noZeroWidth || GetWidth(s, lane_id) > SMALL_NUMBER)) {
			// If position is within a lane, we can return it without further checks
			if (fabs(t - laneCenterOffset) < (GetWidth(s, lane_id) / 2.)) {
				min_offset = t - laneCenterOffset;
				candidate_lane_idx = i;
				break;
			}
			if (candidate_lane_idx == -1 || fabs(t - laneCenterOffset) < fabs(min_offset)) {
				min_offset = t - laneCenterOffset;
				candidate_lane_idx = i;
			}
		}
	}

	offset = min_offset;

	if (candidate_lane_idx == -1) {
		// Fall back to reference lane
		candidate_lane_idx = GetLaneIdxById(0);
	}

	return candidate_lane_idx;
}

int PolyLineBase::EvaluateSegmentByLocalS(int i, double local_s, double cornerRadius, TrajVertex& pos) {
	TrajVertex* vp0 = &vertex_[i];

	if (i >= GetNumberOfVertices() - 1) {
		pos.x = vp0->x;
		pos.y = vp0->y;
		pos.z = vp0->z;
		pos.h = vp0->h;
		pos.s = vp0->s;
		pos.p = vp0->p;
		pos.time = vp0->time;
		pos.speed = vp0->speed;
	} else if (i >= 0) {
		TrajVertex* vp1 = &vertex_[i + 1];

		double length = MAX(vertex_[i + 1].s - vertex_[i].s, SMALL_NUMBER);

		local_s = CLAMP(local_s, 0, length);

		double a = local_s / length;  // a = interpolation factor

		pos.x = (1 - a) * vp0->x + a * vp1->x;
		pos.y = (1 - a) * vp0->y + a * vp1->y;
		pos.z = (1 - a) * vp0->z + a * vp1->z;
		pos.time = (1 - a) * vp0->time + a * vp1->time;
		pos.speed = (1 - a) * vp0->speed + a * vp1->speed;
		pos.s = (1 - a) * vp0->s + a * vp1->s;
		pos.p = (1 - a) * vp0->p + a * vp1->p;

		if (vertex_[i + 1].calcHeading && !interpolateHeading_) {
			// Strategy: Align to line, but interpolate at corners
			double radius = MIN(4.0, length);
			if (local_s < radius) {
				// passed a corner
				a = (radius + local_s) / (2 * radius);
				if (i > 0) {
					pos.h = GetAngleInInterval2PI(vertex_[i - 1].h
												  + a * GetAngleDifference(vertex_[i].h, vertex_[i - 1].h));
				} else {
					// No previous value to interpolate
					pos.h = vertex_[i].h;
				}
			} else if (local_s > length - radius) {
				a = (radius + (length - local_s)) / (2 * radius);
				if (i > GetNumberOfVertices() - 2) {
					// Last segment, no next point to interpolate
					pos.h = a * vertex_[i].h;
				} else {
					pos.h = GetAngleInInterval2PI(
						vertex_[i].h + (1 - a) * GetAngleDifference(vertex_[i + 1].h, vertex_[i].h));
				}
			} else {
				pos.h = vertex_[i].h;
			}
		} else {
			// Interpolate
			pos.h = GetAngleInInterval2PI(vp0->h + a * GetAngleDifference(vp1->h, vp0->h));
		}
	} else {
		return -1;
	}

	return 0;
}

TrajVertex* PolyLineBase::AddVertex(double x, double y, double z, double h) {
	TrajVertex v;

	v.calcHeading = false;
	vertex_.push_back(v);

	return UpdateVertex(GetNumberOfVertices() - 1, x, y, z, GetAngleInInterval2PI(h));
}

TrajVertex* PolyLineBase::AddVertex(double x, double y, double z) {
	TrajVertex v;

	v.calcHeading = true;
	vertex_.push_back(v);

	return UpdateVertex(GetNumberOfVertices() - 1, x, y, z);
}

TrajVertex* PolyLineBase::AddVertex(TrajVertex p) {
	vertex_.push_back(p);

	if (p.calcHeading) {
		return UpdateVertex(GetNumberOfVertices() - 1, p.x, p.y, p.z);
	} else {
		return UpdateVertex(GetNumberOfVertices() - 1, p.x, p.y, p.z, p.h);
	}
}

TrajVertex* PolyLineBase::UpdateVertex(int i, double x, double y, double z) {
	TrajVertex* v = &vertex_[i];

	v->x = x;
	v->y = y;
	v->z = z;

	if (i > 0) {
		TrajVertex* vp = &vertex_[i - 1];

		if (v->calcHeading) {
			// Calulate heading from line segment between this and previous vertices
			if (PointDistance2D(v->x, v->y, vp->x, v->y) < SMALL_NUMBER) {
				// If points conside, use heading of previous vertex
				v->h = vp->h;
			} else {
				v->h = GetAngleInInterval2PI(atan2(v->y - vp->y, v->x - vp->x));
			}
		}

		if (vp->calcHeading) {
			// Update heading of previous vertex now that outgoing line segment is known
			vp->h = v->h;
		}

		// Update polyline length
		double dist = PointDistance2D(x, y, vp->x, vp->y);
		length_ += dist;
	} else if (i == 0) {
		length_ = 0;
	}

	v->s = length_;

	return &vertex_[i];
}

TrajVertex* PolyLineBase::UpdateVertex(int i, double x, double y, double z, double h) {
	TrajVertex* v = &vertex_[i];

	v->h = h;

	UpdateVertex(i, x, y, z);

	return &vertex_[i];
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, double cornerRadius, int startAtIndex) {
	double s_local = 0;
	int i = startAtIndex;

	if (GetNumberOfVertices() < 1) {
		return -1;
	}

	if (s > GetVertex(-1)->s) {
		// end of trajectory
		s = length_;
		s_local = 0;
		i = GetNumberOfVertices() - 1;
	} else {
		for (; i < GetNumberOfVertices() - 1 && vertex_[i + 1].s <= s; i++)
			;

		double s0 = vertex_[i].s;
		s_local = s - s0;
	}

	EvaluateSegmentByLocalS(i, s_local, cornerRadius, pos);
	pos.s = s;

	return i;
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, double cornerRadius) {
	return Evaluate(s, pos, cornerRadius, 0);
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, int startAtIndex) {
	return Evaluate(s, pos, 0.0, startAtIndex);
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos) {
	return Evaluate(s, pos, 0.0, 0);
}

int PolyLineBase::Time2S(double time, double& s) {
	if (GetNumberOfVertices() < 1) {
		return -1;
	}

	// start looking from current index
	int i = vIndex_;

	for (size_t j = 0; j < GetNumberOfVertices(); j++) {
		if (vertex_[i].time <= time && vertex_[i + 1].time > time) {
			double w = (time - vertex_[i].time) / (vertex_[i + 1].time - vertex_[i].time);
			s = vertex_[i].s + w * (vertex_[i + 1].s - vertex_[i].s);
			vIndex_ = i;
			return 0;
		}

		if (++i >= GetNumberOfVertices() - 1) {
			// Reached end of buffer, continue from start
			i = 0;
		}
	}

	// s seems out of range, grab last element
	s = GetVertex(-1)->s;

	return 0;
}

int PolyLineBase::FindClosestPoint(double xin, double yin, TrajVertex& pos, int& index, int startAtIndex) {
	// look along the line segments
	TrajVertex tmpPos;
	double sLocal = 0.0;
	double sLocalMin = 0.0;
	int iMin = startAtIndex;
	double distMin = LARGE_NUMBER;

	// If a teleportation is made by the Ghost, a reset of trajectory has benn made. Hence, we can't look from
	// the usual point ad has to set startAtIndex = 0

	if (startAtIndex > GetNumberOfVertices() - 1) {
		startAtIndex = 0;
		index = 0;
	}

	// Find closest line segment
	for (int i = startAtIndex; i < GetNumberOfVertices() - 1; i++) {
		ProjectPointOnVector2D(xin, yin, vertex_[i].x, vertex_[i].y, vertex_[i + 1].x, vertex_[i + 1].y,
							   tmpPos.x, tmpPos.y);
		double distTmp = PointDistance2D(xin, yin, tmpPos.x, tmpPos.y);

		bool inside = PointInBetweenVectorEndpoints(tmpPos.x, tmpPos.y, vertex_[i].x, vertex_[i].y,
													vertex_[i + 1].x, vertex_[i + 1].y, sLocal);
		if (!inside) {
			// Find combined longitudinal and lateral distance to line endpoint
			// sLocal represent now (outside line segment) distance to closest line segment end point
			distTmp = sqrt(distTmp * distTmp + sLocal * sLocal);
			if (sLocal < 0) {
				sLocal = 0;
			} else {
				sLocal = vertex_[i + 1].s - vertex_[i].s;
			}
		} else {
			// rescale normalized s
			sLocal *= (vertex_[i + 1].s - vertex_[i].s);
		}

		if (distTmp < distMin) {
			iMin = (int)i;
			sLocalMin = sLocal;
			distMin = distTmp;
		}
	}

	if (distMin < LARGE_NUMBER) {
		EvaluateSegmentByLocalS(iMin, sLocalMin, 0.0, pos);
		index = iMin;
		return 0;
	} else {
		return -1;
	}
}

int PolyLineBase::FindPointAhead(double s_start,
								 double distance,
								 TrajVertex& pos,
								 int& index,
								 int startAtIndex) {
	index = Evaluate(s_start + distance, pos, startAtIndex);

	return 0;
}

TrajVertex* PolyLineBase::GetVertex(int index) {
	if (GetNumberOfVertices() < 1) {
		return nullptr;
	}

	if (index == -1) {
		return &vertex_.back();
	} else {
		return &vertex_[index];
	}
}

void PolyLineBase::Reset() {
	vertex_.clear();
	vIndex_ = 0;
	length_ = 0;
}

void PolyLineShape::AddVertex(Position pos, double time, bool calculateHeading) {
	Vertex* v = new Vertex();
	v->pos_ = pos;
	vertex_.push_back(v);
	pline_.AddVertex({pos.GetTrajectoryS(), pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), time, 0.0, 0.0,
					  calculateHeading});
}

int PolyLineShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	double s = 0;
	int i = 0;

	if (pline_.GetNumberOfVertices() < 1) {
		return -1;
	}

	if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_S && p > pline_.GetVertex(-1)->s
		|| ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME && p > pline_.GetVertex(-1)->time) {
		// end of trajectory
		s = GetLength();
		i = (int)vertex_.size() - 1;
	} else {
		for (; i < vertex_.size() - 1
			   && (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_S && pline_.vertex_[i + 1].s < p
				   || ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME && pline_.vertex_[i + 1].time < p);
			 i++)
			;

		if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME) {
			double a = (p - pline_.vertex_[i].time)
					   / (pline_.vertex_[i + 1].time - pline_.vertex_[i].time);	 // a = interpolation factor
			s = pline_.vertex_[i].s + a * (pline_.vertex_[i + 1].s - pline_.vertex_[i].s);
		} else {
			s = p;
		}
	}

	pline_.Evaluate(s, pos, i);

	return 0;
}

double PolyLineShape::GetStartTime() {
	if (vertex_.size() == 0) {
		return 0.0;
	}

	return pline_.vertex_[0].time;
}

double PolyLineShape::GetDuration() {
	if (vertex_.size() == 0) {
		return 0.0;
	}

	return pline_.vertex_.back().time - pline_.vertex_[0].time;
}

double NurbsShape::CoxDeBoor(double x, int i, int k, const std::vector<double>& t) {
	// Inspiration: Nurbs Curve Example @
	// https://nccastaff.bournemouth.ac.uk/jmacey/OldWeb/RobTheBloke/www/opengl_programming.html

	if (k == 1) {
		if (t[i] <= x && x < t[i + 1]) {
			return 1.0;
		}
		return 0.0;
	}

	double den1 = t[i + k - 1] - t[i];
	double den2 = t[i + k] - t[i + 1];
	double eq1 = 0.0;
	double eq2 = 0.0;

	if (den1 > 0) {
		eq1 = ((x - t[i]) / den1) * CoxDeBoor(x, i, k - 1, t);
	}

	if (den2 > 0) {
		eq2 = (t[i + k] - x) / den2 * CoxDeBoor(x, i + 1, k - 1, t);
	}

	return eq1 + eq2;
}

void NurbsShape::CalculatePolyLine() {
	if (ctrlPoint_.size() < 1) {
		return;
	}
	Position tmpRoadPos;

	// Calculate approximate length - to find a reasonable step length

	length_ = 0;
	double steplen = 1.0;  // steplen in meters
	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		ctrlPoint_[i].pos_.ReleaseRelation();
		ctrlPoint_[i].t_ = knot_[i + order_ - 1];
		if (i > 0) {
			length_ += PointDistance2D(ctrlPoint_[i - 1].pos_.GetX(), ctrlPoint_[i - 1].pos_.GetY(),
									   ctrlPoint_[i].pos_.GetX(), ctrlPoint_[i].pos_.GetY());
		}
	}

	if (length_ == 0) {
		throw std::runtime_error("Nurbs zero length - check controlpoints");
	}

	// Calculate arc length
	double newLength = 0.0;
	int nSteps = (int)(1 + length_ / steplen);
	double p_steplen = knot_.back() / nSteps;
	TrajVertex pos = {0, 0, 0, 0, 0, 0, 0, 0, false};
	TrajVertex oldpos = {0, 0, 0, 0, 0, 0, 0, 0, false};
	TrajVertex tmppos = {0, 0, 0, 0, 0, 0, 0, 0, false};

	pline_.Reset();
	for (int i = 0; i < nSteps + 1; i++) {
		double t = i * p_steplen;
		EvaluateInternal(t, pos);

		// Calulate heading from line segment between this and previous vertices
		if (i < nSteps) {
			EvaluateInternal(t + 0.01 * p_steplen, tmppos);
		} else {
			EvaluateInternal(t - 0.01 * p_steplen, tmppos);
		}

		if (PointDistance2D(tmppos.x, tmppos.y, pos.x, pos.y) < SMALL_NUMBER) {
			// If points conside, use heading from polyline
			pos.calcHeading = false;
		} else {
			if (i < nSteps) {
				pos.h = GetAngleInInterval2PI(atan2(tmppos.y - pos.y, tmppos.x - pos.x));
			} else {
				pos.h = GetAngleInInterval2PI(atan2(pos.y - tmppos.y, pos.x - tmppos.x));
			}
		}

		if (i > 0) {
			newLength += PointDistance2D(pos.x, pos.y, oldpos.x, oldpos.y);
		}
		pos.s = newLength;

		// Find max contributing controlpoint for time interpolation
		for (int j = 0; j < ctrlPoint_.size(); j++) {
			if (d_[j] > dPeakValue_[j]) {
				dPeakValue_[j] = d_[j];
				dPeakT_[j] = t;
			}
		}

		pline_.AddVertex(pos);
		pline_.vertex_[i].p = i * p_steplen;
		oldpos = pos;
		// Resolve Z value - from road elevation
		tmpRoadPos.SetInertiaPos(pos.x, pos.y, pos.h);
		pos.z = tmpRoadPos.GetZ();
		pline_.vertex_[i].z = pos.z;
	}

	// Calculate time interpolations
	int currentCtrlPoint = 0;
	for (int i = 0; i < pline_.vertex_.size(); i++) {
		if (pline_.vertex_[i].p >= dPeakT_[currentCtrlPoint + 1]) {
			currentCtrlPoint = MIN(currentCtrlPoint + 1, (int)(ctrlPoint_.size()) - 2);
		}
		double w = (pline_.vertex_[i].p - dPeakT_[currentCtrlPoint])
				   / (dPeakT_[currentCtrlPoint + 1] - dPeakT_[currentCtrlPoint]);
		pline_.vertex_[i].time
			= ctrlPoint_[currentCtrlPoint].time_
			  + w * (ctrlPoint_[currentCtrlPoint + 1].time_ - ctrlPoint_[currentCtrlPoint].time_);
	}

	length_ = newLength;
}

int NurbsShape::EvaluateInternal(double t, TrajVertex& pos) {
	pos.x = pos.y = 0.0;

	// Find knot span
	t = CLAMP(t, knot_[0], knot_.back() - SMALL_NUMBER);

	double rationalWeight = 0.0;

	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		// calculate the effect of this point on the curve
		d_[i] = CoxDeBoor(t, (int)i, order_, knot_);
		rationalWeight += d_[i] * ctrlPoint_[i].weight_;
	}

	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		if (d_[i] > SMALL_NUMBER) {
			// sum effect of CV on this part of the curve
			pos.x += d_[i] * ctrlPoint_[i].pos_.GetX() * ctrlPoint_[i].weight_ / rationalWeight;
			pos.y += d_[i] * ctrlPoint_[i].pos_.GetY() * ctrlPoint_[i].weight_ / rationalWeight;
		}
	}

	return 0;
}

void NurbsShape::AddControlPoint(Position pos, double time, double weight, bool calcHeading) {
	if (calcHeading == false) {
		LOG_ONCE("Info: Explicit orientation in Nurbs trajectory control points not supported yet");
	}
	ctrlPoint_.push_back(ControlPoint(pos, time, weight, true));
	d_.push_back(0);
	dPeakT_.push_back(0);
	dPeakValue_.push_back(0);
}

void NurbsShape::AddKnots(std::vector<double> knots) {
	knot_ = knots;

	if (knot_.back() < SMALL_NUMBER) {
		return;
	}
}

int NurbsShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	if (order_ < 1 || ctrlPoint_.size() < order_ || GetLength() < SMALL_NUMBER) {
		return -1;
	}

	double s = p;

	if (ptype == TRAJ_PARAM_TYPE_TIME) {
		pline_.Time2S(p, s);
	}

	pline_.Evaluate(s, pos, pline_.vIndex_);

	EvaluateInternal(pos.p, pos);

	return 0;
}

double NurbsShape::GetStartTime() {
	if (ctrlPoint_.size() == 0) {
		return 0.0;
	}

	return ctrlPoint_[0].time_;
}

double NurbsShape::GetDuration() {
	if (ctrlPoint_.size() == 0) {
		return 0.0;
	}

	return ctrlPoint_.back().time_ - ctrlPoint_[0].time_;
}

ClothoidShape::ClothoidShape(roadmanager::Position pos,
							 double curv,
							 double curvPrime,
							 double len,
							 double tStart,
							 double tEnd)
	: Shape(ShapeType::CLOTHOID) {
	pos_ = pos;
	spiral_ = new roadmanager::Spiral(0, pos_.GetX(), pos_.GetY(), pos_.GetH(), len, curv,
									  curv + curvPrime * len);
	t_start_ = tStart;
	t_end_ = tEnd;
	pline_.interpolateHeading_ = true;
}

void ClothoidShape::CalculatePolyLine() {
	// Create polyline placeholder representation
	double stepLen = 1.0;
	int steps = (int)(spiral_->GetLength() / stepLen);
	pline_.Reset();
	TrajVertex v;

	for (size_t i = 0; i < steps + 1; i++) {
		if (i < steps) {
			EvaluateInternal((double)i, v);
		} else {
			// Add endpoint of spiral
			EvaluateInternal(spiral_->GetLength(), v);
		}

		// resolve road coordinates to get elevation at point
		pos_.SetInertiaPos(v.x, v.y, v.h, true);
		v.z = pos_.GetZ();

		v.p = v.s = (double)i;
		v.time = t_start_ + (i * stepLen / spiral_->GetLength()) * t_end_;

		pline_.AddVertex(v);
	}
}

int ClothoidShape::EvaluateInternal(double s, TrajVertex& pos) {
	spiral_->EvaluateDS(s, &pos.x, &pos.y, &pos.h);

	return 0;
}

int ClothoidShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME) {
		if (p >= t_start_ && p <= t_end_) {
			double t = p - t_start_;
			// Transform time parameter value into a s value
			p = GetLength() * (t - t_start_) / (t_end_ - t_start_);
		} else {
			LOG("Requested time %.2f outside range [%.2f, %.2f]", p, t_start_, t_end_);
			p = GetLength();
		}
	} else if (p > GetLength()) {
		p = GetLength();
	}

	pline_.Evaluate(p, pos);

	spiral_->EvaluateDS(p, &pos.x, &pos.y, &pos.h);

	pos.s = p;

	return 0;
}

double ClothoidShape::GetStartTime() {
	return t_start_;
}

double ClothoidShape::GetDuration() {
	return t_end_ - t_start_;
}

int Route::AddWaypoint(Position* position) {
	if (minimal_waypoints_.size() > 0) {
		// Keep only one consecutive waypoint per road
		// Keep first specified waypoint for first road
		// then, for following roads, keep the last waypoint.

		if (position->GetTrackId() == minimal_waypoints_.back().GetTrackId()) {
			if (minimal_waypoints_.size() == 1) {
				// Ignore
				LOG("Ignoring additional waypoint for road %d (s %.2f)", position->GetTrackId(),
					position->GetS());
				all_waypoints_.push_back(*position);
				return -1;
			} else	// at least two road-unique waypoints
			{
				// Keep this, remove previous
				LOG("Removing previous waypoint for same road %d (at s %.2f)",
					minimal_waypoints_.back().GetTrackId(), minimal_waypoints_.back().GetS());
				minimal_waypoints_.pop_back();
			}
		}

		// Check that there is a valid path from previous waypoint
		RoadPath* path = new RoadPath(&minimal_waypoints_.back(), position);
		double dist = 0;

		if (path->Calculate(dist, false) == 0) {
			// Path is found by tracing previous nodes
			RoadPath::PathNode* previous = 0;
			std::vector<RoadPath::PathNode*> nodes;

			if (path->visited_.size() > 0) {
				previous = path->visited_.back()->previous;
				nodes.push_back(path->visited_.back());
				while (previous != nullptr) {
					nodes.push_back(previous);
					previous = previous->previous;
				}
			}

			if (nodes.size() > 1) {
				// Add internal waypoints, one for each road along the path
				for (int i = (int)nodes.size() - 1; i >= 1; i--) {
					// Find out lane ID of the connecting road
					Position connected_pos
						= Position(nodes[i - 1]->fromRoad->GetId(), nodes[i - 1]->fromLaneId, 0, 0);
					all_waypoints_.push_back(*position);
					minimal_waypoints_.push_back(connected_pos);
					LOG("Route::AddWaypoint Added intermediate waypoint %d roadId %d laneId %d",
						(int)minimal_waypoints_.size() - 1, connected_pos.GetTrackId(),
						nodes[i - 1]->fromLaneId);
				}
			}

			length_ += dist;
		} else {
			invalid_route_ = true;
		}
	} else {
		// First waypoint, make it the current position
		currentPos_ = *position;
	}
	all_waypoints_.push_back(*position);
	minimal_waypoints_.push_back(*position);
	LOG("Route::AddWaypoint Added waypoint %d: %d, %d, %.2f", (int)minimal_waypoints_.size() - 1,
		position->GetTrackId(), position->GetLaneId(), position->GetS());

	return 0;
}

void Route::CheckValid() {
	if (invalid_route_) {
		LOG("Warning: Route %s is not valid, will be ignored for the default controller.", getName().c_str());
		minimal_waypoints_.clear();
	}
}

Road* Route::GetRoadAtOtherEndOfConnectingRoad(Road* incoming_road) {
	Road* connecting_road = Position::GetOpenDrive()->GetRoadById(GetTrackId());
	Junction* junction = Position::GetOpenDrive()->GetJunctionById(connecting_road->GetJunction());

	if (junction == 0) {
		LOG("Unexpected: Road %d not a connecting road", connecting_road->GetId());
		return 0;
	}

	return junction->GetRoadAtOtherEndOfConnectingRoad(connecting_road, incoming_road);
}

int Route::GetDirectionRelativeRoad() {
	return GetWayPointDirection(waypoint_idx_);
}

int Route::GetWayPointDirection(int index) {
	if (minimal_waypoints_.size() == 0 || index < 0 || index >= minimal_waypoints_.size()) {
		LOG("Waypoint index %d out of range (%d)", index, minimal_waypoints_.size());
		return 0;
	}

	if (minimal_waypoints_.size() == 1) {
		LOG("Only one waypoint, no direction");
		return 0;
	}

	OpenDrive* od = minimal_waypoints_[index].GetOpenDrive();
	Road* road = od->GetRoadById(minimal_waypoints_[index].GetTrackId());
	if (road == nullptr) {
		LOG("Waypoint %d invalid road id %d!", index, minimal_waypoints_[index].GetTrackId());
		return 0;
	}

	int direction = 0;
	Position* pos2 = nullptr;

	// Looking in the direction of heading
	direction
		= minimal_waypoints_[index].GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0
			  ? -1
			  : 1;

	if (index < minimal_waypoints_.size() - 1) {
		// Looking in the direction of heading
		direction
			= currentPos_.GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0 ? -1 : 1;

		// Look at next waypoint
		pos2 = GetWaypoint(index + 1);
	} else if (index > 0) {
		// Looking in the opposite direction of heading
		direction
			= currentPos_.GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0 ? 1 : -1;

		// Look at previous waypoint
		pos2 = GetWaypoint(index - 1);
	}

	if (direction == 1 && road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))
		|| direction == -1
			   && road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		// Expected case, route direction aligned with waypoint headings
		return 1;
	} else if (road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))
			   && road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		LOG("Road %d connects to both ends of road %d using relative heading of waypoint", pos2->GetTrackId(),
			road->GetId());
		return direction;
	} else if (road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		return 1 * direction;
	} else if (road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		return -1 * direction;
	}

	LOG("Unexpected case, failed to find out direction of route (from road id %d)", road->GetId());

	return direction;
}

void Route::setName(std::string name) {
	this->name_ = name;
}

std::string Route::getName() {
	return name_;
}

void RMTrajectory::Freeze() {
	if (shape_->type_ == Shape::ShapeType::POLYLINE) {
		PolyLineShape* pline = (PolyLineShape*)shape_;

		for (size_t i = 0; i < pline->vertex_.size(); i++) {
			Position* pos = &pline->vertex_[i]->pos_;
			pos->ReleaseRelation();

			if (pline->pline_.vertex_[i].calcHeading) {
				pline->pline_.UpdateVertex((int)i, pos->GetX(), pos->GetY(), pos->GetZ());
			} else {
				pline->pline_.UpdateVertex((int)i, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetH());
			}
		}
	} else if (shape_->type_ == Shape::ShapeType::CLOTHOID) {
		ClothoidShape* clothoid = (ClothoidShape*)shape_;

		clothoid->pos_.ReleaseRelation();

		clothoid->spiral_->SetX(clothoid->pos_.GetX());
		clothoid->spiral_->SetY(clothoid->pos_.GetY());
		clothoid->spiral_->SetHdg(clothoid->pos_.GetH());

		clothoid->CalculatePolyLine();
	} else {
		NurbsShape* nurbs = (NurbsShape*)shape_;

		nurbs->CalculatePolyLine();
	}
}

double RMTrajectory::GetTimeAtS(double s) {
	// Find out corresponding time-value using polyline representation
	TrajVertex v;
	shape_->pline_.Evaluate(s, v);

	return v.time;
}

double RMTrajectory::GetStartTime() {
	return shape_->GetStartTime();
}

double RMTrajectory::GetDuration() {
	return shape_->GetDuration();
}
