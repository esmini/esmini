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
  * When used standalone (outside ScenarioEngine) the road manager is initialized via the Position class like this:
  *   roadmanager::Position::LoadOpenDrive("example.xodr");
  *
  * Simplest use case is to put a vehicle on the road and simply move it forward along the road, e.g:
  *
  *   car->pos = new roadmanager::Position(3, -2, 10, 0);
  *   while(true)
  *   {
  *	    car->pos->MoveAlongS(0.1);
  *   }
  *
  * The first line will create a Position object initialized at road with ID = 3, in lane = -2 and at lane offset = 0
  * Then the position is updated along that road and lane, moving 10 cm at a time.
  *
  * A bit more realistic example:
  *
  *   car->pos = new roadmanager::Position(odrManager->GetRoadByIdx(0)->GetId(), -2, 10, 0);
  *   while(true)
  *   {
  *	    car->pos->MoveAlongS(speed * dt);
  *   }
  *
  * Here we refer to the ID of the first road in the network. And instead of static delta movement, the distance
  * is a function of speed and delta time since last update.
  *
  */

#include <iostream>
#include <cstring>
#include <random>
#include <time.h>
#include <limits>
#include <algorithm>
#include <map>
#include <sstream>

#include "RoadManager.hpp"
#include "odrSpiral.h"
#include "pugixml.hpp"
#include "CommonMini.hpp"


using namespace std;
using namespace roadmanager;

#define CURV_ZERO 0.00001
#define MAX_TRACK_DIST 10
#define OSI_POINT_CALC_STEPSIZE 1 // [m]
#define OSI_TANGENT_LINE_TOLERANCE 0.01 // [m]
#define OSI_POINT_DIST_SCALE 0.025
#define ROADMARK_WIDTH_STANDARD 0.15
#define ROADMARK_WIDTH_BOLD 0.20


static int g_Lane_id;
static int g_Laneb_id;

const char* object_type_str[] =
{
	"barrier",
	"bike",
	"building",
	"bus",
	"car",
	"crosswalk",
	"gantry",
	"motorbike",
	"none",
	"obstacle",
	"parkingspace",
	"patch",
	"pedestrian",
	"pole",
	"railing",
	"roadmark",
	"soundbarrier",
	"streetlamp",
	"trafficisland",
	"trailer",
	"train",
	"tram",
	"tree",
	"van",
	"vegetation",
	"wind"
};

const std::map<std::string, Signal::OSIType> Signal::types_mapping_ = {
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
	{"TYPE_PRIORITY_TO_OPPOSITE_DIRECTION_UPSIDE_DOWN", Signal::TYPE_PRIORITY_TO_OPPOSITE_DIRECTION_UPSIDE_DOWN},
	{"TYPE_PRESCRIBED_LEFT_TURN", Signal::TYPE_PRESCRIBED_LEFT_TURN},
	{"TYPE_PRESCRIBED_RIGHT_TURN", Signal::TYPE_PRESCRIBED_RIGHT_TURN},
	{"TYPE_PRESCRIBED_STRAIGHT", Signal::TYPE_PRESCRIBED_STRAIGHT},
	{"TYPE_PRESCRIBED_RIGHT_WAY", Signal::TYPE_PRESCRIBED_RIGHT_WAY},
	{"TYPE_PRESCRIBED_LEFT_WAY", Signal::TYPE_PRESCRIBED_LEFT_WAY},
	{"TYPE_PRESCRIBED_RIGHT_TURN_AND_STRAIGHT", Signal::TYPE_PRESCRIBED_RIGHT_TURN_AND_STRAIGHT},
	{"TYPE_PRESCRIBED_LEFT_TURN_AND_STRAIGHT", Signal::TYPE_PRESCRIBED_LEFT_TURN_AND_STRAIGHT},
	{"TYPE_PRESCRIBED_LEFT_TURN_AND_RIGHT_TURN", Signal::TYPE_PRESCRIBED_LEFT_TURN_AND_RIGHT_TURN},
	{"TYPE_PRESCRIBED_LEFT_TURN_RIGHT_TURN_AND_STRAIGHT", Signal::TYPE_PRESCRIBED_LEFT_TURN_RIGHT_TURN_AND_STRAIGHT},
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
	{"TYPE_BICYCLES_PEDESTRIANS_SEPARATED_RIGHT_ONLY", Signal::TYPE_BICYCLES_PEDESTRIANS_SEPARATED_RIGHT_ONLY},
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
	{"TYPE_PRIORITY_OVER_OPPOSITE_DIRECTION_UPSIDE_DOWN", Signal::TYPE_PRIORITY_OVER_OPPOSITE_DIRECTION_UPSIDE_DOWN},
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
	{"TYPE_SIDEWALK_PERPENDICULAR_HALF_PARKING_RIGHT", Signal::TYPE_SIDEWALK_PERPENDICULAR_HALF_PARKING_RIGHT},
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
	{"TYPE_DIRECTION_PREANNOUNCEMENT_HIGHWAY_ENTRIES", Signal::TYPE_DIRECTION_PREANNOUNCEMENT_HIGHWAY_ENTRIES},
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
	{"TYPE_TRAFFIC_LIGHT_GREEN_ARROW", Signal::TYPE_TRAFFIC_LIGHT_GREEN_ARROW}
};

Signal::Signal(double s, double t, int id, std::string name, bool dynamic, Orientation orientation, double z_offset, std::string country, int osi_type,
	std::string type, std::string subtype, std::string value_str, std::string unit, double height, double width, std::string text, double h_offset,
	double pitch, double roll, double x, double y, double z, double h) : s_(s), t_(t), id_(id), name_(name), dynamic_(dynamic), orientation_(orientation), z_offset_(z_offset),
	country_(country), osi_type_(osi_type), type_(type), subtype_(subtype), value_str_(value_str), unit_(unit), height_(height), width_(width), text_(text),
	h_offset_(h_offset), pitch_(pitch), roll_(roll), length_(0), RoadObject(x, y, z, h)
{
	value_ = strtod(value_str);
}

Signal::OSIType Signal::GetOSITypeFromString(const std::string& type)
{
	if (types_mapping_.count(type) != 0)
	{
		return types_mapping_.find(type)->second;
	}
	return Signal::TYPE_UNKNOWN;
}

std::string Signal::GetCombinedTypeSubtypeValueStr(std::string type, std::string subtype, std::string value)
{
	std::string str;

	if (!type.empty())
	{
		str = type;

		if (!subtype.empty())
		{
			str += "." + subtype;
		}

		if (!value.empty())
		{
			str += "-" + value;
		}
	}

	return str;
}

int roadmanager::GetNewGlobalLaneId()
{
	int returnvalue = g_Lane_id;
	g_Lane_id++;
	return returnvalue;
}

int roadmanager::GetNewGlobalLaneBoundaryId()
{
	int returnvalue = g_Laneb_id;
	g_Laneb_id++;
	return returnvalue;
}

int roadmanager::CheckOverlapingOSIPoints(OSIPoints* first_set, OSIPoints* second_set, double tolerance)
{
	std::vector<double> distances;
	int retvalue = 0;

	if (first_set == nullptr || second_set == nullptr)
	{
		return -1;
	}

	distances.push_back(PointDistance2D(first_set->GetPoint(0).x,
									first_set->GetPoint(0).y,
									second_set->GetPoint(0).x,
									second_set->GetPoint(0).y));
	distances.push_back(PointDistance2D(first_set->GetPoint(0).x,
									first_set->GetPoint(0).y,
									second_set->GetPoint(second_set->GetNumOfOSIPoints()-1).x,
									second_set->GetPoint(second_set->GetNumOfOSIPoints()-1).y));
	distances.push_back(PointDistance2D(first_set->GetPoint(first_set->GetNumOfOSIPoints()-1).x,
									first_set->GetPoint(first_set->GetNumOfOSIPoints()-1).y,
									second_set->GetPoint(0).x,
									second_set->GetPoint(0).y));
	distances.push_back(PointDistance2D(first_set->GetPoint(first_set->GetNumOfOSIPoints()-1).x,
									first_set->GetPoint(first_set->GetNumOfOSIPoints()-1).y,
									second_set->GetPoint(second_set->GetNumOfOSIPoints()-1).x,
									second_set->GetPoint(second_set->GetNumOfOSIPoints()-1).y));


	for (int i=0; i<distances.size();i++)
	{
		if (distances[i] < tolerance)
		{
			retvalue++;
		}
	}

	return retvalue;
}


double Polynomial::Evaluate(double p)
{
	p *= p_scale_;

	return (a_ + p * b_ + p * p*c_ + p * p*p*d_);
}

double Polynomial::EvaluatePrim(double p)
{
	p *= p_scale_;

	return (b_ + 2 * p*c_ + 3 * p*p*d_);
}

double Polynomial::EvaluatePrimPrim(double p)
{
	p *= p_scale_;

	return (2 * c_ + 6 * p*d_);
}

void Polynomial::Set(double a, double b, double c, double d, double p_scale)
{
	a_ = a;
	b_ = b;
	c_ = c;
	d_ = d;
	p_scale_ = p_scale;
}

PointStruct& OSIPoints::GetPoint(int i)
{
	if (point_.size() <= i || point_.size() == 0)
	{
		throw std::runtime_error("OSIPoints::GetPoint(int i) -> exceeds index");
	}
	else if (i < 0)
	{
		throw std::runtime_error("OSIPoints::GetXFromIdx(int i) -> index must be larger than 0");
	}
	else
	{
		return point_[i];
	}
}

double OSIPoints::GetXfromIdx(int i)
{
	if(point_.size() <= i || point_.size() == 0)
	{
		throw std::runtime_error("OSIPoints::GetXFromIdx(int i) -> exceeds index");
	}
	else if(i < 0)
	{
		throw std::runtime_error("OSIPoints::GetXFromIdx(int i) -> index must be larger than 0");
	}
	else
	{
		return point_[i].x;
	}
}

double OSIPoints::GetYfromIdx(int i)
{
	if (point_.size() <= i || point_.size() == 0)
	{
		throw std::runtime_error("OSIPoints::GetYFromIdx(int i) -> exceeds index");
	}
	else if(i < 0)
	{
		throw std::runtime_error("OSIPoints::GetYFromIdx(int i) -> index must be larger than 0");
	}
	else
	{
		return point_[i].y;
	}
}

double OSIPoints::GetZfromIdx(int i)
{
	if (point_.size() <= i || point_.size() == 0)
	{
		throw std::runtime_error("OSIPoints::GetZFromIdx(int i) -> exceeds index");
	}
	else if(i < 0)
	{
		throw std::runtime_error("OSIPoints::GetZFromIdx(int i) -> index must be larger than 0");
	}
	else
	{
		return point_[i].z;
	}
}

int OSIPoints::GetNumOfOSIPoints()
{
	return (int)point_.size();
}

double OSIPoints::GetLength()
{
	double length = 0;
	for (int i=0; i<point_.size()-1; i++)
	{
		length += PointDistance2D(point_[i].x,point_[i].y,point_[i+1].x,point_[i+1].y);
	}
	return length;
}

void Geometry::Print()
{
	LOG("Geometry virtual Print\n");
}

void Geometry::EvaluateDS(double ds, double *x, double *y, double *h)
{
	(void)ds; (void)x; (void)y; (void)h;
	LOG("Geometry virtual Evaluate\n");
}

void Line::Print()
{
	LOG("Line x: %.2f, y: %.2f, h: %.2f length: %.2f\n", GetX(), GetY(), GetHdg(), GetLength());
}

void Line::EvaluateDS(double ds, double *x, double *y, double *h)
{
	*h = GetHdg();
	*x = GetX() + ds * cos(*h);
	*y = GetY() + ds * sin(*h);
}

double Arc::GetRadius()
{
	if (abs(curvature_) < SMALL_NUMBER)
	{
		return LARGE_NUMBER;
	}
	else
	{
		return std::fabs(1.0 / curvature_);
	}
}

void Arc::Print()
{
	LOG("Arc x: %.2f, y: %.2f, h: %.2f curvature: %.2f length: %.2f\n", GetX(), GetY(), GetHdg(), curvature_, GetLength());
}

void Arc::EvaluateDS(double ds, double *x, double *y, double *h)
{
	double x_local = 0.0;
	double y_local = 0.0;

	if (abs(curvature_) < SMALL_NUMBER)  // line
	{
		*x = GetX() + ds * cos(GetHdg());
		*y = GetY() + ds * sin(GetHdg());
		*h = GetHdg();
	}
	else
	{
		// arc_length = angle * radius -> angle = arc_length / radius = arc_length * curvature
		double angle = ds * curvature_;

		// Now calculate x, y in a local unit circle coordinate system
		if (curvature_ < 0)
		{
			// starting from 90 degrees going clockwise
			x_local = cos(angle + M_PI / 2.0);
			y_local = sin(angle + M_PI / 2.0) - 1;  // -1 transform to y = 0
		}
		else
		{
			// starting from -90 degrees going counter clockwise
			x_local = cos(angle + 3.0 * M_PI_2);
			y_local = sin(angle + 3.0 * M_PI_2) + 1;  // +1 transform to y = 0
		}

		// Rotate according to heading and scale according to radius
		*x = GetX() + GetRadius() * (x_local * cos(GetHdg()) - y_local * sin(GetHdg()));
		*y = GetY() + GetRadius() * (x_local * sin(GetHdg()) + y_local * cos(GetHdg()));
		*h = GetHdg() + angle;
	}
}

Spiral::Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end) :
	Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_SPIRAL),
	curv_start_(curv_start), curv_end_(curv_end), c_dot_(0.0), x0_(0.0), y0_(0.0), h0_(0.0), s0_(0.0), arc_(0), line_(0)
{
	SetCDot((curv_end_ - curv_start_) / length_);

	if (fabs(GetCDot()) < SMALL_NUMBER)
	{
		// constant radius => clothoid is actually a line or an arc
		if (fabs(this->GetCurvStart()) < SMALL_NUMBER)  // Line
		{
			line_ = new Line(s, x, y, hdg, length);
		}
		else  // Arc
		{
			arc_ = new Arc(s, x, y, hdg, length, curv_start);
		}
	}
	else
	{
		if (fabs(curv_start_) > SMALL_NUMBER)
		{
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

void Spiral::Print()
{
	LOG("Spiral x: %.2f, y: %.2f, h: %.2f start curvature: %.4f end curvature: %.4f length: %.2f %s\n",
		GetX(), GetY(), GetHdg(), GetCurvStart(), GetCurvEnd(), GetLength(),
		arc_ != 0 ? " - actually an Arc" : line_ != 0 ? "- actually a Line" : "");
}

void Spiral::EvaluateDS(double ds, double* x, double* y, double* h)
{
	double xTmp, yTmp, t;

	if (line_ != 0)
	{
		line_->EvaluateDS(ds, x, y, h);
	}
	else if (arc_ != 0)
	{
		arc_->EvaluateDS(ds, x, y, h);
	}
	else
	{
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

double Spiral::EvaluateCurvatureDS(double ds)
{
	if (line_ != 0)
	{
		return 0.0;
	}
	else if (arc_ != 0)
	{
		return arc_->GetCurvature();
	}
	else
	{
		return (curv_start_ + (ds / GetLength()) * (curv_end_ - curv_start_));
	}
}

void Spiral::SetX(double x)
{
	if (line_ != 0)
	{
		line_->SetX(x);
	}
	else if (arc_ != 0)
	{
		arc_->SetX(x);
	}
	else
	{
		x_ = x;
	}
}

void Spiral::SetY(double y)
{
	if (line_ != 0)
	{
		line_->SetY(y);
	}
	else if (arc_ != 0)
	{
		arc_->SetY(y);
	}
	else
	{
		y_ = y;
	}
}

void Spiral::SetHdg(double h)
{
	if (line_ != 0)
	{
		line_->SetHdg(h);
	}
	else if (arc_ != 0)
	{
		arc_->SetHdg(h);
	}
	else
	{
		hdg_ = h;
	}
}

Poly3::Poly3(double s, double x, double y, double hdg, double length, double a, double b, double c, double d) :
	Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_POLY3), umax_(0.0)
{
	poly3_.Set(a, b, c, d);

	double xTmp = 0;
	double yTmp = 0;

	EvaluateDSLocal(GetLength()-SMALL_NUMBER, xTmp, yTmp);
	SetUMax(xTmp);
}

void Poly3::Print()
{
	LOG("Poly3 x: %.2f, y: %.2f, h: %.2f length: %.2f a: %.2f b: %.2f c: %.2f d: %.2f\n",
		GetX(), GetY(), GetHdg(), GetLength(), poly3_.GetA(), poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

void Poly3::EvaluateDSLocal(double ds, double &u, double &v)
{
	double distTmp = 0;
	double steplen = MIN(10, ds);  // along u axis - to be tuned

	u = v = 0;

	if (ds > length_ - SMALL_NUMBER)
	{
		u = umax_;
		v = poly3_.Evaluate(u);
	}
	else if (ds > SMALL_NUMBER)
	{
		for (double uTmp = 0; uTmp < length_; uTmp += steplen)
		{
			double vTmp = poly3_.Evaluate(uTmp);
			double delta = sqrt((uTmp - u) * (uTmp - u) + (vTmp - v) * (vTmp - v));

			if (distTmp + delta > ds)
			{
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

void Poly3::EvaluateDS(double ds, double *x, double *y, double *h)
{
	double u_local = 0;
	double v_local = 0;

	EvaluateDSLocal(ds, u_local, v_local);

	*x = GetX() + u_local * cos(GetHdg()) - v_local * sin(GetHdg());
	*y = GetY() + u_local * sin(GetHdg()) + v_local * cos(GetHdg());
	*h = GetHdg() + atan(poly3_.EvaluatePrim(u_local));
}

double Poly3::EvaluateCurvatureDS(double ds)
{
	return poly3_.EvaluatePrimPrim(ds);
}

void ParamPoly3::Print()
{
	LOG("ParamPoly3 x: %.2f, y: %.2f, h: %.2f length: %.2f U: %.8f, %.8f, %.8f, %.8f V: %.8f, %.8f, %.8f, %.8f\n",
		GetX(), GetY(), GetHdg(), GetLength(),
		poly3U_.GetA(), poly3U_.GetB(), poly3U_.GetC(), poly3U_.GetD(),
		poly3V_.GetA(), poly3V_.GetB(), poly3V_.GetC(), poly3V_.GetD()
	);
}

void ParamPoly3::EvaluateDS(double ds, double *x, double *y, double *h)
{
	double p = S2P(ds);
	double hdg = GetHdg();

	double u_local = poly3U_.Evaluate(p);
	double v_local = poly3V_.Evaluate(p);

	*x = GetX() + u_local * cos(hdg) - v_local * sin(hdg);
	*y = GetY() + u_local * sin(hdg) + v_local * cos(hdg);
	*h = hdg + atan2(poly3V_.EvaluatePrim(p), poly3U_.EvaluatePrim(p));
}

double ParamPoly3::EvaluateCurvatureDS(double ds)
{
	double up = poly3U_.EvaluatePrim(ds);
	double upp = poly3U_.EvaluatePrimPrim(ds);
	double vp = poly3V_.EvaluatePrim(ds);
	double vpp = poly3V_.EvaluatePrimPrim(ds);
	double denominator = pow(pow(up, 2) + pow(vp, 2), 1.5);

	// https://en.wikipedia.org/wiki/Curvature
	// see section "In terms of a general parametrization"

	if (denominator < SMALL_NUMBER)
	{
		return 0;  // derivatives is 0 -> no curvature/straight line
	}

	return (up * vpp - vp * upp) / denominator;
}

void ParamPoly3::calcS2PMap(PRangeType p_range)
{
	double len = 0;
	double p_step_len = 1.0 / double(PARAMPOLY3_STEPS);
	double p = 0;

	if (p_range == PRangeType::P_RANGE_ARC_LENGTH)
	{
		p_step_len = length_/(PARAMPOLY3_STEPS);
	}

	// Calculate actual arc length of the curve
	s2p_map_[0][0] = 0;
	for (size_t i = 1; i < PARAMPOLY3_STEPS+1; i++)
	{
		p += p_step_len;

		double pm = p - 0.5 * p_step_len; // midpoint method
		double integrator = sqrt(
			pow(3 * poly3U_.GetD() * pm * pm + 2 * poly3U_.GetC() * pm + poly3U_.GetB(), 2) +
			pow(3 * poly3V_.GetD() * pm * pm + 2 * poly3V_.GetC() * pm + poly3V_.GetB(), 2));

		len += p_step_len * integrator;
		s2p_map_[i][0] = len;
	}

	// Map length (ds) to p for each sub-segment, adjust for incorrect length attribute
	double scale_factor;
	scale_factor = length_ / len;

	for (size_t i = 0; i < PARAMPOLY3_STEPS+1; i++)
	{
		s2p_map_[i][0] *= scale_factor;
		s2p_map_[i][1] = i * length_ / PARAMPOLY3_STEPS;
	}
}

double ParamPoly3::S2P(double s)
{
	for (size_t i = 0; i < PARAMPOLY3_STEPS; i++)
	{
		if (s2p_map_[i + 1][0] > s)
		{
			double w = (s - s2p_map_[i][0]) / (s2p_map_[i + 1][0] - s2p_map_[i][0]);
			return s2p_map_[i][1] + w * (s2p_map_[i + 1][1] - s2p_map_[i][1]);
		}
	}
	return s2p_map_[PARAMPOLY3_STEPS][1];
}

void Elevation::Print()
{
	LOG("Elevation: s: %.2f A: %.4f B: %.4f C: %.4f D: %.4f\n",
		GetS(), poly3_.GetA(), poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

void LaneLink::Print()
{
	LOG("LaneLink type: %d id: %d\n", type_, id_);
}


void Lane::SetGlobalId()
{
	global_id_ = GetNewGlobalLaneId();
}

void LaneBoundaryOSI::SetGlobalId()
{
	global_id_ = GetNewGlobalLaneBoundaryId();
}

void LaneRoadMarkTypeLine::SetGlobalId()
{
	global_id_ = GetNewGlobalLaneBoundaryId();
}

LaneWidth *Lane::GetWidthByIndex(int index)
{
	if(lane_width_.size() <= index || lane_width_.size() == 0)
	{
		throw std::runtime_error("Lane::GetWidthByIndex(int index) -> exceeds index");
	}
	else if(lane_width_.size() < 0)
	{
		throw std::runtime_error("Lane::GetWidthByIndex(int index) -> index must be larger than 0");
	}
	else
	{
		return lane_width_[index];
	}
}

LaneWidth *Lane::GetWidthByS(double s)
{
	if (lane_width_.size() == 0)
	{
		return 0;  // No lanewidth defined
	}
	for (int i=0; i+1<(int)lane_width_.size(); i++)
	{
		if (s < lane_width_[i + 1]->GetSOffset())
		{
			return lane_width_[i];
		}
	}
	return lane_width_.back();
}

void Lane::AddLaneWidth(LaneWidth* lane_width)
{
	if (lane_width_.size() > 0 &&
		lane_width->GetSOffset() < lane_width_.back()->GetSOffset())
	{
		for (size_t i = 0; i < lane_width_.size(); i++)
		{
			if (lane_width->GetSOffset() < lane_width_[i]->GetSOffset())
			{
				lane_width_.insert(lane_width_.begin() + i, lane_width);
				return;
			}
		}
	}

	lane_width_.push_back(lane_width);
}

void Lane::AddLaneRoadMark(LaneRoadMark* lane_roadMark)
{
	if (lane_roadMark_.size() > 0 &&
		lane_roadMark->GetSOffset() < lane_roadMark_.back()->GetSOffset())
	{
		for (size_t i = 0; i < lane_roadMark_.size(); i++)
		{
			if (lane_roadMark->GetSOffset() < lane_roadMark_[i]->GetSOffset())
			{
				lane_roadMark_.insert(lane_roadMark_.begin() + i, lane_roadMark);
				return;
			}
		}
	}

	lane_roadMark_.push_back(lane_roadMark);
}


LaneLink *Lane::GetLink(LinkType type)
{
	for (int i=0; i<(int)link_.size(); i++)
	{
		LaneLink *l = link_[i];
		if (l->GetType() == type)
		{
			return l;
		}
	}
	return 0; // No link of requested type exists
}

void LaneWidth::Print()
{
	LOG("LaneWidth: sOffset: %.2f, a: %.2f, b: %.2f, c: %.2f, d: %.2f\n",
		s_offset_, poly3_.GetA(), poly3_.GetB(), poly3_.GetC(), poly3_.GetD());
}

LaneRoadMark* Lane::GetLaneRoadMarkByIdx(int idx)
{
	if(lane_roadMark_.size() <= idx || lane_roadMark_.size() == 0)
	{
		throw std::runtime_error("Lane::GetLaneRoadMarkByIdx(int idx) -> exceeds index");
	}
	else if(lane_roadMark_.size() < 0)
	{
		throw std::runtime_error("Lane::GetLaneRoadMarkByIdx(int idx) -> index must be larger than 0");
	}
	else
	{
		return lane_roadMark_[idx];
	}
}

std::vector<int> Lane::GetLineGlobalIds()
{
	std::vector<int> line_ids;
	for (int i = 0; i<GetNumberOfRoadMarks(); i++)
	{
		LaneRoadMark* laneroadmark =  GetLaneRoadMarkByIdx(i);
		for (int j = 0; j<laneroadmark->GetNumberOfRoadMarkTypes(); j++)
		{
			LaneRoadMarkType* laneroadmarktype = laneroadmark->GetLaneRoadMarkTypeByIdx(j);

			for (int h = 0; h<laneroadmarktype->GetNumberOfRoadMarkTypeLines(); h++)
			{
				LaneRoadMarkTypeLine* laneroadmarktypeline =  laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(h);
				line_ids.push_back(laneroadmarktypeline->GetGlobalId());
			}
		}
	}

	return line_ids;
}

int Lane::GetLaneBoundaryGlobalId()
{
	if (lane_boundary_)
	{
		return lane_boundary_->GetGlobalId();
	}
	else
	{
		return -1;
	}
}

RoadMarkColor LaneRoadMark::ParseColor(pugi::xml_node node)
{
	RoadMarkColor color = RoadMarkColor::UNDEFINED;

	if (node.attribute("color") != 0 && strcmp(node.attribute("color").value(), ""))
	{
		if (!strcmp(node.attribute("color").value(), "standard"))
		{
			color = RoadMarkColor::STANDARD_COLOR;
		}
		else  if (!strcmp(node.attribute("color").value(), "blue"))
		{
			color = RoadMarkColor::BLUE;
		}
		else  if (!strcmp(node.attribute("color").value(), "green"))
		{
			color = RoadMarkColor::GREEN;
		}
		else  if (!strcmp(node.attribute("color").value(), "red"))
		{
			color = RoadMarkColor::RED;
		}
		else  if (!strcmp(node.attribute("color").value(), "white"))
		{
			color = RoadMarkColor::WHITE;
		}
		else  if (!strcmp(node.attribute("color").value(), "yellow"))
		{
			color = RoadMarkColor::YELLOW;
		}
		else
		{
			color = RoadMarkColor::UNKNOWN;
			LOG("Unexpected and unsupported roadmark color %s", node.attribute("color").value());
		}
	}

	return color;
}

std::string LaneRoadMark::RoadMarkColor2Str(RoadMarkColor color)
{
	if (color == RoadMarkColor::BLUE)
	{
		return "blue";
	}
	else if (color == RoadMarkColor::GREEN)
	{
		return "green";
	}
	else if (color == RoadMarkColor::RED)
	{
		return "red";
	}
	else if (color == RoadMarkColor::STANDARD_COLOR)
	{
		return "standard";
	}
	else if (color == RoadMarkColor::WHITE)
	{
		return "white";
	}
	else if (color == RoadMarkColor::YELLOW)
	{
		return "yellow";
	}
	else if (color == RoadMarkColor::UNDEFINED)
	{
		return "undefined";
	}

	return "Unrecognized color id: "  + std::to_string(static_cast<int>(color));
}

LaneRoadMarkType* LaneRoadMark::GetLaneRoadMarkTypeByIdx(int idx)
{
	if (idx < (int)lane_roadMarkType_.size())
	{
		return lane_roadMarkType_[idx];
	}

	return 0;
}

void LaneRoadMark::AddType(LaneRoadMarkType* lane_roadMarkType)
{
	lane_roadMarkType_.push_back(lane_roadMarkType);
}

LaneRoadMarkTypeLine* LaneRoadMarkType::GetLaneRoadMarkTypeLineByIdx(int idx)
{
	if (idx < (int)lane_roadMarkTypeLine_.size())
	{
		return lane_roadMarkTypeLine_[idx];
	}

	return 0;
}

void LaneRoadMarkType::AddLine(LaneRoadMarkTypeLine *lane_roadMarkTypeLine)
{
	lane_roadMarkTypeLine->SetGlobalId();

	if (lane_roadMarkTypeLine_.size() > 0 &&
		lane_roadMarkTypeLine->GetSOffset() < lane_roadMarkTypeLine_.back()->GetSOffset())
	{
		for (size_t i = 0; i < lane_roadMarkTypeLine_.size(); i++)
		{
			if (lane_roadMarkTypeLine->GetSOffset() < lane_roadMarkTypeLine_[i]->GetSOffset())
			{
				lane_roadMarkTypeLine_.insert(lane_roadMarkTypeLine_.begin() + i, lane_roadMarkTypeLine);
				return;
			}
		}
	}

	lane_roadMarkTypeLine_.push_back(lane_roadMarkTypeLine);
}

void Lane::SetLaneBoundary(LaneBoundaryOSI *lane_boundary)
{
	lane_boundary->SetGlobalId();
	lane_boundary_ = lane_boundary;
}

void LaneOffset::Print()
{
	LOG("LaneOffset s %.2f a %.4f b %.2f c %.2f d %.2f length %.2f\n",
		s_, polynomial_.GetA(), polynomial_.GetB(), polynomial_.GetC(), polynomial_.GetD(), length_);
}

double LaneOffset::GetLaneOffset(double s)
{
	return (polynomial_.Evaluate(s - s_));
}

double LaneOffset::GetLaneOffsetPrim(double s)
{
	return (polynomial_.EvaluatePrim(s - s_));
}

void Lane::Print()
{
	LOG("Lane: %d, type: %d, level: %d\n", id_, type_, level_);

	for (size_t i = 0; i < link_.size(); i++)
	{
		link_[i]->Print();
	}

	for (size_t i=0; i<lane_width_.size(); i++)
	{
		lane_width_[i]->Print();
	}
}

bool Lane::IsCenter()
{
	if (GetId() == 0)
	{
		return true;  // Ref lane no width -> no driving
	}
	else
	{
		return false;
	}
}
bool Lane::IsType(Lane::LaneType type)
{
	if (GetId() == 0)
	{
		return false;  // Ref lane no width -> no driving
	}

	return bool(type_ & type);
}

bool Lane::IsDriving()
{
	if (GetId() == 0)
	{
		return false;  // Ref lane no width -> no driving
	}

	return bool(type_ & Lane::LaneType::LANE_TYPE_ANY_DRIVING);
}

LaneSection* Road::GetLaneSectionByIdx(int idx)
{
	if (idx >= 0 && idx < lane_section_.size())
	{
		return lane_section_[idx];
	}
	else
	{
		return 0;
	}
}

int Road::GetLaneSectionIdxByS(double s, int start_at)
{
	if (start_at < 0 || start_at > lane_section_.size() - 1)
	{
		return -1;
	}

	LaneSection *lane_section = lane_section_[start_at];
	size_t i = start_at;

	if (s < lane_section->GetS() && start_at > 0)
	{
		// Look backwards
		for (i = start_at - 1; i > 0; i--)  // No need to check the first one
		{
			lane_section = GetLaneSectionByIdx((int)i);
			if (s > lane_section->GetS())
			{
				break;
			}
		}
	}
	else
	{
		// look forward
		for (i = start_at; i < GetNumberOfLaneSections()-1; i++) // No need to check the last one
		{
			lane_section = GetLaneSectionByIdx((int)i);
			if (s < lane_section->GetS() + lane_section->GetLength())
			{
				break;
			}
		}
	}

	return (int)i;
}

int Road::GetLaneInfoByS(double s, int start_lane_section_idx, int start_lane_id, LaneInfo& lane_info, int laneTypeMask)
{
	lane_info.lane_section_idx_ = start_lane_section_idx;
	lane_info.lane_id_ = start_lane_id;

	if (lane_info.lane_section_idx_ >= (int)lane_section_.size() || lane_info.lane_section_idx_ < 0)
	{
		LOG("Error idx %d > n_lane_sections %d\n", lane_info.lane_section_idx_, (int)lane_section_.size());
	}
	else
	{
		LaneSection *lane_section = lane_section_[lane_info.lane_section_idx_];
		double t = SIGN(lane_info.lane_id_) * lane_section->GetCenterOffset(s, lane_info.lane_id_);

		// check if we passed current section
		if (s > lane_section->GetS() + lane_section->GetLength() || s < lane_section->GetS())
		{
			if (s > lane_section->GetS() + lane_section->GetLength())
			{
				while (s > lane_section->GetS() + lane_section->GetLength() && lane_info.lane_section_idx_ + 1 < GetNumberOfLaneSections())
				{
					// Find out connecting lane, then move to next lane section
					lane_info.lane_id_ = lane_section->GetConnectingLaneId(lane_info.lane_id_, SUCCESSOR);
					lane_section = GetLaneSectionByIdx(++lane_info.lane_section_idx_);
				}
			}
			else if (s < lane_section->GetS())
			{
				while (s < lane_section->GetS() && lane_info.lane_section_idx_ > 0)
				{
					// Move to previous lane section
					lane_info.lane_id_ = lane_section->GetConnectingLaneId(lane_info.lane_id_, PREDECESSOR);
					lane_section = GetLaneSectionByIdx(--lane_info.lane_section_idx_);
				}
			}

			// If new lane is not of snapping type, try to move into a close valid lane
			Lane* lane = lane_section->GetLaneById(lane_info.lane_id_);
			if (lane == 0 || !(laneTypeMask & lane_section->GetLaneById(lane_info.lane_id_)->GetLaneType()))
			{
				double offset = 0;

				if (lane == 0)
				{
					LOG("No valid connecting lane (s: %.2f lane_id %d) - looking for closest valid lane", s, lane_info.lane_id_);
				}

				lane_info.lane_id_ = lane_section->GetLaneByIdx(lane_section->GetClosestLaneIdx(s, t, offset, true, laneTypeMask))->GetId();
				if (lane_info.lane_id_ == 0 || SIGN(lane_info.lane_id_) != SIGN(start_lane_id))
				{
					LOG("Failed to find a closest snapping lane");
					return -1;
				}
			}
		}
	}

	return 0;
}

int Road::GetConnectingLaneId(RoadLink* road_link, int fromLaneId, int connectingRoadId)
{
	Lane* lane;

	if (road_link->GetElementId() == -1)
	{
		LOG("No connecting road or junction at rid %d link_type %s", GetId(), OpenDrive::LinkType2Str(road_link->GetType()).c_str());
		return 0;
	}

	if (road_link->GetType() == LinkType::SUCCESSOR)
	{
		lane = lane_section_.back()->GetLaneById(fromLaneId);
	}
	else
	{
		lane = lane_section_[0]->GetLaneById(fromLaneId);
	}

	if (lane == nullptr)
	{
		LOG("Failed to get connecting lane %d %d %d", GetId(), fromLaneId, connectingRoadId);
		return 0;
	}

	if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
	{
		if (road_link->GetElementId() != connectingRoadId)
		{
			LOG("Wrong connectingRoadId %d (expected %d)", road_link->GetElementId(), connectingRoadId);
			return 0;
		}

		LaneLink* lane_link = lane->GetLink(road_link->GetType());
		if (lane_link != 0)
		{
			return lane->GetLink(road_link->GetType())->GetId();
		}
		else
		{
			return 0;
		}
	}
	else if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
	{
		Junction* junction = Position::GetOpenDrive()->GetJunctionById(road_link->GetElementId());

		if (junction == 0)
		{
			LOG("Error: junction %d not existing\n", road_link->GetElementId());
			return 0;
		}

		int n_connections = junction->GetNumberOfRoadConnections(GetId(), lane->GetId());

		for (int i = 0; i < n_connections; i++)
		{
			LaneRoadLaneConnection lane_road_lane_connection =
				junction->GetRoadConnectionByIdx(GetId(), lane->GetId(), i);

			if (lane_road_lane_connection.GetConnectingRoadId() == connectingRoadId)
			{
				return lane_road_lane_connection.GetConnectinglaneId();
			}
		}
	}

	return 0;
}

double Road::GetLaneWidthByS(double s, int lane_id)
{
	LaneSection* lsec = GetLaneSectionByS(s, 0);

	if (lsec == nullptr)
	{
		return 0.0;
	}

	return lsec->GetWidth(s, lane_id);
}

Lane::LaneType Road::GetLaneTypeByS(double s, int lane_id)
{
	LaneSection* lsec = GetLaneSectionByS(s, 0);

	if (lsec == nullptr)
	{
		return Lane::LaneType::LANE_TYPE_ANY;
	}

	Lane* lane = lsec->GetLaneById(lane_id);
	if (lane == nullptr)
	{
		return Lane::LaneType::LANE_TYPE_ANY;
	}

	return lane->GetLaneType();
}

double Road::GetSpeedByS(double s)
{
	if (type_.size() > 0)
	{
		size_t i;
		for (i = 0; i < type_.size() - 1 && s > type_[i + 1]->s_; i++);

		return type_[i]->speed_;
	}

	// No type entries, fall back to a speed based on nr of lanes
	return 0;
}

Geometry* Road::GetGeometry(int idx)
{
	if (idx < 0 || idx + 1 > (int)geometry_.size())
	{
		LOG("Road::GetGeometry index %d out of range [0:%d]\n", idx, (int)geometry_.size());
		return 0;
	}
	return geometry_[idx];
}

void LaneSection::Print()
{
	LOG("LaneSection: %.2f, %d lanes:\n", s_, (int)lane_.size());

	for (size_t i=0; i<lane_.size(); i++)
	{
		lane_[i]->Print();
	}
}

Lane* LaneSection::GetLaneByIdx(int idx)
{
	if (idx < (int)lane_.size())
	{
		return lane_[idx];
	}

	return 0;
}

bool LaneSection::IsOSILaneById(int id)
{
	Lane* lane = GetLaneById(id);
	if (lane == 0)
	{
		return false;
	}
	else
	{
		return !lane->IsCenter();
	}


}

Lane* LaneSection::GetLaneById(int id)
{
	for (size_t i=0; i<lane_.size(); i++)
	{
		if (lane_[i]->GetId() == id)
		{
			return lane_[i];
		}
	}
	return 0;
}

int LaneSection::GetLaneIdByIdx(int idx)
{
	if (idx > (int)lane_.size() - 1)
	{
		LOG("LaneSection::GetLaneIdByIdx Error: index %d, only %d lanes\n", idx, (int)lane_.size());
		return 0;
	}
	else
	{
		return (lane_[idx]->GetId());
	}
}

int LaneSection::GetLaneIdxById(int id)
{
	for (int i = 0; i<(int)lane_.size(); i++)
	{
		if (lane_[i]->GetId() == id)
		{
			return i;
		}
	}
	return -1;
}

int LaneSection::GetLaneGlobalIdByIdx(int idx)
{
	if (idx < 0 || idx > (int)lane_.size() - 1)
	{
		LOG("LaneSection::GetLaneIdByIdx Error: index %d, only %d lanes\n", idx, (int)lane_.size());
		return 0;
	}
	else
	{
		return (lane_[idx]->GetGlobalId());
	}
}
int LaneSection::GetLaneGlobalIdById(int id)
{
	for (size_t i=0; i<(int)lane_.size(); i++)
	{
		if (lane_[i]->GetId() == id)
		{
			return lane_[i]->GetGlobalId();
		}
	}
	return -1;
}

int LaneSection::GetNumberOfDrivingLanes()
{
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++)
	{
		if (lane_[i]->IsDriving())
		{
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNumberOfDrivingLanesSide(int side)
{
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++)
	{
		if (SIGN(lane_[i]->GetId()) == SIGN(side) && lane_[i]->IsDriving())
		{
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNUmberOfLanesRight()
{
	int counter = 0;

	for (size_t i=0; i<lane_.size(); i++)
	{
		if (lane_[i]->GetId() < 0)
		{
			counter++;
		}
	}
	return counter;
}

int LaneSection::GetNUmberOfLanesLeft()
{
	int counter = 0;

	for (size_t i = 0; i < lane_.size(); i++)
	{
		if (lane_[i]->GetId() > 0)
		{
			counter++;
		}
	}
	return counter;
}

double LaneSection::GetWidth(double s, int lane_id)
{
	if (lane_id == 0)
	{
		return 0.0;  // reference lane has no width
	}

	// Enforce s within range of section
	s = CLAMP(s, s_, s_ + GetLength());

	Lane *lane = GetLaneById(lane_id);
	if (lane == 0)
	{
		return 0.0;
	}

	LaneWidth *lane_width = lane->GetWidthByS(s - s_);
	if (lane_width == 0) // No lane width registered
	{
		return 0.0;
	}

	// Calculate local s-parameter in width segment
	double ds = s - (s_ + lane_width->GetSOffset());

	// Calculate width at local s
	return lane_width->poly3_.Evaluate(ds);
}

double LaneSection::GetOuterOffset(double s, int lane_id)
{
	if (lane_id == 0)
	{
		return 0;
	}

	double width = GetWidth(s, lane_id);

	if (abs(lane_id) == 1)
	{
		// this is the last lane, next to reference lane of width = 0. Stop here.
		return width;
	}
	else
	{
		int step = lane_id < 0 ? +1 : -1;
		return (width + GetOuterOffset(s, lane_id + step));
	}
}

double LaneSection::GetCenterOffset(double s, int lane_id)
{
	if (lane_id == 0)
	{
		// Reference lane (0) has no width
		return 0.0;
	}
	double outer_offset = GetOuterOffset(s, lane_id);
	double width = GetWidth(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return outer_offset - width / 2;
}

double LaneSection::GetOuterOffsetHeading(double s, int lane_id)
{
	if (lane_id == 0)
	{
		return 0.0;  // reference lane has no width
	}

	Lane *lane = GetLaneById(lane_id);
	if (lane == 0)
	{
		return 0.0;
	}

	LaneWidth *lane_width = lane->GetWidthByS(s - s_);
	if (lane_width == 0) // No lane width registered
	{
		return 0.0;
	}

	// Calculate local s-parameter in width segment
	double ds = s - (s_ + lane_width->GetSOffset());

	// Calculate heading at local s
	double heading = lane_width->poly3_.EvaluatePrim(ds);

	if (abs(lane_id) == 1)
	{
		// this is the last lane, next to reference lane of width = 0. Stop here.
		return heading;
	}
	else
	{
		int step = lane_id < 0 ? +1 : -1;
		return (heading + GetOuterOffsetHeading(s, lane_id + step));
	}
}

double LaneSection::GetCenterOffsetHeading(double s, int lane_id)
{
	int step = lane_id < 0 ? +1 : -1;

	if (lane_id == 0)
	{
		// Reference lane (0) has no width
		return 0.0;
	}
	double inner_offset_heading = GetOuterOffsetHeading(s, lane_id + step);
	double outer_offset_heading = GetOuterOffsetHeading(s, lane_id);

	// Center is simply mean value of inner and outer lane boundries
	return (inner_offset_heading + outer_offset_heading) / 2;
}

void LaneSection::AddLane(Lane *lane)
{
	lane->SetGlobalId();

	// Keep list sorted on lane ID, from + to -
	if (lane_.size() > 0 && lane->GetId() > lane_.back()->GetId())
	{
		for (size_t i = 0; i < lane_.size(); i++)
		{
			if (lane->GetId() > lane_[i]->GetId())
			{
				lane_.insert(lane_.begin() + i, lane);
				break;
			}
		}
	}
	else
	{
		lane_.push_back(lane);
	}
}

int LaneSection::GetConnectingLaneId(int incoming_lane_id, LinkType link_type)
{
	int id = incoming_lane_id;

	if (GetLaneById(id) == 0)
	{
		LOG("Lane id %d not available in lane section!", id);
		return 0;
	}

	if (GetLaneById(id)->GetLink(link_type))
	{
		id = GetLaneById(id)->GetLink(link_type)->GetId();
	}
	else
	{
		// if no driving lane found - stay on same index
		id = incoming_lane_id;
	}

	return id;
}

double LaneSection::GetWidthBetweenLanes(int lane_id1, int lane_id2, double s)
{
	double lanewidth = (std::fabs(GetCenterOffset(s, lane_id1)) - std::fabs(GetCenterOffset(s, lane_id2)));

	return lanewidth;
}

// Offset from lane1 to lane2 in direction of reference line
double LaneSection::GetOffsetBetweenLanes(int lane_id1, int lane_id2, double s)
{
	double laneCenter1 = GetCenterOffset(s, lane_id1) * SIGN(lane_id1);
	double laneCenter2 = GetCenterOffset(s, lane_id2) * SIGN(lane_id2);
	return (laneCenter2 - laneCenter1);
}

// Offset from closest left road mark to current position
RoadMarkInfo Lane::GetRoadMarkInfoByS(int track_id, int lane_id, double s)
{
	Position* pos = new roadmanager::Position();
	Road *road = pos->GetRoadById(track_id);
	LaneSection *lsec;
	Lane *lane;
	LaneRoadMark *lane_roadMark;
	LaneRoadMarkType *lane_roadMarkType;
	LaneRoadMarkTypeLine *lane_roadMarkTypeLine;
	RoadMarkInfo rm_info = {-1, -1};
	int lsec_idx, number_of_lsec, number_of_roadmarks, number_of_roadmarktypes, number_of_roadmarklines;
	double s_roadmark, s_roadmarkline, s_end_roadmark, s_end_roadmarkline = 0, lsec_end = 0;
	if (road == 0)
	{
		LOG("Position::Set Error: track %d not available", track_id);
		lsec_idx = -1;
	}
	else
	{
		lsec_idx = road->GetLaneSectionIdxByS(s);
	}

	lsec = road->GetLaneSectionByIdx(lsec_idx);

	if (lsec == 0)
	{
		LOG("Position::Set Error: lane section %d not available", lsec_idx);
	}
	else
	{
		number_of_lsec = road->GetNumberOfLaneSections();
		if (lsec_idx == number_of_lsec-1)
		{
			lsec_end = road->GetLength();
		}
		else
		{
			lsec_end = road->GetLaneSectionByIdx(lsec_idx+1)->GetS();
		}
	}

	lane = lsec->GetLaneById(lane_id);
	if (lane == 0)
	{
		LOG("Position::Set Error: lane section %d not available", lane_id);
	}

	number_of_roadmarks = lane->GetNumberOfRoadMarks();

	if (number_of_roadmarks > 0)
	{
		for (int m=0; m<number_of_roadmarks; m++)
		{
			lane_roadMark = lane->GetLaneRoadMarkByIdx(m);
			s_roadmark = lsec->GetS() + lane_roadMark->GetSOffset();
			if (m == number_of_roadmarks-1)
			{
				s_end_roadmark = lsec_end;
			}
			else
			{
				s_end_roadmark = lane->GetLaneRoadMarkByIdx(m+1)->GetSOffset();
			}

			// Check the existence of "type" keyword under roadmark
			number_of_roadmarktypes = lane_roadMark->GetNumberOfRoadMarkTypes();
			if (number_of_roadmarktypes != 0)
			{
				lane_roadMarkType = lane_roadMark->GetLaneRoadMarkTypeByIdx(0);
				number_of_roadmarklines = lane_roadMarkType->GetNumberOfRoadMarkTypeLines();

				// Looping through each roadmarkline under roadmark
				for (int n=0; n<number_of_roadmarklines; n++)
				{
					lane_roadMarkTypeLine = lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(n);
					s_roadmarkline = s_roadmark + lane_roadMarkTypeLine->GetSOffset();
					if (lane_roadMarkTypeLine != 0)
					{
						if (n == number_of_roadmarklines-1)
						{
							s_end_roadmarkline = s_end_roadmark;
						}
						else
						{
							s_end_roadmarkline = lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(n+1)->GetSOffset();
						}
					}

					if (s >= s_roadmarkline && s < s_end_roadmarkline)
					{
						rm_info.roadmark_idx_ = m;
						rm_info.roadmarkline_idx_ = n;
					}
					else
					{
						continue;
					}
				}
			}
			else
			{
				rm_info.roadmarkline_idx_ = 0;
				if (s >= s_roadmark && s < s_end_roadmark)
				{
					rm_info.roadmark_idx_ = m;
				}
				else
				{
					continue;
				}
			}
		}
	}
	delete pos;
	return rm_info;
}

RoadLink::RoadLink(LinkType type, pugi::xml_node node) : contact_point_type_(ContactPointType::CONTACT_POINT_UNDEFINED)
{
	string element_type = node.attribute("elementType").value();
	string contact_point_type = "";
	type_ = type;
	element_id_ = atoi(node.attribute("elementId").value());

	if (node.attribute("contactPoint") != NULL)
	{
		contact_point_type = node.attribute("contactPoint").value();
	}

	if (element_type == "road")
	{
		element_type_ = ELEMENT_TYPE_ROAD;
		if (contact_point_type == "start")
		{
			contact_point_type_ = CONTACT_POINT_START;
		}
		else if (contact_point_type == "end")
		{
			contact_point_type_ = CONTACT_POINT_END;
		}
		else if (contact_point_type.empty())
		{
			LOG("Missing contact point type\n");
		}
		else
		{
			LOG("Unsupported contact point type: %s\n", contact_point_type.c_str());
			contact_point_type_ = CONTACT_POINT_UNDEFINED;
		}
	}
	else if (element_type == "junction")
	{
		element_type_ = ELEMENT_TYPE_JUNCTION;
		contact_point_type_ = CONTACT_POINT_JUNCTION;
	}
	else if (element_type.empty())
	{
		LOG("Missing element type\n");
	}
	else
	{
		LOG("Unsupported element type: %s\n", element_type.c_str());
		element_type_ = ELEMENT_TYPE_UNKNOWN;
	}
}

bool RoadLink::operator==(RoadLink& rhs)
{
	return
		(
			rhs.type_ == type_ &&
			rhs.element_type_ == element_type_ &&
			rhs.element_id_ == element_id_ &&
			rhs.contact_point_type_ == contact_point_type_
		);
}

void RoadLink::Print()
{
	cout << "RoadLink type: " << type_ << " id: " << element_id_ << " element type: " << element_type_ << " contact point type: " << contact_point_type_ << endl;
}

Road::~Road()
{
	for (size_t i = 0; i < geometry_.size(); i++)
	{
		delete(geometry_[i]);
	}
	geometry_.clear();
	for (size_t i = 0; i < elevation_profile_.size(); i++)
	{
		delete(elevation_profile_[i]);
	}
	elevation_profile_.clear();
	for (size_t i = 0; i < super_elevation_profile_.size(); i++)
	{
		delete(super_elevation_profile_[i]);
	}
	super_elevation_profile_.clear();

	for (size_t i = 0; i < link_.size(); i++)
	{
		delete(link_[i]);
	}
	link_.clear();
	for (size_t i = 0; i < lane_section_.size(); i++)
	{
		delete(lane_section_[i]);
	}
	lane_section_.clear();
	for (size_t i = 0; i < lane_offset_.size(); i++)
	{
		delete(lane_offset_[i]);
	}
	lane_offset_.clear();
	for (size_t i = 0; i < signal_.size(); i++)
	{
		delete(signal_[i]);
	}
	signal_.clear();
	for (size_t i = 0; i < object_.size(); i++)
	{
		delete(object_[i]);
	}
	object_.clear();
}

void Road::Print()
{
	LOG("Road id: %d length: %.2f\n", id_, GetLength());
	cout << "Geometries:" << endl;

	for (size_t i = 0; i < geometry_.size(); i++)
	{
		cout << "Geometry type: " << geometry_[i]->GetType() << endl;
	}

	for (size_t i=0; i<link_.size(); i++)
	{
		link_[i]->Print();
	}

	for (size_t i=0; i<lane_section_.size(); i++)
	{
		lane_section_[i]->Print();
	}

	for (size_t i=0; i<lane_offset_.size(); i++)
	{
		lane_offset_[i]->Print();
	}
}

void Road::AddLine(Line *line)
{
	geometry_.push_back((Geometry*)line);
}

void Road::AddArc(Arc *arc)
{
	geometry_.push_back((Geometry*)arc);
}

void Road::AddSpiral(Spiral *spiral)
{
	geometry_.push_back((Geometry*)spiral);
}

void Road::AddPoly3(Poly3 *poly3)
{
	geometry_.push_back((Geometry*)poly3);
}

void Road::AddParamPoly3(ParamPoly3 *param_poly3)
{
	geometry_.push_back((Geometry*)param_poly3);
}

void Road::AddElevation(Elevation *elevation)
{
	// Adjust last elevation length
	if (elevation_profile_.size() > 0)
	{
		Elevation *e_previous = elevation_profile_.back();
		e_previous->SetLength(elevation->GetS() - e_previous->GetS());
	}
	elevation->SetLength(length_ - elevation->GetS());

	elevation_profile_.push_back((Elevation*)elevation);
}

void Road::AddSuperElevation(Elevation *super_elevation)
{
	// Adjust last super elevation length
	if (super_elevation_profile_.size() > 0)
	{
		Elevation *e_previous = super_elevation_profile_.back();
		e_previous->SetLength(super_elevation->GetS() - e_previous->GetS());
	}
	super_elevation->SetLength(length_ - super_elevation->GetS());

	super_elevation_profile_.push_back((Elevation*)super_elevation);
}

Elevation* Road::GetElevation(int idx)
{
	if (idx < 0 || idx >= elevation_profile_.size())
	{
		return 0;
	}

	return elevation_profile_[idx];
}

Elevation* Road::GetSuperElevation(int idx)
{
	if (idx < 0 || idx >= super_elevation_profile_.size())
	{
		return 0;
	}

	return super_elevation_profile_[idx];
}

void Road::AddSignal(Signal *signal)
{
	// Adjust signal length
	if (signal_.size() > 0)
	{
		Signal *sig_previous = signal_.back();
		sig_previous->SetLength(signal->GetS() - sig_previous->GetS());
	}
	signal->SetLength(length_ - signal->GetS());

	//LOG("Add signal[%d]: \"%s\" type %d subtype %d to road %d", (int)signal_.size(), signal->GetName().c_str(),
	//	signal->GetType(), signal->GetSubType(), GetId());
	signal_.push_back((Signal*)signal);
}

int Road::GetNumberOfSignals()
{
	return (int)signal_.size();
}

Signal* Road::GetSignal(int idx)
{
	if (idx < 0 || idx >= signal_.size())
	{
		return 0;
	}

	return signal_[idx];
}

void Road::AddObject(RMObject* object)
{
	/*LOG("Add object[%d]: %s", (int)object_.size(), object->GetName().c_str());*/
	object_.push_back(object);
}

RMObject* Road::GetRoadObject(int idx)
{
	if (idx < 0 || idx >= object_.size())
	{
		return 0;
	}

	return object_[idx];
}

OutlineCornerRoad::OutlineCornerRoad(int roadId, double s, double t, double dz, double height, double center_s, double center_t, double center_heading):
	roadId_(roadId), s_(s), t_(t), dz_(dz), height_(height), center_s_(center_s), center_t_(center_t), center_heading_(center_heading)
{

}

void OutlineCornerRoad::GetPos(double& x, double& y, double& z)
{
	roadmanager::Position pos;
	pos.SetTrackPos(roadId_, s_, t_);
	x = pos.GetX();
	y = pos.GetY();
	z = pos.GetZ() + dz_;
}

void OutlineCornerRoad::GetPosLocal(double& x, double& y, double& z)
{
	roadmanager::Position pref;
	pref.SetTrackPos(roadId_, center_s_, center_t_);
	roadmanager::Position point;
	point.SetTrackPos(roadId_, s_, t_);
	double total_heading = GetAngleSum(pref.GetH(), center_heading_);

	Global2LocalCoordinates( point.GetX(), point.GetY(),
							  pref.GetX(), pref.GetY(), total_heading,
							 x, y);

	z = pref.GetZ() + dz_;
}

OutlineCornerLocal::OutlineCornerLocal(int roadId, double s, double t, double u, double v, double zLocal, double height, double heading) :
	roadId_(roadId), s_(s), t_(t), u_(u), v_(v), zLocal_(zLocal), height_(height), heading_(heading)
{

}

void OutlineCornerLocal::GetPos(double& x, double& y, double& z)
{
	roadmanager::Position pref;
	pref.SetTrackPos(roadId_, s_, t_);
	double total_heading = GetAngleSum(pref.GetH(), heading_);
	double u2, v2;
	RotateVec2D(u_, v_, total_heading, u2, v2);

	x = pref.GetX() + u2;
	y = pref.GetY() + v2;
	z = pref.GetZ() + zLocal_;
}

void OutlineCornerLocal::GetPosLocal(double& x, double& y, double& z)
{
	x = u_;
	y = v_;
	z = zLocal_;
}

std::string RMObject::Type2Str(RMObject::ObjectType type)
{
	int t = static_cast<int>(type);
	if (t < sizeof(object_type_str) / sizeof(char*))
	{
		return object_type_str[t];
	}
	else
	{
		LOG("Unsupported object type: %d", t);
	}

	return "";
}

RMObject::ObjectType RMObject::Str2Type(std::string type)
{
	int n_types = static_cast<int>(sizeof(object_type_str) / sizeof(char*));

	for (int i = 0; i < n_types;i++)
	{
		if (type == object_type_str[i])
		{
			return static_cast<ObjectType>(i);
		}
	}

	LOG("Unsupported object type: %s - interpret as NONE", type.c_str());

	return RMObject::ObjectType::NONE;
}

double Road::GetLaneOffset(double s)
{
	int i = 0;

	if (lane_offset_.size() == 0)
	{
		return 0;
	}

	for (; i + 1 < (int)lane_offset_.size(); i++)
	{
		if (s < lane_offset_[i + 1]->GetS())
		{
			break;
		}
	}
	return (lane_offset_[i]->GetLaneOffset(s));
}

double Road::GetLaneOffsetPrim(double s)
{
	int i = 0;

	if (lane_offset_.size() == 0)
	{
		return 0;
	}

	for (; i + 1 < (int)lane_offset_.size(); i++)
	{
		if (s < lane_offset_[i + 1]->GetS())
		{
			break;
		}
	}
	return (lane_offset_[i]->GetLaneOffsetPrim(s));
}

int Road::GetNumberOfLanes(double s)
{
	LaneSection *lsec = GetLaneSectionByS(s);

	if (lsec)
	{
		return (lsec->GetNumberOfLanes());
	}

	return 0;
}

int Road::GetNumberOfDrivingLanes(double s)
{
	LaneSection *lsec = GetLaneSectionByS(s);

	if (lsec)
	{
		return (lsec->GetNumberOfDrivingLanes());
	}

	return 0;
}

Lane* Road::GetDrivingLaneByIdx(double s, int idx)
{
	int count = 0;

	LaneSection *ls = GetLaneSectionByS(s);

	for (int i = 0; i < ls->GetNumberOfLanes(); i++)
	{
		if (ls->GetLaneByIdx(i)->IsDriving())
		{
			if (count++ == idx)
			{
				return ls->GetLaneByIdx(i);
			}
		}
	}

	return 0;
}

Lane* Road::GetDrivingLaneSideByIdx(double s, int side, int idx)
{
	int count = 0;

	LaneSection* ls = GetLaneSectionByS(s);

	for (int i = 0; i < ls->GetNumberOfLanes(); i++)
	{
		Lane* lane = ls->GetLaneByIdx(i);
		if (lane->IsDriving() && SIGN(lane->GetId()) == side)
		{
			if (count++ == idx)
			{
				return lane;
			}
		}
	}

	return 0;
}

Lane* Road::GetDrivingLaneById(double s, int id)
{
	LaneSection *ls = GetLaneSectionByS(s);
	Lane* lane = ls->GetLaneById(id);
	if (lane && lane->IsDriving())
	{
		return lane;
	}

	return 0;
}


int Road::GetNumberOfDrivingLanesSide(double s, int side)
{
	int i;

	for (i = 0; i < GetNumberOfLaneSections() - 1; i++)
	{
		if (s < lane_section_[i+1]->GetS())
		{
			break;
		}
	}

	return (lane_section_[i]->GetNumberOfDrivingLanesSide(side));
}

int Road::GetConnectedLaneIdAtS(int lane_id, double s_start, double s_target)
{
	int connected_lane_id = 0;

	if (s_target < 0 - SMALL_NUMBER)
	{
		// -1 indicates end of road
		s_target = GetLength();
	}

	if (s_start < 0 - SMALL_NUMBER)
	{
		// -1 indicates end of road
		s_start = GetLength();
	}

	// Find lane ID at start/end of the road
	int lsec_idx = GetLaneSectionIdxByS(s_start);
	if (lsec_idx >= 0)
	{
		LaneSection* lsec = GetLaneSectionByIdx(lsec_idx);
		if (lsec != nullptr)
		{
			int lane_id_tmp = lane_id;
			double s = s_start;
			if (s_target > s_start - SMALL_NUMBER)
			{
				for (int j = lsec_idx; lane_id_tmp != 0 && j < GetNumberOfLaneSections() - 1 && s < s_target; j++)
				{
					// find connecting lane id
					lsec = GetLaneSectionByIdx(j);
					lane_id_tmp = lsec->GetConnectingLaneId(lane_id_tmp, LinkType::SUCCESSOR);
					s = lsec->GetS();
				}
			}
			else
			{
				for (int j = lsec_idx; lane_id_tmp != 0 && j > 0 && s > s_target; j--)
				{
					// find connecting lane id
					lsec = GetLaneSectionByIdx(j);
					lane_id_tmp = lsec->GetConnectingLaneId(lane_id_tmp, LinkType::PREDECESSOR);
					s = lsec->GetS();
				}
			}
			connected_lane_id = lane_id_tmp;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}

	return connected_lane_id;
}

bool Road::IsDirectlyConnected(Road* road, LinkType link_type, ContactPointType* contact_point, int fromLaneId)
{
	if (road == nullptr)
	{
		return false;
	}

	RoadLink* link = GetLink(link_type);
	if (link == nullptr)
	{
		return false;  // lacking successor
	}

	if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		if (link->GetElementId() == road->GetId())
		{
			if (contact_point != nullptr)
			{
				*contact_point = link->GetContactPointType();
			}
			return true;
		}
	}
	else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		// Check all connections
		Junction* junction = Position::GetOpenDrive()->GetJunctionById(link->GetElementId());
		for (int i = 0; junction != nullptr && i < (int)junction->GetNumberOfConnections(); i++)
		{
			Connection* connection = junction->GetConnectionByIdx(i);
			if (connection->GetIncomingRoad() == this &&
				connection->GetConnectingRoad() == road)
			{
				if (contact_point != nullptr)
				{
					*contact_point = connection->GetContactPoint();
				}
				// need to calculate contact point for non direct junctions?

				if (fromLaneId != 0)
				{
					for (int j = 0; j < (int)connection->GetNumberOfLaneLinks(); j++)
					{
						if (connection->GetLaneLink(j)->from_ == fromLaneId)
						{
							return true;
						}
					}
				}
				else
				{
					return true;
				}
			}
		}
	}

	return false;
}

bool Road::IsSuccessor(Road* road, ContactPointType* contact_point, int fromLaneId)
{
	return IsDirectlyConnected(road, LinkType::SUCCESSOR, contact_point, fromLaneId) != 0;
}

bool Road::IsPredecessor(Road* road, ContactPointType* contact_point, int fromLaneId)
{
	return IsDirectlyConnected(road, LinkType::PREDECESSOR, contact_point, fromLaneId) != 0;
}

bool Road::IsDirectlyConnected(Road* road, double* curvature, int fromLaneId)
{
	ContactPointType contact_point;

	// Unspecified link, check both ends
	if (IsSuccessor(road, &contact_point, fromLaneId) || IsPredecessor(road, &contact_point, fromLaneId))
	{
		// Find out curvature
		if (contact_point == ContactPointType::CONTACT_POINT_START && road->geometry_.size() > 0)
		{
			if (curvature)
			{
				*curvature = road->geometry_[0]->EvaluateCurvatureDS(0);
			}
		}
		else if (contact_point == ContactPointType::CONTACT_POINT_END && road->geometry_.size() > 0)
		{
			Geometry* geom = road->geometry_.back();
			if (curvature)
			{
				*curvature = geom->EvaluateCurvatureDS(geom->GetLength());
			}
		}
		else
		{
			LOG("Unexpected contact type %d between roads %d and %d", contact_point, GetId(), road->GetId());
		}
		return true;
	}

	return false;
}

double Road::GetWidth(double s, int side, int laneTypeMask)
{
	double offset0 = 0;
	double offset1 = 0;
	size_t i = 0;
	int index = 0;

	for (; i < GetNumberOfLaneSections() - 1; i++)
	{
		if (s < lane_section_[i+1]->GetS())
		{
			break;
		}
	}

	if (i < GetNumberOfLaneSections())
	{
		LaneSection* lsec = lane_section_[i];
		// Since the lanes are sorted from left to right,
		// side == +1 means first lane and side == -1 means last lane

		if (side > 0)
		{
			index = 0;
		}
		else
		{
			index = lsec->GetNumberOfLanes() - 1;
		}

		int lane_id = lsec->GetLaneIdByIdx(index);
		int step = side > 0 ? +1 : -1;

		// Find outmost lane matching requested lane type
		while (lane_id != 0 && !(lsec->GetLaneByIdx(index)->GetLaneType() & laneTypeMask))
		{
			lane_id = lsec->GetLaneIdByIdx(index += step);
		}
		offset0 = fabs(lsec->GetOuterOffset(s, lane_id));

		if (side == 0)
		{
			// offset0 holds rightmost offset, now find outmost lane on left side of centerlane
			index = 0;
			lane_id = lsec->GetLaneIdByIdx(index);

			while (lane_id != 0 && !(lsec->GetLaneByIdx(index)->GetLaneType() & laneTypeMask))
			{
				lane_id = lsec->GetLaneIdByIdx(index += 1);
			}
			offset1 = fabs(lsec->GetOuterOffset(s, lane_id));
		}
	}

	return offset0 + offset1;
}

void Road::AddLaneOffset(LaneOffset *lane_offset)
{
	// Adjust lane offset length
	if (lane_offset_.size() > 0)
	{
		LaneOffset *lo_previous = lane_offset_.back();
		lo_previous->SetLength(lane_offset->GetS() - lo_previous->GetS());
	}
	lane_offset->SetLength(length_ - lane_offset->GetS());

	lane_offset_.push_back((LaneOffset*)lane_offset);
}

double Road::GetCenterOffset(double s, int lane_id)
{
	// First find out what lane section
	LaneSection *lane_section = GetLaneSectionByS(s);
	if (lane_section)
	{
		return lane_section->GetCenterOffset(s, lane_id);
	}

	return 0.0;
}

Road::RoadTypeEntry* Road::GetRoadType(int idx)
{
	if (type_.size() > 0)
	{
		return type_[idx];
	}
	else
	{
		return 0;
	}
}

RoadLink* Road::GetLink(LinkType type)
{
	for (size_t i=0; i<link_.size(); i++)
	{
		if (link_[i]->GetType() == type)
		{
			return link_[i];
		}
	}
	return 0;  // Link of requested type is missing
}

void Road::AddLaneSection(LaneSection *lane_section)
{
	// Adjust last elevation section length
	if (lane_section_.size() > 0)
	{
		LaneSection *ls_previous = lane_section_.back();
		ls_previous->SetLength(lane_section->GetS() - ls_previous->GetS());
	}
	lane_section->SetLength(length_ - lane_section->GetS());

	lane_section_.push_back((LaneSection*)lane_section);
}

bool Road::GetZAndPitchByS(double s, double *z, double* z_prim, double *z_primPrim, double *pitch, int *index)
{
	if (GetNumberOfElevations() > 0)
	{
		if (*index < 0 || *index >= GetNumberOfElevations())
		{
			*index = 0;
		}
		Elevation *elevation = GetElevation(*index);
		if (elevation == NULL)
		{
			LOG("Elevation error NULL, nelev: %d elev_idx: %d\n", GetNumberOfElevations(), *index);
			return false;
		}

		if (elevation && s > elevation->GetS() + elevation->GetLength() - SMALL_NUMBER)
		{
			while (s > elevation->GetS() + elevation->GetLength() - SMALL_NUMBER && *index < GetNumberOfElevations() - 1)
			{
				// Move to next elevation section
				elevation = GetElevation(++*index);
			}
		}
		else if (elevation && s < elevation->GetS())
		{
			while (s < elevation->GetS() && *index > 0)
			{
				// Move to previous elevation section
				elevation = GetElevation(--*index);
			}
		}

		if (elevation)
		{
			double p = s - elevation->GetS();
			*z = elevation->poly3_.Evaluate(p);
			*z_prim = elevation->poly3_.EvaluatePrim(p);
			*z_primPrim = elevation->poly3_.EvaluatePrimPrim(p);
			*pitch = -atan(elevation->poly3_.EvaluatePrim(p));
			return true;
		}
	}

	*z = 0.0;
	*pitch = 0.0;

	return false;
}

bool Road::UpdateZAndRollBySAndT(double s, double t, double *z, double* roadSuperElevationPrim, double *roll, int *index)
{
	if (GetNumberOfSuperElevations() > 0)
	{
		if (*index < 0 || *index >= GetNumberOfSuperElevations())
		{
			*index = 0;
		}
		Elevation *super_elevation = GetSuperElevation(*index);
		if (super_elevation == NULL)
		{
			LOG("Superelevation error NULL, nelev: %d elev_idx: %d\n", GetNumberOfSuperElevations(), *index);
			return false;
		}

		if (super_elevation && s > super_elevation->GetS() + super_elevation->GetLength())
		{
			while (s > super_elevation->GetS() + super_elevation->GetLength() && *index < GetNumberOfSuperElevations() - 1)
			{
				// Move to next elevation section
				super_elevation = GetSuperElevation(++ *index);
			}
		}
		else if (super_elevation && s < super_elevation->GetS())
		{
			while (s < super_elevation->GetS() && *index > 0)
			{
				// Move to previous elevation section
				super_elevation = GetSuperElevation(-- *index);
			}
		}

		if (super_elevation)
		{
			double ds = s - super_elevation->GetS();
			*roll = super_elevation->poly3_.Evaluate(ds);
			*z += sin(*roll) * (t + GetLaneOffset(s));
			*roadSuperElevationPrim = super_elevation->poly3_.EvaluatePrim(ds);
			return true;
		}
	}
	return false;
}

Road* OpenDrive::GetRoadById(int id)
{
	for (size_t i=0; i<road_.size(); i++)
	{
		if (road_[i]->GetId() == id)
		{
			return road_[i];
		}
	}
	return 0;
}

Road *OpenDrive::GetRoadByIdx(int idx)
{
	if (idx >= 0 && idx < (int)road_.size())
	{
		return road_[idx];
	}
	else
	{
		return 0;
	}
}

Geometry *OpenDrive::GetGeometryByIdx(int road_idx, int geom_idx)
{
	if (road_idx >= 0 && road_idx < (int)road_.size())
	{
		return road_[road_idx]->GetGeometry(geom_idx);
	}
	else
	{
		return 0;
	}
}

Junction* OpenDrive::GetJunctionById(int id)
{
	for (size_t i=0; i<junction_.size(); i++)
	{
		if (junction_[i]->GetId() == id)
		{
			return junction_[i];
		}
	}
	return 0;
}

Junction *OpenDrive::GetJunctionByIdx(int idx)
{
	if (idx >= 0 && idx < (int)junction_.size())
	{
		return junction_[idx];
	}
	else
	{
		LOG("GetJunctionByIdx error (idx %d, njunctions %d)\n", idx, (int)junction_.size());
		return 0;
	}
}

OpenDrive::OpenDrive(const char *filename) : speed_unit_(SpeedUnit::UNDEFINED)
{
	if (!LoadOpenDriveFile(filename))
	{
		throw std::invalid_argument(std::string("Failed to load OpenDrive file: ") + filename);
	}
}

void OpenDrive::InitGlobalLaneIds()
{
	g_Lane_id = 0;
	g_Laneb_id = 0;
}

Controller* OpenDrive::GetControllerByIdx(int index)
{
	if (index >= 0 && index < controller_.size())
	{
		return &controller_[index];
	}

	return 0;
}

Controller* OpenDrive::GetControllerById(int id)
{
	// look for this controller in global list
	for (int i = 0; i < GetNumberOfControllers(); i++)
	{
		if (id == GetControllerByIdx(i)->GetId())
		{
			return GetControllerByIdx(i);
		}
	}

	return nullptr;
}

std::string ReadAttribute(pugi::xml_node node, std::string attribute_name, bool required)
{
	if (!strcmp(attribute_name.c_str(), ""))
	{
		if (required)
		{
			LOG("Warning: Required but empty attribute");
		}
		return "";
	}

	pugi::xml_attribute attr;

	if ((attr = node.attribute(attribute_name.c_str())))
	{
		return attr.value();
	}
	else
	{
		if (required)
		{
			LOG("Warning: missing required attribute: %s -> %s", node.name(), attribute_name.c_str());
		}
	}

	return "";
}

void OpenDrive::Clear()
{
	InitGlobalLaneIds();

	for (size_t i = 0; i < road_.size(); i++)
	{
		delete road_[i];
	}
	road_.clear();

	for (size_t i = 0; i < junction_.size(); i++)
	{
		delete junction_[i];
	}
	junction_.clear();

	SetSpeedUnit(SpeedUnit::UNDEFINED);
}

bool OpenDrive::LoadOpenDriveFile(const char *filename, bool replace)
{
	if (replace)
	{
		Clear();
	}

	odr_filename_ = filename;

	if (odr_filename_ == "")
	{
		return false;
	}

	pugi::xml_document doc;

	// First assume absolute path
	pugi::xml_parse_result result = doc.load_file(filename);
	if (!result)
	{
		LOG("%s at offset (character position): %d", result.description(), result.offset);
		return false;
	}

	pugi::xml_node node = doc.child("OpenDRIVE");
	if (node == NULL)
	{
		LOG("Invalid OpenDRIVE file, can't find OpenDRIVE element");
		return false;
	}

	//Initialize GeoRef structure
	geo_ref_ = {
		std::numeric_limits<double>::quiet_NaN(),
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
		std::numeric_limits<int>::quiet_NaN()
	};

	pugi::xml_node header_node = node.child("header");
	if (node != NULL)
	{
		versionMajor_ = strtoi(header_node.attribute("revMajor").value());
		versionMinor_ = strtoi(header_node.attribute("revMinor").value());

		if(header_node.child("geoReference") != NULL)
		{
			//Get the string to parse, geoReference tag is just a string with the data separated by spaces and each attribute start with a + character
			std::string geo_ref_str = header_node.child_value("geoReference");
			ParseGeoLocalization(geo_ref_str);
		}
	}

	for (pugi::xml_node road_node = node.child("road"); road_node; road_node = road_node.next_sibling("road"))
	{
		std::string rname = road_node.attribute("name").value();

		int rid = (int)road_.size();  // preliminary road id, use if not specified in OpenDRIVE file
		if (road_node.attribute("id").empty() || !strcmp(road_node.attribute("id").value(), ""))
		{
			LOG("Id for road \"%s\" missing, assigning id %d", rname.c_str(), rid);
		}
		else
		{
			rid = atoi(road_node.attribute("id").value());
		}

		double roadlength = atof(road_node.attribute("length").value());
		int junction_id = atoi(road_node.attribute("junction").value());
		Road::RoadRule rrule = Road::RoadRule::RIGHT_HAND_TRAFFIC;  // right hand traffic is default

		if (!road_node.attribute("rule").empty())
		{
			std::string rule_str = road_node.attribute("rule").value();
			if (rule_str == "LHT" || rule_str == "lht")
			{
				rrule = Road::RoadRule::LEFT_HAND_TRAFFIC;
			}
		}

		Road* r = new Road(rid, rname, rrule);
		r->SetLength(roadlength);
		r->SetJunction(junction_id);

		for (pugi::xml_node type_node = road_node.child("type"); type_node; type_node = type_node.next_sibling("type"))
		{
			Road::RoadTypeEntry *r_type = new Road::RoadTypeEntry();

			std::string type = type_node.attribute("type").value();
			if (type == "unknown")
			{
				r_type->road_type_ = Road::RoadType::ROADTYPE_UNKNOWN;
			}
			else if (type == "rural")
			{
				r_type->road_type_ = Road::RoadType::ROADTYPE_RURAL;
			}
			else if (type == "motorway")
			{
				r_type->road_type_ = Road::RoadType::ROADTYPE_MOTORWAY;
			}
			else if (type == "town")
			{
				r_type->road_type_ = Road::RoadType::ROADTYPE_TOWN;
			}
			else if (type == "lowSpeed")
			{
				r_type->road_type_ = Road::RoadType::ROADTYPE_LOWSPEED;
			}
			else if (type == "pedestrian")
			{
				r_type->road_type_ = Road::RoadType::ROADTYPE_PEDESTRIAN;
			}
			else if (type == "bicycle")
			{
				r_type->road_type_ = Road::RoadType::ROADTYPE_BICYCLE;
			}
			else if (type == "")
			{
				LOG("Missing road type - setting default (rural)");
				r_type->road_type_ = Road::RoadType::ROADTYPE_RURAL;
			}
			else
			{
				LOG("Unsupported road type: %s - assuming rural", type.c_str());
				r_type->road_type_ = Road::RoadType::ROADTYPE_RURAL;
			}

			r_type->s_ = atof(type_node.attribute("s").value());

			// Check for optional speed record
			r_type->unit_ = SpeedUnit::MS;  // default
			pugi::xml_node speed = type_node.child("speed");
			if (speed != NULL)
			{
				r_type->speed_ = atof(speed.attribute("max").value());
				std::string unit = speed.attribute("unit").value();
				if (unit == "km/h")
				{
					r_type->speed_ /= 3.6;  // Convert to m/s from km/h
					r_type->unit_ = SpeedUnit::KMH;
				}
				else if (unit == "mph")
				{
					r_type->speed_ *= 0.44704; // Convert to m/s from mph
					r_type->unit_ = SpeedUnit::MPH;
				}
				else if (unit == "m/s")
				{
					// SI unit - do nothing
				}
				else
				{
					LOG("Unsupported speed unit: %s - set UNDEFINED", unit.c_str());
					r_type->unit_ = SpeedUnit::UNDEFINED;
				}
			}
			if (Position::GetOpenDrive()->GetSpeedUnit() == SpeedUnit::UNDEFINED)
			{
				Position::GetOpenDrive()->SetSpeedUnit(r_type->unit_);
			}

			r->AddRoadType(r_type);
		}

		pugi::xml_node link = road_node.child("link");
		if (link != NULL)
		{
			pugi::xml_node successor = link.child("successor");
			if (successor != NULL)
			{
				r->AddLink(new RoadLink(SUCCESSOR, successor));
			}

			pugi::xml_node predecessor = link.child("predecessor");
			if (predecessor != NULL)
			{
				r->AddLink(new RoadLink(PREDECESSOR, predecessor));
			}

			if (r->GetJunction() != -1)
			{
				// As connecting road it is expected to have connections in both ends
				if (successor == NULL)
				{
					LOG("Warning: connecting road %d in junction %d lacks successor", r->GetId(), r->GetJunction());
				}
				if (predecessor == NULL)
				{
					LOG("Warning: connecting road %d in junction %d lacks predesessor", r->GetId(), r->GetJunction());
				}
			}
		}

		pugi::xml_node plan_view = road_node.child("planView");
		if (plan_view != NULL)
		{
			for (pugi::xml_node geometry = plan_view.child("geometry"); geometry; geometry = geometry.next_sibling())
			{
				double s = atof(geometry.attribute("s").value());
				double x = atof(geometry.attribute("x").value());
				double y = atof(geometry.attribute("y").value());
				double hdg = atof(geometry.attribute("hdg").value());
				double glength = atof(geometry.attribute("length").value());

				pugi::xml_node type = geometry.last_child();
				if (type != NULL)
				{
					// Find out the type of geometry
					if (!strcmp(type.name(), "line"))
					{
						r->AddLine(new Line(s, x, y, hdg, glength));
					}
					else if (!strcmp(type.name(), "arc"))
					{
						double curvature = atof(type.attribute("curvature").value());
						r->AddArc(new Arc(s, x, y, hdg, glength, curvature));
					}
					else if (!strcmp(type.name(), "spiral"))
					{
						double curv_start = atof(type.attribute("curvStart").value());
						double curv_end = atof(type.attribute("curvEnd").value());
						r->AddSpiral(new Spiral(s, x, y, hdg, glength, curv_start, curv_end));
					}
					else if (!strcmp(type.name(), "poly3"))
					{
						double a = atof(type.attribute("a").value());
						double b = atof(type.attribute("b").value());
						double c = atof(type.attribute("c").value());
						double d = atof(type.attribute("d").value());
						r->AddPoly3(new Poly3(s, x, y, hdg, glength, a, b, c, d));
					}
					else if (!strcmp(type.name(), "paramPoly3"))
					{
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
						if (attr && !strcmp(attr.value(), "arcLength"))
						{
							p_range = ParamPoly3::P_RANGE_ARC_LENGTH;
						}

						ParamPoly3 *pp3 = new ParamPoly3(s, x, y, hdg, glength, aU, bU, cU, dU, aV, bV, cV, dV, p_range);
						if (pp3 != NULL)
						{
							r->AddParamPoly3(pp3);
						}
						else
						{
							LOG("ParamPoly3: Major error\n");
						}
					}
					else
					{
						cout << "Unknown geometry type: " << type.name() << endl;
					}
				}
				else
				{
					cout << "Type == NULL" << endl;
				}
			}
		}

		pugi::xml_node elevation_profile = road_node.child("elevationProfile");
		if (elevation_profile != NULL)
		{
			for (pugi::xml_node elevation = elevation_profile.child("elevation"); elevation; elevation = elevation.next_sibling())
			{
				double s = atof(elevation.attribute("s").value());
				double a = atof(elevation.attribute("a").value());
				double b = atof(elevation.attribute("b").value());
				double c = atof(elevation.attribute("c").value());
				double d = atof(elevation.attribute("d").value());

				Elevation *ep = new Elevation(s, a, b, c, d);
				if (ep != NULL)
				{
					r->AddElevation(ep);
				}
				else
				{
					LOG("Elevation: Major error\n");
				}
			}
		}

		pugi::xml_node super_elevation_profile = road_node.child("lateralProfile");
		if (super_elevation_profile != NULL)
		{
			for (pugi::xml_node super_elevation = super_elevation_profile.child("superelevation"); super_elevation; super_elevation = super_elevation.next_sibling("superelevation"))
			{
				double s = atof(super_elevation.attribute("s").value());
				double a = atof(super_elevation.attribute("a").value());
				double b = atof(super_elevation.attribute("b").value());
				double c = atof(super_elevation.attribute("c").value());
				double d = atof(super_elevation.attribute("d").value());

				Elevation *ep = new Elevation(s, a, b, c, d);
				if (ep != NULL)
				{
					r->AddSuperElevation(ep);
				}
				else
				{
					LOG("SuperElevation: Major error\n");
				}
			}
		}

		pugi::xml_node lanes = road_node.child("lanes");
		if (lanes != NULL)
		{
			for (pugi::xml_node_iterator child = lanes.children().begin(); child != lanes.children().end(); child++)
			{
				if (!strcmp(child->name(), "laneOffset"))
				{
					double s = atof(child->attribute("s").value());
					double a = atof(child->attribute("a").value());
					double b = atof(child->attribute("b").value());
					double c = atof(child->attribute("c").value());
					double d = atof(child->attribute("d").value());
					r->AddLaneOffset(new LaneOffset(s, a, b, c, d));
				}
				else if (!strcmp(child->name(), "laneSection"))
				{
					double s = atof(child->attribute("s").value());
					LaneSection *lane_section = new LaneSection(s);
					r->AddLaneSection(lane_section);

					for (pugi::xml_node_iterator child2 = child->children().begin(); child2 != child->children().end(); child2++)
					{
						if (!strcmp(child2->name(), "left"))
						{
							//LOG("Lane left\n");
						}
						else if (!strcmp(child2->name(), "right"))
						{
							//LOG("Lane right\n");
						}
						else if (!strcmp(child2->name(), "center"))
						{
							//LOG("Lane center\n");
						}
						else if (!strcmp(child2->name(), "userData"))
						{
							// Not supported
							continue;
						}
						else
						{
							LOG("Unsupported lane side: %s\n", child2->name());
							continue;
						}
						for (pugi::xml_node_iterator lane_node = child2->children().begin(); lane_node != child2->children().end(); lane_node++)
						{
							if (strcmp(lane_node->name(), "lane"))
							{
								LOG("Unexpected element: %s, expected \"lane\"\n", lane_node->name());
								continue;
							}

							Lane::LaneType lane_type = Lane::LANE_TYPE_NONE;
							if (lane_node->attribute("type") == 0 || !strcmp(lane_node->attribute("type").value(), ""))
							{
								LOG("Lane type error");
							}
							if (!strcmp(lane_node->attribute("type").value(), "none"))
							{
								lane_type = Lane::LANE_TYPE_NONE;
							}
							else  if (!strcmp(lane_node->attribute("type").value(), "driving"))
							{
								lane_type = Lane::LANE_TYPE_DRIVING;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "stop"))
							{
								lane_type = Lane::LANE_TYPE_STOP;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "shoulder"))
							{
								lane_type = Lane::LANE_TYPE_SHOULDER;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "biking"))
							{
								lane_type = Lane::LANE_TYPE_BIKING;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "sidewalk"))
							{
								lane_type = Lane::LANE_TYPE_SIDEWALK;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "border"))
							{
								lane_type = Lane::LANE_TYPE_BORDER;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "restricted"))
							{
								lane_type = Lane::LANE_TYPE_RESTRICTED;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "parking"))
							{
								lane_type = Lane::LANE_TYPE_PARKING;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "bidirectional"))
							{
								lane_type = Lane::LANE_TYPE_BIDIRECTIONAL;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "median"))
							{
								lane_type = Lane::LANE_TYPE_MEDIAN;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "special1"))
							{
								lane_type = Lane::LANE_TYPE_SPECIAL1;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "special2"))
							{
								lane_type = Lane::LANE_TYPE_SPECIAL2;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "special3"))
							{
								lane_type = Lane::LANE_TYPE_SPECIAL3;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "roadmarks"))
							{
								lane_type = Lane::LANE_TYPE_ROADMARKS;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "tram"))
							{
								lane_type = Lane::LANE_TYPE_TRAM;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "rail"))
							{
								lane_type = Lane::LANE_TYPE_RAIL;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "entry") ||
								!strcmp(lane_node->attribute("type").value(), "mwyEntry"))
							{
								lane_type = Lane::LANE_TYPE_ENTRY;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "exit") ||
								!strcmp(lane_node->attribute("type").value(), "mwyExit"))
							{
								lane_type = Lane::LANE_TYPE_EXIT;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "offRamp"))
							{
								lane_type = Lane::LANE_TYPE_OFF_RAMP;
							}
							else if (!strcmp(lane_node->attribute("type").value(), "onRamp"))
							{
								lane_type = Lane::LANE_TYPE_ON_RAMP;
							}
							else
							{
								LOG("unknown lane type: %s (road id=%d)\n", lane_node->attribute("type").value(), r->GetId());
							}

							int lane_id = atoi(lane_node->attribute("id").value());

							// If lane ID == 0, make sure it's not a driving lane
							if (lane_id == 0 && lane_type == Lane::LANE_TYPE_DRIVING)
							{
								lane_type = Lane::LANE_TYPE_NONE;
							}

							Lane *lane = new Lane(lane_id, lane_type);
							if (lane == NULL)
							{
								LOG("Error: creating lane\n");
								return false;
							}
							lane_section->AddLane(lane);

							// Link
							pugi::xml_node lane_link = lane_node->child("link");
							if (lane_link != NULL)
							{
								pugi::xml_node successor = lane_link.child("successor");
								if (successor != NULL)
								{
									lane->AddLink(new LaneLink(SUCCESSOR, atoi(successor.attribute("id").value())));
								}
								pugi::xml_node predecessor = lane_link.child("predecessor");
								if (predecessor != NULL)
								{
									lane->AddLink(new LaneLink(PREDECESSOR, atoi(predecessor.attribute("id").value())));
								}
							}

							// Width
							for (pugi::xml_node width = lane_node->child("width"); width; width = width.next_sibling("width"))
							{
								double s_offset = atof(width.attribute("sOffset").value());
								double a = atof(width.attribute("a").value());
								double b = atof(width.attribute("b").value());
								double c = atof(width.attribute("c").value());
								double d = atof(width.attribute("d").value());
								lane->AddLaneWidth(new LaneWidth(s_offset, a, b, c, d));
							}

							// roadMark
							for (pugi::xml_node roadMark = lane_node->child("roadMark"); roadMark; roadMark = roadMark.next_sibling("roadMark"))
							{
								// s_offset
								double s_offset = atof(roadMark.attribute("sOffset").value());

								// type
								LaneRoadMark::RoadMarkType roadMark_type = LaneRoadMark::NONE_TYPE;
								if (roadMark.attribute("type") == 0 || !strcmp(roadMark.attribute("type").value(), ""))
								{
									LOG("Lane road mark type error");
								}
								if (!strcmp(roadMark.attribute("type").value(), "none"))
								{
									roadMark_type = LaneRoadMark::NONE_TYPE;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "solid"))
								{
									roadMark_type = LaneRoadMark::SOLID;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "broken"))
								{
									roadMark_type = LaneRoadMark::BROKEN;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "solid solid"))
								{
									roadMark_type = LaneRoadMark::SOLID_SOLID;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "solid broken"))
								{
									roadMark_type = LaneRoadMark::SOLID_BROKEN;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "broken solid"))
								{
									roadMark_type = LaneRoadMark::BROKEN_SOLID;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "broken broken"))
								{
									roadMark_type = LaneRoadMark::BROKEN_BROKEN;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "botts dots"))
								{
									roadMark_type = LaneRoadMark::BOTTS_DOTS;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "grass"))
								{
									roadMark_type = LaneRoadMark::GRASS;
								}
								else  if (!strcmp(roadMark.attribute("type").value(), "curb"))
								{
									roadMark_type = LaneRoadMark::CURB;
								}
								else
								{
									LOG("unknown lane road mark type: %s (road id=%d)\n", roadMark.attribute("type").value(), r->GetId());
								}

								// weight - consider it optional with default value = STANDARD
								LaneRoadMark::RoadMarkWeight roadMark_weight = LaneRoadMark::STANDARD;
								if (roadMark.attribute("weight") != 0 && strcmp(roadMark.attribute("weight").value(), ""))
								{
									if (!strcmp(roadMark.attribute("weight").value(), "standard"))
									{
										roadMark_weight = LaneRoadMark::STANDARD;
									}
									else  if (!strcmp(roadMark.attribute("weight").value(), "bold"))
									{
										roadMark_weight = LaneRoadMark::BOLD;
									}
									else
									{
										LOG("unknown lane road mark weight: %s (road id=%d) setting to standard\n",
											roadMark.attribute("type").value(), r->GetId());
										roadMark_weight = LaneRoadMark::STANDARD;
									}
								}

								// color - consider it optional with default value = STANDARD_COLOR
								RoadMarkColor roadMark_color = LaneRoadMark::ParseColor(roadMark);
								if (GetVersionMajor() == 1 && GetVersionMinor() > 4 && roadMark_color == RoadMarkColor::UNDEFINED)
								{
									LOG("Missing lane road mark color: %s (road id=%d), set to standard (white)",
										LaneRoadMark::RoadMarkColor2Str(roadMark_color).c_str(), r->GetId());
									roadMark_color = RoadMarkColor::STANDARD_COLOR;
								}

								// material
								LaneRoadMark::RoadMarkMaterial roadMark_material = LaneRoadMark::STANDARD_MATERIAL;

								// optional laneChange
								LaneRoadMark::RoadMarkLaneChange roadMark_laneChange = LaneRoadMark::NONE_LANECHANGE;
								if (!roadMark.attribute("laneChange").empty())
								{
									if (!strcmp(roadMark.attribute("laneChange").value(), ""))
									{
										LOG("Lane roadmark lanechange error");
									}
									else
									{
										if (!strcmp(roadMark.attribute("laneChange").value(), "none"))
										{
											roadMark_laneChange = LaneRoadMark::NONE_LANECHANGE;
										}
										else  if (!strcmp(roadMark.attribute("laneChange").value(), "increase"))
										{
											roadMark_laneChange = LaneRoadMark::INCREASE;
										}
										else  if (!strcmp(roadMark.attribute("laneChange").value(), "decrease"))
										{
											roadMark_laneChange = LaneRoadMark::DECREASE;
										}
										else  if (!strcmp(roadMark.attribute("laneChange").value(), "both"))
										{
											roadMark_laneChange = LaneRoadMark::BOTH;
										}
										else
										{
											LOG("unknown lane road mark lane change: %s (road id=%d)\n", roadMark.attribute("laneChange").value(), r->GetId());
										}
									}
								}

								double roadMark_width;
								if (roadMark.attribute("width").empty())
								{
									roadMark_width = (roadMark_weight == LaneRoadMark::BOLD) ? ROADMARK_WIDTH_BOLD : ROADMARK_WIDTH_STANDARD;
								}
								else
								{
									roadMark_width = atof(roadMark.attribute("width").value());
								}

								double roadMark_height = atof(roadMark.attribute("height").value());
								LaneRoadMark *lane_roadMark = new LaneRoadMark(s_offset, roadMark_type, roadMark_weight, roadMark_color,
								roadMark_material, roadMark_laneChange, roadMark_width, roadMark_height);
								lane->AddLaneRoadMark(lane_roadMark);

								// sub_type
								LaneRoadMarkType* lane_roadMarkType = 0;
								for (pugi::xml_node sub_type = roadMark.child("type"); sub_type; sub_type = sub_type.next_sibling("type"))
								{
									if (sub_type != NULL)
									{
										std::string sub_type_name = sub_type.attribute("name").value();
										double sub_type_width = atof(sub_type.attribute("width").value());
										lane_roadMarkType = new LaneRoadMarkType(sub_type_name, sub_type_width);
										lane_roadMark->AddType(lane_roadMarkType);

										for (pugi::xml_node line = sub_type.child("line"); line; line = line.next_sibling("line"))
										{
											double llength = atof(line.attribute("length").value());
											double space = atof(line.attribute("space").value());
											double t_offset = atof(line.attribute("tOffset").value());
											double s_offset_l = atof(line.attribute("sOffset").value());

											if (!line.attribute("color").empty())
											{
												RoadMarkColor tmp_color = LaneRoadMark::ParseColor(line);
												if (tmp_color != RoadMarkColor::UNDEFINED)
												{
													roadMark_color = tmp_color;  // supersedes the setting in <RoadMark> element (available from odr v1.5)
												}
											}

											// rule (optional)
											LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule = LaneRoadMarkTypeLine::NONE;
											if (line.attribute("rule") != 0 && strcmp(line.attribute("rule").value(), ""))
											{
												if (!strcmp(line.attribute("rule").value(), "none"))
												{
													rule = LaneRoadMarkTypeLine::NONE;
												}
												else  if (!strcmp(line.attribute("rule").value(), "caution"))
												{
													rule = LaneRoadMarkTypeLine::CAUTION;
												}
												else  if (!strcmp(line.attribute("rule").value(), "no passing"))
												{
													rule = LaneRoadMarkTypeLine::NO_PASSING;
												}
												else
												{
													LOG("unknown lane road mark type line rule: %s (road id=%d)\n", line.attribute("rule").value(), r->GetId());
												}
											}

											double width = atof(line.attribute("width").value());

											LaneRoadMarkTypeLine *lane_roadMarkTypeLine = new LaneRoadMarkTypeLine(llength, space, t_offset, s_offset_l, rule, width,
												roadMark_color);
											lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
										}
									}
								}
								if (lane_roadMarkType == 0)
								{
									if (roadMark_type == LaneRoadMark::NONE_TYPE)
									{
										lane_roadMarkType = new LaneRoadMarkType("stand-in", roadMark_width);
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule = LaneRoadMarkTypeLine::NONE;
										LaneRoadMarkTypeLine* lane_roadMarkTypeLine =
											new LaneRoadMarkTypeLine(0, 0, 0, 0, rule, roadMark_width, roadMark_color);
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
									}
									else if (roadMark_type == LaneRoadMark::SOLID || roadMark_type == LaneRoadMark::CURB)
									{
										lane_roadMarkType = new LaneRoadMarkType("stand-in", roadMark_width);
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule = LaneRoadMarkTypeLine::NONE;
										LaneRoadMarkTypeLine* lane_roadMarkTypeLine =
											new LaneRoadMarkTypeLine(0, 0, 0, 0, rule, roadMark_width, roadMark_color);
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
									}
									else if (roadMark_type == LaneRoadMark::SOLID_SOLID)
									{
										lane_roadMarkType = new LaneRoadMarkType("stand-in", roadMark_width);
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule = LaneRoadMarkTypeLine::NONE;
										LaneRoadMarkTypeLine* lane_roadMarkTypeLine = new
											LaneRoadMarkTypeLine(0, 0, -roadMark_width, 0, rule, roadMark_width, roadMark_color);
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
										LaneRoadMarkTypeLine* lane_roadMarkTypeLine2 = new
											LaneRoadMarkTypeLine(0, 0, roadMark_width, 0, rule, roadMark_width, roadMark_color);
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine2);
									}
									else if (roadMark_type == LaneRoadMark::BROKEN)
									{
										lane_roadMarkType = new LaneRoadMarkType("stand-in", roadMark_width);
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule = LaneRoadMarkTypeLine::NONE;
										LaneRoadMarkTypeLine* lane_roadMarkTypeLine =
											new LaneRoadMarkTypeLine(4, 8, 0, 0, rule, roadMark_width, roadMark_color);
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
									}
									else if (roadMark_type == LaneRoadMark::BROKEN_BROKEN)
									{
										lane_roadMarkType = new LaneRoadMarkType("stand-in", roadMark_width);
										lane_roadMark->AddType(lane_roadMarkType);
										LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule = LaneRoadMarkTypeLine::NONE;
										LaneRoadMarkTypeLine* lane_roadMarkTypeLine =
											new LaneRoadMarkTypeLine(4, 8, -roadMark_width, 0, rule, roadMark_width, roadMark_color);
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine);
										LaneRoadMarkTypeLine* lane_roadMarkTypeLine2 =
											new LaneRoadMarkTypeLine(4, 8, roadMark_width, 0, rule, roadMark_width, roadMark_color);
										lane_roadMarkType->AddLine(lane_roadMarkTypeLine2);
									}
									else
									{
										LOG("No road mark created for road %d lane %d. Type %d not supported. Either switch type or add a roadMark <type> element.", r->GetId(), lane_id, roadMark_type);
									}
								}
							}
						}
					}
					// Check lane indices
					int lastLaneId = 0;
					for (int i = 0; i < lane_section->GetNumberOfLanes(); i++)
					{
						int laneId = lane_section->GetLaneByIdx(i)->GetId();
						if (i > 0 && laneId != lastLaneId - 1)
						{
							LOG("Warning: expected laneId %d missing of roadId %d. Found laneIds %d and %d",
								lastLaneId - 1, r->GetId(), lastLaneId, laneId);
						}
						lastLaneId = laneId;
					}
				}
				else
				{
					LOG("Unsupported lane type: %s\n", child->name());
				}
			}
		}

		road_.push_back(r);

		pugi::xml_node signals = road_node.child("signals");
		if (signals != NULL)
		{
			//Variables to check if the country file is loaded
			bool country_file_loaded = false;
			std::string current_country = "";
			for (pugi::xml_node signal = signals.child("signal"); signal; signal = signal.next_sibling())
			{
				if (!strcmp(signal.name(), "signal"))
				{
					double s = atof(signal.attribute("s").value());
					double t = atof(signal.attribute("t").value());
					int ids = atoi(signal.attribute("id").value());
					std::string name = signal.attribute("name").value();

					// dynamic
					bool dynamic = false;
					if (!strcmp(signal.attribute("dynamic").value(), ""))
					{
						LOG("Signal dynamic check error");
					}
					if (!strcmp(signal.attribute("dynamic").value(), "no"))
					{
						dynamic = false;
					}
					else  if (!strcmp(signal.attribute("dynamic").value(), "yes"))
					{
						dynamic = true;
					}
					else
					{
						LOG("unknown dynamic signal identification: %s (road ids=%d)\n", signal.attribute("dynamic").value(), r->GetId());
					}

					// orientation
					Signal::Orientation orientation = Signal::NONE;
					if (signal.attribute("orientation") == 0 || !strcmp(signal.attribute("orientation").value(), ""))
					{
						LOG("Road signal orientation error");
					}
					if (!strcmp(signal.attribute("orientation").value(), "none"))
					{
						orientation = Signal::NONE;
					}
					else  if (!strcmp(signal.attribute("orientation").value(), "+"))
					{
						orientation = Signal::POSITIVE;
					}
					else  if (!strcmp(signal.attribute("orientation").value(), "-"))
					{
						orientation = Signal::NEGATIVE;
					}
					else
					{
						LOG("unknown road signal orientation: %s (road ids=%d)\n", signal.attribute("orientation").value(), r->GetId());
					}

					double  z_offset = atof(signal.attribute("zOffset").value());
					std::string country = ToLower(signal.attribute("country").value());

					// Load the country file for types
					if(!country.empty() && (!country_file_loaded || current_country != country))
					{
						current_country = country;
						country_file_loaded = LoadSignalsByCountry(country);
					}

					std::string type;
					std::string subtype;
					std::string value;

					type = signal.attribute("type").value();
					subtype = signal.attribute("subtype").value();
					value = signal.attribute("value").value();
					int osi_type = static_cast<int>(Signal::OSIType::TYPE_UNKNOWN);

					if (!type.empty())
					{
						std::string type_to_find = Signal::GetCombinedTypeSubtypeValueStr(type, subtype, value);

						if (signals_types_.count(country + type_to_find) != 0)
						{
							std::string enum_string = signals_types_.find(country + type_to_find)->second;
							osi_type = static_cast<int>(Signal::GetOSITypeFromString(enum_string));
						}

						if (osi_type == static_cast<int>(Signal::OSIType::TYPE_UNKNOWN))
						{
							// Try without value
							if (signals_types_.count(country + type + (subtype.empty() ? "" : "." + subtype)) != 0)
							{
								std::string enum_string = signals_types_.find(country + type + (subtype.empty() ? "" : "." + subtype))->second;
								osi_type = static_cast<int>(Signal::GetOSITypeFromString(enum_string));
							}
							if (osi_type == static_cast<int>(Signal::OSIType::TYPE_UNKNOWN))
							{
								LOG("Signal Type %s doesn't exists for country %s", type_to_find.c_str(), country.c_str());
							}
						}
					}

					std::string unit = signal.attribute("unit").value();
					double height = atof(signal.attribute("height").value());
					double width = atof(signal.attribute("width").value());
					std::string text = signal.attribute("text").value();
					double h_offset = atof(signal.attribute("hOffset").value());
					double pitch = atof(signal.attribute("pitch").value());
					double roll = atof(signal.attribute("roll").value());

					Position pos(rid, s, t);

					Signal* sig = new Signal(s, t, ids, name, dynamic, orientation, z_offset, country, osi_type, type, subtype, value, unit, height,
						width, text, h_offset, pitch, roll, pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetHRoad() + (orientation == Signal::Orientation::NEGATIVE ? M_PI : 0.0));
					if (sig != NULL)
					{
						r->AddSignal(sig);
					}
					else
					{
						LOG("Signal: Major error\n");
					}

					for (pugi::xml_node validity_node = signal.child("validity"); validity_node; validity_node = validity_node.next_sibling("validity"))
					{
						ValidityRecord validity;
						validity.fromLane_ = atoi(validity_node.attribute("fromLane").value());
						validity.toLane_ = atoi(validity_node.attribute("toLane").value());
						sig->validity_.push_back(validity);
					}
				}
				else
				{
					LOG_ONCE("INFO: signal element \"%s\" not supported yet", signal.name());
				}
			}
		}

		pugi::xml_node objects = road_node.child("objects");
		if (objects != NULL)
		{
			for (pugi::xml_node object = objects.child("object"); object; object = object.next_sibling("object"))
			{
				// Read any repeat element first, since its s-value overrides the one in the object element

				std::vector<Repeat*> Repeats;
				for (pugi::xml_node repeat_node = object.child("repeat"); repeat_node; repeat_node = repeat_node.next_sibling("repeat"))
				{
					std::string rattr;
					double rs = (rattr = ReadAttribute(repeat_node, "s", true)) == "" ? 0.0 : std::stod(rattr);
					double rlength = (rattr = ReadAttribute(repeat_node, "length", true)) == "" ? 0.0 : std::stod(rattr);
					double rdistance = (rattr = ReadAttribute(repeat_node, "distance", true)) == "" ? 0.0 : std::stod(rattr);
					double rtStart = (rattr = ReadAttribute(repeat_node, "tStart", true)) == "" ? 0.0 : std::stod(rattr);
					double rtEnd = (rattr = ReadAttribute(repeat_node, "tEnd", true)) == "" ? 0.0 : std::stod(rattr);
					double rheightStart = (rattr = ReadAttribute(repeat_node, "heightStart", true)) == "" ? 0.0 : std::stod(rattr);
					double rheightEnd = (rattr = ReadAttribute(repeat_node, "heightEnd", true)) == "" ? 0.0 : std::stod(rattr);
					double rzOffsetStart = (rattr = ReadAttribute(repeat_node, "zOffsetStart", true)) == "" ? 0.0 : std::stod(rattr);
					double rzOffsetEnd = (rattr = ReadAttribute(repeat_node, "zOffsetEnd", true)) == "" ? 0.0 : std::stod(rattr);

					double rwidthStart = (rattr = ReadAttribute(repeat_node, "widthStart", false)) == "" ? 0.0 : std::stod(rattr);
					double rwidthEnd = (rattr = ReadAttribute(repeat_node, "widthEnd", false)) == "" ? 0.0 : std::stod(rattr);
					double rlengthStart = (rattr = ReadAttribute(repeat_node, "lengthStart", false)) == "" ? 0.0 : std::stod(rattr);
					double rlengthEnd = (rattr = ReadAttribute(repeat_node, "lengthEnd", false)) == "" ? 0.0 : std::stod(rattr);
					double rradiusStart = (rattr = ReadAttribute(repeat_node, "radiusStart", false)) == "" ? 0.0 : std::stod(rattr);
					double rradiusEnd = (rattr = ReadAttribute(repeat_node, "radiusEnd", false)) == "" ? 0.0 : std::stod(rattr);

					Repeat* repeat = new Repeat(rs, rlength, rdistance, rtStart, rtEnd, rheightStart, rheightEnd, rzOffsetStart, rzOffsetEnd);
					Repeats.push_back(repeat);

					if (fabs(rwidthStart) > SMALL_NUMBER) repeat->SetWidthStart(rwidthStart);
					if (fabs(rwidthEnd) > SMALL_NUMBER) repeat->SetWidthEnd(rwidthEnd);
					if (fabs(rlengthStart) > SMALL_NUMBER) repeat->SetLengthStart(rlengthStart);
					if (fabs(rlengthEnd) > SMALL_NUMBER) repeat->SetLengthEnd(rlengthEnd);

					if (fabs(rradiusStart) > SMALL_NUMBER) printf("Attribute object/repeat/radiusStart not supported yet\n");
					if (fabs(rradiusEnd) > SMALL_NUMBER) printf("Attribute object/repeat/radiusEnd not supported yet\n");


				}

				double s;
				if (Repeats.size() > 0)
				{
					s = Repeats[0]->GetS();
				}
				else
				{
					s = atof(object.attribute("s").value());
				}
				double t = atof(object.attribute("t").value());
				int ids = atoi(object.attribute("id").value());
				std::string name = object.attribute("name").value();

				// orientation
				RMObject::Orientation orientation = RMObject::Orientation::NONE;
				if (object.attribute("orientation") != 0 && strcmp(object.attribute("orientation").value(), ""))
				{
					if (!strcmp(object.attribute("orientation").value(), "none"))
					{
						orientation = RMObject::Orientation::NONE;
					}
					else  if (!strcmp(object.attribute("orientation").value(), "+"))
					{
						orientation = RMObject::Orientation::POSITIVE;
					}
					else  if (!strcmp(object.attribute("orientation").value(), "-"))
					{
						orientation = RMObject::Orientation::NEGATIVE;
					}
					else
					{
						LOG("unknown road object orientation: %s (road ids=%d)\n", object.attribute("orientation").value(), r->GetId());
					}
				}
				std::string type_str = object.attribute("type").value();
				RMObject::ObjectType type = RMObject::Str2Type(type_str);
				double z_offset = atof(object.attribute("zOffset").value());
				double length = atof(object.attribute("length").value());
				double height = atof(object.attribute("height").value());
				double width = atof(object.attribute("width").value());
				double heading = atof(object.attribute("hdg").value());
				double pitch = atof(object.attribute("pitch").value());
				double roll = atof(object.attribute("roll").value());

				Position pos(rid, s, t);

				RMObject* obj = new RMObject(s, t, ids, name, orientation, z_offset, type, length, height,
					width, heading, pitch, roll, pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetHRoad());

				if (Repeats.size() > 0)
				{
					for(Repeat* rp: Repeats)
					{
						obj->AddRepeat(rp);
					}
					obj->SetRepeat(Repeats[0]);
				}

				pugi::xml_node outlines_node = object.child("outlines");
				if (outlines_node != NULL)
				{
					for (pugi::xml_node outline_node = outlines_node.child("outline"); outline_node; outline_node = outline_node.next_sibling())
					{
						int id = atoi(outline_node.attribute("id").value());
						bool closed = !strcmp(outline_node.attribute("closed").value(), "true") ? true : false;
						Outline* outline = new Outline(id, Outline::FillType::FILL_TYPE_UNDEFINED, closed);

						for (pugi::xml_node corner_node = outline_node.first_child(); corner_node; corner_node = corner_node.next_sibling())
						{
							OutlineCorner* corner = 0;

							if (!strcmp(corner_node.name(), "cornerRoad"))
							{
								double sc = atof(corner_node.attribute("s").value());
								double tc = atof(corner_node.attribute("t").value());
								double dz = atof(corner_node.attribute("dz").value());
								double heightc = atof(corner_node.attribute("height").value());

								corner = (OutlineCorner*)(new OutlineCornerRoad(r->GetId(), sc, tc, dz, heightc, s, t, heading));
							}
							else if (!strcmp(corner_node.name(), "cornerLocal"))
							{
								double u = atof(corner_node.attribute("u").value());
								double v = atof(corner_node.attribute("v").value());
								double zLocal = atof(corner_node.attribute("z").value());
								double heightc = atof(corner_node.attribute("height").value());

								corner = (OutlineCorner*)(new OutlineCornerLocal(r->GetId(), obj->GetS(), obj->GetT(), u, v, zLocal, heightc, heading));
							}
							outline->AddCorner(corner);
						}
						obj->AddOutline(outline);
					}
				}

				for (pugi::xml_node validity_node = object.child("validity"); validity_node; validity_node = validity_node.next_sibling("validity"))
				{
					ValidityRecord validity;
					validity.fromLane_ = atoi(validity_node.attribute("fromLane").value());
					validity.toLane_ = atoi(validity_node.attribute("toLane").value());
					obj->validity_.push_back(validity);
				}

				if (obj != NULL)
				{
					r->AddObject(obj);
				}
				else
				{
					LOG("RMObject: Major error\n");
				}
			}
		}

		if (r->GetNumberOfLaneSections() == 0)
		{
			// Add empty center reference lane
			LaneSection *lane_section = new LaneSection(0.0);
			lane_section->AddLane(new Lane(0, Lane::LANE_TYPE_NONE));
			r->AddLaneSection(lane_section);
		}
	}

	for (pugi::xml_node controller_node = node.child("controller"); controller_node; controller_node = controller_node.next_sibling("controller"))
	{
		int id = atoi(controller_node.attribute("id").value());
		std::string name = controller_node.attribute("name").value();
		int sequence = atoi(controller_node.attribute("sequence").value());
		Controller controller(id, name, sequence);

		for (pugi::xml_node control_node = controller_node.child("control"); control_node; control_node = control_node.next_sibling("control"))
		{
			Control control;

			control.signalId_ = atoi(control_node.attribute("signalId").value());
			control.type_ = control_node.attribute("type").value();
			controller.AddControl(control);
		}

		AddController(controller);
	}

	for (pugi::xml_node junction_node = node.child("junction"); junction_node; junction_node = junction_node.next_sibling("junction"))
	{
		int idj = atoi(junction_node.attribute("id").value());
		std::string name = junction_node.attribute("name").value();
		std::string junction_type_str = junction_node.attribute("type").value();

		Junction::JunctionType junction_type = Junction::JunctionType::DEFAULT;
		if (junction_type_str == "direct")
		{
			junction_type = Junction::JunctionType::DIRECT;
		}
		else if (junction_type_str == "virtual")
		{
			LOG("Virtual junction type found. Not supported yet. Continue treating it as default type");
			junction_type = Junction::JunctionType::DEFAULT;
		}

		Junction *j = new Junction(idj, name, junction_type);

		for (pugi::xml_node connection_node = junction_node.child("connection"); connection_node; connection_node = connection_node.next_sibling("connection"))
		{
			if (connection_node != NULL)
			{
				int idc = atoi(connection_node.attribute("id").value());
				(void)idc;
				int incoming_road_id = atoi(connection_node.attribute("incomingRoad").value());
				Road *incoming_road = GetRoadById(incoming_road_id);

				int connecting_road_id = -1;
				if (junction_type == Junction::JunctionType::DIRECT)
				{
					connecting_road_id = atoi(connection_node.attribute("linkedRoad").value());
				}
				else
				{
					connecting_road_id = atoi(connection_node.attribute("connectingRoad").value());
				}
				Road* connecting_road = GetRoadById(connecting_road_id);

				if (connecting_road == nullptr)
				{
					LOG("Missing connecting road with id %d", connecting_road_id);
					return false;
				}

				// Check that the connecting road is referring back to this junction
				if (j->GetType() != Junction::JunctionType::DIRECT && connecting_road->GetJunction() != j->GetId())
				{
					LOG("Warning: Connecting road (id %d) junction attribute (%d) is not referring back to junction %d which is making use of it",
						connecting_road->GetId(), connecting_road->GetJunction(), j->GetId());
				}

				ContactPointType contact_point = CONTACT_POINT_UNDEFINED;
				std::string contact_point_str = connection_node.attribute("contactPoint").value();
				if (contact_point_str == "start")
				{
					contact_point = CONTACT_POINT_START;
				}
				else if (contact_point_str == "end")
				{
					contact_point = CONTACT_POINT_END;
				}
				else
				{
					LOG("Unsupported contact point: %s\n", contact_point_str.c_str());
				}

				Connection *connection = new Connection(incoming_road, connecting_road, contact_point);

				for (pugi::xml_node lane_link_node = connection_node.child("laneLink"); lane_link_node; lane_link_node = lane_link_node.next_sibling("laneLink"))
				{
					int from_id = atoi(lane_link_node.attribute("from").value());
					int to_id = atoi(lane_link_node.attribute("to").value());
					connection->AddJunctionLaneLink(from_id, to_id);
				}
				j->AddConnection(connection);
			}
		}

		for (pugi::xml_node controller_node = junction_node.child("controller"); controller_node; controller_node = controller_node.next_sibling("controller"))
		{
			JunctionController controller;
			controller.id_ = atoi(controller_node.attribute("id").value());
			controller.type_ = controller_node.attribute("type").value();
			controller.sequence_ = atoi(controller_node.attribute("sequence").value());
			j->AddController(controller);
		}

		junction_.push_back(j);
	}

	CheckConnections();

	if (!SetRoadOSI())
	{
		LOG("Failed to create OSI points for OpenDrive road!");
	}

	return true;
}

void RMObject::SetRepeat(Repeat* repeat)
{
	repeat_ = repeat;
}

Connection::Connection(Road* incoming_road, Road *connecting_road, ContactPointType contact_point)
{
	// Find corresponding road objects
	incoming_road_ = incoming_road;
	connecting_road_ = connecting_road;
	contact_point_ = contact_point;
}

Connection::~Connection()
{
	for (size_t i=0; i<lane_link_.size(); i++)
	{
		delete lane_link_[i];
	}
}

void Connection::AddJunctionLaneLink(int from, int to)
{
	lane_link_.push_back(new JunctionLaneLink(from, to));
}

int Connection::GetConnectingLaneId(int incoming_lane_id)
{
	for (size_t i = 0; i < lane_link_.size(); i++)
	{
		if (lane_link_[i]->from_ == incoming_lane_id)
		{
			return lane_link_[i]->to_;
		}
	}
	return 0;
}

void Connection::Print()
{
	LOG("Connection: incoming %d connecting %d\n", incoming_road_->GetId(), connecting_road_->GetId());
	for (size_t i = 0; i < lane_link_.size(); i++)
	{
		lane_link_[i]->Print();
	}
}

Junction::~Junction()
{
	for (size_t i=0; i<connection_.size(); i++)
	{
		delete connection_[i];
	}
}

int Junction::GetNumberOfRoadConnections(int roadId, int laneId)
{
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++)
	{
		Connection * connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad() && roadId == connection->GetIncomingRoad()->GetId())
		{
			for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++)
			{
				JunctionLaneLink *lane_link = connection->GetLaneLink(j);
				if (laneId == lane_link->from_)
				{
					counter++;
				}
			}
		}
	}
	return counter;
}

LaneRoadLaneConnection Junction::GetRoadConnectionByIdx(int roadId, int laneId, int idx, int laneTypeMask)
{
	int counter = 0;
	LaneRoadLaneConnection lane_road_lane_connection;

	for (int i = 0; i < GetNumberOfConnections(); i++)
	{
		Connection *connection = GetConnectionByIdx(i);

		if (connection && connection->GetIncomingRoad() && roadId == connection->GetIncomingRoad()->GetId())
		{
			for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++)
			{
				JunctionLaneLink *lane_link = connection->GetLaneLink(j);
				if (laneId == lane_link->from_)
				{
					if (counter == idx)
					{
						lane_road_lane_connection.SetLane(laneId);
						lane_road_lane_connection.contact_point_ = connection->GetContactPoint();
						lane_road_lane_connection.SetConnectingRoad(connection->GetConnectingRoad()->GetId());
						lane_road_lane_connection.SetConnectingLane(lane_link->to_);
						// find out driving direction
						int laneSectionId;
						if (lane_link->to_ < 0)
						{
							laneSectionId = 0;
						}
						else
						{
							laneSectionId = connection->GetConnectingRoad()->GetNumberOfLaneSections() - 1;
						}
						Lane* lane = connection->GetConnectingRoad()->GetLaneSectionByIdx(laneSectionId)->GetLaneById(lane_link->to_);

						if (lane == nullptr)
						{
							LOG("Error: OpenDrive::GetJunctionConnection target lane not found! from %d, %d to %d, %d\n",
								roadId, laneId, connection->GetConnectingRoad()->GetId(), lane_link->to_);
						}
						else if (!(lane->GetLaneType() & laneTypeMask))
						{
							LOG("OpenDrive::GetJunctionConnection target lane not driving! from %d, %d to %d, %d\n",
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
void Junction::SetGlobalId()
{
	global_id_ = GetNewGlobalLaneId();
}

bool Junction::IsOsiIntersection()
{
	if (connection_.size() > 0 && connection_[0]->GetIncomingRoad() && connection_[0]->GetIncomingRoad() && connection_[0]->GetIncomingRoad()->GetRoadType(0) != 0)
	{
		if (connection_[0]->GetIncomingRoad()->GetRoadType(0)->road_type_ == Road::RoadType::ROADTYPE_MOTORWAY)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		LOG_ONCE("Type of roads are missing, cannot determine for OSI intersection or not, assuming that it is an intersection.");
		return true;
	}
}

int Junction::GetNoConnectionsFromRoadId(int incomingRoadId)
{
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++)
	{
		Connection * connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad()->GetId() == incomingRoadId)
		{
			counter++;
		}
	}

	return counter;
}

int Junction::GetConnectingRoadIdFromIncomingRoadId(int incomingRoadId, int index)
{
	int counter = 0;

	for (int i = 0; i < GetNumberOfConnections(); i++)
	{
		Connection * connection = GetConnectionByIdx(i);
		if (connection && connection->GetIncomingRoad()->GetId() == incomingRoadId)
		{
			if (counter == index)
			{
				return GetConnectionByIdx(i)->GetConnectingRoad()->GetId();
			}
			else
			{
				counter++;
			}
		}
	}
	return -1;
}

void Junction::Print()
{
	LOG("Junction %d %s: \n", id_, name_.c_str());

	for (size_t i=0; i<connection_.size(); i++)
	{
		connection_[i]->Print();
	}
}

JunctionController* Junction::GetJunctionControllerByIdx(int index)
{
	if (index >= 0 && index < controller_.size())
	{
		return &controller_[index];
	}

	return 0;
}

Road* Junction::GetRoadAtOtherEndOfConnectingRoad(Road* connecting_road, Road* incoming_road)
{
	if (connecting_road->GetJunction() == 0)
	{
		LOG("Unexpected: Road %d not a connecting road", connecting_road->GetId());
		return 0;
	}

	// Check both ends
	LinkType link_type[2] = { LinkType::PREDECESSOR, LinkType::SUCCESSOR };
	for (int i = 0; i < 2; i++)
	{
		RoadLink* link = connecting_road->GetLink(link_type[i]);
		if (link && link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
		{
			if (link->GetElementId() == incoming_road->GetId())
			{
				// Get road at other end
				RoadLink* link2 = connecting_road->GetLink(link_type[(i + 1) % 2]);
				if (link2 && link2->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
				{
					return Position::GetOpenDrive()->GetRoadById(link2->GetElementId());
				}
			}
		}
	}

	LOG("Failed to find road at other end of the connecting road %d from road %d", connecting_road->GetId(), incoming_road->GetId());
	return nullptr;
}

bool RoadPath::CheckRoad(Road *checkRoad, RoadPath::PathNode *srcNode, Road *fromRoad, int fromLaneId)
{
	// Register length of this road and find node in other end of the road (link)
	RoadLink* nextLink = 0;
	ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;

	int nextLaneId = fromRoad->GetConnectingLaneId(srcNode->link, fromLaneId, checkRoad->GetId());
	if (nextLaneId == 0)
	{
		return false;
	}

	if (srcNode->link->GetElementType() == RoadLink::RoadLink::ELEMENT_TYPE_ROAD)
	{
		// node link is a road, find link in the other end of it
		if (srcNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END)
		{
			nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
			contact_point = ContactPointType::CONTACT_POINT_START;
		}
		else
		{
			nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
			contact_point = ContactPointType::CONTACT_POINT_END;
		}
	}
	else if (srcNode->link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		Junction* junction = Position::GetOpenDrive()->GetJunctionById(srcNode->link->GetElementId());
		if (junction && junction->GetType() == Junction::JunctionType::DIRECT)
		{
			if (checkRoad->GetLink(LinkType::SUCCESSOR) && checkRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == junction->GetId())
			{
				// Node link is a direct junction. The pivot road (fromRoad) is successor
				// to the checked road. Hence next link is in the other end, predecessor.
				nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
				contact_point = ContactPointType::CONTACT_POINT_START;
			}
			else if (checkRoad->GetLink(LinkType::PREDECESSOR) && checkRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == junction->GetId())
			{
				// Node link is a direct junction. The pivot road (fromRoad) is predecessor
				// to the checked road. Hence next link is in the other end, successor.
				nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
				contact_point = ContactPointType::CONTACT_POINT_END;
			}
		}
		else
		{
			if (checkRoad->GetLink(LinkType::SUCCESSOR) &&
				checkRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == fromRoad->GetId())
			{
				// Node link is a non direct junction. The pivot road (fromRoad) is successor
				// to the checked road. Hence next link is in the other end, predecessor.
				nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
				contact_point = ContactPointType::CONTACT_POINT_START;
			}
			else if (checkRoad->GetLink(LinkType::PREDECESSOR) &&
				checkRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == fromRoad->GetId())
			{
				// Node link is a non direct junction. The pivot road (fromRoad) is predecessor
				// to the checked road. Hence next link is in the other end, successor.
				nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
				contact_point = ContactPointType::CONTACT_POINT_END;
			}
		}
	}

	if (nextLink == 0)
	{
		// end of road
		return false;
	}

	if (contact_point == ContactPointType::CONTACT_POINT_START)
	{
		nextLaneId = checkRoad->GetConnectedLaneIdAtS(nextLaneId, -1.0, 0.0);
	}
	else
	{
		nextLaneId = checkRoad->GetConnectedLaneIdAtS(nextLaneId, 0.0, -1.0);
	}

	// Check if next node is already visited
	for (size_t i = 0; i < visited_.size(); i++)
	{
		if (visited_[i]->link == nextLink)
		{
			// Already visited, ignore and return
			return false;
		}
	}

	// Check if next node is already among unvisited
	size_t i;
	for (i = 0; i < unvisited_.size(); i++)
	{
		if (unvisited_[i]->link == nextLink)
		{
			// Consider it, i.e. calc distance and potentially store it (if less than old)
			if (srcNode->dist + checkRoad->GetLength() < unvisited_[i]->dist)
			{
				unvisited_[i]->dist = srcNode->dist + checkRoad->GetLength();
			}
		}
	}

	if (i == unvisited_.size())
	{
		// link not visited before, add it
		PathNode *pNode = new PathNode;
		pNode->dist = srcNode->dist + checkRoad->GetLength();
		pNode->link = nextLink;
		pNode->fromRoad = checkRoad;
		pNode->fromLaneId = nextLaneId;
		pNode->previous = srcNode;
		pNode->contactPoint = contact_point;
		unvisited_.push_back(pNode);
	}

	return true;
}

int RoadPath::Calculate(double &dist, bool bothDirections, double maxDist)
{
	OpenDrive* odr = startPos_->GetOpenDrive();
	RoadLink *link = 0;
	Junction *junction = 0;
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

	if (pivotRoad == nullptr)
	{
		LOG("Invalid startpos road ID: %d", startPos_->GetTrackId());
		return -2;
	}

	if (targetRoad == nullptr)
	{
		LOG("Invalid targetpos road ID: %d", targetPos_->GetTrackId());
		return -2;
	}

	for (i = 0; i < (bothDirections ? 2 : 1); i++)
	{
		ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
		if (bothDirections)
		{
			if (i == 0)
			{
				contact_point = ContactPointType::CONTACT_POINT_START;
				link = pivotRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
			}
			else
			{
				contact_point = ContactPointType::CONTACT_POINT_END;
				link = pivotRoad->GetLink(LinkType::SUCCESSOR);  // Find link to previous road or junction
			}
		}
		else
		{
			// Look only in forward direction, w.r.t. entity heading
			if (startPos_->GetHRelative() < M_PI_2 || startPos_->GetHRelative() > 3 * M_PI_2)
			{
				// Along road direction
				contact_point = ContactPointType::CONTACT_POINT_END;
				link = pivotRoad->GetLink(LinkType::SUCCESSOR);  // Find link to next road or junction
			}
			else
			{
				// Opposite road direction
				contact_point = ContactPointType::CONTACT_POINT_START;
				link = pivotRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
			}
		}

		if (link)
		{
			PathNode* pNode = new PathNode;
			pNode->link = link;
			pNode->fromRoad = pivotRoad;

			if (contact_point == ContactPointType::CONTACT_POINT_END)
			{
				pivotLaneId = pivotRoad->GetConnectedLaneIdAtS(pivotLaneId, startPos_->GetS(), -1.0);
			}
			else if (contact_point == ContactPointType::CONTACT_POINT_START)
			{
				pivotLaneId = pivotRoad->GetConnectedLaneIdAtS(pivotLaneId, startPos_->GetS(), 0);
			}
			else
			{
				LOG("Unexpected contact point type: %d", contact_point);
			}

			pNode->fromLaneId = pivotLaneId;
			pNode->previous = 0;
			pNode->contactPoint = contact_point;
			if (contact_point == ContactPointType::CONTACT_POINT_START)
			{
				pNode->dist = startPos_->GetS();  // distance to first road link is distance to start of road
			}
			else if (contact_point == ContactPointType::CONTACT_POINT_END)
			{
				pNode->dist = pivotRoad->GetLength() - startPos_->GetS();  // distance to end of road
			}

			unvisited_.push_back(pNode);
		}
	}

	if (startRoad == targetRoad)
	{
		dist = targetPos_->GetS() - startPos_->GetS();

		// Special case: On same road, distance is equal to delta s
		if (startPos_->GetLaneId() < 0)
		{
			if (startPos_->GetHRelative() > M_PI_2 && startPos_->GetHRelative() < 3 * M_PI_2)
			{
				// facing opposite road direction
				dist *= -1;
			}
		}
		else
		{
			// decreasing in lanes with positive IDs
			dist *= -1;

			if (startPos_->GetHRelative() < M_PI_2 || startPos_->GetHRelative() > 3 * M_PI_2)
			{
				// facing along road direction
				dist *= -1;
			}
		}

		return 0;
	}

	if (unvisited_.size() == 0)
	{
		// No links
		dist = 0;
		return -1;
	}

	for (i = 0; i < 100 && !found && unvisited_.size() > 0 && tmpDist < maxDist; i++)
	{
		found = false;

		// Find unvisited PathNode with shortest distance
		double minDist = LARGE_NUMBER;
		int minIndex = 0;
		for (size_t j = 0; j < unvisited_.size(); j++)
		{
			if (unvisited_[j]->dist < minDist)
			{
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
		if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
		{
			// only one edge (road)
			nextRoad = odr->GetRoadById(link->GetElementId());

			if (nextRoad == targetRoad)
			{
				// Special case: On same road, distance is equal to delta s, direction considered
				if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_START)
				{
					tmpDist += targetPos_->GetS();
				}
				else
				{
					tmpDist += nextRoad->GetLength() - targetPos_->GetS();
				}

				found = true;
			}
			else
			{
				CheckRoad(nextRoad, unvisited_[minIndex], pivotRoad, pivotLaneId);
			}
		}
		else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
		{
			// check all junction links (connecting roads) that has pivot road as incoming road
			junction = odr->GetJunctionById(link->GetElementId());
			if (junction == nullptr)
			{
				LOG("Failed to lookup junction with id %d", link->GetElementId());
			}
			for (size_t j = 0; junction && j < junction->GetNoConnectionsFromRoadId(pivotRoad->GetId()); j++)
			{
				nextRoad = odr->GetRoadById(junction->GetConnectingRoadIdFromIncomingRoadId(pivotRoad->GetId(), (int)j));
				if (nextRoad == 0)
				{
					return 0;
				}

				if (nextRoad == targetRoad)  // target road reached
				{
					ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
					//if (nextRoad->IsSuccessor(pivotRoad, &contact_point) || nextRoad->IsPredecessor(pivotRoad, &contact_point))
					if (pivotRoad->IsSuccessor(nextRoad, &contact_point) || pivotRoad->IsPredecessor(nextRoad, &contact_point))
					{
						if (contact_point == ContactPointType::CONTACT_POINT_START)
						{
							tmpDist += targetPos_->GetS();
						}
						else if (contact_point == ContactPointType::CONTACT_POINT_END)
						{
							tmpDist += nextRoad->GetLength() - targetPos_->GetS();
						}
						else
						{
							LOG("Unexpected contact point %s", OpenDrive::ContactPointType2Str(contact_point).c_str());
							return -1;
						}
					}
					else
					{
						LOG("Failed to check link in junction");
						return -1;
					}
					found = true;
				}
				else
				{
					CheckRoad(nextRoad, unvisited_[minIndex], pivotRoad, pivotLaneId);
				}
			}
		}

		// Mark pivot link as visited (move it from unvisited to visited)
		visited_.push_back(unvisited_[minIndex]);
		unvisited_.erase(unvisited_.begin() + minIndex);
	}

	if (found)
	{
		// Find out whether the path goes forward or backwards from starting position
		if (visited_.size() > 0)
		{
			RoadPath::PathNode* node = visited_.back();

			while (node)
			{
				if (node->previous == 0)
				{
					// This is the first node - inspect whether it is in front or behind start position
					bool isPred = node->link == startRoad->GetLink(LinkType::PREDECESSOR);
					bool isGTPi2 = abs(startPos_->GetHRelative()) > M_PI_2;
					bool isLT3Pi2 = abs(startPos_->GetHRelative()) < 3 * M_PI / 2;
					bool isSucc = node->link == startRoad->GetLink(LinkType::SUCCESSOR);
					bool isLTPi2 = !isGTPi2;
					bool isGT3Pi2 = !isLT3Pi2;
					bool isPredAndBack = isPred && isGTPi2 && isLT3Pi2;
					bool isSuccAndFront = isSucc && (isLTPi2 || isGT3Pi2);
					if (isPredAndBack || isSuccAndFront)
					{
						direction_ = 1;
					}
					else
					{
						direction_ = -1;
					}
					firstNode_ = node;
				}
				node = node->previous;
			}
		}
	}

	dist = direction_ * tmpDist;

	// Also take intial heading of the start position into consideration for the sign of the distance
	if (startPos_->GetHRelativeDrivingDirection() > M_PI_2 && startPos_->GetHRelativeDrivingDirection() < 3 * M_PI_2)
	{
		dist *= -1;
	}

	return found ? 0 : -1;
}


RoadPath::~RoadPath()
{
	for (size_t i = 0; i < visited_.size(); i++)
	{
		delete(visited_[i]);
	}
	visited_.clear();

	for (size_t i = 0; i < unvisited_.size(); i++)
	{
		delete(unvisited_[i]);
	}
	unvisited_.clear();
}

OpenDrive::~OpenDrive()
{
	Clear();
}

int OpenDrive::GetTrackIdxById(int id)
{
	for (int i = 0; i<(int)road_.size(); i++)
	{
		if (road_[i]->GetId() == id)
		{
			return i;
		}
	}
	LOG("OpenDrive::GetTrackIdxById Error: Road id %d not found\n", id);
	return -1;
}

int OpenDrive::GetTrackIdByIdx(int idx)
{
	if (idx >= 0 && idx < (int)road_.size())
	{
		return (road_[idx]->GetId());
	}
	LOG("OpenDrive::GetTrackIdByIdx: idx %d out of range [0:%d]\n", idx, (int)road_.size());
	return 0;
}

bool OpenDrive::IsIndirectlyConnected(int road1_id, int road2_id, int* &connecting_road_id, int* &connecting_lane_id, int lane1_id, int lane2_id)
{
	Road *road1 = GetRoadById(road1_id);
	Road *road2 = GetRoadById(road2_id);
	RoadLink *link = 0;

	LinkType link_type[2] = { SUCCESSOR , PREDECESSOR };

	// Try both ends
	for (int k = 0; k < 2; k++)
	{
		link = road1->GetLink(link_type[k]);
		if (link == 0)
		{
			continue;
		}

		LaneSection *lane_section = 0;

		if (link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
		{
			if (link->GetElementId() == road2->GetId())
			{
				if (lane1_id != 0 && lane2_id != 0)
				{
					// Check lane connected
					if (link_type[k] == SUCCESSOR)
					{
						lane_section = road1->GetLaneSectionByIdx(road1->GetNumberOfLaneSections() - 1);
					}
					else if (link_type[k] == PREDECESSOR)
					{
						lane_section = road1->GetLaneSectionByIdx(0);
					}
					else
					{
						LOG("Error LinkType %d not suppoered\n", link_type[k]);
						return false;
					}
					if (lane_section == 0)
					{
						LOG("Error lane section == 0\n");
						return false;
					}
					Lane *lane = lane_section->GetLaneById(lane1_id);
					(void)lane;
					if (!(lane_section->GetConnectingLaneId(lane1_id, link_type[k]) == lane2_id))
					{
						return false;
					}
					// Now, check other end lane connectivitiy
				}
				return true;
			}
		}
		// check whether the roads are connected via a junction connecting road and specified lane
		else if (link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
		{
			Junction* junction = GetJunctionById(link->GetElementId());

			for (int i = 0; i < junction->GetNumberOfConnections(); i++)
			{
				Connection* connection = junction->GetConnectionByIdx(i);

				if (connection->GetIncomingRoad()->GetId() == road1_id)
				{
					Road* connecting_road = connection->GetConnectingRoad();
					RoadLink* exit_link = 0;

					// Found a connecting road - first check if this is the second road
					if (connecting_road->GetId() == road2_id)
					{
						// Check lanes
						for (int j = 0; j < connection->GetNumberOfLaneLinks(); j++)
						{
							if (connection->GetLaneLink(j)->from_ == lane1_id &&
								connection->GetLaneLink(j)->to_ == lane2_id)
							{
								return true;
							}
						}
					}

					// Then check if it connects to second road
					if (connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START)
					{
						exit_link = connecting_road->GetLink(SUCCESSOR);
					}
					else
					{
						exit_link = connecting_road->GetLink(PREDECESSOR);
					}

					if (exit_link->GetElementId() == road2_id)
					{
						// Finally check that lanes are connected through the junction
						// Look at lane section and locate lane connecting both roads
						// Assume connecting road has only one lane section
						lane_section = connecting_road->GetLaneSectionByIdx(0);
						if (lane_section == 0)
						{
							LOG("Error lane section == 0\n");
							return false;
						}
						for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
						{
							Lane* lane = lane_section->GetLaneByIdx(j);
							LaneLink* lane_link_predecessor = lane->GetLink(PREDECESSOR);
							LaneLink* lane_link_successor = lane->GetLink(SUCCESSOR);
							if (lane_link_predecessor == 0 || lane_link_successor == 0)
							{
								continue;
							}
							if ((connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START &&
								lane_link_predecessor->GetId() == lane1_id &&
								lane_link_successor->GetId() == lane2_id) ||
								(connection->GetContactPoint() == ContactPointType::CONTACT_POINT_END &&
									lane_link_predecessor->GetId() == lane2_id &&
									lane_link_successor->GetId() == lane1_id))
							{
								// Found link
								if (connecting_road_id != 0)
								{
									*connecting_road_id = connection->GetConnectingRoad()->GetId();
								}
								if (connecting_lane_id != 0)
								{
									*connecting_lane_id = lane->GetId();
								}
								return true;
							}
						}
					}
				}
			}
		}
		else
		{
			LOG("Error: LinkElementType %d unsupported\n", link->GetElementType());
		}
	}

	LOG("Link not found");

	return false;
}

int OpenDrive::CheckConnectedRoad(Road *road, RoadLink *link, ContactPointType expected_contact_point_type, RoadLink *link2)
{
	if (link2 == 0)
	{
		return -1;
	}

	if (link2->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		if (link->GetElementId() == road->GetId())
		{
			if (link->GetContactPointType() != expected_contact_point_type)
			{
				LOG("Found connecting road from other end, but contact point is wrong (expected START, got %s)",
					ContactPointType2Str(link->GetContactPointType()).c_str());
				return -1;
			}
		}
	}

	return 0;
}

int OpenDrive::CheckJunctionConnection(Junction *junction, Connection *connection)
{
	if (junction == 0)
	{
		return -1;
	}

	// Check if junction is referred to from the connected road
	Road *road = connection->GetConnectingRoad();
	if (road == 0)
	{
		LOG("Error no connecting road");
		return -1;
	}

	RoadLink *link[2];
	link[0] = road->GetLink(LinkType::PREDECESSOR);
	link[1] = road->GetLink(LinkType::SUCCESSOR);
	for (int i = 0; i < 2; i++)
	{
		if (link[i] != 0)
		{
			if (junction->GetType() == Junction::JunctionType::DIRECT)
			{
				if ((i == 0 && // link points to predecessor
					 connection->GetContactPoint() == ContactPointType::CONTACT_POINT_START &&
					 link[i]->GetContactPointType() == ContactPointType::CONTACT_POINT_JUNCTION &&
					 link[i]->GetElementId() != junction->GetId()) ||
					(i == 1 && // link points to successor
					 connection->GetContactPoint() == ContactPointType::CONTACT_POINT_END &&
					 link[i]->GetContactPointType() == ContactPointType::CONTACT_POINT_JUNCTION &&
					 link[i]->GetElementId() != junction->GetId()))
				{
					LOG("Expected direct junction linkedRoad to connect back to junction id %d, found id %d", link[i]->GetElementId());
					return -1;
				}

				// Create counter connections, treating linkedRoad as incomingRoad
				// Find out contact point for original incoming road
				ContactPointType new_contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
				if (connection->GetIncomingRoad()->GetLink(LinkType::SUCCESSOR) &&
					connection->GetIncomingRoad()->GetLink(LinkType::SUCCESSOR)->GetElementId() == junction->GetId())
				{
					new_contact_point = ContactPointType::CONTACT_POINT_END;
				}
				else if (connection->GetIncomingRoad()->GetLink(LinkType::PREDECESSOR) &&
					connection->GetIncomingRoad()->GetLink(LinkType::PREDECESSOR)->GetElementId() == junction->GetId())
				{
					new_contact_point = ContactPointType::CONTACT_POINT_START;
				}
				else
				{
					LOG("Failed to find out contactpoint of direct junction incoming road");
					return -1;
				}

				// Check that it does not already exist
				Connection* new_connection = 0;
				for (size_t k = 0; k < junction->GetNumberOfConnections(); k++)
				{
					if (junction->GetConnectionByIdx((int)k)->GetIncomingRoad() == connection->GetConnectingRoad() &&
						junction->GetConnectionByIdx((int)k)->GetConnectingRoad() == road)
					{
						new_connection = junction->GetConnectionByIdx((int)k);
						break;
					}
				}
				if (!new_connection)
				{
					new_connection = new Connection(connection->GetConnectingRoad(), connection->GetIncomingRoad(), new_contact_point);
					for (size_t j = 0; j < connection->GetNumberOfLaneLinks(); j++)
					{
						JunctionLaneLink* tmp_link = connection->GetLaneLink((int)j);
						new_connection->AddJunctionLaneLink(tmp_link->to_, tmp_link->from_);
					}
					junction->AddConnection(new_connection);
				}
			}
			else
			{
				if (link[i]->GetElementType() != RoadLink::ElementType::ELEMENT_TYPE_ROAD)
				{
					LOG("Expected element type ROAD, found %s", ElementType2Str(link[i]->GetElementType()).c_str());
					return -1;
				}

				if (link[i]->GetElementId() != connection->GetIncomingRoad()->GetId())
				{
					// Check connection from this outgoing road
					Road* roadc = GetRoadById(link[i]->GetElementId());
					if (roadc == nullptr)
					{
						LOG("Failed to resolve road %d in junction %d", link[i]->GetElementId(), junction->GetId());
						return 0;
					}
					RoadLink* link2[2];
					link2[0] = roadc->GetLink(LinkType::PREDECESSOR);
					link2[1] = roadc->GetLink(LinkType::SUCCESSOR);
					for (int j = 0; j < 2; j++)
					{
						if (link2[j] != 0)
						{
							if (link2[j]->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION &&
								link2[j]->GetElementId() == junction->GetId())
							{
								// Now finally find the reverse link
								for (int k = 0; k < junction->GetNumberOfConnections(); k++)
								{
									if (junction->GetConnectionByIdx(k)->GetIncomingRoad() == roadc)
									{
										// Sharing same connecting road?
										if (junction->GetConnectionByIdx(k)->GetConnectingRoad() == connection->GetConnectingRoad())
										{
											return 0;
										}
									}
								}

								// Create counter connections on other side of connecting road
								LinkType newLinkType = (i == 0 ? LinkType::PREDECESSOR : LinkType::SUCCESSOR);
								RoadLink* newLink = connection->GetConnectingRoad()->GetLink(newLinkType);
								if (newLink && newLink->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
								{
									ContactPointType new_contact_point =
										connection->GetContactPoint() == ContactPointType::CONTACT_POINT_END ?
										ContactPointType::CONTACT_POINT_START : ContactPointType::CONTACT_POINT_END;

									// Create new connection to the connecting road from other side
									Connection* newConnection = new Connection(GetRoadById(newLink->GetElementId()), connection->GetConnectingRoad(), new_contact_point);

									// Add lane links - assume only one lane section in the connecting road
									LaneSection* ls = newConnection->GetConnectingRoad()->GetLaneSectionByIdx(0);
									for (int l = 0; l < ls->GetNumberOfLanes(); l++)
									{
										Lane* lane = ls->GetLaneByIdx(l);
										if (lane->GetLink(newLinkType))
										{
											int from_id = lane->GetId();
											int to_id = lane->GetLink(newLinkType)->GetId();
											newConnection->AddJunctionLaneLink(to_id, from_id);
										}
									}
									if (newConnection->GetNumberOfLaneLinks() > 0)
									{
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

int OpenDrive::CheckLink(Road *road, RoadLink *link, ContactPointType expected_contact_point_type)
{
	// does this connection exist in the other direction?
	if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		Road *connecting_road = GetRoadById(link->GetElementId());
		if (connecting_road != 0)
		{
			if (CheckConnectedRoad(road, link, expected_contact_point_type, connecting_road->GetLink(LinkType::PREDECESSOR)) == 0)
			{
				return 0;
			}
			else if (CheckConnectedRoad(road, link, expected_contact_point_type, connecting_road->GetLink(LinkType::SUCCESSOR)) == 0)
			{
				return 0;
			}
			else
			{
				LOG("Warning: Reversed road link %d->%d not found. Might be a flaw in the OpenDRIVE description.", road->GetId(), connecting_road->GetId());
			}
		}
	}
	else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		Junction *junction = GetJunctionById(link->GetElementId());

		// Check all outgoing connections
		if (junction == nullptr)
		{
			LOG("Info: Junction id %d, referred to by road %d, does not exist", link->GetElementId(), road->GetId());
			return -1;
		}

		int nrConnections = junction->GetNumberOfConnections();
		for (int i = 0; i < nrConnections; i++)
		{
			Connection *connection = junction->GetConnectionByIdx((int)i);

			if (connection->GetIncomingRoad() == road)
			{
				CheckJunctionConnection(junction, connection);
			}
		}
	}

	return 0;
}

int OpenDrive::CheckConnections()
{
	int counter = 0;
	RoadLink *link;

	for (size_t i = 0; i < road_.size(); i++)
	{
		// Check for connections
		if ((link = road_[i]->GetLink(LinkType::PREDECESSOR)) != 0)
		{
			CheckLink(road_[i], link, ContactPointType::CONTACT_POINT_START);
		}
		if ((link = road_[i]->GetLink(LinkType::SUCCESSOR)) != 0)
		{
			CheckLink(road_[i], link, ContactPointType::CONTACT_POINT_END);
		}
	}

	return counter;
}

void OpenDrive::Print()
{
	LOG("Roads:\n");
	for (size_t i=0; i<road_.size(); i++)
	{
		road_[i]->Print();
	}

	LOG("junctions\n");
	for (size_t i=0; i<junction_.size(); i++)
	{
		junction_[i]->Print();
	}
}

GeoReference* OpenDrive::GetGeoReference()
{
	return &geo_ref_;
}

std::string OpenDrive::GetGeoReferenceAsString()
{
	std::ostringstream out;
    if(!std::isnan(geo_ref_.lat_0_) && !std::isnan(geo_ref_.lon_0_))
	{
		out.precision(13);
		out << "+proj=" << geo_ref_.proj_ << " +lat_0=" << std::fixed << geo_ref_.lat_0_ << " +lon_0=" << std::fixed << geo_ref_.lon_0_;
	}
    return out.str();
}

void OpenDrive::ParseGeoLocalization(const std::string& geoLocalization)
{
	std::map<std::string, std::string> attributes;
	char space_char = ' ';
	char asignment_char = '=';

	std::stringstream sstream(geoLocalization);
	std::string attribute = "";
	//Get each attribute of geoReference
	while (std::getline(sstream, attribute, space_char))
	{
		std::stringstream sstream_attrib(attribute);
		std::string key_value = "";
		std::string attribute_key = "";
		std::string attribute_value = "";
		//Get key and value of each attribute
		while (std::getline(sstream_attrib, key_value, asignment_char))
		{
			//Keys starts with a + character
			if(key_value.rfind('+', 0) == 0)
			{
				attribute_key = key_value;
			}else
			{
				attribute_value = key_value;
			}
		}
		attributes.emplace(attribute_key, attribute_value);
	}

	for(const auto& attr : attributes)
	{
		if(attr.first == "+a")
		{
			geo_ref_.a_ = std::stod(attr.second);
		}
		else if (attr.first == "+axis")
		{
			geo_ref_.axis_ = std::stod(attr.second);
		}
		else if (attr.first == "+b")
		{
			geo_ref_.b_ = std::stod(attr.second);
		}
		else if (attr.first == "+ellps")
		{
			geo_ref_.ellps_ = attr.second;
		}
		else if (attr.first == "+k")
		{
			geo_ref_.k_ = std::stod(attr.second);
		}
		else if (attr.first == "+k_0")
		{
			geo_ref_.k_0_ = std::stod(attr.second);
		}
		else if (attr.first == "+lat_0")
		{
			geo_ref_.lat_0_ = std::stod(attr.second);
		}
		else if (attr.first == "+lon_0")
		{
			geo_ref_.lon_0_ = std::stod(attr.second);
		}
		else if (attr.first == "+lon_wrap")
		{
			geo_ref_.lon_wrap_ = std::stod(attr.second);
		}
		else if (attr.first == "+over")
		{
			geo_ref_.over_ = std::stod(attr.second);
		}
		else if (attr.first == "+pm")
		{
			geo_ref_.pm_ = attr.second;
		}
		else if (attr.first == "+proj")
		{
			geo_ref_.proj_ = attr.second;
		}
		else if (attr.first == "+units")
		{
			geo_ref_.units_ = attr.second;
		}
		else if (attr.first == "+vunits")
		{
			geo_ref_.vunits_ = attr.second;
		}
		else if (attr.first == "+x_0")
		{
			geo_ref_.x_0_ = std::stod(attr.second);
		}
		else if (attr.first == "+y_0")
		{
			geo_ref_.y_0_ = std::stod(attr.second);
		}
		else if (attr.first == "+datum")
		{
			geo_ref_.datum_ = attr.second;
		}
		else if (attr.first == "+geoidgrids")
		{
			geo_ref_.geo_id_grids_ = attr.second;
		}
		else if (attr.first == "+zone")
		{
			geo_ref_.zone_ = std::stod(attr.second);
		}
		else if (attr.first == "+towgs84")
		{
			geo_ref_.towgs84_ = std::stoi(attr.second);
		}
		else
		{
			LOG("Unsupported geo reference attr: %s", attr.first.c_str());
		}
	}

	if (std::isnan(geo_ref_.lat_0_) || std::isnan(geo_ref_.lon_0_))
	{
		LOG("cannot parse georeference: '%s'. Using default values.", geoLocalization.c_str());
		geo_ref_.lat_0_ = 0.0;
		geo_ref_.lon_0_ = 0.0;
	}
}

bool OpenDrive::LoadSignalsByCountry(const std::string& country)
{
	std::string sign_filename = country + "_traffic_signals.txt";
	std::vector<std::string> file_name_candidates;

	// absolute path or relative to current directory
	file_name_candidates.push_back(sign_filename);
	// Remove all directories from path and look in current directory
	file_name_candidates.push_back(FileNameOf(sign_filename));
	// assume OpenDRIVE file directory is on same level as traffic_signals directory
	file_name_candidates.push_back(DirNameOf(odr_filename_) + "/../../traffic_signals/" + sign_filename);

	for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
	{
		// Also check registered paths
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], sign_filename));
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], "traffic_signals/" + sign_filename));
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], "resources/traffic_signals/" + sign_filename));
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], "/../traffic_signals/" + sign_filename));
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], "/../resources/traffic_signals/" + sign_filename));
	}

	size_t i;
	bool located = false;
	for (i = 0; i < file_name_candidates.size(); i++)
	{
		if (FileExists(file_name_candidates[i].c_str()))
		{
			located = true;
			std::string line;
			// assuming the file is text
			std::ifstream fs;
			fs.open(file_name_candidates[i].c_str());

			if(fs.fail())
			{
				LOG("Signal: Error to load traffic signals file - %s", file_name_candidates[i].c_str());
				if (i < file_name_candidates.size() - 1)
				{
					LOG("  -> trying: %s", file_name_candidates[i + 1].c_str());
				}
			}
			else
			{
				const char delimiter = '=';

				// process each line in turn
				while(std::getline(fs, line))
				{
					std::stringstream sstream(country+line);
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

	if (i == file_name_candidates.size())
	{
		LOG("Failed to load %s file. Tried:", sign_filename.c_str());
		for (int j = 0; j < file_name_candidates.size(); j++)
		{
			LOG("  %s", file_name_candidates[j].c_str());
		}
		return false;
	}

	return true;
}

void Position::Init()
{
	track_id_ = -1;
	lane_id_ = 0;
	s_ = 0.0;
	s_trajectory_ = 0.0;
	t_trajectory_ = 0.0;
	t_ = 0.0;
	offset_ = 0.0;
	x_ = 0.0;
	y_ = 0.0;
	z_ = 0.0;
	h_ = 0.0;
	p_ = 0.0;
	r_ = 0.0;
	velX_ = 0.0;
	velY_ = 0.0;
	velZ_ = 0.0;
	accX_ = 0.0;
	accY_ = 0.0;
	accZ_ = 0.0;
	h_rate_ = 0.0;
	p_rate_ = 0.0;
	r_rate_ = 0.0;
	h_acc_ = 0.0;
	p_acc_ = 0.0;
	r_acc_ = 0.0;
	h_offset_ = 0.0;
	h_road_ = 0.0;
	h_relative_ = 0.0;
	z_relative_ = 0.0;
	curvature_ = 0.0;
	p_road_ = 0.0;
	p_relative_ = 0.0;
	r_road_ = 0.0;
	r_relative_ = 0.0;
	roadSuperElevationPrim_ = 0.0;
	z_roadPrimPrim_ = 0.0;
	z_roadPrim_ = 0.0;
	rel_pos_ = 0;
	align_h_ = ALIGN_MODE::ALIGN_SOFT;
	align_p_ = ALIGN_MODE::ALIGN_SOFT;
	align_r_ = ALIGN_MODE::ALIGN_SOFT;
	align_z_ = ALIGN_MODE::ALIGN_SOFT;
	type_ = PositionType::NORMAL;
	orientation_type_ = OrientationType::ORIENTATION_ABSOLUTE;
	snapToLaneTypes_ = Lane::LaneType::LANE_TYPE_ANY_DRIVING;
	status_ = 0;
	lockOnLane_ = false;

	z_road_ = 0.0;
	track_idx_ = -1;
	geometry_idx_ = -1;
	lane_section_idx_ = -1;
	lane_idx_ = -1;
	elevation_idx_ = -1;
	super_elevation_idx_ = -1;
	osi_point_idx_ = -1;
	route_ = 0;
	trajectory_ = 0;

	// Assume all values are defined. Set resp. mask value explicitly to zero in order to make it undefined.
	orientationSetMask = 3;
}

Position::Position()
{
	Init();
}

Position::Position(int track_id, double s, double t)
{
	Init();
	SetTrackPos(track_id, s, t);
}

Position::Position(int track_id, int lane_id, double s, double offset)
{
	Init();
	SetLanePos(track_id, lane_id, s, offset);
}

Position::Position(double x, double y, double z, double h, double p, double r)
{
	Init();
	SetInertiaPos(x, y, z, h, p, r);
}

Position::Position(double x, double y, double z, double h, double p, double r, bool calculateTrackPosition)
{
	Init();
	SetInertiaPos(x, y, z, h, p, r, calculateTrackPosition);
}

Position::~Position()
{

}

bool Position::LoadOpenDrive(const char *filename)
{
	return(GetOpenDrive()->LoadOpenDriveFile(filename));
}

bool Position::LoadOpenDrive(OpenDrive* odr)
{
	*GetOpenDrive() = *odr;
	return(GetOpenDrive());
}


OpenDrive* Position::GetOpenDrive()
{
	static OpenDrive od;
	return &od;
}

bool OpenDrive::CheckLaneOSIRequirement(std::vector<double> x0, std::vector<double> y0, std::vector<double> x1, std::vector<double> y1)
{
	double x0_tan_diff, y0_tan_diff, x1_tan_diff, y1_tan_diff;
	x0_tan_diff = x0[2]-x0[0];
	y0_tan_diff = y0[2]-y0[0];
	x1_tan_diff = x1[2]-x1[0];
	y1_tan_diff = y1[2]-y1[0];

	// Avoiding Zero Denominator in OSI point calculations
	if(x0_tan_diff == 0) {x0_tan_diff+=0.001; }

	if(y0_tan_diff == 0) {y0_tan_diff+=0.001; }

	if(x1_tan_diff == 0) {x1_tan_diff+=0.001; }

	if(y1_tan_diff == 0) {y1_tan_diff+=0.001; }

	// Creating tangent line around the point (First Point) with given tolerance
	double k_0 = y0_tan_diff/x0_tan_diff;
	double m_0 = y0[1] - k_0*x0[1];

	// Creating tangent line around the point (Second Point) with given tolerance
	double k_1 = y1_tan_diff/x1_tan_diff;
	double m_1 = y1[1] - k_1*x1[1];

	// Intersection point of the tangent lines
	double intersect_tangent_x = (m_0 - m_1) / (k_1 - k_0);
	double intersect_tangent_y = k_0*intersect_tangent_x + m_0;

	// Creating real line between the First Point and Second Point
	double k = (y1[1] - y0[1]) / (x1[1] - x0[1]);
	double m = y0[1] - k*x0[1];

	// The maximum distance can be found between the real line and a tangent line: passing through [u_intersect, y_intersect] with slope "k"
	// The perpendicular line to the tangent line can be formulated as f(Q) = intersect_tangent_y + (intersect_tangent_x / k) - Q/k
	// Then the point on the real line which gives maximum distance -> f(Q) = k*Q + m
	double intersect_x = (intersect_tangent_y + (intersect_tangent_x/k) - m) / (k + 1/k);
	double intersect_y = k*intersect_x + m;
	double max_distance = sqrt(pow(intersect_y-intersect_tangent_y,2) + pow(intersect_x-intersect_tangent_x,2));

	// Max distance can be "nan" when the lane is perfectly straigt and hence k = 0.
	// In this case, it satisfies OSI_LANE_CALC_REQUIREMENT since it is a perfect line
	if (max_distance < SE_Env::Inst().GetOSIMaxLateralDeviation() || std::isnan(max_distance))
	{
		return true;
	}
	else
	{
		return false;
	}
}

static double GetMaxSegmentLen(Position* pivot, Position* pos, double min, double max, double pitchResScale, double rollResScale, bool& osi_requirement)
{
	double max_segment_length;

	// Consider rate of change of pitch and roll for segment length to influence
	// the tesselation (triangulation) of road surface model

	double zRoadPrimPrim = pos->GetZRoadPrimPrim();
	double roadSuperElevationPrim = pos->GetRoadSuperElevationPrim();
	double max_segment_length_candidate1 = pitchResScale / MAX(SMALL_NUMBER, abs(zRoadPrimPrim));
	double max_segment_length_candidate2 = rollResScale / MAX(SMALL_NUMBER, abs(roadSuperElevationPrim));

	max_segment_length = MIN(max_segment_length_candidate1, max_segment_length_candidate2);

	// Adjust for slope
	max_segment_length = max_segment_length / sqrt(pow(pos->GetZRoadPrim(), 2) + 1);

	max_segment_length = MAX(min, MIN(max, max_segment_length));

	if (pivot)
	{
		// check slope and delta z for discontinuities
		if (
			// radical change of slope
			abs(pos->GetZRoadPrim() - pivot->GetZRoadPrim()) > 0.1 - SMALL_NUMBER ||
			// change of z not according to slope (discontinuity found)
			abs((pos->GetZ() - pivot->GetZ()) - pivot->GetZRoadPrim() * (pos->GetS() - pivot->GetS())) > 0.1 - SMALL_NUMBER) // actual
		{
			max_segment_length = min;
			osi_requirement = false;
		}
	}

	return max_segment_length;
}

void OpenDrive::SetLaneOSIPoints()
{
	// Initialization
	Position pos_pivot, pos_tmp, pos_candidate;
	Road *road;
	LaneSection *lsec;
	Lane *lane;
	int number_of_lane_sections, number_of_lanes;
	double lsec_end;
	std::vector<PointStruct> osi_point;
	std::vector<double> x0, y0, x1, y1;
	bool osi_requirement;
	double max_segment_length = SE_Env::Inst().GetOSIMaxLongitudinalDistance();
	double min_segment_length = 0.2;
	int osiintersection;

	// Looping through each road
	for (int i=0; i<road_.size(); i++)
	{
		road = road_[i];

		if (road->GetJunction() == -1)
		{
			osiintersection = -1;
		}
		else
		{
			Junction* junction = GetJunctionById(road->GetJunction());
			if (junction && GetJunctionById(junction->IsOsiIntersection()))
			{
				osiintersection = GetJunctionById(road->GetJunction())->GetGlobalId();
			}
			else
			{
				osiintersection = -1;
			}
		}

		// Looping through each lane section
		number_of_lane_sections = road_[i]->GetNumberOfLaneSections();
		for (int j=0; j<number_of_lane_sections; j++)
		{
			// Get the ending position of the current lane section
			lsec = road->GetLaneSectionByIdx(j);
			if (j == number_of_lane_sections-1)
			{
				lsec_end = road->GetLength();
			}
			else
			{
				lsec_end = road->GetLaneSectionByIdx(j+1)->GetS();
			}

			// Looping through each lane
			number_of_lanes = lsec->GetNumberOfLanes();
			for (int k=0; k<number_of_lanes; k++)
			{
				lane = lsec->GetLaneByIdx(k);
				int counter = 0;

				// [XO, YO] = Real position with no tolerance
				if (pos_pivot.SetLanePos(road->GetId(), lane->GetId(), lsec->GetS(), 0, j) != Position::ReturnCode::OK)
				{
					break;
				}

				// Add the starting point of each lane as osi point
				PointStruct p = { lsec->GetS(), pos_pivot.GetX(), pos_pivot.GetY(), pos_pivot.GetZ(), pos_pivot.GetHRoad() };
				osi_point.push_back(p);

				// [XO, YO] = closest position with given (-) tolerance
				pos_tmp.SetLanePos(road->GetId(), lane->GetId(), MAX(0, lsec->GetS() - OSI_TANGENT_LINE_TOLERANCE), 0, j);
				x0.push_back(pos_tmp.GetX());
				y0.push_back(pos_tmp.GetY());

				// Push real position between the +/- tolerance points
				x0.push_back(pos_pivot.GetX());
				y0.push_back(pos_pivot.GetY());

				// [XO, YO] = closest position with given (+) tolerance
				pos_tmp.SetLanePos(road->GetId(), lane->GetId(), MIN(lsec->GetS() + OSI_TANGENT_LINE_TOLERANCE, lsec_end), 0, j);
				x0.push_back(pos_tmp.GetX());
				y0.push_back(pos_tmp.GetY());

				bool insert = false;
				double step = OSI_POINT_CALC_STEPSIZE;

				pos_candidate = pos_pivot;

				// Looping through sequential points along the track determined by "OSI_POINT_CALC_STEPSIZE"
				while(++counter)
				{
					// Make sure we stay within lane section length
					double s = MIN(pos_candidate.GetS() + step, lsec_end - SMALL_NUMBER/2);

					// [X1, Y1] = Real position with no tolerance
					pos_candidate.SetLanePos(road->GetId(), lane->GetId(), s, 0, j);

					// [X1, Y1] = closest position with given (-) tolerance
					pos_tmp.SetLanePos(road->GetId(), lane->GetId(), MAX(s - OSI_TANGENT_LINE_TOLERANCE, 0), 0, j);
					x1.push_back(pos_tmp.GetX());
					y1.push_back(pos_tmp.GetY());

					x1.push_back(pos_candidate.GetX());
					y1.push_back(pos_candidate.GetY());

					// [X1, Y1] = closest position with given (+) tolerance
					pos_tmp.SetLanePos(road->GetId(), lane->GetId(), MIN(s + OSI_TANGENT_LINE_TOLERANCE, lsec_end), 0, j);
					x1.push_back(pos_tmp.GetX());
					y1.push_back(pos_tmp.GetY());

					// Check OSI Requirement between current given points
					if (NEAR_NUMBERS(pos_pivot.GetH(), pos_candidate.GetH()))
					{
						if (DistanceFromPointToLine2DWithAngle(pos_candidate.GetX(), pos_candidate.GetY(),
							pos_pivot.GetX(), pos_pivot.GetY(), pos_pivot.GetH()) < min_segment_length)
						{
							osi_requirement = true;  // points on a straight segment
						}
						else
						{
							osi_requirement = false;  // same heading but not on a straight line => lane discontinuity
						}
					}
					else
					{
						osi_requirement = CheckLaneOSIRequirement(x0, y0, x1, y1);
					}

					// If requirement is satisfied -> look further points
					// If requirement is not satisfied:
					//    Assign last unique satisfied point as OSI point
					//    Continue searching from the last satisfied point

					// Make sure max segment length is longer than stepsize and considering elevation change rate
					if (osi_requirement)
					{
						max_segment_length = GetMaxSegmentLen(&pos_pivot, &pos_candidate, 1.1 * OSI_POINT_CALC_STEPSIZE, SE_Env::Inst().GetOSIMaxLongitudinalDistance(),
							OSI_POINT_DIST_SCALE, OSI_POINT_DIST_SCALE, osi_requirement);
					}

					if (pos_candidate.GetS() + SMALL_NUMBER > lsec_end - SMALL_NUMBER ||   // end of the lane reached, assign as final OSI point
						osi_requirement && pos_candidate.GetS() - pos_pivot.GetS() > max_segment_length - SMALL_NUMBER ||
						abs(step) < min_segment_length + SMALL_NUMBER)
					{
						p = { pos_candidate.GetS(), pos_candidate.GetX(), pos_candidate.GetY(), pos_candidate.GetZ(), pos_candidate.GetHRoad() };
						osi_point.push_back(p);
						insert = false;

						if (pos_candidate.GetS() + SMALL_NUMBER > lsec_end - SMALL_NUMBER)
						{
							break;
						}

						// If last step length was small, guess next one will also be small to reduce search
						step = MIN(OSI_POINT_CALC_STEPSIZE, 2.0 * (pos_candidate.GetS() - pos_pivot.GetS()));

						pos_pivot = pos_candidate;

						// reuse candidate x-y collectors for pivot position
						x0 = x1;
						y0 = y1;
					}
					else
					{
						if (osi_requirement == false)
						{
							insert = true;  // indicate that a point needs to be inserted
							step = -abs(step) / 2.0;  // look backwards half current stepsize
						}
						else if (insert)
						{
							step = abs(step) / 2.0;  // look forward half current stepsize
						}
					}

					// Clear x-y collectors for next iteration
					x1.clear();
					y1.clear();
				}

				// Set all collected osi points for the current lane
				lane->osi_points_.Set(osi_point);
				lane->SetOSIIntersection(osiintersection);

				// Clear osi collectors for next iteration
				osi_point.clear();
			}
		}
	}
}

void OpenDrive::SetLaneBoundaryPoints()
{
	// Initialization
	Position pos;
	Road *road;
	LaneSection *lsec;
	Lane *lane;
	int number_of_lane_sections, number_of_lanes;
	double lsec_end;
	std::vector<double> x0, y0, x1, y1;
	std::vector<PointStruct> osi_point;
	double s0, s1, s1_prev;
	bool osi_requirement;
	double max_segment_length = SE_Env::Inst().GetOSIMaxLongitudinalDistance();

	// Looping through each road
	for (int i=0; i<road_.size(); i++)
	{
		road = road_[i];

		// Looping through each lane section
		number_of_lane_sections = road_[i]->GetNumberOfLaneSections();
		for (int j=0; j<number_of_lane_sections; j++)
		{
			// Get the ending position of the current lane section
			lsec = road->GetLaneSectionByIdx(j);
			if (j == number_of_lane_sections-1)
			{
				lsec_end = road->GetLength();
			}
			else
			{
				lsec_end = road->GetLaneSectionByIdx(j+1)->GetS();
			}

			// Starting points of the each lane section for OSI calculations
			s0 = lsec->GetS();
			s1 = s0 + OSI_POINT_CALC_STEPSIZE;
			s1_prev = s0;

			// Looping through each lane
			number_of_lanes = lsec->GetNumberOfLanes();
			for (int k=0; k<number_of_lanes; k++)
			{
				lane = lsec->GetLaneByIdx(k);
				int counter = 0;
				int n_roadmarks = lane->GetNumberOfRoadMarks();

				if (n_roadmarks == 0)
				{
					// Looping through sequential points along the track determined by "OSI_POINT_CALC_STEPSIZE"
					while(true)
					{
						counter++;

						// Make sure we stay within lane section length
						s1 = MIN(s1, lsec_end - OSI_TANGENT_LINE_TOLERANCE);

						// [XO, YO] = closest position with given (-) tolerance
						pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), MAX(0, s0-OSI_TANGENT_LINE_TOLERANCE), 0, j);
						x0.push_back(pos.GetX());
						y0.push_back(pos.GetY());

						// [XO, YO] = Real position with no tolerance
						pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), s0, 0, j);
						x0.push_back(pos.GetX());
						y0.push_back(pos.GetY());

						// Add the starting point of each lane as osi point
						if (counter == 1)
						{
							PointStruct p = { s0, pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetHRoad() };
							osi_point.push_back(p);
						}

						// [XO, YO] = closest position with given (+) tolerance
						pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), s0+OSI_TANGENT_LINE_TOLERANCE, 0, j);
						x0.push_back(pos.GetX());
						y0.push_back(pos.GetY());

						// [X1, Y1] = closest position with given (-) tolerance
						pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), s1-OSI_TANGENT_LINE_TOLERANCE, 0, j);
						x1.push_back(pos.GetX());
						y1.push_back(pos.GetY());

						// [X1, Y1] = Real position with no tolerance
						pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), s1, 0, j);
						x1.push_back(pos.GetX());
						y1.push_back(pos.GetY());

						// [X1, Y1] = closest position with given (+) tolerance
						pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), s1+OSI_TANGENT_LINE_TOLERANCE, 0, j);
						x1.push_back(pos.GetX());
						y1.push_back(pos.GetY());

						// Check OSI Requirement between current given points
						if (x1[1]-x0[1] != 0 && y1[1]-y0[1] != 0)
						{
							osi_requirement = CheckLaneOSIRequirement(x0, y0, x1, y1);
						}
						else
						{
							osi_requirement = true;
						}

						// Make sure max segment length is longer than stepsize
						if (osi_requirement)
						{
							max_segment_length = GetMaxSegmentLen(0, &pos, 1.1 * OSI_POINT_CALC_STEPSIZE, SE_Env::Inst().GetOSIMaxLongitudinalDistance(),
								OSI_POINT_DIST_SCALE, OSI_POINT_DIST_SCALE, osi_requirement);
						}

						// If requirement is satisfied -> look further points
						// If requirement is not satisfied:
						// Assign last satisfied point as OSI point
						// Continue searching from the last satisfied point
						if (osi_requirement && s1 - s0 < max_segment_length)
						{
							s1_prev = s1;
							s1 = s1 + OSI_POINT_CALC_STEPSIZE;

						}
						else
						{
							s0 = s1_prev;
							s1_prev = s1;
							s1 = s0 + OSI_POINT_CALC_STEPSIZE;

							if (counter != 1)
							{
								pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), s0, 0, j);
								PointStruct p = { s0, pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetHRoad() };
								osi_point.push_back(p);
							}
						}

						// If the end of the lane reached, assign end of the lane as final OSI point for current lane
						if (s1 + OSI_TANGENT_LINE_TOLERANCE >= lsec_end)
						{
							pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), MAX(0, lsec_end - SMALL_NUMBER), 0, j);
							PointStruct p = { lsec_end, pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetHRoad() };
							osi_point.push_back(p);
							break;
						}

						// Clear x-y collectors for next iteration
						x0.clear();
						y0.clear();
						x1.clear();
						y1.clear();
					}
					// Initialization of LaneBoundary class
					LaneBoundaryOSI * lb = new LaneBoundaryOSI((int)0);
					// add the lane boundary class to the lane class and generating the global id
					lane->SetLaneBoundary(lb);
					//Fills up the osi points in the lane boundary class
					lb->osi_points_.Set(osi_point);
					// Clear osi collectors for next iteration
					osi_point.clear();

					// Re-assign the starting point of the next lane as the start point of the current lane section for OSI calculations
					s0 = lsec->GetS();
					s1 = s0+OSI_POINT_CALC_STEPSIZE;
					s1_prev = s0;
				}
			}
		}
	}
}

void OpenDrive::SetRoadMarkOSIPoints()
{
	// Initialization
	Position pos_pivot, pos_tmp, pos_candidate;
	Road *road;
	LaneSection *lsec;
	Lane *lane;
	LaneRoadMark *lane_roadMark;
	LaneRoadMarkType *lane_roadMarkType;
	LaneRoadMarkTypeLine *lane_roadMarkTypeLine;
	int number_of_lane_sections, number_of_lanes, number_of_roadmarks, number_of_roadmarktypes, number_of_roadmarklines;
	double lsec_end, s_roadmark, s_end_roadmark, s_roadmarkline, s_end_roadmarkline;
	std::vector<double> x0, y0, x1, y1;
	std::vector<PointStruct> osi_point;
	bool osi_requirement;
	double max_segment_length = SE_Env::Inst().GetOSIMaxLongitudinalDistance();
	double min_segment_length = 0.2;

	// Looping through each road
	for (int i=0; i<road_.size(); i++)
	{
		road = road_[i];

		// Looping through each lane section
		number_of_lane_sections = road_[i]->GetNumberOfLaneSections();
		for (int j=0; j<number_of_lane_sections; j++)
		{
			// Get the ending position of the current lane section
			lsec = road->GetLaneSectionByIdx(j);
			if (j == number_of_lane_sections-1)
			{
				lsec_end = road->GetLength();
			}
			else
			{
				lsec_end = road->GetLaneSectionByIdx(j+1)->GetS();
			}

			// Looping through each lane
			number_of_lanes = lsec->GetNumberOfLanes();
			for (int k=0; k<number_of_lanes; k++)
			{
				lane = lsec->GetLaneByIdx(k);

				// Looping through each roadMark within the lane
				number_of_roadmarks = lane->GetNumberOfRoadMarks();
				if (number_of_roadmarks != 0)
				{

					for (int m=0; m<number_of_roadmarks; m++)
					{
						lane_roadMark = lane->GetLaneRoadMarkByIdx(m);
						s_roadmark = lsec->GetS() + lane_roadMark->GetSOffset();
						if (m == number_of_roadmarks-1)
						{
							s_end_roadmark = MAX(0, lsec_end - SMALL_NUMBER);
						}
						else
						{
							s_end_roadmark = MAX(0, lsec->GetS() + lane->GetLaneRoadMarkByIdx(m+1)->GetSOffset() - SMALL_NUMBER);
						}

						// Check the existence of "type" keyword under roadmark
						number_of_roadmarktypes = lane_roadMark->GetNumberOfRoadMarkTypes();
						if (number_of_roadmarktypes != 0)
						{
							lane_roadMarkType = lane_roadMark->GetLaneRoadMarkTypeByIdx(0);
							number_of_roadmarklines = lane_roadMarkType->GetNumberOfRoadMarkTypeLines();

							int inner_index = -1;
							if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::BROKEN_SOLID ||
								lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::SOLID_BROKEN)
							{
								if (number_of_roadmarklines < 2)
								{
									LOG_AND_QUIT("You need to specify at least 2 line for broken solid or solid broken roadmark type");
								}
								std::vector<double> sort_solidbroken_brokensolid;
								for (int q=0; q<number_of_roadmarklines; q++)
								{
									sort_solidbroken_brokensolid.push_back(lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(q)->GetTOffset());
								}

								if (lane->GetId() < 0 || lane->GetId() == 0)
								{
									inner_index = (int)(std::max_element(sort_solidbroken_brokensolid.begin(), sort_solidbroken_brokensolid.end()) - sort_solidbroken_brokensolid.begin());
								}
								else
								{
									inner_index = (int)(std::min_element(sort_solidbroken_brokensolid.begin(), sort_solidbroken_brokensolid.end()) - sort_solidbroken_brokensolid.begin());
								}

							}

							// Looping through each roadmarkline under roadmark
							for (int n=0; n<number_of_roadmarklines; n++)
							{
								lane_roadMarkTypeLine = lane_roadMarkType->GetLaneRoadMarkTypeLineByIdx(n);
								s_roadmarkline = s_roadmark + lane_roadMarkTypeLine->GetSOffset();
								if (lane_roadMarkTypeLine != 0)
								{
									s_end_roadmarkline = s_end_roadmark;

									bool broken = false;
									if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::BROKEN_SOLID)
									{
										if (inner_index == n)
										{
											broken = true;
										}
									}

									if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::SOLID_BROKEN)
									{
										broken = true;
										if (inner_index == n)
										{
											broken = false;
										}
									}

									if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::BOTTS_DOTS)
									{
										// Setting OSI points for each dot
										while (true)
										{
											pos_candidate.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, s_roadmarkline, 0, j);
											PointStruct p = { s_roadmarkline, pos_candidate.GetX(), pos_candidate.GetY(), pos_candidate.GetZ(), pos_candidate.GetHRoad() };
											osi_point.push_back(p);

											s_roadmarkline += lane_roadMarkTypeLine->GetSpace();
											if (s_roadmarkline < SMALL_NUMBER || s_roadmarkline > s_end_roadmarkline - SMALL_NUMBER)
											{
												if (s_roadmarkline < SMALL_NUMBER)
												{
													LOG("Roadmark length + space = 0 - ignoring");
												}
												break;
											}
										}
									}
									else if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::BROKEN ||
										lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::BROKEN_BROKEN ||
										broken)
									{
										// Setting OSI points for each roadmarkline
										while(true)
										{
											pos_candidate.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, s_roadmarkline, 0, j);
											PointStruct p = { s_roadmarkline, pos_candidate.GetX(), pos_candidate.GetY(), pos_candidate.GetZ(), pos_candidate.GetHRoad() };
											osi_point.push_back(p);

											double s_rm_end = MIN(s_roadmarkline + lane_roadMarkTypeLine->GetLength(), s_end_roadmark);
											pos_candidate.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, s_rm_end, 0, j);
											p = { s_rm_end, pos_candidate.GetX(), pos_candidate.GetY(), pos_candidate.GetZ(), pos_candidate.GetHRoad() };
											osi_point.push_back(p);

											s_roadmarkline += lane_roadMarkTypeLine->GetLength() + lane_roadMarkTypeLine->GetSpace();
											if (s_roadmarkline < SMALL_NUMBER || s_roadmarkline > s_end_roadmarkline - SMALL_NUMBER)
											{
												if (s_roadmarkline < SMALL_NUMBER)
												{
													LOG("Roadmark length + space = 0 - ignoring");
												}
												break;
											}
										}
									}
									else if (lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::SOLID ||
										lane_roadMark->GetType() == LaneRoadMark::RoadMarkType::SOLID_SOLID ||
										!broken)
									{
										int counter = 0;

										// [XO, YO] = Real position with no tolerance
										pos_pivot.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, s_roadmarkline, 0, j);

										// Add the starting point of each lane as osi point
										PointStruct p = { s_roadmarkline, pos_pivot.GetX(), pos_pivot.GetY(), pos_pivot.GetZ(), pos_pivot.GetHRoad() };
										osi_point.push_back(p);

										// [XO, YO] = closest position with given (-) tolerance
										pos_tmp.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, MAX(0, s_roadmarkline - OSI_TANGENT_LINE_TOLERANCE), 0, j);
										x0.push_back(pos_tmp.GetX());
										y0.push_back(pos_tmp.GetY());

										// Push real position between the +/- tolerance points
										x0.push_back(pos_pivot.GetX());
										y0.push_back(pos_pivot.GetY());

										// [XO, YO] = closest position with given (+) tolerance
										pos_tmp.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, MIN(s_roadmarkline + OSI_TANGENT_LINE_TOLERANCE, road->GetLength()), 0, j);
										x0.push_back(pos_tmp.GetX());
										y0.push_back(pos_tmp.GetY());

										bool insert = false;
										double step = OSI_POINT_CALC_STEPSIZE;

										pos_candidate = pos_pivot;

										while(++counter)
										{
											// Make sure we stay within lane section length
											double s = MIN(pos_candidate.GetS() + step, s_end_roadmark - SMALL_NUMBER / 2);

											// [X1, Y1] = Real position with no tolerance
											pos_candidate.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, s, 0, j);

											// [X1, Y1] = closest position with given (-) tolerance
											pos_tmp.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, MAX(s - OSI_TANGENT_LINE_TOLERANCE, 0), 0, j);
											x1.push_back(pos_tmp.GetX());
											y1.push_back(pos_tmp.GetY());

											x1.push_back(pos_candidate.GetX());
											y1.push_back(pos_candidate.GetY());

											// [X1, Y1] = closest position with given (+) tolerance
											pos_tmp.SetRoadMarkPos(road->GetId(), lane->GetId(), m, 0, n, MIN(s + OSI_TANGENT_LINE_TOLERANCE, lsec_end), 0, j);
											x1.push_back(pos_tmp.GetX());
											y1.push_back(pos_tmp.GetY());

											// Check OSI Requirement between current given points
											if (NEAR_NUMBERS(pos_pivot.GetH(), pos_candidate.GetH()))
											{
												if (DistanceFromPointToLine2DWithAngle(pos_candidate.GetX(), pos_candidate.GetY(),
													pos_pivot.GetX(), pos_pivot.GetY(), pos_pivot.GetH()) < min_segment_length)
												{
													osi_requirement = true;  // points on a straight segment
												}
												else
												{
													osi_requirement = false;  // same heading but not on a straight line => lane discontinuity
												}
											}
											else
											{
												osi_requirement = CheckLaneOSIRequirement(x0, y0, x1, y1);
											}

											// If requirement is satisfied -> look further points
											// If requirement is not satisfied:
											//    Assign last unique satisfied point as OSI point
											//    Continue searching from the last satisfied point

											// Make sure max segment length is longer than stepsize and considering elevation change rate
											if (osi_requirement)
											{
												max_segment_length = GetMaxSegmentLen(&pos_pivot, &pos_candidate, 1.1 * OSI_POINT_CALC_STEPSIZE, SE_Env::Inst().GetOSIMaxLongitudinalDistance(),
													OSI_POINT_DIST_SCALE, OSI_POINT_DIST_SCALE, osi_requirement);
											}

											if (pos_candidate.GetS() + SMALL_NUMBER > s_end_roadmark - SMALL_NUMBER ||   // end of the lane reached, assign as final OSI point
												osi_requirement && pos_candidate.GetS() - pos_pivot.GetS() > max_segment_length - SMALL_NUMBER ||
												abs(step) < min_segment_length + SMALL_NUMBER)
											{
												p = { pos_candidate.GetS(), pos_candidate.GetX(), pos_candidate.GetY(), pos_candidate.GetZ(), pos_candidate.GetHRoad() };
												osi_point.push_back(p);
												insert = false;

												if (pos_candidate.GetS() + SMALL_NUMBER > s_end_roadmark - SMALL_NUMBER)
												{
													break;
												}

												// If last step length was small, guess next one will also be small to reduce search
												step = MIN(OSI_POINT_CALC_STEPSIZE, 2.0 * (pos_candidate.GetS() - pos_pivot.GetS()));

												pos_pivot = pos_candidate;

												// reuse candidate x-y collectors for pivot position
												x0 = x1;
												y0 = y1;
											}
											else
											{
												if (osi_requirement == false)
												{
													insert = true;  // indicate that a point needs to be inserted
													step = -abs(step) / 2.0;  // look backwards half current stepsize
												}
												else if (insert)
												{
													step = abs(step) / 2.0;  // look forward half current stepsize
												}
											}
											// Clear x-y collectors for next iteration
											x1.clear();
											y1.clear();
										}
									}

									// Set all collected osi points for the current lane rpadmarkline
									lane_roadMarkTypeLine->osi_points_.Set(osi_point);

									// Clear osi collectors for roadmarks for next iteration
									osi_point.clear();
								}
								else
								{
									LOG("LaneRoadMarkTypeLine %d for LaneRoadMarkType for LaneRoadMark %d for lane %d is not defined", n, m, lane->GetId());
								}
							}
						}
					}
				}
			}
		}
	}
}

bool OpenDrive::SetRoadOSI()
{
	if (this == Position::GetOpenDrive())
	{
		SetLaneOSIPoints();
		SetRoadMarkOSIPoints();
		SetLaneBoundaryPoints();
		return true;
	}

	return false;
}

int LaneSection::GetClosestLaneIdx(double s, double t, double &offset, bool noZeroWidth, int laneTypeMask)
{
	double min_offset = t;  // Initial offset relates to reference line
	int candidate_lane_idx = -1;

	for (int i = 0; i < GetNumberOfLanes(); i++)  // Search through all lanes
	{
		int lane_id = GetLaneIdByIdx(i);

		// Only consider lanes in same direction
		if (lane_id == 0 || SIGN(lane_id) != SIGN(t))
		{
			continue;
		}

		double laneCenterOffset = SIGN(lane_id) * GetCenterOffset(s, lane_id);

		// Only consider lanes with matching lane type
		if (laneTypeMask & GetLaneById(lane_id)->GetLaneType() && (!noZeroWidth || GetWidth(s, lane_id) > SMALL_NUMBER))
		{
			// If position is within a lane, we can return it without further checks
			if (fabs(t - laneCenterOffset) < (GetWidth(s, lane_id) / 2.))
			{
				min_offset = t - laneCenterOffset;
				candidate_lane_idx = i;
				break;
			}
			if (candidate_lane_idx == -1 || fabs(t - laneCenterOffset) < fabs(min_offset))
			{
				min_offset = t - laneCenterOffset;
				candidate_lane_idx = i;
			}
		}
	}

	offset = min_offset;

	if (candidate_lane_idx == -1)
	{
		// Fall back to reference lane
		candidate_lane_idx = GetLaneIdxById(0);
	}

	return candidate_lane_idx;
}

int Position::GotoClosestDrivingLaneAtCurrentPosition()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0)
	{
		LOG("No road %d", track_idx_);
		return -1;
	}

	LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

	if (lane_section == 0)
	{
		LOG("No lane section for idx %d - keeping current lane setting\n", lane_section_idx_);
		return -1;
	}

	double offset;
	int lane_idx = lane_section->GetClosestLaneIdx(s_, t_, offset, true);

	if (lane_idx == -1)
	{
		LOG("Failed to find a valid drivable lane");
		return -1;
	}

	lane_id_ = lane_section->GetLaneIdByIdx(lane_idx);

	offset_ = offset;

	return 0;
}

void Position::Track2Lane()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0)
	{
		LOG("Position::Track2Lane Error: No road %d\n", track_idx_);
		return;
	}

	Geometry *geometry = road->GetGeometry(geometry_idx_);
	if (geometry == 0)
	{
		LOG("Position::Track2Lane Error: No geometry %d\n", geometry_idx_);
		return;
	}

	// Find LaneSection according to s, starting from current
	int lane_section_idx = road->GetLaneSectionIdxByS(s_, lane_section_idx_);
	LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx);
	if (lane_section == 0)
	{
		LOG("No lane section for idx %d - keeping current lane setting", lane_section_idx_);
		return;
	}

	// Find the closest driving lane within the lane section
	double offset;
	int lane_idx = lane_section->GetClosestLaneIdx(s_, t_, offset, true, snapToLaneTypes_);

	if (lane_idx == -1)
	{
		LOG("Failed find closest lane");
		return;
	}

	offset_ = offset;
	// Update cache indices
	lane_idx_ = lane_idx;
	lane_id_ = lane_section->GetLaneIdByIdx(lane_idx_);
	lane_section_idx_ = lane_section_idx;
}

typedef struct
{
	int lsec_idx;            // lane section index
	int k;                   // osi point index (on given lane section)
	SE_Vector p;             // position (x, y)
	SE_Vector n;             // normal
	double h;                // heading
	double z;                // height
	PointStruct* osi_point;  // osi point reference
} XYZHVertex;

Position::ReturnCode Position::XYZH2TrackPos(double x3, double y3, double z3, double h3, bool connectedOnly, int roadId, bool check_overlapping_roads)
{
	// Overall method:
	//   1. Iterate over all roads, looking at OSI points of each lane sections center line (lane 0)
	//   2. Identify line segment (between two OSI points) closest to xyz point
	//   3. Identify which vertex of the line is closest
	//   4. Given the normals of lines on each side of the vertex, identify which line the points projects onto
	//   5. The s value for projected xyz point on the line segment corresponds to the rate
	//      between angle from xyz point to projected point and the difference of angle normals

	Road *road, *current_road = 0;
	Road *roadMin = 0;
	bool directlyConnected = false;
	double weight = 0; // Add some resistance to switch from current road, applying a stronger bound to current road
	double curvature = 0;
	bool search_done = false;
	double closestS = 0;
	int jMin=-1, kMin=-1;
	double closestPointDist = INFINITY;
	bool closestPointInside = false;
	bool insideCurrentRoad = false;  // current postion projects on current road
	double curvatureAbsMin = INFINITY;
	bool closestPointDirectlyConnected = false;
	overlapping_roads.clear();

	if (GetOpenDrive()->GetNumOfRoads() == 0)
	{
		return ReturnCode::ERROR_GENERIC;
	}

	// First step is to identify closest road and OSI line segment

	size_t nrOfRoads;
	if (route_ && route_->IsValid())
	{
		// Route assigned. Iterate over all roads in the route. I.e. check all waypoints road ID.
		nrOfRoads = route_->minimal_waypoints_.size();
	}
	else
	{
		// Iterate over all roads in the road network
		nrOfRoads = GetOpenDrive()->GetNumOfRoads();
	}


	if (roadId == -1)
	{
		current_road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	}
	else
	{
		// Look only at specified road
		current_road = GetOpenDrive()->GetRoadById(roadId);
		nrOfRoads = 0;
	}

	for (int i = -2; !search_done && i < (int)nrOfRoads; i++)
	{
		// i == -2: Check limited point window around last known point
		// i == -1: Check current road
		// i > 0: Check all other roads
		if (i < 0)
		{
			// First check current road (from last known position).
			if (current_road)
			{
				road = current_road;
			}
			else
			{
				continue;  // Skip, no current road
			}
		}
		else
		{
			if (current_road && i == track_idx_)
			{
				continue; // Skip, already checked this one
			}
			else
			{
				if (route_ && route_->IsValid())
				{
					road = GetOpenDrive()->GetRoadById(route_->minimal_waypoints_[i].GetTrackId());
				}
				else
				{
					road = GetOpenDrive()->GetRoadByIdx(i);
				}
				if (connectedOnly)
				{
					// Check whether the road is reachble from current position
					Position tmpPos(road->GetId(), 0.0, 0.0);
					PositionDiff posDiff;
					if (Delta(&tmpPos, posDiff) == false)
					{
						continue;  // skip unreachable road
					}
				}
			}
		}

		// Check whether complete road is too far away - then skip to next
		const double potentialWidthOfRoad = 25;
		if (PointDistance2D(x3, y3, road->GetGeometry(0)->GetX(), road->GetGeometry(0)->GetY()) -
			(road->GetLength() + potentialWidthOfRoad) > closestPointDist)  // add potential width of the road
		{
			continue;
		}

		weight = 0;
		curvature = INFINITY;

		// Add resistance to leave current road or directly connected ones
		// actual weights are totally unscientific... up to tuning
		if (road != current_road)
		{
			if (current_road && current_road->IsDirectlyConnected(road, &curvature, lane_id_))
			{
				directlyConnected = true;
			}
			else
			{
				weight += 3;  // For non connected roads add additional "penalty" threshold
				directlyConnected = false;
			}
		}

		// First find distance from current position
		double distFromCurrentPos = GetLengthOfLine2D(x3, y3, GetX(), GetY());

		int startLaneSecIdx = 0;
		int startOSIPointIdx = 0;
		int search_win = 10;

		// If new point is close to current/old, then look at the 10 closest osi points surrounding current one
		if (i == -2)
		{
			if (osi_point_idx_ > -1 && distFromCurrentPos < 5.0 && lane_section_idx_ > -1)
			{
				startLaneSecIdx = lane_section_idx_;
				startOSIPointIdx = osi_point_idx_ - search_win / 2;
				int n_points = search_win / 2;

				if (startOSIPointIdx < 0)
				{
					n_points = -startOSIPointIdx;
					if (startLaneSecIdx > 0)
					{
						// look in previous lane sections
						for (size_t j = 0; n_points > 0 && startLaneSecIdx > 0 && j < search_win / 2; j++)
						{
							startLaneSecIdx--;
							startOSIPointIdx = road->GetLaneSectionByIdx(startLaneSecIdx)->GetLaneById(0)->GetOSIPoints()->GetNumOfOSIPoints() - n_points;
							if (startOSIPointIdx < 0)
							{
								n_points = -startOSIPointIdx;
								startOSIPointIdx = 0;
							}
							else
							{
								break;
							}
						}
					}
					else
					{
						startOSIPointIdx = 0;
					}
				}
			}
			else
			{
				continue;
			}
		}

		XYZHVertex v[3];  // circular buffer for osi points and additional info
		int l0 = -1, l1 = -2, l2 = -3;  // circular buffer indices
		int counter = 0;  // only for potential first (i==-2) limited search round
		double s_norm = -1.0;
		int jMinLocal = startLaneSecIdx;   // keep track of "best" lane section candidate
		int kMinLocal = startOSIPointIdx;  // keep track of "best" osi point candidate (on given lane section)

		for (int j = startLaneSecIdx; (i != -2 || counter < search_win) && j < road->GetNumberOfLaneSections() && !search_done; j++)
		{
			OSIPoints* osiPoints = road->GetLaneSectionByIdx(j)->GetLaneById(0)->GetOSIPoints();

			// add two loops at last lane section to handle calculations at last segments. Skip last point on intermediate segments, since
			// it's duplicated by first in following segment
			int n_iter = (j == road->GetNumberOfLaneSections() - 1) ? osiPoints->GetNumOfOSIPoints() + 2 : osiPoints->GetNumOfOSIPoints() - 1;
			for (int k = (j == startLaneSecIdx ? startOSIPointIdx : 0); (i != -2 || counter < search_win) && k < n_iter; k++, counter++)
			{
				double distTmp = 0.0;
				double weightedDist = 0.0;
				bool inside = false;

				// OSI points is an approximation of actual geometry
				// To find out whether a point is within a road (segment), check if the point is within the
				// area formed by extending normals at OSI segment endpoints.
				// Normal at endpoints is calculate as mean normal between the two neighbor OSI segments.
				Position pos;
				l2 = l1;
				l1 = l0;

				if (k < osiPoints->GetNumOfOSIPoints())
				{
					l0 = (l0 + 1) % 3;

					v[l0].k = k;
					v[l0].lsec_idx = j;
					v[l0].osi_point = &(osiPoints->GetPoint(k));

					if (j == 0 && k == 0 ||  // first segment
						(j == road->GetNumberOfLaneSections() - 1) && (k == osiPoints->GetNumOfOSIPoints() - 1)) // last segment
					{
						if (j == 0 && k == 0)
						{
							// road startpoint, pick actual heading
							pos.SetTrackPos(road->GetId(), 0, 0.0);
						}
						else
						{
							// road endpoint, pick actual heading
							pos.SetTrackPos(road->GetId(), road->GetLength(), 0.0);
							v[l0].h = pos.GetH();
						}
						v[l0].p.Set(pos.GetX(), pos.GetY());
						v[l0].z = pos.GetZ();
						v[l0].n = SE_Vector(1.0, 0.0).Rotate(GetAngleSum(pos.GetH(), M_PI_2));  // +90 degree from heading
					}
					else
					{
						// intermediate point
						v[l0].p.Set(v[l0].osi_point->x, v[l0].osi_point->y);
						v[l0].z = v[l0].osi_point->z;
					}

				}

				if (counter > 0 && !((j == road->GetNumberOfLaneSections() - 1) && (k > osiPoints->GetNumOfOSIPoints() - 1)))
				{
					// All vertex headings, except the last one, are based on osi points
					v[l1].h = GetAngleOfVector(v[l0].p.x() - v[l1].p.x(), v[l0].p.y() - v[l1].p.y());
				}

				if (counter < 2)
				{
					continue;  // skip first two rounds, wait until needed data collected
				}
				else if (!((j == road->GetNumberOfLaneSections() - 1) && (k > osiPoints->GetNumOfOSIPoints() - 1)))
				{
					// Calculate normal of previous segment based on mean heading of previous two osi segments.
					// Except last vertex where actual normal is used
					double h_mean = GetAngleInInterval2PI(v[l2].h + 0.5 * GetAngleDifference(v[l1].h, v[l2].h));
					v[l1].n = SE_Vector(1.0, 0.0).Rotate(GetAngleSum(h_mean, M_PI_2));  // +90 degree from heading
				}

				// in case of multiple roads with the same reference line, also look at width of the road of relevant side
				// side of road is determined by cross product of position (relative OSI point) and road heading
				double cp = GetCrossProduct2D(cos(v[l2].osi_point->h), sin(v[l2].osi_point->h), x3 - v[l2].p.x(), y3 - v[l2].p.y());
				double width = road->GetWidth(v[l2].osi_point->s, SIGN(cp), ~Lane::LaneType::LANE_TYPE_NONE);

				// Now check if the point is  between extended normals
				inside = IsPointWithinSectorBetweenTwoLines(SE_Vector(x3, y3), v[l2].p, v[l2].p + v[l2].n, v[l1].p, v[l1].p + v[l1].n, s_norm);
				// Also require that the point is on the positive side of the first vector
				inside = (inside && s_norm >= 0.0);

				// Find out distance from the point projected at (extended) osi segment. We're only interested in the orthogonal distance
				distTmp = DistanceFromPointToLine2D(x3, y3, v[l2].p.x(), v[l2].p.y(), v[l1].p.x(), v[l1].p.y(), 0, 0);

				// Find closest point of the two
				if (PointSquareDistance2D(x3, y3, v[l2].p.x(), v[l2].p.y()) <
					PointSquareDistance2D(x3, y3, v[l1].p.x(), v[l1].p.y()))
				{
					kMinLocal = v[l2].k;
					jMinLocal = v[l2].lsec_idx;
				}
				else
				{
					kMinLocal = v[l1].k;
					jMinLocal = v[l1].lsec_idx;
				}

				// subtract width of the road
				distTmp = distTmp - width;
				if (distTmp < 0)
				{
					// On road - distance is zero, but continue search because
					// we could be in a junction where roads are overlapping
					distTmp = 0;
				}

				double z = v[l1].z;
				if (inside)
				{
					z = (1 - s_norm) * v[l2].z + s_norm * v[l1].z;
				}
				else
				{
					// Find distance to line endpoint
					distTmp = sqrt(distTmp * distTmp + s_norm * s_norm);
				}

				weightedDist = distTmp;

				if (fabs(z3 - z) > 2.0)
				{
					// Add threshold for considering z - to avoid noise in co-planar distance calculations
					weightedDist += fabs(z3 - z);
				}

				if (!insideCurrentRoad && road == current_road)
				{
					// Register whether current position is on current road
					// Allow for 2 meter lateral slack outside road edges
					insideCurrentRoad = inside && weightedDist < 2;
				}
				else if (insideCurrentRoad)
				{
					// Only add weight if position inside current road
					// longitudinal (end points) and lateral (road width)
					weightedDist += weight;
				}
				if (weightedDist < closestPointDist + SMALL_NUMBER)
				{
					bool directlyConnectedCandidate = directlyConnected;

					if (directlyConnected && closestPointDirectlyConnected)
					{
						// For directly connected roads (junction), we might have options
						// among equally close ones, find the one which goes the most straight forward
						if (fabs(weightedDist - closestPointDist) < SMALL_NUMBER)
						{
							if (abs(curvature) > curvatureAbsMin)
							{
								directlyConnectedCandidate = false;
							}
						}
					}

					if (directlyConnectedCandidate || weightedDist < closestPointDist)
					{
						closestPointDist = weightedDist;
						roadMin = road;
						jMin = jMinLocal;
						kMin = kMinLocal;
						closestPointInside = inside;
						closestPointDirectlyConnected = directlyConnected;
						osi_point_idx_ = kMinLocal;

						if (directlyConnected)
						{
							curvatureAbsMin = abs(curvature);
						}
					}
				}
				else if (i == -2)
				{
					// distance is now increasing, indicating that we already passed the closest point
					if (closestPointInside && closestPointDist < SMALL_NUMBER && !check_overlapping_roads)
					{
						search_done = true;
						break;
					}
				}

				if (inside && distTmp < SMALL_NUMBER && (overlapping_roads.size() == 0 || overlapping_roads.back() != road->GetId()))
				{
					overlapping_roads.push_back(road->GetId());  // add overlap candidate
				}
			}
		}
	}

	if (closestPointInside)
	{
		status_ &= ~static_cast<int>(Position::PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}
	else
	{
		status_ |= static_cast<int>(Position::PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}

	// The closest OSI vertex has been identified
	// Now, find out exact road s-value based on interpolation of normal angles
	// for the two lines having the vertex in common

	if (roadMin == 0)
	{
		LOG("Error finding minimum distance\n");
		return ReturnCode::ERROR_GENERIC;
	}

	if (jMin != -1 && kMin != -1)
	{
		// Find out what line the points projects to, starting or ending with closest point?
		// Do this by comparing the angle to the position with the road normal at found point

		PointStruct osip_closest, osip_first, osip_second;
		osip_closest = roadMin->GetLaneSectionByIdx(jMin)->GetLaneById(0)->GetOSIPoints()->GetPoint(kMin);

		double xTangent = cos(osip_closest.h);
		double yTangent = sin(osip_closest.h);
		double dotP = GetDotProduct2D(xTangent, yTangent, x3 - osip_closest.x, y3 - osip_closest.y);

		int jFirst, jSecond, kFirst, kSecond;

		if (dotP > 0)
		{
			// Positive dot product means closest OSI point is behind
			osip_first = osip_closest;
			jFirst = jMin;
			kFirst = kMin;

			if (kMin < roadMin->GetLaneSectionByIdx(jMin)->GetLaneById(0)->GetOSIPoints()->GetNumOfOSIPoints() - 1)
			{
				jSecond = jMin;
				kSecond = kMin + 1;
			}
			else
			{
				if (jMin < roadMin->GetNumberOfLaneSections() - 1)
				{
					jSecond = jMin + 1;
					if (roadMin->GetLaneSectionByIdx(jSecond)->GetLaneById(0)->GetOSIPoints()->GetNumOfOSIPoints() > 1)
					{
						kSecond = 1; // Skip first point, it's the same as last in last lane section
					}
					else
					{
						kSecond = 0; // Only one point available in lane section - don't go further
					}
				}
				else
				{
					// Last point
					jSecond = jMin;
					kSecond = kMin;
				}
			}
			osip_second = roadMin->GetLaneSectionByIdx(jSecond)->GetLaneById(0)->GetOSIPoints()->GetPoint(kSecond);
		}
		else
		{
			// Negative dot product means closest OSI point is ahead
			osip_second = osip_closest;
			jSecond = jMin;
			kSecond = kMin;

			if (kMin > 0)
			{
				jFirst = jMin;
				kFirst = kMin - 1;
			}
			else
			{
				if (jMin > 0)
				{
					jFirst = jMin - 1;
					if (roadMin->GetLaneSectionByIdx(jFirst)->GetLaneById(0)->GetOSIPoints()->GetNumOfOSIPoints() > 1)
					{
						// Skip last point, it's the same as first in successor lane section
						kFirst = roadMin->GetLaneSectionByIdx(jFirst)->GetLaneById(0)->GetOSIPoints()->GetNumOfOSIPoints() - 2;
					}
					else
					{
						// Only one point available in lane section - don't go further
						kFirst = roadMin->GetLaneSectionByIdx(jFirst)->GetLaneById(0)->GetOSIPoints()->GetNumOfOSIPoints() - 1;
					}
				}
				else
				{
					// First point
					jFirst = jMin;
					kFirst = kMin;
				}
			}
			osip_first = roadMin->GetLaneSectionByIdx(jFirst)->GetLaneById(0)->GetOSIPoints()->GetPoint(kFirst);
		}

		if (jFirst == jSecond && kFirst == kSecond)
		{
			// Same point
			closestS = osip_first.s;
			closestPointInside = false;
		}
		else
		{
			// Different points
			double angleBetweenNormals, angleToPosition;
			double normalIntersectionX, normalIntersectionY;
			double sNorm = 0;

			closestPointInside = true;

			// Check for straight line
			if (fabs(osip_first.h - osip_second.h) < 1e-5)  // Select threshold to avoid precision issues in calculations
			{
				double px, py;
				ProjectPointOnVector2D(x3, y3, osip_first.x, osip_first.y, osip_second.x, osip_second.y, px, py);

				// Find relative position of projected point on line segment
				double l1 = GetLengthOfLine2D(osip_first.x, osip_first.y, px, py);
				double l2 = GetLengthOfLine2D(osip_first.x, osip_first.y, osip_second.x, osip_second.y);
				sNorm = l1 / l2;
			}
			else
			{
				// Find normals at end points of line segment
				double xn0, yn0, xn1, yn1;
				RotateVec2D(cos(osip_first.h), sin(osip_first.h), M_PI_2, xn0, yn0);
				RotateVec2D(cos(osip_second.h), sin(osip_second.h), M_PI_2, xn1, yn1);

				// Find intersection of extended normals
				GetIntersectionOfTwoLineSegments(
					osip_first.x, osip_first.y,
					osip_first.x + xn0, osip_first.y + yn0,
					osip_second.x, osip_second.y,
					osip_second.x + xn1, osip_second.y + yn1,
					normalIntersectionX, normalIntersectionY);

				// Align normal vectors to direction from intersection towards line segment
				NormalizeVec2D(osip_first.x - normalIntersectionX, osip_first.y - normalIntersectionY, xn0, yn0);
				NormalizeVec2D(osip_second.x - normalIntersectionX, osip_second.y - normalIntersectionY, xn1, yn1);

				// Find angle between normals
				angleBetweenNormals = acos(GetDotProduct2D(-xn0, -yn0, -xn1, -yn1));

				// Find angle between the two vectors:
				// 1. line between normals intersection and the point of query
				// 2. Normal in the first point of closest line segment (turned around to match direction of first line)
				double lx = normalIntersectionX - x3;
				double ly = normalIntersectionY - y3;
				double lLength = sqrt(lx * lx + ly * ly);
				angleToPosition = acos(CLAMP(GetDotProduct2D(-xn0, -yn0, lx / lLength, ly / lLength), -1.0, 1.0));

				// Finally calculate interpolation factor
				if (fabs(angleBetweenNormals) < SMALL_NUMBER)
				{
					sNorm = 0.0;
				}
				else
				{
					sNorm = angleToPosition / angleBetweenNormals;
				}

				//printf("road_id %d jMin %d kMin %d lx %.2f ly %.2f angle0 %.2f angle1 %.2f normalIntersectionX %.2f normalIntersectionY %.2f sNorm %.2f\n",
				//	roadMin->GetId(), jMin, kMin, lx, ly, angleToPosition, angleBetweenNormals, normalIntersectionX, normalIntersectionY, sNorm);
			}

			closestS = (1 - sNorm) * osip_first.s + sNorm * osip_second.s;
			closestS = CLAMP(closestS, 0, roadMin->GetLength());
		}
	}
	else
	{
		LOG("Unexpected: No closest OSI point found!");
	}

	double fixedLaneOffset = 0;
	int fixedLaneId = 0;
	if (lockOnLane_)
	{
		// Register lateral position of previous lane
		LaneSection* lsec = current_road->GetLaneSectionByIdx(lane_section_idx_);
		if (lsec)
		{
			fixedLaneOffset = SIGN(lane_id_) * lsec->GetCenterOffset(s_, lane_id_);

			// Now find cloest lane at that lateral position, at updated s value
			double laneOffset;
			int lane_idx = lsec->GetClosestLaneIdx(closestS, fixedLaneOffset, laneOffset, true, Lane::LaneType::LANE_TYPE_ANY_DRIVING);
			fixedLaneId = lsec->GetLaneIdByIdx(lane_idx);
		}
	}

	// Set position exact on center line
	ReturnCode retvalue = SetTrackPos(roadMin->GetId(), closestS, 0, true);

	double xCenterLine = x_;
	double yCenterLine = y_;

	// Find out actual lateral position
	double latOffset = PointToLineDistance2DSigned(
		x3, y3, xCenterLine, yCenterLine,
		xCenterLine + cos(GetHRoad()), yCenterLine + sin(GetHRoad()));

	// Update lateral offsets
	if (lockOnLane_)
	{
		SetLanePos(roadMin->GetId(), fixedLaneId, closestS, latOffset - fixedLaneOffset);
	}
	else
	{
		SetTrackPos(roadMin->GetId(), closestS, latOffset, false);
	}

	static int rid = 0;
	if (roadMin->GetId() != rid)
	{
		rid = roadMin->GetId();
	}

	// Set specified position and heading
	SetX(x3);
	SetY(y3);
	SetHeading(h3);

	EvaluateRoadZPitchRoll();

	if (!closestPointInside)
	{
		// if outside road endpoint boundries, ignore road pitch
		p_road_ = 0.0;
		SetPitch(0.0);
	}

	// If on a route, calculate corresponding route position
	if (route_ && route_->IsValid())
	{
		CalcRoutePosition();
	}

	return retvalue;
}

bool Position::EvaluateRoadZPitchRoll()
{
	if (track_id_ < 0)
	{
		return false;
	}

	bool ret_value = false;

	Road* road = GetRoadById(track_id_);
	if (road != nullptr)
	{
		ret_value = road->GetZAndPitchByS(s_, &z_road_, &z_roadPrim_, &z_roadPrimPrim_, &p_road_, &elevation_idx_);
		ret_value &= road->UpdateZAndRollBySAndT(s_, t_, &z_road_, &roadSuperElevationPrim_, &r_road_, &super_elevation_idx_);
	}
	else
	{
		LOG("Failed to lookup road id %d", track_id_);
	}

	if (align_z_ == ALIGN_MODE::ALIGN_SOFT)
	{
		z_ = z_road_ + z_relative_;
	}
	else if (align_z_ == ALIGN_MODE::ALIGN_HARD)
	{
		z_ = z_road_;
	}
	else
	{
		z_ = z_relative_;
	}

	return ret_value;
}

Position::ReturnCode Position::Track2XYZ()
{
	if (GetOpenDrive()->GetNumOfRoads() == 0)
	{
		return ReturnCode::ERROR_GENERIC;
	}

	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road == 0)
	{
		LOG("Position::Track2XYZ Error: No road %d\n", track_idx_);
		return ReturnCode::ERROR_GENERIC;
	}

	Geometry *geometry = road->GetGeometry(geometry_idx_);
	if (geometry == 0)
	{
		LOG("Position::Track2XYZ Error: No geometry %d\n", geometry_idx_);
		return ReturnCode::ERROR_GENERIC;
	}

	geometry->EvaluateDS(s_ - geometry->GetS(), &x_, &y_, &h_road_);

	// Consider lateral t position, perpendicular to track heading
	double x_local = (t_ + road->GetLaneOffset(s_)) * cos(h_road_ + M_PI_2);
	double y_local = (t_ + road->GetLaneOffset(s_)) * sin(h_road_ + M_PI_2);

	h_road_ += atan(road->GetLaneOffsetPrim(s_)) + h_offset_;
	h_road_ = GetAngleInInterval2PI(h_road_);

	x_ += x_local;
	y_ += y_local;

	// z = Elevation
	EvaluateRoadZPitchRoll();

	EvaluateOrientation();

	return ReturnCode::OK;
}

void Position::LaneBoundary2Track()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	t_ = 0;

	if (road != 0 && road->GetNumberOfLaneSections() > 0)
	{
		LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		if (lane_section != 0 && lane_id_ !=0)
		{
			t_ = offset_ + lane_section->GetOuterOffset(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
			h_offset_ = lane_section->GetOuterOffsetHeading(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
		}
	}
}

void Position::Lane2Track()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	t_ = 0;

	if (road != 0 && road->GetNumberOfLaneSections() > 0)
	{
		LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		if (lane_section != 0)
		{
			t_ = offset_ + lane_section->GetCenterOffset(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
			h_offset_ = lane_section->GetCenterOffsetHeading(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
		}
	}
}

void Position::RoadMark2Track()
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	t_ = 0;

	if (road != 0 && road->GetNumberOfLaneSections() > 0)
	{
		LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		if (lane_section != 0 && lane_id_ !=0)
		{
			t_ = offset_ + lane_section->GetOuterOffset(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
			h_offset_ = lane_section->GetOuterOffsetHeading(s_, lane_id_) * (lane_id_ < 0 ? -1 : 1);
		}

		Lane *lane = lane_section->GetLaneByIdx(lane_idx_);
		LaneRoadMark *lane_roadmark = lane->GetLaneRoadMarkByIdx(roadmark_idx_);
		LaneRoadMarkType *lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(roadmarktype_idx_);
		LaneRoadMarkTypeLine *lane_roadmarktypeline = lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(roadmarkline_idx_);

		if (lane_roadmarktypeline != 0)
		{
			t_ = t_ + lane_roadmarktypeline->GetTOffset();
		}
	}
}

void Position::XYZ2Track()
{
	XYZH2TrackPos(x_, y_, z_, h_);
}

Position::ReturnCode Position::SetLongitudinalTrackPos(int track_id, double s)
{
	Road *road;

	if (GetOpenDrive()->GetNumOfRoads() == 0)
	{
		return ReturnCode::ERROR_GENERIC;
	}

	if ((road = GetOpenDrive()->GetRoadById(track_id)) == 0)
	{
		LOG("Position::Set Error: track %d not found", track_id);

		// Just hard code values and return
		track_id_ = track_id;
		s_ = s;

		return ReturnCode::ERROR_GENERIC;
	}
	if (track_id != track_id_)
	{
		// update internal track and geometry indices
		track_id_ = track_id;
		track_idx_ = GetOpenDrive()->GetTrackIdxById(track_id);
		geometry_idx_ = 0;
		elevation_idx_ = 0;
		super_elevation_idx_ = 0;
		lane_section_idx_ = 0;
		lane_id_ = 0;
		lane_idx_ = 0;
		osi_point_idx_ = 0;
	}


	Geometry *geometry = road->GetGeometry(geometry_idx_);
	if (geometry == nullptr)
	{
		return ReturnCode::ERROR_GENERIC;
	}

	// check if still on same geometry
	if (s > geometry->GetS() + geometry->GetLength())
	{
		while (s > geometry->GetS() + geometry->GetLength() && geometry_idx_ < road->GetNumberOfGeometries() - 1)
		{
			// Move to next geometry
			geometry = road->GetGeometry(++geometry_idx_);
		}
	}
	else if (s < geometry->GetS())
	{
		while (s < geometry->GetS() && geometry_idx_ > 0)
		{
			// Move to previous geometry
			geometry = road->GetGeometry(--geometry_idx_);
		}
	}


	if (s > road->GetLength())
	{
		if (s > road->GetLength() + SMALL_NUMBER)
		{
			LOG("Position::Set Warning: s (%.2f) too large, track %d only %.2f m long\n", s, track_id_, road->GetLength());
		}
		s_ = road->GetLength();
		status_ |= static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROAD);
		return ReturnCode::ERROR_END_OF_ROAD;
	}
	else
	{
		s_ = s;
	}

	if (s_ < SMALL_NUMBER && road->GetLink(LinkType::PREDECESSOR) == nullptr ||
		s_ > road->GetLength() - SMALL_NUMBER && road->GetLink(LinkType::SUCCESSOR) == nullptr)
	{
		status_ |= static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}
	else
	{
		status_ &= ~static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}

	return ReturnCode::OK;
}

Position::ReturnCode Position::SetTrackPos(int track_id, double s, double t, bool UpdateXY)
{
	ReturnCode retval_long = SetLongitudinalTrackPos(track_id, s);

	if (retval_long != ReturnCode::ERROR_GENERIC)
	{
		t_ = t;
		Track2Lane();
		if (UpdateXY)
		{
			ReturnCode retval_lat = Track2XYZ();
			if ((int)retval_lat < 0)
			{
				return retval_lat;
			}
		}
	}
	return retval_long;
}

void Position::ForceLaneId(int lane_id)
{
	if (lane_idx_ < 0 || lane_section_idx_ < 0)
	{
		return;
	}
	// find out lateral distance between current and target lane
	Road *road = GetRoadById(GetTrackId());

	double lat_dist = road->GetLaneSectionByIdx(lane_section_idx_)->GetOffsetBetweenLanes(lane_id_, lane_id, GetS());

	lane_id_ = lane_id;
	offset_ -= lat_dist;

	// Update track position (t) as well
	Lane2Track();
}

std::string OpenDrive::ContactPointType2Str(ContactPointType type)
{
	if (type == ContactPointType::CONTACT_POINT_START)
	{
		return "PREDECESSOR";
	}
	else if (type == ContactPointType::CONTACT_POINT_END)
	{
		return "SUCCESSOR";
	}
	else if (type == ContactPointType::CONTACT_POINT_JUNCTION)
	{
		return "JUNCTION";
	}
	else if (type == ContactPointType::CONTACT_POINT_UNDEFINED)
	{
		return "UNDEFINED";
	}
	else
	{
		return "UNDEFINED";
	}
}

std::string OpenDrive::ElementType2Str(RoadLink::ElementType type)
{
	if (type == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		return "JUNCTION";
	}
	else if (type == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
	{
		return "ROAD";
	}
	else if (type == RoadLink::ElementType::ELEMENT_TYPE_UNKNOWN)
	{
		return "UNKNOWN";
	}
	else
	{
		return "UNDEFINED";
	}
}

std::string OpenDrive::LinkType2Str(LinkType type)
{
	if (type == LinkType::PREDECESSOR)
	{
		return "PREDECESSOR";
	}
	else if (type == LinkType::SUCCESSOR)
	{
		return "SUCCESSOR";
	}
	else if (type == LinkType::NONE)
	{
		return "NONE";
	}
	else
	{
		return std::string("Unknown link type: " + std::to_string(type));
	}
}

int Position::TeleportTo(Position* position)
{
	roadmanager::Position tmpPos;

	if (position->GetRelativePosition() == this)
	{
		// Special case: Relation short circuit - need to make a copy before reseting
		tmpPos.CopyRMPos(this);

		position->SetRelativePosition(&tmpPos, position->GetType());
	}

	CopyRMPos(position);

	if (position->GetRelativePosition() != nullptr)
	{
		// Resolve any relative positions
		ReleaseRelation();
	}

	if (GetRoute())   // on a route
	{
		CalcRoutePosition();
	}

	return 0;
}

Position::ReturnCode Position::MoveToConnectingRoad(RoadLink *road_link, ContactPointType &contact_point_type, double junctionSelectorAngle)
{
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	Road *next_road = 0;
	LaneSection *lane_section;
	Lane *lane;
	int new_lane_id = 0;
	ReturnCode ret_val = ReturnCode::OK;

	if (road == 0)
	{
		LOG("Invalid road id %d\n", road->GetId());
		return ReturnCode::ERROR_GENERIC;
	}

	if (road_link == 0)
	{
		LOG("Lacking link of road id %d\n", road->GetId());
		return ReturnCode::ERROR_GENERIC;
	}

	if (road_link->GetElementId() == -1)
	{
		LOG("No connecting road or junction at rid %d link_type %s", road->GetId(), OpenDrive::LinkType2Str(road_link->GetType()).c_str());
		return ReturnCode::ERROR_GENERIC;
	}

	// Get lane info from current road
	lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	if (lane_section == 0)
	{
		LOG("No lane section rid %d ls_idx %d link_type  %s", road->GetId(), lane_section_idx_, OpenDrive::LinkType2Str(road_link->GetType()).c_str());
		return ReturnCode::ERROR_GENERIC;
	}

	lane = lane_section->GetLaneByIdx(lane_idx_);
	if (lane == 0)
	{
		LOG("No lane rid %d lidx %d nlanes %d link_type %s lsecidx %d\n",
			road->GetId(), lane_idx_, lane_section->GetNumberOfLanes(), OpenDrive::LinkType2Str(road_link->GetType()).c_str(), lane_section_idx_);
		return ReturnCode::ERROR_GENERIC;
	}

	if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_ROAD)
	{
		LaneLink *lane_link = lane->GetLink(road_link->GetType());
		if (lane_link != 0)
		{
			new_lane_id = lane->GetLink(road_link->GetType())->GetId();
			if (new_lane_id == 0)
			{
				LOG("Road+ new lane id %d\n", new_lane_id);
			}
		}
		else
		{
			//LOG("No lane link from rid %d lid %d to rid %d", GetTrackId(), GetLaneId(), road_link->GetElementId());
		}
		contact_point_type = road_link->GetContactPointType();
		next_road = GetOpenDrive()->GetRoadById(road_link->GetElementId());

		ret_val = ReturnCode::ENTERED_NEW_ROAD;
	}
	else if (road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
	{
		Junction *junction = GetOpenDrive()->GetJunctionById(road_link->GetElementId());

		if (junction == 0)
		{
			LOG("Error: junction %d not existing\n", road_link->GetElementId());
			return ReturnCode::ERROR_GENERIC;
		}

		int connection_idx = 0;
		int n_connections = junction->GetNumberOfRoadConnections(road->GetId(), lane->GetId());

		if (n_connections == 0)
		{
//			LOG("No connections from road id %d lane id %d in junction %d", road->GetId(), lane->GetId(), junction->GetId());
			return ReturnCode::ERROR_GENERIC;
		}
		else if (n_connections == 1)
		{
			connection_idx = 0;
		}
		else
		{
			// find valid connecting road, if multiple choices choose either most straight one OR by random
			if (GetRoute())
			{
				// Choose direction of the route
				Route* r = GetRoute();

				// Find next road in route
				Road* outgoing_road_target = r->GetRoadAtOtherEndOfConnectingRoad(road);
				int optimal_connection_idx = -1;
				for (int i = 0; i < n_connections; i++)
				{
					LaneRoadLaneConnection lane_road_lane_connection =
						junction->GetRoadConnectionByIdx(road->GetId(), lane->GetId(), i, snapToLaneTypes_);
					Road* connecting_road = GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId());
					if (connecting_road)
					{
						Road* outgoing_road = junction->GetRoadAtOtherEndOfConnectingRoad(connecting_road, road);
						if (outgoing_road == outgoing_road_target)
						{
							connection_idx = i;
							if (connecting_road->GetId() == r->GetTrackId()) {
								optimal_connection_idx = connection_idx;
							}
						}
					}
				}
				if (optimal_connection_idx != -1) {
					connection_idx = optimal_connection_idx;
				}
			}
			else if (junctionSelectorAngle >= 0.0)
			{
				// Find the straighest link
				int best_road_index = 0;
				double min_heading_diff = 1E10; // set huge number
				for (int i = 0; i < n_connections; i++)
				{
					LaneRoadLaneConnection lane_road_lane_connection =
						junction->GetRoadConnectionByIdx(road->GetId(), lane->GetId(), i, snapToLaneTypes_);
					next_road = GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId());

					// Get a position at end of the connecting road
					Position test_pos;
					double outHeading = 0.0;
					if (lane_road_lane_connection.contact_point_ == CONTACT_POINT_START)
					{
						test_pos.SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), 0);
						outHeading = test_pos.GetHRoad();
					}
					else if (lane_road_lane_connection.contact_point_ == CONTACT_POINT_END)
					{
						test_pos.SetLanePos(next_road->GetId(), new_lane_id, 0, 0);
						outHeading = GetAngleSum(test_pos.GetHRoad(), M_PI);
					}
					else
					{
						LOG("Unexpected contact point type: %d", road_link->GetContactPointType());
					}

					// Compare heading angle difference, find smallest
					double deltaHeading = GetAngleInInterval2PI(GetAngleDifference(outHeading, GetHRoadInDrivingDirection()));
					double heading_diff = GetAbsAngleDifference(deltaHeading, junctionSelectorAngle);
					if (heading_diff < min_heading_diff)
					{
						min_heading_diff = heading_diff;
						best_road_index = i;
					}
				}
				connection_idx = best_road_index;
			}
			else  // randomize
			{
				connection_idx = (int)(n_connections * (double)(SE_Env::Inst().GetGenerator())() / (SE_Env::Inst().GetGenerator()).max());
			}
		}

		LaneRoadLaneConnection lane_road_lane_connection = junction->GetRoadConnectionByIdx(road->GetId(), lane->GetId(), connection_idx, snapToLaneTypes_);
		contact_point_type = lane_road_lane_connection.contact_point_;

		new_lane_id = lane_road_lane_connection.GetConnectinglaneId();
		next_road = GetOpenDrive()->GetRoadById(lane_road_lane_connection.GetConnectingRoadId());

		ret_val = ReturnCode::MADE_JUNCTION_CHOICE;
	}

	if (next_road == 0)
	{
		LOG("No next road\n");
		return ReturnCode::ERROR_GENERIC;
	}

	if (new_lane_id == 0)
	{
		LOG("No connection from rid %d lid %d -> rid %d eltype %d - try moving to closest lane\n",
			road->GetId(), lane->GetId(), road_link->GetElementId(), road_link->GetElementType());

		// Find closest lane on new road - by convert to track pos and then set lane offset = 0
		if (road_link->GetContactPointType() == CONTACT_POINT_START)
		{
			SetTrackPos(next_road->GetId(), 0, GetT(), false);
		}
		else if (road_link->GetContactPointType() == CONTACT_POINT_END)
		{
			SetTrackPos(next_road->GetId(), next_road->GetLength(), GetT(), false);
		}
		offset_ = 0;

		return ret_val;
	}

	double new_offset = offset_;
	if ((road_link->GetType() == LinkType::PREDECESSOR && contact_point_type == ContactPointType::CONTACT_POINT_START) ||
		(road_link->GetType() == LinkType::SUCCESSOR && contact_point_type == ContactPointType::CONTACT_POINT_END))
	{
		h_relative_ = GetAngleSum(h_relative_, M_PI);
		new_offset = -offset_;
	}

	// Find out if connecting to start or end of new road
	if (road_link->GetContactPointType() == CONTACT_POINT_START)
	{
		// Specify first (0) lane section
		SetLanePos(next_road->GetId(), new_lane_id, 0, new_offset, 0);
	}
	else if (road_link->GetContactPointType() == CONTACT_POINT_END)
	{
		// Find out and specify last lane section
		SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), new_offset, GetRoadById(road_link->GetElementId())->GetNumberOfLaneSections()-1);
	}
	else if (road_link->GetContactPointType() == CONTACT_POINT_JUNCTION && road_link->GetElementType() == RoadLink::ELEMENT_TYPE_JUNCTION)
	{
		if (contact_point_type == CONTACT_POINT_START)
		{
			SetLanePos(next_road->GetId(), new_lane_id, 0, new_offset);
		}
		else if (contact_point_type == CONTACT_POINT_END)
		{
			SetLanePos(next_road->GetId(), new_lane_id, next_road->GetLength(), new_offset);
		}
		else
		{
			LOG("Unexpected contact point: %d\n", contact_point_type);
		}
	}
	else
	{
		LOG("Unsupported contact point type %d\n", road_link->GetContactPointType());
		return ReturnCode::ERROR_GENERIC;
	}

	return ret_val;
}

double Position::DistanceToDS(double ds)
{
	// Add or subtract stepsize according to curvature and offset, in order to keep constant speed
	double curvature = GetCurvature();
	double offset = GetT();

	// Also compensate for any lane offset at current road position (if available)
	if (GetOpenDrive())
	{
		roadmanager::Road* road = GetOpenDrive()->GetRoadById(GetTrackId());
		if (road != nullptr)
		{
			offset += road->GetLaneOffset(GetS());
		}
	}

	if (abs(curvature) > SMALL_NUMBER)
	{
		// Approximate delta length by sampling curvature in current position
		if (curvature * offset > 1.0 - SMALL_NUMBER)
		{
			// Radius not large enough for offset, probably being closer to another road segment
			XYZH2TrackPos(GetX(), GetY(), GetY(), GetH(), true);
			SetHeadingRelative(GetHRelative());
			curvature = GetCurvature();
			offset = GetT();
		}
		double stepScaleFactor = 1.0 / (1.0 - curvature * offset);
		ds *= stepScaleFactor;
	}

	return ds;
}

Position::ReturnCode Position::MoveAlongS(double ds, double dLaneOffset, double junctionSelectorAngle, bool actualDistance)
{
	RoadLink *link;
	int max_links = 8;  // limit lookahead through junctions/links
	ContactPointType contact_point_type;
	ReturnCode ret_val = ReturnCode::OK;

	if (actualDistance)
	{
		ds = DistanceToDS(ds);
	}

	if (type_ == PositionType::RELATIVE_LANE)
	{
		// Create a temporary position to evaluate in relative lane coordinates
		Position pos = *this->rel_pos_;

		// First move position along s
		pos.MoveAlongS(ds);

		// Then move laterally
		pos.SetLanePos(pos.track_id_, pos.lane_id_ + this->lane_id_, pos.s_, pos.offset_ + this->offset_);

		this->x_ = pos.x_;
		this->y_ = pos.y_;
		this->z_ = pos.z_;
		this->h_ = pos.h_;
		this->p_ = pos.p_;
		this->r_ = pos.r_;

		return ret_val;
	}

	if (GetOpenDrive()->GetNumOfRoads() == 0 || track_idx_ < 0)
	{
		// No roads available or current track undefined
		return Position::ReturnCode::OK;
	}

	double s_stop = 0;
	double ds_signed = ds * (IsAngleForward(GetHRelative()) ? 1 : -1);
	double signed_dLaneOffset = dLaneOffset;

	// move from road to road until ds-value is within road length or maximum of connections has been crossed
	for (int i = 0; i < max_links; i++)
	{
		if (s_ + ds_signed > GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength())
		{
			// Calculate remaining s-value once we moved to the connected road
			ds_signed = s_ + ds_signed - GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength();
			link = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLink(SUCCESSOR);

			// register s-value at end of the road, to be used in case of bad connection
			s_stop = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLength();
		}
		else if (s_ + ds_signed < 0)
		{
			// Calculate remaining s-value once we moved to the connected road
			ds_signed = s_ + ds_signed;
			link = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetLink(PREDECESSOR);

			// register s-value at end of the road, to be used in case of bad connection
			s_stop = 0;
		}
		else  // New position is within current track (road)
		{
			break;
		}

		// If link is OK then move to the start- or endpoint of the connected road, depending on contact point
		if (link == 0 || link->GetElementId() == -1 ||
			static_cast<int>((ret_val = MoveToConnectingRoad(link, contact_point_type, junctionSelectorAngle))) < 0)
		{
			// Failed to find a connection, stay at end of current road
			SetLanePos(track_id_, lane_id_, s_stop, offset_);

			status_ |= static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROAD);
			return ReturnCode::ERROR_END_OF_ROAD;
		}

		// Adjust sign of ds based on connection point
		if (contact_point_type == ContactPointType::CONTACT_POINT_END)
		{
			ds_signed = -fabs(ds_signed);
			signed_dLaneOffset = -dLaneOffset;
		}
		else
		{
			ds_signed = fabs(ds_signed);
			signed_dLaneOffset = dLaneOffset;
		}
	}

	// Finally, update the position with the adjusted s and offset values
	Road* road = GetOpenDrive()->GetRoadById(track_id_);

	ReturnCode ret_val2 = SetLanePos(track_id_, lane_id_, s_ + ds_signed, offset_ + signed_dLaneOffset);

	if (ret_val2 >= ReturnCode::OK)
	{
		// Check if lane has narrowed down to zero width
		LaneInfo li;
		if (road->GetLaneInfoByS(GetS(), lane_section_idx_, lane_id_, li, snapToLaneTypes_) != 0)
		{
			ret_val = ReturnCode::ERROR_GENERIC;
		}
		else if (road->GetLaneWidthByS(GetS(), li.lane_id_) < SMALL_NUMBER)
		{
			double offset = 0;
			int old_lane_id = lane_id_;
			int new_lane_idx = road->GetLaneSectionByIdx(li.lane_section_idx_)->GetClosestLaneIdx(GetS(), GetT(), offset, true, snapToLaneTypes_);
			int new_lane_id = road->GetLaneSectionByIdx(li.lane_section_idx_)->GetLaneByIdx(new_lane_idx)->GetId();
			SetLanePos(track_id_, new_lane_id, GetS(), 0);
			LOG("Lane %d on road %d is or became zero width, moved to closest available lane: %d", old_lane_id, road->GetId(), GetLaneId());
		}
	}
	else
	{
		ret_val = ret_val2;  // errors override status codes from e.g. MoveToConnectingRoad
	}

	if (s_ < SMALL_NUMBER && road->GetLink(LinkType::PREDECESSOR) == nullptr  ||
		s_ > road->GetLength() - SMALL_NUMBER && road->GetLink(LinkType::SUCCESSOR) == nullptr)
	{
		status_ |= static_cast<int>(Position::PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}
	else
	{
		status_ &= ~static_cast<int>(Position::PositionStatusMode::POS_STATUS_END_OF_ROAD);
	}

	return ret_val;
}

Position::ReturnCode Position::SetLanePos(int track_id, int lane_id, double s, double offset, int lane_section_idx)
{
	offset_ = offset;
	ReturnCode retvalue = ReturnCode::OK;

	if ((retvalue = SetLongitudinalTrackPos(track_id, s)) == ReturnCode::ERROR_GENERIC)
	{
		lane_id_ = lane_id;
		offset_ = offset;
		return retvalue;
	}

	Road *road = GetOpenDrive()->GetRoadById(track_id);
	if (road == 0)
	{
		LOG("Position::Set Error: track %d not available", track_id);
		lane_id_ = lane_id;
		offset_ = offset;
		return ReturnCode::ERROR_GENERIC;
	}

	if (lane_id != lane_id_ && lane_section_idx == -1)
	{
		// New lane ID might indicate a discreet jump to a new, distant position, reset lane section, if not specified in func parameter)
		lane_section_idx = road->GetLaneSectionIdxByS(s);
	}

	LaneSection *lane_section = 0;
	if (lane_section_idx > -1)  // If lane section was specified or reset
	{
		lane_section_idx_ = lane_section_idx;
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		lane_id_ = lane_id;
	}
	else  // Find LaneSection and info according to s
	{
		LaneInfo li;
		if (road->GetLaneInfoByS(GetS(), lane_section_idx_, lane_id_, li, snapToLaneTypes_) != 0)
		{
			retvalue = ReturnCode::ERROR_GENERIC;
		}

		lane_section_idx_ = li.lane_section_idx_;
		lane_id_ = li.lane_id_;

		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
	}

	if (lane_section != 0)
	{
		lane_idx_ = lane_section->GetLaneIdxById(lane_id_);
		if (lane_idx_ == -1)
		{
			LOG("lane_idx %d fail for lane id %d\n", lane_idx_, lane_id_);
			lane_idx_ = 0;
		}
	}
	else
	{
		LOG("Position::Set (lanepos) Error - lanesection NULL lsidx %d rid %d lid %d",
			lane_section_idx_, road->GetId(), lane_id_);
	}

	Lane2Track();
	Track2XYZ();

	return retvalue;
}

void Position::SetLaneBoundaryPos(int track_id, int lane_id, double s, double offset, int lane_section_idx)
{
	offset_ = offset;
	int old_lane_id = lane_id_;
	int old_track_id = track_id_;
	ReturnCode retval;

	if ((int)(retval = SetLongitudinalTrackPos(track_id, s)) < 0)
	{
		lane_id_ = lane_id;
		offset_ = offset;
		return;
	}

	Road *road = GetOpenDrive()->GetRoadById(track_id);
	if (road == 0)
	{
		LOG("Position::Set Error: track %d not available", track_id);
		lane_id_ = lane_id;
		offset_ = offset;
		return;
	}

	if (lane_id != lane_id_ && lane_section_idx == -1)
	{
		// New lane ID might indicate a discreet jump to a new, distant position, reset lane section, if not specified in func parameter)
		lane_section_idx = road->GetLaneSectionIdxByS(s);
	}

	LaneSection *lane_section = 0;
	if (lane_section_idx > -1)  // If lane section was specified or reset
	{
		lane_section_idx_ = lane_section_idx;
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		lane_id_ = lane_id;
	}
	else  // Find LaneSection and info according to s
	{
		LaneInfo li;
		if (road->GetLaneInfoByS(GetS(), lane_section_idx_, lane_id_, li, snapToLaneTypes_) == 0)
		{
			lane_section_idx_ = li.lane_section_idx_;
			lane_id_ = li.lane_id_;

			lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
		}
	}

	if (lane_section != 0)
	{
		lane_idx_ = lane_section->GetLaneIdxById(lane_id_);
		if (lane_idx_ == -1)
		{
			LOG("lane_idx %d fail for lane id %d\n", lane_idx_, lane_id_);
			lane_idx_ = 0;
		}
	}
	else
	{
		LOG("Position::Set (lanepos) Error - lanesection NULL lsidx %d rid %d lid %d",
			lane_section_idx_, road->GetId(), lane_id_);
	}

	// Check road direction when on new track
	if (old_lane_id != 0 && lane_id_ != 0 && track_id_ != old_track_id && SIGN(lane_id_) != SIGN(old_lane_id))
	{
		h_relative_ = GetAngleSum(h_relative_, M_PI);
	}

	// If moved over to opposite driving direction, then turn relative heading 180 degrees
	//if (old_lane_id != 0 && lane_id_ != 0 && SIGN(lane_id_) != SIGN(old_lane_id))
	//{
	//	h_relative_ = GetAngleSum(h_relative_, M_PI);
	//}

	//Lane2Track();
	LaneBoundary2Track();
	Track2XYZ();

	return;
}

void Position::SetRoadMarkPos(int track_id, int lane_id, int roadmark_idx, int roadmarktype_idx, int roadmarkline_idx, double s, double offset, int lane_section_idx)
{
	offset_ = offset;
	int old_lane_id = lane_id_;
	int old_track_id = track_id_;

	Road* road = GetOpenDrive()->GetRoadById(track_id);
	if (road == 0)
	{
		LOG("Position::Set Error: track %d not available", track_id);
		lane_id_ = lane_id;
		offset_ = offset;
		return;
	}

	if (s > road->GetLength())
	{
		// Truncate road mark point to road length
		s = road->GetLength();
	}

	if ((int)SetLongitudinalTrackPos(track_id, s) < 0)
	{
		lane_id_ = lane_id;
		offset_ = offset;
		roadmark_idx_ = roadmark_idx;
		roadmarktype_idx_ = roadmarktype_idx;
		roadmarkline_idx_ = roadmarkline_idx;
		return;
	}


	if (lane_id != lane_id_ && lane_section_idx == -1)
	{
		// New lane ID might indicate a discreet jump to a new, distant position, reset lane section, if not specified in func parameter)
		lane_section_idx = road->GetLaneSectionIdxByS(s);
	}

	LaneSection *lane_section = 0;
	if (lane_section_idx > -1)  // If lane section was specified or reset
	{
		lane_section_idx_ = lane_section_idx;
		lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

		lane_id_ = lane_id;
	}
	else  // Find LaneSection and info according to s
	{
		LaneInfo li;
		if (road->GetLaneInfoByS(GetS(), lane_section_idx_, lane_id_, li, snapToLaneTypes_) == 0)
		{
			lane_section_idx_ = li.lane_section_idx_;
			lane_id_ = li.lane_id_;

			lane_section = road->GetLaneSectionByIdx(lane_section_idx_);
		}
	}

	if (lane_section != 0)
	{
		lane_idx_ = lane_section->GetLaneIdxById(lane_id_);
		if (lane_idx_ == -1)
		{
			LOG("lane_idx %d fail for lane id %d\n", lane_idx_, lane_id_);
			lane_idx_ = 0;
		}
	}
	else
	{
		LOG("Position::Set (lanepos) Error - lanesection NULL lsidx %d rid %d lid %d",
			lane_section_idx_, road->GetId(), lane_id_);
	}

	// Check road direction when on new track
	if (old_lane_id != 0 && lane_id_ != 0 && track_id_ != old_track_id && SIGN(lane_id_) != SIGN(old_lane_id))
	{
		h_relative_ = GetAngleSum(h_relative_, M_PI);
	}

	Lane *lane = lane_section->GetLaneByIdx(lane_idx_);
	if (lane != 0)
	{
		roadmark_idx_ = roadmark_idx;
	}

	LaneRoadMark *lane_roadmark = lane->GetLaneRoadMarkByIdx(roadmark_idx_);
	if (lane_roadmark != 0)
	{
		s_ = MIN(s_, road->GetLength());
	}
	else
	{
		LOG("roadmark_idx_ %d fail for lane id %d\n", roadmark_idx_, lane_idx_);
		roadmark_idx_ = 0;
	}

	if (lane_roadmark->GetNumberOfRoadMarkTypes() != 0)
	{
		roadmarktype_idx_ = roadmarktype_idx;
	}
	else
	{
		roadmarktype_idx_ = 0;
	}

	LaneRoadMarkType *lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(roadmarktype_idx_);
	if (lane_roadmarktype != 0)
	{
		roadmarkline_idx_ = roadmarkline_idx;
		LaneRoadMarkTypeLine *lane_roadmarktypeline = lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(roadmarkline_idx_);
		if (lane_roadmarktypeline != 0)
		{
			s_ = MIN(s_, road->GetLength());
		}
		else
		{
			LOG("roadmarktypeline_idx_ %d fail for roadmarktype_idx %d\n", roadmarkline_idx_, roadmarktype_idx_);
			roadmarkline_idx_ = 0;
		}
	}
	else
	{
		LOG("roadmarktype_idx_ %d fail for roadmark_idx %d\n", roadmarktype_idx_, roadmark_idx_);
		roadmarkline_idx_ = 0;
	}


	RoadMark2Track();
	Track2XYZ();
}

int Position::SetInertiaPos(double x, double y, double z, double h, double p, double r, bool updateTrackPos)
{
	x_ = x;
	y_ = y;
	z_ = z;

	if (updateTrackPos)
	{
		XYZ2Track();
	}

	// Now when road orientation is known, call functions for
	// updating angles both absolute and relative the road
	SetHeading(h);
	SetPitch(p);
	SetRoll(r);

	EvaluateOrientation();

	return 0;
}

int Position::SetInertiaPos(double x, double y, double h, bool updateTrackPos)
{
	x_ = x;
	y_ = y;

	if (updateTrackPos)
	{
		XYZ2Track();
	}

	// Now when road orientation is known, call functions for
	// updating angles both absolute and relative the road
	SetHeading(h);

	EvaluateOrientation();

	if (align_z_ == ALIGN_MODE::ALIGN_SOFT)
	{
		SetZRelative(z_relative_);
	}
	else if (align_z_ == ALIGN_MODE::ALIGN_HARD)
	{
		SetZ(z_road_);
	}

	return 0;
}

void Position::SetHeading(double heading)
{
	h_ = heading;
	h_relative_ = GetAngleInInterval2PI(GetAngleDifference(h_, h_road_)); // Something wrong with -angles
}

void Position::SetHeadingRelative(double heading)
{
	h_relative_ = GetAngleInInterval2PI(heading);
	h_ = GetAngleSum(h_road_, h_relative_);
}

void Position::SetHeadingRelativeRoadDirection(double heading)
{
	if (h_relative_ > M_PI_2 && h_relative_ < 3 * M_PI_2)
	{
		// Driving towards road direction
		h_relative_ = GetAngleInInterval2PI(heading + M_PI);
	}
	else
	{
		h_relative_ = GetAngleInInterval2PI(heading);
	}
	h_ = GetAngleSum(h_road_, h_relative_);
}

void Position::SetRoll(double roll)
{
	r_ = roll;
	r_relative_ = GetAngleInInterval2PI(GetAngleDifference(r_, r_road_));
}

void Position::SetRollRelative(double roll)
{
	r_relative_ = GetAngleInInterval2PI(roll);
	r_ = GetAngleSum(r_road_, r_relative_);
}

void Position::SetPitch(double pitch)
{
	p_ = pitch;
	p_relative_ = GetAngleInInterval2PI(GetAngleDifference(p_, p_road_));
}

void Position::SetPitchRelative(double pitch)
{
	p_relative_ = GetAngleInInterval2PI(pitch);
	p_ = GetAngleSum(p_road_, p_relative_);
}

void Position::SetZ(double z)
{
	z_relative_ = z - z_road_;
	z_ = z;
}

void Position::SetZRelative(double z)
{
	z_relative_ = z;
	z_ = z_road_ + z_relative_;
}

void Position::EvaluateOrientation()
{
	if (align_h_ != ALIGN_MODE::ALIGN_NONE || align_p_ != ALIGN_MODE::ALIGN_NONE || align_r_ != ALIGN_MODE::ALIGN_NONE)
	{
		R0R12EulerAngles(
			align_h_ != ALIGN_MODE::ALIGN_NONE ? GetHRoad() : 0.0,
			align_p_ != ALIGN_MODE::ALIGN_NONE ? GetPRoad() : 0.0,
			align_r_ != ALIGN_MODE::ALIGN_NONE ? GetRRoad() : 0.0,
			align_h_ != ALIGN_MODE::ALIGN_HARD ? GetHRelative() : 0.0,
			align_p_ != ALIGN_MODE::ALIGN_HARD ? GetPRelative() : 0.0,
			align_r_ != ALIGN_MODE::ALIGN_HARD ? GetRRelative() : 0.0,
			h_,
			p_,
			r_
		);
		h_ = GetAngleInInterval2PI(h_);
		p_ = GetAngleInInterval2PI(p_);
		r_ = GetAngleInInterval2PI(r_);
	}
	else
	{
		h_ = GetHRelative();
		p_ = GetPRelative();
		r_ = GetRRelative();
	}
}

double Position::GetCurvature()
{
	Geometry *geom = GetOpenDrive()->GetGeometryByIdx(track_idx_, geometry_idx_);

	if (geom)
	{
		return geom->EvaluateCurvatureDS(GetS() - geom->GetS());
	}
	else
	{
		return 0;
	}
}

int Position::GetDrivingDirectionRelativeRoad() const
{
	if (GetTrackId() >= 0 && GetRoadById(GetTrackId()) != nullptr)
	{
		// Consider road rule (left hand or right hand traffic)
		if (GetLaneId() < 0 && GetRoadById(GetTrackId())->GetRule() == Road::RoadRule::LEFT_HAND_TRAFFIC ||
			GetLaneId() > 0 && GetRoadById(GetTrackId())->GetRule() == Road::RoadRule::RIGHT_HAND_TRAFFIC)
		{
			return -1;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return GetLaneId() > 0 ? -1 : 1;
	}
}

double Position::GetHRoadInDrivingDirection() const
{
	return GetAngleSum(GetHRoad(), GetDrivingDirectionRelativeRoad() < 0 ? M_PI : 0.0);
}

double Position::GetPRoadInDrivingDirection()
{
	return GetPRoad() * GetDrivingDirectionRelativeRoad();
}

double Position::GetHRelativeDrivingDirection() const
{
	return GetAngleDifference(h_, GetDrivingDirection());
}

double Position::GetSpeedLimit()
{
	double speed_limit = 70 / 3.6;  // some default speed
	Road *road = GetOpenDrive()->GetRoadByIdx(track_idx_);

	if (road)
	{
		speed_limit = road->GetSpeedByS(s_);

		if (speed_limit < SMALL_NUMBER)
		{
			// No speed limit defined, set a value depending on number of lanes
			speed_limit = GetOpenDrive()->GetRoadByIdx(track_idx_)->GetNumberOfDrivingLanesSide(GetS(), SIGN(GetLaneId())) > 1 ? 120 / 3.6 : 60 / 3.6;
		}
	}

	return speed_limit;
}

double Position::GetDrivingDirection() const
{
	double x, y, h;
	Geometry *geom = GetOpenDrive()->GetGeometryByIdx(track_idx_, geometry_idx_);

	if (!geom)
	{
		return h_;
	}

	geom->EvaluateDS(GetS() - geom->GetS(), &x, &y, &h);

	// adjust 180 degree according to side of road
	if (GetLaneId() > 0)  // Left side of road reference line
	{
		h = GetAngleSum(h, M_PI);
	}

	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road != nullptr && road->GetRule() == Road::RoadRule::LEFT_HAND_TRAFFIC)
	{
		h = GetAngleSum(h, M_PI);
	}

	return(h);
}

double Position::GetVelLat()
{
	double vlat = 0.0;
	double vlong = 0.0;
	RotateVec2D(GetVelX(), GetVelY(), -GetH(), vlong, vlat);

	return vlat;
}

double Position::GetVelLong()
{
	double vlat = 0.0;
	double vlong = 0.0;
	RotateVec2D(GetVelX(), GetVelY(), -GetH(), vlong, vlat);

	return vlong;
}

void Position::GetVelLatLong(double &vlat, double &vlong)
{
	RotateVec2D(GetVelX(), GetVelY(), -GetH(), vlong, vlat);
}

double Position::GetAccLat()
{
	double alat = 0.0;
	double along = 0.0;
	RotateVec2D(GetAccX(), GetAccY(), -GetH(), along, alat);

	return alat;
}

double Position::GetAccLong()
{
	double alat = 0.0;
	double along = 0.0;
	RotateVec2D(GetAccX(), GetAccY(), -GetH(), along, alat);

	return along;
}

void Position::GetAccLatLong(double& alat, double& along)
{
	RotateVec2D(GetAccX(), GetAccY(), -GetH(), along, alat);
}

double Position::GetVelT()
{
	double vs = 0.0;
	double vt = 0.0;
	RotateVec2D(GetVelX(), GetVelY(), -GetHRoad(), vs, vt);

	return vt;
}

double Position::GetVelS()
{
	double vs = 0.0;
	double vt = 0.0;
	RotateVec2D(GetVelX(), GetVelY(), -GetHRoad(), vs, vt);

	return vs;
}

void Position::GetVelTS(double& vt, double& vs)
{
	RotateVec2D(GetVelX(), GetVelY(), -GetHRoad(), vs, vt);
}

double Position::GetAccT()
{
	double at = 0.0;
	double as = 0.0;
	RotateVec2D(GetAccX(), GetAccY(), -GetHRoad(), as, at);

	return at;
}

double Position::GetAccS()
{
	double at = 0.0;
	double as = 0.0;
	RotateVec2D(GetAccX(), GetAccY(), -GetHRoad(), as, at);

	return as;
}

void Position::GetAccTS(double& at, double& as)
{
	RotateVec2D(GetAccX(), GetAccY(), -GetHRoad(), as, at);
}

void Position::CopyRMPos(Position *from)
{
	// Use a temporary pos object to preserve some fields
	Position tmp_pos = *this;

	*this = *from;
	route_ = tmp_pos.route_;
	velX_ = tmp_pos.velX_;
	velY_ = tmp_pos.velY_;
	velZ_ = tmp_pos.velZ_;
	accX_ = tmp_pos.accX_;
	accY_ = tmp_pos.accY_;
	accZ_ = tmp_pos.accZ_;
	h_rate_ = tmp_pos.h_rate_;
	p_rate_ = tmp_pos.p_rate_;
	r_rate_ = tmp_pos.r_rate_;
	h_acc_ = tmp_pos.h_acc_;
	p_acc_ = tmp_pos.p_acc_;
	r_acc_ = tmp_pos.r_acc_;
}

void Position::PrintTrackPos()
{
	LOG("	Track pos: (road_id %d, s %.2f, t %.2f, h %.2f)", track_id_, s_, t_, h_);
}

void Position::PrintLanePos()
{
	LOG("	Lane pos: (road_id %d, lane_id %d, s %.2f, offset %.2f, h %.2f)", track_id_, lane_id_, s_, offset_, h_);
}

void Position::PrintInertialPos()
{
	LOG("	Inertial pos: (x %.2f, y %.2f, z %.2f, h %.2f, p %.2f, r %.2f)", x_, y_, z_, h_, p_, r_);
}

void Position::Print()
{
	LOG("Pos(%.2f, %.2f, %.2f) Rot(%.2f, %.2f, %.2f) roadId %d laneId %d offset %.2f t %.2f",
		GetX(), GetY(), GetZ(), GetH(), GetP(), GetR(), GetTrackId(), GetLaneId(), GetOffset(), GetT());
}

void Position::PrintXY()
{
	LOG("%.2f, %.2f\n", x_, y_);
}

bool Position::IsOffRoad()
{
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road)
	{
		// Check whether outside road width
		if (fabs(t_) > road->GetWidth(GetS(), SIGN(t_), ~Lane::LaneType::LANE_TYPE_NONE))
		{
			return true;
		}
	}

	return false;
}

bool Position::IsInJunction()
{
	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road)
	{
		return road->GetJunction() != -1;
	}

	return false;
}

int Position::GetNumberOfRoadsOverlapping()
{
	XYZH2TrackPos(GetX(), GetY(), GetZ(), GetH(), false, -1, true);

	return static_cast<int>(overlapping_roads.size());
}

int Position::GetOverlappingRoadId(int index)
{
	if (overlapping_roads.size() == 0 || index >= overlapping_roads.size() || index < 0)
	{
		return -1;
	}

	return overlapping_roads[index];
}

double Position::getRelativeDistance(double targetX, double targetY, double &x, double &y) const
{
	// Calculate diff vector from current to target
	double diff_x, diff_y;

	diff_x = targetX - GetX();
	diff_y = targetY - GetY();

	// Align with closest road driving direction
	double hAlign = 0.0;
	if (GetHRelative() > M_PI_2 && GetHRelative() < 3 * M_PI_2)
	{
		hAlign = M_PI;
	}
	hAlign = -GetAngleSum(GetHRoad(), hAlign);
	x = diff_x * cos(hAlign) - diff_y * sin(hAlign);
	y = diff_x * sin(hAlign) + diff_y * cos(hAlign);

	// Now just check whether diff vector X-component is less than 0 (behind current)
	int sign = x > 0 ? 1 : -1;

	// Return length of dist vector
	return sign * sqrt((x * x) + (y * y));
}

int Position::CalcRoutePosition()
{
	if (route_ == 0 || !route_->IsValid())
	{
		return -1;
	}

	if ((int)route_->SetTrackS(GetTrackId(), GetS()) >= 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

int Position::SetRoute(Route *route)
{
	route_ = route;

	// Also find out current position in terms of route position
	return CalcRoutePosition();
}

void Position::SetTrajectory(RMTrajectory* trajectory)
{
	trajectory_ = trajectory;

	// Reset trajectory S value
	s_trajectory_ = 0;
}

bool Position::Delta(Position* pos_b, PositionDiff &diff, bool bothDirections, double maxDist) const
{
	double dist = 0;
	bool found;

	RoadPath *path = new RoadPath(this, pos_b);
	found = (path->Calculate(dist, bothDirections, maxDist) == 0);
	if (found)
	{
		int laneIdB = pos_b->GetLaneId();
		double tB = pos_b->GetT();

		if (path->visited_.size() > 0) {
			RoadPath::PathNode* lastNode = path->visited_.back();
			RoadPath::PathNode* firstNode = path->firstNode_;
			if (firstNode == nullptr)
			{
				LOG("Missing first node in path");
				return false;
			}
			if (lastNode == nullptr)
			{
				LOG("Missing last node in path");
				return false;
			}
			bool isPathForward = firstNode->link->GetType() == LinkType::SUCCESSOR;
			bool isPathBackward = firstNode->link->GetType() == LinkType::PREDECESSOR;
			bool isConnectedToEnd = lastNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END;
			bool isConnectedToStart = lastNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_START;
			bool isConnectedToJunction = lastNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_JUNCTION;
			if (isConnectedToJunction) {
				Road* lastNodeRoad = lastNode->fromRoad;
				Road* lastRoad = GetRoadById(pos_b->GetTrackId());
				RoadLink* linkPred = lastRoad->GetLink(LinkType::PREDECESSOR);
				RoadLink* linkSucc = lastRoad->GetLink(LinkType::SUCCESSOR);
				isConnectedToStart = linkPred && lastNodeRoad && lastNodeRoad->GetId() == linkPred->GetElementId();
				isConnectedToEnd   = linkSucc && lastNodeRoad && lastNodeRoad->GetId() == linkSucc->GetElementId();
			}

			bool isHeadToHead = isPathForward && isConnectedToEnd;
			bool isToeToToe = isPathBackward && isConnectedToStart;

			// If start and end roads are oppotite directed, inverse one side for delta calculations
			if (isHeadToHead || isToeToToe) {
				laneIdB = -laneIdB;
				tB = -tB;
			}
		}

		// calculate delta lane id and lateral position
		diff.dLaneId = -SIGN(GetLaneId()) * (laneIdB - GetLaneId());
		diff.dt = -SIGN(GetLaneId()) * (tB - GetT());

		diff.ds = dist;

#if 0   // Change to 1 to print some info on stdout - e.g. for debugging
		printf("Dist %.2f Path (reversed): %d", dist, pos_b.GetTrackId());
		if (path->visited_.size() > 0)
		{
			RoadPath::PathNode* node = path->visited_.back();

			while (node)
			{
				if (node->fromRoad != 0)
				{
					printf(" <- %d", node->fromRoad->GetId());
				}
				node = node->previous;
			}
		}
		printf("\n");
#endif
	}
	else  // no valid route found
	{
		diff.dLaneId = 0;
		diff.ds = LARGE_NUMBER;
		diff.dt = LARGE_NUMBER;
	}

	getRelativeDistance(pos_b->GetX(), pos_b->GetY(), diff.dx, diff.dy);

	delete path;

	return found;
}

int Position::Distance(Position* pos_b, CoordinateSystem cs, RelativeDistanceType relDistType, double& dist, double maxDist)
{
	// Handle/convert depricated value
	if (relDistType == RelativeDistanceType::REL_DIST_CARTESIAN)
	{
		relDistType = RelativeDistanceType::REL_DIST_EUCLIDIAN;
	}

	if (relDistType == RelativeDistanceType::REL_DIST_EUCLIDIAN)
	{
		double dx, dy;
		dist = getRelativeDistance(pos_b->GetX(), pos_b->GetY(), dx, dy);
	}
	else if (relDistType == RelativeDistanceType::REL_DIST_LATERAL || relDistType == RelativeDistanceType::REL_DIST_LONGITUDINAL)
	{
		if (cs == CoordinateSystem::CS_LANE)
		{
			LOG_ONCE("Lane coordinateSystem not supported yet. Falling back to Road coordinate system.");
			cs = CoordinateSystem::CS_ROAD;
		}

		if (cs == CoordinateSystem::CS_ROAD)
		{
			PositionDiff diff;
			bool routeFound = Delta(pos_b, diff, true, maxDist);
			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? diff.dt : diff.ds;
			if (routeFound == false)
			{
				return -1;
			}
		}
		else if (cs == CoordinateSystem::CS_ENTITY)
		{
			double dx, dy;

			getRelativeDistance(pos_b->GetX(), pos_b->GetY(), dx, dy);

			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? dy : dx;
		}
		else if (cs == CoordinateSystem::CS_TRAJECTORY)
		{
			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? GetTrajectoryT() : GetTrajectoryS();
		}
	}
	else
	{
		LOG("Unhandled case: cs %d reDistType %d freeSpace false\n", cs, relDistType);
		return -1;
	}

	return 0;
}

int Position::Distance(double x, double y, CoordinateSystem cs, RelativeDistanceType relDistType, double& dist, double maxDist)
{
	// Handle/convert depricated value
	if (relDistType == RelativeDistanceType::REL_DIST_CARTESIAN)
	{
		relDistType = RelativeDistanceType::REL_DIST_EUCLIDIAN;
	}

	if (relDistType == RelativeDistanceType::REL_DIST_EUCLIDIAN)
	{
		double dx, dy;
		dist = getRelativeDistance(x, y, dx, dy);
	}
	else if (relDistType == RelativeDistanceType::REL_DIST_LATERAL || relDistType == RelativeDistanceType::REL_DIST_LONGITUDINAL)
	{
		if (cs == CoordinateSystem::CS_LANE)
		{
			LOG_ONCE("Lane coordinateSystem not supported yet. Falling back to Road coordinate system.");
			cs = CoordinateSystem::CS_ROAD;
		}

		if (cs == CoordinateSystem::CS_ROAD)
		{
			Position pos_b(x, y, 0, 0, 0, 0);
			PositionDiff diff;
			bool routeFound = Delta(&pos_b, diff, true, maxDist);
			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? diff.dt : diff.ds;
			if (routeFound == false)
			{
				return -1;
			}
		}
		else if (cs == CoordinateSystem::CS_ENTITY)
		{
			double dx, dy;

			getRelativeDistance(x, y, dx, dy);

			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? dy : dx;
		}
		else if (cs == CoordinateSystem::CS_TRAJECTORY)
		{
			dist = relDistType == RelativeDistanceType::REL_DIST_LATERAL ? GetTrajectoryT() : GetTrajectoryS();
		}
	}
	else
	{
		LOG("Unhandled case: cs %d reDistType %d freeSpace false\n", cs, relDistType);
		return -1;
	}

	return 0;
}

bool Position::IsAheadOf(Position target_position)
{
	// Calculate diff vector from current to target
	double diff_x, diff_y;
	double diff_x0;

	diff_x = target_position.GetX() - GetX();
	diff_y = target_position.GetY() - GetY();

	// Compensate for current heading (rotate so that current heading = 0)
	// Only x component needed
	diff_x0 = diff_x * cos(-GetH()) - diff_y * sin(-GetH());

	// Now just check whether diff vector X-component is less than 0 (behind current)
	return(diff_x0 < 0);
}

int Position::GetRoadLaneInfo(RoadLaneInfo *data)
{
	double curvature = GetCurvature();
	if (fabs(curvature) > SMALL_NUMBER)
	{
		double radius = 1.0 / curvature;
		radius -= GetT(); // curvature positive in left curves, lat_offset positive left of reference lane
		data->curvature = (1.0 / radius);
	}
	else
	{
		// curvature close to zero (straight segment), radius infitite - curvature the same in all lanes
		data->curvature = curvature;
	}

	data->pos[0] = GetX();
	data->pos[1] = GetY();
	data->pos[2] = GetZRoad();
	data->heading = GetHRoad();
	data->pitch = GetPRoad();
	data->roll = GetRRoad();
	data->laneId = GetLaneId();
	data->laneOffset = GetOffset();
	data->roadId = GetTrackId();
	data->junctionId = GetJunctionId();
	data->t = GetT();
	data->s = GetS();

	// Then find out the width of the lane at current s-value
	Road *road = GetRoadById(GetTrackId());
	if (road)
	{
		data->width = road->GetLaneWidthByS(GetS(), GetLaneId());
		data->speed_limit = road->GetSpeedByS(GetS());
	}

	return 0;
}

int Position::GetRoadLaneInfo(double lookahead_distance, RoadLaneInfo *data, LookAheadMode lookAheadMode)
{
	Position target(*this);  // Make a copy of current position

	if (lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_ROAD_CENTER)
	{
		// Look along reference lane requested, move pivot position to t=0 plus a small number in order to
		// fall into the right direction
		target.SetTrackPos(target.GetTrackId(), target.GetS(), SMALL_NUMBER * SIGN(GetLaneId()));
	}
	else if (lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER)
	{
		// Look along current lane center requested, move pivot position accordingly
		target.SetLanePos(target.GetTrackId(), target.GetLaneId(), target.GetS(), 0);
	}

	if (fabs(lookahead_distance) > SMALL_NUMBER)
	{
		if ((int)target.MoveAlongS(lookahead_distance, 0.0, 0.0) < 0)
		{
			return -1;
		}
	}

	target.GetRoadLaneInfo(data);

	return 0;
}

int Position::CalcProbeTarget(Position *target, RoadProbeInfo *data)
{
	int retval = target->GetRoadLaneInfo(&data->road_lane_info);

	if (retval == 0)
	{
		// find out local x, y, z
		double diff_x = target->GetX() - GetX();
		double diff_y = target->GetY() - GetY();
		double diff_z = target->GetZRoad() - GetZRoad();

		data->relative_pos[0] = diff_x * cos(-GetH()) - diff_y * sin(-GetH());
		data->relative_pos[1] = diff_x * sin(-GetH()) + diff_y * cos(-GetH());
		data->relative_pos[2] = diff_z;

#if 0
		// for validation
		data->global_pos[0] = GetX() + data->local_pos[0] * cos(GetH()) - data->local_pos[1] * sin(GetH());
		data->global_pos[1] = GetY() + data->local_pos[0] * sin(GetH()) + data->local_pos[1] * cos(GetH());
		data->global_pos[2] = GetZ() + data->local_pos[2];
#endif

		// Calculate angle - by dot product
		if (fabs(data->relative_pos[0]) < SMALL_NUMBER && fabs(data->relative_pos[1]) < SMALL_NUMBER && fabs(data->relative_pos[2]) < SMALL_NUMBER)
		{
			data->relative_h = GetH();
		}
		else
		{
			double dot_prod =
				(data->relative_pos[0] * 1.0 + data->relative_pos[1] * 0.0) /
				sqrt(data->relative_pos[0] * data->relative_pos[0] + data->relative_pos[1] * data->relative_pos[1]);
			data->relative_h = SIGN(data->relative_pos[1]) * acos(dot_prod);
		}
	}

	return retval;
}

Position::ReturnCode Position::GetProbeInfo(double lookahead_distance, RoadProbeInfo *data, LookAheadMode lookAheadMode)
{
	ReturnCode retval = ReturnCode::OK;

	if (GetOpenDrive()->GetNumOfRoads() == 0)
	{
		return ReturnCode::ERROR_GENERIC;
	}
	Position target(*this);  // Make a copy of current position

	if (lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_ROAD_CENTER)
	{
		// Look along reference lane requested, move pivot position to t=0 plus a small number in order to
		// fall into the right direction
		retval = target.SetTrackPos(target.GetTrackId(), target.GetS(), SMALL_NUMBER * SIGN(GetLaneId()));
	}
	else if (lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER)
	{
		// Look along current lane center requested, move pivot position accordingly
		retval = target.SetLanePos(target.GetTrackId(), target.GetLaneId(), target.GetS(), 0);
	}

	if (fabs(lookahead_distance) > SMALL_NUMBER)
	{

		if (target.route_ && target.route_->IsValid())
		{
			retval = target.MoveRouteDS(lookahead_distance,
				lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
		}
		else
		{
			retval = target.MoveAlongS(lookahead_distance, 0.0, 0.0,
				lookAheadMode == LookAheadMode::LOOKAHEADMODE_AT_LANE_CENTER);
		}
	}

	if (retval != ReturnCode::ERROR_GENERIC)
	{
		CalcProbeTarget(&target, data);
	}

	return retval;
}

Position::ReturnCode Position::GetProbeInfo(Position *target_pos, RoadProbeInfo *data)
{
	if (CalcProbeTarget(target_pos, data) != 0)
	{
		return ReturnCode::ERROR_GENERIC;
	}

	return ReturnCode::OK;
}

int Position::GetTrackId() const
{
	if (rel_pos_ && rel_pos_ != this &&
		(type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD))
	{
		return rel_pos_->GetTrackId();
	}

	return track_id_;
}

int Position::GetJunctionId() const
{
	if (rel_pos_ && rel_pos_ != this &&
		(type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD))
	{
		return rel_pos_->GetJunctionId();
	}

	Road* road = GetOpenDrive()->GetRoadByIdx(track_idx_);
	if (road)
	{
		return road->GetJunction();
	}

	return -1;
}

int Position::GetLaneId() const
{
	if (rel_pos_ && rel_pos_ != this && type_ == PositionType::RELATIVE_LANE)
	{
		return rel_pos_->GetLaneId() + lane_id_;
	}

	return lane_id_;
}

int Position::GetLaneGlobalId()
{
	Road *road = GetRoadById(GetTrackId());
	if (road == 0)
	{
		// No road
		return -1;
	}

	if (road->GetJunction() != -1)
	{
		if (GetOpenDrive()->GetJunctionById(road->GetJunction())->IsOsiIntersection())
		{
			return GetOpenDrive()->GetJunctionById(road->GetJunction())->GetGlobalId();
		}
	}

	LaneSection *lane_section = road->GetLaneSectionByIdx(lane_section_idx_);

	if (lane_section == 0)
	{
		LOG("No lane section for idx %d - keeping current lane setting\n", lane_section_idx_);
		return -2;
	}

	double offset;
	int lane_idx = lane_section->GetClosestLaneIdx(s_, t_, offset, false, Lane::LaneType::LANE_TYPE_ANY);

	if (lane_idx == -1)
	{
		LOG("Failed to find a valid drivable lane");
		return -3;
	}

	// Check if it is not a center lane
	int lane_id = lane_section->GetLaneIdByIdx(lane_idx);
	if(!lane_section->IsOSILaneById(lane_id))
	{
		if(offset>=0)
		{
			lane_id = 1;
		} else {
			lane_id = -1;
		}
		lane_idx = lane_section->GetLaneIdxById(lane_id);
	}

	return lane_section->GetLaneGlobalIdByIdx(lane_idx);
}

double Position::GetS() const
{
	if (rel_pos_ && rel_pos_ != this &&
		(type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD))
	{
		return rel_pos_->GetS() + s_;
	}

	return s_;
}

double Position::GetT() const
{
	if (rel_pos_ && rel_pos_ != this && type_ == PositionType::RELATIVE_ROAD)
	{
		return rel_pos_->GetT() + t_;
	}

	return t_;
}

double Position::GetOffset()
{
	if (rel_pos_ && rel_pos_ != this &&	type_ == PositionType::RELATIVE_ROAD)
	{
		return rel_pos_->GetOffset() + offset_;
	}

	return offset_;
}

double Position::GetX() const
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return x_;
	}
	else if (type_ == PositionType::RELATIVE_OBJECT)
	{
		return rel_pos_->GetX() + x_ * cos(rel_pos_->GetH()) - y_ * sin(rel_pos_->GetH());
	}
	else if (type_ == PositionType::RELATIVE_WORLD)
	{
		return x_ + rel_pos_->GetX();
	}
	else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)
	{
		// Create a temporary position to evaluate in relative lane coordinates
		Position pos = *this->rel_pos_;

		// If valid road ID, then move laterally
		if (pos.GetTrackId() != -1)
		{
			pos.SetLanePos(pos.GetTrackId(), pos.GetLaneId() + lane_id_, pos.GetS() + s_, pos.GetOffset() + offset_);
		}

		return pos.GetX();
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return x_;
}

double Position::GetY() const
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return y_;
	}
	else if (type_ == PositionType::RELATIVE_OBJECT)
	{
		return rel_pos_->GetY() + y_ * cos(rel_pos_->GetH()) + x_ * sin(rel_pos_->GetH());
	}
	else if (type_ == PositionType::RELATIVE_WORLD)
	{
		return y_ + rel_pos_->GetY();
	}
	else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)
	{
		// Create a temporary position to evaluate in relative lane coordinates
		Position pos = *this->rel_pos_;

		// If valid road ID, then move laterally
		if (pos.GetTrackId() != -1)
		{
			pos.SetLanePos(pos.GetTrackId(), pos.GetLaneId() + lane_id_, pos.GetS() + s_, pos.GetOffset() + offset_);
		}

		return pos.GetY();
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return y_;
}

double Position::GetZ() const
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return z_;
	}
	else if (type_ == PositionType::RELATIVE_OBJECT || type_ == PositionType::RELATIVE_WORLD)
	{
		return z_ + rel_pos_->GetZ();
	}
	else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)
	{
		// Create a temporary position to evaluate in relative lane coordinates
		Position pos = *this->rel_pos_;

		// If valid road ID, then move laterally
		if (pos.GetTrackId() != -1)
		{
			pos.SetLanePos(pos.GetTrackId(), pos.GetLaneId() + lane_id_, pos.GetS() + s_, pos.GetOffset() + offset_);
		}

		return pos.GetZ();
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return z_;
}

double Position::GetH() const
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return h_;
	}
	else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return h_;
		}
		else
		{
			return h_ + rel_pos_->GetH();
		}
	}
	else if (type_ == PositionType::RELATIVE_ROAD)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return h_;
		}
		else
		{
			return h_ + GetHRoad();
		}
	}
	else if (type_ == PositionType::RELATIVE_LANE)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return h_;
		}
		else
		{
			// Lane coordinate system not fully implemented yet. For now relate heading to the
			// driving direction of current lane, also considering road rule (right/left hand traffic)
			return h_ + GetHRoadInDrivingDirection();
		}
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return h_;
}

double Position::GetHRelative() const
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return h_relative_;
	}
	else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return h_relative_;
		}
		else
		{
			return GetAngleInInterval2PI(h_relative_ + rel_pos_->GetHRelative());
		}
	}
	else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return h_relative_;
		}
		else
		{
			return GetAngleInInterval2PI(h_relative_ + GetHRoadInDrivingDirection());
		}
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return h_relative_;
}

double Position::GetP()
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return p_;
	}
	else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return p_;
		}
		else
		{
			return GetAngleSum(p_, rel_pos_->GetP());
		}
	}
	else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return p_;
		}
		else
		{
			return GetAngleSum(p_, GetPRoadInDrivingDirection());
		}
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return p_;
}

double Position::GetPRelative()
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return p_relative_;
	}
	else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return p_relative_;
		}
		else
		{
			return GetAngleInInterval2PI(p_relative_ + rel_pos_->GetPRelative());
		}
	}
	else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return p_relative_;
		}
		else
		{
			return GetAngleInInterval2PI(p_relative_ + GetPRoadInDrivingDirection());
		}
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return p_relative_;
}

double Position::GetR()
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return r_;
	}
	else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return r_;
		}
		else
		{
			return GetAngleSum(r_, rel_pos_->GetR());
		}
	}
	else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return r_;
		}
		else
		{
			return r_;  // road R not implemented yet
		}
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return r_;
}

double Position::GetRRelative()
{
	if (!rel_pos_ || rel_pos_ == this)
	{
		return r_relative_;
	}
	else if (type_ == PositionType::RELATIVE_WORLD || type_ == PositionType::RELATIVE_OBJECT)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return r_relative_;
		}
		else
		{
			return GetAngleInInterval2PI(r_relative_ + rel_pos_->GetRRelative());
		}
	}
	else if (type_ == PositionType::RELATIVE_LANE || type_ == PositionType::RELATIVE_ROAD)
	{
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			return r_relative_;
		}
		else
		{
			return GetAngleInInterval2PI(r_relative_ + r_road_);
		}
	}
	else
	{
		LOG("Unexpected PositionType: %d", type_);
	}

	return r_relative_;
}

int Position::SetRoutePosition(Position *position)
{
	if(!route_ || !route_->IsValid())
	{
		return -1;
	}

	// Is it a valid position, i.e. is it along the route
	for (size_t i=0; i<route_->minimal_waypoints_.size(); i++)
	{
		if (route_->minimal_waypoints_[i].GetTrackId() == position->GetTrackId()) // Same road
		{
			// Update current position
			Route *tmp = route_;  // save route pointer, copy the
			*this = *position;
			route_ = tmp;
			return 0;
		}
	}

	return -1;
}

double Position::GetRouteS()
{
	if (!route_ || !route_->IsValid())
	{
		return 0.0;
	}

	return route_->GetPathS();
}

Position::ReturnCode Position::MoveRouteDS(double ds, bool actualDistance)
{
	Position::ReturnCode retval = ReturnCode::OK;

	if (!route_ || !route_->IsValid())
	{
		return ReturnCode::ERROR_GENERIC;
	}

	// Idea:
	// Calculate adjusted ds for entity actual distance
	// if already in junction:
	//   Move entity first
	//   Move route pos to same % along connecting road as entity
	// else:
	//   Move route position object first
	//   Move entity position object
	//   if both entered junction:
	// 	   Move route pos to same % along connecting road as entity
	//   else if only one entered junction:
	// 	   Sync both to end of incoming road

	if (actualDistance)
	{
		ds = DistanceToDS(ds);
	}

	Road* entity_road = Position::GetOpenDrive()->GetRoadById(GetTrackId());
	if (entity_road == nullptr)
	{
		return ReturnCode::ERROR_GENERIC;
	}

	double s_route = route_->GetTrackS();
	double s_entity = GetS();
	double t_entity = GetT();

	if (entity_road->GetJunction() > -1)
	{
		MoveAlongS(ds, false);  // actual distance = false since ds is already adjusted

		if (Position::GetOpenDrive()->GetRoadById(GetTrackId())->GetJunction() > -1)
		{
			if ((int)route_->CopySFractionOfLength(this) < 0)
			{
				retval = route_->SetTrackS(GetTrackId(), GetS());
			}
			else
			{
				// Update route s value
				retval = route_->SetPathS(route_->GetPathS() + ds);
			}
		}
		else
		{
			// If out of junction, sync positions again
			retval = route_->SetTrackS(GetTrackId(), GetS());
		}
	}
	else
	{
		retval = route_->MovePathDS(ds * (IsAngleForward(GetHRelative()) ? 1 : -1));
		MoveAlongS(ds, false);  // actual distance = false since ds is already adjusted

		Road* route_road2 = Position::GetOpenDrive()->GetRoadById(route_->GetTrackId());
		Road* entity_road2 = Position::GetOpenDrive()->GetRoadById(GetTrackId());

		if (entity_road2->GetJunction() > -1 && route_road2->GetJunction() > -1)
		{
			if ((int)route_->CopySFractionOfLength(this) < 0)
			{
				retval = route_->SetTrackS(GetTrackId(), GetS());
			}
		}
		else if (entity_road2->GetJunction() > -1 || route_road2->GetJunction() > -1)
		{
			if (entity_road2->GetJunction() > -1)
			{
				SetTrackPos(route_->GetTrackId(), s_route + ds, t_entity);
			}
			else
			{
				retval = route_->SetTrackS(route_->GetTrackId(), s_entity + ds);
			}
		}
	}

	if (retval == ReturnCode::ERROR_END_OF_ROUTE)
	{
		LOG("End of route at road_id %d lane_id %d s %.2f", GetTrackId(), GetLaneId(), GetS());
		SetRoute(nullptr);
	}

	return retval;
}

int Position::SetRouteLanePosition(Route* route, double path_s, int lane_id, double lane_offset)
{
	route->SetPathS(path_s);

	SetLanePos(route->GetTrackId(), lane_id, route->GetTrackS(), lane_offset);

	return 0;
}

int PolyLineBase::EvaluateSegmentByLocalS(int i, double local_s, double cornerRadius, TrajVertex& pos)
{
	TrajVertex* vp0 = &vertex_[i];

	if (i >= GetNumberOfVertices() - 1)
	{
		pos.x = vp0->x;
		pos.y = vp0->y;
		pos.z = vp0->z;
		pos.h = vp0->h;
		pos.s = vp0->s;
		pos.p = vp0->p;
		pos.time = vp0->time;
		pos.speed = vp0->speed;
	}
	else if (i >= 0)
	{
		TrajVertex* vp1 = &vertex_[i + 1];

		double length = MAX(vertex_[i + 1].s - vertex_[i].s, SMALL_NUMBER);

		local_s = CLAMP(local_s, 0, length);

		double a = local_s / length; // a = interpolation factor

		pos.x = (1 - a) * vp0->x + a * vp1->x;
		pos.y = (1 - a) * vp0->y + a * vp1->y;
		pos.z = (1 - a) * vp0->z + a * vp1->z;
		pos.time = (1 - a) * vp0->time + a * vp1->time;
		pos.speed = (1 - a) * vp0->speed + a * vp1->speed;
		pos.s = (1 - a) * vp0->s + a * vp1->s;
		pos.p = (1 - a) * vp0->p + a * vp1->p;

		if (vertex_[i + 1].calcHeading && !interpolateHeading_)
		{
			// Strategy: Align to line, but interpolate at corners
			double radius = MIN(2.0, length / 2.0);
			if (local_s < radius)
			{
				// passed a corner
				a = (radius + local_s) / (2 * radius);
				if (i > 0)
				{
					pos.h = GetAngleInInterval2PI(vertex_[i-1].h + a * GetAngleDifference(vertex_[i].h, vertex_[i-1].h));
				}
				else
				{
					// No previous value to interpolate
					pos.h = vertex_[i].h;
				}
			}
			else if (local_s > length - radius)
			{
				a = (radius + (length - local_s)) / (2 * radius);
				if (i > GetNumberOfVertices() - 2)
				{
					// Last segment, no next point to interpolate
					pos.h = a * vertex_[i].h;
				}
				else
				{
					pos.h = GetAngleInInterval2PI(vertex_[i].h + (1 - a) * GetAngleDifference(vertex_[i+1].h, vertex_[i].h));
				}
			}
			else
			{
				pos.h = vertex_[i].h;
			}
		}
		else
		{
			// Interpolate
			pos.h = GetAngleInInterval2PI(vp0->h + a * GetAngleDifference(vp1->h, vp0->h));
		}
	}
	else
	{
		return -1;
	}

	return 0;
}

TrajVertex* PolyLineBase::AddVertex(double x, double y, double z, double h)
{
	TrajVertex v;

	v.calcHeading = false;
	vertex_.push_back(v);

	return UpdateVertex(GetNumberOfVertices() - 1, x, y, z, GetAngleInInterval2PI(h));
}

TrajVertex* PolyLineBase::AddVertex(double x, double y, double z)
{
	TrajVertex v;

	v.calcHeading = true;
	vertex_.push_back(v);

	return UpdateVertex(GetNumberOfVertices() - 1, x, y, z);
}

TrajVertex* PolyLineBase::AddVertex(TrajVertex p)
{
	vertex_.push_back(p);

	if (p.calcHeading)
	{
		return UpdateVertex(GetNumberOfVertices() - 1, p.x, p.y, p.z);
	}
	else
	{
		return UpdateVertex(GetNumberOfVertices() - 1, p.x, p.y, p.z, p.h);
	}
}

TrajVertex* PolyLineBase::UpdateVertex(int i, double x, double y, double z)
{
	TrajVertex* v = &vertex_[i];

	v->x = x;
	v->y = y;
	v->z = z;

	if (i > 0)
	{
		TrajVertex* vp = &vertex_[i - 1];

		if (v->calcHeading)
		{
			// Calulate heading from line segment between this and previous vertices
			if (PointDistance2D(v->x, v->y, vp->x, v->y) < SMALL_NUMBER)
			{
				// If points conside, use heading of previous vertex
				v->h = vp->h;
			}
			else
			{
				v->h = GetAngleInInterval2PI(atan2(v->y - vp->y, v->x - vp->x));
			}
		}

		if (vp->calcHeading)
		{
			// Update heading of previous vertex now that outgoing line segment is known
			vp->h = v->h;
		}

		// Update polyline length
		double dist = PointDistance2D(x, y, vp->x, vp->y);
		length_ += dist;
	}
	else if (i == 0)
	{
		length_ = 0;
	}

	v->s = length_;

	return &vertex_[i];
}

TrajVertex* PolyLineBase::UpdateVertex(int i, double x, double y, double z, double h)
{
	TrajVertex* v = &vertex_[i];

	v->h = h;

	UpdateVertex(i, x, y, z);

	return &vertex_[i];
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, double cornerRadius, int startAtIndex)
{
	double s_local = 0;
	int i = CLAMP(0, startAtIndex, GetNumberOfVertices() - 1);

	if (GetNumberOfVertices() < 1)
	{
		return -1;
	}

	if (s > GetVertex(-1)->s)
	{
		// end of trajectory
		s = length_;
		s_local = 0;
		i = GetNumberOfVertices() - 1;
	}
	else if (s >= vertex_[i].s)
	{
		for (; i < GetNumberOfVertices() - 1 && s >= vertex_[i + 1].s; i++);
	}
	else if (s < vertex_[i].s)
	{
		for (; i > 0 && s < vertex_[i].s; i--);
	}

	double s0 = vertex_[i].s;
	s_local = s - s0;

	EvaluateSegmentByLocalS(i, s_local, cornerRadius, pos);
	pos.s = s;

	return i;
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, double cornerRadius)
{
	return Evaluate(s, pos, cornerRadius, 0);
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, int startAtIndex)
{
	return Evaluate(s, pos, 0.0, startAtIndex);
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos)
{
	return Evaluate(s, pos, 0.0, 0);
}

int PolyLineBase::Time2S(double time, double& s)
{
	int step = 1;
	if (GetNumberOfVertices() < 1 || time < vertex_[0].time)
	{
		s = 0.0;
		return 0;
	}

	// start looking from current index
	int i = current_index_;

	if (time < vertex_[i].time)
	{
		step = -1;  // Search backwards
	}

	for (size_t j = 0; j < GetNumberOfVertices(); j++)
	{
		if (vertex_[i].time <= time && vertex_[i + 1].time > time)
		{
			double w = (time - vertex_[i].time) / (vertex_[i + 1].time - vertex_[i].time);
			s = vertex_[i].s + w * (vertex_[i + 1].s - vertex_[i].s);
			current_index_ = i;
			current_s_ = s;
			return 0;
		}

		i += step;
		if (i >= GetNumberOfVertices() - 1)
		{
			// Reached end of buffer, continue from start
			i = 0;
		}
		else if (i < 0)
		{
			i = GetNumberOfVertices() - 1;
		}
	}

	// s seems out of range, grab last element
	s = GetVertex(-1)->s;

	return 0;
}

int PolyLineBase::FindClosestPoint(double xin, double yin, TrajVertex& pos, int& index, int startAtIndex)
{
	// look along the line segments
	TrajVertex tmpPos;
	double sLocal = 0.0;
	double sLocalMin = 0.0;
	int iMin = startAtIndex;
	double distMin = LARGE_NUMBER;
	int i = 0;
	int step = 1;

	// If a teleportation is made by the Ghost, a reset of trajectory has been made. Hence, we can't look from the usual point. Set startAtIndex = 0
	if (startAtIndex > GetNumberOfVertices() - 1)
	{
		startAtIndex = 0;
		i = 0;
	}
	else
	{
		i = startAtIndex < 0 ? 0 : startAtIndex;
	}

	// Find closest line segment

	while (i >= 0 && i < GetNumberOfVertices() - 1)
	{
		ProjectPointOnVector2D(xin, yin, vertex_[i].x, vertex_[i].y, vertex_[i + 1].x, vertex_[i + 1].y, tmpPos.x, tmpPos.y);
		double distTmp = PointDistance2D(xin, yin, tmpPos.x, tmpPos.y);

		bool inside = PointInBetweenVectorEndpoints(tmpPos.x, tmpPos.y, vertex_[i].x, vertex_[i].y, vertex_[i + 1].x, vertex_[i + 1].y, sLocal);
		if (!inside)
		{
			// Find combined longitudinal and lateral distance to line endpoint
			// sLocal represent now (outside line segment) distance to closest line segment end point
			distTmp = sqrt(distTmp * distTmp + sLocal * sLocal);
			if (sLocal < 0)
			{
				sLocal = 0;
			}
			else
			{
				sLocal = vertex_[i + 1].s - vertex_[i].s;
			}
		}
		else
		{
			// rescale normalized s
			sLocal *= (vertex_[i + 1].s - vertex_[i].s);
		}

		if (distTmp < distMin)
		{
			iMin = (int)i;
			sLocalMin = sLocal;
			distMin = distTmp;
		}
		else if (startAtIndex > 0)
		{
			// Look for a local minimum distance
			// Distance is increasing
			// After looking in forward direction, go backwards from the start index
			if (step == 1)
			{
				i = startAtIndex;  // go back to start index
				step = -1;  // and continue search in other direction
			}
			else
			{
				break;  // Now give up
			}
		}
		i += step;
	}

	if (distMin < LARGE_NUMBER)
	{
		EvaluateSegmentByLocalS(iMin, sLocalMin, 0.0, pos);
		index = iMin;
		return 0;
	}
	else
	{
		return -1;
	}
}

int PolyLineBase::FindPointAhead(double s_start, double distance, TrajVertex& pos, int& index, int startAtIndex)
{
	index = Evaluate(s_start + distance, pos, startAtIndex);

	return 0;
}

int PolyLineBase::FindPointAtTime(double time, TrajVertex& pos, int& index, int startAtIndex)
{
	double s = 0;
	if (Time2S(time, s) != 0)
	{
		return -1;
	}

	index = Evaluate(s, pos, startAtIndex);

	return 0;
}

int PolyLineBase::FindPointAtTimeRelative(double time, TrajVertex& pos, int& index, int startAtIndex)
{
	double s = 0;
	if (vertex_.size() == 0)
	{
		return -1;
	}

	if (Time2S(vertex_[0].time + time, s) != 0)
	{
		return -1;
	}

	index = Evaluate(s, pos, startAtIndex);

	return 0;
}

TrajVertex* PolyLineBase::GetVertex(int index)
{
	if (GetNumberOfVertices() < 1)
	{
		return nullptr;
	}

	if (index == -1)
	{
		return &vertex_.back();
	}
	else
	{
		return &vertex_[index];
	}
}

void PolyLineBase::Reset()
{
	vertex_.clear();
	current_index_ = 0;
	current_s_ = 0.0;
	length_ = 0;
}

void PolyLineShape::AddVertex(Position pos, double time, bool calculateHeading)
{
	Vertex* v = new Vertex();
	v->pos_ = pos;
	vertex_.push_back(v);
	pline_.AddVertex({ pos.GetTrajectoryS(), pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), time, 0.0, 0.0, calculateHeading });
}

int PolyLineShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos)
{
	double s = 0;
	int i = 0;

	if (pline_.GetNumberOfVertices() < 1)
	{
		return -1;
	}

	if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_S && p > pline_.GetVertex(-1)->s ||
		ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME && p > pline_.GetVertex(-1)->time)
	{
		// end of trajectory
		s = GetLength();
		i = (int)vertex_.size() - 1;
	}
	else
	{
		for (; i < vertex_.size() - 1 &&
			(ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_S && pline_.vertex_[i + 1].s < p ||
			 ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME && pline_.vertex_[i + 1].time < p); i++);

		if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME)
		{
			double a = (p - pline_.vertex_[i].time) / (pline_.vertex_[i + 1].time - pline_.vertex_[i].time); // a = interpolation factor
			s = pline_.vertex_[i].s + a * (pline_.vertex_[i + 1].s - pline_.vertex_[i].s);
		}
		else
		{
			s = p;
		}
	}

	pline_.Evaluate(s, pos, i);

	return 0;
}

double PolyLineShape::GetStartTime()
{
	if (vertex_.size() == 0)
	{
		return 0.0;
	}

	return pline_.vertex_[0].time;
}

double PolyLineShape::GetDuration()
{
	if (vertex_.size() == 0)
	{
		return 0.0;
	}

	return pline_.vertex_.back().time - pline_.vertex_[0].time;
}

double NurbsShape::CoxDeBoor(double x, int i, int k, const std::vector<double>& t)
{
	// Inspiration: Nurbs Curve Example @
	// https://nccastaff.bournemouth.ac.uk/jmacey/OldWeb/RobTheBloke/www/opengl_programming.html

	if (k == 1)
	{
		if (t[i] <= x && x < t[i + 1])
		{
			return 1.0;
		}
		return 0.0;
	}

	double den1 = t[i + k - 1] - t[i];
	double den2 = t[i + k] - t[i + 1];
	double eq1 = 0.0;
	double eq2 = 0.0;

	if (den1 > 0)
	{
		eq1 = ((x - t[i]) / den1) * CoxDeBoor(x, i, k-1, t);
	}

	if (den2 > 0)
	{
		eq2 = (t[i + k] - x) / den2 * CoxDeBoor(x, i + 1, k-1, t);
	}

	return eq1 + eq2;
}

void NurbsShape::CalculatePolyLine()
{
	if (ctrlPoint_.size() < 1)
	{
		return;
	}
	Position tmpRoadPos;

	// Calculate approximate length - to find a reasonable step length

	length_ = 0;
	double steplen = 1.0; // steplen in meters
	for (size_t i = 0; i < ctrlPoint_.size(); i++)
	{
		ctrlPoint_[i].pos_.ReleaseRelation();
		ctrlPoint_[i].t_ = knot_[i + order_ - 1];
		if (i > 0)
		{
			length_ += PointDistance2D(
				ctrlPoint_[i - 1].pos_.GetX(),
				ctrlPoint_[i - 1].pos_.GetY(),
				ctrlPoint_[i].pos_.GetX(),
				ctrlPoint_[i].pos_.GetY());
		}
	}

	if (length_ == 0)
	{
		throw std::runtime_error("Nurbs zero length - check controlpoints");
	}

	// Calculate arc length
	double newLength = 0.0;
	int nSteps = (int)(1 + length_ / steplen);
	double p_steplen = knot_.back() / nSteps;
	TrajVertex pos = { 0, 0, 0, 0, 0, 0, 0, 0, false };
	TrajVertex oldpos = { 0, 0, 0, 0, 0, 0, 0, 0, false };
	TrajVertex tmppos = { 0, 0, 0, 0, 0, 0, 0, 0, false };

	pline_.Reset();
	for (int i = 0; i < nSteps + 1; i++)
	{
		double t = i * p_steplen;
		EvaluateInternal(t, pos);

		// Calulate heading from line segment between this and previous vertices
		if (i < nSteps)
		{
			EvaluateInternal(t + 0.01 * p_steplen, tmppos);
		}
		else
		{
			EvaluateInternal(t - 0.01 * p_steplen, tmppos);
		}

		if (PointDistance2D(tmppos.x, tmppos.y, pos.x, pos.y) < SMALL_NUMBER)
		{
			// If points conside, use heading from polyline
			pos.calcHeading = false;
		}
		else
		{
			if (i < nSteps)
			{
				pos.h = GetAngleInInterval2PI(atan2(tmppos.y - pos.y, tmppos.x - pos.x));
			}
			else
			{
				pos.h = GetAngleInInterval2PI(atan2(pos.y - tmppos.y, pos.x - tmppos.x));
			}
		}

		if (i > 0)
		{
			newLength += PointDistance2D(pos.x, pos.y, oldpos.x, oldpos.y);
		}
		pos.s = newLength;

		// Find max contributing controlpoint for time interpolation
		for (int j = 0; j < ctrlPoint_.size(); j++)
		{
			if (d_[j] > dPeakValue_[j])
			{
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
	for (int i = 0; i < pline_.vertex_.size(); i++)
	{
		if (pline_.vertex_[i].p >= dPeakT_[currentCtrlPoint + 1])
		{
			currentCtrlPoint = MIN(currentCtrlPoint + 1, (int)(ctrlPoint_.size()) - 2);
		}
		double w = (pline_.vertex_[i].p - dPeakT_[currentCtrlPoint]) / (dPeakT_[currentCtrlPoint + 1] - dPeakT_[currentCtrlPoint]);
		pline_.vertex_[i].time = ctrlPoint_[currentCtrlPoint].time_ + w * (ctrlPoint_[currentCtrlPoint + 1].time_ - ctrlPoint_[currentCtrlPoint].time_);
	}

	length_ = newLength;
}

int NurbsShape::EvaluateInternal(double t, TrajVertex& pos)
{
	pos.x = pos.y = 0.0;

	// Find knot span
	t = CLAMP(t, knot_[0], knot_.back() - SMALL_NUMBER);

	double rationalWeight = 0.0;

	for (size_t i = 0; i < ctrlPoint_.size(); i++)
	{
		// calculate the effect of this point on the curve
		d_[i] = CoxDeBoor(t, (int)i, order_, knot_);
		rationalWeight += d_[i] * ctrlPoint_[i].weight_;
	}

	for (size_t i = 0; i < ctrlPoint_.size(); i++)
	{
		if (d_[i] > SMALL_NUMBER)
		{
			// sum effect of CV on this part of the curve
			pos.x += d_[i] * ctrlPoint_[i].pos_.GetX() * ctrlPoint_[i].weight_ / rationalWeight;
			pos.y += d_[i] * ctrlPoint_[i].pos_.GetY() * ctrlPoint_[i].weight_ / rationalWeight;
		}
	}

	return 0;
}

void NurbsShape::AddControlPoint(Position pos, double time, double weight, bool calcHeading)
{
	if (calcHeading == false)
	{
		LOG_ONCE("Info: Explicit orientation in Nurbs trajectory control points not supported yet");
	}
	ctrlPoint_.push_back(ControlPoint(pos, time, weight, true));
	d_.push_back(0);
	dPeakT_.push_back(0);
	dPeakValue_.push_back(0);
}

void NurbsShape::AddKnots(std::vector<double> knots)
{
	knot_ = knots;

	if (knot_.back() < SMALL_NUMBER)
	{
		return;
	}
}

int NurbsShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos)
{
	if (order_ < 1 || ctrlPoint_.size() < order_ || GetLength() < SMALL_NUMBER)
	{
		return -1;
	}

	double s = p;

	if (ptype == TRAJ_PARAM_TYPE_TIME)
	{
		pline_.Time2S(p, s);
	}

	pline_.Evaluate(s, pos, pline_.current_index_);

	EvaluateInternal(pos.p, pos);

	return 0;
}

double NurbsShape::GetStartTime()
{
	if (ctrlPoint_.size() == 0)
	{
		return 0.0;
	}

	return ctrlPoint_[0].time_;
}

double NurbsShape::GetDuration()
{
	if (ctrlPoint_.size() == 0)
	{
		return 0.0;
	}

	return ctrlPoint_.back().time_ - ctrlPoint_[0].time_;
}

ClothoidShape::ClothoidShape(roadmanager::Position pos, double curv, double curvPrime, double len, double tStart, double tEnd) : Shape(ShapeType::CLOTHOID)
{
	pos_ = pos;
	spiral_ = new roadmanager::Spiral(0, pos_.GetX(), pos_.GetY(), pos_.GetH(), len, curv, curv + curvPrime * len);
	t_start_ = tStart;
	t_end_ = tEnd;
	pline_.interpolateHeading_ = true;
}

void ClothoidShape::CalculatePolyLine()
{
	// Create polyline placeholder representation
	double stepLen = 1.0;
	int steps = (int)(spiral_->GetLength() / stepLen);
	pline_.Reset();
	TrajVertex v;

	for (size_t i = 0; i < steps + 1; i++)
	{
		if (i < steps)
		{
			EvaluateInternal((double)i, v);
		}
		else
		{
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

int ClothoidShape::EvaluateInternal(double s, TrajVertex& pos)
{
	spiral_->EvaluateDS(s, &pos.x, &pos.y, &pos.h);

	return 0;
}

int ClothoidShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos)
{
	if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME)
	{
		if (p >= t_start_ && p <= t_end_)
		{
			double t = p - t_start_;
			// Transform time parameter value into a s value
			p = GetLength() * (t - t_start_) / (t_end_ - t_start_);
		}
		else
		{
			LOG("Requested time %.2f outside range [%.2f, %.2f]", p, t_start_, t_end_);
			p = GetLength();
		}
	}
	else if (p > GetLength())
	{
		p = GetLength();
	}

	pline_.Evaluate(p, pos);

	spiral_->EvaluateDS(p, &pos.x, &pos.y, &pos.h);

	pos.s = p;

	return 0;
}

double ClothoidShape::GetStartTime()
{
	return t_start_;
}

double ClothoidShape::GetDuration()
{
	return t_end_ - t_start_;
}

int Position::MoveTrajectoryDS(double ds)
{
	if (!trajectory_)
	{
		return -1;
	}

	TrajVertex pos;
	trajectory_->shape_->Evaluate(s_trajectory_ + ds, Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, pos);

	SetInertiaPos(pos.x, pos.y, pos.h);

	s_trajectory_ = pos.s;

	return 0;
}

int Position::SetTrajectoryPosByTime(double time)
{
	if (!trajectory_)
	{
		return -1;
	}

	TrajVertex pos;
	trajectory_->shape_->Evaluate(time, Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_TIME, pos);

	SetInertiaPos(pos.x, pos.y, pos.h);

	s_trajectory_ = pos.s;

	return 0;
}

int Position::SetTrajectoryS(double s)
{
	if (!trajectory_)
	{
		return -1;
	}

	TrajVertex pos;
	trajectory_->shape_->Evaluate(s, Shape::TrajectoryParamType::TRAJ_PARAM_TYPE_S, pos);
	SetInertiaPos(pos.x, pos.y, pos.z, pos.h, 0.0, 0.0, true);

	s_trajectory_ = pos.s;

	return 0;
}

Position::ReturnCode Position::SetRouteS(double route_s)
{
	Position::ReturnCode retval = Position::ReturnCode::OK;

	if (!GetRoute())
	{
		return ReturnCode::ERROR_GENERIC;
	}

	retval = route_->SetPathS(route_s);
	if ((int)retval < 0 && retval != ReturnCode::ERROR_END_OF_ROUTE)
	{
		return retval;
	}

	// Register current driving direction
	int driving_direction = 1;
	if (GetAbsAngleDifference(GetH(), GetDrivingDirection()) > M_PI_2)
	{
		driving_direction = -1;
	}

	double offset_dir_neutral = offset_ * SIGN(GetLaneId());
	SetLanePos(route_->GetTrackId(), route_->GetLaneId(), route_->GetTrackS(), offset_dir_neutral);

	if (retval == ReturnCode::ERROR_END_OF_ROUTE)
	{
		LOG("Reached end of route, reset and continue");
		SetRoute(nullptr);
		status_ |= static_cast<int>(PositionStatusMode::POS_STATUS_END_OF_ROUTE);
		return ReturnCode::ERROR_END_OF_ROUTE;
	}

	return ReturnCode::OK;
}

void Position::ReleaseRelation()
{
	// Fetch values and then disconnect
	double x = GetX();
	double y = GetY();
	double z = GetZ();
	int roadId = GetTrackId();
	int laneId = GetLaneId();
	double s = GetS();
	double t = GetT();
	double offset = GetOffset();
	double p = GetP();
	double r = GetR();
	double h = GetH();
	double hAbs = h_;
	double hRel = h_relative_;
	double pAbs = p_;
	double rAbs = r_;
	PositionType type = type_;

	SetRelativePosition(0, PositionType::NORMAL);

	if (type == Position::PositionType::RELATIVE_ROAD)
	{
		// Resolve requested position
		SetTrackPos(roadId, s, t);
		if (orientation_type_ == OrientationType::ORIENTATION_ABSOLUTE)
		{
			SetHeading(hAbs);
			SetPitch(pAbs);
			SetRoll(rAbs);
		}
	}
	else if (type == Position::PositionType::RELATIVE_LANE)
	{
		SetLanePos(roadId, laneId, s, offset);
		if (orientation_type_ == OrientationType::ORIENTATION_RELATIVE)
		{
			SetHeadingRelative(GetAngleSum(hRel, GetDrivingDirectionRelativeRoad() < 0 ? M_PI : 0.0));
		}
		else
		{
			SetHeading(hAbs);
			SetPitch(pAbs);
			SetRoll(rAbs);
		}
	}
	else if (type == PositionType::RELATIVE_OBJECT || type == PositionType::RELATIVE_WORLD)
	{
		SetInertiaPos(x, y, z, h, p, r, true);
	}
}

int Route::AddWaypoint(Position* position)
{
	int retval = 0;

	if (minimal_waypoints_.size() > 0)
	{
		// Keep only one consecutive waypoint per road
		// Keep first specified waypoint for first road
		// then, for following roads, keep the last waypoint.

		if (position->GetTrackId() == minimal_waypoints_.back().GetTrackId())
		{
			if (minimal_waypoints_.size() == 1)
			{
				// Ignore
				LOG("Ignoring additional waypoint for road %d (s %.2f)",
					position->GetTrackId(), position->GetS());
				all_waypoints_.push_back(*position);
				return -1;
			}
			else  // at least two road-unique waypoints
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
		retval = path->Calculate(dist, false);
		if (retval == 0)
		{
			// Path is found by tracing previous nodes
			RoadPath::PathNode* previous = 0;
			std::vector<RoadPath::PathNode*> nodes;

			if (path->visited_.size() > 0)
			{
				previous = path->visited_.back()->previous;
				nodes.push_back(path->visited_.back());
				while (previous != nullptr)
				{
					nodes.push_back(previous);
					previous = previous->previous;
				}
			}

			if (nodes.size() > 1)
			{
				// Add internal waypoints, one for each road along the path
				for (int i = (int)nodes.size() - 1; i >= 1; i--)
				{
					// Find out lane ID of the connecting road and add the waypoint at 1/3 of the road length
					Position connected_pos = Position(nodes[i - 1]->fromRoad->GetId(), nodes[i - 1]->fromLaneId, 0.0, 0.0);
					connected_pos.MoveAlongS(nodes[i - 1]->fromRoad->GetLength() * 0.33);

					if (connected_pos.GetLaneId() < 0)
					{
						connected_pos.SetHeadingRelative(0.0);
					}
					else
					{
						connected_pos.SetHeadingRelative(M_PI);
					}
					all_waypoints_.push_back(connected_pos);
					minimal_waypoints_.push_back(connected_pos);
					LOG("Route::AddWaypoint Added intermediate waypoint %d roadId %d laneId %d",
						(int)minimal_waypoints_.size() - 1, connected_pos.GetTrackId(), nodes[i - 1]->fromLaneId);
				}
			}

			length_ += fabs(dist);
		}
		else if (retval < 0)
		{
			invalid_route_ = true;
		}
	}
	else
	{
		// First waypoint, make it the current position
		currentPos_ = *position;
	}
	if (retval >= -1)
	{
		// Add waypoint defined by scenario
		scenario_waypoints_.push_back(*position);

		// Add all waypoints including a valid road ID (retval == -2 indicates invalid road ID)
		all_waypoints_.push_back(*position);
		LOG("Route::AddWaypoint Added waypoint %d: %d, %d, %.2f", (int)all_waypoints_.size() - 1, position->GetTrackId(), position->GetLaneId(), position->GetS());

		if (retval == 0)
		{
			// For OpenSCENARIO routes, add only waypoints to which a path has been found
			minimal_waypoints_.push_back(*position);
		}
		else
		{
			LOG("Route::AddWaypoint Skip waypoint for scenario routes since path not found");
		}
	}
	else
	{
		LOG("Route::AddWaypoint Failed to add waypoint %d: %d, %d, %.2f", (int)minimal_waypoints_.size() - 1, position->GetTrackId(), position->GetLaneId(), position->GetS());
	}

	return 0;
}

void Route::CheckValid()
{
	if (invalid_route_)
	{
		LOG("Warning: Route %s is not valid, will be ignored for the default controller.", getName().c_str());
		minimal_waypoints_.clear();
	}
}

Position::ReturnCode Route::CopySFractionOfLength(Position* pos)
{
	Road* road0 = Position::GetOpenDrive()->GetRoadById(pos->GetTrackId());
	Road* road1 = Position::GetOpenDrive()->GetRoadById(GetTrackId());

	Position::ReturnCode retval = Position::ReturnCode::OK;

	if (road0 == nullptr || road1 == nullptr)
	{
		return Position::ReturnCode::ERROR_GENERIC;
	}

	// Check direction, same of not
	int direction = 0;
	if (road0 == road1)
	{
		direction = 1;
	}
	else if (road0->GetJunction() == road1->GetJunction())
	{
		// connecting roads in same junction
		RoadLink* link0_pre = road0->GetLink(LinkType::PREDECESSOR);
		RoadLink* link1_pre = road1->GetLink(LinkType::PREDECESSOR);
		RoadLink* link0_suc = road0->GetLink(LinkType::SUCCESSOR);
		RoadLink* link1_suc = road1->GetLink(LinkType::SUCCESSOR);

		if (link0_pre && link1_pre && *link0_pre == *link1_pre)
		{
			direction = 1;
		}
		else if (link0_suc && link1_suc && *link0_suc == *link1_suc)
		{
			direction = 1;
		}
		else if (link0_pre && link1_suc)
		{
			if ((link0_pre->GetType() == LinkType::PREDECESSOR && link1_suc->GetType() == LinkType::SUCCESSOR) ||
				(link0_pre->GetType() == LinkType::SUCCESSOR && link1_suc->GetType() == LinkType::PREDECESSOR) &&
				link0_pre->GetElementType() == link1_suc->GetElementType() &&
				link0_pre->GetElementId() == link1_suc->GetElementId() &&
				link0_pre->GetContactPointType() == link1_suc->GetContactPointType())
			direction = -1;
		}
		else if (link0_suc && link1_pre)
		{
			if ((link0_suc->GetType() == LinkType::PREDECESSOR && link1_pre->GetType() == LinkType::SUCCESSOR) ||
				(link0_suc->GetType() == LinkType::SUCCESSOR && link1_pre->GetType() == LinkType::PREDECESSOR) &&
				link0_suc->GetElementType() == link1_pre->GetElementType() &&
				link0_suc->GetElementId() == link1_pre->GetElementId() &&
				link0_suc->GetContactPointType() == link1_pre->GetContactPointType())
			direction = -1;
		}
		else
		{
			retval = Position::ReturnCode::ERROR_GENERIC;
		}
	}

	if (direction != 0)
	{
		double fraction = CLAMP(pos->GetS() / MAX(SMALL_NUMBER, road0->GetLength()), 0.0, 1.0);
		if (direction < 0)
		{
			fraction = 1 - fraction;
		}
		currentPos_.SetLanePos(GetTrackId(), GetLaneId(), fraction * road1->GetLength(), 0.0);
	}

	return retval;
}

Position::ReturnCode Route::SetTrackS(int trackId, double s)
{
	// Loop over waypoints - look for current track ID and sum the distance (route s) up to current position
	double dist = 0;
	double local_s;

	if (s < 0 || minimal_waypoints_.size() == 0)
	{
		path_s_ = 0.0;
		waypoint_idx_ = 0;
		if (minimal_waypoints_.size() > 0)
		{
			currentPos_.SetTrackPos(GetWaypoint(waypoint_idx_)->GetTrackId(), 0.0, 0.0);
		}
		else
		{
			currentPos_.SetTrackPos(-1, 0.0, 0.0);
		}

		return Position::ReturnCode::ERROR_END_OF_ROUTE;
	}

	for (size_t i = 0; i < minimal_waypoints_.size(); i++)
	{
		int route_direction = GetWayPointDirection((int)i);

		if (route_direction == 0)
		{
			LOG("Unexpected lack of connection in route at waypoint %d", i);
			return Position::ReturnCode::ERROR_GENERIC;
		}

		// Add length of intermediate waypoint road
		dist += Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength();

		if (i == 0)
		{
			// Subtract initial s-value for the first waypoint
			if (route_direction > 0)  // route in waypoint road direction
			{
				dist -= minimal_waypoints_[0].GetS();
				dist = MAX(dist, 0.0);
			}
			else
			{
				// route in opposite road direction - remaining distance equals waypoint s-value
				dist = minimal_waypoints_[0].GetS();
			}
		}

		if (trackId == minimal_waypoints_[i].GetTrackId())
		{
			// current position is at the road of this waypoint - i.e. along the route
			// remove remaming s from road
			if (route_direction > 0)
			{
				dist -= (Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength() - s);
			}
			else
			{
				dist -= s;
			}

			path_s_ = dist;
			local_s = s;
			waypoint_idx_ = (int)i;
			currentPos_.SetLanePos(GetWaypoint(waypoint_idx_)->GetTrackId(), GetWaypoint(waypoint_idx_)->GetLaneId(), local_s, 0.0);

			if (path_s_ < 0 || path_s_ > GetLength())
			{
				return Position::ReturnCode::ERROR_END_OF_ROUTE;
			}
			else
			{
				return Position::ReturnCode::OK;
			}
		}
	}

	// Failed to map current position to the current route
	return Position::ReturnCode::ERROR_GENERIC;
}

Position::ReturnCode Route::MovePathDS(double ds)
{
	if (minimal_waypoints_.size() == 0)
	{
		return Position::ReturnCode::ERROR_GENERIC;
	}

	// Consider route direction
	ds *= GetWayPointDirection(waypoint_idx_);

	return SetPathS(GetPathS() + ds);
}

Position::ReturnCode Route::SetPathS(double s)
{
	// Loop over waypoints - until reaching s meters
	double dist = 0;
	double local_s = 0.0;

	if (minimal_waypoints_.size() == 0)
	{
		path_s_ = 0.0;
		waypoint_idx_ = 0;
		currentPos_.SetTrackPos(GetWaypoint(waypoint_idx_)->GetTrackId(), 0.0, 0.0);
		return Position::ReturnCode::OK;
	}

	for (size_t i = 0; i < minimal_waypoints_.size(); i++)
	{
		int route_direction = GetWayPointDirection((int)i);

		if (route_direction == 0)
		{
			LOG("Unexpected lack of connection in route at waypoint %d", i);
			return Position::ReturnCode::ERROR_GENERIC;
		}

		// Add length of intermediate waypoint road
		dist += Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength();

		if (i == 0)
		{
			// Subtract initial s-value for the first waypoint
			if (route_direction > 0)  // route in waypoint road direction
			{
				dist -= minimal_waypoints_[0].GetS();
			}
			else
			{
				// route in opposite road direction - remaining distance equals waypoint s-value
				dist = minimal_waypoints_[0].GetS();
			}
		}

		if (s < dist)
		{
			// current position is at the road of this waypoint - i.e. along the route
			// remove remaming s from road
			if (route_direction > 0)
			{
				local_s = s - (dist - Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength());
			}
			else
			{
				local_s = dist - s;
			}

			waypoint_idx_ = (int)i;
			currentPos_.SetLanePos(GetWaypoint(waypoint_idx_)->GetTrackId(), GetWaypoint(waypoint_idx_)->GetLaneId(), local_s, 0.0);

			if (s < GetLength())
			{
				path_s_ = s;
				return Position::ReturnCode::OK;
			}
			else
			{
				path_s_ = GetLength();
				return Position::ReturnCode::ERROR_END_OF_ROUTE;
			}
		}
		else if (i == minimal_waypoints_.size() - 1)
		{
			// Past end of route
			if (route_direction > 0)
			{
				local_s = Position::GetOpenDrive()->GetRoadById(minimal_waypoints_[i].GetTrackId())->GetLength();
			}
			else
			{
				local_s = 0.0;
			}
			waypoint_idx_ = (int)i;
			currentPos_.SetLanePos(GetWaypoint(waypoint_idx_)->GetTrackId(), GetWaypoint(waypoint_idx_)->GetLaneId(), local_s, 0.0);
		}
	}

	// Failed to map current position, past the end of route
	path_s_ = GetLength();
	return Position::ReturnCode::ERROR_END_OF_ROUTE;
}

Position* Route::GetWaypoint(int index)
{
	if (index == -1)  // Get current
	{
		index = waypoint_idx_;
	}

	if (minimal_waypoints_.size() == 0 || index < 0 || index >= minimal_waypoints_.size())
	{
		LOG("Waypoint index %d out of range (%d)", index, minimal_waypoints_.size());
		return 0;
	}

	return &minimal_waypoints_[index];
}

Road* Route::GetRoadAtOtherEndOfConnectingRoad(Road* incoming_road)
{
	Road* connecting_road = Position::GetOpenDrive()->GetRoadById(GetTrackId());
	Junction* junction = Position::GetOpenDrive()->GetJunctionById(connecting_road->GetJunction());

	if (junction == 0)
	{
		LOG("Unexpected: Road %d not a connecting road", connecting_road->GetId());
		return 0;
	}

	return junction->GetRoadAtOtherEndOfConnectingRoad(connecting_road, incoming_road);
}

int Route::GetWayPointDirection(int index)
{
	if (minimal_waypoints_.size() == 0 || index < 0 || index >= minimal_waypoints_.size())
	{
		LOG("Waypoint index %d out of range (%d)", index, minimal_waypoints_.size());
		return 0;
	}

	OpenDrive *od = minimal_waypoints_[index].GetOpenDrive();
	Road *road = od->GetRoadById(minimal_waypoints_[index].GetTrackId());
	if (road == nullptr)
	{
		LOG("Waypoint %d invalid road id %d!", index, minimal_waypoints_[index].GetTrackId());
		return 0;
	}

	int direction = 0;
	Position* pos2 = nullptr;

	// Looking in the direction of heading
	direction = minimal_waypoints_[index].GetHRelative() > M_PI_2 && minimal_waypoints_[index].GetHRelative() < 3 * M_PI_2 ? -1 : 1;

	if (index < minimal_waypoints_.size() - 1)
	{
		// Not at last waypoint, so look at next waypoint for direction
		pos2 = GetWaypoint(index + 1);
		if (direction == 1 && road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId())) ||
			direction == -1 && road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId())))
		{
			// Expected case, route direction aligned with waypoint headings
			return direction;
		}
		else
		{
			LOG("Warning: Relative heading not aligned with route direction");
			return -1 * direction;
		}
	}
	else if (index > 0)
	{
		// At last waypoint, so look at previous waypoint
		pos2 = GetWaypoint(index - 1);

		if (direction == -1 && road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId())) ||
			direction == 1 && road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId())))
		{
			// Expected case, route direction aligned with waypoint headings
			return direction;
		}
		else
		{
			LOG("Warning: Relative heading not aligned with route direction");
			return -1 * direction;
		}
	}

	// At first and only (minimal) waypoint, so try to find another waypoint for direction
	if (all_waypoints_.size() > 1 && (road->GetId() == all_waypoints_[1].GetTrackId()))
	{
		return direction;
	}
	else
	{
		LOG("Only one waypoint, no direction");
		return 0;
	}
}

void Route::setName(std::string name)
{
	this->name_ = name;
}

std::string Route::getName()
{
	return name_;
}

void RMTrajectory::Freeze(FollowingMode following_mode)
{
	if (shape_->type_ == Shape::ShapeType::POLYLINE)
	{
		PolyLineShape* pline = (PolyLineShape*)shape_;

		for (size_t i = 0; i < pline->vertex_.size(); i++)
		{
			Position* pos = &pline->vertex_[i]->pos_;
			pos->ReleaseRelation();

			if (following_mode == FollowingMode::FOLLOW && !pos->IsOrientationTypeSet(Position::OrientationSetMask::H))
			{
				// If heading is not provided AND follow mode is requested, then calculate heading
				pline->pline_.vertex_[i].calcHeading = true;
			}

			if (pline->pline_.vertex_[i].calcHeading)
			{
				pline->pline_.UpdateVertex((int)i, pos->GetX(), pos->GetY(), pos->GetZ());
			}
			else
			{
				pline->pline_.UpdateVertex((int)i, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetH());
			}
		}
	}
	else if (shape_->type_ == Shape::ShapeType::CLOTHOID)
	{
		ClothoidShape* clothoid = (ClothoidShape*)shape_;

		clothoid->pos_.ReleaseRelation();

		clothoid->spiral_->SetX(clothoid->pos_.GetX());
		clothoid->spiral_->SetY(clothoid->pos_.GetY());
		clothoid->spiral_->SetHdg(clothoid->pos_.GetH());

		clothoid->CalculatePolyLine();
	}
	else
	{
		NurbsShape* nurbs = (NurbsShape*)shape_;

		nurbs->CalculatePolyLine();
	}
}

double RMTrajectory::GetTimeAtS(double s)
{
	// Find out corresponding time-value using polyline representation
	TrajVertex v;
	shape_->pline_.Evaluate(s, v);

	return v.time;
}

double RMTrajectory::GetStartTime()
{
	return shape_->GetStartTime();
}

double RMTrajectory::GetDuration()
{
	return shape_->GetDuration();
}
