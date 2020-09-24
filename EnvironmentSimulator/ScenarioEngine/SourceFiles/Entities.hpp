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

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "Trail.hpp"
#include "OSCBoundingBox.hpp"

namespace scenarioengine
{

	class Object
	{
	public:
		typedef enum
		{
			VEHICLE,
			PEDESTRIAN,
			MISC_OBJECT
		} Type;

		typedef enum
		{
			UNDEFINED,
			INTERNAL,
			EXTERNAL,
			HYBRID_EXTERNAL,
			HYBRID_GHOST,
			SUMO
		} Control;

		struct Property
		{
			std::string name_;
			std::string value_;
		};

		Type type_;
		int category_holder_; // placehoder for specific object category in vehicle, pedestrian or misobject
		std::string name_;
		int id_;
		int trail_follow_index_;  // only in case of hybrid_external following a ghost
		double trail_follow_s_;  // only in case of hybrid_external following a ghost
		double trail_closest_pos_[3];
		Control control_;
		double speed_;
		double wheel_angle_;
		double wheel_rot_;
		roadmanager::Position pos_;
		roadmanager::Route *route_;
		std::string model_filepath_;
		int model_id_;
		Object *ghost_;     // If hybrid control mode, this will point to the ghost entity
		ObjectTrail trail_;
		double odometer_;
		OSCBoundingBox boundingbox_;
		double end_of_road_timestamp_;  
		double off_road_timestamp_;

		bool dirty_long_;
		bool dirty_lat_;
		bool reset_;

		struct {
			double pos_x;
			double pos_y;
			double vel_x;
			double vel_y;
			double h;
			double h_rate;
		} state_old;

		Object(Type type) : type_(type), id_(0), trail_follow_index_(0), control_(Object::Control::INTERNAL),
			speed_(0), wheel_angle_(0), wheel_rot_(0), route_(0), model_filepath_(""), ghost_(0), trail_follow_s_(0),
			odometer_(0), end_of_road_timestamp_(0.0), off_road_timestamp_(0.0), dirty_long_(0), dirty_lat_(0), reset_(0)
		{
			trail_closest_pos_[0] = 0.0;
			trail_closest_pos_[1] = 0.0;
			trail_closest_pos_[2] = 0.0;

			state_old.pos_x = 0;
			state_old.pos_y = 0;
			state_old.vel_x = 0;
			state_old.vel_y = 0;
			state_old.h = 0;
			state_old.h_rate = 0;
		}
		void SetControl(Control control) { control_ = control; }
		void SetEndOfRoad(bool state, double time = 0.0);
		bool IsEndOfRoad() { return end_of_road_timestamp_ > SMALL_NUMBER; }
		double GetEndOfRoadTimestamp() { return end_of_road_timestamp_; }
		void SetOffRoad(bool state, double time = 0.0);
		bool IsOffRoad() { return off_road_timestamp_ > SMALL_NUMBER; }
		double GetOffRoadTimestamp() { return off_road_timestamp_; }
		Control GetControl() { return control_; }
	};

	class Vehicle : public Object
	{
	public:
		typedef enum
		{
			CAR,
			VAN,
			TRUCK,
			SEMITRAILER,
			BUS,
			MOTORBIKE,
			BICYCLE,
			TRAIN,
			TRAM
		} Category;

		Category category_;


		Vehicle() : Object(Object::Type::VEHICLE), category_(Category::CAR) {
			category_holder_ = static_cast<int>(category_);
		}

		void SetCategory(std::string category)
		{
			if (category == "car")
			{
				category_ = Vehicle::Category::CAR;
			}
			else if (category == "truck")
			{
				category_ = Vehicle::Category::TRUCK;
			}
			else if (category == "bus")
			{
				category_ = Vehicle::Category::BUS;
			}
			else if (category == "bicycle")
			{
				category_ = Vehicle::Category::BICYCLE;
			}
			else
			{
				LOG("Vehicle category %s not supported yet", category.c_str());
			}

			return;
		}
	};

    class Pedestrian : public Object
	{
	public:
		typedef enum
		{
			PEDESTRIAN,
			WHEELCHAIR,
			ANIMAL
		} Category;

		std::string model_; /**< Definition of the model of the pedestrian. */
		double mass_; /**< The mass of a pedestrian in kg. */
		std::string name_;
		Category pedestrain_category_; /**< Category type of pedestrian. */
		
		// name, boundingBox and properties are included in base Object class.

		Pedestrian() : Object(Object::Type::PEDESTRIAN), 
		model_(""), mass_(0.0), name_(""), 
		pedestrain_category_(Category::PEDESTRIAN) {
			category_holder_ = static_cast<int>(pedestrain_category_);
		}

		void SetCategory(std::string category)
		{
			if (category == "pedestrian")
			{
				pedestrain_category_ = Pedestrian::Category::PEDESTRIAN;
			}
			else if (category == "wheelchair")
			{
				pedestrain_category_ = Pedestrian::Category::WHEELCHAIR;
			}
			else if (category == "animal")
			{
				pedestrain_category_ = Pedestrian::Category::ANIMAL;
			}
			else
			{
				LOG("Pedestrian category %s not supported yet", category.c_str());
			}

			return;
		}

	};

 class MiscObject : public Object
	{
	public:
		typedef enum
		{
			NONE,
			OBSTACLE,
			POLE,
			TREE,
			VEGETATION,
			BARRIER,
			BUILDING,
			PARKINGSPACE,
			PATCH,
			RAILING,
			TRAFFICISLAND,
			CROSSWALK,
			STREETLAMP,
			GANTRY,
			SOUNDBARRIER,
			WIND,
			ROADMARK
		} Category;

		std::string model_; 
		double mass_;
		std::string name_; 
		Category category_; 

		MiscObject() : Object(Object::Type::MISC_OBJECT), model_(""), mass_(0.0), name_(""), 
		category_(Category::NONE) {
			category_holder_ = static_cast<int>(category_);
		}

		void SetCategory(std::string category)
		{
			if (category == "none")
			{
				category_ = MiscObject::Category::NONE;
			}
			else if (category == "obstacle")
			{
				category_ = MiscObject::Category::OBSTACLE;
			}
			else if (category == "pole")
			{
				category_ = MiscObject::Category::POLE;
			}
			else if (category == "tree")
			{
				category_ = MiscObject::Category::TREE;
			}
			else if (category == "vegetation")
			{
				category_ = MiscObject::Category::VEGETATION;
			}
			else if (category == "barrier")
			{
				category_ = MiscObject::Category::BARRIER;
			}
			else if (category == "building")
			{
				category_ = MiscObject::Category::BUILDING;
			}
			else if (category == "parkingSpace")
			{
				category_ = MiscObject::Category::PARKINGSPACE;
			}
			else if (category == "patch")
			{
				category_ = MiscObject::Category::PATCH;
			}
			else if (category == "railing")
			{
				category_ = MiscObject::Category::RAILING;
			}
			else if (category == "trafficIsland")
			{
				category_ = MiscObject::Category::TRAFFICISLAND;
			}
			else if (category == "crosswalk")
			{
				category_ = MiscObject::Category::CROSSWALK;
			}
			else if (category == "streetLamp")
			{
				category_ = MiscObject::Category::STREETLAMP;
			}
			else if (category == "gantry")
			{
				category_ = MiscObject::Category::GANTRY;
			}
			else if (category == "soundBarrier")
			{
				category_ = MiscObject::Category::SOUNDBARRIER;
			}
			else if (category == "wind")
			{
				category_ = MiscObject::Category::WIND;
			}
			else if (category == "roadMark")
			{
				category_ = MiscObject::Category::ROADMARK;
			}
			else
			{
				LOG("MiscObject category %s not supported yet", category.c_str());
			}

			return;
		}

	};
	// define a contorller class just to parse sumo controller config file
	class Controller
	{
	public:
		std::string name_;
		std::string config_filepath_;

		Controller() : name_(""), config_filepath_("") {}

	};

	class Entities
	{
	public:

		Entities() {};

		void Print()
		{
			LOG("");
		}

		std::vector<Object*> object_;

		// create a sumo vehicle template and sumo config file in Entities class
		Object* sumo_vehicle;
		std::string sumo_config_path;
		float sumo_x_offset;
		float sumo_y_offset;

		int addObject(Object* obj);
		void removeObject(int id);
		void removeObject(std::string name);
		int getNewId();
		bool indexExists(int id);
		bool nameExists(std::string name);
	};

}
