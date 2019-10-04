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

		struct Property
		{
			std::string name_;
			std::string value_;
		};

		Type type_;
		std::string name_;
		int id_;
		bool extern_control_;
		double speed_;
		double wheel_angle;
		roadmanager::Position pos_;
		roadmanager::Route *route_;
		std::string model_filepath_;
		int model_id_;

		Object(Type type) : type_(type), id_(0), extern_control_(false), speed_(0), route_(0), model_filepath_(""), wheel_angle(0) {}
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

		Vehicle() : Object(Object::Type::VEHICLE), category_(Category::CAR) {}

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
			else
			{
				LOG("Vehicle category %s not supported yet", category.c_str());
			}

			return;
		}
	};

	class Entities
	{

	public:

		Entities() {}

		void Print()
		{
			LOG("");
		}

		std::vector<Object*> object_;

	};

}
