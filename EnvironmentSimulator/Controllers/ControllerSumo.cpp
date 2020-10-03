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

#include "CommonMini.hpp"
#include "ControllerSumo.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "pugixml.hpp"

#include <utils/geom/PositionVector.h>
#include <libsumo/Simulation.h>
#include <libsumo/Vehicle.h>
#include <libsumo/TraCIDefs.h>

using namespace scenarioengine;

ControllerSumo::ControllerSumo(Controller::Type type, std::string name, std::string configfile_path, 
	Entities* entities, ScenarioGateway* gateway, Parameters* parameters) :
	time_(0), sumo_x_offset_(0), sumo_y_offset_(0), configfile_path_(configfile_path), 
	Controller(type, name, entities, gateway)
{
	if (docsumo_.load_file(configfile_path.c_str()).status == pugi::status_file_not_found)
	{
		LOG("Failed to load SUMO config file %s", configfile_path.c_str());
		throw std::invalid_argument(std::string("Cannot open file: ") + configfile_path);
		return;
	}

	std::vector<std::string> file_name_candidates;
	file_name_candidates.push_back(parameters->ReadAttribute(docsumo_.child("configuration").child("input").child("net-file"), "value"));
	file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(configfile_path), file_name_candidates[0]));
	pugi::xml_parse_result sumonet;
	size_t i;
	for (i = 0; i < file_name_candidates.size(); i++)
	{
		sumonet = docsumo_.load_file(file_name_candidates[i].c_str());
		if (sumonet.status == pugi::status_ok)
		{
			break;
		}
	}
	if (sumonet.status != pugi::status_ok)
	{
		// Give up
		LOG("Failed to load SUMO net file %s", file_name_candidates[0].c_str());
		throw std::invalid_argument(std::string("Cannot open file: ") + file_name_candidates[0]);
		return;
	}

	pugi::xml_node location = docsumo_.child("net").child("location");
	std::string netoffset = parameters->ReadAttribute(location, "netOffset");
	std::size_t delim = netoffset.find(',');
	sumo_x_offset_ = std::stof(netoffset.substr(0, delim));
	sumo_y_offset_ = std::stof(netoffset.substr(delim + 1, netoffset.npos));

	std::vector<std::string> options;

	options.push_back("-c " + configfile_path);
	options.push_back("--xml-validation");
	options.push_back("never");

	libsumo::Simulation::load(options);
}

void ControllerSumo::Init()
{
	// Adds all vehicles added in openscenario to the sumosimulation
	for (size_t j = 0; j < entities_->object_.size(); j++)
	{
		if (!(entities_->object_[j]->GetControllerType() == Controller::Type::CONTROLLER_GHOST))
		{
			libsumo::Vehicle::add(entities_->object_[j]->name_, "");
		}
	}

	PostFrame();
}

void ControllerSumo::PostFrame()
{
	// Updates all positions for non-sumo controlled vehicles
	for (size_t i = 0; i < entities_->object_.size(); i++)
	{
		if (entities_->object_[i]->GetControllerType() != Controller::Type::CONTROLLER_SUMO &&
			entities_->object_[i]->GetControllerType() != Controller::Type::CONTROLLER_GHOST)
		{
			libsumo::Vehicle::moveToXY(entities_->object_[i]->name_, "random", 0, entities_->object_[i]->pos_.GetX() + sumo_x_offset_, 
				entities_->object_[i]->pos_.GetY() + sumo_y_offset_, entities_->object_[i]->pos_.GetH(), 0);
			libsumo::Vehicle::setSpeed(entities_->object_[i]->name_, entities_->object_[i]->speed_);
		}
	}
}

void ControllerSumo::Step(double timeStep)
{
	// stepping funciton for sumo, adds/removes vehicles (based on sumo), 
	// updates all positions of vehicles in the simulation that are controlled by sumo
	// do sumo timestep
	time_ += timeStep;
	libsumo::Simulation::step(time_);

	// check if any new cars has been added by sumo and add them to entities
	if (libsumo::Simulation::getDepartedNumber() > 0) {
		std::vector<std::string> deplist = libsumo::Simulation::getDepartedIDList();
		for (size_t i = 0; i < deplist.size(); i++)
		{
			if (!entities_->nameExists(deplist[i]))
			{
				Vehicle* vehicle = new Vehicle(); 
				// copy the default vehicle stuff here (add bounding box and so on)
				LOG("Adding new vehicle: %s", deplist[i].c_str());
				vehicle->name_ = deplist[i];
				vehicle->controller_ = entities_->sumo_vehicle->controller_;
				vehicle->model_filepath_ = entities_->sumo_vehicle->model_filepath_;
				entities_->addObject(vehicle);
			}
		}
	}

	// check if any cars have been removed by sumo and remove them from scenarioGateway and entities
	if (libsumo::Simulation::getArrivedNumber() > 0) {
		std::vector<std::string> arrivelist = libsumo::Simulation::getArrivedIDList();
		for (size_t i = 0; i < arrivelist.size(); i++)
		{
			for (size_t j = 0; j < entities_->object_.size(); j++)
			{
				if (arrivelist[i] == entities_->object_[j]->name_) {
					LOG("Removing vehicle: %s", arrivelist[i].c_str());
					entities_->removeObject(arrivelist[i]);
					gateway_->removeObject(arrivelist[i]);
				}
			}
		}
	}

	// Update the position of all cars controlled by sumo
	for (size_t i = 0; i < entities_->object_.size(); i++)
	{
		if (entities_->object_[i]->GetControllerType() == Controller::Type::CONTROLLER_SUMO)
		{
			Object* obj = entities_->object_[i];

			std::string sumoid = obj->name_;
			libsumo::TraCIPosition pos = libsumo::Vehicle::getPosition3D(sumoid);
			obj->speed_ = libsumo::Vehicle::getSpeed(sumoid);
			obj->pos_.SetInertiaPos(pos.x - sumo_x_offset_, pos.y - sumo_y_offset_, pos.z,
				-libsumo::Vehicle::getAngle(sumoid) * M_PI / 180 + M_PI / 2, libsumo::Vehicle::getSlope(sumoid) * M_PI / 180, 0);

			obj->dirty_lat_ = true;
			obj->dirty_long_ = true;

			// Report updated state to scenario gateway
			gateway_->reportObject(obj->id_, obj->name_, static_cast<int>(Object::Type::VEHICLE), static_cast<int>(Vehicle::Category::CAR),
				0, obj->GetControllerType(), obj->boundingbox_, 0,
				obj->GetSpeed(), obj->wheel_angle_, obj->wheel_rot_,
				obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ(),
				obj->pos_.GetH(), obj->pos_.GetP(), 0);
		}
	}

	Controller::Step(timeStep);
}

void ControllerSumo::Activate(int domainMask)
{
	// Reset time
	time_ = 0;

	Controller::Activate(domainMask);
}