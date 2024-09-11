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
#include "logger.hpp"

#include <utils/geom/PositionVector.h>
#include <libsumo/Simulation.h>
#include <libsumo/Vehicle.h>
#include <libsumo/TraCIDefs.h>

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerSumo(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerSumo(initArgs);
}

ControllerSumo::ControllerSumo(InitArgs* args) : Controller(args)
{
    // SUMO controller forced into override mode - will not perform any scenario actions
    if (mode_ != ControlOperationMode::MODE_OVERRIDE)
    {
        LOG_WARN("SUMO controller mode \"{}\" not applicable. Using override mode instead.", Mode2Str(mode_));
        mode_ = ControlOperationMode::MODE_OVERRIDE;
    }

    if (args->properties->file_.filepath_.empty())
    {
        LOG_ERROR("No filename!");
        return;
    }

    if (docsumo_.load_file(args->properties->file_.filepath_.c_str()).status == pugi::status_file_not_found)
    {
        LOG_ERROR("Failed to load SUMO config file {}", args->properties->file_.filepath_);
        throw std::invalid_argument(std::string("Cannot open file: ") + args->properties->file_.filepath_);
        return;
    }

    std::vector<std::string> file_name_candidates;
    file_name_candidates.push_back(args->parameters->ReadAttribute(docsumo_.child("configuration").child("input").child("net-file"), "value"));
    file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(args->properties->file_.filepath_), file_name_candidates[0]));
    pugi::xml_parse_result sumonet;
    size_t                 i;
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
        LOG_ERROR("Failed to load SUMO net file {}", file_name_candidates[0]);
        throw std::invalid_argument(std::string("Cannot open file: ") + file_name_candidates[0]);
        return;
    }

    pugi::xml_node location  = docsumo_.child("net").child("location");
    std::string    netoffset = args->parameters->ReadAttribute(location, "netOffset");
    std::size_t    delim     = netoffset.find(',');
    sumo_x_offset_           = std::stof(netoffset.substr(0, delim));
    sumo_y_offset_           = std::stof(netoffset.substr(delim + 1, netoffset.npos));

    std::vector<std::string> options;

    options.push_back("-c " + args->properties->file_.filepath_);
    options.push_back("--xml-validation");
    options.push_back("never");

    libsumo::Simulation::load(options);
}

void ControllerSumo::Init()
{
}

void ControllerSumo::Step(double timeStep)
{
    // stepping funciton for sumo, adds/removes vehicles (based on sumo),
    // updates all positions of vehicles in the simulation that are controlled by sumo
    // do sumo timestep
    time_ += timeStep;
    libsumo::Simulation::step(time_);

    // check if any new cars has been added by sumo and add them to entities
    if (libsumo::Simulation::getDepartedNumber() > 0)
    {
        std::vector<std::string> deplist = libsumo::Simulation::getDepartedIDList();
        for (size_t i = 0; i < deplist.size(); i++)
        {
            if (!entities_->nameExists(deplist[i]))
            {
                Vehicle* vehicle = new Vehicle();
                // copy the default vehicle stuff here (add bounding box and so on)
                LOG_INFO("SUMO controller: Add vehicle to scenario: {}", deplist[i]);
                vehicle->name_ = deplist[i];
                vehicle->AssignController(this);
                vehicle->model3d_     = template_vehicle_->model3d_;
                vehicle->scaleMode_   = EntityScaleMode::BB_TO_MODEL;
                vehicle->role_        = static_cast<int>(Object::Role::CIVIL);
                vehicle->category_    = Vehicle::Category::CAR;
                vehicle->odometer_    = 0.0;
                vehicle->boundingbox_ = template_vehicle_->boundingbox_;
                entities_->addObject(vehicle, true);
            }
        }
    }

    // check if any cars have been removed by sumo and remove them from scenarioGateway and entities
    if (libsumo::Simulation::getArrivedNumber() > 0)
    {
        std::vector<std::string> arrivelist = libsumo::Simulation::getArrivedIDList();
        for (size_t i = 0; i < arrivelist.size(); i++)
        {
            for (size_t j = 0; j < entities_->object_.size(); j++)
            {
                if (arrivelist[i] == entities_->object_[j]->name_)
                {
                    Object* obj = entities_->GetObjectByName(arrivelist[i]);
                    if (obj != nullptr)
                    {
                        LOG_INFO("SUMO controller: Remove vehicle from scenario: {}", arrivelist[i]);
                        gateway_->removeObject(arrivelist[i]);
                        if (obj->objectEvents_.size() > 0 || obj->initActions_.size() > 0)
                        {
                            entities_->deactivateObject(obj);
                        }
                        else
                        {
                            entities_->removeObject(obj, false);
                        }
                    }
                    else
                    {
                        LOG_ERROR("Failed to remove vehicle: {} - not found", arrivelist[i]);
                    }
                }
            }
        }
    }

    // Add missing vehicles from OpenSCENARIO to the sumo simulation
    std::vector<std::string> idlist = libsumo::Vehicle::getIDList();
    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        if (entities_->object_[i]->IsActive() && !entities_->object_[i]->IsGhost() &&
            std::find(idlist.begin(), idlist.end(), entities_->object_[i]->GetName()) == idlist.end())  // not already in sumo list
        {
            std::string id = entities_->object_[i]->name_;
            LOG_INFO("SUMO controller: Add vehicle to SUMO: {}", id);
            libsumo::Vehicle::add(id, "", "DEFAULT_VEHTYPE");
            libsumo::Vehicle::moveToXY(id,
                                       "random",
                                       0,
                                       entities_->object_[i]->pos_.GetX() + static_cast<double>(sumo_x_offset_),
                                       entities_->object_[i]->pos_.GetY() + static_cast<double>(sumo_y_offset_),
                                       entities_->object_[i]->pos_.GetH(),
                                       0);
            libsumo::Vehicle::setSpeed(id, entities_->object_[i]->speed_);
        }
    }

    // Update the position of all cars controlled by sumo
    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        if (entities_->object_[i]->IsActive())
        {
            if (entities_->object_[i]->IsAnyActiveControllerOfType(Controller::Type::CONTROLLER_TYPE_SUMO))
            {
                Object* obj = entities_->object_[i];

                std::string            sumoid = obj->name_;
                libsumo::TraCIPosition pos    = libsumo::Vehicle::getPosition3D(sumoid);
                obj->speed_                   = libsumo::Vehicle::getSpeed(sumoid);
                obj->pos_.SetInertiaPosMode(pos.x - static_cast<double>(sumo_x_offset_),
                                            pos.y - static_cast<double>(sumo_y_offset_),
                                            pos.z,
                                            -libsumo::Vehicle::getAngle(sumoid) * M_PI / 180 + M_PI / 2,
                                            libsumo::Vehicle::getSlope(sumoid) * M_PI / 180,
                                            0,
                                            roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_ABS |
                                                roadmanager::Position::PosMode::P_ABS | roadmanager::Position::PosMode::R_REL);

                obj->SetDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL);

                // Report updated state to the gateway
                gateway_->reportObject(obj->id_,
                                       obj->name_,
                                       static_cast<int>(obj->type_),
                                       obj->category_,
                                       obj->role_,
                                       obj->model_id_,
                                       obj->model3d_,
                                       obj->GetControllerTypeActiveOnDomain(ControlDomains::DOMAIN_LONG),
                                       obj->boundingbox_,
                                       static_cast<int>(obj->scaleMode_),
                                       0xff,
                                       time_,
                                       obj->speed_,
                                       obj->wheel_angle_,
                                       obj->wheel_rot_,
                                       object_->rear_axle_.positionZ,
                                       object_->front_axle_.positionX,
                                       object_->front_axle_.positionZ,
                                       &obj->pos_);
            }
            else if (!entities_->object_[i]->IsGhost())
            {
                // Updates all positions for non-sumo controlled vehicles
                libsumo::Vehicle::moveToXY(entities_->object_[i]->name_,
                                           "random",
                                           0,
                                           entities_->object_[i]->pos_.GetX() + static_cast<double>(sumo_x_offset_),
                                           entities_->object_[i]->pos_.GetY() + static_cast<double>(sumo_y_offset_),
                                           entities_->object_[i]->pos_.GetH(),
                                           0);
                libsumo::Vehicle::setSpeed(entities_->object_[i]->name_, entities_->object_[i]->speed_);
            }
        }
    }

    Controller::Step(timeStep);
}

int ControllerSumo::Activate(ControlActivationMode lat_activation_mode,
                             ControlActivationMode long_activation_mode,
                             ControlActivationMode light_activation_mode,
                             ControlActivationMode anim_activation_mode)
{
    // Reset time
    time_ = 0;

    // SUMO controller forced into both domains
    if (lat_activation_mode != ControlActivationMode::ON || long_activation_mode != ControlActivationMode::ON)
    {
        LOG_INFO("SUMO controller forced into operation of both domains (lat/long)");
        lat_activation_mode = long_activation_mode = ControlActivationMode::ON;
    }

    return Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);
}

void ControllerSumo::SetSumoVehicle(Object* object)
{
    template_vehicle_ = object;
    object_           = object;
    object_->AssignController(this);
}
